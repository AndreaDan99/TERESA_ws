"""NLF benchmark + overlay on the Jetson Orin.
Port of Body_prediction/methods/nlf/run_nlf.py, parametrized, with Orin-focused
metrics: pure-inference FPS vs end-to-end FPS, per-frame latency, GPU memory,
detection rate. Writes overlay.mp4, sample frames, track npz, metrics.json.

Usage:
  python3 bench_nlf.py --model M.torchscript --frames DIR --out OUT
                       [--batch 8] [--limit 0] [--warmup 2] [--fps 30] [--no-video]
"""
import os, time, json, glob, argparse, platform
import numpy as np, cv2, torch, torchvision

# CRITICAL (Jetson/torch 2.0): disable the TorchScript profiling executor + fuser.
# Otherwise the 2nd forward triggers graph re-fusion that DEADLOCKS on this stack
# (first call fine, every subsequent call hangs). Forces the simple interpreter.
torch._C._jit_set_profiling_executor(False)
torch._C._jit_set_profiling_mode(False)
for _fn, _a in [("_jit_set_texpr_fuser_enabled", False),
                ("_jit_override_can_fuse_on_gpu", False),
                ("_jit_override_can_fuse_on_cpu", False),
                ("_jit_set_nvfuser_enabled", False)]:
    try:
        getattr(torch._C, _fn)(_a)
    except Exception:
        pass

SMPL_NAMES = ['pelvis','l_hip','r_hip','spine1','l_knee','r_knee','spine2','l_ankle',
 'r_ankle','spine3','l_foot','r_foot','neck','l_collar','r_collar','head','l_shoulder',
 'r_shoulder','l_elbow','r_elbow','l_wrist','r_wrist','l_hand','r_hand']
PARENTS = [-1,0,0,0,1,2,3,4,5,6,7,8,9,9,9,12,13,14,16,17,18,19,20,21]
EDGES = [(i, PARENTS[i]) for i in range(1, 24)]


def ap():
    p = argparse.ArgumentParser()
    p.add_argument("--model", required=True)
    p.add_argument("--frames", required=True, help="dir containing frame_*.jpg")
    p.add_argument("--out", required=True)
    p.add_argument("--batch", type=int, default=8)
    p.add_argument("--limit", type=int, default=0, help="max frames (0=all)")
    p.add_argument("--warmup", type=int, default=2, help="warmup batches (excluded from timing)")
    p.add_argument("--fps", type=float, default=30.0, help="output video fps")
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--tag", default="nlf")
    return p.parse_args()


def main():
    a = ap()
    os.makedirs(a.out, exist_ok=True)
    os.makedirs(os.path.join(a.out, "overlay"), exist_ok=True)
    FRAMES = sorted(glob.glob(os.path.join(a.frames, "frame_*.jpg")))
    if a.limit:
        FRAMES = FRAMES[:a.limit]
    T = len(FRAMES)
    assert T > 0, f"no frames in {a.frames}"

    dev = torch.cuda.get_device_name(0)
    model = torch.jit.load(a.model).cuda().eval()   # .cuda() AFTER load — proven recipe
    try:
        pdev = next(model.parameters()).device
    except Exception:
        pdev = "?"
    print(f"[init] model on {pdev} | device={dev} | frames={T} | batch={a.batch}", flush=True)
    img0 = cv2.imread(FRAMES[0]); H, W = img0.shape[:2]
    J = 24
    kpts2d = np.full((T, J, 2), np.nan, np.float32)
    kpts3d = np.full((T, J, 3), np.nan, np.float32)
    unc = np.full((T, J), np.nan, np.float32)
    poses = np.full((T, 72), np.nan, np.float32)
    betas = np.full((T, 10), np.nan, np.float32)
    trans = np.full((T, 3), np.nan, np.float32)
    det = np.zeros(T, bool)

    vw = None
    if not a.no_video:
        vw = cv2.VideoWriter(os.path.join(a.out, "overlay.mp4"),
                             cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, H))

    # ---- warmup (excluded from timing) ----
    wb = torch.stack([torchvision.io.read_image(FRAMES[0])] * a.batch).cuda()
    for _ in range(max(1, a.warmup)):
        with torch.inference_mode(), torch.device('cuda'):
            model.detect_smpl_batched(wb)
    torch.cuda.synchronize()
    torch.cuda.reset_peak_memory_stats()

    # ---- main loop ----
    infer_t = 0.0
    per_frame_lat = []
    e2e_t0 = time.time()
    for b0 in range(0, T, a.batch):
        idxs = list(range(b0, min(b0 + a.batch, T)))
        batch = torch.stack([torchvision.io.read_image(FRAMES[i]) for i in idxs]).cuda()
        torch.cuda.synchronize(); ti = time.time()
        with torch.inference_mode(), torch.device('cuda'):
            pred = model.detect_smpl_batched(batch)
        torch.cuda.synchronize(); dt = time.time() - ti
        infer_t += dt
        per_frame_lat.append(dt / len(idxs))

        for bi, i in enumerate(idxs):
            j2_all = pred['joints2d'][bi]  # [N,24,2]
            img = cv2.imread(FRAMES[i])
            if j2_all.shape[0] > 0:
                j2np = j2_all.cpu().numpy()
                spans = [(np.nanmax(p[:, 0]) - np.nanmin(p[:, 0])) *
                         (np.nanmax(p[:, 1]) - np.nanmin(p[:, 1])) for p in j2np]
                d = int(np.argmax(spans))
                det[i] = True
                kpts3d[i] = pred['joints3d'][bi][d].cpu().numpy()
                j2 = pred['joints2d'][bi][d].cpu().numpy(); kpts2d[i] = j2
                ju = pred['joint_uncertainties'][bi][d].cpu().numpy(); unc[i] = ju
                v2 = pred['vertices2d'][bi][d].cpu().numpy()
                poses[i] = pred['pose'][bi][d].cpu().numpy().reshape(-1)[:72]
                betas[i] = pred['betas'][bi][d].cpu().numpy()
                trans[i] = pred['trans'][bi][d].cpu().numpy()
                if vw is not None:
                    for vx, vy in v2[::4]:
                        if 0 <= vx < W and 0 <= vy < H:
                            cv2.circle(img, (int(vx), int(vy)), 1, (0, 180, 0), -1)
                    for x, c in EDGES:
                        pa, pc = j2[x], j2[c]
                        if np.isfinite(pa).all() and np.isfinite(pc).all():
                            cv2.line(img, (int(pa[0]), int(pa[1])), (int(pc[0]), int(pc[1])), (0, 128, 255), 2)
                    for jx, jy in j2:
                        cv2.circle(img, (int(jx), int(jy)), 3, (255, 0, 0), -1)
            if vw is not None:
                cv2.putText(img, f"NLF {a.tag} | frame {i}", (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                vw.write(img)
                if i % 50 == 0:
                    cv2.imwrite(os.path.join(a.out, "overlay", f"f{i:05d}.jpg"), img)
        if b0 % (a.batch * 10) == 0:
            print(f"  {b0}/{T}  infer {len(idxs)/dt:.1f} fps", flush=True)
    if vw is not None:
        vw.release()
    e2e = time.time() - e2e_t0

    lat = np.array(per_frame_lat) * 1000.0  # ms/frame (amortized over batch)
    metrics = dict(
        tag=a.tag, model=os.path.basename(a.model), device=dev,
        torch=torch.__version__, frames=T, batch=a.batch,
        infer_fps=round(T / infer_t, 2),
        e2e_fps=round(T / e2e, 2),
        infer_ms_per_frame_mean=round(float(lat.mean()), 2),
        batch_latency_ms_p50=round(float(np.percentile(per_frame_lat, 50) * a.batch * 1000), 1),
        det_rate=round(float(det.mean()), 4),
        gpu_mem_mb=round(torch.cuda.max_memory_allocated() / 1e6, 1),
        input_res=f"{W}x{H}",
    )
    np.savez_compressed(os.path.join(a.out, "track.npz"),
                        kpts2d=kpts2d, kpts3d=kpts3d, unc=unc,
                        pose=poses, betas=betas, trans=trans, det=det,
                        meta=json.dumps(dict(joint_names=SMPL_NAMES, fps=a.fps,
                                             space="camera_mm", res=[W, H])))
    with open(os.path.join(a.out, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    print("METRICS", json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
