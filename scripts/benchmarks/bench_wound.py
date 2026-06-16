"""Wound/injury detection benchmark + overlay on the Jetson Orin.
Self-contained port of wound_det/scripts/run_openvocab.py + run_clipseg.py
(no common.py dependency). Pure HuggingFace transformers — no custom CUDA builds.

Methods:
  owlv2   google/owlv2-base-patch16-ensemble    open-vocab box (cleanest, lightest)
  gdino   IDEA-Research/grounding-dino-base      open-vocab box (best recall)
  clipseg CIDAS/clipseg-rd64-refined             text->semantic mask

Usage:
  python3 bench_wound.py --method owlv2 --frames DIR --out OUT [--limit 0] [--no-video]
"""
import os, sys, time, json, glob, argparse
import numpy as np, cv2
from PIL import Image
import torch

WOUND_VOCAB = ["wound", "open wound", "laceration", "cut", "bleeding wound",
               "blood", "injury", "bruise", "surgical incision", "skin lesion"]
CLIPSEG_PROMPTS = ["a wound", "an open wound", "blood on skin", "a cut on the skin",
                   "a laceration", "a surgical incision", "a bruise"]
GDINO_BOX_THRESHOLD, GDINO_TEXT_THRESHOLD = 0.18, 0.12
OWL_THRESHOLD, OWL_NMS_IOU = 0.08, 0.30
CLIPSEG_THRESHOLD, CLIPSEG_MIN_AREA = 0.35, 80

CFG = {
    "owlv2":   dict(model="google/owlv2-base-patch16-ensemble"),
    "gdino":   dict(model="IDEA-Research/grounding-dino-base"),
    "clipseg": dict(model="CIDAS/clipseg-rd64-refined"),
}


def ap():
    p = argparse.ArgumentParser()
    p.add_argument("--method", required=True, choices=list(CFG))
    p.add_argument("--frames", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--limit", type=int, default=0)
    p.add_argument("--warmup", type=int, default=2)
    p.add_argument("--fps", type=float, default=4.0, help="output video fps (frames are subsampled)")
    p.add_argument("--no-video", action="store_true")
    return p.parse_args()


def draw(bgr, dets):
    out = bgr.copy()
    for d in dets:
        if d.get("mask") is not None:
            m = d["mask"]
            ov = out.copy(); ov[m] = (0, 0, 255)
            out = cv2.addWeighted(ov, 0.45, out, 0.55, 0)
        x1, y1, x2, y2 = [int(v) for v in d["box"]]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        s = d.get("score")
        lab = d.get("label", "wound") + (f" {s:.2f}" if s is not None else "")
        cv2.putText(out, lab, (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return out


def main():
    a = ap()
    os.makedirs(a.out, exist_ok=True)
    os.makedirs(os.path.join(a.out, "overlays"), exist_ok=True)
    frames = sorted(glob.glob(os.path.join(a.frames, "*.jpg")))
    if a.limit:
        frames = frames[:a.limit]
    assert frames, f"no frames in {a.frames}"
    device = "cuda" if torch.cuda.is_available() else "cpu"
    mid = CFG[a.method]["model"]
    print(f"[{a.method}] loading {mid} on {device}")

    if a.method == "clipseg":
        from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation
        proc = CLIPSegProcessor.from_pretrained(mid)
        model = CLIPSegForImageSegmentation.from_pretrained(mid).to(device).eval()
    else:
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
        proc = AutoProcessor.from_pretrained(mid)
        model = AutoModelForZeroShotObjectDetection.from_pretrained(mid).to(device).eval()

    gdino_text = ". ".join(v.lower() for v in WOUND_VOCAB) + "."
    owl_queries = [WOUND_VOCAB]

    def infer(pil):
        Wd, Ht = pil.size
        dets = []
        if a.method == "gdino":
            inp = proc(images=pil, text=gdino_text, return_tensors="pt").to(device)
            with torch.no_grad():
                out = model(**inp)
            try:  # transformers 4.44 uses box_threshold; 5.x uses threshold
                res = proc.post_process_grounded_object_detection(
                    out, inp.input_ids, box_threshold=GDINO_BOX_THRESHOLD,
                    text_threshold=GDINO_TEXT_THRESHOLD, target_sizes=[(Ht, Wd)])[0]
            except TypeError:
                res = proc.post_process_grounded_object_detection(
                    out, inp.input_ids, threshold=GDINO_BOX_THRESHOLD,
                    text_threshold=GDINO_TEXT_THRESHOLD, target_sizes=[(Ht, Wd)])[0]
            labs = res.get("text_labels", res.get("labels"))
            for b, s, l in zip(res["boxes"].cpu().numpy(), res["scores"].cpu().numpy(), labs):
                dets.append({"box": [float(v) for v in b], "score": float(s),
                             "label": l if isinstance(l, str) else "wound", "mask": None})
        elif a.method == "owlv2":
            inp = proc(text=owl_queries, images=pil, return_tensors="pt").to(device)
            with torch.no_grad():
                out = model(**inp)
            # transformers 4.44 Owlv2Processor: post_process_object_detection (labels = query idx)
            res = proc.post_process_object_detection(
                out, threshold=OWL_THRESHOLD,
                target_sizes=torch.tensor([(Ht, Wd)]).to(device))[0]
            from torchvision.ops import nms
            bx, sc, lb = res["boxes"], res["scores"], res["labels"]
            keep = nms(bx, sc, OWL_NMS_IOU).cpu().numpy().tolist() if len(bx) else []
            bxn, scn, lbn = bx.cpu().numpy(), sc.cpu().numpy(), lb.cpu().numpy()
            for k in keep:
                li = int(lbn[k])
                lab = WOUND_VOCAB[li] if 0 <= li < len(WOUND_VOCAB) else "wound"
                dets.append({"box": [float(v) for v in bxn[k]], "score": float(scn[k]),
                             "label": lab, "mask": None})
        else:  # clipseg
            inp = proc(text=CLIPSEG_PROMPTS, images=[pil] * len(CLIPSEG_PROMPTS),
                       return_tensors="pt", padding=True).to(device)
            with torch.no_grad():
                out = model(**inp)
            probs = out.logits.sigmoid().detach().cpu().numpy()  # [P,352,352]
            prob_acc = np.zeros((Ht, Wd), np.float32)
            for pr in probs:
                prob_acc = np.maximum(prob_acc, cv2.resize(pr.astype(np.float32), (Wd, Ht)))
            m = (prob_acc > CLIPSEG_THRESHOLD).astype(np.uint8)
            n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
            for ci in range(1, n):
                x, y, w, h, area = stats[ci]
                if area < CLIPSEG_MIN_AREA:
                    continue
                comp = lab == ci
                dets.append({"box": [float(x), float(y), float(x + w), float(y + h)],
                             "score": float(np.clip(prob_acc[comp].mean(), 0, 1)),
                             "label": "wound", "mask": comp})
        return dets

    # warmup
    pil0 = Image.open(frames[0]).convert("RGB")
    for _ in range(max(1, a.warmup)):
        infer(pil0)
    if device == "cuda":
        torch.cuda.synchronize(); torch.cuda.reset_peak_memory_stats()

    img0 = cv2.imread(frames[0]); H, W = img0.shape[:2]
    vw = None
    if not a.no_video:
        vw = cv2.VideoWriter(os.path.join(a.out, "overlay.mp4"),
                             cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, H))
    all_preds, confs, infer_t, n_with, n_total = {}, [], 0.0, 0, 0
    for i, fp in enumerate(frames):
        fn = os.path.basename(fp)
        pil = Image.open(fp).convert("RGB")
        if device == "cuda":
            torch.cuda.synchronize()
        t0 = time.time()
        dets = infer(pil)
        if device == "cuda":
            torch.cuda.synchronize()
        infer_t += time.time() - t0
        # strip masks from json (bulky); keep for drawing
        all_preds[fn] = [{k: v for k, v in d.items() if k != "mask"} for d in dets]
        n_total += len(dets); n_with += 1 if dets else 0
        confs += [d["score"] for d in dets if d.get("score") is not None]
        bgr = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
        ov = draw(bgr, dets)
        if vw is not None:
            cv2.putText(ov, f"{a.method} | {fn}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            if ov.shape[:2] != (H, W):
                ov = cv2.resize(ov, (W, H))
            vw.write(ov)
        if i % 5 == 0:
            cv2.imwrite(os.path.join(a.out, "overlays", fn), ov)
    if vw is not None:
        vw.release()

    n = len(frames)
    metrics = dict(
        method=a.method, model=mid, device=device, torch=torch.__version__,
        gpu_name=(torch.cuda.get_device_name(0) if device == "cuda" else "cpu"),
        n_frames=n, frames_with_detection=n_with,
        detection_rate=round(n_with / n, 4), mean_dets_per_frame=round(n_total / n, 3),
        mean_conf=round(float(np.mean(confs)), 4) if confs else None,
        ms_per_frame=round(infer_t / n * 1000, 1), fps=round(n / infer_t, 2),
        peak_vram_mb=round(torch.cuda.max_memory_allocated() / 1e6, 1) if device == "cuda" else 0,
    )
    with open(os.path.join(a.out, "predictions.json"), "w") as f:
        json.dump(all_preds, f)
    with open(os.path.join(a.out, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    print("METRICS", json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
