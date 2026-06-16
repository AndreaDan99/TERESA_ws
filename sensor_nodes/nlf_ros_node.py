#!/usr/bin/env python3
"""NLF 3D body (SMPL) as a live ROS2 node — subscribes a color Image topic, runs the
NLF TorchScript on the Jetson GPU, publishes 24 SMPL joints (PoseArray, metres, camera
frame) + an overlay Image, and saves periodic overlay JPEGs for verification.

Reuses the exact recipe from bench_nlf.py (JIT-executor OFF; .cuda() after load;
model.detect_smpl_batched on a stacked CHW uint8 cuda tensor).

Run (inside teresa_gpu, same DDS domain as the camera container):
  python3 nlf_ros_node.py --topic /orbbec/color/image_raw \
    --model /work/models/nlf_s_multi_v020.torchscript --save-dir /work/live_nlf_ros --rate 6
"""
import os, json, time, argparse
import numpy as np, cv2, torch, torchvision  # noqa: F401 (torchvision needed by the NLF ops)

# CRITICAL (Jetson/torch 2.0): disable TorchScript profiling executor + fuser, else the
# 2nd forward deadlocks. Must run BEFORE torch.jit.load. (Ported from bench_nlf.py.)
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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge

PARENTS = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21]
EDGES = [(i, PARENTS[i]) for i in range(1, 24)]


class NLFNode(Node):
    def __init__(self, a):
        super().__init__('nlf_ros_node')
        self.a = a
        self.bridge = CvBridge()
        self.latest = None
        self.frame_id_hdr = ''
        self.n = 0
        self.n_det = 0
        self.t_infer = 0.0
        self.last_log = time.time()
        self.vw = None
        if a.save_dir:
            os.makedirs(a.save_dir, exist_ok=True)
        self.get_logger().info(f"loading NLF {a.model} …")
        self.model = torch.jit.load(a.model).cuda().eval()
        # warmup (one dummy forward so the first real frame isn't cold)
        with torch.inference_mode(), torch.device('cuda'):
            self.model.detect_smpl_batched(torch.zeros(1, 3, 256, 256, dtype=torch.uint8).cuda())
        torch.cuda.synchronize()
        self.get_logger().info(f"NLF ready on {torch.cuda.get_device_name(0)}; subscribing {a.topic}")
        self.create_subscription(Image, a.topic, self._cb, qos_profile_sensor_data)
        self.pub_joints = self.create_publisher(PoseArray, a.out_topic, 10)
        self.pub_overlay = self.create_publisher(Image, a.out_topic + '/overlay', 1)
        self.create_timer(1.0 / max(1.0, a.rate), self._process)

    def _cb(self, msg):
        self.latest = msg
        self.frame_id_hdr = msg.header.frame_id

    def dump_metrics(self):
        if not self.a.save_dir or not self.n:
            return
        m = dict(node="nlf", model=os.path.basename(self.a.model),
                 device=torch.cuda.get_device_name(0), torch=torch.__version__,
                 n_frames=self.n, infer_fps=round(self.n / self.t_infer, 2) if self.t_infer else 0,
                 det_rate=round(self.n_det / self.n, 3),
                 gpu_mem_mb=round(torch.cuda.max_memory_allocated() / 1e6, 1))
        with open(os.path.join(self.a.save_dir, "metrics.json"), "w") as f:
            json.dump(m, f, indent=2)
        self.get_logger().info("METRICS " + json.dumps(m))

    def _process(self):
        if self.latest is None:
            return
        msg = self.latest
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        H, W = rgb.shape[:2]
        t = torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1).unsqueeze(0).cuda()
        torch.cuda.synchronize(); t0 = time.time()
        with torch.inference_mode(), torch.device('cuda'):
            pred = self.model.detect_smpl_batched(t)
        torch.cuda.synchronize(); dt = time.time() - t0
        self.t_infer += dt; self.n += 1

        j2_all = pred['joints2d'][0]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        det = False
        if j2_all.shape[0] > 0:
            j2np = j2_all.cpu().numpy()
            spans = [(np.nanmax(p[:, 0]) - np.nanmin(p[:, 0])) *
                     (np.nanmax(p[:, 1]) - np.nanmin(p[:, 1])) for p in j2np]
            d = int(np.argmax(spans)); det = True; self.n_det += 1
            j3 = pred['joints3d'][0][d].cpu().numpy()           # 24x3, mm (camera frame)
            j2 = pred['joints2d'][0][d].cpu().numpy()
            v2 = pred['vertices2d'][0][d].cpu().numpy()
            # publish 24 joints in metres
            pa = PoseArray(); pa.header = msg.header
            for (x, y, z) in j3:
                p = Pose(); p.position.x = float(x) / 1000.0
                p.position.y = float(y) / 1000.0; p.position.z = float(z) / 1000.0
                p.orientation.w = 1.0; pa.poses.append(p)
            self.pub_joints.publish(pa)
            # overlay
            for vx, vy in v2[::4]:
                if 0 <= vx < W and 0 <= vy < H:
                    cv2.circle(bgr, (int(vx), int(vy)), 1, (0, 180, 0), -1)
            for x, c in EDGES:
                pa_, pc_ = j2[x], j2[c]
                if np.isfinite(pa_).all() and np.isfinite(pc_).all():
                    cv2.line(bgr, (int(pa_[0]), int(pa_[1])), (int(pc_[0]), int(pc_[1])), (0, 128, 255), 2)
            for jx, jy in j2:
                cv2.circle(bgr, (int(jx), int(jy)), 3, (255, 0, 0), -1)
        fps = self.n / self.t_infer if self.t_infer else 0.0
        cv2.putText(bgr, f"NLF live | {fps:.1f} fps | det={det}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))
        if self.a.video:
            if self.vw is None:
                self.vw = cv2.VideoWriter(self.a.video, cv2.VideoWriter_fourcc(*"mp4v"),
                                          self.a.out_fps, (W, H))
            self.vw.write(bgr)
        if self.a.save_dir and self.n % self.a.save_every == 0:
            cv2.imwrite(os.path.join(self.a.save_dir, f"nlf_{self.n:05d}.jpg"), bgr)
        if time.time() - self.last_log > 2.0:
            self.get_logger().info(f"frame {self.n} | infer {fps:.1f} fps | det={det}")
            self.last_log = time.time()
        if self.a.max_frames and self.n >= self.a.max_frames:
            self.get_logger().info(f"reached max_frames={self.a.max_frames}; done")
            if self.vw is not None:
                self.vw.release()
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/orbbec/color/image_raw')
    ap.add_argument('--model', required=True)
    ap.add_argument('--out-topic', default='/perception/nlf/points_3d')
    ap.add_argument('--rate', type=float, default=6.0, help='max inference Hz')
    ap.add_argument('--save-dir', default='')
    ap.add_argument('--save-every', type=int, default=5)
    ap.add_argument('--video', default='')
    ap.add_argument('--out-fps', type=float, default=6.0)
    ap.add_argument('--max-frames', type=int, default=0)
    a = ap.parse_args()
    rclpy.init()
    node = NLFNode(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.dump_metrics()
        if node.vw is not None:
            node.vw.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
