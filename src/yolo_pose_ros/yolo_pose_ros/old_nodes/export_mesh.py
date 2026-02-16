import torch
import smplx
import numpy as np
import trimesh

MODEL_DIR = "/home/andrea/smpl_models"
OUT_FILE = "/home/andrea/smpl_neutral.dae"  # oppure .obj

device = torch.device("cpu")

smpl = smplx.create(
    model_path=MODEL_DIR,
    model_type="smplx",
    gender="neutral",
    num_betas=10,
    use_pca=False,
    batch_size=1
).to(device)

with torch.no_grad():
    out = smpl(
        betas=torch.zeros((1, 10)),
        body_pose=torch.zeros((1, 63)),
        global_orient=torch.zeros((1, 3)),
        transl=torch.zeros((1, 3)),
        left_hand_pose=torch.zeros((1, 45)),
        right_hand_pose=torch.zeros((1, 45)),
        jaw_pose=torch.zeros((1, 3)),
        leye_pose=torch.zeros((1, 3)),
        reye_pose=torch.zeros((1, 3)),
        expression=torch.zeros((1, 10)),
    )

verts = out.vertices[0].cpu().numpy()
faces = smpl.faces

mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
mesh.export(OUT_FILE)

print(f"✅ Mesh esportata in {OUT_FILE}")
