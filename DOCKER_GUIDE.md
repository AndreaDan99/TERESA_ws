# 🐳 TERESA — Guida Docker (Jetson Orin AGX)

## Immagini disponibili

| Immagine | Taglia | Cosa contiene | Comando build |
|---|---|---|---|
| `teresa_core:latest` | 19.9 GB | ROS2 Humble + driver Orbbec + RealSense + Z1 bringup + xacro | `docker build -f docker/Dockerfile.l4t35 -t teresa_core:latest .` |
| `teresa_gpu:latest` | 12 GB | ROS2 Humble + PyTorch CUDA + YOLO + NLF | `docker build -f docker/Dockerfile.teresa_gpu -t teresa_gpu:latest .` |
| `andrea_deploy:latest` | 12 GB | Solo PyTorch CUDA (no ROS) — benchmark offline | `docker build -f docker/Dockerfile.andrea_deploy -t andrea_deploy:latest .` |
| `andrea_mp:latest` | 12 GB | MediaPipe isolato — pose benchmark | `docker build -f docker/Dockerfile.mediapipe -t andrea_mp:latest .` |

## Avvio rapido

```bash
# UN comando per avviare entrambi i container
bash teresa_start.sh

# Stato / Stop
bash teresa_start.sh status
bash teresa_start.sh stop
```

Cosa fa `teresa_start.sh`:
- Avvia `teresa_core` (hardware: camere + Z1 + TF) su `ROS_DOMAIN_ID=42`
- Avvia `teresa_gpu` (percezione GPU) su `ROS_DOMAIN_ID=42`
- Monta la workspace in `/ros2_ws` dentro entrambi i container

---

## Architettura

```
┌──────────────────────────┐         DDS (dominio 42)        ┌──────────────────────────┐
│   teresa_core (hardware) │ ←───────────────────────────→  │   teresa_gpu (percezione) │
│                          │                                │                          │
│  📷 Driver Orbbec        │  /orbbec/color/image_raw       │  🧠 YOLO skeleton        │
│  📷 Driver RealSense     │  /camera/camera/color/...      │  🧠 NLF 3D body          │
│  🔌 Driver Z1 arm        │  /tf, /tf_static               │  🧠 Wound detection      │
│  📐 TF statiche          │                                │  🎮 Z1 IK + FSM          │
└──────────────────────────┘                                └──────────────────────────┘
```

I due container parlano via **DDS** (protocollo ROS2). Le camere vanno in `teresa_core` 
perche' hanno bisogno di `--privileged` (USB raw). La GPU va in `teresa_gpu` perche' ha 
bisogno di `--runtime=nvidia`.

---

## Come si usa

### 1. Avvia i container

```bash
ssh orin
bash ~/AndreaDantonaTeresa/TERESA_ws/teresa_start.sh
```

### 2. Entra nel container core (hardware)

```bash
docker exec -it teresa_core bash
source /ros2_ws/install/setup.bash

# Lancia TUTTO (camere + Z1 + TF):
ros2 launch spot_control teresa_core.launch.py

# Oppure solo Orbbec:
ros2 launch orbbec_camera femto_bolt.launch.py \
  camera_name:=orbbec enable_color:=true enable_depth:=true \
  color_width:=1280 color_height:=720 color_fps:=15 \
  depth_width:=1024 depth_height:=1024 depth_fps:=15 \
  depth_registration:=true enable_point_cloud:=false

# Oppure solo RealSense:
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30
```

### 3. Entra nel container GPU (percezione)

```bash
docker exec -it teresa_gpu bash
source /opt/ros/humble/install/setup.bash
source /ros2_ws/install/setup.bash

# Percezione Orbbec (YOLO + NLF + posture):
ros2 launch spot_perception spot_perception.launch.py \
  use_orbbec_driver:=false perception_backend:=yolo

# Full TERESA perception (Orbbec + RealSense):
ros2 launch spot_control teresa_perception.launch.py \
  use_orbbec_driver:=false

# Z1 arm control (quando collegato):
ros2 launch z1_vision z1_control.launch.py

# WBC coordinator (quando Spot connesso):
ros2 launch spot_control wbc.launch.py dry_run:=true
```

### 4. Verifica che tutto funzioni

```bash
# Dentro teresa_gpu:
ros2 topic list | grep -E 'orbbec|camera'
# Dovresti vedere:
#   /orbbec/color/image_raw
#   /orbbec/depth/image_raw
#   /camera/camera/color/image_raw
#   /camera/camera/depth/image_raw

# Test GPU:
python3 -c "import torch; print(torch.cuda.is_available())"
# -> True
```

---

## Hardware

| Dispositivo | Modello | IP/Porta | Stato |
|---|---|---|---|
| Orbbec | Femto Bolt | USB 3.1 + 12V DC (obbligatorio) | OK |
| RealSense | D415 (non D435) | USB 3.2 | OK |
| Z1 Arm | Unitree | 192.168.123.110 (Ethernet) | Non collegato |
| Spot | — | SpotCore DDS | Differito |

**Attenzione:**
- Orbbec: serve alimentatore 12V DC. Solo USB-C non basta.
- RealSense D415: due camere insieme possono saturare USB. Se una fallisce, scollega l'altra.
- Z1: il Jetson deve avere IP `192.168.123.235` sull'interfaccia Ethernet.

---

## Workspace

La workspace ROS2 e' in `~/AndreaDantonaTeresa/TERESA_ws` e viene montata 
in entrambi i container come `/ros2_ws`. 

Per ricompilare dopo modifiche:

```bash
docker exec -it teresa_gpu bash
source /opt/ros/humble/install/setup.bash
cd /ros2_ws
colcon build --symlink-install --parallel-workers 4 \
  --packages-select spot_control spot_perception z1_vision teresa_utils
```

---

## Dati e modelli

I dati pesanti (modelli, cache, test frames) sono su `/ssd/andrea_deploy/`:

| Path | Taglia | Descrizione |
|---|---|---|
| `models/` | 1.4 GB | YOLO, NLF, MediaPipe |
| `hf_cache/` | 4.1 GB | HuggingFace (Grounding-DINO, OWLv2) |
| `frames_nlf/` | 271 MB | Test data NLF |
| `frames_wound/` | 22 MB | Test data wound (manichino Bob) |
| `nlf/ wound/ pose/ cap/` | ~280 MB | Output benchmark (confronto modelli) |
| `report_assets/` | 18 MB | Report HTML |

---

## Troubleshooting

| Problema | Causa | Fix |
|---|---|---|
| `torch.cuda.is_available() == False` | Manca `--runtime=nvidia` | Aggiungi `--runtime=nvidia` |
| RealSense non trovata | USB surriscaldata / occupata | Aspetta 1 min, riprova |
| Orbbec non rilevata | Manca alimentatore 12V | Collega 12V DC |
| `No module named 'xacro'` | Xacro non installato | `apt-get install ros-humble-xacro` |
| `RS2_USB_STATUS_BUSY` | Conflitto USB tra camere | Scollega una camera |
| Launch non trovato | Workspace non ricompilata | `colcon build --symlink-install` |
