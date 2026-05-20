# TERESA — Piano rifattorizzazione launch (Core + App)

## Obiettivo

Ridurre i 6 terminali attuali a 3, separando i driver hardware (Orbbec, RealSense, Z1) dalla logica applicativa (percezione, controllo, WBC). Il Core segnala quando tutto è attivo via `/core/ready`.

```
T1: ros2 launch spot_control teresa_core.launch.py
T2: ros2 launch spot_control teresa_app.launch.py
T3: ros2 run spot_control wbc_keyboard_node
```

---

## Architettura

### `teresa_core.launch.py` — nuovo file in `spot_control/launch/`

| Cosa | Provenienza |
|---|---|
| Orbbec Femto Bolt driver | `spot_perception.launch.py` → estrarre `IncludeLaunchDescription` di `femto_bolt.launch.py` |
| TF statica `my_spot/body → orbbec_link` | `spot_perception.launch.py` → spostare `static_transform_publisher` |
| TF statica `orbbec_link → orbbec_color_optical_frame` | `spot_perception.launch.py` → spostare |
| Z1 bringup (`z1.launch.py`, robot_state_publisher + JTC + `/joint_states`) | `z1_realsense.launch.py` → estrarre `IncludeLaunchDescription` di `z1.launch.py` |
| RealSense driver | `z1_realsense.launch.py` → estrarre `IncludeLaunchDescription` di `rs_launch.py` |
| TF statica `link06 → camera_link` | `z1_realsense.launch.py` → spostare `static_transform_publisher` |
| Nodo `core_ready` | **Nuovo**: monitora `/joint_states` + `/orbbec/color/image_raw` + TF `link00→link06`. Quando tutti attivi, pubblica `/core/ready` Bool True una volta sola. |

### `teresa_app.launch.py` — nuovo file in `spot_control/launch/`

Include tramite `IncludeLaunchDescription`:
- `spot_perception.launch.py` con `use_orbbec_driver:=false`
- `z1_perception.launch.py`
- `z1_control.launch.py`
- `wbc.launch.py`

### `spot_perception.launch.py` — modifica

Aggiungere argomento `use_orbbec_driver` (default `true` per retrocompatibilità):

```python
use_orbbec_driver_arg = DeclareLaunchArgument(
    'use_orbbec_driver', default_value='true',
    description='Lancia Orbbec driver + TF statiche. false se già lanciato da teresa_core')
```

Quando `false`:
- **Salta** `orbbec_launch` (IncludeLaunchDescription di femto_bolt)
- **Salta** `static_tf_body_camera` e `static_tf_camera_optical`
- **Riduce** i `TimerAction`: yolo+posture+bbox+laying a t=1s invece di t=4s (Orbbec già attivo dal core)
- **Lancia** solo: yolo_skeleton, posture_analyzer, bbox_visualizer, laying_human_detector

### `core_ready` node — nuovo file `spot_control/spot_control/core_ready.py`

Nodo ROS 2 minimale:
1. Si sottoscrive a `/joint_states` → ricevuto almeno un messaggio = Z1 driver attivo
2. Si sottoscrive a `/orbbec/color/image_raw` (o un topic equivalente) → Orbbec attivo
3. Controlla TF `link00 → link06` con `lookup_transform` → robot_state_publisher funzionante
4. Quando tutti e 3 OK → pubblica `/core/ready` = True (una volta), logga conferma, esce

### `setup.py` — modifica

Aggiungere entry point per `core_ready`:
```python
'core_ready = spot_control.core_ready:main',
```

I nuovi launch file sono già coperti da `glob('launch/*.py')`.

---

## Terminali finali

```bash
# T1: Core (driver hardware + TF statiche)
ros2 launch spot_control teresa_core.launch.py
# Aspettare: [core_ready] Core pronto — tutti i driver attivi.

# T2: App (percezione + controllo + WBC)
ros2 launch spot_control teresa_app.launch.py
# Aspettare: [TF READY] SpotCore connesso — premi "s" per iniziare.

# T3: Keyboard
ros2 run spot_control wbc_keyboard_node
# Premere "s" → missione parte
```

---

## File da creare

| File | Ruolo |
|---|---|
| `src/spot_control/launch/teresa_core.launch.py` | Core launch: driver + TF statiche + core_ready |
| `src/spot_control/launch/teresa_app.launch.py` | App launch: include i 4 sub-launch applicativi |
| `src/spot_control/spot_control/core_ready.py` | Nodo monitor: controlla driver attivi → pubblica `/core/ready` |

## File da modificare

| File | Modifica |
|---|---|
| `src/spot_perception/launch/spot_perception.launch.py` | Aggiungere arg `use_orbbec_driver`; quando false, salta driver e TF, riduce TimerAction |
| `src/spot_control/setup.py` | Aggiungere entry point `core_ready` |
| `INIT.md` | Aggiornare sezione "Running Spot + Z1 WBC" con i 3 terminali |

## File invariati

`z1_realsense.launch.py`, `z1_perception.launch.py`, `z1_control.launch.py`, `wbc.launch.py` restano come sono — usabili standalone per debug, inclusi da `teresa_app.launch.py`.
