# TERESA Web Control Panel

Interfaccia web per il controllo remoto del sistema TERESA via rosbridge WebSocket.

## Prerequisiti

Sul laptop (dove girano i nodi ROS 2):

```bash
sudo apt install ros-jazzy-rosbridge-suite
```

## Avvio

```bash
# Terminale 1 — rosbridge WebSocket
ros2 run rosbridge_server rosbridge_websocket

# Terminale 2 — server HTTP per i file statici
cd web && python3 -m http.server 8000
```

Apri `http://localhost:8000/teresa_control.html` nel browser.

Nessuna dipendenza lato client — `roslibjs` è caricato da CDN.

---

## `teresa_control.html` — Pannello di controllo

Pagina principale con stato WBC, log eventi, e barra comandi fissa in basso.

### Connessione

| Campo | Default | Descrizione |
|-------|---------|-------------|
| `ws://` | `localhost:9090` | URL del rosbridge WebSocket |

Premi Enter per connetterti.

### Indicatori di stato

| Indicatore | Topic | Significato |
|------------|-------|-------------|
| 🟢 Rosbridge | — | WebSocket connesso |
| 🟢 SpotCore | `/wbc/tf_ready` (Bool) | TF odom→body attiva = SpotCore online |
| WBC state | `/wbc/state` (String) | Stato corrente del coordinator FSM |
| Start pose | — | Posizione salvata al primo START (x, y, yaw°) |

### Banner step mode

Quando `step_mode:=true` nel WBC, le transizioni automatiche tra stati FSM vengono bloccate. Il banner giallo mostra la transizione in attesa (es. `SEARCHING → LOCKING`). Premi **STEP** o `n` per confermare.

### Comandi

| Pulsante | Tasto | Topic / Servizio | Azione |
|----------|:-----:|------------------|--------|
| ▶ **START** | `s` | `/wbc/restart=True` | Salva posa corrente (primo uso) + avvia WBC SEARCHING |
| ↩ **RETURN** | `r` / `q` | `/wbc/restart=False` + `stand` + navigazione P-controller (10 Hz) | Ferma WBC, alza Spot, naviga alla posa di start salvata, riallinea yaw |
| ↻ **UPDATE** | `u` | TF `odom→body` | Aggiorna la posa di partenza con quella corrente |
| → **STEP** | `n` | `/wbc/step_confirm=True` | Conferma transizione in step mode |
| ⬆ **STAND** | `a` | `/my_spot/stand` (Trigger) | Spot si alza |
| ⬇ **SIT** | `c` | `/my_spot/sit` (Trigger) | Spot si siede |
| ⏹ **STOP** | `ESC` | `/wbc/restart=False` + `cmd_vel=0` | Emergency stop |
| 📷 **CAMERAS** | `v` | — | Apre camera view in tab separata |

### Navigazione RETURN

Replica esatta della logica del `wbc_keyboard_controller.py` in JavaScript:

```
ROTATING → DRIVING → REALIGNING → IDLE
```

Parametri: goal tolerance 0.15 m, angular speed 0.4 rad/s, linear speed 0.3 m/s, angle threshold 0.08 rad. Loop a 10 Hz via `setInterval`.

### Step mode

Avviare il WBC con `step_mode:=true`:

```bash
ros2 launch spot_control wbc.launch.py step_mode:=true
```

Ogni transizione automatica tra stati mission (es. `SEARCHING → LOCKING`) viene bloccata e richiede conferma tramite il pulsante STEP o il tasto `n`. Transizioni di emergenza (TF loss, ESC, timeout) passano sempre.

---

## `camera_view.html` — Feed telecamere + YOLO

Pagina separata (apribile dal pannello controllo o indipendentemente) che mostra i feed live delle due telecamere con overlay dei dati YOLO.

### Layout

```
┌─────────────────────────────┬─────────────────────────────┐
│  ORBBEC — Spot view         │  REALSENSE — Z1 wrist view  │
│  [1280×720 px]              │  [1280×720 px]              │
│                             │                             │
│  feed camera +              │  feed camera                │
│  scheletro SMPL overlay      │  bordo verde/giallo per     │
│  (24 joint + connessioni)   │  stato torso tracker        │
│                             │                             │
├─────────────────────────────┴─────────────────────────────┤
│  LYING  |  85%  |  LOCKED  |  18/24                       │
├───────────────────────────────────────────────────────────┤
│  [← Back to Control] [⏸ Pause] [👁 Hide Orbbec] [👁 Hide RS] │
└───────────────────────────────────────────────────────────┘
```

### Topic sottoscritti

| Topic | Tipo | Uso |
|-------|------|-----|
| `/orbbec/color/image_raw` | `sensor_msgs/Image` | Feed Orbbec |
| `/orbbec/color/camera_info` | `sensor_msgs/CameraInfo` | Intrinseci per proiezione 3D→2D |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Feed RealSense |
| `/human_pose/points_3d` | `geometry_msgs/PoseArray` | Scheletro SMPL in 3D (24 keypoint) |
| `/human_pose/posture` | `std_msgs/String` | Postura rilevata |
| `/human_pose/posture_confidence` | `std_msgs/Float32` | Confidenza [0-1] |
| `/torso_tracker_state` | `std_msgs/String` | Stato tracker RealSense |

### Overlay YOLO su Orbbec

I 24 keypoint 3D vengono proiettati in 2D usando i parametri intrinseci della camera (fx, fy, cx, cy da CameraInfo):

```
u = fx * (x/z) + cx    v = fy * (y/z) + cy
```

Lo scheletro è disegnato con colori distinti per zona corporea:

| Zona | Colore |
|------|--------|
| Naso | 🟡 giallo |
| Viso (occhi, orecchie) | 🔵 ciano |
| Busto (spalle, gomiti, polsi) | 🟢 verde |
| Gambe (anche, ginocchia, caviglie) | 🟠 arancione |
| Connessioni | ⚪ bianco semitrasparente |

### Comandi camera view

| Pulsante | Tasto | Effetto |
|----------|:-----:|---------|
| ⏸ **Pause** | `p` / `Space` | Blocca/riprende tutto (immagini + scheletro) |
| 👁 **Hide Orbbec** | — | Ferma solo lo stream Orbbec. Canvas nero, **scheletro YOLO ancora visibile**. Nessuna banda consumata per le immagini. |
| 👁 **Hide RS** | — | Ferma solo lo stream RealSense. Canvas nero. |
| ← **Back to Control** | — | Torna alla pagina di controllo |

I pulsanti Hide sono **indipendenti**: puoi nascondere una camera e tenere l'altra attiva. I topic leggeri (skeleton, posture, confidenza, tracker state) restano sempre attivi.

### Bordo RealSense

Il pannello RealSense cambia colore in base allo stato del torso tracker:

| Stato | Bordo |
|-------|-------|
| `LOCKED` | 🟢 verde con glow |
| `ESTIMATING` / `TRACKING` | 🟡 giallo |
| `IDLE` / altro | grigio default |

---

## Architettura

```
Browser (localhost:8000)               Laptop (ROS 2)
┌──────────────────────┐     WebSocket     ┌──────────────────────────┐
│  teresa_control.html │◄───── :9090 ────►│  rosbridge_websocket     │
│  camera_view.html    │                   │     │                    │
│                      │                   │     ├─► /wbc/restart     │
│  roslibjs (CDN)      │                   │     ├─► /wbc/step_confirm│
│  • TFClient          │                   │     ├─► /my_spot/cmd_vel │
│  • Topic sub/pub     │                   │     ├─► /my_spot/stand   │
│  • Service call      │                   │     ├─► /my_spot/sit     │
│                      │                   │     ├─► image topics     │
│  Navigazione JS      │                   │     └─► TF, YOLO topics  │
│  (P-controller 10Hz) │                   │                          │
│                      │                   │  wbc_coordinator         │
│  Proiezione 3D→2D   │                   │  wbc_qp_controller       │
│  (skeleton overlay)  │                   │  tf_monitor              │
└──────────────────────┘                   └──────────────────────────┘
                                                      │
                                                      │ DDS / WiFi
                                                      ▼
                                              ┌──────────────────┐
                                              │  SpotCore (Spot) │
                                              │  • odom→body TF  │
                                              │  • body_pose     │
                                              │  • cmd_vel       │
                                              │  • stand/sit     │
                                              └──────────────────┘
```

Nessuna connessione diretta browser↔Spot. Tutto passa attraverso rosbridge sul laptop, che comunica con SpotCore via DDS/WiFi già configurata.

---

## Aggiungere nuove funzionalità

La pagina è modulare. Per aggiungere un nuovo pulsante o indicatore:

1. **Topic/subscriber**: aggiungi `new ROSLIB.Topic({...})` in `initTopics()` e un callback
2. **Publisher**: `new ROSLIB.Topic({...})` in `initTopics()` e `pub.publish(msg)` nell'action
3. **Servizio**: `new ROSLIB.Service({...})` in `initTopics()` e `client.callService(req, cb)` nell'action
4. **UI**: bottone HTML + metodo JS nella classe `TeresaController`
5. **TF**: `this.tfClient.subscribe(frame, callback)` per ricevere trasformate in tempo reale

L'area centrale (`#state-big` e `#state-detail`) è libera per widget futuri (grafici, metriche, visualizzazioni 3D).
