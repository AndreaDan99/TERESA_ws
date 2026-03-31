#!/usr/bin/env python3
"""
z1_scan_manager.py
──────────────────
Gestione della sequenza di scansione fast-ultrasound FAST per z1_FSM.

Protocollo FAST (Focused Assessment with Sonography in Trauma):
  4 finestre ecografiche calcolate dai keypoint 3D YOLO del torso.
  Tutti i punti sono clippati a restare dentro [shoulder_mid, hip_mid].

Struttura sequenza:
  idx=0 : Centro hub (navigazione, NESSUNA misura impedance)
  idx=1 : Sottoxifoidea  (cardiaca)
  idx=2 : RUQ - Morrison's pouch (fianco destro)
  idx=3 : LUQ - Koller's pouch  (fianco sinistro)
  idx=4 : Sovrapubica  (pelvica)

Calcolo punti FAST da keypoint anatomici:
  kp5  = spalla lontana (+X in world frame)
  kp6  = spalla vicina  (-X in world frame)
  kp11 = fianco lontano (+X)
  kp12 = fianco vicino  (-X)

  shoulder_mid  = (kp5 + kp6) / 2
  hip_mid       = (kp11 + kp12) / 2
  body_axis     = normalize(hip_mid - shoulder_mid)   [head → feet]
  lateral_axis  = normalize(kp5 - kp6)                [right → left]
  torso_len     = |hip_mid - shoulder_mid|
  shoulder_width = |kp5 - kp6|

  pt_subxiphoid  = shoulder_mid + ratio_subxiphoid_body * torso_len * body_axis
  pt_ruq         = shoulder_mid + ratio_ruq_body * torso_len * body_axis
                                - ratio_ruq_lat  * shoulder_width * lateral_axis
  pt_luq         = shoulder_mid + ratio_luq_body * torso_len * body_axis
                                + ratio_luq_lat  * shoulder_width * lateral_axis
  pt_suprapubic  = hip_mid      - ratio_suprapubic_body * torso_len * body_axis

  Tutti i punti vengono clippati: Y ∈ [shoulder_mid_y, hip_mid_y].

Gli offset sono espressi come (0, dy, dz) RELATIVI al torso_center.
World frame TERESA: X=approccio, Y=testa→piedi, Z=destra→sinistra.

Uso:
    mgr = ScanManager.from_params(node)
    mgr.set_fast_points(kp_xyz, torso_center)  # dopo body scan
    mgr.reset()
    mgr.tick_prelift(fsm)  -> bool
    mgr.on_jtc_switch_success(fsm) -> str
"""

from __future__ import annotations

import numpy as np

from builtin_interfaces.msg  import Duration as BuiltinDuration
from geometry_msgs.msg       import PoseStamped
from std_msgs.msg            import Bool
from visualization_msgs.msg  import Marker


# Nomi per logging dei 5 slot (idx 0..4)
FAST_POINT_NAMES = [
    "Centro (hub)",
    "Sottoxifoidea",
    "RUQ (Morrison's pouch)",
    "LUQ (Koller's pouch)",
    "Sovrapubica",
]


class ScanManager:
    """
    Gestisce la modalità di scansione del braccio.

    Modalità:
      MODE_SINGLE       → singolo punto al centro torso
      qualsiasi altra   → protocollo FAST (5 slot: centro hub + 4 punti FAST)

    World frame (TERESA):
      X → verso il torso  (asse di approccio)
      Y → testa → piedi   (asse corpo)
      Z → destra → sinistra (laterale)
    """

    MODE_SINGLE = "single"

    # ── Costruttori ────────────────────────────────────────────────────────

    def __init__(
        self,
        mode:          str,
        clearance:     float,
        ik_timeout:    float,
        scan_pause_s:  float,
        # Ratios anatomici FAST
        ratio_subxiphoid_body:  float,
        ratio_ruq_body:         float,
        ratio_ruq_lat:          float,
        ratio_luq_body:         float,
        ratio_luq_lat:          float,
        ratio_suprapubic_body:  float,
    ):
        self.mode          = mode
        self.clearance     = clearance
        self._ik_timeout   = ik_timeout
        self.scan_pause_s  = scan_pause_s

        # Ratios anatomici
        self._ratio_subxiphoid_body = ratio_subxiphoid_body
        self._ratio_ruq_body        = ratio_ruq_body
        self._ratio_ruq_lat         = ratio_ruq_lat
        self._ratio_luq_body        = ratio_luq_body
        self._ratio_luq_lat         = ratio_luq_lat
        self._ratio_suprapubic_body = ratio_suprapubic_body

        # Offsets (dx, dy, dz) relativi al torso_center in world frame.
        # offsets[0] = (0,0,0) = centro hub
        # offsets[1..5] = punti FAST (calcolati da set_fast_points)
        # Default = solo centro (single mode o prima che set_fast_points sia chiamato)
        self.offsets: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]

        self.idx: int = 0

        # Posa di approccio del centro (pt0), salvata al primo APPROACHING.
        self._center_approach_pose = None

        # Stato interno SCAN_PRELIFT
        self._prelift_sent:  bool         = False
        self._prelift_step:  int          = 0
        self._prelift_start: float | None = None

    @classmethod
    def from_params(cls, node) -> "ScanManager":
        """
        Costruisce ScanManager leggendo i ROS parameters dal nodo.

        Parametri ROS letti:
          scan_mode             : "single" | "fast_ultrasound" (default "single")
          scan_clearance_x      : clearance -X in SCAN_PRELIFT [m]
          wait_ik_timeout_s     : timeout IK [s]
          scan_pause_s          : pausa [s] al centro tra un punto FAST e l'altro
          fast_*                : ratios anatomici per i 5 punti FAST
        """
        def _declare(name, default):
            try:
                node.declare_parameter(name, default)
            except Exception:
                pass
            return node.get_parameter(name).value

        mode    = _declare("scan_mode",             "single")
        clr     = float(_declare("scan_clearance_x",      0.120))
        tmo     = float(_declare("wait_ik_timeout_s",     15.0))
        pause   = float(_declare("scan_pause_s",           2.0))

        # Ratios anatomici FAST (tutti configurabili via YAML)
        r_sub_b  = float(_declare("fast_subxiphoid_body_ratio",  0.25))
        r_ruq_b  = float(_declare("fast_ruq_body_ratio",         0.40))
        r_ruq_l  = float(_declare("fast_ruq_lat_ratio",          0.50))
        r_luq_b  = float(_declare("fast_luq_body_ratio",         0.35))
        r_luq_l  = float(_declare("fast_luq_lat_ratio",          0.60))
        r_sup_b  = float(_declare("fast_suprapubic_body_ratio",  0.15))

        return cls(
            mode=mode, clearance=clr, ik_timeout=tmo, scan_pause_s=pause,
            ratio_subxiphoid_body=r_sub_b,
            ratio_ruq_body=r_ruq_b, ratio_ruq_lat=r_ruq_l,
            ratio_luq_body=r_luq_b, ratio_luq_lat=r_luq_l,
            ratio_suprapubic_body=r_sup_b,
        )

    # ── Proprietà ─────────────────────────────────────────────────────────

    @property
    def current_offset(self) -> tuple[float, float, float]:
        """Offset (dx, dy, dz) del punto corrente in world frame."""
        if self.idx < len(self.offsets):
            return self.offsets[self.idx]
        return (0.0, 0.0, 0.0)

    @property
    def is_complete(self) -> bool:
        """True se siamo all'ultimo punto (nessun avanzamento possibile)."""
        return self.idx >= len(self.offsets) - 1

    @property
    def is_center_hub(self) -> bool:
        """True se il punto corrente è il centro hub (idx=0, fast mode)."""
        return self.mode != self.MODE_SINGLE and self.idx == 0

    @property
    def current_name(self) -> str:
        """Nome del punto corrente per logging."""
        if self.mode == self.MODE_SINGLE:
            return "singolo"
        if self.idx < len(FAST_POINT_NAMES):
            return FAST_POINT_NAMES[self.idx]
        return f"pt{self.idx}"

    @property
    def next_name(self) -> str:
        """Nome del prossimo punto per logging."""
        if self.mode == self.MODE_SINGLE:
            return "singolo"
        next_idx = self.idx + 1
        if next_idx < len(FAST_POINT_NAMES):
            return FAST_POINT_NAMES[next_idx]
        return f"pt{next_idx}"

    @property
    def fast_total(self) -> int:
        """Numero totale di punti FAST (escluso centro hub)."""
        return max(0, len(self.offsets) - 1)

    @property
    def fast_current(self) -> int:
        """Indice del punto FAST corrente (1-based, 0 se centro hub)."""
        return max(0, self.idx)

    # ── Calcolo punti FAST ─────────────────────────────────────────────────

    def set_fast_points(
        self,
        kp_xyz:       np.ndarray,
        torso_center: np.ndarray,
    ) -> bool:
        """
        Calcola i 5 punti FAST dai keypoint anatomici e li salva come
        offset (0, dy, dz) relativi al torso_center.

        Parameters
        ----------
        kp_xyz       : np.ndarray shape (4, 3) — [kp5, kp6, kp11, kp12] in world frame.
                       NaN per keypoint non rilevato.
        torso_center : np.ndarray shape (3,)   — centro torso fuso dal body scan.

        Returns True se il calcolo è riuscito, False se i keypoint sono insufficienti
        (in tal caso mantiene gli offset di default = solo centro).
        """
        if kp_xyz is None or kp_xyz.shape != (4, 3):
            return False

        kp5, kp6, kp11, kp12 = kp_xyz[0], kp_xyz[1], kp_xyz[2], kp_xyz[3]

        # Controlla che spalle e fianchi siano disponibili
        shoulders_ok = not (np.isnan(kp5).any() or np.isnan(kp6).any())
        hips_ok      = not (np.isnan(kp11).any() or np.isnan(kp12).any())

        if not shoulders_ok:
            return False  # spalle obbligatorie (definiscono gli assi)

        shoulder_mid = (kp5 + kp6) / 2.0

        if hips_ok:
            hip_mid = (kp11 + kp12) / 2.0
        else:
            # Fallback: stima fianchi come shoulder_mid + 0.3m lungo Y world
            hip_mid = shoulder_mid + np.array([0.0, 0.30, 0.0])

        # ── Assi anatomici ─────────────────────────────────────────────
        body_vec = hip_mid - shoulder_mid
        body_len = float(np.linalg.norm(body_vec))
        if body_len < 0.05:   # < 5 cm: non ha senso
            return False
        body_axis = body_vec / body_len

        lat_vec = kp5 - kp6
        lat_len = float(np.linalg.norm(lat_vec))
        if lat_len < 0.02:
            # Fallback: asse Z world come laterale
            lateral_axis = np.array([0.0, 0.0, 1.0])
            shoulder_width = 0.35  # larghezza spalle adulto media [m]
        else:
            lateral_axis   = lat_vec / lat_len
            shoulder_width = lat_len

        # ── 5 punti FAST in world frame ────────────────────────────────
        pt_subxiphoid = (shoulder_mid
                         + self._ratio_subxiphoid_body * body_len * body_axis)

        pt_ruq        = (shoulder_mid
                         + self._ratio_ruq_body * body_len * body_axis
                         - self._ratio_ruq_lat  * shoulder_width * lateral_axis)

        pt_luq        = (shoulder_mid
                         + self._ratio_luq_body * body_len * body_axis
                         + self._ratio_luq_lat  * shoulder_width * lateral_axis)

        # Sovrapubica: DENTRO i fianchi (- = verso le spalle rispetto a hip_mid)
        pt_suprapubic = (hip_mid
                         - self._ratio_suprapubic_body * body_len * body_axis)

        # ── Clipping: tutti i punti restano dentro [shoulder_mid_y, hip_mid_y] ──
        y_min = float(shoulder_mid[1])   # Y spalle (più verso testa)
        y_max = float(hip_mid[1])        # Y fianchi (più verso piedi)

        def _clip_y(pt):
            """Clippa la coordinata Y del punto al bounding box torso."""
            clipped = pt.copy()
            clipped[1] = float(np.clip(pt[1], y_min, y_max))
            return clipped

        # ── Converti in offset relativi al torso_center ────────────────
        def _rel(pt):
            d = _clip_y(pt) - torso_center
            return (0.0, float(d[1]), float(d[2]))   # dx=0 sempre (X gestito da surface)

        self.offsets = [
            (0.0, 0.0, 0.0),      # idx 0: centro hub
            _rel(pt_subxiphoid),  # idx 1
            _rel(pt_ruq),         # idx 2
            _rel(pt_luq),         # idx 3
            _rel(pt_suprapubic),  # idx 4
        ]
        return True

    # ── Controllo sequenza ────────────────────────────────────────────────

    def reset(self):
        """Resetta a inizio ciclo."""
        self.idx = 0
        self.reset_prelift()

    def save_center_approach(self, pose) -> None:
        """Salva la posa JTC del centro (idx=0) per uso in APPROACHING e PRELIFT."""
        self._center_approach_pose = pose

    def reset_prelift(self):
        """Resetta lo stato interno di SCAN_PRELIFT."""
        self._prelift_sent  = False
        self._prelift_step  = 0
        self._prelift_start = None

    def advance(self):
        """Avanza al prossimo punto."""
        if self.idx < len(self.offsets) - 1:
            self.idx += 1

    # ── Tick SCAN_PRELIFT ─────────────────────────────────────────────────

    def tick_prelift(self, fsm) -> bool:
        """
        SCAN_PRELIFT a 2 passi dopo ogni punto FAST:

        Passo 0 — Intermedio: stessa X del centro, Y/Z del punto appena misurato.
        Passo 1 — Centro: ritorna alla posa di approccio del centro (hub).

        Poi: advance() → FSM va in SCAN_PAUSE → poi APPROACHING prossimo punto.

        Returns True quando entrambi i passi completati.
        """
        if not self._prelift_sent:
            if self._center_approach_pose is None:
                fsm.get_logger().warn('⚠️  SCAN_PRELIFT: center_approach_pose non salvata, skip')
                self.advance()
                return True

            off_n = self.current_offset
            off_c = self.offsets[0]
            c     = self._center_approach_pose

            target = PoseStamped()
            target.header.frame_id  = 'world'
            target.header.stamp     = fsm.get_clock().now().to_msg()
            target.pose.position.x  = c.pose.position.x
            target.pose.position.y  = c.pose.position.y + (off_n[1] - off_c[1])
            target.pose.position.z  = c.pose.position.z + (off_n[2] - off_c[2])
            target.pose.orientation = c.pose.orientation

            fsm.ik_done = False
            fsm.pub_ik_enable.publish(Bool(data=True))
            fsm.pub_ik_goal.publish(target)
            self._prelift_sent  = True
            self._prelift_step  = 0
            self._prelift_start = fsm.get_clock().now().nanoseconds * 1e-9
            self._publish_marker(fsm, target)
            fsm.get_logger().info(
                f"🔼 PRELIFT passo0 {self.current_name}: intermedio "
                f"x={target.pose.position.x:.3f} "
                f"y={target.pose.position.y:.3f} "
                f"z={target.pose.position.z:.3f}"
            )
            return False

        # Timeout
        if self._prelift_start is not None:
            elapsed = fsm.get_clock().now().nanoseconds * 1e-9 - self._prelift_start
            if elapsed > self._ik_timeout:
                fsm.get_logger().warn(
                    f"⏱️  PRELIFT timeout passo{self._prelift_step} ({elapsed:.1f}s) → WAITING"
                )
                fsm.pub_ik_enable.publish(Bool(data=False))
                fsm.set_state(fsm.WAITING)
                return False

        if not fsm.ik_done:
            return False

        # Passo 0 completato → invia goal centro
        if self._prelift_step == 0:
            if self._center_approach_pose is None:
                fsm.get_logger().warn('⚠️  PRELIFT: center_approach_pose non salvata, skip passo1')
                fsm.pub_ik_enable.publish(Bool(data=False))
                self.advance()
                return True

            c = self._center_approach_pose
            fsm.ik_done = False
            fsm.pub_ik_enable.publish(Bool(data=True))
            fsm.pub_ik_goal.publish(c)
            self._prelift_step  = 1
            self._prelift_start = fsm.get_clock().now().nanoseconds * 1e-9
            fsm.get_logger().info(
                f"🔼 PRELIFT passo1: ritorno centro "
                f"x={c.pose.position.x:.3f} "
                f"y={c.pose.position.y:.3f} "
                f"z={c.pose.position.z:.3f}"
            )
            return False

        # Passo 1 completato → advance
        fsm.pub_ik_enable.publish(Bool(data=False))
        self.advance()
        fsm.get_logger().info(
            f"✅ PRELIFT completato → centro raggiunto "
            f"(prossimo: {self.current_name})"
        )
        return True

    # ── Decisione dopo SWITCHING_TO_JTC ──────────────────────────────────

    def on_jtc_switch_success(self, fsm) -> str:
        """
        Chiamato quando lo switch JTC riesce dopo impedance.

        Returns
        -------
        "SCAN_PRELIFT" → ci sono altri punti FAST da visitare
        "HOMING"       → scansione completata
        """
        if self.mode != self.MODE_SINGLE and not self.is_complete:
            off = self.current_offset
            fsm.get_logger().info(
                f"✅ Switch → JTC | FAST [{self.fast_current}/{self.fast_total}] "
                f"{self.current_name} completato → PRELIFT"
            )
            return "SCAN_PRELIFT"
        else:
            self.reset()
            fsm.get_logger().info(
                f"✅ Switch → JTC | Scansione FAST completata "
                f"({self.fast_total}/{self.fast_total} punti) → HOMING"
            )
            return "HOMING"

    # ── Helpers ───────────────────────────────────────────────────────────

    def _publish_marker(self, fsm, target: PoseStamped):
        """Marker arancione per visualizzare il target SCAN_PRELIFT in RViz."""
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp    = fsm.get_clock().now().to_msg()
        m.ns = "ik_goal"; m.id = 1
        m.type   = Marker.SPHERE
        m.action = Marker.ADD
        m.pose   = target.pose
        m.scale.x = m.scale.y = m.scale.z = 0.06
        m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.9
        m.lifetime = BuiltinDuration(sec=30, nanosec=0)
        fsm.pub_ik_goal_marker.publish(m)
