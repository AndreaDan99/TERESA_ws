#!/usr/bin/env python3
"""
z1_scan_manager.py
──────────────────
Gestione della sequenza di scansione fast-ultrasound per z1_FSM.

Incapsula tutta la logica scan (offsets, sequenza punti, SCAN_PRELIFT)
così che z1_FSM.py rimanga semplice.

Uso:
    mgr = ScanManager.from_params(node)   # crea da ROS parameters
    mgr.reset()                           # inizio nuovo ciclo
    mgr.advance()                         # passa al prossimo punto
    mgr.tick_prelift(fsm) -> bool         # tick SCAN_PRELIFT, True = done
    mgr.on_jtc_switch_success(fsm) -> str # dopo switch JTC, restituisce next state
"""

from __future__ import annotations

from builtin_interfaces.msg  import Duration as BuiltinDuration
from geometry_msgs.msg       import PoseStamped
from std_msgs.msg            import Bool
from visualization_msgs.msg  import Marker


class ScanManager:
    """
    Gestisce la modalità di scansione del braccio (single | fast_ultrasound).

    World frame (TERESA):
      X → verso il basso / verso il torso  (asse di approccio)
      Y → da faccia verso torso/gambe       (asse corpo, spalla → fianco)
      Z → da fianco dx a fianco sx          (asse laterale)

    Layout 5 punti fast_ultrasound (vista frontale, Z a destra):
              Z- (destra)   Z=0     Z+ (sinistra)
      Y- (spalla)  pt3                    pt4
      Y=0 (centro)           pt0
      Y+ (fianco)  pt1                    pt2
    """

    # ── Costanti modalità ─────────────────────────────────────────────────
    # MODE_SINGLE è l'unica costante hardcoded: tutto ciò che non è "single"
    # è trattato come scansione multi-punto. Il nome della modalità multi-punto
    # (es. "fast_ultrasound") è definito solo nel YAML (scan_mode).
    MODE_SINGLE = "single"

    # ── Costruttori ────────────────────────────────────────────────────────

    def __init__(
        self,
        mode:         str,
        offsets:      list[tuple[float, float, float]],
        clearance:    float,
        ik_timeout:   float,
    ):
        """
        Parameters
        ----------
        mode       : ScanManager.MODE_SINGLE | qualsiasi stringa dal YAML (scan_mode)
        offsets    : lista di (dx, dy, dz) in world frame; offsets[0] = centro
        clearance  : clearance aggiuntivo lungo -X durante SCAN_PRELIFT [m]
        ik_timeout : timeout IK identico a WAIT_IK_DONE [s]
        """
        self.mode       = mode
        self.offsets    = offsets
        self.clearance  = clearance
        self._ik_timeout = ik_timeout

        self.idx: int = 0

        # Posa di approccio del punto centrale (pt0), salvata al primo APPROACHING.
        # Usata da SCAN_PRELIFT come punto di ritorno dopo ogni punto non-centro.
        self._center_approach_pose = None   # PoseStamped | None

        # Stato interno SCAN_PRELIFT (macchina a 2 passi)
        self._prelift_sent:  bool        = False
        self._prelift_step:  int         = 0     # 0 = verso intermedio, 1 = verso centro
        self._prelift_start: float | None = None

    @classmethod
    def from_params(cls, node) -> "ScanManager":
        """
        Costruisce ScanManager leggendo i ROS parameters dal nodo.
        Dichiara i parametri se non già dichiarati.

        Parametri ROS letti:
          scan_mode             : "single" | qualsiasi stringa (default "single")
          scan_delta_lateral    : offset ±Z destra/sinistra [m]
          scan_delta_axial      : offset ±Y spalla/fianco [m]
          scan_center_y_offset  : shift +Y del centro rispetto alle spalle [m]
          scan_clearance_x      : clearance aggiuntivo lungo -X in SCAN_PRELIFT [m]
          wait_ik_timeout_s     : timeout IK condiviso con z1_FSM [s]
        """
        def _declare(name, default):
            try:
                node.declare_parameter(name, default)
            except Exception:
                pass   # già dichiarato
            return node.get_parameter(name).value

        mode = _declare("scan_mode",            "single")
        _dl  = float(_declare("scan_delta_lateral",   0.06))
        _da  = float(_declare("scan_delta_axial",     0.06))
        _cy  = float(_declare("scan_center_y_offset", 0.05))
        clr  = float(_declare("scan_clearance_x",     0.120))   # clearance lungo -X (asse approccio)
        tmo  = float(_declare("wait_ik_timeout_s",    15.0))

        # (dx, dy, dz): dx=0 sempre, dy=asse corpo (Y), dz=laterale (Z)
        offsets = [
            (0.0,  _cy,        0.0),   # 0: centro
            (0.0,  _cy + _da,  -_dl),  # 1: basso-destra
            (0.0,  _cy + _da,  +_dl),  # 2: basso-sinistra
            (0.0,  _cy - _da,  -_dl),  # 3: alto-destra
            (0.0,  _cy - _da,  +_dl),  # 4: alto-sinistra
        ]

        return cls(mode=mode, offsets=offsets, clearance=clr, ik_timeout=tmo)

    # ── Proprietà ─────────────────────────────────────────────────────────

    @property
    def current_offset(self) -> tuple[float, float, float]:
        """Offset (dx, dy, dz) del punto corrente in world frame."""
        return self.offsets[self.idx]

    @property
    def is_complete(self) -> bool:
        """True se siamo all'ultimo punto (nessun avanzamento possibile)."""
        return self.idx >= len(self.offsets) - 1

    # ── Controllo sequenza ────────────────────────────────────────────────

    def reset(self):
        """Resetta a inizio ciclo (chiamato su WAITING)."""
        self.idx = 0
        self.reset_prelift()

    def save_center_approach(self, pose) -> None:
        """Salva la posa JTC del punto centrale (idx=0) per il ritorno in SCAN_PRELIFT."""
        self._center_approach_pose = pose

    def reset_prelift(self):
        """Resetta lo stato interno di SCAN_PRELIFT (chiamato su set_state)."""
        self._prelift_sent  = False
        self._prelift_step  = 0
        self._prelift_start = None

    def advance(self):
        """Avanza al prossimo punto. Chiamato da on_jtc_switch_success."""
        if self.idx < len(self.offsets) - 1:
            self.idx += 1

    # ── Tick SCAN_PRELIFT ─────────────────────────────────────────────────

    def tick_prelift(self, fsm) -> bool:
        """
        SCAN_PRELIFT a 2 passi dopo ogni punto non-centro:

        Passo 0 — Intermedio: JTC va a (centro.x, pt_n.y, pt_n.z)
                  Stessa Y/Z del punto appena fatto, X del centro.
                  Così il braccio si allontana dal torso lungo -X
                  senza cambiare posizione laterale.

        Passo 1 — Centro: JTC torna alla posa di approccio del centro (pt0).
                  Movimento laterale sicuro alla distanza X del centro.

        Poi: advance() → APPROACHING sul prossimo punto.

        Returns True quando entrambi i passi sono completati.
        """
        # ── Passo 0: invia goal intermedio (centro.x, pt_n.y, pt_n.z) ──────
        if not self._prelift_sent:
            # Punto intermedio: approach pose del punto corrente (idx non ancora avanzato)
            # ma con X del centro → stessa formula standoff, solo X sostituita
            target = fsm._make_approach_pose()   # Y, Z del punto corrente
            if target is None:
                return False
            if self._center_approach_pose is not None:
                # Sostituisce la X con quella del centro (stessa distanza dal torso)
                target.pose.position.x = self._center_approach_pose.pose.position.x

            fsm.ik_done = False
            fsm.pub_ik_goal.publish(target)
            fsm.pub_ik_enable.publish(Bool(data=True))
            self._prelift_sent  = True
            self._prelift_step  = 0
            self._prelift_start = fsm.get_clock().now().nanoseconds * 1e-9
            self._publish_marker(fsm, target)
            off = self.current_offset
            fsm.get_logger().info(
                f"🔼 SCAN_PRELIFT passo0: intermedio pt{self.idx} "
                f"(x_centro, y={target.pose.position.y:.3f}, z={target.pose.position.z:.3f})"
            )
            return False

        # ── Timeout ──────────────────────────────────────────────────────────
        if self._prelift_start is not None:
            elapsed = fsm.get_clock().now().nanoseconds * 1e-9 - self._prelift_start
            if elapsed > self._ik_timeout:
                fsm.get_logger().warn(
                    f"⏱️  SCAN_PRELIFT timeout passo{self._prelift_step} ({elapsed:.1f}s) → WAITING"
                )
                fsm.pub_ik_enable.publish(Bool(data=False))
                fsm.set_state(fsm.WAITING)
                return False

        if not fsm.ik_done:
            return False

        # ── Passo 0 completato → invia goal centro ───────────────────────────
        if self._prelift_step == 0:
            if self._center_approach_pose is None:
                # Centro non salvato: salta direttamente ad advance
                fsm.pub_ik_enable.publish(Bool(data=False))
                self.advance()
                return True

            fsm.ik_done = False
            fsm.pub_ik_goal.publish(self._center_approach_pose)
            # ik_enable già True
            self._prelift_step  = 1
            self._prelift_start = fsm.get_clock().now().nanoseconds * 1e-9
            fsm.get_logger().info(
                f"🔼 SCAN_PRELIFT passo1: ritorno al centro "
                f"(x={self._center_approach_pose.pose.position.x:.3f}, "
                f"y={self._center_approach_pose.pose.position.y:.3f}, "
                f"z={self._center_approach_pose.pose.position.z:.3f})"
            )
            return False

        # ── Passo 1 completato → advance → APPROACHING prossimo punto ────────
        fsm.pub_ik_enable.publish(Bool(data=False))
        self.advance()
        off = self.current_offset
        fsm.get_logger().info(
            f"✅ SCAN_PRELIFT completato → APPROACHING pt{self.idx} "
            f"off=({off[0]:.2f},{off[1]:.2f},{off[2]:.2f})"
        )
        return True

    # ── Decisione dopo SWITCHING_TO_JTC ──────────────────────────────────

    def on_jtc_switch_success(self, fsm) -> str:
        """
        Chiamato quando lo switch JTC riesce.
        Decide lo stato successivo e aggiorna l'indice.

        Returns
        -------
        "SCAN_PRELIFT" → ci sono altri punti da scansionare
        "HOMING"       → scansione completata, si torna a home
        """
        if self.mode != self.MODE_SINGLE and not self.is_complete:
            # NON avanzare qui: SCAN_PRELIFT deve prima salire dal punto CORRENTE
            # (stesso Y/Z), poi advance() viene chiamato al termine del PRELIFT.
            off = self.current_offset
            fsm.get_logger().info(
                f"✅ Switch → JTC | Fast US: pt{self.idx} completato → SCAN_PRELIFT "
                f"(sale da off=({off[0]:.2f},{off[1]:.2f},{off[2]:.2f}))"
            )
            return "SCAN_PRELIFT"
        else:
            self.reset()
            fsm.get_logger().info(
                "✅ Switch torque_controller → JTC | scansione completa → HOMING"
            )
            return "HOMING"

    # ── Helpers interni ───────────────────────────────────────────────────

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
