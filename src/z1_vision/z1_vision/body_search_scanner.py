#!/usr/bin/env python3
"""
BodySearchScanner
-----------------
Gestisce la scansione a griglia per trovare la vista ottimale del torso.

Macchina a stati interna (nessun nodo ROS):

  INIT → MOVING → RESETTING → COLLECTING ──┐
                     ↑                      │ avanza al punto successivo
                     └──────────────────────┘
                                            │ tutti i punti visitati o early stop
                                            ↓
                                       BEST_MOVING → EXITING → DONE
                                                   ↘ FAILED

Interazione con la FSM tramite ScanTick restituito da tick():
  SEND_IK(goal)     : FSM invia goal IK e resetta ik_done=False
  RESET_TRACKER     : FSM invia /tracker_scan_next=True
  WAIT              : FSM non fa nulla questo tick
  EXIT_SCAN_MODE    : FSM invia /tracker_scan_mode=False
  DONE(torso_xyz)   : scansione completata, FSM → WAITING
  FAILED            : nessun punto valido, FSM → WAITING

Il feed dati avviene tramite feed_scan_data(data) chiamata dalla FSM
ad ogni messaggio su /torso_scan_point.

Score per punto:
  final_score = detection_rate × avg_per_frame_score
  dove:
    detection_rate     = frame_validi / frame_totali
    per_frame_score    = (n_kp / max_kp) × avg_conf  (calcolato dal tracker)
    avg_per_frame_score = media dei per_frame_score validi
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

from geometry_msgs.msg import PoseStamped


# ── Azioni restituite da tick() ───────────────────────────────────────────────

class ScanAction(str, Enum):
    WAIT           = "WAIT"           # nessuna azione questo tick
    SEND_IK        = "SEND_IK"        # invia goal IK (campo goal popolato)
    RESET_TRACKER  = "RESET_TRACKER"  # invia /tracker_scan_next=True
    EXIT_SCAN_MODE = "EXIT_SCAN_MODE" # invia /tracker_scan_mode=False
    DONE           = "DONE"           # scansione OK (campo torso_xyz popolato)
    FAILED         = "FAILED"         # nessun punto valido trovato


@dataclass
class ScanTick:
    action:    ScanAction
    goal:      Optional[PoseStamped] = None     # valido se action == SEND_IK
    torso_xyz: Optional[np.ndarray]  = None     # valido se action == DONE


# ── Risultato per singolo punto della griglia ─────────────────────────────────

@dataclass
class _PointResult:
    arm_pose:  PoseStamped
    score:     float
    torso_xyz: np.ndarray   # mediana posizioni world frame valide in quel punto


# ── Stato interno ─────────────────────────────────────────────────────────────

class _St(str, Enum):
    INIT        = "INIT"        # genera waypoints, invia primo goal
    MOVING      = "MOVING"      # aspetta ik_done
    RESETTING   = "RESETTING"   # ha inviato RESET_TRACKER, attende 1 tick
    COLLECTING  = "COLLECTING"  # raccoglie dati dal tracker
    BEST_MOVING = "BEST_MOVING" # si sposta alla posa migliore
    EXITING     = "EXITING"     # disattiva scan mode (1 tick)
    DONE        = "DONE"
    FAILED      = "FAILED"


# ── Classe principale ─────────────────────────────────────────────────────────

class BodySearchScanner:
    """
    Classe pura (nessun nodo ROS).
    Viene istanziata dalla FSM e pilotata via tick() a 20 Hz.

    Parametri
    ---------
    scan_poses         : lista di PoseStamped (pose braccio da visitare in ordine)
    scan_point_timeout : [s] timeout per punto (avanza anche senza min_frames)
    scan_min_frames    : frame validi minimi per dichiarare il punto pronto
    early_stop_score   : score (0-1) oltre cui si interrompe la scansione anticipata
    logger             : riferimento al logger ROS (opzionale)
    """

    def __init__(
        self,
        scan_poses:         list[PoseStamped],
        scan_point_timeout: float = 4.0,
        scan_min_frames:    int   = 8,
        early_stop_score:   float = 0.85,
        logger: Any = None,
        transit_indices:    set[int] | None = None,
    ):
        self._poses          = scan_poses
        self._timeout        = scan_point_timeout
        self._min_frames     = scan_min_frames
        self._early_stop     = early_stop_score
        self._log            = logger
        self._transit        = transit_indices or set()

        # Stato della macchina a stati interna
        self._state:     _St               = _St.INIT
        self._idx:       int               = 0        # indice pose corrente
        self._results:   list[_PointResult] = []
        self._point_t0:  float             = 0.0      # timestamp inizio raccolta
        self._move_start: float            = -1.0     # timestamp inizio attesa IK (-1=non avviato)
        self._best_sent: bool              = False    # goal BEST_MOVING già inviato

        # Dati accumulati per il punto corrente
        self._frames_total: int         = 0
        self._frames_valid: int         = 0
        self._score_sum:    float       = 0.0
        self._positions:    list        = []  # list of np.ndarray

        # Buffer messaggi in arrivo (riempito da feed_scan_data, svuotato in tick)
        self._pending: list[list[float]] = []

    # ── API pubblica ──────────────────────────────────────────────────────────

    def reset(self):
        """Riparte da zero (chiamata quando si entra in BODY_SCANNING)."""
        self._state     = _St.INIT
        self._idx       = 0
        self._results   = []
        self._point_t0   = 0.0
        self._move_start = -1.0
        self._best_sent  = False
        self._pending    = []
        self._clear_point()

    def feed_scan_data(self, data: list[float]):
        """
        Chiamata dalla FSM ad ogni messaggio /torso_scan_point ricevuto.
        data = [score, n_kp, conf, x_world, y_world, z_world]
        """
        self._pending.append(data)

    def tick(self, ik_done: bool, now: float) -> ScanTick:
        """
        Chiamata ad ogni ciclo FSM (20 Hz).
        Restituisce ScanTick con l'azione che la FSM deve eseguire.
        """
        if   self._state == _St.INIT:        return self._t_init(now)
        elif self._state == _St.MOVING:      return self._t_moving(ik_done, now)
        elif self._state == _St.RESETTING:   return self._t_resetting(now)
        elif self._state == _St.COLLECTING:  return self._t_collecting(now)
        elif self._state == _St.BEST_MOVING: return self._t_best_moving(ik_done, now)
        elif self._state == _St.EXITING:
            self._state = _St.DONE
            return ScanTick(ScanAction.EXIT_SCAN_MODE)
        elif self._state == _St.DONE:
            best = self._best()
            return (ScanTick(ScanAction.DONE, torso_xyz=best.torso_xyz)
                    if best else ScanTick(ScanAction.FAILED))
        elif self._state == _St.FAILED:
            return ScanTick(ScanAction.FAILED)
        return ScanTick(ScanAction.WAIT)

    # ── Tick per stato ────────────────────────────────────────────────────────

    def _t_init(self, now: float) -> ScanTick:
        if not self._poses:
            self._state = _St.FAILED
            self._log_w("Nessuna posa di scansione disponibile → FAILED")
            return ScanTick(ScanAction.FAILED)
        self._idx = 0
        self._clear_point()
        self._state = _St.MOVING
        self._log_i(f"🔍 Body scan avviato: {len(self._poses)} pose da visitare")
        return ScanTick(ScanAction.SEND_IK, goal=self._poses[0])

    def _t_moving(self, ik_done: bool, now: float) -> ScanTick:
        if not ik_done:
            # Inizializza timer al primo tick di MOVING
            if self._move_start < 0.0:
                self._move_start = now
            # Timeout: IK/JTC non ha risposto → skip posa
            if now - self._move_start > self._timeout:
                self._log_w(
                    f"⏱️ IK timeout (>{self._timeout:.1f}s) posa "
                    f"{self._idx + 1}/{len(self._poses)} → skip"
                )
                # Le pose di transito non generano un risultato
                if self._idx not in self._transit:
                    self._results.append(_PointResult(
                        arm_pose  = self._poses[self._idx],
                        score     = 0.0,
                        torso_xyz = np.zeros(3),
                    ))
                self._idx += 1
                if self._idx >= len(self._poses):
                    best = self._best()
                    if best is None:
                        self._state = _St.FAILED
                        return ScanTick(ScanAction.FAILED)
                    self._state = _St.BEST_MOVING
                    return ScanTick(ScanAction.SEND_IK, goal=best.arm_pose)
                self._move_start = -1.0
                return ScanTick(ScanAction.SEND_IK, goal=self._poses[self._idx])
            return ScanTick(ScanAction.WAIT)
        # Arrivati alla posa
        self._move_start = -1.0
        self._pending.clear()

        # Posa di transito: nessuna raccolta dati, avanza subito alla successiva
        if self._idx in self._transit:
            self._log_i(f"🔀 Transito {self._idx + 1}/{len(self._poses)}: home intermedia → posa successiva")
            self._idx += 1
            if self._idx >= len(self._poses):
                best = self._best()
                if best is None:
                    self._state = _St.FAILED
                    return ScanTick(ScanAction.FAILED)
                self._state = _St.BEST_MOVING
                return ScanTick(ScanAction.SEND_IK, goal=best.arm_pose)
            self._move_start = -1.0
            return ScanTick(ScanAction.SEND_IK, goal=self._poses[self._idx])

        self._clear_point()
        self._point_t0 = now
        self._state    = _St.RESETTING
        self._log_i(f"📍 Punto {self._idx + 1}/{len(self._poses)}: arrivato → reset tracker")
        return ScanTick(ScanAction.RESET_TRACKER)

    def _t_resetting(self, now: float) -> ScanTick:
        # 1 tick di buffer dopo RESET_TRACKER prima di iniziare a raccogliere
        self._point_t0 = now
        self._state    = _St.COLLECTING
        return ScanTick(ScanAction.WAIT)

    def _t_collecting(self, now: float) -> ScanTick:
        # Consuma tutti i frame pendenti
        while self._pending:
            data = self._pending.pop(0)
            if len(data) >= 6:
                self._frames_total += 1
                score = float(data[0])
                if score > 0.0:
                    self._frames_valid += 1
                    self._score_sum    += score
                    self._positions.append(np.array(data[3:6], dtype=float))

        elapsed   = now - self._point_t0
        ready     = self._frames_valid >= self._min_frames
        timed_out = elapsed >= self._timeout

        if not ready and not timed_out:
            return ScanTick(ScanAction.WAIT)

        # ── Calcola e salva il risultato per questo punto ──
        final_score = self._compute_score()
        xyz_median  = (np.median(np.array(self._positions), axis=0)
                       if self._positions else np.zeros(3))
        self._results.append(_PointResult(
            arm_pose  = self._poses[self._idx],
            score     = final_score,
            torso_xyz = xyz_median,
        ))

        tag = "✅" if ready else "⏱️ timeout"
        self._log_i(
            f"{tag} Punto {self._idx + 1}/{len(self._poses)}: "
            f"score={final_score:.3f} "
            f"({self._frames_valid}/{self._frames_total} frame validi)"
        )

        # ── Early stop? ──
        if ready and final_score >= self._early_stop:
            self._log_i(f"🎯 Early stop: score={final_score:.3f} ≥ {self._early_stop}")
            self._idx  += 1
            self._state = _St.BEST_MOVING
            self._best_sent = False
            return ScanTick(ScanAction.WAIT)

        # ── Prossimo punto ──
        self._idx += 1
        if self._idx >= len(self._poses):
            # Tutti i punti visitati
            best = self._best()
            if best is None:
                self._state = _St.FAILED
                self._log_w("Nessun punto valido trovato → FAILED")
                return ScanTick(ScanAction.FAILED)
            self._state     = _St.BEST_MOVING
            self._best_sent = False
            return ScanTick(ScanAction.WAIT)

        # C'è ancora un punto da visitare → invia subito il goal
        self._clear_point()
        self._state = _St.MOVING
        return ScanTick(ScanAction.SEND_IK, goal=self._poses[self._idx])

    def _t_best_moving(self, ik_done: bool, now: float) -> ScanTick:
        if not self._best_sent:
            best = self._best()
            if best is None:
                self._state = _St.FAILED
                return ScanTick(ScanAction.FAILED)
            self._best_sent  = True
            self._move_start = now   # avvia timer timeout
            self._log_i(
                f"🏆 Verso best pose (score={best.score:.3f}) "
                f"torso=[{best.torso_xyz[0]:.3f}, "
                f"{best.torso_xyz[1]:.3f}, "
                f"{best.torso_xyz[2]:.3f}]"
            )
            return ScanTick(ScanAction.SEND_IK, goal=best.arm_pose)

        if not ik_done:
            # Timeout: se IK non risponde entro il timeout, procedi comunque
            if self._move_start >= 0.0 and now - self._move_start > self._timeout:
                self._log_w(
                    f"⏱️ Best-pose IK timeout (>{self._timeout:.1f}s) → EXITING comunque"
                )
                self._state = _St.EXITING
                return ScanTick(ScanAction.WAIT)
            return ScanTick(ScanAction.WAIT)

        # Arrivati alla best pose → disattiva scan mode
        self._state = _St.EXITING
        return ScanTick(ScanAction.WAIT)

    # ── Helper ────────────────────────────────────────────────────────────────

    def _compute_score(self) -> float:
        """
        Score del punto corrente:
          detection_rate × avg_per_frame_score
        dove avg_per_frame_score = mean((n_kp/max_kp) × avg_conf) dei frame validi.
        """
        if self._frames_total == 0:
            return 0.0
        detection_rate = self._frames_valid / self._frames_total
        avg_score      = (self._score_sum / self._frames_valid
                          if self._frames_valid > 0 else 0.0)
        return detection_rate * avg_score

    def best_arm_pose(self) -> Optional[PoseStamped]:
        """Ritorna la PoseStamped del punto con score massimo, o None."""
        best = self._best()
        return best.arm_pose if best is not None else None

    def _best(self) -> Optional[_PointResult]:
        valid = [r for r in self._results if r.score > 0.0]
        return max(valid, key=lambda r: r.score) if valid else None

    def fused_torso_xyz(
        self,
        anchor:   Optional[np.ndarray] = None,
        max_dist: Optional[float]      = None,
    ) -> Optional[np.ndarray]:
        """
        Stima fusa del centro torso: weighted average delle torso_xyz valide,
        pesata per score.

        Se `anchor` e `max_dist` sono forniti, applica outlier rejection:
          - Include solo le misure con |torso_xyz - anchor| <= max_dist
          - Se nessuna misura supera il filtro, ritorna l'anchor stesso
            (massima fiducia nella stima home)

        Ritorna None se nessun punto valido è disponibile.
        """
        valid = [r for r in self._results if r.score > 0.0]
        if not valid:
            return None

        if anchor is not None and max_dist is not None:
            consistent = [r for r in valid
                          if np.linalg.norm(r.torso_xyz - anchor) <= max_dist]
            if consistent:
                if len(consistent) < len(valid):
                    n_rej = len(valid) - len(consistent)
                    if self._log:
                        self._log.info(
                            f'🔍 Fusion: {n_rej} misure arco scartate '
                            f'(dist > {max_dist:.2f}m dall\'anchor home)'
                        )
                valid = consistent
            else:
                # Nessuna misura arco coerente → usa solo l'anchor
                if self._log:
                    self._log.warn(
                        f'⚠️  Fusion: tutte le misure arco lontane dall\'anchor '
                        f'(>{max_dist:.2f}m) → uso stima home'
                    )
                return anchor.copy()

        total_score = sum(r.score for r in valid)
        if total_score < 1e-9:
            return None
        return sum(r.torso_xyz * r.score for r in valid) / total_score

    def _clear_point(self):
        """Azzera i contatori del punto corrente."""
        self._frames_total = 0
        self._frames_valid = 0
        self._score_sum    = 0.0
        self._positions    = []

    def _log_i(self, msg: str):
        if self._log:
            self._log.info(msg)

    def _log_w(self, msg: str):
        if self._log:
            self._log.warn(msg)
