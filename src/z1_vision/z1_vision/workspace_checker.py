#!/usr/bin/env python3
"""
WorkspaceChecker
================
Utility per verificare e clippare un target Cartesiano ai limiti fisici
del braccio robotico Z1 (o qualsiasi robot descritto da un URDF).

Dato un punto target in world frame e la posizione della base del braccio,
calcola il massimo raggiungibile lungo quella direzione tramite ottimizzazione
numerica con Pinocchio (L-BFGS-B) e clippa il target se necessario.

Uso tipico (chiamata bloccante, eseguire in un thread separato):
    checker = WorkspaceChecker(urdf_path)
    clipped, was_clipped, max_safe = checker.clip_target(target_pos)
"""

import numpy as np
import pinocchio as pin
from scipy.optimize import minimize


class WorkspaceChecker:

    def __init__(
        self,
        urdf_path: str,
        ee_frame: str = "link06",
        safety_margin: float = 0.05,
        n_restarts: int = 10,
    ):
        """
        Parameters
        ----------
        urdf_path      : path assoluto al file URDF del robot
        ee_frame       : nome del frame end-effector nel modello Pinocchio
        safety_margin  : margine di sicurezza [m] dal limite massimo (default 0.05 m)
        n_restarts     : numero di restart per l'ottimizzazione.
                         Più alto = più accurato ma più lento.
                         10 restart → ~100-300 ms su CPU moderna.
        """
        self.model         = pin.buildModelFromUrdf(urdf_path)
        self.data          = self.model.createData()
        self.ee_id         = self.model.getFrameId(ee_frame)
        self.safety_margin = safety_margin
        self.n_restarts    = n_restarts

        self._lb     = self.model.lowerPositionLimit
        self._ub     = self.model.upperPositionLimit
        self._bounds = list(zip(self._lb, self._ub))

    # ------------------------------------------------------------------ #
    #  CORE: massima proiezione lungo una direzione                        #
    # ------------------------------------------------------------------ #

    def _max_reach_in_direction(self, direction: np.ndarray) -> float:
        """
        Calcola la massima proiezione dell'EE lungo `direction`
        ottimizzando sullo spazio joint ammissibile.

        Returns
        -------
        max_reach : float  [m]
        """
        d = direction / np.linalg.norm(direction)

        def neg_proj(q: np.ndarray) -> float:
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            p = self.data.oMf[self.ee_id].translation
            return -float(np.dot(p, d))

        best = np.inf
        rng  = np.random.default_rng(seed=42)   # riproducibilità

        for _ in range(self.n_restarts):
            # Punto iniziale casuale dentro i joint limits
            q0  = rng.uniform(self._lb, self._ub)
            res = minimize(
                neg_proj,
                q0,
                method  = "L-BFGS-B",
                bounds  = self._bounds,
                options = {"maxiter": 300, "ftol": 1e-7},
            )
            if res.fun < best:
                best = res.fun

        return -best   # valore positivo = proiezione massima [m]

    # ------------------------------------------------------------------ #
    #  PUBLIC API                                                          #
    # ------------------------------------------------------------------ #

    def clip_target(
        self,
        target_pos: np.ndarray,
        arm_base_pos: np.ndarray | None = None,
    ) -> tuple[np.ndarray, bool, float]:
        """
        Clippa il target al massimo raggiungibile - safety_margin
        lungo la direzione base→target.

        Parameters
        ----------
        target_pos    : posizione target in world frame [x, y, z]
        arm_base_pos  : posizione base braccio in world frame.
                        Se None si assume origine (world == base frame).

        Returns
        -------
        clipped_pos   : np.ndarray [x, y, z]  — target (eventualmente clippato)
        was_clipped   : bool                   — True se il target era fuori workspace
        max_safe_dist : float [m]              — distanza massima sicura calcolata
        """
        if arm_base_pos is None:
            arm_base_pos = np.zeros(3)

        vec  = target_pos - arm_base_pos
        dist = np.linalg.norm(vec)

        if dist < 1e-6:
            return target_pos.copy(), False, 0.0

        direction  = vec / dist
        max_reach  = self._max_reach_in_direction(direction)
        max_safe   = max_reach - self.safety_margin

        if dist <= max_safe:
            # Target raggiungibile: nessuna modifica
            return target_pos.copy(), False, max_safe
        else:
            # Target fuori workspace: clippa lungo la stessa direzione
            max_safe_clamped = max(max_safe, 0.0)
            clipped = arm_base_pos + direction * max_safe_clamped
            return clipped, True, max_safe
