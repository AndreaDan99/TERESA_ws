#!/usr/bin/env python3
"""
Kalman Filter 3D per tracking posizione.
Stato: [x, y, z, vx, vy, vz] (posizione + velocità)
"""

import numpy as np


class Kalman3D:
    """
    Filtro di Kalman 3D con stato [x, y, z, vx, vy, vz].
    
    - predict(): chiamare ad OGNI frame (anche senza misura)
    - update():  chiamare SOLO quando la misura è disponibile
    - get_position(): ritorna [x, y, z] stimato
    """

    def __init__(self, dt=0.033, process_noise=0.01, measurement_noise=0.1):
        """
        Args:
            dt:                 timestep atteso in secondi (default ~30Hz)
            process_noise:      Q — quanto ti fidi del modello (basso = più liscio)
            measurement_noise:  R — quanto ti fidi della misura (basso = più reattivo)
        """
        self.dt = dt

        # ── Stato iniziale ─────────────────────────────────────────
        # [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)

        # ── Matrice di covarianza errore stato ─────────────────────
        self.P = np.eye(6) * 1.0

        # ── Matrice di transizione stato (modello moto uniforme) ───
        # x_new = x + vx*dt
        self.F = np.eye(6)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # ── Matrice di osservazione (vediamo solo posizione) ───────
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        # ── Rumore di processo Q ───────────────────────────────────
        self.Q = np.eye(6) * process_noise
        self.Q[3, 3] = process_noise * 2.0  # velocità più incerta
        self.Q[4, 4] = process_noise * 2.0
        self.Q[5, 5] = process_noise * 2.0

        # ── Rumore di misura R ─────────────────────────────────────
        self.R = np.eye(3) * measurement_noise

        # ── Flag inizializzazione ──────────────────────────────────
        self.initialized = False

    # ──────────────────────────────────────────────────────────────
    def initialize(self, position: np.ndarray):
        """Prima misura: inizializza stato con posizione, velocità zero."""
        self.x[:3] = position
        self.x[3:] = 0.0
        self.P     = np.eye(6) * 0.1
        self.initialized = True

    # ──────────────────────────────────────────────────────────────
    def predict(self, vel_damping: float = 1.0):
        """
        Passo di predizione — chiamare ad OGNI frame.
        
        Args:
            vel_damping: smorzamento velocità [0-1].
                         1.0 = moto uniforme (nessuno smorzamento)
                         0.8 = la velocità decade leggermente ogni frame
        """
        if not self.initialized:
            return

        # Aggiorna dt nella matrice F
        self.F[0, 3] = self.dt
        self.F[1, 4] = self.dt
        self.F[2, 5] = self.dt

        # Applica smorzamento velocità
        F_damp = self.F.copy()
        F_damp[3, 3] = vel_damping
        F_damp[4, 4] = vel_damping
        F_damp[5, 5] = vel_damping

        # Predizione stato e covarianza
        self.x = F_damp @ self.x
        self.P = F_damp @ self.P @ F_damp.T + self.Q

    # ──────────────────────────────────────────────────────────────
    def update(self, measurement: np.ndarray):
        """
        Passo di aggiornamento — chiamare SOLO quando misura disponibile.
        
        Args:
            measurement: np.array([x, y, z]) misurato
        """
        if not self.initialized:
            self.initialize(measurement)
            return

        # Innovazione
        z   = measurement
        y   = z - self.H @ self.x

        # Guadagno di Kalman
        S   = self.H @ self.P @ self.H.T + self.R
        K   = self.P @ self.H.T @ np.linalg.inv(S)

        # Aggiornamento stato e covarianza
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    # ──────────────────────────────────────────────────────────────
    def get_position(self) -> np.ndarray:
        """Ritorna la posizione stimata [x, y, z]."""
        return self.x[:3].copy()

    # ──────────────────────────────────────────────────────────────
    def get_velocity(self) -> np.ndarray:
        """Ritorna la velocità stimata [vx, vy, vz]."""
        return self.x[3:].copy()

    # ──────────────────────────────────────────────────────────────
    # ──────────────────────────────────────────────────────────────
    def get_position_covariance(self) -> np.ndarray:
        """Ritorna il blocco 3x3 della covarianza di posizione."""
        return self.P[:3, :3].copy()

    # ──────────────────────────────────────────────────────────────
    def reset(self):
        """Reset completo del filtro."""
        self.x           = np.zeros(6)
        self.P           = np.eye(6) * 1.0
        self.initialized = False
