#!/usr/bin/env python3
"""
Kalman Filter 3D per tracking keypoints skeleton - OTTIMIZZATO.
"""
import numpy as np

class Kalman3D:
    """
    6-state Kalman Filter per posizione 3D con velocity damping.
    State: [x, y, z, vx, vy, vz]
    """
    def __init__(self, dt=1/30, q=0.2, r=0.02, p0=1.0):
        self.dt = float(dt)
        self.x = np.zeros((6,1), dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * p0
        
        # Transition matrix (constant velocity model)
        self.F = np.eye(6, dtype=np.float64)
        self.F[0,3] = self.F[1,4] = self.F[2,5] = self.dt
        
        # Observation matrix (measure position only)
        self.H = np.zeros((3,6), dtype=np.float64)
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1.0
        
        # Process/measurement noise
        self.Q_base = np.eye(6, dtype=np.float64) * q
        self.Q = self.Q_base.copy()
        self.R = np.eye(3, dtype=np.float64) * r
        
        # NUOVO: Pre-allocate matrici per evitare allocazioni ripetute
        self._F_T = self.F.T.copy()
        self._H_T = self.H.T.copy()
        self._I6 = np.eye(6, dtype=np.float64)
        
        self.initialized = False

    def predict(self, vel_damping=1.0):
        """Prediction step con velocity damping - OTTIMIZZATO."""
        # x = F @ x
        np.dot(self.F, self.x, out=self.x)
        
        # P = F @ P @ F.T + Q (usa temporary per evitare allocazioni)
        temp = self.F @ self.P
        self.P = temp @ self._F_T
        self.P += self.Q
        
        # Velocity damping
        self.x[3:,0] *= float(vel_damping)

    def update(self, z):
        """Update step con measurement z (3D position) - OTTIMIZZATO."""
        z = np.asarray(z, dtype=np.float64).reshape(3,1)
        
        if not self.initialized:
            self.x[0:3] = z
            self.x[3:] = 0.0
            self.initialized = True
            return
        
        # Innovation y = z - H @ x
        y = z - self.H @ self.x
        
        # Innovation covariance S = H @ P @ H.T + R
        # H è semplice [I_3x3 | 0_3x3], quindi H@P prende solo primi 3 cols
        HP = self.P[:3, :]  # Equivalente a H @ P ma più veloce (no multiplication)
        S = HP[:, :3] + self.R  # H @ P @ H.T + R (H.T seleziona primi 3 rows)
        
        # Kalman gain K = P @ H.T @ inv(S)
        PH_T = self.P[:, :3]  # P @ H.T (H.T seleziona primi 3 cols)
        K = np.linalg.solve(S.T, PH_T.T).T  # Risolve S.T @ K.T = PH_T.T
        
        # State update x = x + K @ y
        self.x += K @ y
        
        # Covariance update - JOSEPH FORM (numerically stable)
        # P = (I - K@H) @ P @ (I - K@H).T + K @ R @ K.T
        I_KH = self._I6 - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + (K @ self.R) @ K.T

    def get_position(self):
        """Ritorna posizione stimata [x, y, z]."""
        if not self.initialized:
            return None
        return self.x[0:3,0].copy()

    def set_position(self, p):
        """Forza posizione (per re-init)."""
        self.x[0:3,0] = np.asarray(p, dtype=np.float64).reshape(3)
        self.initialized = True


class Kalman3DFast:
    """
    ALTERNATIVA: Versione ultra-ottimizzata con matrici semplificate.
    Sfrutta il fatto che H = [I_3x3 | 0_3x3] per evitare moltiplicazioni.
    """
    def __init__(self, dt=1/30, q=0.2, r=0.02, p0=1.0):
        self.dt = float(dt)
        self.pos = np.zeros(3, dtype=np.float64)  # [x, y, z]
        self.vel = np.zeros(3, dtype=np.float64)  # [vx, vy, vz]
        
        # Covariance separata per pos e vel (6x6 → 2x 3x3)
        self.P_pos = np.eye(3, dtype=np.float64) * p0
        self.P_vel = np.eye(3, dtype=np.float64) * p0
        self.P_cross = np.zeros((3,3), dtype=np.float64)  # Cross-covariance
        
        # Noise
        self.q_pos = q
        self.q_vel = q
        self.r = r
        
        self.initialized = False
    
    def predict(self, vel_damping=1.0):
        """Prediction step semplificato."""
        # x = x + dt*v
        self.pos += self.dt * self.vel
        
        # Covariance prediction (simplified)
        self.P_pos += self.dt * (self.P_cross + self.P_cross.T) + \
                      self.dt**2 * self.P_vel + self.q_pos * np.eye(3)
        self.P_cross += self.dt * self.P_vel
        self.P_vel += self.q_vel * np.eye(3)
        
        # Velocity damping
        self.vel *= vel_damping
    
    def update(self, z):
        """Update step semplificato (only position measurement)."""
        if not self.initialized:
            self.pos = np.asarray(z, dtype=np.float64)
            self.vel[:] = 0.0
            self.initialized = True
            return
        
        z = np.asarray(z, dtype=np.float64)
        
        # Innovation
        y = z - self.pos
        
        # Innovation covariance
        S = self.P_pos + self.r * np.eye(3)
        
        # Kalman gain (solve invece di inv)
        K_pos = np.linalg.solve(S, self.P_pos).T
        K_vel = np.linalg.solve(S, self.P_cross.T).T
        
        # Update state
        innovation = K_pos @ y
        self.pos += innovation
        self.vel += K_vel @ y
        
        # Update covariance (simplified Joseph form)
        I_K = np.eye(3) - K_pos
        self.P_pos = I_K @ self.P_pos @ I_K.T + K_pos * self.r @ K_pos.T
        self.P_cross = (np.eye(3) - K_vel) @ self.P_cross
    
    def get_position(self):
        """Ritorna posizione stimata."""
        if not self.initialized:
            return None
        return self.pos.copy()
    
    def set_position(self, p):
        """Forza posizione."""
        self.pos = np.asarray(p, dtype=np.float64)
        self.initialized = True
