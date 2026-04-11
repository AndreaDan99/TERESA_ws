#!/usr/bin/env python3
"""
Utility functions per skeleton processing - OTTIMIZZATO.
"""
import numpy as np

# COCO-17 skeleton edges (parent -> child)
SKELETON_EDGES = [
    (0,1), (0,2),      # nose -> eyes
    (1,3), (2,4),      # eyes -> ears
    (0,5), (0,6),      # nose -> shoulders
    (5,7), (7,9),      # left arm
    (6,8), (8,10),     # right arm
    (5,6),             # shoulders
    (5,11), (6,12),    # shoulders -> hips
    (11,12),           # hips
    (11,13), (13,15),  # left leg
    (12,14), (14,16)   # right leg
]

# Costanti anatomiche (valori tipici umani in metri)
TORSO_MIN = 0.25  # bambini/persone sedute compresse
TORSO_MAX = 0.65  # adulti alti
TORSO_TYPICAL = 0.45  # media adulti


def torso_length_constraint(pts, visible, L_ref, stiffness=0.35):
    """
    Applica constraint lunghezza torso (shoulders-hips) - OTTIMIZZATO.
    
    Args:
        pts: lista 17 keypoints [x,y,z] o None
        visible: lista 17 bool (True = detected, False = predicted)
        L_ref: lunghezza torso di riferimento (None = skip)
        stiffness: peso correzione (0-1)
    
    Returns:
        pts corretti (modificati in-place per efficienza)
    """
    # Early exit se no reference
    if L_ref is None:
        return pts
    
    # Validazione L_ref (NUOVO: previene correzioni anomale)
    if not (TORSO_MIN <= L_ref <= TORSO_MAX):
        return pts
    
    # Indici torso
    L_SHOULDER, R_SHOULDER, L_HIP, R_HIP = 5, 6, 11, 12
    
    # Check esistenza (ottimizzato: check diretto invece di any())
    if (pts[L_SHOULDER] is None or pts[R_SHOULDER] is None or 
        pts[L_HIP] is None or pts[R_HIP] is None):
        return pts
    
    # Se tutti visibili (detected di recente), non correggere
    if (visible[L_SHOULDER] and visible[R_SHOULDER] and 
        visible[L_HIP] and visible[R_HIP]):
        return pts
    
    # Calcola midpoints
    sh_mid = 0.5 * (pts[L_SHOULDER] + pts[R_SHOULDER])
    hip_mid = 0.5 * (pts[L_HIP] + pts[R_HIP])
    
    # Vettore torso attuale
    v = sh_mid - hip_mid
    dist = np.linalg.norm(v)
    
    if dist < 1e-6:
        return pts
    
    # OTTIMIZZAZIONE: evita allocazione temporanea di delta
    # Calcola scale factor e applica direttamente
    scale = (L_ref / dist - 1.0) * stiffness
    correction = v * scale
    
    # Applica correzione in-place (più efficiente)
    pts[L_SHOULDER] += correction
    pts[R_SHOULDER] += correction
    
    return pts


def compute_torso_length(pts, idx=None):
    """
    Calcola lunghezza torso da keypoints visibili - OTTIMIZZATO.
    
    Args:
        pts: lista 17 keypoints
        idx: indici da usare (default [5,6,11,12])
    
    Returns:
        float: distanza shoulder_center -> hip_center (o None se invalido)
    """
    if idx is None:
        idx = [5, 6, 11, 12]  # shoulders + hips
    
    # Check esistenza (ottimizzato)
    if (pts[5] is None or pts[6] is None or 
        pts[11] is None or pts[12] is None):
        return None
    
    # Calcola distanza
    sh_mid = 0.5 * (pts[5] + pts[6])
    hip_mid = 0.5 * (pts[11] + pts[12])
    
    length = float(np.linalg.norm(sh_mid - hip_mid))
    
    # Validazione (NUOVO: previene valori anomali)
    if not (TORSO_MIN <= length <= TORSO_MAX):
        return None
    
    return length


def smooth_torso_length(L_new, L_prev, alpha=0.3):
    """
    NUOVO: Smoothing esponenziale per lunghezza torso.
    Previene jitter nel calcolo di L_ref.
    
    Args:
        L_new: nuova misurazione
        L_prev: valore precedente (None se prima misurazione)
        alpha: peso nuova misurazione (0-1, default 0.3 = smooth)
    
    Returns:
        float: valore smoothed
    """
    if L_prev is None:
        return L_new
    
    if L_new is None:
        return L_prev
    
    # Exponential moving average
    return alpha * L_new + (1.0 - alpha) * L_prev


def validate_skeleton(pts, min_joints=4, max_depth=5.0):
    """
    Utile per filtrare detection molto rumorose.
    
    Args:
        pts: lista 17 keypoints
        min_joints: minimo numero keypoints validi
        max_depth: massima profondità accettabile (m)
    
    Returns:
        bool: True se skeleton valido
    """
    valid_count = sum(1 for p in pts if p is not None)
    
    if valid_count < min_joints:
        return False
    
    # Check depth range
    for p in pts:
        if p is not None and p[2] > max_depth:
            return False
    
    return True
