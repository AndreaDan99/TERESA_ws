#!/usr/bin/env python3
"""
Depth image processing per skeleton 3D - OTTIMIZZATO.
"""
import numpy as np

def get_depth_at_pixel(depth_img, u, v, window_size=3):  # RIDOTTO da 5 a 3
    """
    Estrae depth robusto da finestra NxN attorno al pixel (u,v).
    
    Args:
        depth_img: depth image (H x W), valori in metri o millimetri
        u, v: coordinate pixel (intere)
        window_size: dimensione finestra (dispari, default 3 invece di 5)
    
    Returns:
        float: depth mediana (o None se invalido)
    """
    if depth_img is None:
        return None
    
    h, w = depth_img.shape
    half = window_size // 2
    
    # Bounds check veloce
    if not (half <= u < w - half and half <= v < h - half):
        # Fallback con clamp (caso edge)
        u_min = max(0, u - half)
        u_max = min(w, u + half + 1)
        v_min = max(0, v - half)
        v_max = min(h, v + half + 1)
    else:
        # Caso comune: pixel interno (no clamp necessario)
        u_min = u - half
        u_max = u + half + 1
        v_min = v - half
        v_max = v + half + 1
    
    # Estrai patch (view, non copia)
    patch = depth_img[v_min:v_max, u_min:u_max]
    
    # Filtra validi in una sola operazione
    valid = patch[(patch > 0) & np.isfinite(patch)]
    
    if valid.size == 0:
        return None
    
    # OTTIMIZZAZIONE: usa mean invece di median se >5 valori
    # (median è lento, mean con threshold elimina outliers comunque)
    if valid.size > 5:
        # Rimuovi top/bottom 20% (simile a median ma più veloce)
        valid_sorted = np.sort(valid)
        trim = max(1, len(valid_sorted) // 5)
        trimmed = valid_sorted[trim:-trim] if trim > 0 else valid_sorted
        return float(np.mean(trimmed))
    else:
        # Per patch piccole usa median (più robusto)
        return float(np.median(valid))


def filter_depth_outliers(depth, max_depth=3.0, min_depth=0.1):
    """
    Valida depth measurement.
    
    Returns:
        depth se valido, altrimenti None
    """
    if depth is None:
        return None
    
    # Check range
    if not (min_depth <= depth <= max_depth):
        return None
    
    return depth


def batch_get_depth(depth_img, keypoints_uv, window_size=3):
    """
    Estrae depth per batch di keypoints (più efficiente).
    
    Args:
        depth_img: depth image (H x W)
        keypoints_uv: array (N, 2) di coordinate [u, v]
        window_size: dimensione finestra
    
    Returns:
        list di N depth values (None se invalido)
    """
    if depth_img is None or keypoints_uv is None:
        return [None] * len(keypoints_uv)
    
    depths = []
    for uv in keypoints_uv:
        u, v = int(uv[0]), int(uv[1])
        depth = get_depth_at_pixel(depth_img, u, v, window_size)
        depths.append(depth)
    
    return depths
