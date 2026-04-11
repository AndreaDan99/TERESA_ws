#!/usr/bin/env python3
"""
RViz visualization helpers per skeleton.
"""
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from .skeleton_utils import SKELETON_EDGES


def build_skeleton_markers(pose_array, pts, visible):
    """
    Crea MarkerArray per RViz:
    - Joints detected (rosso)
    - Joints predicted (giallo)
    - Bones (verde)
    
    Args:
        pose_array: PoseArray header + poses
        pts: lista 17 keypoints np.array([x,y,z]) o None
        visible: lista 17 bool
    
    Returns:
        MarkerArray
    """
    ma = MarkerArray()
    
    # Pre-calcola header comune
    header = pose_array.header
    
    # Marker 0: Joints detected (RED)
    j_vis = Marker()
    j_vis.header = header
    j_vis.ns = "joints_detected"
    j_vis.id = 0
    j_vis.type = Marker.SPHERE_LIST
    j_vis.action = Marker.ADD  # AGGIUNTO: esplicita azione
    j_vis.scale.x = j_vis.scale.y = j_vis.scale.z = 0.05
    j_vis.color.r = 1.0
    j_vis.color.g = 0.0  # AGGIUNTO: esplicita
    j_vis.color.b = 0.0  # AGGIUNTO: esplicita
    j_vis.color.a = 1.0
    
    # Marker 1: Joints predicted (YELLOW)
    j_pred = Marker()
    j_pred.header = header
    j_pred.ns = "joints_predicted"
    j_pred.id = 1
    j_pred.type = Marker.SPHERE_LIST
    j_pred.action = Marker.ADD  # AGGIUNTO
    j_pred.scale.x = j_pred.scale.y = j_pred.scale.z = 0.04
    j_pred.color.r = 1.0
    j_pred.color.g = 1.0
    j_pred.color.b = 0.0  # AGGIUNTO
    j_pred.color.a = 0.7
    
    # Pre-alloca liste (evita realloc ripetute)
    vis_points = []
    pred_points = []
    
    # Aggiungi keypoints
    for i, p in enumerate(pts):
        if p is None:
            continue
        
        pt = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        
        if visible[i]:
            vis_points.append(pt)
        else:
            pred_points.append(pt)
    
    j_vis.points = vis_points
    j_pred.points = pred_points
    
    # Aggiungi solo se hanno punti (evita marker vuoti)
    if len(vis_points) > 0:
        ma.markers.append(j_vis)
    if len(pred_points) > 0:
        ma.markers.append(j_pred)
    
    # Marker 2: Bones (GREEN)
    bones = Marker()
    bones.header = header
    bones.ns = "bones"
    bones.id = 2
    bones.type = Marker.LINE_LIST
    bones.action = Marker.ADD  # AGGIUNTO
    bones.scale.x = 0.015
    bones.color.r = 0.0  # AGGIUNTO
    bones.color.g = 1.0
    bones.color.b = 0.0  # AGGIUNTO
    bones.color.a = 1.0
    
    # Pre-alloca lista bones
    bone_points = []
    for a, b in SKELETON_EDGES:
        if pts[a] is not None and pts[b] is not None:
            bone_points.append(Point(x=float(pts[a][0]), y=float(pts[a][1]), z=float(pts[a][2])))
            bone_points.append(Point(x=float(pts[b][0]), y=float(pts[b][1]), z=float(pts[b][2])))
    
    bones.points = bone_points
    
    # Aggiungi solo se ci sono bones
    if len(bone_points) > 0:
        ma.markers.append(bones)
    
    return ma
