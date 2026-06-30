#!/usr/bin/env python3
"""
compute_exposure_metrics.py — TERESA exposure metrics after manual review.

Usage:
  python scripts/compute_exposure_metrics.py --predictions experiments/exp_01/predictions.json

How to review predictions.json:
  1. Open the overlay images, check every detection.
  2. For each detection, add:
       "verified": "tp"           real wound correctly detected
       "verified": "fp"           false alarm (hallucination)
       "wound_id": "w1"           which real wound this detection belongs to
                                  (same id = same wound seen in ≥2 photos)
  3. Add a ground_truth list:
       "ground_truth": [
         {"id": "w1", "type": "laceration",
          "position_mm": [120, -45, 680],   ← optional, for 3D error
          "notes": "5 cm cut on left upper chest"},
         {"id": "w2", "type": "burn", ...}
       ]
  4. Add a missed list:
       "missed": ["w3"]            wound IDs placed but NOT detected by GDINO

Output:
  - Console summary with all Table tab:exposure metrics
  - LaTeX row ready to paste
"""

import argparse
import json
import sys
from pathlib import Path
from collections import defaultdict

import numpy as np


def main():
    ap = argparse.ArgumentParser(
        description="Compute TERESA exposure metrics from reviewed predictions")
    ap.add_argument("--predictions", required=True)
    ap.add_argument("--scan_duration_s", type=float, default=None,
                    help="Measured total scan duration (or estimated)")
    ap.add_argument("--output_latex", type=str, default=None)
    args = ap.parse_args()

    pred_path = Path(args.predictions)
    if not pred_path.exists():
        sys.exit(f"ERROR: {pred_path} not found.")

    with open(pred_path) as f:
        data = json.load(f)

    dets = data.get("detections", [])
    gt_list = data.get("ground_truth", [])
    missed_ids = set(data.get("missed", []))
    n_photos = data.get("n_close_up_photos", 0) + 1  # wide + close-ups

    # ═══════════════════════════════════════════════════════════════
    #  Classify detections
    # ═══════════════════════════════════════════════════════════════
    tp_dets = [d for d in dets if d.get("verified") == "tp"]
    fp_dets = [d for d in dets if d.get("verified") == "fp"]
    unverified = [d for d in dets if d.get("verified") not in ("tp", "fp")]

    # Unique wounds found (dedup by wound_id)
    tp_wound_ids = set(d.get("wound_id") for d in tp_dets if d.get("wound_id"))
    n_wounds_found = len(tp_wound_ids)

    # False negatives
    if gt_list:
        all_gt_ids = {g["id"] for g in gt_list}
        fn_ids = all_gt_ids - tp_wound_ids
        if missed_ids:
            fn_ids |= missed_ids
        n_fn = len(fn_ids)
        n_gt = len(all_gt_ids)
    else:
        n_fn = len(missed_ids)
        n_gt = n_wounds_found + n_fn if (n_wounds_found + n_fn) > 0 else None

    n_tp = len(tp_dets)
    n_fp = len(fp_dets)

    # ═══════════════════════════════════════════════════════════════
    #  Wide shot vs close-up breakdown
    # ═══════════════════════════════════════════════════════════════
    wide_tp = [d for d in tp_dets if d.get("source") == "wide"]
    wide_fp = [d for d in fp_dets if d.get("source") == "wide"]
    cu_tp   = [d for d in tp_dets if d.get("source") != "wide"]
    cu_fp   = [d for d in fp_dets if d.get("source") != "wide"]

    # Wounds first detected in wide shot (have ≥1 wide-shot TP)
    wide_wound_ids = set(d.get("wound_id") for d in wide_tp if d.get("wound_id"))
    n_wide_first = len(wide_wound_ids)

    # Wounds found only in close-ups (not in wide shot)
    cu_only_wound_ids = tp_wound_ids - wide_wound_ids
    n_cu_only = len(cu_only_wound_ids)

    # Refinement gain: additional close-up TP beyond what wide shot found
    n_wide_raw = len(wide_tp)
    n_cu_raw = len(cu_tp)
    n_wide_fp_raw = len(wide_fp)
    n_cu_fp_raw = len(cu_fp)
    n_photos_cu = data.get("n_close_up_photos", 0)
    fp_per_cu = n_cu_fp_raw / max(n_photos_cu, 1)

    # ═══════════════════════════════════════════════════════════════
    #  Metrics
    # ═══════════════════════════════════════════════════════════════
    recall = n_wounds_found / (n_wounds_found + n_fn) if (n_wounds_found + n_fn) > 0 else None
    precision = n_tp / (n_tp + n_fp) if (n_tp + n_fp) > 0 else None
    fp_per_scan = n_fp / max(n_photos, 1)
    f1 = (2 * precision * recall / (precision + recall)
          if (precision and recall and precision + recall > 0) else None)

    tp_confs = [d["score"] for d in tp_dets]
    fp_confs = [d["score"] for d in fp_dets]
    mean_tp_conf = np.mean(tp_confs) if tp_confs else None
    mean_fp_conf = np.mean(fp_confs) if fp_confs else None

    # ═══════════════════════════════════════════════════════════════
    #  3D localisation error  (if ground truth has position_mm)
    # ═══════════════════════════════════════════════════════════════
    loc_errors = []
    gt_by_id = {g["id"]: g for g in gt_list if "position_mm" in g}

    if gt_by_id:
        for d in tp_dets:
            wid = d.get("wound_id")
            if wid and wid in gt_by_id:
                gt_pos_mm = np.array(gt_by_id[wid]["position_mm"])
                det_pos = d.get("position_3d")
                if det_pos is not None:
                    det_pos_mm = np.array(det_pos) * 1000.0
                    err_mm = float(np.linalg.norm(det_pos_mm - gt_pos_mm))
                    loc_errors.append(err_mm)

    mean_loc_err = np.mean(loc_errors) if loc_errors else None
    std_loc_err = np.std(loc_errors) if loc_errors else None

    # ═══════════════════════════════════════════════════════════════
    #  Per-class breakdown
    # ═══════════════════════════════════════════════════════════════
    tp_by_class = defaultdict(int)
    fp_by_class = defaultdict(int)
    for d in tp_dets:
        tp_by_class[d["label"]] += 1
    for d in fp_dets:
        fp_by_class[d["label"]] += 1

    # ═══════════════════════════════════════════════════════════════
    #  Report
    # ═══════════════════════════════════════════════════════════════

    print(f"\n{'═' * 60}")
    print(f"  TERESA Exposure — Metrics Report")
    print(f"  Experiment: {data.get('experiment', 'unknown')}")
    print(f"{'═' * 60}")

    if unverified:
        print(f"\n  ⚠ {len(unverified)} unverified detections — "
              f'add "verified": "tp"/"fp" to each')

    print(f"\n  ── Detection Summary ──")
    print(f"  Ground truth wounds:    {n_gt if n_gt is not None else 'not provided'}")
    print(f"  Wounds found (GDINO):   {n_wounds_found}")
    print(f"  Wounds missed (FN):     {n_fn}")
    print(f"  Raw TP detections:      {n_tp}  (≥ wounds found — same wound in ≥2 photos)")
    print(f"  False positives:        {n_fp}")
    print(f"  Photos in scan:         {n_photos}")

    print(f"\n  ── Wide shot vs close-up refinement ──")
    print(f"  Wide shot — raw detections:  {n_wide_raw + n_wide_fp_raw}  "
          f"(TP {n_wide_raw}, FP {n_wide_fp_raw})")
    print(f"  Close-ups — raw detections:  {n_cu_raw + n_cu_fp_raw}  "
          f"(TP {n_cu_raw}, FP {n_cu_fp_raw})")
    print(f"  Wounds first seen in wide:   {n_wide_first}/{n_wounds_found}")
    print(f"  Wounds found only in CU:     {n_cu_only}")
    print(f"  Refinement gain:             {n_cu_raw} additional TP from {n_photos_cu} close-ups")
    print(f"  FP per close-up:             {fp_per_cu:.1f}")

    print(f"\n  ── Table \\ref{{tab:exposure}} Metrics ──")
    print(f"  Scan points per trial:    {n_photos}")
    scan_dur = args.scan_duration_s or (n_photos * 2.0)
    print(f"  Scan duration (s):        {scan_dur:.0f}")
    print(f"  Wide shot detections:     {n_wide_raw + n_wide_fp_raw} "
          f"({n_wide_raw} TP, {n_wide_fp_raw} FP)")
    print(f"  Close-up detections:      {n_cu_raw + n_cu_fp_raw} "
          f"({n_cu_raw} TP, {n_cu_fp_raw} FP)")
    print(f"  Wounds first in wide:     {n_wide_first}/{n_wounds_found}")
    print(f"  Close-up-only wounds:     {n_cu_only}")
    print(f"  Refinement gain (CU TP):  {n_cu_raw}")
    if recall is not None:
        print(f"  Wound recall:             {recall * 100:.1f}%")
    print(f"  False positives/close-up: {fp_per_cu:.1f}")
    if precision is not None:
        print(f"  Precision:                {precision * 100:.1f}%")
    if f1 is not None:
        print(f"  F1:                       {f1:.3f}")

    if mean_loc_err is not None:
        print(f"  3D localisation error:  {mean_loc_err:.0f} ± {std_loc_err:.0f} mm  "
              f"(on {len(loc_errors)} wounds with ground truth)")
    else:
        print(f"  3D localisation error:  not computed — add position_mm to ground_truth")

    print(f"\n  ── Confidence ──")
    if mean_tp_conf is not None:
        print(f"  Mean TP confidence:     {mean_tp_conf:.3f}")
    if mean_fp_conf is not None:
        print(f"  Mean FP confidence:     {mean_fp_conf:.3f}")

    print(f"\n  ── TP by class ──")
    for cls, count in sorted(tp_by_class.items(), key=lambda x: -x[1]):
        print(f"    {cls:28s} {count}")
    if fp_by_class:
        print(f"\n  ── FP by class ──")
        for cls, count in sorted(fp_by_class.items(), key=lambda x: -x[1]):
            print(f"    {cls:28s} {count}")

    # ═══════════════════════════════════════════════════════════════
    #  LaTeX
    # ═══════════════════════════════════════════════════════════════

    if recall is not None:
        recall_str = f"{recall * 100:.0f}"
        fp_str = f"{fp_per_scan:.1f}"
        dur_str = f"{scan_dur:.0f}"

        loc_str = ""
        if mean_loc_err is not None:
            loc_str = f"{mean_loc_err:.0f} $\\pm$ {std_loc_err:.0f}"

        latex_row = (
            f"    Scan points per trial & {n_photos} \\\\\n"
            f"    Scan duration (s) & {dur_str} \\\\\n"
            f"    Wide shot detections & {n_wide_raw + n_wide_fp_raw} "
            f"({n_wide_raw} TP, {n_wide_fp_raw} FP) \\\\\n"
            f"    Close-up detections & {n_cu_raw + n_cu_fp_raw} "
            f"({n_cu_raw} TP, {n_cu_fp_raw} FP) \\\\\n"
            f"    Wounds first seen in wide shot & {n_wide_first}/{n_wounds_found} \\\\\n"
            f"    Close-up-only wounds & {n_cu_only} \\\\\n"
            f"    Refinement gain (CU TP) & {n_cu_raw} \\\\\n"
            f"    Wound recall (\\%) & {recall_str} \\\\\n"
            f"    False positives per close-up & {fp_per_cu:.1f} \\\\\n"
        )
        if loc_str:
            latex_row += f"    3D localisation error (mm) & {loc_str} \\\\\n"

        print(f"\n{'═' * 60}")
        print(f"  LaTeX row (for tab:exposure):")
        print(f"{'═' * 60}")
        print(latex_row)

        if args.output_latex:
            out_path = Path(args.output_latex)
            with open(out_path, "a") as f:
                f.write(f"\n% Experiment: {data.get('experiment', 'unknown')}\n")
                f.write(latex_row)
                f.write("\n")
            print(f"  → Appended to {out_path}")

    # ═══════════════════════════════════════════════════════════════
    #  Save metrics back
    # ═══════════════════════════════════════════════════════════════
    data["metrics"] = {
        "scan_points": n_photos,
        "scan_duration_s": scan_dur,
        "n_ground_truth": n_gt,
        "n_wounds_found": n_wounds_found,
        "n_false_negatives": n_fn,
        "n_true_positive_detections": n_tp,
        "n_false_positive_detections": n_fp,
        "recall": round(recall, 4) if recall is not None else None,
        "precision": round(precision, 4) if precision is not None else None,
        "fp_per_scan": round(fp_per_scan, 4),
        "f1": round(f1, 4) if f1 is not None else None,
        "mean_tp_confidence": round(mean_tp_conf, 4) if mean_tp_conf is not None else None,
        "mean_fp_confidence": round(mean_fp_conf, 4) if mean_fp_conf is not None else None,
        "tp_by_class": dict(tp_by_class),
        "fp_by_class": dict(fp_by_class),
        # Wide shot vs close-up refinement
        "n_wide_detections": n_wide_raw + n_wide_fp_raw,
        "n_wide_tp": n_wide_raw,
        "n_wide_fp": n_wide_fp_raw,
        "n_close_up_detections": n_cu_raw + n_cu_fp_raw,
        "n_close_up_tp": n_cu_raw,
        "n_close_up_fp": n_cu_fp_raw,
        "n_wide_first_wounds": n_wide_first,
        "n_close_up_only_wounds": n_cu_only,
        "refinement_gain": n_cu_raw,
        "fp_per_close_up": round(fp_per_cu, 4),
    }

    if mean_loc_err is not None:
        data["metrics"]["3d_localisation_error_mm_mean"] = round(mean_loc_err, 1)
        data["metrics"]["3d_localisation_error_mm_std"] = round(std_loc_err, 1)
        data["metrics"]["n_localised_wounds"] = len(loc_errors)

    with open(pred_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n  ✓ Metrics saved → {pred_path}")
    print(f"{'═' * 60}\n")


if __name__ == "__main__":
    main()
