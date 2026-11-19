#!/usr/bin/env python3
"""Read-only comparison of MaixHub's exposed report samples, not full inference.

No images, annotations, train/val lists, models, or runtime thresholds are edited.
The report contains a small sample of validation predictions; do not relabel this
as an all-image validation result or a hardware test.
"""
import argparse
import json
import math
from pathlib import Path


def iou(a, b):
    area = lambda q: max(0, q[2]-q[0])*max(0, q[3]-q[1])
    intersection = area((max(a[0], b[0]), max(a[1], b[1]),
                         min(a[2], b[2]), min(a[3], b[3])))
    union = area(a)+area(b)-intersection
    return intersection/union if union else 0.0


def evaluate(path, confidence):
    report = json.loads(path.read_text())
    best = report['best_eval']
    details = []
    for sample in best['val_info']:
        predictions = [p for p in sample['pred'] if p[4] >= confidence]
        used = set()
        matches = []
        for gt in sample['gt']:
            choices = [(iou(p, gt), n, p) for n, p in enumerate(predictions)
                       if n not in used and int(p[5]) == int(gt[5])]
            overlap, n, pred = max(choices, default=(0.0, -1, None))
            if overlap >= 0.5:
                used.add(n)
                dx = (pred[0]+pred[2]-gt[0]-gt[2])/2
                dy = (pred[1]+pred[3]-gt[1]-gt[3])/2
                radius = ((gt[2]-gt[0])+(gt[3]-gt[1]))/4
                matches.append({'score': pred[4], 'iou': overlap,
                                'center_error_px': math.hypot(dx, dy),
                                'estimated_center_error_mm': math.hypot(dx, dy)*41.75/radius})
        details.append({'name': sample.get('nickname', sample['name']),
                        'server_ok': sample.get('ok'),
                        'all_reported_scores': [p[4] for p in sample['pred']],
                        'ground_truths': len(sample['gt']), 'matched': len(matches),
                        'missed': len(sample['gt'])-len(matches),
                        'unmatched_above_gate': len(predictions)-len(used),
                        'matches': matches})
    matched = sum(s['matched'] for s in details)
    missed = sum(s['missed'] for s in details)
    extra = sum(s['unmatched_above_gate'] for s in details)
    return {'path': str(path), 'input': report['inputs'],
            'best_epoch': best['epoch'], 'server_validation_accuracy': best['val_acc'],
            'score_gate': confidence, 'sample_images': len(details),
            'matched': matched, 'missed': missed, 'unmatched_above_gate': extra,
            'exposed_sample_gate_pass': bool(details) and missed == 0 and extra == 0,
            'details': details,
            'scope': 'Only downloaded report best_eval.val_info samples; not all 12 val images.',
            'mm_estimate': 'Assumes GT box encloses 41.75mm outer radius; not camera calibration.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports', nargs='+', type=Path)
    parser.add_argument('--confidence', type=float, default=0.70)
    args = parser.parse_args()
    print(json.dumps([evaluate(path, args.confidence) for path in args.reports],
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
