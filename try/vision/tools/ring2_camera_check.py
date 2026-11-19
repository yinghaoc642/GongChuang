"""Bounded camera-only check of the installed detector; never requests motion.

The installed source/model remain unchanged. The recording proxy forwards the
exact production NN call. Occasional separate 0.20-threshold inferences are
diagnostic only and never feed the production detector or robot protocol.
"""
import argparse
import json
from pathlib import Path
import sys
import time as host_time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--frames', type=int, default=40)
    p.add_argument('--seconds', type=float, default=15)
    args = p.parse_args()
    if not 1 <= args.frames <= 1000 or not 1 <= args.seconds <= 30:
        p.error('Check is limited to 1000 frames and 30 seconds')
    output = Path(args.output)
    output.mkdir(mode=0o700, exist_ok=False)
    sys.path.insert(0, '/maixapp/apps/gc_try_r2_331476_20261119')
    from maix import app, camera, display, image, err
    from ring2_coarse import Ring2Detector, box_candidate

    class RecordingModel:
        def __init__(self, model):
            self.real = model
            self.labels = model.labels
            self.objects = []

        def detect(self, *a, **kw):
            self.objects = self.real.detect(*a, **kw)
            return self.objects

    def describe(obj):
        return dict(label=detector.model.labels[obj.class_id],
                    score=float(obj.score), x=obj.x, y=obj.y, w=obj.w, h=obj.h)

    def save(img, name):
        result = img.save(str(output / name), quality=95)
        if result != err.Err.ERR_NONE:
            raise RuntimeError('Image save failed: ' + str(result))

    detector = Ring2Detector()
    if detector.model is None:
        raise RuntimeError('Installed model did not load')
    recorder = RecordingModel(detector.model)
    detector.model = recorder
    cam = camera.Camera(320, 240, fps=-1, buff_num=1)
    disp = display.Display()
    records = []
    app.set_exit_flag(False)
    try:
        for _ in range(30):
            cam.read()
        start = host_time.monotonic()
        while len(records) < args.frames and host_time.monotonic() - start < args.seconds and not app.need_exit():
            img = cam.read()
            t0 = host_time.monotonic()
            previous = detector.previous
            accepted = detector.detect(img)
            candidate = detector.previous
            elapsed = (host_time.monotonic() - t0) * 1000
            objects = list(recorder.objects)
            box = box_candidate(objects, recorder.labels)
            index = len(records)
            low = None
            if index % 5 == 0:
                low = list(recorder.real.detect(img, conf_th=0.20, iou_th=0.45))
            record = dict(frame=index, production_ms=round(elapsed, 2),
                          objects=[describe(o) for o in objects],
                          geometric_box=box, stable_result=accepted,
                          candidate_before_stability=candidate,
                          previous_candidate=previous,
                          diagnostic_objects=None if low is None else [describe(o) for o in low])
            records.append(record)
            print('FRAME ' + json.dumps(record), flush=True)
            sample = index in (0, 5, 10, 20, 30, args.frames - 1) or index % 50 == 0
            if sample:
                save(img, 'frame_%03d_raw.jpg' % index)
            for obj in (objects if low is None else low):
                color = image.COLOR_GREEN if obj.score >= 0.70 else image.COLOR_YELLOW
                img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=color, thickness=2)
                img.draw_string(max(0, obj.x), max(0, obj.y-16), '2 %.2f' % obj.score, color=color, scale=0.8)
            if accepted is not None:
                x, y, radius, confidence = accepted
                img.draw_circle(x, y, radius, color=image.COLOR_RED, thickness=2)
            img.draw_string(2, 2, 'Ring2 %s' % ('STABLE' if accepted else 'WAIT'), color=image.COLOR_WHITE, scale=0.8)
            disp.show(img)
            if sample:
                save(img, 'frame_%03d_marked.jpg' % index)
        summary = dict(frames=len(records),
                       seconds=round(host_time.monotonic()-start, 3),
                       frames_with_score_070=sum(bool(r['objects']) for r in records),
                       frames_with_geometric_box=sum(r['geometric_box'] is not None for r in records),
                       frames_with_stable_output=sum(r['stable_result'] is not None for r in records),
                       low_threshold_frames=sum(r['diagnostic_objects'] is not None for r in records),
                       low_threshold_hits=sum(bool(r['diagnostic_objects']) for r in records),
                       scope='One scene, camera motion not controlled; diagnostic 0.20 results never used as accepted outputs. No robot request or response sent.')
        (output / 'records.json').write_text(json.dumps(dict(summary=summary, frames=records), indent=2))
        print('SUMMARY ' + json.dumps(summary), flush=True)
    finally:
        cam.close()


if __name__ == '__main__':
    main()
