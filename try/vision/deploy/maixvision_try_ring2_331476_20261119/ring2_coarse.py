"""Mode 11 only. YOLO identity + outer-ring geometry, with strict triple fallback.

MaixPy API source: https://wiki.sipeed.com/maixpy/doc/en/vision/yolov5.html
Keep dual_buff=False: every coordinate must describe the current stationary
frame, never a previous inference buffer. No change to modes 1..10.
"""
import os
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models/ring2/model.mud")
MODEL_KIND = "YOLOv5"
LABELS = ("2r", "ring2", "2")


def box_candidate(objects, labels):
    candidates = []
    for obj in objects:
        if obj.class_id < 0 or obj.class_id >= len(labels):
            continue
        if labels[obj.class_id] not in LABELS or obj.score < 0.70:
            continue
        if obj.w <= 0 or obj.h <= 0 or not 0.75 <= obj.w / obj.h <= 1.33:
            continue
        x, y = obj.x + obj.w / 2, obj.y + obj.h / 2
        radius = (obj.w + obj.h) / 4
        if 16 <= radius <= 100 and radius <= x <= 319-radius and radius <= y <= 239-radius:
            candidates.append((obj.score, x, y, radius))
    candidates.sort(reverse=True)
    if not candidates or (len(candidates)>1 and candidates[0][0]-candidates[1][0] < 0.15):
        return None
    return candidates[0]


def triple_candidate(circles):
    # De-duplicate concentric Hough peaks, retaining the outside circumference.
    rings = []
    for c in sorted(circles, key=lambda c: c.r(), reverse=True):
        if 16 <= c.r() <= 100 and not any((c.x()-o.x())**2+(c.y()-o.y())**2 < (0.5*o.r())**2 for o in rings):
            rings.append(c)
    rings.sort(key=lambda c: c.x())
    valid = []
    for i in range(len(rings)-2):
        for j in range(i+1,len(rings)-1):
            for k in range(j+1,len(rings)):
                a,b,c = rings[i],rings[j],rings[k]
                d1,d2 = b.x()-a.x(),c.x()-b.x()
                if min(d1,d2)<30 or max(d1,d2)/min(d1,d2)>1.25:
                    continue
                radii=(a.r(),b.r(),c.r())
                if max(radii)/min(radii)>1.25 or max(a.y(),b.y(),c.y())-min(a.y(),b.y(),c.y()) > 0.15*(d1+d2)/2:
                    continue
                # Centre spacing 150 / physical radius 41.75 = 3.593.
                ratio=(d1+d2)/(2*b.r())
                if 2.9 <= ratio <= 4.3:
                    valid.append((b.x(),b.y(),b.r(),800))
    return valid[0] if len(valid)==1 else None


class Ring2Detector:
    def __init__(self):
        self.model = None
        self.previous = None
        try:
            import os
            if os.path.exists(MODEL_PATH):
                from maix import nn
                self.model = getattr(nn, MODEL_KIND)(model=MODEL_PATH, dual_buff=False)
                if not any(label in LABELS for label in self.model.labels):
                    raise ValueError("ring2 model has no matching label")
                print("[COARSE2] loaded", MODEL_PATH)
        except Exception as error:
            self.model = None
            print("[COARSE2] model unavailable:", error)

    def reset(self):
        self.previous = None

    def detect(self, img):
        candidate = None
        if self.model is not None:
            try:
                box=box_candidate(self.model.detect(img,conf_th=0.70,iou_th=0.45),self.model.labels)
                if box is not None:
                    score,x,y,r=box
                    # Refine inside the identified ring; reject a small inner
                    # concentric circle as a source of mm/pixel scale.
                    circles=img.find_circles(x_stride=2,y_stride=2,threshold=4500,
                        r_min=max(16,int(r*0.8)),r_max=min(101,int(r*1.2)+1),r_step=2)
                    matching=[c for c in circles if (c.x()-x)**2+(c.y()-y)**2 <= (r*.3)**2]
                    if matching:
                        c=max(matching,key=lambda c:c.r());x,y,r=c.x(),c.y(),c.r()
                    candidate=(int(round(x)),int(round(y)),int(round(r)),int(score*1000))
            except Exception as error:
                print("[COARSE2] inference error:",error)
        else:
            circles=img.find_circles(x_stride=3,y_stride=3,threshold=4500,
                r_min=16,r_max=101,r_step=2)
            candidate=triple_candidate(circles)
        if candidate is None:
            self.reset()
            return None
        previous=self.previous
        self.previous=candidate
        if previous is None or max(abs(candidate[i]-previous[i]) for i in range(3))>3:
            return None
        return candidate
