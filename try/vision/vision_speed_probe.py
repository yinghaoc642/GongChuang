"""MaixCAM circle-recognition speed and accuracy probe.

Run this file on the MaixCAM instead of ``yanyanview5.py``.  It compares the
frozen 01:40 implementation with the aggressive implementation on the SAME
captured image, then exercises each implementation as a complete cold-start
request (full search -> dynamic ROI -> stability decision/fallback).

Edit TEST_MODE and SCENE_LABEL before each physical test:
  * TEST_MODE = 9: normal three-ring pose.
  * TEST_MODE = 10: endpoint pose; run ring1 and ring3 as separate scenes.

All ``r_max`` values in this file are treated as exclusive, matching the
runtime code and MaixPy ``find_circles`` call sites.
"""

import gc
import os
import sys

from maix import app, camera, gpio, pinmap, time

try:
    from maix import image
except Exception:
    image = None

try:
    from maix import sys as maix_sys
except Exception:
    maix_sys = None


# ---------------------------------------------------------------------------
# Test settings: change these on the board, not the profile definitions below.
# ---------------------------------------------------------------------------

TEST_MODE = 9
SCENE_LABEL = "mode9-normal-three-rings"

# One-line mode10 experiment switch.  "runtime" now means the v5.5.1 mode10
# [40, 111), step=3 search, which permits a returned r=110.  Use
# "comparison-40-110-r3" to measure the earlier [40, 110) proposal (r=110 is
# excluded), or "legacy-28-96-r3" to compare the pre-mode-specific range.
MODE10_AGGRESSIVE_RADIUS_VARIANT = "runtime"

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_CENTER_X = IMAGE_WIDTH // 2
IMAGE_CENTER_Y = IMAGE_HEIGHT // 2

# -1 asks the driver for its automatic maximum.  Only force 60 after measured
# CAPTURE output proves that this sensor/firmware/exposure actually delivers it.
TARGET_CAMERA_FPS = -1
CAMERA_BUFFER_COUNT = 1

# Zero leaves the current automatic setting unchanged.  To find the exposure
# limit, repeat the same scene with e.g. 8000, 5000, 3000 us and compare both
# capture FPS and target rate/magnitude.  Change only one variable per run.
MANUAL_EXPOSURE_US = 0
MANUAL_GAIN = 0

# At 50--60 fps this is about one second; at lower sensor rates it gives AE/AWB
# even longer.  A short 10--12 frame warm-up can contaminate all accuracy and
# magnitude statistics with the exposure ramp.
WARMUP_FRAMES = 60
CAPTURE_SAMPLE_COUNT = 100
HOUGH_SAMPLE_COUNT = 100
END_TO_END_TRIAL_COUNT = 30
END_TO_END_MAX_FRAMES = 20

# Raw paired deltas are retained in RAM and summarized after the timed loop so
# printing cannot stall the next capture.  Enable this only when raw rows are
# needed; the median and absolute P95 are always printed.
PRINT_MATCHED_FRAME_DELTAS = False

# Optional experiment.  RGB is always measured first.  When enabled, the
# probe safely tries to make a grayscale copy and compares it on the same
# source frame.  Unsupported firmware prints GRAY SKIP and continues.
ENABLE_GRAYSCALE_COMPARISON = False
GRAYSCALE_SAMPLE_COUNT = 100

CIRCLE_THRESHOLD = 7500

RING_MIN_SPACING_PIXELS = 20
RING_MAX_SPACING_RATIO = 1.45
RING_MAX_LINE_ERROR_RATIO = 0.18
RING_MAX_RADIUS_RATIO = 1.60
RING_CANDIDATE_MIN_RADIUS_PIXELS = 28
RING_CANDIDATE_MIN_LARGEST_RADIUS_RATIO = 0.55

RING_FALLBACK_MAX_CENTER_DISTANCE_PIXELS = 70
RING_FALLBACK_MIN_DISTANCE_MARGIN_PIXELS = 15
RING_FALLBACK_MIN_DOMINANT_RADIUS_RATIO = 0.75

ENDPOINT_MAX_CENTER_DISTANCE_PIXELS = 90
ENDPOINT_MIN_DOMINANT_RADIUS_RATIO = 0.75


# Frozen 01:40 v5.4.3-max-fps settings.
BASELINE_PROFILE = {
    "name": "0140-v5.4.3",
    "full": {
        "x_stride": 3,
        "y_stride": 2,
        "r_min": 28,
        "r_max": 110,
        "r_step": 2,
    },
    "fallback": None,
    "fallback_after_misses": None,
    "tracking": {
        "x_stride": 2,
        "y_stride": 2,
        "r_min": 28,
        "r_max": 110,
        "r_step": 2,
        "radius_lower": 4,
        "radius_upper_exclusive": 4,
        "selection_radius_margin": 4,
        "position_margin": 8,
        "roi_padding": 12,
        "maximum_misses": 2,
    },
    "include_search_frame": False,
    "preserve_first_tracking_gap": False,
    "pending_outlier_migration": False,
    "freeze_tracking_anchor": False,
    "force_valid_tracking_radius_range": False,
    "allow_centered_fallback_on_primary": True,
    "stability": {
        9: {
            "fast_samples": 5,
            "full_samples": 7,
            "fast_center_span": 1,
            "fast_radius_span": 2,
            "full_center_span": 3,
            "full_radius_span": 3,
        },
        10: {
            "fast_samples": 3,
            "full_samples": 5,
            "fast_center_span": 1,
            "fast_radius_span": 1,
            "full_center_span": 2,
            "full_radius_span": 2,
        },
    },
}


# Current v5.5.1 aggressive-latency settings.
AGGRESSIVE_PROFILE = {
    "name": "aggressive-v5.5.1",
    "full": {
        "x_stride": 3,
        "y_stride": 3,
        "r_min": 28,
        "r_max": 96,
        "r_step": 3,
    },
    "fallback": {
        "x_stride": 3,
        "y_stride": 2,
        "r_min": 28,
        "r_max": 111,
        "r_step": 2,
    },
    "fallback_after_misses": 1,
    "tracking": {
        "x_stride": 3,
        "y_stride": 2,
        "r_min": 28,
        # v5.5.1 clamps tracked radii against fallback_r_max, not the narrow
        # first-search maximum, and guarantees a non-empty exclusive range.
        "r_max": 111,
        "r_step": 2,
        "radius_lower": 2,
        "radius_upper_exclusive": 3,
        "selection_radius_margin": 3,
        "position_margin": 8,
        "roi_padding": 10,
        "maximum_misses": 2,
    },
    "include_search_frame": True,
    "preserve_first_tracking_gap": True,
    "pending_outlier_migration": True,
    "freeze_tracking_anchor": True,
    "force_valid_tracking_radius_range": True,
    "allow_centered_fallback_on_primary": False,
    "stability": {
        9: {
            "fast_samples": 3,
            "full_samples": 4,
            "fast_center_span": 1,
            "fast_radius_span": 2,
            "full_center_span": 3,
            "full_radius_span": 3,
        },
        10: {
            "fast_samples": 2,
            "full_samples": 3,
            "fast_center_span": 1,
            "fast_radius_span": 1,
            "full_center_span": 2,
            "full_radius_span": 2,
        },
    },
}


MODE10_RADIUS_VARIANT_ACTIVE = "runtime"
if TEST_MODE == 10:
    # Exact v5.5.1 endpoint radius domain.
    AGGRESSIVE_PROFILE["full"]["r_min"] = 40
    AGGRESSIVE_PROFILE["full"]["r_max"] = 111
    AGGRESSIVE_PROFILE["fallback"]["r_min"] = 40
    AGGRESSIVE_PROFILE["fallback"]["r_max"] = 111
    AGGRESSIVE_PROFILE["tracking"]["r_min"] = 40
    AGGRESSIVE_PROFILE["tracking"]["r_max"] = 111

if (
    TEST_MODE == 10
    and MODE10_AGGRESSIVE_RADIUS_VARIANT == "comparison-40-110-r3"
):
    MODE10_RADIUS_VARIANT_ACTIVE = "comparison-40-110-r3"
    AGGRESSIVE_PROFILE["name"] = (
        "aggressive-v5.5.1-mode10-r40-110-r3-comparison"
    )
    AGGRESSIVE_PROFILE["full"]["r_min"] = 40
    AGGRESSIVE_PROFILE["full"]["r_max"] = 110
    AGGRESSIVE_PROFILE["full"]["r_step"] = 3
    AGGRESSIVE_PROFILE["fallback"]["r_min"] = 40
    AGGRESSIVE_PROFILE["fallback"]["r_max"] = 110
    AGGRESSIVE_PROFILE["tracking"]["r_min"] = 40
    AGGRESSIVE_PROFILE["tracking"]["r_max"] = 110
elif (
    TEST_MODE == 10
    and MODE10_AGGRESSIVE_RADIUS_VARIANT == "legacy-28-96-r3"
):
    MODE10_RADIUS_VARIANT_ACTIVE = "legacy-28-96-r3"
    AGGRESSIVE_PROFILE["name"] = (
        "aggressive-v5.5.1-mode10-r28-96-r3-legacy-comparison"
    )
    AGGRESSIVE_PROFILE["full"]["r_min"] = 28
    AGGRESSIVE_PROFILE["full"]["r_max"] = 96
    AGGRESSIVE_PROFILE["fallback"]["r_min"] = 28
    AGGRESSIVE_PROFILE["tracking"]["r_min"] = 28


PROFILES = (BASELINE_PROFILE, AGGRESSIVE_PROFILE)


# ---------------------------------------------------------------------------
# Small compatibility/statistics helpers (kept MicroPython-friendly).
# ---------------------------------------------------------------------------

def safe_text(value):
    try:
        return str(value).replace("\n", " ").replace("\r", " ")
    except Exception:
        return "<unprintable>"


def percentile(values, percent):
    """Nearest-rank-style percentile that also works on MaixPy."""
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * percent / 100.0))
    if index < 0:
        index = 0
    if index >= len(ordered):
        index = len(ordered) - 1
    return ordered[index]


def average(values):
    if not values:
        return None
    return sum(values) / len(values)


def format_number(value, decimals=1):
    if value is None:
        return "NA"
    try:
        if decimals == 0:
            return str(int(round(value)))
        return ("%." + str(decimals) + "f") % value
    except Exception:
        return safe_text(value)


def coordinate_median(coordinates):
    if not coordinates:
        return None
    return (
        int(round(percentile([item[0] for item in coordinates], 50))),
        int(round(percentile([item[1] for item in coordinates], 50))),
        int(round(percentile([item[2] for item in coordinates], 50))),
    )


def safe_dir(value):
    try:
        return sorted(name for name in dir(value) if not name.startswith("__"))
    except Exception:
        return []


def safe_read_value(owner, name):
    try:
        value = getattr(owner, name)
    except Exception as exc:
        return False, "getattr-error:" + safe_text(exc)
    if callable(value):
        try:
            return True, value()
        except Exception as exc:
            return False, "call-error:" + safe_text(exc)
    return True, value


def print_safe_values(prefix, owner, names):
    for name in names:
        ok, value = safe_read_value(owner, name)
        print(prefix, name, "=", safe_text(value), "ok=" + str(int(ok)))


def read_small_system_file(path, maximum_bytes=1000):
    try:
        with open(path, "r") as handle:
            return handle.read(maximum_bytes).replace("\x00", "").strip()
    except Exception:
        return None


try:
    _ticks_us_available = callable(getattr(time, "ticks_us", None))
    if _ticks_us_available:
        time.ticks_us()
except Exception:
    _ticks_us_available = False


if _ticks_us_available:
    TIMER_SOURCE = "ticks_us"

    def now_us():
        return time.ticks_us()

else:
    TIMER_SOURCE = "ticks_ms*1000"

    def now_us():
        return time.ticks_ms() * 1000


def elapsed_us(start, end=None):
    if end is None:
        end = now_us()
    ticks_diff = getattr(time, "ticks_diff", None)
    if callable(ticks_diff):
        try:
            return ticks_diff(end, start)
        except Exception:
            pass
    return end - start


def sleep_ms(milliseconds):
    sleeper = getattr(time, "sleep_ms", None)
    if callable(sleeper):
        sleeper(milliseconds)


def append_error(error_counts, error):
    if error is None:
        return
    key = safe_text(error)
    error_counts[key] = error_counts.get(key, 0) + 1


# ---------------------------------------------------------------------------
# Runtime-equivalent ring selection.
# ---------------------------------------------------------------------------

def filter_outer_ring_candidates(circles):
    if not circles:
        return []
    largest_radius = max(circle.r() for circle in circles)
    adaptive_minimum_radius = max(
        RING_CANDIDATE_MIN_RADIUS_PIXELS,
        largest_radius * RING_CANDIDATE_MIN_LARGEST_RADIUS_RATIO,
    )
    return [
        circle
        for circle in circles
        if circle.r() >= adaptive_minimum_radius
    ]


def select_center_ring(circles):
    outer_circles = filter_outer_ring_candidates(circles)
    if len(outer_circles) < 3:
        return None
    ordered = sorted(outer_circles, key=lambda circle: circle.x())
    best_circle = None
    best_score = None
    for first_index in range(len(ordered) - 2):
        for second_index in range(first_index + 1, len(ordered) - 1):
            for third_index in range(second_index + 1, len(ordered)):
                first = ordered[first_index]
                second = ordered[second_index]
                third = ordered[third_index]
                spacing_12 = second.x() - first.x()
                spacing_23 = third.x() - second.x()
                if (
                    spacing_12 < RING_MIN_SPACING_PIXELS
                    or spacing_23 < RING_MIN_SPACING_PIXELS
                ):
                    continue
                spacing_ratio = max(spacing_12, spacing_23) / min(
                    spacing_12, spacing_23
                )
                average_spacing = (spacing_12 + spacing_23) * 0.5
                line_error = (
                    max(first.y(), second.y(), third.y())
                    - min(first.y(), second.y(), third.y())
                )
                radii = [first.r(), second.r(), third.r()]
                radius_ratio = max(radii) / max(min(radii), 1)
                if (
                    spacing_ratio > RING_MAX_SPACING_RATIO
                    or line_error
                    > average_spacing * RING_MAX_LINE_ERROR_RATIO
                    or radius_ratio > RING_MAX_RADIUS_RATIO
                ):
                    continue
                score = (
                    abs(spacing_12 - spacing_23)
                    + line_error * 2.0
                    + max(radii) - min(radii)
                    + abs(second.x() - IMAGE_CENTER_X) * 0.1
                    + abs(second.y() - IMAGE_CENTER_Y) * 0.1
                )
                if best_score is None or score < best_score:
                    best_score = score
                    best_circle = second
    return best_circle


def center_distance_squared(circle):
    dx = circle.x() - IMAGE_CENTER_X
    dy = circle.y() - IMAGE_CENTER_Y
    return dx * dx + dy * dy


def select_centered_ring_fallback(circles):
    outer_circles = filter_outer_ring_candidates(circles)
    if not outer_circles:
        return None
    maximum_distance_squared = (
        RING_FALLBACK_MAX_CENTER_DISTANCE_PIXELS
        * RING_FALLBACK_MAX_CENTER_DISTANCE_PIXELS
    )
    near_center = [
        circle
        for circle in outer_circles
        if center_distance_squared(circle) <= maximum_distance_squared
    ]
    if not near_center:
        return None
    largest_radius = max(circle.r() for circle in near_center)
    dominant = [
        circle
        for circle in near_center
        if circle.r()
        >= largest_radius * RING_FALLBACK_MIN_DOMINANT_RADIUS_RATIO
    ]
    ranked = sorted(dominant, key=center_distance_squared)
    best = ranked[0]
    if len(ranked) > 1:
        best_distance = center_distance_squared(best) ** 0.5
        second_distance = center_distance_squared(ranked[1]) ** 0.5
        if (
            second_distance - best_distance
            < RING_FALLBACK_MIN_DISTANCE_MARGIN_PIXELS
        ):
            return None
    return best


def select_endpoint_ring(circles):
    if not circles:
        return None
    maximum_distance_squared = (
        ENDPOINT_MAX_CENTER_DISTANCE_PIXELS
        * ENDPOINT_MAX_CENTER_DISTANCE_PIXELS
    )
    near_center = [
        circle
        for circle in circles
        if center_distance_squared(circle) <= maximum_distance_squared
    ]
    outer_circles = filter_outer_ring_candidates(near_center)
    if not outer_circles:
        return None
    largest_radius = max(circle.r() for circle in outer_circles)
    dominant = [
        circle
        for circle in outer_circles
        if circle.r() >= largest_radius * ENDPOINT_MIN_DOMINANT_RADIUS_RATIO
    ]
    return min(
        dominant,
        key=lambda circle: (center_distance_squared(circle), -circle.r()),
    )


def select_full_search_ring(circles, allow_centered_fallback=True):
    if TEST_MODE == 10:
        return select_endpoint_ring(circles)
    selected = select_center_ring(circles)
    if selected is None and allow_centered_fallback:
        selected = select_centered_ring_fallback(circles)
    return selected


def select_locked_ring(circles, reference, position_margin, radius_margin):
    if reference is None:
        return None
    reference_x, reference_y, reference_radius = reference
    candidates = [
        circle
        for circle in circles
        if abs(circle.x() - reference_x) <= position_margin
        and abs(circle.y() - reference_y) <= position_margin
        and abs(circle.r() - reference_radius) <= radius_margin
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda circle: (
            (circle.x() - reference_x) ** 2
            + (circle.y() - reference_y) ** 2
            + (circle.r() - reference_radius) ** 2
        ),
    )


def circle_magnitude(circle):
    if circle is None:
        return None
    magnitude = getattr(circle, "magnitude", None)
    if not callable(magnitude):
        return None
    try:
        return magnitude()
    except Exception:
        return None


def circle_record(circle):
    if circle is None:
        return None
    return (
        int(circle.x()),
        int(circle.y()),
        int(circle.r()),
        circle_magnitude(circle),
    )


def record_coordinate(record):
    if record is None:
        return None
    return record[0], record[1], record[2]


def tracking_roi(reference, padding):
    center_x, center_y, radius = reference
    half_size = radius + padding
    left = max(center_x - half_size, 0)
    top = max(center_y - half_size, 0)
    right = min(center_x + half_size + 1, IMAGE_WIDTH)
    bottom = min(center_y + half_size + 1, IMAGE_HEIGHT)
    return [left, top, right - left, bottom - top]


def effective_tracking_search(profile, reference):
    """Return ROI search with hard runtime clamps and exclusive r_max."""
    tracking = profile["tracking"]
    if profile["force_valid_tracking_radius_range"]:
        # Exact v5.5.1 circle_tracking_radius_bounds behavior: clamp against
        # the fallback hard maximum and always leave at least one radius below
        # the exclusive upper bound, even for a boundary lock radius.
        r_min = min(
            max(
                tracking["r_min"],
                reference[2] - tracking["radius_lower"],
            ),
            tracking["r_max"] - 1,
        )
        r_max = min(
            tracking["r_max"],
            max(
                r_min + 1,
                reference[2] + tracking["radius_upper_exclusive"],
            ),
        )
    else:
        # Frozen 01:40 behavior.
        r_min = max(
            tracking["r_min"],
            reference[2] - tracking["radius_lower"],
        )
        r_max = min(
            tracking["r_max"],
            reference[2] + tracking["radius_upper_exclusive"],
        )
    return {
        "x_stride": tracking["x_stride"],
        "y_stride": tracking["y_stride"],
        "r_min": r_min,
        "r_max": r_max,
        "r_step": tracking["r_step"],
        "roi": tracking_roi(reference, tracking["roi_padding"]),
    }


def radius_candidate_count(search):
    if search["r_max"] <= search["r_min"]:
        return 0
    return len(range(search["r_min"], search["r_max"], search["r_step"]))


# ---------------------------------------------------------------------------
# Timed detection and reporting.
# ---------------------------------------------------------------------------

def run_detection(
    img,
    search,
    selector,
    reference=None,
    tracking=None,
    allow_centered_fallback=True,
):
    if search["r_max"] <= search["r_min"]:
        return None, 0, 0, "invalid-exclusive-radius-range"
    pipeline_start = now_us()
    hough_start = pipeline_start
    try:
        if "roi" in search:
            circles = img.find_circles(
                roi=search["roi"],
                x_stride=search["x_stride"],
                y_stride=search["y_stride"],
                threshold=CIRCLE_THRESHOLD,
                r_min=search["r_min"],
                r_max=search["r_max"],
                r_step=search["r_step"],
            )
        else:
            circles = img.find_circles(
                x_stride=search["x_stride"],
                y_stride=search["y_stride"],
                threshold=CIRCLE_THRESHOLD,
                r_min=search["r_min"],
                r_max=search["r_max"],
                r_step=search["r_step"],
            )
    except Exception as exc:
        end = now_us()
        return None, elapsed_us(hough_start, end), elapsed_us(
            pipeline_start, end
        ), "find-circles:" + safe_text(exc)
    hough_end = now_us()
    if circles is None:
        circles = []
    try:
        if selector == "full":
            selected = select_full_search_ring(
                circles,
                allow_centered_fallback,
            )
        else:
            selected = select_locked_ring(
                circles,
                reference,
                tracking["position_margin"],
                tracking["selection_radius_margin"],
            )
        record = circle_record(selected)
    except Exception as exc:
        end = now_us()
        return None, elapsed_us(hough_start, hough_end), elapsed_us(
            pipeline_start, end
        ), "selection:" + safe_text(exc)
    pipeline_end = now_us()
    return (
        record,
        elapsed_us(hough_start, hough_end),
        elapsed_us(pipeline_start, pipeline_end),
        None,
    )


def new_measurements():
    return {
        "hough_us": [],
        "pipeline_us": [],
        "coordinates": [],
        "magnitudes": [],
        "errors": {},
    }


def add_measurement(measurements, record, hough_us, pipeline_us, error):
    measurements["hough_us"].append(hough_us)
    measurements["pipeline_us"].append(pipeline_us)
    append_error(measurements["errors"], error)
    coordinate = record_coordinate(record)
    if coordinate is not None:
        measurements["coordinates"].append(coordinate)
        if record[3] is not None:
            measurements["magnitudes"].append(record[3])


def print_measurement_summary(stage, profile_name, measurements, total_frames):
    coordinates = measurements["coordinates"]
    xs = [item[0] for item in coordinates]
    ys = [item[1] for item in coordinates]
    radii = [item[2] for item in coordinates]
    magnitudes = measurements["magnitudes"]
    coordinate_reference = coordinate_median(coordinates)
    print(
        "PROFILE",
        stage,
        profile_name,
        "mode/frames/selected/rate-percent=",
        TEST_MODE,
        total_frames,
        len(coordinates),
        format_number(len(coordinates) * 100.0 / max(total_frames, 1), 1),
    )
    print(
        "TIMING",
        stage,
        profile_name,
        "hough-p50/p95 pipeline-p50/p95-us=",
        format_number(percentile(measurements["hough_us"], 50), 0),
        format_number(percentile(measurements["hough_us"], 95), 0),
        format_number(percentile(measurements["pipeline_us"], 50), 0),
        format_number(percentile(measurements["pipeline_us"], 95), 0),
    )
    print(
        "COORD",
        stage,
        profile_name,
        "median-x/y/r span-x/y/r=",
        coordinate_reference if coordinate_reference is not None else "NA",
        (
            max(xs) - min(xs),
            max(ys) - min(ys),
            max(radii) - min(radii),
        )
        if coordinates
        else "NA",
    )
    print(
        "MAGNITUDE",
        stage,
        profile_name,
        "count/min/p5/median=",
        len(magnitudes),
        format_number(min(magnitudes) if magnitudes else None, 0),
        format_number(percentile(magnitudes, 5), 0),
        format_number(percentile(magnitudes, 50), 0),
    )
    if measurements["errors"]:
        print("ERRORS", stage, profile_name, measurements["errors"])
    return coordinate_reference


def print_paired_delta_summary(stage, paired_deltas, total_frames):
    dx = [item[1] for item in paired_deltas]
    dy = [item[2] for item in paired_deltas]
    dr = [item[3] for item in paired_deltas]
    abs_dx = [abs(value) for value in dx]
    abs_dy = [abs(value) for value in dy]
    abs_dr = [abs(value) for value in dr]
    print(
        "DELTA",
        stage,
        "aggressive-minus-0140 same-frame matched/total/rate-percent=",
        len(paired_deltas),
        total_frames,
        format_number(len(paired_deltas) * 100.0 / max(total_frames, 1), 1),
    )
    print(
        "DELTA",
        stage,
        "signed-median-dx/dy/dr signed-p95-dx/dy/dr "
        "abs-p50-dx/dy/dr abs-p95-dx/dy/dr=",
        (
            percentile(dx, 50),
            percentile(dy, 50),
            percentile(dr, 50),
        )
        if paired_deltas
        else "NA",
        (
            percentile(dx, 95),
            percentile(dy, 95),
            percentile(dr, 95),
        )
        if paired_deltas
        else "NA",
        (
            percentile(abs_dx, 50),
            percentile(abs_dy, 50),
            percentile(abs_dr, 50),
        )
        if paired_deltas
        else "NA",
        (
            percentile(abs_dx, 95),
            percentile(abs_dy, 95),
            percentile(abs_dr, 95),
        )
        if paired_deltas
        else "NA",
    )
    if PRINT_MATCHED_FRAME_DELTAS:
        for frame_index, frame_dx, frame_dy, frame_dr in paired_deltas:
            print(
                "DELTA_ROW",
                stage,
                "frame/dx/dy/dr=",
                frame_index,
                frame_dx,
                frame_dy,
                frame_dr,
            )


def paired_full_benchmark(cam):
    print("===== SAME-FRAME FULL SEARCH A/B =====")
    measurements = {
        BASELINE_PROFILE["name"]: new_measurements(),
        AGGRESSIVE_PROFILE["name"]: new_measurements(),
    }
    paired_deltas = []
    read_times = []
    for frame_index in range(HOUGH_SAMPLE_COUNT):
        read_start = now_us()
        img = cam.read()
        read_times.append(elapsed_us(read_start))

        # Alternating order removes most first/second-call cache bias while
        # preserving the crucial property that both algorithms see one frame.
        ordered_profiles = (
            PROFILES if frame_index % 2 == 0 else tuple(reversed(PROFILES))
        )
        frame_records = {}
        for profile in ordered_profiles:
            record, hough_us, pipeline_us, error = run_detection(
                img,
                profile["full"],
                "full",
                allow_centered_fallback=profile[
                    "allow_centered_fallback_on_primary"
                ],
            )
            frame_records[profile["name"]] = record
            add_measurement(
                measurements[profile["name"]],
                record,
                hough_us,
                pipeline_us,
                error,
            )

        baseline = record_coordinate(frame_records[BASELINE_PROFILE["name"]])
        aggressive = record_coordinate(
            frame_records[AGGRESSIVE_PROFILE["name"]]
        )
        if baseline is not None and aggressive is not None:
            paired_deltas.append(
                (
                    frame_index,
                    aggressive[0] - baseline[0],
                    aggressive[1] - baseline[1],
                    aggressive[2] - baseline[2],
                )
            )

    print(
        "CAPTURE_DURING_AB p50/p95-us effective-p50-fps=",
        format_number(percentile(read_times, 50), 0),
        format_number(percentile(read_times, 95), 0),
        format_number(
            1000000.0 / max(percentile(read_times, 50) or 1, 1),
            1,
        ),
    )
    references = {}
    for profile in PROFILES:
        references[profile["name"]] = print_measurement_summary(
            "FULL",
            profile["name"],
            measurements[profile["name"]],
            HOUGH_SAMPLE_COUNT,
        )
    print_paired_delta_summary("FULL", paired_deltas, HOUGH_SAMPLE_COUNT)
    print(
        "REFERENCE median-not-last 0140/aggressive=",
        references[BASELINE_PROFILE["name"]],
        references[AGGRESSIVE_PROFILE["name"]],
    )
    return references


def print_effective_tracking_range(profile, reference):
    search = effective_tracking_search(profile, reference)
    candidates = radius_candidate_count(search)
    last_radius = (
        search["r_min"] + (candidates - 1) * search["r_step"]
        if candidates
        else None
    )
    print(
        "ROI_EFFECTIVE",
        profile["name"],
        "reference/roi/range-exclusive/step/count/last-sampled=",
        reference,
        search["roi"],
        (search["r_min"], search["r_max"]),
        search["r_step"],
        candidates,
        last_radius if last_radius is not None else "NA",
    )


def paired_tracking_benchmark(cam, references):
    print("===== SAME-FRAME STATIC-REFERENCE ROI A/B =====")
    for profile in PROFILES:
        reference = references.get(profile["name"])
        if reference is None:
            print("ROI SKIP", profile["name"], "no median full-search reference")
            return
        print_effective_tracking_range(profile, reference)

    measurements = {
        BASELINE_PROFILE["name"]: new_measurements(),
        AGGRESSIVE_PROFILE["name"]: new_measurements(),
    }
    paired_deltas = []
    for frame_index in range(HOUGH_SAMPLE_COUNT):
        img = cam.read()
        ordered_profiles = (
            PROFILES if frame_index % 2 == 0 else tuple(reversed(PROFILES))
        )
        frame_records = {}
        for profile in ordered_profiles:
            reference = references[profile["name"]]
            search = effective_tracking_search(profile, reference)
            record, hough_us, pipeline_us, error = run_detection(
                img,
                search,
                "tracking",
                reference,
                profile["tracking"],
            )
            frame_records[profile["name"]] = record
            add_measurement(
                measurements[profile["name"]],
                record,
                hough_us,
                pipeline_us,
                error,
            )
        baseline = record_coordinate(frame_records[BASELINE_PROFILE["name"]])
        aggressive = record_coordinate(
            frame_records[AGGRESSIVE_PROFILE["name"]]
        )
        if baseline is not None and aggressive is not None:
            paired_deltas.append(
                (
                    frame_index,
                    aggressive[0] - baseline[0],
                    aggressive[1] - baseline[1],
                    aggressive[2] - baseline[2],
                )
            )

    for profile in PROFILES:
        print_measurement_summary(
            "ROI",
            profile["name"],
            measurements[profile["name"]],
            HOUGH_SAMPLE_COUNT,
        )
    print_paired_delta_summary("ROI", paired_deltas, HOUGH_SAMPLE_COUNT)


# ---------------------------------------------------------------------------
# Full end-to-end request simulation with dynamic ROI and fallback.
# ---------------------------------------------------------------------------

def new_pipeline_state():
    return {
        "lock": None,
        "tracking_misses": 0,
        "full_misses": 0,
        "samples": [],
        "pending_outlier": None,
        "gap_used": False,
        "fallback_calls": 0,
        "roi_calls": 0,
        "full_calls": 0,
        "preserved_gap_count": 0,
        "stability_reset_count": 0,
        "pending_outlier_count": 0,
        "pending_outlier_replacement_count": 0,
        "window_migration_count": 0,
        "anchor_update_count": 0,
        "errors": {},
    }


def reset_stability(state):
    state["samples"] = []
    state["pending_outlier"] = None
    state["gap_used"] = False


def update_tracking_anchor(state, coordinate):
    state["lock"] = coordinate
    state["tracking_misses"] = 0
    state["anchor_update_count"] += 1


def drop_tracking_lock(state):
    state["lock"] = None
    state["tracking_misses"] = 0


def note_tracking_miss(state, profile):
    state["tracking_misses"] += 1
    state["pending_outlier"] = None
    if (
        state["lock"] is not None
        and state["tracking_misses"]
        < profile["tracking"]["maximum_misses"]
    ):
        if (
            profile["preserve_first_tracking_gap"]
            and state["samples"]
            and not state["gap_used"]
        ):
            # v5.5.1: one isolated missing ROI frame may occur inside a
            # stability window without discarding its accumulated samples.
            state["gap_used"] = True
            state["preserved_gap_count"] += 1
            return
        state["stability_reset_count"] += 1
        reset_stability(state)
        return
    state["stability_reset_count"] += 1
    reset_stability(state)
    drop_tracking_lock(state)


def stable_result(state, profile, coordinate):
    settings = profile["stability"][TEST_MODE]
    samples = state["samples"]
    if not samples:
        state["samples"] = [coordinate]
        state["pending_outlier"] = None
        state["gap_used"] = False
        return None

    prospective = samples + [coordinate]
    xs = [item[0] for item in prospective]
    ys = [item[1] for item in prospective]
    radii = [item[2] for item in prospective]
    if (
        max(xs) - min(xs) > settings["full_center_span"]
        or max(ys) - min(ys) > settings["full_center_span"]
        or max(radii) - min(radii) > settings["full_radius_span"]
    ):
        if not profile["pending_outlier_migration"]:
            # Frozen 01:40 behavior: one outlier immediately moves the
            # stability window to that one sample.
            state["stability_reset_count"] += 1
            reset_stability(state)
            state["samples"] = [coordinate]
            return None

        pending = state["pending_outlier"]
        if pending is None:
            # v5.5.1: keep the current window and wait for a second point.
            state["pending_outlier"] = coordinate
            state["pending_outlier_count"] += 1
            return None

        pending_cluster_is_consistent = (
            abs(coordinate[0] - pending[0])
            <= settings["full_center_span"]
            and abs(coordinate[1] - pending[1])
            <= settings["full_center_span"]
            and abs(coordinate[2] - pending[2])
            <= settings["full_radius_span"]
        )
        if not pending_cluster_is_consistent:
            state["pending_outlier"] = coordinate
            state["pending_outlier_count"] += 1
            state["pending_outlier_replacement_count"] += 1
            return None

        # Two mutually consistent outliers establish a genuine new window.
        # Per the v5.5.1 frozen-anchor policy, anchor that window on its first
        # hit (the pending point), not on every subsequent quantized result.
        reset_stability(state)
        state["samples"] = [pending, coordinate]
        state["window_migration_count"] += 1
        update_tracking_anchor(state, pending)
        prospective = state["samples"]
        xs = [item[0] for item in prospective]
        ys = [item[1] for item in prospective]
        radii = [item[2] for item in prospective]
    else:
        state["samples"] = prospective
        state["pending_outlier"] = None

    center_span = max(max(xs) - min(xs), max(ys) - min(ys))
    radius_span = max(radii) - min(radii)
    fast_ready = (
        len(prospective) >= settings["fast_samples"]
        and center_span <= settings["fast_center_span"]
        and radius_span <= settings["fast_radius_span"]
    )
    full_ready = len(prospective) >= settings["full_samples"]
    if not (fast_ready or full_ready):
        return None
    return (
        int(round(sum(xs) / len(xs))),
        int(round(sum(ys) / len(ys))),
        int(round(sum(radii) / len(radii))),
    )


def process_pipeline_frame(img, profile, state):
    tracking_active = state["lock"] is not None
    used_fallback = False
    if tracking_active:
        state["roi_calls"] += 1
        search = effective_tracking_search(profile, state["lock"])
        selector = "tracking"
        reference = state["lock"]
    else:
        state["full_calls"] += 1
        fallback_after = profile["fallback_after_misses"]
        used_fallback = (
            profile["fallback"] is not None
            and fallback_after is not None
            and state["full_misses"] >= fallback_after
        )
        if used_fallback:
            state["fallback_calls"] += 1
            search = profile["fallback"]
        else:
            search = profile["full"]
        selector = "full"
        reference = None

    record, hough_us, pipeline_us, error = run_detection(
        img,
        search,
        selector,
        reference,
        profile["tracking"],
        allow_centered_fallback=(
            used_fallback
            or profile["allow_centered_fallback_on_primary"]
        ),
    )
    append_error(state["errors"], error)
    coordinate = record_coordinate(record)
    if coordinate is None:
        if tracking_active:
            note_tracking_miss(state, profile)
        else:
            state["full_misses"] += 1
            reset_stability(state)
            drop_tracking_lock(state)
        return None, hough_us, pipeline_us, used_fallback

    state["full_misses"] = 0
    if (
        not tracking_active
        or not profile["freeze_tracking_anchor"]
        or not state["samples"]
    ):
        update_tracking_anchor(state, coordinate)
    else:
        # v5.5.1 keeps the Hough ROI/radius lattice fixed for this window.
        state["tracking_misses"] = 0
    if tracking_active or profile["include_search_frame"]:
        result = stable_result(state, profile, coordinate)
    else:
        result = None
    return result, hough_us, pipeline_us, used_fallback


def end_to_end_benchmark(cam, profile):
    successful_frames = []
    successful_times = []
    all_frames = []
    all_times = []
    hough_times = []
    pipeline_times = []
    results = []
    total_fallback_calls = 0
    trials_with_fallback = 0
    total_roi_calls = 0
    total_full_calls = 0
    total_preserved_gaps = 0
    total_stability_resets = 0
    total_pending_outliers = 0
    total_pending_replacements = 0
    total_window_migrations = 0
    total_anchor_updates = 0
    aggregate_errors = {}

    for _ in range(END_TO_END_TRIAL_COUNT):
        # A new state deliberately guarantees a full-search first frame.
        state = new_pipeline_state()
        trial_start = now_us()
        result = None
        frame_count = 0
        for _frame in range(END_TO_END_MAX_FRAMES):
            frame_count += 1
            img = cam.read()
            result, hough_us, pipeline_us, _used_fallback = (
                process_pipeline_frame(img, profile, state)
            )
            hough_times.append(hough_us)
            pipeline_times.append(pipeline_us)
            if result is not None:
                break
        trial_time = elapsed_us(trial_start)
        all_frames.append(frame_count)
        all_times.append(trial_time)
        if result is not None:
            successful_frames.append(frame_count)
            successful_times.append(trial_time)
            results.append(result)
        total_fallback_calls += state["fallback_calls"]
        total_roi_calls += state["roi_calls"]
        total_full_calls += state["full_calls"]
        total_preserved_gaps += state["preserved_gap_count"]
        total_stability_resets += state["stability_reset_count"]
        total_pending_outliers += state["pending_outlier_count"]
        total_pending_replacements += state[
            "pending_outlier_replacement_count"
        ]
        total_window_migrations += state["window_migration_count"]
        total_anchor_updates += state["anchor_update_count"]
        if state["fallback_calls"]:
            trials_with_fallback += 1
        for error, count in state["errors"].items():
            aggregate_errors[error] = aggregate_errors.get(error, 0) + count

    success_count = len(successful_frames)
    print(
        "E2E",
        profile["name"],
        "mode/trials/success/success-rate-percent/total-frames=",
        TEST_MODE,
        END_TO_END_TRIAL_COUNT,
        success_count,
        format_number(success_count * 100.0 / END_TO_END_TRIAL_COUNT, 1),
        sum(all_frames),
    )
    print(
        "E2E",
        profile["name"],
        "success-frames-p50/p95 all-frames-p50/p95=",
        format_number(percentile(successful_frames, 50), 0),
        format_number(percentile(successful_frames, 95), 0),
        format_number(percentile(all_frames, 50), 0),
        format_number(percentile(all_frames, 95), 0),
    )
    print(
        "E2E",
        profile["name"],
        "success-total-time-p50/p95-ms all-total-time-p50/p95-ms=",
        format_number(
            (percentile(successful_times, 50) or 0) / 1000.0
            if successful_times
            else None,
            1,
        ),
        format_number(
            (percentile(successful_times, 95) or 0) / 1000.0
            if successful_times
            else None,
            1,
        ),
        format_number((percentile(all_times, 50) or 0) / 1000.0, 1),
        format_number((percentile(all_times, 95) or 0) / 1000.0, 1),
    )
    print(
        "E2E",
        profile["name"],
        "fallback-calls/trials-with-fallback full-calls/roi-calls "
        "hough-p50/p95-us pipeline-p50/p95-us=",
        total_fallback_calls,
        trials_with_fallback,
        total_full_calls,
        total_roi_calls,
        format_number(percentile(hough_times, 50), 0),
        format_number(percentile(hough_times, 95), 0),
        format_number(percentile(pipeline_times, 50), 0),
        format_number(percentile(pipeline_times, 95), 0),
    )
    print(
        "E2E_RESULT",
        profile["name"],
        "median-x/y/r=",
        coordinate_median(results) if results else "NA",
    )
    print(
        "E2E_RESILIENCE",
        profile["name"],
        "preserved-gaps/stability-resets/pending-outliers/"
        "pending-replacements/window-migrations/anchor-updates=",
        total_preserved_gaps,
        total_stability_resets,
        total_pending_outliers,
        total_pending_replacements,
        total_window_migrations,
        total_anchor_updates,
    )
    if aggregate_errors:
        print("E2E_ERRORS", profile["name"], aggregate_errors)


# ---------------------------------------------------------------------------
# Optional RGB-versus-grayscale same-frame experiment.
# ---------------------------------------------------------------------------

def grayscale_copy(img):
    if image is None:
        return None, "maix.image unavailable"
    image_format = getattr(image, "Format", None)
    gray_format = (
        getattr(image_format, "FMT_GRAYSCALE", None)
        if image_format is not None
        else None
    )
    converter = getattr(img, "to_format", None)
    if gray_format is not None and callable(converter):
        try:
            return converter(gray_format), None
        except Exception as exc:
            first_error = "to_format:" + safe_text(exc)
    else:
        first_error = "to_format/FMT_GRAYSCALE unavailable"
    converter = getattr(img, "to_grayscale", None)
    if callable(converter):
        try:
            return converter(), None
        except Exception as exc:
            return None, first_error + ";to_grayscale:" + safe_text(exc)
    return None, first_error


def grayscale_benchmark(cam):
    if not ENABLE_GRAYSCALE_COMPARISON:
        print("GRAY disabled; set ENABLE_GRAYSCALE_COMPARISON=True to test")
        return
    print("===== OPTIONAL SAME-FRAME RGB/GRAY AGGRESSIVE FULL =====")
    rgb_measurements = new_measurements()
    gray_measurements = new_measurements()
    paired_deltas = []
    conversion_times = []
    for frame_index in range(GRAYSCALE_SAMPLE_COUNT):
        img = cam.read()
        rgb_record, hough_us, pipeline_us, error = run_detection(
            img,
            AGGRESSIVE_PROFILE["full"],
            "full",
            allow_centered_fallback=AGGRESSIVE_PROFILE[
                "allow_centered_fallback_on_primary"
            ],
        )
        add_measurement(
            rgb_measurements, rgb_record, hough_us, pipeline_us, error
        )
        conversion_start = now_us()
        gray, gray_error = grayscale_copy(img)
        conversion_times.append(elapsed_us(conversion_start))
        if gray is None:
            append_error(gray_measurements["errors"], gray_error)
            break
        gray_record, hough_us, pipeline_us, error = run_detection(
            gray,
            AGGRESSIVE_PROFILE["full"],
            "full",
            allow_centered_fallback=AGGRESSIVE_PROFILE[
                "allow_centered_fallback_on_primary"
            ],
        )
        add_measurement(
            gray_measurements, gray_record, hough_us, pipeline_us, error
        )
        rgb_coordinate = record_coordinate(rgb_record)
        gray_coordinate = record_coordinate(gray_record)
        if rgb_coordinate is not None and gray_coordinate is not None:
            paired_deltas.append(
                (
                    frame_index,
                    gray_coordinate[0] - rgb_coordinate[0],
                    gray_coordinate[1] - rgb_coordinate[1],
                    gray_coordinate[2] - rgb_coordinate[2],
                )
            )
    completed = len(rgb_measurements["hough_us"])
    print_measurement_summary(
        "RGB", AGGRESSIVE_PROFILE["name"], rgb_measurements, completed
    )
    print_measurement_summary(
        "GRAY", AGGRESSIVE_PROFILE["name"], gray_measurements, completed
    )
    print_paired_delta_summary("GRAY-minus-RGB", paired_deltas, completed)
    print(
        "GRAY conversion-p50/p95-us=",
        format_number(percentile(conversion_times, 50), 0),
        format_number(percentile(conversion_times, 95), 0),
    )


# ---------------------------------------------------------------------------
# Camera/hardware setup and main run.
# ---------------------------------------------------------------------------

def print_system_information():
    print("===== HARDWARE / SOFTWARE =====")
    print("TIMER source/resolution-us=", TIMER_SOURCE, 1 if TIMER_SOURCE == "ticks_us" else 1000)
    print("PYTHON version=", safe_text(getattr(sys, "version", "unknown")))
    implementation = getattr(sys, "implementation", None)
    print("PYTHON implementation=", safe_text(implementation))
    uname = getattr(os, "uname", None)
    if callable(uname):
        try:
            print("OS uname=", safe_text(uname()))
        except Exception as exc:
            print("OS uname-error=", safe_text(exc))
    for path in (
        "/proc/device-tree/model",
        "/proc/cpuinfo",
        "/etc/os-release",
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/thermal/thermal_zone1/temp",
        "/sys/class/thermal/thermal_zone2/temp",
        "/sys/class/thermal/thermal_zone3/temp",
    ):
        contents = read_small_system_file(path)
        if contents:
            print("SYSTEM_FILE", path, "=", safe_text(contents))
    print_safe_values("GC", gc, ("mem_free", "mem_alloc"))
    if maix_sys is not None:
        print_safe_values(
            "MAIX_SYS",
            maix_sys,
            (
                "device_id",
                "device_name",
                "device_configs",
                "cpu_freq",
                "npu_freq",
            ),
        )
        print("MAIX_SYS_API", ",".join(safe_dir(maix_sys)))


def create_camera():
    attempts = (
        (
            "width-height-fps-buffer",
            lambda: camera.Camera(
                IMAGE_WIDTH,
                IMAGE_HEIGHT,
                fps=TARGET_CAMERA_FPS,
                buff_num=CAMERA_BUFFER_COUNT,
            ),
        ),
        (
            "width-height-fps",
            lambda: camera.Camera(
                IMAGE_WIDTH,
                IMAGE_HEIGHT,
                fps=TARGET_CAMERA_FPS,
            ),
        ),
        (
            "width-height-only",
            lambda: camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT),
        ),
    )
    errors = []
    for label, constructor in attempts:
        try:
            cam = constructor()
            print("CAMERA_CONSTRUCTOR selected=", label)
            if errors:
                print("CAMERA_CONSTRUCTOR earlier-errors=", errors)
            return cam
        except Exception as exc:
            errors.append(label + ":" + safe_text(exc))
    raise RuntimeError("all camera constructors failed: " + safe_text(errors))


def set_optional_camera_parameter(cam, name, value):
    if value <= 0:
        return
    setter = getattr(cam, name, None)
    if not callable(setter):
        print("CAMERA_SET", name, "unsupported")
        return
    try:
        setter(value)
        print("CAMERA_SET", name, "requested=", value, "ok=1")
    except Exception as exc:
        print(
            "CAMERA_SET",
            name,
            "requested=",
            value,
            "ok=0 error=",
            safe_text(exc),
        )


def warm_up_camera(cam):
    skipper = getattr(cam, "skip_frames", None)
    if callable(skipper):
        try:
            skipper(WARMUP_FRAMES)
            print("WARMUP skip_frames=", WARMUP_FRAMES)
            return
        except Exception as exc:
            print("WARMUP skip_frames failed=", safe_text(exc))
    for _ in range(WARMUP_FRAMES):
        cam.read()
    print("WARMUP manual-reads=", WARMUP_FRAMES)


def print_camera_information(cam, sample_image):
    print("===== CAMERA =====")
    print(
        "CAMERA_REQUEST width/height/fps/buffers/exposure-us/gain=",
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        TARGET_CAMERA_FPS,
        CAMERA_BUFFER_COUNT,
        MANUAL_EXPOSURE_US,
        MANUAL_GAIN,
    )
    print_safe_values(
        "CAMERA_MODULE",
        camera,
        (
            "get_device_name",
            "list_devices",
            "get_devices",
            "device_list",
        ),
    )
    print("CAMERA_MODULE_API", ",".join(safe_dir(camera)))
    print_safe_values(
        "CAMERA_PARAM",
        cam,
        (
            "width",
            "height",
            "fps",
            "buff_num",
            "exposure",
            "gain",
            "hmirror",
            "vflip",
            "luma",
            "contrast",
            "saturation",
            "awb_mode",
            "channel",
            "format",
            "device",
            "get_device_name",
            "is_opened",
        ),
    )
    print("CAMERA_API", ",".join(safe_dir(cam)))
    if sample_image is not None:
        print_safe_values(
            "IMAGE_PARAM",
            sample_image,
            ("width", "height", "format", "size"),
        )
        print("IMAGE_API", ",".join(safe_dir(sample_image)))


def measure_capture_fps(cam):
    read_times = []
    start = now_us()
    for _ in range(CAPTURE_SAMPLE_COUNT):
        read_start = now_us()
        cam.read()
        read_times.append(elapsed_us(read_start))
    elapsed = max(elapsed_us(start), 1)
    print(
        "CAPTURE samples/total-ms/fps/read-p50/p95-us=",
        CAPTURE_SAMPLE_COUNT,
        format_number(elapsed / 1000.0, 1),
        format_number(CAPTURE_SAMPLE_COUNT * 1000000.0 / elapsed, 2),
        format_number(percentile(read_times, 50), 0),
        format_number(percentile(read_times, 95), 0),
    )


def turn_on_illumination():
    try:
        pinmap.set_pin_function("B3", "GPIOB3")
        led = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
        led.value(1)
        print("ILLUMINATION B3/GPIOB3 on=1")
        return led
    except Exception as exc:
        print("ILLUMINATION setup failed; continuing error=", safe_text(exc))
        return None


def main():
    if TEST_MODE not in (9, 10):
        raise ValueError("TEST_MODE must be exactly 9 or 10")
    if MODE10_AGGRESSIVE_RADIUS_VARIANT not in (
        "runtime",
        "comparison-40-110-r3",
        "legacy-28-96-r3",
    ):
        raise ValueError(
            "MODE10_AGGRESSIVE_RADIUS_VARIANT must be runtime or "
            "comparison-40-110-r3 or legacy-28-96-r3"
        )
    if HOUGH_SAMPLE_COUNT < 100:
        raise ValueError("HOUGH_SAMPLE_COUNT must be at least 100")
    if ENABLE_GRAYSCALE_COMPARISON and GRAYSCALE_SAMPLE_COUNT < 100:
        raise ValueError("GRAYSCALE_SAMPLE_COUNT must be at least 100")

    print("===== VISION SPEED PROBE START =====")
    print(
        "TEST mode/scene/hough-samples/e2e-trials/e2e-max-frames=",
        TEST_MODE,
        SCENE_LABEL,
        HOUGH_SAMPLE_COUNT,
        END_TO_END_TRIAL_COUNT,
        END_TO_END_MAX_FRAMES,
    )
    print(
        "TEST_INSTRUCTION mode9=normal-three-rings; "
        "mode10=repeat-separately-for-ring1-and-ring3"
    )
    print(
        "MODE10_RADIUS_VARIANT requested/active=",
        MODE10_AGGRESSIVE_RADIUS_VARIANT,
        MODE10_RADIUS_VARIANT_ACTIVE,
    )
    print(
        "MODE10_RADIUS_BOUNDARY r_max-is-exclusive; runtime=[40,111) "
        "permits-r110; comparison=[40,110) excludes-r110; nominal "
        "r40-step3-grid-points-end-at109"
    )
    for profile in PROFILES:
        full_search = profile["full"]
        full_count = radius_candidate_count(full_search)
        full_last_radius = (
            full_search["r_min"]
            + (full_count - 1) * full_search["r_step"]
            if full_count
            else None
        )
        print(
            "PROFILE_CONFIG",
            profile["name"],
            "full/fallback/tracking/stability/include-search-frame/"
            "preserve-gap/pending-outlier/freeze-anchor/valid-roi-range/"
            "centered-fallback-on-primary=",
            profile["full"],
            profile["fallback"],
            profile["tracking"],
            profile["stability"][TEST_MODE],
            int(profile["include_search_frame"]),
            int(profile["preserve_first_tracking_gap"]),
            int(profile["pending_outlier_migration"]),
            int(profile["freeze_tracking_anchor"]),
            int(profile["force_valid_tracking_radius_range"]),
            int(profile["allow_centered_fallback_on_primary"]),
        )
        print(
            "PROFILE_RADIUS_RANGE",
            profile["name"],
            "full-range-exclusive/step/nominal-grid-count/last-grid "
            "tracking-hard-range-exclusive=",
            (full_search["r_min"], full_search["r_max"]),
            full_search["r_step"],
            full_count,
            full_last_radius if full_last_radius is not None else "NA",
            (
                profile["tracking"]["r_min"],
                profile["tracking"]["r_max"],
            ),
        )
    print_system_information()
    illumination_led = turn_on_illumination()
    cam = create_camera()
    set_optional_camera_parameter(cam, "exposure", MANUAL_EXPOSURE_US)
    set_optional_camera_parameter(cam, "gain", MANUAL_GAIN)
    warm_up_camera(cam)
    sample_image = cam.read()
    print_camera_information(cam, sample_image)
    measure_capture_fps(cam)

    references = paired_full_benchmark(cam)
    paired_tracking_benchmark(cam, references)

    print("===== COLD-START END-TO-END REQUESTS =====")
    for profile in PROFILES:
        end_to_end_benchmark(cam, profile)

    grayscale_benchmark(cam)
    print_safe_values(
        "CAMERA_AFTER",
        cam,
        ("fps", "buff_num", "exposure", "gain"),
    )
    gc.collect()
    print_safe_values("GC_AFTER", gc, ("mem_free", "mem_alloc"))
    print(
        "NEXT send every TEST/MODE10/PROFILE_CONFIG/PROFILE_RADIUS/"
        "CAMERA/CAPTURE/TIMING/COORD/MAGNITUDE/DELTA/ROI_EFFECTIVE/"
        "E2E/E2E_RESILIENCE line.  Repeat mode10 once "
        "with ring1 and once with ring3, then repeat the worst lighting/"
        "reflection/no-target scene before accepting aggressive settings."
    )
    print("===== VISION SPEED PROBE COMPLETE =====")

    # Keep local handles alive until the user exits the app.  If app.need_exit
    # is unavailable on a firmware build, simply return after printing data.
    need_exit = getattr(app, "need_exit", None)
    if callable(need_exit):
        while not need_exit():
            sleep_ms(100)
    return cam, illumination_led


main()
