
from maix import app, uart, time, pinmap, camera, display, image, gpio

VISION_VERSION = "5.5.1-aggressive-latency"
PROTOCOL_VERSION = 2
CALIBRATION_VERSION = (
    "UNVERIFIED-2026-08-03-AGGRESSIVE-LATENCY-R2"
)

pinmap.set_pin_function("B3", "GPIOB3")
illumination_led = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
illumination_led.value(1)

pinmap.set_pin_function("A17", "UART0_RX")
pinmap.set_pin_function("A16", "UART0_TX")
Serial_Maix0 = uart.UART("/dev/ttyS0", 115200)
print(
    "GongChuang vision",
    VISION_VERSION,
    "protocol",
    PROTOCOL_VERSION,
    "calibration",
    CALIBRATION_VERSION,
)

image_width = 320
image_height = 240
camera_requested_fps = -1
camera_buffer_count = 1
cam = camera.Camera(
    image_width,
    image_height,
    fps=camera_requested_fps,
    buff_num=camera_buffer_count,
)
disp = display.Display()
image_center_x = image_width // 2
image_center_y = image_height // 2
try:
    camera_actual_buffer_count = cam.buff_num()
except Exception:
    camera_actual_buffer_count = -1
print(
    "Camera requested-fps/requested-buffer/actual-buffer/"
    "fps/exposure-us/gain:",
    camera_requested_fps,
    camera_buffer_count,
    camera_actual_buffer_count,
    cam.fps(),
    cam.exposure(),
    cam.gain(),
)

area_threshold = 1500
pixels_threshold = 1000

red_thresholds = [[0, 80, 10, 80, 0, 40]]
yellow_thresholds = [[20, 100, -20, 30, 20, 100]]
blue_thresholds = [[-10, 70, -10, 40, -70, -8]]
green_thresholds = [[0, 80, -120, -3, 10, 50]]

circle_threshold = 7500
# MaixPy find_circles() treats every r_max below as an exclusive bound.
circle_r_min = 28
circle_r_max = 96
circle_search_r_step = 3
endpoint_circle_r_min = 40
endpoint_circle_r_max = 111
circle_fallback_r_max = 111
circle_fallback_r_step = 2
circle_fallback_after_full_search_misses = 1
circle_tracking_r_step = 2
circle_search_x_stride = 3
circle_search_y_stride = 3
circle_fallback_x_stride = 3
circle_fallback_y_stride = 2
circle_tracking_x_stride = 3
circle_tracking_y_stride = 2
ring_candidate_min_radius_pixels = 28

ring_candidate_min_largest_radius_ratio = 0.55
require_three_ring_layout = True
ring_min_spacing_pixels = 20
ring_max_spacing_ratio = 1.45
ring_max_line_error_ratio = 0.18
ring_max_radius_ratio = 1.60

allow_centered_ring_fallback = True
ring_fallback_max_center_distance_pixels = 70
ring_fallback_min_distance_margin_pixels = 15

ring_fallback_min_dominant_radius_ratio = 0.75

endpoint_max_center_distance_pixels = 90
endpoint_min_dominant_radius_ratio = 0.75

endpoint_stable_duration = 0
endpoint_fast_stable_minimum_samples = 2
endpoint_stable_minimum_samples = 3
endpoint_fast_stable_max_span = 1
endpoint_fast_stable_max_radius_span = 1
endpoint_stable_max_span = 2
endpoint_stable_max_radius_span = 2

circle_tracking_position_margin_pixels = 8
circle_tracking_radius_margin_pixels = 3
circle_tracking_radius_lower_pixels = 2
circle_tracking_radius_upper_exclusive_pixels = 3
circle_tracking_roi_padding_pixels = 10
circle_tracking_maximum_misses = 2
circle_tracking_lock_x = None
circle_tracking_lock_y = None
circle_tracking_lock_r = None
circle_tracking_miss_count = 0
circle_stability_gap_used = False
draw_circle_diagnostics = False
show_active_preview = False
collect_circle_runtime_diagnostics = False
include_search_frame_in_stability = True
retain_tracking_lock_between_results = True
preserve_stability_on_first_tracking_miss = True
freeze_tracking_anchor_during_stability = True
active_preview_interval_ms = 50
idle_preview_interval_ms = 50
last_preview_time = 0

circle_start_time = None

duration = 0

multi_color_duration = 300
multi_color_max_span = 3
multi_color_minimum_samples = 5

target_color_duration = 0
target_color_max_span = 12
target_color_minimum_samples = 2
color_stable_start_times = [None, None, None, None]
color_min_x = [0, 0, 0, 0]
color_max_x = [0, 0, 0, 0]
color_min_y = [0, 0, 0, 0]
color_max_y = [0, 0, 0, 0]
color_latest_x = [0, 0, 0, 0]
color_latest_y = [0, 0, 0, 0]
color_sample_counts = [0, 0, 0, 0]
color_sum_x = [0, 0, 0, 0]
color_sum_y = [0, 0, 0, 0]

circle_fast_stable_minimum_samples = 3
circle_stable_minimum_samples = 4
circle_fast_stable_max_span = 1
circle_fast_stable_max_radius_span = 2
circle_stable_max_span = 3
circle_stable_max_radius_span = 3
circle_min_x = 0
circle_max_x = 0
circle_min_y = 0
circle_max_y = 0
circle_min_r = 0
circle_max_r = 0
circle_sum_x = 0
circle_sum_y = 0
circle_sum_r = 0
circle_sample_count = 0
circle_last_window_fast = False
circle_pending_outlier = None
fast_window_confidence = 1000
circle_request_start_time = None
circle_request_frame_count = 0
circle_request_hough_time_ms = 0
circle_full_search_miss_count = 0
circle_request_fallback_search_count = 0
circle_last_result_time = None
circle_result_count = 0
deferred_circle_diagnostic = None
last_reported_circle = None

color_round_robin_cursor = 0

current_color_to_detect = 0

frame_header = 0xAA
frame_tail = 0xBB
request_frame_length = 6
protocol_status_ok = 0
protocol_status_no_target = 1
protocol_status_ambiguous = 2
protocol_status_unstable = 3
protocol_status_camera_error = 4
buffer = bytearray()
current_request_sequence = 0
completed_color_request_sequence = -1

def crc8(data):

    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def decode_v2_request_frame(frame):

    if (
        len(frame) != request_frame_length
        or frame[0] != frame_header
        or frame[1] != PROTOCOL_VERSION
        or frame[5] != frame_tail
        or frame[4] != crc8(frame[1:4])
    ):
        return None
    return frame[2], frame[3]

def build_v2_response(
    sequence,
    mode,
    status,
    target_id,
    x,
    y,
    metric,
    confidence,
    timestamp,
):

    payload = (
        f"V2,{sequence},{mode},{status},{target_id},"
        f"{x},{y},{metric},{confidence},{timestamp}"
    )
    checksum = crc8(payload.encode("ascii"))
    return f"{payload}*{checksum:02X}\n"

def detection_mode_for_request(
    request_sequence,
    requested_mode,
    completed_color_sequence,
):

    if requested_mode in (1, 2, 3, 4, 8):

        return (
            0
            if request_sequence == completed_color_sequence
            else requested_mode
        )
    if requested_mode in (9, 10):
        return requested_mode
    return 0

def stability_confidence(span, allowed_span):

    if allowed_span <= 0:
        return 0
    bounded_span = min(max(span, 0), allowed_span)
    return 1000 - int(400 * bounded_span / allowed_span)

def reset_color_stability(color_index):
    color_stable_start_times[color_index] = None
    color_min_x[color_index] = 0
    color_max_x[color_index] = 0
    color_min_y[color_index] = 0
    color_max_y[color_index] = 0
    color_latest_x[color_index] = 0
    color_latest_y[color_index] = 0
    color_sample_counts[color_index] = 0
    color_sum_x[color_index] = 0
    color_sum_y[color_index] = 0

def reset_all_color_stability():
    for color_index in range(4):
        reset_color_stability(color_index)

def update_color_stability(
    color_index,
    x,
    y,
    current_time,
    required_duration=multi_color_duration,
    allowed_span=multi_color_max_span,
    required_samples=multi_color_minimum_samples,
):
    if color_stable_start_times[color_index] is None:
        color_stable_start_times[color_index] = current_time
        color_min_x[color_index] = x
        color_max_x[color_index] = x
        color_min_y[color_index] = y
        color_max_y[color_index] = y
        color_latest_x[color_index] = x
        color_latest_y[color_index] = y
        color_sample_counts[color_index] = 1
        color_sum_x[color_index] = x
        color_sum_y[color_index] = y
        return False

    next_min_x = min(color_min_x[color_index], x)
    next_max_x = max(color_max_x[color_index], x)
    next_min_y = min(color_min_y[color_index], y)
    next_max_y = max(color_max_y[color_index], y)

    if (
        next_max_x - next_min_x > allowed_span
        or next_max_y - next_min_y > allowed_span
    ):

        reset_color_stability(color_index)
        color_stable_start_times[color_index] = current_time
        color_min_x[color_index] = x
        color_max_x[color_index] = x
        color_min_y[color_index] = y
        color_max_y[color_index] = y
        color_latest_x[color_index] = x
        color_latest_y[color_index] = y
        color_sample_counts[color_index] = 1
        color_sum_x[color_index] = x
        color_sum_y[color_index] = y
        return False

    color_min_x[color_index] = next_min_x
    color_max_x[color_index] = next_max_x
    color_min_y[color_index] = next_min_y
    color_max_y[color_index] = next_max_y
    color_latest_x[color_index] = x
    color_latest_y[color_index] = y
    color_sample_counts[color_index] += 1
    color_sum_x[color_index] += x
    color_sum_y[color_index] += y
    return (
        current_time - color_stable_start_times[color_index]
        >= required_duration
        and color_sample_counts[color_index]
        >= required_samples
    )

def stable_color_coordinate(color_index):
    count = max(color_sample_counts[color_index], 1)
    return (
        int(round(color_sum_x[color_index] / count)),
        int(round(color_sum_y[color_index] / count)),
    )

def latest_color_coordinate(color_index):
    return (
        color_latest_x[color_index],
        color_latest_y[color_index],
    )

def color_stability_profile(detection_mode):
    if detection_mode in (1, 2, 3, 4):
        return (
            target_color_duration,
            target_color_max_span,
            target_color_minimum_samples,
        )
    return (
        multi_color_duration,
        multi_color_max_span,
        multi_color_minimum_samples,
    )

def color_response_coordinate(detection_mode, color_index):
    if detection_mode in (1, 2, 3, 4):
        return latest_color_coordinate(color_index)
    return stable_color_coordinate(color_index)

def reset_circle_stability(keep_tracking_lock=False):
    global circle_start_time
    global circle_min_x, circle_max_x, circle_min_y, circle_max_y
    global circle_min_r, circle_max_r
    global circle_sum_x, circle_sum_y, circle_sum_r
    global circle_sample_count
    global circle_last_window_fast
    global circle_pending_outlier, circle_stability_gap_used
    global circle_tracking_lock_x, circle_tracking_lock_y
    global circle_tracking_lock_r, circle_tracking_miss_count

    circle_start_time = None
    circle_min_x = 0
    circle_max_x = 0
    circle_min_y = 0
    circle_max_y = 0
    circle_min_r = 0
    circle_max_r = 0
    circle_sum_x = 0
    circle_sum_y = 0
    circle_sum_r = 0
    circle_sample_count = 0
    circle_last_window_fast = False
    circle_pending_outlier = None
    circle_stability_gap_used = False
    if not keep_tracking_lock:
        circle_tracking_lock_x = None
        circle_tracking_lock_y = None
        circle_tracking_lock_r = None
        circle_tracking_miss_count = 0

def update_circle_tracking_lock(circle):
    global circle_tracking_lock_x, circle_tracking_lock_y
    global circle_tracking_lock_r, circle_tracking_miss_count

    circle_tracking_lock_x = circle.x()
    circle_tracking_lock_y = circle.y()
    circle_tracking_lock_r = circle.r()
    circle_tracking_miss_count = 0

def note_circle_tracking_hit(circle, tracking_active):
    global circle_tracking_miss_count

    if (
        not tracking_active
        or not freeze_tracking_anchor_during_stability
        or circle_start_time is None
    ):
        update_circle_tracking_lock(circle)
    else:
        circle_tracking_miss_count = 0

def circle_tracking_roi():
    if circle_tracking_lock_x is None:
        return []

    half_size = (
        circle_tracking_lock_r
        + circle_tracking_roi_padding_pixels
    )
    left = max(circle_tracking_lock_x - half_size, 0)
    top = max(circle_tracking_lock_y - half_size, 0)
    right = min(
        circle_tracking_lock_x + half_size + 1,
        image_width,
    )
    bottom = min(
        circle_tracking_lock_y + half_size + 1,
        image_height,
    )
    return [left, top, right - left, bottom - top]

def circle_tracking_radius_bounds(lock_radius, active_r_min):
    """Return a valid [r_min, r_max) range for a tracked circle."""

    tracking_r_min = min(
        max(
            active_r_min,
            lock_radius - circle_tracking_radius_lower_pixels,
        ),
        circle_fallback_r_max - 1,
    )
    tracking_r_max = min(
        circle_fallback_r_max,
        max(
            tracking_r_min + 1,
            lock_radius
            + circle_tracking_radius_upper_exclusive_pixels,
        ),
    )
    return tracking_r_min, tracking_r_max

def circle_full_search_profile(detection_mode, use_fallback):
    active_r_min = (
        endpoint_circle_r_min
        if detection_mode == 10
        else circle_r_min
    )
    active_r_max = (
        endpoint_circle_r_max
        if detection_mode == 10
        else circle_r_max
    )
    if use_fallback:
        return (
            circle_fallback_x_stride,
            circle_fallback_y_stride,
            active_r_min,
            circle_fallback_r_max,
            circle_fallback_r_step,
        )
    return (
        circle_search_x_stride,
        circle_search_y_stride,
        active_r_min,
        active_r_max,
        circle_search_r_step,
    )

def centered_ring_fallback_enabled(use_fallback_search):
    return allow_centered_ring_fallback and use_fallback_search

def select_locked_ring(circles):
    if circle_tracking_lock_x is None:
        return None

    candidates = [
        circle
        for circle in circles
        if abs(circle.x() - circle_tracking_lock_x)
        <= circle_tracking_position_margin_pixels
        and abs(circle.y() - circle_tracking_lock_y)
        <= circle_tracking_position_margin_pixels
        and abs(circle.r() - circle_tracking_lock_r)
        <= circle_tracking_radius_margin_pixels
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda circle: (
            (circle.x() - circle_tracking_lock_x) ** 2
            + (circle.y() - circle_tracking_lock_y) ** 2
            + (circle.r() - circle_tracking_lock_r) ** 2
        ),
    )

def note_circle_tracking_miss():
    global circle_tracking_miss_count
    global circle_pending_outlier, circle_stability_gap_used

    circle_tracking_miss_count += 1
    circle_pending_outlier = None
    if (
        circle_tracking_lock_x is not None
        and circle_tracking_miss_count
        < circle_tracking_maximum_misses
    ):
        if (
            preserve_stability_on_first_tracking_miss
            and circle_start_time is not None
            and not circle_stability_gap_used
        ):
            circle_stability_gap_used = True
            return
        reset_circle_stability(keep_tracking_lock=True)
        return
    reset_circle_stability()

def update_circle_stability(circle, current_time):
    global circle_start_time
    global circle_min_x, circle_max_x, circle_min_y, circle_max_y
    global circle_min_r, circle_max_r
    global circle_sum_x, circle_sum_y, circle_sum_r
    global circle_sample_count
    global circle_last_window_fast
    global circle_pending_outlier, circle_stability_gap_used
    global circle_tracking_lock_x, circle_tracking_lock_y
    global circle_tracking_lock_r, circle_tracking_miss_count

    endpoint_mode = current_color_to_detect == 10
    allowed_span = (
        endpoint_stable_max_span
        if endpoint_mode
        else circle_stable_max_span
    )
    allowed_radius_span = (
        endpoint_stable_max_radius_span
        if endpoint_mode
        else circle_stable_max_radius_span
    )
    required_duration = (
        endpoint_stable_duration
        if endpoint_mode
        else duration
    )
    required_samples = (
        endpoint_stable_minimum_samples
        if endpoint_mode
        else circle_stable_minimum_samples
    )
    fast_required_samples = (
        endpoint_fast_stable_minimum_samples
        if endpoint_mode
        else circle_fast_stable_minimum_samples
    )
    fast_allowed_span = (
        endpoint_fast_stable_max_span
        if endpoint_mode
        else circle_fast_stable_max_span
    )
    fast_allowed_radius_span = (
        endpoint_fast_stable_max_radius_span
        if endpoint_mode
        else circle_fast_stable_max_radius_span
    )

    x, y, radius = circle.x(), circle.y(), circle.r()
    if circle_start_time is None:
        circle_start_time = current_time
        circle_min_x = circle_max_x = x
        circle_min_y = circle_max_y = y
        circle_min_r = circle_max_r = radius
        circle_sum_x = x
        circle_sum_y = y
        circle_sum_r = radius
        circle_sample_count = 1
        circle_pending_outlier = None
        circle_stability_gap_used = False
        return None

    next_min_x = min(circle_min_x, x)
    next_max_x = max(circle_max_x, x)
    next_min_y = min(circle_min_y, y)
    next_max_y = max(circle_max_y, y)
    next_min_r = min(circle_min_r, radius)
    next_max_r = max(circle_max_r, radius)
    if (
        next_max_x - next_min_x > allowed_span
        or next_max_y - next_min_y > allowed_span
        or next_max_r - next_min_r > allowed_radius_span
    ):
        if circle_pending_outlier is None:
            circle_pending_outlier = (
                x,
                y,
                radius,
                current_time,
            )
            return None

        pending_x, pending_y, pending_r, pending_time = (
            circle_pending_outlier
        )
        pending_cluster_is_consistent = (
            abs(x - pending_x) <= allowed_span
            and abs(y - pending_y) <= allowed_span
            and abs(radius - pending_r) <= allowed_radius_span
        )
        if not pending_cluster_is_consistent:
            circle_pending_outlier = (
                x,
                y,
                radius,
                current_time,
            )
            return None

        reset_circle_stability(keep_tracking_lock=True)
        circle_start_time = pending_time
        circle_min_x = min(pending_x, x)
        circle_max_x = max(pending_x, x)
        circle_min_y = min(pending_y, y)
        circle_max_y = max(pending_y, y)
        circle_min_r = min(pending_r, radius)
        circle_max_r = max(pending_r, radius)
        circle_sum_x = pending_x + x
        circle_sum_y = pending_y + y
        circle_sum_r = pending_r + radius
        circle_sample_count = 2
        circle_pending_outlier = None
        if circle_tracking_lock_x is not None:
            circle_tracking_lock_x = pending_x
            circle_tracking_lock_y = pending_y
            circle_tracking_lock_r = pending_r
            circle_tracking_miss_count = 0
    else:
        circle_min_x, circle_max_x = next_min_x, next_max_x
        circle_min_y, circle_max_y = next_min_y, next_max_y
        circle_min_r, circle_max_r = next_min_r, next_max_r
        circle_sum_x += x
        circle_sum_y += y
        circle_sum_r += radius
        circle_sample_count += 1
        circle_pending_outlier = None

    center_span = max(
        circle_max_x - circle_min_x,
        circle_max_y - circle_min_y,
    )
    radius_span = circle_max_r - circle_min_r
    fast_window_ready = (
        circle_sample_count >= fast_required_samples
        and center_span <= fast_allowed_span
        and radius_span <= fast_allowed_radius_span
    )
    full_window_ready = (
        circle_sample_count >= required_samples
    )
    if (
        current_time - circle_start_time >= required_duration
        and (fast_window_ready or full_window_ready)
    ):
        circle_last_window_fast = fast_window_ready
        return (
            int(round(circle_sum_x / circle_sample_count)),
            int(round(circle_sum_y / circle_sample_count)),
            int(round(circle_sum_r / circle_sample_count)),
        )
    return None

def filter_outer_ring_candidates(circles):

    if not circles:
        return []

    largest_radius = max(circle.r() for circle in circles)
    adaptive_minimum_radius = max(
        ring_candidate_min_radius_pixels,
        largest_radius
        * ring_candidate_min_largest_radius_ratio,
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

    ordered = sorted(outer_circles, key=lambda c: c.x())
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
                    spacing_12 < ring_min_spacing_pixels
                    or spacing_23 < ring_min_spacing_pixels
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
                    spacing_ratio > ring_max_spacing_ratio
                    or line_error
                    > average_spacing * ring_max_line_error_ratio
                    or radius_ratio > ring_max_radius_ratio
                ):
                    continue

                score = (
                    abs(spacing_12 - spacing_23)
                    + line_error * 2.0
                    + (max(radii) - min(radii))
                    + abs(second.x() - image_center_x) * 0.1
                    + abs(second.y() - image_center_y) * 0.1
                )
                if best_score is None or score < best_score:
                    best_score = score
                    best_circle = second
    return best_circle

def select_centered_ring_fallback(circles):

    outer_circles = filter_outer_ring_candidates(circles)
    if not outer_circles:
        return None

    def center_distance(circle):
        dx = circle.x() - image_center_x
        dy = circle.y() - image_center_y
        return (dx * dx + dy * dy) ** 0.5

    near_center = [
        circle
        for circle in outer_circles
        if center_distance(circle)
        <= ring_fallback_max_center_distance_pixels
    ]
    if not near_center:
        return None

    largest_near_center_radius = max(
        circle.r() for circle in near_center
    )
    dominant_outer_circles = [
        circle
        for circle in near_center
        if circle.r()
        >= largest_near_center_radius
        * ring_fallback_min_dominant_radius_ratio
    ]
    ranked = sorted(
        dominant_outer_circles,
        key=center_distance,
    )
    best = ranked[0]
    best_distance = center_distance(best)
    if (
        len(ranked) > 1
        and center_distance(ranked[1]) - best_distance
        < ring_fallback_min_distance_margin_pixels
    ):
        return None
    return best

def select_endpoint_ring(circles):

    if not circles:
        return None

    def center_distance_squared(circle):
        dx = circle.x() - image_center_x
        dy = circle.y() - image_center_y
        return dx * dx + dy * dy

    maximum_distance_squared = (
        endpoint_max_center_distance_pixels
        * endpoint_max_center_distance_pixels
    )
    near_center = [
        circle
        for circle in circles
        if center_distance_squared(circle)
        <= maximum_distance_squared
    ]
    if not near_center:
        return None

    outer_circles = filter_outer_ring_candidates(near_center)
    if not outer_circles:
        return None

    largest_near_center_radius = max(
        circle.r() for circle in outer_circles
    )
    dominant_outer_circles = [
        circle
        for circle in outer_circles
        if circle.r()
        >= largest_near_center_radius
        * endpoint_min_dominant_radius_ratio
    ]
    return min(
        dominant_outer_circles,
        key=lambda circle: (
            center_distance_squared(circle),
            -circle.r(),
        ),
    )

while not app.need_exit():
    current_time = time.ticks_ms()

    while Serial_Maix0.available():
        byte = Serial_Maix0.read(1)[0]
        if not buffer and byte != frame_header:
            continue
        buffer.append(byte)

        if len(buffer) == request_frame_length:
            decoded_request = decode_v2_request_frame(buffer)
            if decoded_request is None:

                last_header_index = -1
                for index in range(1, len(buffer)):
                    if buffer[index] == frame_header:
                        last_header_index = index
                if last_header_index >= 0:
                    buffer[:] = buffer[last_header_index:]
                else:
                    buffer.clear()
                continue

            request_sequence, data = decoded_request
            new_detection_mode = detection_mode_for_request(
                request_sequence,
                data,
                completed_color_request_sequence,
            )

            if (
                new_detection_mode != current_color_to_detect
                or request_sequence != current_request_sequence
            ):
                if deferred_circle_diagnostic is not None:
                    print(deferred_circle_diagnostic)
                    deferred_circle_diagnostic = None
                reset_circle_stability()
                reset_all_color_stability()
                circle_request_start_time = (
                    current_time
                    if new_detection_mode in (9, 10)
                    else None
                )
                circle_request_frame_count = 0
                circle_request_hough_time_ms = 0
                circle_full_search_miss_count = 0
                circle_request_fallback_search_count = 0
                circle_last_result_time = None
                circle_result_count = 0
                if new_detection_mode in (9, 10):
                    last_reported_circle = None
            current_request_sequence = request_sequence
            current_color_to_detect = new_detection_mode
            buffer.clear()

    img = cam.read()
    current_time = time.ticks_ms()

    red_blobs = []
    yellow_blobs = []
    blue_blobs = []
    green_blobs = []
    circles = []
    outer_ring_candidates = []
    circle = None
    selected_circle = None
    circle_selection_source = None

    if current_color_to_detect in (1, 2, 3, 4, 8):
        color_blob_groups = [
            red_blobs,
            yellow_blobs,
            blue_blobs,
            green_blobs,
        ]
        color_threshold_groups = [
            red_thresholds,
            yellow_thresholds,
            blue_thresholds,
            green_thresholds,
        ]
        if current_color_to_detect == 8:
            active_color_indices = range(4)
        else:
            active_color_indices = (current_color_to_detect - 1,)
        (
            required_color_duration,
            allowed_color_span,
            required_color_samples,
        ) = color_stability_profile(current_color_to_detect)

        for color_index in active_color_indices:
            color_blob_groups[color_index] = img.find_blobs(
                color_threshold_groups[color_index],
                area_threshold=area_threshold,
                pixels_threshold=pixels_threshold,
            )

        red_blobs, yellow_blobs, blue_blobs, green_blobs = (
            color_blob_groups
        )
        stable_color_coordinates = [None, None, None, None]

        for color_index in active_color_indices:
            blobs = color_blob_groups[color_index]
            if not blobs:
                reset_color_stability(color_index)
                continue

            blob = max(blobs, key=lambda b: b.w() * b.h())
            x, y = blob.cx(), blob.cy()
            if update_color_stability(
                color_index,
                x,
                y,
                current_time,
                required_color_duration,
                allowed_color_span,
                required_color_samples,
            ):
                stable_color_coordinates[color_index] = (
                    color_response_coordinate(
                        current_color_to_detect,
                        color_index,
                    )
                )

        selected_color_index = None
        if current_color_to_detect == 8:
            for color_offset in range(4):
                candidate_index = (
                    color_round_robin_cursor + color_offset
                ) % 4
                if stable_color_coordinates[candidate_index] is not None:
                    selected_color_index = candidate_index
                    break
        else:
            target_color_index = current_color_to_detect - 1
            if stable_color_coordinates[target_color_index] is not None:
                selected_color_index = target_color_index

        if selected_color_index is not None:
            response_mode = current_color_to_detect
            x, y = stable_color_coordinates[selected_color_index]
            color_number = selected_color_index + 1
            selected_blob = max(
                color_blob_groups[selected_color_index],
                key=lambda b: b.w() * b.h(),
            )
            metric = min(
                selected_blob.w() * selected_blob.h(),
                65535,
            )
            span = max(
                color_max_x[selected_color_index]
                - color_min_x[selected_color_index],
                color_max_y[selected_color_index]
                - color_min_y[selected_color_index],
            )
            confidence = stability_confidence(
                span, allowed_color_span
            )
            result = build_v2_response(
                current_request_sequence,
                response_mode,
                protocol_status_ok,
                color_number,
                x,
                y,
                metric,
                confidence,
                current_time,
            )
            Serial_Maix0.write_str(result)
            print(
                f"Detected color {color_number} in mode "
                f"{response_mode}, "
                f"sent result: {result}"
            )
            if response_mode == 8:
                color_round_robin_cursor = (
                    selected_color_index + 1
                ) % 4

            current_color_to_detect = 0
            completed_color_request_sequence = (
                current_request_sequence
            )
            reset_all_color_stability()

    elif current_color_to_detect in (9, 10):
        active_circle_r_min = (
            endpoint_circle_r_min
            if current_color_to_detect == 10
            else circle_r_min
        )
        tracking_active = circle_tracking_lock_x is not None
        circle_hough_start_time = (
            time.ticks_ms()
            if collect_circle_runtime_diagnostics
            else 0
        )
        use_fallback_search = False
        if tracking_active:
            tracking_r_min, tracking_r_max = (
                circle_tracking_radius_bounds(
                    circle_tracking_lock_r,
                    active_circle_r_min,
                )
            )
            circles = img.find_circles(
                roi=circle_tracking_roi(),
                x_stride=circle_tracking_x_stride,
                y_stride=circle_tracking_y_stride,
                threshold=circle_threshold,
                r_min=tracking_r_min,
                r_max=tracking_r_max,
                r_step=circle_tracking_r_step,
            )
        else:
            use_fallback_search = (
                circle_full_search_miss_count
                >= circle_fallback_after_full_search_misses
            )
            if use_fallback_search:
                if collect_circle_runtime_diagnostics:
                    circle_request_fallback_search_count += 1
            (
                full_x_stride,
                full_y_stride,
                full_r_min,
                full_r_max,
                full_r_step,
            ) = circle_full_search_profile(
                current_color_to_detect,
                use_fallback_search,
            )
            circles = img.find_circles(
                x_stride=full_x_stride,
                y_stride=full_y_stride,
                threshold=circle_threshold,
                r_min=full_r_min,
                r_max=full_r_max,
                r_step=full_r_step,
            )
        if collect_circle_runtime_diagnostics:
            circle_request_hough_time_ms += (
                time.ticks_ms() - circle_hough_start_time
            )
            circle_request_frame_count += 1
        if circles:
            if draw_circle_diagnostics:
                outer_ring_candidates = filter_outer_ring_candidates(
                    circles
                )
            if tracking_active:
                circle = select_locked_ring(circles)
                if circle is not None:
                    circle_selection_source = "locked-roi"
            elif current_color_to_detect == 10:

                circle = select_endpoint_ring(circles)
                if circle is not None:
                    circle_selection_source = "endpoint-single"
            else:

                if require_three_ring_layout:
                    circle = select_center_ring(circles)
                    if circle is not None:
                        circle_selection_source = "three-ring"
                    elif centered_ring_fallback_enabled(
                        use_fallback_search
                    ):
                        circle = select_centered_ring_fallback(
                            circles
                        )
                        if circle is not None:
                            circle_selection_source = (
                                "center-fallback"
                            )
                else:
                    circle = min(
                        circles,
                        key=lambda c: (
                            (c.x() - image_center_x) ** 2
                            + (c.y() - image_center_y) ** 2
                        )
                    )
                    circle_selection_source = "nearest"

            if circle is None:
                if tracking_active:
                    note_circle_tracking_miss()
                else:
                    circle_full_search_miss_count += 1
                    reset_circle_stability()
                stable_coordinate = None
            else:
                if show_active_preview or draw_circle_diagnostics:
                    selected_circle = circle
                circle_full_search_miss_count = 0
                note_circle_tracking_hit(circle, tracking_active)
                if (
                    tracking_active
                    or include_search_frame_in_stability
                ):
                    stable_coordinate = update_circle_stability(
                        circle, current_time
                    )
                else:
                    stable_coordinate = None
            if stable_coordinate is not None:
                x, y, stable_radius = stable_coordinate
                span = max(
                    circle_max_x - circle_min_x,
                    circle_max_y - circle_min_y,
                    circle_max_r - circle_min_r,
                )
                confidence = (
                    fast_window_confidence
                    if circle_last_window_fast
                    else stability_confidence(
                        span,
                        max(
                            (
                                endpoint_stable_max_span
                                if current_color_to_detect == 10
                                else circle_stable_max_span
                            ),
                            (
                                endpoint_stable_max_radius_span
                                if current_color_to_detect == 10
                                else circle_stable_max_radius_span
                            ),
                        ),
                    )
                )
                result = build_v2_response(
                    current_request_sequence,
                    current_color_to_detect,
                    protocol_status_ok,
                    (
                        1
                        if current_color_to_detect == 10
                        else 2
                    ),
                    x,
                    y,
                    stable_radius,
                    confidence,
                    current_time,
                )
                Serial_Maix0.write_str(result)
                last_reported_circle = (x, y, stable_radius)
                if collect_circle_runtime_diagnostics:
                    circle_result_time = time.ticks_ms()
                    circle_result_interval_ms = max(
                        circle_result_time
                        - (
                            circle_last_result_time
                            if circle_last_result_time is not None
                            else circle_request_start_time
                        ),
                        1,
                    )
                    circle_last_result_time = circle_result_time
                    circle_result_count += 1
                    circle_request_elapsed_ms = max(
                        circle_result_time
                        - circle_request_start_time,
                        1,
                    )
                    circle_effective_fps = (
                        circle_request_frame_count
                        * 1000
                        / circle_request_elapsed_ms
                    )
                    circle_average_hough_ms = (
                        circle_request_hough_time_ms
                        / circle_request_frame_count
                    )
                    deferred_circle_diagnostic = (
                        f"Detected circle ({circle_selection_source}) "
                        f"stable and sent result: {result}; "
                        f"pipeline-frames/ms/fps: "
                        f"{circle_request_frame_count}/"
                        f"{circle_request_elapsed_ms}/"
                        f"{circle_effective_fps:.1f}; "
                        f"hough-total/avg-ms: "
                        f"{circle_request_hough_time_ms}/"
                        f"{circle_average_hough_ms:.1f}; "
                        f"result-count/last-interval-ms: "
                        f"{circle_result_count}/"
                        f"{circle_result_interval_ms}; "
                        f"fallback-full-searches: "
                        f"{circle_request_fallback_search_count}"
                    )
                reset_circle_stability(
                    keep_tracking_lock=(
                        retain_tracking_lock_between_results
                    )
                )

        else:
            if tracking_active:
                note_circle_tracking_miss()
            else:
                circle_full_search_miss_count += 1
                reset_circle_stability()

    all_blobs = red_blobs + yellow_blobs + blue_blobs + green_blobs
    for b in all_blobs:

        x = b.cx()
        y = b.cy()

        radius = (b.w() + b.h()) // 4
        img.draw_circle(x, y, radius, color=image.COLOR_RED)

    if draw_circle_diagnostics:
        for c in circles:
            img.draw_circle(
                c.x(),
                c.y(),
                c.r(),
                color=image.COLOR_GREEN,
                thickness=2,
            )

        for c in outer_ring_candidates:
            img.draw_circle(
                c.x(),
                c.y(),
                c.r(),
                color=image.COLOR_BLUE,
                thickness=2,
            )
    if (
        selected_circle is not None
        and (show_active_preview or draw_circle_diagnostics)
    ):
        img.draw_circle(
            selected_circle.x(),
            selected_circle.y(),
            selected_circle.r(),
            color=image.COLOR_RED,
            thickness=3,
        )
    elif (
        current_color_to_detect == 0
        and last_reported_circle is not None
    ):
        img.draw_circle(
            last_reported_circle[0],
            last_reported_circle[1],
            last_reported_circle[2],
            color=image.COLOR_RED,
            thickness=3,
        )

    preview_due = (
        (
            current_color_to_detect == 0
            and current_time - last_preview_time
            >= idle_preview_interval_ms
        )
        or (
            (show_active_preview or draw_circle_diagnostics)
            and current_time - last_preview_time
            >= active_preview_interval_ms
        )
    )
    if preview_due:
        disp.show(img)
        last_preview_time = current_time
