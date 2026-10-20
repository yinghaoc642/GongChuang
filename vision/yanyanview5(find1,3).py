
from maix import app, uart, time, pinmap, camera, display, image, gpio

VISION_VERSION = "5.3.0-find1,3"
PROTOCOL_VERSION = 2
CALIBRATION_VERSION = (
    "UNVERIFIED-2026-07-30-ENDPOINT-FAST-STABLE"
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

cam = camera.Camera(320, 240)
disp = display.Display()
image_center_x = 160
image_center_y = 120

area_threshold = 1500
pixels_threshold = 1000

red_thresholds = [[0, 80, 10, 80, 0, 40]]
yellow_thresholds = [[20, 100, -20, 30, 20, 100]]
blue_thresholds = [[-10, 70, -10, 40, -70, -8]]
green_thresholds = [[0, 80, -120, -3, 10, 50]]

circle_threshold = 7500
circle_r_min = 28
circle_r_max = 110
circle_r_step = 2
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

endpoint_stable_duration = 120
endpoint_stable_minimum_samples = 2
endpoint_stable_max_span = 2
endpoint_stable_max_radius_span = 2

circle_start_time = None

duration = 500

multi_color_duration = 300
multi_color_max_span = 3
multi_color_minimum_samples = 5
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

circle_stable_minimum_samples = 5
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
circle_diagnostic_interval_ms = 500
last_circle_diagnostic_time = 0

color_round_robin_cursor = 0

last_send_time = 0

send_interval = 100

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

    if requested_mode == 8:

        return (
            0
            if request_sequence == completed_color_sequence
            else 8
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

def update_color_stability(color_index, x, y, current_time):
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
        next_max_x - next_min_x > multi_color_max_span
        or next_max_y - next_min_y > multi_color_max_span
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
        >= multi_color_duration
        and color_sample_counts[color_index]
        >= multi_color_minimum_samples
    )

def stable_color_coordinate(color_index):
    count = max(color_sample_counts[color_index], 1)
    return (
        int(round(color_sum_x[color_index] / count)),
        int(round(color_sum_y[color_index] / count)),
    )

def reset_circle_stability():
    global circle_start_time
    global circle_min_x, circle_max_x, circle_min_y, circle_max_y
    global circle_min_r, circle_max_r
    global circle_sum_x, circle_sum_y, circle_sum_r
    global circle_sample_count

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

def update_circle_stability(circle, current_time):
    global circle_start_time
    global circle_min_x, circle_max_x, circle_min_y, circle_max_y
    global circle_min_r, circle_max_r
    global circle_sum_x, circle_sum_y, circle_sum_r
    global circle_sample_count

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
        reset_circle_stability()
        return update_circle_stability(circle, current_time)

    circle_min_x, circle_max_x = next_min_x, next_max_x
    circle_min_y, circle_max_y = next_min_y, next_max_y
    circle_min_r, circle_max_r = next_min_r, next_max_r
    circle_sum_x += x
    circle_sum_y += y
    circle_sum_r += radius
    circle_sample_count += 1

    if (
        current_time - circle_start_time >= required_duration
        and circle_sample_count >= required_samples
    ):
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
    img = cam.read()
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
                reset_circle_stability()
                reset_all_color_stability()
            current_request_sequence = request_sequence
            current_color_to_detect = new_detection_mode
            buffer.clear()

    red_blobs = []
    yellow_blobs = []
    blue_blobs = []
    green_blobs = []
    circles = []
    outer_ring_candidates = []
    selected_circle = None
    circle_selection_source = None

    if current_color_to_detect == 8:
        red_blobs = img.find_blobs(
            red_thresholds,
            area_threshold=area_threshold,
            pixels_threshold=pixels_threshold
        )
        yellow_blobs = img.find_blobs(
            yellow_thresholds,
            area_threshold=area_threshold,
            pixels_threshold=pixels_threshold
        )
        blue_blobs = img.find_blobs(
            blue_thresholds,
            area_threshold=area_threshold,
            pixels_threshold=pixels_threshold
        )
        green_blobs = img.find_blobs(
            green_thresholds,
            area_threshold=area_threshold,
            pixels_threshold=pixels_threshold
        )

        color_blob_groups = [
            red_blobs,
            yellow_blobs,
            blue_blobs,
            green_blobs,
        ]
        stable_color_coordinates = [None, None, None, None]

        for color_index in range(4):
            blobs = color_blob_groups[color_index]
            if not blobs:
                reset_color_stability(color_index)
                continue

            blob = max(blobs, key=lambda b: b.w() * b.h())
            x, y = blob.cx(), blob.cy()
            if update_color_stability(
                color_index, x, y, current_time
            ):
                stable_color_coordinates[color_index] = (
                    stable_color_coordinate(color_index)
                )

        selected_color_index = None
        for color_offset in range(4):
            candidate_index = (
                color_round_robin_cursor + color_offset
            ) % 4
            if stable_color_coordinates[candidate_index] is not None:
                selected_color_index = candidate_index
                break

        if (
            selected_color_index is not None
            and current_time - last_send_time >= send_interval
        ):
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
                span, multi_color_max_span
            )
            result = build_v2_response(
                current_request_sequence,
                8,
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
                f"Detected color {color_number} stable for 0.3s, "
                f"sent result: {result}"
            )
            last_send_time = current_time
            color_round_robin_cursor = (
                selected_color_index + 1
            ) % 4

            current_color_to_detect = 0
            completed_color_request_sequence = (
                current_request_sequence
            )
            reset_all_color_stability()

    elif current_color_to_detect in (9, 10):
        circles = img.find_circles(
            threshold=circle_threshold,
            r_min=circle_r_min,
            r_max=circle_r_max,
            r_step=circle_r_step
        )
        if circles:
            outer_ring_candidates = filter_outer_ring_candidates(
                circles
            )
            if (
                current_time - last_circle_diagnostic_time
                >= circle_diagnostic_interval_ms
            ):
                print(
                    "Circle candidates raw/outer radii:",
                    [circle.r() for circle in circles],
                    "/",
                    [
                        circle.r()
                        for circle in outer_ring_candidates
                    ],
                )
                last_circle_diagnostic_time = current_time
            if current_color_to_detect == 10:

                circle = select_endpoint_ring(circles)
                if circle is not None:
                    circle_selection_source = "endpoint-single"
            else:

                if require_three_ring_layout:
                    circle = select_center_ring(circles)
                    if circle is not None:
                        circle_selection_source = "three-ring"
                    elif allow_centered_ring_fallback:
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
                reset_circle_stability()
                stable_coordinate = None
            else:
                selected_circle = circle
                stable_coordinate = update_circle_stability(
                    circle, current_time
                )
            if (
                stable_coordinate is not None
                and current_time - last_send_time >= send_interval
            ):
                x, y, stable_radius = stable_coordinate
                span = max(
                    circle_max_x - circle_min_x,
                    circle_max_y - circle_min_y,
                    circle_max_r - circle_min_r,
                )
                confidence = stability_confidence(
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
                print(
                    f"Detected circle ({circle_selection_source}) "
                    f"stable and sent result: {result}"
                )
                last_send_time = current_time
                reset_circle_stability()

        else:
            reset_circle_stability()

    all_blobs = red_blobs + yellow_blobs + blue_blobs + green_blobs
    for b in all_blobs:

        x = b.cx()
        y = b.cy()

        radius = (b.w() + b.h()) // 4
        img.draw_circle(x, y, radius, color=image.COLOR_RED)

    for c in circles:
        img.draw_circle(c.x(), c.y(), c.r(), color=image.COLOR_GREEN, thickness=2)

    for c in outer_ring_candidates:
        img.draw_circle(
            c.x(),
            c.y(),
            c.r(),
            color=image.COLOR_BLUE,
            thickness=2,
        )
    if selected_circle is not None:
        img.draw_circle(
            selected_circle.x(),
            selected_circle.y(),
            selected_circle.r(),
            color=image.COLOR_RED,
            thickness=3,
        )

    disp.show(img)
    time.sleep_ms(10)
