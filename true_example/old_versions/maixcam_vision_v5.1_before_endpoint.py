"""GongChuang competition vision firmware v5 for MaixCAM Pro."""

from maix import app, uart, time, pinmap, camera, display, image, gpio

VISION_VERSION = "5.2.0"
PROTOCOL_VERSION = 2
CALIBRATION_VERSION = "UNVERIFIED-2026-07-30-ENDPOINT-SCAN"

# MaixCAM Pro 的照明 LED 连接到 B3，高电平点亮。
# 程序启动后立即将 B3 复用为 GPIO 输出并保持高电平。
pinmap.set_pin_function("B3", "GPIOB3")
illumination_led = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
illumination_led.value(1)

# A16/A17 分别对应 UART0_TX/UART0_RX，连接 STM32 时 TX/RX 交叉连接
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

# 初始化摄像头和显示屏
cam = camera.Camera(320, 240)
disp = display.Display()
image_center_x = 160
image_center_y = 120

area_threshold = 1500
pixels_threshold = 1000
# 颜色阈值定义
red_thresholds = [[0, 80, 10, 80, 0, 40]]
yellow_thresholds = [[20, 100, -20, 30, 20, 100]]
blue_thresholds = [[-10, 70, -10, 40, -70, -8]]
green_thresholds = [[0, 80, -120, -3, 10, 50]]

# 霍夫圆检测参数
# threshold 越大，检测越严格；当前7500比MaixVision内置示例的3000更严格。
# M7固定在-90 mm低头识别时，1/2/3的弯曲笔画会产生半径约8～20 px的
# 小圆候选；它们不是圆环外沿。先在霍夫阶段排除过小半径，再由下面的
# 自适应尺度筛选处理剩余候选，避免数字2、3阻塞中心圆确认。
circle_threshold = 7500
circle_r_min = 28
circle_r_max = 100
circle_r_step = 2
ring_candidate_min_radius_pixels = 28
# 同一帧只保留不小于最大候选半径55%的圆。真实三环的透视半径差已有
# 1.60倍容差，而数字笔画通常远小于圆环外沿。
ring_candidate_min_largest_radius_ratio = 0.55
require_three_ring_layout = True
ring_min_spacing_pixels = 20
ring_max_spacing_ratio = 1.45
ring_max_line_error_ratio = 0.18
ring_max_radius_ratio = 1.60
# 低头定位后实车视野可能只完整保留中间环。三环整体优先；严格三环匹配
# 失败时，仅允许“靠近画面中心且没有距离相近竞争者”的圆作为中间环。
allow_centered_ring_fallback = True
ring_fallback_max_center_distance_pixels = 70
ring_fallback_min_distance_margin_pixels = 15
# 单圆回退时只在近中心候选中比较半径接近最大外环的圆；数字小圆即使
# 比外环圆心更接近画面中心，也不能抢占目标或制造“歧义而永不发送”。
ring_fallback_min_dominant_radius_ratio = 0.75
# 模式10用于机械臂分别扫描1号和3号端点。机械臂先把待测圆移动到画面中心附近，
# 因此不要求同时看见三个圆；仍先复用外环尺寸过滤，再从中心附近的主导尺寸外环
# 中选择离画面中心最近者。
endpoint_max_center_distance_pixels = 90
endpoint_min_dominant_radius_ratio = 0.75

# 霍夫圆协议的连续命中计时
circle_start_time = None
# 霍夫圆沿用0.5秒连续命中时间
duration = 500
# 新协议0x08：四种颜色分别维护独立稳定窗口。
# 只有连续0.3秒内横、纵坐标各自的最大值与最小值之差均不超过3像素，
# 才允许返回该颜色和坐标。
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

# 圆模式既要持续命中，也要确认连续帧锁定的是同一个物理圆。
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
circle_sample_count = 0
circle_diagnostic_interval_ms = 500
last_circle_diagnostic_time = 0

# 多个颜色同时稳定时，从该游标开始轮询。游标不会因请求结束而清零，
# 避免一个不属于当前批次但长期可见的颜色持续占用每次响应。
color_round_robin_cursor = 0

# 记录上一次发送坐标的时间，初始值为 0
last_send_time = 0
# 设定发送间隔为 0.1 秒（100 毫秒）
send_interval = 100
# 检测模式：8=同时识别红黄蓝绿，9=三圆布局的中间圆，10=端点单圆，其他=停止。
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
    """CRC-8, polynomial 0x07, initial value 0."""
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
    """Return (sequence, mode) for one valid six-byte request."""
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
    """Build one newline-terminated, CRC-protected protocol-v2 line."""
    payload = (
        f"V2,{sequence},{mode},{status},{target_id},"
        f"{x},{y},{metric},{confidence},{timestamp}"
    )
    checksum = crc8(payload.encode("ascii"))
    return f"{payload}*{checksum:02X}\n"


def stability_confidence(span, allowed_span):
    """Heuristic 0..1000 stability score; it is not a probability."""
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
        # 当前点已经超出稳定窗口，以当前点重新开始0.3秒计时。
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
    global circle_sum_x, circle_sum_y, circle_sample_count

    circle_start_time = None
    circle_min_x = 0
    circle_max_x = 0
    circle_min_y = 0
    circle_max_y = 0
    circle_min_r = 0
    circle_max_r = 0
    circle_sum_x = 0
    circle_sum_y = 0
    circle_sample_count = 0


def update_circle_stability(circle, current_time):
    global circle_start_time
    global circle_min_x, circle_max_x, circle_min_y, circle_max_y
    global circle_min_r, circle_max_r
    global circle_sum_x, circle_sum_y, circle_sample_count

    x, y, radius = circle.x(), circle.y(), circle.r()
    if circle_start_time is None:
        circle_start_time = current_time
        circle_min_x = circle_max_x = x
        circle_min_y = circle_max_y = y
        circle_min_r = circle_max_r = radius
        circle_sum_x = x
        circle_sum_y = y
        circle_sample_count = 1
        return None

    next_min_x = min(circle_min_x, x)
    next_max_x = max(circle_max_x, x)
    next_min_y = min(circle_min_y, y)
    next_max_y = max(circle_max_y, y)
    next_min_r = min(circle_min_r, radius)
    next_max_r = max(circle_max_r, radius)
    if (
        next_max_x - next_min_x > circle_stable_max_span
        or next_max_y - next_min_y > circle_stable_max_span
        or next_max_r - next_min_r > circle_stable_max_radius_span
    ):
        reset_circle_stability()
        return update_circle_stability(circle, current_time)

    circle_min_x, circle_max_x = next_min_x, next_max_x
    circle_min_y, circle_max_y = next_min_y, next_max_y
    circle_min_r, circle_max_r = next_min_r, next_max_r
    circle_sum_x += x
    circle_sum_y += y
    circle_sample_count += 1

    if (
        current_time - circle_start_time >= duration
        and circle_sample_count >= circle_stable_minimum_samples
    ):
        return (
            int(round(circle_sum_x / circle_sample_count)),
            int(round(circle_sum_y / circle_sample_count)),
        )
    return None


def filter_outer_ring_candidates(circles):
    """Reject digit-sized Hough circles before layout selection."""
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
    """Return the middle member of the best three-ring constellation."""
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
    """Return one unambiguous near-center ring, otherwise None."""
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
    """Return the dominant outer ring nearest the image center.

    Mode 10 is used after the arm has moved one endpoint ring close to the
    image center. It deliberately does not require a three-ring layout.
    """
    outer_circles = filter_outer_ring_candidates(circles)
    if not outer_circles:
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
        for circle in outer_circles
        if center_distance_squared(circle)
        <= maximum_distance_squared
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

    # 协议v2请求固定为 AA,02,seq,mode,crc,BB。
    while Serial_Maix0.available():
        byte = Serial_Maix0.read(1)[0]
        if not buffer and byte != frame_header:
            continue
        buffer.append(byte)

        if len(buffer) == request_frame_length:
            decoded_request = decode_v2_request_frame(buffer)
            if decoded_request is None:
                # CRC/帧尾错误时保留最后一个可能的新帧头，快速重新同步。
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
            if data == 0x08:
                # 模式8每个请求序号只允许一条响应；周期重发不能再次触发。
                new_detection_mode = (
                    0
                    if request_sequence
                    == completed_color_request_sequence
                    else 8
                )
            elif data == 0x09:
                new_detection_mode = 9
            elif data == 0x0A:
                new_detection_mode = 10
            else:
                new_detection_mode = 0

            # 新序号代表新的独立请求；相同序号的周期重发不得清空稳定窗口。
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

    # 新协议0x08在同一帧中检查全部四种颜色。
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

            # 同色出现多个色块时使用画面面积最大的一个，避免列表顺序变化
            # 导致稳定窗口在不同目标之间跳动。
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

            # 每个模式8请求最多回复一条；保持轮询游标，只清检测窗口。
            # STM32下一次发送新的v2请求序号后会重新开始检测。
            current_color_to_detect = 0
            completed_color_request_sequence = (
                current_request_sequence
            )
            reset_all_color_stability()

    elif current_color_to_detect == 9:
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
            # 正式模式必须先找到近似共线、等间距、等半径的三圆整体，
            # 再把中间成员作为2号基准。低头后若视野只完整保留中间环，
            # 则使用带中心距离和歧义门限的受限回退，不能无条件取最近圆。
            if require_three_ring_layout:
                circle = select_center_ring(circles)
                if circle is not None:
                    circle_selection_source = "three-ring"
                elif allow_centered_ring_fallback:
                    circle = select_centered_ring_fallback(circles)
                    if circle is not None:
                        circle_selection_source = "center-fallback"
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
                x, y = stable_coordinate
                span = max(
                    circle_max_x - circle_min_x,
                    circle_max_y - circle_min_y,
                    circle_max_r - circle_min_r,
                )
                confidence = stability_confidence(
                    span,
                    max(
                        circle_stable_max_span,
                        circle_stable_max_radius_span,
                    ),
                )
                result = build_v2_response(
                    current_request_sequence,
                    9,
                    protocol_status_ok,
                    2,
                    x,
                    y,
                    circle.r(),
                    confidence,
                    current_time,
                )
                Serial_Maix0.write_str(result)
                print(
                    f"Detected circle ({circle_selection_source}) "
                    f"stable for 0.5s, sent result: {result}"
                )
                last_send_time = current_time
                reset_circle_stability()
                # 模式9保持激活并约每0.5秒返回一次，让STM32能够连续取得
                # 两个圆心样本完成稳定判定；完成后STM32发送v2停止帧。
        else:
            reset_circle_stability()

    # 绘制检测到的色块轮廓，将方形改为圆形
    all_blobs = red_blobs + yellow_blobs + blue_blobs + green_blobs
    for b in all_blobs:
        # 获取色块的中心点和半径
        x = b.cx()
        y = b.cy()
        # 计算半径，这里简单使用宽度和高度的平均值
        radius = (b.w() + b.h()) // 4
        img.draw_circle(x, y, radius, color=image.COLOR_RED)

    # 绘制霍夫圆检测结果
    for c in circles:
        img.draw_circle(c.x(), c.y(), c.r(), color=image.COLOR_GREEN, thickness=2)
    # 绿色=原始霍夫候选；蓝色=通过外环尺度过滤；红色=最终2号目标。
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
