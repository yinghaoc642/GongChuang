#!/usr/bin/env python3

from __future__ import annotations

import ast
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VISION = ROOT / "vision" / "yanyanview5.py"
CALIBRATION = ROOT / "vision" / "calibration.json"

def load_literal_constants():
    source = VISION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    constants = {}
    for node in tree.body:
        if (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
        ):
            try:
                constants[node.targets[0].id] = ast.literal_eval(
                    node.value
                )
            except (ValueError, TypeError):
                pass
    return constants

class FakeCircle:
    def __init__(self, x: int, y: int, radius: int) -> None:
        self._x = x
        self._y = y
        self._radius = radius

    def x(self) -> int:
        return self._x

    def y(self) -> int:
        return self._y

    def r(self) -> int:
        return self._radius

def load_ring_selection_functions():
    source = VISION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name
        in {
            "filter_outer_ring_candidates",
            "select_center_ring",
            "select_centered_ring_fallback",
            "select_endpoint_ring",
            "update_circle_tracking_lock",
            "note_circle_tracking_hit",
            "circle_tracking_roi",
            "circle_tracking_radius_bounds",
            "circle_full_search_profile",
            "centered_ring_fallback_enabled",
            "select_locked_ring",
        }
    ]
    module = ast.Module(body=functions, type_ignores=[])
    namespace = {
        "ring_min_spacing_pixels": 20,
        "ring_max_spacing_ratio": 1.45,
        "ring_max_line_error_ratio": 0.18,
        "ring_max_radius_ratio": 1.60,
        "ring_candidate_min_radius_pixels": 28,
        "ring_candidate_min_largest_radius_ratio": 0.55,
        "image_center_x": 160,
        "image_center_y": 120,
        "ring_fallback_max_center_distance_pixels": 70,
        "ring_fallback_min_distance_margin_pixels": 15,
        "ring_fallback_min_dominant_radius_ratio": 0.75,
        "endpoint_max_center_distance_pixels": 90,
        "endpoint_min_dominant_radius_ratio": 0.75,
        "circle_tracking_position_margin_pixels": 8,
        "circle_tracking_radius_margin_pixels": 3,
        "circle_r_min": 28,
        "circle_fallback_r_max": 111,
        "circle_tracking_radius_lower_pixels": 2,
        "circle_tracking_radius_upper_exclusive_pixels": 3,
        "circle_r_max": 96,
        "endpoint_circle_r_min": 40,
        "endpoint_circle_r_max": 111,
        "circle_search_x_stride": 3,
        "circle_search_y_stride": 3,
        "circle_search_r_step": 3,
        "circle_fallback_x_stride": 3,
        "circle_fallback_y_stride": 2,
        "circle_fallback_r_step": 2,
        "allow_centered_ring_fallback": True,
        "circle_tracking_roi_padding_pixels": 10,
        "circle_tracking_lock_x": None,
        "circle_tracking_lock_y": None,
        "circle_tracking_lock_r": None,
        "circle_tracking_miss_count": 0,
        "circle_start_time": None,
        "freeze_tracking_anchor_during_stability": True,
        "image_width": 320,
        "image_height": 240,
    }
    exec(compile(module, str(VISION), "exec"), namespace)
    return (
        namespace["select_center_ring"],
        namespace["select_centered_ring_fallback"],
        namespace["select_endpoint_ring"],
        namespace["update_circle_tracking_lock"],
        namespace["note_circle_tracking_hit"],
        namespace["circle_tracking_roi"],
        namespace["circle_tracking_radius_bounds"],
        namespace["circle_full_search_profile"],
        namespace["centered_ring_fallback_enabled"],
        namespace["select_locked_ring"],
        namespace,
    )

def load_protocol_functions():
    source = VISION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    wanted = {
        "crc8",
        "decode_v2_request_frame",
        "build_v2_response",
        "detection_mode_for_request",
    }
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in wanted
    ]
    module = ast.Module(body=functions, type_ignores=[])
    namespace = {
        "request_frame_length": 6,
        "frame_header": 0xAA,
        "frame_tail": 0xBB,
        "PROTOCOL_VERSION": 2,
    }
    exec(compile(module, str(VISION), "exec"), namespace)
    return (
        namespace["crc8"],
        namespace["decode_v2_request_frame"],
        namespace["build_v2_response"],
        namespace["detection_mode_for_request"],
    )

def load_circle_stability_functions():
    source = VISION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name
        in {
            "reset_circle_stability",
            "note_circle_tracking_miss",
            "update_circle_stability",
        }
    ]
    module = ast.Module(body=functions, type_ignores=[])
    namespace = {
        "duration": 0,
        "circle_fast_stable_minimum_samples": 3,
        "circle_stable_minimum_samples": 4,
        "circle_fast_stable_max_span": 1,
        "circle_fast_stable_max_radius_span": 2,
        "circle_stable_max_span": 3,
        "circle_stable_max_radius_span": 3,
        "endpoint_stable_duration": 0,
        "endpoint_fast_stable_minimum_samples": 2,
        "endpoint_stable_minimum_samples": 3,
        "endpoint_fast_stable_max_span": 1,
        "endpoint_fast_stable_max_radius_span": 1,
        "endpoint_stable_max_span": 2,
        "endpoint_stable_max_radius_span": 2,
        "current_color_to_detect": 9,
        "circle_start_time": None,
        "circle_min_x": 0,
        "circle_max_x": 0,
        "circle_min_y": 0,
        "circle_max_y": 0,
        "circle_min_r": 0,
        "circle_max_r": 0,
        "circle_sum_x": 0,
        "circle_sum_y": 0,
        "circle_sum_r": 0,
        "circle_sample_count": 0,
        "circle_last_window_fast": False,
        "circle_pending_outlier": None,
        "fast_window_confidence": 1000,
        "circle_tracking_lock_x": None,
        "circle_tracking_lock_y": None,
        "circle_tracking_lock_r": None,
        "circle_tracking_miss_count": 0,
        "circle_tracking_maximum_misses": 2,
        "circle_stability_gap_used": False,
        "preserve_stability_on_first_tracking_miss": True,
    }
    exec(compile(module, str(VISION), "exec"), namespace)
    return (
        namespace["update_circle_stability"],
        namespace["reset_circle_stability"],
        namespace["note_circle_tracking_miss"],
        namespace,
    )

def load_color_stability_functions():
    source = VISION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    wanted = {
        "reset_color_stability",
        "update_color_stability",
        "stable_color_coordinate",
        "latest_color_coordinate",
        "color_stability_profile",
        "color_response_coordinate",
    }
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in wanted
    ]
    module = ast.Module(body=functions, type_ignores=[])
    namespace = {
        "multi_color_duration": 300,
        "multi_color_max_span": 3,
        "multi_color_minimum_samples": 5,
        "target_color_duration": 0,
        "target_color_max_span": 12,
        "target_color_minimum_samples": 2,
        "color_stable_start_times": [None, None, None, None],
        "color_min_x": [0, 0, 0, 0],
        "color_max_x": [0, 0, 0, 0],
        "color_min_y": [0, 0, 0, 0],
        "color_max_y": [0, 0, 0, 0],
        "color_latest_x": [0, 0, 0, 0],
        "color_latest_y": [0, 0, 0, 0],
        "color_sample_counts": [0, 0, 0, 0],
        "color_sum_x": [0, 0, 0, 0],
        "color_sum_y": [0, 0, 0, 0],
    }
    exec(compile(module, str(VISION), "exec"), namespace)
    return (
        namespace["update_color_stability"],
        namespace["reset_color_stability"],
        namespace["stable_color_coordinate"],
        namespace["latest_color_coordinate"],
        namespace["color_stability_profile"],
        namespace["color_response_coordinate"],
    )

def main() -> None:
    (
        select_center_ring,
        select_centered_ring_fallback,
        select_endpoint_ring,
        update_circle_tracking_lock,
        note_circle_tracking_hit,
        circle_tracking_roi,
        circle_tracking_radius_bounds,
        circle_full_search_profile,
        centered_ring_fallback_enabled,
        select_locked_ring,
        ring_selection_namespace,
    ) = load_ring_selection_functions()
    (
        crc8,
        decode_request,
        build_response,
        detection_mode_for_request,
    ) = load_protocol_functions()
    (
        update_circle_stability,
        reset_circle_stability,
        note_circle_tracking_miss,
        circle_stability_namespace,
    ) = (
        load_circle_stability_functions()
    )
    (
        update_color_stability,
        reset_color_stability,
        stable_color_coordinate,
        latest_color_coordinate,
        color_stability_profile,
        color_response_coordinate,
    ) = load_color_stability_functions()
    assert crc8(b"123456789") == 0xF4

    assert not update_color_stability(0, 100, 100, 0, 0, 12, 2)
    assert update_color_stability(0, 108, 105, 10, 0, 12, 2)
    assert stable_color_coordinate(0) == (104, 102)
    assert latest_color_coordinate(0) == (108, 105)
    assert color_stability_profile(1) == (0, 12, 2)
    assert color_stability_profile(4) == (0, 12, 2)
    assert color_stability_profile(8) == (300, 3, 5)

    assert color_response_coordinate(1, 0) == (108, 105)
    assert color_response_coordinate(8, 0) == (104, 102)
    reset_color_stability(0)
    assert not update_color_stability(0, 100, 100, 0, 0, 12, 2)

    assert not update_color_stability(0, 113, 100, 10, 0, 12, 2)
    assert update_color_stability(0, 120, 103, 20, 0, 12, 2)

    stable_result = None
    ring_results = []
    for time_ms, radius in enumerate(
        (70, 72, 73, 71)
    ):
        stable_result = update_circle_stability(
            FakeCircle(160, 120, radius),
            time_ms,
        )
        ring_results.append(stable_result)
    assert ring_results[:3] == [None] * 3
    assert stable_result == (160, 120, 72)
    assert circle_stability_namespace["circle_last_window_fast"] is False

    reset_circle_stability()
    fast_ring_result = None
    for time_ms in range(3):
        fast_ring_result = update_circle_stability(
            FakeCircle(160 + time_ms % 2, 120, 70),
            time_ms,
        )
    assert fast_ring_result == (160, 120, 70)
    assert circle_stability_namespace["circle_last_window_fast"] is True

    reset_circle_stability()
    circle_stability_namespace[
        "current_color_to_detect"
    ] = 10
    endpoint_stable_result = None
    endpoint_results = []
    for time_ms, radius in enumerate(
        (84, 85)
    ):
        endpoint_stable_result = update_circle_stability(
            FakeCircle(160, 120, radius),
            time_ms,
        )
        endpoint_results.append(endpoint_stable_result)
    assert endpoint_results[:1] == [None]
    assert endpoint_stable_result == (160, 120, 84)
    assert circle_stability_namespace["circle_last_window_fast"] is True

    reset_circle_stability()
    full_endpoint_results = []
    for time_ms, radius in enumerate((84, 86, 85)):
        full_endpoint_results.append(
            update_circle_stability(
                FakeCircle(160, 120, radius),
                time_ms,
            )
        )
    assert full_endpoint_results[:2] == [None] * 2
    assert full_endpoint_results[2] == (160, 120, 85)
    assert circle_stability_namespace["circle_last_window_fast"] is False

    circle_stability_namespace["circle_tracking_lock_x"] = 160
    circle_stability_namespace["circle_tracking_lock_y"] = 120
    circle_stability_namespace["circle_tracking_lock_r"] = 84
    note_circle_tracking_miss()
    assert circle_stability_namespace["circle_tracking_lock_x"] == 160
    assert circle_stability_namespace["circle_tracking_miss_count"] == 1
    note_circle_tracking_miss()
    assert circle_stability_namespace["circle_tracking_lock_x"] is None
    assert circle_stability_namespace["circle_tracking_miss_count"] == 0

    update_circle_tracking_lock(FakeCircle(160, 120, 60))
    assert circle_tracking_roi() == [90, 50, 141, 141]
    assert circle_tracking_radius_bounds(96, 28) == (94, 99)
    assert circle_tracking_radius_bounds(100, 28) == (98, 103)
    assert circle_tracking_radius_bounds(109, 28) == (107, 111)
    assert circle_tracking_radius_bounds(110, 28) == (108, 111)
    assert circle_tracking_radius_bounds(40, 40) == (40, 43)
    for fallback_radius in range(97, 111):
        lower_radius, upper_radius = circle_tracking_radius_bounds(
            fallback_radius,
            28,
        )
        assert lower_radius < upper_radius
        assert lower_radius <= fallback_radius < upper_radius
    assert circle_full_search_profile(9, False) == (
        3,
        3,
        28,
        96,
        3,
    )
    assert circle_full_search_profile(9, True) == (
        3,
        2,
        28,
        111,
        2,
    )
    assert circle_full_search_profile(10, False) == (
        3,
        3,
        40,
        111,
        3,
    )
    assert circle_full_search_profile(10, True) == (
        3,
        2,
        40,
        111,
        2,
    )
    assert len(range(40, 111, 3)) == 24
    assert 110 in range(40, 111, 2)
    assert centered_ring_fallback_enabled(False) is False
    assert centered_ring_fallback_enabled(True) is True
    tracked = FakeCircle(163, 118, 62)
    assert (
        select_locked_ring(
            [tracked, FakeCircle(175, 120, 60)]
        )
        is tracked
    )
    ring_selection_namespace["circle_start_time"] = 1
    ring_selection_namespace["circle_tracking_miss_count"] = 1
    note_circle_tracking_hit(FakeCircle(162, 119, 61), True)
    assert ring_selection_namespace["circle_tracking_lock_x"] == 160
    assert ring_selection_namespace["circle_tracking_lock_y"] == 120
    assert ring_selection_namespace["circle_tracking_lock_r"] == 60
    assert ring_selection_namespace["circle_tracking_miss_count"] == 0
    ring_selection_namespace["circle_start_time"] = None
    note_circle_tracking_hit(FakeCircle(162, 119, 61), True)
    assert ring_selection_namespace["circle_tracking_lock_x"] == 162
    assert ring_selection_namespace["circle_tracking_lock_y"] == 119
    assert ring_selection_namespace["circle_tracking_lock_r"] == 61

    reset_circle_stability()
    circle_stability_namespace["current_color_to_detect"] = 9
    assert update_circle_stability(FakeCircle(160, 120, 70), 0) is None
    assert update_circle_stability(FakeCircle(161, 120, 70), 1) is None
    assert update_circle_stability(FakeCircle(165, 120, 70), 2) is None
    assert circle_stability_namespace["circle_sample_count"] == 2
    assert update_circle_stability(FakeCircle(160, 120, 70), 3) == (
        160,
        120,
        70,
    )

    reset_circle_stability()
    circle_stability_namespace["circle_tracking_lock_x"] = 160
    circle_stability_namespace["circle_tracking_lock_y"] = 120
    circle_stability_namespace["circle_tracking_lock_r"] = 70
    assert update_circle_stability(FakeCircle(160, 120, 70), 0) is None
    assert update_circle_stability(FakeCircle(164, 120, 70), 1) is None
    assert update_circle_stability(FakeCircle(165, 120, 70), 2) is None
    assert circle_stability_namespace["circle_sample_count"] == 2
    assert circle_stability_namespace["circle_tracking_lock_x"] == 164
    assert update_circle_stability(FakeCircle(165, 120, 70), 3) == (
        165,
        120,
        70,
    )

    reset_circle_stability()
    circle_stability_namespace["current_color_to_detect"] = 10
    circle_stability_namespace["circle_tracking_lock_x"] = 160
    circle_stability_namespace["circle_tracking_lock_y"] = 120
    circle_stability_namespace["circle_tracking_lock_r"] = 84
    assert update_circle_stability(FakeCircle(160, 120, 84), 0) is None
    note_circle_tracking_miss()
    assert circle_stability_namespace["circle_sample_count"] == 1
    assert circle_stability_namespace["circle_tracking_lock_x"] == 160
    assert circle_stability_namespace["circle_stability_gap_used"] is True
    circle_stability_namespace["circle_tracking_miss_count"] = 0
    assert update_circle_stability(FakeCircle(160, 120, 84), 2) == (
        160,
        120,
        84,
    )
    assert circle_stability_namespace["circle_stability_gap_used"] is True

    reset_circle_stability()
    circle_stability_namespace["circle_tracking_lock_x"] = 160
    circle_stability_namespace["circle_tracking_lock_y"] = 120
    circle_stability_namespace["circle_tracking_lock_r"] = 84
    note_circle_tracking_miss()
    assert circle_stability_namespace["circle_sample_count"] == 0
    assert circle_stability_namespace["circle_stability_gap_used"] is False
    assert update_circle_stability(FakeCircle(160, 120, 84), 1) is None
    assert circle_stability_namespace["circle_stability_gap_used"] is False

    left = FakeCircle(60, 121, 40)
    center = FakeCircle(160, 120, 41)
    right = FakeCircle(261, 119, 40)
    distractor = FakeCircle(154, 40, 12)
    assert (
        select_center_ring([right, distractor, left, center])
        is center
    )

    assert select_center_ring([left, center]) is None
    assert select_centered_ring_fallback([center]) is center
    assert (
        select_centered_ring_fallback(
            [FakeCircle(10, 10, 40)]
        )
        is None
    )
    assert (
        select_centered_ring_fallback(
            [
                FakeCircle(150, 120, 40),
                FakeCircle(171, 120, 40),
            ]
        )
        is None
    )
    near_center = FakeCircle(163, 122, 60)
    digit_two = FakeCircle(160, 120, 12)
    digit_three = FakeCircle(158, 121, 18)
    assert (
        select_centered_ring_fallback(
            [digit_two, near_center, digit_three]
        )
        is near_center
    )
    assert (
        select_centered_ring_fallback(
            [digit_two, digit_three]
        )
        is None
    )
    assert (
        select_center_ring(
            [
                FakeCircle(60, 120, 60),
                FakeCircle(160, 120, 61),
                FakeCircle(260, 120, 60),
                FakeCircle(60, 120, 12),
                FakeCircle(160, 120, 16),
                FakeCircle(260, 120, 18),
            ]
        )
        .r()
        == 61
    )
    assert (
        select_center_ring(
            [
                FakeCircle(40, 120, 40),
                FakeCircle(80, 120, 40),
                FakeCircle(250, 120, 40),
            ]
        )
        is None
    )

    endpoint = FakeCircle(166, 117, 42)
    digit_one = FakeCircle(160, 120, 12)
    other_ring = FakeCircle(245, 122, 41)
    assert (
        select_endpoint_ring(
            [digit_one, other_ring, endpoint, digit_three]
        )
        is endpoint
    )
    assert (
        select_endpoint_ring(
            [endpoint, FakeCircle(161, 120, 30)]
        )
        is endpoint
    )

    assert (
        select_endpoint_ring(
            [endpoint, FakeCircle(310, 230, 90)]
        )
        is endpoint
    )
    assert (
        select_endpoint_ring([FakeCircle(10, 10, 42)])
        is None
    )
    assert select_endpoint_ring([digit_one, digit_three]) is None

    request = bytearray([0xAA, 0x02, 0x37, 0x08, 0x00, 0xBB])
    request[4] = crc8(request[1:4])
    assert decode_request(request) == (0x37, 0x08)
    damaged_request = bytearray(request)
    damaged_request[4] ^= 0x01
    assert decode_request(damaged_request) is None
    known_request = bytearray(
        [0xAA, 0x02, 0x34, 0x09, 0x44, 0xBB]
    )
    assert decode_request(known_request) == (0x34, 0x09)
    endpoint_request = bytearray(
        [0xAA, 0x02, 0x35, 0x0A, 0x00, 0xBB]
    )
    endpoint_request[4] = crc8(endpoint_request[1:4])
    assert decode_request(endpoint_request) == (0x35, 0x0A)
    assert detection_mode_for_request(0x35, 10, -1) == 10
    assert detection_mode_for_request(0x35, 10, 0x35) == 10
    assert detection_mode_for_request(0x34, 9, -1) == 9
    assert detection_mode_for_request(0x34, 9, 0x34) == 9
    for target_mode in (1, 2, 3, 4):
        assert (
            detection_mode_for_request(0x40, target_mode, -1)
            == target_mode
        )
        assert detection_mode_for_request(0x40, target_mode, 0x40) == 0
    assert detection_mode_for_request(0x37, 8, -1) == 8
    assert detection_mode_for_request(0x37, 8, 0x37) == 0
    assert detection_mode_for_request(0x38, 7, -1) == 0

    response = build_response(
        0x37, 8, 0, 3, 160, 120, 2400, 867, 123456
    )
    payload, checksum_text = response.rstrip("\n").split("*")
    assert checksum_text == f"{crc8(payload.encode('ascii')):02X}"
    assert payload.startswith("V2,55,8,0,3,160,120,")
    targeted_response = build_response(
        0x40, 3, 0, 3, 168, 123, 2400, 800, 123500
    )
    targeted_payload, targeted_checksum = (
        targeted_response.rstrip("\n").split("*")
    )
    assert targeted_checksum == (
        f"{crc8(targeted_payload.encode('ascii')):02X}"
    )
    assert targeted_payload.startswith(
        "V2,64,3,0,3,168,123,2400,800,"
    )
    endpoint_response = build_response(
        0x35, 10, 0, 1, 161, 119, 42, 1000, 123956
    )
    endpoint_payload, endpoint_checksum = (
        endpoint_response.rstrip("\n").split("*")
    )
    assert endpoint_checksum == (
        f"{crc8(endpoint_payload.encode('ascii')):02X}"
    )
    assert endpoint_payload.startswith(
        "V2,53,10,0,1,161,119,42,1000,"
    )
    assert (
        select_center_ring(
            [
                FakeCircle(60, 80, 40),
                FakeCircle(160, 120, 40),
                FakeCircle(260, 160, 40),
            ]
        )
        is None
    )
    assert (
        select_center_ring(
            [
                FakeCircle(60, 120, 10),
                FakeCircle(160, 120, 30),
                FakeCircle(260, 120, 10),
            ]
        )
        is None
    )

    calibration = json.loads(
        CALIBRATION.read_text(encoding="utf-8")
    )
    runtime_constants = load_literal_constants()
    assert calibration["schema_version"] == 1
    assert (
        calibration["vision_version"]
        == "5.5.1-aggressive-latency"
    )
    assert calibration["vision_version"] == runtime_constants[
        "VISION_VERSION"
    ]
    assert calibration["calibration_version"] == runtime_constants[
        "CALIBRATION_VERSION"
    ]
    assert calibration["protocol_version"] == 2
    assert calibration["camera_runtime"] == {
        "requested_fps": -1,
        "buffer_count": 1,
        "idle_preview_interval_ms": 50,
        "idle_loop_sleep_ms": 0,
    }
    assert (
        calibration["camera_runtime"]["requested_fps"]
        == runtime_constants["camera_requested_fps"]
    )
    assert (
        calibration["camera_runtime"]["buffer_count"]
        == runtime_constants["camera_buffer_count"]
    )
    assert calibration["ring_layout"]["require_three_rings"] is True
    assert (
        calibration["ring_layout"][
            "allow_centered_ring_fallback"
        ]
        is True
    )
    assert (
        calibration["ring_layout"][
            "candidate_minimum_radius_pixels"
        ]
        == 28
    )
    assert calibration["ring_tracking"] == {
        "hough_threshold": 7500,
        "radius_upper_bounds_are_exclusive": True,
        "search_minimum_radius_pixels": 28,
        "search_x_stride_pixels": 3,
        "search_y_stride_pixels": 3,
        "search_radius_step_pixels": 3,
        "search_maximum_radius_pixels": 96,
        "fallback_after_full_search_misses": 1,
        "fallback_x_stride_pixels": 3,
        "fallback_y_stride_pixels": 2,
        "fallback_radius_step_pixels": 2,
        "fallback_maximum_radius_pixels": 111,
        "tracking_x_stride_pixels": 3,
        "tracking_y_stride_pixels": 2,
        "tracking_radius_step_pixels": 2,
        "position_margin_pixels": 8,
        "radius_margin_pixels": 3,
        "tracking_radius_lower_pixels": 2,
        "tracking_radius_upper_exclusive_pixels": 3,
        "roi_padding_pixels": 10,
        "maximum_consecutive_misses": 2,
        "preserve_stability_on_first_tracking_miss": True,
        "retain_tracking_lock_between_results": True,
        "freeze_tracking_anchor_during_stability": True,
        "stable_duration_ms": 0,
        "fast_stable_samples": 3,
        "full_stable_samples": 4,
        "fast_maximum_center_span_pixels": 1,
        "fast_maximum_radius_span_pixels": 2,
        "fast_window_confidence": 1000,
        "maximum_center_span_pixels": 3,
        "maximum_radius_span_pixels": 3,
        "active_preview_enabled": False,
        "active_preview_interval_ms": 50,
        "runtime_diagnostics_enabled": False,
        "include_search_frame_in_stability": True,
    }
    ring_runtime_mapping = {
        "hough_threshold": "circle_threshold",
        "search_minimum_radius_pixels": "circle_r_min",
        "search_x_stride_pixels": "circle_search_x_stride",
        "search_y_stride_pixels": "circle_search_y_stride",
        "search_radius_step_pixels": "circle_search_r_step",
        "search_maximum_radius_pixels": "circle_r_max",
        "fallback_after_full_search_misses": (
            "circle_fallback_after_full_search_misses"
        ),
        "fallback_x_stride_pixels": "circle_fallback_x_stride",
        "fallback_y_stride_pixels": "circle_fallback_y_stride",
        "fallback_radius_step_pixels": "circle_fallback_r_step",
        "fallback_maximum_radius_pixels": "circle_fallback_r_max",
        "tracking_x_stride_pixels": "circle_tracking_x_stride",
        "tracking_y_stride_pixels": "circle_tracking_y_stride",
        "tracking_radius_step_pixels": "circle_tracking_r_step",
        "position_margin_pixels": (
            "circle_tracking_position_margin_pixels"
        ),
        "radius_margin_pixels": (
            "circle_tracking_radius_margin_pixels"
        ),
        "tracking_radius_lower_pixels": (
            "circle_tracking_radius_lower_pixels"
        ),
        "tracking_radius_upper_exclusive_pixels": (
            "circle_tracking_radius_upper_exclusive_pixels"
        ),
        "roi_padding_pixels": "circle_tracking_roi_padding_pixels",
        "maximum_consecutive_misses": (
            "circle_tracking_maximum_misses"
        ),
        "preserve_stability_on_first_tracking_miss": (
            "preserve_stability_on_first_tracking_miss"
        ),
        "stable_duration_ms": "duration",
        "fast_stable_samples": (
            "circle_fast_stable_minimum_samples"
        ),
        "full_stable_samples": "circle_stable_minimum_samples",
        "fast_maximum_center_span_pixels": (
            "circle_fast_stable_max_span"
        ),
        "fast_maximum_radius_span_pixels": (
            "circle_fast_stable_max_radius_span"
        ),
        "fast_window_confidence": "fast_window_confidence",
        "maximum_center_span_pixels": "circle_stable_max_span",
        "maximum_radius_span_pixels": (
            "circle_stable_max_radius_span"
        ),
        "active_preview_enabled": "show_active_preview",
        "active_preview_interval_ms": "active_preview_interval_ms",
        "runtime_diagnostics_enabled": (
            "collect_circle_runtime_diagnostics"
        ),
        "include_search_frame_in_stability": (
            "include_search_frame_in_stability"
        ),
        "retain_tracking_lock_between_results": (
            "retain_tracking_lock_between_results"
        ),
        "freeze_tracking_anchor_during_stability": (
            "freeze_tracking_anchor_during_stability"
        ),
    }
    for calibration_name, runtime_name in ring_runtime_mapping.items():
        assert calibration["ring_tracking"][calibration_name] == (
            runtime_constants[runtime_name]
        )
    endpoint_runtime_mapping = {
        "maximum_center_distance_pixels": (
            "endpoint_max_center_distance_pixels"
        ),
        "minimum_dominant_radius_ratio": (
            "endpoint_min_dominant_radius_ratio"
        ),
        "search_minimum_radius_pixels": "endpoint_circle_r_min",
        "search_radius_upper_exclusive_pixels": (
            "endpoint_circle_r_max"
        ),
        "search_radius_step_pixels": "circle_search_r_step",
        "fallback_minimum_radius_pixels": "endpoint_circle_r_min",
        "fallback_radius_step_pixels": "circle_fallback_r_step",
        "fallback_radius_upper_exclusive_pixels": (
            "circle_fallback_r_max"
        ),
        "stable_duration_ms": "endpoint_stable_duration",
        "fast_stable_samples": (
            "endpoint_fast_stable_minimum_samples"
        ),
        "full_stable_samples": "endpoint_stable_minimum_samples",
        "fast_maximum_center_span_pixels": (
            "endpoint_fast_stable_max_span"
        ),
        "fast_maximum_radius_span_pixels": (
            "endpoint_fast_stable_max_radius_span"
        ),
        "fast_window_confidence": "fast_window_confidence",
        "maximum_center_span_pixels": "endpoint_stable_max_span",
        "maximum_radius_span_pixels": (
            "endpoint_stable_max_radius_span"
        ),
    }
    for calibration_name, runtime_name in endpoint_runtime_mapping.items():
        assert calibration["endpoint_scan"][calibration_name] == (
            runtime_constants[runtime_name]
        )
    assert calibration["endpoint_scan"]["mode"] == 10
    assert (
        calibration["endpoint_scan"][
            "radius_upper_bounds_are_exclusive"
        ]
        is True
    )
    assert calibration["endpoint_scan"]["target_id"] == 1
    assert (
        calibration["endpoint_scan"][
            "requires_three_ring_layout"
        ]
        is False
    )
    assert (
        calibration["endpoint_scan"]["stable_duration_ms"]
        == 0
    )
    assert (
        calibration["endpoint_scan"]["fast_stable_samples"]
        == 2
    )
    assert calibration["endpoint_scan"]["full_stable_samples"] == 3
    assert (
        calibration["endpoint_scan"][
            "fast_maximum_center_span_pixels"
        ]
        == 1
    )
    assert (
        calibration["endpoint_scan"][
            "fast_maximum_radius_span_pixels"
        ]
        == 1
    )
    assert (
        calibration["endpoint_scan"][
            "fast_accept_minimum_confidence"
        ]
        == 1000
    )
    assert (
        calibration["endpoint_scan"][
            "fast_accept_confirmations"
        ]
        == 1
    )
    assert (
        calibration["endpoint_scan"][
            "maximum_center_span_pixels"
        ]
        == 2
    )

    print(
        "PASS Vision v5 targeted-color, ring-layout, endpoint-mode, "
        "protocol-v2 CRC and calibration schema"
    )
    if not calibration["verified"]:
        print(
            "NOTICE Vision calibration is intentionally marked UNVERIFIED; "
            "real-image commissioning is still required"
        )

if __name__ == "__main__":
    main()
