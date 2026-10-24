"""Host checks for yan_2 mode 1; Maix camera segmentation still needs device QA."""

import ast
import contextlib
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[1] / "yan_2" / "main.py"


def load_logic():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    nodes = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            nodes.append(node)
        elif isinstance(node, ast.Assign):
            try:
                ast.literal_eval(node.value)
            except (ValueError, TypeError):
                continue
            nodes.append(node)
    namespace = {"time": SimpleNamespace(ticks_ms=lambda: 1000)}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), namespace)
    namespace["MODE1_RED_SHAPE_DIAGNOSTICS"] = False
    return namespace


class MaskBlob:
    """Blob adapter using raster pixel/perimeter counts and known rotated axes."""

    def __init__(self, points, axes, angle=0, center=(160, 120)):
        self.points = points
        self.center = center
        self.axes = axes
        self.angle = math.radians(angle)
        self.perimeter_pixels = sum(
            (x + dx, y + dy) not in points
            for x, y in points
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
        )
        self.width = max(x for x, _ in points) - min(x for x, _ in points) + 1
        self.height = max(y for _, y in points) - min(y for _, y in points) + 1

    @classmethod
    def ellipse(cls, a=35, b=35, angle=0, distortion=0, center=(160, 120)):
        points = set()
        theta = math.radians(angle)
        for y in range(-65, 66):
            for x in range(-65, 66):
                u = (x * math.cos(theta) + y * math.sin(theta)) / a
                v = (-x * math.sin(theta) + y * math.cos(theta)) / b
                radial_limit = 1 + distortion * math.cos(5 * math.atan2(v, u))
                if u * u + v * v <= radial_limit * radial_limit:
                    points.add((x, y))
        return cls(points, (2 * a, 2 * b), angle, center)

    @classmethod
    def line(cls, angle=45):
        theta = math.radians(angle)
        points = set()
        for y in range(-115, 116):
            for x in range(-115, 116):
                along = x * math.cos(theta) + y * math.sin(theta)
                across = -x * math.sin(theta) + y * math.cos(theta)
                if abs(along) <= 105 and abs(across) <= 5:
                    points.add((x, y))
        return cls(points, (210, 10), angle, center=(100, 100))

    def mini_corners(self):
        a, b = self.axes[0] / 2, self.axes[1] / 2
        return [
            [x * math.cos(self.angle) - y * math.sin(self.angle),
             x * math.sin(self.angle) + y * math.cos(self.angle)]
            for x, y in ((-a, -b), (a, -b), (a, b), (-a, b))
        ]

    def pixels(self):
        return len(self.points)

    def perimeter(self):
        return self.perimeter_pixels

    def w(self):
        return self.width

    def h(self):
        return self.height

    def cx(self):
        return self.center[0]

    def cy(self):
        return self.center[1]


def run_camera_loop(frames, logic=1, mode=1):
    """Execute the complete script with mocked Maix I/O, including UART bytes."""
    model = load_logic()
    request = bytearray([0xAA, 2, 7, mode])
    request.extend([model["crc8"](request[1:]), 0xBB])
    state = SimpleNamespace(frame_index=0, now=1000, sent=[])

    class Serial:
        def available(self):
            return len(request)

        def read(self, length):
            value = bytes(request[:length])
            del request[:length]
            return value

        def write_str(self, line):
            state.sent.append((state.frame_index, line))

    class Frame:
        def __init__(self, blobs):
            self.blobs = blobs

        def find_blobs(self, thresholds, **kwargs):
            assert kwargs == {"area_threshold": 1500, "pixels_threshold": 1000}
            return self.blobs if thresholds == model["red_thresholds"] else []

        def draw_circle(self, *args, **kwargs):
            pass

    def read_frame():
        frame = Frame(frames[state.frame_index])
        state.frame_index += 1
        return frame

    def advance(_):
        state.now += 50

    fake_maix = SimpleNamespace(
        app=SimpleNamespace(need_exit=lambda: state.frame_index >= len(frames)),
        uart=SimpleNamespace(UART=lambda *args: Serial()),
        time=SimpleNamespace(ticks_ms=lambda: state.now, sleep_ms=advance),
        pinmap=SimpleNamespace(set_pin_function=lambda *args: None),
        camera=SimpleNamespace(Camera=lambda *args: SimpleNamespace(read=read_frame)),
        display=SimpleNamespace(Display=lambda: SimpleNamespace(show=lambda _: None)),
        image=SimpleNamespace(COLOR_RED=1, COLOR_GREEN=2, COLOR_BLUE=3),
        gpio=SimpleNamespace(
            Mode=SimpleNamespace(OUT=1),
            GPIO=lambda *args: SimpleNamespace(value=lambda _: None),
        ),
    )
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    for node in tree.body:
        if (isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == "MODE1_RED_DETECTION_LOGIC"):
            node.value = ast.copy_location(ast.Constant(logic), node.value)
    output = io.StringIO()
    with patch.dict(sys.modules, {"maix": fake_maix}), contextlib.redirect_stdout(output):
        exec(compile(tree, str(SOURCE), "exec"), {})
    return state.sent, output.getvalue()


class Mode1RedShapeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.circle = MaskBlob.ellipse(center=(170, 125))
        cls.diagonal = MaskBlob.line()

    def test_circle_ellipses_and_mildly_deformed_shapes(self):
        model = load_logic()
        for shape in (self.circle, MaskBlob.ellipse(42, 24),
                      MaskBlob.ellipse(42, 24, angle=45),
                      MaskBlob.ellipse(35, 31, distortion=0.08)):
            with self.subTest(axes=shape.axes, angle=shape.angle):
                self.assertIs(model["select_color_blob"](1, 0, [shape]), shape)

    def test_lines_including_diagonal_and_very_flat_ellipse_rejected(self):
        model = load_logic()
        for shape in (MaskBlob.line(0), self.diagonal, MaskBlob.line(90),
                      MaskBlob.ellipse(60, 20)):
            self.assertIsNone(model["select_color_blob"](1, 0, [shape]))

    def test_crossed_lines_rejected_even_with_similar_width_and_height(self):
        points = {(x, y) for x in range(-60, 61) for y in range(-60, 61)
                  if abs(x) <= 4 or abs(y) <= 4}
        cross = MaskBlob(points, (120, 120))
        self.assertIsNone(load_logic()["select_color_blob"](1, 0, [cross]))

    def test_corner_order_does_not_change_features(self):
        model = load_logic()
        shape = MaskBlob.ellipse(42, 24, angle=45)
        expected = model["mode1_red_shape_features"](shape)
        corners = shape.mini_corners()
        shape.mini_corners = lambda: [corners[i] for i in (2, 0, 3, 1)]
        self.assertEqual(model["mode1_red_shape_features"](shape), expected)

    def test_legacy_switch_and_other_modes_retain_original_selection(self):
        model = load_logic()
        blobs = [self.circle, self.diagonal]
        for mode, color in ((8, 0), (2, 1), (3, 2), (4, 3)):
            self.assertIs(model["select_color_blob"](mode, color, blobs), self.diagonal)
        model["MODE1_RED_DETECTION_LOGIC"] = 0
        self.assertIs(model["select_color_blob"](1, 0, blobs), self.diagonal)

    def test_full_loop_selects_small_circle_and_reports_its_metric(self):
        sent, _ = run_camera_loop([[self.circle, self.diagonal]] * 4)
        self.assertEqual(len(sent), 1)
        fields, checksum = sent[0][1].strip().split("*")
        self.assertEqual(fields.split(",")[:7], ["V2", "7", "1", "0", "1", "170", "125"])
        self.assertEqual(int(fields.split(",")[7]), self.circle.w() * self.circle.h())
        self.assertEqual(int(checksum, 16), load_logic()["crc8"](fields.encode("ascii")))

    def test_line_only_sends_no_success_and_logs_why(self):
        sent, output = run_camera_loop([[self.diagonal]] * 4)
        self.assertEqual(sent, [])
        self.assertIn("reject-elongated", output)

    def test_rejected_frame_resets_stability(self):
        sent, _ = run_camera_loop([[self.circle], [self.diagonal], [self.circle], [self.circle]])
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0][0], 4)

    def test_complete_loop_legacy_and_mode8_keep_original_target(self):
        for logic, mode, count in ((0, 1, 4), (1, 8, 10)):
            sent, _ = run_camera_loop([[self.circle, self.diagonal]] * count, logic, mode)
            self.assertEqual(len(sent), 1)
            self.assertEqual(sent[0][1].split(",")[5:7], ["100", "100"])

    def test_missing_shape_api_is_diagnostic_and_does_not_fall_back(self):
        broken = MaskBlob.ellipse()
        broken.mini_corners = None
        sent, output = run_camera_loop([[broken]] * 3)
        self.assertEqual(sent, [])
        self.assertIn("reject-shape-api", output)


if __name__ == "__main__":
    unittest.main(verbosity=2)
