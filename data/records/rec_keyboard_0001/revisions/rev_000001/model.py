from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ExtrudeGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


CASE_WIDTH = 0.362
CASE_DEPTH = 0.138
CASE_HEIGHT = 0.008
FRAME_HEIGHT = 0.009
FRAME_SIDE = 0.011
FRAME_FRONT = 0.014
FRAME_REAR = 0.014
TOP_FRAME_LOCAL_ORIGIN_Y = -0.061
OPENING_WIDTH = CASE_WIDTH - 2.0 * FRAME_SIDE
OPENING_DEPTH = CASE_DEPTH - FRAME_FRONT - FRAME_REAR
PLATE_WIDTH = 0.326
PLATE_DEPTH = 0.108
UNIT = 0.019
PRESS_TRAVEL = 0.003
KEY_PITCH_Y = 0.018
KEY_DEPTH = 0.0168
KEY_CLEARANCE = 0.002
KEY_HEIGHT = 0.0068
TOP_SKIN = 0.0008


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


CASE_GRAPHITE = _material("case_graphite", (0.18, 0.19, 0.21, 1.0))
FRAME_BLACK = _material("frame_black", (0.10, 0.11, 0.12, 1.0))
KEY_CHARCOAL = _material("key_charcoal", (0.15, 0.16, 0.17, 1.0))
KEY_TOP = _material("key_top", (0.21, 0.22, 0.24, 1.0))
ACCENT_METAL = _material("accent_metal", (0.67, 0.69, 0.72, 1.0))
LED_LENS = _material("led_lens", (0.72, 0.86, 0.78, 0.55))
RUBBER = _material("rubber", (0.07, 0.07, 0.08, 1.0))


def _rounded_key_mesh(width: float, depth: float):
    profile = rounded_rect_profile(
        width,
        depth,
        radius=min(0.0022, 0.16 * min(width, depth)),
        corner_segments=8,
    )
    geom = ExtrudeGeometry.centered(profile, height=KEY_HEIGHT, cap=True, closed=True)
    mesh_name = f"keycap_{int(round(width * 1000)):03d}x{int(round(depth * 1000)):03d}.obj"
    return mesh_from_geometry(geom, ASSETS.mesh_path(mesh_name))


def _build_key_layout() -> list[dict[str, object]]:
    rows = [
        {
            "y": 0.045,
            "z": 0.0058,
            "x_offset": 0.0,
            "items": [
                ("key_escape", 1.0, 0.0),
                ("key_f1", 1.0, 0.50),
                ("key_f2", 1.0, 0.0),
                ("key_f3", 1.0, 0.0),
                ("key_f4", 1.0, 0.0),
                ("key_f5", 1.0, 0.35),
                ("key_f6", 1.0, 0.0),
                ("key_f7", 1.0, 0.0),
                ("key_f8", 1.0, 0.0),
                ("key_f9", 1.0, 0.35),
                ("key_f10", 1.0, 0.0),
                ("key_f11", 1.0, 0.0),
                ("key_f12", 1.0, 0.0),
                ("key_delete", 1.0, 0.35),
            ],
        },
        {
            "y": 0.027,
            "z": 0.0056,
            "x_offset": 0.0,
            "items": [
                ("key_grave", 1.0, 0.0),
                ("key_1", 1.0, 0.0),
                ("key_2", 1.0, 0.0),
                ("key_3", 1.0, 0.0),
                ("key_4", 1.0, 0.0),
                ("key_5", 1.0, 0.0),
                ("key_6", 1.0, 0.0),
                ("key_7", 1.0, 0.0),
                ("key_8", 1.0, 0.0),
                ("key_9", 1.0, 0.0),
                ("key_0", 1.0, 0.0),
                ("key_minus", 1.0, 0.0),
                ("key_equals", 1.0, 0.0),
                ("key_backspace", 2.0, 0.0),
                ("key_insert", 1.0, 0.25),
            ],
        },
        {
            "y": 0.009,
            "z": 0.0054,
            "x_offset": 0.004,
            "items": [
                ("key_tab", 1.5, 0.0),
                ("key_q", 1.0, 0.0),
                ("key_w", 1.0, 0.0),
                ("key_e", 1.0, 0.0),
                ("key_r", 1.0, 0.0),
                ("key_t", 1.0, 0.0),
                ("key_y", 1.0, 0.0),
                ("key_u", 1.0, 0.0),
                ("key_i", 1.0, 0.0),
                ("key_o", 1.0, 0.0),
                ("key_p", 1.0, 0.0),
                ("key_lbracket", 1.0, 0.0),
                ("key_rbracket", 1.0, 0.0),
                ("key_backslash", 1.5, 0.0),
                ("key_pageup", 1.0, 0.25),
            ],
        },
        {
            "y": -0.009,
            "z": 0.0052,
            "x_offset": 0.007,
            "items": [
                ("key_caps", 1.75, 0.0),
                ("key_a", 1.0, 0.0),
                ("key_s", 1.0, 0.0),
                ("key_d", 1.0, 0.0),
                ("key_f", 1.0, 0.0),
                ("key_g", 1.0, 0.0),
                ("key_h", 1.0, 0.0),
                ("key_j", 1.0, 0.0),
                ("key_k", 1.0, 0.0),
                ("key_l", 1.0, 0.0),
                ("key_semicolon", 1.0, 0.0),
                ("key_apostrophe", 1.0, 0.0),
                ("key_enter", 2.25, 0.0),
                ("key_pagedown", 1.0, 0.25),
            ],
        },
        {
            "y": -0.027,
            "z": 0.0050,
            "x_offset": 0.010,
            "items": [
                ("key_lshift", 2.25, 0.0),
                ("key_z", 1.0, 0.0),
                ("key_x", 1.0, 0.0),
                ("key_c", 1.0, 0.0),
                ("key_v", 1.0, 0.0),
                ("key_b", 1.0, 0.0),
                ("key_n", 1.0, 0.0),
                ("key_m", 1.0, 0.0),
                ("key_comma", 1.0, 0.0),
                ("key_period", 1.0, 0.0),
                ("key_slash", 1.0, 0.0),
                ("key_rshift", 2.75, 0.0),
                ("key_arrow_up", 1.0, 0.25),
            ],
        },
        {
            "y": -0.045,
            "z": 0.0048,
            "x_offset": 0.0,
            "items": [
                ("key_lctrl", 1.25, 0.0),
                ("key_lmeta", 1.25, 0.0),
                ("key_lalt", 1.25, 0.0),
                ("key_space", 6.25, 0.0),
                ("key_ralt", 1.25, 0.0),
                ("key_fn", 1.0, 0.0),
                ("key_menu", 1.0, 0.0),
                ("key_rctrl", 1.25, 0.0),
                ("key_arrow_left", 1.0, 0.25),
                ("key_arrow_down", 1.0, 0.0),
                ("key_arrow_right", 1.0, 0.0),
            ],
        },
    ]
    layout: list[dict[str, object]] = []
    for row in rows:
        items = row["items"]
        total_units = sum(units + gap for _, units, gap in items)
        cursor = -0.5 * total_units * UNIT + float(row["x_offset"])
        for name, units, gap_before in items:
            cursor += gap_before * UNIT
            width = units * UNIT - KEY_CLEARANCE
            layout.append(
                {
                    "name": name,
                    "joint": f"{name}_press",
                    "units": units,
                    "x": cursor + 0.5 * units * UNIT,
                    "y": float(row["y"]),
                    "z": float(row["z"]),
                    "width": width,
                    "depth": KEY_DEPTH,
                }
            )
            cursor += units * UNIT
    return layout


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="consumer_keyboard", assets=ASSETS)

    lower_case = model.part("case_lower")
    lower_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(CASE_WIDTH, CASE_DEPTH, radius=0.017, corner_segments=12),
            height=CASE_HEIGHT,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("case_lower.obj"),
    )
    lower_case.visual(
        lower_mesh, origin=Origin(xyz=(0.0, 0.0, CASE_HEIGHT / 2.0)), material=CASE_GRAPHITE
    )
    lower_case.visual(
        Box((0.028, 0.011, 0.0012)),
        origin=Origin(xyz=(-0.120, -0.046, 0.0006)),
        material=RUBBER,
    )
    lower_case.visual(
        Box((0.028, 0.011, 0.0012)),
        origin=Origin(xyz=(0.120, -0.046, 0.0006)),
        material=RUBBER,
    )
    lower_case.visual(
        Box((0.030, 0.012, 0.0012)),
        origin=Origin(xyz=(-0.126, 0.047, 0.0006)),
        material=RUBBER,
    )
    lower_case.visual(
        Box((0.030, 0.012, 0.0012)),
        origin=Origin(xyz=(0.126, 0.047, 0.0006)),
        material=RUBBER,
    )
    lower_case.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_HEIGHT)),
        mass=0.68,
        origin=Origin(xyz=(0.0, 0.0, CASE_HEIGHT / 2.0)),
    )

    top_frame = model.part("top_frame")
    top_frame.visual(
        Box((CASE_WIDTH, FRAME_FRONT, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CASE_DEPTH - FRAME_FRONT) / 2.0 - TOP_FRAME_LOCAL_ORIGIN_Y,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material=FRAME_BLACK,
    )
    top_frame.visual(
        Box((CASE_WIDTH, FRAME_REAR, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                (0.0),
                (CASE_DEPTH - FRAME_REAR) / 2.0 - TOP_FRAME_LOCAL_ORIGIN_Y,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material=FRAME_BLACK,
    )
    top_frame.visual(
        Box((FRAME_SIDE, OPENING_DEPTH + 0.001, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(-(CASE_WIDTH - FRAME_SIDE) / 2.0, -TOP_FRAME_LOCAL_ORIGIN_Y, FRAME_HEIGHT / 2.0)
        ),
        material=FRAME_BLACK,
    )
    top_frame.visual(
        Box((FRAME_SIDE, OPENING_DEPTH + 0.001, FRAME_HEIGHT)),
        origin=Origin(
            xyz=((CASE_WIDTH - FRAME_SIDE) / 2.0, -TOP_FRAME_LOCAL_ORIGIN_Y, FRAME_HEIGHT / 2.0)
        ),
        material=FRAME_BLACK,
    )
    top_frame.visual(
        Box((0.180, 0.0045, 0.0008)),
        origin=Origin(xyz=(0.0, -0.061 - TOP_FRAME_LOCAL_ORIGIN_Y, FRAME_HEIGHT - 0.0004)),
        material=ACCENT_METAL,
    )
    top_frame.visual(
        Box((0.026, 0.005, 0.0012)),
        origin=Origin(xyz=(0.123, 0.061 - TOP_FRAME_LOCAL_ORIGIN_Y, FRAME_HEIGHT - 0.0006)),
        material=LED_LENS,
    )
    top_frame.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, FRAME_HEIGHT)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    switch_plate = model.part("switch_plate")
    switch_plate.visual(
        Box((PLATE_WIDTH, PLATE_DEPTH, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.00075)),
        material=FRAME_BLACK,
    )
    switch_plate.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH, PLATE_DEPTH, 0.0015)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.00075)),
    )

    model.articulation(
        "case_to_frame",
        ArticulationType.FIXED,
        parent="case_lower",
        child="top_frame",
        origin=Origin(xyz=(0.0, TOP_FRAME_LOCAL_ORIGIN_Y, CASE_HEIGHT)),
    )
    model.articulation(
        "case_to_plate",
        ArticulationType.FIXED,
        parent="case_lower",
        child="switch_plate",
        origin=Origin(xyz=(0.0, 0.0, CASE_HEIGHT)),
    )

    mesh_cache: dict[tuple[int, int], object] = {}
    for key in _build_key_layout():
        width = float(key["width"])
        depth = float(key["depth"])
        cache_key = (int(round(width * 10000)), int(round(depth * 10000)))
        key_mesh = mesh_cache.get(cache_key)
        if key_mesh is None:
            key_mesh = _rounded_key_mesh(width, depth)
            mesh_cache[cache_key] = key_mesh

        part = model.part(str(key["name"]))
        part.visual(
            key_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0041), rpy=(radians(-3.5), 0.0, 0.0)),
            material=KEY_CHARCOAL,
        )
        part.visual(
            Box((max(0.006, width - 0.0046), max(0.006, depth - 0.0048), TOP_SKIN)),
            origin=Origin(xyz=(0.0, 0.0004, 0.00765), rpy=(radians(-3.5), 0.0, 0.0)),
            material=KEY_TOP,
        )
        part.inertial = Inertial.from_geometry(
            Box((width, depth, KEY_HEIGHT + TOP_SKIN)),
            mass=0.008 if float(key["units"]) <= 1.25 else 0.012,
            origin=Origin(xyz=(0.0, 0.0, 0.0042)),
        )
        model.articulation(
            str(key["joint"]),
            ArticulationType.PRISMATIC,
            parent="switch_plate",
            child=str(key["name"]),
            origin=Origin(xyz=(float(key["x"]), float(key["y"]), float(key["z"]))),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.4,
                lower=0.0,
                upper=PRESS_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("top_frame", "case_lower", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("switch_plate", "case_lower", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_gap("top_frame", "case_lower", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_aabb_gap("switch_plate", "case_lower", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_aabb_overlap("key_q", "switch_plate", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_overlap("key_enter", "switch_plate", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("key_space", "switch_plate", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("key_arrow_right", "switch_plate", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_gap("key_q", "switch_plate", axis="z", max_gap=0.005, max_penetration=0.0)
    ctx.expect_aabb_gap("key_enter", "switch_plate", axis="z", max_gap=0.005, max_penetration=0.0)
    ctx.expect_aabb_gap("key_space", "switch_plate", axis="z", max_gap=0.005, max_penetration=0.0)
    ctx.expect_aabb_gap("key_arrow_right", "switch_plate", axis="z", max_gap=0.005, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "key_a_press", "key_a", world_axis="z", direction="negative", min_delta=0.0015
    )
    ctx.expect_joint_motion_axis(
        "key_enter_press", "key_enter", world_axis="z", direction="negative", min_delta=0.0015
    )
    ctx.expect_joint_motion_axis(
        "key_space_press", "key_space", world_axis="z", direction="negative", min_delta=0.0015
    )

    with ctx.pose(key_a_press=PRESS_TRAVEL):
        ctx.expect_aabb_gap("key_a", "switch_plate", axis="z", max_gap=0.0025, max_penetration=0.0)
        ctx.expect_aabb_overlap("key_a", "switch_plate", axes="xy", min_overlap=0.01)

    with ctx.pose(key_enter_press=PRESS_TRAVEL):
        ctx.expect_aabb_gap("key_enter", "switch_plate", axis="z", max_gap=0.0025, max_penetration=0.0)
        ctx.expect_aabb_overlap("key_enter", "switch_plate", axes="xy", min_overlap=0.015)

    with ctx.pose(key_space_press=PRESS_TRAVEL):
        ctx.expect_aabb_gap("key_space", "switch_plate", axis="z", max_gap=0.0025, max_penetration=0.0)
        ctx.expect_aabb_overlap("key_space", "switch_plate", axes="xy", min_overlap=0.015)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
