from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from typing import Any

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


KEY_PITCH = 0.01905
KEY_GAP = 0.0032
KEY_TRAVEL = 0.004
STEM_WIDTH = 0.0112
STEM_DEPTH = 0.0112
STEM_HEIGHT = 0.014

TOP_PLATE_THICKNESS = 0.002
TOP_PLATE_CENTER_Z = 0.014
TOP_PLATE_TOP_Z = TOP_PLATE_CENTER_Z + (TOP_PLATE_THICKNESS * 0.5)
GUIDE_THICKNESS = 0.008
GUIDE_CENTER_Z = 0.009
BOTTOM_THICKNESS = 0.003
WALL_HEIGHT = TOP_PLATE_CENTER_Z - (TOP_PLATE_THICKNESS * 0.5)

FOOT_PIN_RADIUS = 0.004
FOOT_BARREL_OUTER_RADIUS = 0.006
FOOT_BARREL_LENGTH = 0.020
FOOT_PIN_LENGTH = 0.016
FOOT_WIDTH = 0.020
FOOT_LENGTH = 0.028
FOOT_THICKNESS = 0.003
FOOT_DEPLOY_ANGLE = 1.05
FOOT_BRACKET_THICKNESS = 0.003
FOOT_BRACKET_DEPTH = 0.018
FOOT_BRACKET_HEIGHT = 0.016
FOOT_BRACKET_RIB_HEIGHT = 0.006


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _row_items() -> tuple[dict[str, Any], ...]:
    return (
        {
            "name": "function",
            "y": 0.054,
            "clearance": 0.0050,
            "cap_height": 0.0048,
            "depth": 0.0145,
            "start": 2.375,
            "items": (
                ("esc", 1.0),
                (None, 0.75),
                ("f1", 1.0),
                ("f2", 1.0),
                ("f3", 1.0),
                ("f4", 1.0),
                (None, 0.5),
                ("f5", 1.0),
                ("f6", 1.0),
                ("f7", 1.0),
                ("f8", 1.0),
                (None, 0.5),
                ("f9", 1.0),
                ("f10", 1.0),
                ("f11", 1.0),
                ("f12", 1.0),
                (None, 0.5),
                ("print_screen", 1.0),
                ("scroll_lock", 1.0),
                ("pause", 1.0),
            ),
        },
        {
            "name": "number",
            "y": 0.026,
            "clearance": 0.0062,
            "cap_height": 0.0056,
            "depth": 0.0160,
            "start": 0.0,
            "items": (
                ("grave", 1.0),
                ("digit_1", 1.0),
                ("digit_2", 1.0),
                ("digit_3", 1.0),
                ("digit_4", 1.0),
                ("digit_5", 1.0),
                ("digit_6", 1.0),
                ("digit_7", 1.0),
                ("digit_8", 1.0),
                ("digit_9", 1.0),
                ("digit_0", 1.0),
                ("minus", 1.0),
                ("equals", 1.0),
                ("backspace", 2.0),
                (None, 0.5),
                ("insert", 1.0),
                ("home", 1.0),
                ("page_up", 1.0),
                (None, 0.5),
                ("num_lock", 1.0),
                ("num_divide", 1.0),
                ("num_multiply", 1.0),
                ("num_minus", 1.0),
            ),
        },
        {
            "name": "qwerty",
            "y": 0.007,
            "clearance": 0.0071,
            "cap_height": 0.0060,
            "depth": 0.0160,
            "start": 0.0,
            "items": (
                ("tab", 1.5),
                ("q", 1.0),
                ("w", 1.0),
                ("e", 1.0),
                ("r", 1.0),
                ("t", 1.0),
                ("y", 1.0),
                ("u", 1.0),
                ("i", 1.0),
                ("o", 1.0),
                ("p", 1.0),
                ("l_bracket", 1.0),
                ("r_bracket", 1.0),
                ("backslash", 1.5),
                (None, 0.5),
                ("delete", 1.0),
                ("end", 1.0),
                ("page_down", 1.0),
                (None, 0.5),
                ("num_7", 1.0),
                ("num_8", 1.0),
                ("num_9", 1.0),
                ("num_plus", 1.0),
            ),
        },
        {
            "name": "home",
            "y": -0.012,
            "clearance": 0.0080,
            "cap_height": 0.0060,
            "depth": 0.0160,
            "start": 0.0,
            "items": (
                ("caps_lock", 1.75),
                ("a", 1.0),
                ("s", 1.0),
                ("d", 1.0),
                ("f", 1.0),
                ("g", 1.0),
                ("h", 1.0),
                ("j", 1.0),
                ("k", 1.0),
                ("l", 1.0),
                ("semicolon", 1.0),
                ("apostrophe", 1.0),
                ("enter", 2.25),
                (None, 4.0),
                ("num_4", 1.0),
                ("num_5", 1.0),
                ("num_6", 1.0),
            ),
        },
        {
            "name": "bottom",
            "y": -0.031,
            "clearance": 0.0074,
            "cap_height": 0.0060,
            "depth": 0.0160,
            "start": 0.0,
            "items": (
                ("left_shift", 2.25),
                ("z", 1.0),
                ("x", 1.0),
                ("c", 1.0),
                ("v", 1.0),
                ("b", 1.0),
                ("n", 1.0),
                ("m", 1.0),
                ("comma", 1.0),
                ("period", 1.0),
                ("slash", 1.0),
                ("right_shift", 2.75),
                (None, 1.5),
                ("arrow_up", 1.0),
                (None, 1.5),
                ("num_1", 1.0),
                ("num_2", 1.0),
                ("num_3", 1.0),
                ("num_enter", 1.0),
            ),
        },
        {
            "name": "space",
            "y": -0.053,
            "clearance": 0.0063,
            "cap_height": 0.0058,
            "depth": 0.0160,
            "start": 0.0,
            "items": (
                ("left_ctrl", 1.25),
                ("left_meta", 1.25),
                ("left_alt", 1.25),
                ("space", 6.25),
                ("right_alt", 1.25),
                ("menu", 1.25),
                ("right_ctrl", 1.25),
                (None, 0.5),
                ("arrow_left", 1.0),
                ("arrow_down", 1.0),
                ("arrow_right", 1.0),
                (None, 0.5),
                ("num_0", 2.0),
                ("num_dot", 1.0),
            ),
        },
    )


def _build_key_layout() -> list[dict[str, Any]]:
    rows = _row_items()
    max_units = max(
        row["start"] + sum(units for _, units in row["items"])
        for row in rows
    )
    left_x = -0.5 * max_units * KEY_PITCH

    layout: list[dict[str, Any]] = []
    for row in rows:
        cursor = row["start"]
        for key_id, units in row["items"]:
            if key_id is None:
                cursor += units
                continue
            width = (units * KEY_PITCH) - KEY_GAP
            layout.append(
                {
                    "id": key_id,
                    "row": row["name"],
                    "units": units,
                    "x": left_x + ((cursor + (units * 0.5)) * KEY_PITCH),
                    "y": row["y"],
                    "width": width,
                    "depth": row["depth"],
                    "cap_clearance": row["clearance"],
                    "cap_height": row["cap_height"],
                }
            )
            cursor += units

    min_x = min(item["x"] - (item["width"] * 0.5) for item in layout)
    max_x = max(item["x"] + (item["width"] * 0.5) for item in layout)
    x_center = 0.5 * (min_x + max_x)
    for item in layout:
        item["x"] -= x_center
    return layout


def _layout_bounds(layout: list[dict[str, Any]]) -> dict[str, float]:
    min_x = min(item["x"] - (item["width"] * 0.5) for item in layout)
    max_x = max(item["x"] + (item["width"] * 0.5) for item in layout)
    min_y = min(item["y"] - (item["depth"] * 0.5) for item in layout)
    max_y = max(item["y"] + (item["depth"] * 0.5) for item in layout)
    return {
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
        "width": max_x - min_x,
        "depth": max_y - min_y,
    }


def _key_part_name(key_id: str) -> str:
    return f"key_{key_id}"


def _key_joint_name(key_id: str) -> str:
    return f"housing_to_key_{key_id}"


def _build_foot_barrel_mesh(name: str):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (FOOT_BARREL_OUTER_RADIUS, -0.5 * FOOT_BARREL_LENGTH),
                (FOOT_BARREL_OUTER_RADIUS, 0.5 * FOOT_BARREL_LENGTH),
            ],
            [
                (FOOT_PIN_RADIUS, -0.5 * FOOT_BARREL_LENGTH),
                (FOOT_PIN_RADIUS, 0.5 * FOOT_BARREL_LENGTH),
            ],
            segments=32,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi * 0.5),
        name,
    )


KEY_LAYOUT = _build_key_layout()
KEY_IDS = tuple(item["id"] for item in KEY_LAYOUT)
KEY_BOUNDS = _layout_bounds(KEY_LAYOUT)

HOUSING_WIDTH = max(0.448, KEY_BOUNDS["width"] + 0.024)
HOUSING_DEPTH = max(0.150, KEY_BOUNDS["depth"] + 0.030)
GUIDE_WIDTH = min(HOUSING_WIDTH - 0.016, KEY_BOUNDS["width"] + 0.010)
GUIDE_DEPTH = min(HOUSING_DEPTH - 0.018, KEY_BOUNDS["depth"] + 0.012)
HOUSING_HALF_WIDTH = HOUSING_WIDTH * 0.5
HOUSING_HALF_DEPTH = HOUSING_DEPTH * 0.5
REAR_WALL_THICKNESS = 0.008

REAR_POCKET_DEPTH = 0.040
FRONT_BOTTOM_DEPTH = HOUSING_DEPTH - REAR_POCKET_DEPTH
BOTTOM_FRONT_CENTER_Y = -HOUSING_HALF_DEPTH + (FRONT_BOTTOM_DEPTH * 0.5)
BOTTOM_REAR_CENTER_Y = HOUSING_HALF_DEPTH - (REAR_POCKET_DEPTH * 0.5)

FOOT_X_OFFSET = min(0.145, HOUSING_WIDTH * 0.32)
FOOT_POCKET_WIDTH = 0.040
FOOT_HINGE_Y = HOUSING_HALF_DEPTH + 0.002
FOOT_HINGE_Z = -0.001
REAR_FOOT_SLOT_WIDTH = FOOT_BARREL_LENGTH + 0.016

LEFT_OUTER_BAND_WIDTH = HOUSING_HALF_WIDTH - (FOOT_X_OFFSET + (FOOT_POCKET_WIDTH * 0.5))
CENTER_BAND_WIDTH = 2.0 * (FOOT_X_OFFSET - (FOOT_POCKET_WIDTH * 0.5))
REAR_CORNER_WALL_WIDTH = HOUSING_HALF_WIDTH - (FOOT_X_OFFSET + (REAR_FOOT_SLOT_WIDTH * 0.5))
REAR_CENTER_WALL_WIDTH = (2.0 * FOOT_X_OFFSET) - REAR_FOOT_SLOT_WIDTH
REPRESENTATIVE_KEYS = ("f5", "digit_5", "g", "n", "space", "num_5")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_keyboard")

    housing_dark = model.material("housing_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    housing_trim = model.material("housing_trim", rgba=(0.25, 0.27, 0.30, 1.0))
    guide_dark = model.material("guide_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    key_charcoal = model.material("key_charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    stem_black = model.material("stem_black", rgba=(0.05, 0.05, 0.06, 1.0))
    foot_black = model.material("foot_black", rgba=(0.10, 0.10, 0.11, 1.0))
    indicator = model.material("indicator", rgba=(0.25, 0.55, 0.28, 0.85))

    housing = model.part("housing")

    hole_profiles = [_rect_profile(STEM_WIDTH, STEM_DEPTH) for _ in KEY_LAYOUT]
    for profile, spec in zip(hole_profiles, KEY_LAYOUT):
        for point_index, (px, py) in enumerate(profile):
            profile[point_index] = (px + spec["x"], py + spec["y"])

    top_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(HOUSING_WIDTH, HOUSING_DEPTH, 0.018, corner_segments=8),
            hole_profiles,
            height=TOP_PLATE_THICKNESS,
            center=True,
        ),
        "office_keyboard_top_plate",
    )
    guide_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(GUIDE_WIDTH, GUIDE_DEPTH, 0.010, corner_segments=6),
            hole_profiles,
            height=GUIDE_THICKNESS,
            center=True,
        ),
        "office_keyboard_guide_plate",
    )
    housing.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_CENTER_Z)),
        material=housing_dark,
        name="top_plate",
    )
    housing.visual(
        guide_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_CENTER_Z)),
        material=guide_dark,
        name="guide_plate",
    )
    housing.visual(
        Box((HOUSING_WIDTH, 0.012, WALL_HEIGHT)),
        origin=Origin(xyz=(0.0, -HOUSING_HALF_DEPTH + 0.006, WALL_HEIGHT * 0.5)),
        material=housing_dark,
        name="front_wall",
    )
    if REAR_CORNER_WALL_WIDTH > 0.01:
        housing.visual(
            Box((REAR_CORNER_WALL_WIDTH, REAR_WALL_THICKNESS, WALL_HEIGHT)),
            origin=Origin(
                xyz=(
                    -HOUSING_HALF_WIDTH + (REAR_CORNER_WALL_WIDTH * 0.5),
                    HOUSING_HALF_DEPTH - (REAR_WALL_THICKNESS * 0.5),
                    WALL_HEIGHT * 0.5,
                )
            ),
            material=housing_dark,
            name="rear_left_wall",
        )
        housing.visual(
            Box((REAR_CORNER_WALL_WIDTH, REAR_WALL_THICKNESS, WALL_HEIGHT)),
            origin=Origin(
                xyz=(
                    HOUSING_HALF_WIDTH - (REAR_CORNER_WALL_WIDTH * 0.5),
                    HOUSING_HALF_DEPTH - (REAR_WALL_THICKNESS * 0.5),
                    WALL_HEIGHT * 0.5,
                )
            ),
            material=housing_dark,
            name="rear_right_wall",
        )
    if REAR_CENTER_WALL_WIDTH > 0.01:
        housing.visual(
            Box((REAR_CENTER_WALL_WIDTH, REAR_WALL_THICKNESS, WALL_HEIGHT)),
            origin=Origin(
                xyz=(0.0, HOUSING_HALF_DEPTH - (REAR_WALL_THICKNESS * 0.5), WALL_HEIGHT * 0.5)
            ),
            material=housing_dark,
            name="rear_center_wall",
        )
    foot_rib_front_y = HOUSING_HALF_DEPTH - REAR_POCKET_DEPTH
    foot_rib_length = FOOT_HINGE_Y - foot_rib_front_y + (FOOT_BRACKET_DEPTH * 0.5)
    foot_rib_center_y = foot_rib_front_y + (foot_rib_length * 0.5)
    cheek_offset = (FOOT_BARREL_LENGTH * 0.5) + (FOOT_BRACKET_THICKNESS * 0.5)
    for side_name, foot_x in (("left", -FOOT_X_OFFSET), ("right", FOOT_X_OFFSET)):
        for cheek_name, cheek_sign in (("inboard", -1.0), ("outboard", 1.0)):
            cheek_x = foot_x + (cheek_sign * cheek_offset)
            housing.visual(
                Box((FOOT_BRACKET_THICKNESS, FOOT_BRACKET_DEPTH, FOOT_BRACKET_HEIGHT)),
                origin=Origin(xyz=(cheek_x, FOOT_HINGE_Y, FOOT_HINGE_Z)),
                material=housing_trim,
                name=f"{side_name}_foot_{cheek_name}_cheek",
            )
            housing.visual(
                Box((FOOT_BRACKET_THICKNESS, foot_rib_length, FOOT_BRACKET_RIB_HEIGHT)),
                origin=Origin(
                    xyz=(cheek_x, foot_rib_center_y, FOOT_HINGE_Z - 0.002)
                ),
                material=housing_trim,
                name=f"{side_name}_foot_{cheek_name}_rib",
            )
    housing.visual(
        Box((0.010, HOUSING_DEPTH - 0.020, WALL_HEIGHT)),
        origin=Origin(xyz=(-HOUSING_HALF_WIDTH + 0.005, 0.0, WALL_HEIGHT * 0.5)),
        material=housing_dark,
        name="left_wall",
    )
    housing.visual(
        Box((0.010, HOUSING_DEPTH - 0.020, WALL_HEIGHT)),
        origin=Origin(xyz=(HOUSING_HALF_WIDTH - 0.005, 0.0, WALL_HEIGHT * 0.5)),
        material=housing_dark,
        name="right_wall",
    )
    housing.visual(
        Box((HOUSING_WIDTH, FRONT_BOTTOM_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, BOTTOM_FRONT_CENTER_Y, BOTTOM_THICKNESS * 0.5)),
        material=housing_trim,
        name="bottom_front_panel",
    )
    if LEFT_OUTER_BAND_WIDTH > 0.01:
        housing.visual(
            Box((LEFT_OUTER_BAND_WIDTH, REAR_POCKET_DEPTH, BOTTOM_THICKNESS)),
            origin=Origin(
                xyz=(
                    -HOUSING_HALF_WIDTH + (LEFT_OUTER_BAND_WIDTH * 0.5),
                    BOTTOM_REAR_CENTER_Y,
                    BOTTOM_THICKNESS * 0.5,
                )
            ),
            material=housing_trim,
            name="rear_left_panel",
        )
        housing.visual(
            Box((LEFT_OUTER_BAND_WIDTH, REAR_POCKET_DEPTH, BOTTOM_THICKNESS)),
            origin=Origin(
                xyz=(
                    HOUSING_HALF_WIDTH - (LEFT_OUTER_BAND_WIDTH * 0.5),
                    BOTTOM_REAR_CENTER_Y,
                    BOTTOM_THICKNESS * 0.5,
                )
            ),
            material=housing_trim,
            name="rear_right_panel",
        )
    if CENTER_BAND_WIDTH > 0.01:
        housing.visual(
            Box((CENTER_BAND_WIDTH, REAR_POCKET_DEPTH, BOTTOM_THICKNESS)),
            origin=Origin(xyz=(0.0, BOTTOM_REAR_CENTER_Y, BOTTOM_THICKNESS * 0.5)),
            material=housing_trim,
            name="rear_center_panel",
        )
    housing.visual(
        Box((HOUSING_WIDTH, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, HOUSING_HALF_DEPTH - 0.008, 0.019)),
        material=housing_trim,
        name="rear_ridge",
    )
    housing.visual(
        Box((0.032, 0.004, 0.002)),
        origin=Origin(xyz=(HOUSING_HALF_WIDTH - 0.040, HOUSING_HALF_DEPTH - 0.014, 0.016)),
        material=indicator,
        name="indicator_strip",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, 0.028)),
        mass=0.92,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    for spec in KEY_LAYOUT:
        key_id = spec["id"]
        key_part = model.part(_key_part_name(key_id))
        key_part.visual(
            Box((spec["width"], spec["depth"], spec["cap_height"])),
            origin=Origin(
                xyz=(0.0, 0.0, spec["cap_clearance"] + (spec["cap_height"] * 0.5))
            ),
            material=key_charcoal,
            name="cap",
        )
        key_part.visual(
            Box((STEM_WIDTH, STEM_DEPTH, STEM_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, spec["cap_clearance"] - (STEM_HEIGHT * 0.5))),
            material=stem_black,
            name="stem",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((spec["width"], spec["depth"], spec["cap_clearance"] + spec["cap_height"])),
            mass=0.0035 + (0.0012 * spec["units"]),
            origin=Origin(
                xyz=(0.0, 0.0, (spec["cap_clearance"] + spec["cap_height"]) * 0.5)
            ),
        )
        model.articulation(
            _key_joint_name(key_id),
            ArticulationType.PRISMATIC,
            parent=housing,
            child=key_part,
            origin=Origin(xyz=(spec["x"], spec["y"], TOP_PLATE_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=0.08,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    for foot_name, foot_x in (
        ("left_rear_foot", -FOOT_X_OFFSET),
        ("right_rear_foot", FOOT_X_OFFSET),
    ):
        foot = model.part(foot_name)
        foot.visual(
            Cylinder(radius=FOOT_BARREL_OUTER_RADIUS, length=FOOT_BARREL_LENGTH),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=foot_black,
            name="hinge_barrel",
        )
        foot.visual(
            Box((FOOT_WIDTH, FOOT_LENGTH, FOOT_THICKNESS)),
            origin=Origin(xyz=(0.0, -0.017, -0.005)),
            material=foot_black,
            name="foot_blade",
        )
        foot.visual(
            Box((FOOT_WIDTH, 0.008, FOOT_THICKNESS)),
            origin=Origin(xyz=(0.0, -0.031, -0.005)),
            material=foot_black,
            name="foot_toe",
        )
        foot.inertial = Inertial.from_geometry(
            Box((FOOT_WIDTH, 0.040, 0.014)),
            mass=0.018,
            origin=Origin(xyz=(0.0, -0.018, -0.004)),
        )
        model.articulation(
            f"housing_to_{foot_name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=foot,
            origin=Origin(xyz=(foot_x, FOOT_HINGE_Y, FOOT_HINGE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.0,
                lower=0.0,
                upper=FOOT_DEPLOY_ANGLE,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    left_foot = object_model.get_part("left_rear_foot")
    right_foot = object_model.get_part("right_rear_foot")
    left_foot_joint = object_model.get_articulation("housing_to_left_rear_foot")
    right_foot_joint = object_model.get_articulation("housing_to_right_rear_foot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    for key_id in KEY_IDS:
        key_part = object_model.get_part(_key_part_name(key_id))
        ctx.allow_overlap(
            housing,
            key_part,
            elem_a="guide_plate",
            elem_b="stem",
            reason=(
                "The hidden guide plate represents the enclosed switch-guide layer; "
                "the stem visual intentionally telescopes through that internal guide "
                "as a simplified plunger model."
            ),
        )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    housing_aabb = ctx.part_world_aabb(housing)
    if housing_aabb is not None:
        width = housing_aabb[1][0] - housing_aabb[0][0]
        depth = housing_aabb[1][1] - housing_aabb[0][1]
        height = housing_aabb[1][2] - housing_aabb[0][2]
        ctx.check(
            "keyboard_full_size_width",
            0.42 <= width <= 0.48,
            f"expected full-size keyboard width, got {width:.3f} m",
        )
        ctx.check(
            "keyboard_full_size_depth",
            0.13 <= depth <= 0.18,
            f"expected office keyboard depth, got {depth:.3f} m",
        )
        ctx.check(
            "keyboard_plausible_height",
            0.02 <= height <= 0.05,
            f"expected realistic keyboard height, got {height:.3f} m",
        )
    else:
        ctx.fail("housing_aabb_available", "housing AABB was not available")

    missing_parts: list[str] = []
    bad_key_joints: list[str] = []
    key_joint_objects: dict[Any, float] = {}
    for key_id in KEY_IDS:
        part_name = _key_part_name(key_id)
        joint_name = _key_joint_name(key_id)
        try:
            key_part = object_model.get_part(part_name)
            key_joint = object_model.get_articulation(joint_name)
        except Exception:
            missing_parts.append(key_id)
            continue

        key_joint_objects[key_joint] = KEY_TRAVEL
        limits = key_joint.motion_limits
        if (
            key_joint.articulation_type != ArticulationType.PRISMATIC
            or tuple(key_joint.axis) != (0.0, 0.0, -1.0)
            or limits is None
            or limits.lower != 0.0
            or limits.upper != KEY_TRAVEL
        ):
            bad_key_joints.append(joint_name)

        ctx.expect_within(
            key_part,
            housing,
            axes="xy",
            margin=0.001,
            name=f"{key_id}_within_housing",
        )
        ctx.expect_contact(
            key_part,
            housing,
            contact_tol=0.0005,
            name=f"{key_id}_guided_contact",
        )

    ctx.check(
        "all_keys_present",
        not missing_parts,
        "missing key parts or joints: " + ", ".join(missing_parts),
    )
    ctx.check(
        "all_key_joints_prismatic",
        not bad_key_joints,
        "unexpected key articulation setup: " + ", ".join(bad_key_joints),
    )

    left_limits = left_foot_joint.motion_limits
    right_limits = right_foot_joint.motion_limits
    ctx.check(
        "rear_feet_joint_setup",
        (
            left_foot_joint.articulation_type == ArticulationType.REVOLUTE
            and right_foot_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(left_foot_joint.axis) == (1.0, 0.0, 0.0)
            and tuple(right_foot_joint.axis) == (1.0, 0.0, 0.0)
            and left_limits is not None
            and right_limits is not None
            and left_limits.lower == 0.0
            and right_limits.lower == 0.0
            and left_limits.upper == FOOT_DEPLOY_ANGLE
            and right_limits.upper == FOOT_DEPLOY_ANGLE
        ),
        "rear feet should hinge about the keyboard width axis with bounded deployment",
    )

    ctx.expect_contact(
        left_foot,
        housing,
        contact_tol=0.0005,
        name="left_foot_stowed_contact",
    )
    ctx.expect_contact(
        right_foot,
        housing,
        contact_tol=0.0005,
        name="right_foot_stowed_contact",
    )
    ctx.expect_overlap(
        left_foot,
        housing,
        axes="xy",
        min_overlap=0.010,
        name="left_foot_under_rear_edge",
    )
    ctx.expect_overlap(
        right_foot,
        housing,
        axes="xy",
        min_overlap=0.010,
        name="right_foot_under_rear_edge",
    )

    typing_key = object_model.get_part(_key_part_name("j"))
    numpad_key = object_model.get_part(_key_part_name("num_5"))
    typing_pos = ctx.part_world_position(typing_key)
    numpad_pos = ctx.part_world_position(numpad_key)
    if typing_pos is not None and numpad_pos is not None:
        ctx.check(
            "numpad_cluster_separated",
            (numpad_pos[0] - typing_pos[0]) > 0.12,
            "numeric keypad should sit clearly to the right of the typing block",
        )
    else:
        ctx.fail("numpad_cluster_separated", "could not resolve key positions")

    f5_aabb = ctx.part_world_aabb(object_model.get_part(_key_part_name("f5")))
    g_aabb = ctx.part_world_aabb(object_model.get_part(_key_part_name("g")))
    space_aabb = ctx.part_world_aabb(object_model.get_part(_key_part_name("space")))
    if f5_aabb is not None and g_aabb is not None and space_aabb is not None:
        ctx.check(
            "rows_are_stepped",
            g_aabb[1][2] > f5_aabb[1][2] + 0.002 and g_aabb[1][2] > space_aabb[1][2] + 0.001,
            "home-row keys should sit higher than function and bottom rows",
        )
    else:
        ctx.fail("rows_are_stepped", "could not measure row heights")

    for key_id in REPRESENTATIVE_KEYS:
        joint = object_model.get_articulation(_key_joint_name(key_id))
        key_part = object_model.get_part(_key_part_name(key_id))
        rest_aabb = ctx.part_world_aabb(key_part)
        if rest_aabb is None:
            ctx.fail(f"{key_id}_rest_aabb_available", "rest AABB unavailable")
            continue
        with ctx.pose({joint: KEY_TRAVEL}):
            pressed_aabb = ctx.part_world_aabb(key_part)
            if pressed_aabb is None:
                ctx.fail(f"{key_id}_pressed_aabb_available", "pressed AABB unavailable")
                continue
            ctx.check(
                f"{key_id}_travels_downward",
                pressed_aabb[1][2] < rest_aabb[1][2] - 0.003,
                f"{key_id} should depress downward by short office-key travel",
            )
            ctx.expect_contact(
                key_part,
                housing,
                contact_tol=0.0005,
                name=f"{key_id}_pressed_guided_contact",
            )

    with ctx.pose(key_joint_objects):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_keys_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="all_keys_pressed_no_floating")

    left_rest_aabb = ctx.part_world_aabb(left_foot)
    right_rest_aabb = ctx.part_world_aabb(right_foot)
    with ctx.pose({left_foot_joint: FOOT_DEPLOY_ANGLE, right_foot_joint: FOOT_DEPLOY_ANGLE}):
        ctx.expect_contact(
            left_foot,
            housing,
            contact_tol=0.0005,
            name="left_foot_deployed_contact",
        )
        ctx.expect_contact(
            right_foot,
            housing,
            contact_tol=0.0005,
            name="right_foot_deployed_contact",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="rear_feet_deployed_no_overlap")
        ctx.fail_if_isolated_parts(name="rear_feet_deployed_no_floating")
        left_open_aabb = ctx.part_world_aabb(left_foot)
        right_open_aabb = ctx.part_world_aabb(right_foot)
        if left_rest_aabb is not None and left_open_aabb is not None:
            ctx.check(
                "left_rear_foot_deploys_down",
                left_open_aabb[0][2] < left_rest_aabb[0][2] - 0.006,
                "left rear foot should swing down below the housing when opened",
            )
        else:
            ctx.fail("left_rear_foot_deploys_down", "left rear foot AABB unavailable")
        if right_rest_aabb is not None and right_open_aabb is not None:
            ctx.check(
                "right_rear_foot_deploys_down",
                right_open_aabb[0][2] < right_rest_aabb[0][2] - 0.006,
                "right rear foot should swing down below the housing when opened",
            )
        else:
            ctx.fail("right_rear_foot_deploys_down", "right rear foot AABB unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
