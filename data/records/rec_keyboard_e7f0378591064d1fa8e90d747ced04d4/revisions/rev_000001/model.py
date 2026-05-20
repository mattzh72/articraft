from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CASE_WIDTH = 0.250
CASE_DEPTH = 0.110
CASE_HEIGHT = 0.022
CORNER_RADIUS = 0.009
WALL_THICKNESS = 0.004
BOTTOM_THICKNESS = 0.003
PLATE_THICKNESS = 0.006
PLATE_Z_CENTER = CASE_HEIGHT - (PLATE_THICKNESS * 0.5)
PLATE_TOP_Z = CASE_HEIGHT
WALL_SPAN_HEIGHT = CASE_HEIGHT - BOTTOM_THICKNESS - PLATE_THICKNESS
WALL_Z_CENTER = BOTTOM_THICKNESS + (WALL_SPAN_HEIGHT * 0.5)

KEY_ROWS = 5
KEY_COLS = 12
KEY_PITCH = 0.019
KEYCAP_BASE = 0.0172
KEYCAP_TOP = 0.0154
KEYCAP_HEIGHT = 0.0075
KEYCAP_BOTTOM_CLEARANCE = 0.0030
KEY_STEM_SIZE = 0.0136
KEY_STEM_LENGTH = 0.0080
KEY_TRAVEL = 0.0025
KEY_PART_HEIGHT = KEYCAP_HEIGHT + KEY_STEM_LENGTH
KEY_PART_Z_CENTER = KEYCAP_BOTTOM_CLEARANCE + ((KEYCAP_HEIGHT - KEY_STEM_LENGTH) * 0.5)

KNOB_ROW = KEY_ROWS - 1
KNOB_COL = KEY_COLS - 1
KNOB_RADIUS = 0.0080
KNOB_LENGTH = 0.0140
KNOB_SLOT_WIDTH = 0.0165
KNOB_SLOT_DEPTH = 0.0165
KNOB_CENTER_Z = PLATE_TOP_Z + 0.0005
KNOB_BEARING_WIDTH = 0.016
KNOB_BEARING_THICKNESS = 0.002
KNOB_BEARING_HEIGHT = 0.016
KNOB_BEARING_Z = KNOB_CENTER_Z
MATRIX_WIDTH = KEY_COLS * KEY_PITCH
MATRIX_DEPTH = KEY_ROWS * KEY_PITCH
KEY_FRAME_SIDE = (KEY_PITCH - KEY_STEM_SIZE) * 0.5


def _add_plate_frame(
    part,
    *,
    name_prefix: str,
    center_x: float,
    center_y: float,
    outer_width: float,
    outer_depth: float,
    opening_width: float,
    opening_depth: float,
    material,
) -> None:
    side_x = (outer_width - opening_width) * 0.5
    side_y = (outer_depth - opening_depth) * 0.5

    if side_x > 0.0:
        part.visual(
            Box((side_x, outer_depth, PLATE_THICKNESS)),
            origin=Origin(
                xyz=(
                    center_x - (opening_width * 0.5) - (side_x * 0.5),
                    center_y,
                    PLATE_Z_CENTER,
                )
            ),
            material=material,
            name=f"{name_prefix}_left",
        )
        part.visual(
            Box((side_x, outer_depth, PLATE_THICKNESS)),
            origin=Origin(
                xyz=(
                    center_x + (opening_width * 0.5) + (side_x * 0.5),
                    center_y,
                    PLATE_Z_CENTER,
                )
            ),
            material=material,
            name=f"{name_prefix}_right",
        )

    if side_y > 0.0:
        part.visual(
            Box((opening_width, side_y, PLATE_THICKNESS)),
            origin=Origin(
                xyz=(
                    center_x,
                    center_y - (opening_depth * 0.5) - (side_y * 0.5),
                    PLATE_Z_CENTER,
                )
            ),
            material=material,
            name=f"{name_prefix}_front",
        )
        part.visual(
            Box((opening_width, side_y, PLATE_THICKNESS)),
            origin=Origin(
                xyz=(
                    center_x,
                    center_y + (opening_depth * 0.5) + (side_y * 0.5),
                    PLATE_Z_CENTER,
                )
            ),
            material=material,
            name=f"{name_prefix}_rear",
        )


def _key_center(row: int, col: int) -> tuple[float, float]:
    x = (col - ((KEY_COLS - 1) * 0.5)) * KEY_PITCH
    y = (row - ((KEY_ROWS - 1) * 0.5)) * KEY_PITCH
    return x, y


def _key_specs() -> list[dict[str, float | int | str]]:
    specs: list[dict[str, float | int | str]] = []
    for row in range(KEY_ROWS):
        for col in range(KEY_COLS):
            if row == KNOB_ROW and col == KNOB_COL:
                continue
            x, y = _key_center(row, col)
            specs.append(
                {
                    "name": f"key_r{row}_c{col}",
                    "row": row,
                    "col": col,
                    "x": x,
                    "y": y,
                }
            )
    return specs


KEY_SPECS = _key_specs()
KNOB_CENTER_X, KNOB_CENTER_Y = _key_center(KNOB_ROW, KNOB_COL)


def _circle_loop(radius: float, z: float, *, segments: int = 20) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
            z,
        )
        for index in range(segments)
    ]


def _build_keycap_mesh():
    lower = [
        (-KEYCAP_BASE * 0.5, -KEYCAP_BASE * 0.5, 0.0),
        (KEYCAP_BASE * 0.5, -KEYCAP_BASE * 0.5, 0.0),
        (KEYCAP_BASE * 0.5, KEYCAP_BASE * 0.5, 0.0),
        (-KEYCAP_BASE * 0.5, KEYCAP_BASE * 0.5, 0.0),
    ]
    upper = [
        (-KEYCAP_TOP * 0.5, -KEYCAP_TOP * 0.5, KEYCAP_HEIGHT),
        (KEYCAP_TOP * 0.5, -KEYCAP_TOP * 0.5, KEYCAP_HEIGHT),
        (KEYCAP_TOP * 0.5, KEYCAP_TOP * 0.5, KEYCAP_HEIGHT),
        (-KEYCAP_TOP * 0.5, KEYCAP_TOP * 0.5, KEYCAP_HEIGHT),
    ]
    return mesh_from_geometry(
        LoftGeometry([lower, upper], cap=True, closed=True),
        "mechanical_keyboard_keycap",
    )


def _build_knob_mesh():
    half_length = KNOB_LENGTH * 0.5
    body_radius = KNOB_RADIUS
    shoulder_radius = KNOB_RADIUS * 0.94
    end_radius = KNOB_RADIUS * 0.82
    profiles = [
        _circle_loop(end_radius, -half_length),
        _circle_loop(shoulder_radius, -half_length + 0.0018),
        _circle_loop(body_radius, -half_length + 0.0040),
        _circle_loop(body_radius, half_length - 0.0040),
        _circle_loop(shoulder_radius, half_length - 0.0018),
        _circle_loop(end_radius, half_length),
    ]
    return mesh_from_geometry(
        LoftGeometry(profiles, cap=True, closed=True),
        "mechanical_keyboard_volume_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_mechanical_keyboard")

    case_body = model.material("case_body", rgba=(0.18, 0.19, 0.21, 1.0))
    top_plate = model.material("top_plate", rgba=(0.12, 0.13, 0.15, 1.0))
    keycap = model.material("keycap", rgba=(0.90, 0.91, 0.92, 1.0))
    stem = model.material("stem", rgba=(0.28, 0.29, 0.31, 1.0))
    knob = model.material("knob", rgba=(0.22, 0.23, 0.24, 1.0))
    accent = model.material("accent", rgba=(0.78, 0.33, 0.18, 1.0))

    case = model.part("case")
    case.visual(
        Box((CASE_WIDTH, CASE_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=case_body,
        name="bottom_panel",
    )
    case.visual(
        Box((CASE_WIDTH, WALL_THICKNESS, WALL_SPAN_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CASE_DEPTH * 0.5) + (WALL_THICKNESS * 0.5),
                WALL_Z_CENTER,
            )
        ),
        material=case_body,
        name="front_wall",
    )
    case.visual(
        Box((CASE_WIDTH, WALL_THICKNESS, WALL_SPAN_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (CASE_DEPTH * 0.5) - (WALL_THICKNESS * 0.5),
                WALL_Z_CENTER,
            )
        ),
        material=case_body,
        name="rear_wall",
    )
    case.visual(
        Box((WALL_THICKNESS, CASE_DEPTH - (2.0 * WALL_THICKNESS), WALL_SPAN_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CASE_WIDTH * 0.5) + (WALL_THICKNESS * 0.5),
                0.0,
                WALL_Z_CENTER,
            )
        ),
        material=case_body,
        name="left_wall",
    )
    case.visual(
        Box((WALL_THICKNESS, CASE_DEPTH - (2.0 * WALL_THICKNESS), WALL_SPAN_HEIGHT)),
        origin=Origin(
            xyz=(
                (CASE_WIDTH * 0.5) - (WALL_THICKNESS * 0.5),
                0.0,
                WALL_Z_CENTER,
            )
        ),
        material=case_body,
        name="right_wall",
    )
    case.visual(
        Box((CASE_WIDTH, (CASE_DEPTH - MATRIX_DEPTH) * 0.5, PLATE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -((CASE_DEPTH * 0.5) - (((CASE_DEPTH - MATRIX_DEPTH) * 0.5) * 0.5)),
                PLATE_Z_CENTER,
            )
        ),
        material=top_plate,
        name="top_plate_front_border",
    )
    case.visual(
        Box((CASE_WIDTH, (CASE_DEPTH - MATRIX_DEPTH) * 0.5, PLATE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                (CASE_DEPTH * 0.5) - (((CASE_DEPTH - MATRIX_DEPTH) * 0.5) * 0.5),
                PLATE_Z_CENTER,
            )
        ),
        material=top_plate,
        name="top_plate_rear_border",
    )
    case.visual(
        Box(((CASE_WIDTH - MATRIX_WIDTH) * 0.5, CASE_DEPTH, PLATE_THICKNESS)),
        origin=Origin(
            xyz=(
                -((CASE_WIDTH * 0.5) - (((CASE_WIDTH - MATRIX_WIDTH) * 0.5) * 0.5)),
                0.0,
                PLATE_Z_CENTER,
            )
        ),
        material=top_plate,
        name="top_plate_left_border",
    )
    case.visual(
        Box(((CASE_WIDTH - MATRIX_WIDTH) * 0.5, CASE_DEPTH, PLATE_THICKNESS)),
        origin=Origin(
            xyz=(
                (CASE_WIDTH * 0.5) - (((CASE_WIDTH - MATRIX_WIDTH) * 0.5) * 0.5),
                0.0,
                PLATE_Z_CENTER,
            )
        ),
        material=top_plate,
        name="top_plate_right_border",
    )
    _add_plate_frame(
        case,
        name_prefix="knob_slot",
        center_x=KNOB_CENTER_X,
        center_y=KNOB_CENTER_Y,
        outer_width=KEY_PITCH,
        outer_depth=KEY_PITCH,
        opening_width=KNOB_SLOT_WIDTH,
        opening_depth=KNOB_SLOT_DEPTH,
        material=top_plate,
    )
    case.visual(
        Box((KNOB_BEARING_WIDTH, KNOB_BEARING_THICKNESS, KNOB_BEARING_HEIGHT)),
        origin=Origin(
            xyz=(
                0.112,
                KNOB_CENTER_Y - ((KNOB_LENGTH * 0.5) + (KNOB_BEARING_THICKNESS * 0.5)),
                KNOB_BEARING_Z,
            )
        ),
        material=top_plate,
        name="knob_front_bearing",
    )
    case.visual(
        Box((KNOB_BEARING_WIDTH, KNOB_BEARING_THICKNESS, KNOB_BEARING_HEIGHT)),
        origin=Origin(
            xyz=(
                0.112,
                KNOB_CENTER_Y + ((KNOB_LENGTH * 0.5) + (KNOB_BEARING_THICKNESS * 0.5)),
                KNOB_BEARING_Z,
            )
        ),
        material=top_plate,
        name="knob_rear_bearing",
    )
    case.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, 0.034)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    for spec in KEY_SPECS:
        _add_plate_frame(
            case,
            name_prefix=f"{spec['name']}_plate",
            center_x=float(spec["x"]),
            center_y=float(spec["y"]),
            outer_width=KEY_PITCH,
            outer_depth=KEY_PITCH,
            opening_width=KEY_STEM_SIZE,
            opening_depth=KEY_STEM_SIZE,
            material=top_plate,
        )

    keycap_mesh = _build_keycap_mesh()
    knob_mesh = _build_knob_mesh()
    for spec in KEY_SPECS:
        key_name = str(spec["name"])
        key_part = model.part(key_name)
        key_part.visual(
            keycap_mesh,
            origin=Origin(xyz=(0.0, 0.0, KEYCAP_BOTTOM_CLEARANCE)),
            material=keycap,
            name="cap",
        )
        key_part.visual(
            Box((KEY_STEM_SIZE, KEY_STEM_SIZE, KEY_STEM_LENGTH)),
            origin=Origin(xyz=(0.0, 0.0, KEYCAP_BOTTOM_CLEARANCE - (KEY_STEM_LENGTH * 0.5))),
            material=stem,
            name="stem",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((KEYCAP_BASE, KEYCAP_BASE, KEY_PART_HEIGHT)),
            mass=0.005,
            origin=Origin(xyz=(0.0, 0.0, KEY_PART_Z_CENTER)),
        )
        model.articulation(
            f"case_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key_part,
            origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), PLATE_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.08,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    volume_knob = model.part("volume_knob")
    volume_knob.visual(
        knob_mesh,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob,
        name="wheel",
    )
    volume_knob.visual(
        Box((0.004, KNOB_LENGTH * 0.96, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, KNOB_RADIUS - 0.0008)),
        material=accent,
        name="indicator",
    )
    volume_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_LENGTH),
        mass=0.03,
        origin=Origin(),
    )
    model.articulation(
        "case_to_volume_knob",
        ArticulationType.REVOLUTE,
        parent=case,
        child=volume_knob,
        origin=Origin(xyz=(KNOB_CENTER_X, KNOB_CENTER_Y, KNOB_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=4.0,
            lower=-1.4,
            upper=1.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    knob = object_model.get_part("volume_knob")
    knob_joint = object_model.get_articulation("case_to_volume_knob")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    left_front = object_model.get_part("key_r0_c0")
    next_left_front = object_model.get_part("key_r0_c1")
    next_row_front = object_model.get_part("key_r1_c0")
    near_knob = object_model.get_part("key_r4_c10")

    ctx.expect_origin_distance(
        left_front,
        next_left_front,
        axes="x",
        min_dist=KEY_PITCH - 0.0005,
        max_dist=KEY_PITCH + 0.0005,
        name="key_row_pitch",
    )
    ctx.expect_origin_distance(
        left_front,
        next_row_front,
        axes="y",
        min_dist=KEY_PITCH - 0.0005,
        max_dist=KEY_PITCH + 0.0005,
        name="key_column_pitch",
    )
    ctx.expect_origin_gap(
        knob,
        case,
        axis="x",
        min_gap=0.095,
        max_gap=0.110,
        name="knob_is_in_right_corner_zone",
    )
    ctx.expect_origin_gap(
        knob,
        case,
        axis="y",
        min_gap=0.030,
        max_gap=0.045,
        name="knob_is_in_rear_corner_zone",
    )
    ctx.expect_origin_gap(
        knob,
        near_knob,
        axis="x",
        min_gap=0.015,
        max_gap=0.030,
        name="knob_clears_neighboring_key",
    )

    ctx.check(
        "volume_knob_axis_front_to_back",
        tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        f"axis was {knob_joint.axis}",
    )

    knob_limits = knob_joint.motion_limits
    if knob_limits is not None and knob_limits.lower is not None and knob_limits.upper is not None:
        with ctx.pose({knob_joint: knob_limits.lower}):
            ctx.expect_contact(knob, case, name="volume_knob_lower_supported")
            ctx.fail_if_parts_overlap_in_current_pose(name="volume_knob_lower_no_overlap")
        with ctx.pose({knob_joint: knob_limits.upper}):
            ctx.expect_contact(knob, case, name="volume_knob_upper_supported")
            ctx.fail_if_parts_overlap_in_current_pose(name="volume_knob_upper_no_overlap")

    for spec in KEY_SPECS:
        key_name = str(spec["name"])
        key_part = object_model.get_part(key_name)
        key_joint = object_model.get_articulation(f"case_to_{key_name}")
        limits = key_joint.motion_limits

        ctx.check(
            f"{key_joint.name}_axis_vertical",
            tuple(key_joint.axis) == (0.0, 0.0, -1.0),
            f"axis was {key_joint.axis}",
        )

        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"{key_joint.name}_has_limits", "prismatic key joint is missing finite travel limits")
            continue

        with ctx.pose({key_joint: limits.lower}):
            ctx.expect_contact(key_part, case, name=f"{key_name}_rest_supported")
            ctx.expect_gap(
                key_part,
                case,
                axis="z",
                min_gap=0.0024,
                max_gap=0.0036,
                positive_elem="cap",
                negative_elem=f"{key_name}_plate_front",
                name=f"{key_name}_rest_cap_clearance",
            )

        with ctx.pose({key_joint: limits.upper}):
            ctx.expect_contact(key_part, case, name=f"{key_name}_pressed_supported")
            ctx.expect_gap(
                key_part,
                case,
                axis="z",
                min_gap=0.0003,
                max_gap=0.0012,
                positive_elem="cap",
                negative_elem=f"{key_name}_plate_front",
                name=f"{key_name}_pressed_cap_clearance",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
