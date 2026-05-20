from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_FLOOR_THICKNESS = 0.004
CASE_WALL_HEIGHT = 0.014
KEY_TRAVEL = 0.004
KEY_SWITCH_Z = 0.022
KEY_PITCH = 0.019


def _rotate_points(points: list[tuple[float, float]], angle: float) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * x - s * y, s * x + c * y) for x, y in points]


def _translate_points(
    points: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in points]


def _transform_xy(local_x: float, local_y: float, yaw: float) -> tuple[float, float]:
    c = cos(yaw)
    s = sin(yaw)
    return (c * local_x - s * local_y, s * local_x + c * local_y)


def _alice_outline_profile() -> list[tuple[float, float]]:
    front_y = -0.066
    notch_y = -0.030
    back_y = 0.070
    notch_half = 0.028
    return [
        (-0.058, front_y),
        (-0.154, front_y),
        (-0.170, -0.050),
        (-0.174, 0.060),
        (-0.160, back_y),
        (0.160, back_y),
        (0.174, 0.060),
        (0.170, -0.050),
        (0.154, front_y),
        (0.058, front_y),
        (notch_half, notch_y),
        (-notch_half, notch_y),
    ]


def _cluster_opening(center_x: float, center_y: float, yaw: float) -> list[tuple[float, float]]:
    opening = rounded_rect_profile(0.112, 0.094, radius=0.007, corner_segments=8)
    return _translate_points(_rotate_points(opening, yaw), center_x, center_y)


def _add_key(
    model: ArticulatedObject,
    case,
    *,
    name: str,
    x: float,
    y: float,
    yaw: float,
    key_material,
    accent_material,
) -> None:
    key = model.part(name)
    key.visual(
        Box((0.0175, 0.0175, 0.0045)),
        origin=Origin(xyz=(0.0, 0.0, 0.00525)),
        material=key_material,
        name="cap_top",
    )
    key.visual(
        Box((0.0154, 0.0154, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.00125)),
        material=key_material,
        name="cap_skirt",
    )
    key.visual(
        Box((0.0085, 0.0085, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.0055)),
        material=accent_material,
        name="slider",
    )

    model.articulation(
        f"case_to_{name}",
        ArticulationType.PRISMATIC,
        parent=case,
        child=key,
        origin=Origin(xyz=(x, y, KEY_SWITCH_Z), rpy=(0.0, 0.0, yaw)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=KEY_TRAVEL,
        ),
    )


def _add_tent_foot(
    model: ArticulatedObject,
    case,
    *,
    name: str,
    x: float,
    y: float,
    foot_material,
    pad_material,
) -> None:
    foot = model.part(name)
    foot.visual(
        Cylinder(radius=0.0025, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.0025), rpy=(0.0, pi / 2.0, 0.0)),
        material=foot_material,
        name="foot_barrel",
    )
    foot.visual(
        Box((0.016, 0.038, 0.004)),
        origin=Origin(xyz=(0.0, -0.0205, -0.002)),
        material=foot_material,
        name="foot_bar",
    )
    foot.visual(
        Box((0.022, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, -0.037, -0.004)),
        material=pad_material,
        name="foot_pad",
    )

    model.articulation(
        f"case_to_{name}",
        ArticulationType.REVOLUTE,
        parent=case,
        child=foot,
        origin=Origin(xyz=(x, y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="alice_ergonomic_keyboard")

    case_material = model.material("case_shell", rgba=(0.19, 0.20, 0.23, 1.0))
    key_material = model.material("keycap", rgba=(0.10, 0.11, 0.12, 1.0))
    slider_material = model.material("switch_slider", rgba=(0.22, 0.24, 0.27, 1.0))
    foot_material = model.material("tent_foot", rgba=(0.15, 0.16, 0.18, 1.0))
    pad_material = model.material("foot_pad", rgba=(0.08, 0.08, 0.09, 1.0))

    left_center = (-0.084, 0.004)
    right_center = (0.084, 0.004)
    left_yaw = 0.28
    right_yaw = -0.28

    case = model.part("case")
    floor_geom = ExtrudeGeometry.from_z0(_alice_outline_profile(), CASE_FLOOR_THICKNESS)
    case.visual(
        mesh_from_geometry(floor_geom, "case_floor"),
        material=case_material,
        name="case_floor",
    )

    wall_ring_geom = ExtrudeWithHolesGeometry(
        _alice_outline_profile(),
        [
            _cluster_opening(left_center[0], left_center[1], left_yaw),
            _cluster_opening(right_center[0], right_center[1], right_yaw),
        ],
        CASE_WALL_HEIGHT,
        center=True,
    )
    wall_ring_geom.translate(0.0, 0.0, CASE_FLOOR_THICKNESS - 0.0005 + CASE_WALL_HEIGHT / 2.0)
    case.visual(
        mesh_from_geometry(wall_ring_geom, "case_wall_ring"),
        material=case_material,
        name="case_wall_ring",
    )
    case.visual(
        Box((0.332, 0.010, 0.007)),
        origin=Origin(xyz=(0.0, 0.064, 0.0207)),
        material=case_material,
        name="rear_spine",
    )
    case.visual(
        Box((0.024, 0.006, 0.006)),
        origin=Origin(xyz=(-0.142, 0.063, 0.001)),
        material=case_material,
        name="left_hinge_block",
    )
    case.visual(
        Box((0.024, 0.006, 0.006)),
        origin=Origin(xyz=(0.142, 0.063, 0.001)),
        material=case_material,
        name="right_hinge_block",
    )

    row_offsets = [-1.5 * KEY_PITCH, -0.5 * KEY_PITCH, 0.5 * KEY_PITCH, 1.5 * KEY_PITCH]
    left_col_offsets = [-2.0 * KEY_PITCH, -1.0 * KEY_PITCH, 0.0, 1.0 * KEY_PITCH, 2.0 * KEY_PITCH]
    right_col_offsets = [2.0 * KEY_PITCH, 1.0 * KEY_PITCH, 0.0, -1.0 * KEY_PITCH, -2.0 * KEY_PITCH]
    column_stagger = (-0.004, -0.0015, 0.0, 0.0015, 0.004)

    for row_index, row_y in enumerate(row_offsets):
        for col_index, col_x in enumerate(left_col_offsets):
            local_y = row_y + column_stagger[col_index]
            dx, dy = _transform_xy(col_x, local_y, left_yaw)
            case.visual(
                Box((0.0128, 0.0128, 0.0078)),
                origin=Origin(
                    xyz=(left_center[0] + dx, left_center[1] + dy, 0.0076),
                    rpy=(0.0, 0.0, left_yaw),
                ),
                material=slider_material,
                name=f"left_switch_r{row_index}c{col_index}",
            )
            _add_key(
                model,
                case,
                name=f"left_key_r{row_index}c{col_index}",
                x=left_center[0] + dx,
                y=left_center[1] + dy,
                yaw=left_yaw,
                key_material=key_material,
                accent_material=slider_material,
            )

        for col_index, col_x in enumerate(right_col_offsets):
            local_y = row_y + column_stagger[col_index]
            dx, dy = _transform_xy(col_x, local_y, right_yaw)
            case.visual(
                Box((0.0128, 0.0128, 0.0078)),
                origin=Origin(
                    xyz=(right_center[0] + dx, right_center[1] + dy, 0.0076),
                    rpy=(0.0, 0.0, right_yaw),
                ),
                material=slider_material,
                name=f"right_switch_r{row_index}c{col_index}",
            )
            _add_key(
                model,
                case,
                name=f"right_key_r{row_index}c{col_index}",
                x=right_center[0] + dx,
                y=right_center[1] + dy,
                yaw=right_yaw,
                key_material=key_material,
                accent_material=slider_material,
            )

    _add_tent_foot(
        model,
        case,
        name="left_tent_foot",
        x=-0.143,
        y=0.058,
        foot_material=foot_material,
        pad_material=pad_material,
    )
    _add_tent_foot(
        model,
        case,
        name="right_tent_foot",
        x=0.143,
        y=0.058,
        foot_material=foot_material,
        pad_material=pad_material,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    case = object_model.get_part("case")
    left_outer = object_model.get_part("left_key_r1c0")
    left_inner = object_model.get_part("left_key_r1c4")
    right_outer = object_model.get_part("right_key_r1c0")
    right_inner = object_model.get_part("right_key_r1c4")
    left_switch = object_model.get_articulation("case_to_left_key_r1c4")
    right_switch = object_model.get_articulation("case_to_right_key_r1c4")
    left_foot = object_model.get_part("left_tent_foot")
    right_foot = object_model.get_part("right_tent_foot")
    left_foot_hinge = object_model.get_articulation("case_to_left_tent_foot")
    right_foot_hinge = object_model.get_articulation("case_to_right_tent_foot")

    ctx.expect_within(
        left_inner,
        case,
        axes="xy",
        margin=0.0,
        name="left cluster remains inside the continuous case footprint",
    )
    ctx.expect_within(
        right_inner,
        case,
        axes="xy",
        margin=0.0,
        name="right cluster remains inside the continuous case footprint",
    )

    left_outer_pos = ctx.part_world_position(left_outer)
    left_inner_pos = ctx.part_world_position(left_inner)
    right_outer_pos = ctx.part_world_position(right_outer)
    right_inner_pos = ctx.part_world_position(right_inner)
    ctx.check(
        "main key fields cant inward toward the center notch",
        left_outer_pos is not None
        and left_inner_pos is not None
        and right_outer_pos is not None
        and right_inner_pos is not None
        and left_inner_pos[1] > left_outer_pos[1] + 0.01
        and right_inner_pos[1] > right_outer_pos[1] + 0.01
        and left_inner_pos[0] < 0.0
        and right_inner_pos[0] > 0.0,
        details=(
            f"left_outer={left_outer_pos}, left_inner={left_inner_pos}, "
            f"right_outer={right_outer_pos}, right_inner={right_inner_pos}"
        ),
    )

    left_rest = ctx.part_world_position(left_inner)
    right_rest = ctx.part_world_position(right_inner)
    with ctx.pose({left_switch: 0.0038, right_switch: 0.0038}):
        left_pressed = ctx.part_world_position(left_inner)
        right_pressed = ctx.part_world_position(right_inner)

    ctx.check(
        "left key plunges straight down with short travel",
        left_rest is not None
        and left_pressed is not None
        and abs(left_pressed[0] - left_rest[0]) < 1e-4
        and abs(left_pressed[1] - left_rest[1]) < 1e-4
        and 0.003 < (left_rest[2] - left_pressed[2]) < 0.0042,
        details=f"rest={left_rest}, pressed={left_pressed}",
    )
    ctx.check(
        "right key plunges straight down with short travel",
        right_rest is not None
        and right_pressed is not None
        and abs(right_pressed[0] - right_rest[0]) < 1e-4
        and abs(right_pressed[1] - right_rest[1]) < 1e-4
        and 0.003 < (right_rest[2] - right_pressed[2]) < 0.0042,
        details=f"rest={right_rest}, pressed={right_pressed}",
    )

    def pad_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            0.5 * (mins[0] + maxs[0]),
            0.5 * (mins[1] + maxs[1]),
            0.5 * (mins[2] + maxs[2]),
        )

    left_pad_rest = pad_center(ctx.part_element_world_aabb(left_foot, elem="foot_pad"))
    right_pad_rest = pad_center(ctx.part_element_world_aabb(right_foot, elem="foot_pad"))
    with ctx.pose({left_foot_hinge: 1.05, right_foot_hinge: 1.05}):
        left_pad_deployed = pad_center(ctx.part_element_world_aabb(left_foot, elem="foot_pad"))
        right_pad_deployed = pad_center(ctx.part_element_world_aabb(right_foot, elem="foot_pad"))

    ctx.check(
        "left tent foot folds down from the rear corner",
        left_pad_rest is not None
        and left_pad_deployed is not None
        and left_pad_deployed[2] < left_pad_rest[2] - 0.02,
        details=f"rest={left_pad_rest}, deployed={left_pad_deployed}",
    )
    ctx.check(
        "right tent foot folds down from the rear corner",
        right_pad_rest is not None
        and right_pad_deployed is not None
        and right_pad_deployed[2] < right_pad_rest[2] - 0.02,
        details=f"rest={right_pad_rest}, deployed={right_pad_deployed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
