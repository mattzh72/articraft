from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


FRAME_HALF = 0.45
ROSE_HALF = 0.09
ROSE_ATTACH_HALF = 0.066
ROSE_TOP_UNDERSIDE_Z = -0.004
ROSE_BOTTOM_Z = -0.036
TOP_JOINT_Z = -0.048
GRID_JOINT_Z = -0.75
BAR_THICKNESS = 0.016
FERRULE_RADIUS = 0.0036
FERRULE_LENGTH = 0.012
EAR_THICKNESS = 0.003
EAR_WIDTH = 0.010
EAR_HEIGHT = 0.010
SOCKET_DROP = 0.057
BULB_RADIUS = 0.045
MOTION_RANGE = 0.25


def _normalize(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    length = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])
    return (vec[0] / length, vec[1] / length, vec[2] / length)


def _rotation_from_z(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    vx, vy, vz = _normalize(vec)
    yaw = atan2(vy, vx)
    pitch = atan2(sqrt(vx * vx + vy * vy), vz)
    return (0.0, pitch, yaw)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _scaled(vec: tuple[float, float, float], scale: float) -> tuple[float, float, float]:
    return (vec[0] * scale, vec[1] * scale, vec[2] * scale)


def _add_cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    delta = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    length = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2])
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_midpoint(start, end), rpy=_rotation_from_z(delta)),
        material=material,
        name=name,
    )


def _make_rose_bracket(part, center: tuple[float, float, float], axis_xy: tuple[float, float, float], material) -> None:
    offset = (FERRULE_LENGTH + EAR_THICKNESS) * 0.5
    yaw = atan2(axis_xy[1], axis_xy[0])
    stem_length = abs(center[2] - ROSE_TOP_UNDERSIDE_Z)
    part.visual(
        Box((0.012, 0.012, stem_length)),
        origin=Origin(
            xyz=(center[0], center[1], (ROSE_TOP_UNDERSIDE_Z + center[2]) * 0.5),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=material,
    )
    for sign in (-1.0, 1.0):
        part.visual(
            Box((EAR_THICKNESS, EAR_WIDTH, EAR_HEIGHT)),
            origin=Origin(
                xyz=(
                    center[0] + axis_xy[0] * offset * sign,
                    center[1] + axis_xy[1] * offset * sign,
                    center[2],
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=material,
        )


def _build_hanger(
    model: ArticulatedObject,
    name: str,
    *,
    top_joint: tuple[float, float, float],
    bottom_joint: tuple[float, float, float],
    axis_xy: tuple[float, float, float],
    hanger_material,
    ferrule_material,
) -> None:
    hanger = model.part(name)
    delta = (
        bottom_joint[0] - top_joint[0],
        bottom_joint[1] - top_joint[1],
        bottom_joint[2] - top_joint[2],
    )
    length = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2])
    midpoint = (delta[0] * 0.5, delta[1] * 0.5, delta[2] * 0.5)
    hanger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0045, length=length),
        mass=0.35,
        origin=Origin(xyz=midpoint, rpy=_rotation_from_z(delta)),
    )
    _add_cylinder_between(
        hanger,
        (0.0, 0.0, 0.0),
        delta,
        radius=0.0028,
        material=hanger_material,
        name="hanger_wire",
    )
    for joint_center, ferrule_name in [((0.0, 0.0, 0.0), "top_ferrule"), (delta, "bottom_ferrule")]:
        part_center = joint_center
        part_end = (
            part_center[0] + axis_xy[0] * FERRULE_LENGTH * 0.5,
            part_center[1] + axis_xy[1] * FERRULE_LENGTH * 0.5,
            part_center[2],
        )
        part_start = (
            part_center[0] - axis_xy[0] * FERRULE_LENGTH * 0.5,
            part_center[1] - axis_xy[1] * FERRULE_LENGTH * 0.5,
            part_center[2],
        )
        _add_cylinder_between(
            hanger,
            part_start,
            part_end,
            radius=FERRULE_RADIUS,
            material=ferrule_material,
            name=ferrule_name,
        )
        hanger.visual(
            Sphere(radius=0.0048),
            origin=Origin(xyz=joint_center),
            material=ferrule_material,
        )


def _build_grid_corner(
    model: ArticulatedObject,
    name: str,
    *,
    sx: float,
    sy: float,
    axis_xy: tuple[float, float, float],
    frame_material,
    socket_material,
    brass_material,
    bulb_material,
) -> None:
    corner = model.part(name)
    corner.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 0.22)),
        mass=1.2,
        origin=Origin(xyz=(-sx * 0.16, -sy * 0.16, -0.11)),
    )

    offset = (FERRULE_LENGTH + EAR_THICKNESS) * 0.5
    yaw_axis = atan2(axis_xy[1], axis_xy[0])
    for sign in (-1.0, 1.0):
        corner.visual(
            Box((EAR_THICKNESS, EAR_WIDTH, EAR_HEIGHT)),
            origin=Origin(
                xyz=(axis_xy[0] * offset * sign, axis_xy[1] * offset * sign, 0.0),
                rpy=(0.0, 0.0, yaw_axis),
            ),
            material=brass_material,
        )
    corner.visual(
        Box((0.012, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=brass_material,
        name="corner_stem",
    )
    corner.visual(
        Box((0.032, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=frame_material,
        name="corner_block",
    )

    bar_z = -0.030
    side_length = FRAME_HALF
    socket_xy = (-sx * 0.27, -sy * 0.27)
    brace_length = sqrt(socket_xy[0] * socket_xy[0] + socket_xy[1] * socket_xy[1])
    brace_yaw = atan2(socket_xy[1], socket_xy[0])

    corner.visual(
        Box((side_length, BAR_THICKNESS, BAR_THICKNESS)),
        origin=Origin(xyz=(-sx * side_length * 0.5, 0.0, bar_z)),
        material=frame_material,
        name="side_x",
    )
    corner.visual(
        Box((BAR_THICKNESS, side_length, BAR_THICKNESS)),
        origin=Origin(xyz=(0.0, -sy * side_length * 0.5, bar_z)),
        material=frame_material,
        name="side_y",
    )
    corner.visual(
        Box((brace_length, 0.012, 0.012)),
        origin=Origin(
            xyz=(socket_xy[0] * 0.5, socket_xy[1] * 0.5, bar_z),
            rpy=(0.0, 0.0, brace_yaw),
        ),
        material=frame_material,
        name="diagonal_brace",
    )
    corner.visual(
        Box((0.050, 0.050, 0.008)),
        origin=Origin(xyz=(socket_xy[0], socket_xy[1], bar_z)),
        material=frame_material,
        name="socket_plate",
    )
    corner.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(socket_xy[0], socket_xy[1], bar_z - 0.029)),
        material=socket_material,
        name="socket_shell",
    )
    corner.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(socket_xy[0], socket_xy[1], bar_z - 0.068)),
        material=brass_material,
        name="bulb_neck",
    )
    corner.visual(
        Sphere(radius=BULB_RADIUS),
        origin=Origin(xyz=(socket_xy[0], socket_xy[1], bar_z - 0.125)),
        material=bulb_material,
        name="bulb_globe",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_hanging_grid_pendant")

    canopy_black = model.material("canopy_black", rgba=(0.12, 0.12, 0.13, 1.0))
    frame_black = model.material("frame_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hanger_black = model.material("hanger_black", rgba=(0.16, 0.16, 0.17, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.56, 0.44, 0.24, 1.0))
    socket_black = model.material("socket_black", rgba=(0.08, 0.08, 0.09, 1.0))
    bulb_glass = model.material("bulb_glass", rgba=(0.95, 0.92, 0.82, 0.70))

    ceiling_rose = model.part("ceiling_rose")
    ceiling_rose.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.04)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )
    ceiling_rose.visual(
        Box((0.18, 0.18, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=canopy_black,
        name="rose_top",
    )
    ceiling_rose.visual(
        Box((0.18, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, ROSE_HALF - 0.002, -0.019)),
        material=canopy_black,
        name="rose_front_wall",
    )
    ceiling_rose.visual(
        Box((0.18, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, -ROSE_HALF + 0.002, -0.019)),
        material=canopy_black,
        name="rose_back_wall",
    )
    ceiling_rose.visual(
        Box((0.004, 0.172, 0.034)),
        origin=Origin(xyz=(ROSE_HALF - 0.002, 0.0, -0.019)),
        material=canopy_black,
        name="rose_right_wall",
    )
    ceiling_rose.visual(
        Box((0.004, 0.172, 0.034)),
        origin=Origin(xyz=(-ROSE_HALF + 0.002, 0.0, -0.019)),
        material=canopy_black,
        name="rose_left_wall",
    )
    ceiling_rose.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=aged_brass,
        name="cable_gland",
    )

    corners = [
        ("front_left", -1.0, 1.0),
        ("front_right", 1.0, 1.0),
        ("back_right", 1.0, -1.0),
        ("back_left", -1.0, -1.0),
    ]

    for corner_name, sx, sy in corners:
        axis_xy = _normalize((-sy, sx, 0.0))
        rose_joint = (sx * ROSE_ATTACH_HALF, sy * ROSE_ATTACH_HALF, TOP_JOINT_Z)
        grid_joint = (sx * FRAME_HALF, sy * FRAME_HALF, GRID_JOINT_Z)

        _make_rose_bracket(ceiling_rose, rose_joint, axis_xy, aged_brass)
        _build_hanger(
            model,
            f"hanger_{corner_name}",
            top_joint=rose_joint,
            bottom_joint=grid_joint,
            axis_xy=axis_xy,
            hanger_material=hanger_black,
            ferrule_material=aged_brass,
        )
        _build_grid_corner(
            model,
            f"grid_{corner_name}",
            sx=sx,
            sy=sy,
            axis_xy=axis_xy,
            frame_material=frame_black,
            socket_material=socket_black,
            brass_material=aged_brass,
            bulb_material=bulb_glass,
        )

        model.articulation(
            f"rose_to_hanger_{corner_name}",
            ArticulationType.REVOLUTE,
            parent=ceiling_rose,
            child=f"hanger_{corner_name}",
            origin=Origin(xyz=rose_joint),
            axis=axis_xy,
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=1.2,
                lower=-MOTION_RANGE,
                upper=MOTION_RANGE,
            ),
        )
        model.articulation(
            f"hanger_{corner_name}_to_grid_{corner_name}",
            ArticulationType.REVOLUTE,
            parent=f"hanger_{corner_name}",
            child=f"grid_{corner_name}",
            origin=Origin(
                xyz=(
                    grid_joint[0] - rose_joint[0],
                    grid_joint[1] - rose_joint[1],
                    grid_joint[2] - rose_joint[2],
                )
            ),
            axis=axis_xy,
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.2,
                lower=-MOTION_RANGE,
                upper=MOTION_RANGE,
            ),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return (
        (low[0] + high[0]) * 0.5,
        (low[1] + high[1]) * 0.5,
        (low[2] + high[2]) * 0.5,
    )


def _distance(a, b):
    if a is None or b is None:
        return None
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return sqrt(dx * dx + dy * dy + dz * dz)


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

    rose = object_model.get_part("ceiling_rose")
    front_left_hanger = object_model.get_part("hanger_front_left")
    front_right_hanger = object_model.get_part("hanger_front_right")
    back_right_hanger = object_model.get_part("hanger_back_right")
    back_left_hanger = object_model.get_part("hanger_back_left")
    grid_front_left = object_model.get_part("grid_front_left")
    grid_front_right = object_model.get_part("grid_front_right")
    grid_back_right = object_model.get_part("grid_back_right")
    grid_back_left = object_model.get_part("grid_back_left")

    for corner_name, hanger, grid_corner in [
        ("front_left", front_left_hanger, grid_front_left),
        ("front_right", front_right_hanger, grid_front_right),
        ("back_right", back_right_hanger, grid_back_right),
        ("back_left", back_left_hanger, grid_back_left),
    ]:
        ctx.expect_contact(
            rose,
            hanger,
            elem_b="top_ferrule",
            contact_tol=0.0005,
            name=f"{corner_name} hanger ferrule seats in ceiling rose bracket",
        )
        ctx.expect_contact(
            hanger,
            grid_corner,
            elem_a="bottom_ferrule",
            contact_tol=0.0005,
            name=f"{corner_name} hanger ferrule seats in grid corner bracket",
        )

    ctx.expect_contact(
        grid_front_left,
        grid_front_right,
        elem_a="side_x",
        elem_b="side_x",
        contact_tol=0.0005,
        name="front rail sections meet cleanly",
    )
    ctx.expect_contact(
        grid_back_left,
        grid_back_right,
        elem_a="side_x",
        elem_b="side_x",
        contact_tol=0.0005,
        name="rear rail sections meet cleanly",
    )
    ctx.expect_contact(
        grid_front_left,
        grid_back_left,
        elem_a="side_y",
        elem_b="side_y",
        contact_tol=0.0005,
        name="left rail sections meet cleanly",
    )
    ctx.expect_contact(
        grid_front_right,
        grid_back_right,
        elem_a="side_y",
        elem_b="side_y",
        contact_tol=0.0005,
        name="right rail sections meet cleanly",
    )

    top_joint = object_model.get_articulation("rose_to_hanger_front_left")
    bottom_joint = object_model.get_articulation("hanger_front_left_to_grid_front_left")
    rest_corner_pos = ctx.part_world_position(grid_front_left)
    with ctx.pose({top_joint: 0.18}):
        swung_corner_pos = ctx.part_world_position(grid_front_left)
    top_motion = _distance(rest_corner_pos, swung_corner_pos)
    ctx.check(
        "top loop joint moves a suspended grid corner",
        top_motion is not None and top_motion > 0.02,
        details=f"rest={rest_corner_pos}, swung={swung_corner_pos}, distance={top_motion}",
    )

    rest_plate_center = _aabb_center(ctx.part_element_world_aabb(grid_front_left, elem="socket_plate"))
    with ctx.pose({bottom_joint: 0.18}):
        swung_plate_center = _aabb_center(ctx.part_element_world_aabb(grid_front_left, elem="socket_plate"))
    bottom_motion = _distance(rest_plate_center, swung_plate_center)
    ctx.check(
        "bottom loop joint rotates the corner frame module",
        bottom_motion is not None and bottom_motion > 0.01,
        details=f"rest={rest_plate_center}, swung={swung_plate_center}, distance={bottom_motion}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
