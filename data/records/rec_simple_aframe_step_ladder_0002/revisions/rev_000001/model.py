from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


PLATFORM_HEIGHT = 1.215
LADDER_WIDTH = 0.44
FRONT_RAIL_Y = 0.195
REAR_RAIL_Y = 0.185


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _lerp_point(
    a: tuple[float, float, float], b: tuple[float, float, float], t: float
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _point_at_z(
    a: tuple[float, float, float], b: tuple[float, float, float], z: float
) -> tuple[float, float, float]:
    if abs(b[2] - a[2]) < 1e-9:
        return a
    t = (z - a[2]) / (b[2] - a[2])
    return _lerp_point(a, b, t)


def _rpy_for_x_beam(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    horiz = math.hypot(dx, dy)
    pitch = -math.atan2(dz, horiz)
    return (0.0, pitch, yaw)


def _add_box_beam(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    size_y: float,
    size_z: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((_distance(a, b), size_y, size_z)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_x_beam(a, b)),
        material=material,
        name=name,
    )


def _bolt_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_bolt_head(
    part,
    xyz: tuple[float, float, float],
    *,
    axis: str,
    radius: float,
    thickness: float,
    material,
    name: str | None = None,
) -> None:
    if axis == "x":
        size = (thickness, radius * 1.7, radius * 1.7)
    elif axis == "y":
        size = (radius * 1.7, thickness, radius * 1.7)
    else:
        size = (radius * 1.7, radius * 1.7, thickness)
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_step(
    part,
    *,
    name: str,
    x: float,
    z: float,
    width: float,
    depth: float,
    thickness: float,
    rail_outer_y: float,
    tread_material,
    grip_material,
    fastener_material,
) -> None:
    part.visual(
        Box((depth, width, thickness)),
        origin=Origin(xyz=(x, 0.0, z)),
        material=tread_material,
        name=name,
    )
    part.visual(
        Cylinder(radius=thickness * 0.55, length=width * 0.98),
        origin=Origin(
            xyz=(x + depth * 0.46, 0.0, z + thickness * 0.04),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=tread_material,
    )
    for ridge_x in (-depth * 0.18, 0.0, depth * 0.18):
        part.visual(
            Box((depth * 0.12, width * 0.90, 0.0028)),
            origin=Origin(xyz=(x + ridge_x, 0.0, z + thickness * 0.56)),
            material=grip_material,
        )


def _add_spreader_part(
    part,
    *,
    end: tuple[float, float, float],
    material,
    fastener_material,
) -> None:
    _add_box_beam(
        part,
        (0.0, 0.0, 0.0),
        end,
        size_y=0.004,
        size_z=0.022,
        material=material,
        name="brace_bar",
    )
    part.visual(
        Box((0.020, 0.006, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name="pivot_lug",
    )
    part.visual(
        Box((0.016, 0.006, 0.028)),
        origin=Origin(xyz=end),
        material=material,
        name="inner_lug",
    )
    part.visual(
        Box((0.036, 0.006, 0.016)),
        origin=Origin(xyz=(end[0] * 0.55, 0.0, end[2] * 0.55 + 0.010)),
        material=material,
    )
    _add_bolt_head(
        part,
        (0.0, 0.0, 0.0),
        axis="y",
        radius=0.0065,
        thickness=0.010,
        material=fastener_material,
    )
    _add_bolt_head(
        part,
        end,
        axis="y",
        radius=0.0065,
        thickness=0.010,
        material=fastener_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_a_frame_step_ladder", assets=ASSETS)

    ladder_orange = model.material("ladder_orange", rgba=(0.83, 0.43, 0.14, 1.0))
    molded_black = model.material("molded_black", rgba=(0.10, 0.11, 0.12, 1.0))
    brace_gray = model.material("brace_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.71, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))

    top_cap = model.part("top_cap")
    top_cap.visual(
        Box((0.205, 0.465, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_HEIGHT + 0.010)),
        material=molded_black,
        name="top_platform",
    )
    top_cap.visual(
        Box((0.115, 0.145, 0.050)),
        origin=Origin(xyz=(0.050, 0.0, PLATFORM_HEIGHT - 0.018)),
        material=molded_black,
        name="front_cap_body",
    )
    top_cap.visual(
        Box((0.100, 0.120, 0.042)),
        origin=Origin(xyz=(-0.040, 0.0, PLATFORM_HEIGHT - 0.020)),
        material=molded_black,
        name="rear_cap_body",
    )
    top_cap.visual(
        Box((0.130, 0.310, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_HEIGHT + 0.019)),
        material=dark_steel,
        name="platform_pad",
    )
    top_cap.visual(
        Box((0.110, 0.022, 0.044)),
        origin=Origin(xyz=(0.002, 0.228, PLATFORM_HEIGHT - 0.003)),
        material=molded_black,
        name="left_side_cheek",
    )
    top_cap.visual(
        Box((0.110, 0.022, 0.044)),
        origin=Origin(xyz=(0.002, -0.228, PLATFORM_HEIGHT - 0.003)),
        material=molded_black,
        name="right_side_cheek",
    )
    for x, y, name in (
        (0.018, FRONT_RAIL_Y, "front_hinge_boss_left"),
        (0.018, -FRONT_RAIL_Y, "front_hinge_boss_right"),
        (-0.018, REAR_RAIL_Y, "rear_hinge_boss_left"),
        (-0.018, -REAR_RAIL_Y, "rear_hinge_boss_right"),
    ):
        top_cap.visual(
            Box((0.028, 0.022, 0.020)),
            origin=Origin(xyz=(x, y, PLATFORM_HEIGHT + 0.010)),
            material=dark_steel,
            name=name,
        )
        _add_bolt_head(
            top_cap,
            (x, y, PLATFORM_HEIGHT + 0.010),
            axis="y",
            radius=0.007,
            thickness=0.010,
            material=steel,
        )
    top_cap.inertial = Inertial.from_geometry(
        Box((0.22, 0.48, 0.12)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_HEIGHT - 0.020)),
    )

    front_frame = model.part("front_frame")
    _add_box_beam(
        front_frame,
        (0.020, FRONT_RAIL_Y, -0.020),
        (0.280, FRONT_RAIL_Y, -1.200),
        size_y=0.034,
        size_z=0.022,
        material=ladder_orange,
        name="left_rail",
    )
    _add_box_beam(
        front_frame,
        (0.020, -FRONT_RAIL_Y, -0.020),
        (0.280, -FRONT_RAIL_Y, -1.200),
        size_y=0.034,
        size_z=0.022,
        material=ladder_orange,
        name="right_rail",
    )
    _add_box_beam(
        front_frame,
        (0.030, FRONT_RAIL_Y, -0.050),
        (0.108, FRONT_RAIL_Y, -0.430),
        size_y=0.018,
        size_z=0.010,
        material=dark_steel,
    )
    _add_box_beam(
        front_frame,
        (0.030, -FRONT_RAIL_Y, -0.050),
        (0.108, -FRONT_RAIL_Y, -0.430),
        size_y=0.018,
        size_z=0.010,
        material=dark_steel,
    )
    front_frame.visual(
        Box((0.060, 0.372, 0.022)),
        origin=Origin(xyz=(0.055, 0.0, -0.100)),
        material=dark_steel,
        name="upper_crossmember",
    )
    front_frame.visual(
        Box((0.100, 0.014, 0.170)),
        origin=Origin(xyz=(0.062, 0.177, -0.122)),
        material=dark_steel,
        name="left_upper_gusset",
    )
    front_frame.visual(
        Box((0.100, 0.014, 0.170)),
        origin=Origin(xyz=(0.062, -0.177, -0.122)),
        material=dark_steel,
        name="right_upper_gusset",
    )
    _add_step(
        front_frame,
        name="step_01",
        x=0.238,
        z=-1.000,
        width=0.372,
        depth=0.105,
        thickness=0.018,
        rail_outer_y=0.214,
        tread_material=brace_gray,
        grip_material=dark_steel,
        fastener_material=steel,
    )
    _add_step(
        front_frame,
        name="step_02",
        x=0.182,
        z=-0.740,
        width=0.372,
        depth=0.105,
        thickness=0.018,
        rail_outer_y=0.214,
        tread_material=brace_gray,
        grip_material=dark_steel,
        fastener_material=steel,
    )
    _add_step(
        front_frame,
        name="step_03",
        x=0.126,
        z=-0.480,
        width=0.372,
        depth=0.105,
        thickness=0.018,
        rail_outer_y=0.214,
        tread_material=brace_gray,
        grip_material=dark_steel,
        fastener_material=steel,
    )
    _add_step(
        front_frame,
        name="step_04",
        x=0.070,
        z=-0.220,
        width=0.372,
        depth=0.108,
        thickness=0.020,
        rail_outer_y=0.214,
        tread_material=brace_gray,
        grip_material=dark_steel,
        fastener_material=steel,
    )
    front_frame.visual(
        Box((0.090, 0.052, 0.030)),
        origin=Origin(xyz=(0.280, FRONT_RAIL_Y, -1.200)),
        material=rubber,
        name="left_foot",
    )
    front_frame.visual(
        Box((0.090, 0.052, 0.030)),
        origin=Origin(xyz=(0.280, -FRONT_RAIL_Y, -1.200)),
        material=rubber,
        name="right_foot",
    )
    front_frame.visual(
        Box((0.024, 0.024, 0.020)),
        origin=Origin(xyz=(0.018, FRONT_RAIL_Y, -0.010)),
        material=dark_steel,
        name="left_hinge_pad",
    )
    front_frame.visual(
        Box((0.024, 0.024, 0.020)),
        origin=Origin(xyz=(0.018, -FRONT_RAIL_Y, -0.010)),
        material=dark_steel,
        name="right_hinge_pad",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.38, 0.44, 1.24)),
        mass=8.7,
        origin=Origin(xyz=(0.155, 0.0, -0.610)),
    )

    rear_frame = model.part("rear_frame")
    rear_left_top = (-0.018, REAR_RAIL_Y, -0.010)
    rear_left_bottom = (-0.340, REAR_RAIL_Y, -1.200)
    rear_right_top = (-0.018, -REAR_RAIL_Y, -0.010)
    rear_right_bottom = (-0.340, -REAR_RAIL_Y, -1.200)
    _add_box_beam(
        rear_frame,
        rear_left_top,
        rear_left_bottom,
        size_y=0.030,
        size_z=0.020,
        material=ladder_orange,
        name="left_rail",
    )
    _add_box_beam(
        rear_frame,
        rear_right_top,
        rear_right_bottom,
        size_y=0.030,
        size_z=0.020,
        material=ladder_orange,
        name="right_rail",
    )
    _add_box_beam(
        rear_frame,
        _point_at_z(rear_left_top, rear_left_bottom, -0.110),
        _point_at_z(rear_right_top, rear_right_bottom, -0.110),
        size_y=0.020,
        size_z=0.018,
        material=dark_steel,
        name="upper_crossbar",
    )
    _add_box_beam(
        rear_frame,
        _point_at_z(rear_left_top, rear_left_bottom, -0.620),
        _point_at_z(rear_right_top, rear_right_bottom, -0.620),
        size_y=0.018,
        size_z=0.016,
        material=dark_steel,
        name="mid_crossbar",
    )
    _add_box_beam(
        rear_frame,
        _point_at_z(rear_left_top, rear_left_bottom, -0.260),
        _point_at_z(rear_right_top, rear_right_bottom, -0.900),
        size_y=0.014,
        size_z=0.008,
        material=brace_gray,
        name="diagonal_brace_a",
    )
    _add_box_beam(
        rear_frame,
        _point_at_z(rear_right_top, rear_right_bottom, -0.260),
        _point_at_z(rear_left_top, rear_left_bottom, -0.900),
        size_y=0.014,
        size_z=0.008,
        material=brace_gray,
        name="diagonal_brace_b",
    )
    rear_frame.visual(
        Box((0.084, 0.050, 0.030)),
        origin=Origin(xyz=(-0.340, REAR_RAIL_Y, -1.200)),
        material=rubber,
        name="left_foot",
    )
    rear_frame.visual(
        Box((0.084, 0.050, 0.030)),
        origin=Origin(xyz=(-0.340, -REAR_RAIL_Y, -1.200)),
        material=rubber,
        name="right_foot",
    )
    rear_frame.visual(
        Box((0.024, 0.024, 0.020)),
        origin=Origin(xyz=(-0.018, REAR_RAIL_Y, -0.010)),
        material=dark_steel,
        name="left_hinge_pad",
    )
    rear_frame.visual(
        Box((0.024, 0.024, 0.020)),
        origin=Origin(xyz=(-0.018, -REAR_RAIL_Y, -0.010)),
        material=dark_steel,
        name="right_hinge_pad",
    )
    rear_frame.visual(
        Box((0.022, 0.012, 0.022)),
        origin=Origin(xyz=(-0.181, 0.194, -0.620)),
        material=dark_steel,
        name="left_spreader_mount",
    )
    rear_frame.visual(
        Box((0.022, 0.012, 0.022)),
        origin=Origin(xyz=(-0.181, -0.194, -0.620)),
        material=dark_steel,
        name="right_spreader_mount",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.40, 0.40, 1.24)),
        mass=5.4,
        origin=Origin(xyz=(-0.185, 0.0, -0.620)),
    )

    left_front_spreader = model.part("left_front_spreader")
    _add_spreader_part(
        left_front_spreader,
        end=(-0.180, 0.0, 0.130),
        material=brace_gray,
        fastener_material=steel,
    )
    left_front_spreader.inertial = Inertial.from_geometry(
        Box((0.240, 0.016, 0.030)),
        mass=0.45,
        origin=Origin(xyz=(-0.100, 0.0, 0.060)),
    )

    right_front_spreader = model.part("right_front_spreader")
    _add_spreader_part(
        right_front_spreader,
        end=(-0.180, 0.0, 0.130),
        material=brace_gray,
        fastener_material=steel,
    )
    right_front_spreader.inertial = Inertial.from_geometry(
        Box((0.240, 0.016, 0.030)),
        mass=0.45,
        origin=Origin(xyz=(-0.100, 0.0, 0.060)),
    )

    left_rear_spreader = model.part("left_rear_spreader")
    _add_spreader_part(
        left_rear_spreader,
        end=(0.150, 0.0, 0.140),
        material=brace_gray,
        fastener_material=steel,
    )
    left_rear_spreader.inertial = Inertial.from_geometry(
        Box((0.140, 0.016, 0.032)),
        mass=0.38,
        origin=Origin(xyz=(0.050, 0.0, 0.070)),
    )

    right_rear_spreader = model.part("right_rear_spreader")
    _add_spreader_part(
        right_rear_spreader,
        end=(0.150, 0.0, 0.140),
        material=brace_gray,
        fastener_material=steel,
    )
    right_rear_spreader.inertial = Inertial.from_geometry(
        Box((0.140, 0.016, 0.032)),
        mass=0.38,
        origin=Origin(xyz=(0.050, 0.0, 0.070)),
    )

    model.articulation(
        "front_hinge",
        ArticulationType.REVOLUTE,
        parent=top_cap,
        child=front_frame,
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-0.08,
            upper=0.58,
        ),
    )
    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=top_cap,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-0.58,
            upper=0.08,
        ),
    )
    model.articulation(
        "left_front_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_front_spreader,
        origin=Origin(xyz=(0.150, 0.215, -0.610)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.05,
            upper=0.95,
        ),
    )
    model.articulation(
        "right_front_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_front_spreader,
        origin=Origin(xyz=(0.150, -0.215, -0.610)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.05,
            upper=0.95,
        ),
    )
    model.articulation(
        "left_rear_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=left_rear_spreader,
        origin=Origin(xyz=(-0.181, 0.203, -0.620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.85,
            upper=0.10,
        ),
    )
    model.articulation(
        "right_rear_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=right_rear_spreader,
        origin=Origin(xyz=(-0.181, -0.203, -0.620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.85,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    top_cap = object_model.get_part("top_cap")
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    left_front_spreader = object_model.get_part("left_front_spreader")
    right_front_spreader = object_model.get_part("right_front_spreader")
    left_rear_spreader = object_model.get_part("left_rear_spreader")
    right_rear_spreader = object_model.get_part("right_rear_spreader")

    front_hinge = object_model.get_articulation("front_hinge")
    rear_hinge = object_model.get_articulation("rear_hinge")
    left_front_spreader_hinge = object_model.get_articulation("left_front_spreader_hinge")
    right_front_spreader_hinge = object_model.get_articulation("right_front_spreader_hinge")
    left_rear_spreader_hinge = object_model.get_articulation("left_rear_spreader_hinge")
    right_rear_spreader_hinge = object_model.get_articulation("right_rear_spreader_hinge")

    top_platform = top_cap.get_visual("top_platform")
    front_hinge_boss_left = top_cap.get_visual("front_hinge_boss_left")
    front_hinge_boss_right = top_cap.get_visual("front_hinge_boss_right")
    rear_hinge_boss_left = top_cap.get_visual("rear_hinge_boss_left")
    rear_hinge_boss_right = top_cap.get_visual("rear_hinge_boss_right")

    front_left_rail = front_frame.get_visual("left_rail")
    front_right_rail = front_frame.get_visual("right_rail")
    front_left_foot = front_frame.get_visual("left_foot")
    front_right_foot = front_frame.get_visual("right_foot")
    front_left_hinge_pad = front_frame.get_visual("left_hinge_pad")
    front_right_hinge_pad = front_frame.get_visual("right_hinge_pad")
    step_02 = front_frame.get_visual("step_02")
    step_04 = front_frame.get_visual("step_04")

    rear_left_rail = rear_frame.get_visual("left_rail")
    rear_right_rail = rear_frame.get_visual("right_rail")
    rear_left_foot = rear_frame.get_visual("left_foot")
    rear_right_foot = rear_frame.get_visual("right_foot")
    rear_left_hinge_pad = rear_frame.get_visual("left_hinge_pad")
    rear_right_hinge_pad = rear_frame.get_visual("right_hinge_pad")

    lfs_pivot = left_front_spreader.get_visual("pivot_lug")
    lfs_inner = left_front_spreader.get_visual("inner_lug")
    rfs_pivot = right_front_spreader.get_visual("pivot_lug")
    rfs_inner = right_front_spreader.get_visual("inner_lug")
    lrs_pivot = left_rear_spreader.get_visual("pivot_lug")
    lrs_inner = left_rear_spreader.get_visual("inner_lug")
    rrs_pivot = right_rear_spreader.get_visual("pivot_lug")
    rrs_inner = right_rear_spreader.get_visual("inner_lug")

    def _elem_aabb(part, elem):
        return ctx.part_element_world_aabb(part, elem=elem)

    def _aabb_dims(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            maxs[0] - mins[0],
            maxs[1] - mins[1],
            maxs[2] - mins[2],
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            (mins[0] + maxs[0]) * 0.5,
            (mins[1] + maxs[1]) * 0.5,
            (mins[2] + maxs[2]) * 0.5,
        )

    def _gap_x(positive_aabb, negative_aabb):
        if positive_aabb is None or negative_aabb is None:
            return None
        return positive_aabb[0][0] - negative_aabb[1][0]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.check(
        "all_expected_parts_exist",
        all(
            part is not None
            for part in (
                top_cap,
                front_frame,
                rear_frame,
                left_front_spreader,
                right_front_spreader,
                left_rear_spreader,
                right_rear_spreader,
            )
        ),
        details="Expected top cap, both ladder frames, and all four spreader bars.",
    )

    ctx.expect_contact(
        top_cap,
        front_frame,
        elem_a=front_hinge_boss_left,
        elem_b=front_left_hinge_pad,
        contact_tol=1e-4,
        name="front_left_hinge_mounts_to_cap",
    )
    ctx.expect_contact(
        top_cap,
        front_frame,
        elem_a=front_hinge_boss_right,
        elem_b=front_right_hinge_pad,
        contact_tol=1e-4,
        name="front_right_hinge_mounts_to_cap",
    )
    ctx.expect_contact(
        top_cap,
        rear_frame,
        elem_a=rear_hinge_boss_left,
        elem_b=rear_left_hinge_pad,
        contact_tol=1e-4,
        name="rear_left_hinge_mounts_to_cap",
    )
    ctx.expect_contact(
        top_cap,
        rear_frame,
        elem_a=rear_hinge_boss_right,
        elem_b=rear_right_hinge_pad,
        contact_tol=1e-4,
        name="rear_right_hinge_mounts_to_cap",
    )

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="x",
        positive_elem=front_left_foot,
        negative_elem=rear_left_foot,
        min_gap=0.45,
        max_gap=0.62,
        name="left_open_stance_footprint_depth",
    )
    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="x",
        positive_elem=front_right_foot,
        negative_elem=rear_right_foot,
        min_gap=0.45,
        max_gap=0.62,
        name="right_open_stance_footprint_depth",
    )
    ctx.expect_gap(
        top_cap,
        front_frame,
        axis="z",
        positive_elem=top_platform,
        negative_elem=step_04,
        min_gap=0.17,
        max_gap=0.23,
        name="platform_sits_above_highest_tread",
    )

    ctx.expect_contact(
        front_frame,
        left_front_spreader,
        elem_a=front_left_rail,
        elem_b=lfs_pivot,
        contact_tol=0.002,
        name="left_front_spreader_pivot_contacts_front_rail",
    )
    ctx.expect_contact(
        rear_frame,
        left_rear_spreader,
        elem_a=rear_left_rail,
        elem_b=lrs_pivot,
        contact_tol=0.002,
        name="left_rear_spreader_pivot_contacts_rear_rail",
    )
    ctx.expect_contact(
        front_frame,
        right_front_spreader,
        elem_a=front_right_rail,
        elem_b=rfs_pivot,
        contact_tol=0.002,
        name="right_front_spreader_pivot_contacts_front_rail",
    )
    ctx.expect_contact(
        rear_frame,
        right_rear_spreader,
        elem_a=rear_right_rail,
        elem_b=rrs_pivot,
        contact_tol=0.002,
        name="right_rear_spreader_pivot_contacts_rear_rail",
    )
    ctx.expect_overlap(
        left_front_spreader,
        left_rear_spreader,
        axes="xz",
        elem_a=lfs_inner,
        elem_b=lrs_inner,
        min_overlap=0.015,
        name="left_spreader_center_pin_alignment",
    )
    ctx.expect_gap(
        left_front_spreader,
        left_rear_spreader,
        axis="y",
        positive_elem=lfs_inner,
        negative_elem=lrs_inner,
        min_gap=0.002,
        max_gap=0.020,
        name="left_spreader_bars_are_stacked_not_interpenetrating",
    )
    ctx.expect_overlap(
        right_front_spreader,
        right_rear_spreader,
        axes="xz",
        elem_a=rfs_inner,
        elem_b=rrs_inner,
        min_overlap=0.015,
        name="right_spreader_center_pin_alignment",
    )
    ctx.expect_gap(
        right_rear_spreader,
        right_front_spreader,
        axis="y",
        positive_elem=rrs_inner,
        negative_elem=rfs_inner,
        min_gap=0.002,
        max_gap=0.020,
        name="right_spreader_bars_are_stacked_not_interpenetrating",
    )

    for part, elem, label in (
        (front_frame, front_left_foot, "front_left_foot_on_ground"),
        (front_frame, front_right_foot, "front_right_foot_on_ground"),
        (rear_frame, rear_left_foot, "rear_left_foot_on_ground"),
        (rear_frame, rear_right_foot, "rear_right_foot_on_ground"),
    ):
        aabb = _elem_aabb(part, elem)
        ctx.check(
            label,
            aabb is not None and abs(aabb[0][2]) <= 1e-6,
            details=f"Expected {label} bottom z to sit on the ground plane, got {aabb}.",
        )

    platform_aabb = _elem_aabb(top_cap, top_platform)
    step_aabb = _elem_aabb(front_frame, step_02)
    step_dims = _aabb_dims(step_aabb)
    ctx.check(
        "platform_height_is_realistic",
        platform_aabb is not None and 1.21 <= platform_aabb[1][2] <= 1.24,
        details=f"Top platform should land around a 1.2 m working height, got {platform_aabb}.",
    )
    ctx.check(
        "middle_tread_is_practical_size",
        step_dims is not None and step_dims[0] >= 0.10 and step_dims[1] >= 0.36,
        details=f"Middle tread should be wide and deep enough for a real boot, got {step_dims}.",
    )

    open_left_gap = _gap_x(_elem_aabb(front_frame, front_left_foot), _elem_aabb(rear_frame, rear_left_foot))
    with ctx.pose({front_hinge: 0.20, rear_hinge: -0.20}):
        folded_left_gap = _gap_x(
            _elem_aabb(front_frame, front_left_foot),
            _elem_aabb(rear_frame, rear_left_foot),
        )
        folded_right_gap = _gap_x(
            _elem_aabb(front_frame, front_right_foot),
            _elem_aabb(rear_frame, rear_right_foot),
        )
        ctx.check(
            "hinges_reduce_ladder_stance_when_folded",
            open_left_gap is not None
            and folded_left_gap is not None
            and folded_right_gap is not None
            and folded_left_gap < open_left_gap - 0.20
            and folded_right_gap < 0.30,
            details=(
                "Expected the ladder to narrow appreciably when both main hinges move "
                f"toward folded pose, got open={open_left_gap}, folded_left={folded_left_gap}, "
                f"folded_right={folded_right_gap}."
            ),
        )

    for label, joint, part, elem, pose_value in (
        ("left_front_spreader_moves", left_front_spreader_hinge, left_front_spreader, lfs_inner, 0.55),
        ("right_front_spreader_moves", right_front_spreader_hinge, right_front_spreader, rfs_inner, 0.55),
        ("left_rear_spreader_moves", left_rear_spreader_hinge, left_rear_spreader, lrs_inner, -0.55),
        ("right_rear_spreader_moves", right_rear_spreader_hinge, right_rear_spreader, rrs_inner, -0.55),
    ):
        rest_center = _aabb_center(_elem_aabb(part, elem))
        with ctx.pose({joint: pose_value}):
            moved_center = _aabb_center(_elem_aabb(part, elem))
        moved = (
            rest_center is not None
            and moved_center is not None
            and math.dist(rest_center, moved_center) > 0.035
        )
        ctx.check(
            label,
            moved,
            details=f"Expected {label} to sweep through a meaningful arc, got rest={rest_center}, moved={moved_center}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
