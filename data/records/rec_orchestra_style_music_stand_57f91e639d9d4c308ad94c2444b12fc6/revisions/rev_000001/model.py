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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rotate_x(
    point: tuple[float, float, float], angle: float
) -> tuple[float, float, float]:
    x, y, z = point
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (x, y * ca - z * sa, y * sa + z * ca)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_music_stand")

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.69, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    lower_support = model.part("lower_support")
    lower_support.visual(
        Cylinder(radius=0.052, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=powder_black,
        name="tripod_hub",
    )
    lower_support.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=powder_black,
        name="lower_collar",
    )

    outer_tube_shell = LatheGeometry.from_shell_profiles(
        outer_profile=((0.0170, 0.0), (0.0162, 0.020), (0.0162, 0.500), (0.0176, 0.520)),
        inner_profile=((0.0141, 0.002), (0.0133, 0.022), (0.0133, 0.498), (0.0146, 0.518)),
        segments=48,
    )
    lower_support.visual(
        mesh_from_geometry(outer_tube_shell, "music_stand_outer_tube_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=satin_graphite,
        name="outer_tube_shell",
    )
    clamp_collar_shell = LatheGeometry.from_shell_profiles(
        outer_profile=((0.023, 0.0), (0.023, 0.040)),
        inner_profile=((0.0185, 0.0015), (0.0185, 0.0385)),
        segments=40,
    )
    lower_support.visual(
        mesh_from_geometry(clamp_collar_shell, "music_stand_clamp_collar_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=powder_black,
        name="clamp_collar",
    )
    lower_support.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(
            xyz=(0.025, 0.0, 0.620),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="clamp_screw",
    )
    lower_support.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.042, 0.0, 0.620)),
        material=powder_black,
        name="clamp_knob",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.028 * c, 0.028 * s, 0.108),
                (0.155 * c, 0.155 * s, 0.070),
                (0.340 * c, 0.340 * s, 0.018),
            ],
            radius=0.0088,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        lower_support.visual(
            mesh_from_geometry(leg_mesh, f"music_stand_leg_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        lower_support.visual(
            Sphere(radius=0.0115),
            origin=Origin(xyz=(0.340 * c, 0.340 * s, 0.018)),
            material=rubber,
            name=f"foot_{index}",
        )

    lower_support.inertial = Inertial.from_geometry(
        Box((0.74, 0.74, 0.68)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0110, length=1.000),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=steel,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.645)),
        material=powder_black,
        name="head_collar",
    )
    mast.visual(
        Box((0.082, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, -0.020, 0.670)),
        material=powder_black,
        name="desk_yoke_bridge",
    )
    mast.visual(
        Box((0.006, 0.020, 0.052)),
        origin=Origin(xyz=(-0.035, -0.010, 0.670)),
        material=powder_black,
        name="mast_left_ear",
    )
    mast.visual(
        Box((0.006, 0.020, 0.052)),
        origin=Origin(xyz=(0.035, -0.010, 0.670)),
        material=powder_black,
        name="mast_right_ear",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.090, 0.050, 1.040)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.004, 0.150)),
    )

    model.articulation(
        "lower_support_to_mast",
        ArticulationType.PRISMATIC,
        parent=lower_support,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.22,
            lower=0.0,
            upper=0.240,
        ),
    )

    desk = model.part("desk")
    desk_pitch = 0.22
    desk_wire_radius = 0.0042

    desk.visual(
        Cylinder(radius=0.009, length=0.064),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_graphite,
        name="desk_hinge_barrel",
    )
    desk_segments = [
        ((-0.016, 0.0, 0.0), (-0.016, 0.0, 0.034), "desk_hanger_left_stem"),
        ((0.016, 0.0, 0.0), (0.016, 0.0, 0.034), "desk_hanger_right_stem"),
        ((-0.016, 0.0, 0.034), (-0.145, 0.0, 0.095), "desk_hanger_left"),
        ((0.016, 0.0, 0.034), (0.145, 0.0, 0.095), "desk_hanger_right"),
        (
            (-0.270, 0.0, -0.140),
            (0.270, 0.0, -0.140),
            "desk_lower_rail",
        ),
        (
            (-0.270, 0.0, 0.220),
            (0.270, 0.0, 0.220),
            "desk_top_rail",
        ),
        (
            (-0.270, 0.0, -0.140),
            (-0.270, 0.0, 0.220),
            "desk_left_rail",
        ),
        (
            (0.270, 0.0, -0.140),
            (0.270, 0.0, 0.220),
            "desk_right_rail",
        ),
        ((-0.060, 0.0, -0.090), (-0.060, 0.0, 0.220), None),
        ((0.060, 0.0, -0.090), (0.060, 0.0, 0.220), None),
        ((-0.245, 0.0, 0.095), (0.245, 0.0, 0.095), None),
        ((-0.245, 0.0, 0.010), (-0.088, 0.0, 0.010), None),
        ((0.088, 0.0, 0.010), (0.245, 0.0, 0.010), None),
        ((-0.215, 0.0, 0.165), (0.215, 0.0, 0.165), None),
        ((-0.240, 0.0, -0.045), (0.0, 0.0, 0.140), None),
        ((0.240, 0.0, -0.045), (0.0, 0.0, 0.140), None),
    ]
    for start, end, name in desk_segments:
        _add_member(
            desk,
            _rotate_x(start, desk_pitch),
            _rotate_x(end, desk_pitch),
            radius=desk_wire_radius,
            material=satin_graphite,
            name=name,
        )

    desk.visual(
        Box((0.006, 0.020, 0.034)),
        origin=Origin(
            xyz=_rotate_x((-0.031, -0.010, -0.145), desk_pitch),
            rpy=(desk_pitch, 0.0, 0.0),
        ),
        material=powder_black,
        name="desk_shelf_left_ear",
    )
    desk.visual(
        Box((0.006, 0.020, 0.034)),
        origin=Origin(
            xyz=_rotate_x((0.031, -0.010, -0.145), desk_pitch),
            rpy=(desk_pitch, 0.0, 0.0),
        ),
        material=powder_black,
        name="desk_shelf_right_ear",
    )
    desk.inertial = Inertial.from_geometry(
        Box((0.560, 0.120, 0.390)),
        mass=1.3,
        origin=Origin(xyz=(0.0, -0.010, 0.020)),
    )

    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=-0.45,
            upper=0.55,
        ),
    )

    shelf = model.part("shelf")
    shelf_pitch = -1.02
    shelf_wire_radius = 0.0038
    shelf_width = 0.380
    shelf_depth = 0.070

    shelf.visual(
        Cylinder(radius=0.0075, length=0.056),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_graphite,
        name="shelf_hinge_barrel",
    )

    shelf_rear_y = 0.016
    shelf_barrel_half = 0.028
    shelf_segments = [
        (
            (-0.018, 0.0, 0.0),
            (-0.018, 0.018, -0.018),
            "shelf_left_hinge_drop",
        ),
        (
            (0.018, 0.0, 0.0),
            (0.018, 0.018, -0.018),
            "shelf_right_hinge_drop",
        ),
        (
            (-0.018, 0.018, -0.018),
            (-shelf_width * 0.5, shelf_rear_y, 0.0),
            "shelf_left_support_arm",
        ),
        (
            (0.018, 0.018, -0.018),
            (shelf_width * 0.5, shelf_rear_y, 0.0),
            "shelf_right_support_arm",
        ),
        (
            (-shelf_width * 0.5, shelf_depth, 0.0),
            (shelf_width * 0.5, shelf_depth, 0.0),
            "shelf_front_rail",
        ),
        (
            (-shelf_width * 0.5, shelf_rear_y, 0.0),
            (-shelf_width * 0.5, shelf_depth, 0.0),
            None,
        ),
        (
            (shelf_width * 0.5, shelf_rear_y, 0.0),
            (shelf_width * 0.5, shelf_depth, 0.0),
            None,
        ),
        ((-0.090, shelf_rear_y, 0.0), (-0.090, shelf_depth, 0.0), None),
        ((0.090, shelf_rear_y, 0.0), (0.090, shelf_depth, 0.0), None),
        (
            (-shelf_width * 0.5, shelf_depth, 0.0),
            (-shelf_width * 0.5, shelf_depth, 0.018),
            None,
        ),
        (
            (shelf_width * 0.5, shelf_depth, 0.0),
            (shelf_width * 0.5, shelf_depth, 0.018),
            None,
        ),
        (
            (-shelf_width * 0.5, shelf_depth, 0.018),
            (shelf_width * 0.5, shelf_depth, 0.018),
            "shelf_music_stop",
        ),
    ]
    for start, end, name in shelf_segments:
        _add_member(
            shelf,
            _rotate_x(start, shelf_pitch),
            _rotate_x(end, shelf_pitch),
            radius=shelf_wire_radius,
            material=satin_graphite,
            name=name,
        )

    shelf.inertial = Inertial.from_geometry(
        Box((0.400, 0.100, 0.120)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.015, -0.025)),
    )

    shelf_hinge_origin = _rotate_x((0.0, -0.010, -0.145), desk_pitch)
    model.articulation(
        "desk_to_shelf",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=shelf,
        origin=Origin(xyz=shelf_hinge_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.6,
            lower=-0.20,
            upper=1.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_support = object_model.get_part("lower_support")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    shelf = object_model.get_part("shelf")

    slide = object_model.get_articulation("lower_support_to_mast")
    desk_tilt = object_model.get_articulation("mast_to_desk")
    shelf_fold = object_model.get_articulation("desk_to_shelf")

    for name, part in (
        ("lower support exists", lower_support),
        ("mast exists", mast),
        ("desk exists", desk),
        ("shelf exists", shelf),
    ):
        ctx.check(name, part is not None)

    ctx.expect_within(
        mast,
        lower_support,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_tube_shell",
        margin=0.0015,
        name="mast stays centered in lower support tube at rest",
    )
    ctx.expect_overlap(
        mast,
        lower_support,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_tube_shell",
        min_overlap=0.340,
        name="mast remains inserted in lower support tube at rest",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            mast,
            lower_support,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_tube_shell",
            margin=0.0015,
            name="mast stays centered in lower support tube when extended",
        )
        ctx.expect_overlap(
            mast,
            lower_support,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_tube_shell",
            min_overlap=0.080,
            name="mast retains insertion when extended",
        )
        extended_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast extends upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.20,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    ctx.expect_gap(
        desk,
        mast,
        axis="x",
        positive_elem="desk_hinge_barrel",
        negative_elem="mast_left_ear",
        min_gap=0.0,
        max_gap=0.0005,
        name="desk hinge barrel seats against left mast ear",
    )
    ctx.expect_gap(
        mast,
        desk,
        axis="x",
        positive_elem="mast_right_ear",
        negative_elem="desk_hinge_barrel",
        min_gap=0.0,
        max_gap=0.0005,
        name="desk hinge barrel seats against right mast ear",
    )

    top_bar_rest = ctx.part_element_world_aabb(desk, elem="desk_top_rail")
    with ctx.pose({desk_tilt: desk_tilt.motion_limits.lower}):
        top_bar_forward = ctx.part_element_world_aabb(desk, elem="desk_top_rail")
    with ctx.pose({desk_tilt: desk_tilt.motion_limits.upper}):
        top_bar_back = ctx.part_element_world_aabb(desk, elem="desk_top_rail")
    ctx.check(
        "desk tilt changes top rail position",
        top_bar_forward is not None
        and top_bar_back is not None
        and abs(top_bar_forward[0][1] - top_bar_back[0][1]) > 0.12,
        details=f"forward={top_bar_forward}, back={top_bar_back}",
    )

    ctx.expect_gap(
        shelf,
        desk,
        axis="x",
        positive_elem="shelf_hinge_barrel",
        negative_elem="desk_shelf_left_ear",
        min_gap=0.0,
        max_gap=0.0005,
        name="shelf hinge barrel seats against left desk ear",
    )
    ctx.expect_gap(
        desk,
        shelf,
        axis="x",
        positive_elem="desk_shelf_right_ear",
        negative_elem="shelf_hinge_barrel",
        min_gap=0.0,
        max_gap=0.0005,
        name="shelf hinge barrel seats against right desk ear",
    )

    shelf_front_open = ctx.part_element_world_aabb(shelf, elem="shelf_front_rail")
    with ctx.pose({shelf_fold: shelf_fold.motion_limits.upper}):
        shelf_front_folded = ctx.part_element_world_aabb(shelf, elem="shelf_front_rail")
    ctx.check(
        "shelf folds toward the desk",
        shelf_front_open is not None
        and shelf_front_folded is not None
        and shelf_front_folded[0][1] > shelf_front_open[0][1] + 0.02
        and shelf_front_folded[0][2] > shelf_front_open[0][2] + 0.03,
        details=f"open={shelf_front_open}, folded={shelf_front_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
