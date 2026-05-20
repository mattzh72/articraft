from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


ASSETS = AssetContext.from_script(__file__)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _arm_path(side_sign: float) -> list[tuple[float, float, float]]:
    return [
        (-side_sign * 0.13, 0.39, -0.03),
        (-side_sign * 0.09, 0.28, -0.05),
        (-side_sign * 0.02, 0.10, -0.085),
        (side_sign * 0.06, -0.05, -0.08),
        (side_sign * 0.15, -0.17, -0.05),
        (side_sign * 0.21, -0.23, -0.03),
    ]


def _tire_mesh(name: str, *, tire_radius: float, wheel_width: float, inner_radius: float):
    half_width = wheel_width * 0.5
    profile = [
        (inner_radius * 1.02, -half_width * 0.92),
        (tire_radius * 0.84, -half_width),
        (tire_radius * 0.96, -half_width * 0.64),
        (tire_radius, -half_width * 0.22),
        (tire_radius, half_width * 0.22),
        (tire_radius * 0.96, half_width * 0.64),
        (tire_radius * 0.84, half_width),
        (inner_radius * 1.02, half_width * 0.92),
        (inner_radius * 0.82, half_width * 0.34),
        (inner_radius * 0.74, 0.0),
        (inner_radius * 0.82, -half_width * 0.34),
        (inner_radius * 1.02, -half_width * 0.92),
    ]
    return _save_mesh(name, LatheGeometry(profile, segments=64).rotate_y(pi / 2.0))


def _add_trailing_arm(part, *, side_sign: float, arm_mesh, steel, rubber) -> None:
    hub_contact_plane = (side_sign * 0.21, -0.23, -0.03)
    part.visual(
        Box((0.05, 0.08, 0.10)),
        origin=Origin(xyz=(side_sign * 0.025, 0.0, 0.0)),
        material=steel,
        name="beam_saddle",
    )
    part.visual(
        Cylinder(radius=0.050, length=0.10),
        origin=Origin(xyz=(-side_sign * 0.13, 0.39, -0.03), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="front_bushing",
    )
    part.visual(arm_mesh, material=steel, name="main_tube")
    part.visual(
        Box((0.12, 0.12, 0.10)),
        origin=Origin(xyz=(side_sign * 0.15, -0.18, -0.02)),
        material=steel,
        name="hub_carrier",
    )
    part.visual(
        Cylinder(radius=0.074, length=0.012),
        origin=Origin(
            xyz=(hub_contact_plane[0] + side_sign * 0.006, hub_contact_plane[1], hub_contact_plane[2]),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="hub_flange",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.090),
        origin=Origin(
            xyz=(hub_contact_plane[0] + side_sign * 0.045, hub_contact_plane[1], hub_contact_plane[2]),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="spindle",
    )


def _add_wheel(part, *, side_sign: float, tire_mesh, rubber, wheel_silver, dark_steel) -> None:
    wheel_width = 0.215
    tire_radius = 0.310
    tire_center_x = side_sign * 0.095
    spin_origin = Origin(xyz=(tire_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0))

    part.visual(tire_mesh, origin=Origin(xyz=(tire_center_x, 0.0, 0.0)), material=rubber, name="tire")
    part.visual(Cylinder(radius=0.225, length=0.175), origin=spin_origin, material=wheel_silver, name="rim")
    part.visual(Cylinder(radius=0.155, length=0.125), origin=spin_origin, material=dark_steel, name="inner_barrel")
    part.visual(
        Cylinder(radius=0.095, length=0.082),
        origin=Origin(xyz=(side_sign * 0.041, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(-side_sign * 0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_silver,
        name="mount_pad",
    )
    part.visual(
        Cylinder(radius=0.044, length=0.028),
        origin=Origin(xyz=(side_sign * 0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="center_hat",
    )
    part.visual(
        Cylinder(radius=0.070, length=0.008),
        origin=Origin(xyz=(side_sign * 0.1865, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_silver,
        name="outer_cap",
    )
    part.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(side_sign * 0.162, 0.0, 0.229), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="valve_stem",
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trailing_arm_rear_axle", assets=ASSETS)

    beam_steel = model.material("beam_steel", rgba=(0.38, 0.41, 0.45, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.71, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    bushing_rubber = model.material("bushing_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    left_arm_mesh = _save_mesh(
        "rear_axle_left_trailing_arm_tube.obj",
        tube_from_spline_points(
            _arm_path(-1.0),
            radius=0.038,
            samples_per_segment=18,
            radial_segments=20,
        ),
    )
    right_arm_mesh = _save_mesh(
        "rear_axle_right_trailing_arm_tube.obj",
        tube_from_spline_points(
            _arm_path(1.0),
            radius=0.038,
            samples_per_segment=18,
            radial_segments=20,
        ),
    )
    tire_mesh = _tire_mesh(
        "rear_axle_tire.obj",
        tire_radius=0.310,
        wheel_width=0.215,
        inner_radius=0.158,
    )

    beam = model.part("torsion_beam")
    beam.visual(
        Cylinder(radius=0.045, length=0.74),
        origin=Origin(xyz=(0.0, -0.03, 0.34), rpy=(0.0, pi / 2.0, 0.0)),
        material=beam_steel,
        name="crossmember",
    )
    beam.visual(
        Box((0.12, 0.08, 0.10)),
        origin=Origin(xyz=(-0.43, 0.0, 0.35)),
        material=beam_steel,
        name="left_receiver",
    )
    beam.visual(
        Box((0.12, 0.08, 0.10)),
        origin=Origin(xyz=(0.43, 0.0, 0.35)),
        material=beam_steel,
        name="right_receiver",
    )
    beam.visual(
        Box((0.28, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, -0.02, 0.285)),
        material=dark_steel,
        name="center_stiffener",
    )
    beam.inertial = Inertial.from_geometry(
        Box((1.08, 0.22, 0.16)),
        mass=30.0,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
    )

    left_arm = model.part("left_trailing_arm")
    _add_trailing_arm(left_arm, side_sign=-1.0, arm_mesh=left_arm_mesh, steel=beam_steel, rubber=bushing_rubber)
    left_arm.inertial = Inertial.from_geometry(
        Box((0.40, 0.76, 0.20)),
        mass=17.0,
        origin=Origin(xyz=(-0.03, 0.08, -0.01)),
    )

    right_arm = model.part("right_trailing_arm")
    _add_trailing_arm(right_arm, side_sign=1.0, arm_mesh=right_arm_mesh, steel=beam_steel, rubber=bushing_rubber)
    right_arm.inertial = Inertial.from_geometry(
        Box((0.40, 0.76, 0.20)),
        mass=17.0,
        origin=Origin(xyz=(0.03, 0.08, -0.01)),
    )

    left_wheel = model.part("left_wheel")
    _add_wheel(left_wheel, side_sign=-1.0, tire_mesh=tire_mesh, rubber=rubber, wheel_silver=wheel_silver, dark_steel=dark_steel)
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.310, length=0.215),
        mass=14.0,
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _add_wheel(right_wheel, side_sign=1.0, tire_mesh=tire_mesh, rubber=rubber, wheel_silver=wheel_silver, dark_steel=dark_steel)
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.310, length=0.215),
        mass=14.0,
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "beam_to_left_arm",
        ArticulationType.FIXED,
        parent=beam,
        child=left_arm,
        origin=Origin(xyz=(-0.445, 0.0, 0.35)),
    )
    model.articulation(
        "beam_to_right_arm",
        ArticulationType.FIXED,
        parent=beam,
        child=right_arm,
        origin=Origin(xyz=(0.445, 0.0, 0.35)),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.REVOLUTE,
        parent=left_arm,
        child=left_wheel,
        origin=Origin(xyz=(-0.21, -0.23, -0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=28.0, lower=-2.0 * pi, upper=2.0 * pi),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.REVOLUTE,
        parent=right_arm,
        child=right_wheel,
        origin=Origin(xyz=(0.21, -0.23, -0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=28.0, lower=-2.0 * pi, upper=2.0 * pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    beam = object_model.get_part("torsion_beam")
    left_arm = object_model.get_part("left_trailing_arm")
    right_arm = object_model.get_part("right_trailing_arm")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    crossmember = beam.get_visual("crossmember")
    left_receiver = beam.get_visual("left_receiver")
    right_receiver = beam.get_visual("right_receiver")
    left_saddle = left_arm.get_visual("beam_saddle")
    right_saddle = right_arm.get_visual("beam_saddle")
    left_main = left_arm.get_visual("main_tube")
    right_main = right_arm.get_visual("main_tube")
    left_bushing = left_arm.get_visual("front_bushing")
    right_bushing = right_arm.get_visual("front_bushing")
    left_flange = left_arm.get_visual("hub_flange")
    right_flange = right_arm.get_visual("hub_flange")
    left_pad = left_wheel.get_visual("mount_pad")
    right_pad = right_wheel.get_visual("mount_pad")

    ctx.allow_overlap(
        left_arm,
        left_wheel,
        reason="The left spindle intentionally nests inside the wheel center bore and hub shell.",
    )
    ctx.allow_overlap(
        right_arm,
        right_wheel,
        reason="The right spindle intentionally nests inside the wheel center bore and hub shell.",
    )
    ctx.allow_overlap(
        beam,
        left_arm,
        reason="The left inboard saddle shell is welded into the torsion beam receiver sleeve.",
        elem_a="left_receiver",
        elem_b="beam_saddle",
    )
    ctx.allow_overlap(
        beam,
        right_arm,
        reason="The right inboard saddle shell is welded into the torsion beam receiver sleeve.",
        elem_a="right_receiver",
        elem_b="beam_saddle",
    )
    ctx.allow_overlap(
        beam,
        left_arm,
        reason="The left tubular arm is represented as sleeved into the receiver box for a welded junction.",
        elem_a="left_receiver",
        elem_b="main_tube",
    )
    ctx.allow_overlap(
        beam,
        right_arm,
        reason="The right tubular arm is represented as sleeved into the receiver box for a welded junction.",
        elem_a="right_receiver",
        elem_b="main_tube",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_overlap(
        beam,
        left_arm,
        axes="x",
        min_overlap=0.035,
        elem_a=left_receiver,
        elem_b=left_saddle,
        name="left_arm_inserted_into_receiver",
    )
    ctx.expect_overlap(
        beam,
        left_arm,
        axes="yz",
        min_overlap=0.045,
        elem_a=left_receiver,
        elem_b=left_saddle,
        name="left_arm_receiver_overlap",
    )
    ctx.expect_overlap(
        beam,
        right_arm,
        axes="x",
        min_overlap=0.035,
        elem_a=right_receiver,
        elem_b=right_saddle,
        name="right_arm_inserted_into_receiver",
    )
    ctx.expect_overlap(
        beam,
        right_arm,
        axes="yz",
        min_overlap=0.045,
        elem_a=right_receiver,
        elem_b=right_saddle,
        name="right_arm_receiver_overlap",
    )
    ctx.expect_gap(
        left_arm,
        beam,
        axis="y",
        min_gap=0.26,
        positive_elem=left_bushing,
        negative_elem=crossmember,
        name="left_bushing_forward_of_beam",
    )
    ctx.expect_gap(
        right_arm,
        beam,
        axis="y",
        min_gap=0.26,
        positive_elem=right_bushing,
        negative_elem=crossmember,
        name="right_bushing_forward_of_beam",
    )
    ctx.expect_gap(
        left_wheel,
        left_arm,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_pad,
        negative_elem=left_flange,
        name="left_wheel_on_flange",
    )
    ctx.expect_overlap(
        left_wheel,
        left_arm,
        axes="yz",
        min_overlap=0.09,
        elem_a=left_pad,
        elem_b=left_flange,
        name="left_wheel_flange_footprint",
    )
    ctx.expect_gap(
        right_arm,
        right_wheel,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_flange,
        negative_elem=right_pad,
        name="right_wheel_on_flange",
    )
    ctx.expect_overlap(
        right_wheel,
        right_arm,
        axes="yz",
        min_overlap=0.09,
        elem_a=right_pad,
        elem_b=right_flange,
        name="right_wheel_flange_footprint",
    )
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="y", max_dist=0.001, name="wheels_share_axle_line_y")
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="z", max_dist=0.001, name="wheels_share_axle_line_z")
    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="x",
        min_dist=1.20,
        max_dist=1.40,
        name="rear_track_width_realistic",
    )

    beam_aabb = ctx.part_world_aabb(beam)
    left_wheel_aabb = ctx.part_world_aabb(left_wheel)
    right_wheel_aabb = ctx.part_world_aabb(right_wheel)
    if beam_aabb is not None:
        beam_width = beam_aabb[1][0] - beam_aabb[0][0]
        ctx.check("beam_width_realistic", 0.90 <= beam_width <= 1.05, f"beam width was {beam_width:.3f} m")
    if left_wheel_aabb is not None:
        left_radius = (left_wheel_aabb[1][2] - left_wheel_aabb[0][2]) * 0.5
        ctx.check("wheel_radius_realistic", 0.29 <= left_radius <= 0.33, f"wheel radius was {left_radius:.3f} m")
    if left_wheel_aabb is not None and right_wheel_aabb is not None:
        ground_delta = abs(left_wheel_aabb[0][2] - right_wheel_aabb[0][2])
        ctx.check("wheel_ground_plane_match", ground_delta <= 0.001, f"wheel bottom delta was {ground_delta:.6f} m")

    left_valve_rest = _aabb_center(ctx.part_element_world_aabb(left_wheel, elem="valve_stem"))
    right_valve_rest = _aabb_center(ctx.part_element_world_aabb(right_wheel, elem="valve_stem"))

    with ctx.pose({left_spin: pi / 2.0, right_spin: -pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_new_overlaps")
        ctx.fail_if_isolated_parts(name="wheel_spin_pose_no_floating")
        ctx.expect_gap(
            left_wheel,
            left_arm,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=left_pad,
            negative_elem=left_flange,
            name="left_wheel_stays_on_flange_when_spun",
        )
        ctx.expect_gap(
            right_arm,
            right_wheel,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=right_flange,
            negative_elem=right_pad,
            name="right_wheel_stays_on_flange_when_spun",
        )
        left_valve_spun = _aabb_center(ctx.part_element_world_aabb(left_wheel, elem="valve_stem"))
        right_valve_spun = _aabb_center(ctx.part_element_world_aabb(right_wheel, elem="valve_stem"))
        if left_valve_rest is not None and left_valve_spun is not None:
            travel = abs(left_valve_spun[1] - left_valve_rest[1]) + abs(left_valve_spun[2] - left_valve_rest[2])
            ctx.check("left_wheel_revolute_motion_visible", travel >= 0.20, f"left valve travel was {travel:.3f} m")
        if right_valve_rest is not None and right_valve_spun is not None:
            travel = abs(right_valve_spun[1] - right_valve_rest[1]) + abs(right_valve_spun[2] - right_valve_rest[2])
            ctx.check("right_wheel_revolute_motion_visible", travel >= 0.20, f"right valve travel was {travel:.3f} m")

    for joint in (left_spin, right_spin):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END
