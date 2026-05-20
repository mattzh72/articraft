from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _tube(points: list[tuple[float, float, float]], radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=10,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _add_transport_wheel(part, *, side: float, rubber, rim, hub_metal) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=0.260, length=0.055),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.195, length=0.044),
        origin=spin_origin,
        material=rim,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.060, length=0.064),
        origin=spin_origin,
        material=hub_metal,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.118, length=0.008),
        origin=Origin(xyz=(0.020 * side, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim,
        name="outer_disc",
    )
    part.visual(
        Cylinder(radius=0.035, length=0.014),
        origin=Origin(xyz=(0.026 * side, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_metal,
        name="cap",
    )


def _add_caster_wheel(part, *, rubber, hub_metal) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=spin_origin,
        material=hub_metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.003, length=0.004),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_metal,
        name="right_axle_stub",
    )
    part.visual(
        Cylinder(radius=0.003, length=0.004),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_metal,
        name="left_axle_stub",
    )


def _add_caster_fork(part, *, steel) -> None:
    part.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=steel,
        name="stem",
    )
    part.visual(
        Box((0.022, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=steel,
        name="crown",
    )
    part.visual(
        Box((0.004, 0.008, 0.016)),
        origin=Origin(xyz=(0.011, 0.0, -0.022)),
        material=steel,
        name="right_leg",
    )
    part.visual(
        Box((0.004, 0.008, 0.016)),
        origin=Origin(xyz=(-0.011, 0.0, -0.022)),
        material=steel,
        name="left_leg",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hybrid_hand_truck")

    frame_red = model.material("frame_red", rgba=(0.74, 0.13, 0.09, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.21, 0.22, 0.24, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.58, 0.38, 1.24)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.03, 0.62)),
    )

    left_rail_points = [
        (0.180, 0.008, 0.150),
        (0.188, 0.012, 0.520),
        (0.190, 0.010, 0.920),
        (0.178, -0.014, 1.120),
        (0.164, -0.020, 1.160),
    ]
    frame.visual(_tube(left_rail_points, 0.017, "left_frame_rail"), material=frame_red, name="left_rail")
    frame.visual(_tube(_mirror_x(left_rail_points), 0.017, "right_frame_rail"), material=frame_red, name="right_rail")
    frame.visual(
        Cylinder(radius=0.016, length=0.328),
        origin=Origin(xyz=(0.0, -0.020, 1.160), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="top_handle_bar",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.092),
        origin=Origin(xyz=(0.164, -0.063, 1.160), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.092),
        origin=Origin(xyz=(-0.164, -0.063, 1.160), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.372),
        origin=Origin(xyz=(0.0, 0.010, 0.830), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="upper_crossbrace",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.376),
        origin=Origin(xyz=(0.0, 0.010, 0.540), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="mid_crossbrace",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.350),
        origin=Origin(xyz=(0.0, 0.012, 0.160), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="lower_crossbrace",
    )
    frame.visual(
        Box((0.216, 0.028, 0.104)),
        origin=Origin(xyz=(0.0, -0.014, 0.112)),
        material=dark_steel,
        name="toe_support",
    )
    frame.visual(
        Box((0.380, 0.260, 0.012)),
        origin=Origin(xyz=(0.0, 0.130, 0.072)),
        material=bright_steel,
        name="toe_plate",
    )
    frame.visual(
        Box((0.360, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, 0.260, 0.0885)),
        material=bright_steel,
        name="toe_lip",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.416),
        origin=Origin(xyz=(0.0, -0.055, 0.260), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_bar",
    )
    axle_brace_points = [
        (0.182, 0.012, 0.150),
        (0.188, -0.020, 0.205),
        (0.190, -0.055, 0.260),
    ]
    frame.visual(_tube(axle_brace_points, 0.014, "left_axle_brace"), material=frame_red, name="left_axle_brace")
    frame.visual(_tube(_mirror_x(axle_brace_points), 0.014, "right_axle_brace"), material=frame_red, name="right_axle_brace")
    frame.visual(
        Box((0.040, 0.034, 0.086)),
        origin=Origin(xyz=(0.188, -0.043, 0.246)),
        material=dark_steel,
        name="left_wheel_bracket",
    )
    frame.visual(
        Box((0.040, 0.034, 0.086)),
        origin=Origin(xyz=(-0.188, -0.043, 0.246)),
        material=dark_steel,
        name="right_wheel_bracket",
    )
    frame.visual(
        Box((0.170, 0.023, 0.024)),
        origin=Origin(xyz=(0.0, -0.0095, 0.078)),
        material=dark_steel,
        name="hinge_mount",
    )
    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.260, length=0.055),
        mass=2.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_transport_wheel(left_wheel, side=1.0, rubber=rubber, rim=bright_steel, hub_metal=dark_steel)

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.260, length=0.055),
        mass=2.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_transport_wheel(right_wheel, side=-1.0, rubber=rubber, rim=bright_steel, hub_metal=dark_steel)

    caster_module = model.part("caster_module")
    caster_module.inertial = Inertial.from_geometry(
        Box((0.200, 0.140, 0.060)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.055, -0.015)),
    )
    caster_module.visual(
        Cylinder(radius=0.009, length=0.170),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_tube",
    )
    caster_module.visual(
        Box((0.014, 0.020, 0.026)),
        origin=Origin(xyz=(0.083, 0.002, -0.008)),
        material=dark_steel,
        name="left_hinge_link",
    )
    caster_module.visual(
        Box((0.014, 0.020, 0.026)),
        origin=Origin(xyz=(-0.083, 0.002, -0.008)),
        material=dark_steel,
        name="right_hinge_link",
    )
    caster_module.visual(
        Box((0.014, 0.110, 0.008)),
        origin=Origin(xyz=(0.083, 0.060, -0.022)),
        material=dark_steel,
        name="left_swing_arm",
    )
    caster_module.visual(
        Box((0.014, 0.110, 0.008)),
        origin=Origin(xyz=(-0.083, 0.060, -0.022)),
        material=dark_steel,
        name="right_swing_arm",
    )
    caster_module.visual(
        Box((0.194, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.110, -0.014)),
        material=dark_steel,
        name="front_crossmember",
    )
    caster_module.visual(
        Box((0.024, 0.014, 0.016)),
        origin=Origin(xyz=(0.090, 0.110, -0.022)),
        material=dark_steel,
        name="left_swivel_block",
    )
    caster_module.visual(
        Box((0.024, 0.014, 0.016)),
        origin=Origin(xyz=(-0.090, 0.110, -0.022)),
        material=dark_steel,
        name="right_swivel_block",
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.026, 0.008, 0.028)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )
    _add_caster_fork(left_caster_fork, steel=bright_steel)

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.026, 0.008, 0.028)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )
    _add_caster_fork(right_caster_fork, steel=bright_steel)

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.010),
        mass=0.18,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(left_caster_wheel, rubber=rubber, hub_metal=dark_steel)

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.010),
        mass=0.18,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(right_caster_wheel, rubber=rubber, hub_metal=dark_steel)

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(0.240, -0.055, 0.260)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(-0.240, -0.055, 0.260)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "caster_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=caster_module,
        origin=Origin(xyz=(0.0, -0.050, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.5, lower=-0.55, upper=0.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=caster_module,
        child=left_caster_fork,
        origin=Origin(xyz=(0.090, 0.110, -0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=caster_module,
        child=right_caster_fork,
        origin=Origin(xyz=(-0.090, 0.110, -0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    caster_module = object_model.get_part("caster_module")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")

    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    caster_fold = object_model.get_articulation("caster_fold")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_wheel_spin = object_model.get_articulation("left_caster_wheel_spin")
    right_caster_wheel_spin = object_model.get_articulation("right_caster_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.expect_origin_distance(left_wheel, right_wheel, axes="x", min_dist=0.46, max_dist=0.50)
    ctx.expect_overlap(left_wheel, frame, axes="z", min_overlap=0.15)
    ctx.expect_overlap(right_wheel, frame, axes="z", min_overlap=0.15)
    ctx.expect_within(caster_module, frame, axes="x", margin=0.04)
    ctx.expect_origin_distance(left_caster_fork, right_caster_fork, axes="x", min_dist=0.175, max_dist=0.205)
    ctx.expect_origin_distance(left_caster_wheel, right_caster_wheel, axes="x", min_dist=0.175, max_dist=0.205)

    ctx.check(
        "transport wheels spin about axle axis",
        left_wheel_spin.axis == (1.0, 0.0, 0.0) and right_wheel_spin.axis == (1.0, 0.0, 0.0),
        f"left={left_wheel_spin.axis}, right={right_wheel_spin.axis}",
    )
    ctx.check(
        "caster module folds on transverse hinge",
        caster_fold.axis == (1.0, 0.0, 0.0),
        f"caster_fold axis={caster_fold.axis}",
    )
    ctx.check(
        "caster forks swivel on vertical pivots",
        left_caster_swivel.axis == (0.0, 0.0, 1.0) and right_caster_swivel.axis == (0.0, 0.0, 1.0),
        f"left={left_caster_swivel.axis}, right={right_caster_swivel.axis}",
    )
    ctx.check(
        "caster wheels spin on local axles",
        left_caster_wheel_spin.axis == (1.0, 0.0, 0.0) and right_caster_wheel_spin.axis == (1.0, 0.0, 0.0),
        f"left={left_caster_wheel_spin.axis}, right={right_caster_wheel_spin.axis}",
    )

    folded_left = ctx.part_world_position(left_caster_wheel)
    folded_right = ctx.part_world_position(right_caster_wheel)
    with ctx.pose({caster_fold: -0.55}):
        deployed_left = ctx.part_world_position(left_caster_wheel)
        deployed_right = ctx.part_world_position(right_caster_wheel)

    fold_ok = (
        folded_left is not None
        and folded_right is not None
        and deployed_left is not None
        and deployed_right is not None
        and deployed_left[2] < folded_left[2] - 0.04
        and deployed_right[2] < folded_right[2] - 0.04
    )
    ctx.check(
        "folding module lowers both caster wheels",
        fold_ok,
        f"folded_left={folded_left}, deployed_left={deployed_left}, folded_right={folded_right}, deployed_right={deployed_right}",
    )

    with ctx.pose({caster_fold: 0.0}):
        ctx.expect_gap(
            frame,
            left_caster_wheel,
            axis="z",
            positive_elem="toe_plate",
            min_gap=0.005,
            max_gap=0.035,
            name="left caster wheel clears underside of toe plate",
        )
        ctx.expect_gap(
            frame,
            right_caster_wheel,
            axis="z",
            positive_elem="toe_plate",
            min_gap=0.005,
            max_gap=0.035,
            name="right caster wheel clears underside of toe plate",
        )
        ctx.expect_overlap(
            left_caster_wheel,
            frame,
            axes="xy",
            min_overlap=0.008,
            elem_b="toe_plate",
            name="left caster wheel tucks under toe plate footprint",
        )
        ctx.expect_overlap(
            right_caster_wheel,
            frame,
            axes="xy",
            min_overlap=0.008,
            elem_b="toe_plate",
            name="right caster wheel tucks under toe plate footprint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
