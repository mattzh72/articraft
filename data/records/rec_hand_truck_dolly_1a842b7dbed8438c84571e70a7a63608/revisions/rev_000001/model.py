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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_wheel_visuals(
    part,
    *,
    tire_radius: float,
    tire_width: float,
    rim_radius: float,
    hub_radius: float,
    rim_material,
    hub_material,
    tire_material,
    prefix: str,
) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=tire_material,
        name=f"{prefix}_tire",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=tire_width * 0.74),
        origin=spin_origin,
        material=rim_material,
        name=f"{prefix}_rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=tire_width * 0.34),
        origin=spin_origin,
        material=hub_material,
        name=f"{prefix}_hub",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.42, length=tire_width * 0.12),
        origin=spin_origin,
        material=rim_material,
        name=f"{prefix}_hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convertible_dolly")

    industrial_yellow = model.material("industrial_yellow", rgba=(0.90, 0.69, 0.14, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_black = model.material("grip_black", rgba=(0.13, 0.13, 0.14, 1.0))

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.018, length=1.00),
        origin=Origin(xyz=(0.13, 0.0, 0.73)),
        material=industrial_yellow,
        name="left_upright",
    )
    frame.visual(
        Cylinder(radius=0.018, length=1.00),
        origin=Origin(xyz=(-0.13, 0.0, 0.73)),
        material=industrial_yellow,
        name="right_upright",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 1.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=industrial_yellow,
        name="top_handle_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(0.13, 0.0, 1.28)),
        material=grip_black,
        name="left_handle_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(-0.13, 0.0, 1.28)),
        material=grip_black,
        name="right_handle_grip",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.86), rpy=(0.0, pi / 2.0, 0.0)),
        material=industrial_yellow,
        name="mid_crossbrace",
    )
    frame.visual(
        Box((0.26, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=industrial_yellow,
        name="lower_crossbrace",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.32),
        origin=Origin(xyz=(0.0, -0.06, 0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_axle_beam",
    )
    frame.visual(
        Box((0.06, 0.06, 0.14)),
        origin=Origin(xyz=(0.16, -0.03, 0.23)),
        material=dark_steel,
        name="left_axle_plate",
    )
    frame.visual(
        Box((0.06, 0.06, 0.14)),
        origin=Origin(xyz=(-0.16, -0.03, 0.23)),
        material=dark_steel,
        name="right_axle_plate",
    )
    frame.visual(
        Box((0.04, 0.03, 0.41)),
        origin=Origin(xyz=(0.13, 0.015, 0.355)),
        material=industrial_yellow,
        name="left_plate_support",
    )
    frame.visual(
        Box((0.04, 0.03, 0.41)),
        origin=Origin(xyz=(-0.13, 0.015, 0.355)),
        material=industrial_yellow,
        name="right_plate_support",
    )
    frame.visual(
        Box((0.38, 0.18, 0.008)),
        origin=Origin(xyz=(0.0, 0.10, 0.155)),
        material=industrial_yellow,
        name="main_nose_plate",
    )
    frame.visual(
        Box((0.38, 0.012, 0.03)),
        origin=Origin(xyz=(0.0, 0.184, 0.144)),
        material=industrial_yellow,
        name="nose_plate_front_lip",
    )
    frame.visual(
        Box((0.012, 0.18, 0.025)),
        origin=Origin(xyz=(0.184, 0.10, 0.1465)),
        material=industrial_yellow,
        name="left_nose_plate_lip",
    )
    frame.visual(
        Box((0.012, 0.18, 0.025)),
        origin=Origin(xyz=(-0.184, 0.10, 0.1465)),
        material=industrial_yellow,
        name="right_nose_plate_lip",
    )
    frame.visual(
        Box((0.12, 0.010, 0.05)),
        origin=Origin(xyz=(0.0, 0.015, 0.167)),
        material=industrial_yellow,
        name="center_nose_plate_rear_flange",
    )
    frame.visual(
        Box((0.020, 0.008, 0.034)),
        origin=Origin(xyz=(0.10, 0.038, 0.198)),
        material=dark_steel,
        name="left_front_keeper",
    )
    frame.visual(
        Box((0.020, 0.008, 0.034)),
        origin=Origin(xyz=(0.10, 0.012, 0.198)),
        material=dark_steel,
        name="left_rear_keeper",
    )
    frame.visual(
        Box((0.020, 0.008, 0.034)),
        origin=Origin(xyz=(-0.10, 0.038, 0.198)),
        material=dark_steel,
        name="right_front_keeper",
    )
    frame.visual(
        Box((0.020, 0.008, 0.034)),
        origin=Origin(xyz=(-0.10, 0.012, 0.198)),
        material=dark_steel,
        name="right_rear_keeper",
    )
    frame.visual(
        Box((0.24, 0.03, 0.012)),
        origin=Origin(xyz=(0.0, 0.025, 0.221)),
        material=dark_steel,
        name="hinge_bridge",
    )

    left_rear_wheel = model.part("left_rear_wheel")
    _add_wheel_visuals(
        left_rear_wheel,
        tire_radius=0.135,
        tire_width=0.055,
        rim_radius=0.095,
        hub_radius=0.044,
        rim_material=bright_steel,
        hub_material=dark_steel,
        tire_material=rubber,
        prefix="left_rear",
    )

    right_rear_wheel = model.part("right_rear_wheel")
    _add_wheel_visuals(
        right_rear_wheel,
        tire_radius=0.135,
        tire_width=0.055,
        rim_radius=0.095,
        hub_radius=0.044,
        rim_material=bright_steel,
        hub_material=dark_steel,
        tire_material=rubber,
        prefix="right_rear",
    )

    caster_bar = model.part("caster_bar")
    caster_bar.visual(
        Box((0.020, 0.018, 0.036)),
        origin=Origin(xyz=(0.10, 0.0, -0.010)),
        material=dark_steel,
        name="left_hinge_knuckle",
    )
    caster_bar.visual(
        Box((0.020, 0.018, 0.036)),
        origin=Origin(xyz=(-0.10, 0.0, -0.010)),
        material=dark_steel,
        name="right_hinge_knuckle",
    )
    caster_bar.visual(
        Box((0.24, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, 0.225, -0.020)),
        material=industrial_yellow,
        name="front_caster_crossbar",
    )
    caster_bar.visual(
        Box((0.024, 0.19, 0.024)),
        origin=Origin(xyz=(0.08, 0.095, -0.020)),
        material=industrial_yellow,
        name="left_caster_arm",
    )
    caster_bar.visual(
        Box((0.024, 0.19, 0.024)),
        origin=Origin(xyz=(-0.08, 0.095, -0.020)),
        material=industrial_yellow,
        name="right_caster_arm",
    )
    caster_bar.visual(
        Box((0.16, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.110, -0.020)),
        material=dark_steel,
        name="center_tie_bar",
    )
    for side, x_pos in (("left", 0.08), ("right", -0.08)):
        caster_bar.visual(
            Box((0.040, 0.040, 0.008)),
            origin=Origin(xyz=(x_pos, 0.225, -0.030)),
            material=dark_steel,
            name=f"{side}_swivel_socket",
        )
        caster_bar.visual(
            Box((0.030, 0.030, 0.018)),
            origin=Origin(xyz=(x_pos, 0.195, -0.030)),
            material=dark_steel,
            name=f"{side}_caster_gusset",
        )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.visual(
        Cylinder(radius=0.010, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=bright_steel,
        name="left_stem",
    )
    left_caster_fork.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=dark_steel,
        name="left_swivel_race",
    )
    left_caster_fork.visual(
        Box((0.046, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=dark_steel,
        name="left_fork_bridge",
    )
    left_caster_fork.visual(
        Box((0.006, 0.016, 0.055)),
        origin=Origin(xyz=(0.018, 0.0, -0.0675)),
        material=dark_steel,
        name="left_outer_fork_plate",
    )
    left_caster_fork.visual(
        Box((0.006, 0.016, 0.055)),
        origin=Origin(xyz=(-0.018, 0.0, -0.0675)),
        material=dark_steel,
        name="left_inner_fork_plate",
    )

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.visual(
        Cylinder(radius=0.010, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=bright_steel,
        name="right_stem",
    )
    right_caster_fork.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=dark_steel,
        name="right_swivel_race",
    )
    right_caster_fork.visual(
        Box((0.046, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=dark_steel,
        name="right_fork_bridge",
    )
    right_caster_fork.visual(
        Box((0.006, 0.016, 0.055)),
        origin=Origin(xyz=(0.018, 0.0, -0.0675)),
        material=dark_steel,
        name="right_outer_fork_plate",
    )
    right_caster_fork.visual(
        Box((0.006, 0.016, 0.055)),
        origin=Origin(xyz=(-0.018, 0.0, -0.0675)),
        material=dark_steel,
        name="right_inner_fork_plate",
    )

    left_caster_wheel = model.part("left_caster_wheel")
    _add_wheel_visuals(
        left_caster_wheel,
        tire_radius=0.055,
        tire_width=0.030,
        rim_radius=0.038,
        hub_radius=0.015,
        rim_material=bright_steel,
        hub_material=dark_steel,
        tire_material=rubber,
        prefix="left_caster",
    )

    right_caster_wheel = model.part("right_caster_wheel")
    _add_wheel_visuals(
        right_caster_wheel,
        tire_radius=0.055,
        tire_width=0.030,
        rim_radius=0.038,
        hub_radius=0.015,
        rim_material=bright_steel,
        hub_material=dark_steel,
        tire_material=rubber,
        prefix="right_caster",
    )

    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(0.2175, -0.06, 0.23)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.2175, -0.06, 0.23)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_caster_bar",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=caster_bar,
        origin=Origin(xyz=(0.0, 0.025, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=caster_bar,
        child=left_caster_fork,
        origin=Origin(xyz=(0.08, 0.225, -0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=caster_bar,
        child=right_caster_fork,
        origin=Origin(xyz=(-0.08, 0.225, -0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")
    caster_bar = object_model.get_part("caster_bar")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")

    left_rear_spin = object_model.get_articulation("left_rear_wheel_spin")
    right_rear_spin = object_model.get_articulation("right_rear_wheel_spin")
    caster_bar_hinge = object_model.get_articulation("frame_to_caster_bar")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_spin = object_model.get_articulation("left_caster_wheel_spin")
    right_caster_spin = object_model.get_articulation("right_caster_wheel_spin")

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

    ctx.expect_contact(frame, left_rear_wheel, name="left_rear_wheel_mounted_to_axle_plate")
    ctx.expect_contact(frame, right_rear_wheel, name="right_rear_wheel_mounted_to_axle_plate")
    ctx.expect_contact(frame, caster_bar, name="caster_bar_clipped_into_hinge_mounts")
    ctx.expect_contact(caster_bar, left_caster_fork, name="left_caster_swivel_seated_under_bar")
    ctx.expect_contact(caster_bar, right_caster_fork, name="right_caster_swivel_seated_under_bar")
    ctx.expect_contact(left_caster_fork, left_caster_wheel, name="left_caster_wheel_captured_in_fork")
    ctx.expect_contact(right_caster_fork, right_caster_wheel, name="right_caster_wheel_captured_in_fork")

    ctx.check(
        "rear_wheels_spin_on_transverse_axle",
        left_rear_spin.axis == (1.0, 0.0, 0.0) and right_rear_spin.axis == (1.0, 0.0, 0.0),
        f"rear axes were {left_rear_spin.axis} and {right_rear_spin.axis}",
    )
    ctx.check(
        "caster_bar_folds_about_transverse_hinge",
        caster_bar_hinge.axis == (1.0, 0.0, 0.0),
        f"caster bar hinge axis was {caster_bar_hinge.axis}",
    )
    ctx.check(
        "caster_swivels_turn_on_vertical_stems",
        left_caster_swivel.axis == (0.0, 0.0, 1.0) and right_caster_swivel.axis == (0.0, 0.0, 1.0),
        f"caster swivel axes were {left_caster_swivel.axis} and {right_caster_swivel.axis}",
    )
    ctx.check(
        "front_caster_wheels_spin_on_their_axles",
        left_caster_spin.axis == (1.0, 0.0, 0.0) and right_caster_spin.axis == (1.0, 0.0, 0.0),
        f"front wheel axes were {left_caster_spin.axis} and {right_caster_spin.axis}",
    )

    hinge_limits = caster_bar_hinge.motion_limits
    ctx.check(
        "caster_bar_has_real_fold_range",
        hinge_limits is not None and hinge_limits.upper is not None and hinge_limits.upper >= 1.30,
        f"caster bar limits were {hinge_limits}",
    )

    rest_front_left = ctx.part_world_position(left_caster_wheel)
    assert rest_front_left is not None
    folded_angle = hinge_limits.upper if hinge_limits is not None and hinge_limits.upper is not None else 1.30
    with ctx.pose({caster_bar_hinge: folded_angle}):
        folded_front_left = ctx.part_world_position(left_caster_wheel)
        assert folded_front_left is not None
        ctx.check(
            "folded_caster_module_lifts_front_wheels_up_under_frame",
            folded_front_left[2] > rest_front_left[2] + 0.20,
            f"front wheel z moved from {rest_front_left[2]:.3f} to {folded_front_left[2]:.3f}",
        )
        ctx.check(
            "folded_caster_module_swings_back_toward_nose_plate",
            folded_front_left[1] < rest_front_left[1] - 0.04,
            f"front wheel y moved from {rest_front_left[1]:.3f} to {folded_front_left[1]:.3f}",
        )
        ctx.expect_contact(caster_bar, left_caster_fork, name="left_caster_stem_remains_seated_when_folded")
        ctx.expect_contact(caster_bar, right_caster_fork, name="right_caster_stem_remains_seated_when_folded")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
