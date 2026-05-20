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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="miter_saw")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.80, 0.82, 0.85, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    warning_orange = model.material("warning_orange", rgba=(0.93, 0.48, 0.12, 1.0))
    clamp_red = model.material("clamp_red", rgba=(0.68, 0.10, 0.12, 1.0))
    black_grip = model.material("black_grip", rgba=(0.08, 0.08, 0.09, 1.0))

    def xz_section(
        width: float,
        height: float,
        radius: float,
        y: float,
        z: float,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, zz + z) for x, zz in rounded_rect_profile(width, height, radius)]

    base = model.part("base")
    base_casting = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.54, 0.34, 0.06), 0.038),
        "miter_saw_base_casting",
    )
    base.visual(
        base_casting,
        origin=Origin(xyz=(0.0, -0.005, 0.019)),
        material=cast_aluminum,
        name="base_casting",
    )
    base.visual(
        Box((0.62, 0.10, 0.018)),
        origin=Origin(xyz=(0.0, -0.14, 0.009)),
        material=cast_aluminum,
        name="front_stance",
    )
    base.visual(
        Box((0.22, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.155, 0.043)),
        material=cast_aluminum,
        name="rear_pedestal",
    )
    base.visual(
        Cylinder(radius=0.138, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=dark_gray,
        name="swivel_seat",
    )
    base.visual(
        Box((0.050, 0.170, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=charcoal,
        name="kerf_channel",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.62, 0.40, 0.11)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    table = model.part("table")
    turntable_plate = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.220, 0.220, exponent=2.0, segments=64),
            [rounded_rect_profile(0.060, 0.180, 0.006)],
            height=0.018,
            center=True,
        ),
        "miter_saw_turntable_plate_v2",
    )
    table.visual(
        turntable_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_aluminum,
        name="turntable_plate",
    )
    table.visual(
        Box((0.028, 0.080, 0.008)),
        origin=Origin(xyz=(0.0, -0.085, 0.004)),
        material=dark_gray,
        name="miter_handle",
    )
    table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.018),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    fence = model.part("fence")
    fence.visual(
        Box((0.230, 0.014, 0.060)),
        origin=Origin(xyz=(-0.150, 0.128, 0.030)),
        material=steel,
        name="left_fence",
    )
    fence.visual(
        Box((0.230, 0.014, 0.060)),
        origin=Origin(xyz=(0.150, 0.128, 0.030)),
        material=steel,
        name="right_fence",
    )
    fence.visual(
        Box((0.540, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.140, 0.011)),
        material=dark_gray,
        name="fence_brace",
    )
    fence.inertial = Inertial.from_geometry(
        Box((0.54, 0.03, 0.06)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.134, 0.030)),
    )

    yoke = model.part("rear_yoke")
    yoke.visual(
        Box((0.050, 0.040, 0.210)),
        origin=Origin(xyz=(-0.070, 0.176, 0.105)),
        material=dark_gray,
        name="left_upright",
    )
    yoke.visual(
        Box((0.050, 0.040, 0.210)),
        origin=Origin(xyz=(0.070, 0.176, 0.105)),
        material=dark_gray,
        name="right_upright",
    )
    yoke.visual(
        Box((0.045, 0.055, 0.055)),
        origin=Origin(xyz=(-0.062, 0.176, 0.205)),
        material=charcoal,
        name="left_pivot_ear",
    )
    yoke.visual(
        Box((0.045, 0.055, 0.055)),
        origin=Origin(xyz=(0.062, 0.176, 0.205)),
        material=charcoal,
        name="right_pivot_ear",
    )
    yoke.visual(
        Box((0.180, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, 0.207, 0.220)),
        material=charcoal,
        name="rear_cross_brace",
    )
    yoke.visual(
        Box((0.180, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.175, 0.010)),
        material=charcoal,
        name="yoke_base",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.18, 0.09, 0.24)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.176, 0.120)),
    )

    arm = model.part("saw_arm")
    arm.visual(
        Box((0.050, 0.190, 0.050)),
        origin=Origin(xyz=(0.0, -0.105, 0.026)),
        material=warning_orange,
        name="arm_shell",
    )
    arm.visual(
        Box((0.048, 0.040, 0.046)),
        origin=Origin(xyz=(0.0, -0.010, 0.023)),
        material=charcoal,
        name="pivot_block",
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.031),
        origin=Origin(xyz=(-0.024, -0.010, 0.023), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.031),
        origin=Origin(xyz=(0.024, -0.010, 0.023), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    arm.visual(
        Box((0.028, 0.075, 0.070)),
        origin=Origin(xyz=(-0.032, -0.185, 0.100)),
        material=warning_orange,
        name="upper_guard_bridge",
    )
    arm.visual(
        Cylinder(radius=0.150, length=0.012),
        origin=Origin(xyz=(-0.055, -0.225, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warning_orange,
        name="blade_housing",
    )
    arm.visual(
        Cylinder(radius=0.050, length=0.074),
        origin=Origin(xyz=(0.018, -0.225, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="motor_housing",
    )
    arm.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(-0.024, -0.225, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spindle_washer",
    )
    arm.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=(0.0, -0.118, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_grip,
        name="handle_grip",
    )
    arm.visual(
        Box((0.020, 0.048, 0.062)),
        origin=Origin(xyz=(-0.018, -0.108, 0.060)),
        material=charcoal,
        name="handle_support",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.19, 0.30, 0.18)),
        mass=5.5,
        origin=Origin(xyz=(0.02, -0.130, 0.010)),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.127, length=0.003),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_metal,
        name="blade_disk",
    )
    blade.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="blade_hub",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.127, length=0.010),
        mass=1.1,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    clamp_guide = model.part("clamp_guide")
    clamp_guide.visual(
        Box((0.030, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_gray,
        name="guide_mount",
    )
    clamp_guide.visual(
        Cylinder(radius=0.004, length=0.110),
        origin=Origin(xyz=(-0.010, -0.002, 0.055)),
        material=steel,
        name="left_guide_post",
    )
    clamp_guide.visual(
        Cylinder(radius=0.004, length=0.110),
        origin=Origin(xyz=(0.010, -0.002, 0.055)),
        material=steel,
        name="right_guide_post",
    )
    clamp_guide.visual(
        Box((0.032, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.002, 0.115)),
        material=steel,
        name="guide_top",
    )
    clamp_guide.inertial = Inertial.from_geometry(
        Box((0.03, 0.02, 0.12)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.008, 0.060)),
    )

    clamp_slider = model.part("clamp_slider")
    clamp_slider.visual(
        Box((0.012, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, -0.002, -0.010)),
        material=clamp_red,
        name="carriage",
    )
    clamp_slider.visual(
        Cylinder(radius=0.0045, length=0.070),
        origin=Origin(xyz=(0.0, -0.020, -0.055)),
        material=steel,
        name="clamp_rod",
    )
    clamp_slider.visual(
        Box((0.012, 0.024, 0.016)),
        origin=Origin(xyz=(0.0, -0.014, -0.026)),
        material=clamp_red,
        name="rod_carrier",
    )
    clamp_slider.visual(
        Box((0.028, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.020, -0.092)),
        material=clamp_red,
        name="pressure_pad",
    )
    clamp_slider.visual(
        Box((0.012, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.012, -0.008)),
        material=clamp_red,
        name="knob_bracket",
    )
    clamp_slider.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.0, 0.026, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_grip,
        name="knob_grip",
    )
    clamp_slider.inertial = Inertial.from_geometry(
        Box((0.06, 0.02, 0.11)),
        mass=0.35,
        origin=Origin(xyz=(-0.010, 0.010, -0.045)),
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=math.radians(-50.0),
            upper=math.radians(50.0),
        ),
    )
    model.articulation(
        "base_to_fence",
        ArticulationType.FIXED,
        parent=base,
        child=fence,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )
    model.articulation(
        "base_to_rear_yoke",
        ArticulationType.FIXED,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )
    model.articulation(
        "yoke_to_saw_arm",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=arm,
        origin=Origin(xyz=(0.0, 0.176, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.3,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "arm_to_blade",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(-0.040, -0.220, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=140.0),
    )
    model.articulation(
        "fence_to_clamp_guide",
        ArticulationType.FIXED,
        parent=fence,
        child=clamp_guide,
        origin=Origin(xyz=(0.212, 0.118, 0.060)),
    )
    model.articulation(
        "guide_to_clamp_slider",
        ArticulationType.PRISMATIC,
        parent=clamp_guide,
        child=clamp_slider,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.10,
            lower=0.0,
            upper=0.040,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    table = object_model.get_part("table")
    fence = object_model.get_part("fence")
    yoke = object_model.get_part("rear_yoke")
    arm = object_model.get_part("saw_arm")
    blade = object_model.get_part("blade")
    clamp_guide = object_model.get_part("clamp_guide")
    clamp_slider = object_model.get_part("clamp_slider")

    table_yaw = object_model.get_articulation("base_to_table")
    arm_pivot = object_model.get_articulation("yoke_to_saw_arm")
    blade_spin = object_model.get_articulation("arm_to_blade")
    clamp_slide = object_model.get_articulation("guide_to_clamp_slider")

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

    ctx.expect_contact(table, base, elem_a="turntable_plate", elem_b="swivel_seat")
    ctx.expect_contact(fence, base)
    ctx.expect_contact(yoke, base)
    ctx.expect_contact(arm, yoke)
    ctx.expect_contact(blade, arm, elem_a="blade_hub", elem_b="spindle_washer")
    ctx.expect_contact(clamp_guide, fence)
    ctx.expect_contact(clamp_slider, clamp_guide)

    ctx.expect_within(table, base, axes="xy", margin=0.08)
    ctx.expect_overlap(blade, table, axes="x", min_overlap=0.002)
    ctx.expect_overlap(blade, table, axes="y", min_overlap=0.08)

    ctx.check(
        "table_yaw_axis_is_vertical",
        tuple(table_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={table_yaw.axis}",
    )
    ctx.check(
        "arm_pivot_axis_is_transverse",
        tuple(arm_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"axis={arm_pivot.axis}",
    )
    ctx.check(
        "blade_spin_axis_matches_spindle",
        tuple(blade_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={blade_spin.axis}",
    )
    ctx.check(
        "clamp_slide_axis_is_vertical",
        tuple(clamp_slide.axis) == (0.0, 0.0, -1.0),
        details=f"axis={clamp_slide.axis}",
    )

    blade_rest = ctx.part_world_position(blade)
    clamp_rest = ctx.part_world_position(clamp_slider)
    assert blade_rest is not None
    assert clamp_rest is not None

    with ctx.pose({arm_pivot: math.radians(45.0)}):
        blade_lowered = ctx.part_world_position(blade)
        assert blade_lowered is not None
        ctx.check(
            "arm_lowering_drops_blade",
            blade_lowered[2] < blade_rest[2] - 0.08,
            details=f"rest_z={blade_rest[2]:.4f}, lowered_z={blade_lowered[2]:.4f}",
        )
        ctx.expect_overlap(
            blade,
            table,
            axes="y",
            min_overlap=0.08,
            name="lowered_blade_tracks_over_cutting_zone",
        )

    with ctx.pose({clamp_slide: 0.035}):
        clamp_lowered = ctx.part_world_position(clamp_slider)
        assert clamp_lowered is not None
        ctx.check(
            "clamp_slider_moves_downward",
            clamp_lowered[2] < clamp_rest[2] - 0.02,
            details=f"rest_z={clamp_rest[2]:.4f}, lowered_z={clamp_lowered[2]:.4f}",
        )
        ctx.expect_gap(
            clamp_slider,
            table,
            axis="z",
            min_gap=0.0,
            max_gap=0.030,
            positive_elem="pressure_pad",
            negative_elem="turntable_plate",
            name="lowered_clamp_pad_remains_above_table",
        )

    with ctx.pose({table_yaw: math.radians(40.0)}):
        ctx.expect_contact(table, base, elem_a="turntable_plate", elem_b="swivel_seat")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
