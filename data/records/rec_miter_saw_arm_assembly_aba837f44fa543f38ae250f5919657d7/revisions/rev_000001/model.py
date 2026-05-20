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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _annulus_sector_profile(
    *,
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    samples: int = 28,
) -> list[tuple[float, float]]:
    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        angle = start_angle + (end_angle - start_angle) * t
        y = outer_radius * math.cos(angle)
        z = outer_radius * math.sin(angle)
        outer.append((-z, y))
    for index in range(samples, -1, -1):
        t = index / samples
        angle = start_angle + (end_angle - start_angle) * t
        y = inner_radius * math.cos(angle)
        z = inner_radius * math.sin(angle)
        inner.append((-z, y))
    return outer + inner


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[i] + maximum[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_miter_saw")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.73, 0.74, 0.76, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.44, 0.46, 0.49, 1.0))
    dark_charcoal = model.material("dark_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.83, 0.84, 0.86, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.86, 0.34, 0.10, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base_shell_mesh = _save_mesh(
        "miter_saw_base_shell",
        ExtrudeGeometry(rounded_rect_profile(0.56, 0.30, 0.035), 0.024, center=True),
    )
    table_deck_mesh = _save_mesh(
        "miter_saw_table_deck",
        ExtrudeGeometry(rounded_rect_profile(0.32, 0.18, 0.020), 0.018, center=True),
    )
    upper_guard_mesh = _save_mesh(
        "miter_saw_upper_guard",
        ExtrudeGeometry(
            _annulus_sector_profile(
                outer_radius=0.145,
                inner_radius=0.114,
                start_angle=math.radians(-18.0),
                end_angle=math.radians(218.0),
                samples=32,
            ),
            0.090,
            center=True,
        ).rotate_y(math.pi / 2.0),
    )
    carry_handle_mesh = _save_mesh(
        "miter_saw_carry_handle_loop",
        tube_from_spline_points(
            [
                (-0.060, 0.000, 0.000),
                (-0.052, 0.022, 0.006),
                (-0.028, 0.045, 0.013),
                (0.000, 0.058, 0.018),
                (0.028, 0.045, 0.013),
                (0.052, 0.022, 0.006),
                (0.060, 0.000, 0.000),
            ],
            radius=0.007,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )

    base = model.part("base")
    base.visual(
        base_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=cast_aluminum,
        name="base_shell",
    )
    base.visual(
        Box((0.09, 0.12, 0.040)),
        origin=Origin(xyz=(0.22, 0.10, 0.032)),
        material=cast_aluminum,
        name="right_front_wing",
    )
    base.visual(
        Box((0.09, 0.12, 0.040)),
        origin=Origin(xyz=(-0.22, 0.10, 0.032)),
        material=cast_aluminum,
        name="left_front_wing",
    )
    base.visual(
        Box((0.34, 0.06, 0.080)),
        origin=Origin(xyz=(0.0, -0.170, 0.064)),
        material=cast_aluminum,
        name="rear_bridge",
    )
    base.visual(
        Cylinder(radius=0.110, length=0.060),
        origin=Origin(xyz=(0.0, 0.010, 0.054)),
        material=cast_aluminum,
        name="turntable_pedestal",
    )
    base.visual(
        Cylinder(radius=0.090, length=0.006),
        origin=Origin(xyz=(0.0, 0.010, 0.081)),
        material=steel_gray,
        name="bearing_ring",
    )
    base.visual(
        Box((0.055, 0.070, 0.306)),
        origin=Origin(xyz=(0.120, -0.160, 0.177)),
        material=cast_aluminum,
        name="right_hinge_tower",
    )
    base.visual(
        Box((0.055, 0.070, 0.306)),
        origin=Origin(xyz=(-0.120, -0.160, 0.177)),
        material=cast_aluminum,
        name="left_hinge_tower",
    )
    base.visual(
        Box((0.060, 0.050, 0.104)),
        origin=Origin(xyz=(0.205, -0.078, 0.052)),
        material=cast_aluminum,
        name="right_fence_support",
    )
    base.visual(
        Box((0.060, 0.050, 0.104)),
        origin=Origin(xyz=(-0.205, -0.078, 0.052)),
        material=cast_aluminum,
        name="left_fence_support",
    )
    base.visual(
        Box((0.160, 0.016, 0.064)),
        origin=Origin(xyz=(0.125, -0.086, 0.136)),
        material=steel_gray,
        name="right_fence",
    )
    base.visual(
        Box((0.160, 0.016, 0.064)),
        origin=Origin(xyz=(-0.125, -0.086, 0.136)),
        material=steel_gray,
        name="left_fence",
    )
    base.visual(
        Box((0.060, 0.020, 0.028)),
        origin=Origin(xyz=(0.200, 0.130, 0.038)),
        material=dark_charcoal,
        name="detent_housing",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.235, 0.130, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="detent_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.56, 0.34, 0.34)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.01, 0.17)),
    )

    miter_table = model.part("miter_table")
    miter_table.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel_gray,
        name="turntable_disk",
    )
    miter_table.visual(
        table_deck_mesh,
        origin=Origin(xyz=(0.0, 0.065, 0.028)),
        material=cast_aluminum,
        name="deck_top",
    )
    miter_table.visual(
        Box((0.080, 0.115, 0.006)),
        origin=Origin(xyz=(0.0, 0.058, 0.040)),
        material=steel_gray,
        name="throat_plate",
    )
    miter_table.visual(
        Box((0.040, 0.050, 0.020)),
        origin=Origin(xyz=(0.095, 0.138, 0.028)),
        material=dark_charcoal,
        name="miter_lock",
    )
    miter_table.inertial = Inertial.from_geometry(
        Box((0.34, 0.30, 0.05)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.050, 0.025)),
    )

    motor_arm = model.part("motor_arm")
    motor_arm.visual(
        Cylinder(radius=0.030, length=0.185),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_gray,
        name="hinge_barrel",
    )
    motor_arm.visual(
        Box((0.035, 0.220, 0.055)),
        origin=Origin(xyz=(0.060, 0.100, 0.000)),
        material=cast_aluminum,
        name="right_yoke",
    )
    motor_arm.visual(
        Box((0.035, 0.220, 0.055)),
        origin=Origin(xyz=(-0.060, 0.100, 0.000)),
        material=cast_aluminum,
        name="left_yoke",
    )
    motor_arm.visual(
        Box((0.145, 0.060, 0.055)),
        origin=Origin(xyz=(0.0, 0.125, 0.000)),
        material=cast_aluminum,
        name="yoke_crossmember",
    )
    motor_arm.visual(
        Box((0.090, 0.055, 0.120)),
        origin=Origin(xyz=(0.0, 0.165, 0.045)),
        material=cast_aluminum,
        name="neck_block",
    )
    motor_arm.visual(
        Cylinder(radius=0.050, length=0.110),
        origin=Origin(xyz=(0.0, 0.182, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_gray,
        name="arbor_housing",
    )
    motor_arm.visual(
        upper_guard_mesh,
        origin=Origin(xyz=(0.0, 0.180, -0.015)),
        material=safety_orange,
        name="upper_guard",
    )
    motor_arm.visual(
        Box((0.140, 0.070, 0.050)),
        origin=Origin(xyz=(0.0, 0.192, 0.090)),
        material=safety_orange,
        name="crown_cover",
    )
    motor_arm.visual(
        Box((0.024, 0.030, 0.042)),
        origin=Origin(xyz=(0.075, 0.192, 0.126)),
        material=safety_orange,
        name="right_handle_boss",
    )
    motor_arm.visual(
        Box((0.024, 0.030, 0.042)),
        origin=Origin(xyz=(-0.075, 0.192, 0.126)),
        material=safety_orange,
        name="left_handle_boss",
    )
    motor_arm.visual(
        Cylinder(radius=0.060, length=0.160),
        origin=Origin(xyz=(0.100, 0.190, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="motor_body",
    )
    motor_arm.visual(
        Cylinder(radius=0.037, length=0.070),
        origin=Origin(xyz=(0.185, 0.190, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="motor_cap",
    )
    motor_arm.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.190, 0.190, 0.132), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="top_handle",
    )
    motor_arm.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, 0.180, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_gray,
        name="blade_hub",
    )
    motor_arm.visual(
        Cylinder(radius=0.127, length=0.004),
        origin=Origin(xyz=(0.0, 0.180, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_metal,
        name="blade_disk",
    )
    motor_arm.visual(
        Box((0.070, 0.040, 0.070)),
        origin=Origin(xyz=(-0.015, 0.118, -0.060)),
        material=dark_charcoal,
        name="dust_chute",
    )
    motor_arm.inertial = Inertial.from_geometry(
        Box((0.44, 0.34, 0.30)),
        mass=14.0,
        origin=Origin(xyz=(0.05, 0.16, 0.04)),
    )

    carry_handle = model.part("carry_handle")
    carry_handle.visual(
        Cylinder(radius=0.008, length=0.120),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="handle_pivot",
    )
    carry_handle.visual(
        carry_handle_mesh,
        material=handle_black,
        name="handle_loop",
    )
    carry_handle.inertial = Inertial.from_geometry(
        Box((0.14, 0.08, 0.06)),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.020, 0.025)),
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=miter_table,
        origin=Origin(xyz=(0.0, 0.010, 0.084)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-0.80,
            upper=0.80,
        ),
    )
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=motor_arm,
        origin=Origin(xyz=(0.0, -0.160, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.0,
            lower=-0.40,
            upper=0.55,
        ),
    )
    model.articulation(
        "arm_to_handle",
        ArticulationType.REVOLUTE,
        parent=motor_arm,
        child=carry_handle,
        origin=Origin(xyz=(0.0, 0.192, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=1.25,
        ),
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

    base = object_model.get_part("base")
    miter_table = object_model.get_part("miter_table")
    motor_arm = object_model.get_part("motor_arm")
    carry_handle = object_model.get_part("carry_handle")

    table_joint = object_model.get_articulation("base_to_table")
    arm_joint = object_model.get_articulation("base_to_arm")
    handle_joint = object_model.get_articulation("arm_to_handle")

    ctx.expect_gap(
        miter_table,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="bearing_ring",
        min_gap=0.0,
        max_gap=0.002,
        name="turntable sits on the base bearing ring",
    )
    ctx.expect_overlap(
        miter_table,
        base,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="bearing_ring",
        min_overlap=0.15,
        name="turntable stays centered over the bearing ring",
    )

    table_lock_rest = _center_from_aabb(ctx.part_element_world_aabb(miter_table, elem="miter_lock"))
    with ctx.pose({table_joint: math.radians(35.0)}):
        ctx.expect_gap(
            miter_table,
            base,
            axis="z",
            positive_elem="turntable_disk",
            negative_elem="bearing_ring",
            min_gap=0.0,
            max_gap=0.002,
            name="turntable remains seated while rotated",
        )
        table_lock_turned = _center_from_aabb(ctx.part_element_world_aabb(miter_table, elem="miter_lock"))
    ctx.check(
        "miter table rotates about the vertical axis",
        table_lock_rest is not None
        and table_lock_turned is not None
        and abs(table_lock_turned[0] - table_lock_rest[0]) > 0.025
        and abs(table_lock_turned[1] - table_lock_rest[1]) > 0.015
        and abs(table_lock_turned[2] - table_lock_rest[2]) < 0.003,
        details=f"rest={table_lock_rest}, turned={table_lock_turned}",
    )

    blade_rest = _center_from_aabb(ctx.part_element_world_aabb(motor_arm, elem="blade_disk"))
    with ctx.pose({arm_joint: math.radians(18.0)}):
        blade_raised = _center_from_aabb(ctx.part_element_world_aabb(motor_arm, elem="blade_disk"))
    with ctx.pose({arm_joint: math.radians(-18.0)}):
        blade_lowered = _center_from_aabb(ctx.part_element_world_aabb(motor_arm, elem="blade_disk"))
        ctx.expect_gap(
            motor_arm,
            miter_table,
            axis="z",
            positive_elem="blade_disk",
            negative_elem="deck_top",
            min_gap=0.002,
            max_gap=0.120,
            name="lowered blade stays just above the table deck",
        )
    ctx.check(
        "positive arm motion raises the blade",
        blade_rest is not None
        and blade_raised is not None
        and blade_raised[2] > blade_rest[2] + 0.040,
        details=f"rest={blade_rest}, raised={blade_raised}",
    )
    ctx.check(
        "negative arm motion lowers the blade",
        blade_rest is not None
        and blade_lowered is not None
        and blade_lowered[2] < blade_rest[2] - 0.040,
        details=f"rest={blade_rest}, lowered={blade_lowered}",
    )

    handle_rest = _center_from_aabb(ctx.part_element_world_aabb(carry_handle, elem="handle_loop"))
    with ctx.pose({handle_joint: 1.00}):
        handle_up = _center_from_aabb(ctx.part_element_world_aabb(carry_handle, elem="handle_loop"))
    ctx.check(
        "carry handle folds upward from the arm cover",
        handle_rest is not None
        and handle_up is not None
        and handle_up[2] > handle_rest[2] + 0.018,
        details=f"rest={handle_rest}, up={handle_up}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
