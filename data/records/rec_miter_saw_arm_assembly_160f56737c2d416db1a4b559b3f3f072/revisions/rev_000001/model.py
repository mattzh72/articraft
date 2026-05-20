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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_sector_profile(
    *,
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    segments: int = 40,
) -> list[tuple[float, float]]:
    if end_angle <= start_angle:
        end_angle += math.tau

    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        y = outer_radius * math.cos(angle)
        z = outer_radius * math.sin(angle)
        outer.append((-z, y))
    for index in range(segments, -1, -1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        y = inner_radius * math.cos(angle)
        z = inner_radius * math.sin(angle)
        inner.append((-z, y))
    return outer + inner


def _sector_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    start_angle_deg: float,
    end_angle_deg: float,
    thickness: float,
) -> object:
    geom = ExtrudeGeometry(
        _annular_sector_profile(
            outer_radius=outer_radius,
            inner_radius=inner_radius,
            start_angle=math.radians(start_angle_deg),
            end_angle=math.radians(end_angle_deg),
        ),
        thickness,
        cap=True,
        center=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((low + high) * 0.5 for low, high in zip(aabb[0], aabb[1]))


def _axis_signature(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(round(value, 3) for value in axis)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_miter_saw_arm_assembly")

    base_dark = model.material("base_dark", rgba=(0.19, 0.21, 0.23, 1.0))
    cast_aluminum = model.material("cast_aluminum", rgba=(0.71, 0.73, 0.75, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.74, 0.77, 0.80, 1.0))
    turntable_gray = model.material("turntable_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    saw_yellow = model.material("saw_yellow", rgba=(0.92, 0.73, 0.15, 1.0))
    guard_gray = model.material("guard_gray", rgba=(0.64, 0.66, 0.70, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))

    upper_guard_mesh = _sector_shell_mesh(
        "miter_saw_upper_guard_v3",
        outer_radius=0.145,
        inner_radius=0.112,
        start_angle_deg=55.0,
        end_angle_deg=240.0,
        thickness=0.007,
    )
    lower_guard_mesh = _sector_shell_mesh(
        "miter_saw_lower_guard_v3",
        outer_radius=0.146,
        inner_radius=0.110,
        start_angle_deg=185.0,
        end_angle_deg=355.0,
        thickness=0.006,
    )

    base = model.part("base")
    base.visual(
        Box((0.60, 0.46, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=base_dark,
        name="bed",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=turntable_gray,
        name="pedestal",
    )
    base.visual(
        Box((0.24, 0.06, 0.018)),
        origin=Origin(xyz=(0.0, -0.18, 0.034)),
        material=base_dark,
        name="miter_scale_apron",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.135, length=0.020),
        material=turntable_gray,
        name="table_disk",
    )
    turntable.visual(
        Box((0.020, 0.060, 0.018)),
        origin=Origin(xyz=(0.090, -0.135, 0.019)),
        material=turntable_gray,
        name="miter_stem",
    )
    turntable.visual(
        Box((0.060, 0.040, 0.008)),
        origin=Origin(xyz=(0.095, -0.165, 0.014)),
        material=turntable_gray,
        name="miter_handle",
    )

    fence = model.part("fence")
    fence.visual(
        Box((0.220, 0.018, 0.075)),
        origin=Origin(xyz=(-0.170, 0.155, 0.0625)),
        material=cast_aluminum,
        name="left_fence_wing",
    )
    fence.visual(
        Box((0.220, 0.018, 0.075)),
        origin=Origin(xyz=(0.170, 0.155, 0.0625)),
        material=cast_aluminum,
        name="right_fence_wing",
    )
    fence.visual(
        Box((0.540, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.171, 0.033)),
        material=cast_aluminum,
        name="fence_base",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.100, 0.040, 0.360)),
        origin=Origin(xyz=(0.0, 0.205, 0.205)),
        material=base_dark,
        name="rear_post",
    )
    yoke.visual(
        Box((0.105, 0.020, 0.025)),
        origin=Origin(xyz=(0.0, 0.185, 0.305)),
        material=base_dark,
        name="cross_beam",
    )
    yoke.visual(
        Box((0.015, 0.050, 0.080)),
        origin=Origin(xyz=(-0.0375, 0.175, 0.270)),
        material=base_dark,
        name="left_fork",
    )
    yoke.visual(
        Box((0.015, 0.050, 0.080)),
        origin=Origin(xyz=(0.0375, 0.175, 0.270)),
        material=base_dark,
        name="right_fork",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.021, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_aluminum,
        name="pivot_boss",
    )
    arm.visual(
        Box((0.048, 0.200, 0.036)),
        origin=Origin(xyz=(0.0, -0.100, -0.002)),
        material=cast_aluminum,
        name="arm_beam",
    )
    arm.visual(
        Box((0.056, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, -0.225, -0.004)),
        material=cast_aluminum,
        name="mount_nose",
    )
    arm.visual(
        Box((0.040, 0.100, 0.040)),
        origin=Origin(xyz=(0.0, -0.105, 0.034)),
        material=saw_yellow,
        name="handle_support",
    )
    arm.visual(
        Cylinder(radius=0.011, length=0.095),
        origin=Origin(xyz=(0.0, -0.115, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="top_handle",
    )

    blade_housing = model.part("blade_housing")
    blade_housing.visual(
        Box((0.012, 0.170, 0.160)),
        origin=Origin(xyz=(0.0, -0.035, 0.040)),
        material=cast_aluminum,
        name="back_plate",
    )
    blade_housing.visual(
        Box((0.048, 0.080, 0.085)),
        origin=Origin(xyz=(0.030, -0.030, 0.070)),
        material=saw_yellow,
        name="gear_case",
    )
    blade_housing.visual(
        Cylinder(radius=0.040, length=0.065),
        origin=Origin(xyz=(0.068, -0.068, 0.072), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=saw_yellow,
        name="motor_can",
    )
    blade_housing.visual(
        Box((0.012, 0.050, 0.100)),
        origin=Origin(xyz=(-0.002, -0.082, 0.040)),
        material=guard_gray,
        name="guard_bridge",
    )
    blade_housing.visual(
        upper_guard_mesh,
        origin=Origin(xyz=(-0.003, -0.120, -0.020)),
        material=guard_gray,
        name="upper_guard_shell",
    )
    blade_housing.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, -0.120, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="spindle_boss",
    )
    blade_housing.visual(
        Box((0.020, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.030, 0.072)),
        material=cast_aluminum,
        name="hinge_mount",
    )
    blade_housing.visual(
        Box((0.004, 0.018, 0.030)),
        origin=Origin(xyz=(-0.011, -0.030, 0.090)),
        material=cast_aluminum,
        name="left_guard_lug",
    )
    blade_housing.visual(
        Box((0.004, 0.018, 0.030)),
        origin=Origin(xyz=(0.011, -0.030, 0.090)),
        material=cast_aluminum,
        name="right_guard_lug",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.127, length=0.0022),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="blade_disc",
    )
    blade.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="spindle_flange",
    )

    lower_guard = model.part("lower_guard")
    lower_guard.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hinge_barrel",
    )
    lower_guard.visual(
        Box((0.010, 0.020, 0.050)),
        origin=Origin(xyz=(-0.011, -0.010, -0.028)),
        material=guard_gray,
        name="guard_link",
    )
    lower_guard.visual(
        Box((0.008, 0.026, 0.050)),
        origin=Origin(xyz=(-0.016, -0.025, -0.065)),
        material=guard_gray,
        name="guard_strap",
    )
    lower_guard.visual(
        Box((0.008, 0.140, 0.110)),
        origin=Origin(xyz=(-0.020, -0.085, -0.105)),
        material=guard_gray,
        name="shell_connector",
    )
    lower_guard.visual(
        Box((0.010, 0.090, 0.090)),
        origin=Origin(xyz=(-0.020, -0.190, -0.140)),
        material=guard_gray,
        name="shell_rib",
    )
    lower_guard.visual(
        lower_guard_mesh,
        origin=Origin(xyz=(-0.020, -0.090, -0.110)),
        material=guard_gray,
        name="lower_guard_shell",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-math.radians(50.0),
            upper=math.radians(50.0),
        ),
    )
    model.articulation(
        "base_to_fence",
        ArticulationType.FIXED,
        parent=base,
        child=fence,
        origin=Origin(),
    )
    model.articulation(
        "base_to_yoke",
        ArticulationType.FIXED,
        parent=base,
        child=yoke,
        origin=Origin(),
    )
    model.articulation(
        "yoke_to_arm",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=arm,
        origin=Origin(xyz=(0.0, 0.150, 0.270), rpy=(-0.55, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=0.68,
        ),
    )
    model.articulation(
        "arm_to_blade_housing",
        ArticulationType.FIXED,
        parent=arm,
        child=blade_housing,
        origin=Origin(xyz=(0.0, -0.250, -0.004)),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=blade_housing,
        child=blade,
        origin=Origin(xyz=(0.0, -0.120, -0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=80.0),
    )
    model.articulation(
        "housing_to_lower_guard",
        ArticulationType.REVOLUTE,
        parent=blade_housing,
        child=lower_guard,
        origin=Origin(xyz=(0.0, -0.030, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-1.05,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    fence = object_model.get_part("fence")
    yoke = object_model.get_part("yoke")
    arm = object_model.get_part("arm")
    blade_housing = object_model.get_part("blade_housing")
    blade = object_model.get_part("blade")
    lower_guard = object_model.get_part("lower_guard")
    mount_nose = arm.get_visual("mount_nose")
    back_plate = blade_housing.get_visual("back_plate")
    gear_case = blade_housing.get_visual("gear_case")
    spindle_flange = blade.get_visual("spindle_flange")
    hinge_barrel = lower_guard.get_visual("hinge_barrel")

    table_yaw = object_model.get_articulation("base_to_turntable")
    arm_pitch = object_model.get_articulation("yoke_to_arm")
    blade_spin = object_model.get_articulation("housing_to_blade")
    guard_pivot = object_model.get_articulation("housing_to_lower_guard")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        blade,
        blade_housing,
        elem_a=spindle_flange,
        elem_b=back_plate,
        reason="Blade flange is represented without the matching spindle bore cut through the housing back plate.",
    )
    ctx.allow_overlap(
        lower_guard,
        blade_housing,
        elem_a=hinge_barrel,
        elem_b=back_plate,
        reason="Lower guard hinge barrel passes through an implied hinge bore in the back plate.",
    )
    ctx.allow_overlap(
        arm,
        blade_housing,
        elem_a=mount_nose,
        elem_b=back_plate,
        reason="The simplified rear head casting captures the arm's mounting tongue inside the pivoting head bracket.",
    )
    ctx.allow_overlap(
        arm,
        blade_housing,
        elem_a=mount_nose,
        elem_b=gear_case,
        reason="The gearbox casing is modeled as a simplified outer shell wrapped around the arm's front mounting tongue.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "turntable_yaw_axis",
        table_yaw.articulation_type == ArticulationType.REVOLUTE and _axis_signature(table_yaw.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical yaw axis, got type={table_yaw.articulation_type} axis={table_yaw.axis}",
    )
    ctx.check(
        "arm_pitch_axis",
        arm_pitch.articulation_type == ArticulationType.REVOLUTE and _axis_signature(arm_pitch.axis) == (1.0, 0.0, 0.0),
        f"Expected transverse pitch axis, got type={arm_pitch.articulation_type} axis={arm_pitch.axis}",
    )
    ctx.check(
        "blade_spin_axis",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS and _axis_signature(blade_spin.axis) == (1.0, 0.0, 0.0),
        f"Expected spindle spin around X, got type={blade_spin.articulation_type} axis={blade_spin.axis}",
    )
    ctx.check(
        "lower_guard_axis",
        guard_pivot.articulation_type == ArticulationType.REVOLUTE and _axis_signature(guard_pivot.axis) == (1.0, 0.0, 0.0),
        f"Expected lower guard hinge around X, got type={guard_pivot.articulation_type} axis={guard_pivot.axis}",
    )

    ctx.expect_contact(turntable, base, elem_a="table_disk", elem_b="pedestal", name="turntable_seated_on_pedestal")
    ctx.expect_contact(fence, base, name="fence_mounted_to_base")
    ctx.expect_gap(fence, turntable, axis="y", min_gap=0.001, max_gap=0.020, name="fence_clears_turntable")
    ctx.expect_contact(yoke, base, name="rear_yoke_mounted_to_base")
    ctx.expect_contact(arm, yoke, name="arm_supported_by_yoke")
    ctx.expect_contact(blade_housing, arm, name="blade_housing_mounted_to_arm")
    ctx.expect_contact(
        blade,
        blade_housing,
        elem_a="spindle_flange",
        elem_b="spindle_boss",
        name="blade_seated_on_spindle",
    )
    ctx.expect_contact(lower_guard, blade_housing, name="lower_guard_hinged_to_housing")
    ctx.expect_overlap(
        blade_housing,
        blade,
        axes="yz",
        elem_a="upper_guard_shell",
        elem_b="blade_disc",
        min_overlap=0.080,
        name="upper_guard_covers_blade_projection",
    )
    ctx.expect_overlap(
        lower_guard,
        blade,
        axes="yz",
        elem_a="lower_guard_shell",
        elem_b="blade_disc",
        min_overlap=0.070,
        name="lower_guard_covers_blade_projection",
    )

    handle_rest_aabb = ctx.part_element_world_aabb(turntable, elem="miter_handle")
    assert handle_rest_aabb is not None
    handle_rest_center = _aabb_center(handle_rest_aabb)
    table_limits = table_yaw.motion_limits
    assert table_limits is not None and table_limits.upper is not None
    with ctx.pose({table_yaw: table_limits.upper}):
        handle_turn_aabb = ctx.part_element_world_aabb(turntable, elem="miter_handle")
        assert handle_turn_aabb is not None
        handle_turn_center = _aabb_center(handle_turn_aabb)
        ctx.check(
            "turntable_handle_swings_with_yaw",
            handle_turn_center[0] > handle_rest_center[0] + 0.08 and handle_turn_center[1] > handle_rest_center[1] + 0.02,
            f"Rest handle center={handle_rest_center}, turned center={handle_turn_center}",
        )

    blade_rest_pos = ctx.part_world_position(blade)
    assert blade_rest_pos is not None
    arm_limits = arm_pitch.motion_limits
    assert arm_limits is not None and arm_limits.upper is not None
    with ctx.pose({arm_pitch: arm_limits.upper}):
        blade_cut_pos = ctx.part_world_position(blade)
        assert blade_cut_pos is not None
        ctx.check(
            "arm_pitch_lowers_blade",
            blade_cut_pos[2] < blade_rest_pos[2] - 0.18,
            f"Rest spindle height={blade_rest_pos[2]:.4f}, cut height={blade_cut_pos[2]:.4f}",
        )
        ctx.expect_gap(
            blade,
            turntable,
            axis="z",
            positive_elem="blade_disc",
            negative_elem="table_disk",
            min_gap=0.0,
            max_gap=0.016,
            name="blade_stops_just_above_table",
        )

    guard_rest_aabb = ctx.part_element_world_aabb(lower_guard, elem="lower_guard_shell")
    assert guard_rest_aabb is not None
    guard_rest_center = _aabb_center(guard_rest_aabb)
    guard_limits = guard_pivot.motion_limits
    assert guard_limits is not None and guard_limits.lower is not None
    with ctx.pose({guard_pivot: guard_limits.lower}):
        guard_open_aabb = ctx.part_element_world_aabb(lower_guard, elem="lower_guard_shell")
        assert guard_open_aabb is not None
        guard_open_center = _aabb_center(guard_open_aabb)
        ctx.check(
            "lower_guard_retracts_upward",
            guard_open_center[2] > guard_rest_center[2] + 0.03,
            f"Rest guard center={guard_rest_center}, open center={guard_open_center}",
        )
        ctx.expect_contact(lower_guard, blade_housing, name="lower_guard_stays_hinged_when_open")

    with ctx.pose({blade_spin: 1.7}):
        ctx.expect_contact(
            blade,
            blade_housing,
            elem_a="spindle_flange",
            elem_b="spindle_boss",
            name="blade_remains_supported_while_spinning",
        )

    for articulation in (table_yaw, arm_pitch, guard_pivot):
        limits = articulation.motion_limits
        assert limits is not None and limits.lower is not None and limits.upper is not None
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
