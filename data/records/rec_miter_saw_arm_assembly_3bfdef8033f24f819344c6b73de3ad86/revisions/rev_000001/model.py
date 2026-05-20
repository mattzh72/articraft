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
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _arc_points(
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int = 24,
) -> list[tuple[float, float]]:
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    return [
        (radius * math.cos(start + (end - start) * (i / segments)), radius * math.sin(start + (end - start) * (i / segments)))
        for i in range(segments + 1)
    ]


def _annular_sector_profile(
    *,
    inner_radius: float,
    outer_radius: float,
    start_deg: float,
    end_deg: float,
    segments: int = 28,
) -> list[tuple[float, float]]:
    outer = _arc_points(outer_radius, start_deg, end_deg, segments=segments)
    inner = list(reversed(_arc_points(inner_radius, start_deg, end_deg, segments=segments)))
    return outer + inner


def _build_base_body_mesh():
    return ExtrudeGeometry(rounded_rect_profile(0.40, 0.28, 0.05), 0.06, center=True)


def _build_arm_beam_mesh():
    return sweep_profile_along_spline(
        [
            (0.0, 0.012, 0.055),
            (0.0, -0.030, 0.100),
            (0.0, -0.075, 0.132),
            (0.0, -0.120, 0.138),
        ],
        profile=rounded_rect_profile(0.024, 0.058, radius=0.008),
        samples_per_segment=16,
        cap_profile=True,
    )


def _build_upper_cover_mesh():
    return ExtrudeGeometry(
        _annular_sector_profile(
            inner_radius=0.136,
            outer_radius=0.168,
            start_deg=15.0,
            end_deg=175.0,
            segments=34,
        ),
        0.088,
        center=True,
    )


def _build_lower_guard_mesh():
    return ExtrudeGeometry(
        _annular_sector_profile(
            inner_radius=0.130,
            outer_radius=0.154,
            start_deg=188.0,
            end_deg=345.0,
            segments=34,
        ),
        0.076,
        center=True,
    )


def _build_handle_mesh():
    return tube_from_spline_points(
        [
            (-0.024, -0.098, 0.178),
            (-0.030, -0.120, 0.196),
            (0.030, -0.120, 0.196),
            (0.024, -0.098, 0.178),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_miter_saw")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.74, 0.77, 0.79, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.44, 0.48, 0.52, 1.0))
    dark_charcoal = model.material("dark_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.80, 0.82, 0.84, 1.0))
    accent_red = model.material("accent_red", rgba=(0.76, 0.18, 0.12, 1.0))
    translucent_guard = model.material("translucent_guard", rgba=(0.72, 0.78, 0.83, 0.82))

    base = model.part("base")
    base.visual(
        _mesh("miter_saw_base_body", _build_base_body_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=cast_aluminum,
        name="base_body",
    )
    base.visual(
        Box((0.094, 0.060, 0.020)),
        origin=Origin(xyz=(-0.125, -0.090, 0.010)),
        material=dark_charcoal,
        name="left_foot",
    )
    base.visual(
        Box((0.094, 0.060, 0.020)),
        origin=Origin(xyz=(0.125, -0.090, 0.010)),
        material=dark_charcoal,
        name="right_foot",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=machine_gray,
        name="turntable_pedestal",
    )
    base.visual(
        Box((0.110, 0.090, 0.012)),
        origin=Origin(xyz=(-0.130, 0.008, 0.066)),
        material=cast_aluminum,
        name="left_wing",
    )
    base.visual(
        Box((0.110, 0.090, 0.012)),
        origin=Origin(xyz=(0.130, 0.008, 0.066)),
        material=cast_aluminum,
        name="right_wing",
    )
    base.visual(
        Box((0.240, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.140, 0.075)),
        material=machine_gray,
        name="rear_support_block",
    )
    base.visual(
        Box((0.240, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.140, 0.100)),
        material=machine_gray,
        name="rear_bridge",
    )
    base.visual(
        Box((0.028, 0.032, 0.045)),
        origin=Origin(xyz=(-0.120, 0.090, 0.0825)),
        material=cast_aluminum,
        name="left_fence_post",
    )
    base.visual(
        Box((0.028, 0.032, 0.045)),
        origin=Origin(xyz=(0.120, 0.090, 0.0825)),
        material=cast_aluminum,
        name="right_fence_post",
    )
    base.visual(
        Box((0.160, 0.016, 0.058)),
        origin=Origin(xyz=(-0.105, 0.090, 0.134)),
        material=cast_aluminum,
        name="left_fence",
    )
    base.visual(
        Box((0.160, 0.016, 0.058)),
        origin=Origin(xyz=(0.105, 0.090, 0.134)),
        material=cast_aluminum,
        name="right_fence",
    )
    base.visual(
        Box((0.032, 0.038, 0.160)),
        origin=Origin(xyz=(-0.096, 0.140, 0.170)),
        material=machine_gray,
        name="left_pivot_tower",
    )
    base.visual(
        Box((0.032, 0.038, 0.160)),
        origin=Origin(xyz=(0.096, 0.140, 0.170)),
        material=machine_gray,
        name="right_pivot_tower",
    )
    base.visual(
        Box((0.070, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.122, 0.065)),
        material=accent_red,
        name="miter_pointer",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.28, 0.28)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.115, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_aluminum,
        name="table_disk",
    )
    turntable.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=machine_gray,
        name="table_hub",
    )
    turntable.visual(
        Box((0.175, 0.026, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=dark_charcoal,
        name="kerf_insert",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.026),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    saw_arm = model.part("saw_arm")
    saw_arm.visual(
        Cylinder(radius=0.022, length=0.160),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_gray,
        name="pivot_barrel",
    )
    saw_arm.visual(
        _mesh("miter_saw_arm_beam", _build_arm_beam_mesh()),
        material=machine_gray,
        name="arm_beam",
    )
    saw_arm.visual(
        Box((0.042, 0.055, 0.048)),
        origin=Origin(xyz=(0.046, -0.088, 0.118)),
        material=machine_gray,
        name="gearbox_support",
    )
    saw_arm.visual(
        Cylinder(radius=0.052, length=0.048),
        origin=Origin(xyz=(0.062, -0.110, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_gray,
        name="gearbox_housing",
    )
    saw_arm.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.013, -0.110, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="arbor_flange",
    )
    saw_arm.visual(
        Cylinder(radius=0.040, length=0.112),
        origin=Origin(xyz=(0.122, -0.118, 0.098), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="motor_body",
    )
    saw_arm.visual(
        Cylinder(radius=0.025, length=0.028),
        origin=Origin(xyz=(0.188, -0.118, 0.098), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="motor_endcap",
    )
    saw_arm.visual(
        _mesh("miter_saw_upper_cover", _build_upper_cover_mesh()),
        origin=Origin(xyz=(0.0, -0.110, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_gray,
        name="upper_cover",
    )
    saw_arm.visual(
        Box((0.035, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, -0.112, 0.152)),
        material=dark_charcoal,
        name="handle_mount",
    )
    saw_arm.visual(
        Box((0.014, 0.020, 0.050)),
        origin=Origin(xyz=(-0.024, -0.102, 0.158)),
        material=dark_charcoal,
        name="left_handle_post",
    )
    saw_arm.visual(
        Box((0.014, 0.020, 0.050)),
        origin=Origin(xyz=(0.024, -0.102, 0.158)),
        material=dark_charcoal,
        name="right_handle_post",
    )
    saw_arm.visual(
        _mesh("miter_saw_carry_handle", _build_handle_mesh()),
        material=black_rubber,
        name="carry_handle",
    )
    saw_arm.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.0, -0.120, 0.196), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="front_grip",
    )
    saw_arm.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.036, -0.090, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_gray,
        name="guard_hinge_boss",
    )
    saw_arm.inertial = Inertial.from_geometry(
        Box((0.22, 0.20, 0.24)),
        mass=4.2,
        origin=Origin(xyz=(0.050, -0.070, 0.110)),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.127, length=0.0038),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_steel,
        name="blade_disk",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.127, length=0.010),
        mass=0.7,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    lower_guard = model.part("lower_guard")
    lower_guard.visual(
        _mesh("miter_saw_lower_guard", _build_lower_guard_mesh()),
        origin=Origin(xyz=(0.036, -0.020, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=translucent_guard,
        name="guard_shell",
    )
    lower_guard.visual(
        Box((0.012, 0.022, 0.100)),
        origin=Origin(xyz=(-0.008, -0.029, 0.032)),
        material=machine_gray,
        name="guard_link",
    )
    lower_guard.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_gray,
        name="guard_hinge_boss",
    )
    lower_guard.inertial = Inertial.from_geometry(
        Box((0.08, 0.18, 0.22)),
        mass=0.45,
        origin=Origin(xyz=(0.020, -0.030, -0.020)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=math.radians(-52.0),
            upper=math.radians(52.0),
        ),
    )
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=saw_arm,
        origin=Origin(xyz=(0.0, 0.125, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=0.90,
        ),
    )
    model.articulation(
        "arm_to_blade",
        ArticulationType.CONTINUOUS,
        parent=saw_arm,
        child=blade,
        origin=Origin(xyz=(0.0, -0.110, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )
    model.articulation(
        "arm_to_lower_guard",
        ArticulationType.REVOLUTE,
        parent=saw_arm,
        child=lower_guard,
        origin=Origin(xyz=(-0.036, -0.090, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=math.radians(-80.0),
            upper=math.radians(15.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    saw_arm = object_model.get_part("saw_arm")
    blade = object_model.get_part("blade")
    lower_guard = object_model.get_part("lower_guard")

    table_joint = object_model.get_articulation("base_to_turntable")
    arm_joint = object_model.get_articulation("base_to_arm")
    blade_joint = object_model.get_articulation("arm_to_blade")
    guard_joint = object_model.get_articulation("arm_to_lower_guard")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(turntable, base, elem_a="table_disk", elem_b="turntable_pedestal")
    ctx.expect_contact(saw_arm, base)
    ctx.expect_contact(lower_guard, saw_arm, elem_a="guard_hinge_boss", elem_b="guard_hinge_boss")
    ctx.expect_gap(blade, turntable, axis="z", min_gap=0.10)
    ctx.expect_overlap(blade, saw_arm, axes="yz", min_overlap=0.10, elem_a="blade_disk", elem_b="upper_cover")
    ctx.expect_overlap(lower_guard, blade, axes="yz", min_overlap=0.10, elem_a="guard_shell", elem_b="blade_disk")
    ctx.expect_overlap(turntable, base, axes="xy", min_overlap=0.10)

    ctx.check(
        "turntable_axis_is_vertical",
        table_joint.axis == (0.0, 0.0, 1.0),
        f"Expected vertical turntable axis, got {table_joint.axis!r}",
    )
    ctx.check(
        "arm_axis_is_horizontal",
        arm_joint.axis == (1.0, 0.0, 0.0),
        f"Expected horizontal arm pivot axis, got {arm_joint.axis!r}",
    )
    ctx.check(
        "blade_axis_matches_arbor",
        blade_joint.axis == (1.0, 0.0, 0.0),
        f"Expected blade arbor axis along X, got {blade_joint.axis!r}",
    )
    ctx.check(
        "guard_axis_matches_blade_cover_hinge",
        guard_joint.axis == (1.0, 0.0, 0.0),
        f"Expected lower guard hinge axis along X, got {guard_joint.axis!r}",
    )

    blade_rest = ctx.part_world_position(blade)
    with ctx.pose({arm_joint: 0.75}):
        blade_lowered = ctx.part_world_position(blade)
        ctx.expect_gap(blade, turntable, axis="z", min_gap=0.0, max_gap=0.03)
        ctx.expect_overlap(lower_guard, blade, axes="yz", min_overlap=0.09, elem_a="guard_shell", elem_b="blade_disk")

    arm_drop_ok = (
        blade_rest is not None
        and blade_lowered is not None
        and blade_lowered[2] < blade_rest[2] - 0.09
    )
    ctx.check(
        "arm_lowers_blade_toward_table",
        arm_drop_ok,
        f"Blade positions did not show a realistic drop: rest={blade_rest}, lowered={blade_lowered}",
    )

    with ctx.pose({table_joint: math.radians(40.0)}):
        ctx.expect_contact(turntable, base, elem_a="table_disk", elem_b="turntable_pedestal")
        ctx.expect_overlap(turntable, base, axes="xy", min_overlap=0.10)

    with ctx.pose({guard_joint: math.radians(-45.0), arm_joint: 0.55}):
        ctx.expect_contact(lower_guard, saw_arm, elem_a="guard_hinge_boss", elem_b="guard_hinge_boss")
        ctx.expect_overlap(lower_guard, blade, axes="yz", min_overlap=0.09, elem_a="guard_shell", elem_b="blade_disk")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
