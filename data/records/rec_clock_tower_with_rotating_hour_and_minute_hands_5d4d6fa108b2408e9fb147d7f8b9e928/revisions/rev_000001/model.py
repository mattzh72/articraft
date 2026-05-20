from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _arch_profile(width: float, height: float, *, segments: int = 20) -> list[tuple[float, float]]:
    radius = width * 0.5
    spring = (height * 0.5) - radius
    points = [(-radius, -height * 0.5), (radius, -height * 0.5), (radius, spring)]
    for step in range(segments + 1):
        angle = pi * step / segments
        points.append((radius * cos(angle), spring + radius * sin(angle)))
    points.append((-radius, -height * 0.5))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="renaissance_clock_tower")

    travertine = model.material("travertine", rgba=(0.78, 0.73, 0.63, 1.0))
    pale_stone = model.material("pale_stone", rgba=(0.86, 0.82, 0.72, 1.0))
    shadow_stone = model.material("shadow_stone", rgba=(0.48, 0.43, 0.36, 1.0))
    bronze = model.material("bronze", rgba=(0.48, 0.34, 0.18, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.67, 0.58, 0.34, 1.0))
    dial_paint = model.material("dial_paint", rgba=(0.94, 0.92, 0.84, 1.0))
    hand_black = model.material("hand_black", rgba=(0.12, 0.11, 0.10, 1.0))

    arcade_panel_mesh = _save_mesh(
        "arcade_panel",
        ExtrudeWithHolesGeometry(
            _rect_profile(3.06, 3.60),
            [_arch_profile(2.18, 3.18, segments=24)],
            0.38,
            center=True,
        ).rotate_x(pi / 2.0),
    )
    arch_shadow_mesh = _save_mesh(
        "arcade_shadow",
        ExtrudeGeometry(
            _arch_profile(2.18, 3.18, segments=24),
            0.24,
            center=True,
        ).rotate_x(pi / 2.0),
    )

    arcade_base = model.part("arcade_base")
    arcade_base.inertial = Inertial.from_geometry(
        Box((5.40, 5.40, 4.60)),
        mass=82000.0,
        origin=Origin(xyz=(0.0, 0.0, 2.30)),
    )
    arcade_base.visual(
        Box((5.40, 5.40, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=travertine,
        name="foundation_plinth",
    )
    arcade_base.visual(
        Box((4.50, 4.50, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 4.40)),
        material=travertine,
        name="arcade_entablature",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            arcade_base.visual(
                Box((0.72, 0.72, 3.60)),
                origin=Origin(xyz=(sx * 1.89, sy * 1.89, 2.40)),
                material=travertine,
                name=f"corner_pier_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
    for z_band, size_z in ((0.82, 0.18), (4.11, 0.18)):
        arcade_base.visual(
            Box((4.52, 0.12, size_z)),
            origin=Origin(xyz=(0.0, 2.31, z_band)),
            material=pale_stone,
            name=f"front_band_{int(z_band * 100)}",
        )
        arcade_base.visual(
            Box((4.52, 0.12, size_z)),
            origin=Origin(xyz=(0.0, -2.31, z_band)),
            material=pale_stone,
            name=f"rear_band_{int(z_band * 100)}",
        )
        arcade_base.visual(
            Box((0.12, 4.52, size_z)),
            origin=Origin(xyz=(2.31, 0.0, z_band)),
            material=pale_stone,
            name=f"right_band_{int(z_band * 100)}",
        )
        arcade_base.visual(
            Box((0.12, 4.52, size_z)),
            origin=Origin(xyz=(-2.31, 0.0, z_band)),
            material=pale_stone,
            name=f"left_band_{int(z_band * 100)}",
        )
    arcade_base.visual(
        arcade_panel_mesh,
        origin=Origin(xyz=(0.0, 2.06, 2.40)),
        material=travertine,
        name="front_arcade_panel",
    )
    arcade_base.visual(
        arch_shadow_mesh,
        origin=Origin(xyz=(0.0, 1.83, 2.40)),
        material=shadow_stone,
        name="front_arcade_shadow",
    )
    arcade_base.visual(
        arcade_panel_mesh,
        origin=Origin(xyz=(0.0, -2.06, 2.40), rpy=(0.0, 0.0, pi)),
        material=travertine,
        name="rear_arcade_panel",
    )
    arcade_base.visual(
        arch_shadow_mesh,
        origin=Origin(xyz=(0.0, -1.83, 2.40), rpy=(0.0, 0.0, pi)),
        material=shadow_stone,
        name="rear_arcade_shadow",
    )
    arcade_base.visual(
        arcade_panel_mesh,
        origin=Origin(xyz=(2.06, 0.0, 2.40), rpy=(0.0, 0.0, -pi / 2.0)),
        material=travertine,
        name="right_arcade_panel",
    )
    arcade_base.visual(
        arch_shadow_mesh,
        origin=Origin(xyz=(1.83, 0.0, 2.40), rpy=(0.0, 0.0, -pi / 2.0)),
        material=shadow_stone,
        name="right_arcade_shadow",
    )
    arcade_base.visual(
        arcade_panel_mesh,
        origin=Origin(xyz=(-2.06, 0.0, 2.40), rpy=(0.0, 0.0, pi / 2.0)),
        material=travertine,
        name="left_arcade_panel",
    )
    arcade_base.visual(
        arch_shadow_mesh,
        origin=Origin(xyz=(-1.83, 0.0, 2.40), rpy=(0.0, 0.0, pi / 2.0)),
        material=shadow_stone,
        name="left_arcade_shadow",
    )

    upper_shaft = model.part("upper_shaft")
    upper_shaft.inertial = Inertial.from_geometry(
        Box((3.96, 3.96, 11.00)),
        mass=104000.0,
        origin=Origin(xyz=(0.0, 0.0, 5.50)),
    )
    upper_shaft.visual(
        Box((3.96, 3.96, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=pale_stone,
        name="shaft_base_course",
    )
    upper_shaft.visual(
        Box((3.70, 3.70, 11.00)),
        origin=Origin(xyz=(0.0, 0.0, 5.50)),
        material=travertine,
        name="main_shaft",
    )
    upper_shaft.visual(
        Box((3.88, 3.88, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 9.65)),
        material=pale_stone,
        name="clock_level_stringcourse",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            upper_shaft.visual(
                Box((0.28, 0.28, 11.00)),
                origin=Origin(xyz=(sx * 1.71, sy * 1.71, 5.50)),
                material=pale_stone,
                name=f"shaft_quoin_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    cornice_tier = model.part("cornice_tier")
    cornice_tier.inertial = Inertial.from_geometry(
        Box((4.70, 4.70, 0.96)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
    )
    cornice_tier.visual(
        Box((3.98, 3.98, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=pale_stone,
        name="cornice_bed_mould",
    )
    cornice_tier.visual(
        Box((4.20, 4.20, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=travertine,
        name="cornice_lower_project",
    )
    cornice_tier.visual(
        Box((4.70, 4.70, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=pale_stone,
        name="cornice_corona",
    )
    cornice_tier.visual(
        Box((4.36, 4.36, 0.36)),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=travertine,
        name="cornice_cap",
    )

    clock_dial = model.part("clock_dial")
    clock_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=1.22, length=0.08),
        mass=120.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    clock_dial.visual(
        Cylinder(radius=1.22, length=0.08),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="bezel",
    )
    clock_dial.visual(
        Cylinder(radius=1.10, length=0.024),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dial_paint,
        name="dial_face",
    )
    for marker in range(12):
        angle = (pi / 2.0) - (marker * pi / 6.0)
        marker_height = 0.26 if marker % 3 == 0 else 0.18
        marker_width = 0.08 if marker % 3 == 0 else 0.06
        radius = 0.86
        clock_dial.visual(
            Box((marker_width, 0.012, marker_height)),
            origin=Origin(
                xyz=(radius * cos(angle), 0.046, radius * sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=aged_brass,
            name=f"hour_marker_{marker:02d}",
        )

    hour_hand = model.part("hour_hand")
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.22, 0.03, 0.86)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.055, 0.20)),
    )
    hour_hand.visual(
        Cylinder(radius=0.14, length=0.03),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hand_black,
        name="hour_hub",
    )
    hour_hand.visual(
        Box((0.14, 0.012, 0.60)),
        origin=Origin(xyz=(0.0, 0.058, 0.23)),
        material=hand_black,
        name="hour_stem",
    )
    hour_hand.visual(
        Box((0.10, 0.012, 0.24)),
        origin=Origin(xyz=(0.0, 0.058, -0.18)),
        material=hand_black,
        name="hour_tail",
    )
    hour_hand.visual(
        Box((0.06, 0.012, 0.24)),
        origin=Origin(xyz=(0.0, 0.058, 0.62)),
        material=hand_black,
        name="hour_tip",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.16, 0.02, 1.28)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.080, 0.26)),
    )
    minute_hand.visual(
        Cylinder(radius=0.10, length=0.02),
        origin=Origin(xyz=(0.0, 0.080, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hand_black,
        name="minute_hub",
    )
    minute_hand.visual(
        Box((0.09, 0.010, 0.98)),
        origin=Origin(xyz=(0.0, 0.083, 0.36)),
        material=hand_black,
        name="minute_stem",
    )
    minute_hand.visual(
        Box((0.06, 0.010, 0.28)),
        origin=Origin(xyz=(0.0, 0.083, -0.17)),
        material=hand_black,
        name="minute_tail",
    )
    minute_hand.visual(
        Box((0.04, 0.010, 0.30)),
        origin=Origin(xyz=(0.0, 0.083, 0.94)),
        material=hand_black,
        name="minute_tip",
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.FIXED,
        parent=arcade_base,
        child=upper_shaft,
        origin=Origin(xyz=(0.0, 0.0, 4.60)),
    )
    model.articulation(
        "shaft_to_cornice",
        ArticulationType.FIXED,
        parent=upper_shaft,
        child=cornice_tier,
        origin=Origin(xyz=(0.0, 0.0, 11.00)),
    )
    model.articulation(
        "shaft_to_clock_dial",
        ArticulationType.FIXED,
        parent=upper_shaft,
        child=clock_dial,
        origin=Origin(xyz=(0.0, 1.89, 8.00)),
    )
    model.articulation(
        "hour_hand_rotation",
        ArticulationType.REVOLUTE,
        parent=clock_dial,
        child=hour_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0, lower=-2.0 * pi, upper=2.0 * pi),
    )
    model.articulation(
        "minute_hand_rotation",
        ArticulationType.REVOLUTE,
        parent=clock_dial,
        child=minute_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0, lower=-2.0 * pi, upper=2.0 * pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    arcade_base = object_model.get_part("arcade_base")
    upper_shaft = object_model.get_part("upper_shaft")
    cornice_tier = object_model.get_part("cornice_tier")
    clock_dial = object_model.get_part("clock_dial")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    main_shaft = upper_shaft.get_visual("main_shaft")
    bezel = clock_dial.get_visual("bezel")
    dial_face = clock_dial.get_visual("dial_face")
    hour_joint = object_model.get_articulation("hour_hand_rotation")
    minute_joint = object_model.get_articulation("minute_hand_rotation")

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

    ctx.expect_contact(upper_shaft, arcade_base, name="upper_shaft_seats_on_arcade_base")
    ctx.expect_contact(cornice_tier, upper_shaft, name="cornice_caps_shaft")
    ctx.expect_gap(
        clock_dial,
        upper_shaft,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-6,
        negative_elem=main_shaft,
        name="clock_dial_is_flush_to_upper_shaft_front",
    )
    ctx.expect_overlap(
        clock_dial,
        upper_shaft,
        axes="xz",
        min_overlap=2.0,
        name="clock_dial_is_centered_on_upper_shaft_front",
    )
    ctx.expect_contact(hour_hand, clock_dial, elem_b=bezel, name="hour_hand_mounts_at_face_hub")
    ctx.expect_contact(minute_hand, hour_hand, name="minute_hand_stacks_on_hour_hand_hub")

    ctx.check(
        "hour_hand_joint_axis_is_face_normal",
        tuple(round(value, 3) for value in hour_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {hour_joint.axis}",
    )
    ctx.check(
        "minute_hand_joint_axis_is_face_normal",
        tuple(round(value, 3) for value in minute_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {minute_joint.axis}",
    )
    ctx.check(
        "clock_hands_have_bidirectional_motion",
        (
            hour_joint.motion_limits is not None
            and minute_joint.motion_limits is not None
            and hour_joint.motion_limits.lower is not None
            and hour_joint.motion_limits.upper is not None
            and minute_joint.motion_limits.lower is not None
            and minute_joint.motion_limits.upper is not None
            and hour_joint.motion_limits.lower < 0.0 < hour_joint.motion_limits.upper
            and minute_joint.motion_limits.lower < 0.0 < minute_joint.motion_limits.upper
        ),
        details="clock hand joints should swing in both rotational directions around the face hub",
    )

    with ctx.pose({"hour_hand_rotation": pi / 3.0, "minute_hand_rotation": -pi / 2.0}):
        ctx.expect_gap(
            hour_hand,
            clock_dial,
            axis="y",
            min_gap=0.0,
            max_gap=0.03,
            negative_elem=dial_face,
            name="hour_hand_remains_just_in_front_of_dial",
        )
        ctx.expect_gap(
            minute_hand,
            hour_hand,
            axis="y",
            min_gap=0.0,
            max_gap=0.02,
            name="minute_hand_remains_stacked_in_front_of_hour_hand",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
