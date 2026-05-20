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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_cylinder_x(part, radius: float, length: float, *, xyz, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(part, radius: float, length: float, *, xyz, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_differential_pumpkin():
    profile = [
        (0.060, -0.240),
        (0.074, -0.205),
        (0.100, -0.165),
        (0.145, -0.085),
        (0.178, 0.000),
        (0.152, 0.095),
        (0.112, 0.165),
        (0.082, 0.205),
        (0.062, 0.240),
    ]
    return LatheGeometry(profile, segments=64).rotate_x(pi / 2.0)


def _build_half_axle(
    model: ArticulatedObject,
    *,
    name: str,
    side_sign: float,
    tube_paint,
    machined_steel,
    rubber,
):
    part = model.part(name)
    part.inertial = Inertial.from_geometry(
        Box((0.18, 0.58, 0.18)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.27 * side_sign, -0.01)),
    )

    part.visual(
        Box((0.140, 0.060, 0.160)),
        origin=Origin(xyz=(0.0, 0.075 * side_sign, 0.0)),
        material=tube_paint,
        name="pivot_bracket",
    )
    _add_cylinder_x(
        part,
        radius=0.040,
        length=0.140,
        xyz=(0.0, 0.030 * side_sign, 0.0),
        material=rubber,
        name="pivot_bushing",
    )
    _add_cylinder_y(
        part,
        radius=0.046,
        length=0.350,
        xyz=(0.0, 0.225 * side_sign, 0.0),
        material=tube_paint,
        name="axle_tube",
    )
    part.visual(
        Box((0.100, 0.200, 0.050)),
        origin=Origin(xyz=(0.0, 0.160 * side_sign, -0.055)),
        material=tube_paint,
        name="lower_brace",
    )
    _add_cylinder_y(
        part,
        radius=0.072,
        length=0.100,
        xyz=(0.0, 0.450 * side_sign, 0.0),
        material=machined_steel,
        name="outer_bell",
    )
    _add_cylinder_y(
        part,
        radius=0.061,
        length=0.020,
        xyz=(0.0, 0.510 * side_sign, 0.0),
        material=machined_steel,
        name="hub_mount_face",
    )
    _add_cylinder_x(
        part,
        radius=0.026,
        length=0.120,
        xyz=(0.0, 0.470 * side_sign, -0.018),
        material=machined_steel,
        name="pivot_reinforcement",
    )
    return part


def _build_hub(
    model: ArticulatedObject,
    *,
    name: str,
    side_sign: float,
    machined_steel,
    dark_steel,
):
    part = model.part(name)
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.220),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.110 * side_sign, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )

    _add_cylinder_y(
        part,
        radius=0.056,
        length=0.080,
        xyz=(0.0, 0.040 * side_sign, 0.0),
        material=dark_steel,
        name="inner_collar",
    )
    _add_cylinder_y(
        part,
        radius=0.100,
        length=0.008,
        xyz=(0.0, 0.004 * side_sign, 0.0),
        material=dark_steel,
        name="backing_plate",
    )
    _add_cylinder_y(
        part,
        radius=0.093,
        length=0.120,
        xyz=(0.0, 0.140 * side_sign, 0.0),
        material=machined_steel,
        name="hub_drum",
    )
    _add_cylinder_y(
        part,
        radius=0.125,
        length=0.020,
        xyz=(0.0, 0.210 * side_sign, 0.0),
        material=machined_steel,
        name="wheel_flange",
    )
    _add_cylinder_y(
        part,
        radius=0.034,
        length=0.032,
        xyz=(0.0, 0.226 * side_sign, 0.0),
        material=dark_steel,
        name="dust_cap",
    )
    stud_radius = 0.078
    for stud_index in range(5):
        angle = 2.0 * pi * stud_index / 5.0
        _add_cylinder_y(
            part,
            radius=0.0075,
            length=0.032,
            xyz=(stud_radius * cos(angle), 0.226 * side_sign, stud_radius * sin(angle)),
            material=dark_steel,
        )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_axle_rear_suspension")

    beam_paint = model.material("beam_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.24, 0.25, 0.27, 1.0))
    tube_paint = model.material("tube_paint", rgba=(0.20, 0.20, 0.21, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.33, 0.35, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    center_beam = model.part("center_beam")
    center_beam.inertial = Inertial.from_geometry(
        Box((1.20, 0.18, 0.22)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )
    center_beam.visual(
        Box((1.18, 0.14, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=beam_paint,
        name="beam_main",
    )
    center_beam.visual(
        Box((0.36, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=beam_paint,
        name="beam_saddle",
    )
    center_beam.visual(
        Box((0.14, 0.03, 0.10)),
        origin=Origin(xyz=(0.0, 0.055, 0.10)),
        material=beam_paint,
        name="left_saddle_gusset",
    )
    center_beam.visual(
        Box((0.14, 0.03, 0.10)),
        origin=Origin(xyz=(0.0, -0.055, 0.10)),
        material=beam_paint,
        name="right_saddle_gusset",
    )

    differential_housing = model.part("differential_housing")
    differential_housing.inertial = Inertial.from_geometry(
        Box((0.44, 0.56, 0.34)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )
    differential_housing.visual(
        Box((0.34, 0.22, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=cast_iron,
        name="mount_flange",
    )
    differential_housing.visual(
        Box((0.140, 0.120, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=cast_iron,
        name="mount_neck",
    )
    differential_housing.visual(
        Box((0.250, 0.240, 0.190)),
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        material=cast_iron,
        name="pumpkin_shell",
    )
    differential_housing.visual(
        Box((0.160, 0.068, 0.160)),
        origin=Origin(xyz=(0.0, 0.154, -0.185)),
        material=cast_iron,
        name="left_transition_case",
    )
    differential_housing.visual(
        Box((0.160, 0.068, 0.160)),
        origin=Origin(xyz=(0.0, -0.154, -0.185)),
        material=cast_iron,
        name="right_transition_case",
    )
    _add_cylinder_x(
        differential_housing,
        radius=0.102,
        length=0.050,
        xyz=(0.085, 0.0, -0.185),
        material=dark_steel,
        name="front_cover",
    )
    _add_cylinder_x(
        differential_housing,
        radius=0.078,
        length=0.050,
        xyz=(-0.080, 0.0, -0.185),
        material=cast_iron,
        name="rear_bearing_cap",
    )
    differential_housing.visual(
        Box((0.180, 0.014, 0.180)),
        origin=Origin(xyz=(0.0, 0.183, -0.185)),
        material=cast_iron,
        name="left_side_plate",
    )
    differential_housing.visual(
        Box((0.180, 0.014, 0.180)),
        origin=Origin(xyz=(0.0, -0.183, -0.185)),
        material=cast_iron,
        name="right_side_plate",
    )
    _add_cylinder_x(
        differential_housing,
        radius=0.055,
        length=0.120,
        xyz=(0.0, 0.205, -0.185),
        material=cast_iron,
        name="left_pivot_boss",
    )
    _add_cylinder_x(
        differential_housing,
        radius=0.055,
        length=0.120,
        xyz=(0.0, -0.205, -0.185),
        material=cast_iron,
        name="right_pivot_boss",
    )
    differential_housing.visual(
        Box((0.18, 0.24, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.295)),
        material=cast_iron,
        name="lower_rib",
    )
    for bolt_x in (-0.110, 0.110):
        for bolt_y in (-0.095, 0.095):
            differential_housing.visual(
                Cylinder(radius=0.010, length=0.020),
                origin=Origin(xyz=(bolt_x, bolt_y, 0.010)),
                material=machined_steel,
                name=f"mount_bolt_{'r' if bolt_x > 0 else 'l'}_{'u' if bolt_y > 0 else 'd'}",
            )
    cover_bolt_radius = 0.074
    for bolt_index in range(6):
        angle = 2.0 * pi * bolt_index / 6.0
        _add_cylinder_x(
            differential_housing,
            radius=0.008,
            length=0.022,
            xyz=(
                0.110,
                cover_bolt_radius * cos(angle),
                -0.185 + cover_bolt_radius * sin(angle),
            ),
            material=machined_steel,
            name=f"front_cover_bolt_{bolt_index}",
        )

    left_half_axle = _build_half_axle(
        model,
        name="left_half_axle",
        side_sign=1.0,
        tube_paint=tube_paint,
        machined_steel=machined_steel,
        rubber=rubber,
    )
    right_half_axle = _build_half_axle(
        model,
        name="right_half_axle",
        side_sign=-1.0,
        tube_paint=tube_paint,
        machined_steel=machined_steel,
        rubber=rubber,
    )
    left_hub = _build_hub(
        model,
        name="left_hub",
        side_sign=1.0,
        machined_steel=machined_steel,
        dark_steel=dark_steel,
    )
    right_hub = _build_hub(
        model,
        name="right_hub",
        side_sign=-1.0,
        machined_steel=machined_steel,
        dark_steel=dark_steel,
    )

    model.articulation(
        "beam_to_differential",
        ArticulationType.FIXED,
        parent=center_beam,
        child=differential_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )
    model.articulation(
        "left_swing",
        ArticulationType.REVOLUTE,
        parent=differential_housing,
        child=left_half_axle,
        origin=Origin(xyz=(0.0, 0.230, -0.185)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.55, upper=0.35),
    )
    model.articulation(
        "right_swing",
        ArticulationType.REVOLUTE,
        parent=differential_housing,
        child=right_half_axle,
        origin=Origin(xyz=(0.0, -0.230, -0.185)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.35, upper=0.55),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=left_half_axle,
        child=left_hub,
        origin=Origin(xyz=(0.0, 0.520, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=right_half_axle,
        child=right_hub,
        origin=Origin(xyz=(0.0, -0.520, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_beam = object_model.get_part("center_beam")
    differential_housing = object_model.get_part("differential_housing")
    left_half_axle = object_model.get_part("left_half_axle")
    right_half_axle = object_model.get_part("right_half_axle")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    left_swing = object_model.get_articulation("left_swing")
    right_swing = object_model.get_articulation("right_swing")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

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
    ctx.allow_overlap(
        differential_housing,
        left_half_axle,
        elem_a="left_pivot_boss",
        elem_b="pivot_bushing",
        reason="Swing-axle pivot modeled as a solid pivot boss nested inside a solid bushing envelope.",
    )
    ctx.allow_overlap(
        differential_housing,
        right_half_axle,
        elem_a="right_pivot_boss",
        elem_b="pivot_bushing",
        reason="Swing-axle pivot modeled as a solid pivot boss nested inside a solid bushing envelope.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        differential_housing,
        center_beam,
        elem_a="mount_flange",
        elem_b="beam_saddle",
        name="differential_flange_contacts_center_beam_saddle",
    )
    ctx.expect_contact(
        left_half_axle,
        differential_housing,
        elem_a="pivot_bushing",
        elem_b="left_pivot_boss",
        name="left_half_axle_pivots_from_left_diff_face",
    )
    ctx.expect_contact(
        right_half_axle,
        differential_housing,
        elem_a="pivot_bushing",
        elem_b="right_pivot_boss",
        name="right_half_axle_pivots_from_right_diff_face",
    )
    ctx.expect_contact(
        left_hub,
        left_half_axle,
        elem_a="inner_collar",
        elem_b="hub_mount_face",
        name="left_hub_seats_on_left_axle_tube",
    )
    ctx.expect_contact(
        right_hub,
        right_half_axle,
        elem_a="inner_collar",
        elem_b="hub_mount_face",
        name="right_hub_seats_on_right_axle_tube",
    )

    def _axis_close(actual, expected) -> bool:
        return all(abs(float(a) - float(b)) <= 1e-9 for a, b in zip(actual, expected))

    ctx.check(
        "swing_joint_axes_are_horizontal",
        _axis_close(left_swing.axis, (1.0, 0.0, 0.0)) and _axis_close(right_swing.axis, (1.0, 0.0, 0.0)),
        f"left axis={left_swing.axis}, right axis={right_swing.axis}",
    )
    ctx.check(
        "hub_spin_axes_follow_axle_line",
        _axis_close(left_hub_spin.axis, (0.0, 1.0, 0.0)) and _axis_close(right_hub_spin.axis, (0.0, 1.0, 0.0)),
        f"left axis={left_hub_spin.axis}, right axis={right_hub_spin.axis}",
    )

    rest_left_hub_pos = ctx.part_world_position(left_hub)
    rest_right_hub_pos = ctx.part_world_position(right_hub)
    if rest_left_hub_pos is not None:
        with ctx.pose({left_swing: 0.35}):
            swung_up = ctx.part_world_position(left_hub)
        with ctx.pose({left_swing: -0.35}):
            swung_down = ctx.part_world_position(left_hub)
        ctx.check(
            "left_swing_changes_hub_height",
            swung_up is not None
            and swung_down is not None
            and swung_up[2] > rest_left_hub_pos[2] + 0.10
            and swung_down[2] < rest_left_hub_pos[2] - 0.10,
            f"rest={rest_left_hub_pos}, up={swung_up}, down={swung_down}",
        )
    else:
        ctx.fail("left_hub_position_available", "left hub world position was unavailable")

    if rest_right_hub_pos is not None:
        with ctx.pose({right_swing: -0.35}):
            right_swung_up = ctx.part_world_position(right_hub)
        with ctx.pose({right_swing: 0.35}):
            right_swung_down = ctx.part_world_position(right_hub)
        with ctx.pose({right_hub_spin: 1.20}):
            spun_right = ctx.part_world_position(right_hub)
        ctx.check(
            "right_swing_raises_right_hub",
            right_swung_up is not None and right_swung_up[2] > rest_right_hub_pos[2] + 0.10,
            f"rest={rest_right_hub_pos}, up={right_swung_up}",
        )
        ctx.check(
            "right_swing_lowers_right_hub_in_opposite_direction",
            right_swung_down is not None and right_swung_down[2] < rest_right_hub_pos[2] - 0.10,
            f"rest={rest_right_hub_pos}, down={right_swung_down}",
        )
        ctx.check(
            "hub_spin_keeps_right_hub_origin_fixed",
            spun_right is not None
            and max(abs(spun_right[i] - rest_right_hub_pos[i]) for i in range(3)) <= 1e-6,
            f"rest={rest_right_hub_pos}, spun={spun_right}",
        )
    else:
        ctx.fail("right_hub_position_available", "right hub world position was unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
