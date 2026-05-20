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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_bolt_head(
    part,
    *,
    xyz: tuple[float, float, float],
    axis: str,
    radius: float,
    length: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def _add_wheel_visuals(part, *, side_sign: float, rubber, steel, hub_dark) -> None:
    inner_sign = -side_sign
    part.visual(
        Cylinder(radius=0.112, length=0.050),
        origin=Origin(rpy=_axis_rpy("y")),
        material=rubber,
        name="tread",
    )
    part.visual(
        Cylinder(radius=0.086, length=0.040),
        origin=Origin(rpy=_axis_rpy("y")),
        material=hub_dark,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, inner_sign * 0.020, 0.0), rpy=_axis_rpy("y")),
        material=steel,
        name="hub_boss",
    )
    part.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(0.0, -inner_sign * 0.020, 0.0), rpy=_axis_rpy("y")),
        material=steel,
        name="hub_cap",
    )
    _add_bolt_head(
        part,
        xyz=(0.0, -inner_sign * 0.022, 0.0),
        axis="y",
        radius=0.006,
        length=0.004,
        material=steel,
        name="retainer_head",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_wheelie_bin")

    body_poly = model.material("body_poly", rgba=(0.18, 0.20, 0.21, 1.0))
    lid_poly = model.material("lid_poly", rgba=(0.23, 0.25, 0.26, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.86, 0.73, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    body_depth = 0.710
    body_width = 0.580
    wall_t = 0.014
    shell_bottom = 0.080
    shell_top = 0.960
    shell_height = shell_top - shell_bottom
    half_depth = body_depth * 0.5
    half_width = body_width * 0.5
    wheel_center_x = -0.255
    wheel_center_y = 0.315
    wheel_radius = 0.112
    hinge_x = -0.364
    hinge_z = 0.974

    body = model.part("body")
    body.visual(
        Box((0.575, body_width - 2.0 * wall_t, 0.020)),
        origin=Origin(xyz=(0.0675, 0.0, 0.090)),
        material=body_poly,
        name="bottom_panel",
    )
    body.visual(
        Box((wall_t, body_width, shell_height)),
        origin=Origin(xyz=(half_depth - wall_t * 0.5, 0.0, (shell_bottom + shell_top) * 0.5)),
        material=body_poly,
        name="front_wall",
    )
    body.visual(
        Box((wall_t, body_width, shell_height)),
        origin=Origin(xyz=(-(half_depth - wall_t * 0.5), 0.0, (shell_bottom + shell_top) * 0.5)),
        material=body_poly,
        name="rear_wall",
    )
    body.visual(
        Box((body_depth - 2.0 * wall_t, wall_t, 0.700)),
        origin=Origin(xyz=(0.0, half_width - wall_t * 0.5, 0.610)),
        material=body_poly,
        name="left_wall",
    )
    body.visual(
        Box((body_depth - 2.0 * wall_t, wall_t, 0.700)),
        origin=Origin(xyz=(0.0, -(half_width - wall_t * 0.5), 0.610)),
        material=body_poly,
        name="right_wall",
    )
    body.visual(
        Box((0.400, wall_t, 0.180)),
        origin=Origin(xyz=(0.148, half_width - wall_t * 0.5, 0.170)),
        material=body_poly,
        name="left_lower_wall",
    )
    body.visual(
        Box((0.400, wall_t, 0.180)),
        origin=Origin(xyz=(0.148, -(half_width - wall_t * 0.5), 0.170)),
        material=body_poly,
        name="right_lower_wall",
    )
    body.visual(
        Box((0.028, 0.140, 0.700)),
        origin=Origin(xyz=(-0.327, 0.0, 0.440)),
        material=dark_steel,
        name="rear_spine",
    )
    body.visual(
        Box((0.090, 0.060, 0.080)),
        origin=Origin(xyz=(0.285, 0.180, 0.040)),
        material=guard_yellow,
        name="left_front_foot",
    )
    body.visual(
        Box((0.090, 0.060, 0.080)),
        origin=Origin(xyz=(0.285, -0.180, 0.040)),
        material=guard_yellow,
        name="right_front_foot",
    )
    body.visual(
        Box((0.030, 0.120, 0.420)),
        origin=Origin(xyz=(0.336, 0.180, 0.320)),
        material=dark_steel,
        name="left_front_rib",
    )
    body.visual(
        Box((0.030, 0.120, 0.420)),
        origin=Origin(xyz=(0.336, -0.180, 0.320)),
        material=dark_steel,
        name="right_front_rib",
    )
    body.visual(
        Box((0.040, 0.240, 0.042)),
        origin=Origin(xyz=(-0.332, 0.0, 0.948)),
        material=dark_steel,
        name="rear_bridge",
    )
    body.visual(
        Box((0.016, 0.090, 0.060)),
        origin=Origin(xyz=(0.363, 0.0, 0.870)),
        material=steel,
        name="keeper_anchor",
    )
    body.visual(
        Box((0.014, 0.080, 0.052)),
        origin=Origin(xyz=(0.368, 0.0, 0.870)),
        material=steel,
        name="front_keeper",
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((0.220, 0.080, 0.012)),
            origin=Origin(xyz=(-0.268, side_sign * 0.325, 0.271)),
            material=guard_yellow,
            name=f"{side_name}_wheel_guard_top",
        )
        body.visual(
            Box((0.110, 0.012, 0.170)),
            origin=Origin(xyz=(-0.260, side_sign * 0.355, 0.183)),
            material=guard_yellow,
            name=f"{side_name}_wheel_guard_outer",
        )
        body.visual(
            Box((0.012, 0.090, 0.180)),
            origin=Origin(xyz=(-0.377, side_sign * 0.315, 0.180)),
            material=guard_yellow,
            name=f"{side_name}_wheel_guard_rear",
        )
        body.visual(
            Box((0.090, 0.060, 0.050)),
            origin=Origin(xyz=(-0.266, side_sign * 0.205, 0.255)),
            material=dark_steel,
            name=f"{side_name}_axle_bracket",
        )
        body.visual(
            Box((0.026, 0.036, 0.240)),
            origin=Origin(xyz=(-0.322, side_sign * 0.206, 0.190)),
            material=dark_steel,
            name=f"{side_name}_axle_upright",
        )
        body.visual(
            Box((0.030, 0.030, 0.140)),
            origin=Origin(xyz=(-0.326, side_sign * 0.190, 0.250)),
            material=dark_steel,
            name=f"{side_name}_axle_brace",
        )
        body.visual(
            Box((0.050, 0.022, 0.110)),
            origin=Origin(xyz=(-0.378, side_sign * 0.257, 0.930)),
            material=guard_yellow,
            name=f"{side_name}_hinge_cheek",
        )
        body.visual(
            Box((0.018, 0.050, 0.060)),
            origin=Origin(xyz=(-0.396, side_sign * 0.257, 0.924)),
            material=dark_steel,
            name=f"{side_name}_hinge_backstrap",
        )
        body.visual(
            Box((0.100, 0.028, 0.090)),
            origin=Origin(xyz=(-0.320, side_sign * 0.255, 0.860)),
            material=dark_steel,
            name=f"{side_name}_hinge_reinforcement",
        )
        body.visual(
            Box((0.024, 0.048, 0.090)),
            origin=Origin(xyz=(-0.362, side_sign * 0.160, 0.900)),
            material=steel,
            name=f"{side_name}_stop_block",
        )
        _add_bolt_head(
            body,
            xyz=(-0.242, side_sign * 0.361, 0.215),
            axis="y",
            radius=0.006,
            length=0.008,
            material=steel,
            name=f"{side_name}_guard_bolt_upper",
        )
        _add_bolt_head(
            body,
            xyz=(-0.280, side_sign * 0.361, 0.135),
            axis="y",
            radius=0.006,
            length=0.008,
            material=steel,
            name=f"{side_name}_guard_bolt_lower",
        )
        _add_bolt_head(
            body,
            xyz=(-0.392, side_sign * 0.270, 0.940),
            axis="y",
            radius=0.006,
            length=0.004,
            material=steel,
            name=f"{side_name}_hinge_bolt_upper",
        )
        _add_bolt_head(
            body,
            xyz=(-0.392, side_sign * 0.270, 0.895),
            axis="y",
            radius=0.006,
            length=0.004,
            material=steel,
            name=f"{side_name}_hinge_bolt_lower",
        )

    _add_bolt_head(
        body,
        xyz=(0.373, 0.025, 0.887),
        axis="x",
        radius=0.0055,
        length=0.010,
        material=steel,
        name="keeper_bolt_upper",
    )
    _add_bolt_head(
        body,
        xyz=(0.373, -0.025, 0.853),
        axis="x",
        radius=0.0055,
        length=0.010,
        material=steel,
        name="keeper_bolt_lower",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.760, 0.700, 1.000)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
    )

    axle = model.part("rear_axle")
    axle.visual(
        Cylinder(radius=0.014, length=0.540),
        origin=Origin(xyz=(wheel_center_x, 0.0, wheel_radius), rpy=_axis_rpy("y")),
        material=dark_steel,
        name="axle_shaft",
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        axle.visual(
            Box((0.050, 0.030, 0.120)),
            origin=Origin(xyz=(wheel_center_x, side_sign * 0.205, 0.170)),
            material=dark_steel,
            name=f"{side_name}_hanger",
        )
        axle.visual(
            Cylinder(radius=0.022, length=0.020),
            origin=Origin(xyz=(wheel_center_x, side_sign * 0.280, wheel_radius), rpy=_axis_rpy("y")),
            material=steel,
            name=f"{side_name}_end_cap",
        )
    axle.inertial = Inertial.from_geometry(
        Box((0.080, 0.600, 0.160)),
        mass=3.2,
        origin=Origin(xyz=(wheel_center_x, 0.0, 0.140)),
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(left_wheel, side_sign=1.0, rubber=rubber, steel=steel, hub_dark=dark_steel)
    left_wheel.inertial = Inertial.from_geometry(
        Box((0.224, 0.050, 0.224)),
        mass=2.6,
        origin=Origin(),
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(right_wheel, side_sign=-1.0, rubber=rubber, steel=steel, hub_dark=dark_steel)
    right_wheel.inertial = Inertial.from_geometry(
        Box((0.224, 0.050, 0.224)),
        mass=2.6,
        origin=Origin(),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.790, 0.586, 0.018)),
        origin=Origin(xyz=(0.395, 0.0, 0.022)),
        material=lid_poly,
        name="top_panel",
    )
    lid.visual(
        Box((0.770, 0.014, 0.050)),
        origin=Origin(xyz=(0.390, 0.293, -0.003)),
        material=lid_poly,
        name="left_skirt",
    )
    lid.visual(
        Box((0.770, 0.014, 0.050)),
        origin=Origin(xyz=(0.390, -0.293, -0.003)),
        material=lid_poly,
        name="right_skirt",
    )
    lid.visual(
        Box((0.018, 0.580, 0.050)),
        origin=Origin(xyz=(0.781, 0.0, -0.003)),
        material=lid_poly,
        name="front_skirt",
    )
    lid.visual(
        Cylinder(radius=0.016, length=0.480),
        origin=Origin(rpy=_axis_rpy("y")),
        material=dark_steel,
        name="rear_tube",
    )
    lid.visual(
        Box((0.120, 0.050, 0.008)),
        origin=Origin(xyz=(0.072, 0.185, 0.011)),
        material=steel,
        name="left_hinge_strap",
    )
    lid.visual(
        Box((0.120, 0.050, 0.008)),
        origin=Origin(xyz=(0.072, -0.185, 0.011)),
        material=steel,
        name="right_hinge_strap",
    )
    lid.visual(
        Box((0.520, 0.040, 0.022)),
        origin=Origin(xyz=(0.320, 0.160, 0.002)),
        material=dark_steel,
        name="left_longitudinal_rib",
    )
    lid.visual(
        Box((0.520, 0.040, 0.022)),
        origin=Origin(xyz=(0.320, -0.160, 0.002)),
        material=dark_steel,
        name="right_longitudinal_rib",
    )
    lid.visual(
        Box((0.050, 0.420, 0.022)),
        origin=Origin(xyz=(0.610, 0.0, 0.002)),
        material=dark_steel,
        name="cross_rib",
    )
    lid.visual(
        Box((0.034, 0.060, 0.056)),
        origin=Origin(xyz=(0.773, 0.0, -0.012)),
        material=steel,
        name="lock_tongue",
    )
    lid.visual(
        Box((0.070, 0.050, 0.022)),
        origin=Origin(xyz=(0.060, 0.160, -0.010)),
        material=steel,
        name="left_stop_pad",
    )
    lid.visual(
        Box((0.070, 0.050, 0.022)),
        origin=Origin(xyz=(0.060, -0.160, -0.010)),
        material=steel,
        name="right_stop_pad",
    )
    _add_bolt_head(
        lid,
        xyz=(0.070, 0.185, 0.033),
        axis="z",
        radius=0.006,
        length=0.004,
        material=steel,
        name="left_hinge_fastener",
    )
    _add_bolt_head(
        lid,
        xyz=(0.070, -0.185, 0.033),
        axis="z",
        radius=0.006,
        length=0.004,
        material=steel,
        name="right_hinge_fastener",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.730, 0.600, 0.090)),
        mass=4.4,
        origin=Origin(xyz=(0.360, 0.0, 0.005)),
    )

    model.articulation(
        "body_to_axle",
        ArticulationType.FIXED,
        parent=body,
        child=axle,
        origin=Origin(),
    )
    model.articulation(
        "axle_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=left_wheel,
        origin=Origin(xyz=(wheel_center_x, wheel_center_y, wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "axle_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=right_wheel,
        origin=Origin(xyz=(wheel_center_x, -wheel_center_y, wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=2.5,
            lower=0.0,
            upper=1.75,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    axle = object_model.get_part("rear_axle")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")
    left_spin = object_model.get_articulation("axle_to_left_wheel")
    right_spin = object_model.get_articulation("axle_to_right_wheel")

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

    ctx.expect_contact(axle, body, elem_a="left_hanger", elem_b="left_axle_bracket")
    ctx.expect_contact(axle, body, elem_a="right_hanger", elem_b="right_axle_bracket")
    ctx.expect_contact(left_wheel, axle, elem_a="hub_boss", elem_b="left_end_cap")
    ctx.expect_contact(right_wheel, axle, elem_a="hub_boss", elem_b="right_end_cap")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.45)
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem="lock_tongue",
            negative_elem="front_keeper",
            min_gap=0.002,
            max_gap=0.020,
        )

    with ctx.pose({lid_hinge: 1.55}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="rear_bridge",
            min_gap=0.28,
        )

    with ctx.pose({left_spin: 1.1, right_spin: -0.8}):
        ctx.expect_contact(left_wheel, axle, elem_a="hub_boss", elem_b="left_end_cap")
        ctx.expect_contact(right_wheel, axle, elem_a="hub_boss", elem_b="right_end_cap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
