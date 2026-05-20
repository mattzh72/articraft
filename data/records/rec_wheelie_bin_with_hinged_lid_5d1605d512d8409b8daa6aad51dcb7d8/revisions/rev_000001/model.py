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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_loop(
    width: float,
    depth: float,
    corner_radius: float,
    *,
    z: float,
    y_shift: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        depth,
        min(corner_radius, width * 0.49, depth * 0.49),
        corner_segments=corner_segments,
    )
    return [(x, y + y_shift, z) for x, y in profile]


def _wheel_shell_mesh(name: str, *, radius: float, width: float, hole_radius: float):
    half_width = width * 0.5
    outer_profile = [
        (radius * 0.67, -half_width),
        (radius * 0.85, -half_width * 0.98),
        (radius * 0.96, -half_width * 0.76),
        (radius, -half_width * 0.34),
        (radius, half_width * 0.34),
        (radius * 0.96, half_width * 0.76),
        (radius * 0.85, half_width * 0.98),
        (radius * 0.67, half_width),
    ]
    inner_profile = [
        (hole_radius, -half_width),
        (hole_radius, half_width),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            lip_samples=8,
        ).rotate_y(math.pi / 2.0),
    )


def _wheel_cap_ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    depth: float,
):
    half_depth = depth * 0.5
    outer_profile = [
        (outer_radius * 0.90, -half_depth),
        (outer_radius, -half_depth * 0.55),
        (outer_radius, half_depth * 0.55),
        (outer_radius * 0.90, half_depth),
    ]
    inner_profile = [
        (inner_radius, -half_depth),
        (inner_radius, half_depth),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
            lip_samples=6,
        ).rotate_y(math.pi / 2.0),
    )


def _add_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    side_sign: float,
    tire_radius: float,
    tire_width: float,
    hub_trim_radius: float,
    rubber,
    hub_polymer,
    steel,
) -> None:
    part.visual(
        _wheel_shell_mesh(
            f"{mesh_prefix}_shell",
            radius=tire_radius,
            width=tire_width,
            hole_radius=0.022,
        ),
        material=rubber,
        name="wheel_shell",
    )
    part.visual(
        Cylinder(radius=0.027, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_polymer,
        name="hub_barrel",
    )
    cap_mesh = _wheel_cap_ring_mesh(
        f"{mesh_prefix}_cap",
        outer_radius=hub_trim_radius,
        inner_radius=0.027,
        depth=0.008,
    )
    part.visual(
        cap_mesh,
        origin=Origin(xyz=(0.017 * side_sign, 0.0, 0.0)),
        material=hub_polymer,
        name="outer_cap",
    )
    part.visual(
        _wheel_cap_ring_mesh(
            f"{mesh_prefix}_bearing_trim",
            outer_radius=0.042,
            inner_radius=0.024,
            depth=0.006,
        ),
        origin=Origin(xyz=(-0.017 * side_sign, 0.0, 0.0)),
        material=steel,
        name="inner_trim",
    )
    part.visual(
        Box((0.006, 0.010, 0.018)),
        origin=Origin(
            xyz=(0.021 * side_sign, 0.066, 0.068),
            rpy=(0.0, 0.0, 0.15 * side_sign),
        ),
        material=rubber,
        name="valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wheelie_bin")

    painted_steel = model.material("painted_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    lid_polymer = model.material("lid_polymer", rgba=(0.17, 0.18, 0.19, 1.0))
    hub_polymer = model.material("hub_polymer", rgba=(0.52, 0.54, 0.57, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    elastomer = model.material("elastomer", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.64, 0.80, 1.02)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.01, 0.51)),
    )

    body.visual(
        Box((0.490, 0.600, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.074)),
        material=painted_steel,
        name="floor",
    )
    body.visual(
        Box((0.532, 0.016, 0.900)),
        origin=Origin(xyz=(0.0, 0.314, 0.525), rpy=(0.050, 0.0, 0.0)),
        material=painted_steel,
        name="front_wall",
    )
    body.visual(
        Box((0.532, 0.016, 0.920)),
        origin=Origin(xyz=(0.0, -0.286, 0.535), rpy=(-0.062, 0.0, 0.0)),
        material=painted_steel,
        name="rear_wall",
    )
    body.visual(
        Box((0.016, 0.650, 0.905)),
        origin=Origin(xyz=(0.255, 0.015, 0.527), rpy=(0.0, 0.055, 0.0)),
        material=painted_steel,
        name="left_wall",
    )
    body.visual(
        Box((0.016, 0.650, 0.905)),
        origin=Origin(xyz=(-0.255, 0.015, 0.527), rpy=(0.0, -0.055, 0.0)),
        material=painted_steel,
        name="right_wall",
    )

    for name, x, y in (
        ("front_left_corner", 0.237, 0.278),
        ("front_right_corner", -0.237, 0.278),
        ("rear_left_corner", 0.231, -0.258),
        ("rear_right_corner", -0.231, -0.258),
    ):
        body.visual(
            Cylinder(radius=0.035, length=0.885),
            origin=Origin(xyz=(x, y, 0.515)),
            material=painted_steel,
            name=name,
        )

    rim_mesh = _save_mesh(
        "wheelie_bin_rim",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.578, 0.718, 0.060, corner_segments=8),
            [rounded_rect_profile(0.548, 0.688, 0.046, corner_segments=8)],
            height=0.028,
            center=True,
        ),
    )
    body.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, 0.015, 0.961)),
        material=painted_steel,
        name="rim",
    )

    body.visual(
        Box((0.332, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, 0.344, 0.862)),
        material=painted_steel,
        name="front_lift_bar",
    )
    body.visual(
        Box((0.072, 0.072, 0.090)),
        origin=Origin(xyz=(0.197, 0.330, 0.858)),
        material=painted_steel,
        name="left_lift_bar_bracket",
    )
    body.visual(
        Box((0.072, 0.072, 0.090)),
        origin=Origin(xyz=(-0.197, 0.330, 0.858)),
        material=painted_steel,
        name="right_lift_bar_bracket",
    )
    body.visual(
        Box((0.340, 0.030, 0.075)),
        origin=Origin(xyz=(0.0, 0.316, 0.0375)),
        material=elastomer,
        name="front_bumper",
    )
    body.visual(
        Box((0.440, 0.050, 0.075)),
        origin=Origin(xyz=(0.0, -0.345, 0.925)),
        material=painted_steel,
        name="rear_handle_spine",
    )
    body.visual(
        Box((0.085, 0.095, 0.180)),
        origin=Origin(xyz=(0.205, -0.316, 0.860)),
        material=painted_steel,
        name="left_rear_shoulder",
    )
    body.visual(
        Box((0.085, 0.095, 0.180)),
        origin=Origin(xyz=(-0.205, -0.316, 0.860)),
        material=painted_steel,
        name="right_rear_shoulder",
    )

    body.visual(
        Box((0.094, 0.070, 0.060)),
        origin=Origin(xyz=(0.176, -0.338, 0.940)),
        material=painted_steel,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.094, 0.070, 0.060)),
        origin=Origin(xyz=(-0.176, -0.338, 0.940)),
        material=painted_steel,
        name="right_hinge_bracket",
    )
    body.visual(
        Box((0.338, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, -0.382, 0.958)),
        material=painted_steel,
        name="hinge_backplate",
    )

    body.visual(
        Cylinder(radius=0.010, length=0.550),
        origin=Origin(xyz=(0.0, -0.315, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="rear_axle",
    )
    body.visual(
        Cylinder(radius=0.042, length=0.006),
        origin=Origin(xyz=(0.278, -0.315, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="left_axle_stub",
    )
    body.visual(
        Cylinder(radius=0.042, length=0.006),
        origin=Origin(xyz=(-0.278, -0.315, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="right_axle_stub",
    )
    body.visual(
        Box((0.050, 0.160, 0.120)),
        origin=Origin(xyz=(0.220, -0.248, 0.135)),
        material=painted_steel,
        name="left_axle_cradle",
    )
    body.visual(
        Box((0.050, 0.160, 0.120)),
        origin=Origin(xyz=(-0.220, -0.248, 0.135)),
        material=painted_steel,
        name="right_axle_cradle",
    )
    body.visual(
        Box((0.340, 0.100, 0.100)),
        origin=Origin(xyz=(0.0, -0.275, 0.170)),
        material=painted_steel,
        name="rear_cross_bridge",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.620, 0.760, 0.060)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.380, 0.010)),
    )

    lid_shell_mesh = _save_mesh(
        "wheelie_bin_lid_shell",
        LoftGeometry(
            [
                _rounded_loop(0.615, 0.755, 0.070, z=0.008, y_shift=0.3775),
                _rounded_loop(0.607, 0.748, 0.068, z=0.026, y_shift=0.3740),
                _rounded_loop(0.597, 0.738, 0.064, z=0.042, y_shift=0.3690),
            ],
            cap=True,
            closed=True,
        ),
    )
    lid.visual(lid_shell_mesh, material=lid_polymer, name="lid_shell")

    lid_skirt_mesh = _save_mesh(
        "wheelie_bin_lid_skirt",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.602, 0.724, 0.068, corner_segments=8),
            [rounded_rect_profile(0.584, 0.706, 0.056, corner_segments=8)],
            height=0.014,
            center=True,
        ),
    )
    lid.visual(
        lid_skirt_mesh,
        origin=Origin(xyz=(0.0, 0.388, -0.001)),
        material=lid_polymer,
        name="lid_skirt",
    )
    lid.visual(
        Box((0.500, 0.032, 0.014)),
        origin=Origin(xyz=(0.0, 0.022, 0.007)),
        material=lid_polymer,
        name="rear_hinge_bridge",
    )
    lid.visual(
        Box((0.320, 0.026, 0.015)),
        origin=Origin(xyz=(0.0, 0.720, -0.0005)),
        material=lid_polymer,
        name="lid_rest_bar",
    )
    lid.visual(
        Box((0.240, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.742, 0.010)),
        material=elastomer,
        name="pull_grip",
    )

    for name, x, span_x in (
        ("left_hinge_knuckle", -0.182, 0.094),
        ("center_hinge_knuckle", 0.000, 0.126),
        ("right_hinge_knuckle", 0.182, 0.094),
    ):
        lid.visual(
            Box((span_x, 0.020, 0.018)),
            origin=Origin(xyz=(x, 0.006, 0.004)),
            material=lid_polymer,
            name=name,
        )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.046),
        mass=1.4,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        left_wheel,
        mesh_prefix="left_wheel",
        side_sign=1.0,
        tire_radius=0.105,
        tire_width=0.046,
        hub_trim_radius=0.060,
        rubber=elastomer,
        hub_polymer=hub_polymer,
        steel=axle_steel,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.046),
        mass=1.4,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        right_wheel,
        mesh_prefix="right_wheel",
        side_sign=-1.0,
        tire_radius=0.105,
        tire_width=0.046,
        hub_trim_radius=0.060,
        rubber=elastomer,
        hub_polymer=hub_polymer,
        steel=axle_steel,
    )

    lid_hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.351, 0.983)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(0.304, -0.315, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(-0.304, -0.315, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )

    lid.meta["primary_joint"] = lid_hinge.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")
    left_spin = object_model.get_articulation("body_to_left_wheel")
    right_spin = object_model.get_articulation("body_to_right_wheel")

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

    ctx.check(
        "lid_hinge_axis_is_rear_width_axis",
        tuple(round(value, 6) for value in lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"lid hinge axis was {lid_hinge.axis}",
    )
    ctx.check(
        "wheel_spin_axes_are_axial",
        tuple(round(value, 6) for value in left_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(round(value, 6) for value in right_spin.axis) == (1.0, 0.0, 0.0),
        f"left axis={left_spin.axis}, right axis={right_spin.axis}",
    )
    ctx.expect_contact(
        left_wheel,
        body,
        elem_b="left_axle_stub",
        name="left_wheel_supported_by_axle_stub",
    )
    ctx.expect_contact(
        right_wheel,
        body,
        elem_b="right_axle_stub",
        name="right_wheel_supported_by_axle_stub",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="lid_rest_bar",
            elem_b="rim",
            name="lid_rests_on_rim_when_closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.42,
            name="lid_covers_opening_footprint",
        )

    with ctx.pose({lid_hinge: 1.20}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="pull_grip",
            min_gap=0.14,
            name="opened_lid_front_edge_clears_body",
        )

    with ctx.pose({left_spin: 1.1}):
        ctx.expect_contact(
            left_wheel,
            body,
            elem_b="left_axle_stub",
            name="left_wheel_stays_supported_while_rolling",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
