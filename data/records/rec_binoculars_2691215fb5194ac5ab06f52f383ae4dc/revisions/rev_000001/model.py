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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _x_cylinder(
    *,
    radius: float,
    length: float,
    center: tuple[float, float, float],
):
    return Cylinder(radius=radius, length=length), Origin(
        xyz=center,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def _rounded_section(
    *,
    x: float,
    center_y: float,
    center_z: float,
    width_y: float,
    height_z: float,
    radius: float,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width_y,
        height_z,
        radius,
        corner_segments=corner_segments,
    )
    return [(x, center_y + py, center_z + pz) for py, pz in profile]


def _build_barrel_body_mesh(side_sign: float):
    sections = [
        _rounded_section(
            x=-0.084,
            center_y=side_sign * 0.034,
            center_z=-0.018,
            width_y=0.042,
            height_z=0.046,
            radius=0.011,
        ),
        _rounded_section(
            x=-0.050,
            center_y=side_sign * 0.046,
            center_z=-0.014,
            width_y=0.052,
            height_z=0.056,
            radius=0.013,
        ),
        _rounded_section(
            x=-0.006,
            center_y=side_sign * 0.060,
            center_z=-0.006,
            width_y=0.062,
            height_z=0.072,
            radius=0.017,
        ),
        _rounded_section(
            x=0.040,
            center_y=side_sign * 0.070,
            center_z=-0.022,
            width_y=0.066,
            height_z=0.060,
            radius=0.016,
        ),
        _rounded_section(
            x=0.070,
            center_y=side_sign * 0.074,
            center_z=-0.030,
            width_y=0.060,
            height_z=0.052,
            radius=0.014,
        ),
    ]
    return section_loft(sections)


def _add_barrel_visuals(
    model: ArticulatedObject,
    part,
    *,
    side_sign: float,
    body_mesh_name: str,
    armor_material,
    trim_material,
    glass_material,
    right_side_features: bool,
) -> None:
    body_mesh = mesh_from_geometry(_build_barrel_body_mesh(side_sign), body_mesh_name)
    part.visual(body_mesh, material=armor_material, name="armor_shell")

    objective_geom, objective_origin = _x_cylinder(
        radius=0.033,
        length=0.112,
        center=(0.084, side_sign * 0.074, -0.034),
    )
    part.visual(
        objective_geom,
        origin=objective_origin,
        material=armor_material,
        name="objective_barrel",
    )

    objective_ring_geom, objective_ring_origin = _x_cylinder(
        radius=0.035,
        length=0.014,
        center=(0.137, side_sign * 0.074, -0.034),
    )
    part.visual(
        objective_ring_geom,
        origin=objective_ring_origin,
        material=trim_material,
        name="objective_ring",
    )

    objective_glass_geom, objective_glass_origin = _x_cylinder(
        radius=0.029,
        length=0.003,
        center=(0.141, side_sign * 0.074, -0.034),
    )
    part.visual(
        objective_glass_geom,
        origin=objective_glass_origin,
        material=glass_material,
        name="objective_window",
    )

    eyepiece_geom, eyepiece_origin = _x_cylinder(
        radius=0.020,
        length=0.050,
        center=(-0.094, side_sign * 0.034, -0.018),
    )
    part.visual(
        eyepiece_geom,
        origin=eyepiece_origin,
        material=trim_material,
        name="eyepiece_barrel",
    )

    eyecup_geom, eyecup_origin = _x_cylinder(
        radius=0.024,
        length=0.032,
        center=(-0.132, side_sign * 0.034, -0.018),
    )
    part.visual(
        eyecup_geom,
        origin=eyecup_origin,
        material=armor_material,
        name="eyecup",
    )

    eyepiece_glass_geom, eyepiece_glass_origin = _x_cylinder(
        radius=0.017,
        length=0.003,
        center=(-0.145, side_sign * 0.034, -0.018),
    )
    part.visual(
        eyepiece_glass_geom,
        origin=eyepiece_glass_origin,
        material=glass_material,
        name="eyepiece_window",
    )

    part.visual(
        Box((0.104, 0.010, 0.018)),
        origin=Origin(xyz=(0.000, side_sign * 0.015, -0.003)),
        material=armor_material,
        name="hinge_bridge",
    )
    part.visual(
        Box((0.082, 0.022, 0.024)),
        origin=Origin(xyz=(-0.004, side_sign * 0.026, -0.014)),
        material=armor_material,
        name="bridge_drop",
    )
    part.visual(
        Box((0.068, 0.018, 0.020)),
        origin=Origin(xyz=(-0.014, side_sign * 0.041, -0.020)),
        material=armor_material,
        name="inner_brace",
    )

    if right_side_features:
        for index, center_x in enumerate((-0.032, 0.032)):
            hinge_geom, hinge_origin = _x_cylinder(
                radius=0.0105,
                length=0.024,
                center=(center_x, 0.000, 0.000),
            )
            part.visual(
                hinge_geom,
                origin=hinge_origin,
                material=trim_material,
                name=f"hinge_knuckle_{index}",
            )

        part.visual(
            Box((0.044, 0.020, 0.010)),
            origin=Origin(xyz=(-0.040, 0.014, 0.009)),
            material=armor_material,
            name="focus_support_web",
        )

        for index, center_x in enumerate((-0.055, -0.025)):
            support_geom, support_origin = _x_cylinder(
                radius=0.009,
                length=0.010,
                center=(center_x, 0.000, 0.030),
            )
            part.visual(
                support_geom,
                origin=support_origin,
                material=trim_material,
                name=f"focus_support_knuckle_{index}",
            )
            part.visual(
                Box((0.006, 0.010, 0.014)),
                origin=Origin(xyz=(center_x, 0.004, 0.021)),
                material=trim_material,
                name=f"focus_support_post_{index}",
            )

        part.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(-0.012, 0.058, 0.026)),
            material=trim_material,
            name="compass_socket",
        )
    else:
        hinge_geom, hinge_origin = _x_cylinder(
            radius=0.0105,
            length=0.040,
            center=(0.000, 0.000, 0.000),
        )
        part.visual(
            hinge_geom,
            origin=hinge_origin,
            material=trim_material,
            name="hinge_knuckle_center",
        )

    part.inertial = Inertial.from_geometry(
        Box((0.290, 0.130, 0.090)),
        mass=0.78,
        origin=Origin(xyz=(0.000, side_sign * 0.060, -0.020)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_porro_prism_binocular")

    armor_material = model.material("armor_black", rgba=(0.14, 0.15, 0.16, 1.0))
    trim_material = model.material("trim_graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    glass_material = model.material("glass_green", rgba=(0.36, 0.47, 0.44, 0.45))
    metal_material = model.material("metal_dark", rgba=(0.30, 0.31, 0.34, 1.0))
    compass_dial_material = model.material("compass_dial", rgba=(0.84, 0.84, 0.76, 1.0))
    compass_needle_material = model.material("compass_needle", rgba=(0.70, 0.16, 0.14, 1.0))

    right_barrel = model.part("right_barrel")
    _add_barrel_visuals(
        model,
        right_barrel,
        side_sign=1.0,
        body_mesh_name="marine_binocular_right_body",
        armor_material=armor_material,
        trim_material=trim_material,
        glass_material=glass_material,
        right_side_features=True,
    )

    left_barrel = model.part("left_barrel")
    _add_barrel_visuals(
        model,
        left_barrel,
        side_sign=-1.0,
        body_mesh_name="marine_binocular_left_body",
        armor_material=armor_material,
        trim_material=trim_material,
        glass_material=glass_material,
        right_side_features=False,
    )

    model.articulation(
        "center_hinge",
        ArticulationType.REVOLUTE,
        parent=right_barrel,
        child=left_barrel,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.0,
            lower=-0.22,
            upper=0.22,
        ),
    )

    focus_wheel = model.part("focus_wheel")
    focus_hub_geom, focus_hub_origin = _x_cylinder(
        radius=0.009,
        length=0.020,
        center=(0.000, 0.000, 0.000),
    )
    focus_wheel.visual(
        focus_hub_geom,
        origin=focus_hub_origin,
        material=metal_material,
        name="focus_hub",
    )
    focus_tire_geom, focus_tire_origin = _x_cylinder(
        radius=0.016,
        length=0.016,
        center=(0.000, 0.000, 0.000),
    )
    focus_wheel.visual(
        focus_tire_geom,
        origin=focus_tire_origin,
        material=armor_material,
        name="focus_tire",
    )
    for index, center_x in enumerate((-0.008, 0.008)):
        ridge_geom, ridge_origin = _x_cylinder(
            radius=0.017,
            length=0.002,
            center=(center_x, 0.000, 0.000),
        )
        focus_wheel.visual(
            ridge_geom,
            origin=ridge_origin,
            material=trim_material,
            name=f"focus_ridge_{index}",
        )
    focus_wheel.inertial = Inertial.from_geometry(
        Box((0.022, 0.034, 0.034)),
        mass=0.08,
        origin=Origin(),
    )
    model.articulation(
        "focus_wheel_spin",
        ArticulationType.REVOLUTE,
        parent=right_barrel,
        child=focus_wheel,
        origin=Origin(xyz=(-0.040, 0.000, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=10.0,
            lower=-6.0,
            upper=6.0,
        ),
    )

    compass_housing = model.part("compass_housing")
    compass_housing.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=trim_material,
        name="compass_base",
    )
    compass_housing.visual(
        Cylinder(radius=0.013, length=0.002),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=compass_dial_material,
        name="compass_dial",
    )
    compass_housing.visual(
        Box((0.018, 0.0022, 0.0012)),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=compass_needle_material,
        name="compass_needle",
    )
    compass_housing.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=glass_material,
        name="compass_dome",
    )
    compass_housing.inertial = Inertial.from_geometry(
        Box((0.036, 0.036, 0.034)),
        mass=0.05,
        origin=Origin(xyz=(0.000, 0.000, 0.017)),
    )
    model.articulation(
        "compass_rotation",
        ArticulationType.REVOLUTE,
        parent=right_barrel,
        child=compass_housing,
        origin=Origin(xyz=(-0.012, 0.058, 0.029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.5,
            lower=-0.9,
            upper=0.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    right_barrel = object_model.get_part("right_barrel")
    left_barrel = object_model.get_part("left_barrel")
    focus_wheel = object_model.get_part("focus_wheel")
    compass_housing = object_model.get_part("compass_housing")
    center_hinge = object_model.get_articulation("center_hinge")
    focus_joint = object_model.get_articulation("focus_wheel_spin")
    compass_joint = object_model.get_articulation("compass_rotation")

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
        "hinge_axis_is_longitudinal",
        tuple(center_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected hinge axis (1, 0, 0), got {center_hinge.axis}",
    )
    ctx.check(
        "focus_wheel_axis_is_longitudinal",
        tuple(focus_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected focus axis (1, 0, 0), got {focus_joint.axis}",
    )
    ctx.check(
        "compass_axis_is_vertical",
        tuple(compass_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected compass axis (0, 0, 1), got {compass_joint.axis}",
    )

    ctx.expect_contact(
        left_barrel,
        right_barrel,
        name="hinge_halves_touch_at_rest",
    )
    ctx.expect_contact(
        focus_wheel,
        right_barrel,
        name="focus_wheel_seated_in_bridge",
    )
    ctx.expect_contact(
        compass_housing,
        right_barrel,
        name="compass_housing_seated_in_socket",
    )
    ctx.expect_gap(
        right_barrel,
        left_barrel,
        axis="y",
        positive_elem="eyecup",
        negative_elem="eyecup",
        min_gap=0.015,
        name="eyecups_clear_each_other_at_rest",
    )

    with ctx.pose({center_hinge: 0.18}):
        ctx.expect_gap(
            right_barrel,
            left_barrel,
            axis="y",
            positive_elem="eyecup",
            negative_elem="eyecup",
            min_gap=0.008,
            name="eyecups_stay_clear_when_hinge_closes",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
