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
    section_loft,
)


def _yz_section(
    x: float,
    width: float,
    height: float,
    *,
    z_center: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _prop_blade_mesh(
    *,
    name: str,
    span: float,
    root_width: float,
    tip_width: float,
    thickness: float,
):
    planform = [
        (0.0, -root_width * 0.46),
        (span * 0.10, -root_width * 0.50),
        (span * 0.62, -tip_width * 0.72),
        (span, -tip_width * 0.24),
        (span, tip_width * 0.24),
        (span * 0.62, tip_width * 0.72),
        (span * 0.10, root_width * 0.50),
        (0.0, root_width * 0.46),
    ]
    return mesh_from_geometry(ExtrudeGeometry(planform, thickness, center=True), name)


def _add_lift_propeller(
    part,
    *,
    blade_mesh,
    blade_material,
    hub_material,
    hub_radius: float,
    hub_height: float,
    blade_offset: float,
) -> None:
    part.visual(
        Cylinder(radius=hub_radius, length=hub_height),
        origin=Origin(xyz=(0.0, 0.0, hub_height * 0.5)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.55, length=hub_height * 0.35),
        origin=Origin(xyz=(0.0, 0.0, hub_height + hub_height * 0.175)),
        material=hub_material,
        name="cap",
    )
    part.visual(
        blade_mesh,
        origin=Origin(xyz=(hub_radius * 0.92, 0.0, blade_offset)),
        material=blade_material,
        name="blade_a",
    )
    part.visual(
        blade_mesh,
        origin=Origin(xyz=(hub_radius * 0.92, 0.0, blade_offset), rpy=(0.0, 0.0, math.pi)),
        material=blade_material,
        name="blade_b",
    )


def _add_pusher_propeller(
    part,
    *,
    blade_mesh,
    blade_material,
    hub_material,
    hub_radius: float,
    hub_length: float,
    blade_x: float,
) -> None:
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(xyz=(hub_length * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.55, length=hub_length * 0.35),
        origin=Origin(
            xyz=(hub_length + hub_length * 0.175, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hub_material,
        name="cap",
    )
    blade_rpy = (-math.pi / 2.0, 0.0, math.pi / 2.0)
    part.visual(
        blade_mesh,
        origin=Origin(xyz=(blade_x, 0.0, 0.0), rpy=blade_rpy),
        material=blade_material,
        name="blade_a",
    )
    part.visual(
        blade_mesh,
        origin=Origin(xyz=(blade_x, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, -math.pi / 2.0)),
        material=blade_material,
        name="blade_b",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixed_wing_vtol_hybrid_drone")

    wing_gray = model.material("wing_gray", rgba=(0.45, 0.49, 0.53, 1.0))
    dark_carbon = model.material("dark_carbon", rgba=(0.09, 0.10, 0.11, 1.0))
    matte_black = model.material("matte_black", rgba=(0.13, 0.14, 0.15, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.87, 0.42, 0.15, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.20, 0.27, 0.32, 0.55))

    wing_span = 1.65
    wing_chord = 0.62
    wing_thickness = 0.055
    wing_half_thickness = wing_thickness * 0.5

    lift_hub_radius = 0.028
    lift_hub_height = 0.022
    lift_motor_radius = 0.040
    lift_motor_height = 0.055
    lift_blade_thickness = 0.008
    lift_blade_span = 0.152
    lift_blade_root = 0.040
    lift_blade_tip = 0.018
    lift_blade_z = lift_hub_height + lift_blade_thickness * 0.5 + 0.002

    pusher_hub_radius = 0.030
    pusher_hub_length = 0.026
    pusher_blade_span = 0.140
    pusher_blade_root = 0.038
    pusher_blade_tip = 0.018
    pusher_blade_thickness = 0.008
    pusher_blade_x = pusher_hub_length * 0.55 + pusher_blade_thickness * 0.5

    lift_blade_mesh = _prop_blade_mesh(
        name="lift_prop_blade",
        span=lift_blade_span,
        root_width=lift_blade_root,
        tip_width=lift_blade_tip,
        thickness=lift_blade_thickness,
    )
    pusher_blade_mesh = _prop_blade_mesh(
        name="pusher_prop_blade",
        span=pusher_blade_span,
        root_width=pusher_blade_root,
        tip_width=pusher_blade_tip,
        thickness=pusher_blade_thickness,
    )

    airframe = model.part("airframe")

    wing_geom = ExtrudeGeometry(
        rounded_rect_profile(wing_chord, wing_thickness, 0.018, corner_segments=10),
        wing_span,
    ).rotate_x(math.pi / 2.0)
    airframe.visual(
        mesh_from_geometry(wing_geom, "wing_shell"),
        material=wing_gray,
        name="wing_shell",
    )

    fuselage_geom = section_loft(
        [
            _yz_section(-0.33, 0.055, 0.060, z_center=0.042, radius=0.018),
            _yz_section(-0.18, 0.115, 0.100, z_center=0.052, radius=0.026),
            _yz_section(0.02, 0.170, 0.125, z_center=0.060, radius=0.032),
            _yz_section(0.20, 0.110, 0.090, z_center=0.050, radius=0.022),
        ]
    )
    airframe.visual(
        mesh_from_geometry(fuselage_geom, "center_fuselage"),
        material=wing_gray,
        name="center_fuselage",
    )
    airframe.visual(
        Box((0.06, 0.08, 0.030)),
        origin=Origin(xyz=(-0.31, 0.0, 0.025)),
        material=sensor_glass,
        name="nose_sensor",
    )
    airframe.visual(
        Box((0.39, 0.095, 0.078)),
        origin=Origin(xyz=(0.482, 0.0, 0.040)),
        material=wing_gray,
        name="tail_boom",
    )
    airframe.visual(
        Box((0.090, 0.060, 0.185)),
        origin=Origin(xyz=(0.70, 0.0, 0.118)),
        material=wing_gray,
        name="tail_pylon",
    )
    airframe.visual(
        Cylinder(radius=0.034, length=0.090),
        origin=Origin(xyz=(0.785, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="pusher_motor",
    )

    lift_positions = {
        "front_left": (-0.23, 0.68),
        "front_right": (-0.23, -0.68),
        "rear_left": (0.23, 0.68),
        "rear_right": (0.23, -0.68),
    }
    motor_top_z = wing_half_thickness + lift_motor_height
    for corner_name, (x_pos, y_pos) in lift_positions.items():
        airframe.visual(
            Box((0.072, 0.072, 0.040)),
            origin=Origin(xyz=(x_pos, y_pos, wing_half_thickness + 0.010)),
            material=wing_gray,
            name=f"lift_mount_{corner_name}",
        )
        airframe.visual(
            Cylinder(radius=lift_motor_radius, length=lift_motor_height),
            origin=Origin(xyz=(x_pos, y_pos, wing_half_thickness + lift_motor_height * 0.5)),
            material=matte_black,
            name=f"lift_motor_{corner_name}",
        )
        airframe.visual(
            Cylinder(radius=lift_motor_radius * 0.52, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, motor_top_z - 0.005)),
            material=accent_orange,
            name=f"lift_cap_{corner_name}",
        )

    airframe.inertial = Inertial.from_geometry(
        Box((1.00, 1.65, 0.26)),
        mass=5.2,
        origin=Origin(xyz=(0.14, 0.0, 0.08)),
    )

    lift_prop_front_left = model.part("lift_prop_front_left")
    _add_lift_propeller(
        lift_prop_front_left,
        blade_mesh=lift_blade_mesh,
        blade_material=dark_carbon,
        hub_material=matte_black,
        hub_radius=lift_hub_radius,
        hub_height=lift_hub_height,
        blade_offset=lift_blade_z,
    )
    lift_prop_front_left.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.035),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    lift_prop_front_right = model.part("lift_prop_front_right")
    _add_lift_propeller(
        lift_prop_front_right,
        blade_mesh=lift_blade_mesh,
        blade_material=dark_carbon,
        hub_material=matte_black,
        hub_radius=lift_hub_radius,
        hub_height=lift_hub_height,
        blade_offset=lift_blade_z,
    )
    lift_prop_front_right.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.035),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    lift_prop_rear_left = model.part("lift_prop_rear_left")
    _add_lift_propeller(
        lift_prop_rear_left,
        blade_mesh=lift_blade_mesh,
        blade_material=dark_carbon,
        hub_material=matte_black,
        hub_radius=lift_hub_radius,
        hub_height=lift_hub_height,
        blade_offset=lift_blade_z,
    )
    lift_prop_rear_left.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.035),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    lift_prop_rear_right = model.part("lift_prop_rear_right")
    _add_lift_propeller(
        lift_prop_rear_right,
        blade_mesh=lift_blade_mesh,
        blade_material=dark_carbon,
        hub_material=matte_black,
        hub_radius=lift_hub_radius,
        hub_height=lift_hub_height,
        blade_offset=lift_blade_z,
    )
    lift_prop_rear_right.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.035),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    pusher_prop = model.part("pusher_prop")
    _add_pusher_propeller(
        pusher_prop,
        blade_mesh=pusher_blade_mesh,
        blade_material=dark_carbon,
        hub_material=matte_black,
        hub_radius=pusher_hub_radius,
        hub_length=pusher_hub_length,
        blade_x=pusher_blade_x,
    )
    pusher_prop.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=0.035),
        mass=0.12,
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    for prop_name, corner_name in (
        ("lift_prop_front_left", "front_left"),
        ("lift_prop_front_right", "front_right"),
        ("lift_prop_rear_left", "rear_left"),
        ("lift_prop_rear_right", "rear_right"),
    ):
        x_pos, y_pos = lift_positions[corner_name]
        model.articulation(
            f"airframe_to_{prop_name}",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=prop_name,
            origin=Origin(xyz=(x_pos, y_pos, motor_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=120.0),
        )

    model.articulation(
        "airframe_to_pusher_prop",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=pusher_prop,
        origin=Origin(xyz=(0.83, 0.0, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=140.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    lift_prop_front_left = object_model.get_part("lift_prop_front_left")
    lift_prop_front_right = object_model.get_part("lift_prop_front_right")
    lift_prop_rear_left = object_model.get_part("lift_prop_rear_left")
    lift_prop_rear_right = object_model.get_part("lift_prop_rear_right")
    pusher_prop = object_model.get_part("pusher_prop")

    front_left_joint = object_model.get_articulation("airframe_to_lift_prop_front_left")
    front_right_joint = object_model.get_articulation("airframe_to_lift_prop_front_right")
    rear_left_joint = object_model.get_articulation("airframe_to_lift_prop_rear_left")
    rear_right_joint = object_model.get_articulation("airframe_to_lift_prop_rear_right")
    pusher_joint = object_model.get_articulation("airframe_to_pusher_prop")

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

    lift_joints = [front_left_joint, front_right_joint, rear_left_joint, rear_right_joint]
    ctx.check(
        "lift_rotors_use_vertical_continuous_axes",
        all(joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, 0.0, 1.0) for joint in lift_joints),
        details="All four corner lift rotors should spin continuously about +Z.",
    )
    ctx.check(
        "pusher_rotor_uses_forward_continuous_axis",
        pusher_joint.articulation_type == ArticulationType.CONTINUOUS and pusher_joint.axis == (1.0, 0.0, 0.0),
        details="The tail pusher rotor should spin continuously about the forward X axis.",
    )

    for prop_name, prop, motor_name in (
        ("front_left_mount", lift_prop_front_left, "lift_motor_front_left"),
        ("front_right_mount", lift_prop_front_right, "lift_motor_front_right"),
        ("rear_left_mount", lift_prop_rear_left, "lift_motor_rear_left"),
        ("rear_right_mount", lift_prop_rear_right, "lift_motor_rear_right"),
    ):
        ctx.expect_contact(
            prop,
            airframe,
            elem_a="hub",
            elem_b=motor_name,
            name=f"{prop_name}_hub_contacts_motor",
        )
        ctx.expect_overlap(
            prop,
            airframe,
            axes="xy",
            min_overlap=0.045,
            elem_a="hub",
            elem_b=motor_name,
            name=f"{prop_name}_hub_centered_on_motor",
        )
        ctx.expect_gap(
            prop,
            airframe,
            axis="z",
            min_gap=0.045,
            positive_elem="blade_a",
            negative_elem="wing_shell",
            name=f"{prop_name}_blade_clearance_above_wing",
        )

    ctx.expect_contact(
        pusher_prop,
        airframe,
        elem_a="hub",
        elem_b="pusher_motor",
        name="pusher_hub_contacts_tail_motor",
    )
    ctx.expect_overlap(
        pusher_prop,
        airframe,
        axes="yz",
        min_overlap=0.045,
        elem_a="hub",
        elem_b="pusher_motor",
        name="pusher_hub_centered_on_tail_motor",
    )
    ctx.expect_gap(
        pusher_prop,
        airframe,
        axis="x",
        min_gap=0.012,
        positive_elem="blade_a",
        negative_elem="pusher_motor",
        name="pusher_blade_aft_of_tail_motor",
    )

    ctx.expect_origin_distance(
        lift_prop_front_left,
        lift_prop_front_right,
        axes="y",
        min_dist=1.20,
        name="front_lift_rotors_span_wing_width",
    )
    ctx.expect_origin_distance(
        lift_prop_front_left,
        lift_prop_rear_left,
        axes="x",
        min_dist=0.40,
        name="left_lift_rotors_staggered_fore_aft",
    )
    ctx.expect_origin_gap(
        pusher_prop,
        airframe,
        axis="x",
        min_gap=0.75,
        name="pusher_rotor_mounted_at_tail_end",
    )

    with ctx.pose(
        {
            front_left_joint: math.pi / 2.0,
            front_right_joint: math.pi / 2.0,
            rear_left_joint: math.pi / 2.0,
            rear_right_joint: math.pi / 2.0,
            pusher_joint: math.pi / 2.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotor_clearance_after_quarter_turn")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
