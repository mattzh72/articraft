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
)


def _h_frame_profile(
    *,
    outer_width: float,
    outer_length: float,
    rail_width: float,
    bridge_length: float,
) -> list[tuple[float, float]]:
    half_w = outer_width * 0.5
    half_l = outer_length * 0.5
    inner_x = half_w - rail_width
    bridge_y = bridge_length * 0.5
    return [
        (-half_w, -half_l),
        (-inner_x, -half_l),
        (-inner_x, -bridge_y),
        (inner_x, -bridge_y),
        (inner_x, -half_l),
        (half_w, -half_l),
        (half_w, half_l),
        (inner_x, half_l),
        (inner_x, bridge_y),
        (-inner_x, bridge_y),
        (-inner_x, half_l),
        (-half_w, half_l),
    ]


def _prop_blade_profile() -> list[tuple[float, float]]:
    return [
        (0.004, 0.0028),
        (0.010, 0.0052),
        (0.024, 0.0074),
        (0.040, 0.0068),
        (0.055, 0.0043),
        (0.061, 0.0012),
        (0.061, -0.0012),
        (0.055, -0.0043),
        (0.040, -0.0060),
        (0.024, -0.0068),
        (0.010, -0.0048),
        (0.004, -0.0022),
    ]


def _add_single_blade_propeller(
    part,
    *,
    prop_mesh,
    prop_material,
    nut_material,
    blade_yaw: float,
) -> None:
    c = math.cos(blade_yaw)
    s = math.sin(blade_yaw)
    part.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=prop_material,
        name="hub",
    )
    part.visual(
        prop_mesh,
        origin=Origin(rpy=(0.0, 0.0, blade_yaw)),
        material=prop_material,
        name="blade",
    )
    part.visual(
        Box((0.010, 0.004, 0.003)),
        origin=Origin(
            xyz=(-0.012 * c, -0.012 * s, 0.0015),
            rpy=(0.0, 0.0, blade_yaw),
        ),
        material=nut_material,
        name="counterweight",
    )
    part.visual(
        Cylinder(radius=0.0035, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=nut_material,
        name="locknut",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0635, length=0.007),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_h_frame_quadrotor")

    carbon = model.material("carbon", rgba=(0.11, 0.12, 0.13, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.79, 1.0))
    motor_black = model.material("motor_black", rgba=(0.08, 0.08, 0.09, 1.0))
    prop_black = model.material("prop_black", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    camera_glass = model.material("camera_glass", rgba=(0.18, 0.25, 0.30, 1.0))

    bottom_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            _h_frame_profile(
                outer_width=0.090,
                outer_length=0.122,
                rail_width=0.022,
                bridge_length=0.026,
            ),
            0.003,
        ),
        "bottom_h_plate",
    )
    top_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            _h_frame_profile(
                outer_width=0.076,
                outer_length=0.088,
                rail_width=0.019,
                bridge_length=0.022,
            ),
            0.0025,
        ),
        "top_h_plate",
    )
    prop_blade_mesh = mesh_from_geometry(
        ExtrudeGeometry(_prop_blade_profile(), 0.0024).translate(0.0, 0.0, 0.0012),
        "single_blade_prop",
    )

    body = model.part("body")
    body.visual(
        bottom_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=carbon,
        name="bottom_plate",
    )
    body.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.01625)),
        material=carbon,
        name="top_plate",
    )

    arm_specs = {
        "front_left": (-0.034, 0.081),
        "front_right": (0.034, 0.081),
        "rear_left": (-0.034, -0.081),
        "rear_right": (0.034, -0.081),
    }
    for name, (x_pos, y_pos) in arm_specs.items():
        body.visual(
            Box((0.016, 0.050, 0.006)),
            origin=Origin(xyz=(x_pos, y_pos, 0.003)),
            material=carbon,
            name=f"arm_{name}",
        )

    motor_specs = {
        "front_left": (-0.034, 0.101, 0.013),
        "front_right": (0.034, 0.101, 0.013),
        "rear_left": (-0.034, -0.101, 0.013),
        "rear_right": (0.034, -0.101, 0.013),
    }
    for name, (x_pos, y_pos, z_pos) in motor_specs.items():
        body.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            material=motor_black,
            name=f"motor_{name}",
        )

    for x_pos in (-0.027, 0.027):
        for y_pos in (-0.021, 0.021):
            body.visual(
                Cylinder(radius=0.003, length=0.014),
                origin=Origin(xyz=(x_pos, y_pos, 0.010)),
                material=aluminum,
                name=f"standoff_{'left' if x_pos < 0.0 else 'right'}_{'rear' if y_pos < 0.0 else 'front'}",
            )

    body.visual(
        Box((0.034, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=motor_black,
        name="flight_stack",
    )
    body.visual(
        Box((0.038, 0.052, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=rubber,
        name="battery_pad",
    )
    body.visual(
        Box((0.026, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.024, 0.011)),
        material=motor_black,
        name="camera_pod",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.036, 0.011), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=camera_glass,
        name="camera_lens",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.100, 0.240, 0.030)),
        mass=0.68,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    front_left_prop = model.part("front_left_prop")
    front_right_prop = model.part("front_right_prop")
    rear_left_prop = model.part("rear_left_prop")
    rear_right_prop = model.part("rear_right_prop")

    _add_single_blade_propeller(
        front_left_prop,
        prop_mesh=prop_blade_mesh,
        prop_material=prop_black,
        nut_material=aluminum,
        blade_yaw=3.0 * math.pi / 4.0,
    )
    _add_single_blade_propeller(
        front_right_prop,
        prop_mesh=prop_blade_mesh,
        prop_material=prop_black,
        nut_material=aluminum,
        blade_yaw=math.pi / 4.0,
    )
    _add_single_blade_propeller(
        rear_left_prop,
        prop_mesh=prop_blade_mesh,
        prop_material=prop_black,
        nut_material=aluminum,
        blade_yaw=-3.0 * math.pi / 4.0,
    )
    _add_single_blade_propeller(
        rear_right_prop,
        prop_mesh=prop_blade_mesh,
        prop_material=prop_black,
        nut_material=aluminum,
        blade_yaw=-math.pi / 4.0,
    )

    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_left_prop,
        origin=Origin(xyz=(-0.034, 0.101, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=180.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_right_prop,
        origin=Origin(xyz=(0.034, 0.101, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=180.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_left_prop,
        origin=Origin(xyz=(-0.034, -0.101, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=180.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_right_prop,
        origin=Origin(xyz=(0.034, -0.101, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=180.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_left_prop = object_model.get_part("front_left_prop")
    front_right_prop = object_model.get_part("front_right_prop")
    rear_left_prop = object_model.get_part("rear_left_prop")
    rear_right_prop = object_model.get_part("rear_right_prop")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")

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

    prop_checks = [
        (front_left_prop, front_left_spin, "motor_front_left", "front left"),
        (front_right_prop, front_right_spin, "motor_front_right", "front right"),
        (rear_left_prop, rear_left_spin, "motor_rear_left", "rear left"),
        (rear_right_prop, rear_right_spin, "motor_rear_right", "rear right"),
    ]
    for prop, joint, motor_name, label in prop_checks:
        limits = joint.motion_limits
        ctx.check(
            f"{label} rotor uses a vertical continuous axle",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_contact(
            prop,
            body,
            elem_a="hub",
            elem_b=motor_name,
            name=f"{label} prop hub is seated on its motor",
        )
        ctx.expect_overlap(
            prop,
            body,
            axes="xy",
            min_overlap=0.014,
            elem_a="hub",
            elem_b=motor_name,
            name=f"{label} prop hub stays centered over its motor",
        )
        ctx.expect_gap(
            prop,
            body,
            axis="z",
            min_gap=0.002,
            positive_elem="blade",
            negative_elem="top_plate",
            name=f"{label} blade clears the top deck",
        )

    ctx.expect_origin_distance(
        front_left_prop,
        front_right_prop,
        axes="x",
        min_dist=0.06,
        max_dist=0.08,
        name="left and right front rotors have realistic spacing",
    )
    ctx.expect_origin_distance(
        front_left_prop,
        rear_left_prop,
        axes="y",
        min_dist=0.19,
        max_dist=0.22,
        name="front and rear rotors have realistic H-frame spacing",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
