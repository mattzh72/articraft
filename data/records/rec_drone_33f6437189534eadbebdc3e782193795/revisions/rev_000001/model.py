from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(profile, dx: float, dy: float, yaw: float = 0.0):
    ca = math.cos(yaw)
    sa = math.sin(yaw)
    return [(dx + x * ca - y * sa, dy + x * sa + y * ca) for x, y in profile]


def _add_body_hinge_hardware(body, *, name: str, x: float, y: float, yaw: float, mat_plastic, mat_metal):
    """Fixed half of a vertical knuckle hinge at one body corner."""
    hinge_z = 0.008
    ux = math.cos(yaw)
    uy = math.sin(yaw)

    # A plastic leaf pad ties the hinge barrels visibly back into the carbon plate.
    body.visual(
        Box((0.028, 0.026, 0.005)),
        origin=Origin(
            xyz=(x - ux * 0.024, y - uy * 0.024, 0.0065),
            rpy=(0.0, 0.0, yaw),
        ),
        material=mat_plastic,
        name=f"{name}_hinge_leaf",
    )
    for suffix, dz in (("lower", -0.006), ("upper", 0.006)):
        body.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(x, y, hinge_z + dz)),
            material=mat_plastic,
            name=f"{name}_hinge_{suffix}",
        )
    body.visual(
        Cylinder(radius=0.003, length=0.022),
        origin=Origin(xyz=(x, y, hinge_z)),
        material=mat_metal,
        name=f"{name}_hinge_pin",
    )


def _add_carbon_weave(part, *, mat_weave):
    """Sparse raised weave strands to make the carbon-fibre composite readable."""
    top_z = 0.0044
    for i, y in enumerate((-0.032, -0.016, 0.0, 0.016, 0.032)):
        part.visual(
            Box((0.120, 0.0012, 0.0008)),
            origin=Origin(xyz=(0.0, y, top_z), rpy=(0.0, 0.0, 0.0)),
            material=mat_weave,
            name=f"weave_long_{i}",
        )
    for i, x in enumerate((-0.045, -0.022, 0.0, 0.022, 0.045)):
        part.visual(
            Box((0.110, 0.0010, 0.0008)),
            origin=Origin(xyz=(x, 0.0, top_z + 0.0002), rpy=(0.0, 0.0, math.radians(90))),
            material=mat_weave,
            name=f"weave_cross_{i}",
        )
    for i, (x, yaw) in enumerate(((-0.026, math.radians(35)), (0.026, -math.radians(35)))):
        part.visual(
            Box((0.105, 0.0014, 0.0008)),
            origin=Origin(xyz=(x, 0.0, top_z + 0.0004), rpy=(0.0, 0.0, yaw)),
            material=mat_weave,
            name=f"weave_bias_{i}",
        )


def _add_arm_geometry(arm, *, mat_carbon, mat_plastic, mat_metal):
    """Moving arm with a knuckle collar, hollow rectangular tube, and motor pod."""
    tube_start = 0.032
    tube_len = 0.145
    tube_w = 0.024
    tube_h = 0.014
    wall = 0.003
    tube_cx = tube_start + tube_len / 2.0
    tube_z = 0.004

    # Moving middle knuckle of the vertical fold hinge.
    arm.visual(
        Cylinder(radius=0.0086, length=0.0055),
        origin=Origin(),
        material=mat_plastic,
        name="root_knuckle",
    )
    arm.visual(
        Box((0.032, 0.021, 0.0055)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=mat_plastic,
        name="root_collar",
    )

    # Four closed wall strips leave the arm visibly open as a rectangular tube.
    arm.visual(
        Box((tube_len, tube_w, wall)),
        origin=Origin(xyz=(tube_cx, 0.0, tube_z + tube_h / 2.0 - wall / 2.0)),
        material=mat_carbon,
        name="tube_top_wall",
    )
    arm.visual(
        Box((tube_len, tube_w, wall)),
        origin=Origin(xyz=(tube_cx, 0.0, tube_z - tube_h / 2.0 + wall / 2.0)),
        material=mat_carbon,
        name="tube_bottom_wall",
    )
    for side, y in (("pos", tube_w / 2.0 - wall / 2.0), ("neg", -tube_w / 2.0 + wall / 2.0)):
        arm.visual(
            Box((tube_len, wall, tube_h)),
            origin=Origin(xyz=(tube_cx, y, tube_z)),
            material=mat_carbon,
            name=f"tube_{side}_wall",
        )

    motor_x = 0.188
    arm.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(motor_x, 0.0, tube_z)),
        material=mat_metal,
        name="motor_pod",
    )
    arm.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(motor_x, 0.0, tube_z + 0.011)),
        material=mat_plastic,
        name="motor_cap",
    )


def _add_camera_pod(camera, *, mat_plastic, mat_camera, mat_glass):
    """Tilt bracket plus fixed cube-form camera body and lens."""
    camera.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mat_plastic,
        name="tilt_barrel",
    )
    for side, y in (("upper", 0.010), ("lower", -0.010)):
        camera.visual(
            Box((0.052, 0.004, 0.006)),
            origin=Origin(xyz=(0.026, y, -0.009)),
            material=mat_plastic,
            name=f"bracket_arm_{side}",
        )
    camera.visual(
        Box((0.034, 0.034, 0.030)),
        origin=Origin(xyz=(0.060, 0.0, -0.014)),
        material=mat_camera,
        name="camera_body",
    )
    camera.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.080, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mat_glass,
        name="lens",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_racing_quadrotor")

    carbon = model.material("carbon_fibre_composite", rgba=(0.015, 0.016, 0.017, 1.0))
    carbon_weave = model.material("raised_grey_carbon_weave", rgba=(0.15, 0.16, 0.16, 1.0))
    matte_plastic = model.material("matte_black_plastic", rgba=(0.02, 0.02, 0.022, 1.0))
    dark_plastic = model.material("dark_hinge_plastic", rgba=(0.055, 0.052, 0.050, 1.0))
    metal = model.material("gunmetal_motor", rgba=(0.26, 0.27, 0.28, 1.0))
    pin_metal = model.material("brushed_steel_pins", rgba=(0.65, 0.66, 0.62, 1.0))
    camera_glass = model.material("blue_coated_lens", rgba=(0.03, 0.10, 0.18, 1.0))

    body = model.part("body")

    outer = rounded_rect_profile(0.170, 0.120, 0.012, corner_segments=8)
    slot_profile = rounded_rect_profile(0.042, 0.018, 0.008, corner_segments=8)
    holes = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            holes.append(
                _shift_profile(
                    slot_profile,
                    sx * 0.048,
                    sy * 0.033,
                    yaw=sy * sx * math.radians(38.0),
                )
            )
    body_plate = ExtrudeWithHolesGeometry(outer, holes, 0.008, center=True)
    body.visual(
        mesh_from_geometry(body_plate, "radiused_x_body_plate"),
        material=carbon,
        name="body_plate",
    )
    _add_carbon_weave(body, mat_weave=carbon_weave)

    # Underside battery recess: a dark lowered floor with lips/rails that are
    # connected to the plate underside, not just painted on.
    body.visual(
        Box((0.082, 0.048, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.0050)),
        material=matte_plastic,
        name="battery_recess_floor",
    )
    for i, y in enumerate((-0.028, 0.028)):
        body.visual(
            Box((0.088, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, y, -0.0060)),
            material=dark_plastic,
            name=f"battery_rail_{i}",
        )
    for i, x in enumerate((-0.046, 0.046)):
        body.visual(
            Box((0.006, 0.052, 0.004)),
            origin=Origin(xyz=(x, 0.0, -0.0060)),
            material=dark_plastic,
            name=f"battery_lip_{i}",
        )

    hinge_z = 0.008
    hinge_specs = {
        "front_left": (0.064, 0.044, math.radians(45.0), 0.0, math.radians(135.0)),
        "front_right": (0.064, -0.044, math.radians(-45.0), -math.radians(135.0), 0.0),
        "rear_left": (-0.064, 0.044, math.radians(135.0), -math.radians(135.0), 0.0),
        "rear_right": (-0.064, -0.044, math.radians(-135.0), 0.0, math.radians(135.0)),
    }

    for name, (x, y, yaw, lower, upper) in hinge_specs.items():
        _add_body_hinge_hardware(
            body,
            name=name,
            x=x,
            y=y,
            yaw=yaw,
            mat_plastic=dark_plastic,
            mat_metal=pin_metal,
        )

        arm = model.part(f"{name}_arm")
        _add_arm_geometry(arm, mat_carbon=carbon, mat_plastic=dark_plastic, mat_metal=metal)
        model.articulation(
            f"body_to_{name}_arm",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=(x, y, hinge_z), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=lower, upper=upper),
        )

        prop = model.part(f"{name}_propeller")
        rotor = FanRotorGeometry(
            0.070,
            0.012,
            2,
            thickness=0.004,
            blade_pitch_deg=18.0,
            blade_sweep_deg=10.0,
            blade_root_chord=0.017,
            blade_tip_chord=0.011,
            blade=FanRotorBlade(shape="narrow", tip_pitch_deg=10.0, camber=0.08),
            hub=FanRotorHub(style="flat", bore_diameter=0.004),
            center=False,
        )
        prop.visual(
            mesh_from_geometry(rotor, f"{name}_two_blade_prop"),
            material=matte_plastic,
            name="rotor",
        )
        model.articulation(
            f"{name}_prop_spin",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=prop,
            origin=Origin(xyz=(0.188, 0.0, 0.017)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=300.0),
        )

    # Fixed half of the camera tilt hinge under the front nose.
    for side, y in (("port", 0.020), ("starboard", -0.020)):
        body.visual(
            Box((0.026, 0.006, 0.014)),
            origin=Origin(xyz=(0.088, y, -0.008)),
            material=dark_plastic,
            name=f"camera_hinge_ear_{side}",
        )
    body.visual(
        Cylinder(radius=0.003, length=0.048),
        origin=Origin(xyz=(0.094, 0.0, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="camera_tilt_pin",
    )

    camera = model.part("camera_pod")
    _add_camera_pod(camera, mat_plastic=dark_plastic, mat_camera=matte_plastic, mat_glass=camera_glass)
    model.articulation(
        "body_to_camera_pod",
        ArticulationType.REVOLUTE,
        parent=body,
        child=camera,
        origin=Origin(xyz=(0.094, 0.0, -0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.60, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    hinge_names = ("front_left", "front_right", "rear_left", "rear_right")
    ctx.check(
        "four folding arms and four propellers",
        all(object_model.get_part(f"{n}_arm") for n in hinge_names)
        and all(object_model.get_part(f"{n}_propeller") for n in hinge_names),
    )

    for name in hinge_names:
        arm = object_model.get_part(f"{name}_arm")
        prop = object_model.get_part(f"{name}_propeller")
        fold = object_model.get_articulation(f"body_to_{name}_arm")
        spin = object_model.get_articulation(f"{name}_prop_spin")

        ctx.allow_overlap(
            body,
            arm,
            elem_a=f"{name}_hinge_pin",
            elem_b="root_knuckle",
            reason="A steel hinge pin is intentionally captured through the moving plastic knuckle proxy.",
        )
        ctx.expect_within(
            body,
            arm,
            axes="xy",
            inner_elem=f"{name}_hinge_pin",
            outer_elem="root_knuckle",
            margin=0.001,
            name=f"{name} hinge pin stays inside knuckle bore footprint",
        )
        ctx.expect_overlap(
            body,
            arm,
            axes="z",
            elem_a=f"{name}_hinge_pin",
            elem_b="root_knuckle",
            min_overlap=0.004,
            name=f"{name} hinge pin is retained through the knuckle",
        )
        ctx.expect_gap(
            prop,
            arm,
            axis="z",
            positive_elem="rotor",
            negative_elem="motor_cap",
            max_gap=0.002,
            max_penetration=0.0002,
            name=f"{name} propeller axle is flush above motor",
        )
        ctx.check(
            f"{name} fold axis is vertical and perpendicular to arm tube",
            tuple(round(v, 6) for v in fold.axis) == (0.0, 0.0, 1.0)
            and fold.motion_limits is not None
            and (fold.motion_limits.upper - fold.motion_limits.lower) > 2.2,
        )
        ctx.check(
            f"{name} propeller uses continuous axle",
            spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        )

    camera_joint = object_model.get_articulation("body_to_camera_pod")
    camera = object_model.get_part("camera_pod")
    ctx.allow_overlap(
        body,
        camera,
        elem_a="camera_tilt_pin",
        elem_b="tilt_barrel",
        reason="The camera tilt hinge pin is intentionally captured through the rotating barrel.",
    )
    ctx.expect_within(
        body,
        camera,
        axes="xz",
        inner_elem="camera_tilt_pin",
        outer_elem="tilt_barrel",
        margin=0.001,
        name="camera tilt pin is centered in barrel cross-section",
    )
    ctx.expect_overlap(
        body,
        camera,
        axes="y",
        elem_a="camera_tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.025,
        name="camera tilt pin spans the barrel",
    )
    ctx.check(
        "camera pod has bounded tilt hinge",
        camera_joint.articulation_type == ArticulationType.REVOLUTE
        and camera_joint.motion_limits is not None
        and camera_joint.motion_limits.lower < 0.0
        and camera_joint.motion_limits.upper > 0.0,
    )

    return ctx.report()


object_model = build_object_model()
