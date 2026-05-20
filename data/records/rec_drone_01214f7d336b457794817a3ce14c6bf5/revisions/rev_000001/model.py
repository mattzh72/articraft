from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_LENGTH = 0.34
BODY_WIDTH = 0.20
BODY_HEIGHT = 0.08

HINGE_X = 0.190
HINGE_Y = 0.120
ARM_LENGTH = 0.255
MOTOR_X = 0.285
PROP_Z = 0.048


ARM_SPECS = {
    "front_left": {
        "hinge": (HINGE_X, HINGE_Y, 0.002),
        "yaw": math.radians(45.0),
        "limits": (0.0, math.radians(95.0)),
    },
    "front_right": {
        "hinge": (HINGE_X, -HINGE_Y, 0.002),
        "yaw": math.radians(-45.0),
        "limits": (math.radians(-95.0), 0.0),
    },
    "rear_left": {
        "hinge": (-HINGE_X, HINGE_Y, 0.002),
        "yaw": math.radians(135.0),
        "limits": (math.radians(-95.0), 0.0),
    },
    "rear_right": {
        "hinge": (-HINGE_X, -HINGE_Y, 0.002),
        "yaw": math.radians(-135.0),
        "limits": (0.0, math.radians(95.0)),
    },
}


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _add_body_socket(body, cue: str, xyz: tuple[float, float, float], yaw: float, material) -> None:
    x, y, z = xyz
    # A small fairing grows out of the rectangular fuselage corner and carries the
    # vertical folding-arm hinge pin.
    body.visual(
        Box((0.064, 0.030, 0.018)),
        origin=Origin(
            xyz=(x - 0.020 * math.cos(yaw), y - 0.020 * math.sin(yaw), z - 0.028),
            rpy=(0.0, 0.0, yaw),
        ),
        material=material,
        name=f"{cue}_fairing",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=xyz),
        material=material,
        name=f"{cue}_socket",
    )


def _add_arm_visuals(arm, material_arm, material_motor, material_light) -> None:
    arm.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=material_motor,
        name="hinge_eye",
    )
    arm.visual(
        Box((ARM_LENGTH, 0.026, 0.018)),
        origin=Origin(xyz=(0.013 + ARM_LENGTH / 2.0, 0.0, 0.004)),
        material=material_arm,
        name="carbon_beam",
    )
    arm.visual(
        Box((0.070, 0.034, 0.018)),
        origin=Origin(xyz=(MOTOR_X - 0.050, 0.0, 0.006)),
        material=material_arm,
        name="tip_reinforcement",
    )
    arm.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=Origin(xyz=(MOTOR_X, 0.0, 0.019)),
        material=material_motor,
        name="motor_pod",
    )
    arm.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(MOTOR_X, 0.0, 0.043)),
        material=material_motor,
        name="prop_shaft",
    )
    arm.visual(
        Box((0.018, 0.006, 0.010)),
        origin=Origin(xyz=(MOTOR_X, 0.030, 0.020)),
        material=material_light,
        name="nav_light",
    )


def _add_propeller(propeller, cue: str, material_prop, material_hub) -> None:
    rotor = FanRotorGeometry(
        outer_radius=0.116,
        hub_radius=0.018,
        blade_count=3,
        thickness=0.008,
        blade_pitch_deg=24.0,
        blade_sweep_deg=28.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=10.0, camber=0.10),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.004, rear_collar_radius=0.014, bore_diameter=0.004),
    )
    propeller.visual(
        mesh_from_geometry(rotor, f"{cue}_propeller_rotor"),
        material=material_prop,
        name="rotor",
    )
    propeller.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=material_hub,
        name="spinner",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_folding_quadcopter")

    body_mat = model.material("warm_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    panel_mat = model.material("matte_black_panel", rgba=(0.015, 0.016, 0.018, 1.0))
    carbon_mat = model.material("carbon_black", rgba=(0.02, 0.022, 0.024, 1.0))
    gunmetal_mat = model.material("gunmetal", rgba=(0.20, 0.21, 0.22, 1.0))
    glass_mat = model.material("blue_black_glass", rgba=(0.02, 0.05, 0.075, 1.0))
    red_mat = model.material("navigation_red", rgba=(0.8, 0.04, 0.02, 1.0))
    green_mat = model.material("navigation_green", rgba=(0.02, 0.55, 0.10, 1.0))

    body = model.part("body")
    body_shape = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
        .edges()
        .fillet(0.014)
    )
    body.visual(mesh_from_cadquery(body_shape, "rounded_rect_body"), material=body_mat, name="body_shell")
    body.visual(
        Box((0.190, 0.118, 0.006)),
        origin=Origin(xyz=(-0.015, 0.0, BODY_HEIGHT / 2.0 + 0.003)),
        material=panel_mat,
        name="top_battery_panel",
    )
    body.visual(
        Box((0.006, 0.080, 0.032)),
        origin=Origin(xyz=(BODY_LENGTH / 2.0 + 0.003, 0.0, 0.002)),
        material=glass_mat,
        name="front_sensor_window",
    )
    body.visual(
        Box((0.080, 0.060, 0.012)),
        origin=Origin(xyz=(0.120, 0.0, -BODY_HEIGHT / 2.0 - 0.006)),
        material=gunmetal_mat,
        name="gimbal_plate",
    )

    for cue, spec in ARM_SPECS.items():
        _add_body_socket(body, cue, spec["hinge"], spec["yaw"], gunmetal_mat)

    for cue, spec in ARM_SPECS.items():
        arm = model.part(f"{cue}_arm")
        light = green_mat if "right" in cue else red_mat
        _add_arm_visuals(arm, carbon_mat, gunmetal_mat, light)
        lower, upper = spec["limits"]
        model.articulation(
            f"{cue}_arm_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=spec["hinge"], rpy=(0.0, 0.0, spec["yaw"])),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=lower, upper=upper),
        )

        propeller = model.part(f"{cue}_propeller")
        _add_propeller(propeller, cue, carbon_mat, gunmetal_mat)
        model.articulation(
            f"{cue}_propeller_spin",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=propeller,
            origin=Origin(xyz=(MOTOR_X, 0.0, PROP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=120.0),
        )

    gimbal_yaw = model.part("gimbal_yaw")
    gimbal_yaw.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=gunmetal_mat,
        name="pan_motor",
    )
    gimbal_yaw.visual(
        Cylinder(radius=0.012, length=0.039),
        origin=Origin(xyz=(0.0, 0.0, -0.0455)),
        material=gunmetal_mat,
        name="drop_stem",
    )

    gimbal_tilt = model.part("gimbal_tilt")
    gimbal_tilt.visual(
        Cylinder(radius=0.018, length=0.104),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal_mat,
        name="tilt_axle",
    )
    gimbal_tilt.visual(
        Box((0.024, 0.018, 0.082)),
        origin=Origin(xyz=(0.0, 0.052, -0.040)),
        material=gunmetal_mat,
        name="yoke_cheek_0",
    )
    gimbal_tilt.visual(
        Box((0.024, 0.018, 0.082)),
        origin=Origin(xyz=(0.0, -0.052, -0.040)),
        material=gunmetal_mat,
        name="yoke_cheek_1",
    )
    gimbal_tilt.visual(
        Box((0.040, 0.092, 0.012)),
        origin=Origin(xyz=(-0.010, 0.0, -0.075)),
        material=gunmetal_mat,
        name="lower_yoke_bridge",
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.066, 0.056, 0.044)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=panel_mat,
        name="camera_body",
    )
    camera.visual(
        Cylinder(radius=0.011, length=0.086),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal_mat,
        name="roll_trunnion",
    )
    camera.visual(
        Cylinder(radius=0.019, length=0.036),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal_mat,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.098, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_mat,
        name="front_glass",
    )

    model.articulation(
        "body_to_gimbal_yaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=gimbal_yaw,
        origin=Origin(xyz=(0.120, 0.0, -BODY_HEIGHT / 2.0 - 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "gimbal_yaw_to_tilt",
        ArticulationType.REVOLUTE,
        parent=gimbal_yaw,
        child=gimbal_tilt,
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.60, upper=0.55),
    )
    model.articulation(
        "gimbal_tilt_to_camera",
        ArticulationType.REVOLUTE,
        parent=gimbal_tilt,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=-0.60, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    for cue in ARM_SPECS:
        arm = object_model.get_part(f"{cue}_arm")
        propeller = object_model.get_part(f"{cue}_propeller")
        hinge = object_model.get_articulation(f"{cue}_arm_hinge")
        spin = object_model.get_articulation(f"{cue}_propeller_spin")

        ctx.check(
            f"{cue} arm hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
            details=f"{cue} hinge type is {hinge.articulation_type}",
        )
        ctx.check(
            f"{cue} propeller spins continuously",
            spin.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{cue} propeller joint type is {spin.articulation_type}",
        )

        ctx.allow_overlap(
            body,
            arm,
            elem_a=f"{cue}_socket",
            elem_b="hinge_eye",
            reason="The folding arm hinge eye is intentionally captured around the body-side vertical hinge pin.",
        )
        ctx.expect_overlap(
            body,
            arm,
            axes="xy",
            elem_a=f"{cue}_socket",
            elem_b="hinge_eye",
            min_overlap=0.014,
            name=f"{cue} hinge pin is captured in the arm eye",
        )
        ctx.expect_gap(
            arm,
            body,
            axis="z",
            positive_elem="hinge_eye",
            negative_elem=f"{cue}_socket",
            max_penetration=0.036,
            name=f"{cue} hinge capture is local in height",
        )
        ctx.expect_gap(
            propeller,
            arm,
            axis="z",
            positive_elem="rotor",
            negative_elem="motor_pod",
            max_gap=0.008,
            max_penetration=0.004,
            name=f"{cue} propeller sits on its motor",
        )
        ctx.allow_overlap(
            arm,
            propeller,
            elem_a="prop_shaft",
            elem_b="rotor",
            reason="The motor shaft is intentionally inserted into the propeller hub to carry the continuous rotor.",
        )
        ctx.expect_overlap(
            arm,
            propeller,
            axes="xy",
            elem_a="prop_shaft",
            elem_b="rotor",
            min_overlap=0.010,
            name=f"{cue} propeller hub is centered on its shaft",
        )

    for joint_name in ("body_to_gimbal_yaw", "gimbal_yaw_to_tilt", "gimbal_tilt_to_camera"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint_name} type is {joint.articulation_type}",
        )

    gimbal_yaw = object_model.get_part("gimbal_yaw")
    gimbal_tilt = object_model.get_part("gimbal_tilt")
    ctx.allow_overlap(
        gimbal_yaw,
        gimbal_tilt,
        elem_a="drop_stem",
        elem_b="tilt_axle",
        reason="The vertical gimbal stem is intentionally captured in the pitch motor axle at the stacked gimbal joint.",
    )
    ctx.expect_overlap(
        gimbal_yaw,
        gimbal_tilt,
        axes="xy",
        elem_a="drop_stem",
        elem_b="tilt_axle",
        min_overlap=0.020,
        name="gimbal tilt axle is centered under the yaw stem",
    )

    camera = object_model.get_part("camera")
    camera_position = ctx.part_world_position(camera)
    ctx.check(
        "camera gimbal is under the nose",
        camera_position is not None and camera_position[0] > 0.08 and camera_position[2] < -0.12,
        details=f"camera origin={camera_position}",
    )

    front_left_propeller = object_model.get_part("front_left_propeller")
    front_left_hinge = object_model.get_articulation("front_left_arm_hinge")
    rest_pos = ctx.part_world_position(front_left_propeller)
    with ctx.pose({front_left_hinge: ARM_SPECS["front_left"]["limits"][1]}):
        folded_pos = ctx.part_world_position(front_left_propeller)
    ctx.check(
        "front folding arm swings inward at its hinge",
        rest_pos is not None and folded_pos is not None and folded_pos[0] < rest_pos[0] - 0.08,
        details=f"rest={rest_pos}, folded={folded_pos}",
    )

    return ctx.report()


object_model = build_object_model()
