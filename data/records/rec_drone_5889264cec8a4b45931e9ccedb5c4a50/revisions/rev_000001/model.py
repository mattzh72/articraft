from __future__ import annotations

import math

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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_rotor_quadrotor")

    carbon = model.material("carbon_black", rgba=(0.02, 0.025, 0.025, 1.0))
    dark = model.material("matte_graphite", rgba=(0.09, 0.095, 0.10, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.70, 0.72, 1.0))
    orange = model.material("warning_orange", rgba=(1.0, 0.36, 0.06, 1.0))
    blue = model.material("avionics_blue", rgba=(0.05, 0.18, 0.34, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        Box((0.34, 0.20, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=blue,
        name="avionics_bay",
    )
    airframe.visual(
        Box((1.04, 0.040, 0.034)),
        origin=Origin(xyz=(0.0, 0.300, 0.060)),
        material=carbon,
        name="side_rail_0",
    )
    airframe.visual(
        Box((1.04, 0.040, 0.034)),
        origin=Origin(xyz=(0.0, -0.300, 0.060)),
        material=carbon,
        name="side_rail_1",
    )
    airframe.visual(
        Box((0.040, 0.640, 0.034)),
        origin=Origin(xyz=(0.500, 0.0, 0.060)),
        material=carbon,
        name="end_rail_0",
    )
    airframe.visual(
        Box((0.040, 0.640, 0.034)),
        origin=Origin(xyz=(-0.500, 0.0, 0.060)),
        material=carbon,
        name="end_rail_1",
    )
    airframe.visual(
        Box((0.040, 0.640, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.086), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=carbon,
        name="mid_cross_tube",
    )

    hinge_points = (
        (0.560, 0.350, 0.132),
        (0.560, -0.350, 0.132),
        (-0.560, 0.350, 0.132),
        (-0.560, -0.350, 0.132),
    )

    for idx, (x, y, hinge_z) in enumerate(hinge_points):
        start_x = 0.150 if x > 0.0 else -0.150
        start_y = 0.090 if y > 0.0 else -0.090
        arm_center = ((start_x + x) / 2.0, (start_y + y) / 2.0, 0.074)
        arm_length = math.hypot(x - start_x, y - start_y) + 0.040
        arm_yaw = math.atan2(y - start_y, x - start_x)
        airframe.visual(
            Box((arm_length, 0.034, 0.034)),
            origin=Origin(xyz=arm_center, rpy=(0.0, 0.0, arm_yaw)),
            material=carbon,
            name=f"arm_{idx}",
        )
        airframe.visual(
            Box((0.105, 0.168, 0.030)),
            origin=Origin(xyz=(x, y, 0.074)),
            material=carbon,
            name=f"yoke_base_{idx}",
        )
        for sign, suffix in ((1.0, "a"), (-1.0, "b")):
            airframe.visual(
                Box((0.086, 0.018, 0.102)),
                origin=Origin(xyz=(x, y + sign * 0.071, 0.132)),
                material=carbon,
                name=f"yoke_cheek_{idx}_{suffix}",
            )
            airframe.visual(
                Cylinder(radius=0.019, length=0.008),
                origin=Origin(
                    xyz=(x, y + sign * 0.083, hinge_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=aluminum,
                name=f"bearing_cap_{idx}_{suffix}",
            )

    prop_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.160,
            0.030,
            2,
            thickness=0.014,
            blade_pitch_deg=24.0,
            blade_sweep_deg=16.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=10.0, camber=0.10),
            hub=FanRotorHub(style="spinner", bore_diameter=0.006),
            center=False,
        ),
        "two_blade_propeller",
    )

    for idx, (x, y, hinge_z) in enumerate(hinge_points):
        pod = model.part(f"pod_{idx}")
        pod.visual(
            Cylinder(radius=0.014, length=0.116),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="tilt_trunnion",
        )
        pod.visual(
            Cylinder(radius=0.046, length=0.083),
            origin=Origin(xyz=(0.0, 0.0, 0.0415)),
            material=dark,
            name="motor_can",
        )
        pod.visual(
            Cylinder(radius=0.049, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.090)),
            material=aluminum,
            name="top_collar",
        )
        pod.visual(
            Cylinder(radius=0.012, length=0.023),
            origin=Origin(xyz=(0.0, 0.0, 0.1085)),
            material=aluminum,
            name="prop_shaft",
        )
        pod.visual(
            Box((0.080, 0.018, 0.022)),
            origin=Origin(xyz=(0.0, -0.053, 0.026)),
            material=orange,
            name="service_lug",
        )

        model.articulation(
            f"tilt_{idx}",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=pod,
            origin=Origin(xyz=(x, y, hinge_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-0.25, upper=1.45),
        )

        propeller = model.part(f"propeller_{idx}")
        propeller.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=aluminum,
            name="hub_socket",
        )
        propeller.visual(
            prop_mesh,
            origin=Origin(),
            material=Material("satin_black", rgba=(0.015, 0.015, 0.014, 1.0)),
            name="two_blade_rotor",
        )
        model.articulation(
            f"axle_{idx}",
            ArticulationType.CONTINUOUS,
            parent=pod,
            child=propeller,
            origin=Origin(xyz=(0.0, 0.0, 0.120)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=80.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.check(
        "airframe plus four tilting pods and four propellers",
        len(object_model.parts) == 9,
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "four pod tilt hinges and four propeller axles",
        len(object_model.articulations) == 8,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    for idx in range(4):
        tilt = object_model.get_articulation(f"tilt_{idx}")
        axle = object_model.get_articulation(f"axle_{idx}")
        ctx.check(
            f"pod_{idx} uses a bounded tilt hinge",
            tilt.articulation_type == ArticulationType.REVOLUTE
            and tilt.axis == (0.0, 1.0, 0.0)
            and tilt.motion_limits is not None
            and tilt.motion_limits.lower < 0.0
            and tilt.motion_limits.upper > 1.2,
            details=f"type={tilt.articulation_type}, axis={tilt.axis}, limits={tilt.motion_limits}",
        )
        ctx.check(
            f"propeller_{idx} spins on its own axle",
            axle.articulation_type == ArticulationType.CONTINUOUS
            and axle.axis == (0.0, 0.0, 1.0),
            details=f"type={axle.articulation_type}, axis={axle.axis}",
        )
        ctx.expect_gap(
            f"propeller_{idx}",
            f"pod_{idx}",
            axis="z",
            positive_elem="hub_socket",
            negative_elem="prop_shaft",
            min_gap=0.0,
            max_gap=0.001,
            name=f"propeller_{idx} hub sits on its motor shaft",
        )
        ctx.expect_overlap(
            f"propeller_{idx}",
            f"pod_{idx}",
            axes="xy",
            elem_a="hub_socket",
            elem_b="prop_shaft",
            min_overlap=0.018,
            name=f"propeller_{idx} hub is centered on shaft",
        )

    prop_0 = object_model.get_part("propeller_0")
    tilt_0 = object_model.get_articulation("tilt_0")
    rest_position = ctx.part_world_position(prop_0)
    with ctx.pose({tilt_0: 1.0}):
        tilted_position = ctx.part_world_position(prop_0)
    ctx.check(
        "front pod tilt drives propeller forward",
        rest_position is not None
        and tilted_position is not None
        and tilted_position[0] > rest_position[0] + 0.080,
        details=f"rest={rest_position}, tilted={tilted_position}",
    )

    return ctx.report()


object_model = build_object_model()
