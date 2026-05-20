from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_roll_axis_module")

    anodized = model.material("clear_anodized_aluminum", rgba=(0.66, 0.68, 0.69, 1.0))
    dark_anodized = model.material("dark_anodized_aluminum", rgba=(0.06, 0.065, 0.07, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    black_screw = model.material("black_socket_screws", rgba=(0.01, 0.012, 0.014, 1.0))

    shaft_z = 0.160
    bearing_x = (-0.150, 0.150)
    bearing_length = 0.070

    frame = model.part("frame")
    frame.visual(
        Box((0.560, 0.190, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_anodized,
        name="base_plate",
    )
    frame.visual(
        Box((0.040, 0.165, 0.225)),
        origin=Origin(xyz=(-0.255, 0.0, 0.1365)),
        material=dark_anodized,
        name="side_upright_0",
    )
    frame.visual(
        Box((0.040, 0.165, 0.225)),
        origin=Origin(xyz=(0.255, 0.0, 0.1365)),
        material=dark_anodized,
        name="side_upright_1",
    )
    frame.visual(
        Box((0.550, 0.165, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.261)),
        material=dark_anodized,
        name="top_crossmember",
    )
    frame.visual(
        Box((0.550, 0.020, 0.205)),
        origin=Origin(xyz=(0.0, 0.082, 0.1425)),
        material=dark_anodized,
        name="rear_spine",
    )

    for i, x in enumerate(bearing_x):
        frame.visual(
            Box((0.085, 0.074, 0.078)),
            origin=Origin(xyz=(x, 0.0, 0.062)),
            material=dark_anodized,
            name=f"bearing_pedestal_{i}",
        )
        frame.visual(
            Box((0.074, 0.128, 0.030)),
            origin=Origin(xyz=(x, 0.0, shaft_z + 0.045)),
            material=anodized,
            name=f"bearing_top_{i}",
        )
        frame.visual(
            Box((0.074, 0.128, 0.030)),
            origin=Origin(xyz=(x, 0.0, shaft_z - 0.045)),
            material=anodized,
            name=f"bearing_bottom_{i}",
        )
        frame.visual(
            Box((0.074, 0.028, 0.120)),
            origin=Origin(xyz=(x, -0.050, shaft_z)),
            material=anodized,
            name=f"bearing_side_0_{i}",
        )
        frame.visual(
            Box((0.074, 0.028, 0.120)),
            origin=Origin(xyz=(x, 0.050, shaft_z)),
            material=anodized,
            name=f"bearing_side_1_{i}",
        )
        frame.visual(
            Box((0.080, 0.070, 0.013)),
            origin=Origin(xyz=(x, 0.0, shaft_z + 0.02375)),
            material=bearing_steel,
            name=f"bearing_liner_top_{i}",
        )
        frame.visual(
            Box((0.080, 0.070, 0.013)),
            origin=Origin(xyz=(x, 0.0, shaft_z - 0.02375)),
            material=bearing_steel,
            name=f"bearing_liner_bottom_{i}",
        )
        frame.visual(
            Box((0.080, 0.0185, 0.061)),
            origin=Origin(xyz=(x, -0.02675, shaft_z)),
            material=bearing_steel,
            name=f"bearing_liner_side_0_{i}",
        )
        frame.visual(
            Box((0.080, 0.0185, 0.061)),
            origin=Origin(xyz=(x, 0.02675, shaft_z)),
            material=bearing_steel,
            name=f"bearing_liner_side_1_{i}",
        )
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                frame.visual(
                    Cylinder(radius=0.0048, length=0.004),
                    origin=Origin(
                        xyz=(x - bearing_length / 2.0 - 0.002, sy * 0.039, shaft_z + sz * 0.039),
                        rpy=(0.0, math.pi / 2.0, 0.0),
                    ),
                    material=black_screw,
                    name=f"cartridge_screw_{i}_{int(sy > 0)}_{int(sz > 0)}",
                )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.0175, length=0.430),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.035, length=0.076),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="clamp_collar",
    )
    rotor.visual(
        Box((0.088, 0.042, 0.052)),
        origin=Origin(xyz=(0.0, -0.033, 0.0)),
        material=anodized,
        name="clamp_bridge",
    )
    rotor.visual(
        Box((0.116, 0.076, 0.026)),
        origin=Origin(xyz=(0.0, -0.074, 0.0)),
        material=anodized,
        name="payload_neck",
    )
    rotor.visual(
        Box((0.132, 0.012, 0.086)),
        origin=Origin(xyz=(0.0, -0.112, 0.0)),
        material=anodized,
        name="payload_plate",
    )
    rotor.visual(
        Box((0.112, 0.010, 0.058)),
        origin=Origin(xyz=(0.0, -0.1195, 0.0)),
        material=dark_anodized,
        name="payload_mount_face",
    )
    for ix, x in enumerate((-0.039, 0.039)):
        for iz, z in enumerate((-0.025, 0.025)):
            rotor.visual(
                Cylinder(radius=0.0065, length=0.005),
                origin=Origin(xyz=(x, -0.125, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=black_screw,
                name=f"payload_screw_{ix}_{iz}",
            )

    model.articulation(
        "roll_axis",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    roll = object_model.get_articulation("roll_axis")

    ctx.check(
        "single coaxial roll joint",
        len(object_model.articulations) == 1
        and roll.articulation_type == ArticulationType.CONTINUOUS
        and tuple(roll.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )
    for i in (0, 1):
        ctx.allow_overlap(
            frame,
            rotor,
            elem_a=f"bearing_liner_top_{i}",
            elem_b="shaft",
            reason="The bearing race is modeled with a tiny local preload against the rotating shaft.",
        )
        ctx.allow_overlap(
            frame,
            rotor,
            elem_a=f"bearing_liner_bottom_{i}",
            elem_b="shaft",
            reason="The bearing race is modeled with a tiny local preload against the rotating shaft.",
        )
        ctx.expect_overlap(
            rotor,
            frame,
            axes="x",
            elem_a="shaft",
            elem_b=f"bearing_liner_top_{i}",
            min_overlap=0.060,
            name=f"shaft passes through bearing cartridge {i}",
        )
        ctx.expect_gap(
            frame,
            rotor,
            axis="z",
            positive_elem=f"bearing_liner_top_{i}",
            negative_elem="shaft",
            max_penetration=0.0005,
            max_gap=0.001,
            name=f"top bearing clearance {i}",
        )
        ctx.expect_gap(
            rotor,
            frame,
            axis="z",
            positive_elem="shaft",
            negative_elem=f"bearing_liner_bottom_{i}",
            max_penetration=0.0005,
            max_gap=0.001,
            name=f"bottom bearing clearance {i}",
        )
        ctx.expect_gap(
            frame,
            rotor,
            axis="y",
            positive_elem=f"bearing_liner_side_1_{i}",
            negative_elem="shaft",
            min_gap=0.0,
            max_gap=0.001,
            name=f"positive bearing side clearance {i}",
        )
        ctx.expect_gap(
            rotor,
            frame,
            axis="y",
            positive_elem="shaft",
            negative_elem=f"bearing_liner_side_0_{i}",
            min_gap=0.0,
            max_gap=0.001,
            name=f"negative bearing side clearance {i}",
        )

    rest_payload = ctx.part_element_world_aabb(rotor, elem="payload_plate")
    with ctx.pose({roll: math.pi / 2.0}):
        turned_payload = ctx.part_element_world_aabb(rotor, elem="payload_plate")
    rest_center_y = (rest_payload[0][1] + rest_payload[1][1]) / 2.0 if rest_payload else None
    turned_center_z = (turned_payload[0][2] + turned_payload[1][2]) / 2.0 if turned_payload else None
    ctx.check(
        "payload bracket rotates about fixed shaft axis",
        rest_center_y is not None
        and turned_center_z is not None
        and rest_center_y < -0.10
        and turned_center_z < 0.08,
        details=f"rest_aabb={rest_payload}, turned_aabb={turned_payload}",
    )

    return ctx.report()


object_model = build_object_model()
