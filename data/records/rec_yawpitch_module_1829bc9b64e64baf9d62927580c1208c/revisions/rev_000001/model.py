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
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_pan_tilt_unit")

    model.material("matte_black", rgba=(0.025, 0.028, 0.032, 1.0))
    model.material("anodized_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("machined_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("dark_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    model.material("fastener_black", rgba=(0.02, 0.02, 0.022, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.240, 0.180, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="matte_black",
        name="mounting_foot",
    )
    housing.visual(
        Cylinder(radius=0.076, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material="matte_black",
        name="motor_housing",
    )
    housing.visual(
        Cylinder(radius=0.094, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material="anodized_graphite",
        name="fixed_bearing_race",
    )
    for index, (x, y) in enumerate(
        ((-0.085, -0.060), (-0.085, 0.060), (0.085, -0.060), (0.085, 0.060))
    ):
        housing.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.028)),
            material="fastener_black",
            name=f"base_screw_{index}",
        )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.086, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material="machined_aluminum",
        name="turntable_disk",
    )
    yaw_stage.visual(
        Cylinder(radius=0.057, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material="anodized_graphite",
        name="rotary_cap",
    )
    yaw_stage.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.155, 0.118, 0.155),
                span_width=0.086,
                trunnion_diameter=0.032,
                trunnion_center_z=0.112,
                base_thickness=0.030,
                corner_radius=0.006,
                center=False,
            ),
            "saddle_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="anodized_graphite",
        name="saddle_yoke",
    )
    yaw_stage.visual(
        Box((0.050, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.043, 0.033)),
        material="dark_rubber",
        name="yaw_index_mark",
    )

    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        Cylinder(radius=0.013, length=0.154),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_aluminum",
        name="pitch_axle",
    )
    tilt_cradle.visual(
        Cylinder(radius=0.024, length=0.007),
        origin=Origin(xyz=(-0.0805, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_aluminum",
        name="axle_collar_0",
    )
    tilt_cradle.visual(
        Cylinder(radius=0.024, length=0.007),
        origin=Origin(xyz=(0.0805, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_aluminum",
        name="axle_collar_1",
    )
    tilt_cradle.visual(
        Box((0.064, 0.058, 0.052)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material="machined_aluminum",
        name="cradle_body",
    )
    tilt_cradle.visual(
        Box((0.050, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.058, 0.0)),
        material="machined_aluminum",
        name="plate_neck",
    )

    output_plate = model.part("output_plate")
    output_plate.visual(
        Box((0.092, 0.014, 0.066)),
        material="machined_aluminum",
        name="mounting_plate",
    )
    output_plate.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="anodized_graphite",
        name="output_boss",
    )
    for index, (x, z) in enumerate(
        ((-0.030, -0.020), (-0.030, 0.020), (0.030, -0.020), (0.030, 0.020))
    ):
        output_plate.visual(
            Cylinder(radius=0.005, length=0.003),
            origin=Origin(xyz=(x, 0.0084, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="fastener_black",
            name=f"plate_screw_{index}",
        )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.4, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=tilt_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=1.8, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "cradle_to_plate",
        ArticulationType.FIXED,
        parent=tilt_cradle,
        child=output_plate,
        origin=Origin(xyz=(0.0, 0.0875, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    yaw_stage = object_model.get_part("yaw_stage")
    tilt_cradle = object_model.get_part("tilt_cradle")
    output_plate = object_model.get_part("output_plate")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")

    ctx.check(
        "two user-facing revolute axes",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE,
        details=f"yaw={yaw.articulation_type}, pitch={pitch.articulation_type}",
    )

    ctx.expect_gap(
        yaw_stage,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable seats on fixed bearing race",
    )
    ctx.expect_overlap(
        yaw_stage,
        housing,
        axes="xy",
        min_overlap=0.150,
        name="lower rotary stage is centered over housing",
    )
    ctx.expect_overlap(
        tilt_cradle,
        yaw_stage,
        axes="x",
        elem_a="pitch_axle",
        elem_b="saddle_yoke",
        min_overlap=0.120,
        name="pitch axle spans the saddle yoke",
    )
    ctx.expect_within(
        tilt_cradle,
        yaw_stage,
        axes="y",
        inner_elem="cradle_body",
        outer_elem="saddle_yoke",
        margin=0.002,
        name="tilt cradle body sits inside the yoke cheeks",
    )
    ctx.expect_gap(
        output_plate,
        tilt_cradle,
        axis="y",
        max_gap=0.001,
        max_penetration=0.000001,
        name="output plate is flush on the cradle neck",
    )
    ctx.expect_overlap(
        output_plate,
        tilt_cradle,
        axes="xz",
        min_overlap=0.030,
        name="output plate is supported by the cradle neck",
    )

    rest_plate = ctx.part_world_position(output_plate)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_plate = ctx.part_world_position(output_plate)
    ctx.check(
        "yaw axis pans the upper assembly horizontally",
        rest_plate is not None
        and yawed_plate is not None
        and yawed_plate[0] < rest_plate[0] - 0.060
        and abs(yawed_plate[2] - rest_plate[2]) < 0.002,
        details=f"rest={rest_plate}, yawed={yawed_plate}",
    )

    with ctx.pose({pitch: 0.70}):
        pitched_up = ctx.part_world_position(output_plate)
    with ctx.pose({pitch: -0.70}):
        pitched_down = ctx.part_world_position(output_plate)
    ctx.check(
        "pitch axis tilts the output plate up and down",
        rest_plate is not None
        and pitched_up is not None
        and pitched_down is not None
        and pitched_up[2] > rest_plate[2] + 0.045
        and pitched_down[2] < rest_plate[2] - 0.045,
        details=f"rest={rest_plate}, up={pitched_up}, down={pitched_down}",
    )

    return ctx.report()


object_model = build_object_model()
