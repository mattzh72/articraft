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
    model = ArticulatedObject(name="marine_sensor_pan_tilt_head")

    marine_blue = model.material("marine_blue", rgba=(0.04, 0.10, 0.16, 1.0))
    dark_blue = model.material("dark_blue", rgba=(0.015, 0.035, 0.055, 1.0))
    black_glass = model.material("black_glass", rgba=(0.01, 0.015, 0.02, 1.0))
    stainless = model.material("stainless", rgba=(0.68, 0.72, 0.72, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    fixed_base = model.part("fixed_base")
    fixed_base.visual(
        Cylinder(radius=0.31, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_blue,
        name="mounting_flange",
    )
    fixed_base.visual(
        Cylinder(radius=0.225, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=marine_blue,
        name="bearing_pedestal",
    )
    fixed_base.visual(
        Cylinder(radius=0.245, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material=rubber,
        name="weather_seal",
    )
    for index, (x, y) in enumerate(
        ((0.215, 0.0), (-0.215, 0.0), (0.0, 0.215), (0.0, -0.215))
    ):
        fixed_base.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(xyz=(x, y, 0.035)),
            material=stainless,
            name=f"bolt_head_{index}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.230, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=marine_blue,
        name="yaw_disk",
    )
    turntable.visual(
        Cylinder(radius=0.185, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=dark_blue,
        name="top_bearing_cap",
    )
    turntable.visual(
        Box((0.500, 0.180, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=marine_blue,
        name="fork_foot",
    )
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.500, 0.180, 0.520),
            span_width=0.350,
            trunnion_diameter=0.090,
            trunnion_center_z=0.390,
            base_thickness=0.085,
            corner_radius=0.020,
            center=False,
        ),
        "fork_yoke",
    )
    turntable.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=marine_blue,
        name="fork_yoke",
    )
    turntable.visual(
        Box((0.360, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, -0.082, 0.105)),
        material=dark_blue,
        name="rear_cable_bridge",
    )

    sensor_frame = model.part("sensor_frame")
    # The child frame is centered on the horizontal trunnion axis.
    sensor_frame.visual(
        Box((0.290, 0.090, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=dark_blue,
        name="top_rail",
    )
    sensor_frame.visual(
        Box((0.290, 0.090, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=dark_blue,
        name="bottom_rail",
    )
    sensor_frame.visual(
        Box((0.038, 0.090, 0.295)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=dark_blue,
        name="side_rail_0",
    )
    sensor_frame.visual(
        Box((0.038, 0.090, 0.295)),
        origin=Origin(xyz=(-0.145, 0.0, 0.0)),
        material=dark_blue,
        name="side_rail_1",
    )
    sensor_frame.visual(
        Box((0.255, 0.034, 0.215)),
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=black_glass,
        name="sensor_window",
    )
    sensor_frame.visual(
        Cylinder(radius=0.047, length=0.100),
        origin=Origin(xyz=(0.200, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="trunnion_0",
    )
    sensor_frame.visual(
        Cylinder(radius=0.047, length=0.100),
        origin=Origin(xyz=(-0.200, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="trunnion_1",
    )
    sensor_frame.visual(
        Cylinder(radius=0.020, length=0.145),
        origin=Origin(xyz=(0.0, 0.030, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="cable_gland",
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=fixed_base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=sensor_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=-0.70, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_base = object_model.get_part("fixed_base")
    turntable = object_model.get_part("turntable")
    sensor_frame = object_model.get_part("sensor_frame")
    yaw_axis = object_model.get_articulation("yaw_axis")
    pitch_axis = object_model.get_articulation("pitch_axis")

    ctx.allow_overlap(
        turntable,
        sensor_frame,
        elem_a="fork_yoke",
        elem_b="trunnion_0",
        reason="The trunnion journal is intentionally captured in the fork bore as the pitch bearing.",
    )
    ctx.allow_overlap(
        turntable,
        sensor_frame,
        elem_a="fork_yoke",
        elem_b="trunnion_1",
        reason="The trunnion journal is intentionally captured in the fork bore as the pitch bearing.",
    )
    ctx.expect_overlap(
        sensor_frame,
        turntable,
        axes="x",
        min_overlap=0.030,
        elem_a="trunnion_0",
        elem_b="fork_yoke",
        name="trunnion 0 remains inserted in fork bore",
    )
    ctx.expect_overlap(
        sensor_frame,
        turntable,
        axes="x",
        min_overlap=0.030,
        elem_a="trunnion_1",
        elem_b="fork_yoke",
        name="trunnion 1 remains inserted in fork bore",
    )
    ctx.expect_gap(
        turntable,
        fixed_base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="yaw_disk",
        negative_elem="weather_seal",
        name="yaw disk is seated on the base seal",
    )
    ctx.expect_overlap(
        turntable,
        fixed_base,
        axes="xy",
        min_overlap=0.18,
        elem_a="yaw_disk",
        elem_b="bearing_pedestal",
        name="turntable is centered on fixed pedestal",
    )
    ctx.expect_gap(
        sensor_frame,
        turntable,
        axis="z",
        min_gap=0.18,
        positive_elem="bottom_rail",
        negative_elem="yaw_disk",
        name="sensor frame is nested above the low yaw disk",
    )
    ctx.expect_within(
        sensor_frame,
        turntable,
        axes="x",
        margin=0.010,
        name="sensor frame is suspended between fork arms",
    )

    rest_aabb = ctx.part_element_world_aabb(sensor_frame, elem="sensor_window")
    with ctx.pose({yaw_axis: 0.65, pitch_axis: 0.55}):
        ctx.expect_within(
            sensor_frame,
            turntable,
            axes="x",
            margin=0.010,
            name="pitched frame stays between fork arms",
        )
        pitched_aabb = ctx.part_element_world_aabb(sensor_frame, elem="sensor_window")
    rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    pitched_center_z = None if pitched_aabb is None else (pitched_aabb[0][2] + pitched_aabb[1][2]) * 0.5
    ctx.check(
        "pitch axis tilts the sensor window",
        rest_center_z is not None
        and pitched_center_z is not None
        and abs(pitched_center_z - rest_center_z) > 0.020,
        details=f"rest_z={rest_center_z}, pitched_z={pitched_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
