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
    model = ArticulatedObject(name="marine_sensor_pan_tilt_head")

    dark_hardcoat = model.material("dark_hardcoat", rgba=(0.04, 0.055, 0.065, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.66, 0.69, 0.70, 1.0))
    cast_gray = model.material("cast_gray", rgba=(0.39, 0.43, 0.45, 1.0))
    seal_black = model.material("seal_black", rgba=(0.01, 0.012, 0.014, 1.0))
    glass = model.material("smoked_glass", rgba=(0.02, 0.035, 0.045, 0.72))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.335, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_hardcoat,
        name="mount_flange",
    )
    base.visual(
        Cylinder(radius=0.235, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=cast_gray,
        name="stationary_bearing_housing",
    )
    base.visual(
        Cylinder(radius=0.255, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=machined_aluminum,
        name="machined_lower_face",
    )
    base.visual(
        Cylinder(radius=0.222, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=seal_black,
        name="azimuth_seal",
    )
    for index in range(8):
        angle = 2.0 * math.pi * index / 8.0
        base.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(
                xyz=(0.282 * math.cos(angle), 0.282 * math.sin(angle), 0.058)
            ),
            material=machined_aluminum,
            name=f"base_bolt_{index}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.235, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=graphite,
        name="rotating_skirt",
    )
    turntable.visual(
        Cylinder(radius=0.305, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=machined_aluminum,
        name="machined_turntable",
    )
    turntable.visual(
        Cylinder(radius=0.215, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_hardcoat,
        name="face_groove",
    )
    turntable.visual(
        Cylinder(radius=0.118, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=cast_gray,
        name="central_boss",
    )
    turntable.visual(
        Box((0.430, 0.470, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=cast_gray,
        name="fork_foot",
    )
    for index in range(10):
        angle = 2.0 * math.pi * index / 10.0
        turntable.visual(
            Cylinder(radius=0.009, length=0.009),
            origin=Origin(
                xyz=(0.242 * math.cos(angle), 0.242 * math.sin(angle), 0.092)
            ),
            material=dark_hardcoat,
            name=f"turntable_bolt_{index}",
        )

    for side, (y, inner_bearing_name, outer_cap_name) in enumerate(
        (
            (-0.205, "inner_bearing_0", "outer_cap_0"),
            (0.205, "inner_bearing_1", "outer_cap_1"),
        )
    ):
        turntable.visual(
            Box((0.205, 0.074, 0.430)),
            origin=Origin(xyz=(0.0, y, 0.335)),
            material=cast_gray,
            name=f"fork_cheek_{side}",
        )
        turntable.visual(
            Box((0.290, 0.092, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.156)),
            material=cast_gray,
            name=f"cheek_foot_{side}",
        )
        turntable.visual(
            Cylinder(radius=0.103, length=0.078),
            origin=Origin(xyz=(0.0, y, 0.552), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=cast_gray,
            name=f"rounded_cheek_{side}",
        )
        turntable.visual(
            Cylinder(radius=0.061, length=0.018),
            origin=Origin(xyz=(0.0, y * 0.825, 0.552), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=inner_bearing_name,
        )
        turntable.visual(
            Cylinder(radius=0.074, length=0.020),
            origin=Origin(xyz=(0.0, y * 1.145, 0.552), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_hardcoat,
            name=outer_cap_name,
        )
        turntable.visual(
            Box((0.052, 0.082, 0.310)),
            origin=Origin(xyz=(-0.103, y, 0.295), rpy=(0.0, -0.38, 0.0)),
            material=cast_gray,
            name=f"rear_gusset_{side}",
        )
        turntable.visual(
            Box((0.052, 0.082, 0.310)),
            origin=Origin(xyz=(0.103, y, 0.295), rpy=(0.0, 0.38, 0.0)),
            material=cast_gray,
            name=f"front_gusset_{side}",
        )

    sensor_frame = model.part("sensor_frame")
    sensor_frame.visual(
        Box((0.370, 0.180, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=dark_hardcoat,
        name="top_rail",
    )
    sensor_frame.visual(
        Box((0.370, 0.180, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=dark_hardcoat,
        name="bottom_rail",
    )
    sensor_frame.visual(
        Box((0.045, 0.180, 0.310)),
        origin=Origin(xyz=(-0.1625, 0.0, 0.0)),
        material=dark_hardcoat,
        name="side_rail_0",
    )
    sensor_frame.visual(
        Box((0.045, 0.180, 0.310)),
        origin=Origin(xyz=(0.1625, 0.0, 0.0)),
        material=dark_hardcoat,
        name="side_rail_1",
    )
    sensor_frame.visual(
        Box((0.210, 0.140, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="sensor_body",
    )
    for side, x in enumerate((-0.126, 0.126)):
        sensor_frame.visual(
            Box((0.052, 0.144, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=graphite,
            name=f"body_lug_{side}",
        )
    sensor_frame.visual(
        Box((0.166, 0.006, 0.088)),
        origin=Origin(xyz=(0.0, 0.073, 0.0)),
        material=glass,
        name="sensor_window",
    )
    sensor_frame.visual(
        Cylinder(radius=0.030, length=0.286),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_aluminum,
        name="trunnion_shaft",
    )
    for side, (y, trunnion_cap_name, hub_boss_name) in enumerate(
        (
            (-0.151125, "trunnion_cap_0", "hub_boss_0"),
            (0.151125, "trunnion_cap_1", "hub_boss_1"),
        )
    ):
        sensor_frame.visual(
            Cylinder(radius=0.052, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=machined_aluminum,
            name=trunnion_cap_name,
        )
        sensor_frame.visual(
            Cylinder(radius=0.036, length=0.020),
            origin=Origin(xyz=(0.0, y * 0.86, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=hub_boss_name,
        )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.3, lower=-2.75, upper=2.75),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=sensor_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.552)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.1, lower=-0.78, upper=0.88),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    sensor_frame = object_model.get_part("sensor_frame")
    yaw = object_model.get_articulation("yaw_axis")
    pitch = object_model.get_articulation("pitch_axis")

    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        positive_elem="rotating_skirt",
        negative_elem="azimuth_seal",
        max_gap=0.003,
        max_penetration=0.000001,
        name="turntable rides just above seal",
    )
    ctx.expect_gap(
        sensor_frame,
        turntable,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="machined_turntable",
        min_gap=0.250,
        name="pitched frame is nested above base",
    )
    ctx.expect_origin_gap(
        sensor_frame,
        turntable,
        axis="z",
        min_gap=0.54,
        max_gap=0.57,
        name="trunnion axis sits high on fork",
    )
    ctx.expect_contact(
        sensor_frame,
        turntable,
        elem_a="trunnion_cap_0",
        elem_b="inner_bearing_0",
        contact_tol=0.0005,
        name="port trunnion cap seats in bearing",
    )
    ctx.expect_contact(
        sensor_frame,
        turntable,
        elem_a="trunnion_cap_1",
        elem_b="inner_bearing_1",
        contact_tol=0.0005,
        name="starboard trunnion cap seats in bearing",
    )

    closed_aabb = ctx.part_world_aabb(sensor_frame)
    with ctx.pose({pitch: -0.78}):
        low_aabb = ctx.part_world_aabb(sensor_frame)
        ctx.expect_gap(
            sensor_frame,
            turntable,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="machined_turntable",
            min_gap=0.18,
            name="down pitch clears turntable face",
        )
    with ctx.pose({pitch: 0.88, yaw: 1.2}):
        high_aabb = ctx.part_world_aabb(sensor_frame)
        ctx.expect_gap(
            sensor_frame,
            turntable,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="machined_turntable",
            min_gap=0.18,
            name="up pitch clears turntable face",
        )

    def _span_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1], aabb[1][1])

    sensor_y_span = _span_y(closed_aabb)
    ctx.check(
        "sensor frame remains between fork cheeks",
        sensor_y_span is not None and sensor_y_span[0] > -0.162 and sensor_y_span[1] < 0.162,
        details=f"sensor_y_span={sensor_y_span}",
    )
    ctx.check(
        "pitch joint changes frame attitude",
        closed_aabb is not None
        and low_aabb is not None
        and high_aabb is not None
        and (low_aabb[1][0] - low_aabb[0][0]) != (high_aabb[1][0] - high_aabb[0][0]),
        details=f"closed={closed_aabb}, low={low_aabb}, high={high_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
