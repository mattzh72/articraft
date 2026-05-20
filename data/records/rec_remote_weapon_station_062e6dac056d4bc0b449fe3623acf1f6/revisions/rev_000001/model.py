from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_remote_weapon_station")

    matte_olive = model.material("matte_olive", rgba=(0.25, 0.29, 0.22, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.16, 0.19, 0.15, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.10, 0.11, 0.12, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    glass = model.material("smoked_sensor_glass", rgba=(0.05, 0.10, 0.13, 0.85))

    # Static low pedestal: a compact armored plinth with a bearing surface on top.
    base = model.part("pedestal")
    base.visual(
        Box((0.72, 0.50, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=dark_olive,
        name="base_plate",
    )
    base.visual(
        Box((0.44, 0.34, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=matte_olive,
        name="armored_plinth",
    )
    base.visual(
        Cylinder(radius=0.205, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=gunmetal,
        name="fixed_bearing",
    )

    # Rotating upper station.  The yaw joint frame is on the top of the fixed
    # bearing; all geometry is authored relative to that azimuth axis.
    yaw = model.part("yaw_ring")
    yaw_ring_shape = cq.Workplane("XY").circle(0.235).circle(0.160).extrude(0.024)
    yaw.visual(
        mesh_from_cadquery(yaw_ring_shape, "yaw_bearing_ring"),
        material=gunmetal,
        name="yaw_bearing_ring",
    )
    yaw.visual(
        Box((0.40, 0.32, 0.060)),
        origin=Origin(xyz=(0.020, 0.0, 0.054)),
        material=matte_olive,
        name="upper_deck",
    )
    for i, (x, y) in enumerate(((0.17, 0.12), (0.17, -0.12), (-0.13, 0.12), (-0.13, -0.12))):
        yaw.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(x, y, 0.028)),
            material=black,
            name=f"ring_bolt_{i}",
        )

    # Side cheeks form the elevation cradle support.  The cheeks are connected
    # through the deck and carry trunnion bosses on their outside faces.
    cheek_plate_shape = (
        cq.Workplane("XY")
        .box(0.320, 0.045, 0.200)
        .cut(
            cq.Workplane("XZ")
            .center(-0.040, 0.0)
            .circle(0.043)
            .extrude(0.080, both=True)
        )
    )
    yaw.visual(
        mesh_from_cadquery(cheek_plate_shape, "side_cheek_0"),
        origin=Origin(xyz=(0.040, 0.170, 0.180)),
        material=matte_olive,
        name="side_cheek_0",
    )
    yaw.visual(
        Cylinder(radius=0.062, length=0.060),
        origin=Origin(xyz=(0.0, 0.195, 0.180), rpy=(math.pi / 2, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_boss_0",
    )
    yaw.visual(
        mesh_from_cadquery(cheek_plate_shape, "side_cheek_1"),
        origin=Origin(xyz=(0.040, -0.170, 0.180)),
        material=matte_olive,
        name="side_cheek_1",
    )
    yaw.visual(
        Cylinder(radius=0.062, length=0.060),
        origin=Origin(xyz=(0.0, -0.195, 0.180), rpy=(math.pi / 2, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_boss_1",
    )

    # Short side bracket for the independent sensor pod.  The top and bottom
    # jaws leave a visible slot around the pod's small tilt axle.
    yaw.visual(
        Box((0.140, 0.100, 0.035)),
        origin=Origin(xyz=(-0.100, 0.246, 0.160)),
        material=matte_olive,
        name="sensor_bracket_arm",
    )
    yaw.visual(
        Box((0.020, 0.077, 0.114)),
        origin=Origin(xyz=(-0.150, 0.3335, 0.160)),
        material=matte_olive,
        name="sensor_clip_web_0",
    )
    yaw.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(-0.100, 0.300, 0.160), rpy=(math.pi / 2, 0.0, 0.0)),
        material=gunmetal,
        name="sensor_axle_bushing",
    )
    yaw.visual(
        Box((0.085, 0.074, 0.024)),
        origin=Origin(xyz=(-0.100, 0.335, 0.205)),
        material=matte_olive,
        name="sensor_clip_top",
    )
    yaw.visual(
        Box((0.085, 0.074, 0.024)),
        origin=Origin(xyz=(-0.100, 0.335, 0.115)),
        material=matte_olive,
        name="sensor_clip_bottom",
    )

    weapon = model.part("weapon_cradle")
    weapon.visual(
        Cylinder(radius=0.030, length=0.365),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_axle",
    )
    weapon.visual(
        Box((0.250, 0.130, 0.095)),
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        material=gunmetal,
        name="receiver",
    )
    weapon.visual(
        Box((0.120, 0.110, 0.070)),
        origin=Origin(xyz=(0.000, 0.0, -0.006)),
        material=gunmetal,
        name="rear_mount_block",
    )
    weapon.visual(
        Cylinder(radius=0.023, length=0.430),
        origin=Origin(xyz=(0.420, 0.0, 0.004), rpy=(0.0, math.pi / 2, 0.0)),
        material=black,
        name="barrel",
    )
    weapon.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.660, 0.0, 0.004), rpy=(0.0, math.pi / 2, 0.0)),
        material=gunmetal,
        name="muzzle_brake",
    )
    weapon.visual(
        Box((0.105, 0.095, 0.025)),
        origin=Origin(xyz=(0.170, 0.0, 0.060)),
        material=black,
        name="top_cover",
    )

    sensor = model.part("sensor_pod")
    sensor.visual(
        Cylinder(radius=0.020, length=0.075),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=gunmetal,
        name="pod_tilt_axle",
    )
    sensor.visual(
        Box((0.080, 0.050, 0.038)),
        origin=Origin(xyz=(0.040, 0.043, 0.0)),
        material=gunmetal,
        name="pod_neck",
    )
    sensor.visual(
        Box((0.165, 0.090, 0.095)),
        origin=Origin(xyz=(0.118, 0.095, 0.0)),
        material=dark_olive,
        name="pod_housing",
    )
    sensor.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.205, 0.095, 0.010), rpy=(0.0, math.pi / 2, 0.0)),
        material=glass,
        name="main_optic",
    )
    sensor.visual(
        Cylinder(radius=0.014, length=0.011),
        origin=Origin(xyz=(0.205, 0.067, -0.025), rpy=(0.0, math.pi / 2, 0.0)),
        material=glass,
        name="secondary_optic",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yaw,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=yaw,
        child=weapon,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.28, upper=0.62, effort=80.0, velocity=0.9),
    )
    model.articulation(
        "sensor_tilt",
        ArticulationType.REVOLUTE,
        parent=yaw,
        child=sensor,
        origin=Origin(xyz=(-0.100, 0.335, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=0.55, effort=12.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    yaw = object_model.get_part("yaw_ring")
    weapon = object_model.get_part("weapon_cradle")
    sensor = object_model.get_part("sensor_pod")
    elevation = object_model.get_articulation("elevation")
    sensor_tilt = object_model.get_articulation("sensor_tilt")

    # The weapon trunnion axle is deliberately captured in the two cheek bosses.
    for boss_name in ("trunnion_boss_0", "trunnion_boss_1"):
        ctx.allow_overlap(
            yaw,
            weapon,
            elem_a=boss_name,
            elem_b="trunnion_axle",
            reason="The elevation axle is intentionally seated through the cheek trunnion boss.",
        )
        ctx.expect_overlap(
            yaw,
            weapon,
            axes="xyz",
            elem_a=boss_name,
            elem_b="trunnion_axle",
            min_overlap=0.012,
            name=f"{boss_name} captures elevation axle",
        )

    ctx.allow_overlap(
        yaw,
        sensor,
        elem_a="sensor_axle_bushing",
        elem_b="pod_tilt_axle",
        reason="The sensor pod tilt axle is intentionally captured in the side bracket bushing.",
    )
    ctx.expect_overlap(
        yaw,
        sensor,
        axes="xyz",
        elem_a="sensor_axle_bushing",
        elem_b="pod_tilt_axle",
        min_overlap=0.010,
        name="sensor bushing captures pod axle",
    )

    # The sensor axle must remain visibly clipped between the top and bottom jaws.
    ctx.expect_within(
        sensor,
        yaw,
        axes="y",
        inner_elem="pod_tilt_axle",
        outer_elem="sensor_clip_top",
        margin=0.002,
        name="sensor axle lies within clip width",
    )
    ctx.expect_gap(
        yaw,
        sensor,
        axis="z",
        positive_elem="sensor_clip_top",
        negative_elem="pod_tilt_axle",
        min_gap=0.004,
        max_gap=0.018,
        name="top clip clears sensor axle",
    )
    ctx.expect_gap(
        sensor,
        yaw,
        axis="z",
        positive_elem="pod_tilt_axle",
        negative_elem="sensor_clip_bottom",
        min_gap=0.004,
        max_gap=0.018,
        name="bottom clip clears sensor axle",
    )

    rest_weapon_pos = ctx.part_world_position(weapon)
    with ctx.pose({elevation: 0.55}):
        raised_weapon_pos = ctx.part_world_position(weapon)
        ctx.expect_overlap(
            yaw,
            weapon,
            axes="y",
            elem_a="trunnion_boss_0",
            elem_b="trunnion_axle",
            min_overlap=0.012,
            name="raised weapon stays on trunnions",
        )
    ctx.check(
        "weapon elevation joint preserves pivot",
        rest_weapon_pos is not None
        and raised_weapon_pos is not None
        and abs(rest_weapon_pos[2] - raised_weapon_pos[2]) < 0.002,
        details=f"rest={rest_weapon_pos}, raised={raised_weapon_pos}",
    )

    rest_sensor_pos = ctx.part_world_position(sensor)
    with ctx.pose({sensor_tilt: 0.45}):
        tilted_sensor_pos = ctx.part_world_position(sensor)
        ctx.expect_within(
            sensor,
            yaw,
            axes="y",
            inner_elem="pod_tilt_axle",
            outer_elem="sensor_clip_bottom",
            margin=0.002,
            name="tilted sensor axle remains clipped",
        )
    ctx.check(
        "sensor tilt joint preserves side bracket pivot",
        rest_sensor_pos is not None
        and tilted_sensor_pos is not None
        and abs(rest_sensor_pos[1] - tilted_sensor_pos[1]) < 0.002,
        details=f"rest={rest_sensor_pos}, tilted={tilted_sensor_pos}",
    )

    return ctx.report()


object_model = build_object_model()
