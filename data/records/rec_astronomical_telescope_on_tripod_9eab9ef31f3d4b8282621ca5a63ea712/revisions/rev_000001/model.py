from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _annular_x(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """A centered annular cylinder whose axis is the local X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="maksutov_alt_az_telescope")

    satin_black = model.material("satin_black", rgba=(0.01, 0.012, 0.014, 1.0))
    graphite = model.material("graphite_mount", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.21, 0.22, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.025, 0.025, 0.023, 1.0))
    white = model.material("warm_white_tube", rgba=(0.86, 0.86, 0.80, 1.0))
    glass = model.material("blue_coated_glass", rgba=(0.20, 0.38, 0.55, 0.58))
    mirror = model.material("black_mirror_spot", rgba=(0.005, 0.007, 0.010, 1.0))
    metal = model.material("machined_metal", rgba=(0.55, 0.56, 0.55, 1.0))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.045, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=dark_gray,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.13, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=graphite,
        name="tripod_collar",
    )
    tripod.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        material=graphite,
        name="az_bearing_base",
    )
    tripod.visual(
        Cylinder(radius=0.075, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=graphite,
        name="head_neck",
    )

    leg_top_z = 0.72
    leg_foot_z = 0.08
    leg_radius = 0.58
    leg_len = math.sqrt(leg_radius**2 + (leg_top_z - leg_foot_z) ** 2)
    leg_pitch = math.asin((leg_top_z - leg_foot_z) / leg_len)
    for idx, yaw in enumerate((math.radians(90), math.radians(210), math.radians(330))):
        foot_x = leg_radius * math.cos(yaw)
        foot_y = leg_radius * math.sin(yaw)
        center = (foot_x / 2.0, foot_y / 2.0, (leg_top_z + leg_foot_z) / 2.0)
        tripod.visual(
            Box((leg_len, 0.045, 0.035)),
            origin=Origin(xyz=center, rpy=(0.0, leg_pitch, yaw)),
            material=dark_gray,
            name=f"leg_{idx}",
        )
        tripod.visual(
            Box((0.16, 0.075, 0.080)),
            origin=Origin(xyz=(foot_x, foot_y, 0.045), rpy=(0.0, 0.0, yaw)),
            material=rubber,
            name=f"foot_{idx}",
        )
    for idx, yaw in enumerate((math.radians(30), math.radians(150), math.radians(270))):
        tripod.visual(
            Box((0.38, 0.025, 0.020)),
            origin=Origin(
                xyz=(0.19 * math.cos(yaw), 0.19 * math.sin(yaw), 0.36),
                rpy=(0.0, 0.0, yaw),
            ),
            material=graphite,
            name=f"spreader_{idx}",
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.145, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=graphite,
        name="az_turntable",
    )
    arm.visual(
        Cylinder(radius=0.115, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_gray,
        name="motor_dome",
    )
    arm.visual(
        Box((0.18, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, -0.17, 0.105)),
        material=graphite,
        name="arm_foot",
    )
    arm.visual(
        Box((0.135, 0.105, 0.44)),
        origin=Origin(xyz=(0.0, -0.30, 0.285)),
        material=graphite,
        name="upright",
    )
    arm.visual(
        Cylinder(radius=0.090, length=0.140),
        origin=Origin(xyz=(0.0, -0.30, 0.58), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="alt_bearing",
    )
    arm.visual(
        Box((0.15, 0.11, 0.08)),
        origin=Origin(xyz=(0.0, -0.30, 0.475)),
        material=graphite,
        name="bearing_boss",
    )

    tube = model.part("tube")
    tube_shell = mesh_from_cadquery(_annular_x(0.145, 0.125, 0.44), "maksutov_tube_shell")
    tube.visual(
        tube_shell,
        origin=Origin(xyz=(0.0, 0.30, 0.0)),
        material=white,
        name="tube_shell",
    )
    front_cell = mesh_from_cadquery(_annular_x(0.154, 0.112, 0.045), "front_corrector_cell")
    tube.visual(
        front_cell,
        origin=Origin(xyz=(0.222, 0.30, 0.0)),
        material=satin_black,
        name="front_cell",
    )
    rear_cell = mesh_from_cadquery(_annular_x(0.148, 0.050, 0.055), "rear_visual_cell")
    tube.visual(
        rear_cell,
        origin=Origin(xyz=(-0.238, 0.30, 0.0)),
        material=satin_black,
        name="rear_cell",
    )
    cx, co = _cyl_x(0.118, 0.010)
    tube.visual(
        cx,
        origin=Origin(xyz=(0.246, 0.30, 0.0), rpy=co.rpy),
        material=glass,
        name="corrector_lens",
    )
    tube.visual(
        Cylinder(radius=0.035, length=0.006),
        origin=Origin(xyz=(0.252, 0.30, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mirror,
        name="secondary_spot",
    )
    tube.visual(
        Box((0.17, 0.06, 0.09)),
        origin=Origin(xyz=(-0.292, 0.30, 0.0)),
        material=satin_black,
        name="rear_back",
    )
    tube.visual(
        Cylinder(radius=0.030, length=0.095),
        origin=Origin(xyz=(-0.345, 0.30, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="eyepiece_tube",
    )
    tube.visual(
        Cylinder(radius=0.022, length=0.055),
        origin=Origin(xyz=(-0.403, 0.30, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="eyepiece",
    )
    tube.visual(
        Cylinder(radius=0.055, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="alt_shaft",
    )
    tube.visual(
        Box((0.21, 0.115, 0.110)),
        origin=Origin(xyz=(0.0, 0.135, 0.0)),
        material=satin_black,
        name="side_saddle",
    )
    tube.visual(
        Box((0.33, 0.045, 0.050)),
        origin=Origin(xyz=(0.0, 0.176, 0.0)),
        material=satin_black,
        name="dovetail_bar",
    )
    finder_geom = mesh_from_cadquery(_annular_x(0.033, 0.024, 0.25), "finder_scope_tube")
    tube.visual(
        finder_geom,
        origin=Origin(xyz=(0.010, 0.30, 0.205)),
        material=satin_black,
        name="finder_scope",
    )
    tube.visual(
        Box((0.038, 0.035, 0.085)),
        origin=Origin(xyz=(-0.080, 0.30, 0.157)),
        material=satin_black,
        name="finder_stem_0",
    )
    tube.visual(
        Box((0.038, 0.035, 0.085)),
        origin=Origin(xyz=(0.100, 0.30, 0.157)),
        material=satin_black,
        name="finder_stem_1",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.960)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5),
    )
    model.articulation(
        "altitude",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=tube,
        origin=Origin(xyz=(0.0, -0.30, 0.58)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.30, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    arm = object_model.get_part("arm")
    tube = object_model.get_part("tube")
    azimuth = object_model.get_articulation("azimuth")
    altitude = object_model.get_articulation("altitude")

    ctx.allow_overlap(
        arm,
        tube,
        elem_a="alt_bearing",
        elem_b="alt_shaft",
        reason="The rotating altitude shaft is intentionally captured inside the fixed bearing at the top of the single arm.",
    )

    ctx.expect_gap(
        arm,
        tripod,
        axis="z",
        positive_elem="az_turntable",
        negative_elem="az_bearing_base",
        max_gap=0.001,
        max_penetration=1e-5,
        name="azimuth turntable sits on the bearing base",
    )
    ctx.expect_overlap(
        arm,
        tripod,
        axes="xy",
        elem_a="az_turntable",
        elem_b="az_bearing_base",
        min_overlap=0.12,
        name="azimuth bearing footprints align",
    )
    ctx.expect_within(
        tube,
        arm,
        axes="xz",
        inner_elem="alt_shaft",
        outer_elem="alt_bearing",
        margin=0.002,
        name="altitude shaft is centered inside bearing",
    )
    ctx.expect_overlap(
        tube,
        arm,
        axes="y",
        elem_a="alt_shaft",
        elem_b="alt_bearing",
        min_overlap=0.12,
        name="altitude shaft remains captured in bearing",
    )

    front_rest = ctx.part_element_world_aabb(tube, elem="corrector_lens")
    with ctx.pose({altitude: 0.8}):
        front_raised = ctx.part_element_world_aabb(tube, elem="corrector_lens")
    ctx.check(
        "altitude raises the front corrector",
        front_rest is not None
        and front_raised is not None
        and ((front_raised[0][2] + front_raised[1][2]) / 2.0)
        > ((front_rest[0][2] + front_rest[1][2]) / 2.0) + 0.12,
        details=f"rest={front_rest}, raised={front_raised}",
    )

    arm_rest = ctx.part_world_position(arm)
    with ctx.pose({azimuth: math.pi / 2.0}):
        arm_rotated = ctx.part_world_position(arm)
        ctx.expect_gap(
            arm,
            tripod,
            axis="z",
            positive_elem="az_turntable",
            negative_elem="az_bearing_base",
            max_gap=0.001,
            max_penetration=1e-5,
            name="azimuth bearing stays seated while rotating",
        )
    ctx.check(
        "azimuth joint keeps the rotating arm on the vertical bearing axis",
        arm_rest is not None
        and arm_rotated is not None
        and abs(arm_rest[0] - arm_rotated[0]) < 1e-6
        and abs(arm_rest[1] - arm_rotated[1]) < 1e-6,
        details=f"rest={arm_rest}, rotated={arm_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
