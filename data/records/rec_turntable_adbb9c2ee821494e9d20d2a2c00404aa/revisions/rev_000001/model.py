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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLINTH_WIDTH = 0.54
PLINTH_DEPTH = 0.38
PLINTH_BODY_HEIGHT = 0.055
PLINTH_VISUAL_Z = 0.040
PLINTH_TOP_Z = PLINTH_VISUAL_Z + PLINTH_BODY_HEIGHT / 2.0

PLATTER_CENTER = (-0.070, 0.030)
PLATTER_BOTTOM_Z = PLINTH_TOP_Z
TONEARM_BASE_XYZ = (0.185, -0.115, PLINTH_TOP_Z)
TONEARM_PIVOT_Z = 0.056


def _rounded_plinth() -> cq.Workplane:
    """Low, wide turntable plinth with softly rounded corners and attached feet."""

    body = (
        cq.Workplane("XY")
        .box(PLINTH_WIDTH, PLINTH_DEPTH, PLINTH_BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.026)
        .edges(">Z")
        .fillet(0.004)
    )

    foot_radius = 0.034
    foot_height = 0.012
    foot_z = -PLINTH_BODY_HEIGHT / 2.0 - foot_height + 0.003
    for x in (-0.210, 0.210):
        for y in (-0.135, 0.135):
            foot = (
                cq.Workplane("XY")
                .circle(foot_radius)
                .extrude(foot_height)
                .translate((x, y, foot_z))
            )
            body = body.union(foot)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = Material("dark_walnut", rgba=(0.18, 0.11, 0.055, 1.0))
    black = Material("satin_black", rgba=(0.010, 0.010, 0.012, 1.0))
    rubber = Material("matte_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    steel = Material("polished_steel", rgba=(0.86, 0.86, 0.82, 1.0))
    red_label = Material("red_record_label", rgba=(0.55, 0.06, 0.035, 1.0))
    cartridge_blue = Material("blue_cartridge", rgba=(0.05, 0.10, 0.24, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        mesh_from_cadquery(_rounded_plinth(), "rounded_plinth", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_VISUAL_Z)),
        material=walnut,
        name="plinth_shell",
    )
    plinth.visual(
        Box((0.130, 0.026, 0.004)),
        origin=Origin(xyz=(0.155, 0.125, PLINTH_TOP_Z + 0.002)),
        material=black,
        name="speed_inlay",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.152, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=aluminum,
        name="platter_rim",
    )
    platter.visual(
        Cylinder(radius=0.134, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=black,
        name="vinyl_record",
    )
    platter.visual(
        Cylinder(radius=0.036, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.03075)),
        material=red_label,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.004, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=steel,
        name="center_spindle",
    )

    tonearm_base = model.part("tonearm_base")
    tonearm_base.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=black,
        name="mounting_pad",
    )
    tonearm_base.visual(
        Cylinder(radius=0.027, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=aluminum,
        name="pivot_pedestal",
    )
    tonearm_base.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=steel,
        name="pivot_pin",
    )

    # At q=0 the arm points diagonally from the side-mounted pivot toward the
    # record surface, like a cued tonearm hovering near the outer grooves.
    vector_to_platter = (
        PLATTER_CENTER[0] - TONEARM_BASE_XYZ[0],
        PLATTER_CENTER[1] - TONEARM_BASE_XYZ[1],
    )
    yaw_to_record = math.atan2(-vector_to_platter[1], -vector_to_platter[0])

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="pivot_hub",
    )
    tonearm.visual(
        Cylinder(radius=0.004, length=0.220),
        origin=Origin(xyz=(-0.110, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.014, length=0.046),
        origin=Origin(xyz=(0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.044, 0.024, 0.006)),
        origin=Origin(xyz=(-0.232, 0.0, -0.003)),
        material=black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.019, 0.014, 0.012)),
        origin=Origin(xyz=(-0.240, 0.0, -0.012)),
        material=cartridge_blue,
        name="cartridge",
    )
    tonearm.visual(
        Sphere(radius=0.002),
        origin=Origin(xyz=(-0.247, 0.0, -0.0195)),
        material=steel,
        name="stylus_tip",
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(PLATTER_CENTER[0], PLATTER_CENTER[1], PLATTER_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "plinth_to_tonearm_base",
        ArticulationType.FIXED,
        parent=plinth,
        child=tonearm_base,
        origin=Origin(xyz=TONEARM_BASE_XYZ),
    )
    model.articulation(
        "tonearm_pivot",
        ArticulationType.REVOLUTE,
        parent=tonearm_base,
        child=tonearm,
        origin=Origin(xyz=(0.0, 0.0, TONEARM_PIVOT_Z), rpy=(0.0, 0.0, yaw_to_record)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm_base = object_model.get_part("tonearm_base")
    tonearm = object_model.get_part("tonearm")
    platter_joint = object_model.get_articulation("plinth_to_platter")
    arm_joint = object_model.get_articulation("tonearm_pivot")

    ctx.check(
        "platter joint is continuous and vertical",
        platter_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in platter_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={platter_joint.articulation_type}, axis={platter_joint.axis}",
    )
    ctx.check(
        "tonearm joint is limited revolute and vertical",
        arm_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in arm_joint.axis) == (0.0, 0.0, 1.0)
        and arm_joint.motion_limits is not None
        and arm_joint.motion_limits.lower is not None
        and arm_joint.motion_limits.upper is not None,
        details=f"type={arm_joint.articulation_type}, axis={arm_joint.axis}, limits={arm_joint.motion_limits}",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem="platter_rim",
        negative_elem="plinth_shell",
        name="platter rides on the low plinth",
    )
    ctx.expect_gap(
        tonearm_base,
        plinth,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem="mounting_pad",
        negative_elem="plinth_shell",
        name="tonearm base is mounted on plinth",
    )
    ctx.expect_gap(
        tonearm,
        tonearm_base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem="pivot_hub",
        negative_elem="pivot_pedestal",
        name="rotating tonearm hub sits on its pivot base",
    )

    rest_pos = ctx.part_world_position(tonearm)
    with ctx.pose({arm_joint: 0.40, platter_joint: math.pi}):
        swung_pos = ctx.part_world_position(tonearm)
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            min_gap=0.001,
            positive_elem="stylus_tip",
            negative_elem="vinyl_record",
            name="stylus remains just above record while swinging",
        )
    ctx.check(
        "tonearm pivot remains fixed while the arm swings",
        rest_pos is not None
        and swung_pos is not None
        and abs(rest_pos[0] - swung_pos[0]) < 1e-6
        and abs(rest_pos[1] - swung_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
