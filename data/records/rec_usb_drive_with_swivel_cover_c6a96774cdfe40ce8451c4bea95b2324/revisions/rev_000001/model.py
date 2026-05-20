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


BODY_LENGTH = 0.046
BODY_WIDTH = 0.018
BODY_THICKNESS = 0.0072
PIVOT_X = -0.0175
CONNECTOR_LENGTH = 0.014
CONNECTOR_WIDTH = 0.0128
CONNECTOR_HEIGHT = 0.0048
CONNECTOR_X = BODY_LENGTH / 2 + CONNECTOR_LENGTH / 2 - 0.0005

COVER_MIN_X = -0.055
COVER_MAX_X = 0.006
COVER_LENGTH = COVER_MAX_X - COVER_MIN_X
COVER_CENTER_X = (COVER_MIN_X + COVER_MAX_X) / 2
COVER_WIDTH = 0.0224
COVER_TOP_Z = 0.00525
COVER_TOP_THICKNESS = 0.0012
COVER_FLANGE_HEIGHT = 0.0062
PIN_CLEARANCE_RADIUS = 0.00205


def _rounded_body_shell() -> object:
    return (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_THICKNESS)
        .edges("|Z")
        .fillet(0.0026)
    )


def _connector_shell() -> object:
    outer = cq.Workplane("XY").box(
        CONNECTOR_LENGTH,
        CONNECTOR_WIDTH,
        CONNECTOR_HEIGHT,
    )
    inner = cq.Workplane("XY").box(
        CONNECTOR_LENGTH + 0.002,
        CONNECTOR_WIDTH - 0.0017,
        CONNECTOR_HEIGHT - 0.00155,
    )
    return outer.cut(inner)


def _cover_shell() -> object:
    top_plate = cq.Workplane("XY").box(
        COVER_LENGTH,
        COVER_WIDTH,
        COVER_TOP_THICKNESS,
    ).translate((COVER_CENTER_X, 0.0, COVER_TOP_Z))

    flange_z = COVER_TOP_Z - COVER_TOP_THICKNESS / 2 - COVER_FLANGE_HEIGHT / 2
    side_flange_a = cq.Workplane("XY").box(
        COVER_LENGTH,
        0.00115,
        COVER_FLANGE_HEIGHT,
    ).translate((COVER_CENTER_X, COVER_WIDTH / 2 - 0.000575, flange_z))
    side_flange_b = cq.Workplane("XY").box(
        COVER_LENGTH,
        0.00115,
        COVER_FLANGE_HEIGHT,
    ).translate((COVER_CENTER_X, -COVER_WIDTH / 2 + 0.000575, flange_z))
    pivot_pad = (
        cq.Workplane("XY")
        .circle(0.0059)
        .extrude(COVER_TOP_THICKNESS)
        .translate((0.0, 0.0, COVER_TOP_Z - COVER_TOP_THICKNESS / 2))
    )

    shell = top_plate.union(side_flange_a).union(side_flange_b).union(pivot_pad)
    pin_clearance = (
        cq.Workplane("XY")
        .circle(PIN_CLEARANCE_RADIUS)
        .extrude(0.018)
        .translate((0.0, 0.0, -0.006))
    )
    return shell.cut(pin_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_swivel_usb_drive")

    matte_graphite = model.material("matte_graphite", rgba=(0.055, 0.057, 0.060, 1.0))
    soft_black = model.material("soft_black", rgba=(0.004, 0.004, 0.004, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.64, 0.66, 0.68, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_seam = model.material("dark_seam", rgba=(0.012, 0.013, 0.014, 1.0))
    contact_gold = model.material("contact_gold", rgba=(0.95, 0.68, 0.23, 1.0))

    drive = model.part("drive")
    drive.visual(
        mesh_from_cadquery(_rounded_body_shell(), "rounded_drive_body", tolerance=0.00035),
        material=matte_graphite,
        name="body_shell",
    )
    drive.visual(
        Box((0.031, 0.0105, 0.00018)),
        origin=Origin(xyz=(-0.0015, 0.0, BODY_THICKNESS / 2 + 0.00007)),
        material=soft_black,
        name="top_inlay",
    )
    drive.visual(
        Box((0.00042, BODY_WIDTH + 0.00035, 0.00022)),
        origin=Origin(xyz=(-0.009, 0.0, BODY_THICKNESS / 2 + 0.00005)),
        material=dark_seam,
        name="rear_seam",
    )
    drive.visual(
        Box((0.00042, BODY_WIDTH + 0.00035, 0.00022)),
        origin=Origin(xyz=(0.0145, 0.0, BODY_THICKNESS / 2 + 0.00005)),
        material=dark_seam,
        name="front_seam",
    )
    drive.visual(
        Box((0.0032, BODY_WIDTH + 0.001, BODY_THICKNESS + 0.0004)),
        origin=Origin(xyz=(BODY_LENGTH / 2 - 0.00035, 0.0, 0.0)),
        material=satin_steel,
        name="front_collar",
    )
    drive.visual(
        Box((0.0058, 0.0064, 0.00016)),
        origin=Origin(xyz=(-BODY_LENGTH / 2 + 0.0046, 0.0, BODY_THICKNESS / 2 + 0.00013)),
        material=dark_seam,
        name="lanyard_slot",
    )

    drive.visual(
        mesh_from_cadquery(_connector_shell(), "usb_type_a_shell", tolerance=0.00022),
        origin=Origin(xyz=(CONNECTOR_X, 0.0, 0.0)),
        material=satin_steel,
        name="connector_shell",
    )
    drive.visual(
        Box((0.0142, 0.0072, 0.00105)),
        origin=Origin(xyz=(CONNECTOR_X, 0.0, -0.00055)),
        material=soft_black,
        name="connector_tongue",
    )
    for index, y in enumerate((-0.0027, -0.0009, 0.0009, 0.0027)):
        drive.visual(
            Box((0.0042, 0.00095, 0.00016)),
            origin=Origin(xyz=(CONNECTOR_X + 0.0024, y, 0.000035)),
            material=contact_gold,
            name=f"contact_{index}",
        )
    for index, y in enumerate((-0.0031, 0.0031)):
        drive.visual(
            Box((0.0030, 0.00155, 0.00016)),
            origin=Origin(xyz=(CONNECTOR_X - 0.0006, y, CONNECTOR_HEIGHT / 2 + 0.00004)),
            material=dark_seam,
            name=f"shell_aperture_{index}",
        )

    drive.visual(
        Cylinder(radius=0.00125, length=0.0107),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.00065)),
        material=brushed_steel,
        name="pin_shaft",
    )
    drive.visual(
        Cylinder(radius=0.0024, length=0.0008),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.00625)),
        material=brushed_steel,
        name="pin_head",
    )
    drive.visual(
        Cylinder(radius=0.00225, length=0.0007),
        origin=Origin(xyz=(PIVOT_X, 0.0, -0.00495)),
        material=brushed_steel,
        name="pin_tail",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_shell(), "swivel_metal_cover", tolerance=0.00028),
        material=satin_steel,
        name="cover_shell",
    )
    cover.visual(
        Box((0.041, 0.0048, 0.00012)),
        origin=Origin(xyz=(-0.030, 0.0, COVER_TOP_Z + COVER_TOP_THICKNESS / 2 + 0.00006)),
        material=brushed_steel,
        name="brushed_center",
    )
    cover.visual(
        Box((0.043, 0.0140, 0.00016)),
        origin=Origin(xyz=(-0.0285, 0.0, COVER_TOP_Z - COVER_TOP_THICKNESS / 2 - 0.00008)),
        material=dark_seam,
        name="inner_liner",
    )

    model.articulation(
        "drive_to_cover",
        ArticulationType.REVOLUTE,
        parent=drive,
        child=cover,
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=4.0, lower=0.0, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drive = object_model.get_part("drive")
    cover = object_model.get_part("cover")
    hinge = object_model.get_articulation("drive_to_cover")

    ctx.expect_overlap(
        drive,
        cover,
        axes="xy",
        elem_a="pin_shaft",
        elem_b="cover_shell",
        min_overlap=0.0015,
        name="cover bearing is centered on the side pin",
    )
    ctx.expect_gap(
        drive,
        cover,
        axis="z",
        positive_elem="pin_head",
        negative_elem="cover_shell",
        min_gap=0.0,
        max_gap=0.00008,
        name="pin head retains cover with a nearly flush controlled seam",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            drive,
            cover,
            axis="x",
            positive_elem="connector_shell",
            negative_elem="cover_shell",
            min_gap=0.030,
            name="opened swivel cover leaves connector exposed",
        )

    with ctx.pose({hinge: math.pi}):
        ctx.expect_overlap(
            cover,
            drive,
            axes="xy",
            elem_a="cover_shell",
            elem_b="connector_shell",
            min_overlap=0.010,
            name="rotated cover sweeps over the connector footprint",
        )

    return ctx.report()


object_model = build_object_model()
