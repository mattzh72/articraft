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
import cadquery as cq
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cctv_mast")

    base = model.part("base")
    base.visual(
        Box((0.8, 0.8, 0.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.1)),
        name="ballast",
    )
    
    # Outer pole is hollow to allow the inner pole to slide inside
    outer_pole_cq = (
        cq.Workplane("XY")
        .circle(0.08)
        .circle(0.06)
        .extrude(2.0)
    ).translate((0, 0, 0.2))
    
    base.visual(
        mesh_from_cadquery(outer_pole_cq, "outer_pole_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="outer_pole",
    )

    inner_pole = model.part("inner_pole")
    inner_pole.visual(
        Cylinder(radius=0.055, length=2.0),
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        name="inner_pole_tube",
    )

    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_pole,
        origin=Origin(xyz=(0.0, 0.0, 0.2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.5, lower=0.0, upper=1.8),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Box((0.15, 0.2, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="pan_base",
    )
    pan_head.visual(
        Box((0.12, 0.04, 0.25)),
        origin=Origin(xyz=(0.0, 0.08, 0.175)),
        name="pan_arm_left",
    )
    pan_head.visual(
        Box((0.12, 0.04, 0.25)),
        origin=Origin(xyz=(0.0, -0.08, 0.175)),
        name="pan_arm_right",
    )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=inner_pole,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.25, 0.12, 0.12)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        name="camera_body",
    )
    camera.visual(
        Cylinder(radius=0.04, length=0.08),
        origin=Origin(xyz=(0.215, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="lens",
    )

    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-math.pi/2, upper=math.pi/2),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    inner_pole = object_model.get_part("inner_pole")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")

    ctx.expect_within(
        inner_pole,
        base,
        axes="xy",
        inner_elem="inner_pole_tube",
        outer_elem="outer_pole",
        margin=0.005,
        name="inner pole stays centered in outer pole",
    )
    ctx.expect_overlap(
        inner_pole,
        base,
        axes="z",
        elem_a="inner_pole_tube",
        elem_b="outer_pole",
        min_overlap=0.1,
        name="collapsed inner pole remains inserted in outer pole",
    )

    mast_extension = object_model.get_articulation("mast_extension")

    with ctx.pose({mast_extension: 1.8}):
        ctx.expect_within(
            inner_pole,
            base,
            axes="xy",
            inner_elem="inner_pole_tube",
            outer_elem="outer_pole",
            margin=0.005,
            name="extended inner pole stays centered in outer pole",
        )
        ctx.expect_overlap(
            inner_pole,
            base,
            axes="z",
            elem_a="inner_pole_tube",
            elem_b="outer_pole",
            min_overlap=0.1,
            name="extended inner pole retains insertion in outer pole",
        )

    ctx.expect_gap(
        pan_head,
        inner_pole,
        axis="z",
        min_gap=0.0,
        max_penetration=0.0,
        positive_elem="pan_base",
        negative_elem="inner_pole_tube",
        name="pan head sits on inner pole",
    )

    ctx.expect_within(
        camera,
        pan_head,
        axes="y",
        inner_elem="camera_body",
        name="camera fits between pan arms",
    )

    return ctx.report()

object_model = build_object_model()
