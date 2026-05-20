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


SLEEVE_HEIGHT = 0.44
SLEEVE_TOP_Z = 0.555
MAST_RETAINED_AT_REST = 0.36
MAST_VISIBLE_AT_REST = 0.66
MAST_TRAVEL = 0.24
YOKE_TRUNNION_Z = 0.135


def _lamp_can_shape():
    body_radius = 0.105
    bezel_radius = 0.114
    inner_radius = 0.091
    rear_radius = 0.095
    body_length = 0.200
    bezel_depth = 0.024
    rear_depth = 0.032
    back_thickness = 0.014
    can = cq.Workplane("XZ").circle(body_radius).extrude(body_length / 2.0, both=True)
    can = can.faces(">Y").workplane().circle(bezel_radius).extrude(bezel_depth)
    can = can.faces("<Y").workplane().circle(rear_radius).extrude(rear_depth)
    can = can.faces(">Y").workplane().circle(inner_radius).cutBlind(
        -(body_length + bezel_depth + rear_depth - back_thickness)
    )

    rear_box = cq.Workplane("XY").box(0.110, 0.040, 0.052).translate((0.0, -0.118, 0.055))

    return can.union(rear_box)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight")

    textured_black = model.material("textured_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.20, 0.20, 0.21, 1.0))
    graphite = model.material("graphite", rgba=(0.29, 0.30, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.59, 0.61, 0.63, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.76, 0.86, 0.94, 0.40))

    lamp_can_mesh = mesh_from_cadquery(_lamp_can_shape(), "lamp_can")

    base = model.part("base")
    base.visual(
        Box((0.440, 0.300, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=textured_black,
        name="base_plate",
    )
    base.visual(
        Box((0.180, 0.120, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=satin_black,
        name="base_riser",
    )
    base.visual(
        Box((0.020, 0.080, 0.443)),
        origin=Origin(xyz=(-0.046, 0.0, 0.3335)),
        material=textured_black,
        name="sleeve_left",
    )
    base.visual(
        Box((0.020, 0.080, 0.443)),
        origin=Origin(xyz=(0.046, 0.0, 0.3335)),
        material=textured_black,
        name="sleeve_right",
    )
    base.visual(
        Box((0.072, 0.016, 0.443)),
        origin=Origin(xyz=(0.0, 0.032, 0.3335)),
        material=textured_black,
        name="sleeve_front",
    )
    base.visual(
        Box((0.072, 0.016, 0.443)),
        origin=Origin(xyz=(0.0, -0.032, 0.3335)),
        material=textured_black,
        name="sleeve_rear",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.070, 0.0, 0.398), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="clamp_stem",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.098, 0.0, 0.398), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="clamp_knob",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.066, 0.042, 1.020)),
        origin=Origin(
            xyz=(0.0, 0.0, (MAST_VISIBLE_AT_REST - MAST_RETAINED_AT_REST) * 0.5),
        ),
        material=graphite,
        name="mast_tube",
    )
    mast.visual(
        Box((0.092, 0.068, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, MAST_VISIBLE_AT_REST + 0.0125)),
        material=satin_black,
        name="mast_cap",
    )
    mast.visual(
        Box((0.003, 0.020, 0.080)),
        origin=Origin(xyz=(-0.0345, 0.0, -0.300)),
        material=steel,
        name="guide_left",
    )
    mast.visual(
        Box((0.003, 0.020, 0.080)),
        origin=Origin(xyz=(0.0345, 0.0, -0.300)),
        material=steel,
        name="guide_right",
    )
    mast.visual(
        Box((0.018, 0.003, 0.080)),
        origin=Origin(xyz=(0.0, 0.0225, -0.300)),
        material=steel,
        name="guide_front",
    )
    mast.visual(
        Box((0.018, 0.003, 0.080)),
        origin=Origin(xyz=(0.0, -0.0225, -0.300)),
        material=steel,
        name="guide_rear",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.140, 0.070, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=textured_black,
        name="yoke_base",
    )
    yoke.visual(
        Box((0.252, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.040, 0.005)),
        material=textured_black,
        name="rear_bridge",
    )
    yoke.visual(
        Box((0.028, 0.090, 0.160)),
        origin=Origin(xyz=(-0.126, 0.0, 0.080)),
        material=textured_black,
        name="arm_left",
    )
    yoke.visual(
        Box((0.028, 0.090, 0.160)),
        origin=Origin(xyz=(0.126, 0.0, 0.080)),
        material=textured_black,
        name="arm_right",
    )
    yoke.visual(
        Cylinder(radius=0.052, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_black,
        name="pan_hub",
    )

    lamp = model.part("lamp")
    lamp.visual(lamp_can_mesh, material=satin_black, name="can_shell")
    lamp.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(-0.107, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="trunnion_left",
    )
    lamp.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.107, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="trunnion_right",
    )
    lamp.visual(
        Cylinder(radius=0.093, length=0.018),
        origin=Origin(xyz=(0.0, 0.1165, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )

    sleeve_slide = model.articulation(
        "sleeve_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=0.20,
            lower=0.0,
            upper=MAST_TRAVEL,
        ),
    )
    yoke_pan = model.articulation(
        "yoke_pan",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, MAST_VISIBLE_AT_REST + 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.20,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    lamp_tilt = model.articulation(
        "lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, YOKE_TRUNNION_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.50,
            lower=-math.radians(50.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    lamp = object_model.get_part("lamp")

    sleeve_slide = object_model.get_articulation("sleeve_slide")
    yoke_pan = object_model.get_articulation("yoke_pan")
    lamp_tilt = object_model.get_articulation("lamp_tilt")

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        margin=0.002,
        name="mast stays centered in sleeve at rest",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        min_overlap=0.200,
        name="mast retains insertion at rest",
    )

    rest_yoke_pos = ctx.part_world_position("yoke")
    with ctx.pose({sleeve_slide: MAST_TRAVEL}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            margin=0.002,
            name="mast stays centered when extended",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            min_overlap=0.110,
            name="mast retains insertion when extended",
        )
        extended_yoke_pos = ctx.part_world_position("yoke")

    ctx.check(
        "mast extends upward",
        rest_yoke_pos is not None
        and extended_yoke_pos is not None
        and extended_yoke_pos[2] > rest_yoke_pos[2] + 0.20,
        details=f"rest={rest_yoke_pos}, extended={extended_yoke_pos}",
    )

    rest_lens = _aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    with ctx.pose({yoke_pan: 1.0}):
        panned_lens = _aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "yoke pan swings lamp around the stand axis",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[0] < rest_lens[0] - 0.07
        and panned_lens[1] < rest_lens[1] - 0.03,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    with ctx.pose({lamp_tilt: 0.7}):
        tilted_lens = _aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "lamp tilt raises the beam aim",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] > rest_lens[2] + 0.06
        and tilted_lens[1] < rest_lens[1] - 0.02,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
