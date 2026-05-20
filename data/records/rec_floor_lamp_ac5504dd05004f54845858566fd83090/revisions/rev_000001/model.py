from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

HUB_Z = 0.73
HUB_HEIGHT = 0.065
POST_RADIUS = 0.013
POST_TOP_Z = 1.44
YOKE_CENTER_Z = 1.35
HINGE_X = 0.11

LEG_TOP_RADIUS = 0.030
LEG_FOOT_RADIUS = 0.305
LEG_FOOT_Z = 0.035
LEG_RADIUS = 0.015

YOKE_ARM_LENGTH = 0.048
YOKE_ARM_THICKNESS = 0.018
YOKE_ARM_HEIGHT = 0.104
YOKE_ARM_OFFSET_Y = 0.074
YOKE_BRIDGE_X = 0.066
YOKE_BRIDGE_LENGTH = 0.028
YOKE_BRIDGE_HEIGHT = 0.044
YOKE_BORE_RADIUS = 0.010

HEAD_REAR_X = -0.018
HEAD_FRONT_X = 0.217
HEAD_OUTER_RADIUS_REAR = 0.040
HEAD_OUTER_RADIUS_MID = 0.052
HEAD_OUTER_RADIUS_FRONT = 0.073
TRUNNION_RADIUS = 0.0100
TRUNNION_LENGTH = 0.033
TRUNNION_START_Y = 0.041


def _solid_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Shape:
    return cq.Workplane("XY").box(*size).translate(center).val()


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
) -> cq.Shape:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    return cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(*start),
        cq.Vector(dx / length, dy / length, dz / length),
    )


def _largest_solid(shape: cq.Shape) -> cq.Shape:
    solids = shape.Solids() if hasattr(shape, "Solids") else []
    if not solids:
        return shape
    return max(solids, key=lambda solid: solid.Volume())


def _build_legs_shape() -> cq.Shape:
    legs = cq.Solid.makeCylinder(
        0.050,
        0.030,
        cq.Vector(0.0, 0.0, HUB_Z - 0.015),
        cq.Vector(0.0, 0.0, 1.0),
    )
    top_z = HUB_Z + 0.008
    for idx in range(3):
        angle = idx * (2.0 * math.pi / 3.0)
        start = (
            LEG_TOP_RADIUS * math.cos(angle),
            LEG_TOP_RADIUS * math.sin(angle),
            top_z,
        )
        end = (
            LEG_FOOT_RADIUS * math.cos(angle),
            LEG_FOOT_RADIUS * math.sin(angle),
            LEG_FOOT_Z,
        )
        leg = _cylinder_between(start, end, LEG_RADIUS)
        foot = cq.Solid.makeCylinder(
            LEG_RADIUS * 1.12,
            LEG_FOOT_Z + 0.012,
            cq.Vector(end[0], end[1], 0.0),
            cq.Vector(0.0, 0.0, 1.0),
        )
        leg = leg.fuse(foot)
        legs = legs.fuse(leg)

    return legs


def _build_frame_shape() -> cq.Shape:
    hub = cq.Solid.makeCylinder(
        0.024,
        HUB_HEIGHT,
        cq.Vector(0.0, 0.0, HUB_Z - HUB_HEIGHT / 2.0),
        cq.Vector(0.0, 0.0, 1.0),
    )
    post = cq.Solid.makeCylinder(
        POST_RADIUS,
        POST_TOP_Z - (HUB_Z + HUB_HEIGHT / 2.0),
        cq.Vector(0.0, 0.0, HUB_Z + HUB_HEIGHT / 2.0),
        cq.Vector(0.0, 0.0, 1.0),
    )
    top_collar = cq.Solid.makeCylinder(
        0.024,
        0.032,
        cq.Vector(0.0, 0.0, YOKE_CENTER_Z - 0.016),
        cq.Vector(0.0, 0.0, 1.0),
    )
    finial = cq.Solid.makeCylinder(
        0.008,
        0.050,
        cq.Vector(0.0, 0.0, POST_TOP_Z - 0.010),
        cq.Vector(0.0, 0.0, 1.0),
    )
    support_arm = _solid_box((0.056, 0.042, 0.036), (0.028, 0.0, YOKE_CENTER_Z))
    support_gusset = _solid_box((0.030, 0.030, 0.055), (0.014, 0.0, YOKE_CENTER_Z - 0.022))

    return hub.fuse(post).fuse(top_collar).fuse(finial).fuse(support_arm).fuse(support_gusset)


def _build_yoke_shape() -> cq.Shape:
    bridge = _solid_box(
        (YOKE_BRIDGE_LENGTH, 0.154, YOKE_BRIDGE_HEIGHT),
        (YOKE_BRIDGE_X, 0.0, YOKE_CENTER_Z),
    )
    upper_strap = _solid_box((0.018, 0.080, 0.018), (0.044, 0.0, YOKE_CENTER_Z + 0.020))
    lower_strap = _solid_box((0.018, 0.080, 0.018), (0.044, 0.0, YOKE_CENTER_Z - 0.020))
    arm_0 = _solid_box(
        (YOKE_ARM_LENGTH, YOKE_ARM_THICKNESS, YOKE_ARM_HEIGHT),
        (HINGE_X - YOKE_ARM_LENGTH / 2.0 + 0.014, YOKE_ARM_OFFSET_Y, YOKE_CENTER_Z),
    )
    arm_1 = _solid_box(
        (YOKE_ARM_LENGTH, YOKE_ARM_THICKNESS, YOKE_ARM_HEIGHT),
        (HINGE_X - YOKE_ARM_LENGTH / 2.0 + 0.014, -YOKE_ARM_OFFSET_Y, YOKE_CENTER_Z),
    )

    yoke = bridge.fuse(upper_strap).fuse(lower_strap).fuse(arm_0).fuse(arm_1)
    bore = cq.Solid.makeCylinder(
        YOKE_BORE_RADIUS,
        0.220,
        cq.Vector(HINGE_X, -0.110, YOKE_CENTER_Z),
        cq.Vector(0.0, 1.0, 0.0),
    )
    return yoke.cut(bore)


def _build_head_shell_shape() -> cq.Shape:
    outer_barrel = cq.Solid.makeCylinder(
        0.052,
        0.115,
        cq.Vector(HEAD_REAR_X, 0.0, 0.0),
        cq.Vector(1.0, 0.0, 0.0),
    )
    outer_flare = cq.Solid.makeCone(
        0.052,
        0.073,
        0.108,
        cq.Vector(0.097, 0.0, 0.0),
        cq.Vector(1.0, 0.0, 0.0),
    )
    bezel = cq.Solid.makeCylinder(
        0.076,
        0.012,
        cq.Vector(0.205, 0.0, 0.0),
        cq.Vector(1.0, 0.0, 0.0),
    )
    shell = outer_barrel.fuse(outer_flare).fuse(bezel)

    trunnion_0 = cq.Solid.makeCylinder(
        TRUNNION_RADIUS,
        TRUNNION_LENGTH,
        cq.Vector(0.0, TRUNNION_START_Y, 0.0),
        cq.Vector(0.0, 1.0, 0.0),
    )
    trunnion_1 = cq.Solid.makeCylinder(
        TRUNNION_RADIUS,
        TRUNNION_LENGTH,
        cq.Vector(0.0, -(TRUNNION_START_Y + TRUNNION_LENGTH), 0.0),
        cq.Vector(0.0, 1.0, 0.0),
    )
    return shell.fuse(trunnion_0).fuse(trunnion_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_floor_spotlight")

    metal_black = model.material("metal_black", rgba=(0.12, 0.12, 0.13, 1.0))
    walnut = model.material("walnut", rgba=(0.40, 0.26, 0.16, 1.0))
    warm_cream = model.material("warm_cream", rgba=(0.87, 0.83, 0.75, 1.0))
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_legs_shape(), "legs"),
        material=walnut,
        name="legs",
    )
    base.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material=metal_black,
        name="frame",
    )
    base.visual(
        mesh_from_cadquery(_build_yoke_shape(), "yoke"),
        material=metal_black,
        name="yoke",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_build_head_shell_shape(), "head_shell"),
        material=warm_cream,
        name="shell",
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(HINGE_X, 0.0, YOKE_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=math.radians(-50.0),
            upper=math.radians(40.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    tilt = object_model.get_articulation("base_to_head")

    ctx.allow_overlap(
        base,
        head,
        elem_a="yoke",
        elem_b="shell",
        reason="The spotlight's side trunnions are fused into the shell visual and intentionally seat inside the yoke cheek bores at the tilt hinge.",
    )

    ctx.expect_within(
        head,
        base,
        axes="y",
        inner_elem="shell",
        outer_elem="yoke",
        margin=0.001,
        name="head shell stays between yoke arms",
    )
    ctx.expect_gap(
        head,
        base,
        axis="x",
        positive_elem="shell",
        negative_elem="frame",
        min_gap=0.03,
        name="spotlight shell projects forward of the central post",
    )

    lower = tilt.motion_limits.lower if tilt.motion_limits is not None else None
    upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None
    if lower is None or upper is None:
        ctx.fail("tilt limits are defined", "The spotlight tilt articulation is missing bounded motion limits.")
        return ctx.report()

    with ctx.pose({tilt: lower}):
        ctx.expect_gap(
            head,
            base,
            axis="x",
            positive_elem="shell",
            negative_elem="frame",
            min_gap=0.0,
            name="downward tilt keeps the spotlight shell clear of the post",
        )
        lower_lens = ctx.part_element_world_aabb(head, elem="shell")

    with ctx.pose({tilt: upper}):
        upper_lens = ctx.part_element_world_aabb(head, elem="shell")

    lower_center_z = None if lower_lens is None else 0.5 * (lower_lens[0][2] + lower_lens[1][2])
    upper_center_z = None if upper_lens is None else 0.5 * (upper_lens[0][2] + upper_lens[1][2])
    ctx.check(
        "positive tilt raises the spotlight beam",
        lower_center_z is not None and upper_center_z is not None and upper_center_z > lower_center_z + 0.12,
        details=f"lower_center_z={lower_center_z}, upper_center_z={upper_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
