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


CANOPY_RADIUS = 0.060
CANOPY_HEIGHT = 0.022
STEM_RADIUS = 0.010
STEM_LENGTH = 0.032

SHADE_OUTER_RADIUS = 0.210
SHADE_INNER_RADIUS = 0.206
SHADE_TOP_Z = 0.055
SHADE_BOTTOM_Z = 0.195
SHADE_HEIGHT = SHADE_BOTTOM_Z - SHADE_TOP_Z

TOP_BAND_OUTER_RADIUS = SHADE_OUTER_RADIUS
TOP_BAND_INNER_RADIUS = 0.196
TOP_BAND_HEIGHT = 0.010
TOP_BAND_CENTER_Z = 0.058

BOTTOM_BAND_OUTER_RADIUS = SHADE_OUTER_RADIUS
BOTTOM_BAND_INNER_RADIUS = 0.196
BOTTOM_BAND_HEIGHT = 0.010
BOTTOM_BAND_CENTER_Z = 0.192

LOWER_RING_OUTER_RADIUS = 0.197
LOWER_RING_INNER_RADIUS = 0.195
LOWER_RING_HEIGHT = 0.008
LOWER_RING_CENTER_Z = 0.192

HUB_RADIUS = 0.036
HUB_HEIGHT = 0.006
HUB_CENTER_Z = 0.057
ARM_WIDTH = 0.012
ARM_THICKNESS = 0.004
ARM_LENGTH = TOP_BAND_INNER_RADIUS - HUB_RADIUS + 0.004
ARM_CENTER_RADIUS = HUB_RADIUS + ARM_LENGTH / 2.0 - 0.002

DIFFUSER_HINGE_X = 0.214
DIFFUSER_Z = 0.194
DIFFUSER_FRAME_OUTER_RADIUS = 0.1935
DIFFUSER_FRAME_INNER_RADIUS = 0.184
DIFFUSER_FRAME_HEIGHT = 0.008
DIFFUSER_PANEL_RADIUS = 0.1845
DIFFUSER_PANEL_HEIGHT = 0.004

HINGE_BARREL_RADIUS = 0.004
BODY_KNUCKLE_LENGTH = 0.015
BODY_KNUCKLE_Y = 0.017
DIFFUSER_KNUCKLE_LENGTH = 0.010


def _annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height / 2.0, both=True)
    )


def _cylinder(radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height / 2.0, both=True)


def _cylinder_along_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _notched_annulus(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    notch_depth: float,
    notch_width: float,
) -> cq.Workplane:
    ring = _annulus(outer_radius, inner_radius, height)
    notch = cq.Workplane("XY").box(notch_depth, notch_width, height * 1.5).translate(
        (outer_radius - notch_depth / 2.0 + 0.0005, 0.0, 0.0)
    )
    return ring.cut(notch)


def _body_hinge_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.018, 0.056, 0.004).translate((0.005, 0.0, -0.006))
    upper_web = cq.Workplane("XY").box(0.008, 0.014, 0.006).translate((0.000, BODY_KNUCKLE_Y, -0.003))
    lower_web = cq.Workplane("XY").box(0.008, 0.014, 0.006).translate((0.000, -BODY_KNUCKLE_Y, -0.003))
    knuckle_a = _cylinder_along_y(HINGE_BARREL_RADIUS, BODY_KNUCKLE_LENGTH).translate(
        (0.0, BODY_KNUCKLE_Y, 0.0)
    )
    knuckle_b = _cylinder_along_y(HINGE_BARREL_RADIUS, BODY_KNUCKLE_LENGTH).translate(
        (0.0, -BODY_KNUCKLE_Y, 0.0)
    )
    return plate.union(upper_web).union(lower_web).union(knuckle_a).union(knuckle_b)


def _diffuser_hinge_shape() -> cq.Workplane:
    arm = cq.Workplane("XY").box(0.017, 0.050, 0.008).translate((-0.0125, 0.0, 0.008))
    center_web = cq.Workplane("XY").box(0.004, 0.010, 0.008).translate((-0.002, 0.0, 0.004))
    knuckle = _cylinder_along_y(HINGE_BARREL_RADIUS, DIFFUSER_KNUCKLE_LENGTH)
    return arm.union(center_web).union(knuckle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_shade_ceiling_fixture")

    model.material("satin_nickel", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("shade_cream", rgba=(0.92, 0.91, 0.86, 1.0))
    model.material("diffuser_white", rgba=(0.94, 0.95, 0.96, 0.92))

    body = model.part("body")
    body.visual(
        Cylinder(radius=CANOPY_RADIUS, length=CANOPY_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT / 2.0)),
        material="satin_nickel",
        name="canopy",
    )
    body.visual(
        Cylinder(radius=STEM_RADIUS, length=STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT + STEM_LENGTH / 2.0)),
        material="satin_nickel",
        name="stem",
    )
    body.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, HUB_CENTER_Z)),
        material="satin_nickel",
        name="hub",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        body.visual(
            Box((ARM_LENGTH, ARM_WIDTH, ARM_THICKNESS)),
            origin=Origin(
                xyz=(
                    ARM_CENTER_RADIUS * math.cos(angle),
                    ARM_CENTER_RADIUS * math.sin(angle),
                    HUB_CENTER_Z - 0.001,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material="satin_nickel",
            name=f"arm_{index}",
        )

    body.visual(
        mesh_from_cadquery(_annulus(TOP_BAND_OUTER_RADIUS, TOP_BAND_INNER_RADIUS, TOP_BAND_HEIGHT), "top_band"),
        origin=Origin(xyz=(0.0, 0.0, TOP_BAND_CENTER_Z)),
        material="satin_nickel",
        name="top_band",
    )
    body.visual(
        mesh_from_cadquery(_annulus(SHADE_OUTER_RADIUS, SHADE_INNER_RADIUS, SHADE_HEIGHT), "shade_shell"),
        origin=Origin(xyz=(0.0, 0.0, (SHADE_TOP_Z + SHADE_BOTTOM_Z) / 2.0)),
        material="shade_cream",
        name="shade_shell",
    )
    body.visual(
        mesh_from_cadquery(
            _notched_annulus(
                BOTTOM_BAND_OUTER_RADIUS,
                BOTTOM_BAND_INNER_RADIUS,
                BOTTOM_BAND_HEIGHT,
                notch_depth=0.020,
                notch_width=0.070,
            ),
            "bottom_band",
        ),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_BAND_CENTER_Z)),
        material="satin_nickel",
        name="bottom_band",
    )
    body.visual(
        mesh_from_cadquery(
            _annulus(LOWER_RING_OUTER_RADIUS, LOWER_RING_INNER_RADIUS, LOWER_RING_HEIGHT),
            "lower_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, LOWER_RING_CENTER_Z)),
        material="satin_nickel",
        name="lower_ring",
    )
    body.visual(
        mesh_from_cadquery(_body_hinge_shape(), "body_hinge"),
        origin=Origin(xyz=(DIFFUSER_HINGE_X, 0.0, DIFFUSER_Z)),
        material="satin_nickel",
        name="body_hinge",
    )

    diffuser = model.part("diffuser")
    diffuser.visual(
        mesh_from_cadquery(
            _annulus(
                DIFFUSER_FRAME_OUTER_RADIUS,
                DIFFUSER_FRAME_INNER_RADIUS,
                DIFFUSER_FRAME_HEIGHT,
            ),
            "diffuser_frame",
        ),
        origin=Origin(xyz=(-DIFFUSER_HINGE_X, 0.0, 0.0)),
        material="satin_nickel",
        name="diffuser_frame",
    )
    diffuser.visual(
        Cylinder(radius=DIFFUSER_PANEL_RADIUS, length=DIFFUSER_PANEL_HEIGHT),
        origin=Origin(xyz=(-DIFFUSER_HINGE_X, 0.0, 0.0)),
        material="diffuser_white",
        name="diffuser_panel",
    )
    diffuser.visual(
        mesh_from_cadquery(_diffuser_hinge_shape(), "diffuser_hinge"),
        origin=Origin(),
        material="satin_nickel",
        name="diffuser_hinge",
    )

    model.articulation(
        "body_to_diffuser",
        ArticulationType.REVOLUTE,
        parent=body,
        child=diffuser,
        origin=Origin(xyz=(DIFFUSER_HINGE_X, 0.0, DIFFUSER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=3.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    diffuser = object_model.get_part("diffuser")
    hinge = object_model.get_articulation("body_to_diffuser")

    ctx.expect_within(
        diffuser,
        body,
        axes="xy",
        inner_elem="diffuser_frame",
        outer_elem="lower_ring",
        margin=0.0035,
        name="closed diffuser frame stays inside the lower opening",
    )
    ctx.expect_overlap(
        diffuser,
        body,
        axes="xy",
        elem_a="diffuser_panel",
        elem_b="lower_ring",
        min_overlap=0.36,
        name="closed diffuser remains centered beneath the lower ring",
    )

    closed_aabb = ctx.part_world_aabb(diffuser)
    lower_ring_aabb = ctx.part_element_world_aabb(body, elem="lower_ring")
    diffuser_frame_aabb = ctx.part_element_world_aabb(diffuser, elem="diffuser_frame")
    ctx.check(
        "closed diffuser sits near the ring plane",
        lower_ring_aabb is not None
        and diffuser_frame_aabb is not None
        and abs(diffuser_frame_aabb[1][2] - lower_ring_aabb[1][2]) <= 0.004,
        details=f"lower_ring={lower_ring_aabb}, diffuser_frame={diffuser_frame_aabb}",
    )

    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(diffuser)
        ctx.check(
            "diffuser swings clearly downward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.10
            and open_aabb[0][0] > closed_aabb[0][0] + 0.08,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
