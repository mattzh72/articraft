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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_prism(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """A centered annular solid whose thickness is along local Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -0.5 * thickness))
    )


def _tube(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A centered vertical tube, used for the hollow telescoping sleeve/collars."""
    return _annular_prism(outer_radius, inner_radius, height)


def _tripod_base() -> cq.Workplane:
    """Weighted hub, low tripod feet, and the lower socket as one molded base."""
    base = cq.Workplane("XY").circle(0.135).extrude(0.035)
    base = base.union(cq.Workplane("XY").circle(0.032).extrude(0.060))
    for yaw_deg in (0.0, 120.0, 240.0):
        leg = (
            cq.Workplane("XY")
            .box(0.46, 0.040, 0.026)
            .translate((0.205, 0.0, 0.018))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), yaw_deg)
        )
        base = base.union(leg)
    return base


def _ring_head_body() -> cq.Workplane:
    """Flat salon ring-light housing in local XY, with the tilt axis on local X."""
    body = _annular_prism(0.225, 0.165, 0.040)
    # Short trunnion bosses are unioned into the annular housing at the left and
    # right sides so the tilt pivots read as part of the ring head.
    right_boss = cq.Workplane("YZ").circle(0.026).extrude(0.046).translate((0.218, 0.0, 0.0))
    left_boss = cq.Workplane("YZ").circle(0.026).extrude(0.046).translate((-0.264, 0.0, 0.0))
    return body.union(right_boss).union(left_boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="salon_ring_light")

    matte_black = model.material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    satin_black = model.material("satin_black", rgba=(0.025, 0.025, 0.03, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.58, 0.58, 0.55, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(1.0, 0.88, 0.58, 0.72))
    dark_rubber = model.material("dark_rubber", rgba=(0.006, 0.006, 0.006, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_tripod_base(), "tripod_base"),
        origin=Origin(),
        material=dark_rubber,
        name="tripod_base",
    )
    base.visual(
        mesh_from_cadquery(_tube(0.026, 0.020, 1.10), "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        material=brushed_metal,
        name="outer_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_tube(0.043, 0.023, 0.080), "stand_collar"),
        origin=Origin(xyz=(0.0, 0.0, 1.140)),
        material=satin_black,
        name="stand_collar",
    )
    base.visual(
        Cylinder(radius=0.005, length=0.079),
        origin=Origin(xyz=(0.0, -0.055, 1.140), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="collar_clamp_screw",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, -0.084, 1.140), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="collar_clamp_knob",
    )
    # Two stationary hinge knuckles and leaves on the +X side of the collar.
    base.visual(
        Box((0.034, 0.018, 0.010)),
        origin=Origin(xyz=(0.052, 0.0, 1.110)),
        material=satin_black,
        name="side_hinge_leaf_lower",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.070, 0.0, 1.110)),
        material=brushed_metal,
        name="side_hinge_knuckle_lower",
    )
    base.visual(
        Box((0.034, 0.018, 0.010)),
        origin=Origin(xyz=(0.052, 0.0, 1.170)),
        material=satin_black,
        name="side_hinge_leaf_upper",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.070, 0.0, 1.170)),
        material=brushed_metal,
        name="side_hinge_knuckle_upper",
    )

    mast_yoke = model.part("mast_yoke")
    mast_yoke.visual(
        Cylinder(radius=0.016, length=1.25),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=brushed_metal,
        name="inner_mast",
    )
    mast_yoke.visual(
        Box((0.052, 0.046, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=satin_black,
        name="yoke_neck_block",
    )
    mast_yoke.visual(
        Box((0.540, 0.034, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=satin_black,
        name="yoke_lower_bridge",
    )
    for side, x in (("0", -0.276), ("1", 0.276)):
        mast_yoke.visual(
            Box((0.024, 0.034, 0.330)),
            origin=Origin(xyz=(x, 0.0, 0.500)),
            material=satin_black,
            name=f"yoke_cheek_{side}",
        )
        mast_yoke.visual(
            Cylinder(radius=0.025, length=0.010),
            origin=Origin(xyz=(x + (0.017 if x > 0.0 else -0.017), 0.0, 0.650), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name=f"tilt_screw_cap_{side}",
        )

    ring_head = model.part("ring_head")
    ring_head.visual(
        mesh_from_cadquery(_ring_head_body(), "ring_housing"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="ring_housing",
    )
    ring_head.visual(
        mesh_from_cadquery(_annular_prism(0.207, 0.176, 0.006), "led_diffuser"),
        origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_diffuser,
        name="led_diffuser",
    )
    ring_head.visual(
        Box((0.055, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.020, -0.220)),
        material=matte_black,
        name="bottom_cable_boss",
    )

    accessory_arm = model.part("accessory_arm")
    accessory_arm.visual(
        Cylinder(radius=0.008, length=0.038),
        origin=Origin(),
        material=brushed_metal,
        name="hinge_center_knuckle",
    )
    accessory_arm.visual(
        Cylinder(radius=0.008, length=0.290),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="folding_arm_tube",
    )
    accessory_arm.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.285, 0.0, 0.0)),
        material=satin_black,
        name="ball_mount",
    )
    accessory_arm.visual(
        Box((0.036, 0.082, 0.056)),
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        material=matte_black,
        name="accessory_clamp",
    )
    accessory_arm.visual(
        Box((0.010, 0.096, 0.015)),
        origin=Origin(xyz=(0.330, 0.0, 0.027)),
        material=satin_black,
        name="clamp_upper_lip",
    )
    accessory_arm.visual(
        Box((0.010, 0.096, 0.015)),
        origin=Origin(xyz=(0.330, 0.0, -0.027)),
        material=satin_black,
        name="clamp_lower_lip",
    )

    model.articulation(
        "stand_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast_yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.340),
    )
    model.articulation(
        "ring_tilt",
        ArticulationType.REVOLUTE,
        parent=mast_yoke,
        child=ring_head,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.55, upper=0.95),
    )
    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=accessory_arm,
        origin=Origin(xyz=(0.070, 0.0, 1.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-1.35, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast_yoke = object_model.get_part("mast_yoke")
    ring_head = object_model.get_part("ring_head")
    accessory_arm = object_model.get_part("accessory_arm")
    stand_slide = object_model.get_articulation("stand_slide")
    ring_tilt = object_model.get_articulation("ring_tilt")
    arm_hinge = object_model.get_articulation("arm_hinge")

    ctx.check(
        "primary mechanisms are authored",
        stand_slide.articulation_type == ArticulationType.PRISMATIC
        and ring_tilt.articulation_type == ArticulationType.REVOLUTE
        and arm_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"slide={stand_slide.articulation_type}, tilt={ring_tilt.articulation_type}, arm={arm_hinge.articulation_type}",
    )
    ctx.expect_within(
        mast_yoke,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="telescoping mast is centered in hollow sleeve",
    )
    ctx.expect_overlap(
        mast_yoke,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.30,
        name="collapsed mast remains deeply inserted",
    )
    ctx.expect_gap(
        ring_head,
        mast_yoke,
        axis="z",
        positive_elem="ring_housing",
        negative_elem="yoke_lower_bridge",
        min_gap=0.045,
        name="ring clears the lower yoke bridge",
    )
    ctx.expect_overlap(
        accessory_arm,
        base,
        axes="xy",
        elem_a="hinge_center_knuckle",
        elem_b="side_hinge_knuckle_upper",
        min_overlap=0.012,
        name="accessory arm knuckle shares the side hinge axis",
    )

    rest_mast = ctx.part_world_position(mast_yoke)
    with ctx.pose({stand_slide: 0.340}):
        ctx.expect_within(
            mast_yoke,
            base,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended mast stays centered in sleeve",
        )
        ctx.expect_overlap(
            mast_yoke,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.035,
            name="extended mast retains insertion",
        )
        extended_mast = ctx.part_world_position(mast_yoke)
    ctx.check(
        "stand slide raises the yoke",
        rest_mast is not None and extended_mast is not None and extended_mast[2] > rest_mast[2] + 0.30,
        details=f"rest={rest_mast}, extended={extended_mast}",
    )

    rest_ring = ctx.part_element_world_aabb(ring_head, elem="led_diffuser")
    with ctx.pose({ring_tilt: 0.70}):
        tilted_ring = ctx.part_element_world_aabb(ring_head, elem="led_diffuser")
    ctx.check(
        "ring head pitches on horizontal tilt axis",
        rest_ring is not None
        and tilted_ring is not None
        and abs((tilted_ring[1][1] - tilted_ring[0][1]) - (rest_ring[1][1] - rest_ring[0][1])) > 0.10,
        details=f"rest={rest_ring}, tilted={tilted_ring}",
    )

    rest_arm = ctx.part_world_aabb(accessory_arm)
    with ctx.pose({arm_hinge: 1.20}):
        swung_arm = ctx.part_world_aabb(accessory_arm)
    ctx.check(
        "side accessory arm folds around collar hinge",
        rest_arm is not None
        and swung_arm is not None
        and swung_arm[1][1] > rest_arm[1][1] + 0.18,
        details=f"rest={rest_arm}, swung={swung_arm}",
    )

    return ctx.report()


object_model = build_object_model()
