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
    model = ArticulatedObject(name="compact_speedlight_flash")

    matte_black = model.material("matte_black", rgba=(0.015, 0.015, 0.014, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.035, 0.036, 0.034, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.18, 0.17, 1.0))
    shoe_metal = model.material("shoe_metal", rgba=(0.55, 0.55, 0.52, 1.0))
    diffuser = model.material("warm_diffuser", rgba=(1.0, 0.94, 0.72, 0.72))
    label_white = model.material("label_white", rgba=(0.86, 0.86, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.034, 0.046, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=shoe_metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.038, 0.005, 0.006)),
        origin=Origin(xyz=(0.0, 0.0255, 0.007)),
        material=shoe_metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.038, 0.005, 0.006)),
        origin=Origin(xyz=(0.0, -0.0255, 0.007)),
        material=shoe_metal,
        name="shoe_rail_1",
    )
    body.visual(
        Box((0.024, 0.032, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=matte_black,
        name="shoe_neck",
    )
    body.visual(
        Box((0.041, 0.058, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=matte_black,
        name="battery_body",
    )
    body.visual(
        Box((0.036, 0.052, 0.035)),
        origin=Origin(xyz=(0.002, 0.0, 0.1075)),
        material=dark_rubber,
        name="upper_step",
    )
    body.visual(
        Box((0.004, 0.050, 0.060)),
        origin=Origin(xyz=(0.0225, 0.0, 0.052)),
        material=dark_rubber,
        name="battery_door",
    )
    body.visual(
        Box((0.003, 0.003, 0.050)),
        origin=Origin(xyz=(0.0255, 0.018, 0.053)),
        material=graphite,
        name="door_groove_0",
    )
    body.visual(
        Box((0.003, 0.003, 0.050)),
        origin=Origin(xyz=(0.0255, -0.018, 0.053)),
        material=graphite,
        name="door_groove_1",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=graphite,
        name="swivel_socket",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=graphite,
        name="swivel_disc",
    )
    swivel.visual(
        Box((0.022, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=matte_black,
        name="neck_post",
    )
    swivel.visual(
        Box((0.024, 0.086, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=matte_black,
        name="yoke_bridge",
    )
    swivel.visual(
        Box((0.017, 0.007, 0.052)),
        origin=Origin(xyz=(0.0, 0.0395, 0.060)),
        material=matte_black,
        name="yoke_arm_0",
    )
    swivel.visual(
        Box((0.017, 0.007, 0.052)),
        origin=Origin(xyz=(0.0, -0.0395, 0.060)),
        material=matte_black,
        name="yoke_arm_1",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.007, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="tilt_pin",
    )
    head.visual(
        Box((0.016, 0.064, 0.020)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=matte_black,
        name="pivot_block",
    )
    head.visual(
        Box((0.080, 0.064, 0.046)),
        origin=Origin(xyz=(0.046, 0.0, 0.006)),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.050, 0.028)),
        origin=Origin(xyz=(0.0868, 0.0, 0.008)),
        material=diffuser,
        name="front_lens",
    )
    head.visual(
        Box((0.003, 0.052, 0.005)),
        origin=Origin(xyz=(0.0865, 0.0, 0.0245)),
        material=graphite,
        name="lens_brow",
    )
    head.visual(
        Box((0.038, 0.048, 0.003)),
        origin=Origin(xyz=(0.052, 0.0, 0.0295)),
        material=label_white,
        name="bounce_card",
    )

    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.131)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=3.0,
            lower=-0.35,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("head")
    swivel_joint = object_model.get_articulation("body_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")

    ctx.check(
        "two aiming joints",
        len(object_model.articulations) == 2,
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "lower joint is vertical swivel",
        tuple(swivel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={swivel_joint.axis}",
    )
    ctx.check(
        "upper joint is horizontal tilt",
        abs(tilt_joint.axis[1]) == 1.0 and tilt_joint.axis[0] == 0.0 and tilt_joint.axis[2] == 0.0,
        details=f"axis={tilt_joint.axis}",
    )

    ctx.expect_contact(
        body,
        swivel,
        elem_a="swivel_socket",
        elem_b="swivel_disc",
        name="swivel turntable sits on body socket",
    )
    ctx.expect_contact(
        swivel,
        head,
        elem_a="yoke_arm_0",
        elem_b="tilt_pin",
        contact_tol=0.0015,
        name="tilt pin reaches yoke arm",
    )
    ctx.expect_overlap(
        head,
        swivel,
        axes="x",
        elem_a="tilt_pin",
        elem_b="yoke_arm_0",
        min_overlap=0.006,
        name="tilt pin is centered in yoke cheek",
    )

    def _aabb_center_z(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[2] + hi[2]) * 0.5

    def _aabb_center_xy(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    rest_head_z = _aabb_center_z(head, "head_shell")
    with ctx.pose({tilt_joint: 1.0}):
        tilted_head_z = _aabb_center_z(head, "head_shell")
    ctx.check(
        "tilt raises lamp head",
        rest_head_z is not None and tilted_head_z is not None and tilted_head_z > rest_head_z + 0.02,
        details=f"rest_z={rest_head_z}, tilted_z={tilted_head_z}",
    )

    rest_head_xy = _aabb_center_xy(head, "head_shell")
    with ctx.pose({swivel_joint: 1.0}):
        swiveled_head_xy = _aabb_center_xy(head, "head_shell")
    ctx.check(
        "swivel changes lamp heading",
        rest_head_xy is not None
        and swiveled_head_xy is not None
        and abs(swiveled_head_xy[1] - rest_head_xy[1]) > 0.025,
        details=f"rest_xy={rest_head_xy}, swiveled_xy={swiveled_head_xy}",
    )

    return ctx.report()


object_model = build_object_model()
