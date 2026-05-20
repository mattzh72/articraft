from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_tripod_device")

    powder_black = model.material("powder_black", rgba=(0.03, 0.035, 0.04, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.25, 0.27, 0.29, 1.0))
    molded_plastic = model.material("molded_plastic", rgba=(0.10, 0.105, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.015, 0.016, 1.0))
    device_glass = model.material("device_glass", rgba=(0.02, 0.045, 0.075, 1.0))
    fastener_zinc = model.material("fastener_zinc", rgba=(0.72, 0.72, 0.68, 1.0))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=molded_plastic,
        name="leg_hub",
    )
    tripod.visual(
        Cylinder(radius=0.016, length=0.610),
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material=powder_black,
        name="center_tube",
    )
    tripod.visual(
        Cylinder(radius=0.030, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
        material=molded_plastic,
        name="top_socket",
    )
    tripod.visual(
        Cylinder(radius=0.038, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.754)),
        material=fastener_zinc,
        name="bearing_washer",
    )
    tripod.visual(
        Box((0.050, 0.014, 0.020)),
        origin=Origin(xyz=(0.055, 0.0, 0.724)),
        material=molded_plastic,
        name="snap_lug",
    )

    leg_angles = (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_geom = tube_from_spline_points(
            [
                (0.030 * c, 0.030 * s, 0.112),
                (0.170 * c, 0.170 * s, 0.062),
                (0.430 * c, 0.430 * s, 0.020),
            ],
            radius=0.011,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        )
        tripod.visual(
            mesh_from_geometry(leg_geom, f"tripod_leg_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        tripod.visual(
            Sphere(radius=0.020),
            origin=Origin(xyz=(0.430 * c, 0.430 * s, 0.020)),
            material=rubber,
            name=f"foot_{index}",
        )

    tripod.inertial = Inertial.from_geometry(
        Box((0.92, 0.92, 0.78)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    pan_plate = model.part("pan_plate")
    pan_plate.visual(
        Cylinder(radius=0.066, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_graphite,
        name="pan_bearing",
    )
    pan_plate.visual(
        Cylinder(radius=0.035, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=molded_plastic,
        name="head_plate",
    )
    pan_plate.visual(
        Box((0.120, 0.176, 0.024)),
        origin=Origin(xyz=(0.018, 0.0, 0.057)),
        material=molded_plastic,
        name="yoke_bridge",
    )
    pan_plate.visual(
        Box((0.040, 0.014, 0.126)),
        origin=Origin(xyz=(0.018, -0.075, 0.108)),
        material=molded_plastic,
        name="yoke_0",
    )
    pan_plate.visual(
        Box((0.040, 0.014, 0.126)),
        origin=Origin(xyz=(0.018, 0.075, 0.108)),
        material=molded_plastic,
        name="yoke_1",
    )
    pan_plate.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.018, -0.092, 0.122), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=fastener_zinc,
        name="tilt_bolt_head",
    )
    pan_plate.visual(
        Box((0.070, 0.018, 0.018)),
        origin=Origin(xyz=(-0.077, 0.0, 0.046)),
        material=molded_plastic,
        name="pan_lock_tab",
    )
    pan_plate.inertial = Inertial.from_geometry(
        Box((0.16, 0.20, 0.18)),
        mass=0.36,
        origin=Origin(xyz=(0.02, 0.0, 0.08)),
    )

    bracket = model.part("bracket")
    bracket.visual(
        Cylinder(radius=0.014, length=0.136),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=fastener_zinc,
        name="tilt_pin",
    )
    bracket.visual(
        Box((0.090, 0.062, 0.032)),
        origin=Origin(xyz=(0.048, 0.0, 0.000)),
        material=molded_plastic,
        name="pivot_web",
    )
    bracket.visual(
        Box((0.014, 0.202, 0.260)),
        origin=Origin(xyz=(0.085, 0.0, 0.000)),
        material=molded_plastic,
        name="backplate",
    )
    bracket.visual(
        Box((0.022, 0.012, 0.260)),
        origin=Origin(xyz=(0.109, -0.101, 0.000)),
        material=molded_plastic,
        name="side_rail_0",
    )
    bracket.visual(
        Box((0.022, 0.012, 0.260)),
        origin=Origin(xyz=(0.109, 0.101, 0.000)),
        material=molded_plastic,
        name="side_rail_1",
    )
    bracket.visual(
        Box((0.052, 0.190, 0.020)),
        origin=Origin(xyz=(0.112, 0.0, -0.103)),
        material=molded_plastic,
        name="fixed_jaw",
    )
    bracket.visual(
        Box((0.018, 0.145, 0.205)),
        origin=Origin(xyz=(0.101, 0.0, 0.010)),
        material=device_glass,
        name="device_body",
    )
    bracket.visual(
        Box((0.006, 0.060, 0.010)),
        origin=Origin(xyz=(0.095, 0.0, 0.118)),
        material=fastener_zinc,
        name="top_stop_clip",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.16, 0.22, 0.32)),
        mass=0.55,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.visual(
        Box((0.042, 0.190, 0.018)),
        origin=Origin(xyz=(0.129, 0.0, 0.000)),
        material=molded_plastic,
        name="upper_pad",
    )
    clamp_jaw.visual(
        Box((0.016, 0.036, 0.040)),
        origin=Origin(xyz=(0.124, 0.0, 0.026)),
        material=molded_plastic,
        name="pull_tab",
    )
    clamp_jaw.visual(
        Box((0.032, 0.240, 0.014)),
        origin=Origin(xyz=(0.124, 0.0, 0.052)),
        material=molded_plastic,
        name="top_bridge",
    )
    clamp_jaw.visual(
        Box((0.020, 0.012, 0.050)),
        origin=Origin(xyz=(0.111, -0.114, 0.026)),
        material=molded_plastic,
        name="guide_0",
    )
    clamp_jaw.visual(
        Box((0.020, 0.012, 0.050)),
        origin=Origin(xyz=(0.111, 0.114, 0.026)),
        material=molded_plastic,
        name="guide_1",
    )
    clamp_jaw.inertial = Inertial.from_geometry(
        Box((0.060, 0.205, 0.075)),
        mass=0.11,
        origin=Origin(xyz=(0.123, 0.0, 0.020)),
    )

    model.articulation(
        "tripod_to_pan",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=pan_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.762)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pan_to_bracket",
        ArticulationType.REVOLUTE,
        parent=pan_plate,
        child=bracket,
        origin=Origin(xyz=(0.018, 0.0, 0.122)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.6, lower=-0.75, upper=1.05),
    )
    model.articulation(
        "bracket_to_clamp",
        ArticulationType.PRISMATIC,
        parent=bracket,
        child=clamp_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.1215)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.16, lower=0.0, upper=0.060),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod = object_model.get_part("tripod")
    pan_plate = object_model.get_part("pan_plate")
    bracket = object_model.get_part("bracket")
    clamp_jaw = object_model.get_part("clamp_jaw")
    pan = object_model.get_articulation("tripod_to_pan")
    tilt = object_model.get_articulation("pan_to_bracket")
    clamp = object_model.get_articulation("bracket_to_clamp")

    ctx.expect_contact(
        pan_plate,
        tripod,
        elem_a="pan_bearing",
        elem_b="bearing_washer",
        contact_tol=0.001,
        name="pan plate sits on bearing washer",
    )
    ctx.expect_contact(
        bracket,
        pan_plate,
        elem_a="tilt_pin",
        elem_b="yoke_0",
        contact_tol=0.001,
        name="tilt pin is captured by yoke cheek",
    )
    ctx.expect_contact(
        clamp_jaw,
        bracket,
        elem_a="upper_pad",
        elem_b="device_body",
        contact_tol=0.001,
        name="sliding clamp pad closes onto device",
    )
    ctx.expect_within(
        clamp_jaw,
        bracket,
        axes="y",
        inner_elem="upper_pad",
        outer_elem="backplate",
        margin=0.010,
        name="clamp pad stays inside bracket width",
    )

    pan_rest = ctx.part_world_aabb(bracket)
    with ctx.pose({pan: 0.9}):
        pan_rotated = ctx.part_world_aabb(bracket)
    ctx.check(
        "pan joint rotates bracket footprint",
        pan_rest is not None
        and pan_rotated is not None
        and abs(pan_rest[0][0] - pan_rotated[0][0]) > 0.025,
        details=f"rest={pan_rest}, rotated={pan_rotated}",
    )

    tilt_rest = ctx.part_world_aabb(bracket)
    with ctx.pose({tilt: 0.65}):
        tilt_up = ctx.part_world_aabb(bracket)
    ctx.check(
        "tilt joint raises device bracket",
        tilt_rest is not None
        and tilt_up is not None
        and tilt_up[1][2] > tilt_rest[1][2] + 0.025,
        details=f"rest={tilt_rest}, tilted={tilt_up}",
    )

    closed_pos = ctx.part_world_position(clamp_jaw)
    with ctx.pose({clamp: 0.055}):
        open_pos = ctx.part_world_position(clamp_jaw)
        ctx.expect_gap(
            clamp_jaw,
            bracket,
            axis="z",
            positive_elem="upper_pad",
            negative_elem="device_body",
            min_gap=0.045,
            name="clamp opens above device",
        )
    ctx.check(
        "clamp slider moves upward",
        closed_pos is not None and open_pos is not None and open_pos[2] > closed_pos[2] + 0.050,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
