from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_bulk_candy_vendor")

    steel = model.material("brushed_steel", rgba=(0.70, 0.70, 0.66, 1.0))
    dark = model.material("shadowed_chute", rgba=(0.05, 0.055, 0.06, 1.0))
    acrylic = model.material("clear_acrylic", rgba=(0.74, 0.93, 1.0, 0.30))
    blue_acrylic = model.material("blue_tinted_acrylic", rgba=(0.55, 0.82, 1.0, 0.38))
    black_rubber = model.material("black_grip", rgba=(0.02, 0.02, 0.018, 1.0))
    red_candy = model.material("red_candy", rgba=(0.90, 0.06, 0.06, 1.0))
    yellow_candy = model.material("yellow_candy", rgba=(1.0, 0.78, 0.05, 1.0))
    green_candy = model.material("green_candy", rgba=(0.16, 0.70, 0.20, 1.0))
    purple_candy = model.material("purple_candy", rgba=(0.48, 0.14, 0.75, 1.0))

    body = model.part("body")

    # Lower metal metering box, built as a hollow front frame so the chute is
    # visibly recessed rather than painted on a solid block.
    body.visual(Box((0.38, 0.30, 0.028)), origin=Origin(xyz=(0.0, 0.0, 0.094)), material=steel, name="bottom_pan")
    body.visual(Box((0.38, 0.30, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.385)), material=steel, name="top_pan")
    body.visual(Box((0.030, 0.30, 0.292)), origin=Origin(xyz=(-0.175, 0.0, 0.239)), material=steel, name="side_wall_0")
    body.visual(Box((0.030, 0.30, 0.292)), origin=Origin(xyz=(0.175, 0.0, 0.239)), material=steel, name="side_wall_1")
    body.visual(Box((0.38, 0.028, 0.292)), origin=Origin(xyz=(0.0, 0.136, 0.239)), material=steel, name="rear_wall")
    body.visual(Box((0.38, 0.040, 0.170)), origin=Origin(xyz=(0.0, -0.155, 0.300)), material=steel, name="front_panel")
    body.visual(Box((0.090, 0.040, 0.110)), origin=Origin(xyz=(-0.135, -0.155, 0.160)), material=steel, name="front_stile_0")
    body.visual(Box((0.090, 0.040, 0.110)), origin=Origin(xyz=(0.135, -0.155, 0.160)), material=steel, name="front_stile_1")
    body.visual(Box((0.38, 0.040, 0.026)), origin=Origin(xyz=(0.0, -0.155, 0.214)), material=steel, name="chute_header")
    body.visual(Box((0.38, 0.040, 0.026)), origin=Origin(xyz=(0.0, -0.155, 0.104)), material=steel, name="chute_sill")
    body.visual(Box((0.165, 0.010, 0.078)), origin=Origin(xyz=(0.0, -0.088, 0.158)), material=dark, name="chute_back")
    body.visual(Box((0.010, 0.082, 0.082)), origin=Origin(xyz=(-0.088, -0.123, 0.158)), material=dark, name="chute_side_0")
    body.visual(Box((0.010, 0.082, 0.082)), origin=Origin(xyz=(0.088, -0.123, 0.158)), material=dark, name="chute_side_1")
    body.visual(
        Box((0.170, 0.060, 0.007)),
        origin=Origin(xyz=(0.0, -0.108, 0.142), rpy=(0.32, 0.0, 0.0)),
        material=steel,
        name="chute_ramp",
    )

    # Wheel mount on the front of the metering box.
    body.visual(
        Cylinder(radius=0.042, length=0.035),
        origin=Origin(xyz=(0.0, -0.1875, 0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_boss",
    )

    # Clear rectangular upper hopper: open at the top, with metal lower and
    # upper retaining rims and individual transparent wall panels.
    body.visual(Box((0.39, 0.31, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.405)), material=steel, name="lower_collar")
    body.visual(Box((0.340, 0.008, 0.260)), origin=Origin(xyz=(0.0, -0.132, 0.535)), material=acrylic, name="front_window")
    body.visual(Box((0.340, 0.008, 0.260)), origin=Origin(xyz=(0.0, 0.132, 0.535)), material=acrylic, name="rear_window")
    body.visual(Box((0.008, 0.264, 0.260)), origin=Origin(xyz=(-0.174, 0.0, 0.535)), material=acrylic, name="side_window_0")
    body.visual(Box((0.008, 0.264, 0.260)), origin=Origin(xyz=(0.174, 0.0, 0.535)), material=acrylic, name="side_window_1")
    body.visual(Box((0.370, 0.018, 0.018)), origin=Origin(xyz=(0.0, -0.136, 0.670)), material=steel, name="top_rail_front")
    body.visual(Box((0.370, 0.018, 0.018)), origin=Origin(xyz=(0.0, 0.136, 0.670)), material=steel, name="top_rail_rear")
    body.visual(Box((0.018, 0.290, 0.018)), origin=Origin(xyz=(-0.176, 0.0, 0.670)), material=steel, name="top_rail_0")
    body.visual(Box((0.018, 0.290, 0.018)), origin=Origin(xyz=(0.176, 0.0, 0.670)), material=steel, name="top_rail_1")
    body.visual(Box((0.055, 0.055, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.414)), material=dark, name="meter_throat")

    # A supported pile of colorful candy pieces inside the clear hopper.
    for name, xyz, radius, material in (
        ("candy_0", (-0.075, -0.045, 0.438), 0.023, red_candy),
        ("candy_1", (-0.025, -0.055, 0.438), 0.024, yellow_candy),
        ("candy_2", (0.030, -0.040, 0.439), 0.024, green_candy),
        ("candy_3", (0.078, -0.020, 0.438), 0.023, purple_candy),
        ("candy_4", (-0.055, 0.020, 0.439), 0.024, green_candy),
        ("candy_5", (0.005, 0.020, 0.440), 0.025, red_candy),
        ("candy_6", (0.060, 0.035, 0.438), 0.023, yellow_candy),
        ("candy_7", (-0.010, -0.012, 0.464), 0.024, purple_candy),
    ):
        body.visual(Sphere(radius=radius), origin=Origin(xyz=xyz), material=material, name=name)

    # Rear hinge knuckles and fixed hinge leaves for the lift-up lid.
    for name, x in (("lid_hinge_0", -0.137), ("lid_hinge_1", 0.137)):
        body.visual(
            Cylinder(radius=0.011, length=0.080),
            origin=Origin(xyz=(x, 0.154, 0.700), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=name,
        )
        body.visual(Box((0.082, 0.010, 0.030)), origin=Origin(xyz=(x, 0.142, 0.686)), material=steel, name=f"hinge_leaf_{name[-1]}")
    body.visual(
        Cylinder(radius=0.004, length=0.350),
        origin=Origin(xyz=(0.0, 0.154, 0.700), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lid_hinge_pin",
    )

    # Lower hinge lugs for the small chute flap.
    body.visual(Box((0.026, 0.022, 0.026)), origin=Origin(xyz=(-0.094, -0.161, 0.118)), material=steel, name="flap_lug_0")
    body.visual(Box((0.026, 0.022, 0.026)), origin=Origin(xyz=(0.094, -0.161, 0.118)), material=steel, name="flap_lug_1")

    lid = model.part("lid")
    lid.visual(
        Box((0.360, 0.270, 0.010)),
        origin=Origin(xyz=(0.0, -0.155, 0.006)),
        material=blue_acrylic,
        name="lid_panel",
    )
    lid.visual(Box((0.175, 0.040, 0.006)), origin=Origin(xyz=(0.0, -0.020, 0.001)), material=steel, name="hinge_leaf")
    lid.visual(
        Cylinder(radius=0.011, length=0.150),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    lid.visual(Box((0.090, 0.020, 0.030)), origin=Origin(xyz=(0.0, -0.292, 0.024)), material=blue_acrylic, name="front_tab")

    wheel = model.part("dispense_wheel")
    wheel.visual(
        Cylinder(radius=0.078, length=0.028),
        origin=Origin(xyz=(0.0, -0.038, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wheel_disk",
    )
    wheel.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, -0.057, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="center_grip",
    )
    wheel.visual(Box((0.128, 0.012, 0.016)), origin=Origin(xyz=(0.0, -0.071, 0.0)), material=black_rubber, name="grip_bar_x")
    wheel.visual(Box((0.016, 0.012, 0.128)), origin=Origin(xyz=(0.0, -0.071, 0.0)), material=black_rubber, name="grip_bar_z")

    flap = model.part("chute_flap")
    flap.visual(
        Box((0.154, 0.010, 0.078)),
        origin=Origin(xyz=(0.0, -0.003, 0.039)),
        material=steel,
        name="flap_plate",
    )
    flap.visual(
        Cylinder(radius=0.006, length=0.150),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="flap_hinge",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.154, 0.700)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.0, -0.205, 0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, -0.164, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("dispense_wheel")
    flap = object_model.get_part("chute_flap")
    lid_joint = object_model.get_articulation("body_to_lid")
    wheel_joint = object_model.get_articulation("body_to_wheel")
    flap_joint = object_model.get_articulation("body_to_flap")

    ctx.allow_overlap(
        body,
        wheel,
        elem_a="axle_boss",
        elem_b="axle_shaft",
        reason="The dispense wheel shaft is intentionally captured inside the metal front axle boss.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="lid_hinge_pin",
        elem_b="hinge_barrel",
        reason="A slender metal hinge pin intentionally passes through the acrylic lid hinge barrel.",
    )
    ctx.allow_overlap(
        body,
        flap,
        elem_a="chute_sill",
        elem_b="flap_hinge",
        reason="The chute flap hinge barrel is intentionally seated into the lower metal sill.",
    )
    ctx.expect_within(
        wheel,
        body,
        axes="xz",
        inner_elem="axle_shaft",
        outer_elem="axle_boss",
        margin=0.001,
        name="wheel shaft centered in axle boss",
    )
    ctx.expect_overlap(
        wheel,
        body,
        axes="y",
        elem_a="axle_shaft",
        elem_b="axle_boss",
        min_overlap=0.018,
        name="wheel shaft retained in axle boss",
    )
    ctx.expect_within(
        body,
        lid,
        axes="yz",
        inner_elem="lid_hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.002,
        name="lid hinge pin centered in barrel",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="x",
        elem_a="lid_hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.12,
        name="lid hinge pin spans barrel",
    )
    ctx.expect_overlap(
        body,
        flap,
        axes="x",
        elem_a="chute_sill",
        elem_b="flap_hinge",
        min_overlap=0.13,
        name="flap hinge spans chute sill",
    )
    ctx.expect_gap(
        flap,
        body,
        axis="z",
        positive_elem="flap_hinge",
        negative_elem="chute_sill",
        max_penetration=0.006,
        name="flap hinge shallowly seats in sill",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_rail_front",
        min_gap=0.005,
        max_gap=0.04,
        name="closed acrylic lid sits above hopper rim",
    )
    ctx.expect_within(
        flap,
        body,
        axes="xz",
        inner_elem="flap_plate",
        outer_elem="chute_back",
        margin=0.020,
        name="chute flap aligns with recessed opening",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    closed_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({lid_joint: 1.0}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({flap_joint: 0.75}):
        opened_flap_aabb = ctx.part_world_aabb(flap)

    ctx.check(
        "lid lifts upward on rear hinge",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )
    ctx.check(
        "chute flap swings outward",
        closed_flap_aabb is not None
        and opened_flap_aabb is not None
        and opened_flap_aabb[0][1] < closed_flap_aabb[0][1] - 0.015,
        details=f"closed={closed_flap_aabb}, opened={opened_flap_aabb}",
    )
    ctx.check(
        "dispense wheel is continuous",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint.motion_limits.lower is None
        and wheel_joint.motion_limits.upper is None,
        details=f"type={wheel_joint.articulation_type}, limits={wheel_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
