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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_watch_winder")

    wood = model.material("aged_walnut", rgba=(0.29, 0.15, 0.065, 1.0))
    endgrain = model.material("dark_endgrain", rgba=(0.18, 0.085, 0.035, 1.0))
    brass = model.material("aged_brass", rgba=(0.78, 0.57, 0.26, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black = model.material("blackened_steel", rgba=(0.025, 0.027, 0.026, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    cushion = model.material("oxblood_cushion", rgba=(0.23, 0.025, 0.035, 1.0))
    glass = model.material("smoked_glass", rgba=(0.25, 0.38, 0.42, 0.34))
    label = model.material("cream_label", rgba=(0.86, 0.78, 0.62, 1.0))

    base = model.part("box")

    # Old presentation-box core: a real open wooden tray, not a solid block.
    base.visual(Box((0.284, 0.364, 0.014)), origin=Origin(xyz=(0.0, 0.0, 0.007)), material=wood, name="bottom")
    base.visual(Box((0.014, 0.364, 0.108)), origin=Origin(xyz=(-0.135, 0.0, 0.066)), material=wood, name="rear_wall")
    base.visual(Box((0.014, 0.364, 0.096)), origin=Origin(xyz=(0.135, 0.0, 0.060)), material=wood, name="front_wall")
    base.visual(Box((0.284, 0.014, 0.104)), origin=Origin(xyz=(0.0, -0.175, 0.064)), material=wood, name="side_wall_0")
    base.visual(Box((0.284, 0.014, 0.104)), origin=Origin(xyz=(0.0, 0.175, 0.064)), material=wood, name="side_wall_1")

    # Raised lip and dark gasket under the lid.
    base.visual(Box((0.258, 0.018, 0.008)), origin=Origin(xyz=(0.0, -0.165, 0.122)), material=endgrain, name="top_lip_0")
    base.visual(Box((0.258, 0.018, 0.008)), origin=Origin(xyz=(0.0, 0.165, 0.122)), material=endgrain, name="top_lip_1")
    base.visual(Box((0.018, 0.330, 0.008)), origin=Origin(xyz=(0.128, 0.0, 0.122)), material=endgrain, name="front_lip")
    base.visual(Box((0.018, 0.330, 0.008)), origin=Origin(xyz=(-0.128, 0.0, 0.122)), material=endgrain, name="rear_lip")
    base.visual(Box((0.238, 0.012, 0.002)), origin=Origin(xyz=(0.0, -0.158, 0.125)), material=rubber, name="gasket_0")
    base.visual(Box((0.238, 0.012, 0.002)), origin=Origin(xyz=(0.0, 0.158, 0.125)), material=rubber, name="gasket_1")
    base.visual(Box((0.012, 0.306, 0.002)), origin=Origin(xyz=(0.122, 0.0, 0.125)), material=rubber, name="front_gasket")

    # Internal motor deck and bolted reinforcements for the cradle supports.
    base.visual(Box((0.220, 0.260, 0.010)), origin=Origin(xyz=(0.000, 0.0, 0.019)), material=endgrain, name="motor_deck")
    base.visual(Box((0.080, 0.034, 0.006)), origin=Origin(xyz=(0.0, -0.115, 0.027)), material=brass, name="bearing_foot_0")
    base.visual(Box((0.080, 0.034, 0.006)), origin=Origin(xyz=(0.0, 0.115, 0.027)), material=brass, name="bearing_foot_1")
    base.visual(Box((0.012, 0.018, 0.054)), origin=Origin(xyz=(-0.018, -0.115, 0.055)), material=brass, name="bearing_post_0a")
    base.visual(Box((0.012, 0.018, 0.054)), origin=Origin(xyz=(0.018, -0.115, 0.055)), material=brass, name="bearing_post_0b")
    base.visual(Box((0.012, 0.018, 0.054)), origin=Origin(xyz=(-0.018, 0.115, 0.055)), material=brass, name="bearing_post_1a")
    base.visual(Box((0.012, 0.018, 0.054)), origin=Origin(xyz=(0.018, 0.115, 0.055)), material=brass, name="bearing_post_1b")
    base.visual(Box((0.046, 0.020, 0.012)), origin=Origin(xyz=(0.0, -0.115, 0.056)), material=brass, name="bearing_saddle_0")
    base.visual(Box((0.046, 0.020, 0.012)), origin=Origin(xyz=(0.0, 0.115, 0.056)), material=brass, name="bearing_saddle_1")
    bearing_ring_mesh = mesh_from_geometry(TorusGeometry(0.014, 0.004, radial_segments=28, tubular_segments=16), "bearing_ring")
    base.visual(bearing_ring_mesh, origin=Origin(xyz=(0.0, -0.115, 0.074), rpy=(math.pi / 2, 0.0, 0.0)), material=brass, name="bearing_ring_0")
    base.visual(bearing_ring_mesh, origin=Origin(xyz=(0.0, 0.115, 0.074), rpy=(math.pi / 2, 0.0, 0.0)), material=brass, name="bearing_ring_1")

    # Retrofit service hatches: visible, bolted access panels for old-school maintainability.
    base.visual(Box((0.004, 0.110, 0.052)), origin=Origin(xyz=(-0.143, 0.0, 0.058)), material=black, name="rear_hatch")
    base.visual(Box((0.104, 0.004, 0.048)), origin=Origin(xyz=(0.030, 0.183, 0.057)), material=black, name="side_hatch")
    for sx in (-0.030, 0.030):
        for sz in (0.040, 0.076):
            base.visual(Cylinder(radius=0.0032, length=0.003), origin=Origin(xyz=(-0.146, sx, sz), rpy=(0.0, math.pi / 2, 0.0)), material=steel, name=f"rear_hatch_bolt_{sx}_{sz}")
    for sx in (-0.010, 0.070):
        for sz in (0.039, 0.075):
            base.visual(Cylinder(radius=0.0032, length=0.003), origin=Origin(xyz=(sx, 0.186, sz), rpy=(-math.pi / 2, 0.0, 0.0)), material=steel, name=f"side_hatch_bolt_{sx}_{sz}")

    # Split hinge leaves and alternating knuckles avoid fake collisions while clearly showing support points.
    hinge_y_segments = ((-0.1425, 0.045), (0.0, 0.070), (0.1425, 0.045))
    for idx, (yc, length) in enumerate(hinge_y_segments):
        base.visual(Box((0.006, length, 0.034)), origin=Origin(xyz=(-0.144, yc, 0.111)), material=brass, name=f"fixed_hinge_leaf_{idx}")
        base.visual(Cylinder(radius=0.006, length=length), origin=Origin(xyz=(-0.146, yc, 0.126), rpy=(math.pi / 2, 0.0, 0.0)), material=brass, name=f"fixed_hinge_knuckle_{idx}")

    # Front latch receiver and corner straps make the box read as a retrofit object, not a clean prototype.
    base.visual(Box((0.006, 0.052, 0.022)), origin=Origin(xyz=(0.144, 0.0, 0.092)), material=brass, name="latch_receiver")
    for x in (-0.132, 0.132):
        for y in (-0.172, 0.172):
            base.visual(Box((0.026, 0.004, 0.072)), origin=Origin(xyz=(x, y, 0.062)), material=brass, name=f"corner_strap_{x}_{y}")

    lid = model.part("lid")
    # The lid frame origin is the hinge axis.  The closed lid extends in local +X.
    lid.visual(Box((0.285, 0.020, 0.020)), origin=Origin(xyz=(0.145, -0.167, 0.010)), material=wood, name="side_rail_0")
    lid.visual(Box((0.285, 0.020, 0.020)), origin=Origin(xyz=(0.145, 0.167, 0.010)), material=wood, name="side_rail_1")
    lid.visual(Box((0.045, 0.354, 0.020)), origin=Origin(xyz=(0.040, 0.0, 0.010)), material=wood, name="rear_rail")
    lid.visual(Box((0.040, 0.354, 0.022)), origin=Origin(xyz=(0.274, 0.0, 0.011)), material=wood, name="front_rail")
    lid.visual(Box((0.232, 0.310, 0.006)), origin=Origin(xyz=(0.156, 0.0, 0.010)), material=glass, name="glass_pane")
    lid.visual(Box((0.022, 0.070, 0.020)), origin=Origin(xyz=(0.292, 0.0, 0.002)), material=brass, name="latch_tongue")
    for idx, yc in enumerate((-0.080, 0.080)):
        lid.visual(Box((0.042, 0.070, 0.004)), origin=Origin(xyz=(0.021, yc, 0.003)), material=brass, name=f"moving_hinge_leaf_{idx}")
        lid.visual(Cylinder(radius=0.006, length=0.070), origin=Origin(xyz=(0.0, yc, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=brass, name=f"moving_hinge_knuckle_{idx}")

    cradle = model.part("cradle")
    # Axis-aligned shaft with visible two-point bearing support.
    cradle.visual(Cylinder(radius=0.0105, length=0.260), origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name="axle")
    cradle.visual(Cylinder(radius=0.045, length=0.128), origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=cushion, name="watch_cushion")
    for yc in (-0.071, 0.071):
        cradle.visual(Cylinder(radius=0.026, length=0.010), origin=Origin(xyz=(0.0, yc, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=brass, name=f"adapter_flange_{yc}")
        for angle in (math.radians(45), math.radians(135), math.radians(225), math.radians(315)):
            bx = 0.017 * math.cos(angle)
            bz = 0.017 * math.sin(angle)
            cradle.visual(Cylinder(radius=0.0025, length=0.004), origin=Origin(xyz=(bx, yc, bz), rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name=f"adapter_bolt_{yc}_{round(angle, 2)}")
    cradle.visual(Box((0.010, 0.128, 0.010)), origin=Origin(xyz=(0.040, 0.0, 0.024)), material=rubber, name="watch_strap_0")
    cradle.visual(Box((0.010, 0.128, 0.010)), origin=Origin(xyz=(0.040, 0.0, -0.024)), material=rubber, name="watch_strap_1")
    cradle.visual(Cylinder(radius=0.027, length=0.008), origin=Origin(xyz=(0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)), material=black, name="watch_case")
    cradle.visual(Cylinder(radius=0.021, length=0.009), origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)), material=label, name="watch_dial")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.146, 0.0, 0.126)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=0.0, upper=1.20),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    box = object_model.get_part("box")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_spin = object_model.get_articulation("cradle_spin")

    ctx.allow_overlap(
        box,
        cradle,
        elem_a="bearing_ring_0",
        elem_b="axle",
        reason="The steel axle is intentionally captured with slight seated interference in the bronze bearing ring.",
    )
    ctx.allow_overlap(
        box,
        cradle,
        elem_a="bearing_ring_1",
        elem_b="axle",
        reason="The steel axle is intentionally captured with slight seated interference in the bronze bearing ring.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(lid, box, axes="xy", min_overlap=0.20, name="closed lid covers the presentation box")
        ctx.expect_gap(
            lid,
            box,
            axis="z",
            positive_elem="front_rail",
            negative_elem="front_lip",
            max_penetration=0.0,
            max_gap=0.002,
            name="closed front rail seats on the raised lip",
        )

    ctx.expect_within(cradle, box, axes="xy", margin=0.004, name="rotating cradle remains inside the box footprint")
    ctx.expect_within(cradle, box, axes="xz", elem_a="axle", elem_b="bearing_ring_0", margin=0.001, name="first bearing surrounds the axle")
    ctx.expect_within(cradle, box, axes="xz", elem_a="axle", elem_b="bearing_ring_1", margin=0.001, name="second bearing surrounds the axle")
    ctx.expect_overlap(cradle, box, axes="y", elem_a="axle", elem_b="bearing_ring_0", min_overlap=0.006, name="axle passes through first bearing support")
    ctx.expect_overlap(cradle, box, axes="y", elem_a="axle", elem_b="bearing_ring_1", min_overlap=0.006, name="axle passes through second bearing support")

    closed_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    with ctx.pose({lid_hinge: 1.0, cradle_spin: 1.2}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_rail")
        ctx.expect_within(cradle, box, axes="xy", margin=0.004, name="spinning cradle stays captured by the support rings")

    ctx.check(
        "lid hinge opens the front rail upward",
        closed_front is not None and open_front is not None and open_front[0][2] > closed_front[0][2] + 0.08,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    return ctx.report()


object_model = build_object_model()
