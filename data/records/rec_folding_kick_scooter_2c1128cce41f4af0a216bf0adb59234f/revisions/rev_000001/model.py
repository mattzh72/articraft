from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="seated_folding_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.73, 0.74, 1.0))
    dark_frame = model.material("charcoal_frame", rgba=(0.035, 0.040, 0.045, 1.0))
    deck_blue = model.material("blue_deck_shell", rgba=(0.05, 0.22, 0.80, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    latch_red = model.material("red_snap_latch", rgba=(0.85, 0.03, 0.02, 1.0))
    seat_vinyl = model.material("black_seat_vinyl", rgba=(0.015, 0.012, 0.010, 1.0))
    rim_silver = model.material("silver_rim", rgba=(0.82, 0.84, 0.82, 1.0))

    body = model.part("body")
    # Long, low, flat ride-on scooter deck.  The root frame is centered on the
    # deck plan, with +X at the handlebar/front end and -X at the seat/rear end.
    body.visual(
        Box((0.90, 0.24, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=deck_blue,
        name="deck_shell",
    )
    body.visual(
        Box((0.76, 0.185, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.166)),
        material=rubber,
        name="grip_mat",
    )
    for i, y in enumerate((-0.070, -0.035, 0.0, 0.035, 0.070)):
        body.visual(
            Box((0.70, 0.009, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.1725)),
            material=dark_frame,
            name=f"deck_rib_{i}",
        )

    # Front and rear fork plates wrap around the wheel sides while remaining
    # clear of the tires.  They overlap the deck underside so each fork is a
    # continuous piece of the fixed scooter frame.
    for prefix, x, bridge_name in (
        ("front", 0.515, "front_fork_bridge"),
        ("rear", -0.515, "rear_fork_bridge"),
    ):
        body.visual(
            Box((0.110, 0.150, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.238)),
            material=hinge_metal,
            name=bridge_name,
        )
        for side, y in (("near", -0.058), ("far", 0.058)):
            body.visual(
                Box((0.140, 0.018, 0.182)),
                origin=Origin(xyz=(x, y, 0.137)),
                material=hinge_metal,
                name=f"{prefix}_fork_{side}",
            )

    # Folding handlebar hinge clevis on the front of the deck.
    body.visual(
        Box((0.115, 0.155, 0.032)),
        origin=Origin(xyz=(0.410, 0.0, 0.1785)),
        material=hinge_metal,
        name="front_hinge_base",
    )
    for side, y in (("near", -0.064), ("far", 0.064)):
        body.visual(
            Box((0.042, 0.026, 0.090)),
            origin=Origin(xyz=(0.410, y, 0.2235)),
            material=hinge_metal,
            name=f"front_hinge_ear_{side}",
        )

    # Rear clevis for the snap-fold seat support arm.
    body.visual(
        Box((0.135, 0.160, 0.032)),
        origin=Origin(xyz=(-0.355, 0.0, 0.1785)),
        material=hinge_metal,
        name="rear_hinge_base",
    )
    for side, y in (("near", -0.064), ("far", 0.064)):
        body.visual(
            Box((0.046, 0.026, 0.092)),
            origin=Origin(xyz=(-0.355, y, 0.2245)),
            material=hinge_metal,
            name=f"rear_hinge_ear_{side}",
        )
    body.visual(
        Box((0.055, 0.025, 0.020)),
        origin=Origin(xyz=(-0.300, 0.086, 0.202)),
        material=latch_red,
        name="seat_latch_catch",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.021, length=0.102),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="stem_hinge_barrel",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.660),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=aluminum,
        name="stem_tube",
    )
    stem.visual(
        Box((0.070, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        material=hinge_metal,
        name="bar_clamp",
    )
    stem.visual(
        Cylinder(radius=0.016, length=0.480),
        origin=Origin(xyz=(0.0, 0.0, 0.690), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handlebar_bar",
    )
    for side, y in (("near", -0.255), ("far", 0.255)):
        stem.visual(
            Cylinder(radius=0.022, length=0.105),
            origin=Origin(xyz=(0.0, y, 0.690), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{side}",
        )
    stem.visual(
        Box((0.048, 0.024, 0.018)),
        origin=Origin(xyz=(0.010, 0.027, 0.055)),
        material=latch_red,
        name="stem_snap_button",
    )

    seat_arm = model.part("seat_arm")
    seat_arm.visual(
        Cylinder(radius=0.021, length=0.102),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="seat_hinge_barrel",
    )
    seat_arm.visual(
        Cylinder(radius=0.017, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=aluminum,
        name="seat_post",
    )
    seat_arm.visual(
        Box((0.085, 0.060, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.462)),
        material=hinge_metal,
        name="seat_bracket",
    )
    seat_arm.visual(
        Box((0.300, 0.190, 0.052)),
        origin=Origin(xyz=(0.075, 0.0, 0.505)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat_arm.visual(
        Box((0.044, 0.024, 0.018)),
        origin=Origin(xyz=(0.025, 0.027, 0.060)),
        material=latch_red,
        name="seat_snap_button",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.079,
            0.046,
            rim=WheelRim(inner_radius=0.050, flange_height=0.006, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.025,
                width=0.040,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.005, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.003, window_radius=0.009),
            bore=WheelBore(style="round", diameter=0.012),
        ),
        "scooter_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.105,
            0.054,
            inner_radius=0.073,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.055),
            tread=TireTread(style="chevron", depth=0.0045, count=20, angle_deg=22.0, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.005, radius=0.003),
        ),
        "scooter_tire",
    )

    for name in ("front_wheel", "rear_wheel"):
        wheel = model.part(name)
        # Wheel helper meshes spin around local X; rotate them so the axle is
        # across the scooter width (+Y in the body frame).
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_silver,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.008, length=0.098),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name="axle_pin",
        )

    model.articulation(
        "stem_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stem,
        origin=Origin(xyz=(0.410, 0.0, 0.226)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-1.42, upper=0.10),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=seat_arm,
        origin=Origin(xyz=(-0.355, 0.0, 0.226)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2, lower=0.0, upper=1.42),
    )
    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="front_wheel",
        origin=Origin(xyz=(0.565, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="rear_wheel",
        origin=Origin(xyz=(-0.565, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    stem = object_model.get_part("stem")
    seat_arm = object_model.get_part("seat_arm")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    stem_hinge = object_model.get_articulation("stem_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")

    ctx.expect_overlap(
        front_wheel,
        body,
        axes="y",
        elem_a="tire",
        elem_b="front_fork_bridge",
        min_overlap=0.050,
        name="front tire sits between fork plates",
    )
    ctx.expect_overlap(
        rear_wheel,
        body,
        axes="y",
        elem_a="tire",
        elem_b="rear_fork_bridge",
        min_overlap=0.050,
        name="rear tire sits between fork plates",
    )
    ctx.expect_gap(
        body,
        front_wheel,
        axis="z",
        positive_elem="front_fork_bridge",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.040,
        name="front fork bridge clears tire crown",
    )
    ctx.expect_gap(
        body,
        rear_wheel,
        axis="x",
        positive_elem="deck_shell",
        negative_elem="tire",
        min_gap=0.0,
        max_gap=0.020,
        name="rear wheel sits just behind deck",
    )
    ctx.expect_gap(
        front_wheel,
        body,
        axis="x",
        positive_elem="tire",
        negative_elem="deck_shell",
        min_gap=0.0,
        max_gap=0.020,
        name="front wheel sits just ahead of deck",
    )
    ctx.expect_gap(
        body,
        rear_wheel,
        axis="z",
        positive_elem="rear_fork_bridge",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.040,
        name="rear fork bridge clears tire crown",
    )

    handle_upright = ctx.part_element_world_aabb(stem, elem="handlebar_bar")
    with ctx.pose({stem_hinge: -1.35}):
        handle_folded = ctx.part_element_world_aabb(stem, elem="handlebar_bar")
    ctx.check(
        "stem folds down toward deck",
        handle_upright is not None
        and handle_folded is not None
        and handle_folded[1][2] < handle_upright[1][2] - 0.35
        and handle_folded[0][0] < handle_upright[0][0] - 0.30,
        details=f"upright={handle_upright}, folded={handle_folded}",
    )

    seat_upright = ctx.part_element_world_aabb(seat_arm, elem="seat_cushion")
    with ctx.pose({seat_hinge: 1.35}):
        seat_folded = ctx.part_element_world_aabb(seat_arm, elem="seat_cushion")
    ctx.check(
        "seat support folds forward over deck",
        seat_upright is not None
        and seat_folded is not None
        and seat_folded[1][2] < seat_upright[1][2] - 0.20
        and seat_folded[0][0] > seat_upright[0][0] + 0.35,
        details=f"upright={seat_upright}, folded={seat_folded}",
    )

    return ctx.report()


object_model = build_object_model()
