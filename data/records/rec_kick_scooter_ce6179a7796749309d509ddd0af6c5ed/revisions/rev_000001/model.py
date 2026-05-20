from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
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
    model = ArticulatedObject(name="stunt_kick_scooter")

    aluminum = model.material("brushed_aluminum", color=(0.72, 0.74, 0.72, 1.0))
    blue = model.material("anodized_blue", color=(0.03, 0.18, 0.75, 1.0))
    black = model.material("matte_black", color=(0.01, 0.01, 0.012, 1.0))
    dark_rubber = model.material("dark_rubber", color=(0.006, 0.006, 0.005, 1.0))
    grip = model.material("textured_grip", color=(0.025, 0.025, 0.022, 1.0))
    hardware = model.material("dark_hardware", color=(0.08, 0.08, 0.085, 1.0))

    wheel_core_mesh = mesh_from_geometry(
        WheelGeometry(
            0.041,
            0.028,
            rim=WheelRim(
                inner_radius=0.027,
                flange_height=0.003,
                flange_thickness=0.002,
                bead_seat_depth=0.0015,
            ),
            hub=WheelHub(
                radius=0.015,
                width=0.024,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.024,
                    hole_diameter=0.003,
                ),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.005),
            bore=WheelBore(style="round", diameter=0.008),
        ),
        "scooter_wheel_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.030,
            inner_radius=0.040,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.03),
            tread=TireTread(style="ribbed", depth=0.0025, count=28, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0012),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.003, radius=0.002),
        ),
        "scooter_tire",
    )

    deck = model.part("deck")
    deck.visual(
        Box((0.52, 0.110, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=blue,
        name="deck_shell",
    )
    for x, name in ((0.260, "front_round_end"), (-0.260, "rear_round_end")):
        deck.visual(
            Cylinder(radius=0.055, length=0.035),
            origin=Origin(xyz=(x, 0.0, 0.130)),
            material=blue,
            name=name,
        )
    deck.visual(
        Box((0.445, 0.078, 0.004)),
        origin=Origin(xyz=(-0.010, 0.0, 0.1495)),
        material=grip,
        name="grip_tape",
    )
    deck.visual(
        Box((0.090, 0.010, 0.020)),
        origin=Origin(xyz=(0.305, 0.054, 0.145)),
        material=blue,
        name="front_side_boss_0",
    )
    deck.visual(
        Box((0.090, 0.010, 0.020)),
        origin=Origin(xyz=(0.305, -0.054, 0.145)),
        material=blue,
        name="front_side_boss_1",
    )
    deck.visual(
        Box((0.070, 0.010, 0.075)),
        origin=Origin(xyz=(0.340, 0.020, 0.184)),
        material=aluminum,
        name="steering_bearing_cheek_0",
    )
    deck.visual(
        Box((0.070, 0.010, 0.075)),
        origin=Origin(xyz=(0.340, -0.020, 0.184)),
        material=aluminum,
        name="steering_bearing_cheek_1",
    )
    deck.visual(
        Box((0.072, 0.012, 0.070)),
        origin=Origin(xyz=(-0.310, 0.036, 0.086)),
        material=aluminum,
        name="rear_dropout_0",
    )
    deck.visual(
        Box((0.072, 0.012, 0.070)),
        origin=Origin(xyz=(-0.310, -0.036, 0.086)),
        material=aluminum,
        name="rear_dropout_1",
    )
    deck.visual(
        Box((0.050, 0.070, 0.010)),
        origin=Origin(xyz=(-0.292, 0.0, 0.116)),
        material=aluminum,
        name="rear_bridge",
    )
    deck.visual(
        Box((0.060, 0.030, 0.007)),
        origin=Origin(xyz=(0.055, -0.065, 0.109)),
        material=hardware,
        name="kickstand_hinge_bridge",
    )
    deck.visual(
        Box((0.010, 0.017, 0.035)),
        origin=Origin(xyz=(0.033, -0.068, 0.094)),
        material=hardware,
        name="kickstand_hinge_tab_0",
    )
    deck.visual(
        Box((0.010, 0.017, 0.035)),
        origin=Origin(xyz=(0.077, -0.068, 0.094)),
        material=hardware,
        name="kickstand_hinge_tab_1",
    )

    front_steerer = model.part("front_steerer")
    front_steerer.visual(
        Cylinder(radius=0.015, length=0.755),
        origin=Origin(xyz=(0.0, 0.0, 0.3725)),
        material=aluminum,
        name="fixed_bar_stem",
    )
    front_steerer.visual(
        Cylinder(radius=0.013, length=0.370),
        origin=Origin(xyz=(0.0, 0.0, 0.740), rpy=(-pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="crossbar",
    )
    front_steerer.visual(
        Cylinder(radius=0.017, length=0.105),
        origin=Origin(xyz=(0.0, 0.237, 0.740), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark_rubber,
        name="grip_0",
    )
    front_steerer.visual(
        Cylinder(radius=0.017, length=0.105),
        origin=Origin(xyz=(0.0, -0.237, 0.740), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark_rubber,
        name="grip_1",
    )
    front_steerer.visual(
        Box((0.055, 0.086, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, -0.0125)),
        material=blue,
        name="fork_crown",
    )
    front_steerer.visual(
        Cylinder(radius=0.008, length=0.105),
        origin=Origin(xyz=(0.025, 0.035, -0.0675)),
        material=aluminum,
        name="fork_leg_0",
    )
    front_steerer.visual(
        Cylinder(radius=0.008, length=0.105),
        origin=Origin(xyz=(0.025, -0.035, -0.0675)),
        material=aluminum,
        name="fork_leg_1",
    )
    front_steerer.visual(
        Box((0.030, 0.012, 0.024)),
        origin=Origin(xyz=(0.025, 0.035, -0.0925)),
        material=hardware,
        name="front_axle_lug_0",
    )
    front_steerer.visual(
        Box((0.030, 0.012, 0.024)),
        origin=Origin(xyz=(0.025, -0.035, -0.0925)),
        material=hardware,
        name="front_axle_lug_1",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, pi / 2)),
        material=dark_rubber,
        name="tire",
    )
    front_wheel.visual(
        wheel_core_mesh,
        origin=Origin(rpy=(0.0, 0.0, pi / 2)),
        material=aluminum,
        name="wheel_core",
    )
    front_wheel.visual(
        Cylinder(radius=0.0055, length=0.058),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=hardware,
        name="axle_spacer",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, pi / 2)),
        material=dark_rubber,
        name="tire",
    )
    rear_wheel.visual(
        wheel_core_mesh,
        origin=Origin(rpy=(0.0, 0.0, pi / 2)),
        material=aluminum,
        name="wheel_core",
    )
    rear_wheel.visual(
        Cylinder(radius=0.0055, length=0.060),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=hardware,
        name="axle_spacer",
    )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=hardware,
        name="hinge_pin",
    )
    kickstand.visual(
        Cylinder(radius=0.0065, length=0.118),
        origin=Origin(xyz=(0.0, -0.061, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="folding_leg",
    )
    kickstand.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.0, -0.126, 0.0)),
        material=dark_rubber,
        name="rubber_foot",
    )

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_steerer,
        origin=Origin(xyz=(0.340, 0.0, 0.1475)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_steerer,
        child=front_wheel,
        origin=Origin(xyz=(0.025, 0.0, -0.0925)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.310, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )
    model.articulation(
        "kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=kickstand,
        origin=Origin(xyz=(0.055, -0.068, 0.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_steerer = object_model.get_part("front_steerer")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    kickstand = object_model.get_part("kickstand")

    steering = object_model.get_articulation("steering_yaw")
    kickstand_hinge = object_model.get_articulation("kickstand_hinge")

    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="z",
        min_gap=0.001,
        positive_elem="deck_shell",
        negative_elem="tire",
        name="rear wheel clears the deck underside",
    )
    ctx.expect_gap(
        deck,
        kickstand,
        axis="z",
        min_gap=0.001,
        positive_elem="deck_shell",
        negative_elem="folding_leg",
        name="stowed kickstand sits below the deck",
    )
    ctx.expect_contact(
        front_steerer,
        front_wheel,
        elem_a="front_axle_lug_0",
        elem_b="axle_spacer",
        contact_tol=0.001,
        name="front fork lug captures the wheel axle",
    )

    front_rest = ctx.part_world_position(front_wheel)
    with ctx.pose({steering: 0.60}):
        front_turned = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork yaws around the steering axis",
        front_rest is not None
        and front_turned is not None
        and abs(front_turned[1] - front_rest[1]) > 0.010,
        details=f"rest={front_rest}, turned={front_turned}",
    )

    stand_rest = ctx.part_element_world_aabb(kickstand, elem="rubber_foot")
    with ctx.pose({kickstand_hinge: 1.10}):
        stand_down = ctx.part_element_world_aabb(kickstand, elem="rubber_foot")
    rest_z = stand_rest[0][2] if stand_rest is not None else None
    down_z = stand_down[0][2] if stand_down is not None else None
    ctx.check(
        "kickstand swings downward from its side hinge",
        rest_z is not None and down_z is not None and down_z < rest_z - 0.070,
        details=f"rest_foot_min_z={rest_z}, deployed_foot_min_z={down_z}",
    )

    return ctx.report()


object_model = build_object_model()
