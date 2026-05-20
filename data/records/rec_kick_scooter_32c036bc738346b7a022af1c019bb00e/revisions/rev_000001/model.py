from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
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
    model = ArticulatedObject(name="folding_kick_scooter")

    aluminum = Material("brushed_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_metal = Material("dark_powder_coat", rgba=(0.05, 0.055, 0.06, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    grip_tape = Material("grip_tape", rgba=(0.015, 0.017, 0.018, 1.0))
    red_release = Material("red_release_catch", rgba=(0.80, 0.04, 0.025, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.028,
            rim=WheelRim(
                inner_radius=0.030,
                flange_height=0.004,
                flange_thickness=0.0025,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.015,
                width=0.024,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.022,
                    hole_diameter=0.0024,
                ),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0024, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        "scooter_polyurethane_wheel_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.032,
            inner_radius=0.053,
            carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.0025, count=24, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.004, radius=0.0025),
        ),
        "scooter_polyurethane_tire",
    )

    deck = model.part("deck")
    # A narrow rounded-rectangle deck: a straight extrusion plus vertical round caps.
    deck.visual(
        Box((0.62, 0.105, 0.026)),
        origin=Origin(xyz=(-0.005, 0.0, 0.115)),
        material=aluminum,
        name="deck_center",
    )
    for x, name in [(-0.315, "rear_round_cap"), (0.305, "front_round_cap")]:
        deck.visual(
            Cylinder(radius=0.0525, length=0.026),
            origin=Origin(xyz=(x, 0.0, 0.115)),
            material=aluminum,
            name=name,
        )
    deck.visual(
        Box((0.54, 0.083, 0.004)),
        origin=Origin(xyz=(-0.035, 0.0, 0.1295)),
        material=grip_tape,
        name="grip_strip",
    )
    deck.visual(
        Box((0.58, 0.010, 0.018)),
        origin=Origin(xyz=(-0.010, 0.057, 0.117)),
        material=dark_metal,
        name="side_rail_0",
    )
    deck.visual(
        Box((0.58, 0.010, 0.018)),
        origin=Origin(xyz=(-0.010, -0.057, 0.117)),
        material=dark_metal,
        name="side_rail_1",
    )

    # Rear wheel hanger and axle, fixed to the deck.
    deck.visual(
        Box((0.090, 0.012, 0.080)),
        origin=Origin(xyz=(-0.445, 0.038, 0.100)),
        material=dark_metal,
        name="rear_fork_leg_0",
    )
    deck.visual(
        Box((0.090, 0.012, 0.080)),
        origin=Origin(xyz=(-0.445, -0.038, 0.100)),
        material=dark_metal,
        name="rear_fork_leg_1",
    )
    deck.visual(
        Box((0.105, 0.012, 0.018)),
        origin=Origin(xyz=(-0.390, 0.038, 0.128)),
        material=dark_metal,
        name="rear_side_stay_0",
    )
    deck.visual(
        Box((0.105, 0.012, 0.018)),
        origin=Origin(xyz=(-0.390, -0.038, 0.128)),
        material=dark_metal,
        name="rear_side_stay_1",
    )
    deck.visual(
        Cylinder(radius=0.008, length=0.102),
        origin=Origin(xyz=(-0.460, 0.0, 0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle",
    )

    # Deck-side folding hinge: two outside knuckles and cheek plates around the nose.
    deck.visual(
        Box((0.075, 0.105, 0.020)),
        origin=Origin(xyz=(0.335, 0.0, 0.136)),
        material=dark_metal,
        name="hinge_base_plate",
    )
    for y, name in [(0.040, "hinge_cheek_0"), (-0.040, "hinge_cheek_1")]:
        deck.visual(
            Box((0.050, 0.026, 0.050)),
            origin=Origin(xyz=(0.350, y, 0.151)),
            material=dark_metal,
            name=name,
        )
        deck.visual(
            Cylinder(radius=0.018, length=0.032),
            origin=Origin(xyz=(0.350, y, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"fixed_hinge_knuckle_{0 if y > 0 else 1}",
        )

    # Rotating front fork / upright stem assembly.  Its local frame is the hinge pin.
    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.0165, length=0.043),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="folding_hinge_knuckle",
    )
    stem.visual(
        Box((0.055, 0.040, 0.055)),
        origin=Origin(xyz=(0.018, 0.0, 0.024)),
        material=aluminum,
        name="hinge_lug",
    )
    stem.visual(
        Box((0.018, 0.032, 0.030)),
        origin=Origin(xyz=(0.047, 0.0, 0.016)),
        material=red_release,
        name="folding_release_catch",
    )
    stem.visual(
        Box((0.048, 0.034, 0.018)),
        origin=Origin(xyz=(0.018, 0.0, 0.055)),
        material=aluminum,
        name="stem_socket",
    )

    stem_length = 0.760
    stem_angle = math.radians(-8.0)
    stem_dx = math.sin(stem_angle) * stem_length
    stem_dz = math.cos(stem_angle) * stem_length
    stem.visual(
        Cylinder(radius=0.018, length=stem_length),
        origin=Origin(
            xyz=(0.018 + 0.5 * stem_dx, 0.0, 0.060 + 0.5 * stem_dz),
            rpy=(0.0, stem_angle, 0.0),
        ),
        material=aluminum,
        name="main_stem_tube",
    )
    stem.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(
            xyz=(0.018 + 0.14 * stem_dx, 0.0, 0.060 + 0.14 * stem_dz),
            rpy=(0.0, stem_angle, 0.0),
        ),
        material=dark_metal,
        name="lower_clamp_collar",
    )
    stem.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(
            xyz=(0.018 + 0.68 * stem_dx, 0.0, 0.060 + 0.68 * stem_dz),
            rpy=(0.0, stem_angle, 0.0),
        ),
        material=dark_metal,
        name="upper_clamp_collar",
    )
    top_x = 0.018 + stem_dx
    top_z = 0.060 + stem_dz
    stem.visual(
        Cylinder(radius=0.014, length=0.460),
        origin=Origin(xyz=(top_x, 0.0, top_z + 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handlebar_crossbar",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.115),
        origin=Origin(xyz=(top_x, 0.287, top_z + 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="grip_0",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.115),
        origin=Origin(xyz=(top_x, -0.287, top_z + 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="grip_1",
    )

    # Front fork yoke descends from the same rotating stem to the front axle.
    stem.visual(
        Box((0.100, 0.092, 0.022)),
        origin=Origin(xyz=(0.080, 0.0, 0.017)),
        material=aluminum,
        name="front_fork_crown",
    )
    stem.visual(
        Box((0.028, 0.011, 0.128)),
        origin=Origin(xyz=(0.110, 0.038, -0.057)),
        material=aluminum,
        name="front_fork_leg_0",
    )
    stem.visual(
        Box((0.028, 0.011, 0.128)),
        origin=Origin(xyz=(0.110, -0.038, -0.057)),
        material=aluminum,
        name="front_fork_leg_1",
    )
    stem.visual(
        Cylinder(radius=0.008, length=0.102),
        origin=Origin(xyz=(0.110, 0.0, -0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_axle",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="rear_tire",
    )
    rear_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="rear_wheel_core",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="front_tire",
    )
    front_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="front_wheel_core",
    )

    model.articulation(
        "deck_to_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(0.350, 0.0, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.0,
            lower=math.radians(-80.0),
            upper=math.radians(90.0),
        ),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.460, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=45.0),
    )
    model.articulation(
        "stem_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=stem,
        child=front_wheel,
        origin=Origin(xyz=(0.110, 0.0, -0.085)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=45.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    rear_wheel = object_model.get_part("rear_wheel")
    front_wheel = object_model.get_part("front_wheel")
    fold = object_model.get_articulation("deck_to_stem")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")
    front_spin = object_model.get_articulation("stem_to_front_wheel")

    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="rear_wheel_core",
        reason="The rear wheel spins around a captured metal axle passing through the molded hub bore.",
    )
    ctx.allow_overlap(
        stem,
        front_wheel,
        elem_a="front_axle",
        elem_b="front_wheel_core",
        reason="The front wheel spins around a captured fork axle passing through the molded hub bore.",
    )

    ctx.check(
        "wheel joints spin continuously",
        rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and front_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"rear={rear_spin.articulation_type}, front={front_spin.articulation_type}",
    )
    ctx.check(
        "stem hinge travel is about 170 degrees",
        fold.motion_limits is not None
        and abs((fold.motion_limits.upper - fold.motion_limits.lower) - math.radians(170.0)) < math.radians(2.0)
        and abs(fold.motion_limits.upper - math.radians(90.0)) < math.radians(2.0),
        details=f"limits={fold.motion_limits}",
    )
    ctx.expect_overlap(rear_wheel, deck, axes="y", min_overlap=0.025, name="rear wheel centered under deck width")
    ctx.expect_overlap(front_wheel, stem, axes="y", min_overlap=0.025, name="front wheel centered in fork width")
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a="rear_axle",
        elem_b="rear_wheel_core",
        min_overlap=0.025,
        name="rear axle passes through wheel hub",
    )
    ctx.expect_overlap(
        stem,
        front_wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="front_wheel_core",
        min_overlap=0.025,
        name="front axle passes through wheel hub",
    )
    ctx.expect_within(
        deck,
        rear_wheel,
        axes="xz",
        inner_elem="rear_axle",
        outer_elem="rear_wheel_core",
        margin=0.004,
        name="rear axle centered in wheel hub profile",
    )
    ctx.expect_within(
        stem,
        front_wheel,
        axes="xz",
        inner_elem="front_axle",
        outer_elem="front_wheel_core",
        margin=0.004,
        name="front axle centered in wheel hub profile",
    )

    rear_box = ctx.part_world_aabb(rear_wheel)
    front_box = ctx.part_world_aabb(front_wheel)
    ctx.check(
        "small wheels sit on the same ground plane",
        rear_box is not None
        and front_box is not None
        and abs(rear_box[0][2]) < 0.003
        and abs(front_box[0][2]) < 0.003,
        details=f"rear={rear_box}, front={front_box}",
    )

    upright_bar = ctx.part_element_world_aabb(stem, elem="handlebar_crossbar")
    with ctx.pose({fold: math.radians(90.0)}):
        folded_bar = ctx.part_element_world_aabb(stem, elem="handlebar_crossbar")

    ctx.check(
        "folding hinge swings the handlebar rearward and downward",
        upright_bar is not None
        and folded_bar is not None
        and folded_bar[1][2] < upright_bar[1][2] - 0.35
        and folded_bar[0][0] < upright_bar[0][0] - 0.45,
        details=f"upright={upright_bar}, folded={folded_bar}",
    )

    return ctx.report()


object_model = build_object_model()
