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
    TireGeometry,
    TireGroove,
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


DECK_LENGTH = 0.66
DECK_WIDTH = 0.19
DECK_THICKNESS = 0.035
DECK_CENTER_Z = 0.1125
DECK_TOP_Z = DECK_CENTER_Z + DECK_THICKNESS / 2.0

WHEEL_RADIUS = 0.070
WHEEL_WIDTH = 0.045
FRONT_WHEEL_X = 0.435
FRONT_WHEEL_Y = 0.065
FRONT_WHEEL_Z = WHEEL_RADIUS
REAR_WHEEL_X = -0.405
REAR_WHEEL_Z = WHEEL_RADIUS

STEM_HINGE_X = 0.245
STEM_HINGE_Z = DECK_TOP_Z + 0.025
STEM_HEIGHT = 0.760
STEM_FOLD_LIMIT = 1.50


def _rod_origin(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an origin and length for a cylinder whose local +Z spans p0->p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    midpoint = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0)
    return Origin(xyz=midpoint, rpy=(0.0, pitch, yaw)), length


def _add_rod(part, p0, p1, radius, material, name):
    origin, length = _rod_origin(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wheel_folding_kick_scooter")

    deck_blue = model.material("powder_coated_blue", rgba=(0.05, 0.30, 0.78, 1.0))
    grip_black = model.material("grit_black", rgba=(0.015, 0.015, 0.014, 1.0))
    rubber_black = model.material("matte_rubber_black", rgba=(0.03, 0.03, 0.032, 1.0))
    brushed_metal = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.75, 1.0))
    rim_white = model.material("white_plastic_rim", rgba=(0.92, 0.94, 0.90, 1.0))
    hinge_dark = model.material("dark_hinge_pin", rgba=(0.10, 0.105, 0.11, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            WHEEL_WIDTH * 0.82,
            rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.018,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.022, hole_diameter=0.0025),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.009),
        ),
        "scooter_plastic_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.054,
            tread=TireTread(style="circumferential", depth=0.0025, count=3),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0018),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
        ),
        "scooter_black_tire",
    )

    deck = model.part("deck")
    deck_core_length = DECK_LENGTH - DECK_WIDTH
    deck.visual(
        Box((deck_core_length, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=deck_blue,
        name="deck_center",
    )
    for x, name in ((-deck_core_length / 2.0, "rear_round"), (deck_core_length / 2.0, "front_round")):
        deck.visual(
            Cylinder(radius=DECK_WIDTH / 2.0, length=DECK_THICKNESS),
            origin=Origin(xyz=(x, 0.0, DECK_CENTER_Z)),
            material=deck_blue,
            name=name,
        )

    grip_width = DECK_WIDTH - 0.040
    grip_length = DECK_LENGTH - 0.140
    grip_core_length = grip_length - grip_width
    grip_z = DECK_TOP_Z + 0.001
    deck.visual(
        Box((grip_core_length, grip_width, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, grip_z)),
        material=grip_black,
        name="grip_center",
    )
    for x, name in ((-grip_core_length / 2.0, "grip_rear_round"), (grip_core_length / 2.0, "grip_front_round")):
        deck.visual(
            Cylinder(radius=grip_width / 2.0, length=0.003),
            origin=Origin(xyz=(x, 0.0, grip_z)),
            material=grip_black,
            name=name,
        )

    for y, name in ((-0.083, "side_rail_0"), (0.083, "side_rail_1")):
        deck.visual(
            Box((0.53, 0.014, 0.014)),
            origin=Origin(xyz=(0.0, y, DECK_CENTER_Z - 0.018)),
            material=brushed_metal,
            name=name,
        )

    deck.visual(
        Box((0.120, 0.012, 0.055)),
        origin=Origin(xyz=(-0.375, -0.040, 0.0825)),
        material=brushed_metal,
        name="rear_fork_0",
    )
    deck.visual(
        Box((0.120, 0.012, 0.055)),
        origin=Origin(xyz=(-0.375, 0.040, 0.0825)),
        material=brushed_metal,
        name="rear_fork_1",
    )
    deck.visual(
        Cylinder(radius=0.008, length=0.105),
        origin=Origin(xyz=(REAR_WHEEL_X, 0.0, REAR_WHEEL_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="rear_axle_pin",
    )
    deck.visual(
        Box((0.105, 0.062, 0.006)),
        origin=Origin(xyz=(-0.395, 0.0, 0.154)),
        material=deck_blue,
        name="rear_brake_fender",
    )
    for y, name in ((-0.032, "rear_fender_strut_0"), (0.032, "rear_fender_strut_1")):
        deck.visual(
            Box((0.016, 0.006, 0.046)),
            origin=Origin(xyz=(-0.365, y, 0.132)),
            material=brushed_metal,
            name=name,
        )

    for y, name in ((-0.060, "hinge_cheek_0"), (0.060, "hinge_cheek_1")):
        deck.visual(
            Box((0.080, 0.016, 0.070)),
            origin=Origin(xyz=(STEM_HINGE_X, y, STEM_HINGE_Z)),
            material=brushed_metal,
            name=name,
        )
    deck.visual(
        Cylinder(radius=0.009, length=0.150),
        origin=Origin(xyz=(STEM_HINGE_X, 0.0, STEM_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="hinge_pin",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Box((0.026, 0.105, 0.060)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=brushed_metal,
        name="front_mount",
    )
    _add_rod(front_fork, (0.026, 0.0, 0.020), (0.058, 0.0, 0.075), 0.014, brushed_metal, "fork_stem")
    _add_rod(front_fork, (0.058, 0.0, 0.075), (0.108, -0.096, 0.070), 0.009, brushed_metal, "fork_arm_0")
    _add_rod(front_fork, (0.058, 0.0, 0.075), (0.108, 0.096, 0.070), 0.009, brushed_metal, "fork_arm_1")
    _add_rod(front_fork, (0.058, 0.0, 0.075), (0.110, 0.0, 0.040), 0.010, brushed_metal, "fork_center_web")
    front_fork.visual(
        Box((0.040, 0.012, 0.150)),
        origin=Origin(xyz=(0.110, -0.096, -0.005)),
        material=brushed_metal,
        name="front_lug_0",
    )
    front_fork.visual(
        Box((0.040, 0.012, 0.150)),
        origin=Origin(xyz=(0.110, 0.096, -0.005)),
        material=brushed_metal,
        name="front_lug_1",
    )
    front_fork.visual(
        Box((0.040, 0.018, 0.150)),
        origin=Origin(xyz=(0.110, 0.0, -0.005)),
        material=brushed_metal,
        name="front_center_lug",
    )
    front_fork.visual(
        Cylinder(radius=0.008, length=0.145),
        origin=Origin(
            xyz=(FRONT_WHEEL_X - DECK_LENGTH / 2.0, -FRONT_WHEEL_Y, FRONT_WHEEL_Z - DECK_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_dark,
        name="front_axle_pin_0",
    )
    front_fork.visual(
        Cylinder(radius=0.008, length=0.145),
        origin=Origin(
            xyz=(FRONT_WHEEL_X - DECK_LENGTH / 2.0, FRONT_WHEEL_Y, FRONT_WHEEL_Z - DECK_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_dark,
        name="front_axle_pin_1",
    )

    stem = model.part("handlebar_stem")
    stem.visual(
        Cylinder(radius=0.025, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="folding_barrel",
    )
    _add_rod(stem, (0.0, 0.0, 0.055), (0.0, 0.0, STEM_HEIGHT), 0.016, brushed_metal, "upright_tube")
    _add_rod(stem, (0.0, -0.235, STEM_HEIGHT), (0.0, 0.235, STEM_HEIGHT), 0.014, brushed_metal, "handlebar")
    _add_rod(stem, (0.0, -0.310, STEM_HEIGHT), (0.0, -0.235, STEM_HEIGHT), 0.018, rubber_black, "grip_0")
    _add_rod(stem, (0.0, 0.235, STEM_HEIGHT), (0.0, 0.310, STEM_HEIGHT), 0.018, rubber_black, "grip_1")
    stem.visual(
        Box((0.035, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=brushed_metal,
        name="lower_clamp",
    )

    wheel_parts = []
    for name, y in (("front_wheel_0", -FRONT_WHEEL_Y), ("front_wheel_1", FRONT_WHEEL_Y), ("rear_wheel", 0.0)):
        wheel = model.part(name)
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_white,
            name="rim",
        )
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber_black,
            name="tire",
        )
        wheel_parts.append((wheel, y))

    model.articulation(
        "deck_to_front_fork",
        ArticulationType.FIXED,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(DECK_LENGTH / 2.0, 0.0, DECK_CENTER_Z)),
    )
    model.articulation(
        "deck_to_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(STEM_HINGE_X, 0.0, STEM_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=STEM_FOLD_LIMIT),
    )

    front_wheel_0, _ = wheel_parts[0]
    front_wheel_1, _ = wheel_parts[1]
    rear_wheel, _ = wheel_parts[2]
    model.articulation(
        "front_axle_0",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel_0,
        origin=Origin(xyz=(FRONT_WHEEL_X - DECK_LENGTH / 2.0, -FRONT_WHEEL_Y, FRONT_WHEEL_Z - DECK_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=30.0),
    )
    model.articulation(
        "front_axle_1",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel_1,
        origin=Origin(xyz=(FRONT_WHEEL_X - DECK_LENGTH / 2.0, FRONT_WHEEL_Y, FRONT_WHEEL_Z - DECK_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=30.0),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, 0.0, REAR_WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    stem = object_model.get_part("handlebar_stem")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    rear_wheel = object_model.get_part("rear_wheel")
    stem_hinge = object_model.get_articulation("deck_to_stem")

    ctx.allow_overlap(
        deck,
        stem,
        elem_a="hinge_pin",
        elem_b="folding_barrel",
        reason="The stem folding barrel is intentionally captured around the deck hinge pin.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle_pin",
        elem_b="rim",
        reason="The rear wheel hub is intentionally captured on the rear axle pin.",
    )
    ctx.allow_overlap(
        front_fork,
        front_wheel_0,
        elem_a="front_axle_pin_0",
        elem_b="rim",
        reason="The front wheel hub is intentionally captured on its fork axle pin.",
    )
    ctx.allow_overlap(
        front_fork,
        front_wheel_1,
        elem_a="front_axle_pin_1",
        elem_b="rim",
        reason="The front wheel hub is intentionally captured on its fork axle pin.",
    )

    ctx.check(
        "three rolling wheel axles",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in ("front_axle_0", "front_axle_1", "rear_axle")
        ),
        "Expected two independent front wheel axles and one rear wheel axle.",
    )
    ctx.check(
        "folding stem hinge limit",
        stem_hinge.motion_limits is not None
        and stem_hinge.motion_limits.lower == 0.0
        and stem_hinge.motion_limits.upper is not None
        and stem_hinge.motion_limits.upper > 1.2,
        "The handlebar stem should fold well down from the upright pose.",
    )
    ctx.expect_overlap(
        deck,
        stem,
        axes="y",
        elem_a="hinge_pin",
        elem_b="folding_barrel",
        min_overlap=0.050,
        name="folding barrel is retained on hinge pin",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a="rear_axle_pin",
        elem_b="rim",
        min_overlap=0.035,
        name="rear axle passes through wheel hub",
    )
    ctx.expect_overlap(
        front_fork,
        front_wheel_0,
        axes="y",
        elem_a="front_axle_pin_0",
        elem_b="rim",
        min_overlap=0.035,
        name="front axle 0 passes through wheel hub",
    )
    ctx.expect_overlap(
        front_fork,
        front_wheel_1,
        axes="y",
        elem_a="front_axle_pin_1",
        elem_b="rim",
        min_overlap=0.035,
        name="front axle 1 passes through wheel hub",
    )

    ctx.expect_gap(
        front_wheel_0,
        front_fork,
        axis="y",
        positive_elem="tire",
        negative_elem="front_lug_0",
        min_gap=0.001,
        max_gap=0.015,
        name="front wheel 0 clears its outside fork lug",
    )
    ctx.expect_gap(
        front_fork,
        front_wheel_1,
        axis="y",
        positive_elem="front_lug_1",
        negative_elem="tire",
        min_gap=0.001,
        max_gap=0.015,
        name="front wheel 1 clears its outside fork lug",
    )
    ctx.expect_gap(
        rear_wheel,
        deck,
        axis="y",
        positive_elem="tire",
        negative_elem="rear_fork_0",
        min_gap=0.001,
        max_gap=0.020,
        name="rear wheel clears negative rear fork plate",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="y",
        positive_elem="rear_fork_1",
        negative_elem="tire",
        min_gap=0.001,
        max_gap=0.020,
        name="rear wheel clears positive rear fork plate",
    )

    rest_bar = ctx.part_element_world_aabb(stem, elem="handlebar")
    with ctx.pose({stem_hinge: STEM_FOLD_LIMIT}):
        folded_bar = ctx.part_element_world_aabb(stem, elem="handlebar")
    if rest_bar is not None and folded_bar is not None:
        rest_min, rest_max = rest_bar
        folded_min, folded_max = folded_bar
        rest_center = (
            (float(rest_min[0]) + float(rest_max[0])) / 2.0,
            (float(rest_min[1]) + float(rest_max[1])) / 2.0,
            (float(rest_min[2]) + float(rest_max[2])) / 2.0,
        )
        folded_center = (
            (float(folded_min[0]) + float(folded_max[0])) / 2.0,
            (float(folded_min[1]) + float(folded_max[1])) / 2.0,
            (float(folded_min[2]) + float(folded_max[2])) / 2.0,
        )
        ctx.check(
            "stem folds rearward and downward",
            folded_center[0] < rest_center[0] - 0.40 and folded_center[2] < rest_center[2] - 0.45,
            details=f"rest_center={rest_center}, folded_center={folded_center}",
        )
    else:
        ctx.fail("stem handlebar aabb available", "Could not measure the handlebar visual at folded pose.")

    return ctx.report()


object_model = build_object_model()
