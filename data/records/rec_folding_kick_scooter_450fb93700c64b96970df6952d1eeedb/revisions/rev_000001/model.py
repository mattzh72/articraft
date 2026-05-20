from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


DECK_LENGTH = 0.62
DECK_WIDTH = 0.145
DECK_HEIGHT = 0.035
DECK_Z = 0.095
DECK_TOP = DECK_Z + DECK_HEIGHT / 2.0
DECK_BOTTOM = DECK_Z - DECK_HEIGHT / 2.0

WHEEL_RADIUS = 0.055
WHEEL_WIDTH = 0.032
REAR_AXLE_X = -0.380
FRONT_HINGE_X = 0.335
FRONT_HINGE_Z = 0.155
FRONT_AXLE_X = 0.075
FRONT_AXLE_Z = WHEEL_RADIUS - FRONT_HINGE_Z


def _rounded_deck_mesh():
    deck_shape = (
        cq.Workplane("XY")
        .box(DECK_LENGTH, DECK_WIDTH, DECK_HEIGHT)
        .edges("|Z")
        .fillet(0.015)
    )
    return mesh_from_cadquery(deck_shape, "rounded_wide_deck")


def _scooter_wheel_meshes(prefix: str):
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.043,
            WHEEL_WIDTH * 0.86,
            rim=WheelRim(inner_radius=0.030, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.015,
                width=0.024,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.019, hole_diameter=0.0026),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.001),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0024, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.008),
        ),
        f"{prefix}_aluminum_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.044,
            tread=TireTread(style="circumferential", depth=0.0025, count=3),
            grooves=(TireGroove(center_offset=0.0, width=0.0035, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
        ),
        f"{prefix}_urethane_tire",
    )
    return wheel_mesh, tire_mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stunt_scooter")

    deck_blue = model.material("anodized_blue", rgba=(0.05, 0.20, 0.70, 1.0))
    dark_grip = model.material("black_grip_tape", rgba=(0.015, 0.015, 0.013, 1.0))
    black_rubber = model.material("matte_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    polished = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.75, 1.0))
    dark_metal = model.material("dark_hardware", rgba=(0.08, 0.085, 0.09, 1.0))

    deck = model.part("deck")
    deck.visual(
        _rounded_deck_mesh(),
        origin=Origin(xyz=(0.0, 0.0, DECK_Z)),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        Box((0.54, 0.115, 0.004)),
        origin=Origin(xyz=(-0.015, 0.0, DECK_TOP + 0.0018)),
        material=dark_grip,
        name="grip_tape",
    )
    deck.visual(
        Box((0.116, 0.014, 0.018)),
        origin=Origin(xyz=(-0.345, 0.041, 0.055)),
        material=polished,
        name="rear_fork_arm_0",
    )
    deck.visual(
        Box((0.116, 0.014, 0.018)),
        origin=Origin(xyz=(-0.345, -0.041, 0.055)),
        material=polished,
        name="rear_fork_arm_1",
    )
    deck.visual(
        Box((0.033, 0.014, 0.016)),
        origin=Origin(xyz=(-0.296, 0.041, (DECK_BOTTOM + 0.064) / 2.0)),
        material=polished,
        name="rear_hanger_0",
    )
    deck.visual(
        Box((0.033, 0.014, 0.016)),
        origin=Origin(xyz=(-0.296, -0.041, (DECK_BOTTOM + 0.064) / 2.0)),
        material=polished,
        name="rear_hanger_1",
    )
    deck.visual(
        Box((0.028, 0.018, 0.040)),
        origin=Origin(xyz=(REAR_AXLE_X, 0.041, WHEEL_RADIUS)),
        material=dark_metal,
        name="rear_dropout_0",
    )
    deck.visual(
        Box((0.028, 0.018, 0.040)),
        origin=Origin(xyz=(REAR_AXLE_X, -0.041, WHEEL_RADIUS)),
        material=dark_metal,
        name="rear_dropout_1",
    )
    deck.visual(
        Box((0.075, 0.120, 0.023)),
        origin=Origin(xyz=(0.300, 0.0, DECK_TOP + 0.0105)),
        material=polished,
        name="fold_base",
    )
    for y, suffix in ((0.047, "0"), (-0.047, "1")):
        deck.visual(
            Box((0.026, 0.018, 0.064)),
            origin=Origin(xyz=(FRONT_HINGE_X, y, FRONT_HINGE_Z - 0.006)),
            material=polished,
            name=f"fold_cheek_{suffix}",
        )
        deck.visual(
            Cylinder(radius=0.019, length=0.025),
            origin=Origin(xyz=(FRONT_HINGE_X, y, FRONT_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"lower_hinge_knuckle_{suffix}",
        )
    deck.visual(
        Cylinder(radius=0.005, length=0.096),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, WHEEL_RADIUS), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle_pin",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.017, length=0.044),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="upper_hinge_barrel",
    )
    neck.visual(
        Box((0.052, 0.060, 0.055)),
        origin=Origin(xyz=(0.015, 0.0, 0.028)),
        material=polished,
        name="fold_clamp_body",
    )
    neck.visual(
        Box((0.040, 0.052, 0.070)),
        origin=Origin(xyz=(0.020, 0.0, -0.020)),
        material=polished,
        name="fork_neck_web",
    )
    neck.visual(
        Box((0.070, 0.090, 0.018)),
        origin=Origin(xyz=(0.065, 0.0, -0.025)),
        material=polished,
        name="fork_crown",
    )
    for y, suffix in ((0.041, "0"), (-0.041, "1")):
        neck.visual(
            Box((0.018, 0.012, 0.092)),
            origin=Origin(xyz=(FRONT_AXLE_X, y, -0.074)),
            material=polished,
            name=f"front_fork_leg_{suffix}",
        )
        neck.visual(
            Box((0.028, 0.018, 0.040)),
            origin=Origin(xyz=(FRONT_AXLE_X, y, FRONT_AXLE_Z)),
            material=dark_metal,
            name=f"front_dropout_{suffix}",
        )
    neck.visual(
        Cylinder(radius=0.005, length=0.096),
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, FRONT_AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_axle_pin",
    )
    neck.visual(
        Cylinder(radius=0.017, length=0.690),
        origin=Origin(xyz=(0.015, 0.0, 0.390)),
        material=polished,
        name="stem_tube",
    )
    for z, suffix in ((0.090, "lower"), (0.140, "upper")):
        neck.visual(
            Cylinder(radius=0.024, length=0.038),
            origin=Origin(xyz=(0.015, 0.0, z)),
            material=dark_metal,
            name=f"{suffix}_clamp_ring",
        )
        neck.visual(
            Cylinder(radius=0.0045, length=0.032),
            origin=Origin(xyz=(0.043, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=f"{suffix}_pinch_bolt",
        )
    neck.visual(
        Cylinder(radius=0.014, length=0.430),
        origin=Origin(xyz=(0.015, 0.0, 0.742), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="handlebar",
    )
    for y, suffix in ((0.195, "0"), (-0.195, "1")):
        neck.visual(
            Cylinder(radius=0.018, length=0.105),
            origin=Origin(xyz=(0.015, y, 0.742), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name=f"grip_{suffix}",
        )

    front_wheel = model.part("front_wheel")
    front_core, front_tire = _scooter_wheel_meshes("front_wheel")
    front_wheel.visual(
        front_core,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=polished,
        name="wheel_core",
    )
    front_wheel.visual(
        front_tire,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="tire",
    )

    rear_wheel = model.part("rear_wheel")
    rear_core, rear_tire = _scooter_wheel_meshes("rear_wheel")
    rear_wheel.visual(
        rear_core,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=polished,
        name="wheel_core",
    )
    rear_wheel.visual(
        rear_tire,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="tire",
    )

    model.articulation(
        "neck_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=neck,
        origin=Origin(xyz=(FRONT_HINGE_X, 0.0, FRONT_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "front_axle",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=front_wheel,
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, FRONT_AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=30.0, lower=-2.0 * math.pi, upper=2.0 * math.pi),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=30.0, lower=-2.0 * math.pi, upper=2.0 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    neck = object_model.get_part("neck")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    fold = object_model.get_articulation("neck_fold")
    front_axle = object_model.get_articulation("front_axle")
    rear_axle = object_model.get_articulation("rear_axle")

    ctx.check("wide_deck_present", deck is not None, "Expected a wide deck root part.")
    ctx.check("folding_neck_present", neck is not None, "Expected an articulated folding neck.")
    ctx.check("front_wheel_present", front_wheel is not None, "Expected a front wheel part.")
    ctx.check("rear_wheel_present", rear_wheel is not None, "Expected a rear wheel part.")
    ctx.check("fold_joint_present", fold is not None, "Expected a neck fold revolute joint.")
    ctx.check("front_axle_joint_present", front_axle is not None, "Expected a front axle revolute joint.")
    ctx.check("rear_axle_joint_present", rear_axle is not None, "Expected a rear axle revolute joint.")

    if neck is not None and front_wheel is not None:
        ctx.allow_overlap(
            neck,
            front_wheel,
            elem_a="front_axle_pin",
            elem_b="wheel_core",
            reason="The visible front axle pin intentionally passes through the wheel hub bore.",
        )
        ctx.expect_overlap(
            neck,
            front_wheel,
            axes="y",
            min_overlap=0.020,
            elem_a="front_axle_pin",
            elem_b="wheel_core",
            name="front axle pin spans through the hub",
        )
    if deck is not None and rear_wheel is not None:
        ctx.allow_overlap(
            deck,
            rear_wheel,
            elem_a="rear_axle_pin",
            elem_b="wheel_core",
            reason="The visible rear axle pin intentionally passes through the wheel hub bore.",
        )
        ctx.expect_overlap(
            deck,
            rear_wheel,
            axes="y",
            min_overlap=0.020,
            elem_a="rear_axle_pin",
            elem_b="wheel_core",
            name="rear axle pin spans through the hub",
        )

    if deck is not None and neck is not None:
        ctx.expect_gap(
            neck,
            deck,
            axis="z",
            min_gap=0.0,
            max_gap=0.080,
            positive_elem="upper_hinge_barrel",
            negative_elem="fold_base",
            name="fold barrel is mounted above deck clamp base",
        )
        rest_handle_aabb = ctx.part_element_world_aabb(neck, elem="handlebar")
        with ctx.pose({fold: 1.35}):
            folded_handle_aabb = ctx.part_element_world_aabb(neck, elem="handlebar")
            ctx.expect_overlap(
                neck,
                deck,
                axes="x",
                min_overlap=0.15,
                elem_a="stem_tube",
                elem_b="deck_shell",
                name="folded stem lies over the deck length",
            )
            ctx.expect_gap(
                neck,
                deck,
                axis="z",
                min_gap=0.005,
                positive_elem="stem_tube",
                negative_elem="grip_tape",
                name="folded stem clears the grip tape",
            )
        if rest_handle_aabb is not None and folded_handle_aabb is not None:
            rest_handle_x = (float(rest_handle_aabb[0][0]) + float(rest_handle_aabb[1][0])) / 2.0
            folded_handle_x = (float(folded_handle_aabb[0][0]) + float(folded_handle_aabb[1][0])) / 2.0
        else:
            rest_handle_x = None
            folded_handle_x = None
        ctx.check(
            "fold_joint_sweeps_neck_backward",
            rest_handle_x is not None and folded_handle_x is not None and folded_handle_x < rest_handle_x - 0.40,
            details=f"rest_handle_x={rest_handle_x}, folded_handle_x={folded_handle_x}",
        )

    if front_wheel is not None and neck is not None:
        ctx.expect_within(
            front_wheel,
            neck,
            axes="y",
            margin=0.004,
            inner_elem="tire",
            outer_elem="fork_crown",
            name="front wheel sits between fork sides",
        )
    if rear_wheel is not None and deck is not None:
        ctx.expect_overlap(
            rear_wheel,
            deck,
            axes="x",
            min_overlap=0.020,
            elem_a="tire",
            elem_b="rear_fork_arm_0",
            name="rear wheel aligns with rear axle fork",
        )

    return ctx.report()


object_model = build_object_model()
