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


STEEL = Material("satin_black_steel", color=(0.02, 0.025, 0.025, 1.0))
DECK_PAINT = Material("matte_olive_frame", color=(0.20, 0.28, 0.16, 1.0))
GRIP = Material("coarse_black_grip", color=(0.015, 0.015, 0.014, 1.0))
RUBBER = Material("dark_pneumatic_rubber", color=(0.005, 0.005, 0.005, 1.0))
ALLOY = Material("brushed_aluminum", color=(0.72, 0.72, 0.68, 1.0))
ORANGE = Material("safety_orange_reflector", color=(1.0, 0.36, 0.05, 1.0))


def _offroad_wheel_meshes(prefix: str):
    tire = TireGeometry(
        0.180,
        0.085,
        inner_radius=0.126,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.07),
        tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.52),
        grooves=(
            TireGroove(center_offset=-0.020, width=0.006, depth=0.003),
            TireGroove(center_offset=0.020, width=0.006, depth=0.003),
        ),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )
    wheel = WheelGeometry(
        0.132,
        0.070,
        rim=WheelRim(
            inner_radius=0.083,
            flange_height=0.010,
            flange_thickness=0.005,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(
            radius=0.034,
            width=0.052,
            cap_style="domed",
            bolt_pattern=BoltPattern(
                count=6,
                circle_diameter=0.044,
                hole_diameter=0.004,
            ),
        ),
        face=WheelFace(dish_depth=0.008, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=6, thickness=0.004, window_radius=0.013),
        bore=WheelBore(style="round", diameter=0.014),
    )
    return (
        mesh_from_geometry(tire, f"{prefix}_block_tire"),
        mesh_from_geometry(wheel, f"{prefix}_spoked_rim"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="off_road_kick_scooter")

    deck = model.part("deck")
    deck.visual(
        Box((0.84, 0.18, 0.055)),
        origin=Origin(xyz=(0.08, 0.0, 0.250)),
        material=DECK_PAINT,
        name="deck_shell",
    )
    deck.visual(
        Box((0.72, 0.135, 0.008)),
        origin=Origin(xyz=(0.10, 0.0, 0.284)),
        material=GRIP,
        name="grip_tape",
    )
    deck.visual(
        Box((0.73, 0.018, 0.035)),
        origin=Origin(xyz=(0.08, 0.103, 0.260)),
        material=STEEL,
        name="side_rail_0",
    )
    deck.visual(
        Box((0.73, 0.018, 0.035)),
        origin=Origin(xyz=(0.08, -0.103, 0.260)),
        material=STEEL,
        name="side_rail_1",
    )
    deck.visual(
        Cylinder(radius=0.055, length=0.210),
        origin=Origin(xyz=(0.500, 0.0, 0.535)),
        material=STEEL,
        name="head_tube",
    )
    deck.visual(
        Box((0.070, 0.050, 0.158)),
        origin=Origin(xyz=(0.420, 0.0, 0.354)),
        material=DECK_PAINT,
        name="steering_gusset",
    )
    deck.visual(
        Box((0.090, 0.043, 0.036)),
        origin=Origin(xyz=(-0.050, -0.105, 0.226)),
        material=STEEL,
        name="hinge_lug",
    )
    deck.visual(
        Box((0.055, 0.040, 0.040)),
        origin=Origin(xyz=(0.430, 0.110, 0.260)),
        material=ORANGE,
        name="front_reflector",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.028, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=STEEL,
        name="steerer",
    )
    front_fork.visual(
        Box((0.050, 0.050, 0.275)),
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
        material=STEEL,
        name="handlebar_stem",
    )
    front_fork.visual(
        Cylinder(radius=0.023, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.945), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="handlebar",
    )
    front_fork.visual(
        Box((0.050, 0.065, 0.050)),
        origin=Origin(xyz=(0.0, 0.325, 0.945)),
        material=GRIP,
        name="grip_0",
    )
    front_fork.visual(
        Box((0.050, 0.065, 0.050)),
        origin=Origin(xyz=(0.0, -0.325, 0.945)),
        material=GRIP,
        name="grip_1",
    )
    front_fork.visual(
        Box((0.220, 0.180, 0.045)),
        origin=Origin(xyz=(0.080, 0.0, 0.012)),
        material=STEEL,
        name="fork_crown",
    )
    for index, y in enumerate((-0.064, 0.064)):
        front_fork.visual(
            Box((0.055, 0.026, 0.220)),
            origin=Origin(xyz=(0.205, y, -0.085)),
            material=STEEL,
            name=f"fork_leg_{index}",
        )
        front_fork.visual(
            Box((0.074, 0.035, 0.060)),
            origin=Origin(xyz=(0.220, y, -0.210)),
            material=STEEL,
            name=f"dropout_{index}",
        )

    front_wheel = model.part("front_wheel")
    front_tire, front_rim = _offroad_wheel_meshes("front")
    wheel_visual_origin = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    front_wheel.visual(front_tire, origin=wheel_visual_origin, material=RUBBER, name="tire")
    front_wheel.visual(front_rim, origin=wheel_visual_origin, material=ALLOY, name="rim")
    front_wheel.visual(
        Cylinder(radius=0.014, length=0.155),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ALLOY,
        name="axle",
    )

    rear_bracket = model.part("rear_bracket")
    for index, y in enumerate((-0.064, 0.064)):
        rear_bracket.visual(
            Box((0.230, 0.026, 0.050)),
            origin=Origin(xyz=(0.035, y * 1.95, 0.070)),
            material=STEEL,
            name=f"swing_arm_{index}",
        )
        rear_bracket.visual(
            Box((0.052, 0.036, 0.090)),
            origin=Origin(xyz=(-0.065, y * 1.95, 0.025)),
            material=STEEL,
            name=f"rear_dropout_{index}",
        )
    rear_bracket.visual(
        Box((0.040, 0.280, 0.060)),
        origin=Origin(xyz=(0.140, 0.0, 0.070)),
        material=STEEL,
        name="deck_bridge",
    )
    rear_bracket.visual(
        Box((0.055, 0.040, 0.040)),
        origin=Origin(xyz=(0.140, -0.158, 0.070)),
        material=ORANGE,
        name="rear_reflector",
    )

    rear_wheel = model.part("rear_wheel")
    rear_tire, rear_rim = _offroad_wheel_meshes("rear")
    rear_wheel.visual(rear_tire, origin=wheel_visual_origin, material=RUBBER, name="tire")
    rear_wheel.visual(rear_rim, origin=wheel_visual_origin, material=ALLOY, name="rim")
    rear_wheel.visual(
        Cylinder(radius=0.014, length=0.285),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ALLOY,
        name="axle",
    )

    side_stand = model.part("side_stand")
    side_stand.visual(
        Cylinder(radius=0.017, length=0.086),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="hinge_barrel",
    )
    side_stand.visual(
        Cylinder(radius=0.012, length=0.270),
        origin=Origin(xyz=(0.0, -0.1000, -0.1140), rpy=(2.430, 0.0, 0.0)),
        material=STEEL,
        name="stand_leg",
    )
    side_stand.visual(
        Box((0.115, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, -0.188, -0.214)),
        material=STEEL,
        name="foot_pad",
    )

    model.articulation(
        "deck_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(0.500, 0.0, 0.390)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "front_fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.220, 0.0, -0.210)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=30.0),
    )
    model.articulation(
        "deck_to_rear_bracket",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_bracket,
        origin=Origin(xyz=(-0.500, 0.0, 0.180)),
    )
    model.articulation(
        "rear_bracket_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=rear_bracket,
        child=rear_wheel,
        origin=Origin(xyz=(-0.065, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=30.0),
    )
    model.articulation(
        "deck_to_side_stand",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=side_stand,
        origin=Origin(xyz=(-0.050, -0.123, 0.226)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_bracket = object_model.get_part("rear_bracket")
    rear_wheel = object_model.get_part("rear_wheel")
    side_stand = object_model.get_part("side_stand")
    steer = object_model.get_articulation("deck_to_front_fork")
    stand_hinge = object_model.get_articulation("deck_to_side_stand")

    ctx.allow_overlap(
        deck,
        front_fork,
        elem_a="head_tube",
        elem_b="steerer",
        reason="The steerable fork steerer is intentionally captured inside the fixed head tube bearing.",
    )
    ctx.expect_within(
        front_fork,
        deck,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_tube",
        margin=0.002,
        name="steerer is centered in the head tube",
    )
    ctx.expect_overlap(
        front_fork,
        deck,
        axes="z",
        elem_a="steerer",
        elem_b="head_tube",
        min_overlap=0.050,
        name="steerer remains captured by head tube",
    )

    ctx.allow_overlap(
        deck,
        side_stand,
        elem_a="hinge_lug",
        elem_b="hinge_barrel",
        reason="The side stand barrel is represented as a captured pin inside the under-deck hinge lug.",
    )
    ctx.expect_overlap(
        side_stand,
        deck,
        axes="xz",
        elem_a="hinge_barrel",
        elem_b="hinge_lug",
        min_overlap=0.020,
        name="side stand barrel is retained in hinge lug",
    )
    ctx.expect_gap(
        deck,
        side_stand,
        axis="y",
        positive_elem="hinge_lug",
        negative_elem="hinge_barrel",
        max_penetration=0.022,
        name="side stand hinge embed is local",
    )

    for dropout_name in ("dropout_0", "dropout_1"):
        ctx.allow_overlap(
            front_fork,
            front_wheel,
            elem_a=dropout_name,
            elem_b="axle",
            reason="The front wheel axle is intentionally captured through the fork dropout slot.",
        )
        ctx.expect_within(
            front_wheel,
            front_fork,
            axes="xz",
            inner_elem="axle",
            outer_elem=dropout_name,
            margin=0.001,
            name=f"front axle is seated in {dropout_name}",
        )
        ctx.expect_overlap(
            front_wheel,
            front_fork,
            axes="y",
            elem_a="axle",
            elem_b=dropout_name,
            min_overlap=0.020,
            name=f"front axle passes through {dropout_name}",
        )

    for dropout_name in ("rear_dropout_0", "rear_dropout_1"):
        ctx.allow_overlap(
            rear_bracket,
            rear_wheel,
            elem_a=dropout_name,
            elem_b="axle",
            reason="The rear wheel axle is intentionally captured through the rear swing bracket dropout slot.",
        )
        ctx.expect_within(
            rear_wheel,
            rear_bracket,
            axes="xz",
            inner_elem="axle",
            outer_elem=dropout_name,
            margin=0.001,
            name=f"rear axle is seated in {dropout_name}",
        )
        ctx.expect_overlap(
            rear_wheel,
            rear_bracket,
            axes="y",
            elem_a="axle",
            elem_b=dropout_name,
            min_overlap=0.020,
            name=f"rear axle passes through {dropout_name}",
        )

    ctx.expect_overlap(
        front_wheel,
        front_fork,
        axes="z",
        elem_a="tire",
        elem_b="fork_leg_0",
        min_overlap=0.15,
        name="front fork legs span the wheel height",
    )
    ctx.expect_overlap(
        rear_wheel,
        rear_bracket,
        axes="z",
        elem_a="tire",
        elem_b="swing_arm_0",
        min_overlap=0.04,
        name="rear swing bracket sits over wheel axle",
    )
    ctx.expect_gap(
        deck,
        rear_bracket,
        axis="x",
        positive_elem="deck_shell",
        negative_elem="deck_bridge",
        min_gap=-0.001,
        max_gap=0.003,
        name="rear bracket bridge meets deck end",
    )

    rest_front = ctx.part_world_position(front_wheel)
    with ctx.pose({steer: 0.55}):
        steered_front = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork yaws wheel about vertical steering axis",
        rest_front is not None
        and steered_front is not None
        and steered_front[1] > rest_front[1] + 0.035,
        details=f"rest={rest_front}, steered={steered_front}",
    )

    rest_stand_aabb = ctx.part_world_aabb(side_stand)
    with ctx.pose({stand_hinge: 1.20}):
        folded_stand_aabb = ctx.part_world_aabb(side_stand)
    ctx.check(
        "side stand folds upward under deck",
        rest_stand_aabb is not None
        and folded_stand_aabb is not None
        and folded_stand_aabb[0][2] > rest_stand_aabb[0][2] + 0.10,
        details=f"deployed={rest_stand_aabb}, folded={folded_stand_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
