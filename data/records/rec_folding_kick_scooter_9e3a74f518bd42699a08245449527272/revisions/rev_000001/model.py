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


WHEEL_RADIUS = 0.072
WHEEL_WIDTH = 0.034
WHEEL_Z = WHEEL_RADIUS
FRONT_WHEEL_X = 0.49
REAR_WHEEL_X = -0.49
HINGE_X = 0.38
HINGE_Z = 0.170
STEM_FOLDED = math.pi / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_adult_kick_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    deck_paint = model.material("matte_graphite_deck", rgba=(0.16, 0.17, 0.18, 1.0))
    grip = model.material("black_rubber_grip", rgba=(0.015, 0.015, 0.014, 1.0))
    tire_rubber = model.material("dark_polyurethane_tire", rgba=(0.025, 0.026, 0.028, 1.0))
    rim_finish = model.material("silver_wheel_rim", rgba=(0.82, 0.84, 0.86, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.72, 0.14, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=deck_paint,
        name="deck_plate",
    )
    deck.visual(
        Box((0.56, 0.118, 0.005)),
        origin=Origin(xyz=(-0.015, 0.0, 0.125)),
        material=grip,
        name="grip_tape",
    )
    deck.visual(
        Box((0.11, 0.12, 0.028)),
        origin=Origin(xyz=(0.355, 0.0, 0.132)),
        material=dark_metal,
        name="front_hinge_base",
    )
    for y in (-0.055, 0.055):
        deck.visual(
            Box((0.18, 0.026, 0.026)),
            origin=Origin(xyz=(0.405, y, 0.158)),
            material=dark_metal,
            name=f"front_crown_{'negative' if y < 0 else 'positive'}",
        )
    for y in (-0.062, 0.062):
        deck.visual(
            Box((0.070, 0.014, 0.060)),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z)),
            material=dark_metal,
            name=f"hinge_cheek_{'negative' if y < 0 else 'positive'}",
        )
    deck.visual(
        Cylinder(radius=0.006, length=0.160),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="fold_hinge_pin",
    )

    # Front fork: side plates straddle the narrow in-line wheel and connect to the crown.
    for y in (-0.055, 0.055):
        deck.visual(
            Box((0.018, 0.012, 0.115)),
            origin=Origin(xyz=(FRONT_WHEEL_X, y, 0.112)),
            material=dark_metal,
            name=f"front_fork_arm_{'negative' if y < 0 else 'positive'}",
        )
    deck.visual(
        Cylinder(radius=0.007, length=0.130),
        origin=Origin(xyz=(FRONT_WHEEL_X, 0.0, WHEEL_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_axle_pin",
    )

    # Rear fork arms are low, deck-mounted yoke plates around the rear wheel.
    for y in (-0.055, 0.055):
        deck.visual(
            Box((0.165, 0.012, 0.026)),
            origin=Origin(xyz=(-0.432, y, 0.076)),
            material=dark_metal,
            name=f"rear_fork_arm_{'negative' if y < 0 else 'positive'}",
        )
    deck.visual(
        Cylinder(radius=0.007, length=0.130),
        origin=Origin(xyz=(REAR_WHEEL_X, 0.0, WHEEL_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="rear_axle_pin",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.018, length=0.056),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    stem.visual(
        Box((0.044, 0.052, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_metal,
        name="lower_clamp",
    )
    stem.visual(
        Cylinder(radius=0.016, length=0.800),
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        material=aluminum,
        name="upright_tube",
    )
    for z in (0.225, 0.545):
        stem.visual(
            Cylinder(radius=0.021, length=0.038),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_metal,
            name=f"stem_collar_{int(z * 1000)}",
        )
    stem.visual(
        Cylinder(radius=0.0165, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.815), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="t_bar",
    )
    for y in (-0.225, 0.225):
        stem.visual(
            Cylinder(radius=0.020, length=0.145),
            origin=Origin(xyz=(0.0, y, 0.815), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grip,
            name=f"handle_grip_{'negative' if y < 0 else 'positive'}",
        )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            WHEEL_WIDTH - 0.004,
            rim=WheelRim(inner_radius=0.035, flange_height=0.005, flange_thickness=0.0025),
            hub=WheelHub(
                radius=0.017,
                width=0.023,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.024, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.0035, front_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0028, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "kick_scooter_wheel_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.052,
            tread=TireTread(style="circumferential", depth=0.0022, count=3),
            grooves=(TireGroove(center_offset=0.0, width=0.0035, depth=0.0018),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
        ),
        "kick_scooter_polyurethane_tire",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(wheel_mesh, material=rim_finish, name="wheel_core")
    front_wheel.visual(tire_mesh, material=tire_rubber, name="tire")

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(wheel_mesh, material=rim_finish, name="wheel_core")
    rear_wheel.visual(tire_mesh, material=tire_rubber, name="tire")

    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0, lower=0.0, upper=STEM_FOLDED),
    )
    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_wheel,
        origin=Origin(xyz=(FRONT_WHEEL_X, 0.0, WHEEL_Z), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, 0.0, WHEEL_Z), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    fold_hinge = object_model.get_articulation("fold_hinge")
    front_axle = object_model.get_articulation("front_axle")
    rear_axle = object_model.get_articulation("rear_axle")

    ctx.allow_overlap(
        deck,
        stem,
        elem_a="fold_hinge_pin",
        elem_b="hinge_barrel",
        reason="The folding hinge pin is intentionally captured through the stem barrel.",
    )
    for wheel, axle_elem in ((front_wheel, "front_axle_pin"), (rear_wheel, "rear_axle_pin")):
        ctx.allow_overlap(
            deck,
            wheel,
            elem_a=axle_elem,
            elem_b="wheel_core",
            reason="The axle pin intentionally passes through the wheel hub to make a captured revolute axle.",
        )
        ctx.expect_within(
            deck,
            wheel,
            axes="xz",
            inner_elem=axle_elem,
            outer_elem="wheel_core",
            margin=0.001,
            name=f"{axle_elem} is centered in wheel hub",
        )
        ctx.expect_overlap(
            deck,
            wheel,
            axes="y",
            elem_a=axle_elem,
            elem_b="wheel_core",
            min_overlap=0.030,
            name=f"{axle_elem} spans through wheel hub",
        )
    ctx.expect_within(
        deck,
        stem,
        axes="xz",
        inner_elem="fold_hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.003,
        name="fold hinge pin is centered inside stem barrel",
    )
    ctx.expect_overlap(
        deck,
        stem,
        axes="y",
        elem_a="fold_hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.050,
        name="fold hinge pin spans through the stem barrel",
    )

    ctx.check(
        "stem hinge folds a right angle",
        fold_hinge.motion_limits is not None
        and fold_hinge.motion_limits.lower == 0.0
        and abs(float(fold_hinge.motion_limits.upper) - STEM_FOLDED) < 1.0e-6,
        details=f"limits={fold_hinge.motion_limits!r}",
    )
    ctx.check(
        "front and rear wheels spin on revolute axes",
        front_axle.articulation_type == ArticulationType.CONTINUOUS
        and rear_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"front={front_axle.articulation_type!r}, rear={rear_axle.articulation_type!r}",
    )
    ctx.expect_origin_gap(front_wheel, rear_wheel, axis="x", min_gap=0.90, name="two in-line wheels are separated along deck")
    ctx.expect_origin_gap(stem, deck, axis="z", min_gap=0.02, name="upright T-bar rises above deck hinge")

    upright_aabb = ctx.part_world_aabb(stem)
    with ctx.pose({fold_hinge: STEM_FOLDED}):
        folded_aabb = ctx.part_world_aabb(stem)

    if upright_aabb is not None and folded_aabb is not None:
        upright_height = float(upright_aabb[1][2] - upright_aabb[0][2])
        folded_height = float(folded_aabb[1][2] - folded_aabb[0][2])
        folded_length = float(folded_aabb[1][0] - folded_aabb[0][0])
        ctx.check("upright adult T-bar height", upright_height > 0.80, details=f"height={upright_height:.3f}")
        ctx.check("folded stem is low and flat", folded_height < 0.075, details=f"height={folded_height:.3f}")
        ctx.check("folded stem lies along the deck", folded_length > 0.72, details=f"length={folded_length:.3f}")
    else:
        ctx.fail("stem pose AABBs available", f"upright={upright_aabb!r}, folded={folded_aabb!r}")

    return ctx.report()


object_model = build_object_model()
