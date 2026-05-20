from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
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
    rounded_rect_profile,
)


DECK_LENGTH = 1.10
DECK_WIDTH = 0.32
DECK_THICKNESS = 0.065
WHEEL_RADIUS = 0.105
WHEEL_WIDTH = 0.060


def _wheel_and_tire(part, name_prefix: str, rim_material, tire_material) -> None:
    """Add a realistic scooter wheel centered on the part origin.

    Wheel helpers spin about local X; the axle joints rotate the child frame so
    local X becomes the world/parent Y axle direction.
    """

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.074,
            WHEEL_WIDTH * 0.82,
            rim=WheelRim(
                inner_radius=0.050,
                flange_height=0.004,
                flange_thickness=0.003,
                bead_seat_depth=0.003,
            ),
            hub=WheelHub(radius=0.026, width=0.044, cap_style="domed"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.003, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.012),
        ),
        f"{name_prefix}_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.073,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.0045, count=22, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        f"{name_prefix}_tire",
    )
    part.visual(tire_mesh, material=tire_material, name="tire")
    part.visual(wheel_mesh, material=rim_material, name="rim")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_deck_electric_scooter")

    deck_mat = model.material("satin_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    grip_mat = model.material("textured_black_rubber", rgba=(0.015, 0.016, 0.016, 1.0))
    tire_mat = model.material("matte_black_tire", rgba=(0.005, 0.005, 0.004, 1.0))
    rim_mat = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    metal_mat = model.material("dark_anodized_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    hinge_mat = model.material("gunmetal_hinge", rgba=(0.18, 0.19, 0.20, 1.0))
    screen_mat = model.material("glossy_display_glass", rgba=(0.02, 0.06, 0.09, 1.0))
    accent_mat = model.material("cool_blue_display", rgba=(0.0, 0.45, 0.85, 1.0))

    deck = model.part("deck")
    deck_profile = rounded_rect_profile(DECK_LENGTH, DECK_WIDTH, 0.075, corner_segments=12)
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(deck_profile, DECK_THICKNESS),
            "wide_rounded_deck",
        ),
        material=deck_mat,
        name="deck_shell",
    )

    grip_profile = rounded_rect_profile(0.96, 0.245, 0.040, corner_segments=10)
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(grip_profile, 0.006),
            "rubber_grip_pad",
        ),
        origin=Origin(xyz=(-0.030, 0.0, DECK_THICKNESS / 2 + 0.002)),
        material=grip_mat,
        name="grip_pad",
    )
    for i, y in enumerate((-0.090, -0.060, -0.030, 0.0, 0.030, 0.060, 0.090)):
        deck.visual(
            Box((0.88, 0.009, 0.005)),
            origin=Origin(xyz=(-0.035, y, DECK_THICKNESS / 2 + 0.006)),
            material=grip_mat,
            name=f"grip_rib_{i}",
        )

    # Front folding-stem hinge yoke, fixed to the front top of the deck.
    for y in (-0.130, 0.130):
        deck.visual(
            Box((0.078, 0.035, 0.056)),
            origin=Origin(xyz=(0.490, y, 0.059)),
            material=hinge_mat,
            name=f"front_yoke_{'neg' if y < 0 else 'pos'}",
        )
    deck.visual(
        Box((0.125, 0.210, 0.010)),
        origin=Origin(xyz=(0.482, 0.0, DECK_THICKNESS / 2 + 0.006)),
        material=hinge_mat,
        name="front_hinge_plate",
    )
    deck.visual(
        Cylinder(radius=0.010, length=0.300),
        origin=Origin(xyz=(0.490, 0.0, 0.075), rpy=(math.pi / 2, 0.0, 0.0)),
        material=rim_mat,
        name="front_hinge_pin",
    )

    # Rear suspension hinge brackets hang just under the rear of the deck.
    for y in (-0.130, 0.130):
        deck.visual(
            Box((0.085, 0.035, 0.055)),
            origin=Origin(xyz=(-0.535, y, -0.055)),
            material=hinge_mat,
            name=f"rear_yoke_{'neg' if y < 0 else 'pos'}",
        )
    deck.visual(
        Box((0.150, 0.240, 0.012)),
        origin=Origin(xyz=(-0.500, 0.0, -0.024)),
        material=hinge_mat,
        name="rear_hinge_mount",
    )
    deck.visual(
        Cylinder(radius=0.009, length=0.300),
        origin=Origin(xyz=(-0.535, 0.0, -0.055), rpy=(math.pi / 2, 0.0, 0.0)),
        material=rim_mat,
        name="rear_hinge_pin",
    )

    stem = model.part("stem")
    # Hinge barrel and crown live in the folding stem child, around the joint origin.
    stem.visual(
        Cylinder(radius=0.025, length=0.180),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=hinge_mat,
        name="fold_barrel",
    )
    stem.visual(
        Box((0.200, 0.205, 0.050)),
        origin=Origin(xyz=(0.120, 0.0, 0.005)),
        material=metal_mat,
        name="fork_crown",
    )
    stem.visual(
        Cylinder(radius=0.024, length=0.920),
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
        material=metal_mat,
        name="upright_tube",
    )
    for y in (-0.075, 0.075):
        stem.visual(
            Box((0.030, 0.026, 0.145)),
            origin=Origin(xyz=(0.200, y, -0.065)),
            material=metal_mat,
            name=f"fork_blade_{'neg' if y < 0 else 'pos'}",
        )
    stem.visual(
        Cylinder(radius=0.007, length=0.170),
        origin=Origin(xyz=(0.200, 0.0, -0.130), rpy=(math.pi / 2, 0.0, 0.0)),
        material=rim_mat,
        name="front_axle_pin",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.905), rpy=(math.pi / 2, 0.0, 0.0)),
        material=metal_mat,
        name="handlebar",
    )
    for y in (-0.315, 0.315):
        stem.visual(
            Cylinder(radius=0.024, length=0.130),
            origin=Origin(xyz=(0.0, y, 0.905), rpy=(math.pi / 2, 0.0, 0.0)),
            material=grip_mat,
            name=f"grip_{'neg' if y < 0 else 'pos'}",
        )
    stem.visual(
        Box((0.065, 0.150, 0.038)),
        origin=Origin(xyz=(-0.030, 0.0, 0.830)),
        material=metal_mat,
        name="display_housing",
    )
    stem.visual(
        Box((0.006, 0.118, 0.024)),
        origin=Origin(xyz=(-0.065, 0.0, 0.835)),
        material=screen_mat,
        name="display_glass",
    )
    stem.visual(
        Box((0.007, 0.045, 0.006)),
        origin=Origin(xyz=(-0.069, 0.0, 0.842)),
        material=accent_mat,
        name="display_readout",
    )

    front_wheel = model.part("front_wheel")
    _wheel_and_tire(front_wheel, "front", rim_mat, tire_mat)

    swing_arm = model.part("swing_arm")
    swing_arm.visual(
        Cylinder(radius=0.020, length=0.210),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=hinge_mat,
        name="pivot_barrel",
    )
    swing_arm.visual(
        Cylinder(radius=0.007, length=0.170),
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=rim_mat,
        name="rear_axle_pin",
    )
    for y in (-0.085, 0.085):
        swing_arm.visual(
            Box((0.230, 0.028, 0.032)),
            origin=Origin(xyz=(-0.105, y, -0.023)),
            material=metal_mat,
            name=f"arm_rail_{'neg' if y < 0 else 'pos'}",
        )
        swing_arm.visual(
            Box((0.026, 0.026, 0.150)),
            origin=Origin(xyz=(-0.200, y, 0.045)),
            material=metal_mat,
            name=f"fender_strut_{'neg' if y < 0 else 'pos'}",
        )
    swing_arm.visual(
        Box((0.245, 0.235, 0.014)),
        origin=Origin(xyz=(-0.205, 0.0, 0.123)),
        material=grip_mat,
        name="rear_fender",
    )

    rear_wheel = model.part("rear_wheel")
    _wheel_and_tire(rear_wheel, "rear", rim_mat, tire_mat)

    model.articulation(
        "deck_to_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(0.490, 0.0, 0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "stem_to_front_wheel",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=front_wheel,
        origin=Origin(xyz=(0.200, 0.0, -0.130), rpy=(0.0, 0.0, math.pi / 2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "deck_to_swing_arm",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=swing_arm,
        origin=Origin(xyz=(-0.535, 0.0, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0, lower=-0.20, upper=0.35),
    )
    model.articulation(
        "swing_arm_to_rear_wheel",
        ArticulationType.REVOLUTE,
        parent=swing_arm,
        child=rear_wheel,
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(0.0, 0.0, math.pi / 2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    swing_arm = object_model.get_part("swing_arm")
    rear_wheel = object_model.get_part("rear_wheel")

    stem_hinge = object_model.get_articulation("deck_to_stem")
    front_axle = object_model.get_articulation("stem_to_front_wheel")
    swing_hinge = object_model.get_articulation("deck_to_swing_arm")
    rear_axle = object_model.get_articulation("swing_arm_to_rear_wheel")

    ctx.allow_overlap(
        deck,
        stem,
        elem_a="front_hinge_pin",
        elem_b="fold_barrel",
        reason="The folding-stem hinge pin is intentionally captured inside the stem barrel.",
    )
    ctx.allow_overlap(
        deck,
        swing_arm,
        elem_a="rear_hinge_pin",
        elem_b="pivot_barrel",
        reason="The rear suspension pivot pin is intentionally captured inside the swing-arm barrel.",
    )
    ctx.allow_overlap(
        stem,
        front_wheel,
        elem_a="front_axle_pin",
        elem_b="rim",
        reason="The front axle pin intentionally passes through the wheel hub bore.",
    )
    ctx.allow_overlap(
        swing_arm,
        rear_wheel,
        elem_a="rear_axle_pin",
        elem_b="rim",
        reason="The rear axle pin intentionally passes through the wheel hub bore.",
    )

    ctx.expect_overlap(
        deck,
        stem,
        axes="y",
        min_overlap=0.150,
        elem_a="front_hinge_pin",
        elem_b="fold_barrel",
        name="front hinge pin spans the folding barrel",
    )
    ctx.expect_overlap(
        deck,
        swing_arm,
        axes="y",
        min_overlap=0.180,
        elem_a="rear_hinge_pin",
        elem_b="pivot_barrel",
        name="rear pivot pin spans the swing-arm barrel",
    )
    ctx.expect_overlap(
        stem,
        front_wheel,
        axes="y",
        min_overlap=0.040,
        elem_a="front_axle_pin",
        elem_b="rim",
        name="front axle pin crosses the wheel hub",
    )
    ctx.expect_overlap(
        swing_arm,
        rear_wheel,
        axes="y",
        min_overlap=0.040,
        elem_a="rear_axle_pin",
        elem_b="rim",
        name="rear axle pin crosses the wheel hub",
    )

    for joint, label in (
        (front_axle, "front wheel has a revolute axle"),
        (rear_axle, "rear wheel has a revolute axle"),
        (stem_hinge, "stem folds on a revolute hinge"),
        (swing_hinge, "rear swing arm pivots on a revolute hinge"),
    ):
        ctx.check(
            label,
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint.name} type={joint.articulation_type}",
        )

    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        min_gap=0.004,
        name="front wheel clears the front of the deck",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        min_gap=0.020,
        name="rear wheel clears the rear of the deck",
    )

    upright_aabb = ctx.part_world_aabb(stem)
    with ctx.pose({stem_hinge: stem_hinge.motion_limits.upper}):
        folded_aabb = ctx.part_world_aabb(stem)
    ctx.check(
        "folding stem lowers over the deck",
        upright_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < upright_aabb[1][2] - 0.25,
        details=f"upright={upright_aabb}, folded={folded_aabb}",
    )

    rear_rest = ctx.part_world_position(rear_wheel)
    with ctx.pose({swing_hinge: swing_hinge.motion_limits.upper}):
        rear_compressed = ctx.part_world_position(rear_wheel)
    ctx.check(
        "swing arm compression lifts the rear axle",
        rear_rest is not None
        and rear_compressed is not None
        and rear_compressed[2] > rear_rest[2] + 0.045,
        details=f"rest={rear_rest}, compressed={rear_compressed}",
    )

    return ctx.report()


object_model = build_object_model()
