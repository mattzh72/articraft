from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_handle_platform_trolley")

    steel = model.material("brushed_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_galvanized_steel", rgba=(0.24, 0.25, 0.25, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    blue_grip = model.material("blue_plastic_grip", rgba=(0.02, 0.12, 0.55, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.78, 0.48, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=steel,
        name="steel_deck",
    )
    # Raised perimeter lips and low anti-slip ribs make the load platform read
    # as a pressed steel deck rather than a plain slab.
    deck.visual(
        Box((0.78, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, -0.228, 0.187)),
        material=dark_steel,
        name="side_lip_0",
    )
    deck.visual(
        Box((0.78, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.228, 0.187)),
        material=dark_steel,
        name="side_lip_1",
    )
    deck.visual(
        Box((0.026, 0.48, 0.020)),
        origin=Origin(xyz=(-0.377, 0.0, 0.187)),
        material=dark_steel,
        name="rear_lip",
    )
    deck.visual(
        Box((0.026, 0.48, 0.020)),
        origin=Origin(xyz=(0.377, 0.0, 0.187)),
        material=dark_steel,
        name="front_lip",
    )
    for i, y in enumerate((-0.135, -0.075, -0.015, 0.045, 0.105, 0.165)):
        deck.visual(
            Box((0.62, 0.010, 0.004)),
            origin=Origin(xyz=(0.015, y, 0.179)),
            material=steel,
            name=f"tread_rib_{i}",
        )

    caster_positions = {
        "rear_0": (-0.285, -0.170),
        "rear_1": (-0.285, 0.170),
        "front_0": (0.285, -0.170),
        "front_1": (0.285, 0.170),
    }
    for name, (x, y) in caster_positions.items():
        deck.visual(
            Box((0.112, 0.082, 0.008)),
            origin=Origin(xyz=(x, y, 0.129)),
            material=dark_steel,
            name=f"{name}_mount_plate",
        )

    # Two aligned side hinges carry the folding rear handle.  Each side rail is
    # independently hinged; the second hinge mimics the first so the pair folds
    # as one U-shaped handle in the tree representation.
    hinge_x = -0.352
    hinge_z = 0.218
    hinge_y = (-0.200, 0.200)
    handle_0 = model.part("handle_0")
    handle_0.visual(
        Cylinder(radius=0.014, length=0.710),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=dark_steel,
        name="side_tube",
    )
    handle_0.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    crossbar_length = 0.386
    handle_0.visual(
        Cylinder(radius=0.014, length=crossbar_length),
        origin=Origin(
            xyz=(0.0, crossbar_length / 2.0, 0.704),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="top_crossbar",
    )
    handle_0.visual(
        Cylinder(radius=0.018, length=0.155),
        origin=Origin(xyz=(0.0, 0.194, 0.704), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blue_grip,
        name="hand_grip",
    )

    handle_1 = model.part("handle_1")
    handle_1.visual(
        Cylinder(radius=0.014, length=0.710),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=dark_steel,
        name="side_tube",
    )
    handle_1.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )

    for i, y in enumerate(hinge_y):
        deck.visual(
            Box((0.070, 0.056, 0.012)),
            origin=Origin(xyz=(hinge_x, y, 0.188)),
            material=dark_steel,
            name=f"handle_hinge_plate_{i}",
        )
        deck.visual(
            Box((0.042, 0.040, 0.010)),
            origin=Origin(xyz=(hinge_x, y, 0.196)),
            material=dark_steel,
            name=f"handle_hinge_saddle_{i}",
        )

    hinge_limits = MotionLimits(effort=80.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0)
    primary_hinge = model.articulation(
        "handle_hinge_0",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle_0,
        origin=Origin(xyz=(hinge_x, hinge_y[0], hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "handle_hinge_1",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle_1,
        origin=Origin(xyz=(hinge_x, hinge_y[1], hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=hinge_limits,
        mimic=Mimic(joint=primary_hinge.name, multiplier=1.0, offset=0.0),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.045,
            0.032,
            inner_radius=0.030,
            tread=TireTread(style="block", depth=0.003, count=16, land_ratio=0.62),
            sidewall=TireSidewall(style="square", bulge=0.015),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "small_caster_tire",
    )
    swivel_limits = MotionLimits(effort=25.0, velocity=6.0)
    wheel_spin_limits = MotionLimits(effort=15.0, velocity=30.0)
    for name, (x, y) in caster_positions.items():
        caster = model.part(f"{name}_caster")
        caster.visual(
            Cylinder(radius=0.034, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=dark_steel,
            name="swivel_race",
        )
        caster.visual(
            Cylinder(radius=0.014, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=dark_steel,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.076, 0.050, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.028)),
            material=dark_steel,
            name="fork_bridge",
        )
        caster.visual(
            Box((0.006, 0.044, 0.092)),
            origin=Origin(xyz=(-0.028, 0.0, -0.080)),
            material=dark_steel,
            name="fork_cheek_0",
        )
        caster.visual(
            Box((0.006, 0.044, 0.092)),
            origin=Origin(xyz=(0.028, 0.0, -0.080)),
            material=dark_steel,
            name="fork_cheek_1",
        )
        caster.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(-0.035, 0.0, -0.079), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="axle_cap_0",
        )
        caster.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.035, 0.0, -0.079), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="axle_cap_1",
        )
        model.articulation(
            f"{name}_swivel",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(x, y, 0.125)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=swivel_limits,
        )

        wheel = model.part(f"{name}_wheel")
        wheel.visual(tire_mesh, material=black_rubber, name="rubber_tire")
        wheel.visual(
            Cylinder(radius=0.0305, length=0.036),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="metal_hub",
        )
        model.articulation(
            f"{name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.079)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=wheel_spin_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    h0 = object_model.get_articulation("handle_hinge_0")
    h1 = object_model.get_articulation("handle_hinge_1")
    ctx.check(
        "rear handle hinges share a 90 degree range",
        h0.motion_limits is not None
        and h1.motion_limits is not None
        and abs((h0.motion_limits.upper or 0.0) - math.pi / 2.0) < 0.02
        and abs((h1.motion_limits.upper or 0.0) - math.pi / 2.0) < 0.02
        and h0.axis == (0.0, 1.0, 0.0)
        and h1.axis == (0.0, 1.0, 0.0),
        details=f"h0={h0.motion_limits}, h1={h1.motion_limits}, axes={h0.axis},{h1.axis}",
    )
    ctx.check(
        "rear handle hinge axes are aligned across the deck",
        abs(h0.origin.xyz[0] - h1.origin.xyz[0]) < 1e-6
        and abs(h0.origin.xyz[2] - h1.origin.xyz[2]) < 1e-6
        and abs(h0.origin.xyz[1] - h1.origin.xyz[1]) > 0.35,
        details=f"origins={h0.origin.xyz}, {h1.origin.xyz}",
    )

    handle_0 = object_model.get_part("handle_0")
    deck = object_model.get_part("deck")
    upright_aabb = ctx.part_world_aabb(handle_0)
    with ctx.pose({h0: math.pi / 2.0}):
        folded_aabb = ctx.part_world_aabb(handle_0)
    ctx.check(
        "folding handle moves from upright to flat over the deck",
        upright_aabb is not None
        and folded_aabb is not None
        and (upright_aabb[1][2] - upright_aabb[0][2]) > 0.65
        and (folded_aabb[1][2] - folded_aabb[0][2]) < 0.08
        and folded_aabb[1][0] > 0.25,
        details=f"upright={upright_aabb}, folded={folded_aabb}",
    )

    for name in ("rear_0", "rear_1", "front_0", "front_1"):
        swivel = object_model.get_articulation(f"{name}_swivel")
        spin = object_model.get_articulation(f"{name}_wheel_spin")
        wheel = object_model.get_part(f"{name}_wheel")
        ctx.check(
            f"{name} caster swivels and wheel spins",
            swivel.articulation_type == ArticulationType.CONTINUOUS
            and spin.articulation_type == ArticulationType.CONTINUOUS
            and swivel.axis == (0.0, 0.0, 1.0)
            and spin.axis == (1.0, 0.0, 0.0),
            details=f"swivel={swivel.articulation_type},{swivel.axis}; spin={spin.articulation_type},{spin.axis}",
        )
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.020,
            max_gap=0.060,
            name=f"{name} wheel sits below the steel deck",
        )

    return ctx.report()


object_model = build_object_model()
