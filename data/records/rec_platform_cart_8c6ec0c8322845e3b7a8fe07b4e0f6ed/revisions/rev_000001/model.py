from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    ClevisBracketGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
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
    model = ArticulatedObject(name="folding_platform_cart")

    blue = model.material("blue_powder_coat", rgba=(0.05, 0.22, 0.62, 1.0))
    dark_blue = model.material("dark_blue_edge", rgba=(0.03, 0.10, 0.28, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    tread_grey = model.material("ribbed_grey_rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    light_metal = model.material("light_metal", rgba=(0.82, 0.84, 0.82, 1.0))

    deck_length = 0.95
    deck_width = 0.56
    deck_thickness = 0.045
    deck_center_z = 0.2025
    deck_top_z = deck_center_z + deck_thickness / 2.0
    deck_bottom_z = deck_center_z - deck_thickness / 2.0

    deck = model.part("deck")
    deck.visual(
        Box((deck_length, deck_width, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_center_z)),
        material=blue,
        name="deck_pan",
    )
    deck.visual(
        Box((0.88, 0.48, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, deck_top_z + 0.003)),
        material=tread_grey,
        name="ribbed_mat",
    )
    deck.visual(
        Box((deck_length + 0.035, 0.025, 0.050)),
        origin=Origin(xyz=(0.0, deck_width / 2.0 + 0.005, deck_top_z - 0.002)),
        material=dark_blue,
        name="side_rail_0",
    )
    deck.visual(
        Box((deck_length + 0.035, 0.025, 0.050)),
        origin=Origin(xyz=(0.0, -deck_width / 2.0 - 0.005, deck_top_z - 0.002)),
        material=dark_blue,
        name="side_rail_1",
    )
    deck.visual(
        Box((0.030, deck_width + 0.035, 0.050)),
        origin=Origin(xyz=(deck_length / 2.0 + 0.004, 0.0, deck_top_z - 0.002)),
        material=dark_blue,
        name="front_rail",
    )
    deck.visual(
        Box((0.030, deck_width + 0.035, 0.050)),
        origin=Origin(xyz=(-deck_length / 2.0 - 0.004, 0.0, deck_top_z - 0.002)),
        material=dark_blue,
        name="rear_rail",
    )
    for y in (-0.10, 0.10):
        deck.visual(
            Box((0.60, 0.030, 0.030)),
            origin=Origin(xyz=(0.0, y, deck_bottom_z - 0.010)),
            material=galvanized,
            name=f"underside_rib_{0 if y < 0 else 1}",
        )

    caster_x = deck_length / 2.0 - 0.090
    caster_y = deck_width / 2.0 - 0.085
    caster_positions = {
        "front_left": (caster_x, caster_y),
        "front_right": (caster_x, -caster_y),
        "rear_left": (-caster_x, caster_y),
        "rear_right": (-caster_x, -caster_y),
    }
    mount_bottom_z = deck_bottom_z - 0.012
    for label, (x, y) in caster_positions.items():
        deck.visual(
            Box((0.135, 0.105, 0.012)),
            origin=Origin(xyz=(x, y, deck_bottom_z - 0.006)),
            material=galvanized,
            name=f"{label}_mount",
        )

    hinge_x = -deck_length / 2.0 - 0.050
    hinge_z = deck_top_z + 0.022
    for y in (-0.235, 0.235):
        deck.visual(
            Box((0.034, 0.055, 0.044)),
            origin=Origin(xyz=(hinge_x, y, hinge_z - 0.044)),
            material=galvanized,
            name=f"handle_saddle_{0 if y < 0 else 1}",
        )
        deck.visual(
            Box((0.060, 0.050, 0.020)),
            origin=Origin(xyz=(hinge_x + 0.025, y, deck_top_z - 0.007)),
            material=galvanized,
            name=f"handle_leaf_{0 if y < 0 else 1}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.022, length=0.535),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="hinge_tube",
    )
    for y in (-0.230, 0.230):
        suffix = 0 if y < 0 else 1
        handle.visual(
            Cylinder(radius=0.017, length=0.720),
            origin=Origin(xyz=(0.0, y, 0.360)),
            material=blue,
            name=f"upright_{suffix}",
        )
        handle.visual(
            Cylinder(radius=0.012, length=0.350),
            origin=Origin(xyz=(0.025, y, 0.220), rpy=(0.0, math.radians(18.0), 0.0)),
            material=blue,
            name=f"lower_brace_{suffix}",
        )
    handle.visual(
        Cylinder(radius=0.017, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.720), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="top_tube",
    )
    handle.visual(
        Cylinder(radius=0.022, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.720), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="foam_grip",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.455),
        origin=Origin(xyz=(0.0, 0.0, 0.410), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="middle_crossbar",
    )
    model.articulation(
        "deck_to_handle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    fork_mesh = mesh_from_geometry(
        ClevisBracketGeometry(
            (0.115, 0.055, 0.105),
            gap_width=0.058,
            bore_diameter=0.016,
            bore_center_z=0.085,
            base_thickness=0.012,
            corner_radius=0.004,
        ),
        "caster_fork",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.062,
            0.044,
            inner_radius=0.043,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.035),
            tread=TireTread(style="block", depth=0.004, count=16, land_ratio=0.58),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.004, radius=0.0025),
        ),
        "caster_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.044,
            0.038,
            rim=WheelRim(inner_radius=0.028, flange_height=0.004, flange_thickness=0.0025),
            hub=WheelHub(
                radius=0.018,
                width=0.022,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        "caster_rim",
    )

    axle_drop = -0.085
    for label, (x, y) in caster_positions.items():
        caster = model.part(f"{label}_caster")
        caster.visual(
            Cylinder(radius=0.043, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material=galvanized,
            name="swivel_race",
        )
        caster.visual(
            Cylinder(radius=0.016, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=galvanized,
            name="kingpin",
        )
        caster.visual(
            fork_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.052), rpy=(0.0, math.pi, 0.0)),
            material=galvanized,
            name="fork_yoke",
        )
        caster.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(0.061, 0.0, axle_drop), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=light_metal,
            name="axle_cap_0",
        )
        caster.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(-0.061, 0.0, axle_drop), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=light_metal,
            name="axle_cap_1",
        )
        model.articulation(
            f"deck_to_{label}_caster",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(x, y, mount_bottom_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )

        wheel = model.part(f"{label}_wheel")
        wheel.visual(tire_mesh, material=black_rubber, name="tire")
        wheel.visual(rim_mesh, material=light_metal, name="rim")
        model.articulation(
            f"{label}_caster_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, axle_drop)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    handle = object_model.get_part("handle")
    handle_hinge = object_model.get_articulation("deck_to_handle")

    caster_labels = ("front_left", "front_right", "rear_left", "rear_right")
    ctx.check(
        "four articulated casters",
        all(object_model.get_part(f"{label}_caster") for label in caster_labels),
        details="Expected one swivel caster assembly at each deck corner.",
    )
    ctx.check(
        "four spinning wheels",
        all(object_model.get_part(f"{label}_wheel") for label in caster_labels),
        details="Expected one wheel under each caster fork.",
    )

    for label in caster_labels:
        caster = object_model.get_part(f"{label}_caster")
        wheel = object_model.get_part(f"{label}_wheel")
        ctx.expect_within(
            wheel,
            caster,
            axes="x",
            margin=0.008,
            name=f"{label} wheel fits between fork cheeks",
        )
        ctx.expect_overlap(
            wheel,
            caster,
            axes="z",
            min_overlap=0.030,
            name=f"{label} wheel is captured vertically by fork",
        )
        ctx.expect_overlap(
            caster,
            deck,
            axes="xy",
            min_overlap=0.035,
            name=f"{label} caster sits under deck corner",
        )

    upright_grip = ctx.part_element_world_aabb(handle, elem="foam_grip")
    with ctx.pose({handle_hinge: 1.45}):
        folded_grip = ctx.part_element_world_aabb(handle, elem="foam_grip")
    ctx.check(
        "handle folds forward over deck",
        upright_grip is not None
        and folded_grip is not None
        and (folded_grip[0][0] + folded_grip[1][0]) / 2.0
        > (upright_grip[0][0] + upright_grip[1][0]) / 2.0 + 0.55
        and folded_grip[1][2] < upright_grip[1][2] - 0.40,
        details=f"upright={upright_grip}, folded={folded_grip}",
    )

    return ctx.report()


object_model = build_object_model()
