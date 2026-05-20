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
    model = ArticulatedObject(name="sliding_platform_cart")

    steel = model.material("powder_coated_steel", rgba=(0.05, 0.16, 0.24, 1.0))
    edge_steel = model.material("blue_edge_rail", rgba=(0.02, 0.20, 0.46, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    wheel_metal = model.material("zinc_plated_metal", rgba=(0.72, 0.72, 0.66, 1.0))
    extension_mat = model.material("sliding_deck_red", rgba=(0.55, 0.08, 0.04, 1.0))
    dark_rail = model.material("dark_guide_rail", rgba=(0.03, 0.04, 0.05, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((1.10, 0.62, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.2525)),
        material=steel,
        name="main_deck_slab",
    )
    deck.visual(
        Box((1.16, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.337, 0.287)),
        material=edge_steel,
        name="side_rail_0",
    )
    deck.visual(
        Box((1.16, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, -0.337, 0.287)),
        material=edge_steel,
        name="side_rail_1",
    )
    deck.visual(
        Box((0.055, 0.62, 0.055)),
        origin=Origin(xyz=(-0.577, 0.0, 0.287)),
        material=edge_steel,
        name="rear_rail",
    )
    deck.visual(
        Box((0.030, 0.56, 0.030)),
        origin=Origin(xyz=(0.562, 0.0, 0.208)),
        material=edge_steel,
        name="front_nose_guard",
    )

    for i, x in enumerate((-0.34, -0.17, 0.0, 0.17, 0.34)):
        deck.visual(
            Box((0.055, 0.53, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.2875)),
            material=rubber,
            name=f"grip_strip_{i}",
        )

    # Under-deck C-channel guide rails.  The two open slots capture the
    # extension's side runners without intersecting them.
    for side, y_sign in enumerate((1.0, -1.0)):
        y = 0.275 * y_sign
        outer_y = 0.302 * y_sign
        deck.visual(
            Box((0.92, 0.050, 0.016)),
            origin=Origin(xyz=(0.095, y, 0.214)),
            material=dark_rail,
            name=f"guide_top_{side}",
        )
        deck.visual(
            Box((0.92, 0.012, 0.050)),
            origin=Origin(xyz=(0.095, outer_y, 0.190)),
            material=dark_rail,
            name=f"guide_wall_{side}",
        )
        deck.visual(
            Box((0.92, 0.044, 0.012)),
            origin=Origin(xyz=(0.095, y, 0.166)),
            material=dark_rail,
            name=f"guide_lip_{side}",
        )

    # Mounting pads for the four swivel caster bearings.
    caster_positions = [
        (0.405, 0.360),
        (0.405, -0.360),
        (-0.405, 0.360),
        (-0.405, -0.360),
    ]
    for i, (x, y) in enumerate(caster_positions):
        deck.visual(
            Box((0.145, 0.130, 0.020)),
            origin=Origin(xyz=(x, y, 0.214)),
            material=wheel_metal,
            name=f"caster_pad_{i}",
        )
        deck.visual(
            Cylinder(radius=0.021, length=0.012),
            origin=Origin(xyz=(x, y, 0.202)),
            material=wheel_metal,
            name=f"pivot_socket_{i}",
        )

    extension = model.part("extension")
    extension.visual(
        Box((0.70, 0.44, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=extension_mat,
        name="extension_plate",
    )
    extension.visual(
        Box((0.050, 0.48, 0.055)),
        origin=Origin(xyz=(0.375, 0.0, -0.005)),
        material=extension_mat,
        name="front_pull_lip",
    )
    for side, y_sign in enumerate((1.0, -1.0)):
        extension.visual(
            Box((0.70, 0.018, 0.020)),
            origin=Origin(xyz=(0.0, 0.247 * y_sign, 0.009)),
            material=wheel_metal,
            name=f"runner_bar_{side}",
        )
        extension.visual(
            Box((0.70, 0.016, 0.050)),
            origin=Origin(xyz=(0.0, 0.228 * y_sign, -0.007)),
            material=extension_mat,
            name=f"runner_web_{side}",
        )

    model.articulation(
        "deck_to_extension",
        ArticulationType.PRISMATIC,
        parent=deck,
        child=extension,
        origin=Origin(xyz=(0.18, 0.0, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.42),
    )

    wheel_geom = WheelGeometry(
        0.041,
        0.046,
        rim=WheelRim(
            inner_radius=0.026,
            flange_height=0.004,
            flange_thickness=0.003,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(
            radius=0.018,
            width=0.034,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.024, hole_diameter=0.003),
        ),
        face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.018),
    )
    tire_geom = TireGeometry(
        0.058,
        0.050,
        inner_radius=0.038,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
        tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.56),
        grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )

    for i, (x, y) in enumerate(caster_positions):
        fork = model.part(f"caster_fork_{i}")
        fork.visual(
            Cylinder(radius=0.044, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=wheel_metal,
            name="swivel_bearing",
        )
        fork.visual(
            Cylinder(radius=0.015, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=wheel_metal,
            name="kingpin",
        )
        fork.visual(
            Box((0.014, 0.110, 0.115)),
            origin=Origin(xyz=(0.041, 0.0, -0.077)),
            material=wheel_metal,
            name="fork_cheek_0",
        )
        fork.visual(
            Box((0.014, 0.110, 0.115)),
            origin=Origin(xyz=(-0.041, 0.0, -0.077)),
            material=wheel_metal,
            name="fork_cheek_1",
        )
        fork.visual(
            Box((0.096, 0.064, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=wheel_metal,
            name="fork_crown",
        )
        fork.visual(
            Cylinder(radius=0.007, length=0.092),
            origin=Origin(xyz=(0.0, 0.0, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_metal,
            name="axle_pin",
        )

        model.articulation(
            f"deck_to_fork_{i}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(x, y, 0.196)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )

        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            mesh_from_geometry(tire_geom, f"caster_tire_{i}"),
            origin=Origin(),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(wheel_geom, f"caster_wheel_{i}"),
            origin=Origin(),
            material=wheel_metal,
            name="rim_hub",
        )

        model.articulation(
            f"fork_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.085)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    extension = object_model.get_part("extension")
    slide = object_model.get_articulation("deck_to_extension")

    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            extension,
            deck,
            axes="x",
            min_overlap=0.60,
            elem_a="extension_plate",
            elem_b="main_deck_slab",
            name="extension nests under the deck at rest",
        )
        ctx.expect_within(
            extension,
            deck,
            axes="y",
            margin=0.0,
            elem_a="extension_plate",
            elem_b="main_deck_slab",
            name="nested extension remains within deck width",
        )
        rest_pos = ctx.part_world_position(extension)

    with ctx.pose({slide: 0.42}):
        ctx.expect_overlap(
            extension,
            deck,
            axes="x",
            min_overlap=0.25,
            elem_a="extension_plate",
            elem_b="main_deck_slab",
            name="extended platform keeps retained insertion",
        )
        ctx.expect_within(
            extension,
            deck,
            axes="y",
            margin=0.0,
            elem_a="extension_plate",
            elem_b="main_deck_slab",
            name="extended platform remains laterally guided",
        )
        extended_pos = ctx.part_world_position(extension)

    ctx.check(
        "extension slides outward along deck length",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.35,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    for i in range(4):
        fork = object_model.get_part(f"caster_fork_{i}")
        wheel = object_model.get_part(f"wheel_{i}")
        ctx.expect_gap(
            fork,
            wheel,
            axis="x",
            min_gap=0.003,
            max_gap=0.018,
            positive_elem="fork_cheek_0",
            negative_elem="tire",
            name=f"wheel_{i} clears positive fork cheek",
        )
        ctx.expect_gap(
            wheel,
            fork,
            axis="x",
            min_gap=0.003,
            max_gap=0.018,
            positive_elem="tire",
            negative_elem="fork_cheek_1",
            name=f"wheel_{i} clears negative fork cheek",
        )
        ctx.expect_overlap(
            wheel,
            fork,
            axes="z",
            min_overlap=0.020,
            elem_a="tire",
            elem_b="fork_cheek_0",
            name=f"wheel_{i} is vertically captured by fork cheek",
        )
        ctx.expect_overlap(
            wheel,
            fork,
            axes="z",
            min_overlap=0.020,
            elem_a="tire",
            elem_b="fork_cheek_1",
            name=f"wheel_{i} is vertically captured by opposite cheek",
        )

    return ctx.report()


object_model = build_object_model()
