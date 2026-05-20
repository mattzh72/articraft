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
    model = ArticulatedObject(name="compact_platform_cart")

    blue = model.material("powder_coated_blue", rgba=(0.05, 0.18, 0.48, 1.0))
    dark_blue = model.material("dark_blue_edge", rgba=(0.025, 0.06, 0.16, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    zinc = model.material("zinc_hardware", rgba=(0.45, 0.46, 0.43, 1.0))

    deck_len = 0.75
    deck_w = 0.42
    deck_th = 0.050
    rear_x = -0.285
    front_x = 0.275
    rear_axle_z = -0.125
    rear_wheel_y = 0.255
    caster_y = 0.150

    deck = model.part("deck")
    deck.visual(
        Box((deck_len, deck_w, deck_th)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue,
        name="deck_plate",
    )
    # Rolled bumpers and side lips give the small pressed deck a dolly-cart
    # silhouette instead of a plain slab.
    deck.visual(
        Box((deck_len + 0.020, 0.030, 0.032)),
        origin=Origin(xyz=(0.0, deck_w / 2 + 0.008, 0.004)),
        material=dark_blue,
        name="side_lip_0",
    )
    deck.visual(
        Box((deck_len + 0.020, 0.030, 0.032)),
        origin=Origin(xyz=(0.0, -deck_w / 2 - 0.008, 0.004)),
        material=dark_blue,
        name="side_lip_1",
    )
    deck.visual(
        Box((0.036, deck_w + 0.060, 0.040)),
        origin=Origin(xyz=(deck_len / 2 + 0.002, 0.0, 0.004)),
        material=dark_blue,
        name="front_bumper",
    )
    deck.visual(
        Box((0.036, deck_w + 0.060, 0.040)),
        origin=Origin(xyz=(-deck_len / 2 - 0.002, 0.0, 0.004)),
        material=dark_blue,
        name="rear_bumper",
    )
    for i, y in enumerate((-0.115, 0.0, 0.115)):
        deck.visual(
            Box((0.54, 0.026, 0.006)),
            origin=Origin(xyz=(0.020, y, deck_th / 2 + 0.001)),
            material=black,
            name=f"grip_strip_{i}",
        )
    # Underside frame members and pads that all overlap the deck skin slightly.
    deck.visual(
        Box((0.64, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.195, -0.038)),
        material=zinc,
        name="under_rail_0",
    )
    deck.visual(
        Box((0.64, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.195, -0.038)),
        material=zinc,
        name="under_rail_1",
    )
    deck.visual(
        Box((0.060, 0.330, 0.028)),
        origin=Origin(xyz=(rear_x, 0.0, -0.040)),
        material=zinc,
        name="rear_crossmember",
    )
    deck.visual(
        Box((0.080, 0.220, 0.026)),
        origin=Origin(xyz=(front_x, 0.0, -0.040)),
        material=zinc,
        name="front_crossmember",
    )
    for i, y in enumerate((-0.170, 0.170)):
        deck.visual(
            Box((0.052, 0.022, 0.110)),
            origin=Origin(xyz=(rear_x, y, -0.076)),
            material=zinc,
            name=f"rear_axle_hanger_{i}",
        )
    deck.visual(
        Cylinder(radius=0.0135, length=0.540),
        origin=Origin(xyz=(rear_x, 0.0, rear_axle_z), rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="rear_axle",
    )
    for i, y in enumerate((-caster_y, caster_y)):
        deck.visual(
            Box((0.120, 0.095, 0.014)),
            origin=Origin(xyz=(front_x, y, -0.032)),
            material=zinc,
            name=f"caster_mount_{i}",
        )
    # Rear handle hinge lugs mounted to the deck end; the moving handle tube
    # passes between them.
    for i, y in enumerate((-0.195, 0.195)):
        deck.visual(
            Box((0.048, 0.025, 0.090)),
            origin=Origin(xyz=(-deck_len / 2 + 0.020, y, 0.046)),
            material=zinc,
            name=f"handle_lug_{i}",
        )

    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.078,
            0.046,
            inner_radius=0.052,
            carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.0045, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.003),
        ),
        "rear_tire",
    )
    rear_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.040,
            rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.022,
                width=0.032,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.0035),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.027),
        ),
        "rear_rim",
    )
    for idx, y in enumerate((-rear_wheel_y, rear_wheel_y)):
        wheel = model.part(f"rear_wheel_{idx}")
        wheel.visual(
            rear_tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2)),
            material=black,
            name="tire",
        )
        wheel.visual(
            rear_rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2)),
            material=steel,
            name="rim",
        )
        model.articulation(
            f"rear_wheel_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=wheel,
            origin=Origin(xyz=(rear_x, y, rear_axle_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.052,
            0.032,
            inner_radius=0.034,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.003, count=16, land_ratio=0.62),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.003, radius=0.0025),
        ),
        "caster_tire",
    )
    caster_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.034,
            0.028,
            rim=WheelRim(inner_radius=0.021, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(radius=0.014, width=0.020, cap_style="flat"),
            face=WheelFace(dish_depth=0.002, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.005),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        "caster_rim",
    )

    for idx, y in enumerate((-caster_y, caster_y)):
        fork = model.part(f"caster_fork_{idx}")
        fork.visual(
            Cylinder(radius=0.027, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=zinc,
            name="swivel_cap",
        )
        fork.visual(
            Cylinder(radius=0.009, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, -0.042)),
            material=steel,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.070, 0.060, 0.014)),
            origin=Origin(xyz=(0.030, 0.0, -0.028)),
            material=zinc,
            name="fork_bridge",
        )
        for side, sy in enumerate((-0.026, 0.026)):
            fork.visual(
                Box((0.060, 0.008, 0.140)),
                origin=Origin(xyz=(0.070, sy, -0.105)),
                material=zinc,
                name=f"side_plate_{side}",
            )
        fork.visual(
            Cylinder(radius=0.008, length=0.064),
            origin=Origin(xyz=(0.070, 0.0, -0.110), rpy=(pi / 2, 0.0, 0.0)),
            material=steel,
            name="wheel_pin",
        )
        model.articulation(
            f"caster_swivel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(front_x, y, -0.046)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=8.0),
        )

        caster = model.part(f"caster_wheel_{idx}")
        caster.visual(
            caster_tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2)),
            material=black,
            name="tire",
        )
        caster.visual(
            caster_rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2)),
            material=steel,
            name="rim",
        )
        model.articulation(
            f"caster_wheel_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=caster,
            origin=Origin(xyz=(0.070, 0.0, -0.110)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=22.0),
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.015, length=0.365),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    for i, y in enumerate((-0.150, 0.150)):
        handle.visual(
            Cylinder(radius=0.012, length=0.680),
            origin=Origin(xyz=(0.0, y, 0.340)),
            material=steel,
            name=f"upright_tube_{i}",
        )
    handle.visual(
        Cylinder(radius=0.013, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.680), rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="top_bar",
    )
    handle.visual(
        Cylinder(radius=0.017, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.680), rpy=(pi / 2, 0.0, 0.0)),
        material=black,
        name="rubber_grip",
    )
    handle.visual(
        Cylinder(radius=0.009, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.360), rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="middle_crossbar",
    )
    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(-deck_len / 2 + 0.020, 0.0, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.10, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    handle = object_model.get_part("handle")
    hinge = object_model.get_articulation("handle_hinge")

    ctx.check(
        "cart has mixed wheel articulations",
        all(
            object_model.get_articulation(name) is not None
            for name in (
                "rear_wheel_spin_0",
                "rear_wheel_spin_1",
                "caster_swivel_0",
                "caster_swivel_1",
                "caster_wheel_spin_0",
                "caster_wheel_spin_1",
            )
        ),
        details="Expected two fixed rear wheel spins plus two caster swivels and wheel spins.",
    )

    for idx in (0, 1):
        rear = object_model.get_part(f"rear_wheel_{idx}")
        caster_fork = object_model.get_part(f"caster_fork_{idx}")
        caster_wheel = object_model.get_part(f"caster_wheel_{idx}")
        ctx.allow_overlap(
            deck,
            rear,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The fixed rear axle is intentionally captured through the simplified spinning wheel hub.",
        )
        ctx.allow_overlap(
            caster_fork,
            caster_wheel,
            elem_a="wheel_pin",
            elem_b="rim",
            reason="The caster wheel pin is intentionally captured through the simplified spinning hub.",
        )
        ctx.expect_overlap(
            deck,
            rear,
            axes="y",
            elem_a="rear_axle",
            elem_b="rim",
            min_overlap=0.020,
            name=f"rear axle {idx} passes through hub",
        )
        ctx.expect_overlap(
            caster_fork,
            caster_wheel,
            axes="y",
            elem_a="wheel_pin",
            elem_b="rim",
            min_overlap=0.020,
            name=f"caster pin {idx} passes through hub",
        )
        ctx.expect_gap(
            deck,
            rear,
            axis="z",
            positive_elem="deck_plate",
            negative_elem="tire",
            min_gap=0.004,
            name=f"rear wheel {idx} clears deck underside",
        )
        ctx.expect_within(
            caster_wheel,
            caster_fork,
            axes="y",
            inner_elem="tire",
            margin=0.000,
            name=f"caster wheel {idx} sits between fork cheeks",
        )

    handle_top = ctx.part_element_world_aabb(handle, elem="rubber_grip")
    deck_top = ctx.part_element_world_aabb(deck, elem="deck_plate")
    ctx.check(
        "handle is upright at rest",
        handle_top is not None
        and deck_top is not None
        and handle_top[1][2] > deck_top[1][2] + 0.55,
        details=f"handle_top={handle_top}, deck_top={deck_top}",
    )
    rest_top_x = None if handle_top is None else (handle_top[0][0] + handle_top[1][0]) / 2.0
    rest_top_z = None if handle_top is None else (handle_top[0][2] + handle_top[1][2]) / 2.0
    with ctx.pose({hinge: 1.35}):
        folded_top = ctx.part_element_world_aabb(handle, elem="rubber_grip")
    folded_top_x = None if folded_top is None else (folded_top[0][0] + folded_top[1][0]) / 2.0
    folded_top_z = None if folded_top is None else (folded_top[0][2] + folded_top[1][2]) / 2.0
    ctx.check(
        "handle folds forward and down",
        rest_top_x is not None
        and rest_top_z is not None
        and folded_top_x is not None
        and folded_top_z is not None
        and folded_top_x > rest_top_x + 0.45
        and folded_top_z < rest_top_z - 0.35,
        details=f"rest=({rest_top_x}, {rest_top_z}), folded=({folded_top_x}, {folded_top_z})",
    )

    return ctx.report()


object_model = build_object_model()
