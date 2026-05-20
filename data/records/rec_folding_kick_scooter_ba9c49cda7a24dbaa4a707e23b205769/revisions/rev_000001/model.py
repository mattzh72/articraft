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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cargo_folding_scooter")

    deck_finish = model.material("matte_graphite", rgba=(0.08, 0.09, 0.10, 1.0))
    grip_finish = model.material("textured_deck_grip", rgba=(0.015, 0.016, 0.018, 1.0))
    basket_finish = model.material("powder_blue_basket", rgba=(0.08, 0.28, 0.55, 1.0))
    metal_finish = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber_finish = model.material("soft_black_rubber", rgba=(0.02, 0.02, 0.022, 1.0))

    deck = model.part("deck")

    # A long, low cargo scooter deck with visible grip surface and side rails.
    deck.visual(
        Box((0.82, 0.22, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=deck_finish,
        name="deck_panel",
    )
    deck.visual(
        Box((0.66, 0.165, 0.006)),
        origin=Origin(xyz=(-0.055, 0.0, 0.211)),
        material=grip_finish,
        name="grip_pad",
    )
    for y, name in ((-0.109, "side_rail_0"), (0.109, "side_rail_1")):
        deck.visual(
            Box((0.78, 0.014, 0.030)),
            origin=Origin(xyz=(-0.015, y, 0.205)),
            material=metal_finish,
            name=name,
        )

    # Front and rear fork cheeks hold the two single-track deck wheels.
    for x, prefix in ((0.535, "front"), (-0.535, "rear")):
        for y, suffix in ((-0.041, "cheek_0"), (0.041, "cheek_1")):
            deck.visual(
                Box((0.040, 0.014, 0.145)),
                origin=Origin(xyz=(x, y, 0.145)),
                material=metal_finish,
                name=f"{prefix}_fork_{suffix}",
            )
        deck.visual(
            Box((0.160, 0.014, 0.026)),
            origin=Origin(xyz=(x - 0.070 if x > 0 else x + 0.070, -0.041, 0.190)),
            material=metal_finish,
            name=f"{prefix}_stay_0",
        )
        deck.visual(
            Box((0.160, 0.014, 0.026)),
            origin=Origin(xyz=(x - 0.070 if x > 0 else x + 0.070, 0.041, 0.190)),
            material=metal_finish,
            name=f"{prefix}_stay_1",
        )
        for y, suffix in ((-0.053, "axle_cap_0"), (0.053, "axle_cap_1")):
            deck.visual(
                Cylinder(radius=0.018, length=0.010),
                origin=Origin(xyz=(x, y, 0.105), rpy=(pi / 2.0, 0.0, 0.0)),
                material=metal_finish,
                name=f"{prefix}_{suffix}",
            )
    deck.visual(
        Cylinder(radius=0.007, length=0.112),
        origin=Origin(xyz=(0.535, 0.0, 0.105), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_finish,
        name="front_axle_shaft",
    )
    deck.visual(
        Cylinder(radius=0.007, length=0.112),
        origin=Origin(xyz=(-0.535, 0.0, 0.105), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_finish,
        name="rear_axle_shaft",
    )

    # Raised front neck and cheeked folding hinge bracket for the handlebar stem.
    deck.visual(
        Cylinder(radius=0.026, length=0.102),
        origin=Origin(xyz=(0.392, 0.0, 0.255)),
        material=metal_finish,
        name="front_neck_tube",
    )
    deck.visual(
        Box((0.080, 0.070, 0.024)),
        origin=Origin(xyz=(0.392, 0.0, 0.214)),
        material=metal_finish,
        name="neck_base",
    )
    deck.visual(
        Box((0.055, 0.120, 0.022)),
        origin=Origin(xyz=(0.395, 0.0, 0.314)),
        material=metal_finish,
        name="stem_hinge_bridge",
    )
    deck.visual(
        Box((0.068, 0.014, 0.076)),
        origin=Origin(xyz=(0.395, -0.055, 0.350)),
        material=metal_finish,
        name="stem_hinge_cheek_0",
    )
    deck.visual(
        Box((0.068, 0.014, 0.076)),
        origin=Origin(xyz=(0.395, 0.055, 0.350)),
        material=metal_finish,
        name="stem_hinge_cheek_1",
    )

    # Front cargo basket: open wire-frame sides, bottom grid, and deck-mounted posts.
    basket_x = 0.200
    basket_z0 = 0.252
    basket_z1 = 0.452
    x_front = basket_x + 0.160
    x_back = basket_x - 0.160
    y_side = 0.140
    for z, level in ((basket_z0, "lower"), (basket_z1, "upper")):
        deck.visual(
            Box((0.332, 0.012, 0.012)),
            origin=Origin(xyz=(basket_x, -y_side, z)),
            material=basket_finish,
            name=f"basket_{level}_rail_0",
        )
        deck.visual(
            Box((0.332, 0.012, 0.012)),
            origin=Origin(xyz=(basket_x, y_side, z)),
            material=basket_finish,
            name=f"basket_{level}_rail_1",
        )
        deck.visual(
            Box((0.012, 0.292, 0.012)),
            origin=Origin(xyz=(x_front, 0.0, z)),
            material=basket_finish,
            name=f"basket_{level}_rail_2",
        )
        deck.visual(
            Box((0.012, 0.292, 0.012)),
            origin=Origin(xyz=(x_back, 0.0, z)),
            material=basket_finish,
            name=f"basket_{level}_rail_3",
        )
    for x in (x_back, x_front):
        for y in (-y_side, y_side):
            deck.visual(
                Box((0.014, 0.014, 0.214)),
                origin=Origin(xyz=(x, y, (basket_z0 + basket_z1) / 2.0)),
                material=basket_finish,
                name=f"basket_corner_{'front' if x > basket_x else 'rear'}_{'side1' if y > 0 else 'side0'}",
            )
    for y in (-0.090, -0.030, 0.030, 0.090):
        deck.visual(
            Box((0.010, 0.009, 0.190)),
            origin=Origin(xyz=(x_front, y, (basket_z0 + basket_z1) / 2.0)),
            material=basket_finish,
            name=f"basket_front_slat_{len(deck.visuals)}",
        )
        deck.visual(
            Box((0.010, 0.009, 0.190)),
            origin=Origin(xyz=(x_back, y, (basket_z0 + basket_z1) / 2.0)),
            material=basket_finish,
            name=f"basket_rear_slat_{len(deck.visuals)}",
        )
    for x in (basket_x - 0.090, basket_x - 0.030, basket_x + 0.030, basket_x + 0.090):
        deck.visual(
            Box((0.009, 0.010, 0.190)),
            origin=Origin(xyz=(x, -y_side, (basket_z0 + basket_z1) / 2.0)),
            material=basket_finish,
            name=f"basket_side_slat_{len(deck.visuals)}",
        )
        deck.visual(
            Box((0.009, 0.010, 0.190)),
            origin=Origin(xyz=(x, y_side, (basket_z0 + basket_z1) / 2.0)),
            material=basket_finish,
            name=f"basket_side_slat_{len(deck.visuals)}",
        )
    for y in (-0.084, 0.0, 0.084):
        deck.visual(
            Box((0.322, 0.009, 0.010)),
            origin=Origin(xyz=(basket_x, y, basket_z0 - 0.004)),
            material=basket_finish,
            name=f"basket_floor_x_{len(deck.visuals)}",
        )
    for x in (basket_x - 0.105, basket_x, basket_x + 0.105):
        deck.visual(
            Box((0.009, 0.282, 0.010)),
            origin=Origin(xyz=(x, 0.0, basket_z0 - 0.004)),
            material=basket_finish,
            name=f"basket_floor_y_{len(deck.visuals)}",
        )
    for x in (basket_x - 0.120, basket_x + 0.120):
        for y in (-0.086, 0.086):
            deck.visual(
                Box((0.016, 0.016, 0.060)),
                origin=Origin(xyz=(x, y, 0.232)),
                material=basket_finish,
                name=f"basket_mount_{len(deck.visuals)}",
            )

    # Rear underside hinge bracket for the deployable kickstand prop.
    deck.visual(
        Box((0.046, 0.014, 0.052)),
        origin=Origin(xyz=(-0.360, -0.042, 0.132)),
        material=metal_finish,
        name="kickstand_lug_0",
    )
    deck.visual(
        Box((0.046, 0.014, 0.052)),
        origin=Origin(xyz=(-0.360, 0.042, 0.132)),
        material=metal_finish,
        name="kickstand_lug_1",
    )
    deck.visual(
        Box((0.075, 0.075, 0.012)),
        origin=Origin(xyz=(-0.360, 0.0, 0.160)),
        material=metal_finish,
        name="kickstand_hinge_plate",
    )

    wheel_mesh = WheelGeometry(
        0.085,
        0.034,
        rim=WheelRim(inner_radius=0.056, flange_height=0.007, flange_thickness=0.003),
        hub=WheelHub(
            radius=0.024,
            width=0.030,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.0035),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.008),
        bore=WheelBore(style="round", diameter=0.012),
    )
    tire_mesh = TireGeometry(
        0.105,
        0.045,
        inner_radius=0.086,
        tread=TireTread(style="circumferential", depth=0.004, count=4),
        grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
    )
    for name in ("front_wheel", "rear_wheel"):
        wheel = model.part(name)
        wheel.visual(
            mesh_from_geometry(wheel_mesh, f"{name}_rim"),
            material=metal_finish,
            name="rim",
        )
        wheel.visual(
            mesh_from_geometry(tire_mesh, f"{name}_tire"),
            material=rubber_finish,
            name="tire",
        )

    stem = model.part("handlebar_stem")
    stem.visual(
        Cylinder(radius=0.019, length=0.096),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_finish,
        name="hinge_barrel",
    )
    stem.visual(
        Cylinder(radius=0.021, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=metal_finish,
        name="lower_collar",
    )
    stem.visual(
        Cylinder(radius=0.015, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=metal_finish,
        name="stem_tube",
    )
    stem.visual(
        Cylinder(radius=0.014, length=0.480),
        origin=Origin(xyz=(0.0, 0.0, 0.760), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_finish,
        name="handlebar",
    )
    for y, name in ((-0.285, "grip_0"), (0.285, "grip_1")):
        stem.visual(
            Cylinder(radius=0.018, length=0.110),
            origin=Origin(xyz=(0.0, y, 0.760), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber_finish,
            name=name,
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.013, length=0.070),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_finish,
        name="hinge_barrel",
    )
    kickstand.visual(
        Box((0.026, 0.028, 0.168)),
        origin=Origin(xyz=(0.050, 0.0, -0.080), rpy=(0.0, -0.560, 0.0)),
        material=metal_finish,
        name="prop_leg",
    )
    kickstand.visual(
        Box((0.086, 0.040, 0.018)),
        origin=Origin(xyz=(0.105, 0.0, -0.158)),
        material=rubber_finish,
        name="rubber_foot",
    )

    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child="front_wheel",
        origin=Origin(xyz=(0.535, 0.0, 0.105), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child="rear_wheel",
        origin=Origin(xyz=(-0.535, 0.0, 0.105), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )
    model.articulation(
        "stem_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(0.395, 0.0, 0.350)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.6, lower=0.0, upper=1.35),
    )
    model.articulation(
        "kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=kickstand,
        origin=Origin(xyz=(-0.360, 0.0, 0.132)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    stem = object_model.get_part("handlebar_stem")
    kickstand = object_model.get_part("kickstand")
    stem_hinge = object_model.get_articulation("stem_hinge")
    kickstand_hinge = object_model.get_articulation("kickstand_hinge")

    ctx.allow_overlap(
        deck,
        front_wheel,
        elem_a="front_axle_shaft",
        elem_b="rim",
        reason="The visible axle shaft is intentionally captured through the wheel hub bore.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle_shaft",
        elem_b="rim",
        reason="The visible axle shaft is intentionally captured through the wheel hub bore.",
    )

    for joint_name in ("front_axle", "rear_axle", "stem_hinge", "kickstand_hinge"):
        ctx.check(
            f"{joint_name}_present",
            object_model.get_articulation(joint_name) is not None,
            details=f"Missing {joint_name}.",
        )

    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        positive_elem="tire",
        negative_elem="deck_panel",
        min_gap=0.010,
        name="front_wheel_clears_deck_front",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        positive_elem="deck_panel",
        negative_elem="tire",
        min_gap=0.010,
        name="rear_wheel_clears_deck_rear",
    )
    ctx.expect_overlap(
        front_wheel,
        deck,
        axes="y",
        elem_a="rim",
        elem_b="front_axle_shaft",
        min_overlap=0.030,
        name="front_axle_passes_through_hub",
    )
    ctx.expect_overlap(
        rear_wheel,
        deck,
        axes="y",
        elem_a="rim",
        elem_b="rear_axle_shaft",
        min_overlap=0.030,
        name="rear_axle_passes_through_hub",
    )
    ctx.expect_within(
        deck,
        front_wheel,
        axes="xz",
        inner_elem="front_axle_shaft",
        outer_elem="rim",
        margin=0.020,
        name="front_axle_centered_in_hub",
    )
    ctx.expect_within(
        deck,
        rear_wheel,
        axes="xz",
        inner_elem="rear_axle_shaft",
        outer_elem="rim",
        margin=0.020,
        name="rear_axle_centered_in_hub",
    )
    ctx.expect_contact(
        stem,
        deck,
        elem_a="hinge_barrel",
        elem_b="stem_hinge_cheek_0",
        contact_tol=0.001,
        name="stem_barrel_sits_between_hinge_cheeks",
    )
    ctx.expect_contact(
        kickstand,
        deck,
        elem_a="hinge_barrel",
        elem_b="kickstand_lug_0",
        contact_tol=0.001,
        name="kickstand_barrel_sits_between_lugs",
    )

    stem_rest_aabb = ctx.part_world_aabb(stem)
    with ctx.pose({stem_hinge: 1.35}):
        stem_folded_aabb = ctx.part_world_aabb(stem)
    if stem_rest_aabb is not None and stem_folded_aabb is not None:
        rest_min, rest_max = stem_rest_aabb
        fold_min, fold_max = stem_folded_aabb
        ctx.check(
            "stem_folds_back_over_deck",
            float(fold_min[0]) < float(rest_min[0]) - 0.35 and float(fold_max[2]) < float(rest_max[2]) - 0.28,
            details=f"rest={stem_rest_aabb!r}, folded={stem_folded_aabb!r}",
        )

    kickstand_down_aabb = ctx.part_world_aabb(kickstand)
    with ctx.pose({kickstand_hinge: 1.35}):
        kickstand_stowed_aabb = ctx.part_world_aabb(kickstand)
    if kickstand_down_aabb is not None and kickstand_stowed_aabb is not None:
        down_min, _ = kickstand_down_aabb
        stow_min, _ = kickstand_stowed_aabb
        ctx.check(
            "kickstand_rotates_up_to_stow",
            float(stow_min[2]) > float(down_min[2]) + 0.050,
            details=f"down={kickstand_down_aabb!r}, stowed={kickstand_stowed_aabb!r}",
        )

    return ctx.report()


object_model = build_object_model()
