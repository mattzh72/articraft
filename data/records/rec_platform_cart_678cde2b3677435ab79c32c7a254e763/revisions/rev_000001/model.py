from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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
    model = ArticulatedObject(name="warehouse_platform_cart")

    model.material("deck_blue", color=(0.05, 0.16, 0.30, 1.0))
    model.material("worn_steel", color=(0.58, 0.60, 0.59, 1.0))
    model.material("dark_rubber", color=(0.015, 0.015, 0.014, 1.0))
    model.material("zinc_tray", color=(0.72, 0.74, 0.72, 1.0))
    model.material("black_grip", color=(0.02, 0.025, 0.03, 1.0))
    model.material("warning_yellow", color=(0.95, 0.68, 0.05, 1.0))

    deck = model.part("deck")

    # Main deck: a low steel platform with reinforced edging and a ribbed top.
    deck.visual(
        Box((1.20, 0.74, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material="deck_blue",
        name="deck_plate",
    )
    deck.visual(
        Box((1.22, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, 0.387, 0.430)),
        material="worn_steel",
        name="side_bumper_0",
    )
    deck.visual(
        Box((1.22, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, -0.387, 0.430)),
        material="worn_steel",
        name="side_bumper_1",
    )
    deck.visual(
        Box((0.035, 0.74, 0.070)),
        origin=Origin(xyz=(0.617, 0.0, 0.430)),
        material="warning_yellow",
        name="front_bumper",
    )
    deck.visual(
        Box((0.035, 0.74, 0.070)),
        origin=Origin(xyz=(-0.617, 0.0, 0.430)),
        material="worn_steel",
        name="rear_bumper",
    )
    for i, y in enumerate((-0.24, -0.12, 0.0, 0.12, 0.24)):
        deck.visual(
            Box((1.02, 0.018, 0.006)),
            origin=Origin(xyz=(0.02, y, 0.450)),
            material="black_grip",
            name=f"grip_rib_{i}",
        )

    # Under-deck guide rails form two shallow C channels that trap the tray flanges.
    rail_len = 1.00
    for suffix, sign in (("0", 1.0), ("1", -1.0)):
        y_web = sign * 0.262
        y_lip = sign * 0.215
        deck.visual(
            Box((rail_len, 0.025, 0.075)),
            origin=Origin(xyz=(0.05, y_web, 0.315)),
            material="worn_steel",
            name=f"guide_web_{suffix}",
        )
        deck.visual(
            Box((rail_len, 0.075, 0.012)),
            origin=Origin(xyz=(0.05, y_lip, 0.345)),
            material="worn_steel",
            name=f"top_lip_{suffix}",
        )
        deck.visual(
            Box((rail_len, 0.075, 0.012)),
            origin=Origin(xyz=(0.05, y_lip, 0.285)),
            material="worn_steel",
            name=f"bottom_lip_{suffix}",
        )
        for j, x in enumerate((-0.36, 0.02, 0.40)):
            deck.visual(
                Box((0.035, 0.035, 0.055)),
                origin=Origin(xyz=(x, y_web, 0.368)),
                material="worn_steel",
                name=f"rail_hanger_{suffix}_{j}",
            )

    # Push handle, welded to the rear edge of the deck.
    for suffix, y in (("0", 0.305), ("1", -0.305)):
        deck.visual(
            Box((0.080, 0.065, 0.016)),
            origin=Origin(xyz=(-0.555, y, 0.457)),
            material="worn_steel",
            name=f"handle_foot_{suffix}",
        )
        deck.visual(
            Cylinder(radius=0.019, length=0.560),
            origin=Origin(xyz=(-0.555, y, 0.725)),
            material="worn_steel",
            name=f"handle_post_{suffix}",
        )
    deck.visual(
        Cylinder(radius=0.020, length=0.650),
        origin=Origin(xyz=(-0.555, 0.0, 1.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material="worn_steel",
        name="handle_crossbar",
    )
    deck.visual(
        Cylinder(radius=0.024, length=0.430),
        origin=Origin(xyz=(-0.555, 0.0, 1.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material="black_grip",
        name="rubber_grip",
    )

    # Caster mounting plates fixed to the deck underside.
    caster_mounts = (
        ("front_0", 0.49, 0.320),
        ("front_1", 0.49, -0.320),
        ("rear_0", -0.49, 0.320),
        ("rear_1", -0.49, -0.320),
    )
    for name, x, y in caster_mounts:
        deck.visual(
            Box((0.150, 0.090, 0.014)),
            origin=Origin(xyz=(x, y, 0.3855)),
            material="worn_steel",
            name=f"{name}_mount",
        )

    tray = model.part("tray")
    tray.visual(
        Box((0.660, 0.340, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material="zinc_tray",
        name="tray_floor",
    )
    tray.visual(
        Box((0.660, 0.018, 0.062)),
        origin=Origin(xyz=(0.0, 0.158, -0.020)),
        material="zinc_tray",
        name="side_wall_0",
    )
    tray.visual(
        Box((0.660, 0.018, 0.062)),
        origin=Origin(xyz=(0.0, -0.158, -0.020)),
        material="zinc_tray",
        name="side_wall_1",
    )
    tray.visual(
        Box((0.018, 0.334, 0.062)),
        origin=Origin(xyz=(0.330, 0.0, -0.020)),
        material="zinc_tray",
        name="front_wall",
    )
    tray.visual(
        Box((0.018, 0.334, 0.062)),
        origin=Origin(xyz=(-0.330, 0.0, -0.020)),
        material="zinc_tray",
        name="rear_wall",
    )
    tray.visual(
        Box((0.660, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.215, -0.018)),
        material="zinc_tray",
        name="slide_flange_0",
    )
    tray.visual(
        Box((0.660, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, -0.215, -0.018)),
        material="zinc_tray",
        name="slide_flange_1",
    )
    tray.visual(
        Box((0.660, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.176, -0.018)),
        material="zinc_tray",
        name="flange_neck_0",
    )
    tray.visual(
        Box((0.660, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.176, -0.018)),
        material="zinc_tray",
        name="flange_neck_1",
    )
    tray.visual(
        Box((0.030, 0.220, 0.036)),
        origin=Origin(xyz=(0.346, 0.0, -0.012)),
        material="worn_steel",
        name="pull_lip",
    )
    tray_slide = model.articulation(
        "deck_to_tray",
        ArticulationType.PRISMATIC,
        parent=deck,
        child=tray,
        origin=Origin(xyz=(0.180, 0.0, 0.315)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.32),
    )

    # Swiveling caster yokes and freely spinning wheels.
    for index, (base_name, x, y) in enumerate(caster_mounts):
        caster = model.part(f"{base_name}_caster")
        caster.visual(
            Cylinder(radius=0.040, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material="worn_steel",
            name="swivel_bearing",
        )
        caster.visual(
            Cylinder(radius=0.016, length=0.065),
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
            material="worn_steel",
            name="vertical_stem",
        )
        caster.visual(
            Box((0.105, 0.088, 0.035)),
            origin=Origin(xyz=(0.020, 0.0, -0.085)),
            material="worn_steel",
            name="fork_crown",
        )
        caster.visual(
            Box((0.042, 0.008, 0.135)),
            origin=Origin(xyz=(0.050, 0.036, -0.150)),
            material="worn_steel",
            name="fork_cheek_0",
        )
        caster.visual(
            Box((0.042, 0.008, 0.135)),
            origin=Origin(xyz=(0.050, -0.036, -0.150)),
            material="worn_steel",
            name="fork_cheek_1",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.050, 0.048, -0.190), rpy=(pi / 2.0, 0.0, 0.0)),
            material="worn_steel",
            name="axle_cap_0",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.050, -0.048, -0.190), rpy=(pi / 2.0, 0.0, 0.0)),
            material="worn_steel",
            name="axle_cap_1",
        )
        caster.visual(
            Cylinder(radius=0.011, length=0.090),
            origin=Origin(xyz=(0.050, 0.0, -0.190), rpy=(pi / 2.0, 0.0, 0.0)),
            material="worn_steel",
            name="axle_pin",
        )
        model.articulation(
            f"deck_to_{base_name}_caster",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(x, y, 0.3785)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=6.0),
        )

        wheel = model.part(f"{base_name}_wheel")
        tire = TireGeometry(
            0.076,
            0.046,
            inner_radius=0.052,
            tread=TireTread(style="block", depth=0.004, count=16, land_ratio=0.60),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        )
        wheel.visual(
            mesh_from_geometry(tire, f"{base_name}_tire"),
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material="dark_rubber",
            name="rubber_tire",
        )
        wheel.visual(
            Cylinder(radius=0.053, length=0.052),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material="worn_steel",
            name="metal_hub",
        )
        model.articulation(
            f"{base_name}_caster_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.050, 0.0, -0.190)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=20.0),
        )

    deck.meta["tray_slide_joint"] = tray_slide.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    tray = object_model.get_part("tray")
    tray_slide = object_model.get_articulation("deck_to_tray")

    # The tray side flanges ride inside the two under-deck C-channel lips.
    for suffix in ("0", "1"):
        ctx.expect_within(
            tray,
            deck,
            axes="y",
            inner_elem=f"slide_flange_{suffix}",
            outer_elem=f"top_lip_{suffix}",
            margin=0.001,
            name=f"tray flange {suffix} is laterally trapped by the rail lip",
        )
        ctx.expect_gap(
            deck,
            tray,
            axis="z",
            positive_elem=f"top_lip_{suffix}",
            negative_elem=f"slide_flange_{suffix}",
            min_gap=0.025,
            max_gap=0.045,
            name=f"top rail lip clears tray flange {suffix}",
        )
        ctx.expect_gap(
            tray,
            deck,
            axis="z",
            positive_elem=f"slide_flange_{suffix}",
            negative_elem=f"bottom_lip_{suffix}",
            min_gap=0.0,
            max_gap=0.003,
            name=f"bottom rail lip supports tray flange {suffix}",
        )

    rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.32}):
        extended_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            deck,
            axes="x",
            elem_a="slide_flange_0",
            elem_b="top_lip_0",
            min_overlap=0.30,
            name="extended tray flange remains inserted in guide rail",
        )
        ctx.expect_within(
            tray,
            deck,
            axes="y",
            inner_elem="slide_flange_0",
            outer_elem="top_lip_0",
            margin=0.001,
            name="extended tray remains laterally captured",
        )
    ctx.check(
        "tray slides out toward the front",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # Verify the primary caster articulations are present and oriented like real casters:
    # each yoke swivels about a vertical stem, and each wheel spins on a horizontal axle.
    for base_name in ("front_0", "front_1", "rear_0", "rear_1"):
        caster = object_model.get_part(f"{base_name}_caster")
        wheel = object_model.get_part(f"{base_name}_wheel")
        swivel = object_model.get_articulation(f"deck_to_{base_name}_caster")
        wheel_joint = object_model.get_articulation(f"{base_name}_caster_to_wheel")
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="axle_pin",
            elem_b="metal_hub",
            reason="The steel axle is intentionally modeled as a captured pin running through the wheel hub.",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="y",
            elem_a="axle_pin",
            elem_b="metal_hub",
            min_overlap=0.045,
            name=f"{base_name} axle passes through wheel hub",
        )
        ctx.check(
            f"{base_name} caster swivels vertically",
            tuple(swivel.axis) == (0.0, 0.0, 1.0),
            details=f"axis={swivel.axis}",
        )
        ctx.check(
            f"{base_name} wheel spins on axle",
            tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={wheel_joint.axis}",
        )

    return ctx.report()


object_model = build_object_model()
