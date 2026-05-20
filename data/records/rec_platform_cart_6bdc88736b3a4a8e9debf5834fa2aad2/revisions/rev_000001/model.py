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
    model = ArticulatedObject(name="sloped_stockroom_platform_cart")

    deck_blue = model.material("powder_coated_blue", rgba=(0.05, 0.18, 0.32, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.19, 1.0))

    # Object frame: +X is the low caster/front end, -X is the high rear axle and handle end.
    deck_length = 1.14
    deck_width = 0.62
    deck_thick = 0.055
    deck_pitch = math.radians(6.2)
    deck_center_z = 0.392

    def sloped_point(x: float, y: float, z: float) -> tuple[float, float, float]:
        """Map a point from the tilted deck's local coordinates to the cart frame."""
        c = math.cos(deck_pitch)
        s = math.sin(deck_pitch)
        return (c * x + s * z, y, deck_center_z - s * x + c * z)

    deck = model.part("deck")
    deck.visual(
        Box((deck_length, deck_width, deck_thick)),
        origin=Origin(xyz=(0.0, 0.0, deck_center_z), rpy=(0.0, deck_pitch, 0.0)),
        material=deck_blue,
        name="sloped_deck",
    )
    for idx, y in enumerate((-deck_width / 2 - 0.018, deck_width / 2 + 0.018)):
        deck.visual(
            Box((deck_length * 0.94, 0.036, 0.055)),
            origin=Origin(
                xyz=sloped_point(-0.01, y, deck_thick / 2 + 0.018),
                rpy=(0.0, deck_pitch, 0.0),
            ),
            material=deck_blue,
            name=f"side_lip_{idx}",
        )
    deck.visual(
        Box((0.050, deck_width + 0.045, 0.060)),
        origin=Origin(
            xyz=sloped_point(deck_length / 2 - 0.030, 0.0, deck_thick / 2 + 0.020),
            rpy=(0.0, deck_pitch, 0.0),
        ),
        material=deck_blue,
        name="front_lip",
    )

    # Deck-mounted bearing plates for the two front swivel casters.
    front_x = 0.455
    caster_y = 0.225
    caster_mounts: list[tuple[float, float, float]] = []
    for idx, y in enumerate((-caster_y, caster_y)):
        underside_z = sloped_point(front_x, y, -deck_thick / 2)[2]
        plate_z = underside_z - 0.012
        bearing_pivot_z = underside_z - 0.045
        caster_mounts.append((front_x, y, bearing_pivot_z))
        deck.visual(
            Box((0.165, 0.150, 0.030)),
            origin=Origin(xyz=(front_x, y, plate_z)),
            material=steel,
            name=f"caster_plate_{idx}",
        )
        deck.visual(
            Cylinder(radius=0.050, length=0.028),
            origin=Origin(xyz=(front_x, y, bearing_pivot_z + 0.014)),
            material=steel,
            name=f"caster_bearing_{idx}",
        )

    # Rear axle hanger tabs are part of the welded deck frame.
    rear_axle_x = -0.430
    rear_axle_z = 0.180
    rear_axle_radius = 0.022
    hanger_top_z = sloped_point(rear_axle_x, 0.0, -deck_thick / 2)[2] + 0.014
    hanger_bottom_z = rear_axle_z + rear_axle_radius
    for idx, y in enumerate((-0.255, 0.255)):
        deck.visual(
            Box((0.065, 0.038, hanger_top_z - hanger_bottom_z)),
            origin=Origin(xyz=(rear_axle_x, y, (hanger_top_z + hanger_bottom_z) / 2)),
            material=steel,
            name=f"rear_hanger_{idx}",
        )

    # Push handle: a welded U-shaped tube rising from the high rear end.
    handle_x = -0.575
    handle_top_z = 1.115
    handle_y = 0.265
    handle_base_z = sloped_point(-deck_length / 2 + 0.035, 0.0, deck_thick / 2)[2] - 0.005
    for idx, y in enumerate((-handle_y, handle_y)):
        deck.visual(
            Box((0.115, 0.075, 0.018)),
            origin=Origin(xyz=(handle_x, y, handle_base_z + 0.004)),
            material=steel,
            name=f"handle_foot_{idx}",
        )
        deck.visual(
            Cylinder(radius=0.023, length=handle_top_z - handle_base_z),
            origin=Origin(xyz=(handle_x, y, (handle_top_z + handle_base_z) / 2)),
            material=steel,
            name=f"handle_upright_{idx}",
        )
    deck.visual(
        Cylinder(radius=0.026, length=2 * handle_y + 0.055),
        origin=Origin(xyz=(handle_x, 0.0, handle_top_z), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="handle_grip",
    )

    # A fixed rear axle carried by the hanger tabs.
    rear_axle = model.part("rear_axle")
    rear_axle.visual(
        Cylinder(radius=rear_axle_radius, length=0.900),
        origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="axle_bar",
    )
    model.articulation(
        "deck_to_rear_axle",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_axle,
        origin=Origin(xyz=(rear_axle_x, 0.0, rear_axle_z)),
    )

    def add_wheel_part(
        name: str,
        tire_name: str,
        rim_name: str,
        tire_radius: float,
        tire_width: float,
        rim_radius: float,
        bore_diameter: float,
        tread_style: str,
    ):
        wheel = model.part(name)
        tire = TireGeometry(
            tire_radius,
            tire_width,
            inner_radius=rim_radius * 0.96,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            tread=TireTread(style=tread_style, depth=tire_radius * 0.045, count=20, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=tire_width * 0.10, depth=tire_radius * 0.015),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=tire_width * 0.11, radius=tire_radius * 0.02),
        )
        rim = WheelGeometry(
            rim_radius,
            tire_width * 0.88,
            rim=WheelRim(
                inner_radius=rim_radius * 0.58,
                flange_height=tire_radius * 0.035,
                flange_thickness=tire_width * 0.07,
                bead_seat_depth=tire_radius * 0.015,
            ),
            hub=WheelHub(
                radius=rim_radius * 0.33,
                width=tire_width * 0.78,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=rim_radius * 0.36,
                    hole_diameter=rim_radius * 0.055,
                ),
            ),
            face=WheelFace(dish_depth=tire_width * 0.07, front_inset=tire_width * 0.04, rear_inset=tire_width * 0.03),
            spokes=WheelSpokes(style="split_y", count=5, thickness=tire_width * 0.045, window_radius=rim_radius * 0.16),
            bore=WheelBore(style="round", diameter=bore_diameter),
        )
        wheel.visual(
            mesh_from_geometry(tire, tire_name),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(rim, rim_name),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=steel,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=bore_diameter * 0.55, length=tire_width * 0.78),
            origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
            material=steel,
            name="hub_bushing",
        )
        return wheel

    # Two large fixed rear wheels rotate independently on the fixed axle.
    rear_wheel_y = 0.390
    for idx, y in enumerate((-rear_wheel_y, rear_wheel_y)):
        rear_wheel = add_wheel_part(
            f"rear_wheel_{idx}",
            f"rear_tire_{idx}",
            f"rear_rim_{idx}",
            tire_radius=0.180,
            tire_width=0.085,
            rim_radius=0.134,
            bore_diameter=0.060,
            tread_style="block",
        )
        model.articulation(
            f"axle_to_rear_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=rear_axle,
            child=rear_wheel,
            origin=Origin(xyz=(0.0, y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=16.0),
        )

    # Front caster forks swivel about vertical pivots; each wheel spins on its own fork axle.
    caster_wheel_z = -0.180
    for idx, (x, y, pivot_z) in enumerate(caster_mounts):
        fork = model.part(f"caster_fork_{idx}")
        fork.visual(
            Cylinder(radius=0.018, length=0.080),
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
            material=steel,
            name="swivel_stem",
        )
        fork.visual(
            Cylinder(radius=0.044, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, -0.063)),
            material=steel,
            name="fork_crown",
        )
        fork.visual(
            Box((0.112, 0.148, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.065)),
            material=steel,
            name="fork_bridge",
        )
        for cheek_idx, cheek_y in enumerate((-0.065, 0.065)):
            fork.visual(
                Box((0.038, 0.018, 0.205)),
                origin=Origin(xyz=(0.0, cheek_y, -0.170)),
                material=steel,
                name=f"fork_cheek_{cheek_idx}",
            )
        fork.visual(
            Cylinder(radius=0.011, length=0.150),
            origin=Origin(xyz=(0.0, 0.0, caster_wheel_z), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name="caster_axle",
        )
        model.articulation(
            f"deck_to_caster_fork_{idx}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(x, y, pivot_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=8.0),
        )

        caster_wheel = add_wheel_part(
            f"caster_wheel_{idx}",
            f"caster_tire_{idx}",
            f"caster_rim_{idx}",
            tire_radius=0.090,
            tire_width=0.045,
            rim_radius=0.064,
            bore_diameter=0.030,
            tread_style="ribbed",
        )
        model.articulation(
            f"fork_to_caster_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, 0.0, caster_wheel_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    deck = object_model.get_part("deck")
    rear_axle = object_model.get_part("rear_axle")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")
    caster_fork_0 = object_model.get_part("caster_fork_0")
    caster_fork_1 = object_model.get_part("caster_fork_1")
    caster_wheel_0 = object_model.get_part("caster_wheel_0")
    caster_wheel_1 = object_model.get_part("caster_wheel_1")

    ctx.allow_overlap(
        rear_axle,
        rear_wheel_0,
        elem_a="axle_bar",
        elem_b="hub_bushing",
        reason="The rear wheel hub bushing is intentionally captured around the fixed axle proxy.",
    )
    ctx.allow_overlap(
        rear_axle,
        rear_wheel_1,
        elem_a="axle_bar",
        elem_b="hub_bushing",
        reason="The rear wheel hub bushing is intentionally captured around the fixed axle proxy.",
    )
    ctx.allow_overlap(
        caster_fork_0,
        caster_wheel_0,
        elem_a="caster_axle",
        elem_b="hub_bushing",
        reason="The front caster wheel bushing is intentionally captured around the fork axle proxy.",
    )
    ctx.allow_overlap(
        caster_fork_1,
        caster_wheel_1,
        elem_a="caster_axle",
        elem_b="hub_bushing",
        reason="The front caster wheel bushing is intentionally captured around the fork axle proxy.",
    )

    deck_box = ctx.part_element_world_aabb(deck, elem="sloped_deck")
    rear_wheel_box = ctx.part_world_aabb(rear_wheel_0)
    caster_wheel_box = ctx.part_world_aabb(caster_wheel_0)
    if deck_box is not None:
        deck_min, deck_max = deck_box
        ctx.check(
            "deck has stockroom cart length and width",
            (deck_max[0] - deck_min[0]) > 1.05 and (deck_max[1] - deck_min[1]) > 0.58,
            details=f"deck_box={deck_box}",
        )
    if rear_wheel_box is not None and caster_wheel_box is not None:
        rear_diameter = rear_wheel_box[1][2] - rear_wheel_box[0][2]
        caster_diameter = caster_wheel_box[1][2] - caster_wheel_box[0][2]
        ctx.check(
            "rear wheels are visibly larger than front casters",
            rear_diameter > caster_diameter * 1.7,
            details=f"rear_diameter={rear_diameter}, caster_diameter={caster_diameter}",
        )

    ctx.expect_contact(
        rear_axle,
        deck,
        elem_a="axle_bar",
        elem_b="rear_hanger_0",
        contact_tol=0.004,
        name="rear axle is carried by one hanger",
    )
    ctx.expect_contact(
        rear_axle,
        deck,
        elem_a="axle_bar",
        elem_b="rear_hanger_1",
        contact_tol=0.004,
        name="rear axle is carried by other hanger",
    )
    ctx.expect_contact(
        caster_fork_0,
        deck,
        elem_a="swivel_stem",
        elem_b="caster_bearing_0",
        contact_tol=0.004,
        name="first caster stem seats under bearing",
    )
    ctx.expect_contact(
        caster_fork_1,
        deck,
        elem_a="swivel_stem",
        elem_b="caster_bearing_1",
        contact_tol=0.004,
        name="second caster stem seats under bearing",
    )
    ctx.expect_gap(
        deck,
        caster_wheel_0,
        axis="z",
        min_gap=0.015,
        name="front caster wheel clears deck underside",
    )
    ctx.expect_gap(
        deck,
        caster_wheel_1,
        axis="z",
        min_gap=0.015,
        name="other front caster wheel clears deck underside",
    )
    ctx.expect_within(
        rear_axle,
        rear_wheel_0,
        axes="xz",
        inner_elem="axle_bar",
        outer_elem="hub_bushing",
        name="first rear axle passes through hub bushing",
    )
    ctx.expect_overlap(
        rear_wheel_0,
        rear_axle,
        axes="y",
        elem_a="hub_bushing",
        elem_b="axle_bar",
        min_overlap=0.030,
        name="first rear wheel is retained on axle line",
    )
    ctx.expect_within(
        rear_axle,
        rear_wheel_1,
        axes="xz",
        inner_elem="axle_bar",
        outer_elem="hub_bushing",
        name="second rear axle passes through hub bushing",
    )
    ctx.expect_overlap(
        rear_wheel_1,
        rear_axle,
        axes="y",
        elem_a="hub_bushing",
        elem_b="axle_bar",
        min_overlap=0.030,
        name="second rear wheel is retained on axle line",
    )
    ctx.expect_within(
        caster_fork_0,
        caster_wheel_0,
        axes="xz",
        inner_elem="caster_axle",
        outer_elem="hub_bushing",
        name="first caster axle passes through wheel bushing",
    )
    ctx.expect_within(
        caster_fork_1,
        caster_wheel_1,
        axes="xz",
        inner_elem="caster_axle",
        outer_elem="hub_bushing",
        name="second caster axle passes through wheel bushing",
    )
    with ctx.pose({"deck_to_caster_fork_0": math.pi / 2, "fork_to_caster_wheel_0": math.pi / 2}):
        ctx.expect_gap(
            deck,
            caster_wheel_0,
            axis="z",
            min_gap=0.020,
            name="swiveled caster still clears the deck",
        )
    with ctx.pose({"axle_to_rear_wheel_0": math.pi / 2}):
        ctx.expect_overlap(
            rear_wheel_0,
            rear_axle,
            axes="y",
            elem_a="hub_bushing",
            elem_b="axle_bar",
            min_overlap=0.030,
            name="rear wheel remains centered while rotating",
        )

    return ctx.report()


object_model = build_object_model()
