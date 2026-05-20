from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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


DECK_LENGTH = 0.44
DECK_WIDTH = 0.15
DECK_THICKNESS = 0.012
DECK_Z = 0.111
HINGE_Z = 0.078
TRUCK_X = 0.145
FOLD_LIMIT = 1.35
STEER_LIMIT = 0.38
WHEEL_RADIUS = 0.024
WHEEL_WIDTH = 0.020
WHEEL_Y = 0.096


def _deck_section(x: float, width: float, z_center: float, thickness: float):
    half = width * 0.5
    top = z_center + thickness * 0.5
    bottom = z_center - thickness * 0.5
    bevel = min(0.004, half * 0.35, thickness * 0.45)
    return [
        (x, -half + bevel, top),
        (x, half - bevel, top),
        (x, half, top - bevel),
        (x, half, bottom + bevel),
        (x, half - bevel, bottom),
        (x, -half + bevel, bottom),
        (x, -half, bottom + bevel),
        (x, -half, top - bevel),
    ]


def _deck_loft(width_scale: float, thickness: float, z_offset: float):
    sections = []
    nose_len = 0.065
    flat_half = DECK_LENGTH * 0.5 - nose_len
    for i in range(19):
        x = -DECK_LENGTH * 0.5 + DECK_LENGTH * i / 18
        over = max(0.0, abs(x) - flat_half)
        nose_t = over / nose_len
        width = DECK_WIDTH * width_scale * (1.0 - 0.70 * nose_t * nose_t)
        kick = 0.024 * nose_t * nose_t
        camber = 0.0025 * (1.0 - min(1.0, abs(x) / flat_half) ** 2)
        sections.append(_deck_section(x, width, z_offset + kick + camber, thickness))
    geom = MeshGeometry()
    for section in sections:
        for vx, vy, vz in section:
            geom.add_vertex(vx, vy, vz)
    count = len(sections[0])
    for i in range(len(sections) - 1):
        base = i * count
        nxt = (i + 1) * count
        for j in range(count):
            a = base + j
            b = base + (j + 1) % count
            c = nxt + (j + 1) % count
            d = nxt + j
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    # End caps close the thin deck shell, keeping it one connected manufactured part.
    for j in range(1, count - 1):
        geom.add_face(0, j, j + 1)
        end = (len(sections) - 1) * count
        geom.add_face(end, end + j + 1, end + j)
    return geom


def _add_deck_hardware(deck, truck_x: float) -> None:
    deck_bottom = DECK_Z - DECK_THICKNESS * 0.5
    bracket_height = deck_bottom - HINGE_Z + 0.003
    for y in (-0.054, 0.054):
        deck.visual(
            Box((0.020, 0.010, bracket_height)),
            origin=Origin(xyz=(truck_x, y, HINGE_Z + bracket_height * 0.5 - 0.0015)),
            material="brushed_aluminum",
            name=f"hinge_tab_{'p' if truck_x > 0 else 'n'}_{'a' if y < 0 else 'b'}",
        )
        deck.visual(
            Cylinder(radius=0.006, length=0.024),
            origin=Origin(xyz=(truck_x, y, HINGE_Z), rpy=(-math.pi / 2, 0.0, 0.0)),
            material="steel",
            name=f"outer_hinge_{'p' if truck_x > 0 else 'n'}_{'a' if y < 0 else 'b'}",
        )

    screw_z = DECK_Z + DECK_THICKNESS * 0.5 + 0.0002
    for ix, dx in enumerate((-0.018, 0.018)):
        for iy, y in enumerate((-0.026, 0.026)):
            deck.visual(
                Cylinder(radius=0.0032, length=0.0030),
                origin=Origin(xyz=(truck_x + dx, y, screw_z - 0.0005)),
                material="dark_steel",
                name=f"screw_{'p' if truck_x > 0 else 'n'}_{ix}_{iy}",
            )


def _make_wheel_meshes():
    tire = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.017,
        tread=TireTread(style="ribbed", depth=0.0018, count=18, land_ratio=0.64),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.0025, radius=0.0015),
    )
    rim = WheelGeometry(
        0.0172,
        WHEEL_WIDTH * 0.86,
        rim=WheelRim(inner_radius=0.010, flange_height=0.0025, flange_thickness=0.0015),
        hub=WheelHub(radius=0.0065, width=0.016, cap_style="flat"),
        face=WheelFace(dish_depth=0.0015, front_inset=0.001, rear_inset=0.001),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.0012, window_radius=0.003),
        bore=WheelBore(style="round", diameter=0.0075),
    )
    return (
        mesh_from_geometry(tire, "soft_ribbed_skate_tire"),
        mesh_from_geometry(rim, "mini_skate_wheel_core"),
    )


def _add_truck(model: ArticulatedObject, deck, *, name: str, x: float, fold_axis_y: float, pitch: float, tire_mesh, rim_mesh) -> None:
    truck = model.part(f"{name}_truck")
    truck.visual(
        Cylinder(radius=0.0052, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material="steel",
        name="center_hinge",
    )
    truck.visual(
        Box((0.032, 0.046, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material="brushed_aluminum",
        name="fold_leaf",
    )
    truck.visual(
        Box((0.030, 0.044, 0.023)),
        origin=Origin(xyz=(0.0, 0.0, -0.0145)),
        material="brushed_aluminum",
        name="drop_link",
    )
    truck.visual(
        Box((0.064, 0.055, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material="brushed_aluminum",
        name="baseplate",
    )
    truck.visual(
        Cylinder(radius=0.0115, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.026), rpy=(0.0, pitch, 0.0)),
        material="steel",
        name="top_washer",
    )
    truck.visual(
        Cylinder(radius=0.0105, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.0295), rpy=(0.0, pitch, 0.0)),
        material="amber_bushing",
        name="top_bushing",
    )
    truck.visual(
        Cylinder(radius=0.0110, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, -0.0335), rpy=(0.0, pitch, 0.0)),
        material="steel",
        name="center_washer",
    )
    truck.visual(
        Cylinder(radius=0.0100, length=0.0058),
        origin=Origin(xyz=(0.0, 0.0, -0.0367), rpy=(0.0, pitch, 0.0)),
        material="amber_bushing",
        name="lower_bushing",
    )
    truck.visual(
        Cylinder(radius=0.0032, length=0.027),
        origin=Origin(xyz=(0.0, 0.0, -0.0305), rpy=(0.0, pitch, 0.0)),
        material="dark_steel",
        name="kingpin",
    )
    model.articulation(
        f"deck_to_{name}_truck",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=truck,
        origin=Origin(xyz=(x, 0.0, HINGE_Z)),
        axis=(0.0, fold_axis_y, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=FOLD_LIMIT),
    )

    hanger = model.part(f"{name}_hanger")
    hanger.visual(
        Box((0.044, 0.052, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.0140)),
        material="brushed_aluminum",
        name="hanger_body",
    )
    hanger.visual(
        Box((0.024, 0.118, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, -0.0240)),
        material="brushed_aluminum",
        name="hanger_arms",
    )
    for y in (-0.045, 0.045):
        hanger.visual(
            Box((0.015, 0.010, 0.011)),
            origin=Origin(xyz=(0.0, y, -0.0195)),
            material="brushed_aluminum",
            name=f"arm_web_{0 if y < 0 else 1}",
        )
    hanger.visual(
        Cylinder(radius=0.0034, length=0.212),
        origin=Origin(xyz=(0.0, 0.0, -0.0240), rpy=(-math.pi / 2, 0.0, 0.0)),
        material="dark_steel",
        name="axle",
    )
    model.articulation(
        f"{name}_truck_to_hanger",
        ArticulationType.REVOLUTE,
        parent=truck,
        child=hanger,
        origin=Origin(xyz=(0.0, 0.0, -0.034), rpy=(0.0, pitch, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-STEER_LIMIT, upper=STEER_LIMIT),
    )

    for idx, y in enumerate((-WHEEL_Y, WHEEL_Y)):
        wheel = model.part(f"{name}_wheel_{idx}")
        wheel.visual(tire_mesh, material="soft_black_rubber", name="tire")
        wheel.visual(rim_mesh, material="warm_urethane", name="core")
        wheel.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(),
            material="steel",
            name="bearing_cap",
        )
        model.articulation(
            f"{name}_wheel_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=wheel,
            origin=Origin(xyz=(0.0, y, -0.0240), rpy=(0.0, 0.0, math.pi / 2)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=25.0),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_fold_flat_skateboard")
    model.material("maple_ply", rgba=(0.76, 0.55, 0.31, 1.0))
    model.material("charcoal_grip", rgba=(0.018, 0.020, 0.022, 1.0))
    model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.72, 1.0))
    model.material("steel", rgba=(0.48, 0.50, 0.52, 1.0))
    model.material("dark_steel", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("amber_bushing", rgba=(0.95, 0.58, 0.12, 1.0))
    model.material("soft_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    model.material("warm_urethane", rgba=(0.98, 0.78, 0.28, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(_deck_loft(1.0, DECK_THICKNESS, 0.0), "upturned_mini_deck"),
        origin=Origin(xyz=(0.0, 0.0, DECK_Z)),
        material="maple_ply",
        name="maple_deck",
    )
    deck.visual(
        mesh_from_geometry(_deck_loft(0.82, 0.0012, DECK_THICKNESS * 0.5 + 0.00065), "inset_grip_tape"),
        origin=Origin(xyz=(0.0, 0.0, DECK_Z)),
        material="charcoal_grip",
        name="grip_tape",
    )
    _add_deck_hardware(deck, TRUCK_X)
    _add_deck_hardware(deck, -TRUCK_X)

    tire_mesh, rim_mesh = _make_wheel_meshes()
    _add_truck(
        model,
        deck,
        name="front",
        x=TRUCK_X,
        fold_axis_y=1.0,
        pitch=-0.20,
        tire_mesh=tire_mesh,
        rim_mesh=rim_mesh,
    )
    _add_truck(
        model,
        deck,
        name="rear",
        x=-TRUCK_X,
        fold_axis_y=-1.0,
        pitch=0.20,
        tire_mesh=tire_mesh,
        rim_mesh=rim_mesh,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    expected_parts = {
        "deck",
        "front_truck",
        "rear_truck",
        "front_hanger",
        "rear_hanger",
        "front_wheel_0",
        "front_wheel_1",
        "rear_wheel_0",
        "rear_wheel_1",
    }
    ctx.check(
        "deck two trucks four wheels",
        expected_parts.issubset({part.name for part in object_model.parts}),
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "compact desktop footprint",
        DECK_LENGTH <= 0.46 and 2.0 * (WHEEL_Y + WHEEL_WIDTH * 0.5) <= 0.23,
        details=f"deck_length={DECK_LENGTH}, wheel_track={2.0 * (WHEEL_Y + WHEEL_WIDTH * 0.5)}",
    )

    for prefix in ("front", "rear"):
        truck = object_model.get_part(f"{prefix}_truck")
        hanger = object_model.get_part(f"{prefix}_hanger")
        steer = object_model.get_articulation(f"{prefix}_truck_to_hanger")
        fold = object_model.get_articulation(f"deck_to_{prefix}_truck")
        ctx.allow_overlap(
            truck,
            hanger,
            elem_a="kingpin",
            elem_b="hanger_body",
            reason="The steel kingpin intentionally passes through the drilled hanger boss to preload the bushing stack.",
        )
        ctx.expect_overlap(
            truck,
            hanger,
            axes="xy",
            min_overlap=0.003,
            elem_a="kingpin",
            elem_b="hanger_body",
            name=f"{prefix} kingpin captured in hanger boss",
        )
        ctx.expect_gap(
            deck,
            object_model.get_part(f"{prefix}_wheel_0"),
            axis="z",
            positive_elem="maple_deck",
            min_gap=0.030,
            name=f"{prefix} lower wheel clears deck",
        )
        ctx.expect_gap(
            deck,
            object_model.get_part(f"{prefix}_wheel_1"),
            axis="z",
            positive_elem="maple_deck",
            min_gap=0.030,
            name=f"{prefix} upper wheel clears deck",
        )
        ctx.expect_overlap(
            truck,
            hanger,
            axes="xy",
            min_overlap=0.015,
            elem_a="lower_bushing",
            elem_b="hanger_body",
            name=f"{prefix} hanger under bushing stack",
        )

        rest_pos = ctx.part_world_position(hanger)
        with ctx.pose({steer: STEER_LIMIT}):
            ctx.expect_gap(
                deck,
                object_model.get_part(f"{prefix}_wheel_0"),
                axis="z",
                positive_elem="maple_deck",
                min_gap=0.020,
                name=f"{prefix} steered wheel keeps deck clearance",
            )
            steered_pos = ctx.part_world_position(hanger)
        ctx.check(
            f"{prefix} steer axis is local kingpin",
            rest_pos is not None and steered_pos is not None and abs(steered_pos[1] - rest_pos[1]) < 0.004,
            details=f"rest={rest_pos}, steered={steered_pos}",
        )

        rest_wheel_pos = ctx.part_world_position(object_model.get_part(f"{prefix}_wheel_0"))
        with ctx.pose({fold: FOLD_LIMIT}):
            ctx.expect_gap(
                deck,
                object_model.get_part(f"{prefix}_wheel_0"),
                axis="z",
                positive_elem="maple_deck",
                min_gap=0.002,
                name=f"{prefix} folded wheel stays below deck",
            )
            folded_wheel_pos = ctx.part_world_position(object_model.get_part(f"{prefix}_wheel_0"))
        inward = (
            rest_wheel_pos is not None
            and folded_wheel_pos is not None
            and (folded_wheel_pos[0] < rest_wheel_pos[0] - 0.035 if prefix == "front" else folded_wheel_pos[0] > rest_wheel_pos[0] + 0.035)
        )
        ctx.check(
            f"{prefix} truck folds inward",
            inward,
            details=f"rest={rest_wheel_pos}, folded={folded_wheel_pos}",
        )

    return ctx.report()


object_model = build_object_model()
