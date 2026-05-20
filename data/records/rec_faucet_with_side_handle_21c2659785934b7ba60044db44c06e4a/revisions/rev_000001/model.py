from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_handle_kitchen_mixer_faucet")

    chrome = model.material("brushed_chrome", rgba=(0.72, 0.75, 0.76, 1.0))
    dark = model.material("dark_aerator", rgba=(0.02, 0.022, 0.024, 1.0))
    gasket = model.material("black_rubber_gasket", rgba=(0.005, 0.005, 0.004, 1.0))
    red = model.material("hot_mark", rgba=(0.90, 0.04, 0.03, 1.0))
    blue = model.material("cold_mark", rgba=(0.02, 0.14, 0.85, 1.0))

    body = model.part("body")

    # Thin rounded deck escutcheon with a black compression gasket below it.
    gasket_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.300, 0.105, 0.050, corner_segments=16),
        0.004,
        cap=True,
    ).translate(0.0, 0.0, -0.004)
    body.visual(
        mesh_from_geometry(gasket_geom, "deck_gasket"),
        material=gasket,
        name="deck_gasket",
    )

    deck_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.285, 0.092, 0.044, corner_segments=18),
        0.014,
        cap=True,
    )
    body.visual(
        mesh_from_geometry(deck_geom, "deck_plate"),
        material=chrome,
        name="deck_plate",
    )

    # Round pedestal and mixer body, built from stacked collars to read as a
    # plated faucet casting rather than a single plain cylinder.
    body.visual(
        Cylinder(radius=0.046, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=chrome,
        name="round_pedestal",
    )
    body.visual(
        Cylinder(radius=0.039, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=chrome,
        name="lower_collar",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.176),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=chrome,
        name="body_column",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=chrome,
        name="waist_band",
    )
    body.visual(
        Cylinder(radius=0.037, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.242)),
        material=chrome,
        name="top_collar",
    )

    # Side socket boss for the joystick cartridge.  Its axis points out of the
    # right side of the round body, and the moving ball is intentionally seated
    # partly inside it.
    body.visual(
        Cylinder(radius=0.019, length=0.032),
        origin=Origin(xyz=(0.0, 0.049, 0.160), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="socket_boss",
    )
    body.visual(
        Sphere(radius=0.0035),
        origin=Origin(xyz=(-0.013, 0.066, 0.178)),
        material=red,
        name="hot_dot",
    )
    body.visual(
        Sphere(radius=0.0035),
        origin=Origin(xyz=(0.013, 0.066, 0.178)),
        material=blue,
        name="cold_dot",
    )

    # High-arc gooseneck spout: one continuous bent tube rising from the body
    # top, arcing forward, and turning downward into the outlet.
    gooseneck = tube_from_spline_points(
        [
            (0.000, 0.0, 0.232),
            (0.000, 0.0, 0.320),
            (0.030, 0.0, 0.430),
            (0.100, 0.0, 0.505),
            (0.175, 0.0, 0.485),
            (0.220, 0.0, 0.390),
            (0.220, 0.0, 0.302),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=32,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    body.visual(
        mesh_from_geometry(gooseneck, "gooseneck"),
        material=chrome,
        name="gooseneck",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.048),
        origin=Origin(xyz=(0.220, 0.0, 0.278)),
        material=chrome,
        name="outlet_nozzle",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.220, 0.0, 0.252)),
        material=dark,
        name="aerator_screen",
    )

    pivot = model.part("pivot")
    pivot.visual(
        Sphere(radius=0.012),
        material=chrome,
        name="pivot_ball",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.013, length=0.034),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_hub",
    )
    lever.visual(
        Cylinder(radius=0.0052, length=0.052),
        origin=Origin(xyz=(0.0, 0.044, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lever_stem",
    )
    grip_geom = ExtrudeGeometry(
        superellipse_profile(0.040, 0.090, exponent=2.7, segments=56),
        0.011,
        cap=True,
        center=True,
    ).translate(0.0, 0.104, 0.0)
    lever.visual(
        mesh_from_geometry(grip_geom, "lever_grip"),
        material=chrome,
        name="lever_grip",
    )

    temperature = model.articulation(
        "temperature_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pivot,
        origin=Origin(xyz=(0.0, 0.067, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.2, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "flow_lift",
        ArticulationType.REVOLUTE,
        parent=pivot,
        child=lever,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=0.0, upper=0.62),
    )

    # Store intended nominal limits for tests without changing the public link
    # names or articulation names.
    temperature.meta["function"] = "temperature mixing pivot"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    pivot = object_model.get_part("pivot")
    lever = object_model.get_part("lever")
    temperature = object_model.get_articulation("temperature_pivot")
    flow = object_model.get_articulation("flow_lift")

    ctx.allow_overlap(
        body,
        pivot,
        elem_a="socket_boss",
        elem_b="pivot_ball",
        reason="The joystick ball is intentionally captured inside the side socket boss.",
    )
    ctx.expect_overlap(
        body,
        pivot,
        axes="xz",
        elem_a="socket_boss",
        elem_b="pivot_ball",
        min_overlap=0.018,
        name="pivot ball is seated in the side socket",
    )
    ctx.expect_gap(
        pivot,
        body,
        axis="y",
        positive_elem="pivot_ball",
        negative_elem="socket_boss",
        max_penetration=0.012,
        max_gap=0.002,
        name="captured joystick ball penetrates only locally into the socket",
    )

    ctx.allow_overlap(
        pivot,
        lever,
        elem_a="pivot_ball",
        elem_b="lever_hub",
        reason="The lever hub is a captured hinge barrel wrapped around the joystick ball.",
    )
    ctx.expect_overlap(
        pivot,
        lever,
        axes="xz",
        elem_a="pivot_ball",
        elem_b="lever_hub",
        min_overlap=0.020,
        name="lever hub is centered on the joystick pivot",
    )

    gooseneck_box = ctx.part_element_world_aabb(body, elem="gooseneck")
    nozzle_box = ctx.part_element_world_aabb(body, elem="outlet_nozzle")
    deck_box = ctx.part_element_world_aabb(body, elem="deck_plate")
    if gooseneck_box and nozzle_box and deck_box:
        ctx.check(
            "spout is a tall high arc over the deck",
            gooseneck_box[1][2] > deck_box[1][2] + 0.46 and nozzle_box[0][0] > 0.19,
            details=f"gooseneck={gooseneck_box}, nozzle={nozzle_box}, deck={deck_box}",
        )

    rest_grip = ctx.part_element_world_aabb(lever, elem="lever_grip")
    with ctx.pose({flow: 0.62}):
        lifted_grip = ctx.part_element_world_aabb(lever, elem="lever_grip")
    if rest_grip and lifted_grip:
        rest_tip_z = rest_grip[1][2]
        lifted_tip_z = lifted_grip[1][2]
        ctx.check(
            "flow lift raises the joystick handle",
            lifted_tip_z > rest_tip_z + 0.045,
            details=f"rest_tip_z={rest_tip_z}, lifted_tip_z={lifted_tip_z}",
        )

    with ctx.pose({temperature: 0.0}):
        neutral_grip = ctx.part_element_world_aabb(lever, elem="lever_grip")
    with ctx.pose({temperature: 0.55}):
        warm_grip = ctx.part_element_world_aabb(lever, elem="lever_grip")
    with ctx.pose({temperature: -0.55}):
        cold_grip = ctx.part_element_world_aabb(lever, elem="lever_grip")
    if neutral_grip and warm_grip and cold_grip:
        neutral_center_x = 0.5 * (neutral_grip[0][0] + neutral_grip[1][0])
        warm_center_x = 0.5 * (warm_grip[0][0] + warm_grip[1][0])
        cold_center_x = 0.5 * (cold_grip[0][0] + cold_grip[1][0])
        ctx.check(
            "temperature pivot swings the side lever laterally",
            warm_center_x < neutral_center_x - 0.035
            and cold_center_x > neutral_center_x + 0.035,
            details=f"cold_x={cold_center_x}, neutral_x={neutral_center_x}, warm_x={warm_center_x}",
        )

    return ctx.report()


object_model = build_object_model()
