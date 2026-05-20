from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _oval_base_plate() -> cq.Workplane:
    """Wide, low oval escutcheon plate, modeled in meters."""
    return (
        cq.Workplane("XY")
        .ellipse(0.210, 0.080)
        .extrude(0.018)
        .edges()
        .fillet(0.003)
    )


def _lever_handle() -> cq.Workplane:
    """A connected valve lever with a round skirt and upward sloping paddle."""
    lower_skirt = (
        cq.Workplane("XY")
        .circle(0.026)
        .extrude(0.020)
        .edges()
        .fillet(0.0012)
    )
    raised_hub = (
        cq.Workplane("XY")
        .circle(0.016)
        .extrude(0.032)
        .edges()
        .fillet(0.0010)
    )
    lever = (
        cq.Workplane("XY")
        .box(0.120, 0.020, 0.013)
        .edges("|X")
        .fillet(0.006)
        .edges("|Y")
        .fillet(0.003)
        .translate((0.056, 0.0, 0.028))
        .rotate((0.0, 0.0, 0.022), (0.0, 1.0, 0.022), -14.0)
    )
    return lower_skirt.union(raised_hub).union(lever)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laundry_utility_faucet")

    chrome = model.material("polished_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_chrome = model.material("dark_outlet_shadow", rgba=(0.03, 0.035, 0.04, 1.0))
    deck_material = model.material("white_laundry_deck", rgba=(0.90, 0.88, 0.82, 1.0))
    hot_red = model.material("hot_red", rgba=(0.85, 0.04, 0.03, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.02, 0.18, 0.82, 1.0))

    deck_top_z = 0.025
    plate_z = 0.023
    plate_top_z = plate_z + 0.018
    side_boss_radius = 0.031
    side_boss_height = 0.034
    side_boss_top_z = plate_top_z - 0.002 + side_boss_height
    valve_spacing = 0.120

    body = model.part("body")
    body.visual(
        Box((0.520, 0.280, deck_top_z)),
        origin=Origin(xyz=(0.0, 0.0, deck_top_z / 2.0)),
        material=deck_material,
        name="deck",
    )
    body.visual(
        mesh_from_cadquery(_oval_base_plate(), "oval_base_plate"),
        origin=Origin(xyz=(0.0, 0.0, plate_z)),
        material=chrome,
        name="oval_base_plate",
    )

    body.visual(
        Cylinder(radius=0.036, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, plate_top_z - 0.003 + 0.021)),
        material=chrome,
        name="center_boss",
    )
    for x, name in ((-valve_spacing, "hot_boss"), (valve_spacing, "cold_boss")):
        body.visual(
            Cylinder(radius=side_boss_radius, length=side_boss_height),
            origin=Origin(xyz=(x, 0.0, plate_top_z - 0.002 + side_boss_height / 2.0)),
            material=chrome,
            name=name,
        )

    # Straight center neck with a short forward outlet and downturned aerator.
    neck_bottom_z = plate_top_z - 0.003 + 0.042
    neck_length = 0.165
    neck_top_z = neck_bottom_z + neck_length
    body.visual(
        Cylinder(radius=0.018, length=neck_length),
        origin=Origin(xyz=(0.0, 0.0, neck_bottom_z + neck_length / 2.0)),
        material=chrome,
        name="straight_spout_neck",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, neck_bottom_z + 0.007)),
        material=chrome,
        name="neck_collar",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.088),
        origin=Origin(xyz=(0.0, -0.044, neck_top_z - 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="outlet_tube",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.046),
        origin=Origin(xyz=(0.0, -0.088, neck_top_z - 0.037)),
        material=chrome,
        name="downturned_nozzle",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.0, -0.088, neck_top_z - 0.061)),
        material=dark_chrome,
        name="aerator_face",
    )

    handle_mesh = _lever_handle()
    hot_handle = model.part("hot_handle")
    hot_handle.visual(
        mesh_from_cadquery(handle_mesh, "hot_lever_handle"),
        material=chrome,
        name="handle_shell",
    )
    hot_handle.visual(
        Cylinder(radius=0.007, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=hot_red,
        name="temperature_marker",
    )

    cold_handle = model.part("cold_handle")
    cold_handle.visual(
        mesh_from_cadquery(handle_mesh, "cold_lever_handle"),
        material=chrome,
        name="handle_shell",
    )
    cold_handle.visual(
        Cylinder(radius=0.007, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=cold_blue,
        name="temperature_marker",
    )

    for child, x, yaw, name in (
        (hot_handle, -valve_spacing, math.pi, "hot_valve"),
        (cold_handle, valve_spacing, 0.0, "cold_valve"),
    ):
        model.articulation(
            name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=child,
            origin=Origin(xyz=(x, 0.0, side_boss_top_z), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=-0.85, upper=0.85),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hot_handle = object_model.get_part("hot_handle")
    cold_handle = object_model.get_part("cold_handle")
    hot_valve = object_model.get_articulation("hot_valve")
    cold_valve = object_model.get_articulation("cold_valve")

    ctx.check(
        "two independent revolute valve controls",
        hot_valve.articulation_type == ArticulationType.REVOLUTE
        and cold_valve.articulation_type == ArticulationType.REVOLUTE
        and hot_valve.child == "hot_handle"
        and cold_valve.child == "cold_handle",
    )
    ctx.check(
        "valve stems flank the center spout",
        hot_valve.origin.xyz[0] < -0.08
        and cold_valve.origin.xyz[0] > 0.08
        and abs(hot_valve.origin.xyz[1]) < 0.002
        and abs(cold_valve.origin.xyz[1]) < 0.002,
    )
    ctx.check(
        "handles turn about vertical stems",
        abs(hot_valve.axis[2] - 1.0) < 1e-6 and abs(cold_valve.axis[2] - 1.0) < 1e-6,
    )

    for handle, boss, label in (
        (hot_handle, "hot_boss", "hot"),
        (cold_handle, "cold_boss", "cold"),
    ):
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem="handle_shell",
            negative_elem=boss,
            max_gap=0.002,
            max_penetration=0.0005,
            name=f"{label} handle seated on its boss",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="xy",
            elem_a="handle_shell",
            elem_b=boss,
            min_overlap=0.030,
            name=f"{label} handle remains centered on its boss",
        )

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    hot_rest_y = _aabb_center_y(ctx.part_element_world_aabb(hot_handle, elem="handle_shell"))
    cold_rest_y = _aabb_center_y(ctx.part_element_world_aabb(cold_handle, elem="handle_shell"))
    with ctx.pose({hot_valve: -0.65, cold_valve: 0.65}):
        hot_turned_y = _aabb_center_y(ctx.part_element_world_aabb(hot_handle, elem="handle_shell"))
        cold_turned_y = _aabb_center_y(ctx.part_element_world_aabb(cold_handle, elem="handle_shell"))
        ctx.expect_overlap(
            hot_handle,
            body,
            axes="xy",
            elem_a="handle_shell",
            elem_b="hot_boss",
            min_overlap=0.025,
            name="hot handle pivots around boss",
        )
        ctx.expect_overlap(
            cold_handle,
            body,
            axes="xy",
            elem_a="handle_shell",
            elem_b="cold_boss",
            min_overlap=0.025,
            name="cold handle pivots around boss",
        )

    ctx.check(
        "lever tips sweep when valves are posed",
        hot_rest_y is not None
        and cold_rest_y is not None
        and hot_turned_y is not None
        and cold_turned_y is not None
        and abs(hot_turned_y - hot_rest_y) > 0.015
        and abs(cold_turned_y - cold_rest_y) > 0.015,
    )

    return ctx.report()


object_model = build_object_model()
