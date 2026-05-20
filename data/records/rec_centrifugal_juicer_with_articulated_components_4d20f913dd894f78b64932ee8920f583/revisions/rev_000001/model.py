from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.32
BASE_D = 0.26
BASE_H = 0.22
SHOULDER_W = 0.275
SHOULDER_D = 0.235
SHOULDER_H = 0.080
SHOULDER_Z0 = BASE_H
TOP_Z = SHOULDER_Z0 + SHOULDER_H
HINGE_Y = BASE_D / 2.0 + 0.004
HINGE_Z = TOP_Z + 0.015
CHUTE_X = 0.0
CHUTE_Y = -0.152
CHUTE_TOP_Z = 0.220
BIN_W = 0.180
BIN_D = 0.165
BIN_H = 0.150


def _rounded_box(sx: float, sy: float, sz: float, radius: float):
    """CadQuery solid with a rounded plan-view footprint and bottom on z=0."""
    shape = cq.Workplane("XY").box(sx, sy, sz).translate((0.0, 0.0, sz / 2.0))
    if radius > 0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _round_rect_plate_with_hole(
    sx: float,
    sy: float,
    sz: float,
    radius: float,
    hole_radius: float,
    hole_xy: tuple[float, float],
):
    outer = _rounded_box(sx, sy, sz, radius)
    cutter = (
        cq.Workplane("XY")
        .circle(hole_radius)
        .extrude(sz * 4.0)
        .translate((hole_xy[0], hole_xy[1], -sz * 1.5))
    )
    return outer.cut(cutter)


def _ring(outer_radius: float, inner_radius: float, height: float, center_xy=(0.0, 0.0), z0=0.0):
    outer = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(height)
        .translate((center_xy[0], center_xy[1], z0))
    )
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.010)
        .translate((center_xy[0], center_xy[1], z0 - 0.005))
    )
    return outer.cut(inner)


def _rect_tube(
    outer_x: float,
    outer_y: float,
    height: float,
    wall: float,
    center_xy=(0.0, 0.0),
    z0=0.0,
):
    outer = (
        cq.Workplane("XY")
        .box(outer_x, outer_y, height)
        .translate((center_xy[0], center_xy[1], z0 + height / 2.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(outer_x - 2.0 * wall, outer_y - 2.0 * wall, height + 0.030)
        .translate((center_xy[0], center_xy[1], z0 + height / 2.0))
    )
    return outer.cut(inner).edges("|Z").fillet(min(0.004, wall * 0.45))


def _open_top_bin(width: float, depth: float, height: float, wall: float):
    """Open-top rectangular pulp tub. Local origin is at the front lower center."""
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, depth / 2.0, height / 2.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.010)
        .translate((0.0, depth / 2.0, wall + (height + 0.010) / 2.0))
    )
    return outer.cut(inner).edges("|Z").fillet(0.004)


def _basket_mesh():
    wall = _ring(0.066, 0.060, 0.052, z0=0.006)
    top_lip = _ring(0.071, 0.056, 0.006, z0=0.058)
    bottom = _ring(0.063, 0.010, 0.004, z0=0.004)
    ribs = cq.Workplane("XY")
    for angle in range(0, 180, 30):
        rib = (
            cq.Workplane("XY")
            .box(0.105, 0.0022, 0.007)
            .translate((0.0, 0.0, 0.009))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        ribs = ribs.union(rib)
    return wall.union(top_lip).union(bottom).union(ribs)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_hinged_juicer")
    model.material("warm_white", rgba=(0.92, 0.90, 0.84, 1.0))
    model.material("soft_gray", rgba=(0.55, 0.58, 0.60, 1.0))
    model.material("dark_slot", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("clear_smoke", rgba=(0.55, 0.83, 1.0, 0.36))
    model.material("clear_bin", rgba=(0.36, 0.39, 0.42, 0.46))
    model.material("stainless", rgba=(0.82, 0.84, 0.82, 1.0))
    model.material("blade", rgba=(0.95, 0.97, 0.95, 1.0))
    model.material("green_button", rgba=(0.10, 0.58, 0.24, 1.0))
    model.material("orange_button", rgba=(0.95, 0.48, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_box(BASE_W, BASE_D, BASE_H, 0.042), "base_housing"),
        material="warm_white",
        name="housing",
    )
    base.visual(
        mesh_from_cadquery(
            _round_rect_plate_with_hole(SHOULDER_W, SHOULDER_D, SHOULDER_H, 0.035, 0.086, (0.0, -0.012)),
            "top_shoulder",
        ),
        origin=Origin(xyz=(0.0, -0.006, SHOULDER_Z0)),
        material="warm_white",
        name="top_shoulder",
    )
    base.visual(
        mesh_from_cadquery(_ring(0.092, 0.072, 0.018, center_xy=(0.0, -0.018), z0=0.0), "bowl_rim"),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        material="soft_gray",
        name="bowl_rim",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.032),
        origin=Origin(xyz=(0.0, -0.018, 0.235)),
        material="stainless",
        name="drive_spindle",
    )
    base.visual(
        Box((0.118, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, BASE_D / 2.0 + 0.002, 0.135)),
        material="dark_slot",
        name="rear_slot",
    )
    for x, rail_name in ((-0.105, "rear_rail_0"), (0.105, "rear_rail_1")):
        base.visual(
            Box((0.018, 0.165, 0.020)),
            origin=Origin(xyz=(x, BASE_D / 2.0 + 0.080, 0.089)),
            material="soft_gray",
            name=rail_name,
        )
    for i, x in enumerate((-0.078, 0.078)):
        base.visual(
            Box((0.052, 0.016, 0.024)),
            origin=Origin(xyz=(x, HINGE_Y - 0.015, HINGE_Z - 0.015)),
            material="soft_gray",
            name=f"hinge_mount_{i}",
        )
        base.visual(
            Cylinder(radius=0.008, length=0.052),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="soft_gray",
            name=f"hinge_barrel_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(
            _round_rect_plate_with_hole(
                0.250,
                0.220,
                0.016,
                0.032,
                0.078,
                (CHUTE_X, CHUTE_Y + 0.100),
            ),
            "lid_plate",
        ),
        origin=Origin(xyz=(0.0, -0.100, 0.006)),
        material="clear_smoke",
        name="lid_plate",
    )
    lid.visual(
        mesh_from_cadquery(_ring(0.105, 0.078, 0.052, center_xy=(CHUTE_X, CHUTE_Y), z0=0.020), "lid_dome_wall"),
        material="clear_smoke",
        name="dome_wall",
    )
    lid.visual(
        mesh_from_cadquery(_rect_tube(0.074, 0.064, 0.201, 0.006, center_xy=(CHUTE_X, CHUTE_Y), z0=0.019), "feed_chute"),
        material="clear_smoke",
        name="feed_chute",
    )
    lid.visual(
        mesh_from_cadquery(_rect_tube(0.170, 0.150, 0.010, 0.050, center_xy=(CHUTE_X, CHUTE_Y), z0=0.016), "chute_flange"),
        material="clear_smoke",
        name="chute_flange",
    )
    lid.visual(
        Cylinder(radius=0.0075, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="soft_gray",
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.078, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, -0.014, 0.006)),
        material="clear_smoke",
        name="hinge_leaf",
    )

    pusher = model.part("feed_pusher")
    pusher.visual(
        Box((0.048, 0.038, 0.165)),
        origin=Origin(xyz=(0.0, 0.0, -0.0825)),
        material=Material("pusher_white", rgba=(0.93, 0.94, 0.92, 1.0)),
        name="pusher_stem",
    )
    pusher.visual(
        mesh_from_cadquery(_rounded_box(0.084, 0.070, 0.024, 0.010), "pusher_cap"),
        material="warm_white",
        name="pusher_cap",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_mesh(), "cutting_basket"),
        material="stainless",
        name="sieve_basket",
    )
    basket.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="stainless",
        name="hub",
    )
    for i, angle in enumerate((18.0, 198.0)):
        basket.visual(
            Box((0.074, 0.009, 0.004)),
            origin=Origin(xyz=(0.020 * math.cos(math.radians(angle)), 0.020 * math.sin(math.radians(angle)), 0.026), rpy=(0.0, 0.0, math.radians(angle))),
            material="blade",
            name=f"cutting_blade_{i}",
        )

    pulp_bin = model.part("pulp_bin")
    pulp_bin.visual(
        mesh_from_cadquery(_open_top_bin(BIN_W, BIN_D, BIN_H, 0.005), "pulp_bin_tub"),
        material="clear_bin",
        name="tub",
    )
    pulp_bin.visual(
        Box((0.106, 0.014, 0.034)),
        origin=Origin(xyz=(0.0, BIN_D + 0.004, 0.086)),
        material="dark_slot",
        name="pull_handle",
    )
    for x, runner_name in ((-0.093, "side_runner_0"), (0.093, "side_runner_1")):
        pulp_bin.visual(
            Box((0.006, 0.120, 0.012)),
            origin=Origin(xyz=(x, 0.082, 0.034)),
            material="soft_gray",
            name=runner_name,
        )

    buttons = []
    for i, (x, mat) in enumerate(((-0.054, "green_button"), (0.054, "orange_button"))):
        button = model.part(f"button_{i}")
        button.visual(
            mesh_from_cadquery(_rounded_box(0.040, 0.026, 0.012, 0.006), f"button_cap_{i}"),
            material=mat,
            name="button_cap",
        )
        buttons.append((button, x))

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.35),
    )
    model.articulation(
        "pusher_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_X, CHUTE_Y, CHUTE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=0.105),
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, -0.018, TOP_Z - 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )
    model.articulation(
        "pulp_bin_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pulp_bin,
        origin=Origin(xyz=(0.0, BASE_D / 2.0 + 0.005, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.145),
    )
    for i, (button, x) in enumerate(buttons):
        model.articulation(
            f"button_{i}_plunger",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.116, TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lid = object_model.get_part("lid")
    base = object_model.get_part("base")
    pusher = object_model.get_part("feed_pusher")
    pulp_bin = object_model.get_part("pulp_bin")
    button_0 = object_model.get_part("button_0")
    basket_joint = object_model.get_articulation("basket_spin")
    lid_joint = object_model.get_articulation("lid_hinge")
    pusher_joint = object_model.get_articulation("pusher_slide")
    bin_joint = object_model.get_articulation("pulp_bin_slide")
    button_joint = object_model.get_articulation("button_0_plunger")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.002,
        max_gap=0.012,
        positive_elem="lid_plate",
        negative_elem="bowl_rim",
        name="closed clear lid rests just above bowl rim",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_stem",
        outer_elem="feed_chute",
        margin=0.0,
        name="rectangular pusher stays inside the feed chute footprint",
    )
    ctx.expect_contact(
        pulp_bin,
        base,
        elem_a="side_runner_0",
        elem_b="rear_rail_0",
        contact_tol=0.001,
        name="pulp bin side runner is guided by rear rail",
    )
    ctx.expect_gap(
        button_0,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="button_cap",
        negative_elem="top_shoulder",
        name="control button sits on top shoulder",
    )
    ctx.check(
        "basket joint is continuous",
        getattr(basket_joint, "articulation_type", None) == ArticulationType.CONTINUOUS
        or getattr(basket_joint, "type", None) == ArticulationType.CONTINUOUS,
        details=f"basket articulation={basket_joint}",
    )

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.075}):
        lowered_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "feed pusher plunges downward through chute",
        rest_pusher is not None
        and lowered_pusher is not None
        and lowered_pusher[2] < rest_pusher[2] - 0.050,
        details=f"rest={rest_pusher}, lowered={lowered_pusher}",
    )

    rest_bin = ctx.part_world_position(pulp_bin)
    with ctx.pose({bin_joint: 0.120}):
        extended_bin = ctx.part_world_position(pulp_bin)
    ctx.check(
        "rear pulp bin slides outward",
        rest_bin is not None and extended_bin is not None and extended_bin[1] > rest_bin[1] + 0.10,
        details=f"rest={rest_bin}, extended={extended_bin}",
    )

    rest_button = ctx.part_world_position(button_0)
    with ctx.pose({button_joint: 0.005}):
        pressed_button = ctx.part_world_position(button_0)
    ctx.check(
        "control button depresses on short plunger",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[2] < rest_button[2] - 0.004,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    closed_plate = ctx.part_element_world_aabb(lid, elem="lid_plate")
    with ctx.pose({lid_joint: 1.0}):
        open_plate = ctx.part_element_world_aabb(lid, elem="lid_plate")
    ctx.check(
        "hinged lid swings upward from rear hinge",
        closed_plate is not None and open_plate is not None and open_plate[1][2] > closed_plate[1][2] + 0.045,
        details=f"closed={closed_plate}, open={open_plate}",
    )

    return ctx.report()


object_model = build_object_model()
