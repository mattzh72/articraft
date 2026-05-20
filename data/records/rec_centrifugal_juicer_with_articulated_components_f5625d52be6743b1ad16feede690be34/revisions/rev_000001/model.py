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


LID_HINGE_XYZ = (0.0, 0.155, 0.295)
LID_RADIUS = 0.135
LID_CENTER_Y = -0.155
LID_THICKNESS = 0.026
CHUTE_CENTER = (-0.060, -0.125)
CHUTE_INNER = (0.052, 0.040)
CHUTE_WALL = 0.006
CHUTE_HEIGHT = 0.175
CHUTE_TOP_Z = LID_THICKNESS / 2.0 + CHUTE_HEIGHT
PUSHER_TRAVEL = 0.255
FLAP_HINGE = (
    CHUTE_CENTER[0],
    CHUTE_CENTER[1] + CHUTE_INNER[1] / 2.0 + 0.004,
    CHUTE_TOP_Z + 0.002,
)


def _base_housing_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").box(0.34, 0.26, 0.16).translate((0.0, 0.0, 0.080))
    housing = housing.edges("|Z").fillet(0.030)
    front_slope = cq.Workplane("XY").box(0.26, 0.055, 0.050).translate((0.0, -0.095, 0.175))
    top_deck = cq.Workplane("XY").circle(0.140).extrude(0.065).translate((0.0, 0.0, 0.145))
    return housing.union(front_slope).union(top_deck)


def _bowl_ring_shape() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(0.130).circle(0.108).extrude(0.045).translate((0.0, 0.0, 0.205))
    spout = cq.Workplane("XY").box(0.052, 0.095, 0.032).translate((0.0, -0.168, 0.225))
    outlet = cq.Workplane("XY").box(0.036, 0.050, 0.018).translate((0.0, -0.220, 0.224))
    return ring.union(spout).union(outlet)


def _lid_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .circle(LID_RADIUS)
        .extrude(LID_THICKNESS)
        .translate((0.0, LID_CENTER_Y, -LID_THICKNESS / 2.0))
    )
    opening = cq.Workplane("XY").box(CHUTE_INNER[0], CHUTE_INNER[1], LID_THICKNESS * 3.0).translate(
        (CHUTE_CENTER[0], CHUTE_CENTER[1], 0.0)
    )
    hinge_leaf = cq.Workplane("XY").box(0.076, 0.034, 0.010).translate((0.0, -0.014, 0.0))
    return plate.cut(opening).union(hinge_leaf)


def _basket_shape() -> cq.Workplane:
    wall = cq.Workplane("XY").circle(0.097).circle(0.083).extrude(0.042).translate((0.0, 0.0, -0.021))
    bottom = cq.Workplane("XY").circle(0.084).circle(0.017).extrude(0.008).translate((0.0, 0.0, -0.029))
    top_lip = cq.Workplane("XY").circle(0.103).circle(0.083).extrude(0.006).translate((0.0, 0.0, 0.021))
    hub = cq.Workplane("XY").circle(0.024).circle(0.017).extrude(0.030).translate((0.0, 0.0, -0.030))
    basket = wall.union(bottom).union(top_lip).union(hub)

    for angle in range(0, 360, 30):
        rib = (
            cq.Workplane("XY")
            .box(0.080, 0.005, 0.004)
            .translate((0.044, 0.0, 0.028))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        basket = basket.union(rib)
    return basket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_centrifugal_juicer")

    body = model.material("ivory_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("charcoal_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    metal = model.material("brushed_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    clear = model.material("clear_smoked_polycarbonate", rgba=(0.56, 0.82, 0.96, 0.34))
    amber = model.material("amber_flap", rgba=(0.92, 0.70, 0.38, 0.46))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_housing_shape(), "base_housing", tolerance=0.0015),
        material=body,
        name="base_housing",
    )
    base.visual(
        mesh_from_cadquery(_bowl_ring_shape(), "bowl_ring", tolerance=0.001),
        material=dark,
        name="bowl_ring",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.201)),
        material=metal,
        name="motor_coupler",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=metal,
        name="basket_thrust_ring",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.176, -0.036, 0.156), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="side_pivot_boss",
    )
    for side in (-1.0, 1.0):
        x_post = side * 0.066
        x_knuckle = side * 0.064
        base.visual(
            Box((0.040, 0.018, 0.034)),
            origin=Origin(xyz=(x_post, LID_HINGE_XYZ[1] + 0.004, LID_HINGE_XYZ[2] - 0.024)),
            material=dark,
            name=f"rear_hinge_cheek_{'neg' if side < 0.0 else 'pos'}",
        )
        base.visual(
            Box((0.038, 0.060, 0.070)),
            origin=Origin(xyz=(x_post, 0.135, 0.225)),
            material=dark,
            name=f"rear_hinge_post_{'neg' if side < 0.0 else 'pos'}",
        )
        base.visual(
            Cylinder(radius=0.008, length=0.040),
            origin=Origin(xyz=(x_knuckle, LID_HINGE_XYZ[1], LID_HINGE_XYZ[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"rear_hinge_knuckle_{'neg' if side < 0.0 else 'pos'}",
        )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "filter_basket", tolerance=0.0008),
        material=metal,
        name="filter_basket",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_plate_shape(), "clear_lid_plate", tolerance=0.001),
        material=clear,
        name="clear_lid_plate",
    )
    # Transparent rectangular chute walls leave a real open sleeve for the pusher.
    wall_bottom = LID_THICKNESS / 2.0
    wall_center_z = wall_bottom + CHUTE_HEIGHT / 2.0
    outer_x = CHUTE_INNER[0] + 2.0 * CHUTE_WALL
    outer_y = CHUTE_INNER[1] + 2.0 * CHUTE_WALL
    lid.visual(
        Box((CHUTE_WALL, outer_y, CHUTE_HEIGHT)),
        origin=Origin(xyz=(CHUTE_CENTER[0] + CHUTE_INNER[0] / 2.0 + CHUTE_WALL / 2.0, CHUTE_CENTER[1], wall_center_z)),
        material=clear,
        name="chute_wall_xpos",
    )
    lid.visual(
        Box((CHUTE_WALL, outer_y, CHUTE_HEIGHT)),
        origin=Origin(xyz=(CHUTE_CENTER[0] - CHUTE_INNER[0] / 2.0 - CHUTE_WALL / 2.0, CHUTE_CENTER[1], wall_center_z)),
        material=clear,
        name="chute_wall_xneg",
    )
    lid.visual(
        Box((outer_x, CHUTE_WALL, CHUTE_HEIGHT)),
        origin=Origin(xyz=(CHUTE_CENTER[0], CHUTE_CENTER[1] + CHUTE_INNER[1] / 2.0 + CHUTE_WALL / 2.0, wall_center_z)),
        material=clear,
        name="chute_wall_ypos",
    )
    lid.visual(
        Box((outer_x, CHUTE_WALL, CHUTE_HEIGHT)),
        origin=Origin(xyz=(CHUTE_CENTER[0], CHUTE_CENTER[1] - CHUTE_INNER[1] / 2.0 - CHUTE_WALL / 2.0, wall_center_z)),
        material=clear,
        name="chute_wall_yneg",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(CHUTE_CENTER[0] - 0.032, FLAP_HINGE[1], FLAP_HINGE[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="flap_hinge_knuckle_neg",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(CHUTE_CENTER[0] + 0.032, FLAP_HINGE[1], FLAP_HINGE[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="flap_hinge_knuckle_pos",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((CHUTE_INNER[0] - 0.008, CHUTE_INNER[1] - 0.008, 0.190)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=dark,
        name="pusher_stem",
    )
    pusher.visual(
        Box((CHUTE_INNER[0] + 0.024, CHUTE_INNER[1] - 0.002, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=body,
        name="pusher_stop_flange",
    )
    pusher.visual(
        Box((0.064, 0.047, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=body,
        name="pusher_grip",
    )
    pusher.visual(
        Box((0.046, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.174)),
        material=black,
        name="pusher_foot",
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="flap_hinge_barrel",
    )
    flap.visual(
        Box((0.064, 0.004, CHUTE_INNER[1] + 0.012)),
        origin=Origin(xyz=(0.0, 0.003, (CHUTE_INNER[1] + 0.012) / 2.0)),
        material=amber,
        name="flap_panel",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lever_hub",
    )
    lever.visual(
        Box((0.020, 0.015, 0.086)),
        origin=Origin(xyz=(0.017, 0.0, -0.006)),
        material=dark,
        name="lever_paddle",
    )
    lever.visual(
        Box((0.020, 0.018, 0.023)),
        origin=Origin(xyz=(0.034, 0.0, 0.037)),
        material=body,
        name="lever_thumb_pad",
    )

    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=120.0),
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=LID_HINGE_XYZ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.18),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_CENTER[0], CHUTE_CENTER[1], CHUTE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=PUSHER_TRAVEL),
    )
    model.articulation(
        "lid_to_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(xyz=FLAP_HINGE),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "base_to_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(0.183, -0.036, 0.156)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    flap = object_model.get_part("flap")
    basket = object_model.get_part("basket")
    lever = object_model.get_part("lever")

    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    flap_hinge = object_model.get_articulation("lid_to_flap")
    lever_pivot = object_model.get_articulation("base_to_lever")
    basket_spin = object_model.get_articulation("base_to_basket")

    ctx.check("basket uses continuous vertical spin", basket_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("pusher uses prismatic chute slide", pusher_slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("lid and flap are hinged", lid_hinge.articulation_type == ArticulationType.REVOLUTE and flap_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("side lever has rotary pivot", lever_pivot.articulation_type == ArticulationType.REVOLUTE)

    ctx.expect_gap(
        lid,
        pusher,
        axis="x",
        positive_elem="chute_wall_xpos",
        negative_elem="pusher_stem",
        min_gap=0.002,
        max_gap=0.006,
        name="pusher clears positive chute wall",
    )
    ctx.expect_gap(
        pusher,
        lid,
        axis="x",
        positive_elem="pusher_stem",
        negative_elem="chute_wall_xneg",
        min_gap=0.002,
        max_gap=0.006,
        name="pusher clears negative chute wall",
    )
    ctx.expect_gap(
        lid,
        pusher,
        axis="y",
        positive_elem="chute_wall_ypos",
        negative_elem="pusher_stem",
        min_gap=0.002,
        max_gap=0.006,
        name="pusher clears rear chute wall",
    )
    ctx.expect_gap(
        pusher,
        lid,
        axis="y",
        positive_elem="pusher_stem",
        negative_elem="chute_wall_yneg",
        min_gap=0.002,
        max_gap=0.006,
        name="pusher clears front chute wall",
    )
    ctx.expect_within(
        basket,
        base,
        axes="xy",
        inner_elem="filter_basket",
        outer_elem="bowl_ring",
        margin=0.004,
        name="rotating basket sits inside fixed bowl ring",
    )
    ctx.expect_gap(
        lever,
        base,
        axis="x",
        positive_elem="lever_hub",
        negative_elem="side_pivot_boss",
        min_gap=0.0,
        max_gap=0.004,
        name="lever hub sits just outside side pivot",
    )

    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="clear_lid_plate")
    with ctx.pose({lid_hinge: 0.9}):
        raised_lid_aabb = ctx.part_element_world_aabb(lid, elem="clear_lid_plate")
    ctx.check(
        "main lid opens upward from rear hinge",
        rest_lid_aabb is not None
        and raised_lid_aabb is not None
        and raised_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.045,
        details=f"rest={rest_lid_aabb}, raised={raised_lid_aabb}",
    )

    with ctx.pose({pusher_slide: PUSHER_TRAVEL, flap_hinge: math.pi / 2.0}):
        ctx.expect_gap(
            pusher,
            flap,
            axis="z",
            positive_elem="pusher_foot",
            negative_elem="flap_panel",
            min_gap=0.015,
            name="removed pusher clears closed chute flap",
        )
        ctx.expect_overlap(
            flap,
            lid,
            axes="xy",
            elem_a="flap_panel",
            elem_b="chute_wall_yneg",
            min_overlap=0.005,
            name="closed flap covers chute mouth",
        )

    return ctx.report()


object_model = build_object_model()
