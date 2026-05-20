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


BASE_RADIUS = 0.105
BASE_HEIGHT = 0.028
BASE_TOP_PLATE_HEIGHT = 0.004
BASE_TOP_Z = BASE_HEIGHT + BASE_TOP_PLATE_HEIGHT

BODY_RADIUS = 0.082
BODY_WALL = 0.003
BODY_HEIGHT = 0.218
BODY_TOP_Z = BODY_HEIGHT
FOOT_RADIUS = 0.079
FOOT_HEIGHT = 0.016
LID_RADIUS = 0.069
LID_THICKNESS = 0.010
LID_HINGE_Z = BODY_TOP_Z + 0.004


def _make_foot_ring() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(FOOT_RADIUS).extrude(FOOT_HEIGHT)
    inner = cq.Workplane("XY").circle(0.019).extrude(FOOT_HEIGHT + 0.001).translate((0.0, 0.0, -0.0005))
    return outer.cut(inner)


def _make_body_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_HEIGHT)
    inner = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS - BODY_WALL)
        .extrude(BODY_HEIGHT - BODY_WALL)
        .translate((0.0, 0.0, BODY_WALL))
    )
    coupler_socket = cq.Workplane("XY").circle(0.018).extrude(0.024)
    shell = outer.cut(inner).cut(coupler_socket)

    spout = (
        cq.Workplane("XZ")
        .center(0.0, 0.158)
        .rect(0.050, 0.030)
        .extrude(0.046)
        .translate((0.0, 0.078, 0.0))
    )

    return shell.union(spout)


def _make_handle() -> cq.Workplane:
    upper_boss = cq.Workplane("XY").box(0.034, 0.028, 0.032).translate((0.0, -0.086, 0.196))
    lower_boss = cq.Workplane("XY").box(0.032, 0.026, 0.032).translate((0.0, -0.085, 0.070))
    outer_grip = cq.Workplane("XY").box(0.028, 0.024, 0.165).translate((0.0, -0.124, 0.143))
    top_bridge = cq.Workplane("XY").box(0.028, 0.056, 0.028).translate((0.0, -0.105, 0.208))
    bottom_bridge = cq.Workplane("XY").box(0.028, 0.050, 0.028).translate((0.0, -0.104, 0.086))
    plunger_guide = cq.Workplane("XY").circle(0.010).extrude(0.012).translate((0.0, -0.104, 0.208))
    handle = (
        upper_boss.union(lower_boss)
        .union(outer_grip)
        .union(top_bridge)
        .union(bottom_bridge)
        .union(plunger_guide)
    )
    return handle


def _make_lid() -> cq.Workplane:
    lid_disk = cq.Workplane("XY").circle(LID_RADIUS).extrude(LID_THICKNESS)
    dome = (
        cq.Workplane("XY")
        .sphere(0.072)
        .translate((0.0, 0.0, -0.058))
        .intersect(cq.Workplane("XY").cylinder(0.018, 0.075).translate((0.0, 0.0, 0.009)))
    )
    front_tab = cq.Workplane("XY").box(0.022, 0.018, 0.010).translate((0.0, LID_RADIUS - 0.004, 0.005))
    lid = lid_disk.union(dome).union(front_tab)
    return lid.translate((0.0, LID_RADIUS, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_kettle")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))

    base_housing = model.part("base_housing")
    base_housing.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=black_plastic,
        name="base_shell",
    )
    base_housing.visual(
        Cylinder(radius=0.060, length=BASE_TOP_PLATE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + BASE_TOP_PLATE_HEIGHT / 2.0)),
        material=dark_trim,
        name="contact_plate",
    )
    base_housing.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + 0.005)),
        material=dark_trim,
        name="coupler",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_foot_ring(), "kettle_foot_ring"),
        material=black_plastic,
        name="foot_ring",
    )
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "kettle_body_shell"),
        material=stainless,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_make_handle(), "kettle_handle"),
        material=black_plastic,
        name="handle",
    )
    body.visual(
        Box((0.036, 0.018, 0.021)),
        origin=Origin(xyz=(0.0, -BODY_RADIUS + 0.006, BODY_TOP_Z - 0.006)),
        material=dark_trim,
        name="hinge_mount",
    )
    body.visual(
        Box((0.030, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.092, 0.060)),
        material=dark_trim,
        name="switch_cradle",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid(), "kettle_lid"),
        material=black_plastic,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.0045, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0075, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_trim,
        name="plunger_cap",
    )
    plunger.visual(
        Cylinder(radius=0.0038, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_trim,
        name="plunger_stem",
    )

    switch = model.part("switch")
    switch.visual(
        Cylinder(radius=0.0025, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="switch_axle",
    )
    switch.visual(
        Box((0.022, 0.013, 0.007)),
        origin=Origin(xyz=(0.0, 0.002, 0.0105)),
        material=black_plastic,
        name="rocker_paddle",
    )

    for index, x_pos in enumerate((-0.020, 0.020)):
        button = model.part(f"preset_button_{index}")
        button.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=dark_trim,
            name="button_cap",
        )
        model.articulation(
            f"base_to_preset_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base_housing,
            child=button,
            origin=Origin(xyz=(x_pos, 0.055, BASE_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.06,
                lower=0.0,
                upper=0.002,
            ),
        )

    model.articulation(
        "base_to_body",
        ArticulationType.PRISMATIC,
        parent=base_housing,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.40,
            lower=0.0,
            upper=0.120,
        ),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -LID_RADIUS, LID_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.85,
        ),
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, -0.104, 0.221)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )

    model.articulation(
        "body_to_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=switch,
        origin=Origin(xyz=(0.0, -0.092, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-0.32,
            upper=0.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_housing = object_model.get_part("base_housing")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    plunger = object_model.get_part("plunger")
    switch = object_model.get_part("switch")
    preset_button_0 = object_model.get_part("preset_button_0")
    preset_button_1 = object_model.get_part("preset_button_1")
    body_lift = object_model.get_articulation("base_to_body")
    lid_hinge = object_model.get_articulation("body_to_lid")
    plunger_slide = object_model.get_articulation("body_to_plunger")
    switch_pivot = object_model.get_articulation("body_to_switch")
    preset_joint_0 = object_model.get_articulation("base_to_preset_button_0")
    preset_joint_1 = object_model.get_articulation("base_to_preset_button_1")

    ctx.expect_gap(
        body,
        base_housing,
        axis="z",
        positive_elem="foot_ring",
        negative_elem="contact_plate",
        max_gap=0.0015,
        max_penetration=0.0,
        name="kettle sits on the base plate",
    )
    ctx.expect_overlap(
        body,
        base_housing,
        axes="xy",
        elem_a="foot_ring",
        elem_b="contact_plate",
        min_overlap=0.050,
        name="kettle footprint stays centered on the base",
    )

    with ctx.pose({body_lift: 0.080}):
        ctx.expect_gap(
            body,
            base_housing,
            axis="z",
            positive_elem="foot_ring",
            negative_elem="contact_plate",
            min_gap=0.075,
            name="kettle lifts cleanly off the base",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_shell",
            max_gap=0.0045,
            max_penetration=0.0,
            name="lid closes onto the top opening",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="body_shell",
            min_overlap=0.050,
            name="closed lid covers the kettle opening",
        )

    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.60}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_shell",
            min_gap=0.001,
            name="opened lid clears the body",
        )
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid opens upward",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.090,
        details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
    )

    plunger_rest = ctx.part_world_position(plunger)
    with ctx.pose({plunger_slide: 0.004}):
        plunger_pressed = ctx.part_world_position(plunger)
    ctx.check(
        "plunger presses downward",
        plunger_rest is not None and plunger_pressed is not None and plunger_pressed[2] < plunger_rest[2] - 0.003,
        details=f"rest={plunger_rest}, pressed={plunger_pressed}",
    )

    rocker_rest = ctx.part_element_world_aabb(switch, elem="rocker_paddle")
    with ctx.pose({switch_pivot: 0.32}):
        rocker_on = ctx.part_element_world_aabb(switch, elem="rocker_paddle")
    ctx.check(
        "rocker switch tips about its pivot",
        rocker_rest is not None
        and rocker_on is not None
        and rocker_on[1][2] > rocker_rest[1][2] + 0.001,
        details=f"rest={rocker_rest}, on={rocker_on}",
    )

    button_0_rest = ctx.part_world_position(preset_button_0)
    button_1_rest = ctx.part_world_position(preset_button_1)
    with ctx.pose({preset_joint_0: 0.002, preset_joint_1: 0.002}):
        button_0_pressed = ctx.part_world_position(preset_button_0)
        button_1_pressed = ctx.part_world_position(preset_button_1)
    ctx.check(
        "preset buttons depress",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_pressed is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.0015
        and button_1_pressed[2] < button_1_rest[2] - 0.0015,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
