from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

CASE_WIDTH = 0.088
CASE_HEIGHT = 0.176
CASE_DEPTH = 0.036
BUMPER_WIDTH = 0.102
BUMPER_HEIGHT = 0.192
BUMPER_DEPTH = 0.046
FRONT_PLATE_THICK = 0.008
FRONT_FACE_Y = CASE_DEPTH * 0.5
BACK_FACE_Y = -CASE_DEPTH * 0.5
DISPLAY_CENTER_Z = 0.050
KEY_ROW_Z = 0.010
SELECTOR_CENTER_Z = -0.034
JACK_ROW_Z = -0.073
KEY_XS = (-0.027, -0.0135, 0.0, 0.0135, 0.027)


def _front_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(CASE_WIDTH, FRONT_PLATE_THICK, CASE_HEIGHT)
    panel = panel.edges("|Y").fillet(0.008)

    display_cut = (
        cq.Workplane("XY")
        .box(0.050, 0.020, 0.026)
        .translate((0.0, 0.0, DISPLAY_CENTER_Z))
    )
    panel = panel.cut(display_cut)

    dial_recess = (
        cq.Workplane("XZ")
        .circle(0.023)
        .extrude(0.0038)
        .translate((0.0, (FRONT_PLATE_THICK * 0.5) - 0.0038, SELECTOR_CENTER_Z))
    )
    shaft_cut = (
        cq.Workplane("XZ")
        .circle(0.006)
        .extrude(0.020)
        .translate((0.0, -0.010, SELECTOR_CENTER_Z))
    )
    panel = panel.cut(dial_recess).cut(shaft_cut)

    for x_pos in KEY_XS:
        slot = (
            cq.Workplane("XY")
            .box(0.0104, 0.0062, 0.0054)
            .translate((x_pos, 0.0009, KEY_ROW_Z))
        )
        panel = panel.cut(slot)

    return panel


def _bumper_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BUMPER_WIDTH, BUMPER_DEPTH, BUMPER_HEIGHT)
    outer = outer.edges("|Y").fillet(0.013)
    outer = outer.edges("|X").fillet(0.003)

    inner = cq.Workplane("XY").box(0.087, 0.060, 0.177)
    inner = inner.edges("|Y").fillet(0.0075)
    return outer.cut(inner)


def _bezel_shape() -> cq.Workplane:
    bezel = cq.Workplane("XY").box(0.068, 0.0026, 0.046)
    bezel = bezel.edges("|Y").fillet(0.0045)
    window = cq.Workplane("XY").box(0.052, 0.008, 0.030)
    return bezel.cut(window)


def _selector_ring_shape() -> cq.Workplane:
    ring = cq.Workplane("XZ").circle(0.028).circle(0.021).extrude(0.0016)
    return ring.translate((0.0, 0.0, SELECTOR_CENTER_Z))


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_multimeter")

    shell_dark = model.material("shell_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.32, 0.34, 0.36, 1.0))
    rubber_yellow = model.material("rubber_yellow", rgba=(0.87, 0.72, 0.18, 1.0))
    key_grey = model.material("key_grey", rgba=(0.74, 0.76, 0.78, 1.0))
    key_shadow = model.material("key_shadow", rgba=(0.48, 0.50, 0.52, 1.0))
    glass = model.material("display_glass", rgba=(0.28, 0.42, 0.35, 0.55))
    knob_black = model.material("knob_black", rgba=(0.12, 0.13, 0.14, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.95, 0.48, 0.12, 1.0))
    jack_black = model.material("jack_black", rgba=(0.10, 0.10, 0.11, 1.0))
    jack_red = model.material("jack_red", rgba=(0.77, 0.11, 0.10, 1.0))
    jack_amber = model.material("jack_amber", rgba=(0.80, 0.44, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_front_panel_shape(), "multimeter_front_panel"),
        origin=Origin(xyz=(0.0, FRONT_FACE_Y - (FRONT_PLATE_THICK * 0.5), 0.0)),
        material=shell_dark,
        name="front_panel",
    )
    body.visual(
        Box((0.084, 0.030, 0.172)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=shell_dark,
        name="rear_shell",
    )
    body.visual(
        Box((0.008, 0.018, 0.160)),
        origin=Origin(xyz=(-0.040, 0.004, 0.0)),
        material=shell_dark,
        name="left_frame",
    )
    body.visual(
        Box((0.008, 0.018, 0.160)),
        origin=Origin(xyz=(0.040, 0.004, 0.0)),
        material=shell_dark,
        name="right_frame",
    )
    body.visual(
        Box((0.072, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.004, 0.078)),
        material=shell_dark,
        name="top_frame",
    )
    body.visual(
        Box((0.072, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, 0.004, -0.072)),
        material=shell_dark,
        name="bottom_frame",
    )
    body.visual(
        mesh_from_cadquery(_bumper_shape(), "multimeter_bumper"),
        material=rubber_yellow,
        name="bumper",
    )
    body.visual(
        mesh_from_cadquery(_bezel_shape(), "multimeter_bezel"),
        origin=Origin(xyz=(0.0, FRONT_FACE_Y + 0.0013, DISPLAY_CENTER_Z)),
        material=shell_mid,
        name="lcd_bezel",
    )
    body.visual(
        Box((0.050, 0.007, 0.024)),
        origin=Origin(xyz=(0.0, 0.0135, DISPLAY_CENTER_Z)),
        material=glass,
        name="display_glass",
    )
    body.visual(
        mesh_from_cadquery(_selector_ring_shape(), "multimeter_selector_ring"),
        origin=Origin(xyz=(0.0, FRONT_FACE_Y, 0.0)),
        material=shell_mid,
        name="selector_ring",
    )
    body.visual(
        Box((0.060, 0.0024, 0.034)),
        origin=Origin(xyz=(0.0, FRONT_FACE_Y + 0.0012, JACK_ROW_Z)),
        material=shell_mid,
        name="jack_panel",
    )
    for name, x_pos, radius, material in (
        ("jack_com", -0.029, 0.0048, jack_black),
        ("jack_v", -0.010, 0.0050, jack_red),
        ("jack_ma", 0.011, 0.0048, jack_amber),
        ("jack_a", 0.031, 0.0052, jack_red),
    ):
        body.visual(
            Cylinder(radius=radius, length=0.0036),
            origin=Origin(
                xyz=(x_pos, FRONT_FACE_Y + 0.0018, JACK_ROW_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=name,
        )
    body.visual(
        Box((0.060, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, BACK_FACE_Y, -0.079)),
        material=shell_mid,
        name="stand_hinge_mount",
    )

    selector = model.part("selector")
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.014,
                body_style="skirted",
                top_diameter=0.032,
                base_diameter=0.040,
                edge_radius=0.0018,
                side_draft_deg=6.0,
                center=False,
            ),
            "multimeter_selector_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob",
    )
    selector.visual(
        Cylinder(radius=0.0042, length=0.0042),
        origin=Origin(xyz=(0.0, -0.0021, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shell_mid,
        name="shaft",
    )
    selector.visual(
        Box((0.004, 0.0014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0142, 0.013)),
        material=accent_orange,
        name="pointer",
    )

    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, FRONT_FACE_Y, SELECTOR_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.0026, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_mid,
        name="hinge_barrel",
    )
    stand.visual(
        Box((0.042, 0.0034, 0.078)),
        origin=Origin(xyz=(0.0, -0.0015, 0.040)),
        material=shell_mid,
        name="stand_leg",
    )
    stand.visual(
        Box((0.050, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.0028, 0.086)),
        material=shell_mid,
        name="stand_pad",
    )

    stand_limit = math.radians(68.0)
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, BACK_FACE_Y - 0.0028, -0.079)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=stand_limit,
        ),
    )

    key_travel = 0.0011
    for index, x_pos in enumerate(KEY_XS):
        key = model.part(f"key_{index}")
        key.visual(
            Box((0.0124, 0.0024, 0.0068)),
            origin=Origin(xyz=(0.0, 0.0024, 0.0)),
            material=key_grey,
            name="key_cap",
        )
        key.visual(
            Box((0.0092, 0.0052, 0.0054)),
            origin=Origin(xyz=(0.0, -0.0013, 0.0)),
            material=key_shadow,
            name="plunger",
        )
        model.articulation(
            f"body_to_key_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(x_pos, FRONT_FACE_Y, KEY_ROW_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.05,
                lower=0.0,
                upper=key_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")
    key_0 = object_model.get_part("key_0")
    key_4 = object_model.get_part("key_4")

    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_stand")
    key_0_joint = object_model.get_articulation("body_to_key_0")

    ctx.expect_gap(
        selector,
        body,
        axis="y",
        positive_elem="knob",
        negative_elem="front_panel",
        min_gap=0.0,
        max_gap=0.002,
        name="selector sits proud of the front panel",
    )
    ctx.expect_gap(
        body,
        key_0,
        axis="z",
        positive_elem="display_glass",
        negative_elem="key_cap",
        min_gap=0.012,
        name="soft keys sit below the display window",
    )
    ctx.expect_origin_distance(
        key_0,
        key_4,
        axes="z",
        min_dist=0.0,
        max_dist=0.0001,
        name="soft keys share one straight horizontal row",
    )

    pointer_rest = _center_from_aabb(ctx.part_element_world_aabb(selector, elem="pointer"))
    with ctx.pose({selector_joint: math.pi / 2.0}):
        pointer_quarter = _center_from_aabb(ctx.part_element_world_aabb(selector, elem="pointer"))
    ctx.check(
        "selector rotates about its center axis",
        pointer_rest is not None
        and pointer_quarter is not None
        and pointer_quarter[0] > pointer_rest[0] + 0.010
        and pointer_quarter[2] < pointer_rest[2] - 0.010,
        details=f"rest={pointer_rest}, quarter_turn={pointer_quarter}",
    )

    stand_closed = _center_from_aabb(ctx.part_element_world_aabb(stand, elem="stand_pad"))
    with ctx.pose({stand_joint: stand_joint.motion_limits.upper}):
        stand_open = _center_from_aabb(ctx.part_element_world_aabb(stand, elem="stand_pad"))
    ctx.check(
        "rear stand swings out from the back",
        stand_closed is not None
        and stand_open is not None
        and stand_open[1] < stand_closed[1] - 0.020,
        details=f"closed={stand_closed}, open={stand_open}",
    )

    key_rest = ctx.part_world_position(key_0)
    key_4_rest = ctx.part_world_position(key_4)
    with ctx.pose({key_0_joint: key_0_joint.motion_limits.upper}):
        key_pressed = ctx.part_world_position(key_0)
        key_4_still = ctx.part_world_position(key_4)
    ctx.check(
        "soft key presses inward",
        key_rest is not None and key_pressed is not None and key_pressed[1] < key_rest[1] - 0.001,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )
    ctx.check(
        "soft keys articulate independently",
        key_4_rest is not None
        and key_4_still is not None
        and abs(key_4_still[1] - key_4_rest[1]) < 1e-6,
        details=f"key_4_rest={key_4_rest}, key_4_still={key_4_still}",
    )

    return ctx.report()


object_model = build_object_model()
