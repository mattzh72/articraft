from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_X = 0.340
BODY_Y = 0.250
BODY_Z = 0.060

BOWL_RADIUS = 0.114
BOWL_WALL = 0.0045
BOWL_HEIGHT = 0.090
BOWL_Z = 0.082

LID_RADIUS = 0.117
LID_THICKNESS = 0.016
LID_WALL = 0.003
LID_HINGE_Y = 0.114
LID_HINGE_Z = 0.181

CHUTE_OUTER_X = 0.062
CHUTE_OUTER_Y = 0.088
CHUTE_INNER_X = 0.054
CHUTE_INNER_Y = 0.080
CHUTE_HEIGHT = 0.132
CHUTE_X = 0.056
CHUTE_Y = -0.085

DIAL_X = -0.080
BUTTON_XS = (0.008, 0.046, 0.084)
CONTROL_Z = 0.040


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _make_base_body() -> cq.Workplane:
    body = _rounded_box((BODY_X, BODY_Y, BODY_Z), 0.022).translate((0.0, 0.0, BODY_Z * 0.5))

    motor_hump = _rounded_box((0.242, 0.180, 0.024), 0.016).translate((0.0, 0.018, 0.066))
    bowl_collar = (
        cq.Workplane("XY")
        .circle(0.094)
        .circle(0.074)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.074))
    )
    rear_bridge = cq.Workplane("XY").box(0.180, 0.012, 0.020).translate((0.0, 0.120, 0.076))
    hinge_bar = cq.Workplane("XY").box(0.168, 0.010, 0.008).translate((0.0, 0.114, 0.1715))
    hinge_post_0 = cq.Workplane("XY").box(0.012, 0.016, 0.100).translate((-0.072, 0.118, 0.122))
    hinge_post_1 = cq.Workplane("XY").box(0.012, 0.016, 0.100).translate((0.072, 0.118, 0.122))

    shape = body.union(motor_hump).union(bowl_collar).union(rear_bridge).union(hinge_bar).union(hinge_post_0).union(hinge_post_1)

    control_pocket = cq.Workplane("XY").box(0.240, 0.020, 0.044).translate((0.002, -0.116, CONTROL_Z))
    shape = shape.cut(control_pocket)

    for button_x in BUTTON_XS:
        button_pocket = cq.Workplane("XY").box(0.028, 0.024, 0.020).translate((button_x, -0.108, CONTROL_Z))
        shape = shape.cut(button_pocket)

    return shape


def _make_control_band() -> cq.Workplane:
    band = _rounded_box((0.236, 0.008, 0.052), 0.003).translate((0.002, -0.1225, CONTROL_Z))

    dial_hole = (
        cq.Workplane("XZ")
        .circle(0.0225)
        .extrude(0.012)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((DIAL_X, -0.1225, CONTROL_Z))
    )
    band = band.cut(dial_hole)

    for button_x in BUTTON_XS:
        slot = cq.Workplane("XY").box(0.022, 0.012, 0.016).translate((button_x, -0.1225, CONTROL_Z))
        band = band.cut(slot)

    return band


def _make_bowl_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(BOWL_RADIUS).extrude(BOWL_HEIGHT)
    cavity = cq.Workplane("XY").circle(BOWL_RADIUS - BOWL_WALL).extrude(BOWL_HEIGHT - 0.004).translate((0.0, 0.0, 0.004))
    center_hole = cq.Workplane("XY").circle(0.015).extrude(BOWL_HEIGHT + 0.002).translate((0.0, 0.0, -0.001))
    hub_outer = cq.Workplane("XY").circle(0.026).extrude(0.034).translate((0.0, 0.0, 0.004))
    hub_inner = cq.Workplane("XY").circle(0.015).extrude(0.036).translate((0.0, 0.0, 0.003))

    bowl = outer.cut(cavity).cut(center_hole)
    hub = hub_outer.cut(hub_inner)
    return bowl.union(hub)


def _make_lid_plate() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .center(0.0, -LID_RADIUS)
        .circle(LID_RADIUS)
        .extrude(LID_THICKNESS)
        .translate((0.0, 0.0, -LID_THICKNESS * 0.5))
    )
    inner = (
        cq.Workplane("XY")
        .center(0.0, -LID_RADIUS)
        .circle(LID_RADIUS - LID_WALL)
        .extrude(LID_THICKNESS - LID_WALL + 0.001)
        .translate((0.0, 0.0, -LID_THICKNESS * 0.5 - 0.001))
    )
    chute_opening = (
        cq.Workplane("XY")
        .center(CHUTE_X, CHUTE_Y)
        .rect(CHUTE_INNER_X + 0.004, CHUTE_INNER_Y + 0.004)
        .extrude(LID_THICKNESS + 0.006)
        .translate((0.0, 0.0, -LID_THICKNESS * 0.5 - 0.003))
    )
    rear_band = cq.Workplane("XY").box(0.168, 0.014, 0.010).translate((0.0, -0.001, -0.001))
    return outer.union(rear_band).cut(inner).cut(chute_opening)


def _make_chute_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .rect(CHUTE_OUTER_X, CHUTE_OUTER_Y)
        .extrude(CHUTE_HEIGHT)
        .translate((CHUTE_X, CHUTE_Y, -0.008))
    )
    inner = (
        cq.Workplane("XY")
        .rect(CHUTE_INNER_X, CHUTE_INNER_Y)
        .extrude(CHUTE_HEIGHT + 0.010)
        .translate((CHUTE_X, CHUTE_Y, -0.009))
    )
    return outer.cut(inner)


def _make_pusher_stem() -> cq.Workplane:
    return cq.Workplane("XY").box(0.050, 0.072, 0.160).translate((0.0, 0.0, -0.078))


def _make_pusher_cap() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.070, 0.094, 0.014).translate((0.0, 0.0, 0.007))
    handle = cq.Workplane("XY").box(0.020, 0.050, 0.020).translate((0.0, 0.0, 0.024))
    return cap.union(handle)


def _make_cutter_disc() -> cq.Workplane:
    disc = (
        cq.Workplane("XY")
        .circle(0.092)
        .circle(0.020)
        .extrude(0.004)
        .translate((0.0, 0.0, -0.002))
    )
    hub_ring = (
        cq.Workplane("XY")
        .circle(0.026)
        .circle(0.020)
        .extrude(0.010)
        .translate((0.0, 0.0, -0.001))
    )
    center_bushing = cq.Workplane("XY").circle(0.013).extrude(0.010).translate((0.0, 0.0, -0.001))
    blade_0 = cq.Workplane("XY").box(0.090, 0.008, 0.004).translate((0.0, 0.0, 0.004)).rotate(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
        18.0,
    )
    blade_1 = cq.Workplane("XY").box(0.090, 0.008, 0.004).translate((0.0, 0.0, 0.004)).rotate(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
        198.0,
    )
    return disc.union(hub_ring).union(center_bushing).union(blade_0).union(blade_1)


def _make_button_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.024, 0.006, 0.012).translate((0.0, -0.003, 0.0))
    stem = cq.Workplane("XY").box(0.014, 0.012, 0.010).translate((0.0, 0.003, 0.0))
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_food_processor")

    body_white = model.material("body_white", rgba=(0.92, 0.93, 0.94, 1.0))
    control_dark = model.material("control_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    bowl_clear = model.material("bowl_clear", rgba=(0.82, 0.92, 0.96, 0.32))
    lid_clear = model.material("lid_clear", rgba=(0.84, 0.94, 0.98, 0.28))
    pusher_gray = model.material("pusher_gray", rgba=(0.82, 0.84, 0.86, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    spindle_dark = model.material("spindle_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    button_gray = model.material("button_gray", rgba=(0.86, 0.87, 0.88, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_body(), "food_processor_base"),
        material=body_white,
        name="body_shell",
    )
    base.visual(
        mesh_from_cadquery(_make_control_band(), "food_processor_control_band"),
        material=control_dark,
        name="control_band",
    )
    base.visual(
        Cylinder(radius=0.0095, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=spindle_dark,
        name="spindle",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_make_bowl_shape(), "food_processor_bowl"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bowl_clear,
        name="bowl_shell",
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, BOWL_Z)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_plate(), "food_processor_lid_plate"),
        material=lid_clear,
        name="lid_plate",
    )
    lid.visual(
        mesh_from_cadquery(_make_chute_shape(), "food_processor_chute"),
        material=lid_clear,
        name="chute_shell",
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        mesh_from_cadquery(_make_pusher_stem(), "food_processor_pusher_stem"),
        material=pusher_gray,
        name="pusher_stem",
    )
    pusher.visual(
        mesh_from_cadquery(_make_pusher_cap(), "food_processor_pusher_cap"),
        material=pusher_gray,
        name="pusher_cap",
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_X, CHUTE_Y, 0.124)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.088,
        ),
    )

    cutter_disc = model.part("cutter_disc")
    cutter_disc.visual(
        mesh_from_cadquery(_make_cutter_disc(), "food_processor_cutter_disc"),
        material=blade_steel,
        name="cutter_disc",
    )
    model.articulation(
        "base_to_cutter_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cutter_disc,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=24.0),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.020,
                body_style="skirted",
                top_diameter=0.034,
                skirt=KnobSkirt(0.050, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=18, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=12.0),
                center=False,
            ),
            "food_processor_selector_dial",
        ),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_dark,
        name="dial_shell",
    )
    model.articulation(
        "base_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=selector_dial,
        origin=Origin(xyz=(DIAL_X, -0.1265, CONTROL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            mesh_from_cadquery(_make_button_shape(), f"food_processor_button_{index}"),
            material=button_gray,
            name="button_body",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(button_x, -0.1265, CONTROL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0025,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    cutter_disc = object_model.get_part("cutter_disc")
    selector_dial = object_model.get_part("selector_dial")

    ctx.allow_overlap(
        base,
        cutter_disc,
        elem_a="spindle",
        elem_b="cutter_disc",
        reason="The cutter disc hub is intentionally nested onto the central drive spindle.",
    )
    for index in range(3):
        ctx.allow_overlap(
            base,
            object_model.get_part(f"button_{index}"),
            elem_a="control_band",
            elem_b="button_body",
            reason="Each preset button is modeled as a front cap bearing on the control band face.",
        )

    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")

    ctx.expect_overlap(
        lid,
        bowl,
        axes="xy",
        elem_a="lid_plate",
        elem_b="bowl_shell",
        min_overlap=0.20,
        name="lid covers the bowl opening",
    )
    ctx.expect_gap(
        lid,
        bowl,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="bowl_shell",
        min_gap=0.0005,
        max_gap=0.010,
        name="closed lid sits just above the bowl rim",
    )
    ctx.expect_within(
        cutter_disc,
        bowl,
        axes="xy",
        inner_elem="cutter_disc",
        outer_elem="bowl_shell",
        margin=0.0,
        name="cutter disc stays within the bowl footprint",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_stem",
        outer_elem="chute_shell",
        margin=0.0015,
        name="pusher stem fits inside the chute section",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_stem",
        elem_b="chute_shell",
        min_overlap=0.055,
        name="inserted pusher remains deeply engaged in the chute",
    )
    ctx.expect_origin_gap(
        base,
        selector_dial,
        axis="y",
        min_gap=0.11,
        name="selector dial sits on the front control band",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(lid, elem="lid_plate")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_plate_aabb = ctx.part_element_world_aabb(lid, elem="lid_plate")

    ctx.check(
        "rear_hinged_lid_rotates_upward",
        rest_plate_aabb is not None
        and open_plate_aabb is not None
        and open_plate_aabb[1][2] > rest_plate_aabb[1][2] + 0.070,
        details=f"closed={rest_plate_aabb}, open={open_plate_aabb}",
    )

    rest_pusher_aabb = ctx.part_element_world_aabb(pusher, elem="pusher_stem")
    with ctx.pose({pusher_slide: pusher_slide.motion_limits.upper}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_stem",
            outer_elem="chute_shell",
            margin=0.0015,
            name="extended pusher stays aligned in the chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_stem",
            elem_b="chute_shell",
            min_overlap=0.030,
            name="extended pusher still retains insertion in the chute",
        )
        extended_pusher_aabb = ctx.part_element_world_aabb(pusher, elem="pusher_stem")

    ctx.check(
        "pusher_slides_upward_out_of_the_chute",
        rest_pusher_aabb is not None
        and extended_pusher_aabb is not None
        and extended_pusher_aabb[0][2] > rest_pusher_aabb[0][2] + 0.070,
        details=f"rest={rest_pusher_aabb}, extended={extended_pusher_aabb}",
    )

    for index in range(3):
        joint = object_model.get_articulation(f"base_to_button_{index}")
        button = object_model.get_part(f"button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index}_presses_inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
