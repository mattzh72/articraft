from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.250
BASE_DEPTH = 0.220
BASE_HEIGHT = 0.112

BOWL_CENTER = (0.0, 0.030, 0.122)
BOWL_HEIGHT = 0.145
BOWL_TOP_RADIUS = 0.095

LID_THICKNESS = 0.010
LID_OUTER_RADIUS = 0.104
LID_SKIRT_INNER_RADIUS = 0.097
LID_SKIRT_DEPTH = 0.018

MAIN_TUBE_POS = (0.014, 0.020)
MAIN_TUBE_OUTER = (0.060, 0.086)
MAIN_TUBE_INNER = (0.050, 0.076)
MAIN_TUBE_HEIGHT = 0.106

SMALL_TUBE_POS = (0.070, -0.030)
SMALL_TUBE_OUTER_RADIUS = 0.020
SMALL_TUBE_INNER_RADIUS = 0.014
SMALL_TUBE_HEIGHT = 0.060

CONTROL_PANEL_CENTER = (0.0, -0.110, 0.068)
CONTROL_PANEL_OUTER_Y = CONTROL_PANEL_CENTER[1] - 0.005
DIAL_POS = (-0.056, CONTROL_PANEL_OUTER_Y, 0.068)
BUTTON_LAYOUT = (
    ("button_0", (0.030, CONTROL_PANEL_OUTER_Y, 0.080)),
    ("button_1", (0.060, CONTROL_PANEL_OUTER_Y, 0.080)),
    ("button_2", (0.030, CONTROL_PANEL_OUTER_Y, 0.054)),
    ("button_3", (0.060, CONTROL_PANEL_OUTER_Y, 0.054)),
)


def _filleted_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size, centered=(True, True, False))
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _build_base_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT, centered=(True, True, False))
    body = body.edges("|Z").fillet(0.028)
    body = body.faces(">Z").edges().fillet(0.010)
    return body


def _build_control_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.190, 0.010, 0.072, centered=(True, True, True))
    panel = panel.edges("|Y").fillet(0.008)

    recess_depth = 0.006
    for _, (x, _, z) in BUTTON_LAYOUT:
        pocket = cq.Workplane("XY").box(0.026, recess_depth, 0.020, centered=(True, True, True))
        panel = panel.cut(pocket.translate((x, -0.002, z - CONTROL_PANEL_CENTER[2])))

    return panel


def _build_handle_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.034, 0.028, 0.104, centered=(True, True, True)).translate((0.108, 0.0, 0.072))
    back_pad = cq.Workplane("XY").box(0.020, 0.028, 0.110, centered=(True, True, True)).translate((0.089, 0.0, 0.072))
    opening = cq.Workplane("XY").box(0.015, 0.032, 0.064, centered=(True, True, True)).translate((0.111, 0.0, 0.073))
    return outer.union(back_pad).cut(opening)


def _build_lid_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").circle(LID_OUTER_RADIUS).extrude(LID_THICKNESS)
    skirt = (
        cq.Workplane("XY")
        .circle(LID_OUTER_RADIUS)
        .circle(LID_SKIRT_INNER_RADIUS)
        .extrude(-LID_SKIRT_DEPTH)
    )
    center_rise = cq.Workplane("XY").circle(0.038).extrude(0.007)

    main_open = _filleted_box((MAIN_TUBE_INNER[0], MAIN_TUBE_INNER[1], LID_THICKNESS + 0.006), 0.008).translate(
        (MAIN_TUBE_POS[0], MAIN_TUBE_POS[1], -0.003)
    )
    small_open = (
        cq.Workplane("XY")
        .circle(SMALL_TUBE_INNER_RADIUS)
        .extrude(LID_THICKNESS + 0.006)
        .translate((SMALL_TUBE_POS[0], SMALL_TUBE_POS[1], -0.003))
    )

    return shell.union(skirt).union(center_rise).cut(main_open).cut(small_open)


def _build_main_tube_shape() -> cq.Workplane:
    outer = _filleted_box((MAIN_TUBE_OUTER[0], MAIN_TUBE_OUTER[1], MAIN_TUBE_HEIGHT), 0.010)
    inner = _filleted_box((MAIN_TUBE_INNER[0], MAIN_TUBE_INNER[1], MAIN_TUBE_HEIGHT + 0.004), 0.008).translate(
        (0.0, 0.0, -0.002)
    )
    return outer.cut(inner)


def _build_small_tube_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(SMALL_TUBE_OUTER_RADIUS).extrude(SMALL_TUBE_HEIGHT)
    inner = cq.Workplane("XY").circle(SMALL_TUBE_INNER_RADIUS).extrude(SMALL_TUBE_HEIGHT + 0.004).translate((0.0, 0.0, -0.002))
    return outer.cut(inner)


def _build_blade_carrier_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.018).circle(0.0075).extrude(0.018)
    collar = cq.Workplane("XY").circle(0.014).circle(0.0075).extrude(0.012).translate((0.0, 0.0, -0.006))
    blade_1 = cq.Workplane("XY").box(0.118, 0.016, 0.003, centered=(True, True, True)).translate((0.0, 0.0, 0.012))
    blade_1 = blade_1.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 14.0)
    blade_2 = cq.Workplane("XY").box(0.094, 0.014, 0.003, centered=(True, True, True)).translate((0.0, 0.0, 0.004))
    blade_2 = blade_2.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -12.0).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 92.0)
    return hub.union(collar).union(blade_1).union(blade_2)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_food_processor")

    body_white = model.material("body_white", rgba=(0.94, 0.94, 0.92, 1.0))
    panel_black = model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.82, 0.88, 0.92, 0.32))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    knob_graphite = model.material("knob_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    control_silver = model.material("control_silver", rgba=(0.76, 0.77, 0.79, 1.0))
    pusher_dark = model.material("pusher_dark", rgba=(0.22, 0.23, 0.24, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_build_base_shape(), "food_processor_base"), material=body_white, name="base_shell")
    base.visual(
        mesh_from_cadquery(_build_control_panel_shape(), "food_processor_panel"),
        origin=Origin(xyz=CONTROL_PANEL_CENTER),
        material=panel_black,
        name="control_panel",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.0, BOWL_CENTER[1], 0.123)),
        material=steel,
        name="shaft",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, BOWL_CENTER[1], 0.119)),
        material=steel,
        name="drive_hub",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(xyz=(0.0, BOWL_CENTER[1], BASE_HEIGHT + 0.005)),
        material=steel,
        name="bowl_seat",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.056, 0.000),
                    (0.062, 0.012),
                    (0.074, 0.050),
                    (0.086, 0.108),
                    (0.095, 0.145),
                ],
                [
                    (0.000, 0.004),
                    (0.054, 0.012),
                    (0.067, 0.050),
                    (0.079, 0.108),
                    (0.089, 0.145),
                ],
                segments=72,
            ),
            "food_processor_bowl",
        ),
        material=clear_smoke,
        name="bowl_shell",
    )
    model.articulation("base_to_bowl", ArticulationType.FIXED, parent=base, child=bowl, origin=Origin(xyz=BOWL_CENTER))

    handle = model.part("handle")
    handle.visual(mesh_from_cadquery(_build_handle_shape(), "food_processor_handle"), material=clear_smoke, name="handle_shell")
    model.articulation("bowl_to_handle", ArticulationType.FIXED, parent=bowl, child=handle, origin=Origin())

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_build_lid_shell_shape(), "food_processor_lid"), material=clear_smoke, name="lid_shell")
    lid.visual(
        mesh_from_cadquery(_build_main_tube_shape(), "food_processor_main_tube"),
        origin=Origin(xyz=(MAIN_TUBE_POS[0], MAIN_TUBE_POS[1], LID_THICKNESS)),
        material=clear_smoke,
        name="main_chute",
    )
    lid.visual(
        mesh_from_cadquery(_build_small_tube_shape(), "food_processor_small_tube"),
        origin=Origin(xyz=(SMALL_TUBE_POS[0], SMALL_TUBE_POS[1], LID_THICKNESS)),
        material=clear_smoke,
        name="small_tube",
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BOWL_HEIGHT)),
    )

    main_pusher = model.part("main_pusher")
    main_pusher.visual(
        Box((0.044, 0.068, 0.240)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=pusher_dark,
        name="main_body",
    )
    main_pusher.visual(
        Box((0.058, 0.084, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=pusher_dark,
        name="main_cap",
    )
    main_pusher.visual(
        Box((0.070, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=pusher_dark,
        name="main_grip",
    )
    model.articulation(
        "lid_to_main_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=main_pusher,
        origin=Origin(xyz=(MAIN_TUBE_POS[0], MAIN_TUBE_POS[1], LID_THICKNESS + MAIN_TUBE_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.18, lower=0.0, upper=0.055),
    )

    small_pusher = model.part("small_pusher")
    small_pusher.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=pusher_dark,
        name="small_body",
    )
    small_pusher.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=pusher_dark,
        name="small_cap",
    )
    model.articulation(
        "lid_to_small_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=small_pusher,
        origin=Origin(xyz=(SMALL_TUBE_POS[0], SMALL_TUBE_POS[1], LID_THICKNESS + SMALL_TUBE_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.14, lower=0.0, upper=0.030),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        mesh_from_cadquery(_build_blade_carrier_shape(), "food_processor_blade_carrier"),
        material=steel,
        name="blade_carrier",
    )
    model.articulation(
        "base_to_blade_carrier",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade_carrier,
        origin=Origin(xyz=(0.0, BOWL_CENTER[1], 0.144)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=18.0),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.022,
                body_style="tapered",
                top_diameter=0.034,
                base_diameter=0.050,
                crown_radius=0.003,
                edge_radius=0.002,
                center=False,
            ),
            "food_processor_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_graphite,
        name="dial_shell",
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=DIAL_POS),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    for name, (x, y, z) in BUTTON_LAYOUT:
        button = model.part(name)
        button.visual(
            Box((0.026, 0.008, 0.020)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=control_silver,
            name="button_cap",
        )
        model.articulation(
            f"base_to_{name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.0035),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    handle = object_model.get_part("handle")
    lid = object_model.get_part("lid")
    main_pusher = object_model.get_part("main_pusher")
    small_pusher = object_model.get_part("small_pusher")
    blade_carrier = object_model.get_part("blade_carrier")
    dial = object_model.get_part("dial")

    ctx.allow_overlap(
        bowl,
        handle,
        elem_a="bowl_shell",
        elem_b="handle_shell",
        reason="The bowl handle is authored as a separate fixed part but is intentionally fused into the side wall.",
    )
    ctx.allow_overlap(
        bowl,
        blade_carrier,
        elem_a="bowl_shell",
        elem_b="blade_carrier",
        reason="The blade carrier intentionally occupies the hollow prep volume inside the thin-walled bowl shell.",
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="bowl_shell",
        negative_elem="bowl_seat",
        max_gap=0.005,
        max_penetration=0.0001,
        name="bowl seats closely on the base collar",
    )

    main_joint = object_model.get_articulation("lid_to_main_pusher")
    main_limits = main_joint.motion_limits
    ctx.expect_within(
        main_pusher,
        lid,
        axes="xy",
        inner_elem="main_body",
        outer_elem="main_chute",
        margin=0.002,
        name="main pusher stays centered in the large chute",
    )
    ctx.expect_overlap(
        main_pusher,
        lid,
        axes="z",
        elem_a="main_body",
        elem_b="main_chute",
        min_overlap=0.012,
        name="main pusher remains captured in the large chute at rest",
    )
    rest_main = ctx.part_world_position(main_pusher)
    if main_limits is not None and main_limits.upper is not None:
        with ctx.pose({main_joint: main_limits.upper}):
            ctx.expect_within(
                main_pusher,
                lid,
                axes="xy",
                inner_elem="main_body",
                outer_elem="main_chute",
                margin=0.002,
                name="main pusher stays centered when pressed",
            )
            ctx.expect_overlap(
                main_pusher,
                lid,
                axes="z",
                elem_a="main_body",
                elem_b="main_chute",
                min_overlap=0.010,
                name="main pusher retains insertion at full stroke",
            )
            pressed_main = ctx.part_world_position(main_pusher)
        ctx.check(
            "main pusher travels downward",
            rest_main is not None and pressed_main is not None and pressed_main[2] < rest_main[2] - 0.03,
            details=f"rest={rest_main}, pressed={pressed_main}",
        )

    small_joint = object_model.get_articulation("lid_to_small_pusher")
    small_limits = small_joint.motion_limits
    ctx.expect_within(
        small_pusher,
        lid,
        axes="xy",
        inner_elem="small_body",
        outer_elem="small_tube",
        margin=0.002,
        name="small pusher stays centered in the ingredient tube",
    )
    ctx.expect_overlap(
        small_pusher,
        lid,
        axes="z",
        elem_a="small_body",
        elem_b="small_tube",
        min_overlap=0.012,
        name="small pusher remains captured in the ingredient tube at rest",
    )
    rest_small = ctx.part_world_position(small_pusher)
    if small_limits is not None and small_limits.upper is not None:
        with ctx.pose({small_joint: small_limits.upper}):
            ctx.expect_within(
                small_pusher,
                lid,
                axes="xy",
                inner_elem="small_body",
                outer_elem="small_tube",
                margin=0.002,
                name="small pusher stays centered when pressed",
            )
            ctx.expect_overlap(
                small_pusher,
                lid,
                axes="z",
                elem_a="small_body",
                elem_b="small_tube",
                min_overlap=0.010,
                name="small pusher retains insertion at full stroke",
            )
            pressed_small = ctx.part_world_position(small_pusher)
        ctx.check(
            "small pusher travels downward",
            rest_small is not None and pressed_small is not None and pressed_small[2] < rest_small[2] - 0.015,
            details=f"rest={rest_small}, pressed={pressed_small}",
        )

    blade_joint = object_model.get_articulation("base_to_blade_carrier")
    dial_joint = object_model.get_articulation("base_to_dial")
    ctx.check(
        "blade carrier uses continuous rotation",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type}",
    )
    ctx.check(
        "selector dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    ctx.expect_overlap(
        blade_carrier,
        bowl,
        axes="xy",
        elem_a="blade_carrier",
        elem_b="bowl_shell",
        min_overlap=0.080,
        name="blade carrier stays centered under the bowl opening",
    )
    ctx.expect_within(
        dial,
        base,
        axes="xz",
        inner_elem="dial_shell",
        outer_elem="control_panel",
        margin=0.004,
        name="selector dial sits within the control panel footprint",
    )

    for name, _ in BUTTON_LAYOUT:
        button = object_model.get_part(name)
        joint = object_model.get_articulation(f"base_to_{name}")
        limits = joint.motion_limits
        ctx.check(
            f"{name} is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={joint.articulation_type}",
        )
        ctx.expect_within(
            button,
            base,
            axes="xz",
            inner_elem="button_cap",
            outer_elem="control_panel",
            margin=0.003,
            name=f"{name} stays within the panel layout",
        )
        rest_pos = ctx.part_world_position(button)
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                pressed_pos = ctx.part_world_position(button)
            ctx.check(
                f"{name} presses inward",
                rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.002,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    return ctx.report()


object_model = build_object_model()
