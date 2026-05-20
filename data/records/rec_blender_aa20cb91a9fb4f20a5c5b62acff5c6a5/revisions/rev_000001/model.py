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
    mesh_from_geometry,
)


BASE_WIDTH = 0.235
BASE_DEPTH = 0.255
BASE_HEIGHT = 0.142
BASE_TOP_WIDTH = 0.168
BASE_TOP_DEPTH = 0.192
BASE_TOP_SHIFT_Y = 0.018

PANEL_WIDTH = 0.164
PANEL_HEIGHT = 0.094
PANEL_THICKNESS = 0.006
PANEL_TILT = math.radians(55.0)
PANEL_CENTER = (0.0, -0.097, 0.074)

KNOB_POS = (0.0, 0.023, 0.0)
BUTTON_Y = -0.023
BUTTON_XS = (-0.041, 0.0, 0.041)

JUG_ORIGIN = (0.0, 0.014, 0.150)
JUG_HEIGHT = 0.230


def _base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .rect(BASE_WIDTH, BASE_DEPTH)
        .workplane(offset=BASE_HEIGHT)
        .center(0.0, BASE_TOP_SHIFT_Y)
        .rect(BASE_TOP_WIDTH, BASE_TOP_DEPTH)
        .loft(combine=True)
    )

    panel_cavity = cq.Workplane("XY").transformed(
        offset=PANEL_CENTER,
        rotate=(math.degrees(PANEL_TILT), 0.0, 0.0),
    ).box(
        PANEL_WIDTH + 0.018,
        PANEL_HEIGHT + 0.010,
        0.050,
        centered=(True, True, True),
    )

    return body.cut(panel_cavity)


def _panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").transformed(offset=(0.0, 0.0, -PANEL_THICKNESS)).box(
        PANEL_WIDTH,
        PANEL_HEIGHT,
        PANEL_THICKNESS,
        centered=(True, True, False),
    )
    left_tab = cq.Workplane("XY").transformed(offset=(-0.068, 0.0, -0.020)).box(
        0.014,
        0.026,
        0.014,
        centered=(True, True, False),
    )
    right_tab = cq.Workplane("XY").transformed(offset=(0.068, 0.0, -0.020)).box(
        0.014,
        0.026,
        0.014,
        centered=(True, True, False),
    )
    panel = panel.union(left_tab).union(right_tab)

    knob_cut = cq.Workplane("XY").center(KNOB_POS[0], KNOB_POS[1]).circle(0.0065).extrude(0.025, both=True)
    panel = panel.cut(knob_cut)

    for button_x in BUTTON_XS:
        button_cut = (
            cq.Workplane("XY")
            .center(button_x, BUTTON_Y)
            .rect(0.016, 0.010)
            .extrude(0.025, both=True)
        )
        panel = panel.cut(button_cut)

    return panel


def _jug_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .rect(0.100, 0.100)
        .workplane(offset=0.090)
        .rect(0.128, 0.128)
        .workplane(offset=0.140)
        .rect(0.160, 0.160)
        .loft(combine=True)
    )

    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.016)
        .rect(0.078, 0.078)
        .workplane(offset=0.070)
        .rect(0.112, 0.112)
        .workplane(offset=0.159)
        .rect(0.152, 0.152)
        .loft(combine=True)
    )

    return outer.cut(inner)


def _jug_handle_shape() -> cq.Workplane:
    loop = (
        cq.Workplane("XZ")
        .center(0.102, 0.145)
        .rect(0.038, 0.148)
        .rect(0.018, 0.104)
        .extrude(0.020, both=True)
    )

    top_bridge = cq.Workplane("XY").transformed(offset=(0.078, 0.0, 0.191)).box(
        0.022,
        0.022,
        0.022,
        centered=(True, True, True),
    )
    bottom_bridge = cq.Workplane("XY").transformed(offset=(0.073, 0.0, 0.091)).box(
        0.024,
        0.022,
        0.028,
        centered=(True, True, True),
    )
    grip_pad = cq.Workplane("XY").transformed(offset=(0.108, 0.0, 0.145)).box(
        0.016,
        0.024,
        0.090,
        centered=(True, True, True),
    )

    return loop.union(top_bridge).union(bottom_bridge).union(grip_pad)


def _jug_collar_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.104, 0.104, 0.026, centered=(True, True, False))
    inner = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.004)).box(
        0.076,
        0.076,
        0.024,
        centered=(True, True, False),
    )
    return outer.cut(inner)


def _knob_shape() -> cq.Workplane:
    skirt = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.0)).circle(0.028).extrude(0.004)
    body = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.004)).circle(0.024).extrude(0.014)
    crown = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.018)).circle(0.018).extrude(0.004)
    pointer = cq.Workplane("XY").transformed(offset=(0.0, 0.015, 0.0225)).box(
        0.004,
        0.010,
        0.001,
        centered=(True, True, True),
    )
    return skirt.union(body).union(crown).union(pointer)


def _lid_shape() -> cq.Workplane:
    plug = cq.Workplane("XY").transformed(offset=(0.0, 0.0, -0.013)).box(
        0.148,
        0.148,
        0.021,
        centered=(True, True, False),
    )
    flange = cq.Workplane("XY").box(0.170, 0.170, 0.010, centered=(True, True, False))
    crown = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.010)).box(
        0.154,
        0.154,
        0.008,
        centered=(True, True, False),
    )
    port_boss = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.018)).circle(0.022).extrude(0.010)
    hinge_pedestal = cq.Workplane("XY").transformed(offset=(0.0, -0.019, 0.028)).box(
        0.034,
        0.010,
        0.014,
        centered=(True, True, True),
    )
    hinge_bridge = cq.Workplane("XY").transformed(offset=(0.0, -0.019, 0.033)).box(
        0.034,
        0.008,
        0.008,
        centered=(True, True, True),
    )
    left_barrel = cq.Workplane("XY").transformed(
        offset=(-0.011, -0.019, 0.033),
        rotate=(0.0, 90.0, 0.0),
    ).circle(0.004).extrude(0.008, both=True)
    right_barrel = cq.Workplane("XY").transformed(
        offset=(0.011, -0.019, 0.033),
        rotate=(0.0, 90.0, 0.0),
    ).circle(0.004).extrude(0.008, both=True)
    port_hole = cq.Workplane("XY").circle(0.0105).extrude(0.050, both=True)

    return (
        plug.union(flange)
        .union(crown)
        .union(port_boss)
        .union(hinge_pedestal)
        .union(hinge_bridge)
        .union(left_barrel)
        .union(right_barrel)
        .cut(port_hole)
    )


def _flap_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").transformed(offset=(0.0, 0.019, 0.017)).box(
        0.038,
        0.032,
        0.004,
        centered=(True, True, True),
    )
    bridge = cq.Workplane("XY").transformed(offset=(0.0, 0.008, 0.008)).box(
        0.020,
        0.012,
        0.014,
        centered=(True, True, True),
    )
    barrel = cq.Workplane("XY").transformed(
        offset=(0.0, 0.0, 0.0),
        rotate=(0.0, 90.0, 0.0),
    ).circle(0.0038).extrude(0.010, both=True)
    tab = cq.Workplane("XY").transformed(offset=(0.0, 0.034, 0.018)).box(
        0.014,
        0.006,
        0.003,
        centered=(True, True, True),
    )
    return cap.union(bridge).union(barrel).union(tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vacuum_blender")

    base_silver = model.material("base_silver", rgba=(0.73, 0.76, 0.80, 1.0))
    base_charcoal = model.material("base_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_glass = model.material("panel_glass", rgba=(0.09, 0.11, 0.13, 0.82))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    button_metal = model.material("button_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    jug_clear = model.material("jug_clear", rgba=(0.78, 0.90, 0.98, 0.30))
    jug_trim = model.material("jug_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    lid_black = model.material("lid_black", rgba=(0.14, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base_shell"), material=base_silver, name="shell")
    base.visual(
        Box((0.108, 0.108, 0.008)),
        origin=Origin(xyz=(0.0, 0.014, 0.146)),
        material=base_charcoal,
        name="seat",
    )

    panel = model.part("panel")
    panel.visual(mesh_from_cadquery(_panel_shape(), "front_panel"), material=panel_glass, name="plate")
    model.articulation(
        "base_to_panel",
        ArticulationType.FIXED,
        parent=base,
        child=panel,
        origin=Origin(
            xyz=(
                PANEL_CENTER[0],
                PANEL_CENTER[1] + 0.005 * math.sin(PANEL_TILT),
                PANEL_CENTER[2] - 0.005 * math.cos(PANEL_TILT),
            ),
            rpy=(PANEL_TILT, 0.0, 0.0),
        ),
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_cadquery(_knob_shape(), "control_knob"),
        material=knob_black,
        name="cap",
    )
    knob.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=knob_black,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=knob_black,
        name="retainer",
    )
    model.articulation(
        "panel_to_knob",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=knob,
        origin=Origin(xyz=(KNOB_POS[0], KNOB_POS[1], 0.0002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.022, 0.014, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=button_metal,
            name="cap",
        )
        button.visual(
            Box((0.014, 0.008, 0.011)),
            origin=Origin(xyz=(0.0, 0.0, -0.0045)),
            material=button_metal,
            name="stem",
        )
        button.visual(
            Box((0.018, 0.012, 0.002)),
            origin=Origin(xyz=(0.0, 0.0, -0.0102)),
            material=button_metal,
            name="retainer",
        )
        model.articulation(
            f"panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=panel,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_Y, 0.001)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.001,
            ),
        )

    jug = model.part("jug")
    jug.visual(mesh_from_cadquery(_jug_shell_shape(), "jug_shell"), material=jug_clear, name="shell")
    jug.visual(mesh_from_cadquery(_jug_handle_shape(), "jug_handle"), material=jug_trim, name="handle")
    jug.visual(mesh_from_cadquery(_jug_collar_shape(), "jug_collar"), material=jug_trim, name="collar")
    model.articulation(
        "base_to_jug",
        ArticulationType.FIXED,
        parent=base,
        child=jug,
        origin=Origin(xyz=JUG_ORIGIN),
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_shape(), "lid_shell"), material=lid_black, name="shell")
    model.articulation(
        "jug_to_lid",
        ArticulationType.FIXED,
        parent=jug,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, JUG_HEIGHT + 0.0114)),
    )

    flap = model.part("flap")
    flap.visual(mesh_from_cadquery(_flap_shape(), "lid_flap"), material=lid_black, name="cap")
    model.articulation(
        "lid_to_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(xyz=(0.0, -0.019, 0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    socket = model.part("socket")
    socket.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=jug_trim,
        name="body",
    )
    model.articulation(
        "jug_to_socket",
        ArticulationType.FIXED,
        parent=jug,
        child=socket,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="hub",
    )
    for index, (yaw_deg, pitch_deg) in enumerate(((35.0, 20.0), (145.0, -20.0), (215.0, 20.0), (325.0, -20.0))):
        blade.visual(
            Box((0.030, 0.007, 0.002)),
            origin=Origin(
                xyz=(0.017, 0.0, 0.005),
                rpy=(0.0, math.radians(pitch_deg), math.radians(yaw_deg)),
            ),
            material=steel,
            name=f"wing_{index}",
        )
    model.articulation(
        "socket_to_blade",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    panel = object_model.get_part("panel")
    jug = object_model.get_part("jug")
    knob = object_model.get_part("knob")
    lid = object_model.get_part("lid")
    flap = object_model.get_part("flap")

    knob_joint = object_model.get_articulation("panel_to_knob")
    blade_joint = object_model.get_articulation("socket_to_blade")
    flap_joint = object_model.get_articulation("lid_to_flap")

    ctx.expect_gap(jug, base, axis="z", max_gap=0.003, max_penetration=0.0, name="jug seats on base")
    ctx.expect_overlap(jug, base, axes="xy", min_overlap=0.090, name="jug stays centered on base")
    ctx.expect_overlap(panel, base, axes="x", min_overlap=0.120, name="panel spans base front width")
    ctx.expect_overlap(lid, jug, axes="xy", min_overlap=0.130, name="lid covers jug opening")

    ctx.check(
        "knob articulation is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )
    ctx.check(
        "blade articulation is continuous",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type!r}",
    )

    ctx.allow_overlap(
        panel,
        knob,
        elem_a="plate",
        elem_b="retainer",
        reason="The rotary knob is clipped to the rear of the panel by a retaining collar.",
    )
    ctx.allow_isolated_part(
        knob,
        reason="The knob is intentionally modeled with small running clearance to the panel while its rear retainer captures it.",
    )
    for index in range(3):
        ctx.allow_overlap(
            panel,
            f"button_{index}",
            elem_a="plate",
            elem_b="retainer",
            reason="Each push button is captured in the panel by a rear retaining tab.",
        )
        ctx.allow_isolated_part(
            f"button_{index}",
            reason="Each push button keeps a small working clearance to the panel while remaining panel-captured by its rear tab.",
        )
    ctx.allow_overlap(
        jug,
        "socket",
        elem_a="shell",
        elem_b="body",
        reason="The blade drive socket is intentionally seated into the simplified thick jug floor.",
    )
    ctx.allow_overlap(
        lid,
        flap,
        reason="The vacuum flap uses a simplified compact hinge with nested hinge solids around the same pivot.",
    )
    ctx.allow_isolated_part(
        lid,
        reason="The lid is represented with a small sealing-clearance fit above the jug rim rather than a compressed gasket contact patch.",
    )
    ctx.allow_isolated_part(
        flap,
        reason="The flap is carried by the lid hinge with small modeled running clearances around the compact hinge geometry.",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"panel_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.001}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0005,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    button_1 = object_model.get_part("button_1")
    button_1_joint = object_model.get_articulation("panel_to_button_1")
    button_2 = object_model.get_part("button_2")
    button_2_rest = ctx.part_world_position(button_2)
    with ctx.pose({button_1_joint: 0.001}):
        button_2_pressed = ctx.part_world_position(button_2)
    unchanged = False
    if button_2_rest is not None and button_2_pressed is not None:
        unchanged = all(abs(button_2_rest[i] - button_2_pressed[i]) < 1e-6 for i in range(3))
    ctx.check(
        "buttons move independently",
        unchanged,
        details=f"button_2_rest={button_2_rest}, button_2_pressed={button_2_pressed}",
    )

    flap_closed = ctx.part_element_world_aabb(flap, elem="cap")
    with ctx.pose({flap_joint: 1.1}):
        flap_open = ctx.part_element_world_aabb(flap, elem="cap")
    flap_rises = False
    if flap_closed is not None and flap_open is not None:
        flap_rises = flap_open[1][2] > flap_closed[1][2] + 0.020
    ctx.check(
        "flap opens upward",
        flap_rises,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check("knob geometry present", knob_aabb is not None, details=f"aabb={knob_aabb}")

    return ctx.report()


object_model = build_object_model()
