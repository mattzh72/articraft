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
    mesh_from_geometry,
    mesh_from_cadquery,
)


CABINET_WIDTH = 0.680
CABINET_DEPTH = 0.700
CABINET_HEIGHT = 0.920
TOP_DECK_THICKNESS = 0.020
BODY_WALL = 0.028

OPENING_WIDTH = 0.455
OPENING_DEPTH = 0.330
OPENING_CENTER_Y = 0.010

LID_WIDTH = 0.565
LID_DEPTH = 0.520
LID_THICKNESS = 0.030
LID_HINGE_Y = -0.160

BASKET_RADIUS = 0.190
BASKET_HEIGHT = 0.620
BASKET_BASE_Z = 0.130


def _build_cabinet_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT).translate(
        (0.0, 0.0, CABINET_HEIGHT * 0.5)
    )
    outer = outer.edges("|Z").fillet(0.024)

    cavity_height = CABINET_HEIGHT - TOP_DECK_THICKNESS - 0.100
    inner = cq.Workplane("XY").box(
        CABINET_WIDTH - 2.0 * BODY_WALL,
        CABINET_DEPTH - 2.0 * BODY_WALL,
        cavity_height,
    ).translate((0.0, 0.0, 0.100 + cavity_height * 0.5))
    shell = outer.cut(inner)

    top_opening = cq.Workplane("XY").box(
        OPENING_WIDTH,
        OPENING_DEPTH,
        TOP_DECK_THICKNESS + 0.070,
    ).translate((0.0, OPENING_CENTER_Y, CABINET_HEIGHT - 0.5 * (TOP_DECK_THICKNESS + 0.070)))
    return shell.cut(top_opening)


def _build_lid_frame() -> cq.Workplane:
    lid = cq.Workplane("XY").box(LID_WIDTH, LID_DEPTH, LID_THICKNESS).translate(
        (0.0, LID_DEPTH * 0.5, LID_THICKNESS * 0.5)
    )
    lid = lid.edges("|Z").fillet(0.010)
    window_recess = cq.Workplane("XY").box(0.345, 0.270, 0.010).translate((0.0, 0.285, 0.025))
    return lid.cut(window_recess)


def _build_tub_collar() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.208)
        .circle(0.186)
        .extrude(0.060)
        .translate((0.0, 0.0, 0.860))
    )


def _build_basket() -> cq.Workplane:
    shell = cq.Workplane("XY").circle(BASKET_RADIUS).extrude(BASKET_HEIGHT)
    shell = shell.cut(
        cq.Workplane("XY").circle(BASKET_RADIUS - 0.006).extrude(BASKET_HEIGHT - 0.030).translate((0.0, 0.0, 0.022))
    )

    rim = (
        cq.Workplane("XY")
        .circle(BASKET_RADIUS + 0.008)
        .circle(BASKET_RADIUS - 0.010)
        .extrude(0.018)
        .translate((0.0, 0.0, BASKET_HEIGHT - 0.012))
    )

    hub_base = cq.Workplane("XY").circle(0.060).circle(0.029).extrude(0.016)
    hub_sleeve = cq.Workplane("XY").circle(0.042).circle(0.029).extrude(0.185)

    basket = shell.union(rim).union(hub_base).union(hub_sleeve)

    slot_angles = range(0, 360, 30)
    slot_rows = (0.135, 0.245, 0.355, 0.465)
    for angle_deg in slot_angles:
        for row_z in slot_rows:
            cutter = (
                cq.Workplane("XY")
                .box(0.018, 0.034, 0.050)
                .translate((0.0, BASKET_RADIUS - 0.003, row_z))
                .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
            )
            basket = basket.cut(cutter)

    agitator_cap = cq.Workplane("XY").circle(0.060).circle(0.032).extrude(0.022).translate((0.0, 0.0, 0.152))
    basket = basket.union(agitator_cap)

    agitator_slots = range(0, 360, 90)
    for angle_deg in agitator_slots:
        fin = (
            cq.Workplane("XY")
            .box(0.012, 0.048, 0.120)
            .translate((0.0, 0.052, 0.102))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        basket = basket.union(fin)

    spindle_clearance = cq.Workplane("XY").circle(0.030).extrude(0.240)
    return basket.cut(spindle_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.95, 0.96, 1.0))
    console_white = model.material("console_white", rgba=(0.91, 0.92, 0.94, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.72, 0.75, 0.79, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.25, 0.31, 0.36, 0.45))
    stainless = model.material("stainless", rgba=(0.76, 0.79, 0.81, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.18, 0.20, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_shell(), "washer_cabinet_shell"),
        material=cabinet_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((0.620, 0.105, 0.170)),
        origin=Origin(xyz=(0.0, -0.2975, 1.005)),
        material=console_white,
        name="console_body",
    )
    cabinet.visual(
        Box((0.620, 0.028, 0.152)),
        origin=Origin(xyz=(0.0, -0.242, 0.988), rpy=(math.radians(20.0), 0.0, 0.0)),
        material=console_white,
        name="console_fascia",
    )
    cabinet.visual(
        Box((0.248, 0.009, 0.138)),
        origin=Origin(xyz=(-0.168, -0.226, 0.988), rpy=(math.radians(20.0), 0.0, 0.0)),
        material=trim_grey,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.252, 0.086, 0.006)),
        origin=Origin(xyz=(-0.168, -0.297, 1.093)),
        material=trim_grey,
        name="control_pad",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_tub_collar(), "washer_tub_collar"),
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, 0.0)),
        material=trim_grey,
        name="tub_collar",
    )
    cabinet.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, 0.115)),
        material=dark_plastic,
        name="drive_collar",
    )
    cabinet.visual(
        Cylinder(radius=0.029, length=0.210),
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, 0.235)),
        material=dark_plastic,
        name="drive_spindle",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_frame(), "washer_lid_frame"),
        material=cabinet_white,
        name="lid_frame",
    )
    lid.visual(
        Box((0.335, 0.255, 0.004)),
        origin=Origin(xyz=(0.0, 0.285, 0.020)),
        material=dark_glass,
        name="lid_window",
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, CABINET_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=0.0, upper=1.55),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_build_basket(), "washer_basket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="basket_shell",
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, BASKET_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=20.0),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.028,
                body_style="skirted",
                center=False,
                top_diameter=0.036,
                skirt=KnobSkirt(0.056, 0.006, flare=0.08),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            ),
            "washer_control_knob",
        ),
        origin=Origin(),
        material=dark_plastic,
        name="knob_body",
    )
    model.articulation(
        "control_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=control_knob,
        origin=Origin(xyz=(-0.226, -0.314, 1.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box((0.022, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.005, 0.007)),
        material=dark_plastic,
        name="button_cap",
    )
    model.articulation(
        "power_button_push",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=power_button,
        origin=Origin(xyz=(-0.150, -0.307, 1.096)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.050, lower=0.0, upper=0.0035),
    )

    for index, x_pos in enumerate((-0.160, -0.118)):
        water_button = model.part(f"water_button_{index}")
        water_button.visual(
            Box((0.030, 0.010, 0.016)),
            origin=Origin(xyz=(0.0, 0.005, 0.008)),
            material=trim_grey,
            name="button_cap",
        )
        model.articulation(
            f"water_button_{index}_push",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=water_button,
            origin=Origin(xyz=(x_pos, -0.284, 1.096)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.050, lower=0.0, upper=0.0035),
        )

    cabinet.visual(
        Box((0.054, 0.092, 0.004)),
        origin=Origin(xyz=(0.310, -0.152, 0.922)),
        material=trim_grey,
        name="detergent_recess",
    )

    detergent_flap = model.part("detergent_flap")
    detergent_flap.visual(
        Box((0.048, 0.084, 0.014)),
        origin=Origin(xyz=(0.0, 0.042, 0.007)),
        material=console_white,
        name="flap_panel",
    )
    detergent_flap.visual(
        Box((0.022, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.074, 0.016)),
        material=trim_grey,
        name="flap_pull",
    )
    model.articulation(
        "detergent_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=detergent_flap,
        origin=Origin(xyz=(0.310, -0.194, CABINET_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    control_knob = object_model.get_part("control_knob")
    power_button = object_model.get_part("power_button")
    water_button_0 = object_model.get_part("water_button_0")
    water_button_1 = object_model.get_part("water_button_1")
    detergent_flap = object_model.get_part("detergent_flap")
    lid_hinge = object_model.get_articulation("lid_hinge")
    basket_spin = object_model.get_articulation("basket_spin")
    control_knob_spin = object_model.get_articulation("control_knob_spin")
    power_button_push = object_model.get_articulation("power_button_push")
    water_button_0_push = object_model.get_articulation("water_button_0_push")
    water_button_1_push = object_model.get_articulation("water_button_1_push")
    detergent_flap_hinge = object_model.get_articulation("detergent_flap_hinge")

    ctx.check(
        "basket_joint_is_continuous",
        str(basket_spin.articulation_type).endswith("CONTINUOUS"),
        details=f"type={basket_spin.articulation_type!r}",
    )
    ctx.check(
        "control_knob_joint_is_continuous",
        str(control_knob_spin.articulation_type).endswith("CONTINUOUS"),
        details=f"type={control_knob_spin.articulation_type!r}",
    )
    ctx.check(
        "buttons_are_prismatic",
        all(
            str(joint.articulation_type).endswith("PRISMATIC")
            for joint in (power_button_push, water_button_0_push, water_button_1_push)
        ),
        details=(
            f"power={power_button_push.articulation_type!r}, "
            f"water0={water_button_0_push.articulation_type!r}, "
            f"water1={water_button_1_push.articulation_type!r}"
        ),
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_frame",
            negative_elem="cabinet_shell",
            max_gap=0.008,
            max_penetration=0.0,
            name="lid rests on the top deck when closed",
        )
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="xy",
            elem_a="lid_frame",
            elem_b="cabinet_shell",
            min_overlap=0.250,
            name="lid covers the cabinet opening footprint",
        )
        ctx.expect_within(
            basket,
            cabinet,
            axes="xy",
            inner_elem="basket_shell",
            outer_elem="cabinet_shell",
            margin=0.020,
            name="basket stays within the cabinet footprint",
        )
        ctx.expect_gap(
            detergent_flap,
            cabinet,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="cabinet_shell",
            max_gap=0.010,
            max_penetration=0.0,
            name="detergent flap sits flush on the top deck",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_frame")
    with ctx.pose({lid_hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_frame")
    ctx.check(
        "lid_opens_upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.180,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    closed_button_pos = ctx.part_world_position(power_button)
    with ctx.pose({power_button_push: 0.0035}):
        pressed_button_pos = ctx.part_world_position(power_button)
    ctx.check(
        "power_button_presses_inward",
        closed_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < closed_button_pos[2] - 0.002,
        details=f"closed={closed_button_pos}, pressed={pressed_button_pos}",
    )

    closed_flap = ctx.part_element_world_aabb(detergent_flap, elem="flap_panel")
    with ctx.pose({detergent_flap_hinge: 1.15}):
        open_flap = ctx.part_element_world_aabb(detergent_flap, elem="flap_panel")
    ctx.check(
        "detergent_flap_opens_upward",
        closed_flap is not None
        and open_flap is not None
        and open_flap[1][2] > closed_flap[1][2] + 0.060,
        details=f"closed={closed_flap}, open={open_flap}",
    )

    ctx.expect_origin_distance(
        water_button_0,
        water_button_1,
        axes="x",
        min_dist=0.035,
        max_dist=0.050,
        name="water level buttons sit as a tight pair",
    )
    ctx.expect_origin_distance(
        control_knob,
        power_button,
        axes="xy",
        min_dist=0.050,
        max_dist=0.120,
        name="power button sits adjacent to the knob cluster",
    )

    return ctx.report()


object_model = build_object_model()
