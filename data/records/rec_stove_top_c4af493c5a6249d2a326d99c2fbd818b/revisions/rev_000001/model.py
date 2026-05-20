from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

TOP_W = 0.90
TOP_D = 0.52
GLASS_T = 0.006
TOP_R = 0.012

BODY_W = 0.82
BODY_D = 0.40
BODY_H = 0.036

FASCIA_W = 0.34
FASCIA_D = 0.016
FASCIA_H = 0.012
FASCIA_FRONT_Y = -TOP_D * 0.5
FASCIA_CENTER_Y = FASCIA_FRONT_Y + FASCIA_D * 0.5

BRIDGE_D = 0.044
BRIDGE_H = 0.004
BRIDGE_CENTER_Y = (FASCIA_FRONT_Y + FASCIA_D - BODY_D * 0.5) * 0.5

BUTTON_Z = -0.0141
BUTTON_PITCH = 0.034
BUTTON_X = [(-3.5 + index) * BUTTON_PITCH for index in range(8)]
SLOT_W = 0.028


def _make_glass_panel() -> object:
    panel = (
        cq.Workplane("XY")
        .box(TOP_W, TOP_D, GLASS_T)
        .edges("|Z")
        .fillet(TOP_R)
        .translate((0.0, 0.0, -GLASS_T * 0.5))
    )

    zone_specs = (
        (-0.265, 0.110, 0.078),
        (-0.255, -0.095, 0.092),
        (0.000, 0.015, 0.105),
        (0.255, -0.095, 0.092),
        (0.265, 0.110, 0.078),
    )
    groove_half_width = 0.00065
    groove_depth = 0.00035

    for cx, cy, radius in zone_specs:
        panel = (
            panel.faces(">Z")
            .workplane()
            .center(cx, cy)
            .circle(radius + groove_half_width)
            .circle(radius - groove_half_width)
            .cutBlind(-groove_depth)
        )

    return panel


def _make_body() -> object:
    housing = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).translate(
        (0.0, 0.0, -GLASS_T - BODY_H * 0.5)
    )
    bridge = cq.Workplane("XY").box(FASCIA_W, BRIDGE_D, BRIDGE_H).translate(
        (0.0, BRIDGE_CENTER_Y, -GLASS_T - BRIDGE_H * 0.5)
    )
    fascia = cq.Workplane("XY").box(FASCIA_W, FASCIA_D, FASCIA_H).translate(
        (0.0, FASCIA_CENTER_Y, -GLASS_T - FASCIA_H * 0.5)
    )

    body = housing.union(bridge).union(fascia)

    for center_x in BUTTON_X:
        slot = cq.Workplane("XY").box(0.030, 0.020, 0.0076).translate(
            (center_x, FASCIA_FRONT_Y + 0.010, BUTTON_Z)
        )
        body = body.cut(slot)

    return body


def _make_button() -> object:
    cap = cq.Workplane("XY").box(0.028, 0.006, 0.0076).translate((0.0, 0.001, 0.0))
    stem = cq.Workplane("XY").box(0.020, 0.012, 0.0060).translate((0.0, 0.008, 0.0))
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_zone_induction_cooktop")

    chassis_finish = model.material("chassis_finish", rgba=(0.15, 0.15, 0.16, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.05, 0.05, 0.06, 0.94))
    button_finish = model.material("button_finish", rgba=(0.20, 0.20, 0.21, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, -GLASS_T - BODY_H * 0.5)),
        material=chassis_finish,
        name="housing",
    )
    body.visual(
        Box((FASCIA_W, BRIDGE_D, BRIDGE_H)),
        origin=Origin(xyz=(0.0, BRIDGE_CENTER_Y, -GLASS_T - BRIDGE_H * 0.5)),
        material=chassis_finish,
        name="bridge",
    )
    slot_left = -FASCIA_W * 0.5
    fascia_bottom_z = -GLASS_T - FASCIA_H * 0.5
    for index, center_x in enumerate(BUTTON_X):
        slot_min = center_x - SLOT_W * 0.5
        if slot_min > slot_left:
            body.visual(
                Box((slot_min - slot_left, FASCIA_D, FASCIA_H)),
                origin=Origin(
                    xyz=((slot_left + slot_min) * 0.5, FASCIA_CENTER_Y, fascia_bottom_z)
                ),
                material=chassis_finish,
                name=f"fascia_{index}",
            )
        slot_left = center_x + SLOT_W * 0.5
    if slot_left < FASCIA_W * 0.5:
        body.visual(
            Box((FASCIA_W * 0.5 - slot_left, FASCIA_D, FASCIA_H)),
            origin=Origin(
                xyz=((slot_left + FASCIA_W * 0.5) * 0.5, FASCIA_CENTER_Y, fascia_bottom_z)
            ),
            material=chassis_finish,
            name="fascia_8",
        )
    body.inertial = Inertial.from_geometry(
        Box((TOP_W, TOP_D, GLASS_T + BODY_H)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -(GLASS_T + BODY_H) * 0.5)),
    )

    glass = model.part("glass")
    glass.visual(
        mesh_from_cadquery(_make_glass_panel(), "cooktop_glass"),
        material=glass_finish,
        name="surface",
    )
    glass.inertial = Inertial.from_geometry(
        Box((TOP_W, TOP_D, GLASS_T)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, -GLASS_T * 0.5)),
    )
    model.articulation(
        "body_to_glass",
        ArticulationType.FIXED,
        parent=body,
        child=glass,
    )

    button_mesh = mesh_from_cadquery(_make_button(), "cooktop_button")

    for index, center_x in enumerate(BUTTON_X):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            material=button_finish,
            name="button",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.028, 0.014, 0.008)),
            mass=0.012,
            origin=Origin(xyz=(0.0, 0.007, 0.0)),
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(center_x, FASCIA_FRONT_Y, BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.04,
                lower=0.0,
                upper=0.002,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    glass = object_model.get_part("glass")
    buttons = [object_model.get_part(f"button_{index}") for index in range(8)]
    joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(8)]

    ctx.expect_gap(
        glass,
        body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="glass seats flush on chassis",
    )
    ctx.expect_overlap(
        glass,
        body,
        axes="xy",
        min_overlap=0.40,
        name="chassis stays under the glass footprint",
    )
    ctx.expect_gap(
        glass,
        buttons[3],
        axis="z",
        min_gap=0.004,
        name="button bank sits below the glass surface",
    )
    ctx.expect_overlap(
        buttons[3],
        body,
        axes="xz",
        min_overlap=0.005,
        name="center button remains mounted in the front strip",
    )

    glass_aabb = ctx.part_world_aabb(glass)
    if glass_aabb is None:
        ctx.fail("glass_aabb_present", "Expected the glass panel to produce a world AABB.")
    else:
        mins, maxs = glass_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "cooktop_scale",
            0.88 <= size[0] <= 0.92 and 0.50 <= size[1] <= 0.54 and 0.0055 <= size[2] <= 0.0065,
            details=f"glass_size={size!r}",
        )

    for index, joint in enumerate(joints):
        limits = joint.motion_limits
        ctx.check(
            f"button_{index}_travel_is_short",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0015 <= limits.upper <= 0.0025,
            details=f"limits={limits!r}",
        )

    rest_button_3 = ctx.part_world_position(buttons[3])
    rest_button_4 = ctx.part_world_position(buttons[4])
    if rest_button_3 is None or rest_button_4 is None:
        ctx.fail(
            "button_positions_present",
            f"rest_button_3={rest_button_3!r}, rest_button_4={rest_button_4!r}",
        )
    else:
        travel = joints[3].motion_limits.upper if joints[3].motion_limits is not None else None
        with ctx.pose({joints[3]: travel if travel is not None else 0.0}):
            pressed_button_3 = ctx.part_world_position(buttons[3])
            pressed_button_4 = ctx.part_world_position(buttons[4])

        ctx.check(
            "button_press_moves_inward",
            pressed_button_3 is not None and pressed_button_3[1] > rest_button_3[1] + 0.0015,
            details=f"rest={rest_button_3!r}, pressed={pressed_button_3!r}",
        )
        ctx.check(
            "neighbor_button_stays_independent",
            pressed_button_4 is not None and abs(pressed_button_4[1] - rest_button_4[1]) <= 1e-6,
            details=f"rest={rest_button_4!r}, pressed={pressed_button_4!r}",
        )

    edge_a = ctx.part_world_position(buttons[0])
    edge_b = ctx.part_world_position(buttons[7])
    if edge_a is None or edge_b is None:
        ctx.fail("edge_button_positions_present", f"edge_a={edge_a!r}, edge_b={edge_b!r}")
    else:
        ctx.check(
            "button_bank_centered",
            abs((edge_a[0] + edge_b[0]) * 0.5) <= 0.003 and abs(edge_a[0] + edge_b[0]) <= 0.006,
            details=f"edge_a={edge_a!r}, edge_b={edge_b!r}",
        )
        ctx.check(
            "button_bank_on_lower_edge",
            edge_a[1] <= -0.250 and edge_b[1] <= -0.250,
            details=f"edge_a={edge_a!r}, edge_b={edge_b!r}",
        )

    return ctx.report()


object_model = build_object_model()
