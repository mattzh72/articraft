from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobBore,
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


BODY_W = 0.340
BODY_D = 0.235
BODY_H = 0.455

WALL_T = 0.006
REAR_T = 0.006
FASCIA_T = 0.010
ROOF_T = 0.010
HEAD_BASE_Z = 0.308

SLOT_W = 0.220
SLOT_D = 0.018
SLOT_Y = 0.015

BIN_W = 0.298
BIN_D = 0.195
BIN_H = 0.255
BIN_WALL = 0.003
BIN_TRAVEL = 0.090
BIN_FRONT_Y = BODY_D / 2.0 - 0.003
BIN_BOTTOM_Z = 0.028
BIN_CENTER_Z = BIN_BOTTOM_Z + BIN_H / 2.0

SHAFT_Z = 0.355
SHAFT_FRONT_Y = 0.026
SHAFT_REAR_Y = 0.004
SHAFT_LENGTH = 0.296


def _build_bin_shape(width: float, depth: float, height: float, wall: float):
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
    )
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - wall + 0.002, centered=(True, True, False))
        .translate((0.0, 0.0, wall))
    )
    shell = outer.cut(inner)
    return shell.translate((0.0, -depth / 2.0, -height / 2.0))


def _add_cutter_stack(part, *, axle_length: float, count: int, cutter_pitch: float, cutter_radius: float) -> None:
    part.visual(
        Cylinder(radius=0.004, length=axle_length),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="shaft_dark",
        name="axle",
    )
    start_x = -0.5 * cutter_pitch * (count - 1)
    for index in range(count):
        part.visual(
            Cylinder(radius=cutter_radius, length=0.012),
            origin=Origin(
                xyz=(start_x + index * cutter_pitch, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="cutter_steel",
            name=f"cutter_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paper_shredder")

    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    body_panel = model.material("body_panel", rgba=(0.24, 0.25, 0.28, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    shaft_dark = model.material("shaft_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    cutter_steel = model.material("cutter_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    translucent_bin = model.material("translucent_bin", rgba=(0.68, 0.77, 0.83, 0.38))
    flap_tint = model.material("flap_tint", rgba=(0.56, 0.63, 0.68, 0.45))
    knob_finish = model.material("knob_finish", rgba=(0.11, 0.12, 0.13, 1.0))
    button_red = model.material("button_red", rgba=(0.72, 0.16, 0.16, 1.0))
    button_green = model.material("button_green", rgba=(0.26, 0.56, 0.28, 1.0))
    button_grey = model.material("button_grey", rgba=(0.73, 0.75, 0.77, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        Box((WALL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + WALL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=body_dark,
        name="left_side",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - WALL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=body_dark,
        name="right_side",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL_T, REAR_T, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + REAR_T / 2.0, BODY_H / 2.0)),
        material=body_dark,
        name="rear_panel",
    )
    slot_rear_y = SLOT_Y - SLOT_D / 2.0
    slot_front_y = SLOT_Y + SLOT_D / 2.0
    body.visual(
        Box((BODY_W, slot_rear_y + BODY_D / 2.0, ROOF_T)),
        origin=Origin(
            xyz=(0.0, (-BODY_D / 2.0 + slot_rear_y) / 2.0, BODY_H - ROOF_T / 2.0)
        ),
        material=body_dark,
        name="roof_rear",
    )
    body.visual(
        Box((BODY_W, BODY_D / 2.0 - slot_front_y, ROOF_T)),
        origin=Origin(
            xyz=(0.0, (slot_front_y + BODY_D / 2.0) / 2.0, BODY_H - ROOF_T / 2.0)
        ),
        material=body_dark,
        name="roof_front",
    )
    roof_side_w = 0.5 * (BODY_W - SLOT_W)
    body.visual(
        Box((roof_side_w, SLOT_D, ROOF_T)),
        origin=Origin(
            xyz=(-SLOT_W / 2.0 - roof_side_w / 2.0, SLOT_Y, BODY_H - ROOF_T / 2.0)
        ),
        material=body_dark,
        name="roof_left",
    )
    body.visual(
        Box((roof_side_w, SLOT_D, ROOF_T)),
        origin=Origin(
            xyz=(SLOT_W / 2.0 + roof_side_w / 2.0, SLOT_Y, BODY_H - ROOF_T / 2.0)
        ),
        material=body_dark,
        name="roof_right",
    )
    body.visual(
        Box((BODY_W, FASCIA_T, BODY_H - HEAD_BASE_Z)),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 - FASCIA_T / 2.0, HEAD_BASE_Z + (BODY_H - HEAD_BASE_Z) / 2.0)
        ),
        material=body_panel,
        name="front_fascia",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL_T, 0.055, 0.010)),
        origin=Origin(xyz=(0.0, -0.0835, HEAD_BASE_Z + 0.005)),
        material=body_dark,
        name="head_rear_bridge",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL_T, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - FASCIA_T - 0.010, HEAD_BASE_Z + 0.005)),
        material=body_dark,
        name="head_front_lip",
    )
    body.visual(
        Box((0.024, 0.142, 0.010)),
        origin=Origin(xyz=(-0.155, 0.015, HEAD_BASE_Z + 0.005)),
        material=body_dark,
        name="left_drop_rail",
    )
    body.visual(
        Box((0.024, 0.142, 0.010)),
        origin=Origin(xyz=(0.155, 0.015, HEAD_BASE_Z + 0.005)),
        material=body_dark,
        name="right_drop_rail",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL_T, 0.165, 0.008)),
        origin=Origin(xyz=(0.0, -0.029, 0.024)),
        material=body_panel,
        name="bin_floor",
    )
    body.visual(
        Box((0.178, 0.004, 0.082)),
        origin=Origin(xyz=(-0.002, BODY_D / 2.0 + 0.002, 0.386)),
        material=trim_dark,
        name="control_plate",
    )
    for prefix, shaft_y in (("front", SHAFT_FRONT_Y), ("rear", SHAFT_REAR_Y)):
        for side_sign, x_center in ((-1.0, -0.156), (1.0, 0.156)):
            side_name = "left" if side_sign < 0.0 else "right"
            body.visual(
                Box((0.016, 0.022, 0.024)),
                origin=Origin(xyz=(x_center, shaft_y, SHAFT_Z)),
                material=body_panel,
                name=f"{prefix}_{side_name}_bearing",
            )
    for x_center, y_center, name in (
        (-0.147, 0.076, "front_left_foot"),
        (0.147, 0.076, "front_right_foot"),
        (-0.147, -0.080, "rear_left_foot"),
        (0.147, -0.080, "rear_right_foot"),
    ):
        body.visual(
            Box((0.040, 0.040, 0.008)),
            origin=Origin(xyz=(x_center, y_center, 0.004)),
            material=foot_rubber,
            name=name,
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(
            _build_bin_shape(BIN_W, BIN_D, BIN_H, BIN_WALL),
            "paper_shredder_bin",
        ),
        material=translucent_bin,
        name="bin_shell",
    )
    bin_part.visual(
        Box((0.148, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, 0.004, 0.094)),
        material=trim_dark,
        name="pull_lip",
    )
    bin_part.inertial = Inertial.from_geometry(
        Box((BIN_W, BIN_D, BIN_H)),
        mass=0.95,
    )
    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, BIN_FRONT_Y, BIN_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=BIN_TRAVEL,
        ),
    )

    cutter_front = model.part("cutter_front")
    _add_cutter_stack(
        cutter_front,
        axle_length=SHAFT_LENGTH,
        count=10,
        cutter_pitch=0.022,
        cutter_radius=0.010,
    )
    cutter_front.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=SHAFT_LENGTH),
        mass=0.28,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    model.articulation(
        "body_to_cutter_front",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cutter_front,
        origin=Origin(xyz=(0.0, SHAFT_FRONT_Y, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=24.0),
    )

    cutter_rear = model.part("cutter_rear")
    _add_cutter_stack(
        cutter_rear,
        axle_length=SHAFT_LENGTH,
        count=10,
        cutter_pitch=0.022,
        cutter_radius=0.010,
    )
    cutter_rear.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=SHAFT_LENGTH),
        mass=0.28,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    model.articulation(
        "body_to_cutter_rear",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cutter_rear,
        origin=Origin(xyz=(0.0, SHAFT_REAR_Y, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=24.0),
    )

    flap = model.part("flap")
    flap.visual(
        Box((SLOT_W - 0.012, SLOT_D, 0.0018)),
        origin=Origin(xyz=(0.0, SLOT_D / 2.0, -0.0009)),
        material=flap_tint,
        name="flap_panel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((SLOT_W - 0.012, SLOT_D, 0.002)),
        mass=0.04,
    )
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, slot_rear_y, BODY_H - ROOF_T + 0.002)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=4.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    knob = model.part("mode_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.020,
                body_style="skirted",
                top_diameter=0.026,
                skirt=KnobSkirt(0.038, 0.004, flare=0.10),
                grip=KnobGrip(style="fluted", count=16, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
                center=False,
            ),
            "paper_shredder_mode_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="knob_shell",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.038, 0.020, 0.038)),
        mass=0.06,
    )
    model.articulation(
        "body_to_mode_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(-0.055, BODY_D / 2.0 + 0.004, 0.384)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=button_red,
        name="button_cap",
    )
    power_button.inertial = Inertial.from_geometry(Box((0.016, 0.006, 0.016)), mass=0.01)
    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(0.055, BODY_D / 2.0 + 0.004, 0.402)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.0035),
    )

    auto_button = model.part("auto_button")
    auto_button.visual(
        Box((0.016, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=button_green,
        name="button_cap",
    )
    auto_button.inertial = Inertial.from_geometry(Box((0.016, 0.006, 0.012)), mass=0.01)
    model.articulation(
        "body_to_auto_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=auto_button,
        origin=Origin(xyz=(0.055, BODY_D / 2.0 + 0.004, 0.384)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.0035),
    )

    reverse_button = model.part("reverse_button")
    reverse_button.visual(
        Box((0.022, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=button_grey,
        name="button_cap",
    )
    reverse_button.inertial = Inertial.from_geometry(Box((0.022, 0.006, 0.010)), mass=0.01)
    model.articulation(
        "body_to_reverse_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=reverse_button,
        origin=Origin(xyz=(0.055, BODY_D / 2.0 + 0.004, 0.366)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.0035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    flap = object_model.get_part("flap")
    cutter_front = object_model.get_part("cutter_front")
    cutter_rear = object_model.get_part("cutter_rear")

    bin_joint = object_model.get_articulation("body_to_bin")
    flap_joint = object_model.get_articulation("body_to_flap")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("home_office_scale_present", body_aabb is not None, "Expected a world AABB for the shredder body.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("home_office_scale_width", 0.30 <= size[0] <= 0.38, f"size={size!r}")
        ctx.check("home_office_scale_depth", 0.20 <= size[1] <= 0.28, f"size={size!r}")
        ctx.check("home_office_scale_height", 0.40 <= size[2] <= 0.50, f"size={size!r}")

    ctx.expect_within(
        bin_part,
        body,
        axes="x",
        margin=0.020,
        name="bin stays between the shredder side walls",
    )
    ctx.expect_overlap(
        bin_part,
        body,
        axes="y",
        min_overlap=0.180,
        name="closed bin remains deeply seated in the body",
    )

    rest_bin_pos = ctx.part_world_position(bin_part)
    with ctx.pose({bin_joint: BIN_TRAVEL}):
        ctx.expect_within(
            bin_part,
            body,
            axes="x",
            margin=0.020,
            name="extended bin stays centered between the side walls",
        )
        ctx.expect_overlap(
            bin_part,
            body,
            axes="y",
            min_overlap=0.095,
            name="extended bin retains insertion in the shredder body",
        )
        extended_bin_pos = ctx.part_world_position(bin_part)
    ctx.check(
        "bin_pulls_forward",
        rest_bin_pos is not None
        and extended_bin_pos is not None
        and extended_bin_pos[1] > rest_bin_pos[1] + 0.080,
        details=f"rest={rest_bin_pos}, extended={extended_bin_pos}",
    )

    for joint_name, part_name in (
        ("body_to_power_button", "power_button"),
        ("body_to_auto_button", "auto_button"),
        ("body_to_reverse_button", "reverse_button"),
    ):
        joint = object_model.get_articulation(joint_name)
        button = object_model.get_part(part_name)
        rest_pos = ctx.part_world_position(button)
        limits = joint.motion_limits
        pressed_pos = None
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{part_name}_presses_inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0025,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    rest_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: 1.05}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "safety_flap_swings_down_into_slot",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][2] < rest_flap_aabb[0][2] - 0.010,
        details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
    )

    front_pos = ctx.part_world_position(cutter_front)
    rear_pos = ctx.part_world_position(cutter_rear)
    ctx.check(
        "cutter_shafts_share_horizontal_axle_height",
        front_pos is not None
        and rear_pos is not None
        and abs(front_pos[0] - rear_pos[0]) <= 0.001
        and abs(front_pos[2] - rear_pos[2]) <= 0.001,
        details=f"front={front_pos}, rear={rear_pos}",
    )
    ctx.check(
        "cutter_shafts_are_front_and_rear",
        front_pos is not None and rear_pos is not None and front_pos[1] > rear_pos[1] + 0.015,
        details=f"front={front_pos}, rear={rear_pos}",
    )

    for joint_name in ("body_to_mode_knob", "body_to_cutter_front", "body_to_cutter_rear"):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_is_continuous",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits!r}",
        )

    return ctx.report()


object_model = build_object_model()
