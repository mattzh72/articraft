from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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

BODY_W = 0.50
BODY_D = 0.38
BODY_H = 0.32
SHELL_T = 0.012
CORNER_R = 0.022

OPENING_W = 0.338
OPENING_H = 0.220
OPENING_CX = -0.056
OPENING_CZ = -0.008
OPENING_DEPTH = 0.035

CONTROL_PANEL_W = 0.110
CONTROL_PANEL_H = 0.285
CONTROL_PANEL_T = 0.004
CONTROL_PANEL_X = 0.177
CONTROL_PANEL_Y = -BODY_D / 2.0 - CONTROL_PANEL_T / 2.0 + 0.0005

DOOR_W = OPENING_W - 0.008
DOOR_H = OPENING_H - 0.008
DOOR_T = 0.018
DOOR_WINDOW_W = 0.270
DOOR_WINDOW_H = 0.130
DOOR_WINDOW_CZ = 0.115
DOOR_GLASS_W = DOOR_WINDOW_W + 0.020
DOOR_GLASS_H = DOOR_WINDOW_H + 0.020
DOOR_HINGE_Z = OPENING_CZ - OPENING_H / 2.0
DOOR_Y = -BODY_D / 2.0 - DOOR_T / 2.0

FOOT_R = 0.015
FOOT_H = 0.010
FOOT_Z = -BODY_H / 2.0 - FOOT_H / 2.0 + 0.0005
FOOT_X = 0.180
FOOT_Y = 0.120

KNOB_Y = CONTROL_PANEL_Y - CONTROL_PANEL_T / 2.0
KNOB_X = CONTROL_PANEL_X
KNOB_ZS = (0.092, 0.018, -0.056)

SOCKET_LEN = 0.018
SOCKET_BODY_D = 0.032
SOCKET_BODY_H = 0.030
SOCKET_HOLE_R = 0.0070
SOCKET_X = BODY_W / 2.0 - SHELL_T - SOCKET_LEN / 2.0 + 0.0005
SOCKET_Y = 0.018
SOCKET_Z = 0.030

SPIT_CORE_LEN = 0.420
SPIT_JOURNAL_LEN = 0.019
SPIT_SQUARE = 0.0055
SPIT_JOURNAL_R = 0.0052
FORK_COLLAR_LEN = 0.012
FORK_COLLAR_SIZE = 0.015
FORK_CENTER_X = 0.090
PRONG_LEN = 0.055
PRONG_SIZE = 0.0035
PRONG_Z = 0.007


def build_body_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    outer = outer.edges("|Z").fillet(CORNER_R)

    inner = cq.Workplane("XY").box(
        BODY_W - 2.0 * SHELL_T,
        BODY_D - 2.0 * SHELL_T,
        BODY_H - 2.0 * SHELL_T,
    )

    front_opening = cq.Workplane("XY").box(OPENING_W, OPENING_DEPTH, OPENING_H).translate(
        (OPENING_CX, -BODY_D / 2.0 + OPENING_DEPTH / 2.0 - 0.0005, OPENING_CZ)
    )

    return outer.cut(inner).cut(front_opening)


def build_door_frame() -> cq.Workplane:
    frame_blank = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H).translate((0.0, 0.0, DOOR_H / 2.0))
    frame_blank = frame_blank.edges("|Z").fillet(0.008)

    window_cut = cq.Workplane("XY").box(DOOR_WINDOW_W, DOOR_T + 0.010, DOOR_WINDOW_H).translate(
        (0.0, 0.0, DOOR_WINDOW_CZ)
    )

    frame = frame_blank.cut(window_cut)

    handle_bar = cq.Workplane("XY").box(DOOR_W * 0.62, 0.020, 0.016).translate(
        (0.0, -0.017, DOOR_H - 0.018)
    )

    return frame.union(handle_bar).combine()


def build_socket_shape() -> cq.Workplane:
    socket_block = cq.Workplane("XY").box(SOCKET_LEN, SOCKET_BODY_D, SOCKET_BODY_H)
    bore = cq.Workplane("XY").cylinder(SOCKET_LEN + 0.006, SOCKET_HOLE_R, direct=(1.0, 0.0, 0.0))
    entry_notch = cq.Workplane("XY").box(
        SOCKET_LEN + 0.008,
        SOCKET_BODY_D * 0.55,
        SOCKET_BODY_H * 0.45,
    ).translate((0.0, -SOCKET_BODY_D * 0.24, 0.0))
    return socket_block.cut(bore).cut(entry_notch)


def build_spit_shape() -> cq.Workplane:
    core = cq.Workplane("XY").box(SPIT_CORE_LEN, SPIT_SQUARE, SPIT_SQUARE)
    left_journal = cq.Workplane("XY").cylinder(
        SPIT_JOURNAL_LEN,
        SPIT_JOURNAL_R,
        direct=(1.0, 0.0, 0.0),
    ).translate((-(SPIT_CORE_LEN + SPIT_JOURNAL_LEN) / 2.0, 0.0, 0.0))
    right_journal = cq.Workplane("XY").cylinder(
        SPIT_JOURNAL_LEN,
        SPIT_JOURNAL_R,
        direct=(1.0, 0.0, 0.0),
    ).translate(((SPIT_CORE_LEN + SPIT_JOURNAL_LEN) / 2.0, 0.0, 0.0))

    left_collar = cq.Workplane("XY").box(FORK_COLLAR_LEN, FORK_COLLAR_SIZE, FORK_COLLAR_SIZE).translate(
        (-FORK_CENTER_X, 0.0, 0.0)
    )
    right_collar = cq.Workplane("XY").box(FORK_COLLAR_LEN, FORK_COLLAR_SIZE, FORK_COLLAR_SIZE).translate(
        (FORK_CENTER_X, 0.0, 0.0)
    )

    left_prong_top = cq.Workplane("XY").box(PRONG_LEN, PRONG_SIZE, PRONG_SIZE).translate(
        (-FORK_CENTER_X + PRONG_LEN / 2.0 - 0.004, 0.0, PRONG_Z)
    )
    left_prong_bottom = cq.Workplane("XY").box(PRONG_LEN, PRONG_SIZE, PRONG_SIZE).translate(
        (-FORK_CENTER_X + PRONG_LEN / 2.0 - 0.004, 0.0, -PRONG_Z)
    )
    right_prong_top = cq.Workplane("XY").box(PRONG_LEN, PRONG_SIZE, PRONG_SIZE).translate(
        (FORK_CENTER_X - PRONG_LEN / 2.0 + 0.004, 0.0, PRONG_Z)
    )
    right_prong_bottom = cq.Workplane("XY").box(PRONG_LEN, PRONG_SIZE, PRONG_SIZE).translate(
        (FORK_CENTER_X - PRONG_LEN / 2.0 + 0.004, 0.0, -PRONG_Z)
    )

    return (
        core.union(left_journal)
        .union(right_journal)
        .union(left_collar)
        .union(right_collar)
        .union(left_prong_top)
        .union(left_prong_bottom)
        .union(right_prong_top)
        .union(right_prong_bottom)
    )


def build_knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.040,
            0.024,
            body_style="skirted",
            top_diameter=0.032,
            skirt=KnobSkirt(0.048, 0.006, flare=0.05),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_toaster_oven")

    enamel_red = model.material("enamel_red", rgba=(0.64, 0.14, 0.12, 1.0))
    ivory = model.material("ivory", rgba=(0.93, 0.89, 0.78, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.18, 0.22, 0.26, 0.55))
    black = model.material("black", rgba=(0.12, 0.11, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shell(), "body_shell"),
        material=enamel_red,
        name="shell",
    )
    body.visual(
        Box((CONTROL_PANEL_W, CONTROL_PANEL_T, CONTROL_PANEL_H)),
        origin=Origin(xyz=(CONTROL_PANEL_X, CONTROL_PANEL_Y, 0.0)),
        material=ivory,
        name="control_panel",
    )

    for index, bezel_z in enumerate(KNOB_ZS):
        body.visual(
            Cylinder(radius=0.028, length=0.004),
            origin=Origin(
                xyz=(KNOB_X, CONTROL_PANEL_Y, bezel_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"bezel_{index}",
        )

    socket_mesh = mesh_from_cadquery(build_socket_shape(), "spit_socket")
    body.visual(
        socket_mesh,
        origin=Origin(xyz=(-SOCKET_X, SOCKET_Y, SOCKET_Z)),
        material=steel,
        name="left_socket",
    )
    body.visual(
        socket_mesh,
        origin=Origin(xyz=(SOCKET_X, SOCKET_Y, SOCKET_Z)),
        material=steel,
        name="right_socket",
    )
    support_pin_z = SOCKET_Z - (SPIT_JOURNAL_R + 0.0026)
    for name, pin_x in (("left_support", -SOCKET_X), ("right_support", SOCKET_X)):
        body.visual(
            Cylinder(radius=0.0026, length=0.014),
            origin=Origin(
                xyz=(pin_x, SOCKET_Y, support_pin_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=name,
        )

    for index, (foot_x, foot_y) in enumerate(
        (
            (-FOOT_X, -FOOT_Y),
            (-FOOT_X, FOOT_Y),
            (FOOT_X, -FOOT_Y),
            (FOOT_X, FOOT_Y),
        )
    ):
        body.visual(
            Cylinder(radius=FOOT_R, length=FOOT_H),
            origin=Origin(xyz=(foot_x, foot_y, FOOT_Z)),
            material=black,
            name=f"foot_{index}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(build_door_frame(), "door_frame"),
        material=chrome,
        name="door_frame",
    )
    door.visual(
        Box((DOOR_GLASS_W, 0.006, DOOR_GLASS_H)),
        origin=Origin(xyz=(0.0, 0.002, DOOR_WINDOW_CZ)),
        material=dark_glass,
        name="glass",
    )

    top_knob = model.part("top_knob")
    top_knob.visual(
        build_knob_mesh("top_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="knob_shell",
    )

    middle_knob = model.part("middle_knob")
    middle_knob.visual(
        build_knob_mesh("middle_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="knob_shell",
    )

    bottom_knob = model.part("bottom_knob")
    bottom_knob.visual(
        build_knob_mesh("bottom_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="knob_shell",
    )

    spit = model.part("spit")
    spit.visual(
        mesh_from_cadquery(build_spit_shape(), "spit"),
        material=steel,
        name="rod",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(OPENING_CX, DOOR_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    for name, child, knob_z in (
        ("body_to_top_knob", top_knob, KNOB_ZS[0]),
        ("body_to_middle_knob", middle_knob, KNOB_ZS[1]),
        ("body_to_bottom_knob", bottom_knob, KNOB_ZS[2]),
    ):
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=child,
            origin=Origin(xyz=(KNOB_X, KNOB_Y, knob_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

    model.articulation(
        "body_to_spit",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spit,
        origin=Origin(xyz=(0.0, SOCKET_Y, SOCKET_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    top_knob = object_model.get_part("top_knob")
    middle_knob = object_model.get_part("middle_knob")
    bottom_knob = object_model.get_part("bottom_knob")
    spit = object_model.get_part("spit")

    door_hinge = object_model.get_articulation("body_to_door")
    top_knob_joint = object_model.get_articulation("body_to_top_knob")
    middle_knob_joint = object_model.get_articulation("body_to_middle_knob")
    bottom_knob_joint = object_model.get_articulation("body_to_bottom_knob")
    spit_joint = object_model.get_articulation("body_to_spit")

    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="door_frame",
        elem_b="shell",
        min_overlap=0.20,
        name="door covers the front opening footprint",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    with ctx.pose({door_hinge: 1.35}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_frame")

    door_swings_down = (
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.12
        and opened_aabb[1][2] < closed_aabb[1][2] - 0.08
    )
    ctx.check(
        "door swings downward and outward",
        door_swings_down,
        details=f"closed={closed_aabb!r}, opened={opened_aabb!r}",
    )

    for knob_part, name in (
        (top_knob, "top knob"),
        (middle_knob, "middle knob"),
        (bottom_knob, "bottom knob"),
    ):
        ctx.expect_contact(
            knob_part,
            body,
            elem_a="knob_shell",
            elem_b="control_panel",
            name=f"{name} seats on the control panel",
        )

    for joint_name, joint in (
        ("top knob", top_knob_joint),
        ("middle knob", middle_knob_joint),
        ("bottom knob", bottom_knob_joint),
        ("spit", spit_joint),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} uses continuous rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type!r}, limits={limits!r}",
        )

    ctx.expect_overlap(
        spit,
        body,
        axes="yz",
        elem_a="rod",
        elem_b="left_socket",
        min_overlap=0.005,
        name="spit stays aligned with the left socket",
    )
    ctx.expect_overlap(
        spit,
        body,
        axes="yz",
        elem_a="rod",
        elem_b="right_socket",
        min_overlap=0.005,
        name="spit stays aligned with the right socket",
    )
    ctx.expect_overlap(
        spit,
        body,
        axes="x",
        elem_a="rod",
        elem_b="left_socket",
        min_overlap=0.008,
        name="spit remains inserted in the left socket",
    )
    ctx.expect_overlap(
        spit,
        body,
        axes="x",
        elem_a="rod",
        elem_b="right_socket",
        min_overlap=0.008,
        name="spit remains inserted in the right socket",
    )

    return ctx.report()


object_model = build_object_model()
