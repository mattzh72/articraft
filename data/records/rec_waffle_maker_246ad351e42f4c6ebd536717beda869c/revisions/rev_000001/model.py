from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_WIDTH = 0.285
BODY_DEPTH = 0.225
BODY_HEIGHT = 0.041
BODY_PLATE_SIZE = (0.224, 0.160, 0.004)

LID_WIDTH = 0.270
LID_DEPTH = 0.214
LID_PLATE_SIZE = (0.220, 0.146, 0.005)

HINGE_ORIGIN = (0.0, -0.095, 0.054)
BUTTON_CENTERS = ((0.059, 0.078), (0.086, 0.078))


def _make_body_shape() -> cq.Workplane:
    body_shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
        .faces(">Z")
        .edges()
        .fillet(0.007)
    )

    body_shell = (
        body_shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(0.232, 0.168)
        .cutBlind(-0.007)
    )

    for center_x, center_y in BUTTON_CENTERS:
        button_well = (
            cq.Workplane("XY")
            .box(0.014, 0.011, 0.014)
            .translate((center_x, center_y, 0.039))
        )
        body_shell = body_shell.cut(button_well)

    hinge_bridge = (
        cq.Workplane("XY")
        .box(0.086, 0.030, 0.024, centered=(True, True, False))
        .translate((0.0, -0.108, 0.024))
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .edges()
        .fillet(0.004)
    )

    return body_shell.union(hinge_bridge)


def _make_lid_shape() -> cq.Workplane:
    lid_shell = (
        cq.Workplane("YZ")
        .moveTo(0.003, -0.004)
        .lineTo(LID_DEPTH, -0.004)
        .lineTo(LID_DEPTH, 0.016)
        .threePointArc((0.162, 0.044), (0.064, 0.039))
        .lineTo(0.003, 0.013)
        .close()
        .extrude(LID_WIDTH / 2.0, both=True)
    )

    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(0.007)
        .extrude(0.074 / 2.0, both=True)
        .translate((0.0, 0.000, 0.000))
    )

    underside_pocket = (
        cq.Workplane("XY")
        .box(0.228, 0.150, 0.021)
        .translate((0.0, 0.118, 0.007))
    )

    return lid_shell.union(hinge_barrel).cut(underside_pocket)


def _make_button_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.016, 0.013, 0.0036, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0014)
        .faces(">Z")
        .edges()
        .fillet(0.0008)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_waffle_press")

    shell = model.material("shell", rgba=(0.14, 0.15, 0.16, 1.0))
    plate = model.material("plate", rgba=(0.27, 0.28, 0.29, 1.0))
    hinge_trim = model.material("hinge_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    button_finish = model.material("button_finish", rgba=(0.39, 0.40, 0.42, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "waffle_press_body"),
        material=shell,
        name="body_shell",
    )
    body.visual(
        Box(BODY_PLATE_SIZE),
        origin=Origin(xyz=(0.0, 0.000, 0.036)),
        material=plate,
        name="lower_plate",
    )
    body.visual(
        Box((0.092, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, -0.104, 0.046)),
        material=hinge_trim,
        name="bridge_cap",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shape(), "waffle_press_lid"),
        material=shell,
        name="lid_shell",
    )
    lid.visual(
        Box(LID_PLATE_SIZE),
        origin=Origin(xyz=(0.0, 0.108, -0.0015)),
        material=plate,
        name="upper_plate",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=HINGE_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.2,
            lower=0.0,
            upper=1.40,
        ),
    )

    button_mesh = mesh_from_cadquery(_make_button_shape(), "waffle_press_button")
    for index, (button_x, button_y) in enumerate(BUTTON_CENTERS):
        button = model.part(f"program_button_{index}")
        button.visual(
            button_mesh,
            material=button_finish,
            name="button",
        )
        model.articulation(
            f"body_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, button_y, BODY_HEIGHT)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0018,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    button_0 = object_model.get_part("program_button_0")
    button_1 = object_model.get_part("program_button_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    button_joint_0 = object_model.get_articulation("body_to_program_button_0")
    button_joint_1 = object_model.get_articulation("body_to_program_button_1")
    limits = lid_hinge.motion_limits

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.008,
            max_gap=0.018,
            name="closed cooking plates keep a realistic batter gap",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.140,
            name="closed lid covers the lower plate footprint",
        )

    if limits is not None and limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: limits.upper}):
            opened_aabb = ctx.part_world_aabb(lid)
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="upper_plate",
                negative_elem="lower_plate",
                min_gap=0.045,
                name="open lid lifts clearly above the lower plate",
            )
        ctx.check(
            "lid opens upward",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][2] > closed_aabb[1][2] + 0.10,
            details=f"closed_aabb={closed_aabb!r}, opened_aabb={opened_aabb!r}",
        )

    ctx.expect_origin_distance(
        button_0,
        button_1,
        axes="x",
        min_dist=0.020,
        max_dist=0.035,
        name="program buttons remain visibly discrete",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: 0.0018}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_still = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_1: 0.0018}):
        button_1_pressed = ctx.part_world_position(button_1)

    ctx.check(
        "button 0 presses independently",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.0012
        and button_1_rest is not None
        and button_1_still is not None
        and abs(button_1_still[2] - button_1_rest[2]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest!r}, button_0_pressed={button_0_pressed!r}, "
            f"button_1_rest={button_1_rest!r}, button_1_still={button_1_still!r}"
        ),
    )
    ctx.check(
        "button 1 presses independently",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.0012,
        details=f"button_1_rest={button_1_rest!r}, button_1_pressed={button_1_pressed!r}",
    )

    return ctx.report()


object_model = build_object_model()
