from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_DEPTH = 0.26
BODY_WIDTH = 0.31
BODY_HEIGHT = 0.60

CONTROL_RECESS_DEPTH = 0.010
CONTROL_PANEL_HEIGHT = 0.090
CONTROL_PANEL_WIDTH = 0.215
CONTROL_PANEL_THICKNESS = 0.005
CONTROL_CENTER_Z = 0.475

GRILLE_RECESS_DEPTH = 0.007
GRILLE_HEIGHT = 0.360
GRILLE_WIDTH = 0.235
GRILLE_CENTER_Z = 0.230

HANDLE_RECESS_LENGTH = 0.170
HANDLE_RECESS_WIDTH = 0.232
HANDLE_RECESS_DEPTH = 0.028
HANDLE_HINGE_X = -0.062
HANDLE_HINGE_Z = BODY_HEIGHT - 0.020


def _rounded_front_cutter(depth: float, width: float, height: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(depth, width, height, centered=(False, True, True))
        .edges("|X")
        .fillet(radius)
    )


def _make_body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.026)
    )

    grille_pocket = _rounded_front_cutter(
        GRILLE_RECESS_DEPTH,
        GRILLE_WIDTH + 0.010,
        GRILLE_HEIGHT + 0.020,
        0.014,
    ).translate((BODY_DEPTH / 2.0 - GRILLE_RECESS_DEPTH, 0.0, GRILLE_CENTER_Z))

    control_pocket = _rounded_front_cutter(
        CONTROL_RECESS_DEPTH,
        CONTROL_PANEL_WIDTH + 0.018,
        CONTROL_PANEL_HEIGHT + 0.016,
        0.012,
    ).translate((BODY_DEPTH / 2.0 - CONTROL_RECESS_DEPTH, 0.0, CONTROL_CENTER_Z))

    handle_pocket = (
        cq.Workplane("XY")
        .box(
            HANDLE_RECESS_LENGTH,
            HANDLE_RECESS_WIDTH,
            HANDLE_RECESS_DEPTH,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.020)
        .translate((0.0, 0.0, BODY_HEIGHT - HANDLE_RECESS_DEPTH))
    )

    left_handle_pad = (
        cq.Workplane("XY")
        .box(0.020, 0.030, 0.002, centered=(True, True, False))
        .translate((HANDLE_HINGE_X + 0.010, 0.093, BODY_HEIGHT - HANDLE_RECESS_DEPTH))
    )
    right_handle_pad = (
        cq.Workplane("XY")
        .box(0.020, 0.030, 0.002, centered=(True, True, False))
        .translate((HANDLE_HINGE_X + 0.010, -0.093, BODY_HEIGHT - HANDLE_RECESS_DEPTH))
    )

    return (
        shell.cut(grille_pocket)
        .cut(control_pocket)
        .cut(handle_pocket)
        .union(left_handle_pad)
        .union(right_handle_pad)
    )


def _make_control_panel() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            CONTROL_PANEL_HEIGHT,
            CONTROL_PANEL_WIDTH,
            CONTROL_PANEL_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.007)
    )


def _make_handle() -> cq.Workplane:
    arm_length = 0.110
    arm_width = 0.018
    arm_thickness = 0.012
    arm_offset = 0.093
    grip_depth = 0.020
    grip_width = 0.204

    left_arm = cq.Workplane("XY").box(
        arm_length,
        arm_width,
        arm_thickness,
        centered=(False, True, True),
    ).translate((0.0, arm_offset, 0.0))
    right_arm = cq.Workplane("XY").box(
        arm_length,
        arm_width,
        arm_thickness,
        centered=(False, True, True),
    ).translate((0.0, -arm_offset, 0.0))
    grip = cq.Workplane("XY").box(
        grip_depth,
        grip_width,
        arm_thickness + 0.002,
        centered=(False, True, True),
    ).translate((arm_length - grip_depth, 0.0, 0.0))

    return (
        left_arm.union(right_arm)
        .union(grip)
        .edges("|Z")
        .fillet(0.004)
    )


def _make_button_cap(height: float, width: float, depth: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(height, width, depth, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_party_speaker")

    housing = model.material("housing", rgba=(0.09, 0.09, 0.10, 1.0))
    panel = model.material("panel", rgba=(0.13, 0.13, 0.14, 1.0))
    grille_mat = model.material("grille", rgba=(0.04, 0.04, 0.05, 1.0))
    handle_mat = model.material("handle", rgba=(0.15, 0.15, 0.16, 1.0))
    button_mat = model.material("button", rgba=(0.22, 0.22, 0.24, 1.0))
    knob_mat = model.material("knob", rgba=(0.11, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "speaker_body"),
        material=housing,
        name="shell",
    )

    control_face = model.part("control_face")
    control_face.visual(
        mesh_from_cadquery(_make_control_panel(), "control_face"),
        material=panel,
        name="plate",
    )
    model.articulation(
        "body_to_control_face",
        ArticulationType.FIXED,
        parent=body,
        child=control_face,
        origin=Origin(
            xyz=(BODY_DEPTH / 2.0 - CONTROL_RECESS_DEPTH, 0.0, CONTROL_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    grille = model.part("grille")
    grille.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (GRILLE_HEIGHT, GRILLE_WIDTH),
                0.004,
                hole_diameter=0.0055,
                pitch=(0.0105, 0.0105),
                frame=0.012,
                corner_radius=0.010,
                stagger=True,
                center=False,
            ),
            "speaker_grille",
        ),
        material=grille_mat,
        name="panel",
    )
    model.articulation(
        "body_to_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
        origin=Origin(
            xyz=(BODY_DEPTH / 2.0 - GRILLE_RECESS_DEPTH, 0.0, GRILLE_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_make_handle(), "top_handle"),
        material=handle_mat,
        name="frame",
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_HINGE_X, 0.0, HANDLE_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.22,
        ),
    )

    volume_knob = model.part("volume_knob")
    volume_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.018,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(style="knurled", count=28, depth=0.0007, helix_angle_deg=18.0),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
                center=False,
            ),
            "volume_knob",
        ),
        material=knob_mat,
        name="knob",
    )
    model.articulation(
        "control_face_to_volume_knob",
        ArticulationType.CONTINUOUS,
        parent=control_face,
        child=volume_knob,
        origin=Origin(xyz=(0.016, -0.064, CONTROL_PANEL_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=12.0),
    )

    button_positions = (0.010, 0.047, 0.084)
    for index, lateral_y in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            mesh_from_cadquery(
                _make_button_cap(height=0.013, width=0.024, depth=0.006),
                f"button_{index}_cap",
            ),
            material=button_mat,
            name="cap",
        )
        model.articulation(
            f"control_face_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_face,
            child=button,
            origin=Origin(xyz=(-0.006, lateral_y, CONTROL_PANEL_THICKNESS)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0025,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    control_face = object_model.get_part("control_face")
    grille = object_model.get_part("grille")
    handle = object_model.get_part("handle")
    volume_knob = object_model.get_part("volume_knob")
    handle_joint = object_model.get_articulation("body_to_handle")
    knob_joint = object_model.get_articulation("control_face_to_volume_knob")

    ctx.allow_overlap(
        body,
        handle,
        elem_a="shell",
        elem_b="frame",
        reason="The fold-flat carry handle intentionally nests into the recessed top pocket of the simplified housing shell.",
    )

    ctx.expect_gap(
        control_face,
        grille,
        axis="z",
        min_gap=0.010,
        name="control face sits above the main grille",
    )
    ctx.expect_gap(
        volume_knob,
        control_face,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        name="volume knob seats against the control face",
    )

    knob_rest_pos = ctx.part_world_position(volume_knob)
    with ctx.pose({knob_joint: 1.8}):
        turned_pos = ctx.part_world_position(volume_knob)
    ctx.check(
        "volume knob rotates in place",
        knob_rest_pos is not None
        and turned_pos is not None
        and abs(knob_rest_pos[0] - turned_pos[0]) < 1e-9
        and abs(knob_rest_pos[1] - turned_pos[1]) < 1e-9
        and abs(knob_rest_pos[2] - turned_pos[2]) < 1e-9,
        details=f"rest={knob_rest_pos}, turned={turned_pos}",
    )

    handle_limits = handle_joint.motion_limits
    if handle_limits is not None and handle_limits.lower is not None and handle_limits.upper is not None:
        with ctx.pose({handle_joint: handle_limits.lower}):
            closed_handle_aabb = ctx.part_world_aabb(handle)
            body_aabb = ctx.part_world_aabb(body)
        with ctx.pose({handle_joint: handle_limits.upper}):
            open_handle_aabb = ctx.part_world_aabb(handle)

        body_top = body_aabb[1][2] if body_aabb is not None else None
        closed_top = closed_handle_aabb[1][2] if closed_handle_aabb is not None else None
        open_top = open_handle_aabb[1][2] if open_handle_aabb is not None else None

        ctx.check(
            "handle nests into the top recess when closed",
            body_top is not None and closed_top is not None and closed_top <= body_top + 0.002,
            details=f"body_top={body_top}, closed_top={closed_top}",
        )
        ctx.check(
            "handle lifts clearly above the cabinet when opened",
            body_top is not None and open_top is not None and open_top >= body_top + 0.070,
            details=f"body_top={body_top}, open_top={open_top}",
        )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"control_face_to_button_{index}")
        button_limits = button_joint.motion_limits

        ctx.expect_gap(
            button,
            grille,
            axis="z",
            min_gap=0.020,
            name=f"button_{index} stays above the grille",
        )
        ctx.expect_gap(
            button,
            control_face,
            axis="x",
            min_gap=0.0,
            max_gap=0.0002,
            name=f"button_{index} sits on the control face",
        )

        if button_limits is not None and button_limits.upper is not None:
            rest_pos = ctx.part_world_position(button)
            with ctx.pose({button_joint: button_limits.upper}):
                pressed_pos = ctx.part_world_position(button)

            ctx.check(
                f"button_{index} depresses inward",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[0] < rest_pos[0] - 0.0015,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    return ctx.report()


object_model = build_object_model()
