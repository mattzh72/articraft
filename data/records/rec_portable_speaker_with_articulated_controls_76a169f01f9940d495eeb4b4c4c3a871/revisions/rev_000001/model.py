from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.220
BODY_D = 0.082
BODY_H = 0.145
BODY_R = 0.016
FRONT_Y = BODY_D * 0.5

CONTROL_W = 0.162
CONTROL_H = 0.040
CONTROL_Z = 0.109
CONTROL_POCKET_D = 0.008

GRILLE_W = 0.176
GRILLE_H = 0.074
GRILLE_T = 0.0032
GRILLE_Z = 0.048
GRILLE_POCKET_D = 0.009

DIAL_RADIUS = 0.015
BUTTON_W = 0.020
BUTTON_H = 0.012
BUTTON_ROW_Z = -0.010
BUTTON_XS = (-0.040, 0.000, 0.040)

PIVOT_Z = BODY_H - 0.019
PIVOT_SPAN = 0.188
PIVOT_BOSS_LEN = 0.018


def _body_shell() -> object:
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H * 0.5))
        .edges("|Z")
        .fillet(BODY_R)
        .edges(">Z")
        .fillet(0.010)
        .edges("<Z")
        .fillet(0.010)
    )

    control_pocket = (
        cq.Workplane("XY")
        .box(CONTROL_W + 0.014, CONTROL_POCKET_D, CONTROL_H + 0.010)
        .translate((0.0, FRONT_Y - CONTROL_POCKET_D * 0.5 + 0.0005, CONTROL_Z))
    )
    grille_pocket = (
        cq.Workplane("XY")
        .box(GRILLE_W + 0.012, GRILLE_POCKET_D, GRILLE_H + 0.012)
        .translate((0.0, FRONT_Y - GRILLE_POCKET_D * 0.5 + 0.0005, GRILLE_Z))
    )
    left_boss = (
        cq.Workplane("XY")
        .box(PIVOT_BOSS_LEN, 0.010, 0.018)
        .translate((-PIVOT_SPAN * 0.5, 0.0, PIVOT_Z))
    )
    right_boss = (
        cq.Workplane("XY")
        .box(PIVOT_BOSS_LEN, 0.010, 0.018)
        .translate((PIVOT_SPAN * 0.5, 0.0, PIVOT_Z))
    )
    return shell.union(left_boss).union(right_boss).cut(control_pocket).cut(grille_pocket)

def _materials(model: ArticulatedObject) -> dict[str, Material]:
    return {
        "shell": model.material("shell_charcoal", rgba=(0.19, 0.20, 0.22, 1.0)),
        "trim": model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0)),
        "grille": model.material("grille_black", rgba=(0.12, 0.13, 0.14, 1.0)),
        "strap": model.material("strap_black", rgba=(0.10, 0.10, 0.11, 1.0)),
        "dial": model.material("dial_grey", rgba=(0.28, 0.29, 0.31, 1.0)),
        "button": model.material("button_grey", rgba=(0.36, 0.38, 0.40, 1.0)),
    }


def _strap_mesh() -> object:
    bridge = (
        cq.Workplane("XY")
        .box(0.182, 0.007, 0.006)
        .translate((0.0, 0.0, 0.054))
    )
    left_arm = (
        cq.Workplane("XY")
        .box(0.006, 0.007, 0.038)
        .translate((-0.094, 0.0, 0.038))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(0.006, 0.007, 0.038)
        .translate((0.094, 0.0, 0.038))
    )
    return (
        bridge.union(left_arm)
        .union(right_arm)
        .edges("|Y")
        .fillet(0.002)
    )

def _button_mesh() -> object:
    return (
        cq.Workplane("XY")
        .box(BUTTON_W, 0.004, BUTTON_H)
        .translate((0.0, 0.002, 0.0))
        .edges("|Y")
        .fillet(0.0014)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_speaker")
    materials = _materials(model)
    strap_mesh = mesh_from_cadquery(_strap_mesh(), "speaker_strap")
    button_mesh = mesh_from_cadquery(_button_mesh(), "speaker_button")

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "speaker_body"),
        material=materials["shell"],
        name="shell",
    )
    body.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (GRILLE_W, GRILLE_H),
                GRILLE_T,
                hole_diameter=0.0042,
                pitch=(0.0088, 0.0088),
                frame=0.010,
                corner_radius=0.012,
                stagger=True,
            ),
            "speaker_grille",
        ),
        origin=Origin(
            xyz=(0.0, FRONT_Y - GRILLE_POCKET_D + GRILLE_T * 0.5, GRILLE_Z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=materials["grille"],
        name="grille",
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((CONTROL_W, 0.004, CONTROL_H)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=materials["trim"],
        name="control_face",
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(0.0, FRONT_Y - CONTROL_POCKET_D + 0.0005, CONTROL_Z)),
    )

    strap = model.part("strap")
    strap.visual(strap_mesh, material=materials["strap"], name="strap_band")
    model.articulation(
        "body_to_strap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=strap,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.2,
            velocity=1.2,
            lower=0.0,
            upper=0.82,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=materials["dial"],
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=DIAL_RADIUS * 0.80, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0069, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=materials["dial"],
        name="dial_cap",
    )
    model.articulation(
        "control_panel_to_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=dial,
        origin=Origin(xyz=(0.0, 0.004, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(button_mesh, material=materials["button"], name="button_cap")
        model.articulation(
            f"control_panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(button_x, 0.004, BUTTON_ROW_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0018,
            ),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    control_panel = object_model.get_part("control_panel")
    strap = object_model.get_part("strap")
    dial = object_model.get_part("dial")
    strap_joint = object_model.get_articulation("body_to_strap")
    dial_joint = object_model.get_articulation("control_panel_to_dial")
    button_parts = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"control_panel_to_button_{index}") for index in range(3)]

    ctx.expect_gap(
        control_panel,
        body,
        axis="z",
        positive_elem="control_face",
        negative_elem="grille",
        min_gap=0.004,
        name="control panel stays clearly above the grille field",
    )
    ctx.expect_overlap(
        control_panel,
        body,
        axes="x",
        elem_a="control_face",
        elem_b="grille",
        min_overlap=0.140,
        name="control panel and grille share the main front width",
    )
    ctx.expect_overlap(
        dial,
        control_panel,
        axes="xz",
        elem_a="dial_knob",
        elem_b="control_face",
        min_overlap=0.024,
        name="dial stays centered on the control panel",
    )
    ctx.expect_gap(
        strap,
        body,
        axis="z",
        positive_elem="strap_band",
        negative_elem="shell",
        min_gap=0.0,
        max_gap=0.0005,
        name="strap sits on the top pivot line without penetrating the shell",
    )

    for index, button in enumerate(button_parts):
        ctx.expect_overlap(
            button,
            control_panel,
            axes="xz",
            elem_a="button_cap",
            elem_b="control_face",
            min_overlap=0.008,
            name=f"button_{index} stays within the control face footprint",
        )

    ctx.check(
        "dial uses continuous rotation",
        getattr(dial_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"joint_type={getattr(dial_joint, 'articulation_type', None)}",
    )

    for index, (button, joint) in enumerate(zip(button_parts, button_joints)):
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.0018}):
            moved_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses inward",
            rest_pos is not None and moved_pos is not None and moved_pos[1] < rest_pos[1] - 0.001,
            details=f"rest={rest_pos}, pressed={moved_pos}",
        )

    rest_button_0 = ctx.part_world_position(button_parts[0])
    rest_button_1 = ctx.part_world_position(button_parts[1])
    rest_button_2 = ctx.part_world_position(button_parts[2])
    with ctx.pose({button_joints[1]: 0.0018}):
        moved_button_0 = ctx.part_world_position(button_parts[0])
        moved_button_1 = ctx.part_world_position(button_parts[1])
        moved_button_2 = ctx.part_world_position(button_parts[2])
    ctx.check(
        "buttons depress independently",
        (
            rest_button_0 is not None
            and rest_button_1 is not None
            and rest_button_2 is not None
            and moved_button_0 is not None
            and moved_button_1 is not None
            and moved_button_2 is not None
            and abs(moved_button_0[1] - rest_button_0[1]) < 1e-6
            and moved_button_1[1] < rest_button_1[1] - 0.001
            and abs(moved_button_2[1] - rest_button_2[1]) < 1e-6
        ),
        details=f"button_0={moved_button_0}, button_1={moved_button_1}, button_2={moved_button_2}",
    )

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3)) if aabb is not None else None

    rest_strap = _aabb_center(ctx.part_element_world_aabb(strap, elem="strap_band"))
    with ctx.pose({strap_joint: 0.82}):
        folded_strap = _aabb_center(ctx.part_element_world_aabb(strap, elem="strap_band"))
    ctx.check(
        "strap folds rearward from the carry position",
        rest_strap is not None
        and folded_strap is not None
        and folded_strap[1] < rest_strap[1] - 0.020
        and folded_strap[2] < rest_strap[2] - 0.008,
        details=f"rest={rest_strap}, folded={folded_strap}",
    )

    return ctx.report()


object_model = build_object_model()
