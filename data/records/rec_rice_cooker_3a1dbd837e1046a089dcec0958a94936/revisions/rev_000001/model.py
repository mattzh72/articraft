from __future__ import annotations

import math

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


BODY_DEPTH = 0.205
BODY_WIDTH = 0.188
BODY_HEIGHT = 0.114
HINGE_X = -0.090
HINGE_Z = 0.112
LID_DEPTH = 0.185
LID_WIDTH = 0.181
CONTROL_FACE_X = BODY_DEPTH * 0.5 + 0.002


def _body_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.024)
    )

    front_pod = (
        cq.Workplane("XY")
        .box(0.022, 0.094, 0.058, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((BODY_DEPTH * 0.5 - 0.009, 0.0, 0.012))
    )
    latch_pad = (
        cq.Workplane("XY")
        .box(0.014, 0.060, 0.028, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.006)
        .translate((CONTROL_FACE_X - 0.003, 0.0, BODY_HEIGHT - 0.033))
    )

    top_recess = (
        cq.Workplane("XY")
        .box(0.184, 0.164, 0.020, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.020)
        .translate((0.0, 0.0, BODY_HEIGHT - 0.010))
    )

    latch_pocket = (
        cq.Workplane("XY")
        .box(0.018, 0.024, 0.010, centered=(False, True, True))
        .translate((CONTROL_FACE_X - 0.0105, 0.0, BODY_HEIGHT - 0.033))
    )
    menu_pocket_0 = (
        cq.Workplane("XY")
        .box(0.016, 0.018, 0.010, centered=(False, True, True))
        .translate((CONTROL_FACE_X - 0.008, -0.022, 0.031))
    )
    menu_pocket_1 = (
        cq.Workplane("XY")
        .box(0.016, 0.018, 0.010, centered=(False, True, True))
        .translate((CONTROL_FACE_X - 0.008, 0.022, 0.031))
    )
    rear_saddle = (
        cq.Workplane("XY")
        .box(0.030, 0.078, 0.006, centered=(False, True, False))
        .translate((HINGE_X - 0.008, 0.0, BODY_HEIGHT - 0.006))
        .edges("|Z")
        .fillet(0.003)
    )

    return (
        shell.union(front_pod)
        .union(latch_pad)
        .cut(top_recess)
        .cut(latch_pocket)
        .cut(menu_pocket_0)
        .cut(menu_pocket_1)
        .union(rear_saddle)
    )


def _lid_shape() -> cq.Workplane:
    lid_shell = (
        cq.Workplane("XY")
        .box(LID_DEPTH, LID_WIDTH, 0.024, centered=(False, True, False))
        .translate((0.0, 0.0, 0.014))
        .edges("|Z")
        .fillet(0.010)
    )

    crown = (
        cq.Workplane("XY")
        .box(0.128, 0.145, 0.010, centered=(False, True, False))
        .translate((0.030, 0.0, 0.028))
        .edges("|Z")
        .fillet(0.010)
    )

    front_lip = (
        cq.Workplane("XY")
        .box(0.014, 0.050, 0.006, centered=(False, True, False))
        .translate((LID_DEPTH - 0.014, 0.0, 0.012))
        .edges("|Z")
        .fillet(0.004)
    )

    rear_bridge = (
        cq.Workplane("XY")
        .box(0.018, 0.074, 0.014, centered=(False, True, False))
        .translate((0.0, 0.0, 0.002))
        .edges("|Z")
        .fillet(0.004)
    )

    return lid_shell.union(crown).union(front_lip).union(rear_bridge)


def _button_shape(
    *,
    cap_depth: float,
    cap_width: float,
    cap_height: float,
    stem_depth: float,
    stem_width: float,
    stem_height: float,
) -> cq.Workplane:
    cap = cq.Workplane("XY").box(cap_depth, cap_width, cap_height, centered=(False, True, True))
    stem = (
        cq.Workplane("XY")
        .box(stem_depth, stem_width, stem_height, centered=(False, True, True))
        .translate((-stem_depth + 0.001, 0.0, 0.0))
    )
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_rice_cooker")

    body_shell = model.material("body_shell", rgba=(0.92, 0.92, 0.89, 1.0))
    lid_shell = model.material("lid_shell", rgba=(0.89, 0.89, 0.86, 1.0))
    button_light = model.material("button_light", rgba=(0.93, 0.93, 0.95, 1.0))
    button_dark = model.material("button_dark", rgba=(0.47, 0.48, 0.50, 1.0))
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "rice_cooker_body"),
        material=body_shell,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.220, 0.200, 0.135)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "rice_cooker_lid"),
        material=lid_shell,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_DEPTH, LID_WIDTH, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(LID_DEPTH * 0.5, 0.0, 0.010)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_cadquery(
            _button_shape(
                cap_depth=0.005,
                cap_width=0.040,
                cap_height=0.015,
                stem_depth=0.008,
                stem_width=0.024,
                stem_height=0.010,
            ),
            "rice_cooker_latch_button",
        ),
        material=button_dark,
        name="latch_button",
    )
    latch_button.inertial = Inertial.from_geometry(
        Box((0.013, 0.040, 0.015)),
        mass=0.018,
        origin=Origin(xyz=(-0.0015, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(CONTROL_FACE_X + 0.007, 0.0, BODY_HEIGHT - 0.033)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.003,
        ),
    )

    menu_button_mesh = mesh_from_cadquery(
        _button_shape(
            cap_depth=0.004,
            cap_width=0.026,
            cap_height=0.018,
            stem_depth=0.007,
            stem_width=0.018,
            stem_height=0.010,
        ),
        "rice_cooker_menu_button",
    )
    for index, button_y in enumerate((-0.022, 0.022)):
        menu_button = model.part(f"menu_button_{index}")
        menu_button.visual(
            menu_button_mesh,
            material=button_light,
            name="menu_button",
        )
        menu_button.inertial = Inertial.from_geometry(
            Box((0.011, 0.026, 0.018)),
            mass=0.012,
            origin=Origin(xyz=(-0.0015, 0.0, 0.0)),
        )
        model.articulation(
            f"body_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=menu_button,
            origin=Origin(xyz=(CONTROL_FACE_X + 0.006, button_y, 0.031)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.05,
                lower=0.0,
                upper=0.002,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_button = object_model.get_part("latch_button")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    latch_joint = object_model.get_articulation("body_to_latch_button")
    menu_joint_0 = object_model.get_articulation("body_to_menu_button_0")
    menu_joint_1 = object_model.get_articulation("body_to_menu_button_1")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.120,
            name="closed lid covers the cooker opening",
        )

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        closed_aabb = None
        with ctx.pose({lid_hinge: 0.0}):
            closed_aabb = ctx.part_world_aabb(lid)
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.080,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    latch_rest = ctx.part_world_position(latch_button)
    with ctx.pose({latch_joint: latch_joint.motion_limits.upper}):
        latch_pressed = ctx.part_world_position(latch_button)
    ctx.check(
        "front latch button depresses into the body",
        latch_rest is not None and latch_pressed is not None and latch_pressed[0] < latch_rest[0] - 0.002,
        details=f"rest={latch_rest}, pressed={latch_pressed}",
    )

    menu_0_rest = ctx.part_world_position(menu_button_0)
    menu_1_rest = ctx.part_world_position(menu_button_1)
    with ctx.pose({menu_joint_0: menu_joint_0.motion_limits.upper}):
        menu_0_pressed = ctx.part_world_position(menu_button_0)
        menu_1_during_0 = ctx.part_world_position(menu_button_1)
    ctx.check(
        "menu button 0 depresses independently",
        menu_0_rest is not None
        and menu_0_pressed is not None
        and menu_0_pressed[0] < menu_0_rest[0] - 0.0015
        and menu_1_rest is not None
        and menu_1_during_0 is not None
        and abs(menu_1_during_0[0] - menu_1_rest[0]) < 1e-6,
        details=f"button_0_rest={menu_0_rest}, button_0_pressed={menu_0_pressed}, button_1_rest={menu_1_rest}, button_1_during={menu_1_during_0}",
    )

    with ctx.pose({menu_joint_1: menu_joint_1.motion_limits.upper}):
        menu_1_pressed = ctx.part_world_position(menu_button_1)
        menu_0_during_1 = ctx.part_world_position(menu_button_0)
    ctx.check(
        "menu button 1 depresses independently",
        menu_1_rest is not None
        and menu_1_pressed is not None
        and menu_1_pressed[0] < menu_1_rest[0] - 0.0015
        and menu_0_rest is not None
        and menu_0_during_1 is not None
        and abs(menu_0_during_1[0] - menu_0_rest[0]) < 1e-6,
        details=f"button_1_rest={menu_1_rest}, button_1_pressed={menu_1_pressed}, button_0_rest={menu_0_rest}, button_0_during={menu_0_during_1}",
    )

    return ctx.report()


object_model = build_object_model()
