from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.300
BODY_WIDTH = 0.340
BODY_HEIGHT = 0.168
BODY_CORNER_RADIUS = 0.058
BODY_WALL = 0.008

HINGE_X = -0.136
HINGE_Z = 0.169

PANEL_X = 0.150
PANEL_Z = 0.064
PANEL_WIDTH = 0.160
PANEL_HEIGHT = 0.100


def _body_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_DEPTH - 2.0 * BODY_WALL,
            BODY_WIDTH - 2.0 * BODY_WALL,
            BODY_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS - BODY_WALL)
        .translate((0.0, 0.0, BODY_WALL))
    )
    shell = outer.cut(inner)

    control_opening = (
        cq.Workplane("XY")
        .box(0.070, 0.138, 0.088, centered=(True, True, True))
        .translate((BODY_DEPTH * 0.5 - 0.020, 0.0, PANEL_Z))
    )
    return shell.cut(control_opening)


def _base_ring_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BODY_DEPTH * 0.96, BODY_WIDTH * 0.92, 0.018, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.044)
    )


def _hinge_band_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(HINGE_X, 0.157)
        .rect(0.032, 0.024)
        .extrude(BODY_WIDTH * 0.36, both=True)
        .edges("|Y")
        .fillet(0.010)
    )


def _lid_shape() -> cq.Workplane:
    lid_depth = 0.286
    lid_width = 0.314
    lid_height = 0.050
    wall = 0.007

    outer = (
        cq.Workplane("XY")
        .box(lid_depth, lid_width, lid_height, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.032)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            lid_depth - 2.0 * wall,
            lid_width - 2.0 * wall,
            lid_height - wall + 0.002,
            centered=(False, True, False),
        )
        .translate((wall, 0.0, -0.002))
        .edges("|Z")
        .fillet(0.024)
    )
    lid = outer.cut(inner)

    front_lip = (
        cq.Workplane("XY")
        .box(0.020, 0.130, 0.010, centered=(True, True, True))
        .translate((lid_depth - 0.016, 0.0, 0.010))
        .edges("|X")
        .fillet(0.004)
    )
    rear_apron = (
        cq.Workplane("XY")
        .box(0.016, lid_width * 0.80, 0.018, centered=(False, True, False))
        .translate((-0.010, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.006)
    )
    return lid.union(front_lip).union(rear_apron)


def _panel_shape() -> cq.Workplane:
    front_plate = (
        cq.Workplane("XY")
        .box(0.004, PANEL_WIDTH, PANEL_HEIGHT, centered=(True, True, True))
        .translate((-0.002, 0.0, 0.0))
        .edges("|X")
        .fillet(0.014)
    )
    rear_insert = (
        cq.Workplane("XY")
        .box(0.010, 0.128, 0.078, centered=(True, True, True))
        .translate((-0.009, 0.0, 0.0))
        .edges("|X")
        .fillet(0.010)
    )
    panel = front_plate.union(rear_insert)

    latch_cut = (
        cq.Workplane("XY")
        .box(0.030, 0.070, 0.022, centered=(True, True, True))
        .translate((-0.007, 0.0, 0.020))
    )
    left_cut = (
        cq.Workplane("XY")
        .box(0.030, 0.036, 0.020, centered=(True, True, True))
        .translate((-0.007, -0.034, -0.022))
    )
    right_cut = (
        cq.Workplane("XY")
        .box(0.030, 0.036, 0.020, centered=(True, True, True))
        .translate((-0.007, 0.034, -0.022))
    )
    display_recess = (
        cq.Workplane("XY")
        .box(0.003, 0.082, 0.020, centered=(True, True, True))
        .translate((-0.001, 0.0, 0.048))
    )
    return panel.cut(latch_cut).cut(left_cut).cut(right_cut).cut(display_recess)


def _button_shape(
    *,
    width: float,
    height: float,
    cap_thickness: float,
    stem_length: float,
    stem_width: float,
    stem_height: float,
) -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(cap_thickness, width, height, centered=(True, True, True))
        .translate((cap_thickness * 0.5, 0.0, 0.0))
        .edges("|X")
        .fillet(min(width, height) * 0.40)
    )
    stem = (
        cq.Workplane("XY")
        .box(stem_length, stem_width, stem_height, centered=(True, True, True))
        .translate((-stem_length * 0.5, 0.0, 0.0))
        .edges("|X")
        .fillet(min(stem_width, stem_height) * 0.22)
    )
    rear_retainer = (
        cq.Workplane("XY")
        .box(0.004, width + 0.010, height + 0.006, centered=(True, True, True))
        .translate((-stem_length + 0.002, 0.0, 0.0))
        .edges("|X")
        .fillet(min(width + 0.010, height + 0.006) * 0.18)
    )
    return cap.union(stem).union(rear_retainer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_rice_cooker")

    model.material("shell_white", rgba=(0.94, 0.94, 0.92, 1.0))
    model.material("base_charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("button_silver", rgba=(0.81, 0.81, 0.79, 1.0))
    model.material("button_cream", rgba=(0.90, 0.89, 0.85, 1.0))
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "rice_cooker_body_shell"),
        material="shell_white",
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_base_ring_shape(), "rice_cooker_base_ring"),
        material="base_charcoal",
        name="base_ring",
    )
    body.visual(
        mesh_from_cadquery(_hinge_band_shape(), "rice_cooker_hinge_band"),
        material="base_charcoal",
        name="hinge_band",
    )
    body.visual(
        mesh_from_cadquery(_panel_shape(), "rice_cooker_control_panel"),
        origin=Origin(xyz=(PANEL_X, 0.0, PANEL_Z)),
        material="panel_black",
        name="panel_face",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "rice_cooker_lid"),
        material="shell_white",
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.080, 0.080, 0.044), rpy=(0.0, 1.57079632679, 0.0)),
        material="base_charcoal",
        name="steam_vent",
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_cadquery(
            _button_shape(
                width=0.064,
                height=0.018,
                cap_thickness=0.009,
                stem_length=0.018,
                stem_width=0.028,
                stem_height=0.012,
            ),
            "rice_cooker_latch_button",
        ),
        material="button_silver",
        name="button_body",
    )

    menu_button_0 = model.part("menu_button_0")
    menu_button_0.visual(
        mesh_from_cadquery(
            _button_shape(
                width=0.032,
                height=0.016,
                cap_thickness=0.008,
                stem_length=0.016,
                stem_width=0.016,
                stem_height=0.010,
            ),
            "rice_cooker_menu_button_0",
        ),
        material="button_cream",
        name="button_body",
    )

    menu_button_1 = model.part("menu_button_1")
    menu_button_1.visual(
        mesh_from_cadquery(
            _button_shape(
                width=0.032,
                height=0.016,
                cap_thickness=0.008,
                stem_length=0.016,
                stem_width=0.016,
                stem_height=0.010,
            ),
            "rice_cooker_menu_button_1",
        ),
        material="button_cream",
        name="button_body",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.6,
            lower=0.0,
            upper=1.22,
        ),
    )

    model.articulation(
        "latch_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(PANEL_X, 0.0, PANEL_Z + 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.050,
            lower=0.0,
            upper=0.005,
        ),
    )
    model.articulation(
        "menu_button_0_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_0,
        origin=Origin(xyz=(PANEL_X, -0.034, PANEL_Z - 0.022)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.050,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "menu_button_1_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_1,
        origin=Origin(xyz=(PANEL_X, 0.034, PANEL_Z - 0.022)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.050,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def _check_press_motion(ctx: TestContext, joint_name: str, part_name: str, min_travel: float) -> None:
    joint = object_model.get_articulation(joint_name)
    part = object_model.get_part(part_name)
    rest = ctx.part_world_position(part)
    upper = joint.motion_limits.upper if joint.motion_limits is not None else None
    if upper is None:
        ctx.fail(f"{joint_name} has limits", "Expected a bounded press articulation.")
        return
    with ctx.pose({joint: upper}):
        pressed = ctx.part_world_position(part)
    ctx.check(
        f"{part_name} presses inward",
        rest is not None and pressed is not None and pressed[0] < rest[0] - min_travel,
        details=f"rest={rest}, pressed={pressed}",
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_button = object_model.get_part("latch_button")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")

    ctx.allow_overlap(
        body,
        latch_button,
        elem_a="panel_face",
        elem_b="button_body",
        reason="The latch button is captured by the simplified control-panel bezel and intentionally nests into that bezel volume.",
    )
    ctx.allow_overlap(
        body,
        menu_button_0,
        elem_a="panel_face",
        elem_b="button_body",
        reason="The menu button is intentionally retained inside the simplified bezel opening.",
    )
    ctx.allow_overlap(
        body,
        menu_button_1,
        elem_a="panel_face",
        elem_b="button_body",
        reason="The menu button is intentionally retained inside the simplified bezel opening.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="shell",
            min_overlap=0.240,
            name="closed lid covers the cooker body",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="shell",
            min_gap=0.000,
            max_gap=0.010,
            name="closed lid sits just above the shell rim",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if lid_upper is not None:
        with ctx.pose({lid_hinge: lid_upper}):
            open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.110,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    ctx.expect_origin_distance(
        "menu_button_0",
        "menu_button_1",
        axes="y",
        min_dist=0.060,
        max_dist=0.072,
        name="menu buttons stay as two distinct front controls",
    )

    _check_press_motion(ctx, "latch_press", "latch_button", 0.004)
    _check_press_motion(ctx, "menu_button_0_press", "menu_button_0", 0.003)
    _check_press_motion(ctx, "menu_button_1_press", "menu_button_1", 0.003)

    return ctx.report()


object_model = build_object_model()
