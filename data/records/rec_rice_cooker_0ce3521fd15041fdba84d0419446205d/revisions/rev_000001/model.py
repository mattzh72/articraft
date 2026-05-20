from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_RADIUS = 0.112
LOWER_BAND_RADIUS = 0.104
INNER_CAVITY_RADIUS = 0.098
BODY_HEIGHT = 0.176
LOWER_BAND_HEIGHT = 0.048
CAVITY_FLOOR_Z = 0.032

PANEL_Z = 0.088
PANEL_WIDTH = 0.112
PANEL_HEIGHT = 0.096
PANEL_OUTER_RADIUS = BODY_RADIUS + 0.0018
PANEL_INNER_RADIUS = BODY_RADIUS - 0.0004
PANEL_CLIP_DEPTH = 0.042

LATCH_Z = 0.128
LATCH_SLOT_WIDTH = 0.030
LATCH_SLOT_HEIGHT = 0.015
LATCH_TRAVEL = 0.0035

MENU_BUTTON_Z = 0.086
MENU_BUTTON_SPACING = 0.052
MENU_BUTTON_TRAVEL = 0.0025
MENU_SLOT_RADIUS = 0.0095


def _body_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .circle(LOWER_BAND_RADIUS)
        .extrude(LOWER_BAND_HEIGHT)
        .faces(">Z")
        .workplane()
        .circle(BODY_RADIUS)
        .extrude(BODY_HEIGHT - LOWER_BAND_HEIGHT)
    )

    cavity = (
        cq.Workplane("XY")
        .workplane(offset=CAVITY_FLOOR_Z)
        .circle(INNER_CAVITY_RADIUS)
        .extrude(BODY_HEIGHT - CAVITY_FLOOR_Z)
    )
    shell = shell.cut(cavity)

    latch_cut = (
        cq.Workplane("XZ")
        .workplane(offset=PANEL_OUTER_RADIUS + 0.010)
        .center(0.0, LATCH_Z)
        .rect(LATCH_SLOT_WIDTH, LATCH_SLOT_HEIGHT)
        .extrude(-0.040)
    )
    shell = shell.cut(latch_cut)

    for x in (-MENU_BUTTON_SPACING / 2.0, MENU_BUTTON_SPACING / 2.0):
        menu_cut = (
            cq.Workplane("XZ")
            .workplane(offset=PANEL_OUTER_RADIUS + 0.010)
            .center(x, MENU_BUTTON_Z)
            .circle(MENU_SLOT_RADIUS)
            .extrude(-0.040)
        )
        shell = shell.cut(menu_cut)

    return shell


def _control_panel_shape() -> cq.Workplane:
    curved_band = (
        cq.Workplane("XY")
        .workplane(offset=PANEL_Z - PANEL_HEIGHT / 2.0)
        .circle(PANEL_OUTER_RADIUS)
        .circle(PANEL_INNER_RADIUS)
        .extrude(PANEL_HEIGHT)
    )

    clip = (
        cq.Workplane("XY")
        .box(PANEL_WIDTH, PANEL_CLIP_DEPTH, PANEL_HEIGHT + 0.002, centered=(True, True, False))
        .translate((0.0, PANEL_OUTER_RADIUS - 0.006, PANEL_Z - PANEL_HEIGHT / 2.0 - 0.001))
    )
    panel = curved_band.intersect(clip)

    latch_cut = (
        cq.Workplane("XZ")
        .workplane(offset=PANEL_OUTER_RADIUS + 0.010)
        .center(0.0, LATCH_Z)
        .rect(0.046, 0.024)
        .extrude(-0.030)
    )
    panel = panel.cut(latch_cut)

    menu_cut = (
        cq.Workplane("XZ")
        .workplane(offset=PANEL_OUTER_RADIUS + 0.010)
        .center(0.0, MENU_BUTTON_Z)
        .rect(0.078, 0.032)
        .extrude(-0.030)
    )
    panel = panel.cut(menu_cut)

    return panel


def _curved_trim_shape(width: float, height: float, center_x: float, center_z: float) -> cq.Workplane:
    trim_band = (
        cq.Workplane("XY")
        .workplane(offset=center_z - height / 2.0)
        .circle(PANEL_OUTER_RADIUS)
        .circle(BODY_RADIUS - 0.0005)
        .extrude(height)
    )
    clip = (
        cq.Workplane("XY")
        .box(width, PANEL_CLIP_DEPTH, height + 0.002, centered=(True, True, False))
        .translate((center_x, PANEL_OUTER_RADIUS - 0.007, center_z - height / 2.0 - 0.001))
    )
    return trim_band.intersect(clip)


def _lid_geometry() -> LatheGeometry:
    lid_profile = [
        (0.121, 0.000),
        (0.121, 0.015),
        (0.106, 0.033),
        (0.078, 0.049),
        (0.038, 0.058),
        (0.004, 0.061),
        (0.004, 0.053),
        (0.039, 0.049),
        (0.073, 0.040),
        (0.115, 0.018),
        (0.115, 0.000),
    ]
    lid = LatheGeometry(lid_profile, segments=96)
    lid.translate(0.0, BODY_RADIUS, 0.0)
    return lid


def _latch_button_geometry() -> BoxGeometry:
    cap = BoxGeometry((0.036, 0.006, 0.018))
    cap.translate(0.0, 0.003, 0.0)

    stem = BoxGeometry((0.024, 0.019, 0.012))
    stem.translate(0.0, -0.009, 0.0)

    cap.merge(stem)
    return cap


def _menu_button_geometry() -> CylinderGeometry:
    cap = CylinderGeometry(0.011, 0.0045, radial_segments=28)
    cap.rotate_x(math.pi / 2.0)
    cap.translate(0.0, 0.00225, 0.0)

    stem = CylinderGeometry(0.0078, 0.015, radial_segments=24)
    stem.rotate_x(math.pi / 2.0)
    stem.translate(0.0, -0.0065, 0.0)

    cap.merge(stem)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rice_cooker")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.93, 1.0))
    lid_white = model.material("lid_white", rgba=(0.97, 0.97, 0.96, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    button_gray = model.material("button_gray", rgba=(0.80, 0.82, 0.84, 1.0))
    latch_gray = model.material("latch_gray", rgba=(0.72, 0.74, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material=body_white,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_curved_trim_shape(0.106, 0.016, 0.0, 0.148), "panel_top"),
        material=panel_gray,
        name="panel_top",
    )
    body.visual(
        mesh_from_cadquery(_curved_trim_shape(0.096, 0.014, 0.0, 0.063), "panel_bottom"),
        material=panel_gray,
        name="panel_bottom",
    )
    body.visual(
        mesh_from_cadquery(_curved_trim_shape(0.018, 0.020, 0.0, 0.101), "panel_divider"),
        material=panel_gray,
        name="panel_divider",
    )
    body.visual(
        mesh_from_cadquery(_curved_trim_shape(0.014, 0.060, -0.045, 0.102), "panel_side_0"),
        material=panel_gray,
        name="panel_side_0",
    )
    body.visual(
        mesh_from_cadquery(_curved_trim_shape(0.014, 0.060, 0.045, 0.102), "panel_side_1"),
        material=panel_gray,
        name="panel_side_1",
    )
    body.visual(
        Box((0.050, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.116, BODY_HEIGHT - 0.005)),
        material=panel_gray,
        name="hinge_pad",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_lid_geometry(), "lid_shell"),
        material=lid_white,
        name="shell",
    )
    lid.visual(
        Box((0.044, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.002, 0.005)),
        material=lid_white,
        name="hinge_leaf",
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_geometry(_latch_button_geometry(), "latch_button"),
        material=latch_gray,
        name="button",
    )

    menu_button_0 = model.part("menu_button_0")
    menu_button_0.visual(
        mesh_from_geometry(_menu_button_geometry(), "menu_button_0"),
        material=button_gray,
        name="button",
    )

    menu_button_1 = model.part("menu_button_1")
    menu_button_1.visual(
        mesh_from_geometry(_menu_button_geometry(), "menu_button_1"),
        material=button_gray,
        name="button",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -BODY_RADIUS, BODY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    model.articulation(
        "body_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(0.0, PANEL_OUTER_RADIUS + 0.0015, LATCH_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.05, lower=0.0, upper=LATCH_TRAVEL),
    )

    menu_x = MENU_BUTTON_SPACING / 2.0
    menu_y = math.sqrt((PANEL_OUTER_RADIUS + 0.0015) ** 2 - menu_x**2)

    model.articulation(
        "body_to_menu_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_0,
        origin=Origin(xyz=(-menu_x, menu_y, MENU_BUTTON_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.04, lower=0.0, upper=MENU_BUTTON_TRAVEL),
    )
    model.articulation(
        "body_to_menu_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_1,
        origin=Origin(xyz=(menu_x, menu_y, MENU_BUTTON_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.04, lower=0.0, upper=MENU_BUTTON_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch_button = object_model.get_part("latch_button")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_joint = object_model.get_articulation("body_to_latch_button")
    menu_joint_0 = object_model.get_articulation("body_to_menu_button_0")
    menu_joint_1 = object_model.get_articulation("body_to_menu_button_1")

    ctx.allow_overlap(
        body,
        latch_button,
        reason="The latch plunger is modeled with a retained inner stem sliding inside the front switch guide proxy.",
    )
    ctx.allow_overlap(
        body,
        menu_button_0,
        reason="The first menu button keeps an inner plunger stem inside the simplified front control housing.",
    )
    ctx.allow_overlap(
        body,
        menu_button_1,
        reason="The second menu button keeps an inner plunger stem inside the simplified front control housing.",
    )

    ctx.expect_origin_distance(
        menu_button_0,
        menu_button_1,
        axes="x",
        min_dist=0.045,
        name="menu buttons stay visibly separate",
    )
    ctx.expect_origin_gap(
        latch_button,
        menu_button_0,
        axis="z",
        min_gap=0.030,
        name="latch button sits above the menu buttons",
    )

    closed_aabb = None
    open_aabb = None
    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.lower is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.lower}):
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                min_gap=0.0,
                max_gap=0.006,
                name="closed lid sits on the body rim",
            )
            ctx.expect_overlap(
                lid,
                body,
                axes="xy",
                min_overlap=0.190,
                name="closed lid covers the cooker opening",
            )
            closed_aabb = ctx.part_world_aabb(lid)

        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.120,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    latch_rest = ctx.part_world_position(latch_button)
    with ctx.pose({latch_joint: LATCH_TRAVEL}):
        latch_pressed = ctx.part_world_position(latch_button)
        ctx.expect_overlap(
            latch_button,
            body,
            axes="xz",
            min_overlap=0.018,
            name="latch button stays guided by the front latch slot",
        )
    ctx.check(
        "latch button depresses into the body",
        latch_rest is not None and latch_pressed is not None and latch_pressed[1] < latch_rest[1] - 0.002,
        details=f"rest={latch_rest}, pressed={latch_pressed}",
    )

    menu_0_rest = ctx.part_world_position(menu_button_0)
    menu_1_rest = ctx.part_world_position(menu_button_1)

    with ctx.pose({menu_joint_0: MENU_BUTTON_TRAVEL}):
        menu_0_pressed = ctx.part_world_position(menu_button_0)
        menu_1_steady = ctx.part_world_position(menu_button_1)
    ctx.check(
        "menu_button_0 depresses independently",
        menu_0_rest is not None
        and menu_0_pressed is not None
        and menu_0_pressed[1] < menu_0_rest[1] - 0.0015
        and menu_1_rest is not None
        and menu_1_steady is not None
        and abs(menu_1_steady[1] - menu_1_rest[1]) < 0.0005,
        details=(
            f"button_0_rest={menu_0_rest}, button_0_pressed={menu_0_pressed}, "
            f"button_1_rest={menu_1_rest}, button_1_same_pose={menu_1_steady}"
        ),
    )

    with ctx.pose({menu_joint_1: MENU_BUTTON_TRAVEL}):
        menu_1_pressed = ctx.part_world_position(menu_button_1)
        menu_0_steady = ctx.part_world_position(menu_button_0)
    ctx.check(
        "menu_button_1 depresses independently",
        menu_1_rest is not None
        and menu_1_pressed is not None
        and menu_1_pressed[1] < menu_1_rest[1] - 0.0015
        and menu_0_rest is not None
        and menu_0_steady is not None
        and abs(menu_0_steady[1] - menu_0_rest[1]) < 0.0005,
        details=(
            f"button_1_rest={menu_1_rest}, button_1_pressed={menu_1_pressed}, "
            f"button_0_rest={menu_0_rest}, button_0_same_pose={menu_0_steady}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
