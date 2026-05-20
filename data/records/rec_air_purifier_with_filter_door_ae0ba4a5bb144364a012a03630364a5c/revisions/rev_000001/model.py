from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSleeve,
    VentGrilleSlats,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.170
BODY_D = 0.130
BODY_H = 0.285
BODY_RADIUS = 0.017
WALL = 0.003
BOTTOM_WALL = 0.006

INTAKE_W = 0.118
INTAKE_H = 0.188
INTAKE_Z = 0.122

OUTLET_W = 0.086
OUTLET_H = 0.056
OUTLET_OPEN_W = 0.068
OUTLET_OPEN_H = 0.040
OUTLET_X = -0.020
OUTLET_Y = 0.010

ROCKER_W = 0.026
ROCKER_L = 0.015
ROCKER_T = 0.008
ROCKER_X = 0.050
ROCKER_Y = 0.012
ROCKER_AXIS_Z = BODY_H + 0.0045
ROCKER_TRAVEL = 0.23

PANEL_W = BODY_W - 0.018
PANEL_H = 0.244
PANEL_T = 0.004
PANEL_Z = 0.145
PANEL_GAP = 0.0028
PANEL_OPEN = 1.55

TRAY_W = 0.118
TRAY_H = 0.206
TRAY_D = 0.068
TRAY_Z = 0.135
TRAY_JOINT_Y = BODY_D / 2.0 - WALL - 0.014
TRAY_REAR_OFFSET = 0.004
TRAY_TRAVEL = 0.050


def _body_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(BODY_RADIUS)
    )
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * WALL, BODY_D - WALL + 0.020, BODY_H - BOTTOM_WALL - WALL)
        .translate((0.0, WALL / 2.0 + 0.010, BOTTOM_WALL + (BODY_H - BOTTOM_WALL - WALL) / 2.0))
    )

    intake_cut = (
        cq.Workplane("XY")
        .box(INTAKE_W, 0.020, INTAKE_H)
        .translate((0.0, -BODY_D / 2.0 + 0.010, INTAKE_Z))
    )
    outlet_cut = (
        cq.Workplane("XY")
        .box(OUTLET_OPEN_W, OUTLET_OPEN_H, 0.020)
        .translate((OUTLET_X, OUTLET_Y, BODY_H - 0.010))
    )
    rocker_cut = (
        cq.Workplane("XY")
        .box(0.020, 0.010, 0.014)
        .translate((ROCKER_X, ROCKER_Y, BODY_H - 0.007))
    )

    return outer.cut(inner).cut(intake_cut).cut(outlet_cut).cut(rocker_cut)


def _rear_panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_T, PANEL_H)
        .translate((PANEL_W / 2.0, 0.0, 0.0))
    )

    pull_lip = (
        cq.Workplane("XY")
        .box(0.012, 0.008, 0.052)
        .translate((PANEL_W - 0.006, PANEL_T / 2.0 + 0.004, 0.0))
    )

    upper_knuckle = (
        cq.Workplane("XY")
        .cylinder(0.070, 0.0045)
        .translate((0.0, 0.0, PANEL_H / 2.0 - 0.050))
    )
    lower_knuckle = (
        cq.Workplane("XY")
        .cylinder(0.070, 0.0045)
        .translate((0.0, 0.0, -PANEL_H / 2.0 + 0.050))
    )

    return panel.union(pull_lip).union(upper_knuckle).union(lower_knuckle)


def _filter_tray_shape() -> cq.Workplane:
    tray_center_y = TRAY_REAR_OFFSET - TRAY_D / 2.0
    outer = (
        cq.Workplane("XY")
        .box(TRAY_W, TRAY_D, TRAY_H)
        .translate((0.0, tray_center_y, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(TRAY_W - 0.010, TRAY_D + 0.004, TRAY_H - 0.010)
        .translate((0.0, tray_center_y, 0.0))
    )
    pull = (
        cq.Workplane("XY")
        .box(0.082, 0.008, 0.018)
        .translate((0.0, TRAY_REAR_OFFSET + 0.001, -TRAY_H / 2.0 + 0.020))
    )
    runner_a = (
        cq.Workplane("XY")
        .box(0.016, 0.030, 0.170)
        .translate((-TRAY_W / 2.0 - 0.004, -0.011, 0.0))
    )
    runner_b = (
        cq.Workplane("XY")
        .box(0.016, 0.030, 0.170)
        .translate((TRAY_W / 2.0 + 0.004, -0.011, 0.0))
    )
    return outer.cut(inner).union(pull).union(runner_a).union(runner_b)


def _rocker_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(ROCKER_W, ROCKER_L, ROCKER_T)
        .translate((0.0, 0.0, ROCKER_T / 2.0))
        .edges("|Z")
        .fillet(0.0012)
    )
    axle = (
        cq.Workplane("YZ")
        .cylinder(0.028, 0.0018)
        .translate((0.0, 0.0, 0.0))
    )
    return cap.union(axle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_purifier")

    body_color = model.material("body_color", color=(0.85, 0.87, 0.88))
    trim_color = model.material("trim_color", color=(0.20, 0.22, 0.24))
    dark_color = model.material("dark_color", color=(0.14, 0.15, 0.16))
    filter_color = model.material("filter_color", color=(0.91, 0.93, 0.89))
    rubber_color = model.material("rubber_color", color=(0.10, 0.10, 0.11))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "purifier_housing"),
        material=body_color,
        name="housing_shell",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (INTAKE_W, INTAKE_H),
                0.0022,
                slot_size=(0.030, 0.0045),
                pitch=(0.038, 0.015),
                frame=0.010,
                corner_radius=0.006,
                stagger=True,
                center=True,
            ),
            "purifier_intake_grille",
        ),
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 - 0.0007, INTAKE_Z),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=trim_color,
        name="intake_grille",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (OUTLET_W, OUTLET_H),
                frame=0.008,
                face_thickness=0.003,
                duct_depth=0.014,
                duct_wall=0.002,
                slat_pitch=0.012,
                slat_width=0.006,
                slat_angle_deg=22.0,
                corner_radius=0.005,
                slats=VentGrilleSlats(profile="flat", direction="down", divider_count=1, divider_width=0.003),
                frame_profile=VentGrilleFrame(style="radiused", depth=0.001),
                sleeve=VentGrilleSleeve(style="short", depth=0.010, wall=0.002),
                center=False,
            ),
            "purifier_outlet_grille",
        ),
        origin=Origin(
            xyz=(OUTLET_X, OUTLET_Y, BODY_H + 0.0006),
            rpy=(3.141592653589793, 0.0, 0.0),
        ),
        material=trim_color,
        name="outlet_grille",
    )

    for foot_index, (sx, sy) in enumerate(((-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0))):
        body.visual(
            Box((0.022, 0.012, 0.004)),
            origin=Origin(
                xyz=(
                    sx * (BODY_W / 2.0 - 0.027),
                    sy * (BODY_D / 2.0 - 0.020),
                    0.0016,
                )
            ),
            material=rubber_color,
            name=f"foot_{foot_index}",
        )

    for guide_index, sign in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.012, 0.030, 0.176)),
            origin=Origin(
                xyz=(
                    sign * (BODY_W / 2.0 - WALL - 0.006),
                    BODY_D / 2.0 - WALL - 0.015,
                    0.136,
                )
            ),
            material=trim_color,
            name=f"tray_guide_{guide_index}",
        )

    for support_index, support_z in enumerate((PANEL_Z - 0.072, PANEL_Z + 0.072)):
        body.visual(
            Box((0.020, 0.005, 0.070)),
            origin=Origin(
                xyz=(-BODY_W / 2.0 + 0.007, BODY_D / 2.0 - 0.0022, support_z),
            ),
            material=trim_color,
            name=f"panel_hinge_support_{support_index}",
        )

    body.visual(
        Box((0.036, 0.022, 0.0025)),
        origin=Origin(xyz=(ROCKER_X, ROCKER_Y, BODY_H + 0.0013)),
        material=trim_color,
        name="rocker_bezel",
    )
    body.visual(
        Box((0.0035, 0.018, 0.010)),
        origin=Origin(xyz=(ROCKER_X - ROCKER_W / 2.0 - 0.0018, ROCKER_Y, BODY_H + 0.005)),
        material=trim_color,
        name="rocker_cheek_0",
    )
    body.visual(
        Box((0.0035, 0.018, 0.010)),
        origin=Origin(xyz=(ROCKER_X + ROCKER_W / 2.0 + 0.0018, ROCKER_Y, BODY_H + 0.005)),
        material=trim_color,
        name="rocker_cheek_1",
    )

    rear_panel = model.part("rear_panel")
    rear_panel.visual(
        mesh_from_cadquery(_rear_panel_shape(), "rear_panel_mesh"),
        material=body_color,
        name="panel_face",
    )

    filter_tray = model.part("filter_tray")
    tray_center_y = TRAY_REAR_OFFSET - TRAY_D / 2.0
    for side_index, side_sign in enumerate((-1.0, 1.0)):
        filter_tray.visual(
            Box((0.006, TRAY_D, TRAY_H)),
            origin=Origin(xyz=(side_sign * (TRAY_W / 2.0 - 0.003), tray_center_y, 0.0)),
            material=trim_color,
            name=f"tray_side_{side_index}",
        )
    filter_tray.visual(
        Box((TRAY_W, TRAY_D, 0.006)),
        origin=Origin(xyz=(0.0, tray_center_y, TRAY_H / 2.0 - 0.003)),
        material=trim_color,
        name="tray_top",
    )
    filter_tray.visual(
        Box((TRAY_W, TRAY_D, 0.006)),
        origin=Origin(xyz=(0.0, tray_center_y, -TRAY_H / 2.0 + 0.003)),
        material=trim_color,
        name="tray_bottom",
    )
    filter_tray.visual(
        Box((TRAY_W - 0.010, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, TRAY_REAR_OFFSET, -TRAY_H / 2.0 + 0.020)),
        material=trim_color,
        name="tray_back",
    )
    filter_tray.visual(
        Box((0.082, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, TRAY_REAR_OFFSET + 0.004, -TRAY_H / 2.0 + 0.020)),
        material=trim_color,
        name="tray_pull",
    )
    for runner_index, side_sign in enumerate((-1.0, 1.0)):
        filter_tray.visual(
            Box((0.016, 0.030, 0.170)),
            origin=Origin(xyz=(side_sign * (TRAY_W / 2.0 + 0.004), -0.011, 0.0)),
            material=trim_color,
            name=f"tray_runner_{runner_index}",
        )
    filter_tray.visual(
        Box((TRAY_W - 0.006, TRAY_D - 0.012, TRAY_H - 0.006)),
        origin=Origin(xyz=(0.0, tray_center_y, 0.0)),
        material=filter_color,
        name="filter_media",
    )

    rocker = model.part("rocker")
    rocker.visual(
        mesh_from_cadquery(_rocker_shape(), "rocker_mesh"),
        material=dark_color,
        name="rocker_cap",
    )

    rear_panel_joint = model.articulation(
        "body_to_rear_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_panel,
        origin=Origin(
            xyz=(-PANEL_W / 2.0, BODY_D / 2.0 + PANEL_GAP + PANEL_T / 2.0, PANEL_Z),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=1.95,
        ),
    )

    tray_joint = model.articulation(
        "body_to_filter_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_tray,
        origin=Origin(xyz=(0.0, TRAY_JOINT_Y, TRAY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.10,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    rocker_joint = model.articulation(
        "body_to_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(ROCKER_X, ROCKER_Y, ROCKER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=2.0,
            lower=-ROCKER_TRAVEL,
            upper=ROCKER_TRAVEL,
        ),
    )

    rear_panel_joint.meta["qc_samples"] = [0.0, PANEL_OPEN]
    tray_joint.meta["qc_samples"] = [0.0, TRAY_TRAVEL]
    rocker_joint.meta["qc_samples"] = [-ROCKER_TRAVEL, ROCKER_TRAVEL]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    rear_panel = object_model.get_part("rear_panel")
    filter_tray = object_model.get_part("filter_tray")
    rocker = object_model.get_part("rocker")

    rear_panel_joint = object_model.get_articulation("body_to_rear_panel")
    tray_joint = object_model.get_articulation("body_to_filter_tray")
    rocker_joint = object_model.get_articulation("body_to_rocker")

    ctx.expect_gap(
        rear_panel,
        body,
        axis="y",
        max_gap=0.0035,
        max_penetration=0.0,
        positive_elem="panel_face",
        negative_elem="housing_shell",
        name="rear panel closes flush to the purifier shell",
    )
    ctx.expect_overlap(
        rear_panel,
        body,
        axes="xz",
        min_overlap=0.135,
        elem_a="panel_face",
        elem_b="housing_shell",
        name="rear panel covers the rear access opening",
    )
    ctx.expect_gap(
        rear_panel,
        filter_tray,
        axis="y",
        min_gap=0.005,
        positive_elem="panel_face",
        name="closed rear panel clears the stored filter tray",
    )
    ctx.expect_within(
        filter_tray,
        body,
        axes="xz",
        margin=0.010,
        elem_b="housing_shell",
        name="filter tray stays registered inside the purifier opening",
    )
    ctx.expect_gap(
        rocker,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem="rocker_cap",
        negative_elem="housing_shell",
        name="rocker sits proud beside the outlet",
    )

    panel_rest_aabb = ctx.part_world_aabb(rear_panel)
    with ctx.pose({rear_panel_joint: PANEL_OPEN}):
        panel_open_aabb = ctx.part_world_aabb(rear_panel)
    ctx.check(
        "rear panel swings outward from the back",
        panel_rest_aabb is not None
        and panel_open_aabb is not None
        and float(panel_open_aabb[1][1]) > float(panel_rest_aabb[1][1]) + 0.060,
        details=f"rest={panel_rest_aabb}, open={panel_open_aabb}",
    )

    tray_rest_pos = ctx.part_world_position(filter_tray)
    with ctx.pose({rear_panel_joint: PANEL_OPEN, tray_joint: TRAY_TRAVEL}):
        ctx.expect_within(
            filter_tray,
            body,
            axes="xz",
            margin=0.010,
            elem_b="housing_shell",
            name="extended tray stays aligned on the internal guides",
        )
        ctx.expect_overlap(
            filter_tray,
            body,
            axes="y",
            min_overlap=0.024,
            elem_b="housing_shell",
            name="extended tray retains insertion in the purifier body",
        )
        tray_extended_pos = ctx.part_world_position(filter_tray)
    ctx.check(
        "filter tray slides rearward for service",
        tray_rest_pos is not None
        and tray_extended_pos is not None
        and float(tray_extended_pos[1]) > float(tray_rest_pos[1]) + 0.045,
        details=f"rest={tray_rest_pos}, extended={tray_extended_pos}",
    )

    rocker_rest_aabb = ctx.part_element_world_aabb(rocker, elem="rocker_cap")
    with ctx.pose({rocker_joint: ROCKER_TRAVEL}):
        rocker_tipped_aabb = ctx.part_element_world_aabb(rocker, elem="rocker_cap")
    ctx.check(
        "rocker pivots on its local hinge",
        rocker_rest_aabb is not None
        and rocker_tipped_aabb is not None
        and float(rocker_tipped_aabb[1][2]) > float(rocker_rest_aabb[1][2]) + 0.0012,
        details=f"rest={rocker_rest_aabb}, tipped={rocker_tipped_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
