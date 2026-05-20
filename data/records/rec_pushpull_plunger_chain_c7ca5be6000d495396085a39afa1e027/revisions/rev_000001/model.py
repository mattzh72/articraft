from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_L = 0.060
BODY_W = 0.026
BODY_H = 0.022

ROD_CENTER_Z = -0.004
ROD_W = 0.0076
ROD_H = 0.0064
ROD_SHANK_L = 0.045
ROD_TIP_L = 0.006
ROD_TIP_W = 0.0100
ROD_TIP_H = 0.0100
ROD_BUTTON_R = 0.0065
ROD_BUTTON_L = 0.0045
ROD_TRAVEL = 0.0045

TAB_T = 0.003
TAB_W = 0.018
TAB_H = 0.014
TAB_BARREL_R = 0.002
TAB_BARREL_W = 0.010
TAB_LUG_L = 0.004
TAB_LUG_W = 0.009
TAB_LUG_H = 0.004
TAB_OPEN_ANGLE = 0.75


def _guide_body_shape() -> cq.Workplane:
    top_bar = cq.Workplane("XY").box(0.058, 0.018, 0.004).translate((0.001, 0.0, 0.0085))
    left_front_post = cq.Workplane("XY").box(0.006, 0.003, 0.008).translate((0.026, -0.007, 0.0035))
    right_front_post = cq.Workplane("XY").box(0.006, 0.003, 0.008).translate((0.026, 0.007, 0.0035))
    left_ear = cq.Workplane("XY").box(0.004, 0.004, 0.006).translate((0.0305, -0.007, 0.006))
    right_ear = cq.Workplane("XY").box(0.004, 0.004, 0.006).translate((0.0305, 0.007, 0.006))
    left_rear_strut = cq.Workplane("XY").box(0.004, 0.003, 0.013).translate((-0.0285, -0.007, 0.001))
    right_rear_strut = cq.Workplane("XY").box(0.004, 0.003, 0.013).translate((-0.0285, 0.007, 0.001))
    rear_stop = cq.Workplane("XY").box(0.002, 0.014, 0.003).translate((-0.0315, 0.0, ROD_CENTER_Z))

    return (
        top_bar.union(left_front_post)
        .union(right_front_post)
        .union(left_ear)
        .union(right_ear)
        .union(left_rear_strut)
        .union(right_rear_strut)
        .union(rear_stop)
    )


def _rod_shape() -> cq.Workplane:
    shank = cq.Workplane("XY").box(ROD_SHANK_L, ROD_W, ROD_H, centered=(False, True, True))
    tip = (
        cq.Workplane("XY")
        .box(ROD_TIP_L, ROD_TIP_W, ROD_TIP_H, centered=(False, True, True))
        .translate((ROD_SHANK_L, 0.0, 0.0))
    )
    button = (
        cq.Workplane("YZ")
        .circle(ROD_BUTTON_R)
        .extrude(ROD_BUTTON_L)
        .translate((-ROD_BUTTON_L, 0.0, 0.0))
    )
    return shank.union(tip).union(button)


def _tab_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XZ")
        .circle(TAB_BARREL_R)
        .extrude(TAB_BARREL_W / 2, both=True)
    )
    plate = (
        cq.Workplane("XY")
        .box(TAB_T, TAB_W, TAB_H, centered=(False, True, False))
        .translate((TAB_BARREL_R, 0.0, -TAB_H))
    )
    finger_lip = (
        cq.Workplane("XY")
        .box(0.004, 0.012, 0.003, centered=(False, True, False))
        .translate((TAB_BARREL_R + TAB_T - 0.0005, 0.0, -TAB_H - 0.003))
    )
    return barrel.union(plate).union(finger_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plunger_lever_mechanism")

    body_mat = model.material("body_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    rod_mat = model.material("rod_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    tab_mat = model.material("tab_accent", rgba=(0.72, 0.36, 0.14, 1.0))

    guide_body = model.part("guide_body")
    guide_body.visual(
        mesh_from_cadquery(_guide_body_shape(), "guide_body"),
        material=body_mat,
        name="housing",
    )

    plunger_rod = model.part("plunger_rod")
    plunger_rod.visual(
        Box((ROD_SHANK_L, ROD_W, ROD_H)),
        origin=Origin(xyz=(ROD_SHANK_L / 2, 0.0, 0.0)),
        material=rod_mat,
        name="shank",
    )
    plunger_rod.visual(
        Box((ROD_TIP_L, ROD_TIP_W, ROD_TIP_H)),
        origin=Origin(xyz=(ROD_SHANK_L + ROD_TIP_L / 2, 0.0, 0.0)),
        material=rod_mat,
        name="tip",
    )
    plunger_rod.visual(
        Cylinder(radius=0.0055, length=ROD_BUTTON_L),
        origin=Origin(xyz=(-ROD_BUTTON_L / 2, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=rod_mat,
        name="button",
    )

    hinged_tab = model.part("hinged_tab")
    hinged_tab.visual(
        mesh_from_cadquery(_tab_shape(), "hinged_tab"),
        material=tab_mat,
        name="tab",
    )

    model.articulation(
        "body_to_rod",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=plunger_rod,
        origin=Origin(xyz=(-0.026, 0.0, ROD_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.08,
            lower=0.0,
            upper=ROD_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_tab",
        ArticulationType.REVOLUTE,
        parent=guide_body,
        child=hinged_tab,
        origin=Origin(xyz=(BODY_L / 2 + TAB_BARREL_R, 0.0, 0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=TAB_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_body = object_model.get_part("guide_body")
    plunger_rod = object_model.get_part("plunger_rod")
    hinged_tab = object_model.get_part("hinged_tab")
    rod_slide = object_model.get_articulation("body_to_rod")
    tab_hinge = object_model.get_articulation("body_to_tab")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({rod_slide: 0.0, tab_hinge: 0.0}):
        ctx.expect_contact(
            plunger_rod,
            guide_body,
            contact_tol=0.0003,
            name="rod_is_captured_by_rear_stop",
        )
        ctx.expect_contact(
            hinged_tab,
            guide_body,
            contact_tol=0.0003,
            name="tab_is_seated_on_front_face",
        )
        ctx.expect_overlap(
            plunger_rod,
            hinged_tab,
            axes="yz",
            min_overlap=0.007,
            name="rod_tip_is_aligned_with_tab",
        )
        ctx.expect_gap(
            hinged_tab,
            plunger_rod,
            axis="x",
            min_gap=0.0002,
            max_gap=0.006,
            name="closed_tip_sits_just_behind_tab",
        )

    with ctx.pose({rod_slide: ROD_TRAVEL, tab_hinge: 0.55}):
        ctx.expect_overlap(
            plunger_rod,
            hinged_tab,
            axes="yz",
            min_overlap=0.004,
            name="extended_tip_stays_in_line_with_tab",
        )
        ctx.expect_gap(
            hinged_tab,
            plunger_rod,
            axis="x",
            max_penetration=0.0005,
            max_gap=0.006,
            name="extended_tip_reaches_open_tab_without_deep_clash",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
