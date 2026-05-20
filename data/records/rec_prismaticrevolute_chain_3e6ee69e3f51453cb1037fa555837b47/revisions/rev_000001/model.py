from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LEN = 0.230
BODY_W = 0.075
BASE_H = 0.018
RAIL_LEN = 0.182
RAIL_W = 0.012
RAIL_H = 0.016
RAIL_CENTER_Y = 0.022
CENTER_TROUGH_W = 0.026
CENTER_TROUGH_D = 0.006

CARRIAGE_LEN = 0.058
CARRIAGE_W = 0.078
CARRIAGE_H = 0.045
CARRIAGE_POCKET_LEN = 0.048
CARRIAGE_POCKET_W = 0.036
CARRIAGE_POCKET_D = 0.010
EAR_T = 0.008
EAR_W = 0.012
EAR_H = 0.012
EAR_CENTER_Y = 0.017
HINGE_Z = 0.033

TAB_T = 0.006
TAB_W = 0.020
TAB_H = 0.036
BARREL_R = 0.004
BARREL_LEN = 0.022

SLIDE_START_X = -0.045
SLIDE_TRAVEL = 0.090
TAB_OPEN_ANGLE = 1.10

MESH_TOL = 0.0004


def make_guide_body() -> cq.Workplane:
    base = cq.Workplane("XY").box(BODY_LEN, BODY_W, BASE_H).translate((0.0, 0.0, BASE_H / 2.0))
    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_LEN, RAIL_W, RAIL_H)
        .translate((0.0, RAIL_CENTER_Y, BASE_H + RAIL_H / 2.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_LEN, RAIL_W, RAIL_H)
        .translate((0.0, -RAIL_CENTER_Y, BASE_H + RAIL_H / 2.0))
    )
    center_trough = (
        cq.Workplane("XY")
        .box(RAIL_LEN * 0.92, CENTER_TROUGH_W, CENTER_TROUGH_D + 0.002)
        .translate((0.0, 0.0, BASE_H - CENTER_TROUGH_D / 2.0 + 0.001))
    )

    return base.cut(center_trough).union(left_rail).union(right_rail)


def make_carriage() -> cq.Workplane:
    front_face_x = CARRIAGE_LEN / 2.0
    hinge_axis_x = front_face_x + BARREL_R + 0.001
    stop_face_x = hinge_axis_x + BARREL_R
    hinge_pad_t = stop_face_x - front_face_x

    carriage = (
        cq.Workplane("XY")
        .box(CARRIAGE_LEN, CARRIAGE_W, CARRIAGE_H)
        .translate((0.0, 0.0, CARRIAGE_H / 2.0))
    )
    underside_pocket = (
        cq.Workplane("XY")
        .box(CARRIAGE_POCKET_LEN, CARRIAGE_POCKET_W, CARRIAGE_POCKET_D + 0.002)
        .translate((0.0, 0.0, CARRIAGE_POCKET_D / 2.0 - 0.001))
    )
    front_mount = (
        cq.Workplane("XY")
        .box(hinge_pad_t, 0.024, 0.016)
        .translate((front_face_x + hinge_pad_t / 2.0, 0.0, HINGE_Z - 0.004))
    )
    hinge_boss = (
        cq.Workplane("XZ")
        .circle(BARREL_R)
        .extrude(BARREL_LEN / 2.0, both=True)
        .translate((hinge_axis_x, 0.0, HINGE_Z))
    )

    return carriage.cut(underside_pocket).union(front_mount).union(hinge_boss)


def make_output_tab() -> cq.Workplane:
    hinge_bridge = (
        cq.Workplane("XY")
        .box(0.010, 0.016, 0.016)
        .translate((BARREL_R + 0.005, 0.0, -0.004))
    )
    plate = (
        cq.Workplane("XY")
        .box(TAB_T, TAB_W, TAB_H)
        .translate((0.017, 0.0, -0.026))
    )
    toe = (
        cq.Workplane("XY")
        .box(0.010, TAB_W * 0.75, 0.008)
        .translate((0.020, 0.0, -0.043))
    )
    return hinge_bridge.union(plate).union(toe)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_transfer_axis")

    body_mat = model.material("body_dark", rgba=(0.19, 0.21, 0.23, 1.0))
    carriage_mat = model.material("carriage_metal", rgba=(0.73, 0.75, 0.77, 1.0))
    tab_mat = model.material("tab_orange", rgba=(0.92, 0.56, 0.18, 1.0))

    guide_body = model.part("guide_body")
    guide_body.visual(
        mesh_from_cadquery(make_guide_body(), "guide_body_mesh", tolerance=MESH_TOL),
        material=body_mat,
        name="guide_body_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage(), "carriage_mesh", tolerance=MESH_TOL),
        material=carriage_mat,
        name="carriage_shell",
    )

    output_tab = model.part("output_tab")
    output_tab.visual(
        mesh_from_cadquery(make_output_tab(), "output_tab_mesh", tolerance=MESH_TOL),
        material=tab_mat,
        name="tab_shell",
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=carriage,
        origin=Origin(xyz=(SLIDE_START_X, 0.0, BASE_H + RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=output_tab,
        origin=Origin(xyz=(CARRIAGE_LEN / 2.0 + BARREL_R + 0.001, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=TAB_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_body = object_model.get_part("guide_body")
    carriage = object_model.get_part("carriage")
    output_tab = object_model.get_part("output_tab")
    slide = object_model.get_articulation("guide_to_carriage")
    tab_hinge = object_model.get_articulation("carriage_to_tab")

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

    ctx.expect_contact(
        carriage,
        guide_body,
        contact_tol=0.001,
        name="carriage_supported_by_twin_rails",
    )
    ctx.expect_contact(
        output_tab,
        carriage,
        contact_tol=0.001,
        name="tab_closed_against_carriage_face",
    )

    carriage_start = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        carriage_end = ctx.part_world_position(carriage)
    carriage_advances = (
        carriage_start is not None
        and carriage_end is not None
        and carriage_end[0] > carriage_start[0] + 0.080
        and abs(carriage_end[1] - carriage_start[1]) < 1e-4
        and abs(carriage_end[2] - carriage_start[2]) < 1e-4
    )
    ctx.check(
        "carriage_moves_along_prismatic_axis",
        carriage_advances,
        details=f"start={carriage_start}, end={carriage_end}",
    )

    tab_closed_aabb = ctx.part_element_world_aabb(output_tab, elem="tab_shell")
    with ctx.pose({tab_hinge: TAB_OPEN_ANGLE}):
        tab_open_aabb = ctx.part_element_world_aabb(output_tab, elem="tab_shell")
    tab_swings_forward = (
        tab_closed_aabb is not None
        and tab_open_aabb is not None
        and tab_open_aabb[1][0] > tab_closed_aabb[1][0] + 0.012
    )
    ctx.check(
        "output_tab_swings_forward",
        tab_swings_forward,
        details=f"closed_aabb={tab_closed_aabb}, open_aabb={tab_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
