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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.032
BASE_L = 0.102
BASE_T = 0.008

BARREL_OD = 0.046
BARREL_ID = 0.038
BARREL_H = 0.214
BARREL_BASE_FLANGE_R = 0.031
BARREL_BASE_FLANGE_H = 0.012
GUIDE_CAP_T = 0.012
GUIDE_COLLAR_R = 0.015
GUIDE_COLLAR_H = 0.008

ROD_R = 0.0045
ROD_TOTAL = 0.240
ROD_BOTTOM = -0.160
ROD_TOP = ROD_BOTTOM + ROD_TOTAL
SLIDE_TRAVEL = 0.090

HANDLE_W = 0.134
HANDLE_D = 0.024
HANDLE_T = 0.020
HANDLE_BOSS_R = 0.011
HANDLE_BOSS_H = 0.028

FOOT_LEN = 0.078
FOOT_W = 0.020
FOOT_T = 0.006
HINGE_RADIUS = 0.0065
HINGE_SEG_H = 0.002
HINGE_AXIS_Z = FOOT_T / 2.0
HINGE_X = (BASE_W / 2.0) + 0.0055
FOOT_FOLD_LIMIT = 1.35


def _build_body_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(BASE_W, BASE_L, BASE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0025)
    )

    lower_flange = (
        cq.Workplane("XY")
        .circle(BARREL_BASE_FLANGE_R)
        .extrude(BARREL_BASE_FLANGE_H)
        .translate((0.0, 0.0, BASE_T))
    )
    barrel_outer = (
        cq.Workplane("XY")
        .circle(BARREL_OD / 2.0)
        .extrude(BARREL_H)
        .translate((0.0, 0.0, BASE_T))
    )
    barrel_inner = (
        cq.Workplane("XY")
        .circle(BARREL_ID / 2.0)
        .extrude(BARREL_H - 0.014)
        .translate((0.0, 0.0, BASE_T + 0.014))
    )
    guide_cap = (
        cq.Workplane("XY")
        .circle((BARREL_OD / 2.0) - 0.0015)
        .extrude(GUIDE_CAP_T)
        .translate((0.0, 0.0, BASE_T + BARREL_H - GUIDE_CAP_T))
    )
    guide_collar = (
        cq.Workplane("XY")
        .circle(GUIDE_COLLAR_R + 0.0015)
        .extrude(GUIDE_COLLAR_H)
        .translate((0.0, 0.0, BASE_T + BARREL_H - GUIDE_CAP_T))
    )
    rod_hole = (
        cq.Workplane("XY")
        .circle(ROD_R + 0.0005)
        .extrude(GUIDE_CAP_T + GUIDE_COLLAR_H + 0.002)
        .translate((0.0, 0.0, BASE_T + BARREL_H - GUIDE_CAP_T - 0.001))
    )

    body = base_plate.union(lower_flange).union(barrel_outer)
    body = body.cut(barrel_inner)
    body = body.union(guide_cap).union(guide_collar).cut(rod_hole)

    ear_y = 0.016
    ear_len = (HINGE_X - (BASE_W / 2.0)) + 0.003
    ear_x_center = (BASE_W / 2.0) + (ear_len / 2.0) - 0.001
    lower_z = 0.0
    upper_z = 2.0 * HINGE_SEG_H
    for sign in (-1.0, 1.0):
        lower_ear = (
            cq.Workplane("XY")
            .box(ear_len, ear_y, HINGE_SEG_H, centered=(True, True, False))
            .translate((sign * ear_x_center, 0.0, lower_z))
        )
        upper_ear = (
            cq.Workplane("XY")
            .box(ear_len, ear_y, HINGE_SEG_H, centered=(True, True, False))
            .translate((sign * ear_x_center, 0.0, upper_z))
        )
        lower_knuckle = (
            cq.Workplane("XY")
            .circle(HINGE_RADIUS)
            .extrude(HINGE_SEG_H)
            .translate((sign * HINGE_X, 0.0, lower_z))
        )
        upper_knuckle = (
            cq.Workplane("XY")
            .circle(HINGE_RADIUS)
            .extrude(HINGE_SEG_H)
            .translate((sign * HINGE_X, 0.0, upper_z))
        )
        body = body.union(lower_ear).union(upper_ear).union(lower_knuckle).union(upper_knuckle)

    return body


def _build_t_grip_shape() -> cq.Workplane:
    grip_bar = (
        cq.Workplane("XY")
        .box(HANDLE_W, HANDLE_D, HANDLE_T)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, 0.0, ROD_TOP + HANDLE_BOSS_H + (HANDLE_T / 2.0)))
    )
    center_boss = (
        cq.Workplane("XY")
        .circle(HANDLE_BOSS_R)
        .extrude(HANDLE_BOSS_H)
        .translate((0.0, 0.0, ROD_TOP))
    )
    return center_boss.union(grip_bar)


def _build_foot_shape(direction: float) -> cq.Workplane:
    outboard_len = FOOT_LEN - HINGE_RADIUS
    foot_center_x = direction * (HINGE_RADIUS + (outboard_len / 2.0))
    foot_pad = (
        cq.Workplane("XY")
        .box(outboard_len, FOOT_W, FOOT_T)
        .translate((foot_center_x, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.0025)
    )
    center_knuckle = (
        cq.Workplane("XY")
        .circle(HINGE_RADIUS)
        .extrude(HINGE_SEG_H)
        .translate((0.0, 0.0, -(HINGE_SEG_H / 2.0)))
    )
    return foot_pad.union(center_knuckle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_camping_pump")

    model.material("body_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("polymer_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("rod_silver", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("foot_black", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("pump_body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "pump_body_shell"),
        material="body_gray",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.090, 0.110, BASE_T + BARREL_H)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + BARREL_H) / 2.0)),
    )

    shaft_handle = model.part("shaft_handle")
    shaft_handle.visual(
        Cylinder(radius=ROD_R, length=ROD_TOTAL),
        origin=Origin(xyz=(0.0, 0.0, (ROD_TOP + ROD_BOTTOM) / 2.0)),
        material="rod_silver",
        name="pump_rod",
    )
    shaft_handle.visual(
        mesh_from_cadquery(_build_t_grip_shape(), "pump_t_grip"),
        material="polymer_black",
        name="t_grip",
    )
    shaft_handle.visual(
        Cylinder(radius=GUIDE_COLLAR_R * 0.95, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="rod_silver",
        name="rod_collar",
    )
    shaft_handle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=ROD_TOTAL + HANDLE_BOSS_H + HANDLE_T),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    left_foot = model.part("left_foot")
    left_foot.visual(
        mesh_from_cadquery(_build_foot_shape(-1.0), "left_stabilizer_foot"),
        material="foot_black",
        name="left_foot_visual",
    )
    left_foot.inertial = Inertial.from_geometry(
        Box((FOOT_LEN, FOOT_W, FOOT_T)),
        mass=0.05,
        origin=Origin(xyz=(-FOOT_LEN / 2.0, 0.0, 0.0)),
    )

    right_foot = model.part("right_foot")
    right_foot.visual(
        mesh_from_cadquery(_build_foot_shape(1.0), "right_stabilizer_foot"),
        material="foot_black",
        name="right_foot_visual",
    )
    right_foot.inertial = Inertial.from_geometry(
        Box((FOOT_LEN, FOOT_W, FOOT_T)),
        mass=0.05,
        origin=Origin(xyz=(FOOT_LEN / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "shaft_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=shaft_handle,
        origin=Origin(xyz=(0.0, 0.0, BASE_T + BARREL_H - (GUIDE_CAP_T / 2.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=45.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "left_foot_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_foot,
        origin=Origin(xyz=(-HINGE_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FOOT_FOLD_LIMIT,
            effort=8.0,
            velocity=3.0,
        ),
    )
    model.articulation(
        "right_foot_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_foot,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FOOT_FOLD_LIMIT,
            effort=8.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("pump_body")
    shaft_handle = object_model.get_part("shaft_handle")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")

    shaft_slide = object_model.get_articulation("shaft_slide")
    left_foot_fold = object_model.get_articulation("left_foot_fold")
    right_foot_fold = object_model.get_articulation("right_foot_fold")

    ctx.check("pump body exists", body is not None)
    ctx.check("shaft handle exists", shaft_handle is not None)
    ctx.check("left stabilizer foot exists", left_foot is not None)
    ctx.check("right stabilizer foot exists", right_foot is not None)

    ctx.expect_contact(
        left_foot,
        body,
        contact_tol=0.0005,
        name="left foot stays hinged to the base",
    )
    ctx.expect_contact(
        right_foot,
        body,
        contact_tol=0.0005,
        name="right foot stays hinged to the base",
    )

    ctx.expect_within(
        shaft_handle,
        body,
        axes="xy",
        inner_elem="pump_rod",
        outer_elem="body_shell",
        margin=0.002,
        name="rod stays centered inside the barrel footprint",
    )
    ctx.expect_overlap(
        shaft_handle,
        body,
        axes="z",
        elem_a="pump_rod",
        elem_b="body_shell",
        min_overlap=0.16,
        name="collapsed rod remains substantially inserted in the barrel",
    )

    body_aabb = ctx.part_world_aabb(body)
    left_open_aabb = ctx.part_world_aabb(left_foot)
    right_open_aabb = ctx.part_world_aabb(right_foot)
    ctx.check(
        "open feet widen the stance",
        body_aabb is not None
        and left_open_aabb is not None
        and right_open_aabb is not None
        and left_open_aabb[0][0] < body_aabb[0][0] - 0.035
        and right_open_aabb[1][0] > body_aabb[1][0] + 0.035,
        details=f"body={body_aabb}, left={left_open_aabb}, right={right_open_aabb}",
    )

    rest_handle_pos = ctx.part_world_position(shaft_handle)
    with ctx.pose({shaft_slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            shaft_handle,
            body,
            axes="xy",
            inner_elem="pump_rod",
            outer_elem="body_shell",
            margin=0.002,
            name="extended rod stays centered in the barrel footprint",
        )
        ctx.expect_overlap(
            shaft_handle,
            body,
            axes="z",
            elem_a="pump_rod",
            elem_b="body_shell",
            min_overlap=0.07,
            name="extended rod still retains meaningful insertion",
        )
        extended_handle_pos = ctx.part_world_position(shaft_handle)
    ctx.check(
        "handle slides upward along the barrel axis",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.08,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    with ctx.pose({left_foot_fold: FOOT_FOLD_LIMIT, right_foot_fold: FOOT_FOLD_LIMIT}):
        left_folded_aabb = ctx.part_world_aabb(left_foot)
        right_folded_aabb = ctx.part_world_aabb(right_foot)
    ctx.check(
        "feet rotate rearward toward the stored position",
        left_open_aabb is not None
        and right_open_aabb is not None
        and left_folded_aabb is not None
        and right_folded_aabb is not None
        and left_folded_aabb[0][0] > left_open_aabb[0][0] + 0.025
        and right_folded_aabb[1][0] < right_open_aabb[1][0] - 0.025
        and left_folded_aabb[0][1] < left_open_aabb[0][1] - 0.010
        and right_folded_aabb[0][1] < right_open_aabb[0][1] - 0.010,
        details=(
            f"left_open={left_open_aabb}, left_folded={left_folded_aabb}, "
            f"right_open={right_open_aabb}, right_folded={right_folded_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
