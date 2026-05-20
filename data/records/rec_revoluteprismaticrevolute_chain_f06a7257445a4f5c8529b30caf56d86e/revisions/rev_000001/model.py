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


BASE_LEN = 0.22
BASE_W = 0.12
BASE_T = 0.014
BASE_FORWARD = 0.17
PIVOT_R = 0.028
PIVOT_T = 0.004

BODY_LEN = 0.215
BODY_W = 0.050
BODY_H = 0.020
BODY_HEAD_R = 0.032
RAIL_LEN = 0.165
RAIL_W = 0.024
RAIL_H = 0.008
STAGE_ORIGIN_X = 0.100

STAGE_LEN = 0.062
STAGE_W = 0.048
STAGE_H = 0.018
WRIST_PAD_R = 0.019
WRIST_PAD_H = 0.004
WRIST_X = 0.027

FORK_HUB_R = 0.017
FORK_HUB_H = 0.012
FORK_BRIDGE_L = 0.020
FORK_TINE_L = 0.032
FORK_TINE_W = 0.008
FORK_GAP = 0.016
FORK_H = 0.010


def _make_base() -> cq.Workplane:
    plate_center_x = (BASE_FORWARD - (BASE_LEN - BASE_FORWARD)) / 2.0
    plate = (
        cq.Workplane("XY")
        .box(BASE_LEN, BASE_W, BASE_T)
        .edges("|Z")
        .fillet(0.010)
        .translate((plate_center_x, 0.0, -(PIVOT_T + BASE_T / 2.0)))
    )
    pivot = (
        cq.Workplane("XY")
        .circle(PIVOT_R)
        .extrude(PIVOT_T)
        .translate((0.0, 0.0, -PIVOT_T))
    )
    front_rib = (
        cq.Workplane("XY")
        .box(0.090, 0.034, PIVOT_T)
        .translate((0.055, 0.0, -PIVOT_T / 2.0))
    )
    return plate.union(pivot).union(front_rib)


def _make_body() -> cq.Workplane:
    head = cq.Workplane("XY").circle(BODY_HEAD_R).extrude(BODY_H)
    arm = (
        cq.Workplane("XY")
        .box(BODY_LEN, BODY_W, BODY_H)
        .translate((BODY_LEN / 2.0, 0.0, BODY_H / 2.0))
    )
    body = head.union(arm)

    side_pocket_len = RAIL_LEN - 0.015
    side_offset = (RAIL_W / 2.0) + 0.008
    left_pocket = (
        cq.Workplane("XY")
        .box(side_pocket_len, 0.012, 0.004)
        .translate((0.118, side_offset, BODY_H - 0.002))
    )
    right_pocket = (
        cq.Workplane("XY")
        .box(side_pocket_len, 0.012, 0.004)
        .translate((0.118, -side_offset, BODY_H - 0.002))
    )
    body = body.cut(left_pocket).cut(right_pocket)

    rail = (
        cq.Workplane("XY")
        .box(RAIL_LEN, RAIL_W, RAIL_H)
        .translate((0.123, 0.0, BODY_H + RAIL_H / 2.0))
    )
    nose_relief = (
        cq.Workplane("XY")
        .box(0.030, BODY_W * 0.70, BODY_H * 0.45)
        .translate((BODY_LEN - 0.005, 0.0, BODY_H * 0.225))
    )
    return body.union(rail).cut(nose_relief)


def _make_stage() -> cq.Workplane:
    carriage = (
        cq.Workplane("XY")
        .box(STAGE_LEN, STAGE_W, STAGE_H)
        .translate((0.0, 0.0, STAGE_H / 2.0))
    )
    top_relief = (
        cq.Workplane("XY")
        .box(STAGE_LEN * 0.45, STAGE_W * 0.48, 0.004)
        .translate((-0.010, 0.0, STAGE_H - 0.002))
    )
    wrist_pad = (
        cq.Workplane("XY")
        .circle(WRIST_PAD_R)
        .extrude(WRIST_PAD_H)
        .translate((WRIST_X, 0.0, STAGE_H))
    )
    front_cheek = (
        cq.Workplane("XY")
        .box(0.018, STAGE_W * 0.78, 0.006)
        .translate((0.018, 0.0, STAGE_H + 0.001))
    )
    return carriage.cut(top_relief).union(wrist_pad).union(front_cheek)


def _make_fork() -> cq.Workplane:
    tine_offset = (FORK_GAP / 2.0) + (FORK_TINE_W / 2.0)
    hub = cq.Workplane("XY").circle(FORK_HUB_R).extrude(FORK_HUB_H)
    bridge = (
        cq.Workplane("XY")
        .box(FORK_BRIDGE_L, FORK_GAP + (2.0 * FORK_TINE_W), FORK_H)
        .translate((FORK_BRIDGE_L / 2.0, 0.0, FORK_H / 2.0))
    )
    upper_tine = (
        cq.Workplane("XY")
        .box(FORK_TINE_L, FORK_TINE_W, FORK_H)
        .translate((FORK_BRIDGE_L + (FORK_TINE_L / 2.0), tine_offset, FORK_H / 2.0))
    )
    lower_tine = (
        cq.Workplane("XY")
        .box(FORK_TINE_L, FORK_TINE_W, FORK_H)
        .translate((FORK_BRIDGE_L + (FORK_TINE_L / 2.0), -tine_offset, FORK_H / 2.0))
    )
    return hub.union(bridge).union(upper_tine).union(lower_tine)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_rpr_assembly")

    dark_base = model.material("dark_base", rgba=(0.16, 0.18, 0.20, 1.0))
    body_gray = model.material("body_gray", rgba=(0.54, 0.57, 0.60, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.32, 0.35, 0.39, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base(), "base_mesh"),
        origin=Origin(),
        material=dark_base,
        name="base_shell",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body(), "body_mesh"),
        origin=Origin(),
        material=body_gray,
        name="body_shell",
    )

    stage = model.part("linear_stage")
    stage.visual(
        mesh_from_cadquery(_make_stage(), "stage_mesh"),
        origin=Origin(),
        material=carriage_gray,
        name="stage_shell",
    )

    fork = model.part("end_fork")
    fork.visual(
        mesh_from_cadquery(_make_fork(), "fork_mesh"),
        origin=Origin(),
        material=steel,
        name="fork_shell",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-1.2,
            upper=1.2,
        ),
    )

    model.articulation(
        "body_to_stage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stage,
        origin=Origin(xyz=(STAGE_ORIGIN_X, 0.0, BODY_H + RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.25,
            lower=0.0,
            upper=0.070,
        ),
    )

    model.articulation(
        "stage_to_fork",
        ArticulationType.REVOLUTE,
        parent=stage,
        child=fork,
        origin=Origin(xyz=(WRIST_X, 0.0, STAGE_H + WRIST_PAD_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-1.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    stage = object_model.get_part("linear_stage")
    fork = object_model.get_part("end_fork")

    base_to_body = object_model.get_articulation("base_to_body")
    body_to_stage = object_model.get_articulation("body_to_stage")
    stage_to_fork = object_model.get_articulation("stage_to_fork")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

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

    ctx.expect_gap(
        body,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="body seats on base pivot",
    )
    ctx.expect_overlap(
        body,
        base,
        axes="xy",
        min_overlap=0.045,
        name="body pivot footprint overlaps base pivot",
    )

    ctx.expect_gap(
        stage,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="stage sits on body rail",
    )
    ctx.expect_overlap(
        stage,
        body,
        axes="xy",
        min_overlap=0.024,
        name="stage carriage overlaps body guide footprint",
    )

    ctx.expect_gap(
        fork,
        stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="fork sits on stage wrist pad",
    )
    ctx.expect_overlap(
        fork,
        stage,
        axes="xy",
        min_overlap=0.020,
        name="fork hub overlaps stage wrist pad",
    )

    ctx.check(
        "articulation order is revolute-prismatic-revolute",
        base_to_body.joint_type == ArticulationType.REVOLUTE
        and body_to_stage.joint_type == ArticulationType.PRISMATIC
        and stage_to_fork.joint_type == ArticulationType.REVOLUTE,
        details=(
            f"got {base_to_body.joint_type}, {body_to_stage.joint_type}, "
            f"{stage_to_fork.joint_type}"
        ),
    )
    ctx.check(
        "joint axes match low-profile layout",
        tuple(base_to_body.axis) == (0.0, 0.0, 1.0)
        and tuple(body_to_stage.axis) == (1.0, 0.0, 0.0)
        and tuple(stage_to_fork.axis) == (0.0, 0.0, 1.0),
        details=(
            f"axes are {base_to_body.axis}, {body_to_stage.axis}, "
            f"{stage_to_fork.axis}"
        ),
    )

    with ctx.pose({base_to_body: 0.70, body_to_stage: 0.030}):
        swung_stage_pos = ctx.part_world_position(stage)
    ctx.check(
        "positive base rotation swings the stage toward +y",
        swung_stage_pos is not None and swung_stage_pos[1] > 0.070,
        details=f"stage position in swung pose was {swung_stage_pos}",
    )

    with ctx.pose({base_to_body: 0.0, body_to_stage: 0.0}):
        stage_retracted_pos = ctx.part_world_position(stage)
    with ctx.pose({base_to_body: 0.0, body_to_stage: 0.070}):
        stage_extended_pos = ctx.part_world_position(stage)
        ctx.expect_overlap(
            stage,
            body,
            axes="xy",
            min_overlap=0.020,
            name="extended stage remains over the body rail",
        )
    ctx.check(
        "positive prismatic travel extends the carriage forward",
        stage_retracted_pos is not None
        and stage_extended_pos is not None
        and stage_extended_pos[0] > stage_retracted_pos[0] + 0.060,
        details=(
            f"retracted={stage_retracted_pos}, extended={stage_extended_pos}"
        ),
    )

    with ctx.pose({base_to_body: 0.0, body_to_stage: 0.030, stage_to_fork: 0.0}):
        fork_center_closed = aabb_center(
            ctx.part_element_world_aabb(fork, elem="fork_shell")
        )
    with ctx.pose({base_to_body: 0.0, body_to_stage: 0.030, stage_to_fork: 0.60}):
        fork_center_open = aabb_center(
            ctx.part_element_world_aabb(fork, elem="fork_shell")
        )
    ctx.check(
        "positive wrist rotation sweeps the fork toward +y",
        fork_center_closed is not None
        and fork_center_open is not None
        and fork_center_open[1] > fork_center_closed[1] + 0.008,
        details=f"closed={fork_center_closed}, open={fork_center_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
