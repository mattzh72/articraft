from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_W = 0.62
BASE_D = 0.48
BASE_H = 0.095

BASE_SLEEVE_OUTER = 0.168
BASE_SLEEVE_INNER = 0.136
BASE_SLEEVE_H = 0.180
BASE_SLEEVE_TOP_Z = BASE_H + BASE_SLEEVE_H

STAGE1_OUTER = 0.128
STAGE1_INNER = 0.102
STAGE1_COLLAR = 0.154
STAGE1_Z_MIN = 0.0
STAGE1_Z_MAX = 0.325
STAGE1_TRAVEL = 0.260

STAGE2_OUTER = 0.092
STAGE2_INNER = 0.068
STAGE2_COLLAR = 0.116
STAGE2_Z_MIN = -0.120
STAGE2_Z_MAX = 0.300
STAGE2_CAP_T = 0.010
STAGE2_TRAVEL = 0.240

HEAD_DISK_D = 0.062
HEAD_DISK_T = 0.012
HEAD_BODY_X = 0.052
HEAD_BODY_Y = 0.036
HEAD_BODY_H = 0.034
HEAD_CAP_D = 0.016
HEAD_CAP_T = 0.008


def _box(centered_height: float, sx: float, sy: float, sz: float) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, False)).translate(
        (0.0, 0.0, centered_height)
    )


def _square_tube(
    *,
    outer: float,
    inner: float,
    z_min: float,
    z_max: float,
) -> cq.Workplane:
    outer_solid = _box(z_min, outer, outer, z_max - z_min)
    inner_void = _box(z_min - 0.002, inner, inner, (z_max - z_min) + 0.004)
    return outer_solid.cut(inner_void)


def _square_collar(
    *,
    outer: float,
    opening: float,
    z0: float,
    thickness: float,
) -> cq.Workplane:
    collar = _box(z0, outer, outer, thickness)
    opening_cut = _box(z0 - 0.002, opening, opening, thickness + 0.004)
    return collar.cut(opening_cut)


def _make_base_shape() -> cq.Workplane:
    body = _box(0.0, BASE_W, BASE_D, BASE_H)
    body = body.edges("|Z").fillet(0.022)
    body = body.edges(">Z").fillet(0.010)

    plinth_outer = _box(BASE_H, 0.250, 0.250, 0.035)
    plinth_cut = _box(BASE_H - 0.002, BASE_SLEEVE_INNER, BASE_SLEEVE_INNER, 0.039)
    plinth = plinth_outer.cut(plinth_cut)

    sleeve = _square_tube(
        outer=BASE_SLEEVE_OUTER,
        inner=BASE_SLEEVE_INNER,
        z_min=BASE_H,
        z_max=BASE_SLEEVE_TOP_Z,
    )

    top_lip = _square_collar(
        outer=BASE_SLEEVE_OUTER,
        opening=BASE_SLEEVE_INNER,
        z0=BASE_SLEEVE_TOP_Z - 0.012,
        thickness=0.012,
    )

    return body.union(plinth).union(sleeve).union(top_lip)


def _make_stage1_shape() -> cq.Workplane:
    tube = _square_tube(
        outer=STAGE1_OUTER,
        inner=STAGE1_INNER,
        z_min=STAGE1_Z_MIN,
        z_max=STAGE1_Z_MAX,
    )
    lower_collar = _square_tube(
        outer=STAGE1_COLLAR,
        inner=STAGE1_INNER,
        z_min=0.0,
        z_max=0.016,
    )
    top_band = _square_collar(
        outer=0.136,
        opening=STAGE2_OUTER + 0.004,
        z0=STAGE1_Z_MAX - 0.020,
        thickness=0.020,
    )
    return tube.union(lower_collar).union(top_band)


def _make_stage2_shape() -> cq.Workplane:
    tube = _square_tube(
        outer=STAGE2_OUTER,
        inner=STAGE2_INNER,
        z_min=STAGE2_Z_MIN,
        z_max=STAGE2_Z_MAX,
    )
    lower_collar = _square_tube(
        outer=STAGE2_COLLAR,
        inner=STAGE2_INNER,
        z_min=0.0,
        z_max=0.014,
    )
    top_cap = _box(STAGE2_Z_MAX, 0.082, 0.082, STAGE2_CAP_T)
    return tube.union(lower_collar).union(top_cap)


def _make_head_shape() -> cq.Workplane:
    turntable = (
        cq.Workplane("XY")
        .circle(HEAD_DISK_D * 0.5)
        .extrude(HEAD_DISK_T)
    )
    head_body = (
        _box(HEAD_DISK_T, HEAD_BODY_X, HEAD_BODY_Y, HEAD_BODY_H)
        .edges("|Z")
        .fillet(0.006)
    )
    top_cap = (
        cq.Workplane("XY")
        .circle(HEAD_CAP_D * 0.5)
        .extrude(HEAD_CAP_T)
        .translate((0.0, 0.0, HEAD_DISK_T + HEAD_BODY_H))
    )
    return turntable.union(head_body).union(top_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_telescoping_mast")

    base_color = model.material("base_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    mast_color = model.material("mast_anodized", rgba=(0.72, 0.74, 0.77, 1.0))
    head_color = model.material("head_paint", rgba=(0.86, 0.87, 0.89, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base"),
        origin=Origin(),
        material=base_color,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_SLEEVE_TOP_Z)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_SLEEVE_TOP_Z * 0.5)),
    )

    stage1 = model.part("stage1")
    stage1.visual(
        mesh_from_cadquery(_make_stage1_shape(), "stage1"),
        origin=Origin(),
        material=mast_color,
        name="stage1_shell",
    )
    stage1.inertial = Inertial.from_geometry(
        Box((STAGE1_COLLAR, STAGE1_COLLAR, STAGE1_Z_MAX - STAGE1_Z_MIN)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (STAGE1_Z_MIN + STAGE1_Z_MAX))),
    )

    stage2 = model.part("stage2")
    stage2.visual(
        mesh_from_cadquery(_make_stage2_shape(), "stage2"),
        origin=Origin(),
        material=mast_color,
        name="stage2_shell",
    )
    stage2.inertial = Inertial.from_geometry(
        Box((STAGE2_COLLAR, STAGE2_COLLAR, (STAGE2_Z_MAX + STAGE2_CAP_T) - STAGE2_Z_MIN)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (STAGE2_Z_MIN + STAGE2_Z_MAX + STAGE2_CAP_T))),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head_shape(), "head"),
        origin=Origin(),
        material=head_color,
        name="head_shell",
    )
    head.inertial = Inertial.from_geometry(
        Box((HEAD_DISK_D, HEAD_DISK_D, HEAD_DISK_T + HEAD_BODY_H + HEAD_CAP_T)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (HEAD_DISK_T + HEAD_BODY_H + HEAD_CAP_T))),
    )

    model.articulation(
        "base_to_stage1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, BASE_SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.18,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )

    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.0, 0.0, STAGE1_Z_MAX)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1600.0,
            velocity=0.18,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )

    model.articulation(
        "stage2_to_head",
        ArticulationType.REVOLUTE,
        parent=stage2,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, STAGE2_Z_MAX + STAGE2_CAP_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.6,
            lower=-3.14159,
            upper=3.14159,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    head = object_model.get_part("head")

    lift1 = object_model.get_articulation("base_to_stage1")
    lift2 = object_model.get_articulation("stage1_to_stage2")
    turn = object_model.get_articulation("stage2_to_head")

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

    ctx.check(
        "mast has two vertical prismatic stages",
        lift1.articulation_type == ArticulationType.PRISMATIC
        and lift2.articulation_type == ArticulationType.PRISMATIC
        and lift1.axis == (0.0, 0.0, 1.0)
        and lift2.axis == (0.0, 0.0, 1.0)
        and lift1.motion_limits is not None
        and lift2.motion_limits is not None
        and lift1.motion_limits.lower == 0.0
        and lift2.motion_limits.lower == 0.0
        and lift1.motion_limits.upper == STAGE1_TRAVEL
        and lift2.motion_limits.upper == STAGE2_TRAVEL,
        "Expected two upward-travel telescoping prismatic joints.",
    )
    ctx.check(
        "top head spins about vertical",
        turn.articulation_type == ArticulationType.REVOLUTE
        and turn.axis == (0.0, 0.0, 1.0)
        and turn.motion_limits is not None
        and turn.motion_limits.lower is not None
        and turn.motion_limits.upper is not None
        and turn.motion_limits.lower < 0.0
        and turn.motion_limits.upper > 0.0,
        "Expected a centered vertical revolute joint for the turntable head.",
    )

    with ctx.pose({lift1: 0.0, lift2: 0.0, turn: 0.0}):
        ctx.expect_contact(stage1, base, name="stage1 rests on base sleeve")
        ctx.expect_contact(stage2, stage1, name="stage2 rests on stage1 sleeve")
        ctx.expect_contact(head, stage2, name="head seats on stage2 cap")
        ctx.expect_within(stage2, stage1, axes="xy", margin=0.0, name="stage2 stays inside stage1 footprint")
        ctx.expect_within(head, stage2, axes="xy", margin=0.0, name="head stays smaller than top mast stage")

    with ctx.pose({lift1: 0.0, lift2: 0.0, turn: 0.0}):
        collapsed_head_pos = ctx.part_world_position(head)
    with ctx.pose({lift1: STAGE1_TRAVEL, lift2: STAGE2_TRAVEL, turn: 0.0}):
        extended_head_pos = ctx.part_world_position(head)
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=1.00,
            name="extended head stands well above grounded base",
        )
    ctx.check(
        "mast extends upward",
        collapsed_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[2] > collapsed_head_pos[2] + 0.45,
        "Head origin should rise substantially when both telescoping stages extend.",
    )

    with ctx.pose({turn: 0.0}):
        turntable_zero = ctx.part_world_position(head)
    with ctx.pose({turn: 1.2}):
        turntable_offset = ctx.part_world_position(head)
    ctx.check(
        "turntable rotates in place",
        turntable_zero is not None
        and turntable_offset is not None
        and abs(turntable_zero[0] - turntable_offset[0]) <= 1e-6
        and abs(turntable_zero[1] - turntable_offset[1]) <= 1e-6
        and abs(turntable_zero[2] - turntable_offset[2]) <= 1e-6,
        "The rotating head should spin about its own vertical axis without orbiting away.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
