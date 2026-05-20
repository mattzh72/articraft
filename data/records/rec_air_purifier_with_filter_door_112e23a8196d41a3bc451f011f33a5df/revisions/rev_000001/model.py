from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_W = 0.220
BODY_D = 0.125
BODY_H = 0.390

WALL_T = 0.011
FRONT_WALL_T = 0.018
BASE_T = 0.020

PANEL_W = 0.198
PANEL_H = 0.308
PANEL_T = 0.006
PANEL_BOTTOM_Z = 0.036
PANEL_HINGE_Y = -(BODY_D / 2.0) - (PANEL_T / 2.0)
PANEL_HINGE_X = PANEL_W / 2.0

FILTER_W = 0.182
FILTER_D = 0.074
FILTER_H = 0.286
FILTER_BOTTOM_Z = 0.048
FILTER_REAR_Y = -(BODY_D / 2.0) + 0.006
FILTER_TRAVEL = 0.042

TRAY_W = 0.078
TRAY_D = 0.054
TRAY_H = 0.012
TRAY_HINGE_Y = -0.006
OUTLET_W = 0.118
OUTLET_D = 0.020
OUTLET_Y = 0.028


def _box_from_ranges(
    sx: float,
    sy: float,
    sz: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    z0: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz, centered=(True, True, False))
        .translate((cx, cy, z0))
    )


def _build_rear_panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_T, PANEL_H, centered=(False, True, False))
        .translate((-PANEL_W, 0.0, 0.0))
    )

    slot_w = PANEL_W - 0.050
    slot_h = 0.010
    for idx in range(7):
        z = 0.052 + (idx * 0.033)
        slot = (
            cq.Workplane("XY")
            .box(slot_w, 0.012, slot_h, centered=(True, True, False))
            .translate((-(PANEL_W * 0.54), 0.0, z))
        )
        panel = panel.cut(slot)

    latch_finger = (
        cq.Workplane("XZ")
        .center(-PANEL_W + 0.011, PANEL_H * 0.56)
        .circle(0.008)
        .extrude(0.010, both=True)
    )
    panel = panel.cut(latch_finger)

    hinge_spine = (
        cq.Workplane("XY")
        .circle(0.003)
        .extrude(PANEL_H)
        .translate((0.0, 0.0, 0.0))
    )
    panel = panel.union(hinge_spine)

    return panel


def _build_filter_shape() -> cq.Workplane:
    filter_body = _box_from_ranges(
        FILTER_W,
        FILTER_D,
        FILTER_H,
        z0=0.0,
        cy=FILTER_D / 2.0,
    )

    media_recess = _box_from_ranges(
        FILTER_W - 0.020,
        FILTER_D - 0.010,
        FILTER_H - 0.020,
        z0=0.006,
        cy=(FILTER_D - 0.010) / 2.0,
    )
    filter_body = filter_body.cut(media_recess)

    rib_depth = FILTER_D - 0.014
    for idx in range(12):
        x = -0.066 + (idx * 0.012)
        rib = _box_from_ranges(
            0.005,
            rib_depth,
            FILTER_H - 0.032,
            cx=x,
            cy=0.004 + (rib_depth / 2.0),
            z0=0.016,
        )
        filter_body = filter_body.union(rib)

    top_grip = _box_from_ranges(
        0.040,
        0.010,
        0.010,
        cy=0.005,
        z0=FILTER_H - 0.002,
    )
    filter_body = filter_body.union(top_grip)

    return filter_body


def _build_tray_shape() -> cq.Workplane:
    tray = cq.Workplane("XY").box(TRAY_W, TRAY_D, TRAY_H, centered=(True, False, False))

    pocket = _box_from_ranges(
        TRAY_W - 0.012,
        TRAY_D - 0.010,
        TRAY_H - 0.003,
        cy=(TRAY_D - 0.010) / 2.0 + 0.002,
        z0=0.003,
    )
    tray = tray.cut(pocket)

    front_lip = _box_from_ranges(
        0.030,
        0.004,
        0.004,
        cy=TRAY_D - 0.002,
        z0=TRAY_H - 0.004,
    )
    tray = tray.union(front_lip)

    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.95, 0.96, 0.94, 1.0))
    intake_gray = model.material("intake_gray", rgba=(0.83, 0.84, 0.80, 1.0))
    filter_charcoal = model.material("filter_charcoal", rgba=(0.24, 0.26, 0.28, 1.0))
    tray_dark = model.material("tray_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, FRONT_WALL_T, BODY_H)),
        origin=Origin(xyz=(0.0, (BODY_D / 2.0) - (FRONT_WALL_T / 2.0), BODY_H / 2.0)),
        material=shell_white,
        name="front_shell",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=((BODY_W / 2.0) - (WALL_T / 2.0), 0.0, BODY_H / 2.0)),
        material=shell_white,
        name="side_shell_0",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(-(BODY_W / 2.0) + (WALL_T / 2.0), 0.0, BODY_H / 2.0)),
        material=shell_white,
        name="side_shell_1",
    )
    body.visual(
        Box((BODY_W, BODY_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=shell_white,
        name="bottom_shell",
    )
    body.visual(
        Box((BODY_W, 0.066, 0.014)),
        origin=Origin(xyz=(0.0, -0.0295, BODY_H - 0.007)),
        material=shell_white,
        name="top_rear_deck",
    )
    body.visual(
        Box(((BODY_W - OUTLET_W) / 2.0, 0.046, 0.014)),
        origin=Origin(xyz=(0.0845, 0.027, BODY_H - 0.007)),
        material=shell_white,
        name="top_shoulder_0",
    )
    body.visual(
        Box(((BODY_W - OUTLET_W) / 2.0, 0.046, 0.014)),
        origin=Origin(xyz=(-0.0845, 0.027, BODY_H - 0.007)),
        material=shell_white,
        name="top_shoulder_1",
    )
    body.visual(
        Box((BODY_W, 0.013, 0.014)),
        origin=Origin(xyz=(0.0, 0.056, BODY_H - 0.007)),
        material=shell_white,
        name="top_front_lip",
    )
    body.visual(
        Box((PANEL_W, 0.006, PANEL_BOTTOM_Z)),
        origin=Origin(xyz=(0.0, -(BODY_D / 2.0) + 0.003, PANEL_BOTTOM_Z / 2.0)),
        material=shell_white,
        name="rear_frame_bottom",
    )
    body.visual(
        Box((PANEL_W, 0.006, BODY_H - (PANEL_BOTTOM_Z + PANEL_H))),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_D / 2.0) + 0.003,
                PANEL_BOTTOM_Z + PANEL_H + ((BODY_H - (PANEL_BOTTOM_Z + PANEL_H)) / 2.0),
            )
        ),
        material=shell_white,
        name="rear_frame_top",
    )
    body.visual(
        Box(((BODY_W - PANEL_W) / 2.0, 0.006, PANEL_H)),
        origin=Origin(
            xyz=(
                PANEL_HINGE_X + ((BODY_W - PANEL_W) / 4.0),
                -(BODY_D / 2.0) + 0.003,
                PANEL_BOTTOM_Z + (PANEL_H / 2.0),
            )
        ),
        material=shell_white,
        name="rear_frame_hinge",
    )
    body.visual(
        Box(((BODY_W - PANEL_W) / 2.0, 0.006, PANEL_H)),
        origin=Origin(
            xyz=(
                -PANEL_HINGE_X - ((BODY_W - PANEL_W) / 4.0),
                -(BODY_D / 2.0) + 0.003,
                PANEL_BOTTOM_Z + (PANEL_H / 2.0),
            )
        ),
        material=shell_white,
        name="rear_frame_latch",
    )
    body.visual(
        Box((0.008, 0.038, 0.004)),
        origin=Origin(xyz=(0.095, -0.036, 0.046)),
        material=shell_white,
        name="guide_0",
    )
    body.visual(
        Box((0.008, 0.038, 0.004)),
        origin=Origin(xyz=(-0.095, -0.036, 0.046)),
        material=shell_white,
        name="guide_1",
    )
    body.visual(
        Box((0.012, 0.010, PANEL_H)),
        origin=Origin(xyz=(0.104, -0.0575, PANEL_BOTTOM_Z + (PANEL_H / 2.0))),
        material=shell_white,
        name="hinge_boss",
    )
    body.visual(
        Box((0.010, 0.008, 0.055)),
        origin=Origin(xyz=(-0.102, -0.0585, PANEL_BOTTOM_Z + 0.1375)),
        material=shell_white,
        name="latch_boss",
    )

    rear_panel = model.part("rear_panel")
    rear_panel.visual(
        mesh_from_cadquery(_build_rear_panel_shape(), "rear_panel"),
        material=intake_gray,
        name="rear_cover",
    )

    cassette_filter = model.part("cassette_filter")
    cassette_filter.visual(
        mesh_from_cadquery(_build_filter_shape(), "cassette_filter"),
        material=filter_charcoal,
        name="cassette",
    )

    fragrance_tray = model.part("fragrance_tray")
    fragrance_tray.visual(
        mesh_from_cadquery(_build_tray_shape(), "fragrance_tray"),
        material=tray_dark,
        name="tray",
    )

    model.articulation(
        "rear_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_panel,
        origin=Origin(xyz=(PANEL_HINGE_X, PANEL_HINGE_Y, PANEL_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )

    model.articulation(
        "filter_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cassette_filter,
        origin=Origin(xyz=(0.0, FILTER_REAR_Y, FILTER_BOTTOM_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    model.articulation(
        "tray_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=fragrance_tray,
        origin=Origin(xyz=(0.0, TRAY_HINGE_Y, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    rear_panel = object_model.get_part("rear_panel")
    cassette_filter = object_model.get_part("cassette_filter")
    fragrance_tray = object_model.get_part("fragrance_tray")

    rear_panel_hinge = object_model.get_articulation("rear_panel_hinge")
    filter_slide = object_model.get_articulation("filter_slide")
    tray_hinge = object_model.get_articulation("tray_hinge")

    with ctx.pose({rear_panel_hinge: 0.0, filter_slide: 0.0, tray_hinge: 0.0}):
        ctx.expect_gap(
            body,
            rear_panel,
            axis="y",
            negative_elem="rear_cover",
            min_gap=0.0,
            max_gap=0.004,
            name="rear panel seats against the rear frame",
        )
        ctx.expect_overlap(
            body,
            rear_panel,
            axes="xz",
            elem_b="rear_cover",
            min_overlap=0.18,
            name="rear panel covers the service opening",
        )
        ctx.expect_within(
            cassette_filter,
            body,
            axes="xz",
            inner_elem="cassette",
            margin=0.012,
            name="cassette stays centered inside the body opening",
        )
        ctx.expect_overlap(
            cassette_filter,
            body,
            axes="y",
            elem_a="cassette",
            min_overlap=0.04,
            name="cassette remains inserted at rest",
        )
        ctx.expect_gap(
            fragrance_tray,
            body,
            axis="z",
            positive_elem="tray",
            min_gap=0.0,
            max_gap=0.004,
            name="fragrance tray rests on the top deck",
        )
        ctx.expect_overlap(
            fragrance_tray,
            body,
            axes="xy",
            elem_a="tray",
            min_overlap=0.05,
            name="fragrance tray sits over the outlet area",
        )

    panel_closed_aabb = ctx.part_world_aabb(rear_panel)
    with ctx.pose({rear_panel_hinge: math.radians(90.0)}):
        panel_open_aabb = ctx.part_world_aabb(rear_panel)
    ctx.check(
        "rear panel swings outward from the back",
        panel_closed_aabb is not None
        and panel_open_aabb is not None
        and panel_open_aabb[0][1] < panel_closed_aabb[0][1] - 0.05,
        details=f"closed={panel_closed_aabb}, open={panel_open_aabb}",
    )

    filter_rest_pos = ctx.part_world_position(cassette_filter)
    with ctx.pose({rear_panel_hinge: math.radians(105.0), filter_slide: FILTER_TRAVEL}):
        filter_extended_pos = ctx.part_world_position(cassette_filter)
        ctx.expect_within(
            cassette_filter,
            body,
            axes="xz",
            inner_elem="cassette",
            margin=0.012,
            name="extended cassette stays aligned with the guide path",
        )
        ctx.expect_overlap(
            cassette_filter,
            body,
            axes="y",
            elem_a="cassette",
            min_overlap=0.02,
            name="extended cassette retains insertion on the guides",
        )
    ctx.check(
        "cassette slides rearward when pulled",
        filter_rest_pos is not None
        and filter_extended_pos is not None
        and filter_extended_pos[1] < filter_rest_pos[1] - 0.03,
        details=f"rest={filter_rest_pos}, extended={filter_extended_pos}",
    )

    tray_closed_aabb = ctx.part_world_aabb(fragrance_tray)
    with ctx.pose({tray_hinge: math.radians(85.0)}):
        tray_open_aabb = ctx.part_world_aabb(fragrance_tray)
    ctx.check(
        "fragrance tray flips upward above the outlet",
        tray_closed_aabb is not None
        and tray_open_aabb is not None
        and tray_open_aabb[1][2] > tray_closed_aabb[1][2] + 0.03,
        details=f"closed={tray_closed_aabb}, open={tray_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
