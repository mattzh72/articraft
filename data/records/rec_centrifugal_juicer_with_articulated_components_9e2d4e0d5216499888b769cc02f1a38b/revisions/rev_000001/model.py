from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_WIDTH = 0.28
BODY_DEPTH = 0.20
BODY_HEIGHT = 0.145

COLLECTOR_CENTER_X = 0.01
COLLECTOR_OUTER_RADIUS = 0.108
COLLECTOR_INNER_RADIUS = 0.073
COLLECTOR_BOTTOM_Z = 0.140
COLLECTOR_HEIGHT = 0.038
COLLECTOR_TOP_Z = COLLECTOR_BOTTOM_Z + COLLECTOR_HEIGHT

HINGE_X = -0.10
HINGE_Z = COLLECTOR_TOP_Z

CHUTE_X = 0.145
CHUTE_Y = 0.043
CHUTE_OUTER_X = 0.076
CHUTE_OUTER_Y = 0.056
CHUTE_INNER_X = 0.050
CHUTE_INNER_Y = 0.034
CHUTE_HEIGHT = 0.124
CHUTE_BOTTOM_Z = 0.048
CHUTE_TOP_Z = CHUTE_BOTTOM_Z + CHUTE_HEIGHT

LID_OPEN_ANGLE = math.radians(68.0)
PUSHER_TRAVEL = 0.115
FLAP_CLOSED_ANGLE = -1.15
LEVER_SWING = 0.45


def build_body_shell() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.024)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
    )

    hinge_tower = (
        cq.Workplane("XY")
        .box(0.052, 0.122, 0.026)
        .translate((HINGE_X - 0.020, 0.0, BODY_HEIGHT + 0.013))
    )

    bowl_well = (
        cq.Workplane("XY")
        .circle(0.079)
        .extrude(0.090)
        .translate((COLLECTOR_CENTER_X, 0.0, 0.086))
    )

    return housing.union(hinge_tower).cut(bowl_well)


def build_collector_rim() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(COLLECTOR_OUTER_RADIUS)
        .circle(COLLECTOR_INNER_RADIUS)
        .extrude(COLLECTOR_HEIGHT)
        .translate((COLLECTOR_CENTER_X, 0.0, COLLECTOR_BOTTOM_Z))
    )


def build_spout() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .center(0.122, 0.152)
        .rect(0.058, 0.024)
        .workplane(offset=-0.048)
        .center(0.004, -0.006)
        .rect(0.036, 0.015)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XZ")
        .center(0.122, 0.152)
        .rect(0.042, 0.013)
        .workplane(offset=-0.050)
        .center(0.004, -0.006)
        .rect(0.023, 0.007)
        .loft(combine=True)
    )
    return outer.cut(inner)


def build_lid_cover() -> cq.Workplane:
    cover = (
        cq.Workplane("XY")
        .box(0.215, 0.185, 0.072)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.109, 0.0, 0.036))
    )
    cover = cover.faces("<Z").shell(-0.0035)
    opening = cq.Workplane("XY").box(0.054, 0.038, 0.090).translate((CHUTE_X, CHUTE_Y, 0.045))
    return cover.cut(opening)


def build_chute_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CHUTE_OUTER_X, CHUTE_OUTER_Y, CHUTE_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
        .translate((CHUTE_X, CHUTE_Y, CHUTE_BOTTOM_Z + CHUTE_HEIGHT / 2.0))
    )
    inner = cq.Workplane("XY").box(CHUTE_INNER_X, CHUTE_INNER_Y, 0.170).translate(
        (CHUTE_X, CHUTE_Y, CHUTE_BOTTOM_Z + CHUTE_HEIGHT / 2.0)
    )
    return outer.cut(inner)


def build_basket() -> LatheGeometry:
    outer_profile = [
        (0.0, 0.0),
        (0.060, 0.0),
        (0.060, 0.012),
        (0.068, 0.012),
        (0.068, 0.053),
    ]
    inner_profile = [
        (0.0, 0.006),
        (0.052, 0.006),
        (0.052, 0.049),
    ]
    return LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=56)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_centrifugal_juicer")

    body_finish = model.material("body_finish", rgba=(0.83, 0.83, 0.82, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    steel_finish = model.material("steel_finish", rgba=(0.74, 0.76, 0.78, 1.0))
    clear_lid = model.material("clear_lid", rgba=(0.80, 0.88, 0.92, 0.38))
    pusher_finish = model.material("pusher_finish", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shell(), "body_shell"),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(build_collector_rim(), "collector_rim"),
        material=body_finish,
        name="collector_rim",
    )
    body.visual(
        mesh_from_cadquery(build_spout(), "juice_spout"),
        material=body_finish,
        name="juice_spout",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.092, 0.109, 0.104), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="lever_boss",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(COLLECTOR_CENTER_X, 0.0, 0.085), rpy=(0.0, 0.0, 0.0)),
        material=trim_finish,
        name="motor_hub",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(build_lid_cover(), "lid_cover"),
        material=clear_lid,
        name="lid_cover",
    )
    lid.visual(
        mesh_from_cadquery(build_chute_shell(), "chute_shell"),
        material=clear_lid,
        name="chute_shell",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_geometry(build_basket(), "basket_shell"),
        material=steel_finish,
        name="basket_shell",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.044, 0.030, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=pusher_finish,
        name="pusher_shaft",
    )
    pusher.visual(
        Box((0.060, 0.042, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=pusher_finish,
        name="pusher_cap",
    )

    flap = model.part("chute_flap")
    flap.visual(
        Cylinder(radius=0.0024, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="flap_barrel",
    )
    flap.visual(
        Box((0.050, 0.038, 0.003)),
        origin=Origin(xyz=(0.025, 0.0, -0.0015)),
        material=trim_finish,
        name="flap_panel",
    )

    lever = model.part("mode_lever")
    lever.visual(
        Cylinder(radius=0.0055, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_finish,
        name="lever_pivot",
    )
    lever.visual(
        Box((0.012, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, 0.008, 0.012)),
        material=trim_finish,
        name="lever_stem",
    )
    lever.visual(
        Box((0.016, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, 0.016, 0.040)),
        material=trim_finish,
        name="lever_paddle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=LID_OPEN_ANGLE,
        ),
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(COLLECTOR_CENTER_X, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=35.0),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_X, CHUTE_Y, CHUTE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.22,
            lower=0.0,
            upper=PUSHER_TRAVEL,
        ),
    )

    model.articulation(
        "lid_to_chute_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(
            xyz=(CHUTE_X - CHUTE_OUTER_X / 2.0, CHUTE_Y, CHUTE_TOP_Z),
            rpy=(0.0, -1.45, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=FLAP_CLOSED_ANGLE,
            upper=0.25,
        ),
    )

    model.articulation(
        "body_to_mode_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.092, 0.113, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-LEVER_SWING,
            upper=LEVER_SWING,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    pusher = object_model.get_part("pusher")
    flap = object_model.get_part("chute_flap")
    lever = object_model.get_part("mode_lever")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    flap_hinge = object_model.get_articulation("lid_to_chute_flap")
    lever_joint = object_model.get_articulation("body_to_mode_lever")

    ctx.allow_overlap(
        body,
        lever,
        elem_a="lever_boss",
        elem_b="lever_pivot",
        reason="The side mode lever rotates on a short pivot barrel nested into the body-side boss.",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_cover",
        negative_elem="collector_rim",
        max_gap=0.006,
        max_penetration=0.0,
        name="lid seats on collector rim",
    )
    ctx.expect_within(
        basket,
        body,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="collector_rim",
        margin=0.006,
        name="basket stays centered inside collector rim",
    )
    ctx.expect_gap(
        lid,
        basket,
        axis="z",
        positive_elem="lid_cover",
        negative_elem="basket_shell",
        min_gap=0.002,
        max_gap=0.020,
        name="lid clears the basket at rest",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="chute_shell",
        margin=0.006,
        name="pusher shaft stays aligned in the chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_shaft",
        elem_b="chute_shell",
        min_overlap=0.085,
        name="pusher remains inserted into the chute at rest",
    )

    with ctx.pose({pusher_slide: PUSHER_TRAVEL}):
        ctx.expect_gap(
            pusher,
            lid,
            axis="z",
            positive_elem="pusher_shaft",
            negative_elem="chute_shell",
            min_gap=0.008,
            name="pusher lifts clear of the chute opening",
        )

    with ctx.pose({pusher_slide: PUSHER_TRAVEL, flap_hinge: FLAP_CLOSED_ANGLE}):
        ctx.expect_overlap(
            flap,
            lid,
            axes="xy",
            elem_a="flap_panel",
            elem_b="chute_shell",
            min_overlap=0.030,
            name="flap spans the chute opening when the pusher is raised",
        )
        ctx.expect_gap(
            flap,
            lid,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="chute_shell",
            max_gap=0.004,
            max_penetration=0.004,
            name="flap lands near the chute top",
        )

    closed_lid = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: LID_OPEN_ANGLE}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.050,
        details=f"closed={closed_lid!r}, open={open_lid!r}",
    )

    with ctx.pose({lever_joint: -LEVER_SWING}):
        lever_low = ctx.part_world_aabb(lever)
    with ctx.pose({lever_joint: LEVER_SWING}):
        lever_high = ctx.part_world_aabb(lever)
    ctx.check(
        "mode lever sweeps through a visible range",
        lever_low is not None
        and lever_high is not None
        and lever_high[1][2] > lever_low[1][2] + 0.012,
        details=f"low={lever_low!r}, high={lever_high!r}",
    )

    return ctx.report()


object_model = build_object_model()
