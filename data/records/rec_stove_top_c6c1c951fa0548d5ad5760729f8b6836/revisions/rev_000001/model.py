from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.56
BODY_DEPTH = 0.34
BODY_HEIGHT = 0.10
DECK_THICKNESS = 0.008
DECK_TOP = BODY_HEIGHT + DECK_THICKNESS


def _add_burner(body, *, idx: int, x: float, y: float, metal: str, grate: str) -> None:
    bowl_height = 0.016
    grate_height = 0.008
    cap_height = 0.012

    body.visual(
        Cylinder(radius=0.046, length=bowl_height),
        origin=Origin(xyz=(x, y, DECK_TOP + bowl_height / 2.0)),
        material=metal,
        name=f"burner_bowl_{idx}",
    )
    body.visual(
        Cylinder(radius=0.058, length=grate_height),
        origin=Origin(xyz=(x, y, DECK_TOP + 0.012)),
        material=grate,
        name=f"burner_ring_{idx}",
    )
    body.visual(
        Cylinder(radius=0.018, length=cap_height),
        origin=Origin(xyz=(x, y, DECK_TOP + 0.012)),
        material=grate,
        name=f"burner_cap_{idx}",
    )
    body.visual(
        Box((0.118, 0.014, 0.008)),
        origin=Origin(xyz=(x, y, DECK_TOP + 0.016)),
        material=grate,
        name=f"grate_bar_x_{idx}",
    )
    body.visual(
        Box((0.014, 0.118, 0.008)),
        origin=Origin(xyz=(x, y, DECK_TOP + 0.016)),
        material=grate,
        name=f"grate_bar_y_{idx}",
    )


def _add_knob(model: ArticulatedObject, body, *, idx: int, x: float, z: float, metal: str, knob_mat: str):
    knob = model.part(f"knob_{idx}")

    knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.021, length=0.026),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="knob_cap",
    )
    knob.visual(
        Box((0.006, 0.006, 0.015)),
        origin=Origin(xyz=(0.0, -0.037, 0.009)),
        material=metal,
        name="indicator",
    )

    model.articulation(
        f"knob_{idx}_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(x, -0.175, z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0),
    )

    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_gas_stove")

    enamel = model.material("enamel", rgba=(0.20, 0.21, 0.22, 1.0))
    cover_enamel = model.material("cover_enamel", rgba=(0.26, 0.27, 0.28, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.72, 0.70, 1.0))
    grate = model.material("grate", rgba=(0.13, 0.13, 0.13, 1.0))
    knob_mat = model.material("knob", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material=enamel,
        name="housing",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + DECK_THICKNESS / 2.0)),
        material=metal,
        name="deck",
    )
    body.visual(
        Box((0.52, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.161, 0.052)),
        material=cover_enamel,
        name="fascia",
    )
    body.visual(
        Box((0.074, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, -0.161, 0.089)),
        material=cover_enamel,
        name="latch_mount",
    )

    for foot_x in (-0.22, 0.22):
        for foot_y in (-0.12, 0.12):
            body.visual(
                Cylinder(radius=0.012, length=0.010),
                origin=Origin(xyz=(foot_x, foot_y, 0.005)),
                material=rubber,
                name=f"foot_{int((foot_x > 0) * 2 + (foot_y > 0))}",
            )

    _add_burner(body, idx=0, x=-0.125, y=0.035, metal=metal, grate=grate)
    _add_burner(body, idx=1, x=0.125, y=0.035, metal=metal, grate=grate)

    cover = model.part("cover")
    cover.visual(
        Box((0.556, 0.332, 0.012)),
        origin=Origin(xyz=(0.0, -0.166, 0.034)),
        material=cover_enamel,
        name="cover_panel",
    )
    cover.visual(
        Box((0.556, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, -0.326, 0.020)),
        material=cover_enamel,
        name="front_lip",
    )
    cover.visual(
        Box((0.012, 0.308, 0.040)),
        origin=Origin(xyz=(-0.272, -0.154, 0.020)),
        material=cover_enamel,
        name="side_flange_0",
    )
    cover.visual(
        Box((0.012, 0.308, 0.040)),
        origin=Origin(xyz=(0.272, -0.154, 0.020)),
        material=cover_enamel,
        name="side_flange_1",
    )
    cover.visual(
        Box((0.556, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.005, 0.014)),
        material=cover_enamel,
        name="rear_skirt",
    )
    cover.visual(
        Box((0.090, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.303, 0.041)),
        material=metal,
        name="cover_grip",
    )
    cover.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(-0.180, -0.004, 0.012), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="hinge_barrel_0",
    )
    cover.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.180, -0.004, 0.012), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="hinge_barrel_1",
    )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, 0.166, DECK_TOP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.55),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="pivot",
    )
    latch.visual(
        Box((0.050, 0.014, 0.044)),
        origin=Origin(xyz=(0.0, -0.009, -0.022)),
        material=knob_mat,
        name="lever_tab",
    )
    latch.visual(
        Box((0.050, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.016, -0.043)),
        material=metal,
        name="lever_tip",
    )

    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(0.0, -0.176, 0.089)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.0),
    )

    _add_knob(model, body, idx=0, x=-0.125, z=0.048, metal=metal, knob_mat=knob_mat)
    _add_knob(model, body, idx=1, x=0.125, z=0.048, metal=metal, knob_mat=knob_mat)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    latch = object_model.get_part("latch")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")

    cover_hinge = object_model.get_articulation("cover_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")
    knob_0_turn = object_model.get_articulation("knob_0_turn")
    knob_1_turn = object_model.get_articulation("knob_1_turn")

    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="front_lip",
        negative_elem="deck",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="cover lip rests on the deck rim when closed",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_panel",
        elem_b="deck",
        min_overlap=0.28,
        name="closed cover spans the cooking surface",
    )

    closed_cover_front = ctx.part_element_world_aabb(cover, elem="front_lip")
    with ctx.pose({cover_hinge: 1.45}):
        open_cover_front = ctx.part_element_world_aabb(cover, elem="front_lip")
        ctx.check(
            "cover opens upward",
            closed_cover_front is not None
            and open_cover_front is not None
            and open_cover_front[0][2] > closed_cover_front[1][2] + 0.12,
            details=f"closed={closed_cover_front}, open={open_cover_front}",
        )

    ctx.expect_gap(
        body,
        latch,
        axis="y",
        positive_elem="latch_mount",
        negative_elem="pivot",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="latch pivot seats on the front mount",
    )
    closed_latch_tip = ctx.part_element_world_aabb(latch, elem="lever_tip")
    with ctx.pose({latch_pivot: 0.9}):
        open_latch_tip = ctx.part_element_world_aabb(latch, elem="lever_tip")
        ctx.check(
            "latch swings outward from the front edge",
            closed_latch_tip is not None
            and open_latch_tip is not None
            and open_latch_tip[0][1] < closed_latch_tip[0][1] - 0.015,
            details=f"closed={closed_latch_tip}, open={open_latch_tip}",
        )

    ctx.expect_gap(
        body,
        knob_0,
        axis="y",
        positive_elem="fascia",
        negative_elem="shaft",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="left knob shaft meets the fascia",
    )
    ctx.expect_gap(
        body,
        knob_1,
        axis="y",
        positive_elem="fascia",
        negative_elem="shaft",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="right knob shaft meets the fascia",
    )
    ctx.expect_origin_distance(
        knob_0,
        knob_1,
        axes="x",
        min_dist=0.22,
        max_dist=0.28,
        name="knobs are spaced across the front panel",
    )

    for joint in (knob_0_turn, knob_1_turn):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
