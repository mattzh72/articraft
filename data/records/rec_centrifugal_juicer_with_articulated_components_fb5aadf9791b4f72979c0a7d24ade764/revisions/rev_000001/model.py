from __future__ import annotations

import math

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
    mesh_from_geometry,
)

BODY_L = 0.24
BODY_W = 0.18
BODY_H = 0.11
SHOULDER_L = 0.19
SHOULDER_W = 0.15
SHOULDER_H = 0.032
COLLAR_R = 0.090
COLLAR_H = 0.024

LID_HINGE_X = -0.088
LID_HINGE_Z = 0.162
LID_CENTER_X = 0.096
LID_DOME_H = 0.076
CHUTE_BASE_Z = 0.068
CHUTE_H = 0.080
CHUTE_OUTER_R = 0.040
CHUTE_INNER_R = 0.032

BASKET_Z = 0.178
PUSHER_TRAVEL = 0.055


def _build_lid_dome() -> object:
    outer_profile = [
        (0.099, 0.000),
        (0.097, 0.010),
        (0.091, 0.026),
        (0.078, 0.046),
        (0.058, 0.064),
        (0.040, LID_DOME_H),
    ]
    inner_profile = [
        (0.090, 0.004),
        (0.088, 0.012),
        (0.083, 0.026),
        (0.072, 0.044),
        (0.053, 0.059),
        (0.032, 0.072),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
    )


def _build_chute_shell() -> object:
    outer_profile = [
        (CHUTE_OUTER_R, 0.000),
        (CHUTE_OUTER_R, CHUTE_H - 0.004),
        (0.044, CHUTE_H),
    ]
    inner_profile = [
        (CHUTE_INNER_R, 0.000),
        (CHUTE_INNER_R, CHUTE_H - 0.004),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=48,
    )


def _build_ring(outer_radius: float, inner_radius: float, height: float) -> object:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, height)],
        [(inner_radius, 0.0), (inner_radius, height)],
        segments=48,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_kitchen_juicer")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.93, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.66, 0.68, 0.70, 1.0))
    smoky_clear = model.material("smoky_clear", rgba=(0.72, 0.84, 0.90, 0.32))
    basket_steel = model.material("basket_steel", rgba=(0.82, 0.84, 0.87, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.21, 0.23, 1.0))

    base = model.part("base")
    base.visual(
        Box((BODY_L, BODY_W, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
        material=body_white,
        name="base_shell",
    )
    base.visual(
        Box((SHOULDER_L, SHOULDER_W, SHOULDER_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=body_white,
        name="upper_shoulder",
    )
    base.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=trim_gray,
        name="collar",
    )
    base.visual(
        mesh_from_geometry(_build_ring(0.096, 0.060, 0.004), "juicer_lid_seat"),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=trim_gray,
        name="lid_seat",
    )
    base.visual(
        Box((0.024, 0.048, 0.030)),
        origin=Origin(xyz=(0.110, 0.0, 0.100)),
        material=body_white,
        name="spout_root",
    )
    base.visual(
        Box((0.036, 0.034, 0.020)),
        origin=Origin(xyz=(0.137, 0.0, 0.098)),
        material=body_white,
        name="spout_tip",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_dome(), "juicer_lid_dome"),
        origin=Origin(xyz=(LID_CENTER_X, 0.0, 0.0)),
        material=smoky_clear,
        name="dome_shell",
    )
    lid.visual(
        mesh_from_geometry(_build_chute_shell(), "juicer_chute_shell"),
        origin=Origin(xyz=(LID_CENTER_X, 0.0, CHUTE_BASE_Z)),
        material=smoky_clear,
        name="chute_shell",
    )
    lid.visual(
        mesh_from_geometry(_build_ring(0.046, 0.034, 0.004), "juicer_chute_lip"),
        origin=Origin(xyz=(LID_CENTER_X, 0.0, CHUTE_BASE_Z + CHUTE_H - 0.004)),
        material=smoky_clear,
        name="chute_lip",
    )

    basket = model.part("basket")
    basket.visual(
        Cylinder(radius=0.054, length=0.032),
        material=basket_steel,
        name="basket_wall",
    )
    basket.visual(
        Cylinder(radius=0.058, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=basket_steel,
        name="cutter_rim",
    )
    basket.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=trim_gray,
        name="hub",
    )
    basket.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=trim_gray,
        name="feed_hub",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.029, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=dark_gray,
        name="pusher_shaft",
    )
    pusher.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=charcoal,
        name="pusher_flange",
    )
    pusher.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=charcoal,
        name="pusher_handle",
    )
    pusher.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=charcoal,
        name="pusher_grip",
    )

    switch = model.part("switch")
    switch.visual(
        Box((0.004, 0.036, 0.018)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=charcoal,
        name="switch_stub",
    )
    switch.visual(
        Box((0.020, 0.046, 0.026)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=charcoal,
        name="switch_rocker",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.004, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="cap_barrel",
    )
    cap.visual(
        Box((0.020, 0.032, 0.003)),
        origin=Origin(xyz=(0.010, 0.0, -0.003)),
        material=charcoal,
        name="cap_leaf",
    )
    cap.visual(
        Box((0.006, 0.024, 0.005)),
        origin=Origin(xyz=(0.018, 0.0, -0.005)),
        material=charcoal,
        name="cap_tip",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, LID_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, BASKET_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(LID_CENTER_X, 0.0, CHUTE_BASE_Z + CHUTE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.18,
            lower=0.0,
            upper=PUSHER_TRAVEL,
        ),
    )
    model.articulation(
        "base_to_switch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=switch,
        origin=Origin(xyz=(0.124, 0.0, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-0.28,
            upper=0.28,
        ),
    )
    model.articulation(
        "base_to_cap",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cap,
        origin=Origin(xyz=(0.155, 0.0, 0.106)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    pusher = object_model.get_part("pusher")
    switch = object_model.get_part("switch")
    cap = object_model.get_part("cap")

    lid_hinge = object_model.get_articulation("base_to_lid")
    basket_spin = object_model.get_articulation("base_to_basket")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    switch_pivot = object_model.get_articulation("base_to_switch")
    cap_hinge = object_model.get_articulation("base_to_cap")

    ctx.check(
        "critical joints use the intended motion types",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and pusher_slide.articulation_type == ArticulationType.PRISMATIC
        and switch_pivot.articulation_type == ArticulationType.REVOLUTE
        and cap_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"lid={lid_hinge.articulation_type}, basket={basket_spin.articulation_type}, "
            f"pusher={pusher_slide.articulation_type}, switch={switch_pivot.articulation_type}, "
            f"cap={cap_hinge.articulation_type}"
        ),
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="dome_shell",
            negative_elem="lid_seat",
            max_gap=0.006,
            max_penetration=0.0,
            name="lid dome seats on the drive collar",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="dome_shell",
            elem_b="lid_seat",
            min_overlap=0.16,
            name="lid dome covers the juicing collar",
        )

    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.0}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="chute_shell",
            margin=0.003,
            name="pusher stays centered in the chute at rest",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="xy",
            elem_a="pusher_flange",
            elem_b="chute_lip",
            min_overlap=0.070,
            name="pusher flange stays centered over the chute lip at rest",
        )
        ctx.expect_gap(
            pusher,
            lid,
            axis="z",
            positive_elem="pusher_flange",
            negative_elem="chute_lip",
            max_gap=0.001,
            max_penetration=0.0,
            name="pusher flange seats on the chute lip at rest",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="chute_shell",
            min_overlap=0.055,
            name="pusher remains deeply inserted at rest",
        )
        pusher_rest = ctx.part_world_position(pusher)

    with ctx.pose({lid_hinge: 0.0, pusher_slide: PUSHER_TRAVEL}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="chute_shell",
            margin=0.003,
            name="pusher stays centered in the chute when raised",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="chute_shell",
            min_overlap=0.012,
            name="pusher keeps retained insertion at full lift",
        )
        pusher_extended = ctx.part_world_position(pusher)

    ctx.check(
        "pusher lifts upward along the chute axis",
        pusher_rest is not None
        and pusher_extended is not None
        and pusher_extended[2] > pusher_rest[2] + 0.040,
        details=f"rest={pusher_rest}, extended={pusher_extended}",
    )

    with ctx.pose({basket_spin: 0.0}):
        ctx.expect_overlap(
            basket,
            base,
            axes="xy",
            elem_a="basket_wall",
            elem_b="collar",
            min_overlap=0.10,
            name="basket stays centered over the drive collar",
        )
        ctx.expect_gap(
            basket,
            base,
            axis="z",
            positive_elem="basket_wall",
            negative_elem="collar",
            max_gap=0.010,
            max_penetration=1e-6,
            name="basket sits just above the collar",
        )

    with ctx.pose({switch_pivot: 0.0}):
        ctx.expect_overlap(
            switch,
            base,
            axes="yz",
            elem_a="switch_rocker",
            elem_b="base_shell",
            min_overlap=0.025,
            name="switch sits on the lower front face",
        )
        ctx.expect_gap(
            switch,
            base,
            axis="x",
            positive_elem="switch_rocker",
            negative_elem="base_shell",
            min_gap=0.0,
            max_gap=0.012,
            name="switch remains only slightly proud of the front face",
        )

    with ctx.pose({cap_hinge: 0.0}):
        ctx.expect_overlap(
            cap,
            base,
            axes="y",
            elem_a="cap_leaf",
            elem_b="spout_tip",
            min_overlap=0.020,
            name="cap covers the spout tip when closed",
        )
        ctx.expect_gap(
            cap,
            base,
            axis="x",
            positive_elem="cap_leaf",
            negative_elem="spout_tip",
            min_gap=0.0,
            max_gap=0.006,
            name="cap sits at the end of the spout",
        )
        cap_closed_aabb = ctx.part_world_aabb(cap)

    with ctx.pose({cap_hinge: 1.20}):
        cap_open_aabb = ctx.part_world_aabb(cap)

    ctx.check(
        "spout cap swings upward",
        cap_closed_aabb is not None
        and cap_open_aabb is not None
        and cap_open_aabb[1][2] > cap_closed_aabb[1][2] + 0.012,
        details=f"closed={cap_closed_aabb}, open={cap_open_aabb}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        lid_closed_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 1.10}):
        lid_open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward on the rear hinge",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.030,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
