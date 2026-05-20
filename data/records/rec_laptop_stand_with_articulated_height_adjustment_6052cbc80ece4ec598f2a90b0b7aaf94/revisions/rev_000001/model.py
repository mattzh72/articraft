from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_W = 0.30
BASE_D = 0.30
BASE_T = 0.008
GUIDE_Y = -0.075
GUIDE_W = 0.080
GUIDE_D = 0.036
GUIDE_H = 0.168
SLIDE_Y = GUIDE_Y + 0.006
SLIDE_Z0 = 0.056

SPINE_W = 0.036
SPINE_D = 0.016
SPINE_H = 0.310
SPINE_INSERT = 0.045

LOWER_PIVOT_X = 0.120
LOWER_PIVOT_Y = 0.010
LOWER_PIVOT_Z = 0.082
PIN_R = 0.0032
PIN_L = 0.010

BRACE_LENGTH = 0.145
BRACE_T = 0.008
BRACE_EYE_R = 0.009
BRACE_HOLE_R = 0.0039
BRACE_BAR_H = 0.012
BRACE_REST_ANGLE = 0.80

TRAY_W = 0.290
TRAY_D = 0.230
TRAY_PANEL_T = 0.006
TRAY_PIVOT_SPAN = LOWER_PIVOT_X * 2.0
TRAY_SIDE_MARGIN = (TRAY_W - TRAY_PIVOT_SPAN) / 2.0
TRAY_REAR_MARGIN = 0.022
TRAY_PANEL_BOTTOM = 0.012
TRAY_FRONT_LIP_H = 0.016
TRAY_PITCH = 0.24

POSE_Q = 0.18
POSE_SLIDE = 0.080


def _filleted_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _make_base_shape() -> cq.Workplane:
    plate = _filleted_box((BASE_W, BASE_D, BASE_T), 0.010).translate((0.0, 0.0, BASE_T / 2.0))

    back_plate = (
        cq.Workplane("XY")
        .box(GUIDE_W, 0.010, GUIDE_H, centered=(True, True, False))
        .translate((0.0, GUIDE_Y - GUIDE_D * 0.33, BASE_T))
    )
    cheek_x = GUIDE_W * 0.5 - 0.006
    cheek = cq.Workplane("XY").box(0.012, GUIDE_D, GUIDE_H, centered=(True, True, False))
    left_cheek = cheek.translate((-cheek_x, GUIDE_Y, BASE_T))
    right_cheek = cheek.translate((cheek_x, GUIDE_Y, BASE_T))

    gusset_profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.020, BASE_T),
                (-0.020, BASE_T + 0.004),
                (0.018, BASE_T + 0.052),
                (0.018, BASE_T),
            ]
        )
        .close()
        .extrude(0.012, both=True)
    )
    left_gusset = gusset_profile.translate((-0.024, GUIDE_Y + 0.010, 0.0))
    right_gusset = gusset_profile.translate((0.024, GUIDE_Y + 0.010, 0.0))

    foot = (
        cq.Workplane("XY")
        .box(0.100, 0.022, 0.006, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    left_foot = foot.translate((-0.070, 0.095, 0.0))
    right_foot = foot.translate((0.070, 0.095, 0.0))

    return (
        plate.union(back_plate)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_gusset)
        .union(right_gusset)
        .union(left_foot)
        .union(right_foot)
    )


def _make_spine_shape() -> cq.Workplane:
    mast = (
        cq.Workplane("XY")
        .box(SPINE_W, SPINE_D, SPINE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, -SPINE_INSERT))
    )

    crossbar = (
        cq.Workplane("XY")
        .box(TRAY_PIVOT_SPAN - 0.016, 0.014, 0.012)
        .edges("|X")
        .fillet(0.004)
        .translate((0.0, LOWER_PIVOT_Y, LOWER_PIVOT_Z))
    )

    pins = None
    for sign in (-1.0, 1.0):
        pin = (
            cq.Workplane("YZ")
            .circle(PIN_R)
            .extrude(PIN_L, both=True)
            .translate((sign * LOWER_PIVOT_X, LOWER_PIVOT_Y, LOWER_PIVOT_Z))
        )
        pins = pin if pins is None else pins.union(pin)

    saddle_post = (
        cq.Workplane("XY")
        .box(0.050, 0.032, 0.018, centered=(True, True, False))
        .translate((0.0, 0.016, 0.172))
    )
    saddle = (
        cq.Workplane("XY")
        .box(0.110, 0.060, 0.006, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.042, 0.188))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.048, 0.018, 0.012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, 0.220))
    )

    return mast.union(crossbar).union(pins).union(saddle_post).union(saddle).union(top_cap)


def _make_tray_shape() -> cq.Workplane:
    panel_center = (TRAY_PIVOT_SPAN * 0.5, TRAY_D * 0.5 - TRAY_REAR_MARGIN, TRAY_PANEL_BOTTOM + TRAY_PANEL_T * 0.5)
    panel = (
        cq.Workplane("XY")
        .box(TRAY_W, TRAY_D, TRAY_PANEL_T)
        .edges("|Z")
        .fillet(0.006)
        .translate(panel_center)
    )

    slot = (
        cq.Workplane("XY")
        .box(0.028, 0.140, TRAY_PANEL_T * 1.8)
        .translate((0.0, 0.030, TRAY_PANEL_BOTTOM + TRAY_PANEL_T * 0.5))
    )
    slots = (
        slot.translate((TRAY_PIVOT_SPAN * 0.5 - 0.040, 0.050, 0.0))
        .union(slot.translate((TRAY_PIVOT_SPAN * 0.5, 0.050, 0.0)))
        .union(slot.translate((TRAY_PIVOT_SPAN * 0.5 + 0.040, 0.050, 0.0)))
    )
    panel = panel.cut(slots)

    front_lip = (
        cq.Workplane("XY")
        .box(TRAY_W, 0.010, TRAY_FRONT_LIP_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate(
            (
                TRAY_PIVOT_SPAN * 0.5,
                TRAY_D - TRAY_REAR_MARGIN - 0.005,
                TRAY_PANEL_BOTTOM + TRAY_PANEL_T,
            )
        )
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(TRAY_W, 0.012, 0.008, centered=(True, True, False))
        .translate((TRAY_PIVOT_SPAN * 0.5, -TRAY_REAR_MARGIN + 0.006, TRAY_PANEL_BOTTOM))
    )
    center_rib = (
        cq.Workplane("XY")
        .box(0.070, 0.080, 0.010, centered=(True, True, False))
        .translate((TRAY_PIVOT_SPAN * 0.5, 0.010, 0.004))
    )

    lugs = None
    for x_pos in (0.0, TRAY_PIVOT_SPAN):
        lug = (
            cq.Workplane("XY")
            .box(0.016, 0.018, 0.015, centered=(True, True, False))
            .translate((x_pos, 0.0, -0.003))
        )
        pin = cq.Workplane("YZ").circle(PIN_R).extrude(PIN_L, both=True).translate((x_pos, 0.0, 0.0))
        lug_pair = lug.union(pin)
        lugs = lug_pair if lugs is None else lugs.union(lug_pair)

    return panel.union(front_lip).union(rear_rib).union(center_rib).union(lugs)


def _make_brace_shape(direction: float) -> cq.Workplane:
    far_y = direction * BRACE_LENGTH
    body_length = BRACE_LENGTH - BRACE_EYE_R * 2.0 + 0.008

    near_eye = cq.Workplane("YZ").circle(BRACE_EYE_R).extrude(BRACE_T, both=True)
    far_eye = cq.Workplane("YZ").circle(BRACE_EYE_R).extrude(BRACE_T, both=True).translate((0.0, far_y, 0.0))
    bridge = cq.Workplane("XY").box(BRACE_T, body_length, BRACE_BAR_H).translate((0.0, far_y * 0.5, 0.0))

    hole_near = cq.Workplane("YZ").circle(BRACE_HOLE_R).extrude(BRACE_T * 2.0, both=True)
    hole_far = (
        cq.Workplane("YZ")
        .circle(BRACE_HOLE_R)
        .extrude(BRACE_T * 2.0, both=True)
        .translate((0.0, far_y, 0.0))
    )

    return near_eye.union(far_eye).union(bridge).cut(hole_near).cut(hole_far)


def _aabb_yz_match(aabb_a, aabb_b, tol: float = 0.002) -> bool:
    if aabb_a is None or aabb_b is None:
        return False
    return all(
        isclose(aabb_a[idx][axis], aabb_b[idx][axis], abs_tol=tol)
        for idx in (0, 1)
        for axis in (1, 2)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laptop_stand")

    model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("silver", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("tray_dark", rgba=(0.30, 0.32, 0.34, 1.0))
    model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "base"), material="graphite", name="base_shell")

    spine = model.part("spine")
    spine.visual(mesh_from_cadquery(_make_spine_shape(), "spine"), material="silver", name="spine_shell")

    brace_0 = model.part("brace_0")
    brace_0.visual(mesh_from_cadquery(_make_brace_shape(1.0), "brace_0"), material="silver", name="brace_shell")

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_make_tray_shape(), "tray"), material="tray_dark", name="tray_shell")

    brace_1 = model.part("brace_1")
    brace_1.visual(mesh_from_cadquery(_make_brace_shape(-1.0), "brace_1"), material="silver", name="brace_shell")

    model.articulation(
        "base_to_spine",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spine,
        origin=Origin(xyz=(0.0, SLIDE_Y, SLIDE_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=POSE_SLIDE, effort=180.0, velocity=0.18),
    )
    model.articulation(
        "spine_to_brace_0",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=brace_0,
        origin=Origin(xyz=(-LOWER_PIVOT_X, LOWER_PIVOT_Y, LOWER_PIVOT_Z), rpy=(BRACE_REST_ANGLE, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.10, upper=0.32, effort=30.0, velocity=2.0),
    )
    model.articulation(
        "brace_0_to_tray",
        ArticulationType.REVOLUTE,
        parent=brace_0,
        child=tray,
        origin=Origin(xyz=(0.0, BRACE_LENGTH, 0.0), rpy=(TRAY_PITCH - BRACE_REST_ANGLE, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.32, upper=0.10, effort=30.0, velocity=2.0),
    )
    model.articulation(
        "tray_to_brace_1",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=brace_1,
        origin=Origin(xyz=(TRAY_PIVOT_SPAN, 0.0, 0.0), rpy=(BRACE_REST_ANGLE - TRAY_PITCH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.10, upper=0.32, effort=30.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    spine = object_model.get_part("spine")
    tray = object_model.get_part("tray")
    brace_0 = object_model.get_part("brace_0")
    brace_1 = object_model.get_part("brace_1")

    slide = object_model.get_articulation("base_to_spine")
    left_brace = object_model.get_articulation("spine_to_brace_0")
    tray_pitch = object_model.get_articulation("brace_0_to_tray")
    right_brace = object_model.get_articulation("tray_to_brace_1")

    ctx.allow_overlap(
        brace_0,
        spine,
        reason="The lower brace eye is intentionally modeled as a captured pivot around the spine-side support boss.",
    )
    ctx.allow_overlap(
        brace_0,
        tray,
        reason="The upper brace eye is intentionally represented as a compact nested hinge around the tray corner pivot hardware.",
    )
    ctx.allow_overlap(
        brace_1,
        spine,
        reason="The mirrored lower brace eye uses the same simplified captured pivot representation at the spine support.",
    )
    ctx.allow_overlap(
        brace_1,
        tray,
        reason="The mirrored upper brace eye uses the same simplified nested tray-corner hinge representation.",
    )
    ctx.allow_overlap(
        base,
        spine,
        reason="The inner spine is intentionally modeled with retained insertion inside the simplified rear guide that is integrated into the base part.",
    )

    ctx.expect_overlap(tray, base, axes="x", min_overlap=0.22, name="tray stays centered over the base width")

    rest_spine = ctx.part_world_position(spine)
    rest_tray = ctx.part_world_position(tray)
    rest_left = ctx.part_world_aabb(brace_0)
    rest_right = ctx.part_world_aabb(brace_1)

    ctx.check(
        "brace pair matches at rest",
        _aabb_yz_match(rest_left, rest_right),
        details=f"left={rest_left}, right={rest_right}",
    )

    with ctx.pose(
        {
            slide: POSE_SLIDE,
            left_brace: POSE_Q,
            tray_pitch: -POSE_Q,
            right_brace: POSE_Q,
        }
    ):
        raised_spine = ctx.part_world_position(spine)
        raised_tray = ctx.part_world_position(tray)
        raised_left = ctx.part_world_aabb(brace_0)
        raised_right = ctx.part_world_aabb(brace_1)

        ctx.check(
            "spine rises along the guide",
            rest_spine is not None
            and raised_spine is not None
            and raised_spine[2] > rest_spine[2] + 0.07,
            details=f"rest={rest_spine}, raised={raised_spine}",
        )
        ctx.check(
            "tray lifts with the spine",
            rest_tray is not None and raised_tray is not None and raised_tray[2] > rest_tray[2] + 0.08,
            details=f"rest={rest_tray}, raised={raised_tray}",
        )
        ctx.check(
            "brace pair stays aligned when raised",
            _aabb_yz_match(raised_left, raised_right),
            details=f"left={raised_left}, right={raised_right}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
