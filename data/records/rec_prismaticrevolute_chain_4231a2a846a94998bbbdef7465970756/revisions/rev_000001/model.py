from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


FRAME_LEN = 0.42
FRAME_W = 0.16
BASE_T = 0.012
GUIDE_LEN = 0.34
GUIDE_W = 0.052
GUIDE_H = 0.026

CARRIAGE_LEN = 0.16
CARRIAGE_W = 0.105
CARRIAGE_H = 0.056
CARRIAGE_SLOT_LEN = 0.20
CARRIAGE_SLOT_W = GUIDE_W + 0.010
CARRIAGE_SLOT_H = 0.036

HINGE_X = (CARRIAGE_LEN / 2.0) + 0.021
HINGE_Z = 0.022
EAR_LEN = 0.016
EAR_W = 0.016
EAR_H = 0.024
EAR_GAP = 0.040

TAB_BARREL_R = 0.009
TAB_BARREL_LEN = 0.040
TAB_NECK_LEN = 0.010
TAB_NECK_W = 0.032
TAB_NECK_H = 0.016
TAB_PLATE_LEN = 0.055
TAB_PLATE_W = 0.070
TAB_PLATE_T = 0.008
TAB_LIP_LEN = 0.010
TAB_LIP_H = 0.012

SLIDE_RETRACT_X = -0.08
SLIDE_Z = 0.042
SLIDE_TRAVEL = 0.18
TAB_OPEN_LIMIT = 1.10


def _frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(FRAME_LEN, FRAME_W, BASE_T).translate((0.0, 0.0, BASE_T / 2.0))
    guide = cq.Workplane("XY").box(GUIDE_LEN, GUIDE_W, GUIDE_H).translate(
        (0.0, 0.0, BASE_T + (GUIDE_H / 2.0))
    )
    rear_stop = cq.Workplane("XY").box(0.034, 0.112, 0.042).translate((-0.180, 0.0, 0.021))
    left_rib = cq.Workplane("XY").box(0.290, 0.014, 0.020).translate((0.0, 0.064, 0.022))
    right_rib = cq.Workplane("XY").box(0.290, 0.014, 0.020).translate((0.0, -0.064, 0.022))

    shape = base.union(guide).union(rear_stop).union(left_rib).union(right_rib)

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.155, -0.055),
                (-0.155, 0.055),
                (0.155, -0.055),
                (0.155, 0.055),
            ]
        )
        .circle(0.0055)
        .extrude(0.060)
        .translate((0.0, 0.0, -0.001))
    )
    return shape.cut(mount_holes)


def _carriage_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(CARRIAGE_LEN, CARRIAGE_W, CARRIAGE_H)
    channel = outer.cut(
        cq.Workplane("XY").box(CARRIAGE_SLOT_LEN, CARRIAGE_SLOT_W, CARRIAGE_SLOT_H).translate(
            (0.0, 0.0, (-CARRIAGE_H / 2.0) + (CARRIAGE_SLOT_H / 2.0) - 0.001)
        )
    )

    ear_y = (EAR_GAP / 2.0) + (EAR_W / 2.0)
    ear_x = (CARRIAGE_LEN / 2.0) + (EAR_LEN / 2.0) - 0.004
    left_ear = cq.Workplane("XY").box(EAR_LEN, EAR_W, EAR_H).translate((ear_x, ear_y, HINGE_Z))
    right_ear = cq.Workplane("XY").box(EAR_LEN, EAR_W, EAR_H).translate((ear_x, -ear_y, HINGE_Z))
    fork_notch = cq.Workplane("XY").box(0.012, EAR_GAP, 0.022).translate(
        ((CARRIAGE_LEN / 2.0) - 0.002, 0.0, HINGE_Z)
    )
    runner = cq.Workplane("XY").box(0.130, GUIDE_W - 0.004, 0.012).translate((0.0, 0.0, 0.002))

    return channel.union(left_ear).union(right_ear).union(runner).cut(fork_notch)


def _tab_shape() -> cq.Workplane:
    barrel = cq.Workplane("XZ").circle(TAB_BARREL_R).extrude(TAB_BARREL_LEN / 2.0, both=True)
    neck = cq.Workplane("XY").box(TAB_NECK_LEN, TAB_NECK_W, TAB_NECK_H).translate(
        (TAB_NECK_LEN / 2.0, 0.0, -0.002)
    )
    plate = cq.Workplane("XY").box(TAB_PLATE_LEN, TAB_PLATE_W, TAB_PLATE_T).translate(
        (TAB_NECK_LEN + (TAB_PLATE_LEN / 2.0), 0.0, -0.010)
    )
    front_lip = cq.Workplane("XY").box(TAB_LIP_LEN, TAB_PLATE_W * 0.70, TAB_LIP_H).translate(
        (TAB_NECK_LEN + TAB_PLATE_LEN - (TAB_LIP_LEN / 2.0), 0.0, -0.012)
    )

    return barrel.union(neck).union(plate).union(front_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slider_tab_mechanism")

    model.material("frame_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("carriage_aluminum", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("tab_orange", rgba=(0.88, 0.51, 0.14, 1.0))

    slide_frame = model.part("slide_frame")
    slide_frame.visual(
        mesh_from_cadquery(_frame_shape(), "slide_frame"),
        material="frame_steel",
        name="frame_body",
    )
    slide_frame.inertial = Inertial.from_geometry(
        Box((FRAME_LEN, FRAME_W, 0.050)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        material="carriage_aluminum",
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LEN, CARRIAGE_W, CARRIAGE_H)),
        mass=0.95,
        origin=Origin(),
    )

    tab = model.part("tab")
    tab.visual(
        mesh_from_cadquery(_tab_shape(), "tab"),
        material="tab_orange",
        name="tab_body",
    )
    tab.inertial = Inertial.from_geometry(
        Box((TAB_NECK_LEN + TAB_PLATE_LEN, TAB_PLATE_W, 0.024)),
        mass=0.15,
        origin=Origin(xyz=(0.032, 0.0, -0.006)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=slide_frame,
        child=carriage,
        origin=Origin(xyz=(SLIDE_RETRACT_X, 0.0, SLIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=120.0,
            velocity=0.28,
        ),
    )
    model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tab,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TAB_OPEN_LIMIT,
            effort=8.0,
            velocity=2.2,
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

    slide_frame = object_model.get_part("slide_frame")
    carriage = object_model.get_part("carriage")
    tab = object_model.get_part("tab")
    slide = object_model.get_articulation("frame_to_carriage")
    hinge = object_model.get_articulation("carriage_to_tab")

    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0
    hinge_upper = hinge.motion_limits.upper if hinge.motion_limits is not None else 0.0

    ctx.expect_origin_gap(
        tab,
        carriage,
        axis="x",
        min_gap=0.098,
        max_gap=0.104,
        name="tab hinge sits on the carriage leading edge",
    )
    ctx.expect_gap(
        tab,
        carriage,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        name="tab hinge barrel seats just ahead of the carriage fork",
    )
    ctx.expect_within(
        carriage,
        slide_frame,
        axes="y",
        margin=0.0,
        name="carriage stays laterally captured inside the slide frame",
    )

    carriage_rest = ctx.part_world_position(carriage)
    tab_rest = ctx.part_world_position(tab)
    with ctx.pose({slide: slide_upper}):
        ctx.expect_within(
            carriage,
            slide_frame,
            axes="y",
            margin=0.0,
            name="extended carriage stays laterally captured inside the slide frame",
        )
        carriage_extended = ctx.part_world_position(carriage)
        tab_extended = ctx.part_world_position(tab)

    ctx.check(
        "carriage slides forward along the frame",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.16,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )
    ctx.check(
        "tab is carried forward by the carriage",
        tab_rest is not None and tab_extended is not None and tab_extended[0] > tab_rest[0] + 0.16,
        details=f"rest={tab_rest}, extended={tab_extended}",
    )

    with ctx.pose({slide: slide_upper * 0.5, hinge: 0.0}):
        closed_tab_aabb = ctx.part_world_aabb(tab)
    with ctx.pose({slide: slide_upper * 0.5, hinge: hinge_upper}):
        open_tab_aabb = ctx.part_world_aabb(tab)

    ctx.check(
        "tab opens upward from the carriage nose",
        closed_tab_aabb is not None
        and open_tab_aabb is not None
        and open_tab_aabb[1][2] > closed_tab_aabb[1][2] + 0.030,
        details=f"closed={closed_tab_aabb}, open={open_tab_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
