from __future__ import annotations

from math import pi

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


BODY_DEPTH = 0.042
BODY_FRONT_Y = BODY_DEPTH / 2.0
BODY_BACK_Y = -BODY_FRONT_Y

HEAD_DEPTH = 0.026
HEAD_CENTER_Z = 0.198
HEAD_OUTER_RADIUS = 0.033
HEAD_INNER_RADIUS = 0.021

HINGE_X = 0.021
HINGE_Z = 0.229

DIAL_RADIUS = 0.021
DIAL_DEPTH = 0.012
DIAL_CENTER_Z = 0.092

BUTTON_TRAVEL = 0.002
BUTTON_DEPTH = 0.010
BUTTON_POCKET_DEPTH = 0.011

NCV_X = 0.024
NCV_Z = 0.176
RANGE_X = 0.031
RANGE_Z = 0.100

STAND_HINGE_Z = 0.026


def _rounded_box(size: tuple[float, float, float], center: tuple[float, float, float], radius: float):
    solid = cq.Workplane("XY").box(*size).translate(center)
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    return solid


def _circular_cut(center_x: float, center_z: float, radius: float, depth: float):
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(radius)
        .extrude(depth / 2.0, both=True)
        .translate((0.0, BODY_FRONT_Y - depth / 2.0, 0.0))
    )


def _box_cut(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    radius: float = 0.0,
):
    cut = cq.Workplane("XY").box(*size).translate(center)
    if radius > 0.0:
        cut = cut.edges("|Y").fillet(radius)
    return cut


def make_housing() -> cq.Workplane:
    lower_body = _rounded_box((0.074, BODY_DEPTH, 0.132), (0.0, 0.0, 0.066), 0.010)
    upper_body = _rounded_box((0.092, BODY_DEPTH, 0.052), (0.0, 0.0, 0.144), 0.010)

    fixed_head = (
        cq.Workplane("XZ")
        .center(0.0, HEAD_CENTER_Z)
        .circle(HEAD_OUTER_RADIUS)
        .circle(HEAD_INNER_RADIUS)
        .extrude(HEAD_DEPTH / 2.0, both=True)
    )
    fixed_head = fixed_head.cut(
        cq.Workplane("XY")
        .box(0.056, HEAD_DEPTH + 0.006, 0.064)
        .translate((0.028, 0.0, HEAD_CENTER_Z))
    )
    hinge_tab = cq.Workplane("XY").box(0.026, HEAD_DEPTH, 0.026).translate((0.006, 0.0, 0.212))

    housing = lower_body.union(upper_body).union(fixed_head).union(hinge_tab)

    display_recess = _box_cut(
        (0.050, 0.004, 0.036),
        (0.0, BODY_FRONT_Y - 0.002, 0.141),
        radius=0.003,
    )
    dial_pocket = _circular_cut(0.0, DIAL_CENTER_Z, 0.025, 0.003)
    ncv_pocket = _circular_cut(NCV_X, NCV_Z, 0.0066, BUTTON_POCKET_DEPTH)
    range_pocket = _box_cut(
        (0.024, BUTTON_POCKET_DEPTH, 0.012),
        (RANGE_X, BODY_FRONT_Y - BUTTON_POCKET_DEPTH / 2.0, RANGE_Z),
        radius=0.004,
    )

    return housing.cut(display_recess).cut(dial_pocket).cut(ncv_pocket).cut(range_pocket)


def make_grip() -> cq.Workplane:
    return _rounded_box((0.056, BODY_DEPTH + 0.002, 0.086), (0.0, 0.0, 0.048), 0.011)


def make_dial() -> cq.Workplane:
    dial = cq.Workplane("XZ").circle(DIAL_RADIUS).extrude(DIAL_DEPTH / 2.0, both=True)
    dial = dial.union(cq.Workplane("XZ").circle(0.014).extrude(0.004, both=True).translate((0.0, 0.003, 0.0)))
    pointer = cq.Workplane("XY").box(0.004, 0.002, 0.013).translate((0.0, 0.007, 0.008))
    return dial.union(pointer)


def make_jaw() -> cq.Workplane:
    head_center_rel = (-HINGE_X, HEAD_CENTER_Z - HINGE_Z)
    jaw_arc = (
        cq.Workplane("XZ")
        .center(*head_center_rel)
        .circle(0.0305)
        .circle(0.0195)
        .extrude(0.022, both=True)
    )
    selector = cq.Workplane("XY").box(0.024, 0.026, 0.066).translate((0.010, 0.0, -0.030))
    tip_trim = cq.Workplane("XY").box(0.012, 0.030, 0.020).translate((0.013, 0.0, -0.055))
    hinge_lug = cq.Workplane("XY").box(0.010, 0.018, 0.010).translate((0.006, 0.0, -0.005))
    return jaw_arc.intersect(selector).cut(tip_trim).union(hinge_lug)


def make_ncv_button() -> cq.Workplane:
    return cq.Workplane("XZ").circle(0.0065).extrude(BUTTON_DEPTH).translate((0.0, BUTTON_DEPTH, 0.0))


def make_range_button() -> cq.Workplane:
    return cq.Workplane("XY").box(0.020, BUTTON_DEPTH, 0.010).translate((0.0, BUTTON_DEPTH / 2.0, 0.0))


def make_stand() -> cq.Workplane:
    leaf = cq.Workplane("XY").box(0.036, 0.004, 0.060).translate((0.0, -0.002, 0.030))
    foot = cq.Workplane("XY").box(0.028, 0.006, 0.010).translate((0.0, -0.003, 0.006))
    return leaf.union(foot)


def make_dial_bezel() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(0.0, DIAL_CENTER_Z)
        .circle(0.026)
        .circle(0.019)
        .extrude(0.002, both=True)
        .translate((0.0, BODY_FRONT_Y, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_clamp_meter")

    housing_orange = model.material("housing_orange", rgba=(0.93, 0.47, 0.12, 1.0))
    grip_black = model.material("grip_black", rgba=(0.11, 0.11, 0.12, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.33, 0.35, 0.37, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.10, 0.13, 0.14, 1.0))
    button_red = model.material("button_red", rgba=(0.72, 0.10, 0.10, 1.0))
    button_black = model.material("button_black", rgba=(0.16, 0.16, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_housing(), "clamp_meter_housing"),
        material=housing_orange,
        name="housing",
    )
    body.visual(
        mesh_from_cadquery(make_grip(), "clamp_meter_grip"),
        material=grip_black,
        name="grip",
    )
    body.visual(
        mesh_from_cadquery(make_dial_bezel(), "clamp_meter_dial_bezel"),
        material=trim_gray,
        name="dial_bezel",
    )
    body.visual(
        Box((0.046, 0.004, 0.032)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.002, 0.141)),
        material=glass_dark,
        name="display_lens",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(make_jaw(), "clamp_meter_jaw"),
        material=grip_black,
        name="jaw_arc",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(make_dial(), "clamp_meter_dial"),
        material=trim_gray,
        name="dial_knob",
    )

    ncv_button = model.part("ncv_button")
    ncv_button.visual(
        mesh_from_cadquery(make_ncv_button(), "clamp_meter_ncv_button"),
        material=button_red,
        name="ncv_cap",
    )

    range_button = model.part("range_button")
    range_button.visual(
        mesh_from_cadquery(make_range_button(), "clamp_meter_range_button"),
        material=button_black,
        name="range_cap",
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(make_stand(), "clamp_meter_stand"),
        material=trim_gray,
        name="stand_leaf",
    )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.78, effort=4.0, velocity=2.5),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + DIAL_DEPTH / 2.0, DIAL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    model.articulation(
        "body_to_ncv_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=ncv_button,
        origin=Origin(xyz=(NCV_X, BODY_FRONT_Y - 0.008, NCV_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=5.0, velocity=0.05),
    )
    model.articulation(
        "body_to_range_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=range_button,
        origin=Origin(xyz=(RANGE_X, BODY_FRONT_Y - 0.008, RANGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=5.0, velocity=0.05),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, BODY_BACK_Y, STAND_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=3.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    dial = object_model.get_part("dial")
    ncv_button = object_model.get_part("ncv_button")
    range_button = object_model.get_part("range_button")
    stand = object_model.get_part("stand")

    jaw_joint = object_model.get_articulation("body_to_jaw")
    dial_joint = object_model.get_articulation("body_to_dial")
    ncv_joint = object_model.get_articulation("body_to_ncv_button")
    range_joint = object_model.get_articulation("body_to_range_button")
    stand_joint = object_model.get_articulation("body_to_stand")

    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_a="dial_knob",
        elem_b="dial_bezel",
        min_overlap=0.018,
        name="dial stays centered on the front panel",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="y",
        positive_elem="dial_knob",
        negative_elem="housing",
        min_gap=0.0,
        max_gap=0.0020,
        name="dial stays seated on the housing face",
    )
    housing_aabb = ctx.part_element_world_aabb(body, elem="housing")
    ncv_aabb = ctx.part_element_world_aabb(ncv_button, elem="ncv_cap")
    range_aabb = ctx.part_element_world_aabb(range_button, elem="range_cap")
    housing_front_y = housing_aabb[1][1] if housing_aabb is not None else None
    ncv_front_y = ncv_aabb[1][1] if ncv_aabb is not None else None
    range_front_y = range_aabb[1][1] if range_aabb is not None else None

    ctx.check(
        "ncv button protrudes as a real control",
        housing_front_y is not None
        and ncv_front_y is not None
        and housing_front_y + 0.0015 <= ncv_front_y <= housing_front_y + 0.0035,
        details=f"housing_front_y={housing_front_y}, ncv_front_y={ncv_front_y}",
    )
    ctx.check(
        "range button protrudes as a real control",
        housing_front_y is not None
        and range_front_y is not None
        and housing_front_y + 0.0015 <= range_front_y <= housing_front_y + 0.0035,
        details=f"housing_front_y={housing_front_y}, range_front_y={range_front_y}",
    )

    jaw_limits = jaw_joint.motion_limits
    if jaw_limits is not None and jaw_limits.upper is not None:
        closed_jaw = ctx.part_element_world_aabb(jaw, elem="jaw_arc")
        with ctx.pose({jaw_joint: jaw_limits.upper}):
            open_jaw = ctx.part_element_world_aabb(jaw, elem="jaw_arc")
        ctx.check(
            "jaw opens outward from the fixed head",
            closed_jaw is not None
            and open_jaw is not None
            and open_jaw[1][0] > closed_jaw[1][0] + 0.010,
            details=f"closed={closed_jaw}, open={open_jaw}",
        )

    with ctx.pose({dial_joint: pi / 2.0}):
        ctx.expect_overlap(
            dial,
            body,
            axes="xz",
            elem_a="dial_knob",
            elem_b="dial_bezel",
            min_overlap=0.018,
            name="dial remains seated while rotated",
        )

    ncv_limits = ncv_joint.motion_limits
    if ncv_limits is not None and ncv_limits.upper is not None:
        ncv_rest = ctx.part_element_world_aabb(ncv_button, elem="ncv_cap")
        with ctx.pose({ncv_joint: ncv_limits.upper}):
            ncv_pressed = ctx.part_element_world_aabb(ncv_button, elem="ncv_cap")
            pressed_housing = ctx.part_element_world_aabb(body, elem="housing")
        pressed_housing_front = pressed_housing[1][1] if pressed_housing is not None else None
        ncv_pressed_front = ncv_pressed[1][1] if ncv_pressed is not None else None
        ctx.check(
            "ncv button can press nearly flush",
            pressed_housing_front is not None
            and ncv_pressed_front is not None
            and abs(ncv_pressed_front - pressed_housing_front) <= 0.0005,
            details=f"housing_front_y={pressed_housing_front}, pressed_front_y={ncv_pressed_front}",
        )
        ctx.check(
            "ncv button moves inward independently",
            ncv_rest is not None
            and ncv_pressed is not None
            and ncv_pressed[1][1] < ncv_rest[1][1] - 0.0015,
            details=f"rest={ncv_rest}, pressed={ncv_pressed}",
        )

    range_limits = range_joint.motion_limits
    if range_limits is not None and range_limits.upper is not None:
        range_rest = ctx.part_element_world_aabb(range_button, elem="range_cap")
        with ctx.pose({range_joint: range_limits.upper}):
            range_pressed = ctx.part_element_world_aabb(range_button, elem="range_cap")
            pressed_housing = ctx.part_element_world_aabb(body, elem="housing")
        pressed_housing_front = pressed_housing[1][1] if pressed_housing is not None else None
        range_pressed_front = range_pressed[1][1] if range_pressed is not None else None
        ctx.check(
            "range button can press nearly flush",
            pressed_housing_front is not None
            and range_pressed_front is not None
            and abs(range_pressed_front - pressed_housing_front) <= 0.0005,
            details=f"housing_front_y={pressed_housing_front}, pressed_front_y={range_pressed_front}",
        )
        ctx.check(
            "range button moves inward independently",
            range_rest is not None
            and range_pressed is not None
            and range_pressed[1][1] < range_rest[1][1] - 0.0015,
            details=f"rest={range_rest}, pressed={range_pressed}",
        )

    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        stand_closed = ctx.part_element_world_aabb(stand, elem="stand_leaf")
        with ctx.pose({stand_joint: stand_limits.upper}):
            stand_open = ctx.part_element_world_aabb(stand, elem="stand_leaf")
        ctx.check(
            "rear stand folds out behind the meter",
            stand_closed is not None
            and stand_open is not None
            and stand_open[0][1] < stand_closed[0][1] - 0.018,
            details=f"closed={stand_closed}, open={stand_open}",
        )

    return ctx.report()


object_model = build_object_model()
