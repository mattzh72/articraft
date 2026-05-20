from __future__ import annotations

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

BODY_WIDTH = 0.039
BODY_LENGTH = 0.165
BODY_THICKNESS = 0.013
TOP_Z = BODY_THICKNESS * 0.5

SOURCE_POCKET_CENTER_Y = 0.0565
SOURCE_HINGE_Y = 0.0666
SOURCE_POCKET_DEPTH = 0.0044

BUTTON_CENTER_Y = 0.004
BUTTON_X_OFFSET = 0.0077
BUTTON_TRAVEL = 0.0012

LOCK_SLOT_CENTER_Y = -0.021
LOCK_SLOT_CENTER_Z = 0.0008
LOCK_TRAVEL = 0.010


def _rounded_cutter(size_x: float, size_y: float, size_z: float, radius: float):
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z)
        .edges("|Z")
        .fillet(radius)
    )


def _make_body_shape():
    body = cq.Workplane("XY").box(BODY_WIDTH, BODY_LENGTH, BODY_THICKNESS)
    body = body.edges("|Z").fillet(0.006)
    body = body.faces(">Z").edges().fillet(0.0022)
    body = body.faces("<Z").edges().fillet(0.0018)

    source_pocket = _rounded_cutter(0.028, 0.020, SOURCE_POCKET_DEPTH, 0.0018).translate(
        (0.0, SOURCE_POCKET_CENTER_Y, TOP_Z - SOURCE_POCKET_DEPTH * 0.5)
    )
    body = body.cut(source_pocket)

    for button_x in (-BUTTON_X_OFFSET, BUTTON_X_OFFSET):
        button_well = _rounded_cutter(0.0128, 0.0168, 0.0052, 0.0016).translate(
            (button_x, BUTTON_CENTER_Y, TOP_Z - 0.0026)
        )
        body = body.cut(button_well)

    side_slot = (
        cq.Workplane("XY")
        .box(0.0066, 0.024, 0.0058)
        .translate((BODY_WIDTH * 0.5 - 0.0028, LOCK_SLOT_CENTER_Y, LOCK_SLOT_CENTER_Z))
    )
    body = body.cut(side_slot)

    sensor_relief = (
        cq.Workplane("XY")
        .box(0.010, 0.006, 0.0012)
        .translate((0.0, 0.075, TOP_Z - 0.0006))
    )
    return body.cut(sensor_relief)


def _make_source_rocker():
    barrel = cq.Workplane("YZ").circle(0.0012).extrude(0.008, both=True)
    paddle = (
        cq.Workplane("XY")
        .box(0.024, 0.0174, 0.0022)
        .edges("|Z")
        .fillet(0.0015)
        .translate((0.0, -0.0092, -0.0015))
    )
    bridge = cq.Workplane("XY").box(0.016, 0.0026, 0.0014).translate((0.0, -0.0016, -0.0010))
    return barrel.union(bridge).union(paddle)


def _make_volume_button():
    cap = (
        cq.Workplane("XY")
        .box(0.0128, 0.0168, 0.0018)
        .edges("|Z")
        .fillet(0.0016)
        .translate((0.0, 0.0, -0.0009))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.0068, 0.0098, 0.0024)
        .edges("|Z")
        .fillet(0.0008)
        .translate((0.0, 0.0, -0.0030))
    )
    return cap.union(stem)


def _make_lock_switch():
    tab = (
        cq.Workplane("XY")
        .box(0.0032, 0.0100, 0.0040)
        .edges("|Z")
        .fillet(0.0010)
    )
    tongue = (
        cq.Workplane("XY")
        .box(0.0048, 0.0160, 0.0022)
        .edges("|Z")
        .fillet(0.0007)
        .translate((-0.0040, 0.0, -0.0002))
    )
    ridge_0 = cq.Workplane("XY").box(0.0007, 0.0062, 0.0008).translate((0.0012, -0.0019, 0.0024))
    ridge_1 = cq.Workplane("XY").box(0.0007, 0.0062, 0.0008).translate((0.0012, 0.0019, 0.0024))
    return tab.union(tongue).union(ridge_0).union(ridge_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soundbar_remote")

    body_black = model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    button_black = model.material("button_black", rgba=(0.20, 0.20, 0.22, 1.0))
    rocker_grey = model.material("rocker_grey", rgba=(0.28, 0.29, 0.31, 1.0))
    switch_grey = model.material("switch_grey", rgba=(0.35, 0.36, 0.38, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "soundbar_remote_body"),
        material=body_black,
        name="shell",
    )
    body.inertial = Inertial.from_geometry(Box((BODY_WIDTH, BODY_LENGTH, BODY_THICKNESS)), mass=0.14)

    source = model.part("source_rocker")
    source.visual(
        mesh_from_cadquery(_make_source_rocker(), "soundbar_remote_source_rocker"),
        material=rocker_grey,
        name="rocker",
    )
    source.inertial = Inertial.from_geometry(
        Box((0.024, 0.018, 0.004)),
        mass=0.006,
        origin=Origin(xyz=(0.0, -0.009, -0.001)),
    )

    volume_0 = model.part("volume_button_0")
    volume_0.visual(
        mesh_from_cadquery(_make_volume_button(), "soundbar_remote_volume_button_0"),
        material=button_black,
        name="button",
    )
    volume_0.inertial = Inertial.from_geometry(
        Box((0.012, 0.016, 0.005)),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    volume_1 = model.part("volume_button_1")
    volume_1.visual(
        mesh_from_cadquery(_make_volume_button(), "soundbar_remote_volume_button_1"),
        material=button_black,
        name="button",
    )
    volume_1.inertial = Inertial.from_geometry(
        Box((0.012, 0.016, 0.005)),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    lock_switch = model.part("lock_switch")
    lock_switch.visual(
        mesh_from_cadquery(_make_lock_switch(), "soundbar_remote_lock_switch"),
        material=switch_grey,
        name="switch",
    )
    lock_switch.inertial = Inertial.from_geometry(
        Box((0.010, 0.016, 0.006)),
        mass=0.002,
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
    )

    model.articulation(
        "source_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=source,
        origin=Origin(xyz=(0.0, SOURCE_HINGE_Y, TOP_Z + 0.0012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "volume_button_0_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=volume_0,
        origin=Origin(xyz=(-BUTTON_X_OFFSET, BUTTON_CENTER_Y, TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )
    model.articulation(
        "volume_button_1_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=volume_1,
        origin=Origin(xyz=(BUTTON_X_OFFSET, BUTTON_CENTER_Y, TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )
    model.articulation(
        "lock_switch_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lock_switch,
        origin=Origin(xyz=(BODY_WIDTH * 0.5 + 0.0016, LOCK_SLOT_CENTER_Y - 0.005, LOCK_SLOT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.05,
            lower=0.0,
            upper=LOCK_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    source = object_model.get_part("source_rocker")
    volume_0 = object_model.get_part("volume_button_0")
    volume_1 = object_model.get_part("volume_button_1")
    lock_switch = object_model.get_part("lock_switch")

    source_hinge = object_model.get_articulation("source_hinge")
    volume_0_slide = object_model.get_articulation("volume_button_0_slide")
    volume_1_slide = object_model.get_articulation("volume_button_1_slide")
    lock_switch_slide = object_model.get_articulation("lock_switch_slide")

    source_pos = ctx.part_world_position(source)
    volume_0_pos = ctx.part_world_position(volume_0)
    volume_1_pos = ctx.part_world_position(volume_1)
    lock_rest = ctx.part_world_position(lock_switch)

    ctx.check(
        "source rocker stays near top edge",
        source_pos is not None and source_pos[1] > 0.06,
        details=f"source_pos={source_pos}",
    )
    ctx.check(
        "volume buttons form a centered row",
        volume_0_pos is not None
        and volume_1_pos is not None
        and abs(volume_0_pos[1] - volume_1_pos[1]) < 1e-6
        and abs((volume_0_pos[0] + volume_1_pos[0]) * 0.5) < 0.002,
        details=f"volume_0_pos={volume_0_pos}, volume_1_pos={volume_1_pos}",
    )
    ctx.check(
        "lock switch sits on the side rail",
        lock_rest is not None and lock_rest[0] > BODY_WIDTH * 0.5,
        details=f"lock_rest={lock_rest}",
    )

    ctx.expect_overlap(
        source,
        body,
        axes="xy",
        min_overlap=0.019,
        name="source rocker is nested in the top control zone",
    )
    ctx.expect_overlap(
        volume_0,
        body,
        axes="xy",
        min_overlap=0.010,
        name="first volume button remains over the body footprint",
    )
    ctx.expect_overlap(
        volume_1,
        body,
        axes="xy",
        min_overlap=0.010,
        name="second volume button remains over the body footprint",
    )
    ctx.expect_overlap(
        lock_switch,
        body,
        axes="yz",
        min_overlap=0.004,
        name="lock switch stays aligned to the side slot zone",
    )

    source_rest_aabb = ctx.part_element_world_aabb(source, elem="rocker")
    with ctx.pose({source_hinge: source_hinge.motion_limits.upper}):
        source_pressed_aabb = ctx.part_element_world_aabb(source, elem="rocker")
    ctx.check(
        "source rocker tips inward when pressed",
        source_rest_aabb is not None
        and source_pressed_aabb is not None
        and source_pressed_aabb[0][2] < source_rest_aabb[0][2] - 0.0015,
        details=f"rest={source_rest_aabb}, pressed={source_pressed_aabb}",
    )

    volume_0_rest = ctx.part_world_position(volume_0)
    volume_1_rest = ctx.part_world_position(volume_1)
    with ctx.pose(
        {
            volume_0_slide: volume_0_slide.motion_limits.upper,
            volume_1_slide: volume_1_slide.motion_limits.upper,
        }
    ):
        volume_0_pressed = ctx.part_world_position(volume_0)
        volume_1_pressed = ctx.part_world_position(volume_1)
    ctx.check(
        "volume buttons depress independently downward",
        volume_0_rest is not None
        and volume_1_rest is not None
        and volume_0_pressed is not None
        and volume_1_pressed is not None
        and volume_0_pressed[2] < volume_0_rest[2] - 0.0010
        and volume_1_pressed[2] < volume_1_rest[2] - 0.0010,
        details=(
            f"volume_0_rest={volume_0_rest}, volume_0_pressed={volume_0_pressed}, "
            f"volume_1_rest={volume_1_rest}, volume_1_pressed={volume_1_pressed}"
        ),
    )

    with ctx.pose({lock_switch_slide: lock_switch_slide.motion_limits.upper}):
        lock_forward = ctx.part_world_position(lock_switch)
    ctx.check(
        "lock switch slides forward along the side slot",
        lock_rest is not None and lock_forward is not None and lock_forward[1] > lock_rest[1] + 0.008,
        details=f"lock_rest={lock_rest}, lock_forward={lock_forward}",
    )

    return ctx.report()


object_model = build_object_model()
