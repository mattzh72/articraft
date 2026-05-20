from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.084
HALF_BODY_WIDTH = BODY_WIDTH * 0.5
FRONT_WHEEL_X = 0.050
REAR_WHEEL_X = -0.050
WHEEL_CENTER_Y = 0.046
REAR_WHEEL_CENTER_Y = 0.047
WHEEL_RADIUS = 0.018
WHEEL_WIDTH = 0.012
DOOR_LENGTH = 0.062
DOOR_HEIGHT = 0.038
DOOR_THICKNESS = 0.0016
DOOR_CENTER_Z = 0.042
DOOR_FRONT_X = 0.031
LIFTGATE_WIDTH = 0.062
LIFTGATE_HEIGHT = 0.045
LIFTGATE_RUN = 0.010
STEERING_PIVOT_X = 0.050
STEERING_PIVOT_Z = 0.020


def _cad_mesh(shape: object, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.0007, angular_tolerance=0.08)


def _body_shell() -> object:
    shell = (
        cq.Workplane("XZ")
        .moveTo(0.090, 0.014)
        .lineTo(0.090, 0.022)
        .lineTo(0.087, 0.036)
        .lineTo(0.053, 0.047)
        .lineTo(0.018, 0.070)
        .lineTo(-0.044, 0.072)
        .lineTo(-0.068, 0.064)
        .lineTo(-0.080, 0.052)
        .lineTo(-0.085, 0.036)
        .lineTo(-0.086, 0.020)
        .lineTo(-0.078, 0.014)
        .close()
        .extrude(BODY_WIDTH)
        .translate((0.0, HALF_BODY_WIDTH, 0.0))
    )

    cavity = cq.Workplane("XY").box(0.132, 0.060, 0.042).translate((-0.006, 0.0, 0.043))
    front_arch = (
        cq.Workplane("XZ")
        .center(FRONT_WHEEL_X, 0.021)
        .circle(0.021)
        .extrude(BODY_WIDTH + 0.018)
        .translate((0.0, (BODY_WIDTH + 0.018) * 0.5, 0.0))
    )
    rear_arch = (
        cq.Workplane("XZ")
        .center(REAR_WHEEL_X, 0.021)
        .circle(0.021)
        .extrude(BODY_WIDTH + 0.018)
        .translate((0.0, (BODY_WIDTH + 0.018) * 0.5, 0.0))
    )
    liftgate_opening = (
        cq.Workplane("XZ")
        .moveTo(-0.073, 0.068)
        .lineTo(-0.079, 0.068)
        .lineTo(-0.091, 0.018)
        .lineTo(-0.085, 0.018)
        .close()
        .extrude(LIFTGATE_WIDTH + 0.010)
        .translate((0.0, (LIFTGATE_WIDTH + 0.010) * 0.5, 0.0))
    )

    return shell.cut(cavity).cut(front_arch).cut(rear_arch).cut(liftgate_opening)


def _liftgate_panel(*, width: float = LIFTGATE_WIDTH, height: float = LIFTGATE_HEIGHT, run: float = LIFTGATE_RUN, skin: float = 0.0020) -> object:
    return (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(-skin, 0.0)
        .lineTo(-(run + skin), -height)
        .lineTo(-run, -height)
        .close()
        .extrude(width)
        .translate((0.0, width * 0.5, 0.0))
    )


def _add_wheel_visuals(part, mesh_prefix: str, *, wheel_material, tire_material) -> None:
    spin_align = Origin(rpy=(0.0, 0.0, pi * 0.5))
    part.visual(
        mesh_from_geometry(TireGeometry(outer_radius=WHEEL_RADIUS, width=WHEEL_WIDTH, inner_radius=0.0122), f"{mesh_prefix}_tire"),
        origin=spin_align,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.0132, length=0.0088),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=wheel_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.0048, length=0.0105),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=wheel_material,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_hatchback")

    body_red = model.material("body_red", rgba=(0.82, 0.12, 0.12, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.10, 0.11, 1.0))
    axle_gray = model.material("axle_gray", rgba=(0.26, 0.28, 0.30, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.75, 0.77, 0.80, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.23, 0.30, 1.0))
    lamp = model.material("lamp", rgba=(0.88, 0.90, 0.86, 1.0))
    tail_lamp = model.material("tail_lamp", rgba=(0.78, 0.14, 0.14, 1.0))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(_cad_mesh(_body_shell(), "body_shell"), material=body_red, name="shell")
    body.visual(
        Box((0.018, 0.018, 0.004)),
        origin=Origin(xyz=(STEERING_PIVOT_X, 0.0, 0.018)),
        material=axle_gray,
        name="pivot_bracket",
    )
    body.visual(
        Box((0.003, 0.058, 0.030)),
        origin=Origin(xyz=(0.024, 0.0, 0.056), rpy=(0.0, 0.78, 0.0)),
        material=glass,
        name="windshield",
    )
    body.visual(Box((0.006, 0.052, 0.006)), origin=Origin(xyz=(-0.074, 0.0, 0.069)), material=axle_gray, name="hinge_pad")
    body.visual(Box((0.009, 0.016, 0.008)), origin=Origin(xyz=(0.086, 0.024, 0.034)), material=lamp, name="headlamp_0")
    body.visual(Box((0.009, 0.016, 0.008)), origin=Origin(xyz=(0.086, -0.024, 0.034)), material=lamp, name="headlamp_1")
    body.visual(Box((0.018, 0.0046, 0.004)), origin=Origin(xyz=(-0.059, 0.0403, WHEEL_RADIUS)), material=axle_gray, name="rear_stub_0")
    body.visual(Box((0.018, 0.0046, 0.004)), origin=Origin(xyz=(-0.059, -0.0403, WHEEL_RADIUS)), material=axle_gray, name="rear_stub_1")
    body.visual(Box((0.022, 0.050, 0.006)), origin=Origin(xyz=(0.082, 0.0, 0.025)), material=dark_trim, name="bumper")

    left_door = model.part("left_door")
    left_door.visual(Box((DOOR_LENGTH, DOOR_THICKNESS, DOOR_HEIGHT)), origin=Origin(xyz=(-DOOR_LENGTH * 0.5, DOOR_THICKNESS * 0.5, 0.0)), material=body_red, name="panel")
    left_door.visual(
        Box((0.038, 0.0010, 0.014)),
        origin=Origin(xyz=(-0.034, 0.0010, 0.006)),
        material=glass,
        name="window",
    )

    right_door = model.part("right_door")
    right_door.visual(Box((DOOR_LENGTH, DOOR_THICKNESS, DOOR_HEIGHT)), origin=Origin(xyz=(-DOOR_LENGTH * 0.5, -DOOR_THICKNESS * 0.5, 0.0)), material=body_red, name="panel")
    right_door.visual(
        Box((0.038, 0.0010, 0.014)),
        origin=Origin(xyz=(-0.034, -0.0010, 0.006)),
        material=glass,
        name="window",
    )

    liftgate = model.part("liftgate")
    liftgate.visual(_cad_mesh(_liftgate_panel(), "liftgate_panel"), material=body_red, name="panel")
    liftgate.visual(Box((0.012, 0.052, 0.004)), origin=Origin(xyz=(0.004, 0.0, 0.004)), material=axle_gray, name="hinge_arm")
    liftgate.visual(
        _cad_mesh(
            _liftgate_panel(width=LIFTGATE_WIDTH * 0.78, height=LIFTGATE_HEIGHT * 0.56, run=LIFTGATE_RUN * 0.58, skin=0.0008).translate(
                (-0.0016, 0.0, -0.009)
            ),
            "liftgate_glass",
        ),
        material=glass,
        name="glass",
    )
    liftgate.visual(Box((0.003, 0.010, 0.010)), origin=Origin(xyz=(-0.009, 0.024, -0.035)), material=tail_lamp, name="tail_lamp_0")
    liftgate.visual(Box((0.003, 0.010, 0.010)), origin=Origin(xyz=(-0.009, -0.024, -0.035)), material=tail_lamp, name="tail_lamp_1")

    front_axle = model.part("front_axle")
    front_axle.visual(
        Box((0.010, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=axle_gray,
        name="pivot_post",
    )
    front_axle.visual(
        Box((0.010, 0.072, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=axle_gray,
        name="beam",
    )
    front_axle.visual(Box((0.012, 0.004, 0.018)), origin=Origin(xyz=(0.0, 0.038, -0.010)), material=axle_gray, name="upright_0")
    front_axle.visual(Box((0.012, 0.004, 0.018)), origin=Origin(xyz=(0.0, -0.038, -0.010)), material=axle_gray, name="upright_1")
    front_axle.visual(Box((0.028, 0.050, 0.003)), origin=Origin(xyz=(0.008, 0.0, -0.011)), material=axle_gray, name="tie_bar")

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(front_left_wheel, "front_left_wheel", wheel_material=wheel_silver, tire_material=tire_black)

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(front_right_wheel, "front_right_wheel", wheel_material=wheel_silver, tire_material=tire_black)

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(rear_left_wheel, "rear_left_wheel", wheel_material=wheel_silver, tire_material=tire_black)

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(rear_right_wheel, "rear_right_wheel", wheel_material=wheel_silver, tire_material=tire_black)

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(DOOR_FRONT_X, HALF_BODY_WIDTH, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.1),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(DOOR_FRONT_X, -HALF_BODY_WIDTH, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.1),
    )
    model.articulation(
        "liftgate_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=liftgate,
        origin=Origin(xyz=(-0.087, 0.0, 0.067)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.1),
    )
    model.articulation(
        "steer_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_axle,
        origin=Origin(xyz=(STEERING_PIVOT_X, 0.0, STEERING_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=front_axle,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, WHEEL_RADIUS - STEERING_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=front_axle,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, -WHEEL_CENTER_Y, WHEEL_RADIUS - STEERING_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_left_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, REAR_WHEEL_CENTER_Y, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_right_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, -REAR_WHEEL_CENTER_Y, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    front_axle = object_model.get_part("front_axle")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    liftgate = object_model.get_part("liftgate")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    liftgate_hinge = object_model.get_articulation("liftgate_hinge")
    steer_pivot = object_model.get_articulation("steer_pivot")

    ctx.expect_contact(
        body,
        front_axle,
        elem_a="pivot_bracket",
        elem_b="pivot_post",
        contact_tol=0.0005,
        name="front steering axle is supported at the central pivot",
    )
    ctx.expect_overlap(
        left_door,
        body,
        axes="xz",
        min_overlap=0.028,
        elem_a="panel",
        elem_b="shell",
        name="left door stays aligned with the body side",
    )
    ctx.expect_overlap(
        right_door,
        body,
        axes="xz",
        min_overlap=0.028,
        elem_a="panel",
        elem_b="shell",
        name="right door stays aligned with the body side",
    )
    ctx.expect_overlap(
        liftgate,
        body,
        axes="yz",
        min_overlap=0.040,
        elem_a="panel",
        elem_b="shell",
        name="liftgate covers the rear opening area",
    )
    ctx.expect_contact(
        body,
        liftgate,
        elem_a="hinge_pad",
        elem_b="hinge_arm",
        contact_tol=0.0005,
        name="liftgate is hung from a visible top hinge arm",
    )

    left_closed = ctx.part_element_world_aabb(left_door, elem="panel")
    right_closed = ctx.part_element_world_aabb(right_door, elem="panel")
    hatch_closed = ctx.part_element_world_aabb(liftgate, elem="panel")
    fl_rest = ctx.part_world_position(front_left_wheel)
    fr_rest = ctx.part_world_position(front_right_wheel)

    with ctx.pose({left_hinge: 1.0}):
        left_open = ctx.part_element_world_aabb(left_door, elem="panel")
    with ctx.pose({right_hinge: 1.0}):
        right_open = ctx.part_element_world_aabb(right_door, elem="panel")
    with ctx.pose({liftgate_hinge: 1.0}):
        hatch_open = ctx.part_element_world_aabb(liftgate, elem="panel")
    with ctx.pose({steer_pivot: 0.45}):
        fl_steered = ctx.part_world_position(front_left_wheel)
        fr_steered = ctx.part_world_position(front_right_wheel)

    ctx.check(
        "left door opens outward",
        left_closed is not None and left_open is not None and left_open[1][1] > left_closed[1][1] + 0.012,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right door opens outward",
        right_closed is not None and right_open is not None and right_open[0][1] < right_closed[0][1] - 0.012,
        details=f"closed={right_closed}, open={right_open}",
    )
    ctx.check(
        "liftgate opens upward",
        hatch_closed is not None and hatch_open is not None and hatch_open[1][2] > hatch_closed[1][2] + 0.009,
        details=f"closed={hatch_closed}, open={hatch_open}",
    )
    ctx.check(
        "front axle yaws both wheels under steering",
        fl_rest is not None
        and fr_rest is not None
        and fl_steered is not None
        and fr_steered is not None
        and fl_steered[0] < fl_rest[0] - 0.008
        and fr_steered[0] > fr_rest[0] + 0.008,
        details=f"rest_left={fl_rest}, steered_left={fl_steered}, rest_right={fr_rest}, steered_right={fr_steered}",
    )

    return ctx.report()


object_model = build_object_model()
