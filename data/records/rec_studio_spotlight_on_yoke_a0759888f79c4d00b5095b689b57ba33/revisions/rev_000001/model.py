from __future__ import annotations

import math

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
    mesh_from_cadquery,
)


BASE_LENGTH = 0.72
BASE_WIDTH = 0.48
FRAME_THICKNESS = 0.036
FRAME_Z = 0.111
WHEEL_RADIUS = 0.042
WHEEL_WIDTH = 0.032
WHEEL_CENTER_Z = WHEEL_RADIUS
PAN_Z = 0.385
TILT_OFFSET_X = 0.120
TILT_OFFSET_Z = -0.097
CAN_RADIUS = 0.142
GUARD_HINGE_X = 0.346
GUARD_HINGE_Y = 0.146


def _make_can_mesh():
    main_shell = (
        cq.Workplane("YZ")
        .circle(CAN_RADIUS)
        .extrude(0.31)
        .translate((0.03, 0.0, 0.0))
    )
    cavity = (
        cq.Workplane("YZ")
        .circle(0.125)
        .extrude(0.275)
        .translate((0.060, 0.0, 0.0))
    )
    rear_cap = (
        cq.Workplane("YZ")
        .circle(0.110)
        .extrude(0.100)
        .translate((-0.050, 0.0, 0.0))
    )
    front_bezel = (
        cq.Workplane("YZ")
        .circle(0.154)
        .extrude(0.040)
        .translate((0.300, 0.0, 0.0))
    )
    can_body = main_shell.union(rear_cap).union(front_bezel).cut(cavity)
    return mesh_from_cadquery(can_body, "spotlight_can")


def _make_guard_mesh():
    frame_center_y = -GUARD_HINGE_Y
    ring = (
        cq.Workplane("YZ")
        .center(frame_center_y, 0.0)
        .circle(0.155)
        .circle(0.133)
        .extrude(0.004, both=True)
    )
    vertical_bar = (
        cq.Workplane("YZ")
        .center(frame_center_y, 0.0)
        .rect(0.010, 0.272)
        .extrude(0.004, both=True)
    )
    horizontal_bar = (
        cq.Workplane("YZ")
        .center(frame_center_y, 0.0)
        .rect(0.246, 0.010)
        .extrude(0.004, both=True)
    )
    hinge_bridge = (
        cq.Workplane("YZ")
        .center(-0.004, 0.0)
        .rect(0.024, 0.010)
        .extrude(0.004, both=True)
    )
    hinge_barrel = cq.Workplane("XY").circle(0.006).extrude(0.010, both=True)
    guard = ring.union(vertical_bar).union(horizontal_bar).union(hinge_bridge).union(hinge_barrel)
    return mesh_from_cadquery(guard, "spotlight_guard")


def _wheel_locations() -> dict[str, tuple[float, float, float]]:
    return {
        "front_left_wheel": (0.29, 0.19, WHEEL_CENTER_Z),
        "front_right_wheel": (0.29, -0.19, WHEEL_CENTER_Z),
        "rear_left_wheel": (-0.29, 0.19, WHEEL_CENTER_Z),
        "rear_right_wheel": (-0.29, -0.19, WHEEL_CENTER_Z),
    }


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="theatre_spotlight_stand")

    frame_black = model.material("frame_black", rgba=(0.12, 0.12, 0.13, 1.0))
    can_black = model.material("can_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.84, 0.92, 0.45))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, 0.055, FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.2125, FRAME_Z)),
        material=frame_black,
        name="left_rail",
    )
    base.visual(
        Box((BASE_LENGTH, 0.055, FRAME_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.2125, FRAME_Z)),
        material=frame_black,
        name="right_rail",
    )
    base.visual(
        Box((0.060, 0.390, FRAME_THICKNESS)),
        origin=Origin(xyz=(0.330, 0.0, FRAME_Z)),
        material=frame_black,
        name="front_cross",
    )
    base.visual(
        Box((0.060, 0.390, FRAME_THICKNESS)),
        origin=Origin(xyz=(-0.330, 0.0, FRAME_Z)),
        material=frame_black,
        name="rear_cross",
    )
    base.visual(
        Box((0.240, 0.180, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        material=frame_black,
        name="center_plate",
    )
    base.visual(
        Box((0.220, 0.170, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, 0.126)),
        material=dark_steel,
        name="left_brace",
    )
    base.visual(
        Box((0.220, 0.170, 0.018)),
        origin=Origin(xyz=(0.0, -0.105, 0.126)),
        material=dark_steel,
        name="right_brace",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=dark_steel,
        name="post_mount",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.2355)),
        material=frame_black,
        name="post",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.368)),
        material=dark_steel,
        name="pan_collar",
    )

    for wheel_name, (x, y, _) in _wheel_locations().items():
        base.visual(
            Box((0.070, 0.055, 0.012)),
            origin=Origin(xyz=(x, y, 0.093)),
            material=dark_steel,
            name=f"{wheel_name}_plate",
        )
        for side in (-1.0, 1.0):
            base.visual(
                Box((0.052, 0.008, 0.064)),
                origin=Origin(xyz=(x, y + side * 0.020, 0.061)),
                material=dark_steel,
                name=f"{wheel_name}_fork_{0 if side < 0 else 1}",
            )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.034, 0.340, 0.016)),
        origin=Origin(xyz=(0.050, 0.0, -0.050)),
        material=frame_black,
        name="bridge",
    )
    yoke.visual(
        Cylinder(radius=0.045, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_steel,
        name="hub",
    )
    yoke.visual(
        Box((0.160, 0.012, 0.226)),
        origin=Origin(xyz=(0.060, 0.166, -0.113)),
        material=frame_black,
        name="arm_left",
    )
    yoke.visual(
        Box((0.160, 0.012, 0.226)),
        origin=Origin(xyz=(0.060, -0.166, -0.113)),
        material=frame_black,
        name="arm_right",
    )
    yoke.visual(
        Box((0.030, 0.046, 0.090)),
        origin=Origin(xyz=(0.030, 0.065, -0.035)),
        material=dark_steel,
        name="left_gusset",
    )
    yoke.visual(
        Box((0.030, 0.046, 0.090)),
        origin=Origin(xyz=(0.030, -0.065, -0.035)),
        material=dark_steel,
        name="right_gusset",
    )

    can = model.part("can")
    can.visual(_make_can_mesh(), material=can_black, name="can_shell")
    can.visual(
        Cylinder(radius=0.133, length=0.010),
        origin=Origin(xyz=(0.317, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    can.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.0, 0.148, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    can.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.0, -0.148, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    can.visual(
        Box((0.070, 0.022, 0.050)),
        origin=Origin(xyz=(0.030, 0.125, 0.0)),
        material=steel,
        name="left_mount",
    )
    can.visual(
        Box((0.070, 0.022, 0.050)),
        origin=Origin(xyz=(0.030, -0.125, 0.0)),
        material=steel,
        name="right_mount",
    )
    can.visual(
        Box((0.022, 0.012, 0.032)),
        origin=Origin(xyz=(0.330, 0.146, 0.0)),
        material=steel,
        name="guard_tab",
    )
    can.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.330, GUARD_HINGE_Y, -0.011), rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="guard_barrel_0",
    )
    can.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.330, GUARD_HINGE_Y, 0.011), rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="guard_barrel_1",
    )

    guard = model.part("guard")
    guard.visual(_make_guard_mesh(), material=steel, name="guard_frame")

    for wheel_name in _wheel_locations():
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.028, length=0.026),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.012, length=0.036),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="axle_cap",
        )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(TILT_OFFSET_X, 0.0, TILT_OFFSET_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.4,
            lower=-math.radians(70.0),
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "can_to_guard",
        ArticulationType.REVOLUTE,
        parent=can,
        child=guard,
        origin=Origin(xyz=(GUARD_HINGE_X, GUARD_HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    for wheel_name, wheel_xyz in _wheel_locations().items():
        model.articulation(
            f"base_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel_name,
            origin=Origin(xyz=wheel_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    can = object_model.get_part("can")
    guard = object_model.get_part("guard")
    yoke_pan = object_model.get_articulation("base_to_yoke")
    can_tilt = object_model.get_articulation("yoke_to_can")
    guard_hinge = object_model.get_articulation("can_to_guard")

    wheel_joints = [
        object_model.get_articulation("base_to_front_left_wheel"),
        object_model.get_articulation("base_to_front_right_wheel"),
        object_model.get_articulation("base_to_rear_left_wheel"),
        object_model.get_articulation("base_to_rear_right_wheel"),
    ]
    ctx.check(
        "wheels use continuous spin joints",
        all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in wheel_joints),
        details=", ".join(f"{joint.name}:{joint.articulation_type}" for joint in wheel_joints),
    )

    for wheel_name, expected in _wheel_locations().items():
        position = ctx.part_world_position(wheel_name)
        ctx.check(
            f"{wheel_name} stays under the base corners",
            position is not None
            and abs(position[0] - expected[0]) < 1e-6
            and abs(position[1] - expected[1]) < 1e-6
            and abs(position[2] - expected[2]) < 1e-6,
            details=f"expected={expected}, actual={position}",
        )

    with ctx.pose({guard_hinge: 0.0}):
        ctx.expect_gap(
            guard,
            can,
            axis="x",
            positive_elem="guard_frame",
            negative_elem="lens",
            min_gap=0.005,
            max_gap=0.020,
            name="guard rests just ahead of the lens",
        )
        ctx.expect_overlap(
            guard,
            can,
            axes="yz",
            elem_a="guard_frame",
            elem_b="lens",
            min_overlap=0.240,
            name="guard covers the lens opening when shut",
        )

    rest_guard_box = ctx.part_element_world_aabb(guard, elem="guard_frame")
    with ctx.pose({guard_hinge: math.radians(95.0)}):
        open_guard_box = ctx.part_element_world_aabb(guard, elem="guard_frame")
    rest_guard_center = _aabb_center(rest_guard_box)
    open_guard_center = _aabb_center(open_guard_box)
    ctx.check(
        "guard swings out to the side",
        rest_guard_center is not None
        and open_guard_center is not None
        and open_guard_center[0] > rest_guard_center[0] + 0.10,
        details=f"rest={rest_guard_center}, open={open_guard_center}",
    )

    rest_lens_box = ctx.part_element_world_aabb(can, elem="lens")
    with ctx.pose({can_tilt: math.radians(70.0)}):
        raised_lens_box = ctx.part_element_world_aabb(can, elem="lens")
    rest_lens_center = _aabb_center(rest_lens_box)
    raised_lens_center = _aabb_center(raised_lens_box)
    ctx.check(
        "can tilts upward around the yoke",
        rest_lens_center is not None
        and raised_lens_center is not None
        and raised_lens_center[2] > rest_lens_center[2] + 0.10,
        details=f"rest={rest_lens_center}, raised={raised_lens_center}",
    )

    with ctx.pose({yoke_pan: 0.0}):
        rest_pan_box = ctx.part_element_world_aabb(can, elem="lens")
    with ctx.pose({yoke_pan: math.radians(55.0)}):
        panned_box = ctx.part_element_world_aabb(can, elem="lens")
    rest_pan_center = _aabb_center(rest_pan_box)
    panned_center = _aabb_center(panned_box)
    ctx.check(
        "yoke pans the can around the post",
        rest_pan_center is not None
        and panned_center is not None
        and panned_center[1] > rest_pan_center[1] + 0.18,
        details=f"rest={rest_pan_center}, panned={panned_center}",
    )

    return ctx.report()


object_model = build_object_model()
