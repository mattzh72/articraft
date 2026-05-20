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
    WheelGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.062
BODY_HALF_WIDTH = BODY_WIDTH * 0.5
WHEEL_RADIUS = 0.018
WHEEL_WIDTH = 0.012
WHEEL_Y = 0.040
FRONT_WHEEL_X = 0.040
REAR_WHEEL_X = -0.050
WHEEL_Z = WHEEL_RADIUS

DOOR_HINGE_X = 0.022
DOOR_HINGE_Z = 0.036
REAR_HINGE_X = -0.054
REAR_HINGE_Z = 0.061
TRUNK_HINGE_X = -0.048
TRUNK_HINGE_Z = 0.047

OUTER_PROFILE = [
    (0.081, 0.011),
    (0.081, 0.024),
    (0.074, 0.032),
    (0.058, 0.038),
    (0.040, 0.040),
    (0.026, 0.054),
    (-0.010, 0.066),
    (-0.042, 0.064),
    (-0.054, 0.060),
    (-0.060, 0.052),
    (-0.050, 0.050),
    (-0.072, 0.048),
    (-0.080, 0.034),
    (-0.081, 0.011),
]

DOOR_PROFILE = [
    (0.000, -0.016),
    (0.000, 0.010),
    (-0.006, 0.018),
    (-0.022, 0.022),
    (-0.050, 0.021),
    (-0.050, -0.016),
]

DOOR_OPENING_PROFILE = [
    (0.001, -0.017),
    (0.001, 0.011),
    (-0.0065, 0.019),
    (-0.0225, 0.023),
    (-0.051, 0.022),
    (-0.051, -0.017),
]

LIFTGATE_PROFILE = [
    (0.000, 0.0018),
    (0.000, -0.0018),
    (-0.024, -0.026),
    (-0.021, -0.029),
]

TRUNK_PROFILE = [
    (0.000, 0.0016),
    (0.000, -0.0012),
    (-0.025, -0.0044),
    (-0.025, -0.0010),
]


def _profile_solid(points: list[tuple[float, float]], thickness: float, *, centered: bool = True):
    if centered:
        return cq.Workplane("XZ").polyline(points).close().extrude(thickness * 0.5, both=True)
    return cq.Workplane("XZ").polyline(points).close().extrude(thickness, both=False)


def _axle_stub(center_x: float, center_y: float) -> Origin:
    return Origin(xyz=(center_x, center_y, WHEEL_Z), rpy=(pi / 2.0, 0.0, 0.0))


def _body_shell_mesh():
    body = _profile_solid(OUTER_PROFILE, BODY_WIDTH)

    cabin_void = cq.Workplane("XY").box(0.090, 0.050, 0.026).translate((-0.014, 0.0, 0.042))
    windshield_cut = (
        cq.Workplane("XY")
        .box(0.070, 0.052, 0.040)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 24.0)
        .translate((0.032, 0.0, 0.056))
    )
    rear_window_cut = (
        cq.Workplane("XY")
        .box(0.050, 0.050, 0.034)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -42.0)
        .translate((-0.061, 0.0, 0.055))
    )

    left_door_cut = _profile_solid(DOOR_OPENING_PROFILE, 0.016).translate((DOOR_HINGE_X, BODY_HALF_WIDTH - 0.007, DOOR_HINGE_Z))
    right_door_cut = _profile_solid(DOOR_OPENING_PROFILE, 0.016).translate((DOOR_HINGE_X, -BODY_HALF_WIDTH + 0.007, DOOR_HINGE_Z))

    front_arch = cq.Workplane("XZ").center(FRONT_WHEEL_X, WHEEL_Z).circle(WHEEL_RADIUS + 0.0035).extrude(BODY_WIDTH + 0.020, both=True)
    rear_arch = cq.Workplane("XZ").center(REAR_WHEEL_X, WHEEL_Z).circle(WHEEL_RADIUS + 0.0035).extrude(BODY_WIDTH + 0.020, both=True)

    liftgate_cut = (
        cq.Workplane("XY")
        .box(0.012, 0.050, 0.016)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 18.0)
        .translate((-0.064, 0.0, 0.054))
    )
    trunk_cut = cq.Workplane("XY").box(0.028, 0.040, 0.007).translate((-0.060, 0.0, 0.0465))

    body = body.cut(cabin_void)
    body = body.cut(windshield_cut)
    body = body.cut(rear_window_cut)
    body = body.cut(left_door_cut)
    body = body.cut(right_door_cut)
    body = body.cut(front_arch)
    body = body.cut(rear_arch)
    body = body.cut(liftgate_cut)
    body = body.cut(trunk_cut)
    return mesh_from_cadquery(body, "toy_hatchback_body")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_hatchback_car")

    body_paint = model.material("body_paint", rgba=(0.80, 0.14, 0.10, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.36, 0.50, 0.62, 0.38))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.78, 0.79, 0.81, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.45, 0.46, 0.48, 1.0))
    lamp_red = model.material("lamp_red", rgba=(0.76, 0.14, 0.18, 1.0))
    lamp_clear = model.material("lamp_clear", rgba=(0.86, 0.88, 0.86, 0.75))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_paint, name="body_shell")
    body.visual(
        Box((0.030, 0.050, 0.0018)),
        origin=Origin(xyz=(0.026, 0.0, 0.051), rpy=(0.0, 0.72, 0.0)),
        material=glass,
        name="windshield",
    )
    body.visual(
        Box((0.012, 0.024, 0.003)),
        origin=Origin(xyz=(0.077, 0.019, 0.027)),
        material=lamp_clear,
        name="front_lamp_left",
    )
    body.visual(
        Box((0.012, 0.024, 0.003)),
        origin=Origin(xyz=(0.077, -0.019, 0.027)),
        material=lamp_clear,
        name="front_lamp_right",
    )
    body.visual(
        Box((0.004, 0.018, 0.010)),
        origin=Origin(xyz=(-0.079, 0.018, 0.030)),
        material=lamp_red,
        name="rear_lamp_left",
    )
    body.visual(
        Box((0.004, 0.018, 0.010)),
        origin=Origin(xyz=(-0.079, -0.018, 0.030)),
        material=lamp_red,
        name="rear_lamp_right",
    )
    body.visual(
        Cylinder(radius=0.0025, length=0.012),
        origin=Origin(xyz=(FRONT_WHEEL_X, 0.030, WHEEL_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="front_left_axle",
    )
    body.visual(
        Cylinder(radius=0.0025, length=0.012),
        origin=Origin(xyz=(FRONT_WHEEL_X, -0.030, WHEEL_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="front_right_axle",
    )
    body.visual(
        Cylinder(radius=0.0025, length=0.012),
        origin=Origin(xyz=(REAR_WHEEL_X, 0.030, WHEEL_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="rear_left_axle",
    )
    body.visual(
        Cylinder(radius=0.0025, length=0.012),
        origin=Origin(xyz=(REAR_WHEEL_X, -0.030, WHEEL_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="rear_right_axle",
    )
    body.visual(
        Cylinder(radius=0.0014, length=0.020),
        origin=Origin(xyz=(DOOR_HINGE_X, BODY_HALF_WIDTH, DOOR_HINGE_Z)),
        material=trim_black,
        name="left_hinge_pin",
    )
    body.visual(
        Cylinder(radius=0.0014, length=0.020),
        origin=Origin(xyz=(DOOR_HINGE_X, -BODY_HALF_WIDTH, DOOR_HINGE_Z)),
        material=trim_black,
        name="right_hinge_pin",
    )
    body.visual(
        Cylinder(radius=0.0012, length=0.030),
        origin=Origin(xyz=(REAR_HINGE_X, 0.0, REAR_HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="liftgate_pin",
    )
    body.visual(
        Cylinder(radius=0.0012, length=0.024),
        origin=Origin(xyz=(TRUNK_HINGE_X, 0.0, TRUNK_HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="trunk_pin",
    )

    door_mesh = mesh_from_cadquery(_profile_solid(DOOR_PROFILE, 0.004), "toy_side_door")
    left_door = model.part("left_door")
    left_door.visual(door_mesh, material=body_paint, name="door_shell")
    left_door.visual(
        Box((0.034, 0.0018, 0.015)),
        origin=Origin(xyz=(-0.028, 0.0, 0.0115)),
        material=glass,
        name="door_glass",
    )
    left_door.visual(
        Box((0.008, 0.0022, 0.0025)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0005)),
        material=trim_black,
        name="door_handle",
    )
    left_door.visual(
        Cylinder(radius=0.0018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_black,
        name="hinge_sleeve",
    )

    right_door = model.part("right_door")
    right_door.visual(door_mesh, material=body_paint, name="door_shell")
    right_door.visual(
        Box((0.034, 0.0018, 0.015)),
        origin=Origin(xyz=(-0.028, 0.0, 0.0115)),
        material=glass,
        name="door_glass",
    )
    right_door.visual(
        Box((0.008, 0.0022, 0.0025)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0005)),
        material=trim_black,
        name="door_handle",
    )
    right_door.visual(
        Cylinder(radius=0.0018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_black,
        name="hinge_sleeve",
    )

    liftgate = model.part("liftgate")
    liftgate.visual(
        Box((0.003, 0.046, 0.014)),
        origin=Origin(xyz=(-0.007, 0.0, -0.007), rpy=(0.0, 0.32, 0.0)),
        material=body_paint,
        name="gate_shell",
    )
    liftgate.visual(
        Box((0.0015, 0.040, 0.010)),
        origin=Origin(xyz=(-0.0075, 0.0, -0.0075), rpy=(0.0, 0.32, 0.0)),
        material=glass,
        name="gate_glass",
    )
    liftgate.visual(
        Cylinder(radius=0.0018, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="hinge_sleeve",
    )

    trunk_lid = model.part("trunk_lid")
    trunk_lid.visual(
        Box((0.024, 0.038, 0.003)),
        origin=Origin(xyz=(-0.012, 0.0, -0.0015), rpy=(0.0, 0.08, 0.0)),
        material=body_paint,
        name="lid_shell",
    )
    trunk_lid.visual(
        Box((0.012, 0.014, 0.0015)),
        origin=Origin(xyz=(-0.017, 0.0, -0.0006)),
        material=trim_black,
        name="lid_handle",
    )
    trunk_lid.visual(
        Cylinder(radius=0.0018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="hinge_sleeve",
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(outer_radius=WHEEL_RADIUS, width=WHEEL_WIDTH, inner_radius=0.0138),
        "toy_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(radius=0.0145, width=0.010),
        "toy_wheel",
    )

    wheel_positions = {
        "front_left_wheel": (FRONT_WHEEL_X, WHEEL_Y, WHEEL_Z),
        "front_right_wheel": (FRONT_WHEEL_X, -WHEEL_Y, WHEEL_Z),
        "rear_left_wheel": (REAR_WHEEL_X, WHEEL_Y, WHEEL_Z),
        "rear_right_wheel": (REAR_WHEEL_X, -WHEEL_Y, WHEEL_Z),
    }
    for wheel_name in wheel_positions:
        wheel = model.part(wheel_name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=tire_rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=wheel_silver,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.0042, length=0.008),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=axle_steel,
            name="hub",
        )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(DOOR_HINGE_X, BODY_HALF_WIDTH + 0.0002, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=0.0, upper=1.30),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(DOOR_HINGE_X, -BODY_HALF_WIDTH - 0.0002, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=0.0, upper=1.30),
    )
    model.articulation(
        "liftgate_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=liftgate,
        origin=Origin(xyz=(REAR_HINGE_X, 0.0, REAR_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "trunk_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk_lid,
        origin=Origin(xyz=(TRUNK_HINGE_X, 0.0, TRUNK_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=0.0, upper=1.05),
    )

    for wheel_name, (x_pos, y_pos, z_pos) in wheel_positions.items():
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_name,
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    liftgate = object_model.get_part("liftgate")
    trunk_lid = object_model.get_part("trunk_lid")

    left_door_hinge = object_model.get_articulation("left_door_hinge")
    right_door_hinge = object_model.get_articulation("right_door_hinge")
    liftgate_hinge = object_model.get_articulation("liftgate_hinge")
    trunk_lid_hinge = object_model.get_articulation("trunk_lid_hinge")

    ctx.allow_overlap(
        body,
        left_door,
        elem_a="left_hinge_pin",
        elem_b="hinge_sleeve",
        reason="The left toy door rotates around a compact hinge pin captured inside the door sleeve.",
    )
    ctx.allow_overlap(
        body,
        right_door,
        elem_a="right_hinge_pin",
        elem_b="hinge_sleeve",
        reason="The right toy door rotates around a compact hinge pin captured inside the door sleeve.",
    )
    ctx.allow_overlap(
        body,
        liftgate,
        elem_a="liftgate_pin",
        elem_b="hinge_sleeve",
        reason="The upper split rear hatch wraps around a roof-mounted hinge pin.",
    )
    ctx.allow_overlap(
        body,
        trunk_lid,
        elem_a="trunk_pin",
        elem_b="hinge_sleeve",
        reason="The trunk lid pivots around a short rear-deck hinge pin.",
    )

    ctx.expect_overlap(left_door, body, axes="xz", min_overlap=0.028, name="left door covers side opening")
    ctx.expect_overlap(right_door, body, axes="xz", min_overlap=0.028, name="right door covers side opening")
    ctx.expect_overlap(liftgate, body, axes="yz", min_overlap=0.010, name="liftgate covers rear opening")
    ctx.expect_overlap(trunk_lid, body, axes="xy", min_overlap=0.022, name="trunk lid covers rear deck opening")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    left_closed = _aabb_center(ctx.part_element_world_aabb(left_door, elem="door_shell"))
    right_closed = _aabb_center(ctx.part_element_world_aabb(right_door, elem="door_shell"))
    gate_closed = _aabb_center(ctx.part_element_world_aabb(liftgate, elem="gate_shell"))
    trunk_closed = _aabb_center(ctx.part_element_world_aabb(trunk_lid, elem="lid_shell"))

    with ctx.pose({left_door_hinge: 1.10}):
        left_open = _aabb_center(ctx.part_element_world_aabb(left_door, elem="door_shell"))
    with ctx.pose({right_door_hinge: 1.10}):
        right_open = _aabb_center(ctx.part_element_world_aabb(right_door, elem="door_shell"))
    with ctx.pose({liftgate_hinge: 1.00}):
        gate_open = _aabb_center(ctx.part_element_world_aabb(liftgate, elem="gate_shell"))
    with ctx.pose({trunk_lid_hinge: 0.85}):
        trunk_open = _aabb_center(ctx.part_element_world_aabb(trunk_lid, elem="lid_shell"))

    ctx.check(
        "left door swings outward",
        left_closed is not None and left_open is not None and left_open[1] > left_closed[1] + 0.010,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right door swings outward",
        right_closed is not None and right_open is not None and right_open[1] < right_closed[1] - 0.010,
        details=f"closed={right_closed}, open={right_open}",
    )
    ctx.check(
        "liftgate opens upward",
        gate_closed is not None and gate_open is not None and gate_open[2] > gate_closed[2] + 0.008,
        details=f"closed={gate_closed}, open={gate_open}",
    )
    ctx.check(
        "trunk lid opens upward",
        trunk_closed is not None and trunk_open is not None and trunk_open[2] > trunk_closed[2] + 0.006,
        details=f"closed={trunk_closed}, open={trunk_open}",
    )

    return ctx.report()


object_model = build_object_model()
