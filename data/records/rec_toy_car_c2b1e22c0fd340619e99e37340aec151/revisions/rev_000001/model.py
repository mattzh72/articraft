from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

BODY_LENGTH = 0.238
BODY_WIDTH = 0.096
ROOF_WIDTH = 0.082
WHEELBASE = 0.148
TRACK = 0.096
WHEEL_RADIUS = 0.0285
WHEEL_WIDTH = 0.020
DOOR_THICKNESS = 0.004


def _add_toy_wheel(part, mesh_prefix: str, wheel_finish, tire_finish) -> None:
    del mesh_prefix
    wheel_rotation = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=wheel_rotation,
        material=tire_finish,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.0185, length=WHEEL_WIDTH * 0.76),
        origin=wheel_rotation,
        material=wheel_finish,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.007, length=WHEEL_WIDTH * 0.82),
        origin=wheel_rotation,
        material=wheel_finish,
        name="hub",
    )
    part.visual(
        Box((0.024, 0.004, 0.003)),
        material=wheel_finish,
        name="spoke_x",
    )
    part.visual(
        Box((0.003, 0.004, 0.024)),
        material=wheel_finish,
        name="spoke_z",
    )


def _add_side_door(part, *, side_sign: float, length: float, height: float, body_finish, glass_finish, trim_finish) -> None:
    outer_bias = side_sign * 0.0011
    part.visual(
        Box((length, DOOR_THICKNESS, height)),
        origin=Origin(xyz=(-length * 0.5, 0.0, 0.0)),
        material=body_finish,
        name="panel",
    )
    part.visual(
        Box((length * 0.68, DOOR_THICKNESS * 0.70, height * 0.34)),
        origin=Origin(xyz=(-length * 0.52, outer_bias, height * 0.16)),
        material=glass_finish,
        name="window",
    )
    part.visual(
        Box((0.009, DOOR_THICKNESS * 0.65, 0.004)),
        origin=Origin(xyz=(-length * 0.63, outer_bias, -height * 0.08)),
        material=trim_finish,
        name="handle",
    )
    part.visual(
        Cylinder(radius=0.001, length=height * 0.78),
        origin=Origin(xyz=(-0.001, -side_sign * 0.002, 0.0)),
        material=trim_finish,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.001, 0.003, height * 0.68)),
        origin=Origin(xyz=(-0.0005, -side_sign * 0.003, 0.0)),
        material=trim_finish,
        name="hinge_leaf",
    )


def _add_hatch(part, *, body_finish, glass_finish, trim_finish) -> None:
    part.visual(
        Box((0.004, 0.068, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=body_finish,
        name="panel",
    )
    part.visual(
        Box((0.0018, 0.053, 0.016)),
        origin=Origin(xyz=(-0.0011, 0.0, -0.014)),
        material=glass_finish,
        name="window",
    )
    part.visual(
        Box((0.0018, 0.018, 0.004)),
        origin=Origin(xyz=(-0.0011, 0.0, -0.026)),
        material=trim_finish,
        name="handle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_suv")

    body_finish = model.material("body_finish", rgba=(0.83, 0.29, 0.13, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.20, 0.24, 0.28, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.77, 0.79, 0.81, 1.0))
    tire_finish = model.material("tire_finish", rgba=(0.07, 0.07, 0.08, 1.0))
    axle_finish = model.material("axle_finish", rgba=(0.28, 0.30, 0.33, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, 0.094)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
    )

    body.visual(Box((0.080, BODY_WIDTH * 0.92, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.047)), material=trim_finish, name="floor")
    body.visual(Box((0.076, 0.072, 0.026)), origin=Origin(xyz=(0.078, 0.0, 0.065)), material=body_finish, name="hood")
    body.visual(Box((0.118, ROOF_WIDTH, 0.016)), origin=Origin(xyz=(-0.005, 0.0, 0.086)), material=body_finish, name="roof")
    body.visual(Box((0.056, ROOF_WIDTH, 0.018)), origin=Origin(xyz=(0.082, 0.0, 0.085)), material=body_finish, name="windshield_header")
    body.visual(Box((0.052, ROOF_WIDTH, 0.016)), origin=Origin(xyz=(-0.086, 0.0, 0.086)), material=body_finish, name="hatch_header")
    body.visual(Box((0.102, 0.009, 0.026)), origin=Origin(xyz=(0.0, 0.036, 0.055)), material=body_finish, name="left_sill")
    body.visual(Box((0.102, 0.009, 0.026)), origin=Origin(xyz=(0.0, -0.036, 0.055)), material=body_finish, name="right_sill")

    for x_center, x_size, z_center, z_size, name_prefix in (
        (0.056, 0.018, 0.077, 0.034, "front"),
        (0.0, 0.012, 0.068, 0.052, "middle"),
        (-0.060, 0.018, 0.077, 0.034, "rear"),
    ):
        body.visual(
            Box((x_size, 0.009, z_size)),
            origin=Origin(xyz=(x_center, 0.0435, z_center)),
            material=body_finish,
            name=f"left_{name_prefix}_pillar",
        )
        body.visual(
            Box((x_size, 0.009, z_size)),
            origin=Origin(xyz=(x_center, -0.0435, z_center)),
            material=body_finish,
            name=f"right_{name_prefix}_pillar",
        )

    body.visual(Box((0.044, 0.010, 0.022)), origin=Origin(xyz=(-0.085, 0.0435, 0.068)), material=body_finish, name="left_rear_quarter")
    body.visual(Box((0.044, 0.010, 0.022)), origin=Origin(xyz=(-0.085, -0.0435, 0.068)), material=body_finish, name="right_rear_quarter")
    body.visual(Box((0.012, 0.020, 0.040)), origin=Origin(xyz=(-0.107, 0.044, 0.066)), material=body_finish, name="left_tail_post")
    body.visual(Box((0.012, 0.020, 0.040)), origin=Origin(xyz=(-0.107, -0.044, 0.066)), material=body_finish, name="right_tail_post")
    body.visual(Box((0.014, 0.070, 0.010)), origin=Origin(xyz=(-0.112, 0.0, 0.044)), material=trim_finish, name="rear_bumper")
    body.visual(Box((0.014, 0.074, 0.010)), origin=Origin(xyz=(0.112, 0.0, 0.044)), material=trim_finish, name="front_bumper")
    body.visual(Box((0.018, 0.056, 0.014)), origin=Origin(xyz=(0.104, 0.0, 0.054)), material=trim_finish, name="grille")

    for x_center, name_prefix in ((0.074, "front"), (-0.074, "rear")):
        body.visual(
            Box((0.050, 0.014, 0.026)),
            origin=Origin(xyz=(x_center, 0.066, 0.055)),
            material=body_finish,
            name=f"left_{name_prefix}_flare",
        )
        body.visual(
            Box((0.050, 0.014, 0.026)),
            origin=Origin(xyz=(x_center, -0.066, 0.055)),
            material=body_finish,
            name=f"right_{name_prefix}_flare",
        )

    body.visual(Box((0.034, 0.022, 0.010)), origin=Origin(xyz=(0.074, 0.056, 0.066)), material=body_finish, name="left_front_arch_top")
    body.visual(Box((0.034, 0.022, 0.010)), origin=Origin(xyz=(0.074, -0.056, 0.066)), material=body_finish, name="right_front_arch_top")
    body.visual(Box((0.036, 0.022, 0.010)), origin=Origin(xyz=(-0.074, 0.056, 0.066)), material=body_finish, name="left_rear_arch_top")
    body.visual(Box((0.036, 0.022, 0.010)), origin=Origin(xyz=(-0.074, -0.056, 0.066)), material=body_finish, name="right_rear_arch_top")

    body.visual(
        Box((0.002, 0.066, 0.030)),
        origin=Origin(xyz=(0.055, 0.0, 0.076), rpy=(0.0, -0.72, 0.0)),
        material=glass_finish,
        name="windshield",
    )
    body.visual(
        Box((0.002, 0.050, 0.026)),
        origin=Origin(xyz=(-0.083, 0.0, 0.075), rpy=(0.0, 0.88, 0.0)),
        material=glass_finish,
        name="rear_window",
    )
    body.visual(Box((0.016, 0.006, 0.008)), origin=Origin(xyz=(0.090, 0.034, 0.054)), material=trim_finish, name="left_headlamp")
    body.visual(Box((0.016, 0.006, 0.008)), origin=Origin(xyz=(0.090, -0.034, 0.054)), material=trim_finish, name="right_headlamp")

    front_axle = model.part("front_axle")
    front_axle.inertial = Inertial.from_geometry(
        Box((0.026, 0.090, 0.024)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )
    front_axle.visual(Box((0.012, 0.012, 0.010)), origin=Origin(xyz=(0.0, 0.0, -0.005)), material=axle_finish, name="pivot_pad")
    front_axle.visual(Box((0.012, 0.080, 0.010)), origin=Origin(xyz=(0.0, 0.0, -0.014)), material=axle_finish, name="beam")
    front_axle.visual(Box((0.020, 0.008, 0.010)), origin=Origin(xyz=(0.0, 0.032, -0.014)), material=axle_finish, name="left_stub")
    front_axle.visual(Box((0.020, 0.008, 0.010)), origin=Origin(xyz=(0.0, -0.032, -0.014)), material=axle_finish, name="right_stub")
    front_axle.visual(Box((0.010, 0.004, 0.016)), origin=Origin(xyz=(0.0, 0.035, -0.010)), material=axle_finish, name="left_knuckle")
    front_axle.visual(Box((0.010, 0.004, 0.016)), origin=Origin(xyz=(0.0, -0.035, -0.010)), material=axle_finish, name="right_knuckle")

    left_front_door = model.part("left_front_door")
    left_front_door.inertial = Inertial.from_geometry(Box((0.036, 0.006, 0.038)), mass=0.02)
    _add_side_door(
        left_front_door,
        side_sign=1.0,
        length=0.036,
        height=0.038,
        body_finish=body_finish,
        glass_finish=glass_finish,
        trim_finish=trim_finish,
    )

    right_front_door = model.part("right_front_door")
    right_front_door.inertial = Inertial.from_geometry(Box((0.036, 0.006, 0.038)), mass=0.02)
    _add_side_door(
        right_front_door,
        side_sign=-1.0,
        length=0.036,
        height=0.038,
        body_finish=body_finish,
        glass_finish=glass_finish,
        trim_finish=trim_finish,
    )

    left_rear_door = model.part("left_rear_door")
    left_rear_door.inertial = Inertial.from_geometry(Box((0.043, 0.006, 0.038)), mass=0.02)
    _add_side_door(
        left_rear_door,
        side_sign=1.0,
        length=0.043,
        height=0.038,
        body_finish=body_finish,
        glass_finish=glass_finish,
        trim_finish=trim_finish,
    )

    right_rear_door = model.part("right_rear_door")
    right_rear_door.inertial = Inertial.from_geometry(Box((0.043, 0.006, 0.038)), mass=0.02)
    _add_side_door(
        right_rear_door,
        side_sign=-1.0,
        length=0.043,
        height=0.038,
        body_finish=body_finish,
        glass_finish=glass_finish,
        trim_finish=trim_finish,
    )

    rear_hatch = model.part("rear_hatch")
    rear_hatch.inertial = Inertial.from_geometry(Box((0.006, 0.070, 0.032)), mass=0.025)
    _add_hatch(rear_hatch, body_finish=body_finish, glass_finish=glass_finish, trim_finish=trim_finish)

    left_front_wheel = model.part("left_front_wheel")
    left_front_wheel.inertial = Inertial.from_geometry(Box((0.024, 0.058, 0.058)), mass=0.03)
    _add_toy_wheel(left_front_wheel, "left_front_wheel", wheel_finish, tire_finish)

    right_front_wheel = model.part("right_front_wheel")
    right_front_wheel.inertial = Inertial.from_geometry(Box((0.024, 0.058, 0.058)), mass=0.03)
    _add_toy_wheel(right_front_wheel, "right_front_wheel", wheel_finish, tire_finish)

    left_rear_wheel = model.part("left_rear_wheel")
    left_rear_wheel.inertial = Inertial.from_geometry(Box((0.024, 0.058, 0.058)), mass=0.03)
    _add_toy_wheel(left_rear_wheel, "left_rear_wheel", wheel_finish, tire_finish)

    right_rear_wheel = model.part("right_rear_wheel")
    right_rear_wheel.inertial = Inertial.from_geometry(Box((0.024, 0.058, 0.058)), mass=0.03)
    _add_toy_wheel(right_rear_wheel, "right_rear_wheel", wheel_finish, tire_finish)

    model.articulation(
        "front_axle_steer",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_axle,
        origin=Origin(xyz=(WHEELBASE * 0.5, 0.0, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "left_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_axle,
        child=left_front_wheel,
        origin=Origin(xyz=(0.0, TRACK * 0.5, -0.0135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=40.0),
    )
    model.articulation(
        "right_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_axle,
        child=right_front_wheel,
        origin=Origin(xyz=(0.0, -TRACK * 0.5, -0.0135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=40.0),
    )
    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_rear_wheel,
        origin=Origin(xyz=(-WHEELBASE * 0.5, TRACK * 0.5, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=40.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_rear_wheel,
        origin=Origin(xyz=(-WHEELBASE * 0.5, -TRACK * 0.5, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=40.0),
    )

    model.articulation(
        "left_front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_front_door,
        origin=Origin(xyz=(0.047, 0.051, 0.062)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=3.0, lower=0.0, upper=1.22),
    )
    model.articulation(
        "right_front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_front_door,
        origin=Origin(xyz=(0.047, -0.051, 0.062)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=3.0, lower=0.0, upper=1.22),
    )
    model.articulation(
        "left_rear_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_rear_door,
        origin=Origin(xyz=(-0.006, 0.051, 0.062)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=3.0, lower=0.0, upper=1.22),
    )
    model.articulation(
        "right_rear_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_rear_door,
        origin=Origin(xyz=(-0.006, -0.051, 0.062)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=3.0, lower=0.0, upper=1.22),
    )
    model.articulation(
        "rear_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_hatch,
        origin=Origin(xyz=(-0.108, 0.0, 0.079)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=3.0, lower=0.0, upper=1.15),
    )

    return model


def _extent(aabb, axis: int, side: str) -> float | None:
    if aabb is None:
        return None
    return float(aabb[0][axis] if side == "min" else aabb[1][axis])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_front_door = object_model.get_part("left_front_door")
    right_front_door = object_model.get_part("right_front_door")
    rear_hatch = object_model.get_part("rear_hatch")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("toy_suv_length", 0.22 <= size[0] <= 0.25, f"body_size={size!r}")
        ctx.check("toy_suv_width", 0.09 <= size[1] <= 0.15, f"body_size={size!r}")
        ctx.check("toy_suv_height", 0.05 <= size[2] <= 0.06, f"body_size={size!r}")
    else:
        ctx.fail("toy_suv_body_aabb", "Expected world AABB for the body.")

    left_front_hinge = object_model.get_articulation("left_front_door_hinge")
    right_front_hinge = object_model.get_articulation("right_front_door_hinge")
    rear_hatch_hinge = object_model.get_articulation("rear_hatch_hinge")
    front_axle_steer = object_model.get_articulation("front_axle_steer")

    left_closed = ctx.part_element_world_aabb(left_front_door, elem="panel")
    right_closed = ctx.part_element_world_aabb(right_front_door, elem="panel")
    hatch_closed = ctx.part_element_world_aabb(rear_hatch, elem="panel")

    left_open = None
    if left_front_hinge.motion_limits is not None and left_front_hinge.motion_limits.upper is not None:
        with ctx.pose({left_front_hinge: left_front_hinge.motion_limits.upper}):
            left_open = ctx.part_element_world_aabb(left_front_door, elem="panel")
    ctx.check(
        "left_front_door_opens_outward",
        _extent(left_open, 1, "max") is not None
        and _extent(left_closed, 1, "max") is not None
        and _extent(left_open, 1, "max") > _extent(left_closed, 1, "max") + 0.018,
        details=f"closed={left_closed!r}, open={left_open!r}",
    )

    right_open = None
    if right_front_hinge.motion_limits is not None and right_front_hinge.motion_limits.upper is not None:
        with ctx.pose({right_front_hinge: right_front_hinge.motion_limits.upper}):
            right_open = ctx.part_element_world_aabb(right_front_door, elem="panel")
    ctx.check(
        "right_front_door_opens_outward",
        _extent(right_open, 1, "min") is not None
        and _extent(right_closed, 1, "min") is not None
        and _extent(right_open, 1, "min") < _extent(right_closed, 1, "min") - 0.018,
        details=f"closed={right_closed!r}, open={right_open!r}",
    )

    hatch_open = None
    if rear_hatch_hinge.motion_limits is not None and rear_hatch_hinge.motion_limits.upper is not None:
        with ctx.pose({rear_hatch_hinge: rear_hatch_hinge.motion_limits.upper}):
            hatch_open = ctx.part_element_world_aabb(rear_hatch, elem="panel")
    ctx.check(
        "rear_hatch_lifts_up_and_back",
        _extent(hatch_open, 0, "min") is not None
        and _extent(hatch_closed, 0, "min") is not None
        and _extent(hatch_open, 2, "min") is not None
        and _extent(hatch_closed, 2, "min") is not None
        and _extent(hatch_open, 0, "min") < _extent(hatch_closed, 0, "min") - 0.018
        and _extent(hatch_open, 2, "min") > _extent(hatch_closed, 2, "min") + 0.012,
        details=f"closed={hatch_closed!r}, open={hatch_open!r}",
    )

    left_front_rest = ctx.part_world_position(left_front_wheel)
    right_front_rest = ctx.part_world_position(right_front_wheel)
    left_front_steered = None
    right_front_steered = None
    if front_axle_steer.motion_limits is not None and front_axle_steer.motion_limits.upper is not None:
        with ctx.pose({front_axle_steer: front_axle_steer.motion_limits.upper}):
            left_front_steered = ctx.part_world_position(left_front_wheel)
            right_front_steered = ctx.part_world_position(right_front_wheel)
    ctx.check(
        "front_axle_yaws_wheel_pair",
        left_front_rest is not None
        and right_front_rest is not None
        and left_front_steered is not None
        and right_front_steered is not None
        and left_front_steered[0] < left_front_rest[0] - 0.008
        and right_front_steered[0] > right_front_rest[0] + 0.008,
        details=(
            f"left_rest={left_front_rest!r}, left_steered={left_front_steered!r}, "
            f"right_rest={right_front_rest!r}, right_steered={right_front_steered!r}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
