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
    mesh_from_cadquery,
)

BODY_LENGTH = 0.34
BODY_WIDTH = 0.164
ROOF_WIDTH = 0.140
OVERALL_HEIGHT = 0.147
WHEEL_RADIUS = 0.043
WHEEL_WIDTH = 0.032
TRACK_Y = 0.091
FRONT_AXLE_X = 0.104
REAR_AXLE_X = -0.103
WHEEL_CENTER_Z = 0.043

DOOR_THICKNESS = 0.008


def _box(length: float, width: float, height: float, x: float, y: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False)).translate((x, y, z0))


def _arch_cut(x_pos: float, y_pos: float, radius: float, depth: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_pos, WHEEL_CENTER_Z)
        .circle(radius)
        .extrude(depth, both=True)
        .translate((0.0, y_pos, 0.0))
    )


def _body_shell_mesh():
    body = _box(BODY_LENGTH, BODY_WIDTH, 0.060, 0.0, 0.0, 0.020)
    body = body.union(_box(0.190, ROOF_WIDTH, 0.055, -0.006, 0.0, 0.080))
    body = body.union(_box(0.115, BODY_WIDTH, 0.028, 0.103, 0.0, 0.080))
    body = body.union(_box(0.074, ROOF_WIDTH, 0.028, -0.131, 0.0, 0.080))
    body = body.union(_box(0.022, 0.146, 0.038, 0.159, 0.0, 0.020))
    body = body.union(_box(0.020, 0.146, 0.034, -0.160, 0.0, 0.020))
    body = body.union(_box(0.100, 0.028, 0.055, FRONT_AXLE_X, 0.084, 0.020))
    body = body.union(_box(0.100, 0.028, 0.055, FRONT_AXLE_X, -0.084, 0.020))
    body = body.union(_box(0.098, 0.028, 0.055, REAR_AXLE_X, 0.084, 0.020))
    body = body.union(_box(0.098, 0.028, 0.055, REAR_AXLE_X, -0.084, 0.020))
    body = body.union(_box(0.090, 0.018, 0.018, 0.082, 0.053, 0.080))
    body = body.union(_box(0.090, 0.018, 0.018, 0.082, -0.053, 0.080))
    body = body.union(_box(0.034, 0.016, 0.020, 0.102, 0.060, 0.070))
    body = body.union(_box(0.034, 0.016, 0.020, 0.102, -0.060, 0.070))
    body = body.union(_box(0.032, 0.064, 0.014, 0.100, 0.0, 0.058))
    body = body.union(_box(0.050, 0.010, 0.018, 0.096, 0.048, 0.074))
    body = body.union(_box(0.050, 0.010, 0.018, 0.096, -0.048, 0.074))

    body = body.cut(_box(0.246, 0.114, 0.070, -0.010, 0.0, 0.048))
    body = body.cut(_box(0.272, 0.126, 0.032, -0.006, 0.0, 0.020))
    body = body.cut(_box(0.096, 0.088, 0.030, 0.093, 0.0, 0.072))
    body = body.cut(_box(0.092, 0.084, 0.020, 0.092, 0.0, 0.092))

    windshield = (
        cq.Workplane("XY")
        .box(0.080, 0.116, 0.066, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 25.0)
        .translate((0.072, 0.0, 0.108))
    )
    body = body.cut(windshield)

    for y_pos in (0.082, -0.082):
        body = body.cut(_box(0.082, 0.054, 0.082, 0.070, y_pos, 0.046))
        body = body.cut(_box(0.086, 0.054, 0.080, -0.038, y_pos, 0.046))

    body = body.cut(_box(0.020, 0.112, 0.090, -0.156, 0.0, 0.036))
    body = body.cut(_arch_cut(FRONT_AXLE_X, 0.087, 0.052, 0.052))
    body = body.cut(_arch_cut(FRONT_AXLE_X, -0.087, 0.052, 0.052))
    body = body.cut(_arch_cut(REAR_AXLE_X, 0.087, 0.052, 0.052))
    body = body.cut(_arch_cut(REAR_AXLE_X, -0.087, 0.052, 0.052))
    body = body.combine()

    return body


def _underbody_mesh():
    pan = _box(0.252, 0.128, 0.010, -0.004, 0.0, 0.010)
    pan = pan.union(_box(0.022, 0.142, 0.012, FRONT_AXLE_X, 0.0, 0.018))
    pan = pan.union(_box(0.022, 0.142, 0.012, REAR_AXLE_X, 0.0, 0.018))
    pan = pan.union(_box(0.022, 0.022, 0.028, FRONT_AXLE_X, 0.062, 0.018))
    pan = pan.union(_box(0.022, 0.022, 0.028, FRONT_AXLE_X, -0.062, 0.018))
    pan = pan.union(_box(0.022, 0.022, 0.028, REAR_AXLE_X, 0.062, 0.018))
    pan = pan.union(_box(0.022, 0.022, 0.028, REAR_AXLE_X, -0.062, 0.018))
    return pan.combine()


def _add_wheel(
    model: ArticulatedObject,
    *,
    name: str,
    x_pos: float,
    y_pos: float,
) -> None:
    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber",
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.68, length=WHEEL_WIDTH * 0.82),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="rim",
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.24, length=0.036),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="rim",
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.13, length=WHEEL_WIDTH * 0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="trim_black",
        name="cap",
    )
    model.articulation(
        f"{name}_spin",
        ArticulationType.CONTINUOUS,
        parent="underbody",
        child=wheel,
        origin=Origin(xyz=(x_pos, y_pos, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )


def _add_side_door(
    model: ArticulatedObject,
    *,
    name: str,
    hinge_name: str,
    hinge_xyz: tuple[float, float, float],
    axis: tuple[float, float, float],
    length: float,
    height: float,
    side_sign: float,
    window_length: float,
    handle_x: float,
    add_hinge_arm: bool = True,
) -> None:
    door = model.part(name)
    door.visual(
        Box((length, DOOR_THICKNESS, height)),
        origin=Origin(xyz=(-length / 2.0, -side_sign * DOOR_THICKNESS / 2.0, height / 2.0)),
        material="body_paint",
        name="panel",
    )
    door.visual(
        Box((window_length, DOOR_THICKNESS * 0.45, height * 0.34)),
        origin=Origin(
            xyz=(
                -length * 0.53,
                -side_sign * (DOOR_THICKNESS * 0.18),
                height * 0.70,
            )
        ),
        material="trim_black",
        name="window",
    )
    door.visual(
        Box((0.012, DOOR_THICKNESS * 0.30, 0.006)),
        origin=Origin(
            xyz=(
                handle_x,
                -side_sign * (DOOR_THICKNESS * 0.20),
                height * 0.42,
            )
        ),
        material="rim",
        name="handle",
    )
    door.visual(
        Cylinder(radius=0.004, length=height * 0.70),
        origin=Origin(xyz=(0.0, -side_sign * 0.006, height * 0.50)),
        material="trim_black",
        name="hinge_pin",
    )
    if add_hinge_arm:
        door.visual(
            Box((0.014, 0.006, height * 0.34)),
            origin=Origin(xyz=(0.007, -side_sign * 0.007, height * 0.48)),
            material="trim_black",
            name="hinge_arm",
        )
    model.articulation(
        hinge_name,
        ArticulationType.REVOLUTE,
        parent="body",
        child=door,
        origin=Origin(xyz=hinge_xyz),
        axis=axis,
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.35),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_suv")

    model.material("body_paint", rgba=(0.84, 0.22, 0.14, 1.0))
    model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("rim", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shell_mesh(), "body_shell"), material="body_paint", name="shell")
    body.visual(
        Box((0.040, 0.100, 0.022)),
        origin=Origin(xyz=(0.144, 0.0, 0.052)),
        material="trim_black",
        name="grille",
    )
    body.visual(
        Box((0.028, 0.120, 0.012)),
        origin=Origin(xyz=(0.162, 0.0, 0.026)),
        material="trim_black",
        name="front_bumper",
    )
    underbody = model.part("underbody")
    underbody.visual(mesh_from_cadquery(_underbody_mesh(), "underbody"), material="trim_black", name="tray")
    model.articulation(
        "body_to_underbody",
        ArticulationType.FIXED,
        parent="body",
        child=underbody,
        origin=Origin(),
    )

    _add_side_door(
        model,
        name="front_left_door",
        hinge_name="body_to_front_left_door",
        hinge_xyz=(0.060, 0.106, 0.046),
        axis=(0.0, 0.0, -1.0),
        length=0.050,
        height=0.078,
        side_sign=1.0,
        window_length=0.036,
        handle_x=-0.034,
        add_hinge_arm=False,
    )
    _add_side_door(
        model,
        name="front_right_door",
        hinge_name="body_to_front_right_door",
        hinge_xyz=(0.060, -0.106, 0.046),
        axis=(0.0, 0.0, 1.0),
        length=0.050,
        height=0.078,
        side_sign=-1.0,
        window_length=0.036,
        handle_x=-0.034,
        add_hinge_arm=False,
    )
    _add_side_door(
        model,
        name="rear_left_door",
        hinge_name="body_to_rear_left_door",
        hinge_xyz=(0.005, 0.102, 0.046),
        axis=(0.0, 0.0, -1.0),
        length=0.060,
        height=0.076,
        side_sign=1.0,
        window_length=0.040,
        handle_x=-0.042,
    )
    _add_side_door(
        model,
        name="rear_right_door",
        hinge_name="body_to_rear_right_door",
        hinge_xyz=(0.005, -0.102, 0.046),
        axis=(0.0, 0.0, 1.0),
        length=0.060,
        height=0.076,
        side_sign=-1.0,
        window_length=0.040,
        handle_x=-0.042,
    )

    hood = model.part("hood")
    hood.visual(
        Box((0.088, 0.080, 0.008)),
        origin=Origin(xyz=(-0.044, 0.0, 0.004)),
        material="body_paint",
        name="panel",
    )
    hood.visual(
        Box((0.040, 0.010, 0.004)),
        origin=Origin(xyz=(-0.060, 0.0, 0.008)),
        material="trim_black",
        name="scoop",
    )
    model.articulation(
        "body_to_hood",
        ArticulationType.REVOLUTE,
        parent="body",
        child=hood,
        origin=Origin(xyz=(0.138, 0.0, 0.104)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.15),
    )

    hatch = model.part("hatch")
    hatch.visual(
        Box((0.008, 0.106, 0.086)),
        origin=Origin(xyz=(0.012, 0.0, -0.044), rpy=(0.0, 0.10, 0.0)),
        material="body_paint",
        name="panel",
    )
    hatch.visual(
        Box((0.003, 0.086, 0.040)),
        origin=Origin(xyz=(0.014, 0.0, -0.031), rpy=(0.0, 0.10, 0.0)),
        material="trim_black",
        name="window",
    )
    model.articulation(
        "body_to_hatch",
        ArticulationType.REVOLUTE,
        parent="body",
        child=hatch,
        origin=Origin(xyz=(-0.162, 0.0, 0.126)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.20),
    )

    _add_wheel(model, name="front_left_wheel", x_pos=FRONT_AXLE_X, y_pos=TRACK_Y)
    _add_wheel(model, name="front_right_wheel", x_pos=FRONT_AXLE_X, y_pos=-TRACK_Y)
    _add_wheel(model, name="rear_left_wheel", x_pos=REAR_AXLE_X, y_pos=TRACK_Y)
    _add_wheel(model, name="rear_right_wheel", x_pos=REAR_AXLE_X, y_pos=-TRACK_Y)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_left_door = object_model.get_part("front_left_door")
    front_right_door = object_model.get_part("front_right_door")
    rear_left_door = object_model.get_part("rear_left_door")
    rear_right_door = object_model.get_part("rear_right_door")
    hood = object_model.get_part("hood")
    hatch = object_model.get_part("hatch")
    body = object_model.get_part("body")

    front_left_hinge = object_model.get_articulation("body_to_front_left_door")
    front_right_hinge = object_model.get_articulation("body_to_front_right_door")
    rear_left_hinge = object_model.get_articulation("body_to_rear_left_door")
    rear_right_hinge = object_model.get_articulation("body_to_rear_right_door")
    hood_hinge = object_model.get_articulation("body_to_hood")
    hatch_hinge = object_model.get_articulation("body_to_hatch")

    ctx.allow_overlap(
        "body",
        "underbody",
        elem_a="shell",
        elem_b="tray",
        reason="The black toy underbody tray is intentionally represented as nesting into the simplified die-cast shell cavity.",
    )

    for door_name in (
        "front_left_door",
        "front_right_door",
        "rear_left_door",
        "rear_right_door",
    ):
        ctx.allow_overlap(
            "body",
            door_name,
            reason="The toy body is represented as a robust solid casting proxy, so the closed outer door skin is intentionally allowed to nest into that simplified shell.",
        )
    ctx.allow_overlap(
        "body",
        "hatch",
        reason="The rear hatch skin is intentionally allowed to nest into the simplified one-piece toy shell proxy when closed.",
    )

    rest_fl = ctx.part_element_world_aabb(front_left_door, elem="panel")
    rest_fr = ctx.part_element_world_aabb(front_right_door, elem="panel")
    rest_rl = ctx.part_element_world_aabb(rear_left_door, elem="panel")
    rest_rr = ctx.part_element_world_aabb(rear_right_door, elem="panel")
    rest_hood = ctx.part_element_world_aabb(hood, elem="panel")
    rest_hatch = ctx.part_element_world_aabb(hatch, elem="panel")

    with ctx.pose({front_left_hinge: 1.15, front_right_hinge: 1.15, rear_left_hinge: 1.15, rear_right_hinge: 1.15}):
        open_fl = ctx.part_element_world_aabb(front_left_door, elem="panel")
        open_fr = ctx.part_element_world_aabb(front_right_door, elem="panel")
        open_rl = ctx.part_element_world_aabb(rear_left_door, elem="panel")
        open_rr = ctx.part_element_world_aabb(rear_right_door, elem="panel")

    with ctx.pose({hood_hinge: 0.95}):
        open_hood = ctx.part_element_world_aabb(hood, elem="panel")

    with ctx.pose({hatch_hinge: 0.95}):
        open_hatch = ctx.part_element_world_aabb(hatch, elem="panel")

    def _center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) * 0.5

    def _center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) * 0.5

    ctx.check(
        "left doors swing outward",
        rest_fl is not None
        and open_fl is not None
        and rest_rl is not None
        and open_rl is not None
        and _center_y(open_fl) > _center_y(rest_fl) + 0.010
        and _center_y(open_rl) > _center_y(rest_rl) + 0.010,
        details=f"front_left={rest_fl}->{open_fl}, rear_left={rest_rl}->{open_rl}",
    )
    ctx.check(
        "right doors swing outward",
        rest_fr is not None
        and open_fr is not None
        and rest_rr is not None
        and open_rr is not None
        and _center_y(open_fr) < _center_y(rest_fr) - 0.010
        and _center_y(open_rr) < _center_y(rest_rr) - 0.010,
        details=f"front_right={rest_fr}->{open_fr}, rear_right={rest_rr}->{open_rr}",
    )
    ctx.check(
        "hood opens upward",
        rest_hood is not None
        and open_hood is not None
        and _center_z(open_hood) > _center_z(rest_hood) + 0.018,
        details=f"hood={rest_hood}->{open_hood}",
    )
    ctx.check(
        "hatch opens upward",
        rest_hatch is not None
        and open_hatch is not None
        and _center_z(open_hatch) > _center_z(rest_hatch) + 0.018,
        details=f"hatch={rest_hatch}->{open_hatch}",
    )

    return ctx.report()


object_model = build_object_model()
