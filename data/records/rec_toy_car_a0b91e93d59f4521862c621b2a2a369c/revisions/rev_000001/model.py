from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_LENGTH = 0.158
BODY_WIDTH = 0.062
BODY_HEIGHT = 0.046
WHEEL_RADIUS = 0.0165
WHEEL_WIDTH = 0.010
FRONT_AXLE_Y = 0.040
REAR_AXLE_Y = -0.041
WHEEL_CENTER_Z = WHEEL_RADIUS
SIDE_HINGE_X = 0.0296
DOOR_HINGE_Y = 0.023
DOOR_HINGE_Z = 0.022
HOOD_HINGE_Y = 0.018
HOOD_HINGE_Z = 0.0318


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _arch_cut(
    center: tuple[float, float, float],
    *,
    radius: float,
    width: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(width, radius)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _build_body_shell() -> cq.Workplane:
    front_left_fender = _cq_box((0.016, 0.034, 0.021), (0.0205, 0.042, 0.026)).cut(
        _arch_cut((0.0205, FRONT_AXLE_Y, WHEEL_CENTER_Z + 0.0006), radius=0.0175, width=0.030)
    )
    front_right_fender = _cq_box((0.016, 0.034, 0.021), (-0.0205, 0.042, 0.026)).cut(
        _arch_cut((-0.0205, FRONT_AXLE_Y, WHEEL_CENTER_Z + 0.0006), radius=0.0175, width=0.030)
    )
    rear_left_quarter = _cq_box((0.016, 0.048, 0.022), (0.0205, -0.047, 0.027)).cut(
        _arch_cut((0.0205, REAR_AXLE_Y, WHEEL_CENTER_Z + 0.0004), radius=0.0174, width=0.030)
    )
    rear_right_quarter = _cq_box((0.016, 0.048, 0.022), (-0.0205, -0.047, 0.027)).cut(
        _arch_cut((-0.0205, REAR_AXLE_Y, WHEEL_CENTER_Z + 0.0004), radius=0.0174, width=0.030)
    )

    body = _cq_box((0.056, 0.146, 0.008), (0.0, 0.0, 0.010))
    body = body.union(_cq_box((0.006, 0.122, 0.016), (0.024, -0.002, 0.018)))
    body = body.union(_cq_box((0.006, 0.122, 0.016), (-0.024, -0.002, 0.018)))
    body = body.union(front_left_fender)
    body = body.union(front_right_fender)
    body = body.union(rear_left_quarter)
    body = body.union(rear_right_quarter)
    body = body.union(_cq_box((0.050, 0.018, 0.016), (0.0, 0.066, 0.018)))
    body = body.union(_cq_box((0.044, 0.012, 0.010), (0.0, 0.019, 0.030)))
    body = body.union(_cq_box((0.046, 0.054, 0.007), (0.0, -0.007, 0.041)))
    body = body.union(_cq_box((0.050, 0.028, 0.007), (0.0, -0.056, 0.032)))
    body = body.union(_cq_box((0.052, 0.010, 0.016), (0.0, -0.073, 0.018)))
    body = body.union(
        _cq_box((0.004, 0.029, 0.018), (0.020, 0.008, 0.032)).rotate(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -53.0
        )
    )
    body = body.union(
        _cq_box((0.004, 0.029, 0.018), (-0.020, 0.008, 0.032)).rotate(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -53.0
        )
    )
    body = body.union(
        _cq_box((0.004, 0.024, 0.015), (0.020, -0.032, 0.031)).rotate(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 44.0
        )
    )
    body = body.union(
        _cq_box((0.004, 0.024, 0.015), (-0.020, -0.032, 0.031)).rotate(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 44.0
        )
    )
    return body


def _add_wheel_visuals(part, *, tire_mesh, wheel_mesh, tire_material, rim_material, hub_material) -> None:
    part.visual(
        tire_mesh,
        material=tire_material,
        name="tire",
    )
    part.visual(
        wheel_mesh,
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.0030, length=0.012),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="axle_cap",
    )
    part.visual(
        Cylinder(radius=0.0018, length=0.014),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="axle_bore",
    )


def _add_door_visuals(part, *, side_sign: float, paint, trim) -> None:
    part.visual(
        Box((0.0026, 0.046, 0.018)),
        origin=Origin(xyz=(side_sign * 0.0013, -0.023, -0.002)),
        material=paint,
        name="door_panel",
    )
    part.visual(
        Box((0.0024, 0.003, 0.018)),
        origin=Origin(xyz=(side_sign * 0.0012, -0.0020, 0.007)),
        material=paint,
        name="front_frame",
    )
    part.visual(
        Box((0.0024, 0.003, 0.016)),
        origin=Origin(xyz=(side_sign * 0.0012, -0.0440, 0.006)),
        material=paint,
        name="rear_frame",
    )
    part.visual(
        Box((0.0024, 0.040, 0.003)),
        origin=Origin(xyz=(side_sign * 0.0012, -0.022, 0.014)),
        material=paint,
        name="window_rail",
    )
    part.visual(
        Cylinder(radius=0.0012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.0016, 0.008, 0.002)),
        origin=Origin(xyz=(side_sign * 0.0020, -0.022, 0.001)),
        material=trim,
        name="handle",
    )


def _add_hood_visuals(part, *, paint) -> None:
    part.visual(
        Box((0.034, 0.040, 0.0024)),
        origin=Origin(xyz=(0.0, 0.020, 0.0012)),
        material=paint,
        name="hood_panel",
    )
    part.visual(
        Box((0.034, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.038, -0.001)),
        material=paint,
        name="front_lip",
    )
    part.visual(
        Box((0.030, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, 0.0015, -0.0006)),
        material=paint,
        name="hinge_strip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_coupe")

    body_paint = model.material("body_paint", rgba=(0.82, 0.12, 0.15, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    underbody = model.material("underbody", rgba=(0.18, 0.19, 0.21, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.26, 0.32, 0.55))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.77, 0.78, 0.80, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.34, 0.35, 0.37, 1.0))

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.0125,
            carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.0007, count=18, land_ratio=0.62),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.0012, radius=0.0008),
        ),
        "toy_coupe_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.0125,
            0.0075,
            rim=WheelRim(
                inner_radius=0.0096,
                flange_height=0.0014,
                flange_thickness=0.0008,
                bead_seat_depth=0.0006,
            ),
            hub=WheelHub(
                radius=0.0038,
                width=0.0062,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.0048,
                    hole_diameter=0.0007,
                ),
            ),
            face=WheelFace(dish_depth=0.0009, front_inset=0.0005, rear_inset=0.0004),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0009, window_radius=0.0018),
            bore=WheelBore(style="round", diameter=0.0018),
        ),
        "toy_coupe_wheel",
    )

    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body_shell(), "toy_coupe_body"), material=body_paint, name="shell")
    body.visual(
        Box((0.032, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.040, 0.019)),
        material=underbody,
        name="front_compartment",
    )
    body.visual(
        Box((0.038, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, -0.012, 0.018)),
        material=underbody,
        name="interior",
    )
    body.visual(
        Box((0.038, 0.026, 0.002)),
        origin=Origin(xyz=(0.0, 0.008, 0.034), rpy=(-0.96, 0.0, 0.0)),
        material=glass,
        name="windshield",
    )
    body.visual(
        Box((0.040, 0.021, 0.002)),
        origin=Origin(xyz=(0.0, -0.034, 0.034), rpy=(0.72, 0.0, 0.0)),
        material=glass,
        name="rear_window",
    )
    body.visual(
        Box((0.048, 0.0020, 0.0040)),
        origin=Origin(xyz=(0.0, -0.041, 0.0315)),
        material=dark_trim,
        name="trunk_seam",
    )
    for hinge_name, x_pos in (("right_hinge_post", 0.0272), ("left_hinge_post", -0.0272)):
        body.visual(
            Cylinder(radius=0.0012, length=0.018),
            origin=Origin(xyz=(x_pos, DOOR_HINGE_Y, DOOR_HINGE_Z)),
            material=dark_trim,
            name=hinge_name,
        )
    for wheel_name, x_pos, y_pos in (
        ("front_right_stub", 0.0265, FRONT_AXLE_Y),
        ("front_left_stub", -0.0265, FRONT_AXLE_Y),
        ("rear_right_stub", 0.0265, REAR_AXLE_Y),
        ("rear_left_stub", -0.0265, REAR_AXLE_Y),
    ):
        body.visual(
            Cylinder(radius=0.0020, length=0.007),
            origin=Origin(xyz=(x_pos, y_pos, WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_hub,
            name=wheel_name,
        )

    hood = model.part("hood")
    _add_hood_visuals(hood, paint=body_paint)

    left_door = model.part("left_door")
    _add_door_visuals(left_door, side_sign=-1.0, paint=body_paint, trim=wheel_silver)

    right_door = model.part("right_door")
    _add_door_visuals(right_door, side_sign=1.0, paint=body_paint, trim=wheel_silver)

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        _add_wheel_visuals(
            wheel,
            tire_mesh=tire_mesh,
            wheel_mesh=wheel_mesh,
            tire_material=wheel_rubber,
            rim_material=wheel_silver,
            hub_material=wheel_hub,
        )

    model.articulation(
        "hood_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.0, HOOD_HINGE_Y, HOOD_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=3.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(-SIDE_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.12, velocity=3.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(SIDE_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.12, velocity=3.0, lower=0.0, upper=1.10),
    )

    for wheel_name, joint_name, x_pos, y_pos in (
        ("front_left_wheel", "front_left_spin", -0.036, FRONT_AXLE_Y),
        ("front_right_wheel", "front_right_spin", 0.036, FRONT_AXLE_Y),
        ("rear_left_wheel", "rear_left_spin", -0.036, REAR_AXLE_Y),
        ("rear_right_wheel", "rear_right_spin", 0.036, REAR_AXLE_Y),
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_name,
            origin=Origin(xyz=(x_pos, y_pos, WHEEL_CENTER_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.10, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hood = object_model.get_part("hood")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")

    hood_hinge = object_model.get_articulation("hood_hinge")
    left_door_hinge = object_model.get_articulation("left_door_hinge")
    right_door_hinge = object_model.get_articulation("right_door_hinge")

    with ctx.pose({left_door_hinge: 0.0, right_door_hinge: 0.0}):
        ctx.expect_contact(
            right_door,
            body,
            elem_a="hinge_barrel",
            elem_b="right_hinge_post",
            name="right door is mounted on its hinge post",
        )
        ctx.expect_contact(
            left_door,
            body,
            elem_a="hinge_barrel",
            elem_b="left_hinge_post",
            name="left door is mounted on its hinge post",
        )

    hood_closed = ctx.part_world_aabb(hood)
    with ctx.pose({hood_hinge: 1.0}):
        hood_open = ctx.part_world_aabb(hood)
    ctx.check(
        "hood lifts upward",
        hood_closed is not None
        and hood_open is not None
        and hood_open[1][2] > hood_closed[1][2] + 0.014,
        details=f"closed={hood_closed}, open={hood_open}",
    )

    right_closed = ctx.part_world_aabb(right_door)
    with ctx.pose({right_door_hinge: 0.9}):
        right_open = ctx.part_world_aabb(right_door)
    ctx.check(
        "right door swings outward",
        right_closed is not None
        and right_open is not None
        and right_open[1][0] > right_closed[1][0] + 0.010,
        details=f"closed={right_closed}, open={right_open}",
    )

    left_closed = ctx.part_world_aabb(left_door)
    with ctx.pose({left_door_hinge: 0.9}):
        left_open = ctx.part_world_aabb(left_door)
    ctx.check(
        "left door swings outward",
        left_closed is not None
        and left_open is not None
        and left_open[0][0] < left_closed[0][0] - 0.010,
        details=f"closed={left_closed}, open={left_open}",
    )

    return ctx.report()


object_model = build_object_model()
