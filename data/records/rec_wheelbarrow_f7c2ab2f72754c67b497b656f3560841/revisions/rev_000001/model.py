from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
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
    mesh_from_geometry,
)


WHEEL_CENTER = (-0.62, 0.0, 0.19)
WHEEL_RADIUS = 0.18
WHEEL_WIDTH = 0.075


def _cylinder_between(start, end, radius: float) -> tuple[Cylinder, Origin]:
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")
    ux, uy, uz = vx / length, vy / length, vz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return (
        Cylinder(radius=radius, length=length),
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
    )


def _add_tube(part, start, end, radius: float, material: Material, name: str) -> None:
    geometry, origin = _cylinder_between(start, end, radius)
    part.visual(geometry, origin=origin, material=material, name=name)


def _add_weld(part, xyz, radius: float, material: Material, name: str) -> None:
    part.visual(Sphere(radius=radius), origin=Origin(xyz=xyz), material=material, name=name)


def _tray_shell_geometry() -> MeshGeometry:
    """Thin, open-topped tapered wheelbarrow tray with a real interior cavity."""

    geom = MeshGeometry()
    wall = 0.030
    z0 = 0.280
    z_floor = z0 + wall
    z1 = 0.620
    bottom = (0.760, 0.340)
    top = (0.980, 0.620)
    inner_bottom = (bottom[0] - (2.0 * wall), bottom[1] - (2.0 * wall))
    inner_top = (top[0] - (2.0 * wall), top[1] - (2.0 * wall))

    def loop(width: float, depth: float, z: float) -> list[int]:
        hx, hy = width * 0.5, depth * 0.5
        return [
            geom.add_vertex(-hx, -hy, z),
            geom.add_vertex(hx, -hy, z),
            geom.add_vertex(hx, hy, z),
            geom.add_vertex(-hx, hy, z),
        ]

    outer_bottom = loop(*bottom, z0)
    inner_floor = loop(*inner_bottom, z_floor)
    outer_top = loop(*top, z1)
    inner_top_loop = loop(*inner_top, z1 - (wall * 0.25))

    def tri(a: int, b: int, c: int) -> None:
        geom.add_face(a, b, c)

    def quad(a: int, b: int, c: int, d: int) -> None:
        tri(a, b, c)
        tri(a, c, d)

    for i in range(4):
        j = (i + 1) % 4
        # Exterior tapered walls.
        quad(outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        # Rolled lip around the open top.
        quad(outer_top[i], outer_top[j], inner_top_loop[j], inner_top_loop[i])
        # Inner tapered walls.
        quad(inner_top_loop[j], inner_top_loop[i], inner_floor[i], inner_floor[j])
        # Sloped bottom thickness tying the inside floor to the underside.
        quad(outer_bottom[j], outer_bottom[i], inner_floor[i], inner_floor[j])

    # Interior floor and exterior underside.
    tri(inner_floor[0], inner_floor[1], inner_floor[2])
    tri(inner_floor[0], inner_floor[2], inner_floor[3])
    tri(outer_bottom[0], outer_bottom[3], outer_bottom[2])
    tri(outer_bottom[0], outer_bottom[2], outer_bottom[1])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_wheel_wheelbarrow")

    tray_green = model.material("painted_green", rgba=(0.12, 0.42, 0.18, 1.0))
    frame_finish = model.material("dark_galvanized_steel", rgba=(0.26, 0.29, 0.30, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.025, 0.024, 0.023, 1.0))
    rim_finish = model.material("dull_silver_rim", rgba=(0.72, 0.74, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_tray_shell_geometry(), "open_tapered_tray"),
        material=tray_green,
        name="hollow_tray",
    )

    rail_r = 0.014
    cross_r = 0.015
    fork_r = 0.012

    for suffix, y in (("0", -0.25), ("1", 0.25)):
        _add_tube(body, (-0.27, y, 0.265), (0.46, y, 0.305), rail_r, frame_finish, f"side_rail_{suffix}")
        _add_tube(body, (0.46, y, 0.305), (0.94, y, 0.520), rail_r, frame_finish, f"handle_rail_{suffix}")
        _add_tube(body, (0.46, y, 0.305), (0.56, y, 0.035), rail_r, frame_finish, f"resting_leg_{suffix}")
        body.visual(
            Box((0.18, 0.060, 0.045)),
            origin=Origin(xyz=(0.60, y, 0.0225)),
            material=frame_finish,
            name=f"leg_foot_{suffix}",
        )
        _add_tube(body, (0.78, y, 0.450), (0.94, y, 0.520), 0.010, frame_finish, f"handle_gusset_{suffix}")
        _add_tube(body, (0.82, y, 0.520), (1.04, y, 0.520), 0.025, rubber, f"rubber_grip_{suffix}")
        _add_weld(body, (-0.27, y, 0.265), 0.018, frame_finish, f"front_rail_node_{suffix}")
        _add_weld(body, (0.46, y, 0.305), 0.018, frame_finish, f"leg_node_{suffix}")

    # Crossmembers support the tray underside while tying the two handles together.
    _add_tube(body, (-0.22, -0.25, 0.267), (-0.22, 0.25, 0.267), cross_r, frame_finish, "front_crossbar")
    _add_tube(body, (0.36, -0.25, 0.294), (0.36, 0.25, 0.294), cross_r, frame_finish, "rear_crossbar")
    _add_tube(body, (0.70, -0.25, 0.410), (0.70, 0.25, 0.410), 0.012, frame_finish, "handle_crossbar")

    # Narrow front fork keeps the single wheel close to the body.
    _add_tube(body, (-0.27, -0.065, 0.335), (-0.62, -0.065, 0.190), fork_r, frame_finish, "fork_tine_0")
    _add_tube(body, (-0.27, 0.065, 0.335), (-0.62, 0.065, 0.190), fork_r, frame_finish, "fork_tine_1")
    _add_tube(body, (-0.27, -0.25, 0.265), (-0.27, -0.065, 0.335), fork_r, frame_finish, "fork_shoulder_0")
    _add_tube(body, (-0.27, 0.25, 0.265), (-0.27, 0.065, 0.335), fork_r, frame_finish, "fork_shoulder_1")
    _add_tube(body, (-0.62, -0.083, 0.190), (-0.62, -0.045, 0.190), 0.016, frame_finish, "axle_stub_0")
    _add_tube(body, (-0.62, 0.045, 0.190), (-0.62, 0.083, 0.190), 0.016, frame_finish, "axle_stub_1")
    axle_geometry, axle_origin = _cylinder_between((-0.62, -0.052, 0.190), (-0.62, 0.052, 0.190), 0.018)
    body.visual(axle_geometry, origin=axle_origin, material=frame_finish, name="axle_pin")
    for suffix, y in (("0", -0.065), ("1", 0.065)):
        _add_weld(body, (-0.27, y, 0.335), 0.017, frame_finish, f"fork_crown_{suffix}")
        _add_weld(body, (-0.62, y, 0.190), 0.016, frame_finish, f"axle_boss_{suffix}")

    wheel = model.part("front_wheel")
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.137,
                0.052,
                rim=WheelRim(inner_radius=0.095, flange_height=0.010, flange_thickness=0.004),
                hub=WheelHub(radius=0.034, width=0.060, cap_style="domed"),
                face=WheelFace(dish_depth=0.006, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="straight", count=8, thickness=0.004, window_radius=0.010),
                bore=WheelBore(style="round", diameter=0.034),
            ),
            "front_wheel_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rim_finish,
        name="rim",
    )
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                WHEEL_RADIUS,
                WHEEL_WIDTH,
                inner_radius=0.134,
                tread=TireTread(style="block", depth=0.008, count=20, land_ratio=0.56),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.010, radius=0.003),
            ),
            "front_utility_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=WHEEL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=24.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("front_wheel")
    axle = object_model.get_articulation("wheel_axle")

    ctx.check("single_front_wheel", wheel is not None, "Expected one articulated front_wheel part.")
    ctx.check(
        "continuous_wheel_joint",
        axle is not None and axle.articulation_type == ArticulationType.CONTINUOUS,
        f"wheel_axle={axle!r}",
    )
    if body is not None and wheel is not None:
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="axle_pin",
            elem_b="rim",
            reason="The steel axle pin is intentionally captured through the wheel hub bore.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            min_overlap=0.050,
            elem_a="axle_pin",
            elem_b="rim",
            name="axle passes through wheel hub",
        )
        ctx.expect_within(
            body,
            wheel,
            axes="xz",
            margin=0.010,
            inner_elem="axle_pin",
            outer_elem="rim",
            name="axle centered in hub",
        )
        ctx.expect_gap(
            wheel,
            body,
            axis="y",
            min_gap=0.004,
            max_gap=0.035,
            positive_elem="tire",
            negative_elem="fork_tine_0",
            name="fork tine 0 clears tire",
        )
        ctx.expect_gap(
            body,
            wheel,
            axis="y",
            min_gap=0.004,
            max_gap=0.035,
            positive_elem="fork_tine_1",
            negative_elem="tire",
            name="fork tine 1 clears tire",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="xz",
            min_overlap=0.09,
            elem_a="fork_tine_0",
            elem_b="tire",
            name="fork sits close around wheel",
        )
        rest_position = ctx.part_world_position(wheel)
        with ctx.pose({axle: math.pi * 1.5}):
            rotated_position = ctx.part_world_position(wheel)
        ctx.check(
            "wheel_spins_about_fixed_axle",
            rest_position is not None
            and rotated_position is not None
            and all(abs(rest_position[i] - rotated_position[i]) < 1e-6 for i in range(3)),
            details=f"rest={rest_position}, rotated={rotated_position}",
        )

    return ctx.report()


object_model = build_object_model()
