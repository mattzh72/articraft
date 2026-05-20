from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
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


def _tube_origin_between(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
    name: str,
) -> None:
    origin, length = _tube_origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _trapezoid_tray_geometry() -> MeshGeometry:
    """Open, thin-walled wheelbarrow tray: sloped sides, visible lip, and floor."""
    geom = MeshGeometry()

    def profile(
        x_front: float,
        x_rear: float,
        width_front: float,
        width_rear: float,
        z: float,
    ) -> list[int]:
        points = (
            (x_front, -width_front * 0.5, z),
            (x_rear, -width_rear * 0.5, z),
            (x_rear, width_rear * 0.5, z),
            (x_front, width_front * 0.5, z),
        )
        return [geom.add_vertex(*p) for p in points]

    outer_bottom = profile(-0.50, 0.45, 0.38, 0.46, 0.00)
    outer_top = profile(-0.62, 0.56, 0.58, 0.74, 0.36)
    inner_top = profile(-0.575, 0.515, 0.50, 0.66, 0.335)
    inner_floor = profile(-0.43, 0.38, 0.30, 0.38, 0.045)

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(4):
        j = (i + 1) % 4
        # Exterior sloping skin.
        quad(outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        # Interior cavity wall, set in from the outer wall.
        quad(inner_top[i], inner_top[j], inner_floor[j], inner_floor[i])
        # Rolled top lip showing real wall thickness.
        quad(outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        # Underside return around the bottom pan.
        quad(outer_bottom[i], outer_bottom[j], inner_floor[j], inner_floor[i])

    # Visible floor of the open tray.
    geom.add_face(inner_floor[0], inner_floor[1], inner_floor[2])
    geom.add_face(inner_floor[0], inner_floor[2], inner_floor[3])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_wheel_wheelbarrow")

    painted_steel = model.material("painted_steel", rgba=(0.12, 0.42, 0.20, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.05, 0.06, 0.055, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.64, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    rim_material = model.material("yellow_rim", rgba=(0.95, 0.72, 0.12, 1.0))

    barrow = model.part("barrow")
    barrow.visual(
        mesh_from_geometry(_trapezoid_tray_geometry(), "open_tray"),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=painted_steel,
        name="tray",
    )

    # Rounded-looking top rim tubes reinforce that the tray is a thin hollow pan.
    top_z = 0.685
    _add_tube(barrow, (-0.62, -0.29, top_z), (0.56, -0.37, top_z), 0.017, painted_steel, "rim_0")
    _add_tube(barrow, (-0.62, 0.29, top_z), (0.56, 0.37, top_z), 0.017, painted_steel, "rim_1")
    _add_tube(barrow, (-0.62, -0.29, top_z), (-0.62, 0.29, top_z), 0.017, painted_steel, "front_lip")
    _add_tube(barrow, (0.56, -0.37, top_z), (0.56, 0.37, top_z), 0.017, painted_steel, "rear_lip")

    # Two long handle/frame tubes run along the tray sides and converge into the front fork.
    for side, y in (("0", -1.0), ("1", 1.0)):
        _add_tube(
            barrow,
            (1.18, y * 0.39, 0.58),
            (-0.72, y * 0.12, 0.22),
            0.019,
            galvanized,
            f"handle_rail_{side}",
        )
        _add_tube(
            barrow,
            (1.02, y * 0.39, 0.58),
            (1.31, y * 0.40, 0.58),
            0.030,
            rubber,
            f"grip_{side}",
        )
        _add_tube(
            barrow,
            (0.50, y * 0.295, 0.451),
            (0.65, y * 0.36, 0.025),
            0.020,
            galvanized,
            f"rear_leg_{side}",
        )
        barrow.visual(
            Box((0.18, 0.070, 0.030)),
            origin=Origin(xyz=(0.67, y * 0.37, 0.015)),
            material=dark_steel,
            name=f"foot_{side}",
        )
        # Short fork cheek plates sit outside the wheel and carry the axle stubs.
        barrow.visual(
            Box((0.090, 0.040, 0.135)),
            origin=Origin(xyz=(-0.72, y * 0.10, 0.245)),
            material=galvanized,
            name=f"fork_plate_{side}",
        )
        _add_tube(
            barrow,
            (-0.58, y * 0.20, 0.34),
            (-0.72, y * 0.10, 0.25),
            0.020,
            galvanized,
            f"fork_stay_{side}",
        )
        _add_tube(
            barrow,
            (-0.72, y * 0.055, 0.21),
            (-0.72, y * 0.125, 0.21),
            0.016,
            dark_steel,
            f"axle_stub_{side}",
        )

    _add_tube(
        barrow,
        (-0.72, -0.13, 0.21),
        (-0.72, 0.13, 0.21),
        0.010,
        dark_steel,
        "axle",
    )

    # Under-tray crossmembers tie the two side rails to the tray pan.
    _add_tube(barrow, (-0.28, -0.30, 0.345), (-0.28, 0.30, 0.345), 0.017, galvanized, "front_crossbar")
    _add_tube(barrow, (0.32, -0.34, 0.390), (0.32, 0.34, 0.390), 0.017, galvanized, "rear_crossbar")
    for x, name in ((-0.28, "front_mount"), (0.32, "rear_mount")):
        barrow.visual(
            Box((0.13, 0.46, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.365 if x < 0.0 else 0.410)),
            material=dark_steel,
            name=name,
        )

    wheel = model.part("front_wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.175,
                0.080,
                inner_radius=0.126,
                tread=TireTread(style="block", depth=0.008, count=18, land_ratio=0.58),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "front_tire",
        ),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.132,
                0.058,
                rim=WheelRim(inner_radius=0.080, flange_height=0.010, flange_thickness=0.004),
                hub=WheelHub(
                    radius=0.035,
                    width=0.060,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=4, circle_diameter=0.045, hole_diameter=0.006),
                ),
                face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.015),
                bore=WheelBore(style="round", diameter=0.020),
            ),
            "front_wheel_rim",
        ),
        material=rim_material,
        name="rim",
    )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=barrow,
        child=wheel,
        # Joint frame local X is rotated onto the world Y axle line.
        origin=Origin(xyz=(-0.72, 0.0, 0.21), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrow = object_model.get_part("barrow")
    wheel = object_model.get_part("front_wheel")
    axle = object_model.get_articulation("wheel_axle")

    ctx.allow_overlap(
        barrow,
        wheel,
        elem_a="axle",
        elem_b="rim",
        reason="The fixed axle is intentionally captured through the wheel hub/bore proxy.",
    )
    ctx.expect_within(
        barrow,
        wheel,
        axes="xz",
        inner_elem="axle",
        outer_elem="rim",
        margin=0.001,
        name="axle is centered inside wheel hub projection",
    )
    ctx.expect_overlap(
        barrow,
        wheel,
        axes="y",
        elem_a="axle",
        elem_b="rim",
        min_overlap=0.050,
        name="axle passes through wheel hub",
    )

    ctx.check(
        "front wheel has continuous axle joint",
        axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {axle.articulation_type}",
    )
    ctx.check(
        "axle axis is transverse through fork",
        tuple(round(v, 3) for v in axle.axis) == (1.0, 0.0, 0.0)
        and abs(axle.origin.rpy[2] - math.pi / 2.0) < 1.0e-6,
        details=f"axis={axle.axis}, origin_rpy={axle.origin.rpy}",
    )

    wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel sits forward and low",
        wheel_pos is not None and wheel_pos[0] < -0.60 and 0.15 < wheel_pos[2] < 0.28,
        details=f"wheel position={wheel_pos}",
    )

    tray_box = ctx.part_element_world_aabb(barrow, elem="tray")
    wheel_box = ctx.part_world_aabb(wheel)
    if tray_box is not None and wheel_box is not None:
        tray_len = tray_box[1][0] - tray_box[0][0]
        wheel_diameter = wheel_box[1][2] - wheel_box[0][2]
        ctx.check(
            "wheel is small relative to tray",
            wheel_diameter < 0.40 * tray_len,
            details=f"wheel_diameter={wheel_diameter:.3f}, tray_len={tray_len:.3f}",
        )
    else:
        ctx.fail("wheel and tray measurements available", "missing tray or wheel AABB")

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({axle: 2.0}):
        spun_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spins in place on axle",
        rest_pos is not None
        and spun_pos is not None
        and sum((a - b) * (a - b) for a, b in zip(rest_pos, spun_pos)) < 1.0e-10,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
