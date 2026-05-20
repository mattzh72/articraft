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
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


WHEEL_CENTER = (-0.72, 0.0, 0.18)
TIRE_RADIUS = 0.18
TIRE_WIDTH = 0.080


def _superellipse_loop(width: float, depth: float, z: float, *, segments: int = 56) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in superellipse_profile(width, depth, exponent=3.2, segments=segments)]


def _add_side_faces(mesh: MeshGeometry, loop_a: list[int], loop_b: list[int], *, flip: bool = False) -> None:
    count = len(loop_a)
    for i in range(count):
        a0 = loop_a[i]
        a1 = loop_a[(i + 1) % count]
        b0 = loop_b[i]
        b1 = loop_b[(i + 1) % count]
        if flip:
            mesh.add_face(a0, b1, b0)
            mesh.add_face(a0, a1, b1)
        else:
            mesh.add_face(a0, b0, b1)
            mesh.add_face(a0, b1, a1)


def _cap_loop(mesh: MeshGeometry, loop: list[int], *, reverse: bool = False) -> None:
    center = mesh.add_vertex(
        sum(mesh.vertices[index][0] for index in loop) / len(loop),
        sum(mesh.vertices[index][1] for index in loop) / len(loop),
        sum(mesh.vertices[index][2] for index in loop) / len(loop),
    )
    count = len(loop)
    for i in range(count):
        a = loop[i]
        b = loop[(i + 1) % count]
        if reverse:
            mesh.add_face(center, b, a)
        else:
            mesh.add_face(center, a, b)


def _weatherproof_tray_shell() -> MeshGeometry:
    """Open flared basin with a real bottom, sloped walls, and rolled drip lip."""
    mesh = MeshGeometry()

    # Inner cavity and outer skin loops.  The top outer loop is deliberately
    # larger than the inner opening so the rim reads as an overhanging drip edge.
    inner_bottom = _superellipse_loop(0.72, 0.46, 0.405)
    inner_top = _superellipse_loop(1.10, 0.76, 0.720)
    outer_top = _superellipse_loop(1.20, 0.84, 0.713)
    outer_bottom = _superellipse_loop(0.82, 0.56, 0.355)

    loops: list[list[int]] = []
    for loop in (inner_bottom, inner_top, outer_top, outer_bottom):
        indices: list[int] = []
        for x, y, z in loop:
            indices.append(mesh.add_vertex(x, y, z))
        loops.append(indices)

    ib, it, ot, ob = loops
    _add_side_faces(mesh, it, ib, flip=True)   # smooth inner walls
    _add_side_faces(mesh, ob, ot, flip=True)   # weather-facing outer walls
    _add_side_faces(mesh, it, ot)              # closed rolled rim / drip overhang
    _add_side_faces(mesh, ib, ob)              # thickened bottom edge
    _cap_loop(mesh, ib)                        # load-carrying tray floor
    _cap_loop(mesh, ob, reverse=True)          # underside skin

    return mesh


def _rim_guard_path() -> list[tuple[float, float, float]]:
    return _superellipse_loop(1.19, 0.83, 0.715, segments=44)


def _tube_path(points: list[tuple[float, float, float]], radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(points, radius=radius, samples_per_segment=16, radial_segments=18),
        name,
    )


def _closed_tube_path(points: list[tuple[float, float, float]], radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=6,
            radial_segments=18,
            closed_spline=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_wheelbarrow")

    tray_green = model.material("powder_coated_green", rgba=(0.10, 0.35, 0.18, 1.0))
    frame_finish = model.material("galvanized_tube", rgba=(0.58, 0.62, 0.62, 1.0))
    stainless = model.material("stainless_hardware", rgba=(0.82, 0.84, 0.80, 1.0))
    black_rubber = model.material("sealed_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_shadow = model.material("shadowed_undercoat", rgba=(0.04, 0.07, 0.05, 1.0))
    wheel_yellow = model.material("zinc_plated_wheel", rgba=(0.92, 0.72, 0.24, 1.0))

    frame = model.part("tray_frame")

    frame.visual(
        mesh_from_geometry(_weatherproof_tray_shell(), "sealed_flared_tray"),
        material=tray_green,
        name="tray_shell",
    )
    frame.visual(
        _closed_tube_path(_rim_guard_path(), 0.028, "rolled_drain_rim"),
        material=tray_green,
        name="rolled_rim",
    )

    # Twin tubular rails run continuously from the protected axle fork to the
    # rear hand grips, keeping the fabrication silhouette simple and readable.
    for side, label in ((1.0, "upper"), (-1.0, "lower")):
        y = side * 0.32
        frame.visual(
            _tube_path(
                [
                    (-0.72, side * 0.112, 0.180),
                    (-0.60, side * 0.190, 0.305),
                    (-0.37, y, 0.372),
                    (0.18, y, 0.407),
                    (0.58, y, 0.492),
                    (1.02, y, 0.600),
                    (1.24, y, 0.635),
                ],
                0.018,
                f"{label}_side_rail",
            ),
            material=frame_finish,
            name=f"{label}_side_rail",
        )
        frame.visual(
            _tube_path(
                [
                    (0.50, y, 0.472),
                    (0.52, y, 0.235),
                    (0.63, side * 0.345, 0.050),
                ],
                0.018,
                f"{label}_rear_leg",
            ),
            material=frame_finish,
            name=f"{label}_rear_leg",
        )
        frame.visual(
            Box((0.20, 0.090, 0.050)),
            origin=Origin(xyz=(0.655, side * 0.345, 0.025), rpy=(0.0, 0.0, side * 0.08)),
            material=black_rubber,
            name=f"{label}_foot_pad",
        )
        frame.visual(
            Cylinder(radius=0.027, length=0.245),
            origin=Origin(xyz=(1.225, y, 0.633), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"{label}_grip",
        )

    # Cross tubes and tray pads give the basin hard continuity to the handle
    # rails rather than leaving it visually perched.
    for x, z, name in (
        (-0.36, 0.382, "front_cross_tube"),
        (0.08, 0.398, "center_cross_tube"),
        (0.43, 0.460, "rear_cross_tube"),
    ):
        frame.visual(
            Cylinder(radius=0.016, length=0.69),
            origin=Origin(xyz=(x, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=frame_finish,
            name=name,
        )
        frame.visual(
            Box((0.18, 0.46, 0.018)),
            origin=Origin(xyz=(x, 0.0, z + 0.020)),
            material=black_rubber,
            name=f"{name}_seal_pad",
        )

    # Protected front fork and axle: the stationary stainless axle passes through
    # the wheel's hollow bore; rubber boots shield both sides from grit and rain.
    for side, label in ((1.0, "upper"), (-1.0, "lower")):
        frame.visual(
            _tube_path(
                [
                    (WHEEL_CENTER[0], side * 0.092, WHEEL_CENTER[2]),
                    (-0.645, side * 0.126, 0.285),
                    (-0.490, side * 0.245, 0.405),
                ],
                0.020,
                f"{label}_fork_tine",
            ),
            material=frame_finish,
            name=f"{label}_fork_tine",
        )
        frame.visual(
            Box((0.050, 0.020, 0.070)),
            origin=Origin(xyz=(-0.700, side * 0.110, 0.180), rpy=(0.0, 0.18 * side, 0.0)),
            material=stainless,
            name=f"{label}_dropout_plate",
        )
        frame.visual(
            mesh_from_geometry(TorusGeometry(0.020, 0.007, radial_segments=18, tubular_segments=36), f"{label}_axle_boot"),
            origin=Origin(xyz=(WHEEL_CENTER[0], side * 0.055, WHEEL_CENTER[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name=f"{label}_axle_boot",
        )
        frame.visual(
            Cylinder(radius=0.025, length=0.010),
            origin=Origin(xyz=(WHEEL_CENTER[0], side * 0.145, WHEEL_CENTER[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"{label}_axle_nut",
        )

    frame.visual(
        Cylinder(radius=0.0135, length=0.300),
        origin=Origin(xyz=WHEEL_CENTER, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="axle",
    )

    # Stainless tray fasteners are embedded slightly through black sealing pads,
    # showing a weatherproof bolted interface without floating hardware.
    bolt_positions = [
        (-0.36, 0.382, -0.18),
        (-0.36, 0.382, 0.18),
        (0.08, 0.398, -0.20),
        (0.08, 0.398, 0.20),
        (0.43, 0.460, -0.18),
        (0.43, 0.460, 0.18),
    ]
    for index, (x, cross_z, y) in enumerate(bolt_positions):
        z = cross_z + 0.030
        frame.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, 0.0)),
            material=stainless,
            name=f"sealed_bolt_{index}",
        )
        frame.visual(
            Cylinder(radius=0.023, length=0.006),
            origin=Origin(xyz=(x, y, z - 0.006), rpy=(0.0, 0.0, 0.0)),
            material=black_rubber,
            name=f"bolt_gasket_{index}",
        )

    # Dark undercoat plate visually ties the tray bottom to cross tubes and
    # suggests a water-shedding underside.
    frame.visual(
        Box((0.70, 0.42, 0.020)),
        origin=Origin(xyz=(0.02, 0.0, 0.366)),
        material=dark_shadow,
        name="undertray_seal",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.127,
                0.068,
                rim=WheelRim(inner_radius=0.086, flange_height=0.011, flange_thickness=0.004),
                hub=WheelHub(
                    radius=0.044,
                    width=0.070,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=6, circle_diameter=0.058, hole_diameter=0.005),
                ),
                face=WheelFace(dish_depth=0.009, front_inset=0.003, rear_inset=0.003),
                spokes=WheelSpokes(style="straight", count=8, thickness=0.006, window_radius=0.016),
                bore=WheelBore(style="round", diameter=0.055),
            ),
            "wheel_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=wheel_yellow,
        name="rim",
    )
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                TIRE_RADIUS,
                TIRE_WIDTH,
                inner_radius=0.124,
                carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.05),
                tread=TireTread(style="block", depth=0.011, count=24, land_ratio=0.55),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.012, radius=0.004),
            ),
            "utility_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=black_rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="bearing_sleeve",
    )

    model.articulation(
        "axle_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=WHEEL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("tray_frame")
    wheel = object_model.get_part("wheel")
    axle_joint = object_model.get_articulation("axle_to_wheel")

    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="axle",
        elem_b="bearing_sleeve",
        reason="The stainless axle is intentionally captured inside the wheel bearing sleeve.",
    )

    ctx.check("single tray frame present", frame is not None, "Expected the continuous tray/frame root.")
    ctx.check("front wheel present", wheel is not None, "Expected the rotating front wheel.")
    ctx.check(
        "wheel spins on transverse axle",
        axle_joint is not None
        and axle_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in axle_joint.axis) == (0.0, 1.0, 0.0),
        details=f"joint={axle_joint}",
    )
    if frame is None or wheel is None:
        return ctx.report()

    ctx.expect_within(
        wheel,
        frame,
        axes="y",
        inner_elem="tire",
        outer_elem="axle",
        margin=0.002,
        name="wheel captured on axle width",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="xz",
        elem_a="bearing_sleeve",
        elem_b="axle",
        min_overlap=0.010,
        name="axle passes through bearing sleeve",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="bearing_sleeve",
        elem_b="axle",
        min_overlap=0.080,
        name="bearing sleeve remains captured on axle",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    wheel_aabb = ctx.part_world_aabb(wheel)
    if frame_aabb is not None and wheel_aabb is not None:
        fmin, fmax = frame_aabb
        wmin, wmax = wheel_aabb
        ctx.check("rear stance reaches ground", fmin[2] <= 0.006, f"frame_aabb={frame_aabb!r}")
        ctx.check("wheel reaches ground", wmin[2] <= 0.006, f"wheel_aabb={wheel_aabb!r}")
        ctx.check("tray overhangs wheel", fmax[2] > wmax[2] + 0.30, f"frame={frame_aabb!r}, wheel={wheel_aabb!r}")

    return ctx.report()


object_model = build_object_model()
