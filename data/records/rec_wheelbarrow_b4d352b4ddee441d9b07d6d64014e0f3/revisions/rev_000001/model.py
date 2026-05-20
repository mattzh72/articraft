from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rounded_tray_loop(
    length: float,
    width: float,
    z: float,
    *,
    x_offset: float = 0.0,
    points_per_side: int = 8,
    exponent: float = 3.2,
) -> list[tuple[float, float, float]]:
    """Superellipse loop used for the stamped open tray sections."""
    pts: list[tuple[float, float, float]] = []
    count = max(16, points_per_side * 4)
    a = length * 0.5
    b = width * 0.5
    for i in range(count):
        t = 2.0 * pi * i / count
        ct = cos(t)
        st = sin(t)
        # Superellipse parameterization keeps long, inexpensive-to-stamp flats
        # with rounded corners rather than a fragile sharp rectangular tub.
        x = x_offset + a * (1.0 if ct >= 0.0 else -1.0) * abs(ct) ** (2.0 / exponent)
        y = b * (1.0 if st >= 0.0 else -1.0) * abs(st) ** (2.0 / exponent)
        pts.append((x, y, z))
    return pts


def _connect_loops(mesh: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        if flip:
            mesh.add_face(a[i], b[i], a[j])
            mesh.add_face(a[j], b[i], b[j])
        else:
            mesh.add_face(a[i], a[j], b[i])
            mesh.add_face(a[j], b[j], b[i])


def _cap_loop(mesh: MeshGeometry, loop: list[int], *, flip: bool = False) -> None:
    center = [0.0, 0.0, 0.0]
    for idx in loop:
        vx, vy, vz = mesh.vertices[idx]
        center[0] += vx
        center[1] += vy
        center[2] += vz
    scale = 1.0 / len(loop)
    c = mesh.add_vertex(center[0] * scale, center[1] * scale, center[2] * scale)
    for i in range(len(loop)):
        j = (i + 1) % len(loop)
        if flip:
            mesh.add_face(c, loop[j], loop[i])
        else:
            mesh.add_face(c, loop[i], loop[j])


def _tray_shell_geometry() -> MeshGeometry:
    """One-piece open, hollow, drafted tray with a rolled lip and real floor thickness."""
    mesh = MeshGeometry()
    outer_top = _rounded_tray_loop(1.18, 0.72, 0.620, x_offset=-0.060)
    inner_top = _rounded_tray_loop(1.03, 0.585, 0.582, x_offset=-0.055)
    inner_bottom = _rounded_tray_loop(0.66, 0.460, 0.360, x_offset=-0.050)
    outer_bottom = _rounded_tray_loop(0.78, 0.540, 0.300, x_offset=-0.055)

    def add(points: list[tuple[float, float, float]]) -> list[int]:
        return [mesh.add_vertex(x, y, z) for x, y, z in points]

    ot = add(outer_top)
    it = add(inner_top)
    ib = add(inner_bottom)
    ob = add(outer_bottom)
    _connect_loops(mesh, ot, ob)  # outside drafted tub wall
    _connect_loops(mesh, it, ot)  # rolled top flange / lip bridge
    _connect_loops(mesh, ib, it)  # visible inside wall
    _connect_loops(mesh, ob, ib)  # thick floor transition
    _cap_loop(mesh, ib)  # visible inside floor, open only at the top
    _cap_loop(mesh, ob, flip=True)  # underside, so the tray is a real shell, not a surface
    return mesh


def _tube_mesh(points: list[tuple[float, float, float]], name: str, *, radius: float = 0.018):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="cost_optimized_wheelbarrow",
        meta={
            "manufacturing_intent": (
                "One hollow stamped tray, two mirrored bent tube rails with integral handles/fork tines, "
                "two rear leg struts, simple clamp plates, through-axle, and one molded utility wheel."
            )
        },
    )

    tray_plastic = model.material("powder_green_tray", rgba=(0.16, 0.43, 0.23, 1.0))
    tray_shadow = model.material("pressed_rib_shadow", rgba=(0.12, 0.34, 0.18, 1.0))
    black_tube = model.material("black_tube", rgba=(0.035, 0.038, 0.036, 1.0))
    clamp_zinc = model.material("zinc_clamps", rgba=(0.63, 0.65, 0.62, 1.0))
    bolt_dark = model.material("dark_bolts", rgba=(0.07, 0.07, 0.065, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    wheel_poly = model.material("grey_poly_wheel", rgba=(0.70, 0.72, 0.70, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.85, 0.82, 0.82)),
        mass=7.0,
        origin=Origin(xyz=(-0.12, 0.0, 0.36)),
    )

    left_rail = [
        (-1.10, 0.300, 0.780),
        (-0.96, 0.300, 0.700),
        (-0.72, 0.300, 0.490),
        (-0.48, 0.300, 0.280),
        (-0.08, 0.280, 0.250),
        (0.34, 0.230, 0.300),
        (0.55, 0.100, 0.365),
        (0.68, 0.075, 0.200),
    ]
    right_rail = [(x, -y, z) for x, y, z in left_rail]
    frame.visual(_tube_mesh(left_rail, "left_bent_rail"), material=black_tube, name="left_bent_rail")
    frame.visual(_tube_mesh(right_rail, "right_bent_rail"), material=black_tube, name="right_bent_rail")

    for name, x, z, length in (
        ("front_cross_tube", -0.08, 0.250, 0.62),
        ("rear_cross_tube", -0.48, 0.280, 0.70),
        ("nose_bridge_tube", 0.52, 0.355, 0.26),
    ):
        frame.visual(
            Cylinder(radius=0.017, length=length),
            origin=Origin(xyz=(x, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black_tube,
            name=name,
        )

    frame.visual(
        Cylinder(radius=0.014, length=0.220),
        origin=Origin(xyz=(0.68, 0.0, 0.200), rpy=(pi / 2.0, 0.0, 0.0)),
        material=clamp_zinc,
        name="axle",
    )
    for name, y in (("axle_nut_0", 0.112), ("axle_nut_1", -0.112)):
        frame.visual(
            Cylinder(radius=0.026, length=0.016),
            origin=Origin(xyz=(0.68, y, 0.200), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt_dark,
            name=name,
        )

    for side, y in (("left", 0.305), ("right", -0.305)):
        leg_points = [(-0.48, y, 0.280), (-0.52, y * 1.10, 0.150), (-0.56, y * 1.16, 0.036)]
        frame.visual(_tube_mesh(leg_points, f"{side}_rear_leg", radius=0.018), material=black_tube, name=f"{side}_rear_leg")
        frame.visual(
            Cylinder(radius=0.024, length=0.150),
            origin=Origin(xyz=(-0.56, y * 1.16, 0.024), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name=f"{side}_foot",
        )

    # Two broad clamp plates sit under the tray floor and overlap the side rails:
    # cheap laser-cut/pressed tabs instead of many individual brackets.
    for name, x in (("front_clamp_plate", 0.180), ("rear_clamp_plate", -0.245)):
        frame.visual(
            Box((0.135, 0.560, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.288)),
            material=clamp_zinc,
            name=name,
        )
        for y in (-0.190, 0.190):
            frame.visual(
                Cylinder(radius=0.020, length=0.008),
                origin=Origin(xyz=(x, y, 0.272)),
                material=bolt_dark,
                name=f"{name}_bolt_{0 if y < 0 else 1}",
            )
    frame.visual(
        Box((0.240, 0.080, 0.024)),
        origin=Origin(xyz=(-0.365, 0.0, 0.288)),
        material=clamp_zinc,
        name="rear_plate_tie",
    )

    for name, y in (("left_grip", 0.300), ("right_grip", -0.300)):
        frame.visual(
            _tube_mesh(
                [(-1.135, y, 0.800), (-1.075, y, 0.775), (-0.995, y, 0.720)],
                name,
                radius=0.026,
            ),
            material=rubber,
            name=name,
        )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((1.18, 0.72, 0.32)),
        mass=5.0,
        origin=Origin(xyz=(-0.06, 0.0, 0.46)),
    )
    tray.visual(mesh_from_geometry(_tray_shell_geometry(), "open_stamped_tray"), material=tray_plastic, name="tray_shell")

    # Shallow stamped ribs and aligned bolt heads make the one-piece tray read as
    # molded/stamped rather than a plain bucket, while preserving low part count.
    for i, y in enumerate((-0.105, 0.0, 0.105)):
        tray.visual(
            Box((0.475, 0.028, 0.018)),
            origin=Origin(xyz=(-0.040, y, 0.368)),
            material=tray_shadow,
            name=f"floor_rib_{i}",
        )
    for i, x in enumerate((-0.245, 0.180)):
        for j, y in enumerate((-0.190, 0.190)):
            tray.visual(
                Cylinder(radius=0.022, length=0.008),
                origin=Origin(xyz=(x, y, 0.364)),
                material=bolt_dark,
                name=f"tray_bolt_{i}_{j}",
            )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Box((0.090, 0.360, 0.360)),
        mass=2.1,
        origin=Origin(),
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.135,
                0.064,
                rim=WheelRim(inner_radius=0.095, flange_height=0.010, flange_thickness=0.004),
                hub=WheelHub(
                    radius=0.043,
                    width=0.086,
                    cap_style="flat",
                    bolt_pattern=BoltPattern(count=4, circle_diameter=0.052, hole_diameter=0.004),
                ),
                face=WheelFace(dish_depth=0.006, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.005, window_radius=0.012),
                bore=WheelBore(style="round", diameter=0.028),
            ),
            "poly_wheel",
        ),
        material=wheel_poly,
        name="wheel_hub",
    )
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.180,
                0.090,
                inner_radius=0.136,
                tread=TireTread(style="block", depth=0.007, count=20, land_ratio=0.58),
                sidewall=TireSidewall(style="square", bulge=0.025),
                shoulder=TireShoulder(width=0.010, radius=0.003),
            ),
            "utility_tire",
        ),
        material=rubber,
        name="tire",
    )

    model.articulation(
        "tray_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=tray,
        origin=Origin(),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        # Rotate the joint frame so the wheel helper's local X spin axis becomes
        # the real wheelbarrow axle direction (world Y).
        origin=Origin(xyz=(0.68, 0.0, 0.200), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.check("base_parts_present", all(p is not None for p in (frame, tray, wheel)), "Expected frame, tray, and wheel.")
    if frame is None or tray is None or wheel is None or spin is None:
        return ctx.report()

    ctx.expect_gap(
        tray,
        frame,
        axis="z",
        positive_elem="tray_shell",
        negative_elem="front_clamp_plate",
        min_gap=-0.001,
        max_gap=0.002,
        name="front clamp plate supports tray floor",
    )
    ctx.expect_overlap(
        tray,
        frame,
        axes="xy",
        elem_a="tray_shell",
        elem_b="front_clamp_plate",
        min_overlap=0.10,
        name="front clamp plate lies under tray footprint",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="axle",
        elem_b="wheel_hub",
        reason="The simple through-axle is intentionally captured inside the wheel hub bore.",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="wheel_hub",
        elem_b="axle",
        min_overlap=0.070,
        name="wheel hub is retained on axle width",
    )

    wheel_box = ctx.part_world_aabb(wheel)
    if wheel_box is not None:
        mins, maxs = wheel_box
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("single front wheel diameter", 0.34 <= max(size[0], size[2]) <= 0.38, f"wheel size={size!r}")
        ctx.check("narrow wheel stance width", 0.080 <= size[1] <= 0.105, f"wheel size={size!r}")

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({spin: pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
        ctx.check(
            "wheel spins about fixed axle",
            rest_pos is not None and turned_pos is not None and all(abs(rest_pos[i] - turned_pos[i]) < 1e-6 for i in range(3)),
            details=f"rest={rest_pos}, spun={turned_pos}",
        )

    return ctx.report()


object_model = build_object_model()
