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
    TireGroove,
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
    tube_from_spline_points,
)


def _superellipse_loop(
    center_x: float,
    center_z: float,
    length: float,
    width: float,
    z: float,
    *,
    segments: int = 56,
    exponent: float = 3.2,
) -> list[tuple[float, float, float]]:
    """Rounded-rectangle loop in the horizontal XY plane at height z."""

    pts: list[tuple[float, float, float]] = []
    half_x = length * 0.5
    half_y = width * 0.5
    power = 2.0 / exponent
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        x = center_x + half_x * math.copysign(abs(c) ** power, c)
        y = half_y * math.copysign(abs(s) ** power, s)
        pts.append((x, y, z + center_z))
    return pts


def _bridge_loops(mesh: MeshGeometry, a: list[tuple[float, float, float]], b: list[tuple[float, float, float]]) -> None:
    """Append two equal-length loops and quad-strip faces between them."""

    if len(a) != len(b):
        raise ValueError("loops must have equal point counts")
    base_a = len(mesh.vertices)
    for p in a:
        mesh.add_vertex(*p)
    base_b = len(mesh.vertices)
    for p in b:
        mesh.add_vertex(*p)
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        mesh.add_face(base_a + i, base_a + j, base_b + j)
        mesh.add_face(base_a + i, base_b + j, base_b + i)


def _tray_shell_mesh() -> MeshGeometry:
    """Thin, open-topped, flared contractor tray with visible wall thickness."""

    mesh = MeshGeometry()
    # Coordinates are relative to the front axle center.  The tray is high,
    # long, flared at the rim, and smaller at the bottom like a real pressed tub.
    outer_top = _superellipse_loop(0.78, 0.0, 1.12, 0.78, 0.83, segments=64, exponent=3.4)
    outer_bottom = _superellipse_loop(0.78, 0.0, 0.72, 0.43, 0.35, segments=64, exponent=3.1)
    inner_top = _superellipse_loop(0.78, 0.0, 1.04, 0.70, 0.795, segments=64, exponent=3.4)
    inner_bottom = _superellipse_loop(0.78, 0.0, 0.60, 0.31, 0.405, segments=64, exponent=3.0)

    _bridge_loops(mesh, outer_bottom, outer_top)     # outside flared sides
    _bridge_loops(mesh, outer_top, inner_top)        # rolled top lip / rim shelf
    _bridge_loops(mesh, inner_top, inner_bottom)     # inside basin wall
    _bridge_loops(mesh, inner_bottom, outer_bottom)  # thick bottom floor
    return mesh


def _top_rim_mesh() -> MeshGeometry:
    rim_path = _superellipse_loop(0.78, 0.0, 1.13, 0.79, 0.835, segments=24, exponent=3.4)
    return tube_from_spline_points(
        rim_path,
        radius=0.022,
        samples_per_segment=5,
        closed_spline=True,
        radial_segments=18,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_wheelbarrow")

    green = model.material("powder_coated_green", rgba=(0.10, 0.43, 0.18, 1.0))
    black = model.material("black_powdercoat", rgba=(0.015, 0.014, 0.012, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    yellow = model.material("utility_yellow_rim", rgba=(0.95, 0.68, 0.08, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.62, 1.0))
    dark_steel = model.material("dark_bolt_heads", rgba=(0.08, 0.085, 0.08, 1.0))

    # The axle is the kinematic root: the wheel spins on it and the loaded
    # tray/frame assembly can tip around it to show the natural dumping motion.
    axle = model.part("axle")
    axle.visual(
        Cylinder(radius=0.024, length=0.54),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_shaft",
    )
    for idx, y in enumerate((-0.095, 0.095)):
        axle.visual(
            Cylinder(radius=0.038, length=0.010),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"wheel_washer_{idx}",
        )

    wheel = model.part("wheel")
    tire_geom = TireGeometry(
        0.235,
        0.098,
        inner_radius=0.165,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
        tread=TireTread(style="block", depth=0.012, count=22, land_ratio=0.54),
        grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
        sidewall=TireSidewall(style="square", bulge=0.035),
        shoulder=TireShoulder(width=0.012, radius=0.004),
    )
    rim_geom = WheelGeometry(
        0.166,
        0.078,
        rim=WheelRim(
            inner_radius=0.105,
            flange_height=0.012,
            flange_thickness=0.005,
            bead_seat_depth=0.005,
        ),
        hub=WheelHub(
            radius=0.050,
            width=0.060,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.060, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=6, thickness=0.006, window_radius=0.018),
        bore=WheelBore(style="round", diameter=0.052),
    )
    # Wheel/tire helpers spin about local X; rotate the visuals so local X is
    # the wheelbarrow's width axis (world/body Y).
    wheel.visual(
        mesh_from_geometry(tire_geom, "block_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(rim_geom, "yellow_spoked_rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=yellow,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.030, length=0.110),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bearing_sleeve",
    )

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_tray_shell_mesh(), "deep_flared_tray"),
        material=green,
        name="tray_shell",
    )
    body.visual(
        mesh_from_geometry(_top_rim_mesh(), "rolled_tray_rim"),
        material=green,
        name="rolled_rim",
    )

    main_frame_path = [
        (1.80, -0.35, 0.32),
        (1.50, -0.33, 0.22),
        (1.18, -0.31, 0.15),
        (0.70, -0.28, 0.14),
        (0.05, -0.20, 0.09),
        (0.03, -0.20, 0.17),
        (0.00, 0.00, 0.27),
        (0.03, 0.20, 0.17),
        (0.05, 0.20, 0.09),
        (0.70, 0.28, 0.14),
        (1.18, 0.31, 0.15),
        (1.50, 0.33, 0.22),
        (1.80, 0.35, 0.32),
    ]
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                main_frame_path,
                radius=0.022,
                samples_per_segment=8,
                closed_spline=True,
                radial_segments=18,
            ),
            "continuous_tubular_frame",
        ),
        material=black,
        name="main_frame",
    )

    leg_stand_path = [
        (1.18, -0.31, 0.135),
        (1.29, -0.38, -0.12),
        (1.32, -0.38, -0.235),
        (1.32, 0.38, -0.235),
        (1.29, 0.38, -0.12),
        (1.18, 0.31, 0.135),
    ]
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                leg_stand_path,
                radius=0.020,
                samples_per_segment=7,
                radial_segments=16,
                cap_ends=True,
            ),
            "rear_leg_stand",
        ),
        material=black,
        name="leg_stand",
    )
    for idx, y in enumerate((-0.38, 0.38)):
        body.visual(
            Box((0.16, 0.055, 0.030)),
            origin=Origin(xyz=(1.32, y, -0.255)),
            material=rubber,
            name=f"foot_pad_{idx}",
        )

    for idx, x in enumerate((0.55, 1.04)):
        body.visual(
            Cylinder(radius=0.018, length=0.66),
            origin=Origin(xyz=(x, 0.0, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"tray_crossbar_{idx}",
        )
    for idx, x in enumerate((0.55, 1.04)):
        for side, y in enumerate((-0.245, 0.245)):
            body.visual(
                Box((0.075, 0.105, 0.240)),
                origin=Origin(xyz=(x, y, 0.250)),
                material=black,
                name=f"tray_support_{idx}_{side}",
            )
    for idx, x in enumerate((0.48, 1.10)):
        for side, y in enumerate((-0.23, 0.23)):
            body.visual(
                Cylinder(radius=0.018, length=0.36),
                origin=Origin(xyz=(x, y, 0.21), rpy=(0.0, 0.55, 0.0)),
                material=black,
                name=f"tray_strut_{idx}_{side}",
            )

    # Axle bushings and fork plates are part of the tilting body; the shaft is
    # intentionally captured through these bushing proxies.
    body.visual(
        Cylinder(radius=0.046, length=0.052),
        origin=Origin(xyz=(0.0, -0.185, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bushing_0",
    )
    body.visual(
        Box((0.12, 0.018, 0.16)),
        origin=Origin(xyz=(0.07, -0.185, 0.11)),
        material=black,
        name="fork_plate_0",
    )
    body.visual(
        Cylinder(radius=0.046, length=0.052),
        origin=Origin(xyz=(0.0, 0.185, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bushing_1",
    )
    body.visual(
        Box((0.12, 0.018, 0.16)),
        origin=Origin(xyz=(0.07, 0.185, 0.11)),
        material=black,
        name="fork_plate_1",
    )

    body.visual(
        Cylinder(radius=0.033, length=0.24),
        origin=Origin(xyz=(1.72, -0.35, 0.32), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_0",
    )
    body.visual(
        Cylinder(radius=0.033, length=0.24),
        origin=Origin(xyz=(1.72, 0.35, 0.32), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_1",
    )
    for idx, y in enumerate((-0.35, 0.35)):
        body.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(xyz=(1.04, y * 0.70, 0.37), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"tray_bolt_{idx}",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(xyz=(0.46, y * 0.80, 0.58), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"front_bolt_{idx}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=wheel,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )
    model.articulation(
        "dump_pivot",
        ArticulationType.REVOLUTE,
        parent=axle,
        child=body,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.9, lower=0.0, upper=0.78),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    axle = object_model.get_part("axle")
    wheel = object_model.get_part("wheel")
    body = object_model.get_part("body")
    wheel_spin = object_model.get_articulation("wheel_spin")
    dump_pivot = object_model.get_articulation("dump_pivot")

    ctx.allow_overlap(
        axle,
        body,
        elem_a="axle_shaft",
        elem_b="bushing_0",
        reason="The steel axle is intentionally captured inside the fork bushing proxy.",
    )
    ctx.allow_overlap(
        axle,
        body,
        elem_a="axle_shaft",
        elem_b="bushing_1",
        reason="The steel axle is intentionally captured inside the fork bushing proxy.",
    )
    ctx.allow_overlap(
        axle,
        wheel,
        elem_a="axle_shaft",
        elem_b="bearing_sleeve",
        reason="The rotating wheel bearing sleeve is intentionally modeled as a captured solid proxy around the axle.",
    )
    for bushing_name in ("bushing_0", "bushing_1"):
        ctx.expect_within(
            axle,
            body,
            axes="xz",
            inner_elem="axle_shaft",
            outer_elem=bushing_name,
            margin=0.004,
            name=f"{bushing_name} surrounds axle",
        )
        ctx.expect_overlap(
            axle,
            body,
            axes="y",
            elem_a="axle_shaft",
            elem_b=bushing_name,
            min_overlap=0.035,
            name=f"{bushing_name} retains shaft length",
        )

    ctx.expect_within(
        axle,
        wheel,
        axes="xz",
        inner_elem="axle_shaft",
        outer_elem="bearing_sleeve",
        margin=0.0,
        name="bearing sleeve is concentric with axle",
    )
    ctx.expect_overlap(
        axle,
        wheel,
        axes="y",
        elem_a="axle_shaft",
        elem_b="bearing_sleeve",
        min_overlap=0.09,
        name="bearing sleeve captures axle width",
    )

    ctx.check(
        "wheel rotates about axle width",
        tuple(round(v, 3) for v in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={wheel_spin.axis}",
    )
    ctx.check(
        "dump pivot has realistic stop range",
        dump_pivot.motion_limits is not None
        and dump_pivot.motion_limits.lower == 0.0
        and 0.65 <= float(dump_pivot.motion_limits.upper) <= 0.90,
        details=f"limits={dump_pivot.motion_limits}",
    )

    ctx.expect_gap(
        body,
        wheel,
        axis="z",
        positive_elem="tray_shell",
        negative_elem="tire",
        min_gap=0.025,
        name="deep tray clears the top of the tire",
    )
    ctx.expect_overlap(
        body,
        wheel,
        axes="x",
        elem_a="main_frame",
        elem_b="tire",
        min_overlap=0.06,
        name="front frame wraps around the wheel station",
    )

    rest_grip = ctx.part_element_world_aabb(body, elem="grip_0")
    with ctx.pose({dump_pivot: 0.62, wheel_spin: 1.0}):
        raised_grip = ctx.part_element_world_aabb(body, elem="grip_0")
        ctx.expect_gap(
            body,
            wheel,
            axis="z",
            positive_elem="tray_shell",
            negative_elem="tire",
            min_gap=0.05,
            name="tray still clears tire while dumping",
        )

    rest_z = None if rest_grip is None else rest_grip[1][2]
    raised_z = None if raised_grip is None else raised_grip[1][2]
    ctx.check(
        "dump motion lifts the handles",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.35,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
