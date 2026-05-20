from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    CylinderGeometry,
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
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geom in geometries:
        merged.merge(geom)
    return merged


def _cylinder_y(radius: float, length: float, x: float, z: float) -> MeshGeometry:
    return CylinderGeometry(radius=radius, height=length, radial_segments=20).rotate_x(
        -math.pi / 2.0
    ).translate(x, 0.0, z)


def _tray_loop(width: float, depth: float, z: float, *, x_offset: float, radius: float):
    return [
        (x + x_offset, y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _add_loop(geom: MeshGeometry, points) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in points]


def _quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _connect_loops(geom: MeshGeometry, lower: list[int], upper: list[int]) -> None:
    count = len(lower)
    for i in range(count):
        _quad(geom, lower[i], lower[(i + 1) % count], upper[(i + 1) % count], upper[i])


def _cap_loop(geom: MeshGeometry, loop: list[int], *, reverse: bool = False) -> None:
    xs, ys, zs = [], [], []
    for idx in loop:
        x, y, z = geom.vertices[idx]
        xs.append(x)
        ys.append(y)
        zs.append(z)
    center_idx = geom.add_vertex(sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))
    for i in range(len(loop)):
        a = loop[i]
        b = loop[(i + 1) % len(loop)]
        if reverse:
            geom.add_face(center_idx, b, a)
        else:
            geom.add_face(center_idx, a, b)


def _build_tray_shell() -> MeshGeometry:
    """Open-topped flared tray with real wall and floor thickness."""
    geom = MeshGeometry()
    outer_bottom = _add_loop(
        geom, _tray_loop(0.60, 0.34, 0.185, x_offset=0.05, radius=0.060)
    )
    inner_bottom = _add_loop(
        geom, _tray_loop(0.50, 0.245, 0.215, x_offset=0.05, radius=0.050)
    )
    inner_rim = _add_loop(
        geom, _tray_loop(0.735, 0.485, 0.465, x_offset=0.035, radius=0.090)
    )
    outer_rim = _add_loop(
        geom, _tray_loop(0.820, 0.565, 0.500, x_offset=0.030, radius=0.115)
    )

    _connect_loops(geom, outer_bottom, outer_rim)
    _connect_loops(geom, outer_rim, inner_rim)
    _connect_loops(geom, inner_rim, inner_bottom)
    _connect_loops(geom, inner_bottom, outer_bottom)
    _cap_loop(geom, inner_bottom, reverse=False)
    _cap_loop(geom, outer_bottom, reverse=True)
    return geom


def _build_rolled_rim() -> MeshGeometry:
    return tube_from_spline_points(
        _tray_loop(0.840, 0.585, 0.502, x_offset=0.030, radius=0.120),
        radius=0.014,
        samples_per_segment=4,
        closed_spline=True,
        radial_segments=16,
        cap_ends=False,
    )


def _build_tubular_frame() -> MeshGeometry:
    """Twin handles, compact front fork, crossbars, and two rear resting legs."""
    tube_r = 0.015
    side_paths = []
    for side in (-1.0, 1.0):
        y = side * 0.250
        side_paths.append(
            tube_from_spline_points(
                [
                    (0.765, y, 0.305),
                    (0.570, side * 0.245, 0.245),
                    (0.245, side * 0.225, 0.160),
                    (-0.125, side * 0.205, 0.180),
                    (-0.330, side * 0.135, 0.275),
                    (-0.440, side * 0.067, 0.160),
                ],
                radius=tube_r,
                samples_per_segment=10,
                radial_segments=18,
            )
        )
        side_paths.append(
            tube_from_spline_points(
                [
                    (0.350, side * 0.224, 0.175),
                    (0.430, side * 0.238, 0.095),
                    (0.505, side * 0.255, 0.018),
                ],
                radius=0.014,
                samples_per_segment=8,
                radial_segments=18,
            )
        )
        side_paths.append(
            tube_from_spline_points(
                [
                    (-0.330, side * 0.135, 0.275),
                    (-0.290, side * 0.160, 0.350),
                ],
                radius=0.012,
                samples_per_segment=6,
                radial_segments=16,
            )
        )

    crossbars = [
        _cylinder_y(0.017, 0.545, 0.245, 0.160),
        _cylinder_y(0.014, 0.430, -0.205, 0.220),
        _cylinder_y(0.013, 0.335, -0.290, 0.350),
        _cylinder_y(0.014, 0.560, 0.505, 0.024),
    ]

    return _merge_geometries(side_paths + crossbars)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_wheelbarrow")

    tray_green = model.material("tray_green", rgba=(0.14, 0.48, 0.24, 1.0))
    rim_green = model.material("rolled_green", rgba=(0.10, 0.36, 0.18, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.015, 0.014, 1.0))
    steel = model.material("painted_steel", rgba=(0.13, 0.15, 0.16, 1.0))
    yellow = model.material("utility_yellow", rgba=(0.95, 0.70, 0.12, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_build_tray_shell(), "tray_shell"),
        material=tray_green,
        name="tray_shell",
    )
    frame.visual(
        mesh_from_geometry(_build_rolled_rim(), "rolled_rim"),
        material=rim_green,
        name="rolled_rim",
    )
    frame.visual(
        mesh_from_geometry(_build_tubular_frame(), "tubular_frame"),
        material=steel,
        name="tubular_frame",
    )

    # A visible axle pin crosses the fork and supports the wheel through its bore.
    frame.visual(
        Cylinder(radius=0.018, length=0.205),
        origin=Origin(xyz=(-0.440, 0.0, 0.160), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_axle",
    )

    # Four small saddle blocks make the tray-to-frame mounting visually explicit.
    for ix, x in enumerate((-0.105, 0.250)):
        for iy, y in enumerate((-0.215, 0.215)):
            frame.visual(
                Box((0.095, 0.055, 0.052)),
                origin=Origin(xyz=(x, y, 0.182)),
                material=steel,
                name=f"saddle_{ix}_{iy}",
            )

    for i, y in enumerate((-0.250, 0.250)):
        frame.visual(
            Cylinder(radius=0.025, length=0.155),
            origin=Origin(xyz=(0.745, y, 0.305), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"grip_{i}",
        )
        frame.visual(
            Box((0.145, 0.060, 0.032)),
            origin=Origin(xyz=(0.505, y, 0.014)),
            material=steel,
            name=f"foot_{i}",
        )

    front_wheel = model.part("front_wheel")
    tire = TireGeometry(
        0.160,
        0.064,
        inner_radius=0.116,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.055),
        tread=TireTread(style="block", depth=0.008, count=18, land_ratio=0.58),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.007, radius=0.003),
    )
    wheel = WheelGeometry(
        0.118,
        0.058,
        rim=WheelRim(
            inner_radius=0.080,
            flange_height=0.009,
            flange_thickness=0.004,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(
            radius=0.032,
            width=0.060,
            cap_style="domed",
            bolt_pattern=BoltPattern(
                count=5,
                circle_diameter=0.040,
                hole_diameter=0.004,
            ),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.012),
        bore=WheelBore(style="round", diameter=0.035),
    )
    # The wheel helpers spin about local X; rotate their local X onto the
    # wheelbarrow's left-right axle (world Y).
    wheel_rot = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    front_wheel.visual(
        mesh_from_geometry(tire, "front_tire"),
        origin=wheel_rot,
        material=black,
        name="tire",
    )
    front_wheel.visual(
        mesh_from_geometry(wheel, "front_wheel_hub"),
        origin=wheel_rot,
        material=yellow,
        name="wheel_hub",
    )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel,
        origin=Origin(xyz=(-0.440, 0.0, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("front_wheel")
    joint = object_model.get_articulation("wheel_axle")

    ctx.check(
        "front wheel has continuous axle joint",
        joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, 1.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="front_axle",
        elem_b="wheel_hub",
        reason="The axle pin is intentionally seated in the wheel hub bore so the rotating wheel is visibly captured by the fork.",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="wheel_hub",
        elem_b="front_axle",
        min_overlap=0.050,
        name="axle spans through wheel hub",
    )
    ctx.expect_overlap(
        frame,
        wheel,
        axes="xz",
        elem_a="front_axle",
        elem_b="wheel_hub",
        min_overlap=0.025,
        name="hub surrounds axle center",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="y",
        inner_elem="tire",
        outer_elem="front_axle",
        margin=0.003,
        name="tire sits between fork axle ends",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    wheel_aabb = ctx.part_world_aabb(wheel)
    if frame_aabb is not None and wheel_aabb is not None:
        min_x = min(frame_aabb[0][0], wheel_aabb[0][0])
        max_x = max(frame_aabb[1][0], wheel_aabb[1][0])
        min_y = min(frame_aabb[0][1], wheel_aabb[0][1])
        max_y = max(frame_aabb[1][1], wheel_aabb[1][1])
        min_z = min(frame_aabb[0][2], wheel_aabb[0][2])
        max_z = max(frame_aabb[1][2], wheel_aabb[1][2])
        ctx.check(
            "compact wheelbarrow envelope",
            (max_x - min_x) < 1.45 and (max_y - min_y) < 0.70 and (max_z - min_z) < 0.62,
            details=f"size={(max_x - min_x, max_y - min_y, max_z - min_z)}",
        )

    return ctx.report()


object_model = build_object_model()
