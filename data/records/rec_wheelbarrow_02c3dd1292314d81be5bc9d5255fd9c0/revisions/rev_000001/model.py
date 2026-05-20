from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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
    tube_from_spline_points,
    wire_from_points,
)


def _trapezoid_loop(x_rear: float, x_front: float, y_rear: float, y_front: float, z: float) -> list[tuple[float, float, float]]:
    """A consistent 16 point loop around a tapered wheelbarrow tray section."""
    corners = [
        (x_rear, -y_rear, z),
        (x_rear, y_rear, z),
        (x_front, y_front, z),
        (x_front, -y_front, z),
    ]
    pts: list[tuple[float, float, float]] = []
    steps = 4
    for i, start in enumerate(corners):
        end = corners[(i + 1) % len(corners)]
        for j in range(steps):
            t = j / steps
            pts.append(
                (
                    start[0] * (1.0 - t) + end[0] * t,
                    start[1] * (1.0 - t) + end[1] * t,
                    start[2] * (1.0 - t) + end[2] * t,
                )
            )
    return pts


def _add_loop(mesh: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y, z in points]


def _connect_loops(mesh: MeshGeometry, a: list[int], b: list[int], *, reverse: bool = False) -> None:
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        if reverse:
            mesh.add_face(a[i], b[i], b[j])
            mesh.add_face(a[i], b[j], a[j])
        else:
            mesh.add_face(a[i], b[j], b[i])
            mesh.add_face(a[i], a[j], b[j])


def _cap_loop(mesh: MeshGeometry, loop: list[int], *, reverse: bool = False) -> None:
    center = mesh.add_vertex(
        sum(mesh.vertices[i][0] for i in loop) / len(loop),
        sum(mesh.vertices[i][1] for i in loop) / len(loop),
        sum(mesh.vertices[i][2] for i in loop) / len(loop),
    )
    for i in range(len(loop)):
        j = (i + 1) % len(loop)
        if reverse:
            mesh.add_face(center, loop[j], loop[i])
        else:
            mesh.add_face(center, loop[i], loop[j])


def _tray_shell_geometry() -> MeshGeometry:
    """Open, thin-walled, tapered metal tray with a real inside basin."""
    mesh = MeshGeometry()
    outer_top = _add_loop(mesh, _trapezoid_loop(-0.56, 0.53, 0.365, 0.285, 0.535))
    inner_top = _add_loop(mesh, _trapezoid_loop(-0.525, 0.495, 0.320, 0.245, 0.503))
    outer_bottom = _add_loop(mesh, _trapezoid_loop(-0.405, 0.380, 0.235, 0.165, 0.280))
    inner_bottom = _add_loop(mesh, _trapezoid_loop(-0.365, 0.340, 0.190, 0.125, 0.318))

    _connect_loops(mesh, outer_top, outer_bottom)
    _connect_loops(mesh, inner_bottom, inner_top, reverse=True)
    _connect_loops(mesh, outer_top, inner_top, reverse=True)
    _connect_loops(mesh, outer_bottom, inner_bottom)
    _cap_loop(mesh, outer_bottom, reverse=True)
    _cap_loop(mesh, inner_bottom)
    return mesh


def _tray_rim_geometry() -> MeshGeometry:
    return wire_from_points(
        _trapezoid_loop(-0.56, 0.53, 0.365, 0.285, 0.545),
        radius=0.014,
        radial_segments=14,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.055,
        cap_ends=False,
    )


def _frame_tubes_geometry() -> MeshGeometry:
    """Welded tubular handles and cradle, including the narrow front support."""
    frame = MeshGeometry()
    for sign in (-1.0, 1.0):
        frame.merge(
            tube_from_spline_points(
                [
                    (-1.18, sign * 0.405, 0.565),
                    (-0.88, sign * 0.395, 0.520),
                    (-0.48, sign * 0.340, 0.365),
                    (0.10, sign * 0.235, 0.285),
                    (0.50, sign * 0.075, 0.235),
                ],
                radius=0.016,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )

    for x, y, z, r in [
        (-0.62, 0.390, 0.425, 0.016),
        (-0.16, 0.285, 0.292, 0.015),
        (0.24, 0.205, 0.272, 0.015),
        (0.48, 0.086, 0.235, 0.014),
    ]:
        frame.merge(
            wire_from_points(
                [(x, -y, z), (x, y, z)],
                radius=r,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return frame


def _rear_support_geometry() -> MeshGeometry:
    rear = MeshGeometry()
    for sign in (-1.0, 1.0):
        rear.merge(
            wire_from_points(
                [
                    (-0.48, sign * 0.340, 0.365),
                    (-0.68, sign * 0.455, 0.175),
                    (-0.725, sign * 0.470, 0.040),
                ],
                radius=0.018,
                radial_segments=14,
                corner_mode="fillet",
                corner_radius=0.035,
                cap_ends=True,
            )
        )
    rear.merge(
        wire_from_points(
            [(-0.700, -0.470, 0.095), (-0.700, 0.470, 0.095)],
            radius=0.016,
            radial_segments=14,
            cap_ends=True,
        )
    )
    return rear


def _front_fork_geometry() -> MeshGeometry:
    fork = MeshGeometry()
    for sign in (-1.0, 1.0):
        fork.merge(
            wire_from_points(
                [
                    (0.495, sign * 0.074, 0.245),
                    (0.585, sign * 0.066, 0.205),
                    (0.670, sign * 0.060, 0.17835),
                ],
                radius=0.013,
                radial_segments=14,
                corner_mode="fillet",
                corner_radius=0.030,
                cap_ends=True,
            )
        )
        # Short axle bosses stop just outside the tire, leaving the wheel free.
        fork.merge(
            wire_from_points(
                [(0.670, sign * 0.055, 0.17835), (0.670, sign * 0.090, 0.17835)],
                radius=0.010,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return fork


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_green = model.material("painted_green", rgba=(0.08, 0.42, 0.16, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.055, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    gray_steel = model.material("galvanized_steel", rgba=(0.56, 0.58, 0.56, 1.0))

    barrow = model.part("barrow")
    barrow.visual(
        mesh_from_geometry(_tray_shell_geometry(), "tray_shell"),
        material=tray_green,
        name="tray_shell",
    )
    barrow.visual(
        mesh_from_geometry(_tray_rim_geometry(), "tray_rim"),
        material=tray_green,
        name="tray_rim",
    )
    barrow.visual(
        mesh_from_geometry(_frame_tubes_geometry(), "frame_tubes"),
        material=dark_steel,
        name="frame_tubes",
    )
    barrow.visual(
        mesh_from_geometry(_front_fork_geometry(), "front_fork"),
        material=dark_steel,
        name="front_fork",
    )
    barrow.visual(
        mesh_from_geometry(_rear_support_geometry(), "rear_support"),
        material=dark_steel,
        name="rear_support",
    )
    for idx, y in enumerate((-0.405, 0.405)):
        barrow.visual(
            Cylinder(radius=0.026, length=0.255),
            origin=Origin(xyz=(-1.155, y, 0.565), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name=f"handle_grip_{idx}",
        )
    for idx, y in enumerate((-0.470, 0.470)):
        barrow.visual(
            Box((0.185, 0.072, 0.032)),
            origin=Origin(xyz=(-0.725, y, 0.020)),
            material=rubber,
            name=f"rear_foot_{idx}",
        )
    barrow.visual(
        Cylinder(radius=0.016, length=0.190),
        origin=Origin(xyz=(0.670, 0.0, 0.17835), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gray_steel,
        name="wheel_axle_pin",
    )

    wheel = model.part("wheel")
    tire = TireGeometry(
        0.170,
        0.075,
        inner_radius=0.116,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
        tread=TireTread(style="block", depth=0.010, count=18, land_ratio=0.56),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )
    rim = WheelGeometry(
        0.118,
        0.060,
        rim=WheelRim(
            inner_radius=0.073,
            flange_height=0.009,
            flange_thickness=0.004,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(
            radius=0.033,
            width=0.052,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.046, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.012),
        bore=WheelBore(style="round", diameter=0.030),
    )
    wheel.visual(
        mesh_from_geometry(tire, "front_tire"),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(rim, "front_rim"),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=gray_steel,
        name="rim",
    )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=barrow,
        child=wheel,
        origin=Origin(xyz=(0.670, 0.0, 0.17835)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=24.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrow = object_model.get_part("barrow")
    wheel = object_model.get_part("wheel")
    axle = object_model.get_articulation("wheel_axle")

    ctx.allow_overlap(
        barrow,
        wheel,
        elem_a="wheel_axle_pin",
        elem_b="rim",
        reason="The stationary axle pin is intentionally given a tiny interference fit inside the hub bore to show the wheel captured on its axle.",
    )

    ctx.check(
        "front wheel has continuous axle joint",
        axle.articulation_type == ArticulationType.CONTINUOUS and axle.axis == (0.0, 1.0, 0.0),
        details=f"type={axle.articulation_type}, axis={axle.axis}",
    )

    ctx.expect_within(
        barrow,
        wheel,
        axes="xz",
        inner_elem="wheel_axle_pin",
        outer_elem="rim",
        margin=0.001,
        name="axle pin is centered within the wheel hub",
    )
    ctx.expect_overlap(
        barrow,
        wheel,
        axes="y",
        elem_a="wheel_axle_pin",
        elem_b="rim",
        min_overlap=0.050,
        name="axle pin passes through the hub width",
    )

    ctx.expect_within(
        wheel,
        barrow,
        axes="y",
        inner_elem="tire",
        outer_elem="front_fork",
        margin=0.012,
        name="front tire is captured between fork cheeks",
    )
    ctx.expect_overlap(
        wheel,
        barrow,
        axes="xz",
        elem_a="tire",
        elem_b="front_fork",
        min_overlap=0.035,
        name="fork straddles wheel at axle height",
    )

    front_fork_aabb = ctx.part_element_world_aabb(barrow, elem="front_fork")
    rear_0 = ctx.part_element_world_aabb(barrow, elem="rear_foot_0")
    rear_1 = ctx.part_element_world_aabb(barrow, elem="rear_foot_1")
    if front_fork_aabb is not None and rear_0 is not None and rear_1 is not None:
        front_width = front_fork_aabb[1][1] - front_fork_aabb[0][1]
        rear_width = max(rear_0[1][1], rear_1[1][1]) - min(rear_0[0][1], rear_1[0][1])
        ctx.check(
            "rear support is broad and front support is narrow",
            rear_width > 0.85 and front_width < 0.20 and rear_width > front_width * 4.0,
            details=f"rear_width={rear_width:.3f}, front_width={front_width:.3f}",
        )
    else:
        ctx.fail("rear/front support widths measurable", "missing support element AABB")

    tray_aabb = ctx.part_element_world_aabb(barrow, elem="tray_shell")
    grip_0 = ctx.part_element_world_aabb(barrow, elem="handle_grip_0")
    grip_1 = ctx.part_element_world_aabb(barrow, elem="handle_grip_1")
    if tray_aabb is not None and grip_0 is not None and grip_1 is not None:
        grip_min_y = min(grip_0[0][1], grip_1[0][1])
        grip_max_y = max(grip_0[1][1], grip_1[1][1])
        ctx.check(
            "tray sits between the two handles",
            grip_min_y < tray_aabb[0][1] - 0.015 and grip_max_y > tray_aabb[1][1] + 0.015,
            details=f"tray_y=({tray_aabb[0][1]:.3f}, {tray_aabb[1][1]:.3f}), handles_y=({grip_min_y:.3f}, {grip_max_y:.3f})",
        )
    else:
        ctx.fail("tray and handle positions measurable", "missing tray or grip element AABB")

    wheel_aabb = ctx.part_world_aabb(wheel)
    ctx.check(
        "wheel rests at ground height",
        wheel_aabb is not None and abs(wheel_aabb[0][2]) < 0.006,
        details=f"wheel_aabb={wheel_aabb}",
    )

    rest_position = ctx.part_world_position(wheel)
    with ctx.pose({axle: pi / 2.0}):
        turned_position = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spins in place about axle",
        rest_position is not None and turned_position is not None and all(abs(a - b) < 1e-6 for a, b in zip(rest_position, turned_position)),
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
