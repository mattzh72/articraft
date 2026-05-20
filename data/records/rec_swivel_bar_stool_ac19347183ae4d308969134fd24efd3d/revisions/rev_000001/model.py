from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rot_z_point(radius: float, theta: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(theta), radius * math.sin(theta), z)


def _local_to_world(
    origin: tuple[float, float, float],
    theta: float,
    local: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Map local caster coordinates where +X is tangent and +Y is inward radial."""
    tangent = (-math.sin(theta), math.cos(theta), 0.0)
    inward = (-math.cos(theta), -math.sin(theta), 0.0)
    return (
        origin[0] + tangent[0] * local[0] + inward[0] * local[1],
        origin[1] + tangent[1] * local[0] + inward[1] * local[1],
        origin[2] + local[2],
    )


def _cylinder_between(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    horizontal = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx) if horizontal > 1e-9 else 0.0
    pitch = math.atan2(horizontal, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _seat_width_and_height(x: float, y_scale: float) -> tuple[float, float]:
    # Tractor seats are broad at the hips, narrower at the waterfall front, with
    # raised edges and a shallow central dish.
    width = 0.125 + 0.115 * max(0.0, 1.0 - ((x - 0.005) / 0.255) ** 2)
    width += 0.018 * math.exp(-((x + 0.13) / 0.08) ** 2)
    width -= 0.018 * max(0.0, (x - 0.14) / 0.10)
    side_rise = 0.038 * (abs(y_scale) ** 2.15)
    dish = 0.023 * math.exp(-((x - 0.015) / 0.135) ** 2) * (1.0 - abs(y_scale) ** 1.8)
    front_lip = 0.033 * max(0.0, (x - 0.145) / 0.085) ** 2
    rear_roll = 0.021 * max(0.0, (-0.165 - x) / 0.055) ** 2
    z = 0.112 + side_rise - dish + front_lip + rear_roll
    return width, z


def _tractor_seat_shell() -> MeshGeometry:
    geom = MeshGeometry()
    xs = [-0.225, -0.18, -0.125, -0.065, 0.0, 0.065, 0.13, 0.185, 0.235]
    samples_y = 17
    thickness = 0.018

    top_indices: list[list[int]] = []
    bottom_indices: list[list[int]] = []
    for x in xs:
        top_row: list[int] = []
        bottom_row: list[int] = []
        for j in range(samples_y):
            s = -1.0 + 2.0 * j / (samples_y - 1)
            width, z = _seat_width_and_height(x, s)
            y = width * s
            top_row.append(geom.add_vertex(x, y, z))
            bottom_row.append(geom.add_vertex(x, y, z - thickness))
        top_indices.append(top_row)
        bottom_indices.append(bottom_row)

    for i in range(len(xs) - 1):
        for j in range(samples_y - 1):
            a = top_indices[i][j]
            b = top_indices[i + 1][j]
            c = top_indices[i + 1][j + 1]
            d = top_indices[i][j + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

            ab = bottom_indices[i][j]
            bb = bottom_indices[i][j + 1]
            cb = bottom_indices[i + 1][j + 1]
            db = bottom_indices[i + 1][j]
            geom.add_face(ab, bb, cb)
            geom.add_face(ab, cb, db)

    # Side walls along the left and right rolled edges.
    for i in range(len(xs) - 1):
        for j in (0, samples_y - 1):
            a = top_indices[i][j]
            b = bottom_indices[i][j]
            c = bottom_indices[i + 1][j]
            d = top_indices[i + 1][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    # Front and rear closures.
    for i in (0, len(xs) - 1):
        for j in range(samples_y - 1):
            a = top_indices[i][j]
            b = top_indices[i][j + 1]
            c = bottom_indices[i][j + 1]
            d = bottom_indices[i][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    return geom


def _seat_rolled_rim() -> MeshGeometry:
    xs = [-0.225, -0.18, -0.125, -0.065, 0.0, 0.065, 0.13, 0.185, 0.235]
    path: list[tuple[float, float, float]] = []
    for x in xs:
        width, z = _seat_width_and_height(x, 1.0)
        path.append((x, width, z + 0.002))
    for x in reversed(xs):
        width, z = _seat_width_and_height(x, -1.0)
        path.append((x, -width, z + 0.002))
    return tube_from_spline_points(
        path,
        radius=0.009,
        samples_per_segment=8,
        closed_spline=True,
        radial_segments=14,
        cap_ends=False,
    )


def _backrest_pad() -> MeshGeometry:
    geom = MeshGeometry()
    y_steps = 15
    z_steps = 9
    width = 0.36
    height = 0.22
    z0 = 0.07
    thickness = 0.026

    front: list[list[int]] = []
    back: list[list[int]] = []
    for iz in range(z_steps):
        v = iz / (z_steps - 1)
        z = z0 + height * v
        row_front: list[int] = []
        row_back: list[int] = []
        for iy in range(y_steps):
            s = -1.0 + 2.0 * iy / (y_steps - 1)
            y = 0.5 * width * s
            x_mid = -0.058 + 0.023 * (abs(s) ** 1.9)
            crown = 0.008 * math.sin(math.pi * v)
            row_front.append(geom.add_vertex(x_mid + 0.5 * thickness + crown, y, z))
            row_back.append(geom.add_vertex(x_mid - 0.5 * thickness, y, z))
        front.append(row_front)
        back.append(row_back)

    for iz in range(z_steps - 1):
        for iy in range(y_steps - 1):
            a = front[iz][iy]
            b = front[iz + 1][iy]
            c = front[iz + 1][iy + 1]
            d = front[iz][iy + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

            ab = back[iz][iy]
            bb = back[iz][iy + 1]
            cb = back[iz + 1][iy + 1]
            db = back[iz + 1][iy]
            geom.add_face(ab, bb, cb)
            geom.add_face(ab, cb, db)

    for iz in range(z_steps - 1):
        for iy in (0, y_steps - 1):
            a = front[iz][iy]
            b = back[iz][iy]
            c = back[iz + 1][iy]
            d = front[iz + 1][iy]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    for iz in (0, z_steps - 1):
        for iy in range(y_steps - 1):
            a = front[iz][iy]
            b = front[iz][iy + 1]
            c = back[iz][iy + 1]
            d = back[iz][iy]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    return geom


def _flat_foot_ring() -> MeshGeometry:
    """One connected chrome plate/strap mesh for the circular foot ring and its five spokes."""
    geom = MeshGeometry()
    z_mid = 0.36
    half_thick = 0.008
    radial = [0.041, 0.064, 0.160, 0.273, 0.297]
    theta_count = 100
    spoke_half_angle = 0.105

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for r in radial:
        top_row: list[int] = []
        bottom_row: list[int] = []
        for k in range(theta_count):
            theta = math.tau * k / theta_count
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            top_row.append(geom.add_vertex(x, y, z_mid + half_thick))
            bottom_row.append(geom.add_vertex(x, y, z_mid - half_thick))
        top.append(top_row)
        bottom.append(bottom_row)

    def in_spoke(theta: float) -> bool:
        for index in range(5):
            center = index * math.tau / 5.0
            delta = (theta - center + math.pi) % math.tau - math.pi
            if abs(delta) <= spoke_half_angle:
                return True
        return False

    included: set[tuple[int, int]] = set()
    for i in range(len(radial) - 1):
        for k in range(theta_count):
            theta_mid = math.tau * (k + 0.5) / theta_count
            is_collar = i == 0
            is_outer_ring = i == len(radial) - 2
            is_spoke = 1 <= i <= len(radial) - 3 and in_spoke(theta_mid)
            if is_collar or is_outer_ring or is_spoke:
                included.add((i, k))

    def add_quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i, k in sorted(included):
        kn = (k + 1) % theta_count
        add_quad(top[i][k], top[i + 1][k], top[i + 1][kn], top[i][kn])
        add_quad(bottom[i][k], bottom[i][kn], bottom[i + 1][kn], bottom[i + 1][k])

        if (i - 1, k) not in included:
            add_quad(top[i][k], top[i][kn], bottom[i][kn], bottom[i][k])
        if (i + 1, k) not in included:
            add_quad(top[i + 1][k], bottom[i + 1][k], bottom[i + 1][kn], top[i + 1][kn])
        if (i, (k - 1) % theta_count) not in included:
            add_quad(top[i][k], bottom[i][k], bottom[i + 1][k], top[i + 1][k])
        if (i, kn) not in included:
            add_quad(top[i][kn], top[i + 1][kn], bottom[i + 1][kn], bottom[i][kn])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_stool")

    chrome = model.material("brushed_chrome", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.015, 0.014, 0.013, 1.0))
    edge_vinyl = model.material("edge_vinyl", rgba=(0.0, 0.0, 0.0, 1.0))
    tire_rubber = model.material("caster_rubber", rgba=(0.02, 0.02, 0.02, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.38, 0.39, 0.40, 1.0))

    outer_sleeve_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.043, 0.145), (0.043, 0.535)],
            [(0.029, 0.145), (0.029, 0.535)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=5,
        ),
        "outer_lift_sleeve",
    )

    foot_ring_mesh = mesh_from_geometry(_flat_foot_ring(), "foot_ring")

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.047,
            0.040,
            inner_radius=0.031,
            carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.003, count=14, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.070, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=satin_gray,
        name="lower_hub",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.197)),
        material=chrome,
        name="hub_collar",
    )
    base.visual(outer_sleeve_mesh, material=chrome, name="outer_sleeve")
    base.visual(foot_ring_mesh, material=chrome, name="foot_ring")

    for index in range(5):
        theta = index * math.tau / 5.0
        leg_start = _rot_z_point(0.060, theta, 0.155)
        leg_end = _rot_z_point(0.362, theta, 0.125)
        _cylinder_between(base, leg_start, leg_end, 0.017, chrome, f"leg_{index}")

        wheel_center = _rot_z_point(0.382, theta, 0.047)
        yaw = theta + math.pi / 2.0
        for side, offset in (("a", 0.052), ("b", -0.052)):
            base.visual(
                Box((0.006, 0.060, 0.085)),
                origin=Origin(
                    xyz=_local_to_world(wheel_center, theta, (offset, 0.000, 0.030)),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=dark_metal,
                name=f"fork_{index}_{side}",
            )
        base.visual(
            Box((0.116, 0.050, 0.014)),
            origin=Origin(
                xyz=_local_to_world(wheel_center, theta, (0.0, 0.004, 0.071)),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_metal,
            name=f"fork_bridge_{index}",
        )
        _cylinder_between(
            base,
            _local_to_world(wheel_center, theta, (-0.062, 0.0, 0.0)),
            _local_to_world(wheel_center, theta, (0.062, 0.0, 0.0)),
            0.0055,
            chrome,
            f"axle_{index}",
        )
        stem_bottom = _rot_z_point(0.360, theta, 0.105)
        stem_top = _rot_z_point(0.360, theta, 0.152)
        _cylinder_between(base, stem_bottom, stem_top, 0.010, dark_metal, f"caster_stem_{index}")

    lift_column = model.part("lift_column")
    lift_column.visual(
        Cylinder(radius=0.022, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=chrome,
        name="inner_piston",
    )
    lift_column.visual(
        Cylinder(radius=0.041, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=satin_gray,
        name="swivel_cap",
    )
    lift_column.visual(
        Cylinder(radius=0.041, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=satin_gray,
        name="stop_collar",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_geometry(_tractor_seat_shell(), "tractor_seat_shell"),
        material=black_vinyl,
        name="seat_shell",
    )
    seat.visual(
        mesh_from_geometry(_seat_rolled_rim(), "seat_rolled_rim"),
        material=edge_vinyl,
        name="seat_rim",
    )
    seat.visual(
        Cylinder(radius=0.085, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=satin_gray,
        name="swivel_plate",
    )
    seat.visual(
        Cylinder(radius=0.052, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_metal,
        name="seat_post",
    )
    seat.visual(
        Box((0.210, 0.155, 0.020)),
        origin=Origin(xyz=(-0.010, 0.0, 0.080)),
        material=dark_metal,
        name="under_pan",
    )
    seat.visual(
        Box((0.090, 0.025, 0.070)),
        origin=Origin(xyz=(-0.248, 0.118, 0.126)),
        material=dark_metal,
        name="hinge_tab_0",
    )
    seat.visual(
        Box((0.090, 0.025, 0.070)),
        origin=Origin(xyz=(-0.248, -0.118, 0.126)),
        material=dark_metal,
        name="hinge_tab_1",
    )
    seat.visual(
        Cylinder(radius=0.014, length=0.057),
        origin=Origin(xyz=(-0.285, 0.123, 0.157), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel_0",
    )
    seat.visual(
        Cylinder(radius=0.014, length=0.057),
        origin=Origin(xyz=(-0.285, -0.123, 0.157), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel_1",
    )
    seat.visual(
        Cylinder(radius=0.006, length=0.300),
        origin=Origin(xyz=(-0.285, 0.0, 0.157), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )

    backrest = model.part("backrest")
    backrest.visual(
        mesh_from_geometry(_backrest_pad(), "backrest_pad"),
        material=black_vinyl,
        name="back_pad",
    )
    backrest.visual(
        Cylinder(radius=0.013, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="center_barrel",
    )
    _cylinder_between(backrest, (-0.014, 0.042, 0.005), (-0.065, 0.086, 0.094), 0.007, chrome, "back_strut_0")
    _cylinder_between(backrest, (-0.014, -0.042, 0.005), (-0.065, -0.086, 0.094), 0.007, chrome, "back_strut_1")

    wheel_parts = []
    for index in range(5):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(tire_mesh, material=tire_rubber, name="tire")
        wheel.visual(
            Cylinder(radius=0.034, length=0.034),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_gray,
            name="rim_core",
        )
        wheel_parts.append(wheel)

    model.articulation(
        "lift_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.25, lower=0.0, upper=0.22),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=lift_column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0),
    )
    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.285, 0.0, 0.157)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.25, upper=0.55),
    )
    for index, wheel in enumerate(wheel_parts):
        theta = index * math.tau / 5.0
        model.articulation(
            f"caster_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=_rot_z_point(0.382, theta, 0.047), rpy=(0.0, 0.0, theta + math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lift = object_model.get_part("lift_column")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    lift_slide = object_model.get_articulation("lift_slide")
    seat_swivel = object_model.get_articulation("seat_swivel")
    back_hinge = object_model.get_articulation("back_hinge")

    spin_joints = [object_model.get_articulation(f"caster_spin_{i}") for i in range(5)]
    for index in range(5):
        ctx.allow_overlap(
            "base",
            f"wheel_{index}",
            elem_a=f"axle_{index}",
            elem_b="rim_core",
            reason="The caster axle is intentionally captured through the wheel hub proxy so the wheel reads as mounted while spinning.",
        )
        ctx.expect_overlap(
            "base",
            f"wheel_{index}",
            axes="z",
            elem_a=f"axle_{index}",
            elem_b="rim_core",
            min_overlap=0.006,
            name=f"caster {index} axle passes through hub",
        )
    ctx.allow_overlap(
        "seat",
        "backrest",
        elem_a="hinge_pin",
        elem_b="center_barrel",
        reason="The backrest hinge pin is intentionally captured inside the short center barrel.",
    )
    ctx.expect_overlap(
        "seat",
        "backrest",
        axes="y",
        elem_a="hinge_pin",
        elem_b="center_barrel",
        min_overlap=0.10,
        name="backrest hinge pin spans center barrel",
    )

    ctx.check(
        "five caster wheels spin",
        len(spin_joints) == 5 and all(j.articulation_type == ArticulationType.CONTINUOUS for j in spin_joints),
    )
    ctx.check("lift column is prismatic", lift_slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("seat has continuous swivel", seat_swivel.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("backrest has limited hinge", back_hinge.articulation_type == ArticulationType.REVOLUTE)

    with ctx.pose({lift_slide: 0.0, seat_swivel: 0.0}):
        low_seat = ctx.part_world_position(seat)
        low_lift = ctx.part_world_position(lift)
    with ctx.pose({lift_slide: 0.20, seat_swivel: 0.0}):
        high_seat = ctx.part_world_position(seat)
        high_lift = ctx.part_world_position(lift)
    ctx.check(
        "gas lift raises seat",
        low_seat is not None
        and high_seat is not None
        and low_lift is not None
        and high_lift is not None
        and high_seat[2] > low_seat[2] + 0.18
        and high_lift[2] > low_lift[2] + 0.18,
        details=f"low_seat={low_seat}, high_seat={high_seat}, low_lift={low_lift}, high_lift={high_lift}",
    )

    with ctx.pose({seat_swivel: 0.0}):
        seat_center_a = ctx.part_world_position(seat)
    with ctx.pose({seat_swivel: math.pi * 1.25}):
        seat_center_b = ctx.part_world_position(seat)
    ctx.check(
        "seat swivels about lift axis",
        seat_center_a is not None
        and seat_center_b is not None
        and abs(seat_center_a[0] - seat_center_b[0]) < 1e-6
        and abs(seat_center_a[1] - seat_center_b[1]) < 1e-6,
        details=f"before={seat_center_a}, after={seat_center_b}",
    )

    with ctx.pose({back_hinge: 0.0}):
        back_rest_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 0.45}):
        back_tilt_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest tilts rearward",
        back_rest_aabb is not None
        and back_tilt_aabb is not None
        and back_tilt_aabb[0][0] < back_rest_aabb[0][0] - 0.035,
        details=f"rest={back_rest_aabb}, tilted={back_tilt_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
