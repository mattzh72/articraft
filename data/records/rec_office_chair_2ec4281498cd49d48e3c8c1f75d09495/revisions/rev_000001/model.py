from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _superellipse_point(rx: float, ry: float, theta: float, exponent: float) -> tuple[float, float]:
    c = cos(theta)
    s = sin(theta)
    # Superellipse parameterization: exponent > 2 gives flatter sides with
    # rounded corners, which reads like molded furniture shells.
    x = rx * (1.0 if c >= 0.0 else -1.0) * (abs(c) ** (2.0 / exponent))
    y = ry * (1.0 if s >= 0.0 else -1.0) * (abs(s) ** (2.0 / exponent))
    return x, y


def _seat_pan_geometry() -> MeshGeometry:
    """Thin, dished molded seat shell in local chair coordinates.

    X is front/back (+X front), Y is chair width, Z is up.  The mesh is a
    closed thin shell, not a solid block, with a raised waterfall front edge and
    slightly upturned sides.
    """

    geom = MeshGeometry()
    half_depth = 0.245
    half_width = 0.245
    thickness = 0.018
    segments = 72
    ring_scales = (0.25, 0.52, 0.78, 1.0)

    def top_z(x: float, y: float, scale: float) -> float:
        x_n = x / half_depth
        y_n = y / half_width
        front_lip = 0.017 * max(0.0, x_n) ** 2
        rear_lip = 0.010 * max(0.0, -x_n) ** 2
        side_lip = 0.008 * abs(y_n) ** 2
        bowl = -0.011 * (1.0 - scale)
        return 0.058 + front_lip + rear_lip + side_lip + bowl

    top_center = geom.add_vertex(0.0, 0.0, top_z(0.0, 0.0, 0.0))
    bottom_center = geom.add_vertex(0.0, 0.0, top_z(0.0, 0.0, 0.0) - thickness)
    top_rings: list[list[int]] = []
    bottom_rings: list[list[int]] = []

    for scale in ring_scales:
        top_ring: list[int] = []
        bottom_ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * pi * i / segments
            x, y = _superellipse_point(half_depth * scale, half_width * scale, theta, 3.2)
            z = top_z(x, y, scale)
            top_ring.append(geom.add_vertex(x, y, z))
            bottom_ring.append(geom.add_vertex(x, y, z - thickness))
        top_rings.append(top_ring)
        bottom_rings.append(bottom_ring)

    first_top = top_rings[0]
    first_bottom = bottom_rings[0]
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(top_center, first_top[i], first_top[j])
        geom.add_face(bottom_center, first_bottom[j], first_bottom[i])

    for ring_index in range(len(ring_scales) - 1):
        inner_top = top_rings[ring_index]
        outer_top = top_rings[ring_index + 1]
        inner_bottom = bottom_rings[ring_index]
        outer_bottom = bottom_rings[ring_index + 1]
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(inner_top[i], outer_top[i], outer_top[j])
            geom.add_face(inner_top[i], outer_top[j], inner_top[j])
            geom.add_face(inner_bottom[i], outer_bottom[j], outer_bottom[i])
            geom.add_face(inner_bottom[i], inner_bottom[j], outer_bottom[j])

    outer_top = top_rings[-1]
    outer_bottom = bottom_rings[-1]
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer_bottom[i], outer_top[i], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_bottom[j])

    return geom


def _back_shell_geometry() -> MeshGeometry:
    """Curved reclining back shell with a continuous thin perimeter."""

    geom = MeshGeometry()
    half_width = 0.235
    half_height = 0.285
    center_z = 0.335
    thickness = 0.020
    segments = 72
    ring_scales = (0.24, 0.50, 0.76, 1.0)

    def center_x(y: float, z: float) -> float:
        v = max(0.0, min(1.0, (z - (center_z - half_height)) / (2.0 * half_height)))
        side_relief = 0.020 * (abs(y) / half_width) ** 1.8
        lumbar_scoop = -0.020 * (1.0 - (abs(y) / half_width) ** 1.6) * sin(pi * v)
        return -0.030 - 0.105 * v + side_relief + lumbar_scoop

    front_center = geom.add_vertex(center_x(0.0, center_z) + thickness * 0.5, 0.0, center_z)
    rear_center = geom.add_vertex(center_x(0.0, center_z) - thickness * 0.5, 0.0, center_z)
    front_rings: list[list[int]] = []
    rear_rings: list[list[int]] = []

    for scale in ring_scales:
        front_ring: list[int] = []
        rear_ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * pi * i / segments
            y, z_rel = _superellipse_point(half_width * scale, half_height * scale, theta, 2.9)
            z = center_z + z_rel
            x = center_x(y, z)
            front_ring.append(geom.add_vertex(x + thickness * 0.5, y, z))
            rear_ring.append(geom.add_vertex(x - thickness * 0.5, y, z))
        front_rings.append(front_ring)
        rear_rings.append(rear_ring)

    first_front = front_rings[0]
    first_rear = rear_rings[0]
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(front_center, first_front[i], first_front[j])
        geom.add_face(rear_center, first_rear[j], first_rear[i])

    for ring_index in range(len(ring_scales) - 1):
        inner_front = front_rings[ring_index]
        outer_front = front_rings[ring_index + 1]
        inner_rear = rear_rings[ring_index]
        outer_rear = rear_rings[ring_index + 1]
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(inner_front[i], outer_front[i], outer_front[j])
            geom.add_face(inner_front[i], outer_front[j], inner_front[j])
            geom.add_face(inner_rear[i], outer_rear[j], outer_rear[i])
            geom.add_face(inner_rear[i], inner_rear[j], outer_rear[j])

    outer_front = front_rings[-1]
    outer_rear = rear_rings[-1]
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer_rear[i], outer_front[i], outer_front[j])
        geom.add_face(outer_rear[i], outer_front[j], outer_rear[j])

    return geom


def _add_caster_wheel_visuals(part, prefix: str, rubber, nylon, metal) -> None:
    tire = TireGeometry(
        0.035,
        0.024,
        inner_radius=0.020,
        tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.62),
        grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.0035, radius=0.0015),
    )
    wheel = WheelGeometry(
        0.022,
        0.026,
        rim=WheelRim(inner_radius=0.013, flange_height=0.002, flange_thickness=0.0015),
        hub=WheelHub(radius=0.010, width=0.028, cap_style="flat"),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.004),
        bore=WheelBore(style="round", diameter=0.006),
    )
    part.visual(mesh_from_geometry(tire, f"{prefix}_tire"), material=rubber, name="tire")
    part.visual(mesh_from_geometry(wheel, f"{prefix}_rim"), material=nylon, name="rim")
    part.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="hub_sleeve",
    )


def _add_caster_fork_visuals(part, metal, rubber) -> None:
    # Child frame is at the caster swivel bearing.  The wheel axle sits directly
    # below it, giving the caster fork visible vertical-swivel and wheel-spin
    # axes without any unsupported floating members.
    part.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=metal,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=metal,
        name="swivel_collar",
    )
    part.visual(
        Box((0.064, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.010, -0.010)),
        material=metal,
        name="fork_bridge",
    )
    for side, x in enumerate((-0.024, 0.024)):
        part.visual(
            Box((0.010, 0.054, 0.078)),
            origin=Origin(xyz=(x, 0.0, -0.055)),
            material=metal,
            name=f"fork_cheek_{side}",
        )
        part.visual(
            Cylinder(radius=0.006, length=0.007),
            origin=Origin(xyz=(x, 0.0, -0.055), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name=f"axle_cap_{side}",
        )
    part.visual(
        Box((0.056, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.022, -0.090)),
        material=rubber,
        name="tread_guard",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_star_swivel_meeting_chair")

    satin_black = model.material("satin_black", rgba=(0.025, 0.026, 0.028, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    polished_metal = model.material("polished_metal", rgba=(0.72, 0.74, 0.75, 1.0))
    blue_shell = model.material("blue_shell", rgba=(0.12, 0.24, 0.42, 1.0))
    dark_nylon = model.material("dark_nylon", rgba=(0.08, 0.08, 0.085, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.090, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=graphite,
        name="lower_hub",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=polished_metal,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=satin_black,
        name="pedestal_shroud",
    )
    base.visual(
        Cylinder(radius=0.085, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=graphite,
        name="upper_bearing",
    )

    caster_radius = 0.340
    arm_length = 0.315
    arm_mid = 0.175
    caster_angles = (pi / 4.0, 3.0 * pi / 4.0, 5.0 * pi / 4.0, 7.0 * pi / 4.0)
    for i, angle in enumerate(caster_angles):
        base.visual(
            Box((arm_length, 0.060, 0.030)),
            origin=Origin(
                xyz=(arm_mid * cos(angle), arm_mid * sin(angle), 0.105),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"star_arm_{i}",
        )
        base.visual(
            Cylinder(radius=0.038, length=0.030),
            origin=Origin(xyz=(caster_radius * cos(angle), caster_radius * sin(angle), 0.105)),
            material=graphite,
            name=f"caster_socket_{i}",
        )

    seat_shell = model.part("seat_shell")
    seat_shell.visual(
        mesh_from_geometry(_seat_pan_geometry(), "seat_pan"),
        material=blue_shell,
        name="seat_pan",
    )
    seat_shell.visual(
        Cylinder(radius=0.085, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=graphite,
        name="turntable_plate",
    )
    seat_shell.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=satin_black,
        name="seat_boss",
    )
    seat_shell.visual(
        Box((0.315, 0.075, 0.028)),
        origin=Origin(xyz=(-0.050, 0.0, 0.040)),
        material=satin_black,
        name="underseat_rib",
    )
    seat_shell.visual(
        Cylinder(radius=0.010, length=0.210),
        origin=Origin(xyz=(-0.270, 0.0, 0.060), rpy=(pi / 2.0, 0.0, 0.0)),
        material=polished_metal,
        name="back_hinge_pin",
    )
    for side, y in enumerate((-0.185, 0.185)):
        seat_shell.visual(
            Box((0.040, 0.045, 0.050)),
            origin=Origin(xyz=(-0.265, y, 0.058)),
            material=satin_black,
            name=f"hinge_lug_{side}",
        )
    seat_shell.visual(
        Box((0.050, 0.210, 0.020)),
        origin=Origin(xyz=(-0.250, 0.0, 0.060)),
        material=satin_black,
        name="rear_hinge_bridge",
    )
    seat_shell.visual(
        Box((0.030, 0.055, 0.030)),
        origin=Origin(xyz=(-0.235, -0.136, 0.055)),
        material=satin_black,
        name="hinge_web_0",
    )
    seat_shell.visual(
        Box((0.030, 0.055, 0.030)),
        origin=Origin(xyz=(-0.235, 0.136, 0.055)),
        material=satin_black,
        name="hinge_web_1",
    )

    back_shell = model.part("back_shell")
    back_shell.visual(
        mesh_from_geometry(_back_shell_geometry(), "back_shell"),
        material=blue_shell,
        name="back_panel",
    )
    back_shell.visual(
        Box((0.040, 0.365, 0.030)),
        origin=Origin(xyz=(-0.055, 0.0, 0.055)),
        material=satin_black,
        name="lower_hinge_rail",
    )
    back_shell.visual(
        Box((0.026, 0.050, 0.080)),
        origin=Origin(xyz=(0.003, -0.1375, 0.030)),
        material=satin_black,
        name="hinge_ear_0",
    )
    back_shell.visual(
        Box((0.026, 0.050, 0.080)),
        origin=Origin(xyz=(0.003, 0.1375, 0.030)),
        material=satin_black,
        name="hinge_ear_1",
    )
    back_shell.visual(
        Box((0.050, 0.040, 0.050)),
        origin=Origin(xyz=(-0.020, -0.1375, 0.030)),
        material=satin_black,
        name="hinge_bridge_0",
    )
    back_shell.visual(
        Box((0.050, 0.040, 0.050)),
        origin=Origin(xyz=(-0.020, 0.1375, 0.030)),
        material=satin_black,
        name="hinge_bridge_1",
    )

    caster_forks = []
    caster_wheels = []
    for i in range(4):
        fork = model.part(f"caster_fork_{i}")
        _add_caster_fork_visuals(fork, polished_metal, black_rubber)
        caster_forks.append(fork)

        wheel = model.part(f"caster_wheel_{i}")
        _add_caster_wheel_visuals(wheel, f"caster_wheel_{i}", black_rubber, dark_nylon, polished_metal)
        caster_wheels.append(wheel)

    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )
    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=back_shell,
        origin=Origin(xyz=(-0.270, 0.0, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=0.45),
    )

    for i, angle in enumerate(caster_angles):
        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster_forks[i],
            origin=Origin(
                xyz=(caster_radius * cos(angle), caster_radius * sin(angle), 0.090),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=8.0),
        )
        model.articulation(
            f"caster_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=caster_forks[i],
            child=caster_wheels[i],
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat_shell = object_model.get_part("seat_shell")
    back_shell = object_model.get_part("back_shell")
    seat_swivel = object_model.get_articulation("base_to_seat")
    back_recline = object_model.get_articulation("seat_to_back")

    ctx.expect_gap(
        seat_shell,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_plate",
        negative_elem="upper_bearing",
        name="seat turntable rests on pedestal bearing",
    )
    ctx.expect_overlap(
        seat_shell,
        base,
        axes="xy",
        min_overlap=0.090,
        elem_a="turntable_plate",
        elem_b="upper_bearing",
        name="seat swivel bearing is centered over pedestal",
    )

    rest_back_aabb = ctx.part_world_aabb(back_shell)
    with ctx.pose({back_recline: 0.45}):
        reclined_back_aabb = ctx.part_world_aabb(back_shell)
    ctx.check(
        "back shell reclines rearward",
        rest_back_aabb is not None
        and reclined_back_aabb is not None
        and reclined_back_aabb[0][0] < rest_back_aabb[0][0] - 0.10,
        details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
    )

    with ctx.pose({seat_swivel: pi / 2.0}):
        ctx.expect_gap(
            seat_shell,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="turntable_plate",
            negative_elem="upper_bearing",
            name="seat remains seated while swiveled",
        )

    for i in range(4):
        fork = object_model.get_part(f"caster_fork_{i}")
        wheel = object_model.get_part(f"caster_wheel_{i}")
        ctx.expect_overlap(
            wheel,
            fork,
            axes="xy",
            min_overlap=0.010,
            name=f"caster wheel {i} is captured inside fork",
        )
        aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"caster wheel {i} touches floor plane",
            aabb is not None and abs(aabb[0][2]) < 0.004,
            details=f"wheel_aabb={aabb}",
        )
        ctx.check(
            f"caster wheel {i} has spin joint",
            object_model.get_articulation(f"caster_{i}_spin") is not None,
        )

    return ctx.report()


object_model = build_object_model()
