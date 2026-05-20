from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _blade_geometry(angle: float) -> MeshGeometry:
    """A single broad, swept, slightly pitched metal ceiling-fan blade."""
    geom = MeshGeometry()

    root_radius = 0.105
    blade_length = 0.690
    sweep = 0.34
    z_base = 0.014
    thickness = 0.008
    stations = 16

    def centerline(u: float) -> tuple[float, float]:
        r = root_radius + blade_length * u
        theta = sweep * (u**1.35)
        return (r * math.cos(theta), r * math.sin(theta))

    def chord(u: float) -> float:
        return 0.118 + 0.060 * math.sin(math.pi * u) - 0.014 * u

    def rotate_xy(x: float, y: float) -> tuple[float, float]:
        c = math.cos(angle)
        s = math.sin(angle)
        return (c * x - s * y, s * x + c * y)

    rings: list[tuple[int, int, int, int]] = []
    for i in range(stations):
        u = i / (stations - 1)
        u0 = max(0.0, u - 0.01)
        u1 = min(1.0, u + 0.01)
        x0, y0 = centerline(u0)
        x1, y1 = centerline(u1)
        tx = x1 - x0
        ty = y1 - y0
        t_len = math.hypot(tx, ty) or 1.0
        # A normal to the swept centerline gives each station a graceful curve.
        nx = -ty / t_len
        ny = tx / t_len

        cx, cy = centerline(u)
        half_chord = chord(u) * 0.5
        pitch = 0.018 * (1.0 - 0.45 * u)
        camber = -0.005 * math.sin(math.pi * u)

        pts = []
        for side, v in (("left", -1.0), ("right", 1.0)):
            px = cx + nx * half_chord * v
            py = cy + ny * half_chord * v
            z = z_base + camber - pitch * v
            rx, ry = rotate_xy(px, py)
            pts.append((rx, ry, z))

        left, right = pts
        lt = geom.add_vertex(left[0], left[1], left[2] - thickness * 0.5)
        rt = geom.add_vertex(right[0], right[1], right[2] - thickness * 0.5)
        lb = geom.add_vertex(left[0], left[1], left[2] + thickness * 0.5)
        rb = geom.add_vertex(right[0], right[1], right[2] + thickness * 0.5)
        rings.append((lt, rt, lb, rb))

    for a, b in zip(rings[:-1], rings[1:]):
        a_lt, a_rt, a_lb, a_rb = a
        b_lt, b_rt, b_lb, b_rb = b
        # Top and bottom skins.
        geom.add_face(a_lt, b_lt, b_rt)
        geom.add_face(a_lt, b_rt, a_rt)
        geom.add_face(a_lb, a_rb, b_rb)
        geom.add_face(a_lb, b_rb, b_lb)
        # Long rounded-looking side walls.
        geom.add_face(a_lt, a_lb, b_lb)
        geom.add_face(a_lt, b_lb, b_lt)
        geom.add_face(a_rt, b_rt, b_rb)
        geom.add_face(a_rt, b_rb, a_rb)

    # Root and tip caps so every blade is a closed supported solid.
    r0 = rings[0]
    geom.add_face(r0[0], r0[1], r0[3])
    geom.add_face(r0[0], r0[3], r0[2])
    rn = rings[-1]
    geom.add_face(rn[0], rn[2], rn[3])
    geom.add_face(rn[0], rn[3], rn[1])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contemporary_metal_ceiling_fan")

    brushed = model.material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    dark = model.material("shadowed_socket", rgba=(0.08, 0.085, 0.085, 1.0))
    satin = model.material("satin_graphite", rgba=(0.30, 0.31, 0.31, 1.0))
    blade_metal = model.material("cool_blade_metal", rgba=(0.74, 0.76, 0.75, 1.0))

    # Root canopy: a shallow metal cup against the ceiling with a visible
    # socket ring around the ball.
    canopy = model.part("canopy")
    canopy_cup = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.075, -0.006),
            (0.175, 0.030),
            (0.190, 0.095),
            (0.145, 0.155),
            (0.045, 0.190),
        ],
        inner_profile=[
            (0.060, 0.000),
            (0.154, 0.041),
            (0.166, 0.095),
            (0.126, 0.145),
            (0.035, 0.176),
        ],
        segments=80,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )
    canopy.visual(
        mesh_from_geometry(canopy_cup, "canopy_cup"),
        material=brushed,
        name="canopy_cup",
    )
    canopy.visual(
        Cylinder(radius=0.200, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.196)),
        material=brushed,
        name="ceiling_plate",
    )
    canopy.visual(
        mesh_from_geometry(TorusGeometry(radius=0.065, tube=0.010, radial_segments=18, tubular_segments=72), "socket_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=dark,
        name="socket_ring",
    )
    canopy.visual(
        Box((0.031, 0.030, 0.030)),
        # A small dark retainer pad represents the set screw/liner that locks
        # the ball in the socket and gives the fixed swivel a real contact path.
        origin=Origin(xyz=(0.067, 0.0, 0.0)),
        material=dark,
        name="socket_pad",
    )

    # The angled downrod carries the ball at its top.  Its local +Z axis points
    # down along the rod after the fixed swivel articulation is applied.
    downrod = model.part("downrod")
    downrod.visual(
        Sphere(radius=0.052),
        origin=Origin(),
        material=satin,
        name="swivel_ball",
    )
    downrod.visual(
        Cylinder(radius=0.021, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=brushed,
        name="rod_tube",
    )
    downrod.visual(
        Cylinder(radius=0.036, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=brushed,
        name="upper_collar",
    )
    downrod.visual(
        Cylinder(radius=0.040, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.5075)),
        material=brushed,
        name="lower_collar",
    )

    tilt = math.radians(18.0)
    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi - tilt, 0.0)),
    )

    # Round motor housing aligned with the downrod / motor axis.
    motor = model.part("motor_housing")
    motor_shell = LatheGeometry(
        [
            (0.000, -0.105),
            (0.070, -0.105),
            (0.125, -0.093),
            (0.166, -0.062),
            (0.181, -0.023),
            (0.181, 0.023),
            (0.166, 0.062),
            (0.125, 0.093),
            (0.070, 0.105),
            (0.000, 0.105),
        ],
        segments=96,
        closed=True,
    )
    motor.visual(
        mesh_from_geometry(motor_shell, "motor_shell"),
        material=brushed,
        name="motor_shell",
    )
    motor.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=brushed,
        name="top_collar",
    )
    motor.visual(
        Cylinder(radius=0.098, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=satin,
        name="lower_bearing_cap",
    )
    motor.visual(
        Cylinder(radius=0.184, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=satin,
        name="motor_band",
    )

    model.articulation(
        "downrod_to_motor",
        ArticulationType.FIXED,
        parent=downrod,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
    )

    # Rotating four-blade assembly.  All four blades are part of one link so the
    # continuous joint spins the complete assembly about the motor axis.
    rotor = model.part("blade_assembly")
    rotor.visual(
        Cylinder(radius=0.122, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=satin,
        name="rotor_hub",
    )
    rotor.visual(
        mesh_from_geometry(DomeGeometry(radius=0.088, radial_segments=72, height_segments=12, closed=True), "spinner_dome"),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=brushed,
        name="spinner_dome",
    )
    for i in range(4):
        angle = i * math.tau / 4.0
        rotor.visual(
            mesh_from_geometry(_blade_geometry(angle), f"blade_{i}"),
            material=blade_metal,
            name=f"blade_{i}",
        )

    model.articulation(
        "motor_to_blade_assembly",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=28.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rotor_joint = object_model.get_articulation("motor_to_blade_assembly")
    swivel_joint = object_model.get_articulation("canopy_to_downrod")
    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    motor = object_model.get_part("motor_housing")
    rotor = object_model.get_part("blade_assembly")

    ctx.allow_overlap(
        canopy,
        downrod,
        elem_a="socket_pad",
        elem_b="swivel_ball",
        reason=(
            "The fixed ball-and-socket has a small concealed retainer pad that "
            "compresses against the ball to lock the angled downrod in place."
        ),
    )
    ctx.expect_gap(
        canopy,
        downrod,
        axis="x",
        max_penetration=0.002,
        positive_elem="socket_pad",
        negative_elem="swivel_ball",
        name="socket retainer pad seats lightly on the ball",
    )
    ctx.expect_overlap(
        canopy,
        downrod,
        axes="yz",
        min_overlap=0.025,
        elem_a="socket_pad",
        elem_b="swivel_ball",
        name="socket retainer pad is centered on the swivel ball",
    )

    ctx.check(
        "blade assembly uses a continuous motor-axis joint",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(rotor_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={rotor_joint.articulation_type}, axis={rotor_joint.axis}",
    )
    ctx.check(
        "canopy joint is a fixed ball-and-socket swivel",
        swivel_joint.articulation_type == ArticulationType.FIXED,
        details=f"type={swivel_joint.articulation_type}",
    )
    ctx.check(
        "rotor has four named metal blades",
        len([v for v in rotor.visuals if v.name and v.name.startswith("blade_")]) == 4,
        details=f"visuals={[v.name for v in rotor.visuals]}",
    )

    ball_pos = ctx.part_world_position(downrod)
    motor_pos = ctx.part_world_position(motor)
    ctx.check(
        "downrod visibly angles away from the canopy",
        ball_pos is not None
        and motor_pos is not None
        and abs(motor_pos[0] - ball_pos[0]) > 0.15
        and (motor_pos[2] - ball_pos[2]) < -0.45,
        details=f"ball={ball_pos}, motor={motor_pos}",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({rotor_joint: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_center = aabb_center(rest_aabb)
    turned_center = aabb_center(turned_aabb)
    moved = (
        rest_center is not None
        and turned_center is not None
        and math.dist(rest_center[:2], turned_center[:2]) > 0.35
    )
    ctx.check(
        "continuous pose rotates an individual blade",
        moved,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
