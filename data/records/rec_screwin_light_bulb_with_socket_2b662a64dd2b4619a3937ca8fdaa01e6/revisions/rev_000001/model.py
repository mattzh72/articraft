from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z0: float):
    """CadQuery hollow cylinder with the lower face at z0."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _socket_cup_mesh():
    """One connected heavy socket cup with hollow threaded bore and flanges."""
    pedestal = cq.Workplane("XY").cylinder(0.062, 0.046).translate((0.0, 0.0, 0.043))
    lower_flange = _annular_cylinder(0.066, 0.029, 0.016, 0.073)
    cup_wall = _annular_cylinder(0.047, 0.0265, 0.122, 0.083)
    top_lip = _annular_cylinder(0.056, 0.0275, 0.014, 0.197)
    return pedestal.union(lower_flange).union(cup_wall).union(top_lip)


def _helix_tube_mesh(
    radius: float,
    z0: float,
    z1: float,
    turns: float,
    tube_radius: float,
    name: str,
    *,
    phase: float = 0.0,
    clockwise: bool = False,
):
    steps = max(48, int(turns * 42))
    pts = []
    direction = -1.0 if clockwise else 1.0
    for i in range(steps + 1):
        t = i / steps
        angle = phase + direction * turns * 2.0 * pi * t
        pts.append((radius * cos(angle), radius * sin(angle), z0 + (z1 - z0) * t))
    geom = tube_from_spline_points(
        pts,
        radius=tube_radius,
        samples_per_segment=2,
        radial_segments=10,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _bulb_glass_mesh():
    # Smooth, pear-shaped industrial bulb envelope in the bulb part frame.
    profile = [
        (0.000, 0.292),
        (0.022, 0.287),
        (0.044, 0.263),
        (0.060, 0.218),
        (0.064, 0.170),
        (0.056, 0.118),
        (0.040, 0.074),
        (0.026, 0.041),
        (0.021, 0.026),
        (0.000, 0.026),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=64, closed=True), "glass_envelope")


def _orientation_mark_mesh():
    geom = MeshGeometry()
    # A tiny raised pointer wedge on the rotating screw shoulder so rotation is legible.
    verts = [
        (0.024, -0.0035, 0.016),
        (0.024, 0.0035, 0.016),
        (0.034, 0.0000, 0.016),
        (0.024, -0.0035, 0.030),
        (0.024, 0.0035, 0.030),
        (0.034, 0.0000, 0.030),
    ]
    for v in verts:
        geom.add_vertex(*v)
    for face in ((0, 1, 2), (3, 5, 4), (0, 3, 4), (0, 4, 1), (1, 4, 5), (1, 5, 2), (2, 5, 3), (2, 3, 0)):
        geom.add_face(*face)
    return mesh_from_geometry(geom, "orientation_mark")


def _add_radial_box(part, name, material, radius_mid, length, width, height, z, angle):
    x = radius_mid * cos(angle)
    y = radius_mid * sin(angle)
    part.visual(
        Box((length, width, height)),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, angle)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_screw_in_bulb_socket")

    galvanized = model.material("galvanized_steel", rgba=(0.46, 0.48, 0.47, 1.0))
    dark_steel = model.material("dark_oxide_steel", rgba=(0.08, 0.085, 0.08, 1.0))
    safety_yellow = model.material("safety_yellow_guard", rgba=(1.0, 0.72, 0.05, 1.0))
    brass = model.material("brass_contacts", rgba=(0.82, 0.58, 0.22, 1.0))
    nickel = model.material("brushed_nickel_threads", rgba=(0.72, 0.74, 0.72, 1.0))
    black = model.material("black_stop_marks", rgba=(0.02, 0.02, 0.018, 1.0))
    red = model.material("red_lockout_tab", rgba=(0.82, 0.05, 0.035, 1.0))
    glass = model.material("warm_frosted_glass", rgba=(0.98, 0.92, 0.68, 0.42))

    socket = model.part("socket_frame")

    # Load path: horizontal mounting plate, central pedestal, hollow socket cup,
    # radial standoffs, and cage hoops all remain visibly connected.
    socket.visual(
        Box((0.205, 0.160, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=galvanized,
        name="mount_plate",
    )
    socket.visual(
        mesh_from_cadquery(_socket_cup_mesh(), "socket_cup", tolerance=0.0007, angular_tolerance=0.06),
        material=dark_steel,
        name="socket_cup",
    )
    socket.visual(
        _helix_tube_mesh(0.0272, 0.098, 0.188, 3.05, 0.00125, "female_thread", phase=0.55, clockwise=True),
        material=brass,
        name="female_thread",
    )
    socket.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=brass,
        name="bottom_contact",
    )
    socket.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=brass,
        name="contact_post",
    )

    # Fastener logic on the mounting plate and socket flange.
    for i, (x, y) in enumerate(((-0.078, -0.055), (0.078, -0.055), (-0.078, 0.055), (0.078, 0.055))):
        socket.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(x, y, 0.0155)),
            material=dark_steel,
            name=f"plate_bolt_{i}",
        )
    for i in range(6):
        a = i * 2.0 * pi / 6.0 + pi / 6.0
        socket.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(0.049 * cos(a), 0.049 * sin(a), 0.2125)),
            material=galvanized,
            name=f"collar_screw_{i}",
        )

    # Heavy gussets from plate to socket cup.
    for i, a in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        _add_radial_box(
            socket,
            f"gusset_{i}",
            galvanized,
            radius_mid=0.040,
            length=0.080,
            width=0.008,
            height=0.056,
            z=0.042,
            angle=a,
        )

    # Safety cage: connected lower standoffs, three hoops, and vertical rods.
    socket.visual(
        mesh_from_geometry(TorusGeometry(radius=0.075, tube=0.0035, radial_segments=18, tubular_segments=64), "guard_hoop_0"),
        origin=Origin(xyz=(0.0, 0.0, 0.211)),
        material=safety_yellow,
        name="guard_hoop_0",
    )
    socket.visual(
        mesh_from_geometry(TorusGeometry(radius=0.078, tube=0.0032, radial_segments=18, tubular_segments=64), "guard_hoop_1"),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=safety_yellow,
        name="guard_hoop_1",
    )
    socket.visual(
        mesh_from_geometry(TorusGeometry(radius=0.071, tube=0.0030, radial_segments=18, tubular_segments=64), "guard_hoop_2"),
        origin=Origin(xyz=(0.0, 0.0, 0.502)),
        material=safety_yellow,
        name="guard_hoop_2",
    )
    for i in range(8):
        a = i * 2.0 * pi / 8.0
        _add_radial_box(
            socket,
            f"guard_standoff_{i}",
            safety_yellow,
            radius_mid=0.064,
            length=0.031,
            width=0.0065,
            height=0.007,
            z=0.211,
            angle=a,
        )
        socket.visual(
            Cylinder(radius=0.0027, length=0.291),
            origin=Origin(xyz=(0.075 * cos(a), 0.075 * sin(a), 0.3565)),
            material=safety_yellow,
            name=f"guard_rod_{i}",
        )

    # Mechanical over-travel stops for the bulb shoulder key.
    for i, a in enumerate((-0.62, 0.62)):
        _add_radial_box(
            socket,
            f"turn_stop_{i}",
            black,
            radius_mid=0.046,
            length=0.025,
            width=0.010,
            height=0.014,
            z=0.213,
            angle=a,
        )

    # Fixed hinge/strike hardware for the lockout bail.
    socket.visual(
        Box((0.020, 0.008, 0.055)),
        origin=Origin(xyz=(0.057, -0.059, 0.178)),
        material=dark_steel,
        name="lockout_hinge_lug",
    )
    socket.visual(
        Box((0.020, 0.008, 0.040)),
        origin=Origin(xyz=(0.057, 0.061, 0.178)),
        material=dark_steel,
        name="lockout_strike_lug",
    )
    socket.visual(
        Cylinder(radius=0.003, length=0.050),
        origin=Origin(xyz=(0.057, -0.052, 0.178)),
        material=galvanized,
        name="lockout_pin",
    )
    socket.visual(
        Box((0.010, 0.008, 0.004)),
        origin=Origin(xyz=(0.057, -0.055, 0.204)),
        material=galvanized,
        name="pin_keeper_top",
    )
    socket.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.053, -0.053, 0.211), (0.057, -0.0665, 0.194), (0.057, -0.0665, 0.157)],
                radius=0.0032,
                samples_per_segment=5,
                radial_segments=10,
            ),
            "hinge_anchor_strut",
        ),
        material=dark_steel,
        name="hinge_anchor_strut",
    )
    socket.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.050, 0.058, 0.211), (0.057, 0.068, 0.194), (0.057, 0.068, 0.160)],
                radius=0.0032,
                samples_per_segment=5,
                radial_segments=10,
            ),
            "strike_anchor_strut",
        ),
        material=dark_steel,
        name="strike_anchor_strut",
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.019, length=0.106),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=nickel,
        name="screw_core",
    )
    bulb.visual(
        _helix_tube_mesh(0.0208, -0.094, 0.004, 3.25, 0.00195, "male_thread", phase=0.15),
        material=nickel,
        name="male_thread",
    )
    bulb.visual(
        Cylinder(radius=0.027, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=nickel,
        name="thread_shoulder",
    )
    bulb.visual(
        Cylinder(radius=0.0185, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=nickel,
        name="neck_sleeve",
    )
    bulb.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=glass,
        name="glass_neck_band",
    )
    bulb.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        material=brass,
        name="tip_contact",
    )
    bulb.visual(
        _orientation_mark_mesh(),
        material=black,
        name="orientation_mark",
    )
    bulb.visual(
        _bulb_glass_mesh(),
        material=glass,
        name="glass_envelope",
    )

    lockout = model.part("lockout_bail")
    lockout.visual(
        Cylinder(radius=0.006, length=0.046),
        material=red,
        name="hinge_barrel",
    )
    lockout.visual(
        Box((0.014, 0.103, 0.006)),
        origin=Origin(xyz=(0.0, 0.052, 0.0)),
        material=red,
        name="lockout_strap",
    )
    lockout.visual(
        Box((0.026, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.108, 0.0)),
        material=red,
        name="latch_eye",
    )
    lockout.visual(
        Cylinder(radius=0.0045, length=0.0045),
        origin=Origin(xyz=(0.0, 0.108, 0.004)),
        material=black,
        name="padlock_hole_marker",
    )

    model.articulation(
        "thread_turn",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "lockout_hinge",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=lockout,
        origin=Origin(xyz=(0.057, -0.052, 0.178)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket_frame")
    bulb = object_model.get_part("bulb")
    lockout = object_model.get_part("lockout_bail")
    thread_turn = object_model.get_articulation("thread_turn")
    lockout_hinge = object_model.get_articulation("lockout_hinge")

    ctx.allow_overlap(
        lockout,
        socket,
        elem_a="hinge_barrel",
        elem_b="lockout_pin",
        reason=(
            "The red hinge barrel is a solid visual proxy for a sleeve rotating "
            "around the captured lockout pin."
        ),
    )

    ctx.expect_origin_distance(
        socket,
        bulb,
        axes="xy",
        max_dist=0.001,
        name="bulb and socket share the threaded axis",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="screw_core",
        outer_elem="socket_cup",
        margin=0.0,
        name="screw base is radially captured by the socket collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="screw_core",
        elem_b="socket_cup",
        min_overlap=0.085,
        name="screw base remains deeply inserted in the socket",
    )
    ctx.expect_contact(
        lockout,
        socket,
        elem_a="hinge_barrel",
        elem_b="lockout_hinge_lug",
        contact_tol=0.002,
        name="lockout bail is carried by its hinge lug",
    )
    ctx.expect_within(
        socket,
        lockout,
        axes="xy",
        inner_elem="lockout_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="lockout pin stays centered inside the hinge barrel",
    )
    ctx.expect_overlap(
        lockout,
        socket,
        axes="z",
        elem_a="hinge_barrel",
        elem_b="lockout_pin",
        min_overlap=0.040,
        name="hinge barrel is axially captured on the lockout pin",
    )

    mark_rest = ctx.part_element_world_aabb(bulb, elem="orientation_mark")
    latch_rest = ctx.part_element_world_aabb(lockout, elem="latch_eye")
    with ctx.pose({thread_turn: 0.75, lockout_hinge: 1.1}):
        mark_turned = ctx.part_element_world_aabb(bulb, elem="orientation_mark")
        latch_open = ctx.part_element_world_aabb(lockout, elem="latch_eye")

    def _center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    rest_xy = _center_xy(mark_rest)
    turned_xy = _center_xy(mark_turned)
    ctx.check(
        "threaded bulb visibly rotates about the socket axis",
        rest_xy is not None
        and turned_xy is not None
        and abs(rest_xy[1] - turned_xy[1]) > 0.010,
        details=f"rest={rest_xy}, turned={turned_xy}",
    )

    latch_xy = _center_xy(latch_rest)
    open_xy = _center_xy(latch_open)
    ctx.check(
        "lockout bail swings clear on its hinge",
        latch_xy is not None
        and open_xy is not None
        and abs(open_xy[0] - latch_xy[0]) > 0.040,
        details=f"closed={latch_xy}, open={open_xy}",
    )

    return ctx.report()


object_model = build_object_model()
