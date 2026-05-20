from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PITCH = 0.0036
TURN_LIMIT = 2.0 * math.pi


def _ridge_value(phase: float, duty: float = 0.28) -> float:
    """Triangular raised thread centered on a helical phase line."""
    wrapped = (phase + math.pi) % (2.0 * math.pi) - math.pi
    half_width = math.pi * duty
    distance = abs(wrapped)
    if distance >= half_width:
        return 0.0
    return 1.0 - distance / half_width


def _threaded_cylinder_mesh(
    *,
    z_min: float,
    z_max: float,
    minor_radius: float,
    thread_height: float,
    pitch: float,
    radial_segments: int = 112,
    z_segments: int = 92,
) -> MeshGeometry:
    mesh = MeshGeometry()
    rings: list[list[int]] = []
    for iz in range(z_segments + 1):
        z = z_min + (z_max - z_min) * iz / z_segments
        ring: list[int] = []
        for ia in range(radial_segments):
            theta = 2.0 * math.pi * ia / radial_segments
            phase = theta - 2.0 * math.pi * (z - z_min) / pitch
            radius = minor_radius + thread_height * _ridge_value(phase)
            ring.append(mesh.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
        rings.append(ring)

    for iz in range(z_segments):
        for ia in range(radial_segments):
            a = rings[iz][ia]
            b = rings[iz][(ia + 1) % radial_segments]
            c = rings[iz + 1][(ia + 1) % radial_segments]
            d = rings[iz + 1][ia]
            mesh.add_face(a, b, c)
            mesh.add_face(a, c, d)

    bottom_center = mesh.add_vertex(0.0, 0.0, z_min)
    top_center = mesh.add_vertex(0.0, 0.0, z_max)
    for ia in range(radial_segments):
        nxt = (ia + 1) % radial_segments
        mesh.add_face(bottom_center, rings[0][nxt], rings[0][ia])
        mesh.add_face(top_center, rings[-1][ia], rings[-1][nxt])
    return mesh


def _hollow_threaded_collar_mesh(
    *,
    z_min: float,
    z_max: float,
    outer_radius: float,
    inner_radius: float,
    thread_depth: float,
    pitch: float,
    radial_segments: int = 128,
    z_segments: int = 96,
) -> MeshGeometry:
    mesh = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for iz in range(z_segments + 1):
        z = z_min + (z_max - z_min) * iz / z_segments
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for ia in range(radial_segments):
            theta = 2.0 * math.pi * ia / radial_segments
            outer_ring.append(
                mesh.add_vertex(
                    outer_radius * math.cos(theta),
                    outer_radius * math.sin(theta),
                    z,
                )
            )
            phase = theta - 2.0 * math.pi * (z - z_min) / pitch
            radius = inner_radius - thread_depth * _ridge_value(phase)
            inner_ring.append(mesh.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
        outer.append(outer_ring)
        inner.append(inner_ring)

    for iz in range(z_segments):
        for ia in range(radial_segments):
            nxt = (ia + 1) % radial_segments
            # Outside shell.
            mesh.add_face(outer[iz][ia], outer[iz][nxt], outer[iz + 1][nxt])
            mesh.add_face(outer[iz][ia], outer[iz + 1][nxt], outer[iz + 1][ia])
            # Inside threaded surface.
            mesh.add_face(inner[iz][nxt], inner[iz][ia], inner[iz + 1][ia])
            mesh.add_face(inner[iz][nxt], inner[iz + 1][ia], inner[iz + 1][nxt])

    for ia in range(radial_segments):
        nxt = (ia + 1) % radial_segments
        # Bottom and top annular faces make the collar a connected hollow tube.
        mesh.add_face(outer[0][nxt], outer[0][ia], inner[0][ia])
        mesh.add_face(outer[0][nxt], inner[0][ia], inner[0][nxt])
        mesh.add_face(outer[-1][ia], outer[-1][nxt], inner[-1][nxt])
        mesh.add_face(outer[-1][ia], inner[-1][nxt], inner[-1][ia])
    return mesh


def _plain_ring_mesh(
    *,
    z_min: float,
    z_max: float,
    outer_radius: float,
    inner_radius: float,
    radial_segments: int = 96,
) -> MeshGeometry:
    mesh = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for ia in range(radial_segments):
        theta = 2.0 * math.pi * ia / radial_segments
        c = math.cos(theta)
        s = math.sin(theta)
        outer_bottom.append(mesh.add_vertex(outer_radius * c, outer_radius * s, z_min))
        outer_top.append(mesh.add_vertex(outer_radius * c, outer_radius * s, z_max))
        inner_bottom.append(mesh.add_vertex(inner_radius * c, inner_radius * s, z_min))
        inner_top.append(mesh.add_vertex(inner_radius * c, inner_radius * s, z_max))

    for ia in range(radial_segments):
        nxt = (ia + 1) % radial_segments
        mesh.add_face(outer_bottom[ia], outer_bottom[nxt], outer_top[nxt])
        mesh.add_face(outer_bottom[ia], outer_top[nxt], outer_top[ia])
        mesh.add_face(inner_bottom[nxt], inner_bottom[ia], inner_top[ia])
        mesh.add_face(inner_bottom[nxt], inner_top[ia], inner_top[nxt])
        mesh.add_face(outer_top[ia], outer_top[nxt], inner_top[nxt])
        mesh.add_face(outer_top[ia], inner_top[nxt], inner_top[ia])
        mesh.add_face(outer_bottom[nxt], outer_bottom[ia], inner_bottom[ia])
        mesh.add_face(outer_bottom[nxt], inner_bottom[ia], inner_bottom[nxt])
    return mesh


def _lathe_shell_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 96,
) -> MeshGeometry:
    mesh = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for radius, z in outer_profile:
        outer.append(
            [
                mesh.add_vertex(
                    radius * math.cos(2.0 * math.pi * i / segments),
                    radius * math.sin(2.0 * math.pi * i / segments),
                    z,
                )
                for i in range(segments)
            ]
        )
    for radius, z in inner_profile:
        inner.append(
            [
                mesh.add_vertex(
                    radius * math.cos(2.0 * math.pi * i / segments),
                    radius * math.sin(2.0 * math.pi * i / segments),
                    z,
                )
                for i in range(segments)
            ]
        )

    for profile_rings, inward in ((outer, False), (inner, True)):
        for iz in range(len(profile_rings) - 1):
            for ia in range(segments):
                nxt = (ia + 1) % segments
                a = profile_rings[iz][ia]
                b = profile_rings[iz][nxt]
                c = profile_rings[iz + 1][nxt]
                d = profile_rings[iz + 1][ia]
                if inward:
                    mesh.add_face(b, a, c)
                    mesh.add_face(c, a, d)
                else:
                    mesh.add_face(a, b, c)
                    mesh.add_face(a, c, d)

    # Bottom glass neck rim and rounded top cap close the shell.
    for ia in range(segments):
        nxt = (ia + 1) % segments
        mesh.add_face(outer[0][nxt], outer[0][ia], inner[0][ia])
        mesh.add_face(outer[0][nxt], inner[0][ia], inner[0][nxt])
        mesh.add_face(outer[-1][ia], outer[-1][nxt], inner[-1][nxt])
        mesh.add_face(outer[-1][ia], inner[-1][nxt], inner[-1][ia])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_screw_bulb_socket")

    aluminum = model.material("brushed_aluminum", rgba=(0.64, 0.66, 0.67, 1.0))
    brass = model.material("satin_brass", rgba=(0.78, 0.61, 0.30, 1.0))
    nickel = model.material("nickel_thread", rgba=(0.82, 0.82, 0.78, 1.0))
    black = model.material("engraved_black", rgba=(0.02, 0.02, 0.025, 1.0))
    blue = model.material("gap_blue", rgba=(0.05, 0.35, 0.95, 1.0))
    glass = model.material("frosted_glass", rgba=(0.86, 0.94, 1.0, 0.38))
    ceramic = model.material("white_ceramic", rgba=(0.92, 0.90, 0.84, 1.0))

    socket = model.part("socket")
    socket.visual(
        Box((0.120, 0.100, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=aluminum,
        name="datum_base",
    )
    socket.visual(
        mesh_from_geometry(
            _hollow_threaded_collar_mesh(
                z_min=0.008,
                z_max=0.066,
                outer_radius=0.0240,
                inner_radius=0.0156,
                thread_depth=0.0007,
                pitch=PITCH,
            ),
            "socket_thread_collar",
        ),
        material=brass,
        name="thread_collar",
    )
    socket.visual(
        mesh_from_geometry(
            _plain_ring_mesh(z_min=0.0660, z_max=0.0672, outer_radius=0.0242, inner_radius=0.0152),
            "socket_gap_reference_ring",
        ),
        material=blue,
        name="gap_reference",
    )
    socket.visual(
        Cylinder(radius=0.0070, length=0.0050),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=brass,
        name="center_contact",
    )

    # Four ground pads give the socket a calibration-friendly square reference
    # pattern without filling the central bulb cavity.
    socket.visual(
        Box((0.086, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.041, 0.014)),
        material=aluminum,
        name="front_datum_pad",
    )
    socket.visual(
        Box((0.086, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.041, 0.014)),
        material=aluminum,
        name="rear_datum_pad",
    )
    socket.visual(
        Box((0.008, 0.060, 0.012)),
        origin=Origin(xyz=(0.041, 0.0, 0.014)),
        material=aluminum,
        name="side_datum_pad_0",
    )
    socket.visual(
        Box((0.008, 0.060, 0.012)),
        origin=Origin(xyz=(-0.041, 0.0, 0.014)),
        material=aluminum,
        name="side_datum_pad_1",
    )
    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        radius = 0.041
        socket.visual(
            Box((0.0025, 0.016, 0.0014)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.0207),
                rpy=(0.0, 0.0, angle),
            ),
            material=black,
            name=f"index_mark_{idx}",
        )

    # Boss faces are flat datum surfaces for the two orthogonal adjustment screws.
    socket.visual(
        Box((0.018, 0.018, 0.060)),
        origin=Origin(xyz=(0.046, 0.0, 0.038)),
        material=aluminum,
        name="x_adjust_boss",
    )
    socket.visual(
        Box((0.018, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, 0.046, 0.038)),
        material=aluminum,
        name="y_adjust_boss",
    )
    socket.inertial = Inertial.from_geometry(
        Box((0.120, 0.100, 0.070)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    index_ring = model.part("index_ring")
    index_ring.visual(
        mesh_from_geometry(
            _plain_ring_mesh(z_min=0.000, z_max=0.006, outer_radius=0.037, inner_radius=0.027),
            "calibration_index_ring",
        ),
        material=aluminum,
        name="rotary_scale_ring",
    )
    index_ring.visual(
        Box((0.003, 0.014, 0.002)),
        origin=Origin(xyz=(0.0, 0.033, 0.0065)),
        material=black,
        name="zero_witness",
    )
    index_ring.inertial = Inertial.from_geometry(
        Box((0.076, 0.076, 0.008)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    model.articulation(
        "socket_to_index_ring",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=index_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=0.5, lower=-0.087, upper=0.087),
    )

    x_screw = model.part("x_screw")
    x_screw.visual(
        Cylinder(radius=0.0030, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="screw_shank",
    )
    x_screw.visual(
        Cylinder(radius=0.0080, length=0.006),
        origin=Origin(xyz=(0.0205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="thumb_wheel",
    )
    x_screw.inertial = Inertial.from_geometry(
        Box((0.035, 0.018, 0.018)),
        mass=0.025,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )
    model.articulation(
        "socket_to_x_screw",
        ArticulationType.PRISMATIC,
        parent=socket,
        child=x_screw,
        origin=Origin(xyz=(0.055, 0.0, 0.038)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.010, lower=0.0, upper=0.003),
    )

    y_screw = model.part("y_screw")
    y_screw.visual(
        Cylinder(radius=0.0030, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=nickel,
        name="screw_shank",
    )
    y_screw.visual(
        Cylinder(radius=0.0080, length=0.006),
        origin=Origin(xyz=(0.0, 0.0205, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="thumb_wheel",
    )
    y_screw.inertial = Inertial.from_geometry(
        Box((0.018, 0.035, 0.018)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
    )
    model.articulation(
        "socket_to_y_screw",
        ArticulationType.PRISMATIC,
        parent=socket,
        child=y_screw,
        origin=Origin(xyz=(0.0, 0.055, 0.038)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.010, lower=0.0, upper=0.003),
    )

    # An intentionally visual-free intermediate link keeps the screw kinematics
    # explicit: the bulb rotates about the socket axis and the lead joint moves
    # by the documented one-pitch advance used in calibration poses.
    bulb_turn = model.part("bulb_turn")
    model.articulation(
        "socket_to_bulb_turn",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=bulb_turn,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.0, lower=0.0, upper=TURN_LIMIT),
        meta={"thread_pitch": PITCH, "meaning": "positive turn backs the bulb upward by one pitch"},
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(
            _threaded_cylinder_mesh(
                z_min=0.021,
                z_max=0.064,
                minor_radius=0.0126,
                thread_height=0.0010,
                pitch=PITCH,
            ),
            "bulb_external_thread",
        ),
        material=nickel,
        name="external_thread",
    )
    bulb.visual(
        Cylinder(radius=0.0060, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0150)),
        material=nickel,
        name="contact_button",
    )
    bulb.visual(
        Cylinder(radius=0.0100, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0190)),
        material=ceramic,
        name="insulator_nose",
    )
    bulb.visual(
        Cylinder(radius=0.0142, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0670)),
        material=nickel,
        name="crimp_ring",
    )
    bulb.visual(
        Cylinder(radius=0.0185, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0700)),
        material=nickel,
        name="datum_shoulder",
    )
    bulb.visual(
        mesh_from_geometry(
            _lathe_shell_mesh(
                [
                    (0.0100, 0.067),
                    (0.0180, 0.076),
                    (0.0310, 0.096),
                    (0.0380, 0.124),
                    (0.0340, 0.151),
                    (0.0200, 0.174),
                    (0.0040, 0.184),
                ],
                [
                    (0.0086, 0.0685),
                    (0.0164, 0.0780),
                    (0.0290, 0.0980),
                    (0.0358, 0.1240),
                    (0.0320, 0.1495),
                    (0.0182, 0.1715),
                    (0.0022, 0.1815),
                ],
            ),
            "bulb_glass_shell",
        ),
        material=glass,
        name="glass_envelope",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.080, 0.080, 0.175)),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )
    model.articulation(
        "bulb_turn_to_bulb",
        ArticulationType.PRISMATIC,
        parent=bulb_turn,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.010, lower=0.0, upper=PITCH),
        meta={"lead_per_revolution": PITCH},
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    turn = object_model.get_articulation("socket_to_bulb_turn")
    lead = object_model.get_articulation("bulb_turn_to_bulb")
    x_screw_joint = object_model.get_articulation("socket_to_x_screw")
    y_screw_joint = object_model.get_articulation("socket_to_y_screw")
    ring_joint = object_model.get_articulation("socket_to_index_ring")

    ctx.check("socket_and_bulb_present", socket is not None and bulb is not None)
    ctx.check(
        "screw_motion_axes_are_explicit",
        turn is not None
        and lead is not None
        and tuple(round(float(v), 6) for v in turn.axis) == (0.0, 0.0, 1.0)
        and tuple(round(float(v), 6) for v in lead.axis) == (0.0, 0.0, 1.0)
        and lead.meta.get("lead_per_revolution") == PITCH,
        details=f"turn={turn}, lead={lead}",
    )
    if socket is None or bulb is None or turn is None or lead is None:
        return ctx.report()

    ctx.expect_origin_distance(
        bulb,
        socket,
        axes="xy",
        max_dist=0.0005,
        name="bulb_axis_coaxial_with_socket",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="external_thread",
        elem_b="thread_collar",
        min_overlap=0.035,
        name="threaded_base_retained_in_collar",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="contact_button",
        elem_b="center_contact",
        contact_tol=0.0002,
        name="bottom_contact_seated",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="datum_shoulder",
        negative_elem="gap_reference",
        min_gap=0.0006,
        max_gap=0.0040,
        name="controlled_blue_reference_gap",
    )

    rest_position = ctx.part_world_position(bulb)
    with ctx.pose({turn: TURN_LIMIT, lead: PITCH}):
        ctx.expect_origin_distance(
            bulb,
            socket,
            axes="xy",
            max_dist=0.0005,
            name="unscrewed_pose_stays_coaxial",
        )
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="external_thread",
            elem_b="thread_collar",
            min_overlap=0.030,
            name="one_turn_unscrewed_still_retained",
        )
        lifted_position = ctx.part_world_position(bulb)
    ctx.check(
        "one_revolution_advances_one_pitch",
        rest_position is not None
        and lifted_position is not None
        and abs((lifted_position[2] - rest_position[2]) - PITCH) < 0.00025,
        details=f"rest={rest_position}, lifted={lifted_position}, pitch={PITCH}",
    )

    ctx.check(
        "adjustment_joints_present",
        x_screw_joint is not None and y_screw_joint is not None and ring_joint is not None,
        details=f"x={x_screw_joint}, y={y_screw_joint}, ring={ring_joint}",
    )
    if x_screw_joint is not None and y_screw_joint is not None:
        x_screw = object_model.get_part("x_screw")
        y_screw = object_model.get_part("y_screw")
        x_rest = ctx.part_world_position(x_screw)
        y_rest = ctx.part_world_position(y_screw)
        with ctx.pose({x_screw_joint: 0.003, y_screw_joint: 0.003}):
            x_in = ctx.part_world_position(x_screw)
            y_in = ctx.part_world_position(y_screw)
        ctx.check(
            "adjustment_screws_drive_inward",
            x_rest is not None
            and x_in is not None
            and y_rest is not None
            and y_in is not None
            and x_in[0] < x_rest[0] - 0.0025
            and y_in[1] < y_rest[1] - 0.0025,
            details=f"x_rest={x_rest}, x_in={x_in}, y_rest={y_rest}, y_in={y_in}",
        )

    return ctx.report()


object_model = build_object_model()
