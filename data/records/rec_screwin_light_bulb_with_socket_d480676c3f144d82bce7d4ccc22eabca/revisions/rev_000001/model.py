from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


THREAD_PITCH = 0.00363  # Edison E26/E27-class pitch, in meters per turn.
SCREW_TURNS = 2.5
SCREW_TRAVEL = THREAD_PITCH * SCREW_TURNS
SCREW_UPPER = 2.0 * math.pi * SCREW_TURNS


def _helical_thread(
    *,
    root_radius: float,
    crest_radius: float,
    z_min: float,
    z_max: float,
    pitch: float,
    starts: int = 1,
    phase: float = 0.0,
    samples_per_turn: int = 34,
    half_width: float | None = None,
) -> MeshGeometry:
    """Triangular helical rib surface used for visible screw threads."""

    if half_width is None:
        half_width = pitch * 0.23

    geom = MeshGeometry()
    turns = max((z_max - z_min) / pitch, 0.05)
    samples = max(int(turns * samples_per_turn), 8)

    for start in range(starts):
        start_indices: list[tuple[int, int, int]] = []
        start_phase = phase + start * (2.0 * math.pi / starts)
        for i in range(samples + 1):
            t = i / samples
            angle = start_phase + 2.0 * math.pi * turns * t
            z_center = z_min + (z_max - z_min) * t
            c = math.cos(angle)
            s = math.sin(angle)
            lower = geom.add_vertex(
                root_radius * c,
                root_radius * s,
                z_center - half_width,
            )
            crest = geom.add_vertex(
                crest_radius * c,
                crest_radius * s,
                z_center,
            )
            upper = geom.add_vertex(
                root_radius * c,
                root_radius * s,
                z_center + half_width,
            )
            start_indices.append((lower, crest, upper))

        for (a0, b0, c0), (a1, b1, c1) in zip(start_indices, start_indices[1:]):
            # Lower flank, upper flank, and embedded root surface.
            geom.add_face(a0, a1, b1)
            geom.add_face(a0, b1, b0)
            geom.add_face(b0, b1, c1)
            geom.add_face(b0, c1, c0)
            geom.add_face(c0, c1, a1)
            geom.add_face(c0, a1, a0)

        # Close the start and end cuts so the ridge reads as a solid strip.
        a0, b0, c0 = start_indices[0]
        a1, b1, c1 = start_indices[-1]
        geom.add_face(a0, b0, c0)
        geom.add_face(a1, c1, b1)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screw_in_bulb_socket")

    frosted_glass = model.material("frosted_glass", rgba=(0.92, 0.96, 1.0, 0.48))
    warm_glass_core = model.material("warm_inner_glow", rgba=(1.0, 0.86, 0.54, 0.34))
    satin_nickel = model.material("satin_nickel", rgba=(0.78, 0.75, 0.68, 1.0))
    painted_metal = model.material("painted_warm_white_metal", rgba=(0.82, 0.80, 0.74, 1.0))
    dark_polymer = model.material("charcoal_polymer", rgba=(0.035, 0.037, 0.040, 1.0))
    black_elastomer = model.material("black_elastomer", rgba=(0.006, 0.006, 0.006, 1.0))
    brass = model.material("brushed_brass_contact", rgba=(0.95, 0.69, 0.28, 1.0))

    socket = model.part("socket")
    thread_axis = model.part("thread_axis")
    bulb = model.part("bulb")

    # Premium socket: thin painted metal shell with a true open bore, chamfered
    # lip, restrained base flare, and a dark insulating liner.
    socket_shell = LatheGeometry.from_shell_profiles(
        [
            (0.026, 0.000),
            (0.031, 0.004),
            (0.031, 0.009),
            (0.025, 0.014),
            (0.022, 0.036),
            (0.0215, 0.061),
            (0.024, 0.068),
            (0.0225, 0.074),
        ],
        [
            (0.0078, 0.012),
            (0.0138, 0.017),
            (0.0148, 0.035),
            (0.0148, 0.066),
            (0.0160, 0.071),
        ],
        segments=112,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    socket.visual(
        mesh_from_geometry(socket_shell, "socket_shell"),
        material=painted_metal,
        name="socket_shell",
    )

    socket_liner = LatheGeometry.from_shell_profiles(
        [
            (0.0149, 0.016),
            (0.0149, 0.065),
            (0.0158, 0.070),
        ],
        [
            (0.0136, 0.019),
            (0.0138, 0.063),
            (0.0145, 0.068),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )
    socket.visual(
        mesh_from_geometry(socket_liner, "socket_liner"),
        material=dark_polymer,
        name="socket_liner",
    )

    socket_thread = _helical_thread(
        root_radius=0.0140,
        crest_radius=0.01305,
        z_min=0.027,
        z_max=0.067,
        pitch=THREAD_PITCH,
        phase=math.pi * 0.50,
        samples_per_turn=32,
        half_width=THREAD_PITCH * 0.19,
    )
    socket.visual(
        mesh_from_geometry(socket_thread, "socket_internal_thread"),
        material=brass,
        name="socket_internal_thread",
    )

    socket.visual(
        Cylinder(radius=0.0142, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_polymer,
        name="socket_floor",
    )
    socket.visual(
        Cylinder(radius=0.0062, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0160)),
        material=brass,
        name="center_contact",
    )
    socket.visual(
        Cylinder(radius=0.0305, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0032)),
        material=black_elastomer,
        name="base_gasket",
    )

    # The bulb glass is a real thin lathed shell: A19-like, but slimmer and
    # deliberately premium with a tight neck transition into the screw cap.
    bulb_glass = LatheGeometry.from_shell_profiles(
        [
            (0.0090, 0.083),
            (0.0115, 0.088),
            (0.0175, 0.098),
            (0.0265, 0.115),
            (0.0315, 0.139),
            (0.0310, 0.158),
            (0.0250, 0.176),
            (0.0130, 0.188),
            (0.0025, 0.193),
        ],
        [
            (0.0074, 0.085),
            (0.0096, 0.090),
            (0.0153, 0.100),
            (0.0244, 0.116),
            (0.0294, 0.139),
            (0.0289, 0.157),
            (0.0230, 0.173),
            (0.0112, 0.185),
            (0.0011, 0.190),
        ],
        segments=128,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    bulb.visual(
        mesh_from_geometry(bulb_glass, "glass_shell"),
        material=frosted_glass,
        name="glass_shell",
    )
    bulb.visual(
        Cylinder(radius=0.0030, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=warm_glass_core,
        name="glow_stem",
    )

    screw_core = LatheGeometry(
        [
            (0.0052, 0.018),
            (0.0064, 0.020),
            (0.0074, 0.024),
            (0.0108, 0.027),
            (0.01135, 0.033),
            (0.01135, 0.077),
            (0.01220, 0.080),
            (0.01140, 0.084),
            (0.0096, 0.087),
        ],
        segments=112,
        closed=True,
    )
    bulb.visual(
        mesh_from_geometry(screw_core, "screw_core"),
        material=satin_nickel,
        name="screw_core",
    )

    bulb_thread = _helical_thread(
        root_radius=0.01105,
        crest_radius=0.01245,
        z_min=0.032,
        z_max=0.082,
        pitch=THREAD_PITCH,
        phase=0.0,
        samples_per_turn=34,
        half_width=THREAD_PITCH * 0.21,
    )
    bulb.visual(
        mesh_from_geometry(bulb_thread, "external_thread"),
        material=satin_nickel,
        name="external_thread",
    )
    bulb.visual(
        Cylinder(radius=0.0122, length=0.0055),
        origin=Origin(xyz=(0.0, 0.0, 0.0834)),
        material=black_elastomer,
        name="neck_seal",
    )
    bulb.visual(
        Cylinder(radius=0.0056, length=0.0070),
        origin=Origin(xyz=(0.0, 0.0, 0.0210)),
        material=brass,
        name="tip_contact",
    )

    # A serial revolute/prismatic pair encodes the honest screw constraint:
    # rotation and axial travel share one centerline.  Authoring keeps the two
    # scalar joints explicit because the SDK mimic layer only couples like
    # domains; tests below use the coherent pose q_z = pitch * q_turn / 2pi.
    screw_turn = model.articulation(
        "screw_turn",
        ArticulationType.REVOLUTE,
        parent=thread_axis,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.6, lower=0.0, upper=SCREW_UPPER),
        motion_properties=MotionProperties(damping=0.03, friction=0.05),
    )
    model.articulation(
        "screw_lift",
        ArticulationType.PRISMATIC,
        parent=socket,
        child=thread_axis,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.05, lower=0.0, upper=SCREW_TRAVEL),
        motion_properties=MotionProperties(damping=0.05, friction=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    screw_turn = object_model.get_articulation("screw_turn")
    screw_lift = object_model.get_articulation("screw_lift")

    ctx.expect_origin_distance(
        socket,
        bulb,
        axes="xy",
        max_dist=0.0005,
        name="bulb and socket axes are coincident",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="screw_core",
        outer_elem="socket_shell",
        margin=0.001,
        name="screw base is coaxially captured by socket collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="screw_core",
        elem_b="socket_shell",
        min_overlap=0.030,
        name="threaded base is retained inside collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="tip_contact",
        negative_elem="center_contact",
        min_gap=0.0,
        max_gap=0.0012,
        name="electrical tip seats just above center contact",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({screw_turn: SCREW_UPPER, screw_lift: SCREW_TRAVEL}):
        raised_pos = ctx.part_world_position(bulb)
        ctx.expect_origin_distance(
            socket,
            bulb,
            axes="xy",
            max_dist=0.0005,
            name="unscrewing preserves common thread axis",
        )
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="screw_core",
            elem_b="socket_shell",
            min_overlap=0.020,
            name="upper travel still leaves thread engagement",
        )

    ctx.check(
        "coherent screw pose lifts bulb by thread pitch",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + SCREW_TRAVEL * 0.85,
        details=f"rest={rest_pos}, raised={raised_pos}, expected_travel={SCREW_TRAVEL}",
    )

    return ctx.report()


object_model = build_object_model()
