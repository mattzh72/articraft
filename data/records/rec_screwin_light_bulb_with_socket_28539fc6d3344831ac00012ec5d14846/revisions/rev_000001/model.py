from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _lathed_shell(name: str, outer_profile, inner_profile, *, segments: int = 88):
    return _mesh(name, LatheGeometry(list(outer_profile) + list(reversed(inner_profile)), segments=segments))


def _helix_points(
    radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    *,
    samples_per_turn: int = 18,
    phase: float = 0.0,
):
    samples = max(12, int(math.ceil(turns * samples_per_turn)))
    points = []
    for index in range(samples + 1):
        t = index / samples
        angle = phase + (2.0 * math.pi * turns * t)
        z = z_start + (z_end - z_start) * t
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def _thread_mesh(
    name: str,
    *,
    radius: float,
    tube_radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    phase: float = 0.0,
    samples_per_turn: int = 18,
):
    return _mesh(
        name,
        tube_from_spline_points(
            _helix_points(
                radius,
                z_start,
                z_end,
                turns,
                samples_per_turn=samples_per_turn,
                phase=phase,
            ),
            radius=tube_radius,
            samples_per_segment=2,
            radial_segments=16,
            cap_ends=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screw_light_bulb_with_socket")

    frosted_glass = model.material("frosted_glass", rgba=(0.96, 0.97, 0.98, 0.62))
    satin_white = model.material("satin_white", rgba=(0.93, 0.94, 0.95, 1.0))
    painted_graphite = model.material("painted_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    nickel = model.material("nickel", rgba=(0.75, 0.77, 0.79, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.66, 0.33, 1.0))
    elastomer = model.material("elastomer", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.12, 0.12, 0.13, 1.0))

    bulb_glass_mesh = _lathed_shell(
        "bulb_glass_globe",
        [
            (0.0015, 0.112),
            (0.010, 0.111),
            (0.021, 0.104),
            (0.028, 0.093),
            (0.030, 0.078),
            (0.0295, 0.063),
            (0.0245, 0.046),
            (0.0180, 0.037),
            (0.0142, 0.033),
        ],
        [
            (0.0005, 0.1095),
            (0.0087, 0.1085),
            (0.0198, 0.102),
            (0.0265, 0.092),
            (0.0284, 0.078),
            (0.0280, 0.064),
            (0.0230, 0.047),
            (0.0167, 0.039),
            (0.0130, 0.0345),
        ],
        segments=96,
    )
    bulb_neck_mesh = _lathed_shell(
        "bulb_neck_collar",
        [
            (0.0142, 0.036),
            (0.0153, 0.034),
            (0.0158, 0.031),
            (0.0151, 0.0275),
            (0.0140, 0.0235),
            (0.0137, 0.0215),
        ],
        [
            (0.0108, 0.036),
            (0.0116, 0.034),
            (0.0120, 0.031),
            (0.0118, 0.0275),
            (0.0115, 0.0235),
            (0.0112, 0.0215),
        ],
        segments=72,
    )
    bulb_shell_mesh = _lathed_shell(
        "bulb_metal_shell",
        [
            (0.0123, 0.0260),
            (0.0129, 0.0240),
            (0.0130, 0.0180),
            (0.01295, 0.0060),
            (0.01270, 0.0000),
        ],
        [
            (0.0107, 0.0260),
            (0.01115, 0.0240),
            (0.01125, 0.0180),
            (0.01110, 0.0060),
            (0.01090, 0.0006),
        ],
        segments=72,
    )
    bulb_thread_mesh = _thread_mesh(
        "bulb_thread_ridge",
        radius=0.01372,
        tube_radius=0.00062,
        z_start=0.0028,
        z_end=0.0238,
        turns=5.8,
        samples_per_turn=20,
    )

    socket_lower_body_mesh = _lathed_shell(
        "socket_lower_body",
        [
            (0.0187, 0.020),
            (0.0196, 0.014),
            (0.0202, 0.000),
        ],
        [
            (0.0138, 0.020),
            (0.0133, 0.014),
            (0.0128, 0.004),
        ],
        segments=72,
    )
    socket_collar_mesh = _lathed_shell(
        "socket_outer_collar",
        [
            (0.0182, 0.0550),
            (0.0185, 0.0520),
            (0.0185, 0.0300),
            (0.0182, 0.0180),
        ],
        [
            (0.0169, 0.0550),
            (0.01685, 0.0520),
            (0.01665, 0.0300),
            (0.01645, 0.0180),
        ],
        segments=84,
    )
    socket_sleeve_mesh = _lathed_shell(
        "socket_threaded_sleeve",
        [
            (0.0167, 0.0520),
            (0.0167, 0.0200),
        ],
        [
            (0.0158, 0.0520),
            (0.0158, 0.0200),
        ],
        segments=72,
    )
    socket_gasket_mesh = _lathed_shell(
        "socket_rim_gasket",
        [
            (0.0176, 0.0548),
            (0.0175, 0.0522),
        ],
        [
            (0.0158, 0.0548),
            (0.0158, 0.0522),
        ],
        segments=72,
    )
    socket_insulator_mesh = _mesh(
        "socket_insulator_insert",
        LatheGeometry(
            [
                (0.0, 0.0204),
                (0.0042, 0.0204),
                (0.0118, 0.0204),
                (0.01395, 0.0186),
                (0.01345, 0.0140),
                (0.0129, 0.0044),
                (0.0, 0.0044),
            ],
            segments=72,
        ),
    )
    socket_thread_mesh = _thread_mesh(
        "socket_thread_ridge",
        radius=0.01518,
        tube_radius=0.00064,
        z_start=0.0266,
        z_end=0.0492,
        turns=5.8,
        samples_per_turn=20,
    )

    socket = model.part("socket")
    socket.visual(socket_lower_body_mesh, material=dark_polymer, name="lower_body")
    socket.visual(socket_collar_mesh, material=painted_graphite, name="outer_collar")
    socket.visual(socket_sleeve_mesh, material=nickel, name="threaded_sleeve")
    socket.visual(socket_thread_mesh, material=nickel, name="socket_thread")
    socket.visual(socket_gasket_mesh, material=elastomer, name="rim_gasket")
    socket.visual(
        socket_insulator_mesh,
        material=satin_white,
        name="insulator",
    )
    socket.visual(
        Cylinder(radius=0.0038, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0214)),
        material=brass,
        name="center_contact",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0202, length=0.0550),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    bulb = model.part("bulb")
    bulb.visual(bulb_glass_mesh, material=frosted_glass, name="glass_globe")
    bulb.visual(bulb_neck_mesh, material=satin_white, name="neck_collar")
    bulb.visual(bulb_shell_mesh, material=nickel, name="metal_shell")
    bulb.visual(bulb_thread_mesh, material=nickel, name="thread_ridge")
    bulb.visual(
        Cylinder(radius=0.0112, length=0.0036),
        origin=Origin(xyz=(0.0, 0.0, 0.0013)),
        material=elastomer,
        name="base_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0040, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brass,
        name="contact_button",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.112)),
        mass=0.065,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    model.articulation(
        "socket_to_bulb_twist",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0234)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=6.0,
            lower=0.0,
            upper=4.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    twist = object_model.get_articulation("socket_to_bulb_twist")

    glass_globe = bulb.get_visual("glass_globe")
    thread_ridge = bulb.get_visual("thread_ridge")
    contact_button = bulb.get_visual("contact_button")
    socket_thread = socket.get_visual("socket_thread")
    outer_collar = socket.get_visual("outer_collar")
    center_contact = socket.get_visual("center_contact")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = twist.motion_limits
    ctx.check(
        "twist_axis_is_coaxial",
        tuple(twist.axis) == (0.0, 0.0, 1.0),
        details=f"axis={twist.axis}",
    )
    ctx.check(
        "twist_range_allows_real_screw_motion",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper >= math.tau,
        details=f"limits={limits}",
    )

    with ctx.pose({twist: 0.0}):
        ctx.expect_contact(
            bulb,
            socket,
            elem_a=contact_button,
            elem_b=center_contact,
            contact_tol=0.00025,
            name="base_contact_seated",
        )
        ctx.expect_origin_distance(
            bulb,
            socket,
            axes="xy",
            max_dist=1e-6,
            name="bulb_axis_matches_socket_axis",
        )
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem=thread_ridge,
            outer_elem=socket_thread,
            margin=0.0008,
            name="threading_stays_coaxially_nested",
        )
        ctx.expect_gap(
            bulb,
            socket,
            axis="z",
            positive_elem=glass_globe,
            negative_elem=outer_collar,
            min_gap=0.0005,
            max_gap=0.0040,
            name="glass_lifts_cleanly_above_socket_rim",
        )

    with ctx.pose({twist: math.tau}):
        ctx.expect_origin_distance(
            bulb,
            socket,
            axes="xy",
            max_dist=1e-6,
            name="bulb_remains_coaxial_through_one_turn",
        )
        ctx.expect_contact(
            bulb,
            socket,
            elem_a=contact_button,
            elem_b=center_contact,
            contact_tol=0.00025,
            name="center_contact_persists_through_twist",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
