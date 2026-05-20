from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_light_bulb_with_socket")

    frosted_glass = model.material("frosted_glass", rgba=(0.96, 0.96, 0.94, 0.58))
    shell_metal = model.material("shell_metal", rgba=(0.73, 0.74, 0.74, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.28, 1.0))
    phenolic = model.material("phenolic", rgba=(0.14, 0.12, 0.10, 1.0))
    ivory_insulator = model.material("ivory_insulator", rgba=(0.90, 0.88, 0.82, 1.0))
    neck_cement = model.material("neck_cement", rgba=(0.70, 0.67, 0.60, 1.0))

    socket_body = model.part("socket_body")
    socket_body.visual(
        _mesh(
            "socket_body_shell",
            _socket_body_shell_mesh(),
        ),
        material=phenolic,
        name="body_shell",
    )
    socket_body.visual(
        _mesh(
            "socket_contact_isolator",
            _ring_shell_mesh(
                outer_radius=0.0154,
                inner_radius=0.0048,
                z0=0.0006,
                z1=0.0040,
            ),
        ),
        material=ivory_insulator,
        name="contact_isolator",
    )

    socket_body.visual(
        _mesh(
            "socket_sleeve",
            _socket_sleeve_mesh(),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        material=brass,
        name="sleeve_shell",
    )
    socket_body.visual(
        _mesh(
            "socket_internal_thread",
            _helical_thread_mesh(
                radius=0.0137,
                z0=0.0100,
                z1=0.0244,
                turns=3.8,
                tube_radius=0.00062,
                phase=0.10,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        material=brass,
        name="internal_thread",
    )
    socket_body.visual(
        Cylinder(radius=0.0049, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0036)),
        material=brass,
        name="contact_button",
    )
    socket_body.visual(
        Cylinder(radius=0.0024, length=0.0034),
        origin=Origin(xyz=(0.0, 0.0, 0.0017)),
        material=brass,
        name="contact_stem",
    )

    bulb_assembly = model.part("bulb_assembly")
    bulb_assembly.visual(
        _mesh(
            "bulb_screw_shell",
            _bulb_screw_shell_mesh(),
        ),
        material=shell_metal,
        name="screw_shell",
    )
    bulb_assembly.visual(
        _mesh(
            "bulb_external_thread",
            _helical_thread_mesh(
                radius=0.0121,
                z0=0.0025,
                z1=0.0217,
                turns=5.05,
                tube_radius=0.00066,
                phase=0.10,
            ),
        ),
        material=shell_metal,
        name="external_thread",
    )
    bulb_assembly.visual(
        _mesh(
            "bulb_tip_insulator",
            _ring_shell_mesh(
                outer_radius=0.0117,
                inner_radius=0.0039,
                z0=-0.0028,
                z1=0.0008,
            ),
        ),
        material=ivory_insulator,
        name="tip_insulator",
    )
    bulb_assembly.visual(
        Cylinder(radius=0.0146, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0280)),
        material=neck_cement,
        name="neck_cement",
    )
    bulb_assembly.visual(
        _mesh(
            "bulb_glass_shell",
            _bulb_glass_shell_mesh(),
        ),
        material=frosted_glass,
        name="glass_shell",
    )

    bulb_assembly.visual(
        Cylinder(radius=0.0047, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, -0.0024)),
        material=brass,
        name="tip_button",
    )
    bulb_assembly.visual(
        Cylinder(radius=0.0039, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, -0.0006)),
        material=brass,
        name="tip_stem",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.REVOLUTE,
        parent=socket_body,
        child=bulb_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        # A right-hand Edison thread tightens clockwise when viewed from above,
        # so use -Z to keep positive q as the tightening direction.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=6.0,
            lower=0.0,
            upper=math.tau * 4.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket_body = object_model.get_part("socket_body")
    bulb_assembly = object_model.get_part("bulb_assembly")
    thread_joint = object_model.get_articulation("socket_to_bulb")

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

    ctx.expect_contact(
        bulb_assembly,
        socket_body,
        elem_a="tip_button",
        elem_b="contact_button",
        name="bulb_tip_touches_socket_center_contact",
    )
    ctx.expect_origin_distance(
        bulb_assembly,
        socket_body,
        axes="xy",
        max_dist=1e-6,
        name="bulb_and_socket_are_coaxial",
    )
    ctx.expect_overlap(
        bulb_assembly,
        socket_body,
        axes="xy",
        elem_a="screw_shell",
        elem_b="sleeve_shell",
        min_overlap=0.024,
        name="threaded_shell_sits_inside_socket_collar",
    )
    ctx.expect_gap(
        bulb_assembly,
        socket_body,
        axis="z",
        positive_elem="glass_shell",
        negative_elem="body_shell",
        min_gap=0.001,
        max_gap=0.006,
        name="glass_envelope_clears_socket_top",
    )

    limits = thread_joint.motion_limits
    ctx.check(
        "thread_axis_is_vertical",
        thread_joint.axis == (0.0, 0.0, -1.0),
        details=f"expected (0,0,-1), got {thread_joint.axis}",
    )
    ctx.check(
        "thread_turn_range_is_realistic",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper >= math.tau * 3.5,
        details=f"expected at least 3.5 tightening turns, got {limits}",
    )

    with ctx.pose({thread_joint: math.pi / 2.0}):
        ctx.expect_contact(
            bulb_assembly,
            socket_body,
            elem_a="tip_button",
            elem_b="contact_button",
            name="tip_contact_stays_seated_when_bulb_twists",
        )

    return ctx.report()


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _ring_shell_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    segments: int = 64,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _helical_thread_mesh(
    *,
    radius: float,
    z0: float,
    z1: float,
    turns: float,
    tube_radius: float,
    phase: float = 0.0,
) -> MeshGeometry:
    point_count = max(16, int(turns * 28))
    points = []
    for index in range(point_count + 1):
        t = index / point_count
        theta = phase + turns * math.tau * t
        z = z0 + (z1 - z0) * t
        points.append((radius * math.cos(theta), radius * math.sin(theta), z))
    return tube_from_spline_points(
        points,
        radius=tube_radius,
        samples_per_segment=3,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _socket_body_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0260, 0.0000),
            (0.0260, 0.0060),
            (0.0225, 0.0080),
            (0.0215, 0.0260),
            (0.0208, 0.0310),
            (0.0198, 0.0330),
        ],
        [
            (0.0048, 0.0010),
            (0.0120, 0.0025),
            (0.0165, 0.0060),
            (0.0165, 0.0288),
            (0.0160, 0.0325),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )


def _socket_sleeve_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0162, 0.0000),
            (0.0162, 0.0012),
            (0.0153, 0.0022),
            (0.0153, 0.0252),
            (0.0159, 0.0262),
        ],
        [
            (0.0143, 0.0010),
            (0.0143, 0.0247),
            (0.0138, 0.0259),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _bulb_screw_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0121, 0.0000),
            (0.0121, 0.0188),
            (0.0118, 0.0229),
            (0.0113, 0.0264),
            (0.0108, 0.0290),
        ],
        [
            (0.0106, 0.0008),
            (0.0106, 0.0185),
            (0.0101, 0.0226),
            (0.0095, 0.0261),
            (0.0090, 0.0287),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _bulb_glass_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0146, 0.0308),
            (0.0194, 0.0410),
            (0.0268, 0.0530),
            (0.0300, 0.0685),
            (0.0288, 0.0830),
            (0.0220, 0.0975),
            (0.0110, 0.1080),
            (0.0032, 0.1120),
        ],
        [
            (0.0132, 0.0313),
            (0.0178, 0.0417),
            (0.0252, 0.0533),
            (0.0284, 0.0685),
            (0.0274, 0.0826),
            (0.0208, 0.0966),
            (0.0101, 0.1068),
            (0.0018, 0.1120),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )


# >>> USER_CODE_END

object_model = build_object_model()
