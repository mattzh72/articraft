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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _helix_points(
    *,
    radius: float,
    pitch: float,
    turns: float,
    z0: float,
    phase: float = 0.0,
    samples_per_turn: int = 28,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    steps = max(8, int(math.ceil(turns * samples_per_turn)))
    for step in range(steps + 1):
        t = step / steps
        angle = phase + turns * math.tau * t
        points.append(
            (
                radius * math.cos(angle),
                radius * math.sin(angle),
                z0 + pitch * turns * t,
            )
        )
    return points


def _thread_mesh(
    *,
    thread_radius: float,
    center_radius: float,
    pitch: float,
    turns: float,
    z0: float,
    phase: float = 0.0,
):
    return tube_from_spline_points(
        _helix_points(
            radius=center_radius,
            pitch=pitch,
            turns=turns,
            z0=z0,
            phase=phase,
            samples_per_turn=28,
        ),
        radius=thread_radius,
        samples_per_segment=2,
        radial_segments=14,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _bulb_glass_mesh():
    outer_profile = [
        (0.0150, 0.000),
        (0.0180, 0.008),
        (0.0260, 0.025),
        (0.0315, 0.045),
        (0.0305, 0.060),
        (0.0230, 0.079),
        (0.0100, 0.091),
        (0.0000, 0.097),
    ]
    inner_profile = [
        (0.0138, 0.002),
        (0.0168, 0.010),
        (0.0246, 0.027),
        (0.0298, 0.045),
        (0.0288, 0.060),
        (0.0215, 0.078),
        (0.0092, 0.089),
        (0.0000, 0.094),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
    )


def _socket_collar_mesh():
    outer_profile = [
        (0.0212, 0.000),
        (0.0212, 0.003),
        (0.0192, 0.006),
        (0.0192, 0.022),
        (0.0208, 0.026),
        (0.0208, 0.028),
    ]
    inner_profile = [
        (0.01655, 0.000),
        (0.01655, 0.023),
        (0.0158, 0.028),
    ]
    collar = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
    )
    collar.merge(
        _thread_mesh(
            thread_radius=0.00070,
            center_radius=0.01595,
            pitch=0.0037,
            turns=5.1,
            z0=0.002,
            phase=0.18,
        )
    )
    return collar


def _threaded_bulb_shell_mesh():
    shell_height = 0.025
    shell = CylinderGeometry(0.01355, shell_height, radial_segments=48).translate(
        0.0,
        0.0,
        shell_height * 0.5,
    )
    shell.merge(
        _thread_mesh(
            thread_radius=0.00090,
            center_radius=0.01405,
            pitch=0.0037,
            turns=5.1,
            z0=0.001,
            phase=0.18,
        )
    )
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_screwin_bulb_fixture")

    base_black = model.material("base_black", rgba=(0.16, 0.17, 0.18, 1.0))
    datum_gray = model.material("datum_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    socket_metal = model.material("socket_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.82, 0.83, 0.84, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.57, 0.22, 1.0))
    mark_red = model.material("mark_red", rgba=(0.76, 0.16, 0.12, 1.0))
    lamp_glass = model.material("lamp_glass", rgba=(0.94, 0.96, 0.99, 0.48))

    fixture_base = model.part("fixture_base")
    fixture_base.visual(
        Box((0.160, 0.102, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=base_black,
        name="base_plinth",
    )
    fixture_base.visual(
        Box((0.060, 0.042, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=datum_gray,
        name="pedestal_cap",
    )
    fixture_base.visual(
        Box((0.082, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.030, 0.029)),
        material=base_black,
        name="front_datum_bar",
    )
    fixture_base.visual(
        Box((0.016, 0.054, 0.030)),
        origin=Origin(xyz=(-0.034, 0.000, 0.029)),
        material=base_black,
        name="left_datum_cheek",
    )
    fixture_base.visual(
        Box((0.016, 0.054, 0.030)),
        origin=Origin(xyz=(0.034, 0.000, 0.029)),
        material=base_black,
        name="right_datum_cheek",
    )
    fixture_base.visual(
        Box((0.008, 0.032, 0.0014)),
        origin=Origin(xyz=(0.0, -0.040, 0.0147)),
        material=mark_red,
        name="base_index_mark",
    )
    for x_pos in (-0.012, 0.0, 0.012):
        fixture_base.visual(
            Box((0.0018, 0.009, 0.0012)),
            origin=Origin(xyz=(x_pos, -0.032, 0.0146)),
            material=satin_aluminum,
            name=f"base_tick_{'m' if abs(x_pos) < 1e-9 else ('l' if x_pos < 0.0 else 'r')}",
        )
    fixture_base.inertial = Inertial.from_geometry(
        Box((0.160, 0.102, 0.050)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    socket_carriage = model.part("socket_carriage")
    socket_carriage.visual(
        Box((0.044, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=socket_metal,
        name="slide_shoe",
    )
    socket_carriage.visual(
        Box((0.026, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=socket_metal,
        name="guide_boss",
    )
    socket_carriage.visual(
        mesh_from_geometry(_socket_collar_mesh(), "socket_collar"),
        material=socket_metal,
        name="socket_collar",
    )
    socket_carriage.visual(
        Box((0.010, 0.005, 0.010)),
        origin=Origin(xyz=(0.022, 0.0, 0.009)),
        material=satin_aluminum,
        name="right_adjust_tab",
    )
    socket_carriage.visual(
        Box((0.010, 0.005, 0.010)),
        origin=Origin(xyz=(-0.022, 0.0, 0.009)),
        material=satin_aluminum,
        name="left_adjust_tab",
    )
    socket_carriage.visual(
        Box((0.004, 0.010, 0.0016)),
        origin=Origin(xyz=(0.0, -0.0195, 0.0235)),
        material=mark_red,
        name="socket_pointer_mark",
    )
    socket_carriage.inertial = Inertial.from_geometry(
        Box((0.050, 0.042, 0.058)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(_threaded_bulb_shell_mesh(), "bulb_threaded_shell"),
        material=satin_aluminum,
        name="threaded_shell",
    )
    bulb.visual(
        Cylinder(radius=0.0164, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=satin_aluminum,
        name="seating_flange",
    )
    bulb.visual(
        Cylinder(radius=0.0142, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=satin_aluminum,
        name="neck_band",
    )
    bulb.visual(
        Cylinder(radius=0.0038, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, -0.00175)),
        material=brass,
        name="tip_contact",
    )
    bulb.visual(
        mesh_from_geometry(_bulb_glass_mesh(), "bulb_glass_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=lamp_glass,
        name="glass_envelope",
    )
    bulb.visual(
        Box((0.007, 0.0016, 0.022)),
        origin=Origin(xyz=(0.0, -0.0315, 0.062)),
        material=mark_red,
        name="bulb_alignment_mark",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.064, 0.064, 0.126)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
    )

    model.articulation(
        "base_to_socket_carriage",
        ArticulationType.PRISMATIC,
        parent=fixture_base,
        child=socket_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.030,
            lower=0.0,
            upper=0.008,
        ),
    )
    model.articulation(
        "socket_to_bulb",
        ArticulationType.REVOLUTE,
        parent=socket_carriage,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=4.0,
            lower=0.0,
            upper=4.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixture_base = object_model.get_part("fixture_base")
    socket_carriage = object_model.get_part("socket_carriage")
    bulb = object_model.get_part("bulb")
    socket_lift = object_model.get_articulation("base_to_socket_carriage")
    bulb_twist = object_model.get_articulation("socket_to_bulb")

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

    ctx.check(
        "socket_calibration_axis_is_coaxial",
        abs(socket_lift.axis[0]) < 1e-9
        and abs(socket_lift.axis[1]) < 1e-9
        and abs(abs(socket_lift.axis[2]) - 1.0) < 1e-9,
        details=f"unexpected socket lift axis: {socket_lift.axis}",
    )
    ctx.check(
        "bulb_twist_axis_is_coaxial",
        abs(bulb_twist.axis[0]) < 1e-9
        and abs(bulb_twist.axis[1]) < 1e-9
        and abs(abs(bulb_twist.axis[2]) - 1.0) < 1e-9,
        details=f"unexpected bulb twist axis: {bulb_twist.axis}",
    )
    ctx.check(
        "bulb_thread_turn_range_is_multi_turn",
        bulb_twist.motion_limits is not None
        and bulb_twist.motion_limits.upper is not None
        and bulb_twist.motion_limits.lower is not None
        and (bulb_twist.motion_limits.upper - bulb_twist.motion_limits.lower) >= 6.0,
        details="bulb twist articulation should represent multiple screw turns",
    )

    with ctx.pose({socket_lift: 0.0, bulb_twist: 0.0}):
        ctx.expect_contact(
            socket_carriage,
            fixture_base,
            elem_a="slide_shoe",
            elem_b="pedestal_cap",
            contact_tol=0.0006,
            name="socket_carriage_supported_on_pedestal",
        )
        ctx.expect_within(
            bulb,
            socket_carriage,
            axes="xy",
            inner_elem="threaded_shell",
            outer_elem="socket_collar",
            margin=0.006,
            name="bulb_threaded_shell_stays_coaxial_with_socket",
        )
        ctx.expect_gap(
            bulb,
            socket_carriage,
            axis="z",
            positive_elem="seating_flange",
            negative_elem="socket_collar",
            max_gap=0.0015,
            max_penetration=0.0002,
            name="bulb_seating_flange_has_controlled_gap_to_socket_datum",
        )
        ctx.expect_overlap(
            bulb,
            socket_carriage,
            axes="xy",
            elem_a="threaded_shell",
            elem_b="socket_collar",
            min_overlap=0.025,
            name="threaded_regions_share_footprint",
        )

    closed_z = ctx.part_world_position(socket_carriage)[2]
    with ctx.pose({socket_lift: 0.006}):
        raised_z = ctx.part_world_position(socket_carriage)[2]
        ctx.check(
            "socket_calibration_lift_changes_height",
            closed_z is not None
            and raised_z is not None
            and abs((raised_z - closed_z) - 0.006) < 0.001,
            details=f"expected 6 mm lift, got {None if closed_z is None or raised_z is None else raised_z - closed_z}",
        )
        ctx.expect_gap(
            socket_carriage,
            fixture_base,
            axis="z",
            positive_elem="slide_shoe",
            negative_elem="pedestal_cap",
            min_gap=0.005,
            max_gap=0.007,
            name="calibration_pose_opens_visible_service_gap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
