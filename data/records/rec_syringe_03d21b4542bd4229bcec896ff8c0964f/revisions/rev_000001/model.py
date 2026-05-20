from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathe_shell_x(outer_profile, inner_profile, mesh_name: str):
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        lip_samples=10,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def _build_scale_strip_mesh():
    geom = BoxGeometry((0.188, 0.010, 0.0018)).translate(0.100, 0.0, 0.0368)

    for index in range(11):
        x_pos = 0.010 + 0.018 * index
        geom.merge(
            BoxGeometry((0.0022, 0.012, 0.0012)).translate(
                x_pos,
                0.0,
                0.0382,
            )
        )
        if index < 10:
            geom.merge(
                BoxGeometry((0.0018, 0.007, 0.0010)).translate(
                    x_pos + 0.009,
                    0.0,
                    0.0379,
                )
            )

    geom.rotate_x(-0.72)
    return mesh_from_geometry(geom, "scale_strip")


def _build_rear_bolts_mesh():
    bolt_positions = (
        (-0.030, 0.022, 0.020),
        (-0.030, -0.022, 0.020),
        (-0.030, 0.022, -0.020),
        (-0.030, -0.022, -0.020),
    )

    combined = None
    for x_pos, y_pos, z_pos in bolt_positions:
        bolt = CylinderGeometry(radius=0.005, height=0.012).rotate_y(math.pi / 2.0)
        bolt.translate(x_pos, y_pos, z_pos)
        if combined is None:
            combined = bolt
        else:
            combined.merge(bolt)

    combined.merge(BoxGeometry((0.004, 0.054, 0.050)).translate(-0.030, 0.0, 0.0))
    return mesh_from_geometry(combined, "rear_bolts")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_syringe")

    body_coat = model.material("body_coat", rgba=(0.22, 0.24, 0.26, 1.0))
    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    polycarbonate = model.material("polycarbonate", rgba=(0.74, 0.82, 0.88, 0.30))
    seal_black = model.material("seal_black", rgba=(0.08, 0.09, 0.10, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    body = model.part("body")

    rear_housing_mesh = _lathe_shell_x(
        outer_profile=[
            (0.021, -0.190),
            (0.028, -0.184),
            (0.028, -0.150),
            (0.046, -0.150),
            (0.046, -0.118),
            (0.043, -0.094),
        ],
        inner_profile=[
            (0.016, -0.190),
            (0.016, -0.152),
            (0.024, -0.122),
            (0.031, -0.094),
        ],
        mesh_name="rear_housing",
    )
    body.visual(rear_housing_mesh, material=body_coat, name="rear_housing")

    barrel_window_mesh = _lathe_shell_x(
        outer_profile=[
            (0.043, -0.094),
            (0.038, -0.084),
            (0.038, 0.204),
            (0.038, 0.208),
        ],
        inner_profile=[
            (0.031, -0.094),
            (0.031, 0.208),
        ],
        mesh_name="barrel_window",
    )
    body.visual(barrel_window_mesh, material=polycarbonate, name="barrel_window")

    nozzle_shell_mesh = _lathe_shell_x(
        outer_profile=[
            (0.038, 0.208),
            (0.043, 0.220),
            (0.024, 0.235),
            (0.018, 0.260),
            (0.009, 0.292),
            (0.007, 0.302),
        ],
        inner_profile=[
            (0.0195, 0.208),
            (0.020, 0.235),
            (0.008, 0.260),
            (0.0035, 0.292),
            (0.0025, 0.302),
        ],
        mesh_name="nozzle_shell",
    )
    body.visual(nozzle_shell_mesh, material=stainless, name="nozzle_shell")

    rear_wiper_seal_mesh = _lathe_shell_x(
        outer_profile=[
            (0.010, -0.186),
            (0.010, -0.166),
        ],
        inner_profile=[
            (0.0072, -0.186),
            (0.0072, -0.166),
        ],
        mesh_name="rear_wiper_seal",
    )
    body.visual(rear_wiper_seal_mesh, material=seal_black, name="rear_wiper_seal")
    rear_gland_ring_mesh = _lathe_shell_x(
        outer_profile=[
            (0.022, -0.190),
            (0.022, -0.186),
        ],
        inner_profile=[
            (0.0074, -0.190),
            (0.0074, -0.186),
        ],
        mesh_name="rear_gland_ring",
    )
    body.visual(rear_gland_ring_mesh, material=stainless, name="rear_gland_ring")
    forward_stop_ring_mesh = _lathe_shell_x(
        outer_profile=[
            (0.031, 0.204),
            (0.031, 0.208),
        ],
        inner_profile=[
            (0.0195, 0.204),
            (0.0195, 0.208),
        ],
        mesh_name="forward_stop_ring",
    )
    body.visual(forward_stop_ring_mesh, material=stainless, name="forward_stop_ring")

    body.visual(_build_scale_strip_mesh(), material=stainless, name="scale_strip")
    body.visual(_build_rear_bolts_mesh(), material=stainless, name="rear_bolts")

    body.inertial = Inertial.from_geometry(
        Box((0.492, 0.100, 0.100)),
        mass=2.7,
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.006, length=0.436),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0162, length=0.008),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="rear_stop_disc",
    )
    plunger.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(-0.174, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_dark,
        name="handle_hub",
    )
    plunger.visual(
        Cylinder(radius=0.010, length=0.096),
        origin=Origin(xyz=(-0.184, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_dark,
        name="handle_crossbar",
    )
    plunger.visual(
        Cylinder(radius=0.012, length=0.048),
        origin=Origin(xyz=(0.225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="piston_core",
    )
    plunger.visual(
        Cylinder(radius=0.029, length=0.006),
        origin=Origin(xyz=(0.213, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_black,
        name="piston_rear_seal",
    )
    plunger.visual(
        Cylinder(radius=0.029, length=0.006),
        origin=Origin(xyz=(0.237, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_black,
        name="piston_front_seal",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.249, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="piston_nose",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.460, 0.100, 0.070)),
        mass=0.75,
        origin=Origin(xyz=(0.033, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(-0.190, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.14,
            lower=0.0,
            upper=0.154,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("body_to_plunger")

    rear_housing = body.get_visual("rear_housing")
    barrel_window = body.get_visual("barrel_window")
    rear_wiper_seal = body.get_visual("rear_wiper_seal")
    rear_gland_ring = body.get_visual("rear_gland_ring")
    forward_stop_ring = body.get_visual("forward_stop_ring")
    plunger_rod = plunger.get_visual("plunger_rod")
    rear_stop_disc = plunger.get_visual("rear_stop_disc")
    piston_front_seal = plunger.get_visual("piston_front_seal")

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

    limits = slide.motion_limits
    ctx.check(
        "plunger_joint_axis_is_coaxial",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected coaxial +X prismatic axis, got {slide.axis!r}",
    )
    ctx.check(
        "plunger_motion_is_bounded",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.14 <= limits.upper <= 0.18,
        details=f"expected a bounded practical stroke, got {limits!r}",
    )

    ctx.expect_contact(
        plunger,
        body,
        elem_a=rear_stop_disc,
        elem_b=rear_gland_ring,
        contact_tol=0.001,
        name="retracted_stop_contacts_rain_lip",
    )
    ctx.expect_overlap(
        plunger,
        body,
        elem_a=plunger_rod,
        elem_b=rear_wiper_seal,
        axes="yz",
        min_overlap=0.012,
        name="rod_runs_through_rear_wiper_support",
    )
    ctx.expect_within(
        plunger,
        body,
        axes="yz",
        inner_elem=piston_front_seal,
        outer_elem=barrel_window,
        margin=0.001,
        name="piston_stays_coaxial_with_barrel",
    )

    closed_x = ctx.part_world_position(plunger)[0]
    with ctx.pose({slide: limits.upper}):
        advanced_x = ctx.part_world_position(plunger)[0]
        ctx.expect_contact(
            plunger,
            body,
            elem_a=piston_front_seal,
            elem_b=forward_stop_ring,
            contact_tol=0.001,
            name="forward_stop_contacts_internal_shoulder",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_forward_stop")
    ctx.check(
        "plunger_moves_forward_along_body_axis",
        advanced_x > closed_x + 0.13,
        details=f"expected forward travel > 0.13 m, got {advanced_x - closed_x:.4f} m",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
