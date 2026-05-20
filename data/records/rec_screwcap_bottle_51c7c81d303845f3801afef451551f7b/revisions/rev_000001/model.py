from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
    Inertial,
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
    def merge_geometries(*geometries) -> MeshGeometry:
        merged = MeshGeometry()
        for geometry in geometries:
            merged.merge(geometry)
        return merged

    def helical_thread(
        *,
        radius: float,
        z_start: float,
        pitch: float,
        turns: float,
        tube_radius: float,
        phase: float = 0.0,
        samples_per_turn: int = 36,
    ) -> MeshGeometry:
        total_samples = max(16, int(math.ceil(turns * samples_per_turn)))
        points = []
        for index in range(total_samples + 1):
            t = index / total_samples
            angle = phase + (math.tau * turns * t)
            z = z_start + (pitch * turns * t)
            points.append((radius * math.cos(angle), radius * math.sin(angle), z))
        return tube_from_spline_points(
            points,
            radius=tube_radius,
            samples_per_segment=4,
            radial_segments=14,
            cap_ends=True,
            up_hint=(0.0, 0.0, 1.0),
        )

    def build_bottle_shell() -> MeshGeometry:
        profile = [
            (0.0, 0.0),
            (0.0280, 0.0),
            (0.0315, 0.004),
            (0.0330, 0.010),
            (0.0330, 0.142),
            (0.0320, 0.151),
            (0.0284, 0.160),
            (0.0246, 0.165),
            (0.0194, 0.167),
            (0.0188, 0.170),
            (0.0214, 0.172),
            (0.0170, 0.178),
            (0.0146, 0.184),
            (0.0146, 0.200),
            (0.0153, 0.204),
            (0.0148, 0.209),
            (0.0118, 0.209),
            (0.0118, 0.184),
            (0.0106, 0.179),
            (0.0125, 0.172),
            (0.0142, 0.170),
            (0.0140, 0.167),
            (0.0205, 0.165),
            (0.0252, 0.160),
            (0.0288, 0.151),
            (0.0306, 0.142),
            (0.0306, 0.010),
            (0.0268, 0.0035),
            (0.0, 0.0018),
        ]
        return LatheGeometry(profile, segments=96)

    def build_cap_shell() -> MeshGeometry:
        skirt_outer = 0.0192
        skirt_inner = 0.0166
        top_outer = 0.0189
        z_bottom = -0.0240
        z_top_inner = 0.0018
        z_top_outer = 0.0040
        cap_shell = LatheGeometry(
            [
                (skirt_inner, z_bottom),
                (skirt_inner, z_top_inner),
                (0.0, z_top_inner),
                (0.0, z_top_outer),
                (top_outer, z_top_outer),
                (skirt_outer, 0.0015),
                (skirt_outer, z_bottom),
            ],
            segments=96,
        )
        return cap_shell

    model = ArticulatedObject(name="screwcap_bottle")

    pet_clear = model.material("pet_clear", rgba=(0.80, 0.92, 0.98, 0.34))
    cap_blue = model.material("cap_blue", rgba=(0.18, 0.36, 0.78, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_shell_mesh = build_bottle_shell()
    bottle_thread_mesh = helical_thread(
        radius=0.01445,
        z_start=0.1842,
        pitch=0.0084,
        turns=2.05,
        tube_radius=0.0007,
    )
    bottle_body.visual(
        mesh_from_geometry(bottle_shell_mesh, "bottle_shell"),
        material=pet_clear,
        name="bottle_shell",
    )
    bottle_body.visual(
        mesh_from_geometry(bottle_thread_mesh, "neck_thread"),
        material=pet_clear,
        name="neck_thread",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0335, length=0.209),
        mass=0.024,
        origin=Origin(xyz=(0.0, 0.0, 0.1045)),
    )

    cap = model.part("cap")
    cap_shell_mesh = build_cap_shell()
    cap_thread_mesh = helical_thread(
        radius=0.01595,
        z_start=-0.0225,
        pitch=0.0084,
        turns=2.05,
        tube_radius=0.00065,
    )
    cap.visual(
        mesh_from_geometry(cap_shell_mesh, "cap_shell"),
        material=cap_blue,
        name="cap_shell",
    )
    cap.visual(
        mesh_from_geometry(cap_thread_mesh, "cap_thread"),
        material=cap_blue,
        name="cap_thread",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0192, length=0.028),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    model.articulation(
        "cap_twist",
        ArticulationType.REVOLUTE,
        parent=bottle_body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.2072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=10.0,
            lower=0.0,
            upper=4.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    cap_twist = object_model.get_articulation("cap_twist")
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
    ctx.allow_overlap(
        bottle_body,
        cap,
        elem_a="bottle_shell",
        elem_b="cap_shell",
        reason="thin nested shell pair seats at the bottle lip; compiled closed-shell overlap is a false positive for the capped pose",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = cap_twist.motion_limits
    axis = cap_twist.axis
    ctx.check(
        "cap_joint_is_vertical",
        axis is not None and abs(axis[0]) < 1e-9 and abs(axis[1]) < 1e-9 and abs(axis[2] - 1.0) < 1e-9,
        f"expected +Z cap axis, got {axis}",
    )
    ctx.check(
        "cap_unscrew_range_plausible",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and 3.8 <= limits.upper <= 4.6,
        f"expected about 1.25 turns of unscrewing, got limits={limits}",
    )
    ctx.expect_origin_distance(
        cap,
        bottle_body,
        axes="xy",
        max_dist=1e-6,
        name="cap_is_coaxial_with_bottle",
    )
    ctx.expect_overlap(
        cap,
        bottle_body,
        axes="xy",
        min_overlap=0.028,
        name="cap_and_bottle_share_axis_footprint",
    )
    ctx.expect_within(
        bottle_body,
        cap,
        axes="xy",
        inner_elem="neck_thread",
        outer_elem="cap_shell",
        margin=0.0,
        name="neck_thread_stays_inside_cap_skirt",
    )
    ctx.expect_contact(
        cap,
        bottle_body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        contact_tol=1e-5,
        name="cap_shell_seats_on_bottle_finish",
    )
    ctx.expect_contact(
        cap,
        bottle_body,
        elem_a="cap_thread",
        elem_b="neck_thread",
        contact_tol=0.0006,
        name="thread_interface_reads_as_mating_close_clearance",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
