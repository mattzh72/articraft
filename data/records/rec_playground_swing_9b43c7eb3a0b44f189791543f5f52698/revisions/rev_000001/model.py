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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="disc_swing")

    support_blue = model.material("support_blue", rgba=(0.20, 0.40, 0.62, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.48, 0.50, 0.54, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    seat_red = model.material("seat_red", rgba=(0.73, 0.18, 0.16, 1.0))
    base_gray = model.material("base_gray", rgba=(0.32, 0.34, 0.36, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.78, 0.44, 0.12)),
        origin=Origin(xyz=(-0.38, 0.0, 0.06)),
        material=base_gray,
        name="ballast_base",
    )
    support_frame.visual(
        Box((0.18, 0.22, 0.08)),
        origin=Origin(xyz=(-0.08, 0.0, 0.16)),
        material=dark_steel,
        name="mast_shoe",
    )

    main_support_tube = tube_from_spline_points(
        [
            (-0.10, 0.0, 0.12),
            (-0.04, 0.0, 1.18),
            (0.06, 0.0, 2.00),
            (0.34, 0.0, 2.22),
            (0.60, 0.0, 2.15),
        ],
        radius=0.042,
        samples_per_segment=18,
        radial_segments=22,
        cap_ends=True,
    )
    support_frame.visual(
        mesh_from_geometry(main_support_tube, "main_support_tube"),
        material=support_blue,
        name="main_support_tube",
    )

    rear_brace = tube_from_spline_points(
        [
            (-0.62, 0.0, 0.12),
            (-0.36, 0.0, 0.58),
            (-0.18, 0.0, 0.92),
            (-0.04, 0.0, 1.18),
        ],
        radius=0.031,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    support_frame.visual(
        mesh_from_geometry(rear_brace, "rear_brace_tube"),
        material=support_blue,
        name="rear_brace_tube",
    )

    clevis_plate_size = (0.068, 0.010, 0.115)
    support_frame.visual(
        Box(clevis_plate_size),
        origin=Origin(xyz=(0.64, 0.030, 2.115)),
        material=steel_gray,
        name="clevis_left",
    )
    support_frame.visual(
        Box(clevis_plate_size),
        origin=Origin(xyz=(0.64, -0.030, 2.115)),
        material=steel_gray,
        name="clevis_right",
    )
    support_frame.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.64, 0.041, 2.16), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_cap_left",
    )
    support_frame.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.64, -0.041, 2.16), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_cap_right",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((1.35, 0.50, 2.30)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.025, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_barrel",
    )
    yoke.visual(
        Box((0.020, 0.012, 0.092)),
        origin=Origin(xyz=(0.0, 0.013, -0.068)),
        material=steel_gray,
        name="strap_left",
    )
    yoke.visual(
        Box((0.020, 0.012, 0.092)),
        origin=Origin(xyz=(0.0, -0.013, -0.068)),
        material=steel_gray,
        name="strap_right",
    )
    yoke.visual(
        Box((0.058, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.123)),
        material=steel_gray,
        name="lower_bridge",
    )
    yoke.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.149)),
        material=dark_steel,
        name="stem_socket",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.07, 0.06, 0.18)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_steel,
        name="top_spigot",
    )
    stem.visual(
        Cylinder(radius=0.016, length=1.68),
        origin=Origin(xyz=(0.0, 0.0, -0.858)),
        material=steel_gray,
        name="stem_shaft",
    )
    stem.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -1.520)),
        material=dark_steel,
        name="lower_bearing_collar",
    )
    stem.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -1.457)),
        material=dark_steel,
        name="upper_retainer_clip",
    )
    stem.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -1.689)),
        material=dark_steel,
        name="bottom_end_cap",
    )
    stem.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=1.72),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, -0.86)),
    )

    disc_seat = model.part("disc_seat")
    seat_outer_profile = [
        (0.030, 0.026),
        (0.050, 0.028),
        (0.105, 0.014),
        (0.150, 0.004),
        (0.156, -0.004),
        (0.148, -0.014),
        (0.100, -0.022),
        (0.046, -0.026),
        (0.030, -0.026),
    ]
    seat_inner_profile = [
        (0.018, 0.026),
        (0.018, -0.026),
    ]
    seat_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            seat_outer_profile,
            seat_inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "disc_seat_shell",
    )
    disc_seat.visual(
        seat_mesh,
        material=seat_red,
        name="seat_shell",
    )
    disc_seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.156, length=0.056),
        mass=2.3,
    )

    model.articulation(
        "overhead_pivot",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=yoke,
        origin=Origin(xyz=(0.64, 0.0, 2.16)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.8,
            lower=-0.78,
            upper=0.78,
        ),
    )
    model.articulation(
        "yoke_to_stem",
        ArticulationType.FIXED,
        parent=yoke,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, -0.163)),
    )
    model.articulation(
        "seat_spin",
        ArticulationType.CONTINUOUS,
        parent=stem,
        child=disc_seat,
        origin=Origin(xyz=(0.0, 0.0, -1.490)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    support_frame = object_model.get_part("support_frame")
    yoke = object_model.get_part("yoke")
    stem = object_model.get_part("stem")
    disc_seat = object_model.get_part("disc_seat")

    overhead_pivot = object_model.get_articulation("overhead_pivot")
    seat_spin = object_model.get_articulation("seat_spin")

    clevis_left = support_frame.get_visual("clevis_left")
    clevis_right = support_frame.get_visual("clevis_right")
    pivot_barrel = yoke.get_visual("pivot_barrel")
    stem_socket = yoke.get_visual("stem_socket")
    top_spigot = stem.get_visual("top_spigot")
    lower_bearing_collar = stem.get_visual("lower_bearing_collar")
    upper_retainer_clip = stem.get_visual("upper_retainer_clip")
    seat_shell = disc_seat.get_visual("seat_shell")

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
        support_frame,
        yoke,
        elem_a=clevis_left,
        elem_b=pivot_barrel,
        name="left_clevis_contacts_pivot_barrel",
    )
    ctx.expect_contact(
        support_frame,
        yoke,
        elem_a=clevis_right,
        elem_b=pivot_barrel,
        name="right_clevis_contacts_pivot_barrel",
    )
    ctx.expect_contact(
        yoke,
        stem,
        elem_a=stem_socket,
        elem_b=top_spigot,
        name="stem_spigot_seats_in_yoke_socket",
    )
    ctx.expect_contact(
        stem,
        disc_seat,
        elem_a=lower_bearing_collar,
        elem_b=seat_shell,
        name="disc_seat_bears_on_lower_collar",
    )
    ctx.expect_gap(
        stem,
        disc_seat,
        axis="z",
        positive_elem=upper_retainer_clip,
        negative_elem=seat_shell,
        min_gap=0.0005,
        max_gap=0.0015,
        name="upper_retainer_clip_keeps_seat_captured",
    )
    ctx.expect_overlap(
        stem,
        disc_seat,
        axes="xy",
        elem_a=lower_bearing_collar,
        elem_b=seat_shell,
        min_overlap=0.04,
        name="seat_centered_under_stem",
    )

    rest_center = ctx.part_world_position(disc_seat)
    if rest_center is None:
        ctx.fail("disc_seat_has_world_position", "Disc seat position could not be resolved in rest pose.")
    else:
        with ctx.pose({overhead_pivot: 0.55}):
            swung_center = ctx.part_world_position(disc_seat)
            if swung_center is None:
                ctx.fail("disc_seat_has_world_position_swung", "Disc seat position could not be resolved in swung pose.")
            else:
                ctx.check(
                    "overhead_pivot_swings_disc_in_arc",
                    abs(swung_center[0] - rest_center[0]) > 0.75 and swung_center[2] > rest_center[2] + 0.20,
                    details=f"rest={rest_center}, swung={swung_center}",
                )
            ctx.expect_contact(support_frame, yoke, elem_a=clevis_left, elem_b=pivot_barrel)
            ctx.expect_contact(support_frame, yoke, elem_a=clevis_right, elem_b=pivot_barrel)

        with ctx.pose({seat_spin: 1.7}):
            spun_center = ctx.part_world_position(disc_seat)
            if spun_center is None:
                ctx.fail("disc_seat_has_world_position_spun", "Disc seat position could not be resolved in spun pose.")
            else:
                ctx.check(
                    "seat_spin_keeps_disc_on_same_axis",
                    math.dist(rest_center, spun_center) < 1e-6,
                    details=f"rest={rest_center}, spun={spun_center}",
                )
            ctx.expect_contact(stem, disc_seat, elem_a=lower_bearing_collar, elem_b=seat_shell)
            ctx.expect_gap(
                stem,
                disc_seat,
                axis="z",
                positive_elem=upper_retainer_clip,
                negative_elem=seat_shell,
                min_gap=0.0005,
                max_gap=0.0015,
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
