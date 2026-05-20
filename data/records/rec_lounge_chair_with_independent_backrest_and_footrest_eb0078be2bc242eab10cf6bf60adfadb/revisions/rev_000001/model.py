from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reclining_lounge_chair")

    frame_metal = model.material("frame_metal", rgba=(0.18, 0.18, 0.20, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    cushion_fabric = model.material("cushion_fabric", rgba=(0.63, 0.58, 0.52, 1.0))
    accent_fabric = model.material("accent_fabric", rgba=(0.70, 0.66, 0.61, 1.0))
    base_rounded_loop = rounded_rect_profile(1.0, 1.0, 0.18, corner_segments=8)

    def _runner(points: list[tuple[float, float, float]], name: str):
        return mesh_from_geometry(
            wire_from_points(
                points,
                radius=0.018,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=10,
            ),
            name,
        )

    def _xy_loop(
        width: float,
        depth: float,
        radius: float,
        z: float,
        *,
        x_shift: float = 0.0,
        y_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        _ = radius
        return [
            (x * width + x_shift, y * depth + y_shift, z)
            for x, y in base_rounded_loop
        ]

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((0.74, 0.76, 0.30)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )
    left_runner_points = [
        (-0.33, 0.30, 0.02),
        (-0.33, 0.30, 0.15),
        (-0.25, 0.30, 0.23),
        (0.25, 0.30, 0.23),
        (0.33, 0.30, 0.14),
        (0.35, 0.30, 0.02),
    ]
    right_runner_points = [(x, -y, z) for x, y, z in left_runner_points]
    support_frame.visual(
        _runner(left_runner_points, "recliner_left_frame_runner_v2"),
        material=frame_metal,
        name="left_runner",
    )
    support_frame.visual(
        _runner(right_runner_points, "recliner_right_frame_runner_v2"),
        material=frame_metal,
        name="right_runner",
    )
    support_frame.visual(
        Box((0.10, 0.60, 0.04)),
        origin=Origin(xyz=(-0.18, 0.0, 0.26)),
        material=frame_metal,
        name="rear_support_bar",
    )
    support_frame.visual(
        Box((0.10, 0.60, 0.04)),
        origin=Origin(xyz=(0.18, 0.0, 0.26)),
        material=frame_metal,
        name="front_support_bar",
    )
    support_frame.visual(
        Box((0.56, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=frame_metal,
        name="lower_stretcher",
    )
    support_frame.visual(
        Box((0.08, 0.66, 0.04)),
        origin=Origin(xyz=(-0.33, 0.0, 0.02)),
        material=frame_metal,
        name="rear_floor_bar",
    )
    support_frame.visual(
        Box((0.08, 0.66, 0.04)),
        origin=Origin(xyz=(0.35, 0.0, 0.02)),
        material=frame_metal,
        name="front_floor_bar",
    )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.64, 0.72, 0.10)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )
    seat.visual(
        Box((0.62, 0.70, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=charcoal,
        name="seat_platform",
    )
    seat.visual(
        mesh_from_geometry(
            LoftGeometry(
                [
                    _xy_loop(0.56, 0.62, 0.055, 0.02),
                    _xy_loop(0.60, 0.66, 0.085, 0.05, x_shift=-0.01),
                    _xy_loop(0.54, 0.60, 0.095, 0.08, x_shift=-0.02),
                ],
                cap=True,
                closed=True,
            ),
            "recliner_seat_cushion_shell_v2",
        ),
        material=cushion_fabric,
        name="seat_cushion",
    )
    for visual_name, y_pos in (("left_side_rail", 0.29), ("right_side_rail", -0.29)):
        seat.visual(
            Box((0.62, 0.04, 0.06)),
            origin=Origin(xyz=(0.0, y_pos, 0.05)),
            material=frame_metal,
            name=visual_name,
        )
    for visual_name, x_pos, y_pos in (
        ("rear_pivot_left", -0.29, 0.29),
        ("rear_pivot_right", -0.29, -0.29),
        ("front_pivot_left", 0.29, 0.29),
        ("front_pivot_right", 0.29, -0.29),
    ):
        seat.visual(
            Box((0.04, 0.06, 0.10)),
            origin=Origin(xyz=(x_pos, y_pos, 0.05)),
            material=frame_metal,
            name=visual_name,
        )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((0.12, 0.66, 0.70)),
        mass=14.0,
        origin=Origin(xyz=(-0.06, 0.0, 0.35)),
    )
    backrest.visual(
        mesh_from_geometry(
            LoftGeometry(
                [
                    _xy_loop(0.13, 0.58, 0.045, 0.06, x_shift=-0.10),
                    _xy_loop(0.11, 0.64, 0.055, 0.36, x_shift=-0.08),
                    _xy_loop(0.08, 0.58, 0.040, 0.70, x_shift=-0.05),
                ],
                cap=True,
                closed=True,
            ),
            "recliner_backrest_panel_shell_v2",
        ),
        material=accent_fabric,
        name="backrest_panel",
    )
    for visual_name, y_pos in (("backrest_left_ear", 0.29), ("backrest_right_ear", -0.29)):
        backrest.visual(
            Box((0.08, 0.06, 0.10)),
            origin=Origin(xyz=(-0.04, y_pos, 0.05)),
            material=frame_metal,
            name=visual_name,
        )

    footrest = model.part("footrest")
    footrest.inertial = Inertial.from_geometry(
        Box((0.40, 0.58, 0.12)),
        mass=7.0,
        origin=Origin(xyz=(0.20, 0.0, -0.06)),
    )
    footrest.visual(
        mesh_from_geometry(
            LoftGeometry(
                [
                    _xy_loop(0.34, 0.54, 0.050, -0.12, x_shift=0.20),
                    _xy_loop(0.36, 0.58, 0.080, -0.09, x_shift=0.21),
                    _xy_loop(0.32, 0.52, 0.065, -0.06, x_shift=0.22),
                ],
                cap=True,
                closed=True,
            ),
            "recliner_footrest_panel_shell_v2",
        ),
        material=accent_fabric,
        name="footrest_panel",
    )
    for visual_name, y_pos in (("footrest_left_ear", 0.29), ("footrest_right_ear", -0.29)):
        footrest.visual(
            Box((0.08, 0.06, 0.10)),
            origin=Origin(xyz=(0.04, y_pos, -0.05)),
            material=frame_metal,
            name=visual_name,
        )

    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent=support_frame,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )
    model.articulation(
        "backrest_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.31, 0.0, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.6, lower=-0.15, upper=0.85),
    )
    model.articulation(
        "footrest_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=footrest,
        origin=Origin(xyz=(0.31, 0.0, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.85, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    backrest_hinge = object_model.get_articulation("backrest_recline")
    footrest_hinge = object_model.get_articulation("footrest_recline")

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
        "required_parts_present",
        True,
    )
    ctx.check(
        "backrest_axis_transverse",
        tuple(backrest_hinge.axis) == (0.0, 1.0, 0.0),
        f"axis={backrest_hinge.axis}",
    )
    ctx.check(
        "footrest_axis_transverse",
        tuple(footrest_hinge.axis) == (0.0, 1.0, 0.0),
        f"axis={footrest_hinge.axis}",
    )
    ctx.check(
        "seat_is_low_lounge_height",
        0.26 <= ctx.part_world_position(seat)[2] <= 0.32,
        f"seat origin z={ctx.part_world_position(seat)[2]:.3f}",
    )

    with ctx.pose({backrest_hinge: 0.0, footrest_hinge: 0.0}):
        ctx.expect_contact(
            seat,
            support_frame,
            elem_a="seat_platform",
            elem_b="rear_support_bar",
            name="seat_rear_bar_contact",
        )
        ctx.expect_contact(
            seat,
            support_frame,
            elem_a="seat_platform",
            elem_b="front_support_bar",
            name="seat_front_bar_contact",
        )
        ctx.expect_overlap(seat, support_frame, axes="xy", min_overlap=0.56, name="seat_over_frame")
        ctx.expect_origin_gap(
            seat,
            backrest,
            axis="x",
            min_gap=0.28,
            max_gap=0.34,
            name="backrest_hinge_at_rear_edge",
        )
        ctx.expect_origin_gap(
            footrest,
            seat,
            axis="x",
            min_gap=0.28,
            max_gap=0.34,
            name="footrest_hinge_at_front_edge",
        )
        ctx.expect_overlap(backrest, seat, axes="y", min_overlap=0.62, name="backrest_width_alignment")
        ctx.expect_overlap(footrest, seat, axes="y", min_overlap=0.56, name="footrest_width_alignment")
        ctx.expect_contact(
            backrest,
            seat,
            elem_a="backrest_left_ear",
            elem_b="rear_pivot_left",
            name="backrest_left_hinge_contact",
        )
        ctx.expect_contact(
            backrest,
            seat,
            elem_a="backrest_right_ear",
            elem_b="rear_pivot_right",
            name="backrest_right_hinge_contact",
        )
        ctx.expect_contact(
            footrest,
            seat,
            elem_a="footrest_left_ear",
            elem_b="front_pivot_left",
            name="footrest_left_hinge_contact",
        )
        ctx.expect_contact(
            footrest,
            seat,
            elem_a="footrest_right_ear",
            elem_b="front_pivot_right",
            name="footrest_right_hinge_contact",
        )
        ctx.expect_gap(
            backrest,
            seat,
            axis="z",
            positive_elem="backrest_panel",
            negative_elem="seat_cushion",
            min_gap=0.02,
            max_gap=0.12,
            name="backrest_panel_above_seat_cushion",
        )

    backrest_limits = backrest_hinge.motion_limits
    if backrest_limits is not None and backrest_limits.lower is not None and backrest_limits.upper is not None:
        with ctx.pose({backrest_hinge: backrest_limits.lower, footrest_hinge: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="backrest_lower_no_overlap")
            ctx.expect_overlap(backrest, seat, axes="y", min_overlap=0.62, name="backrest_lower_width_alignment")
        with ctx.pose({backrest_hinge: backrest_limits.upper, footrest_hinge: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="backrest_upper_no_overlap")
            ctx.expect_overlap(backrest, seat, axes="y", min_overlap=0.62, name="backrest_upper_width_alignment")

    footrest_limits = footrest_hinge.motion_limits
    if footrest_limits is not None and footrest_limits.lower is not None and footrest_limits.upper is not None:
        with ctx.pose({backrest_hinge: 0.0, footrest_hinge: footrest_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="footrest_lower_no_overlap")
            ctx.expect_overlap(footrest, seat, axes="y", min_overlap=0.56, name="footrest_lower_width_alignment")
        with ctx.pose({backrest_hinge: 0.0, footrest_hinge: footrest_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="footrest_upper_no_overlap")
            ctx.expect_overlap(footrest, seat, axes="y", min_overlap=0.56, name="footrest_upper_width_alignment")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
