from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    wire_from_points,
)


def _rect_section(
    x: float,
    width: float,
    height: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    half_height = height * 0.5
    return [
        (x, -half_width, z_center - half_height),
        (x, half_width, z_center - half_height),
        (x, half_width, z_center + half_height),
        (x, -half_width, z_center + half_height),
    ]


def _spring_coil(
    *,
    length: float,
    centerline_radius: float,
    wire_radius: float,
    turns: int,
    start_x: float,
    y_offset: float,
    z_offset: float,
):
    samples = max(72, turns * 20)
    points: list[tuple[float, float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        angle = 2.0 * pi * turns * t
        points.append(
            (
                start_x + length * t,
                y_offset + centerline_radius * cos(angle),
                z_offset + centerline_radius * sin(angle),
            )
        )
    return wire_from_points(
        points,
        radius=wire_radius,
        radial_segments=14,
        cap_ends=True,
        corner_mode="miter",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_garage_barrier")

    housing_steel = model.material("housing_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.31, 0.33, 0.36, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.93, 0.94, 0.92, 1.0))
    warning_red = model.material("warning_red", rgba=(0.81, 0.10, 0.09, 1.0))
    counterweight_finish = model.material("counterweight_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))

    housing_x = 0.36
    housing_y = 0.30
    housing_h = 1.02
    wall_t = 0.006
    cap_t = 0.012
    hinge_x = -0.105
    hinge_z = 1.11
    bracket_t = 0.012
    bracket_gap = 0.10
    bracket_x = 0.036
    bracket_h = 0.18
    plate_x = -(housing_x * 0.5) - 0.041
    plate_z = hinge_z - 0.024

    housing = model.part("housing")
    housing.visual(
        Box((housing_x, wall_t, housing_h - cap_t)),
        origin=Origin(
            xyz=(
                0.0,
                (housing_y * 0.5) - (wall_t * 0.5),
                (housing_h - cap_t) * 0.5,
            )
        ),
        material=housing_steel,
        name="left_side",
    )
    housing.visual(
        Box((housing_x, wall_t, housing_h - cap_t)),
        origin=Origin(
            xyz=(
                0.0,
                -(housing_y * 0.5) + (wall_t * 0.5),
                (housing_h - cap_t) * 0.5,
            )
        ),
        material=housing_steel,
        name="right_side",
    )
    housing.visual(
        Box((wall_t, housing_y - (2.0 * wall_t) + 0.004, housing_h - cap_t)),
        origin=Origin(
            xyz=(
                (housing_x * 0.5) - (wall_t * 0.5),
                0.0,
                (housing_h - cap_t) * 0.5,
            )
        ),
        material=housing_steel,
        name="front_panel",
    )
    housing.visual(
        Box((0.008, 0.18, 0.34)),
        origin=Origin(xyz=((housing_x * 0.5) - 0.001, 0.0, 0.59)),
        material=bracket_steel,
        name="service_door",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(
            xyz=((housing_x * 0.5) + 0.008, 0.0, 0.60),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=counterweight_finish,
        name="service_handle",
    )
    housing.visual(
        Box((wall_t, housing_y - (2.0 * wall_t) + 0.004, housing_h - cap_t)),
        origin=Origin(
            xyz=(
                -(housing_x * 0.5) + (wall_t * 0.5),
                0.0,
                (housing_h - cap_t) * 0.5,
            )
        ),
        material=housing_steel,
        name="rear_panel",
    )
    housing.visual(
        Box((0.26, housing_y + 0.004, cap_t)),
        origin=Origin(xyz=(0.05, 0.0, housing_h - (cap_t * 0.5))),
        material=housing_steel,
        name="top_cap",
    )
    housing.visual(
        Box((0.12, 0.10, cap_t)),
        origin=Origin(xyz=(-0.12, 0.10, housing_h - (cap_t * 0.5))),
        material=housing_steel,
        name="left_hinge_rail",
    )
    housing.visual(
        Box((0.12, 0.10, cap_t)),
        origin=Origin(xyz=(-0.12, -0.10, housing_h - (cap_t * 0.5))),
        material=housing_steel,
        name="right_hinge_rail",
    )
    housing.visual(
        Box((housing_x + 0.026, housing_y + 0.02, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=bracket_steel,
        name="base_plinth",
    )
    housing.visual(
        Box((bracket_x, bracket_t, bracket_h)),
        origin=Origin(
            xyz=(
                hinge_x,
                (bracket_gap * 0.5) + (bracket_t * 0.5),
                1.11,
            )
        ),
        material=bracket_steel,
        name="left_bracket",
    )
    housing.visual(
        Box((bracket_x, bracket_t, bracket_h)),
        origin=Origin(
            xyz=(
                hinge_x,
                -((bracket_gap * 0.5) + (bracket_t * 0.5)),
                1.11,
            )
        ),
        material=bracket_steel,
        name="right_bracket",
    )

    boom_arm = model.part("boom_arm")
    boom_arm.visual(
        Cylinder(radius=0.028, length=bracket_gap),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=counterweight_finish,
        name="pivot_barrel",
    )
    boom_shell = section_loft(
        [
            _rect_section(0.02, 0.08, 0.10),
            _rect_section(1.55, 0.07, 0.085),
            _rect_section(3.10, 0.05, 0.06),
        ]
    )
    boom_arm.visual(
        mesh_from_geometry(boom_shell, "barrier_boom_shell_v7"),
        material=arm_finish,
        name="boom_shell",
    )
    rear_tail = section_loft(
        [
            _rect_section(-0.112, 0.060, 0.040, z_center=-0.020),
            _rect_section(-0.062, 0.064, 0.055, z_center=-0.006),
            _rect_section(-0.020, 0.066, 0.085, z_center=0.016),
        ]
    )
    boom_arm.visual(
        mesh_from_geometry(rear_tail, "barrier_rear_tail_v7"),
        material=arm_finish,
        name="rear_tail",
    )
    for stripe_name, stripe_x, stripe_y, stripe_z in (
        ("stripe_a", 0.55, 0.082, 0.096),
        ("stripe_b", 1.25, 0.078, 0.090),
        ("stripe_c", 1.95, 0.070, 0.082),
    ):
        boom_arm.visual(
            Box((0.10, stripe_y, stripe_z)),
            origin=Origin(xyz=(stripe_x, 0.0, 0.0)),
            material=warning_red,
            name=stripe_name,
        )
    boom_arm.visual(
        Box((0.22, 0.056, 0.068)),
        origin=Origin(xyz=(2.88, 0.0, 0.0)),
        material=warning_red,
        name="tip_band",
    )
    boom_arm.visual(
        Box((0.038, 0.095, 0.055)),
        origin=Origin(xyz=(-0.050, 0.0, 0.018)),
        material=counterweight_finish,
        name="counterweight_block",
    )
    boom_arm.visual(
        Box((0.012, 0.09, 0.020)),
        origin=Origin(xyz=(-0.105, 0.0, -0.024)),
        material=rubber_black,
        name="rear_bumper",
    )

    spring_stop_plate = model.part("spring_stop_plate")
    spring_stop_plate.visual(
        Box((0.010, 0.16, 0.10)),
        material=counterweight_finish,
        name="stop_plate",
    )
    spring_stop_plate.visual(
        Box((0.041, 0.024, 0.040)),
        origin=Origin(xyz=(0.0205, 0.060, -0.058)),
        material=bracket_steel,
        name="left_mount_tab",
    )
    spring_stop_plate.visual(
        Box((0.041, 0.024, 0.040)),
        origin=Origin(xyz=(0.0205, -0.060, -0.058)),
        material=bracket_steel,
        name="right_mount_tab",
    )
    for side_name, y_offset in (("left", 0.05), ("right", -0.05)):
        spring_stop_plate.visual(
            Cylinder(radius=0.004, length=0.036),
            origin=Origin(
                xyz=(0.023, y_offset, 0.0),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=spring_steel,
            name=f"{side_name}_spring_core",
        )
        spring_stop_plate.visual(
            mesh_from_geometry(
                _spring_coil(
                    length=0.036,
                    centerline_radius=0.0054,
                    wire_radius=0.0025,
                    turns=4,
                    start_x=0.005,
                    y_offset=y_offset,
                    z_offset=0.0,
                ),
                f"barrier_{side_name}_spring_coil_v7",
            ),
            material=spring_steel,
            name=f"{side_name}_spring",
        )

    model.articulation(
        "boom_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=boom_arm,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "housing_to_stop_plate",
        ArticulationType.FIXED,
        parent=housing,
        child=spring_stop_plate,
        origin=Origin(xyz=(plate_x, 0.0, plate_z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    boom_arm = object_model.get_part("boom_arm")
    spring_stop_plate = object_model.get_part("spring_stop_plate")
    boom_hinge = object_model.get_articulation("boom_hinge")
    limits = boom_hinge.motion_limits

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
    ctx.fail_if_isolated_parts(
        max_pose_samples=24,
        name="garage_barrier_pose_sweep_no_floating",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.check(
        "boom_hinge_axis_is_horizontal",
        boom_hinge.axis == (0.0, -1.0, 0.0),
        details=f"Expected horizontal negative-Y hinge axis, got {boom_hinge.axis!r}",
    )
    ctx.check(
        "boom_hinge_limits_are_realistic",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 1.2 <= limits.upper <= 1.45,
        details=f"Expected a realistic barrier rise range, got {limits!r}",
    )
    ctx.expect_contact(
        boom_arm,
        housing,
        elem_a="pivot_barrel",
        elem_b="left_bracket",
        contact_tol=1e-4,
        name="pivot_barrel_contacts_left_bracket",
    )
    ctx.expect_contact(
        boom_arm,
        housing,
        elem_a="pivot_barrel",
        elem_b="right_bracket",
        contact_tol=1e-4,
        name="pivot_barrel_contacts_right_bracket",
    )
    ctx.expect_contact(
        spring_stop_plate,
        housing,
        elem_a="left_mount_tab",
        elem_b="rear_panel",
        contact_tol=1e-4,
        name="left_mount_tab_contacts_rear_panel",
    )
    ctx.expect_contact(
        spring_stop_plate,
        housing,
        elem_a="right_mount_tab",
        elem_b="rear_panel",
        contact_tol=1e-4,
        name="right_mount_tab_contacts_rear_panel",
    )
    ctx.expect_overlap(
        spring_stop_plate,
        housing,
        axes="y",
        min_overlap=0.14,
        elem_a="stop_plate",
        elem_b="rear_panel",
        name="stop_plate_spans_rear_centerline",
    )

    with ctx.pose({boom_hinge: 0.0}):
        ctx.expect_gap(
            boom_arm,
            housing,
            axis="z",
            min_gap=0.03,
            max_gap=0.07,
            positive_elem="boom_shell",
            negative_elem="top_cap",
            name="boom_rest_clear_of_top_cap",
        )
        ctx.expect_contact(
            boom_arm,
            spring_stop_plate,
            elem_a="rear_bumper",
            elem_b="stop_plate",
            contact_tol=1e-4,
            name="rear_bumper_contacts_stop_plate_at_rest",
        )
        ctx.expect_overlap(
            boom_arm,
            spring_stop_plate,
            axes="yz",
            min_overlap=0.019,
            elem_a="rear_bumper",
            elem_b="stop_plate",
            name="rear_bumper_is_captured_by_stop_plate_face",
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({boom_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="boom_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="boom_hinge_lower_no_floating")
        with ctx.pose({boom_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="boom_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="boom_hinge_upper_no_floating")
            ctx.expect_gap(
                spring_stop_plate,
                boom_arm,
                axis="z",
                min_gap=0.02,
                positive_elem="stop_plate",
                negative_elem="rear_bumper",
                name="raised_boom_clears_stop_plate",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
