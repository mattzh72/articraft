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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _profile_from_yz(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-z, y) for y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_compound_miter_saw")

    base_gray = model.material("base_gray", rgba=(0.21, 0.23, 0.25, 1.0))
    fence_gray = model.material("fence_gray", rgba=(0.66, 0.69, 0.72, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.77, 0.80, 0.83, 1.0))
    turntable_aluminum = model.material("turntable_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.31, 0.33, 0.36, 1.0))
    motor_black = model.material("motor_black", rgba=(0.12, 0.12, 0.13, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.90, 0.71, 0.14, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    blade_silver = model.material("blade_silver", rgba=(0.84, 0.85, 0.87, 1.0))
    hub_steel = model.material("hub_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.60, 0.42, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=base_gray,
        name="base_deck",
    )
    base.visual(
        Box((0.16, 0.08, 0.24)),
        origin=Origin(xyz=(0.0, 0.24, 0.18)),
        material=base_gray,
        name="rear_pedestal",
    )
    base.visual(
        Box((0.23, 0.025, 0.09)),
        origin=Origin(xyz=(-0.165, 0.165, 0.105)),
        material=fence_gray,
        name="left_fence",
    )
    base.visual(
        Box((0.23, 0.025, 0.09)),
        origin=Origin(xyz=(0.165, 0.165, 0.105)),
        material=fence_gray,
        name="right_fence",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.34),
        origin=Origin(xyz=(-0.045, 0.03, 0.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="left_rail",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.34),
        origin=Origin(xyz=(0.045, 0.03, 0.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="right_rail",
    )
    base.visual(
        Box((0.11, 0.02, 0.024)),
        origin=Origin(xyz=(0.0, 0.19, 0.18)),
        material=base_gray,
        name="rail_anchor",
    )

    turntable = model.part("turntable")
    turntable_top = ExtrudeWithHolesGeometry(
        _circle_profile(0.15, segments=80),
        [_rect_profile(0.09, 0.22)],
        height=0.035,
        center=False,
    )
    guard_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            _profile_from_yz(
                [
                    (-0.233, -0.050),
                    (-0.230, 0.015),
                    (-0.208, 0.072),
                    (-0.162, 0.112),
                    (-0.102, 0.121),
                    (-0.045, 0.104),
                    (-0.010, 0.066),
                    (0.000, 0.022),
                    (-0.022, -0.005),
                    (-0.070, 0.018),
                    (-0.120, 0.043),
                    (-0.168, 0.045),
                    (-0.202, 0.022),
                    (-0.216, -0.012),
                    (-0.216, -0.050),
                ]
            ),
            0.006,
            center=True,
        ).rotate_y(math.pi / 2.0),
        "upper_guard_plate",
    )
    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.022, -0.012, 0.030),
                (-0.021, -0.040, 0.070),
                (-0.010, -0.078, 0.103),
                (0.010, -0.084, 0.103),
                (0.021, -0.058, 0.075),
                (0.022, -0.026, 0.036),
            ],
            radius=0.007,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
        "saw_handle_loop",
    )
    turntable.visual(
        mesh_from_geometry(turntable_top, "turntable_top"),
        material=turntable_aluminum,
        name="table_top",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.022, 0.06, 0.022)),
        origin=Origin(xyz=(-0.045, 0.0, 0.011)),
        material=carriage_gray,
        name="left_slider",
    )
    carriage.visual(
        Box((0.022, 0.06, 0.022)),
        origin=Origin(xyz=(0.045, 0.0, 0.011)),
        material=carriage_gray,
        name="right_slider",
    )
    carriage.visual(
        Box((0.16, 0.035, 0.032)),
        origin=Origin(xyz=(0.0, 0.020, 0.027)),
        material=carriage_gray,
        name="saddle_bridge",
    )
    carriage.visual(
        Box((0.040, 0.024, 0.080)),
        origin=Origin(xyz=(0.0, 0.038, 0.083)),
        material=carriage_gray,
        name="riser",
    )
    carriage.visual(
        Box((0.10, 0.020, 0.022)),
        origin=Origin(xyz=(0.0, 0.032, 0.112)),
        material=carriage_gray,
        name="pivot_crossbar",
    )
    carriage.visual(
        Box((0.018, 0.050, 0.060)),
        origin=Origin(xyz=(-0.040, 0.015, 0.132)),
        material=carriage_gray,
        name="left_yoke",
    )
    carriage.visual(
        Box((0.018, 0.050, 0.060)),
        origin=Origin(xyz=(0.040, 0.015, 0.132)),
        material=carriage_gray,
        name="right_yoke",
    )

    saw_head = model.part("saw_head")
    saw_head.visual(
        Cylinder(radius=0.015, length=0.062),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_steel,
        name="pivot_axle",
    )
    saw_head.visual(
        Box((0.035, 0.150, 0.040)),
        origin=Origin(xyz=(-0.024, -0.085, 0.020)),
        material=carriage_gray,
        name="arm",
    )
    saw_head.visual(
        Box((0.060, 0.050, 0.035)),
        origin=Origin(xyz=(-0.005, -0.035, 0.030)),
        material=carriage_gray,
        name="pivot_knuckle",
    )
    saw_head.visual(
        Box((0.075, 0.070, 0.050)),
        origin=Origin(xyz=(0.0325, -0.115, 0.027)),
        material=motor_black,
        name="gearcase",
    )
    saw_head.visual(
        Cylinder(radius=0.045, length=0.110),
        origin=Origin(
            xyz=(0.125, -0.145, 0.020),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=motor_black,
        name="motor_body",
    )
    saw_head.visual(
        Cylinder(radius=0.05, length=0.03),
        origin=Origin(
            xyz=(0.195, -0.145, 0.020),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=motor_black,
        name="motor_cap",
    )
    saw_head.visual(
        Cylinder(radius=0.024, length=0.02),
        origin=Origin(
            xyz=(0.0135, -0.125, -0.028),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hub_steel,
        name="spindle_boss",
    )
    saw_head.visual(
        Box((0.03, 0.04, 0.006)),
        origin=Origin(xyz=(0.0135, -0.125, -0.001)),
        material=motor_black,
        name="spindle_mount",
    )
    saw_head.visual(
        guard_plate_mesh,
        origin=Origin(xyz=(-0.022, 0.0, 0.0)),
        material=guard_yellow,
        name="guard_left",
    )
    saw_head.visual(
        guard_plate_mesh,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=guard_yellow,
        name="guard_right",
    )
    saw_head.visual(
        Box((0.060, 0.18, 0.020)),
        origin=Origin(xyz=(0.0, -0.135, 0.084)),
        material=guard_yellow,
        name="guard_top",
    )
    saw_head.visual(
        handle_mesh,
        material=handle_black,
        name="handle_loop",
    )
    saw_head.visual(
        Cylinder(radius=0.005, length=0.055),
        origin=Origin(xyz=(0.0, -0.060, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="trigger_bar",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.108, length=0.003),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_silver,
        name="blade_disc",
    )
    blade.visual(
        Cylinder(radius=0.018, length=0.007),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_steel,
        name="blade_hub",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.87,
            upper=0.87,
        ),
    )
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.13, 0.191)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=0.5,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "carriage_to_saw_head",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=saw_head,
        origin=Origin(xyz=(0.0, -0.005, 0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.4,
            lower=0.0,
            upper=0.55,
        ),
    )
    model.articulation(
        "head_to_blade",
        ArticulationType.CONTINUOUS,
        parent=saw_head,
        child=blade,
        origin=Origin(xyz=(0.0, -0.125, -0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    carriage = object_model.get_part("carriage")
    saw_head = object_model.get_part("saw_head")
    blade = object_model.get_part("blade")

    table_yaw = object_model.get_articulation("base_to_turntable")
    carriage_slide = object_model.get_articulation("base_to_carriage")
    head_pitch = object_model.get_articulation("carriage_to_saw_head")
    blade_spin = object_model.get_articulation("head_to_blade")

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
        "articulation_axes_match_saw_mechanics",
        table_yaw.axis == (0.0, 0.0, 1.0)
        and carriage_slide.axis == (0.0, -1.0, 0.0)
        and head_pitch.axis == (1.0, 0.0, 0.0)
        and blade_spin.axis == (1.0, 0.0, 0.0),
        "Turntable should yaw about +Z, carriage should slide along the rail axis, and head/blade should rotate about the spindle-side X axis.",
    )
    ctx.check(
        "articulation_types_match_saw_mechanics",
        table_yaw.joint_type == ArticulationType.REVOLUTE
        and carriage_slide.joint_type == ArticulationType.PRISMATIC
        and head_pitch.joint_type == ArticulationType.REVOLUTE
        and blade_spin.joint_type == ArticulationType.CONTINUOUS,
        "The miter saw needs yaw, slide, pitch, and blade spin articulations with the expected joint types.",
    )

    with ctx.pose({table_yaw: 0.0, carriage_slide: 0.0, head_pitch: 0.0}):
        ctx.expect_contact(turntable, base, contact_tol=1e-6, name="turntable_seats_on_base")
        ctx.expect_gap(
            turntable,
            base,
            axis="z",
            max_gap=0.0,
            max_penetration=0.0,
            negative_elem="base_deck",
            name="turntable_base_flush_seating",
        )
        ctx.expect_within(
            turntable,
            base,
            axes="xy",
            margin=0.03,
            name="turntable_within_base_footprint",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_a="left_slider",
            elem_b="left_rail",
            contact_tol=0.001,
            name="left_slider_contacts_left_rail",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_a="right_slider",
            elem_b="right_rail",
            contact_tol=0.001,
            name="right_slider_contacts_right_rail",
        )
        ctx.expect_contact(
            saw_head,
            carriage,
            elem_a="pivot_axle",
            elem_b="left_yoke",
            contact_tol=1e-6,
            name="head_axle_contacts_left_yoke",
        )
        ctx.expect_contact(
            blade,
            saw_head,
            elem_a="blade_hub",
            elem_b="spindle_boss",
            contact_tol=1e-6,
            name="blade_hub_contacts_spindle_boss",
        )
        ctx.expect_gap(
            blade,
            turntable,
            axis="z",
            min_gap=0.04,
            max_gap=0.08,
            name="blade_clear_above_turntable_at_rest",
        )
        ctx.expect_gap(
            saw_head,
            turntable,
            axis="z",
            min_gap=0.07,
            name="raised_head_clears_turntable",
        )

    table_limits = table_yaw.motion_limits
    if table_limits is not None and table_limits.lower is not None and table_limits.upper is not None:
        with ctx.pose({table_yaw: table_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="turntable_yaw_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="turntable_yaw_lower_no_floating")
            ctx.expect_contact(turntable, base, contact_tol=1e-6, name="turntable_yaw_lower_stays_seated")
        with ctx.pose({table_yaw: table_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="turntable_yaw_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="turntable_yaw_upper_no_floating")
            ctx.expect_contact(turntable, base, contact_tol=1e-6, name="turntable_yaw_upper_stays_seated")

    slide_limits = carriage_slide.motion_limits
    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({carriage_slide: slide_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_slide_lower_no_floating")
        with ctx.pose({carriage_slide: slide_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_slide_upper_no_floating")
            ctx.expect_contact(
                carriage,
                base,
                elem_a="left_slider",
                elem_b="left_rail",
                contact_tol=0.001,
                name="left_slider_stays_on_left_rail_at_full_extension",
            )
            ctx.expect_contact(
                carriage,
                base,
                elem_a="right_slider",
                elem_b="right_rail",
                contact_tol=0.001,
                name="right_slider_stays_on_right_rail_at_full_extension",
            )

    head_limits = head_pitch.motion_limits
    if head_limits is not None and head_limits.lower is not None and head_limits.upper is not None:
        with ctx.pose({head_pitch: head_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_pitch_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="head_pitch_lower_no_floating")
        with ctx.pose({head_pitch: head_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_pitch_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="head_pitch_upper_no_floating")
            ctx.expect_gap(
                blade,
                base,
                axis="z",
                min_gap=0.01,
                negative_elem="base_deck",
                name="blade_clears_base_deck_at_full_pitch",
            )
            ctx.expect_contact(
                blade,
                turntable,
                elem_a="blade_disc",
                elem_b="table_top",
                contact_tol=0.03,
                name="blade_approaches_turntable_at_full_pitch",
            )

    with ctx.pose({blade_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="blade_spin_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="blade_spin_quarter_turn_no_floating")
        ctx.expect_contact(
            blade,
            saw_head,
            elem_a="blade_hub",
            elem_b="spindle_boss",
            contact_tol=1e-6,
            name="blade_hub_stays_on_spindle_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
