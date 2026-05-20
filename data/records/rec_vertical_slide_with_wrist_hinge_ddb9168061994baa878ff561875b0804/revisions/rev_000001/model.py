from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_tool_slide")

    frame_color = model.material("frame_steel", rgba=(0.26, 0.29, 0.33, 1.0))
    carriage_color = model.material("carriage_green", rgba=(0.28, 0.44, 0.34, 1.0))
    guide_color = model.material("guide_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    wrist_color = model.material("wrist_plate", rgba=(0.66, 0.67, 0.70, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.34, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=frame_color,
        name="base_plate",
    )
    frame.visual(
        Box((0.14, 0.12, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=frame_color,
        name="column",
    )
    frame.visual(
        Box((0.08, 0.04, 0.58)),
        origin=Origin(xyz=(0.0, 0.08, 0.41)),
        material=guide_color,
        name="guide_rail",
    )
    frame.visual(
        Box((0.18, 0.16, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=frame_color,
        name="top_cap",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.34, 0.28, 0.80)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.03, 0.04, 0.16)),
        origin=Origin(xyz=(-0.055, 0.01, 0.0)),
        material=carriage_color,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.03, 0.04, 0.16)),
        origin=Origin(xyz=(0.055, 0.01, 0.0)),
        material=carriage_color,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.14, 0.04, 0.16)),
        origin=Origin(xyz=(0.0, 0.03, 0.0)),
        material=carriage_color,
        name="carriage_bridge",
    )
    carriage.visual(
        Box((0.08, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.02, 0.065)),
        material=carriage_color,
        name="top_rear_cap",
    )
    carriage.visual(
        Box((0.08, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.02, -0.065)),
        material=carriage_color,
        name="bottom_rear_cap",
    )
    carriage.visual(
        Box((0.02, 0.06, 0.10)),
        origin=Origin(xyz=(-0.075, 0.08, 0.015)),
        material=carriage_color,
        name="left_bracket",
    )
    carriage.visual(
        Box((0.02, 0.06, 0.10)),
        origin=Origin(xyz=(0.075, 0.08, 0.015)),
        material=carriage_color,
        name="right_bracket",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.17, 0.11, 0.16)),
        mass=8.0,
        origin=Origin(),
    )

    wrist = model.part("wrist_plate")
    wrist.visual(
        Cylinder(radius=0.012, length=0.13),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=guide_color,
        name="wrist_barrel",
    )
    wrist.visual(
        Box((0.12, 0.014, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=wrist_color,
        name="wrist_blade",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.12, 0.03, 0.13)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.09, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.20,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "carriage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, 0.095, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.5,
            lower=-0.10,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    wrist = object_model.get_part("wrist_plate")

    slide = object_model.get_articulation("frame_to_carriage")
    hinge = object_model.get_articulation("carriage_to_wrist")

    guide_rail = frame.get_visual("guide_rail")
    bridge = carriage.get_visual("carriage_bridge")
    left_bracket = carriage.get_visual("left_bracket")
    right_bracket = carriage.get_visual("right_bracket")
    barrel = wrist.get_visual("wrist_barrel")
    blade = wrist.get_visual("wrist_blade")

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
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_pose_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=12,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check(
        "parts_present",
        all(part is not None for part in (frame, carriage, wrist)),
        "expected frame, carriage, and wrist_plate parts",
    )
    ctx.check(
        "slide_joint_axis_and_limits",
        slide.axis == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == 0.30,
        "carriage must slide vertically along +Z over a 0.30 m stroke",
    )
    ctx.check(
        "hinge_joint_axis_and_limits",
        hinge.axis == (1.0, 0.0, 0.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == -0.10
        and hinge.motion_limits.upper == 1.10,
        "wrist plate must hinge about a horizontal +X axis at the carriage front",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=bridge,
            elem_b=guide_rail,
            name="carriage_bridge_contacts_guide_at_low",
        )
        ctx.expect_origin_distance(
            frame,
            carriage,
            axes="x",
            min_dist=0.0,
            max_dist=0.001,
            name="carriage_centered_on_frame_low",
        )
        ctx.expect_origin_gap(
            carriage,
            frame,
            axis="z",
            min_gap=0.19,
            max_gap=0.21,
            name="carriage_low_height",
        )
        ctx.expect_contact(
            wrist,
            carriage,
            elem_a=barrel,
            elem_b=left_bracket,
            name="wrist_barrel_contacts_left_bracket_at_rest",
        )
        ctx.expect_contact(
            wrist,
            carriage,
            elem_a=barrel,
            elem_b=right_bracket,
            name="wrist_barrel_contacts_right_bracket_at_rest",
        )
        ctx.expect_gap(
            wrist,
            frame,
            axis="y",
            min_gap=0.015,
            name="wrist_rest_stands_forward_of_column",
        )
        ctx.expect_overlap(
            wrist,
            carriage,
            axes="x",
            min_overlap=0.10,
            elem_a=blade,
            elem_b=bridge,
            name="wrist_plate_width_matches_carriage_front",
        )

    slide_limits = slide.motion_limits
    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({slide: slide_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="slide_lower_no_floating")
        with ctx.pose({slide: slide_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="slide_upper_no_floating")
            ctx.expect_contact(
                carriage,
                frame,
                elem_a=bridge,
                elem_b=guide_rail,
                name="carriage_bridge_contacts_guide_at_high",
            )
            ctx.expect_origin_gap(
                carriage,
                frame,
                axis="z",
                min_gap=0.49,
                max_gap=0.51,
                name="carriage_high_height",
            )

    hinge_limits = hinge.motion_limits
    if hinge_limits is not None and hinge_limits.lower is not None and hinge_limits.upper is not None:
        with ctx.pose({hinge: hinge_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="hinge_lower_no_floating")
            ctx.expect_contact(
                wrist,
                carriage,
                elem_a=barrel,
                elem_b=left_bracket,
                name="wrist_barrel_left_contact_at_back_tilt",
            )
            ctx.expect_contact(
                wrist,
                carriage,
                elem_a=barrel,
                elem_b=right_bracket,
                name="wrist_barrel_right_contact_at_back_tilt",
            )
        with ctx.pose({hinge: hinge_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="hinge_upper_no_floating")
            ctx.expect_contact(
                wrist,
                carriage,
                elem_a=barrel,
                elem_b=left_bracket,
                name="wrist_barrel_left_contact_at_forward_tilt",
            )
            ctx.expect_contact(
                wrist,
                carriage,
                elem_a=barrel,
                elem_b=right_bracket,
                name="wrist_barrel_right_contact_at_forward_tilt",
            )
            ctx.expect_gap(
                wrist,
                frame,
                axis="y",
                min_gap=0.03,
                name="wrist_forward_tilt_clears_frame",
            )

    with ctx.pose({slide: 0.30, hinge: 1.10}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_upper_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_upper_pose_no_floating")
    with ctx.pose({slide: 0.30, hinge: -0.10}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_back_tilt_high_slide_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_back_tilt_high_slide_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
