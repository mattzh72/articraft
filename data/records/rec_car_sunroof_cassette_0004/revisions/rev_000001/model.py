from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sunroof_cassette", assets=ASSETS)

    outer_x = 0.92
    outer_y = 0.64
    opening_x = 0.78
    opening_y = 0.50
    frame_depth = 0.065
    side_wall_x = (outer_x - opening_x) / 2.0
    end_wall_y = (outer_y - opening_y) / 2.0

    guide_x = 0.030
    guide_y = 0.58
    guide_z = 0.012
    guide_cx = opening_x / 2.0 - 0.015
    guide_cz = 0.026

    shoe_x = 0.022
    shoe_y = 0.18
    shoe_z = 0.012
    shoe_cz = guide_cz + (guide_z + shoe_z) / 2.0

    glass_x = 0.766
    glass_y = 0.492
    glass_z = 0.006
    seal_z = 0.004

    slide_travel = 0.24
    slider_closed_y = 0.126
    glass_joint_y = 0.116
    glass_joint_z = 0.045
    glass_y_shift = 0.001

    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.34, 0.36, 0.39, 1.0))
    glass_mat = model.material("sunroof_glass", rgba=(0.20, 0.30, 0.36, 0.45))
    seal_mat = model.material("seal", rgba=(0.06, 0.06, 0.07, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((side_wall_x, outer_y, frame_depth)),
        origin=Origin(xyz=(+(opening_x / 2.0 + side_wall_x / 2.0), 0.0, frame_depth / 2.0)),
        material=aluminum,
        name="left_side_frame",
    )
    frame.visual(
        Box((side_wall_x, outer_y, frame_depth)),
        origin=Origin(xyz=(-(opening_x / 2.0 + side_wall_x / 2.0), 0.0, frame_depth / 2.0)),
        material=aluminum,
        name="right_side_frame",
    )
    frame.visual(
        Box((opening_x, end_wall_y, frame_depth)),
        origin=Origin(xyz=(0.0, +(opening_y / 2.0 + end_wall_y / 2.0), frame_depth / 2.0)),
        material=aluminum,
        name="front_header",
    )
    frame.visual(
        Box((side_wall_x, end_wall_y, frame_depth)),
        origin=Origin(
            xyz=(
                +(opening_x / 2.0 + side_wall_x / 2.0),
                -(opening_y / 2.0 + end_wall_y / 2.0),
                frame_depth / 2.0,
            )
        ),
        material=aluminum,
        name="rear_left_cap",
    )
    frame.visual(
        Box((side_wall_x, end_wall_y, frame_depth)),
        origin=Origin(
            xyz=(
                -(opening_x / 2.0 + side_wall_x / 2.0),
                -(opening_y / 2.0 + end_wall_y / 2.0),
                frame_depth / 2.0,
            )
        ),
        material=aluminum,
        name="rear_right_cap",
    )
    frame.visual(
        Box((guide_x, guide_y, guide_z)),
        origin=Origin(xyz=(+guide_cx, 0.0, guide_cz)),
        material=dark_aluminum,
        name="left_guide_rail",
    )
    frame.visual(
        Box((guide_x, guide_y, guide_z)),
        origin=Origin(xyz=(-guide_cx, 0.0, guide_cz)),
        material=dark_aluminum,
        name="right_guide_rail",
    )
    frame.visual(
        Box((opening_x, 0.014, seal_z)),
        origin=Origin(xyz=(0.0, +(opening_y / 2.0 - 0.003), 0.053)),
        material=seal_mat,
        name="front_seat",
    )
    frame.visual(
        Box((opening_x + 0.008, 0.014, seal_z)),
        origin=Origin(xyz=(0.0, -(opening_y / 2.0 - 0.003), 0.053)),
        material=seal_mat,
        name="rear_seat",
    )
    frame.visual(
        Box((0.016, opening_y - 0.020, seal_z)),
        origin=Origin(xyz=(+(opening_x / 2.0 - 0.004), 0.0, 0.053)),
        material=seal_mat,
        name="left_seat",
    )
    frame.visual(
        Box((0.016, opening_y - 0.020, seal_z)),
        origin=Origin(xyz=(-(opening_x / 2.0 - 0.004), 0.0, 0.053)),
        material=seal_mat,
        name="right_seat",
    )
    frame.visual(
        Box((opening_x, opening_y - 0.10, 0.006)),
        origin=Origin(xyz=(0.0, -0.04, 0.003)),
        material=dark_aluminum,
        name="cassette_tray",
    )
    frame.inertial = Inertial.from_geometry(
        Box((outer_x, outer_y, frame_depth)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, frame_depth / 2.0)),
    )

    slider = model.part("slider")
    slider.visual(
        Box((shoe_x, shoe_y, shoe_z)),
        origin=Origin(xyz=(+guide_cx, -0.020, shoe_cz)),
        material=dark_aluminum,
        name="left_shoe",
    )
    slider.visual(
        Box((shoe_x, shoe_y, shoe_z)),
        origin=Origin(xyz=(-guide_cx, -0.020, shoe_cz)),
        material=dark_aluminum,
        name="right_shoe",
    )
    slider.visual(
        Box((opening_x - 0.046, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, -0.020, 0.038)),
        material=dark_aluminum,
        name="rear_bridge",
    )
    slider.visual(
        Box((0.060, 0.120, 0.012)),
        origin=Origin(xyz=(+0.340, 0.040, 0.041)),
        material=dark_aluminum,
        name="left_lift_arm",
    )
    slider.visual(
        Box((0.060, 0.120, 0.012)),
        origin=Origin(xyz=(-0.340, 0.040, 0.041)),
        material=dark_aluminum,
        name="right_lift_arm",
    )
    slider.visual(
        Box((0.040, 0.020, 0.006)),
        origin=Origin(xyz=(+0.320, glass_joint_y - 0.006, 0.032)),
        material=dark_aluminum,
        name="left_hinge_pad",
    )
    slider.visual(
        Box((0.040, 0.020, 0.006)),
        origin=Origin(xyz=(-0.320, glass_joint_y - 0.006, 0.032)),
        material=dark_aluminum,
        name="right_hinge_pad",
    )
    slider.inertial = Inertial.from_geometry(
        Box((opening_x - 0.050, 0.34, 0.040)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    glass = model.part("glass")
    glass.visual(
        Box((glass_x, glass_y, glass_z)),
        origin=Origin(xyz=(0.0, -glass_y / 2.0 + glass_y_shift, 0.017)),
        material=glass_mat,
        name="glass_panel",
    )
    glass.visual(
        Box((0.040, 0.020, 0.024)),
        origin=Origin(xyz=(+0.320, 0.000, 0.002)),
        material=dark_aluminum,
        name="left_hinge_ear",
    )
    glass.visual(
        Box((0.040, 0.020, 0.024)),
        origin=Origin(xyz=(-0.320, 0.000, 0.002)),
        material=dark_aluminum,
        name="right_hinge_ear",
    )
    glass.visual(
        Box((glass_x - 0.066, 0.006, seal_z)),
        origin=Origin(xyz=(0.0, -0.003 + glass_y_shift, 0.01242)),
        material=seal_mat,
        name="front_seal",
    )
    glass.visual(
        Box((glass_x - 0.066, 0.006, seal_z)),
        origin=Origin(xyz=(0.0, -(glass_y - 0.003) + glass_y_shift, 0.0124)),
        material=seal_mat,
        name="rear_seal",
    )
    glass.visual(
        Box((0.006, glass_y - 0.054, seal_z)),
        origin=Origin(xyz=(+(glass_x / 2.0 - 0.003), -glass_y / 2.0 + glass_y_shift, 0.0124)),
        material=seal_mat,
        name="left_seal",
    )
    glass.visual(
        Box((0.006, glass_y - 0.054, seal_z)),
        origin=Origin(xyz=(-(glass_x / 2.0 - 0.003), -glass_y / 2.0 + glass_y_shift, 0.0124)),
        material=seal_mat,
        name="right_seal",
    )
    glass.inertial = Inertial.from_geometry(
        Box((glass_x, glass_y, glass_z)),
        mass=5.8,
        origin=Origin(xyz=(0.0, -glass_y / 2.0 + glass_y_shift, 0.017)),
    )

    model.articulation(
        "frame_to_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slider,
        origin=Origin(xyz=(0.0, slider_closed_y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=slide_travel),
    )
    model.articulation(
        "slider_to_glass",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=glass,
        origin=Origin(xyz=(0.0, glass_joint_y, glass_joint_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    slider = object_model.get_part("slider")
    glass = object_model.get_part("glass")
    slide_joint = object_model.get_articulation("frame_to_slider")
    tilt_joint = object_model.get_articulation("slider_to_glass")

    left_guide = frame.get_visual("left_guide_rail")
    right_guide = frame.get_visual("right_guide_rail")
    front_seat = frame.get_visual("front_seat")
    rear_seat = frame.get_visual("rear_seat")
    left_shoe = slider.get_visual("left_shoe")
    right_shoe = slider.get_visual("right_shoe")
    left_hinge_pad = slider.get_visual("left_hinge_pad")
    right_hinge_pad = slider.get_visual("right_hinge_pad")
    left_hinge_ear = glass.get_visual("left_hinge_ear")
    right_hinge_ear = glass.get_visual("right_hinge_ear")
    front_seal = glass.get_visual("front_seal")
    rear_seal = glass.get_visual("rear_seal")
    left_seal = glass.get_visual("left_seal")
    right_seal = glass.get_visual("right_seal")
    glass_panel = glass.get_visual("glass_panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    slide_limits = slide_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits

    ctx.check(
        "slide_joint_axis_is_rearward",
        tuple(slide_joint.axis) == (0.0, -1.0, 0.0),
        details=f"Expected rearward slide axis, got {slide_joint.axis!r}.",
    )
    ctx.check(
        "tilt_joint_axis_is_transverse",
        tuple(tilt_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"Expected left-right hinge axis, got {tilt_joint.axis!r}.",
    )
    ctx.check(
        "slide_joint_limits_match_cassette_travel",
        slide_limits is not None
        and slide_limits.lower == 0.0
        and abs((slide_limits.upper or 0.0) - 0.24) <= 1e-9,
        details=f"Unexpected slide limits: {slide_limits!r}.",
    )
    ctx.check(
        "tilt_joint_limits_match_vent_angle",
        tilt_limits is not None
        and tilt_limits.lower == 0.0
        and abs((tilt_limits.upper or 0.0) - 0.20) <= 1e-9,
        details=f"Unexpected tilt limits: {tilt_limits!r}.",
    )

    glass_top_aabb = ctx.part_element_world_aabb(glass, elem=glass_panel)
    left_frame_top_aabb = ctx.part_element_world_aabb(frame, elem="left_side_frame")
    ctx.check(
        "glass_sits_flush_with_frame_top",
        glass_top_aabb is not None
        and left_frame_top_aabb is not None
        and abs(glass_top_aabb[1][2] - left_frame_top_aabb[1][2]) <= 0.001,
        details=(
            f"Glass top {glass_top_aabb!r} should be flush with frame top "
            f"{left_frame_top_aabb!r}."
        ),
    )

    rest_glass_pos = ctx.part_world_position(glass)
    rest_front_aabb = ctx.part_element_world_aabb(glass, elem=front_seal)
    rest_rear_aabb = ctx.part_element_world_aabb(glass, elem=rear_seal)

    ctx.expect_within(slider, frame, axes="xy", inner_elem=left_shoe, outer_elem=left_guide)
    ctx.expect_within(slider, frame, axes="xy", inner_elem=right_shoe, outer_elem=right_guide)
    ctx.expect_contact(slider, frame, elem_a=left_shoe, elem_b=left_guide)
    ctx.expect_contact(slider, frame, elem_a=right_shoe, elem_b=right_guide)
    ctx.expect_contact(glass, slider, elem_a=left_hinge_ear, elem_b=left_hinge_pad)
    ctx.expect_contact(glass, slider, elem_a=right_hinge_ear, elem_b=right_hinge_pad)
    ctx.expect_gap(
        glass,
        frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=front_seal,
        negative_elem=front_seat,
    )
    ctx.expect_gap(
        glass,
        frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=rear_seal,
        negative_elem=rear_seat,
    )
    ctx.expect_gap(
        glass,
        frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=left_seal,
        negative_elem=frame.get_visual("left_seat"),
    )
    ctx.expect_gap(
        glass,
        frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=right_seal,
        negative_elem=frame.get_visual("right_seat"),
    )
    ctx.expect_within(glass, frame, axes="xy", inner_elem=glass_panel)
    ctx.expect_overlap(glass, frame, axes="xy", min_overlap=0.45)

    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt_joint: tilt_limits.upper}):
            tilted_front_aabb = ctx.part_element_world_aabb(glass, elem=front_seal)
            tilted_rear_aabb = ctx.part_element_world_aabb(glass, elem=rear_seal)
            ctx.expect_gap(
                glass,
                frame,
                axis="z",
                max_gap=0.002,
                max_penetration=0.0,
                positive_elem=front_seal,
                negative_elem=front_seat,
            )
            ctx.expect_gap(
                glass,
                frame,
                axis="z",
                min_gap=0.085,
                positive_elem=rear_seal,
                negative_elem=rear_seat,
            )
            ctx.check(
                "rear_edge_lifts_in_vent_pose",
                rest_rear_aabb is not None
                and tilted_rear_aabb is not None
                and tilted_rear_aabb[0][2] > rest_rear_aabb[0][2] + 0.08,
                details=f"Rear edge should lift in vent pose: {rest_rear_aabb!r} -> {tilted_rear_aabb!r}.",
            )
            ctx.check(
                "front_edge_stays_nearly_seated_in_vent_pose",
                rest_front_aabb is not None
                and tilted_front_aabb is not None
                and abs(tilted_front_aabb[0][2] - rest_front_aabb[0][2]) <= 0.002,
                details=f"Front edge should stay near the frame seat: {rest_front_aabb!r} -> {tilted_front_aabb!r}.",
            )

    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({slide_joint: slide_limits.upper}):
            slid_glass_pos = ctx.part_world_position(glass)
            ctx.expect_within(slider, frame, axes="xy", inner_elem=left_shoe, outer_elem=left_guide)
            ctx.expect_within(slider, frame, axes="xy", inner_elem=right_shoe, outer_elem=right_guide)
            ctx.expect_contact(slider, frame, elem_a=left_shoe, elem_b=left_guide)
            ctx.expect_contact(slider, frame, elem_a=right_shoe, elem_b=right_guide)
            ctx.expect_gap(
                frame,
                glass,
                axis="y",
                min_gap=0.22,
                positive_elem=front_seat,
                negative_elem=front_seal,
            )
            ctx.check(
                "glass_panel_slides_rearward",
                rest_glass_pos is not None
                and slid_glass_pos is not None
                and slid_glass_pos[1] < rest_glass_pos[1] - 0.23,
                details=f"Glass should move rearward: {rest_glass_pos!r} -> {slid_glass_pos!r}.",
            )

    for joint, label in ((slide_joint, "slide"), (tilt_joint, "tilt")):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_lower_pose_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_upper_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
