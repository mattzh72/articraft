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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_pet_door_insert")

    frame_white = model.material("frame_white", rgba=(0.93, 0.93, 0.90, 1.0))
    flap_tint = model.material("flap_tint", rgba=(0.62, 0.70, 0.78, 0.68))
    slider_gray = model.material("slider_gray", rgba=(0.22, 0.22, 0.24, 1.0))

    outer_width = 0.28
    outer_height = 0.36
    frame_depth = 0.026
    stile_width = 0.04
    rail_height = 0.05

    opening_top_z = outer_height - rail_height

    flap_width = 0.185
    flap_height = 0.20
    flap_thickness = 0.004
    hinge_z = opening_top_z - 0.003
    hinge_radius = 0.0042
    flap_sleeve_radius = 0.005
    flap_panel_height = 0.188

    track_width = 0.022
    track_height = 0.115
    track_plate_depth = 0.004
    track_rail_width = 0.004
    track_rail_depth = 0.008
    track_center_z = 0.1125
    track_center_y = frame_depth / 2.0 - track_plate_depth / 2.0
    track_center_x = flap_width / 2.0 + 0.012

    slider_width = 0.013
    slider_depth = 0.008
    slider_height = 0.040
    slider_rest_z = 0.077
    slider_travel = 0.055

    frame = model.part("frame")
    frame.visual(
        Box((stile_width, frame_depth, outer_height)),
        origin=Origin(xyz=(-(outer_width - stile_width) / 2.0, 0.0, outer_height / 2.0)),
        material=frame_white,
        name="left_stile",
    )
    frame.visual(
        Box((stile_width, frame_depth, outer_height)),
        origin=Origin(xyz=((outer_width - stile_width) / 2.0, 0.0, outer_height / 2.0)),
        material=frame_white,
        name="right_stile",
    )
    frame.visual(
        Box((outer_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, outer_height - rail_height / 2.0)),
        material=frame_white,
        name="top_rail",
    )
    frame.visual(
        Box((outer_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, rail_height / 2.0)),
        material=frame_white,
        name="bottom_rail",
    )

    flange_depth = 0.004
    flange_width = 0.31
    flange_height = 0.39
    flange_border = 0.015
    flange_y = frame_depth / 2.0 + flange_depth / 2.0
    frame.visual(
        Box((flange_border, flange_depth, flange_height)),
        origin=Origin(xyz=(-(flange_width - flange_border) / 2.0, flange_y, flange_height / 2.0)),
        material=frame_white,
        name="left_front_flange",
    )
    frame.visual(
        Box((flange_border, flange_depth, flange_height)),
        origin=Origin(xyz=((flange_width - flange_border) / 2.0, flange_y, flange_height / 2.0)),
        material=frame_white,
        name="right_front_flange",
    )
    frame.visual(
        Box((flange_width, flange_depth, flange_border)),
        origin=Origin(xyz=(0.0, flange_y, flange_height - flange_border / 2.0)),
        material=frame_white,
        name="top_front_flange",
    )
    frame.visual(
        Box((flange_width, flange_depth, flange_border)),
        origin=Origin(xyz=(0.0, flange_y, flange_border / 2.0)),
        material=frame_white,
        name="bottom_front_flange",
    )

    frame.visual(
        Cylinder(radius=hinge_radius, length=flap_width + 0.010),
        origin=Origin(xyz=(0.0, 0.0, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=slider_gray,
        name="hinge_rod",
    )
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        center_x = x_sign * track_center_x
        frame.visual(
            Box((track_width, track_plate_depth, track_height)),
            origin=Origin(xyz=(center_x, track_center_y, track_center_z)),
            material=frame_white,
            name=f"{side_name}_track_backplate",
        )
        for rail_name, rail_offset in (("inner", track_width / 2.0 - track_rail_width / 2.0), ("outer", -(track_width / 2.0 - track_rail_width / 2.0))):
            frame.visual(
                Box((track_rail_width, track_rail_depth, track_height)),
                origin=Origin(
                    xyz=(center_x + rail_offset, track_center_y + track_plate_depth / 2.0 + track_rail_depth / 2.0, track_center_z)
                ),
                material=frame_white,
                name=f"{side_name}_{rail_name}_track_rail",
            )

    frame.inertial = Inertial.from_geometry(
        Box((flange_width, frame_depth + flange_depth, flange_height)),
        mass=0.9,
        origin=Origin(xyz=(0.0, flange_depth / 2.0, flange_height / 2.0)),
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=flap_sleeve_radius, length=flap_width - 0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=slider_gray,
        name="hinge_sleeve",
    )
    flap.visual(
        Box((flap_width - 0.020, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=slider_gray,
        name="top_stiffener",
    )
    flap.visual(
        Box((flap_width, flap_thickness, flap_panel_height)),
        origin=Origin(xyz=(0.0, 0.0, -(0.015 + flap_panel_height / 2.0))),
        material=flap_tint,
        name="flap_panel",
    )
    flap.visual(
        Box((flap_width - 0.015, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -(0.015 + flap_panel_height - 0.006))),
        material=slider_gray,
        name="bottom_weight_strip",
    )
    flap.inertial = Inertial.from_geometry(
        Box((flap_width, 0.014, flap_height)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -(0.015 + flap_panel_height / 2.0))),
    )

    left_slider = model.part("left_lock_slider")
    left_slider.visual(
        Box((slider_width, slider_depth, slider_height)),
        material=slider_gray,
        name="slider_body",
    )
    left_slider.visual(
        Box((0.019, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=slider_gray,
        name="thumb_grip",
    )
    left_slider.visual(
        Box((0.007, 0.006, 0.014)),
        origin=Origin(xyz=(0.0065, 0.0, 0.013)),
        material=slider_gray,
        name="lock_tooth",
    )
    left_slider.inertial = Inertial.from_geometry(
        Box((0.019, 0.014, 0.040)),
        mass=0.05,
        origin=Origin(),
    )

    right_slider = model.part("right_lock_slider")
    right_slider.visual(
        Box((slider_width, slider_depth, slider_height)),
        material=slider_gray,
        name="slider_body",
    )
    right_slider.visual(
        Box((0.019, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=slider_gray,
        name="thumb_grip",
    )
    right_slider.visual(
        Box((0.007, 0.006, 0.014)),
        origin=Origin(xyz=(-0.0065, 0.0, 0.013)),
        material=slider_gray,
        name="lock_tooth",
    )
    right_slider.inertial = Inertial.from_geometry(
        Box((0.019, 0.014, 0.040)),
        mass=0.05,
        origin=Origin(),
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-1.1,
            upper=1.1,
        ),
    )
    model.articulation(
        "frame_to_left_lock_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_slider,
        origin=Origin(xyz=(-track_center_x, track_center_y + track_plate_depth / 2.0 + slider_depth / 2.0, slider_rest_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.12,
            lower=0.0,
            upper=slider_travel,
        ),
    )
    model.articulation(
        "frame_to_right_lock_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_slider,
        origin=Origin(xyz=(track_center_x, track_center_y + track_plate_depth / 2.0 + slider_depth / 2.0, slider_rest_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.12,
            lower=0.0,
            upper=slider_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    left_slider = object_model.get_part("left_lock_slider")
    right_slider = object_model.get_part("right_lock_slider")

    flap_hinge = object_model.get_articulation("frame_to_flap")
    left_slider_joint = object_model.get_articulation("frame_to_left_lock_slider")
    right_slider_joint = object_model.get_articulation("frame_to_right_lock_slider")

    hinge_rod = frame.get_visual("hinge_rod")
    bottom_rail = frame.get_visual("bottom_rail")
    left_track = frame.get_visual("left_track_backplate")
    right_track = frame.get_visual("right_track_backplate")
    flap_panel = flap.get_visual("flap_panel")
    hinge_sleeve = flap.get_visual("hinge_sleeve")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        frame,
        flap,
        elem_a=hinge_rod,
        elem_b=hinge_sleeve,
        reason="The flap hangs from a sleeve wrapped around the frame hinge rod.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(flap, frame, name="flap_is_mounted_to_frame")
    ctx.expect_contact(left_slider, frame, name="left_slider_contacts_track")
    ctx.expect_contact(right_slider, frame, name="right_slider_contacts_track")

    ctx.expect_gap(
        flap,
        frame,
        axis="z",
        negative_elem=bottom_rail,
        min_gap=0.045,
        max_gap=0.070,
        name="flap_hangs_above_threshold",
    )
    ctx.expect_gap(
        flap,
        left_slider,
        axis="z",
        min_gap=0.004,
        max_gap=0.020,
        name="left_slider_starts_below_flap",
    )
    ctx.expect_gap(
        flap,
        right_slider,
        axis="z",
        min_gap=0.004,
        max_gap=0.020,
        name="right_slider_starts_below_flap",
    )
    ctx.expect_within(
        left_slider,
        frame,
        axes="z",
        outer_elem=left_track,
        margin=0.0,
        name="left_slider_within_track_height",
    )
    ctx.expect_within(
        right_slider,
        frame,
        axes="z",
        outer_elem=right_track,
        margin=0.0,
        name="right_slider_within_track_height",
    )

    flap_rest_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)
    assert flap_rest_aabb is not None
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        flap_forward_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)
        assert flap_forward_aabb is not None
        ctx.check(
            "flap_swings_forward",
            flap_forward_aabb[1][1] > flap_rest_aabb[1][1] + 0.16,
            details=f"rest max y {flap_rest_aabb[1][1]:.4f}, open max y {flap_forward_aabb[1][1]:.4f}",
        )
        ctx.expect_contact(flap, frame, name="flap_forward_pose_contact")

    with ctx.pose({flap_hinge: flap_hinge.motion_limits.lower}):
        flap_back_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)
        assert flap_back_aabb is not None
        ctx.check(
            "flap_swings_backward",
            flap_back_aabb[0][1] < flap_rest_aabb[0][1] - 0.16,
            details=f"rest min y {flap_rest_aabb[0][1]:.4f}, open min y {flap_back_aabb[0][1]:.4f}",
        )
        ctx.expect_contact(flap, frame, name="flap_backward_pose_contact")

    left_rest_pos = ctx.part_world_position(left_slider)
    right_rest_pos = ctx.part_world_position(right_slider)
    assert left_rest_pos is not None
    assert right_rest_pos is not None

    with ctx.pose({left_slider_joint: left_slider_joint.motion_limits.upper}):
        left_up_pos = ctx.part_world_position(left_slider)
        assert left_up_pos is not None
        ctx.check(
            "left_slider_moves_vertically",
            abs(left_up_pos[0] - left_rest_pos[0]) < 1e-6
            and abs(left_up_pos[1] - left_rest_pos[1]) < 1e-6
            and left_up_pos[2] > left_rest_pos[2] + 0.05,
            details=f"rest={left_rest_pos}, raised={left_up_pos}",
        )
        ctx.expect_within(left_slider, frame, axes="z", outer_elem=left_track, margin=0.0, name="left_slider_up_within_track")
        ctx.expect_contact(left_slider, frame, name="left_slider_up_contact")

    with ctx.pose({right_slider_joint: right_slider_joint.motion_limits.upper}):
        right_up_pos = ctx.part_world_position(right_slider)
        assert right_up_pos is not None
        ctx.check(
            "right_slider_moves_vertically",
            abs(right_up_pos[0] - right_rest_pos[0]) < 1e-6
            and abs(right_up_pos[1] - right_rest_pos[1]) < 1e-6
            and right_up_pos[2] > right_rest_pos[2] + 0.05,
            details=f"rest={right_rest_pos}, raised={right_up_pos}",
        )
        ctx.expect_within(right_slider, frame, axes="z", outer_elem=right_track, margin=0.0, name="right_slider_up_within_track")
        ctx.expect_contact(right_slider, frame, name="right_slider_up_contact")

    for joint in (flap_hinge, left_slider_joint, right_slider_joint):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
