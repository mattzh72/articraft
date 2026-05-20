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
    model = ArticulatedObject(name="compact_service_access_panel")

    frame_finish = model.material("frame_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.74, 0.75, 0.77, 1.0))
    hardware_finish = model.material("hardware_finish", rgba=(0.56, 0.58, 0.61, 1.0))

    outer_w = 0.320
    outer_h = 0.420
    opening_w = 0.248
    opening_h = 0.348
    frame_depth = 0.050
    bezel_t = 0.004
    side_depth = frame_depth - bezel_t
    border_x = (outer_w - opening_w) / 2.0
    border_z = (outer_h - opening_h) / 2.0

    door_gap = 0.002
    door_w = opening_w - 2.0 * door_gap
    door_h = opening_h - 2.0 * door_gap
    door_depth = 0.016
    skin_t = 0.0025
    return_depth = door_depth - skin_t
    flange_w = 0.017

    hinge_radius = 0.0045
    hinge_x = -opening_w / 2.0 - 0.0025
    hinge_leaf_w = 0.018
    hinge_leaf_t = 0.005
    hinge_leaf_h = 0.300
    knuckle_len = 0.060
    knuckle_centers = (-0.128, -0.064, 0.0, 0.064, 0.128)
    frame_knuckle_centers = (knuckle_centers[1], knuckle_centers[3])
    door_knuckle_centers = (knuckle_centers[0], knuckle_centers[2], knuckle_centers[4])

    front_plane_local_y = -hinge_radius
    skin_center_local_y = front_plane_local_y - skin_t / 2.0
    return_center_local_y = front_plane_local_y - skin_t - return_depth / 2.0
    hardware_pad_local_y = -0.003
    latch_hub_local_y = -0.0015
    latch_bar_local_y = 0.001
    hinge_leaf_local_y = -0.002
    pin_to_panel_edge = hinge_radius

    frame = model.part("frame")
    frame.visual(
        Box((border_x, bezel_t, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + border_x / 2.0, -bezel_t / 2.0, 0.0)),
        material=frame_finish,
        name="left_bezel",
    )
    frame.visual(
        Box((border_x, bezel_t, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - border_x / 2.0, -bezel_t / 2.0, 0.0)),
        material=frame_finish,
        name="right_bezel",
    )
    frame.visual(
        Box((opening_w, bezel_t, border_z)),
        origin=Origin(xyz=(0.0, -bezel_t / 2.0, outer_h / 2.0 - border_z / 2.0)),
        material=frame_finish,
        name="top_bezel",
    )
    frame.visual(
        Box((opening_w, bezel_t, border_z)),
        origin=Origin(xyz=(0.0, -bezel_t / 2.0, -outer_h / 2.0 + border_z / 2.0)),
        material=frame_finish,
        name="bottom_bezel",
    )

    frame.visual(
        Box((border_x, side_depth, outer_h)),
        origin=Origin(
            xyz=(-outer_w / 2.0 + border_x / 2.0, -(bezel_t + side_depth / 2.0), 0.0)
        ),
        material=frame_finish,
        name="left_wall",
    )
    frame.visual(
        Box((border_x, side_depth, outer_h)),
        origin=Origin(
            xyz=(outer_w / 2.0 - border_x / 2.0, -(bezel_t + side_depth / 2.0), 0.0)
        ),
        material=frame_finish,
        name="right_wall",
    )
    frame.visual(
        Box((opening_w, side_depth, border_z)),
        origin=Origin(
            xyz=(0.0, -(bezel_t + side_depth / 2.0), outer_h / 2.0 - border_z / 2.0)
        ),
        material=frame_finish,
        name="top_wall",
    )
    frame.visual(
        Box((opening_w, side_depth, border_z)),
        origin=Origin(
            xyz=(0.0, -(bezel_t + side_depth / 2.0), -outer_h / 2.0 + border_z / 2.0)
        ),
        material=frame_finish,
        name="bottom_wall",
    )

    stop_depth = 0.010
    frame.visual(
        Box((0.005, stop_depth, 0.220)),
        origin=Origin(xyz=(opening_w / 2.0 - 0.0025, -0.009, 0.0)),
        material=frame_finish,
        name="frame_stop_right",
    )
    frame.visual(
        Box((door_w - 2.0 * flange_w, stop_depth, 0.005)),
        origin=Origin(xyz=(0.0, -0.009, opening_h / 2.0 - 0.0025)),
        material=frame_finish,
        name="frame_stop_top",
    )
    frame.visual(
        Box((door_w - 2.0 * flange_w, stop_depth, 0.005)),
        origin=Origin(xyz=(0.0, -0.009, -opening_h / 2.0 + 0.0025)),
        material=frame_finish,
        name="frame_stop_bottom",
    )

    frame.visual(
        Box((hinge_leaf_w, hinge_leaf_t, hinge_leaf_h)),
        origin=Origin(xyz=(hinge_x - hinge_leaf_w / 2.0, hinge_leaf_t / 2.0, 0.0)),
        material=hardware_finish,
        name="frame_hinge_leaf",
    )
    for index, z_pos in enumerate(frame_knuckle_centers):
        frame.visual(
            Cylinder(radius=hinge_radius, length=knuckle_len),
            origin=Origin(xyz=(hinge_x, hinge_radius, z_pos)),
            material=hardware_finish,
            name=f"frame_hinge_knuckle_{index}",
        )

    frame.visual(
        Box((0.010, 0.002, 0.090)),
        origin=Origin(xyz=(opening_w / 2.0 + 0.013, 0.001, 0.0)),
        material=hardware_finish,
        name="frame_strike",
    )
    frame.inertial = Inertial.from_geometry(
        Box((outer_w, frame_depth, outer_h)),
        mass=2.4,
        origin=Origin(xyz=(0.0, -frame_depth / 2.0, 0.0)),
    )

    panel = model.part("service_panel")
    panel.visual(
        Box((door_w, skin_t, door_h)),
        origin=Origin(
            xyz=(pin_to_panel_edge + door_w / 2.0, skin_center_local_y, 0.0)
        ),
        material=panel_finish,
        name="door_skin",
    )
    panel.visual(
        Box((flange_w, return_depth, door_h)),
        origin=Origin(
            xyz=(pin_to_panel_edge + flange_w / 2.0, return_center_local_y, 0.0)
        ),
        material=panel_finish,
        name="door_return_left",
    )
    panel.visual(
        Box((flange_w, return_depth, door_h)),
        origin=Origin(
            xyz=(
                pin_to_panel_edge + door_w - flange_w / 2.0,
                return_center_local_y,
                0.0,
            )
        ),
        material=panel_finish,
        name="door_return_right",
    )
    panel.visual(
        Box((door_w - 2.0 * flange_w, return_depth, flange_w)),
        origin=Origin(
            xyz=(
                pin_to_panel_edge + door_w / 2.0,
                return_center_local_y,
                door_h / 2.0 - flange_w / 2.0,
            )
        ),
        material=panel_finish,
        name="door_return_top",
    )
    panel.visual(
        Box((door_w - 2.0 * flange_w, return_depth, flange_w)),
        origin=Origin(
            xyz=(
                pin_to_panel_edge + door_w / 2.0,
                return_center_local_y,
                -door_h / 2.0 + flange_w / 2.0,
            )
        ),
        material=panel_finish,
        name="door_return_bottom",
    )

    panel.visual(
        Box((hinge_leaf_w, hinge_leaf_t, hinge_leaf_h)),
        origin=Origin(
            xyz=(pin_to_panel_edge + hinge_leaf_w / 2.0, hinge_leaf_local_y, 0.0)
        ),
        material=hardware_finish,
        name="door_hinge_leaf",
    )
    for index, z_pos in enumerate(door_knuckle_centers):
        panel.visual(
            Cylinder(radius=hinge_radius, length=knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=hardware_finish,
            name=f"door_hinge_knuckle_{index}",
        )

    latch_x = pin_to_panel_edge + door_w - 0.028
    panel.visual(
        Box((0.040, 0.003, 0.074)),
        origin=Origin(xyz=(latch_x, hardware_pad_local_y, 0.0)),
        material=hardware_finish,
        name="latch_plate",
    )
    panel.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(latch_x, latch_hub_local_y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hardware_finish,
        name="latch_hub",
    )
    panel.visual(
        Box((0.032, 0.003, 0.009)),
        origin=Origin(xyz=(latch_x, latch_bar_local_y, 0.0)),
        material=hardware_finish,
        name="latch_handle",
    )
    panel.inertial = Inertial.from_geometry(
        Box((door_w, door_depth, door_h)),
        mass=1.1,
        origin=Origin(
            xyz=(pin_to_panel_edge + door_w / 2.0, front_plane_local_y - door_depth / 2.0, 0.0)
        ),
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(hinge_x, hinge_radius, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=2.88,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("service_panel")
    hinge = object_model.get_articulation("frame_to_panel")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=16, name="door_clearance_through_swing")

    hinge_axis_ok = tuple(float(v) for v in hinge.axis) == (0.0, 0.0, 1.0)
    ctx.check(
        "vertical_hinge_axis",
        hinge_axis_ok,
        f"expected hinge axis (0, 0, 1), found {hinge.axis}",
    )

    limits = hinge.motion_limits
    stow_limit_ok = (
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 2.7
    )
    ctx.check(
        "stow_friendly_opening_range",
        stow_limit_ok,
        "panel should open from fully closed to a near-flat stowed angle",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            panel,
            frame,
            elem_a="door_return_right",
            elem_b="frame_stop_right",
            contact_tol=0.0005,
            name="latch_side_seats_on_stop",
        )
        ctx.expect_contact(
            panel,
            frame,
            elem_a="door_return_top",
            elem_b="frame_stop_top",
            contact_tol=0.0005,
            name="top_edge_seats_on_stop",
        )
        ctx.expect_contact(
            panel,
            frame,
            elem_a="door_return_bottom",
            elem_b="frame_stop_bottom",
            contact_tol=0.0005,
            name="bottom_edge_seats_on_stop",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="x",
            positive_elem="frame_strike",
            negative_elem="door_skin",
            min_gap=0.010,
            max_gap=0.030,
            name="latch_side_hardware_is_clearly_outboard",
        )

        door_skin_aabb = ctx.part_element_world_aabb(panel, elem="door_skin")
        right_bezel_aabb = ctx.part_element_world_aabb(frame, elem="right_bezel")
        flush_ok = (
            door_skin_aabb is not None
            and right_bezel_aabb is not None
            and abs(door_skin_aabb[1][1] - right_bezel_aabb[1][1]) <= 0.0005
        )
        ctx.check(
            "closed_panel_sits_flush_with_bezel",
            flush_ok,
            "door face should read flush with the front frame in the closed pose",
        )

    with ctx.pose({hinge: 1.57}):
        frame_aabb = ctx.part_world_aabb(frame)
        latch_aabb = ctx.part_element_world_aabb(panel, elem="latch_handle")
        swings_clear = (
            frame_aabb is not None
            and latch_aabb is not None
            and latch_aabb[0][1] > frame_aabb[1][1] + 0.190
        )
        ctx.check(
            "opened_panel_swings_clear_of_frame",
            swings_clear,
            "mid-swing pose should move the latch side clearly forward of the frame",
        )

    with ctx.pose({hinge: limits.upper if limits is not None and limits.upper is not None else 2.88}):
        left_bezel_aabb = ctx.part_element_world_aabb(frame, elem="left_bezel")
        door_skin_aabb = ctx.part_element_world_aabb(panel, elem="door_skin")
        side_stow_ok = (
            left_bezel_aabb is not None
            and door_skin_aabb is not None
            and door_skin_aabb[1][0] <= left_bezel_aabb[1][0] - 0.004
            and door_skin_aabb[0][1] >= left_bezel_aabb[1][1] + 0.008
        )
        ctx.check(
            "opened_panel_stows_along_hinge_side",
            side_stow_ok,
            "open-limit pose should fold the panel flat along the hinge side, ahead of the frame face",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
