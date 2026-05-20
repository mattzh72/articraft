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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    # Overall proportions for a medium residential recessed wall safe.
    body_width = 0.42
    body_height = 0.54
    body_depth = 0.24
    shell_thickness = 0.012
    frame_depth = 0.020
    frame_width = 0.050

    opening_width = body_width - 2.0 * frame_width
    opening_height = body_height - 2.0 * frame_width

    door_thickness = 0.040
    door_width = opening_width + 0.014
    door_height = opening_height + 0.014
    hinge_radius = 0.010
    hinge_axis_x = 0.018
    hinge_axis_y = -(opening_width / 2.0) - hinge_radius

    dial_radius = 0.040
    handle_bar_length = 0.120

    body_color = model.material("safe_body", rgba=(0.20, 0.22, 0.24, 1.0))
    door_color = model.material("safe_door", rgba=(0.15, 0.16, 0.18, 1.0))
    metal_color = model.material("hardware", rgba=(0.70, 0.72, 0.74, 1.0))
    grip_color = model.material("handle_grip", rgba=(0.26, 0.27, 0.29, 1.0))

    body = model.part("body")

    # Recessed box shell fixed inside a wall opening.
    body.visual(
        Box((shell_thickness, body_width, body_height)),
        origin=Origin(xyz=(-(body_depth - shell_thickness / 2.0), 0.0, 0.0)),
        material=body_color,
        name="back_wall",
    )
    side_depth = body_depth - frame_depth
    body.visual(
        Box((side_depth, shell_thickness, body_height)),
        origin=Origin(
            xyz=(-(body_depth + frame_depth) / 2.0, -(body_width - shell_thickness) / 2.0, 0.0)
        ),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((side_depth, shell_thickness, body_height)),
        origin=Origin(
            xyz=(-(body_depth + frame_depth) / 2.0, (body_width - shell_thickness) / 2.0, 0.0)
        ),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((side_depth, body_width - 2.0 * shell_thickness, shell_thickness)),
        origin=Origin(
            xyz=(-(body_depth + frame_depth) / 2.0, 0.0, (body_height - shell_thickness) / 2.0)
        ),
        material=body_color,
        name="top_wall",
    )
    body.visual(
        Box((side_depth, body_width - 2.0 * shell_thickness, shell_thickness)),
        origin=Origin(
            xyz=(-(body_depth + frame_depth) / 2.0, 0.0, -(body_height - shell_thickness) / 2.0)
        ),
        material=body_color,
        name="bottom_wall",
    )

    # Thick front frame around the opening.
    body.visual(
        Box((frame_depth, frame_width, opening_height)),
        origin=Origin(xyz=(-(frame_depth / 2.0), -(opening_width / 2.0 + frame_width / 2.0), 0.0)),
        material=body_color,
        name="frame_left",
    )
    body.visual(
        Box((frame_depth, frame_width, opening_height)),
        origin=Origin(xyz=(-(frame_depth / 2.0), (opening_width / 2.0 + frame_width / 2.0), 0.0)),
        material=body_color,
        name="frame_right",
    )
    body.visual(
        Box((frame_depth, opening_width, frame_width)),
        origin=Origin(xyz=(-(frame_depth / 2.0), 0.0, (opening_height / 2.0 + frame_width / 2.0))),
        material=body_color,
        name="frame_top",
    )
    body.visual(
        Box((frame_depth, opening_width, frame_width)),
        origin=Origin(xyz=(-(frame_depth / 2.0), 0.0, -(opening_height / 2.0 + frame_width / 2.0))),
        material=body_color,
        name="frame_bottom",
    )

    # Exposed hinge support on the fixed frame.
    body.visual(
        Box((0.014, 0.026, 0.40)),
        origin=Origin(xyz=(0.001, hinge_axis_y - 0.006, 0.0)),
        material=metal_color,
        name="hinge_support",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.10),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.150)),
        material=metal_color,
        name="upper_hinge_knuckle",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.10),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, -0.150)),
        material=metal_color,
        name="lower_hinge_knuckle",
    )

    door = model.part("door")

    slab_center_x = door_thickness / 2.0 - hinge_axis_x
    slab_center_y = hinge_radius + door_width / 2.0
    front_trim_x = slab_center_x + door_thickness / 2.0 + 0.003

    door.visual(
        Box((door_thickness, door_width, door_height)),
        origin=Origin(xyz=(slab_center_x, slab_center_y, 0.0)),
        material=door_color,
        name="door_slab",
    )

    # Reinforced heavy face panel.
    door.visual(
        Box((0.006, door_width - 0.070, 0.070)),
        origin=Origin(xyz=(front_trim_x, slab_center_y, door_height / 2.0 - 0.035)),
        material=door_color,
        name="trim_top",
    )
    door.visual(
        Box((0.006, door_width - 0.070, 0.070)),
        origin=Origin(xyz=(front_trim_x, slab_center_y, -(door_height / 2.0 - 0.035))),
        material=door_color,
        name="trim_bottom",
    )
    door.visual(
        Box((0.006, 0.070, door_height - 0.140)),
        origin=Origin(xyz=(front_trim_x, slab_center_y - (door_width / 2.0 - 0.035), 0.0)),
        material=door_color,
        name="trim_hinge_side",
    )
    door.visual(
        Box((0.006, 0.070, door_height - 0.140)),
        origin=Origin(xyz=(front_trim_x, slab_center_y + (door_width / 2.0 - 0.035), 0.0)),
        material=door_color,
        name="trim_latch_side",
    )

    # Exposed door-side hinge leaf and middle knuckle.
    door.visual(
        Box((0.022, 0.016, 0.260)),
        origin=Origin(xyz=(-0.011, 0.016, 0.0)),
        material=metal_color,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal_color,
        name="middle_hinge_knuckle",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=dial_radius + 0.004, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_color,
        name="dial_base",
    )
    dial.visual(
        Cylinder(radius=dial_radius, length=0.014),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_color,
        name="dial_ring",
    )
    dial.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_color,
        name="dial_center",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_color,
        name="handle_boss",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_color,
        name="handle_spindle",
    )
    handle.visual(
        Box((0.014, handle_bar_length, 0.018)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=grip_color,
        name="handle_bar",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=0.0, upper=1.55),
    )

    dial_y = slab_center_y + 0.004
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(door_thickness - hinge_axis_x, dial_y, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(door_thickness - hinge_axis_x, slab_center_y + 0.004, -0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-1.1, upper=1.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")

    door_hinge = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    handle_joint = object_model.get_articulation("door_to_handle")

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
        "articulation_axes_match_safe_mechanisms",
        door_hinge.axis == (0.0, 0.0, -1.0)
        and dial_joint.axis == (1.0, 0.0, 0.0)
        and handle_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"door axis={door_hinge.axis}, dial axis={dial_joint.axis}, "
            f"handle axis={handle_joint.axis}"
        ),
    )

    ctx.expect_contact(dial, door, elem_a="dial_base", elem_b="door_slab", name="dial_mounts_to_door")
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_boss",
        elem_b="door_slab",
        name="handle_mounts_to_door",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="door_slab",
            negative_elem="frame_right",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed_door_seats_on_frame",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a="door_slab",
            elem_b="frame_right",
            min_overlap=0.010,
            name="closed_door_covers_opening_edge",
        )

        closed_aabb = ctx.part_element_world_aabb(door, elem="door_slab")

    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_slab")

    if closed_aabb is not None and opened_aabb is not None:
        closed_center_x = 0.5 * (closed_aabb[0][0] + closed_aabb[1][0])
        opened_center_x = 0.5 * (opened_aabb[0][0] + opened_aabb[1][0])
        ctx.check(
            "door_opens_outward",
            opened_center_x > closed_center_x + 0.10,
            details=f"closed_center_x={closed_center_x:.4f}, opened_center_x={opened_center_x:.4f}",
        )
    else:
        ctx.fail("door_pose_measurement_available", "Could not measure door_slab AABB in test poses.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
