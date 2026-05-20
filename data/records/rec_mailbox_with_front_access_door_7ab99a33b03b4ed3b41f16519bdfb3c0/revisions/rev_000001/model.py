from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="communal_mailbox")

    frame_mat = model.material("dark_powder_coat", color=(0.16, 0.18, 0.19, 1.0))
    housing_mat = model.material("galvanized_steel", color=(0.55, 0.58, 0.58, 1.0))
    shadow_mat = model.material("dark_recess", color=(0.05, 0.055, 0.06, 1.0))
    door_mat = model.material("blue_gray_door", color=(0.28, 0.35, 0.41, 1.0))
    trim_mat = model.material("brushed_aluminum", color=(0.74, 0.74, 0.70, 1.0))
    label_mat = model.material("pale_label", color=(0.86, 0.82, 0.66, 1.0))
    handle_mat = model.material("black_plastic", color=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("frame")

    # A fixed wall/frame surround: four heavy rails around the mailbox opening.
    frame.visual(
        Box((0.60, 0.07, 0.06)),
        origin=Origin(xyz=(0.0, 0.02, 0.18)),
        material=frame_mat,
        name="top_rail",
    )
    frame.visual(
        Box((0.60, 0.07, 0.06)),
        origin=Origin(xyz=(0.0, 0.02, -0.18)),
        material=frame_mat,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.07, 0.07, 0.42)),
        origin=Origin(xyz=(-0.265, 0.02, 0.0)),
        material=frame_mat,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.07, 0.07, 0.42)),
        origin=Origin(xyz=(0.265, 0.02, 0.0)),
        material=frame_mat,
        name="side_rail_1",
    )

    # The rigid, hollow mailbox housing is fixed into the larger front frame.
    frame.visual(
        Box((0.018, 0.32, 0.32)),
        origin=Origin(xyz=(-0.241, 0.20, 0.0)),
        material=housing_mat,
        name="housing_wall_0",
    )
    frame.visual(
        Box((0.018, 0.32, 0.32)),
        origin=Origin(xyz=(0.241, 0.20, 0.0)),
        material=housing_mat,
        name="housing_wall_1",
    )
    frame.visual(
        Box((0.50, 0.32, 0.018)),
        origin=Origin(xyz=(0.0, 0.20, 0.151)),
        material=housing_mat,
        name="housing_top",
    )
    frame.visual(
        Box((0.50, 0.32, 0.018)),
        origin=Origin(xyz=(0.0, 0.20, -0.151)),
        material=housing_mat,
        name="housing_bottom",
    )
    frame.visual(
        Box((0.50, 0.025, 0.32)),
        origin=Origin(xyz=(0.0, 0.3475, 0.0)),
        material=shadow_mat,
        name="back_panel",
    )

    # Fixed half of the exposed hinge, tied back into the side rail.
    frame.visual(
        Box((0.008, 0.020, 0.29)),
        origin=Origin(xyz=(-0.231, -0.025, 0.0)),
        material=trim_mat,
        name="hinge_leaf",
    )

    door = model.part("door")
    door.visual(
        Box((0.44, 0.018, 0.28)),
        # Door local frame is the vertical hinge axis; the panel extends rightward.
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        material=door_mat,
        name="door_panel",
    )
    door.visual(
        Box((0.39, 0.004, 0.018)),
        origin=Origin(xyz=(0.235, -0.011, 0.104)),
        material=trim_mat,
        name="mail_slot_trim",
    )
    door.visual(
        Box((0.17, 0.004, 0.040)),
        origin=Origin(xyz=(0.235, -0.011, 0.042)),
        material=label_mat,
        name="name_plate",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_mat,
        name="hinge_barrel",
    )

    pull_handle = model.part("pull_handle")
    pull_handle.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="pull_disc",
    )
    pull_handle.visual(
        Box((0.040, 0.005, 0.007)),
        origin=Origin(xyz=(0.0, -0.0305, 0.0)),
        material=trim_mat,
        name="grip_tab",
    )
    pull_handle.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="pivot_boss",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(-0.220, -0.020, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=pull_handle,
        origin=Origin(xyz=(0.365, -0.009, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=1.0, lower=-0.30, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    pull_handle = object_model.get_part("pull_handle")
    door_hinge = object_model.get_articulation("door_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.check(
        "door hinge is vertical",
        tuple(round(v, 6) for v in door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "handle has small rotary travel",
        handle_pivot.motion_limits is not None
        and handle_pivot.motion_limits.lower <= -0.25
        and handle_pivot.motion_limits.upper >= 0.25
        and handle_pivot.motion_limits.upper <= 0.35,
        details=f"limits={handle_pivot.motion_limits}",
    )
    ctx.expect_contact(
        door,
        frame,
        elem_a="hinge_barrel",
        elem_b="hinge_leaf",
        contact_tol=0.002,
        name="door barrel seats on fixed hinge leaf",
    )
    ctx.expect_contact(
        pull_handle,
        door,
        elem_a="pivot_boss",
        elem_b="door_panel",
        contact_tol=0.001,
        name="pull handle boss bears on door face",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.0}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door opens outward from the frame",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_tab = ctx.part_element_world_aabb(pull_handle, elem="grip_tab")
    with ctx.pose({handle_pivot: 0.30}):
        turned_tab = ctx.part_element_world_aabb(pull_handle, elem="grip_tab")
    ctx.check(
        "pull handle visibly rotates on local pivot",
        rest_tab is not None
        and turned_tab is not None
        and (turned_tab[1][2] - turned_tab[0][2]) > (rest_tab[1][2] - rest_tab[0][2]) + 0.008,
        details=f"rest={rest_tab}, turned={turned_tab}",
    )

    return ctx.report()


object_model = build_object_model()
