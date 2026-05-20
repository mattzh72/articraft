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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_apartment_mailbox")

    powder_gray = model.material("powder_gray", rgba=(0.69, 0.71, 0.74, 1.0))
    door_gray = model.material("door_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.62, 0.28, 1.0))
    black = model.material("black", rgba=(0.07, 0.08, 0.09, 1.0))

    frame_width = 0.400
    frame_height = 0.200
    opening_width = 0.312
    opening_height = 0.116
    frame_depth = 0.020

    shell_width = 0.340
    shell_height = 0.140
    shell_depth = 0.276
    shell_wall = 0.012

    door_width = 0.306
    door_height = 0.110
    door_thickness = 0.012

    side_frame_width = (frame_width - opening_width) / 2.0
    top_frame_height = (frame_height - opening_height) / 2.0

    body = model.part("mailbox_body")
    body.visual(
        Box((frame_depth, frame_width, top_frame_height)),
        origin=Origin(xyz=(-frame_depth / 2.0, 0.0, opening_height / 2.0 + top_frame_height / 2.0)),
        material=powder_gray,
        name="frame_top",
    )
    body.visual(
        Box((frame_depth, frame_width, top_frame_height)),
        origin=Origin(xyz=(-frame_depth / 2.0, 0.0, -opening_height / 2.0 - top_frame_height / 2.0)),
        material=powder_gray,
        name="frame_bottom",
    )
    body.visual(
        Box((frame_depth, side_frame_width, opening_height)),
        origin=Origin(xyz=(-frame_depth / 2.0, -opening_width / 2.0 - side_frame_width / 2.0, 0.0)),
        material=powder_gray,
        name="frame_left",
    )
    body.visual(
        Box((frame_depth, side_frame_width, opening_height)),
        origin=Origin(xyz=(-frame_depth / 2.0, opening_width / 2.0 + side_frame_width / 2.0, 0.0)),
        material=powder_gray,
        name="frame_right",
    )
    body.visual(
        Box((shell_depth, shell_width, shell_wall)),
        origin=Origin(xyz=(-0.156, 0.0, shell_height / 2.0 - shell_wall / 2.0)),
        material=powder_gray,
        name="shell_top",
    )
    body.visual(
        Box((shell_depth, shell_width, shell_wall)),
        origin=Origin(xyz=(-0.156, 0.0, -shell_height / 2.0 + shell_wall / 2.0)),
        material=powder_gray,
        name="shell_bottom",
    )
    body.visual(
        Box((shell_depth, shell_wall, shell_height)),
        origin=Origin(xyz=(-0.156, -shell_width / 2.0 + shell_wall / 2.0, 0.0)),
        material=powder_gray,
        name="shell_left",
    )
    body.visual(
        Box((shell_depth, shell_wall, shell_height)),
        origin=Origin(xyz=(-0.156, shell_width / 2.0 - shell_wall / 2.0, 0.0)),
        material=powder_gray,
        name="shell_right",
    )
    body.visual(
        Box((shell_wall, shell_width, shell_height)),
        origin=Origin(xyz=(-0.288, 0.0, 0.0)),
        material=powder_gray,
        name="shell_back",
    )
    body.visual(
        Box((0.018, 0.010, 0.040)),
        origin=Origin(xyz=(-0.031, opening_width / 2.0, 0.0)),
        material=dark_metal,
        name="strike_tab",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.030),
        origin=Origin(xyz=(-0.002, -door_width / 2.0, 0.037)),
        material=dark_metal,
        name="hinge_barrel_upper",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.030),
        origin=Origin(xyz=(-0.002, -door_width / 2.0, -0.037)),
        material=dark_metal,
        name="hinge_barrel_lower",
    )

    door = model.part("front_door")
    door.visual(
        Box((door_thickness, door_width, door_height)),
        origin=Origin(xyz=(-door_thickness / 2.0, door_width / 2.0, 0.0)),
        material=door_gray,
        name="door_panel",
    )
    door.visual(
        Box((0.008, 0.254, 0.012)),
        origin=Origin(xyz=(-0.010, door_width / 2.0, 0.044)),
        material=dark_metal,
        name="stiffener_top",
    )
    door.visual(
        Box((0.008, 0.254, 0.012)),
        origin=Origin(xyz=(-0.010, door_width / 2.0, -0.044)),
        material=dark_metal,
        name="stiffener_bottom",
    )
    door.visual(
        Box((0.008, 0.020, 0.086)),
        origin=Origin(xyz=(-0.010, 0.014, 0.0)),
        material=dark_metal,
        name="stiffener_hinge",
    )
    door.visual(
        Box((0.008, 0.020, 0.086)),
        origin=Origin(xyz=(-0.010, 0.292, 0.0)),
        material=dark_metal,
        name="stiffener_latch",
    )
    door.visual(
        Box((0.002, 0.070, 0.016)),
        origin=Origin(xyz=(0.001, door_width / 2.0, -0.020)),
        material=black,
        name="nameplate_slot",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.044),
        material=dark_metal,
        name="hinge_barrel_center",
    )

    cam_lock = model.part("cam_lock")
    cam_lock.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lock_plug",
    )
    cam_lock.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="outer_escutcheon",
    )
    cam_lock.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_collar",
    )
    cam_lock.visual(
        Box((0.003, 0.014, 0.050)),
        origin=Origin(xyz=(-0.014, 0.0, 0.025)),
        material=dark_metal,
        name="cam_arm",
    )
    cam_lock.visual(
        Box((0.003, 0.020, 0.006)),
        origin=Origin(xyz=(-0.014, 0.007, 0.048)),
        material=dark_metal,
        name="cam_tip",
    )
    cam_lock.visual(
        Box((0.0015, 0.003, 0.012)),
        origin=Origin(xyz=(0.0105, 0.0, 0.0)),
        material=black,
        name="key_slot",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.002, -door_width / 2.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "door_to_cam_lock",
        ArticulationType.REVOLUTE,
        parent=door,
        child=cam_lock,
        origin=Origin(xyz=(-0.006, 0.268, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-1.20,
            upper=1.20,
        ),
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

    body = object_model.get_part("mailbox_body")
    door = object_model.get_part("front_door")
    cam_lock = object_model.get_part("cam_lock")
    door_hinge = object_model.get_articulation("body_to_door")
    lock_joint = object_model.get_articulation("door_to_cam_lock")

    ctx.check(
        "mailbox parts are present",
        body is not None and door is not None and cam_lock is not None,
        details="Expected mailbox body, front door, and cam lock parts.",
    )
    ctx.check(
        "door hinge is a vertical revolute joint",
        door_hinge.articulation_type == ArticulationType.REVOLUTE and tuple(door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}",
    )
    ctx.check(
        "cam lock rotates on the door normal",
        lock_joint.articulation_type == ArticulationType.REVOLUTE and tuple(lock_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={lock_joint.articulation_type}, axis={lock_joint.axis}",
    )

    ctx.allow_overlap(
        door,
        cam_lock,
        elem_a="door_panel",
        elem_b="lock_plug",
        reason="The keyed plug intentionally passes through the bored lock hole in the mailbox door.",
    )

    with ctx.pose({door_hinge: 0.0, lock_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_panel",
            negative_elem="frame_left",
            min_gap=0.002,
            max_gap=0.005,
            name="door keeps a narrow hinge-side clearance inside the frame",
        )
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="frame_right",
            negative_elem="door_panel",
            min_gap=0.002,
            max_gap=0.005,
            name="door keeps a narrow lock-side clearance inside the frame",
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            positive_elem="frame_top",
            negative_elem="door_panel",
            min_gap=0.002,
            max_gap=0.005,
            name="door keeps a narrow top clearance inside the frame",
        )
        ctx.expect_origin_distance(
            cam_lock,
            door,
            axes="y",
            min_dist=0.250,
            max_dist=0.280,
            name="key cylinder sits near the latch edge of the door",
        )

        closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        closed_cam_aabb = ctx.part_element_world_aabb(cam_lock, elem="cam_arm")

    with ctx.pose({door_hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    with ctx.pose({lock_joint: 1.0}):
        turned_cam_aabb = ctx.part_element_world_aabb(cam_lock, elem="cam_arm")

    if closed_aabb is None or open_aabb is None:
        ctx.fail("door panel AABB is available", f"closed={closed_aabb}, open={open_aabb}")
    else:
        ctx.check(
            "door swings outward from the frame",
            open_aabb[1][0] > closed_aabb[1][0] + 0.12,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    if closed_cam_aabb is None or turned_cam_aabb is None:
        ctx.fail("cam arm AABB is available", f"closed={closed_cam_aabb}, turned={turned_cam_aabb}")
    else:
        closed_cam_center = tuple((closed_cam_aabb[0][i] + closed_cam_aabb[1][i]) / 2.0 for i in range(3))
        turned_cam_center = tuple((turned_cam_aabb[0][i] + turned_cam_aabb[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "cam latch rotates within the door plane",
            abs(turned_cam_center[1] - closed_cam_center[1]) > 0.015
            and abs(turned_cam_center[2] - closed_cam_center[2]) > 0.008,
            details=f"closed_center={closed_cam_center}, turned_center={turned_cam_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
