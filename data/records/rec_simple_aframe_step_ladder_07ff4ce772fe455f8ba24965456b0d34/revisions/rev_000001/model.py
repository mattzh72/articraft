from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import asin, atan2, pi

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


FRONT_RAIL_TOP_Z = 1.08
FRONT_RAIL_BOTTOM_X = -0.16
REAR_RAIL_BOTTOM_X_OPEN = 0.22
REAR_RAIL_TOP_X_OFFSET = 0.0
FRONT_RAIL_CENTER_Y = 0.20
REAR_RAIL_CENTER_Y = 0.238
BRACE_MOUNT_Y = 0.229
RECEIVER_Y = 0.230
TOP_CAP_HINGE_X = 0.045


def _front_line_x(z: float) -> float:
    return (FRONT_RAIL_TOP_Z - z) * FRONT_RAIL_BOTTOM_X / FRONT_RAIL_TOP_Z


def _rear_line_x(z_below_hinge: float) -> float:
    return REAR_RAIL_TOP_X_OFFSET + (-z_below_hinge) * REAR_RAIL_BOTTOM_X_OPEN / FRONT_RAIL_TOP_Z


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_aframe_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.64, 0.66, 0.69, 1.0))
    cap_red = model.material("cap_red", rgba=(0.72, 0.16, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.11, 1.0))

    rail_size = (0.045, 0.030, FRONT_RAIL_TOP_Z)
    front_lean = asin(abs(FRONT_RAIL_BOTTOM_X) / FRONT_RAIL_TOP_Z)
    rear_lean_open = -asin(REAR_RAIL_BOTTOM_X_OPEN / FRONT_RAIL_TOP_Z)

    tread_levels = (0.26, 0.52, 0.78)

    brace_pivot_z = 0.49
    brace_pivot_x = _front_line_x(brace_pivot_z)
    receiver_local_z = -0.48
    receiver_local_x = _rear_line_x(receiver_local_z)
    receiver_contact_world_x = TOP_CAP_HINGE_X + receiver_local_x - 0.010
    receiver_contact_world_z = FRONT_RAIL_TOP_Z + receiver_local_z
    spreader_dx = receiver_contact_world_x - brace_pivot_x
    spreader_dz = receiver_contact_world_z - brace_pivot_z
    spreader_open_angle = atan2(spreader_dz, spreader_dx)
    spreader_reach = (spreader_dx**2 + spreader_dz**2) ** 0.5
    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.42, 0.44, 1.16)),
        mass=4.8,
        origin=Origin(xyz=(-0.04, 0.0, 0.58)),
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        front_frame.visual(
            Box(rail_size),
            origin=Origin(
                xyz=(FRONT_RAIL_BOTTOM_X * 0.5, side_sign * FRONT_RAIL_CENTER_Y, FRONT_RAIL_TOP_Z * 0.5),
                rpy=(0.0, front_lean, 0.0),
            ),
            material=aluminum,
            name=f"{side_name}_rail",
        )
        front_frame.visual(
            Box((0.072, 0.036, 0.020)),
            origin=Origin(xyz=(FRONT_RAIL_BOTTOM_X, side_sign * FRONT_RAIL_CENTER_Y, 0.010)),
            material=rubber,
            name=f"{side_name}_foot",
        )
        front_frame.visual(
            Box((0.018, 0.014, 0.045)),
            origin=Origin(xyz=(brace_pivot_x, side_sign * 0.222, brace_pivot_z)),
            material=zinc_steel,
            name=f"{side_name}_brace_mount",
        )

    for index, tread_z in enumerate(tread_levels, start=1):
        tread_x = _front_line_x(tread_z) + 0.018
        front_frame.visual(
            Box((0.090, 0.392, 0.025)),
            origin=Origin(xyz=(tread_x, 0.0, tread_z)),
            material=aluminum,
            name=f"tread_{index}",
        )
        front_frame.visual(
            Cylinder(radius=0.012, length=0.392),
            origin=Origin(
                xyz=(tread_x - 0.034, 0.0, tread_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=aluminum,
            name=f"tread_{index}_nose",
        )

    front_frame.visual(
        Box((0.060, 0.392, 0.032)),
        origin=Origin(xyz=(-0.030, 0.0, 0.93)),
        material=aluminum,
        name="top_cross_member",
    )
    front_frame.visual(
        Box((0.142, 0.404, 0.030)),
        origin=Origin(xyz=(-0.032, 0.0, 1.095)),
        material=cap_red,
        name="cap_base",
    )
    front_frame.visual(
        Box((0.072, 0.404, 0.018)),
        origin=Origin(xyz=(-0.060, 0.0, 1.071)),
        material=cap_red,
        name="cap_front_saddle",
    )
    front_frame.visual(
        Box((0.076, 0.344, 0.016)),
        origin=Origin(xyz=(-0.066, 0.0, 1.118)),
        material=cap_red,
        name="cap_tray_rim",
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        front_frame.visual(
            Box((0.022, 0.014, 0.046)),
            origin=Origin(xyz=(TOP_CAP_HINGE_X - 0.010, side_sign * 0.209, 1.100)),
            material=zinc_steel,
            name=f"{side_name}_hinge_cheek",
        )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.38, 0.44, 1.14)),
        mass=3.3,
        origin=Origin(xyz=(0.07, 0.0, -0.54)),
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        rear_frame.visual(
            Box(rail_size),
            origin=Origin(
                xyz=(
                    REAR_RAIL_TOP_X_OFFSET + REAR_RAIL_BOTTOM_X_OPEN * 0.5,
                    side_sign * REAR_RAIL_CENTER_Y,
                    -FRONT_RAIL_TOP_Z * 0.5,
                ),
                rpy=(0.0, rear_lean_open, 0.0),
            ),
            material=aluminum,
            name=f"{side_name}_rear_rail",
        )
        rear_frame.visual(
            Box((0.072, 0.036, 0.020)),
            origin=Origin(
                xyz=(REAR_RAIL_TOP_X_OFFSET + REAR_RAIL_BOTTOM_X_OPEN, side_sign * REAR_RAIL_CENTER_Y, -1.070)
            ),
            material=rubber,
            name=f"{side_name}_rear_foot",
        )
        rear_frame.visual(
            Box((0.020, 0.018, 0.030)),
            origin=Origin(xyz=(receiver_local_x, side_sign * RECEIVER_Y, receiver_local_z)),
            material=zinc_steel,
            name=f"{side_name}_receiver",
        )
        rear_frame.visual(
            Box((0.018, 0.014, 0.046)),
            origin=Origin(xyz=(0.000, side_sign * 0.223, -0.010)),
            material=zinc_steel,
            name=f"{side_name}_hinge_plate",
        )
        rear_frame.visual(
            Box((0.022, 0.054, 0.020)),
            origin=Origin(xyz=(0.014, side_sign * 0.242, -0.040)),
            material=aluminum,
            name=f"{side_name}_hinge_web",
        )
    rear_frame.visual(
        Box((0.034, 0.500, 0.024)),
        origin=Origin(xyz=(0.055, 0.0, -0.42)),
        material=aluminum,
        name="rear_cross_brace",
    )
    rear_frame.visual(
        Box((0.026, 0.500, 0.018)),
        origin=Origin(xyz=(0.028, 0.0, -0.19)),
        material=aluminum,
        name="upper_backer",
    )

    left_spreader = model.part("left_spreader")
    left_spreader.inertial = Inertial.from_geometry(
        Box((0.23, 0.02, 0.05)),
        mass=0.16,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )
    left_spreader.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
        name="pivot_washer",
    )
    left_spreader.visual(
        Box((0.018, 0.012, 0.014)),
        origin=Origin(xyz=(0.009, 0.006, 0.0)),
        material=zinc_steel,
        name="pivot_clevis",
    )
    left_spreader.visual(
        Box((0.120, 0.012, 0.060)),
        origin=Origin(xyz=(0.050, 0.006, 0.028)),
        material=zinc_steel,
        name="root_bridge",
    )
    left_spreader.visual(
        Box((spreader_reach, 0.014, 0.0035)),
        origin=Origin(
            xyz=(spreader_dx * 0.5, 0.006, spreader_dz * 0.5),
            rpy=(0.0, spreader_open_angle, 0.0),
        ),
        material=zinc_steel,
        name="brace_link",
    )

    right_spreader = model.part("right_spreader")
    right_spreader.inertial = Inertial.from_geometry(
        Box((0.23, 0.02, 0.05)),
        mass=0.16,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )
    right_spreader.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
        name="pivot_washer",
    )
    right_spreader.visual(
        Box((0.018, 0.012, 0.014)),
        origin=Origin(xyz=(0.009, -0.006, 0.0)),
        material=zinc_steel,
        name="pivot_clevis",
    )
    right_spreader.visual(
        Box((0.120, 0.012, 0.060)),
        origin=Origin(xyz=(0.050, -0.006, 0.028)),
        material=zinc_steel,
        name="root_bridge",
    )
    right_spreader.visual(
        Box((spreader_reach, 0.014, 0.0035)),
        origin=Origin(
            xyz=(spreader_dx * 0.5, -0.006, spreader_dz * 0.5),
            rpy=(0.0, spreader_open_angle, 0.0),
        ),
        material=zinc_steel,
        name="brace_link",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(TOP_CAP_HINGE_X, 0.0, FRONT_RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=0.34),
    )
    model.articulation(
        "left_spreader_fold",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_spreader,
        origin=Origin(xyz=(brace_pivot_x, BRACE_MOUNT_Y, brace_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-1.35, upper=0.02),
    )
    model.articulation(
        "right_spreader_fold",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_spreader,
        origin=Origin(xyz=(brace_pivot_x, -BRACE_MOUNT_Y, brace_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-1.35, upper=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    left_spreader = object_model.get_part("left_spreader")
    right_spreader = object_model.get_part("right_spreader")

    rear_hinge = object_model.get_articulation("rear_hinge")
    left_spreader_fold = object_model.get_articulation("left_spreader_fold")
    right_spreader_fold = object_model.get_articulation("right_spreader_fold")

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
        "all_required_subassemblies_present",
        all(
            part is not None
            for part in (front_frame, rear_frame, left_spreader, right_spreader)
        ),
        "Missing one or more required ladder subassemblies.",
    )
    ctx.check(
        "all_required_articulations_present",
        all(
            joint is not None
            for joint in (rear_hinge, left_spreader_fold, right_spreader_fold)
        ),
        "Missing one or more required ladder articulations.",
    )

    ctx.expect_gap(
        rear_frame,
        left_spreader,
        axis="x",
        positive_elem="left_receiver",
        negative_elem="brace_link",
        max_gap=0.004,
        max_penetration=0.001,
        name="left_spreader_reaches_open_stop",
    )
    ctx.expect_gap(
        rear_frame,
        right_spreader,
        axis="x",
        positive_elem="right_receiver",
        negative_elem="brace_link",
        max_gap=0.004,
        max_penetration=0.001,
        name="right_spreader_reaches_open_stop",
    )
    ctx.expect_overlap(
        left_spreader,
        rear_frame,
        axes="yz",
        elem_a="brace_link",
        elem_b="left_receiver",
        min_overlap=0.010,
        name="left_spreader_lines_up_with_receiver",
    )
    ctx.expect_overlap(
        right_spreader,
        rear_frame,
        axes="yz",
        elem_a="brace_link",
        elem_b="right_receiver",
        min_overlap=0.010,
        name="right_spreader_lines_up_with_receiver",
    )

    front_aabb = ctx.part_world_aabb(front_frame)
    rear_aabb = ctx.part_world_aabb(rear_frame)
    if front_aabb is not None and rear_aabb is not None:
        open_depth = max(front_aabb[1][0], rear_aabb[1][0]) - min(front_aabb[0][0], rear_aabb[0][0])
        ctx.check(
            "open_pose_feet_reach_ground",
            abs(front_aabb[0][2]) <= 0.012 and abs(rear_aabb[0][2]) <= 0.012,
            (
                f"Expected both leg pairs to reach the floor in the open pose; "
                f"front min z={front_aabb[0][2]:.4f}, rear min z={rear_aabb[0][2]:.4f}."
            ),
        )
        ctx.check(
            "open_pose_has_stable_base_depth",
            open_depth >= 0.30,
            f"Open base depth too shallow: {open_depth:.4f} m.",
        )

        with ctx.pose({rear_hinge: 0.12, left_spreader_fold: -1.20, right_spreader_fold: -1.20}):
            ctx.fail_if_parts_overlap_in_current_pose(name="closed_pose_no_overlaps")
            closed_front_aabb = ctx.part_world_aabb(front_frame)
            closed_rear_aabb = ctx.part_world_aabb(rear_frame)
            if closed_front_aabb is not None and closed_rear_aabb is not None:
                closed_depth = max(closed_front_aabb[1][0], closed_rear_aabb[1][0]) - min(
                    closed_front_aabb[0][0], closed_rear_aabb[0][0]
                )
                ctx.check(
                    "closed_pose_compacts_for_storage",
                    closed_depth <= open_depth * 0.98,
                    f"Closed depth {closed_depth:.4f} m is not substantially flatter than open depth {open_depth:.4f} m.",
                )
            ctx.expect_overlap(
                front_frame,
                rear_frame,
                axes="yz",
                min_overlap=0.24,
                name="closed_pose_frames_stack_in_plan",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
