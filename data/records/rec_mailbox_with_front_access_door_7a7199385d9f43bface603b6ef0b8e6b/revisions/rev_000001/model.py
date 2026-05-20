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
    model = ArticulatedObject(name="wall_mounted_mailbox")

    body_paint = model.material("body_paint", rgba=(0.23, 0.29, 0.24, 1.0))
    shadow_paint = model.material("shadow_paint", rgba=(0.16, 0.18, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))

    plate_width = 0.34
    plate_height = 0.46
    plate_thickness = 0.004

    body_width = 0.30
    body_height = 0.40
    body_depth = 0.12
    wall_thickness = 0.004

    door_width = 0.304
    door_height = 0.396
    door_thickness = 0.006

    lock_x = door_width * 0.5 - 0.045
    lock_z = door_height - 0.063
    lock_opening_half = 0.012
    spindle_radius = 0.007
    front_bezel_radius = 0.016
    rear_boss_radius = 0.013
    bezel_length = 0.002
    rear_boss_length = 0.002
    cam_hub_radius = 0.007
    cam_hub_length = 0.004
    cam_length = 0.038
    cam_height = 0.008
    cam_thickness = 0.003

    back_plate = model.part("back_plate")
    back_plate.visual(
        Box((plate_width, plate_thickness, plate_height)),
        origin=Origin(xyz=(0.0, plate_thickness * 0.5, plate_height * 0.5)),
        material=body_paint,
        name="back_plate_panel",
    )
    for x_sign in (-1.0, 1.0):
        for z_value in (0.11, 0.35):
            back_plate.visual(
                Cylinder(radius=0.007, length=0.0015),
                origin=Origin(
                    xyz=(x_sign * 0.095, plate_thickness + 0.00075, z_value),
                    rpy=(pi * 0.5, 0.0, 0.0),
                ),
                material=steel,
                name=f"mount_cap_{int((x_sign + 1.0) * 0.5)}_{int(z_value * 1000)}",
            )

    housing = model.part("housing")
    housing.visual(
        Box((body_width, body_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, body_depth * 0.5, wall_thickness * 0.5)),
        material=shadow_paint,
        name="bottom_panel",
    )
    housing.visual(
        Box((body_width, body_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, body_depth * 0.5, body_height - wall_thickness * 0.5)),
        material=body_paint,
        name="top_panel",
    )
    housing.visual(
        Box((wall_thickness, body_depth, body_height - 2.0 * wall_thickness)),
        origin=Origin(
            xyz=(-body_width * 0.5 + wall_thickness * 0.5, body_depth * 0.5, body_height * 0.5),
        ),
        material=body_paint,
        name="left_panel",
    )
    housing.visual(
        Box((wall_thickness, body_depth, body_height - 2.0 * wall_thickness)),
        origin=Origin(
            xyz=(body_width * 0.5 - wall_thickness * 0.5, body_depth * 0.5, body_height * 0.5),
        ),
        material=body_paint,
        name="right_panel",
    )
    housing.visual(
        Box((body_width * 0.82, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, body_depth - 0.003, body_height + 0.002)),
        material=shadow_paint,
        name="lip_support",
    )
    housing.visual(
        Box((body_width * 0.82, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, body_depth + 0.005, body_height + 0.008)),
        material=shadow_paint,
        name="rain_lip",
    )

    door = model.part("door")
    lower_height = lock_z - lock_opening_half
    upper_height = door_height - (lock_z + lock_opening_half)
    middle_height = lock_opening_half * 2.0
    left_width = (lock_x - lock_opening_half) - (-door_width * 0.5)
    right_width = (door_width * 0.5) - (lock_x + lock_opening_half)
    door.visual(
        Box((door_width, door_thickness, lower_height)),
        origin=Origin(xyz=(0.0, 0.0, lower_height * 0.5)),
        material=body_paint,
        name="door_lower_panel",
    )
    door.visual(
        Box((door_width, door_thickness, upper_height)),
        origin=Origin(xyz=(0.0, 0.0, lock_z + lock_opening_half + upper_height * 0.5)),
        material=body_paint,
        name="door_upper_panel",
    )
    door.visual(
        Box((left_width, door_thickness, middle_height)),
        origin=Origin(xyz=(-door_width * 0.5 + left_width * 0.5, 0.0, lock_z)),
        material=body_paint,
        name="door_left_panel",
    )
    door.visual(
        Box((right_width, door_thickness, middle_height)),
        origin=Origin(xyz=(lock_x + lock_opening_half + right_width * 0.5, 0.0, lock_z)),
        material=body_paint,
        name="door_right_panel",
    )

    lock_body = model.part("lock_body")
    lock_body.visual(
        Cylinder(radius=spindle_radius, length=door_thickness + bezel_length + rear_boss_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="lock_sleeve",
    )
    lock_body.visual(
        Cylinder(radius=front_bezel_radius, length=bezel_length),
        origin=Origin(
            xyz=(0.0, door_thickness * 0.5 + bezel_length * 0.5, 0.0),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=steel,
        name="front_bezel",
    )
    lock_body.visual(
        Cylinder(radius=rear_boss_radius, length=rear_boss_length),
        origin=Origin(
            xyz=(0.0, -(door_thickness * 0.5 + rear_boss_length * 0.5), 0.0),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=steel,
        name="rear_boss",
    )

    lock_cam = model.part("lock_cam")
    lock_cam.visual(
        Cylinder(radius=cam_hub_radius, length=cam_hub_length),
        origin=Origin(
            xyz=(0.0, -(door_thickness * 0.5 + rear_boss_length + cam_hub_length * 0.5), 0.0),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=steel,
        name="cam_hub",
    )
    lock_cam.visual(
        Box((cam_length, cam_thickness, cam_height)),
        origin=Origin(
            xyz=(-(cam_length * 0.5) + cam_hub_radius, -(door_thickness * 0.5 + rear_boss_length + cam_hub_length + cam_thickness * 0.5), 0.0),
        ),
        material=shadow_paint,
        name="cam_bar",
    )

    model.articulation(
        "plate_to_housing",
        ArticulationType.FIXED,
        parent=back_plate,
        child=housing,
        origin=Origin(xyz=(0.0, plate_thickness, 0.0)),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, body_depth + door_thickness * 0.5, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.45,
            upper=0.0,
        ),
    )
    model.articulation(
        "door_to_lock_body",
        ArticulationType.FIXED,
        parent=door,
        child=lock_body,
        origin=Origin(xyz=(lock_x, 0.0, lock_z)),
    )
    model.articulation(
        "door_to_lock_cam",
        ArticulationType.REVOLUTE,
        parent=lock_body,
        child=lock_cam,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-0.85,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_plate = object_model.get_part("back_plate")
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    lock_body = object_model.get_part("lock_body")
    lock_cam = object_model.get_part("lock_cam")

    door_hinge = object_model.get_articulation("door_hinge")
    door_to_lock_cam = object_model.get_articulation("door_to_lock_cam")

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
        "door_hinge_axis",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected x-axis hinge, got {door_hinge.axis}",
    )
    ctx.check(
        "lock_cam_axis",
        tuple(door_to_lock_cam.axis) == (0.0, 1.0, 0.0),
        details=f"expected y-axis lock axis, got {door_to_lock_cam.axis}",
    )

    ctx.expect_contact(housing, back_plate, name="housing_contacts_back_plate")
    ctx.expect_within(housing, back_plate, axes="xz", margin=0.0, name="housing_within_back_plate")

    with ctx.pose({door_hinge: 0.0, door_to_lock_cam: 0.0}):
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            negative_elem="bottom_panel",
            min_gap=0.0,
            max_gap=0.0005,
            name="door_closed_flush_to_bottom_frame",
        )
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            negative_elem="top_panel",
            min_gap=0.0,
            max_gap=0.0005,
            name="door_closed_flush_to_top_frame",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="xz",
            min_overlap=0.28,
            name="door_covers_housing_opening",
        )
        ctx.expect_contact(lock_body, door, name="lock_body_contacts_door")
        ctx.expect_contact(lock_cam, lock_body, name="lock_cam_contacts_lock_body")
        ctx.expect_within(lock_body, door, axes="xz", margin=0.0, name="lock_body_stays_on_door_face")

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(door)
        assert closed_aabb is not None
        with ctx.pose({door_hinge: door_limits.lower}):
            open_aabb = ctx.part_world_aabb(door)
            assert open_aabb is not None
            ctx.check(
                "door_swings_forward_when_open",
                open_aabb[1][1] > closed_aabb[1][1] + 0.20,
                details=f"closed={closed_aabb}, open={open_aabb}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="door_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="door_lower_pose_no_floating")
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="door_upper_pose_no_floating")

    lock_limits = door_to_lock_cam.motion_limits
    if lock_limits is not None and lock_limits.lower is not None and lock_limits.upper is not None:
        with ctx.pose({door_to_lock_cam: lock_limits.lower}):
            ctx.expect_contact(lock_body, door, name="lock_body_lower_pose_contact")
            ctx.expect_contact(lock_cam, lock_body, name="lock_lower_pose_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="lock_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="lock_lower_pose_no_floating")
        with ctx.pose({door_to_lock_cam: lock_limits.upper}):
            ctx.expect_contact(lock_body, door, name="lock_body_upper_pose_contact")
            ctx.expect_contact(lock_cam, lock_body, name="lock_upper_pose_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="lock_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="lock_upper_pose_no_floating")

    if lock_limits is not None and lock_limits.upper is not None:
        with ctx.pose({door_hinge: -1.1, door_to_lock_cam: lock_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="combined_pose_no_floating")

    ctx.fail_if_articulation_overlaps(max_pose_samples=16, name="articulation_pose_sweep_clear")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
