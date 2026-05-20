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
    model = ArticulatedObject(name="lv_switchgear_panel")

    overall_width = 0.80
    overall_depth = 0.60
    overall_height = 2.10
    plinth_height = 0.10
    wall_thickness = 0.025
    roof_thickness = 0.040
    floor_thickness = 0.030
    door_thickness = 0.028
    door_width = 0.750
    door_height = 1.930
    hinge_radius = 0.014
    hinge_knuckle_length = 0.180
    hinge_z_offsets = (-0.620, 0.620)

    left_hinge_x = -(overall_width * 0.5 - wall_thickness * 0.5)
    right_hinge_x = overall_width * 0.5 - wall_thickness * 0.5
    front_face_y = overall_depth * 0.5
    rear_face_y = -overall_depth * 0.5
    door_center_z = plinth_height + door_height * 0.5
    front_latch_post_x = left_hinge_x + door_width + 0.010
    rear_latch_post_x = right_hinge_x - door_width - 0.010

    cabinet_steel = model.material("cabinet_steel", rgba=(0.835, 0.845, 0.855, 1.0))
    frame_charcoal = model.material("frame_charcoal", rgba=(0.240, 0.255, 0.275, 1.0))
    hardware_black = model.material("hardware_black", rgba=(0.120, 0.125, 0.135, 1.0))
    interior_gray = model.material("interior_gray", rgba=(0.520, 0.540, 0.560, 1.0))

    enclosure = model.part("enclosure")
    enclosure.visual(
        Box((0.760, 0.560, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=frame_charcoal,
        name="plinth",
    )
    enclosure.visual(
        Box((wall_thickness, overall_depth, overall_height - plinth_height - roof_thickness)),
        origin=Origin(
            xyz=(
                -(overall_width * 0.5 - wall_thickness * 0.5),
                0.0,
                plinth_height + (overall_height - plinth_height - roof_thickness) * 0.5,
            )
        ),
        material=cabinet_steel,
        name="left_side_wall",
    )
    enclosure.visual(
        Box((wall_thickness, overall_depth, overall_height - plinth_height - roof_thickness)),
        origin=Origin(
            xyz=(
                overall_width * 0.5 - wall_thickness * 0.5,
                0.0,
                plinth_height + (overall_height - plinth_height - roof_thickness) * 0.5,
            )
        ),
        material=cabinet_steel,
        name="right_side_wall",
    )
    enclosure.visual(
        Box((overall_width - 2.0 * wall_thickness, overall_depth, floor_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                plinth_height + floor_thickness * 0.5,
            )
        ),
        material=cabinet_steel,
        name="floor_pan",
    )
    enclosure.visual(
        Box((overall_width, overall_depth, roof_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                overall_height - roof_thickness * 0.5,
            )
        ),
        material=cabinet_steel,
        name="roof",
    )
    enclosure.visual(
        Box((0.020, 0.050, door_height)),
        origin=Origin(xyz=(front_latch_post_x, front_face_y - 0.025, door_center_z)),
        material=frame_charcoal,
        name="front_latch_post",
    )
    enclosure.visual(
        Box((0.020, 0.050, door_height)),
        origin=Origin(xyz=(rear_latch_post_x, rear_face_y + 0.025, door_center_z)),
        material=frame_charcoal,
        name="rear_latch_post",
    )
    for index, z_offset in enumerate(hinge_z_offsets):
        enclosure.visual(
            Box((0.022, 0.036, 0.220)),
            origin=Origin(
                xyz=(
                    left_hinge_x - 0.010,
                    front_face_y - 0.018,
                    door_center_z + z_offset,
                )
            ),
            material=frame_charcoal,
            name=f"front_hinge_leaf_{index}",
        )
        enclosure.visual(
            Box((0.022, 0.036, 0.220)),
            origin=Origin(
                xyz=(
                    right_hinge_x + 0.010,
                    rear_face_y + 0.018,
                    door_center_z + z_offset,
                )
            ),
            material=frame_charcoal,
            name=f"rear_hinge_leaf_{index}",
        )

    enclosure.visual(
        Box((0.340, 0.180, 1.550)),
        origin=Origin(xyz=(0.0, -0.020, plinth_height + floor_thickness + 1.550 * 0.5)),
        material=interior_gray,
        name="switchgear_stack",
    )
    enclosure.visual(
        Box((0.030, 0.080, 1.650)),
        origin=Origin(xyz=(-0.210, 0.0, plinth_height + floor_thickness + 1.650 * 0.5)),
        material=frame_charcoal,
        name="left_mount_rail",
    )
    enclosure.visual(
        Box((0.030, 0.080, 1.650)),
        origin=Origin(xyz=(0.210, 0.0, plinth_height + floor_thickness + 1.650 * 0.5)),
        material=frame_charcoal,
        name="right_mount_rail",
    )
    enclosure.inertial = Inertial.from_geometry(
        Box((overall_width, overall_depth, overall_height)),
        mass=185.0,
        origin=Origin(xyz=(0.0, 0.0, overall_height * 0.5)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width * 0.5, door_thickness * 0.5, 0.0)),
        material=cabinet_steel,
        name="front_door_panel",
    )
    front_door.visual(
        Box((0.030, door_thickness + 0.006, door_height)),
        origin=Origin(xyz=(0.015, door_thickness * 0.5 + 0.003, 0.0)),
        material=cabinet_steel,
        name="front_hinge_stile",
    )
    front_door.visual(
        Box((0.026, door_thickness + 0.006, door_height)),
        origin=Origin(xyz=(door_width - 0.013, door_thickness * 0.5 + 0.003, 0.0)),
        material=cabinet_steel,
        name="front_latch_stile",
    )
    for index, z_offset in enumerate(hinge_z_offsets):
        front_door.visual(
            Cylinder(radius=hinge_radius, length=hinge_knuckle_length),
            origin=Origin(xyz=(0.0, hinge_radius, z_offset)),
            material=hardware_black,
            name=f"front_hinge_knuckle_{index}",
        )
        front_door.visual(
            Box((0.018, 0.022, 0.210)),
            origin=Origin(xyz=(0.009, 0.011, z_offset)),
            material=hardware_black,
            name=f"front_knuckle_leaf_{index}",
        )
    for guide_index, guide_z in enumerate((-0.560, 0.560)):
        guide_offset_x = 0.038
        front_door.visual(
            Box((0.012, 0.012, 0.120)),
            origin=Origin(
                xyz=(door_width * 0.5 - guide_offset_x, door_thickness + 0.006, guide_z)
            ),
            material=frame_charcoal,
            name=f"front_lock_guide_left_{guide_index}",
        )
        front_door.visual(
            Box((0.012, 0.012, 0.120)),
            origin=Origin(
                xyz=(door_width * 0.5 + guide_offset_x, door_thickness + 0.006, guide_z)
            ),
            material=frame_charcoal,
            name=f"front_lock_guide_right_{guide_index}",
        )
    front_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=26.0,
        origin=Origin(xyz=(door_width * 0.5, door_thickness * 0.5, 0.0)),
    )

    rear_door = model.part("rear_door")
    rear_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(-door_width * 0.5, -door_thickness * 0.5, 0.0)),
        material=cabinet_steel,
        name="rear_door_panel",
    )
    rear_door.visual(
        Box((0.030, door_thickness + 0.006, door_height)),
        origin=Origin(xyz=(-0.015, -(door_thickness * 0.5 + 0.003), 0.0)),
        material=cabinet_steel,
        name="rear_hinge_stile",
    )
    rear_door.visual(
        Box((0.026, door_thickness + 0.006, door_height)),
        origin=Origin(xyz=(-(door_width - 0.013), -(door_thickness * 0.5 + 0.003), 0.0)),
        material=cabinet_steel,
        name="rear_latch_stile",
    )
    for index, z_offset in enumerate(hinge_z_offsets):
        rear_door.visual(
            Cylinder(radius=hinge_radius, length=hinge_knuckle_length),
            origin=Origin(xyz=(0.0, -hinge_radius, z_offset)),
            material=hardware_black,
            name=f"rear_hinge_knuckle_{index}",
        )
        rear_door.visual(
            Box((0.018, 0.022, 0.210)),
            origin=Origin(xyz=(-0.009, -0.011, z_offset)),
            material=hardware_black,
            name=f"rear_knuckle_leaf_{index}",
        )
    rear_door.visual(
        Box((0.090, 0.018, 0.220)),
        origin=Origin(xyz=(-door_width * 0.5, -(door_thickness + 0.009), 0.0)),
        material=hardware_black,
        name="rear_pull_handle",
    )
    rear_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=24.0,
        origin=Origin(xyz=(-door_width * 0.5, -door_thickness * 0.5, 0.0)),
    )

    locking_bar = model.part("front_locking_bar")
    locking_bar.visual(
        Box((0.050, 0.008, 1.400)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=hardware_black,
        name="locking_bar_spine",
    )
    locking_bar.visual(
        Box((0.018, 0.008, 0.260)),
        origin=Origin(xyz=(0.0, 0.004, 0.730)),
        material=hardware_black,
        name="locking_bar_upper_rod",
    )
    locking_bar.visual(
        Box((0.018, 0.008, 0.260)),
        origin=Origin(xyz=(0.0, 0.004, -0.730)),
        material=hardware_black,
        name="locking_bar_lower_rod",
    )
    for index, latch_z in enumerate((0.730, 0.0, -0.730)):
        locking_bar.visual(
            Box((0.160, 0.010, 0.018)),
            origin=Origin(xyz=(0.055, 0.009, latch_z)),
            material=hardware_black,
            name=f"locking_latch_bridge_{index}",
        )
        locking_bar.visual(
            Box((0.160, 0.014, 0.014)),
            origin=Origin(xyz=(0.095, 0.011, latch_z)),
            material=hardware_black,
            name=f"locking_latch_tab_{index}",
        )
    locking_bar.visual(
        Box((0.100, 0.030, 0.200)),
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material=hardware_black,
        name="locking_handle_body",
    )
    locking_bar.visual(
        Box((0.038, 0.052, 0.140)),
        origin=Origin(xyz=(-0.008, 0.046, 0.0)),
        material=hardware_black,
        name="locking_handle_grip",
    )
    locking_bar.inertial = Inertial.from_geometry(
        Box((0.200, 0.052, 1.700)),
        mass=3.5,
        origin=Origin(xyz=(0.020, 0.026, 0.0)),
    )

    model.articulation(
        "front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=front_door,
        origin=Origin(xyz=(left_hinge_x, front_face_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )
    model.articulation(
        "rear_access_door_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=rear_door,
        origin=Origin(xyz=(right_hinge_x, rear_face_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=110.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )
    model.articulation(
        "front_locking_bar_slide",
        ArticulationType.PRISMATIC,
        parent=front_door,
        child=locking_bar,
        origin=Origin(xyz=(door_width * 0.5, door_thickness, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=-0.040,
            upper=0.040,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    front_door = object_model.get_part("front_door")
    rear_door = object_model.get_part("rear_door")
    locking_bar = object_model.get_part("front_locking_bar")
    front_hinge = object_model.get_articulation("front_door_hinge")
    rear_hinge = object_model.get_articulation("rear_access_door_hinge")
    lock_slide = object_model.get_articulation("front_locking_bar_slide")

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
        "front hinge axis is vertical",
        front_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={front_hinge.axis}",
    )
    ctx.check(
        "rear hinge axis is vertical",
        rear_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={rear_hinge.axis}",
    )
    ctx.check(
        "locking bar slides vertically",
        lock_slide.axis == (0.0, 0.0, 1.0),
        details=f"axis={lock_slide.axis}",
    )

    with ctx.pose({front_hinge: 0.0, rear_hinge: 0.0, lock_slide: 0.0}):
        ctx.expect_contact(
            front_door,
            enclosure,
            name="front door remains mounted to enclosure",
        )
        ctx.expect_contact(
            rear_door,
            enclosure,
            name="rear door remains mounted to enclosure",
        )
        ctx.expect_contact(
            locking_bar,
            front_door,
            name="locking bar remains mounted to front door",
        )
        ctx.expect_gap(
            front_door,
            enclosure,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="front door seats flush on front enclosure plane",
        )
        ctx.expect_gap(
            enclosure,
            rear_door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="rear door seats flush on rear enclosure plane",
        )
        ctx.expect_overlap(
            front_door,
            enclosure,
            axes="xz",
            min_overlap=0.70,
            name="front door covers the front opening",
        )
        ctx.expect_overlap(
            rear_door,
            enclosure,
            axes="xz",
            min_overlap=0.70,
            name="rear door covers the rear opening",
        )
        ctx.expect_within(
            locking_bar,
            front_door,
            axes="xz",
            margin=0.0,
            name="locking bar stays within the front door outline",
        )

    front_closed_aabb = ctx.part_world_aabb(front_door)
    with ctx.pose({front_hinge: front_hinge.motion_limits.upper or math.radians(90.0)}):
        front_open_aabb = ctx.part_world_aabb(front_door)
    ctx.check(
        "front door opens outward from the enclosure front",
        front_closed_aabb is not None
        and front_open_aabb is not None
        and front_open_aabb[1][1] > front_closed_aabb[1][1] + 0.20,
        details=f"closed={front_closed_aabb}, open={front_open_aabb}",
    )

    rear_closed_aabb = ctx.part_world_aabb(rear_door)
    with ctx.pose({rear_hinge: rear_hinge.motion_limits.upper or math.radians(90.0)}):
        rear_open_aabb = ctx.part_world_aabb(rear_door)
    ctx.check(
        "rear access door opens outward from the enclosure rear",
        rear_closed_aabb is not None
        and rear_open_aabb is not None
        and rear_open_aabb[0][1] < rear_closed_aabb[0][1] - 0.20,
        details=f"closed={rear_closed_aabb}, open={rear_open_aabb}",
    )

    lock_lower = lock_slide.motion_limits.lower if lock_slide.motion_limits is not None else None
    lock_upper = lock_slide.motion_limits.upper if lock_slide.motion_limits is not None else None
    with ctx.pose({lock_slide: lock_lower or 0.0}):
        lock_low_aabb = ctx.part_world_aabb(locking_bar)
    with ctx.pose({lock_slide: lock_upper or 0.0}):
        lock_high_aabb = ctx.part_world_aabb(locking_bar)
    ctx.check(
        "locking bar travel raises the three-point bar upward",
        lock_low_aabb is not None
        and lock_high_aabb is not None
        and lock_high_aabb[1][2] > lock_low_aabb[1][2] + 0.05,
        details=f"low={lock_low_aabb}, high={lock_high_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
