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
    model = ArticulatedObject(name="wall_mounted_mailbox")

    body_color = model.material("body_powder_coat", rgba=(0.24, 0.27, 0.29, 1.0))
    flap_color = model.material("flap_black", rgba=(0.11, 0.12, 0.13, 1.0))
    hardware_color = model.material("hardware_silver", rgba=(0.69, 0.72, 0.75, 1.0))

    body_width = 0.320
    body_depth = 0.140
    body_height = 0.420
    shell_thickness = 0.004

    door_width = 0.304
    door_height = 0.404
    door_thickness = 0.004
    door_front_y = body_depth + (door_thickness * 0.5)
    door_hinge_z = shell_thickness

    slot_width = 0.252
    slot_height = 0.018
    slot_bottom_z = 0.308
    slot_top_z = slot_bottom_z + slot_height
    slot_side_rail_width = (door_width - slot_width) * 0.5

    flap_width = 0.264
    flap_height = 0.042
    flap_thickness = 0.0025
    flap_hinge_z = slot_top_z + 0.008

    hood_width = 0.272
    hood_depth = 0.036
    hood_thickness = 0.003
    hood_pitch = -0.33

    housing = model.part("housing")
    housing.visual(
        Box((body_width, shell_thickness, body_height)),
        origin=Origin(xyz=(0.0, shell_thickness * 0.5, body_height * 0.5)),
        material=body_color,
        name="back_panel",
    )
    housing.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(
            xyz=(
                -(body_width * 0.5) + (shell_thickness * 0.5),
                body_depth * 0.5,
                body_height * 0.5,
            )
        ),
        material=body_color,
        name="left_side",
    )
    housing.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(
            xyz=(
                (body_width * 0.5) - (shell_thickness * 0.5),
                body_depth * 0.5,
                body_height * 0.5,
            )
        ),
        material=body_color,
        name="right_side",
    )
    housing.visual(
        Box((body_width - (2.0 * shell_thickness), body_depth, shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth * 0.5,
                body_height - (shell_thickness * 0.5),
            )
        ),
        material=body_color,
        name="top_panel",
    )
    housing.visual(
        Box((body_width - (2.0 * shell_thickness), body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, body_depth * 0.5, shell_thickness * 0.5)),
        material=body_color,
        name="bottom_panel",
    )
    housing.visual(
        Box((0.250, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.007, body_height - 0.028)),
        material=hardware_color,
        name="wall_mount_rail",
    )
    housing.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=3.8,
        origin=Origin(xyz=(0.0, body_depth * 0.5, body_height * 0.5)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        Box((door_width, door_thickness, slot_bottom_z)),
        origin=Origin(xyz=(0.0, 0.0, slot_bottom_z * 0.5)),
        material=body_color,
        name="door_bottom_panel",
    )
    front_door.visual(
        Box((slot_side_rail_width, door_thickness, slot_height)),
        origin=Origin(
            xyz=(
                -(door_width * 0.5) + (slot_side_rail_width * 0.5),
                0.0,
                slot_bottom_z + (slot_height * 0.5),
            )
        ),
        material=body_color,
        name="door_left_slot_rail",
    )
    front_door.visual(
        Box((slot_side_rail_width, door_thickness, slot_height)),
        origin=Origin(
            xyz=(
                (door_width * 0.5) - (slot_side_rail_width * 0.5),
                0.0,
                slot_bottom_z + (slot_height * 0.5),
            )
        ),
        material=body_color,
        name="door_right_slot_rail",
    )
    front_door.visual(
        Box((door_width, door_thickness, door_height - slot_top_z)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                slot_top_z + ((door_height - slot_top_z) * 0.5),
            )
        ),
        material=body_color,
        name="door_top_panel",
    )
    front_door.visual(
        Box((hood_width, hood_depth, hood_thickness)),
        origin=Origin(
            xyz=(0.0, 0.024, slot_top_z + 0.021),
            rpy=(hood_pitch, 0.0, 0.0),
        ),
        material=body_color,
        name="rain_hood",
    )
    front_door.visual(
        Box((0.004, 0.018, 0.018)),
        origin=Origin(
            xyz=(
                -(hood_width * 0.5) + 0.002,
                0.010,
                slot_top_z + 0.013,
            )
        ),
        material=body_color,
        name="hood_left_cheek",
    )
    front_door.visual(
        Box((0.004, 0.018, 0.018)),
        origin=Origin(
            xyz=(
                (hood_width * 0.5) - 0.002,
                0.010,
                slot_top_z + 0.013,
            )
        ),
        material=body_color,
        name="hood_right_cheek",
    )
    front_door.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.008, 0.132),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=hardware_color,
        name="door_pull",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.020, door_height)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, door_height * 0.5)),
    )

    slot_flap = model.part("slot_flap")
    slot_flap.visual(
        Box((flap_width, flap_thickness, flap_height)),
        origin=Origin(xyz=(0.0, 0.0, -(flap_height * 0.5))),
        material=flap_color,
        name="flap_panel",
    )
    slot_flap.visual(
        Box((flap_width * 0.92, 0.006, 0.006)),
        origin=Origin(
            xyz=(
                0.0,
                0.004,
                -flap_height + 0.003,
            )
        ),
        material=flap_color,
        name="flap_pull_lip",
    )
    slot_flap.inertial = Inertial.from_geometry(
        Box((flap_width, 0.008, flap_height)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -(flap_height * 0.5))),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_door,
        origin=Origin(xyz=(0.0, door_front_y, door_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=front_door,
        child=slot_flap,
        origin=Origin(
            xyz=(
                0.0,
                (door_thickness * 0.5) + (flap_thickness * 0.5),
                flap_hinge_z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.20,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    front_door = object_model.get_part("front_door")
    slot_flap = object_model.get_part("slot_flap")
    door_hinge = object_model.get_articulation("door_hinge")
    flap_hinge = object_model.get_articulation("flap_hinge")

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
        "core_parts_and_joints_present",
        {part.name for part in object_model.parts} == {"housing", "front_door", "slot_flap"}
        and {joint.name for joint in object_model.articulations} == {"door_hinge", "flap_hinge"},
        "Expected housing, front_door, slot_flap, door_hinge, and flap_hinge.",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    housing_ok = False
    if housing_aabb is not None:
        housing_dx = housing_aabb[1][0] - housing_aabb[0][0]
        housing_dy = housing_aabb[1][1] - housing_aabb[0][1]
        housing_dz = housing_aabb[1][2] - housing_aabb[0][2]
        housing_ok = (
            0.28 <= housing_dx <= 0.36
            and 0.12 <= housing_dy <= 0.16
            and 0.38 <= housing_dz <= 0.45
        )
    ctx.check(
        "mailbox_body_scale_is_realistic",
        housing_ok,
        "Expected a wall mailbox roughly 0.32 m wide, 0.14 m deep, and 0.42 m tall.",
    )

    ctx.check(
        "hinge_axes_and_limits_are_correct",
        door_hinge.axis == (-1.0, 0.0, 0.0)
        and flap_hinge.axis == (1.0, 0.0, 0.0)
        and door_hinge.motion_limits is not None
        and flap_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and 1.30 <= door_hinge.motion_limits.upper <= 1.55
        and flap_hinge.motion_limits.lower == 0.0
        and 1.00 <= flap_hinge.motion_limits.upper <= 1.30,
        "Door should bottom-hinge outward; flap should top-hinge outward above the slot.",
    )

    with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_contact(
            front_door,
            housing,
            contact_tol=1e-4,
            name="front_door_contacts_housing_when_closed",
        )
        ctx.expect_overlap(
            front_door,
            housing,
            axes="xz",
            min_overlap=0.25,
            name="front_door_covers_mailbox_opening",
        )
        ctx.expect_contact(
            slot_flap,
            front_door,
            contact_tol=1e-4,
            name="slot_flap_seats_on_front_door",
        )
        ctx.expect_overlap(
            slot_flap,
            front_door,
            axes="xz",
            min_overlap=0.018,
            elem_a="flap_panel",
            name="slot_flap_covers_letter_slot_zone",
        )

    door_closed_aabb = ctx.part_world_aabb(front_door)
    door_motion_ok = False
    with ctx.pose({door_hinge: 1.15, flap_hinge: 0.0}):
        door_open_aabb = ctx.part_world_aabb(front_door)
        if door_closed_aabb is not None and door_open_aabb is not None:
            door_motion_ok = (
                door_open_aabb[1][1] > door_closed_aabb[1][1] + 0.12
                and door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.18
            )
    ctx.check(
        "front_door_opens_outward_and_downward",
        door_motion_ok,
        "Opening the lower hinge should swing the door forward and down.",
    )

    flap_closed_aabb = ctx.part_world_aabb(slot_flap)
    flap_motion_ok = False
    with ctx.pose({door_hinge: 0.0, flap_hinge: 1.0}):
        flap_open_aabb = ctx.part_world_aabb(slot_flap)
        if flap_closed_aabb is not None and flap_open_aabb is not None:
            flap_motion_ok = (
                flap_open_aabb[1][1] > flap_closed_aabb[1][1] + 0.02
                and flap_open_aabb[0][2] > flap_closed_aabb[0][2] + 0.008
            )
    ctx.check(
        "slot_flap_lifts_outward_from_slot",
        flap_motion_ok,
        "Opening the upper hinge should lift the slot flap outward from the door face.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
