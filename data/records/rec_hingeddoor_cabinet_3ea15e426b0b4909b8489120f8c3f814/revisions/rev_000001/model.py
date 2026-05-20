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
    model = ArticulatedObject(name="utility_cabinet")

    outer_w = 0.78
    outer_d = 0.48
    body_h = 1.80
    panel_t = 0.018
    feet_h = 0.06
    foot_size = 0.055

    door_t = 0.022
    door_gap = 0.004
    door_w = outer_w - (2.0 * panel_t) - (2.0 * door_gap)
    door_h = body_h - (2.0 * panel_t) - (2.0 * door_gap)

    hinge_r = 0.008
    carcass_knuckle_len = 0.050
    door_knuckle_len = 0.030
    knuckle_offset = 0.5 * (carcass_knuckle_len + door_knuckle_len)

    hinge_axis_x = -(outer_w * 0.5 - panel_t)
    hinge_axis_y = outer_d * 0.5 + hinge_r
    door_axis_z = feet_h + panel_t + door_gap + (door_h * 0.5)
    handle_z = 0.060
    hinge_centers = (0.62, 0.0, -0.62)

    model.material("cabinet_paint", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("door_paint", rgba=(0.86, 0.87, 0.88, 1.0))
    model.material("hinge_metal", rgba=(0.57, 0.59, 0.62, 1.0))
    model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("foot_black", rgba=(0.12, 0.12, 0.13, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((panel_t, outer_d, body_h)),
        origin=Origin(xyz=(-(outer_w * 0.5) + (panel_t * 0.5), 0.0, feet_h + (body_h * 0.5))),
        material="cabinet_paint",
        name="left_side_panel",
    )
    carcass.visual(
        Box((panel_t, outer_d, body_h)),
        origin=Origin(xyz=((outer_w * 0.5) - (panel_t * 0.5), 0.0, feet_h + (body_h * 0.5))),
        material="cabinet_paint",
        name="right_side_panel",
    )
    carcass.visual(
        Box((outer_w - (2.0 * panel_t), outer_d - panel_t, panel_t)),
        origin=Origin(xyz=(0.0, panel_t * 0.5, feet_h + (panel_t * 0.5))),
        material="cabinet_paint",
        name="bottom_panel",
    )
    carcass.visual(
        Box((outer_w - (2.0 * panel_t), outer_d - panel_t, panel_t)),
        origin=Origin(xyz=(0.0, panel_t * 0.5, feet_h + body_h - (panel_t * 0.5))),
        material="cabinet_paint",
        name="top_panel",
    )
    carcass.visual(
        Box((outer_w - (2.0 * panel_t), panel_t, body_h - (2.0 * panel_t))),
        origin=Origin(xyz=(0.0, -(outer_d * 0.5) + (panel_t * 0.5), feet_h + (body_h * 0.5))),
        material="cabinet_paint",
        name="back_panel",
    )

    foot_xy = (
        (-(outer_w * 0.5) + 0.085, -(outer_d * 0.5) + 0.085),
        ((outer_w * 0.5) - 0.085, -(outer_d * 0.5) + 0.085),
        (-(outer_w * 0.5) + 0.085, (outer_d * 0.5) - 0.085),
        ((outer_w * 0.5) - 0.085, (outer_d * 0.5) - 0.085),
    )
    for index, (fx, fy) in enumerate(foot_xy):
        carcass.visual(
            Box((foot_size, foot_size, feet_h)),
            origin=Origin(xyz=(fx, fy, feet_h * 0.5)),
            material="foot_black",
            name=f"foot_{index}",
        )

    for hinge_index, hinge_z in enumerate(hinge_centers):
        carcass.visual(
            Cylinder(radius=hinge_r, length=carcass_knuckle_len),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, door_axis_z + hinge_z)),
            material="hinge_metal",
            name=f"hinge_{hinge_index}_carcass_knuckle",
        )
        carcass.visual(
            Box((0.012, hinge_r, carcass_knuckle_len)),
            origin=Origin(
                xyz=(
                    hinge_axis_x - 0.006,
                    (outer_d * 0.5) + (hinge_r * 0.5),
                    door_axis_z + hinge_z,
                )
            ),
            material="hinge_metal",
            name=f"hinge_{hinge_index}_carcass_leaf",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h + feet_h)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, (body_h + feet_h) * 0.5)),
    )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(
            xyz=(
                (door_w * 0.5) + door_gap,
                -(hinge_r + (door_t * 0.5)),
                0.0,
            )
        ),
        material="door_paint",
        name="door_panel",
    )
    door.visual(
        Box((door_w - 0.090, 0.002, 0.050)),
        origin=Origin(
            xyz=((door_w * 0.5) + door_gap, -hinge_r + 0.001, (door_h * 0.5) - 0.080)
        ),
        material="door_paint",
        name="front_top_stiffener",
    )
    door.visual(
        Box((door_w - 0.090, 0.002, 0.050)),
        origin=Origin(
            xyz=((door_w * 0.5) + door_gap, -hinge_r + 0.001, -(door_h * 0.5) + 0.080)
        ),
        material="door_paint",
        name="front_bottom_stiffener",
    )
    door.visual(
        Box((0.050, 0.002, door_h - 0.180)),
        origin=Origin(xyz=(0.060, -hinge_r + 0.001, 0.0)),
        material="door_paint",
        name="front_left_stiffener",
    )
    door.visual(
        Box((0.050, 0.002, 0.60)),
        origin=Origin(xyz=(door_w - 0.052, -hinge_r + 0.001, 0.49)),
        material="door_paint",
        name="front_right_upper_stiffener",
    )
    door.visual(
        Box((0.050, 0.002, 0.60)),
        origin=Origin(xyz=(door_w - 0.052, -hinge_r + 0.001, -0.49)),
        material="door_paint",
        name="front_right_lower_stiffener",
    )

    for hinge_index, hinge_z in enumerate(hinge_centers):
        for suffix, z_sign in (("upper", 1.0), ("lower", -1.0)):
            door.visual(
                Cylinder(radius=hinge_r, length=door_knuckle_len),
                origin=Origin(xyz=(0.0, 0.0, hinge_z + (z_sign * knuckle_offset))),
                material="hinge_metal",
                name=f"hinge_{hinge_index}_{suffix}_knuckle",
            )
            door.visual(
                Box((0.024, hinge_r, door_knuckle_len)),
                origin=Origin(
                    xyz=(0.012, -(hinge_r * 0.5), hinge_z + (z_sign * knuckle_offset))
                ),
                material="hinge_metal",
                name=f"hinge_{hinge_index}_{suffix}_leaf",
            )

    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t + (2.0 * hinge_r), door_h)),
        mass=13.0,
        origin=Origin(
            xyz=(
                (door_w * 0.5) + door_gap,
                -(hinge_r + (door_t * 0.5)),
                0.0,
            )
        ),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material="handle_black",
        name="rose",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material="handle_black",
        name="hub",
    )
    handle.visual(
        Box((0.058, 0.012, 0.014)),
        origin=Origin(xyz=(0.026, 0.010, -0.010)),
        material="handle_black",
        name="lever_arm",
    )
    handle.visual(
        Cylinder(radius=0.0065, length=0.090),
        origin=Origin(xyz=(0.050, 0.010, -0.048)),
        material="handle_black",
        name="grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.090, 0.022, 0.116)),
        mass=0.35,
        origin=Origin(xyz=(0.030, 0.010, -0.034)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, door_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(door_w - 0.084, -hinge_r, handle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")
    door_hinge = object_model.get_articulation("door_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    top_carcass_knuckle = carcass.get_visual("hinge_0_carcass_knuckle")
    top_door_upper_knuckle = door.get_visual("hinge_0_upper_knuckle")
    top_door_lower_knuckle = door.get_visual("hinge_0_lower_knuckle")
    door_panel = door.get_visual("door_panel")
    handle_rose = handle.get_visual("rose")
    handle_grip = handle.get_visual("grip")

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
    ctx.allow_overlap(
        door,
        handle,
        elem_a=door_panel,
        elem_b=handle_rose,
        reason="The latch spindle/escutcheon seats against the door face and is represented with a shared mounting interface.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    carcass_aabb = ctx.part_world_aabb(carcass)
    if carcass_aabb is not None:
        carcass_size = tuple(carcass_aabb[1][i] - carcass_aabb[0][i] for i in range(3))
        ctx.check(
            "cabinet_proportions",
            0.75 <= carcass_size[0] <= 0.82
            and 0.45 <= carcass_size[1] <= 0.52
            and 1.84 <= carcass_size[2] <= 1.88,
            details=f"unexpected carcass extents {carcass_size}",
        )
    else:
        ctx.fail("cabinet_proportions", "missing carcass AABB")

    ctx.check(
        "door_hinge_axis_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"door hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "handle_pivot_axis_face_normal",
        tuple(handle_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"handle pivot axis was {handle_pivot.axis}",
    )

    ctx.expect_contact(
        door,
        carcass,
        elem_a=top_door_upper_knuckle,
        elem_b=top_carcass_knuckle,
        name="top_upper_hinge_contact_rest",
    )
    ctx.expect_contact(
        door,
        carcass,
        elem_a=top_door_lower_knuckle,
        elem_b=top_carcass_knuckle,
        name="top_lower_hinge_contact_rest",
    )
    ctx.expect_contact(handle, door, elem_a=handle_rose, elem_b=door_panel, name="handle_rose_contacts_door")
    ctx.expect_gap(
        handle,
        door,
        axis="y",
        min_gap=0.003,
        positive_elem=handle_grip,
        negative_elem=door_panel,
        name="handle_grip_stands_proud_of_door",
    )
    ctx.expect_overlap(door, carcass, axes="xz", min_overlap=0.60, name="door_covers_opening")

    door_panel_rest = ctx.part_element_world_aabb(door, elem=door_panel)
    handle_grip_rest = ctx.part_element_world_aabb(handle, elem=handle_grip)

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

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_lower_no_floating")
            ctx.expect_contact(
                door,
                carcass,
                elem_a=top_door_upper_knuckle,
                elem_b=top_carcass_knuckle,
                name="top_upper_hinge_contact_closed",
            )
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_upper_no_floating")
            ctx.expect_contact(
                door,
                carcass,
                elem_a=top_door_lower_knuckle,
                elem_b=top_carcass_knuckle,
                name="top_lower_hinge_contact_open",
            )
            door_panel_open = ctx.part_element_world_aabb(door, elem=door_panel)
            if door_panel_rest is not None and door_panel_open is not None:
                ctx.check(
                    "door_swings_clear_of_front",
                    door_panel_open[1][1] > door_panel_rest[1][1] + 0.25,
                    details=(
                        f"door panel max y only moved from {door_panel_rest[1][1]:.3f} "
                        f"to {door_panel_open[1][1]:.3f}"
                    ),
                )
            else:
                ctx.fail("door_swings_clear_of_front", "missing door panel AABB in swing test")

    handle_limits = handle_pivot.motion_limits
    if handle_limits is not None and handle_limits.lower is not None and handle_limits.upper is not None:
        with ctx.pose({handle_pivot: handle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="handle_lower_no_floating")
            ctx.expect_contact(handle, door, elem_a=handle_rose, elem_b=door_panel, name="handle_lower_contact")
        with ctx.pose({handle_pivot: handle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="handle_upper_no_floating")
            ctx.expect_contact(handle, door, elem_a=handle_rose, elem_b=door_panel, name="handle_upper_contact")
            handle_grip_open = ctx.part_element_world_aabb(handle, elem=handle_grip)
            if handle_grip_rest is not None and handle_grip_open is not None:
                grip_rest_center = 0.5 * (handle_grip_rest[0][0] + handle_grip_rest[1][0])
                grip_open_center = 0.5 * (handle_grip_open[0][0] + handle_grip_open[1][0])
                ctx.check(
                    "handle_rotates_about_pivot",
                    abs(grip_open_center - grip_rest_center) > 0.02,
                    details=(
                        f"handle grip x-center changed only from {grip_rest_center:.3f} "
                        f"to {grip_open_center:.3f}"
                    ),
                )
            else:
                ctx.fail("handle_rotates_about_pivot", "missing handle grip AABB in pivot test")

    with ctx.pose({door_hinge: math.radians(95.0), handle_pivot: 0.50}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
        ctx.expect_contact(
            handle,
            door,
            elem_a=handle_rose,
            elem_b=door_panel,
            name="combined_pose_handle_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
