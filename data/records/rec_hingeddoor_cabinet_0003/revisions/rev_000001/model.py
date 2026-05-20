from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_cabinet", assets=ASSETS)

    cabinet_paint = model.material("cabinet_paint", rgba=(0.79, 0.81, 0.78, 1.0))
    shelf_paint = model.material("shelf_paint", rgba=(0.88, 0.89, 0.87, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.19, 0.20, 0.22, 1.0))

    outer_w = 0.72
    outer_d = 0.42
    body_h = 1.78
    feet_h = 0.05
    outer_h = body_h + feet_h

    wall_t = 0.018
    back_t = 0.006
    shelf_t = 0.018
    door_t = 0.018

    body_bottom_z = feet_h
    body_center_z = body_bottom_z + (body_h / 2.0)
    inner_w = outer_w - (2.0 * wall_t)
    shelf_depth = 0.36
    shelf_center_y = (-outer_d / 2.0) + back_t + (shelf_depth / 2.0)

    hinge_axis_x = (-outer_w / 2.0) - 0.004
    hinge_axis_y = (outer_d / 2.0) + 0.002
    hinge_axis_z = body_center_z
    hinge_barrel_r = 0.006
    fixed_knuckle_len = 0.034
    moving_knuckle_len = 0.036
    hinge_knuckle_offset_z = (moving_knuckle_len / 2.0) + 0.004 + (fixed_knuckle_len / 2.0)
    hinge_zs = (
        body_bottom_z + 0.30,
        body_bottom_z + body_h - 0.30,
    )

    door_edge_to_axis = 0.007
    door_w = outer_w - 0.010
    door_h = body_h - 0.006

    carcass = model.part("carcass")
    carcass.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=((-outer_w / 2.0) + (wall_t / 2.0), 0.0, body_center_z)),
        material=cabinet_paint,
        name="left_wall",
    )
    carcass.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=((outer_w / 2.0) - (wall_t / 2.0), 0.0, body_center_z)),
        material=cabinet_paint,
        name="right_wall",
    )
    carcass.visual(
        Box((inner_w, outer_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom_z + (wall_t / 2.0))),
        material=cabinet_paint,
        name="bottom_panel",
    )
    carcass.visual(
        Box((inner_w, outer_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom_z + body_h - (wall_t / 2.0))),
        material=cabinet_paint,
        name="top_panel",
    )
    carcass.visual(
        Box((inner_w, back_t, body_h - (2.0 * wall_t))),
        origin=Origin(
            xyz=(
                0.0,
                (-outer_d / 2.0) + (back_t / 2.0),
                body_center_z,
            )
        ),
        material=cabinet_paint,
        name="back_panel",
    )

    shelf_zs = (
        body_bottom_z + 0.40,
        body_bottom_z + 0.84,
        body_bottom_z + 1.28,
    )
    for index, shelf_z in enumerate(shelf_zs, start=1):
        carcass.visual(
            Box((inner_w, shelf_depth, shelf_t)),
            origin=Origin(xyz=(0.0, shelf_center_y, shelf_z)),
            material=shelf_paint,
            name=f"shelf_{index}",
        )

    foot_w = 0.06
    foot_d = 0.06
    foot_positions = {
        "foot_front_left": ((-outer_w / 2.0) + 0.07, (outer_d / 2.0) - 0.07, feet_h / 2.0),
        "foot_front_right": ((outer_w / 2.0) - 0.07, (outer_d / 2.0) - 0.07, feet_h / 2.0),
        "foot_back_left": ((-outer_w / 2.0) + 0.07, (-outer_d / 2.0) + 0.07, feet_h / 2.0),
        "foot_back_right": ((outer_w / 2.0) - 0.07, (-outer_d / 2.0) + 0.07, feet_h / 2.0),
    }
    for name, xyz in foot_positions.items():
        carcass.visual(
            Box((foot_w, foot_d, feet_h)),
            origin=Origin(xyz=xyz),
            material=foot_rubber,
            name=name,
        )

    fixed_tab_size = (0.014, 0.014, fixed_knuckle_len)
    fixed_tab_y = (outer_d / 2.0) - 0.010
    for hinge_name, hinge_z in zip(("lower", "upper"), hinge_zs):
        lower_fixed_z = hinge_z - hinge_knuckle_offset_z
        upper_fixed_z = hinge_z + hinge_knuckle_offset_z
        for section_name, section_z in (("lower", lower_fixed_z), ("upper", upper_fixed_z)):
            carcass.visual(
                Cylinder(radius=hinge_barrel_r, length=fixed_knuckle_len),
                origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, section_z)),
                material=hinge_steel,
                name=f"hinge_{hinge_name}_fixed_{section_name}_knuckle",
            )
            carcass.visual(
                Box(fixed_tab_size),
                origin=Origin(
                    xyz=(
                        hinge_axis_x + (fixed_tab_size[0] / 2.0),
                        fixed_tab_y,
                        section_z,
                    )
                ),
                material=hinge_steel,
                name=f"hinge_{hinge_name}_fixed_{section_name}_leaf",
            )

    carcass.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0)),
    )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_edge_to_axis + (door_w / 2.0), door_t / 2.0, 0.0)),
        material=cabinet_paint,
        name="door_panel",
    )
    door.visual(
        Box((0.018, 0.010, 0.34)),
        origin=Origin(xyz=(door_edge_to_axis + door_w - 0.050, door_t + 0.005, 0.0)),
        material=handle_dark,
        name="pull_handle",
    )

    moving_tab_size = (0.020, 0.012, moving_knuckle_len)
    for hinge_name, hinge_z in zip(("lower", "upper"), hinge_zs):
        local_hinge_z = hinge_z - hinge_axis_z
        door.visual(
            Cylinder(radius=hinge_barrel_r * 0.92, length=moving_knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, local_hinge_z)),
            material=hinge_steel,
            name=f"hinge_{hinge_name}_moving_knuckle",
        )
        door.visual(
            Box(moving_tab_size),
            origin=Origin(
                xyz=(
                    moving_tab_size[0] / 2.0,
                    0.004,
                    local_hinge_z,
                )
            ),
            material=hinge_steel,
            name=f"hinge_{hinge_name}_moving_leaf",
        )

    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=12.0,
        origin=Origin(xyz=(door_edge_to_axis + (door_w / 2.0), door_t / 2.0, 0.0)),
    )

    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, hinge_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("carcass_to_door")

    left_wall = carcass.get_visual("left_wall")
    top_panel = carcass.get_visual("top_panel")
    shelf_2 = carcass.get_visual("shelf_2")
    foot_front_left = carcass.get_visual("foot_front_left")
    upper_fixed_lower = carcass.get_visual("hinge_upper_fixed_lower_knuckle")
    upper_fixed_upper = carcass.get_visual("hinge_upper_fixed_upper_knuckle")

    door_panel = door.get_visual("door_panel")
    pull_handle = door.get_visual("pull_handle")
    upper_moving = door.get_visual("hinge_upper_moving_knuckle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=12, name="door_clear_through_swing")

    ctx.check(
        "cabinet_parts_present",
        all(item is not None for item in (carcass, door, door_hinge)),
        "Expected carcass part, door part, and door hinge articulation.",
    )
    ctx.check(
        "cabinet_features_present",
        all(
            item is not None
            for item in (
                left_wall,
                top_panel,
                shelf_2,
                foot_front_left,
                upper_fixed_lower,
                upper_fixed_upper,
                door_panel,
                pull_handle,
                upper_moving,
            )
        ),
        "Missing one or more named cabinet visuals.",
    )
    ctx.check(
        "door_hinge_is_vertical_and_120_deg",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0)
        and door_hinge.motion_limits is not None
        and abs(door_hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(door_hinge.motion_limits.upper - math.radians(120.0)) < 1e-9,
        "Door hinge should rotate about +Z from 0 to 120 degrees.",
    )

    ctx.expect_gap(
        door,
        carcass,
        axis="y",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem=door_panel,
        negative_elem=top_panel,
        name="door_sits_just_proud_of_front_opening",
    )
    ctx.expect_overlap(
        door,
        carcass,
        axes="x",
        min_overlap=0.68,
        elem_a=door_panel,
        name="door_covers_cabinet_width",
    )
    ctx.expect_overlap(
        door,
        carcass,
        axes="z",
        min_overlap=1.72,
        elem_a=door_panel,
        name="door_runs_full_height",
    )
    ctx.expect_within(
        door,
        carcass,
        axes="x",
        margin=0.015,
        inner_elem=door_panel,
        name="door_width_stays_within_carcass_footprint",
    )

    fixed_lower_aabb = ctx.part_element_world_aabb(carcass, elem="hinge_upper_fixed_lower_knuckle")
    fixed_upper_aabb = ctx.part_element_world_aabb(carcass, elem="hinge_upper_fixed_upper_knuckle")
    moving_aabb = ctx.part_element_world_aabb(door, elem="hinge_upper_moving_knuckle")
    hinge_alignment_ok = False
    hinge_alignment_details = "Could not measure upper hinge knuckles."
    if fixed_lower_aabb is not None and fixed_upper_aabb is not None and moving_aabb is not None:
        fixed_center_xy = (
            (fixed_lower_aabb[0][0] + fixed_upper_aabb[1][0]) / 2.0,
            (fixed_lower_aabb[0][1] + fixed_upper_aabb[1][1]) / 2.0,
        )
        moving_center_xy = (
            (moving_aabb[0][0] + moving_aabb[1][0]) / 2.0,
            (moving_aabb[0][1] + moving_aabb[1][1]) / 2.0,
        )
        hinge_alignment_ok = (
            abs(fixed_center_xy[0] - moving_center_xy[0]) <= 0.001
            and abs(fixed_center_xy[1] - moving_center_xy[1]) <= 0.001
            and moving_aabb[0][2] >= fixed_lower_aabb[1][2]
            and moving_aabb[1][2] <= fixed_upper_aabb[0][2]
        )
        hinge_alignment_details = (
            f"fixed_center_xy={fixed_center_xy}, moving_center_xy={moving_center_xy}, "
            f"fixed_lower_top={fixed_lower_aabb[1][2]:.4f}, moving=({moving_aabb[0][2]:.4f}, "
            f"{moving_aabb[1][2]:.4f}), fixed_upper_bottom={fixed_upper_aabb[0][2]:.4f}"
        )
    ctx.check("upper_hinge_knuckles_are_coaxial", hinge_alignment_ok, hinge_alignment_details)

    with ctx.pose({door_hinge: math.radians(120.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_has_no_part_overlap")
        open_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        open_pose_ok = False
        open_pose_details = "Could not measure the open door panel."
        if open_panel_aabb is not None:
            open_pose_ok = open_panel_aabb[1][1] > 0.75 and open_panel_aabb[1][0] < -0.34
            open_pose_details = (
                f"open_panel_aabb_min={open_panel_aabb[0]}, open_panel_aabb_max={open_panel_aabb[1]}"
            )
        ctx.check("door_swings_open_outside_left_side", open_pose_ok, open_pose_details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
