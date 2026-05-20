from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cooktop_cabinet", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.93, 0.91, 1.0))
    burner_black = model.material("burner_black", rgba=(0.14, 0.14, 0.15, 1.0))
    grate_dark = model.material("grate_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    knob_black = model.material("knob_black", rgba=(0.16, 0.16, 0.17, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.78, 0.17, 0.12, 1.0))

    cabinet_width = 0.88
    cabinet_depth = 0.54
    cabinet_height = 0.90
    carcass_height = 0.86
    side_thickness = 0.018
    back_thickness = 0.012
    shelf_thickness = 0.018
    counter_thickness = cabinet_height - carcass_height
    toe_kick_height = 0.09
    front_plane = -cabinet_depth / 2.0

    opening_width = cabinet_width - 2.0 * side_thickness
    opening_depth = cabinet_depth - back_thickness

    cooktop_width = 0.60
    cooktop_depth = 0.46
    cooktop_plate_thickness = 0.006
    drop_box_width = 0.56
    drop_box_depth = 0.42
    drop_box_depth_z = 0.055
    counter_opening_width = 0.58
    counter_opening_depth = 0.44

    door_gap_side = 0.003
    center_seam = 0.004
    door_thickness = 0.018
    door_height = 0.698
    door_width = (opening_width - 2.0 * door_gap_side - center_seam) / 2.0
    hinge_radius = 0.006
    hinge_pin_radius = 0.0035
    hinge_offset = 0.027
    door_center_z = (
        toe_kick_height + shelf_thickness + 0.003 + door_height / 2.0
    )
    door_axis_y = front_plane - 0.010
    door_center_y = front_plane - door_thickness / 2.0
    door_local_center_y = door_center_y - door_axis_y

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        Box((side_thickness, cabinet_depth, carcass_height)),
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0 + side_thickness / 2.0,
                0.0,
                carcass_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="left_side",
    )
    cabinet_body.visual(
        Box((side_thickness, cabinet_depth, carcass_height)),
        origin=Origin(
            xyz=(
                cabinet_width / 2.0 - side_thickness / 2.0,
                0.0,
                carcass_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="right_side",
    )
    cabinet_body.visual(
        Box((opening_width, back_thickness, carcass_height)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth / 2.0 - back_thickness / 2.0,
                carcass_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="back_panel",
    )
    cabinet_body.visual(
        Box((opening_width, opening_depth, shelf_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                (back_thickness / 2.0 + front_plane + door_thickness) / 2.0,
                toe_kick_height + shelf_thickness / 2.0,
            )
        ),
        material=cabinet_white,
        name="base_shelf",
    )
    cabinet_body.visual(
        Box((opening_width, 0.012, toe_kick_height)),
        origin=Origin(xyz=(0.0, front_plane + 0.056, toe_kick_height / 2.0)),
        material=cabinet_white,
        name="toe_kick",
    )
    cabinet_body.visual(
        Box((opening_width, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, front_plane + 0.025, 0.835)),
        material=cabinet_white,
        name="front_rail",
    )

    side_counter_width = (cabinet_width - counter_opening_width) / 2.0
    front_counter_depth = (cabinet_depth - counter_opening_depth) / 2.0
    counter_z = carcass_height + counter_thickness / 2.0
    cabinet_body.visual(
        Box((side_counter_width, counter_opening_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                -counter_opening_width / 2.0 - side_counter_width / 2.0,
                0.0,
                counter_z,
            )
        ),
        material=stainless,
        name="left_counter_strip",
    )
    cabinet_body.visual(
        Box((side_counter_width, counter_opening_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                counter_opening_width / 2.0 + side_counter_width / 2.0,
                0.0,
                counter_z,
            )
        ),
        material=stainless,
        name="right_counter_strip",
    )
    cabinet_body.visual(
        Box((cabinet_width, front_counter_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -counter_opening_depth / 2.0 - front_counter_depth / 2.0,
                counter_z,
            )
        ),
        material=stainless,
        name="front_counter_strip",
    )
    cabinet_body.visual(
        Box((cabinet_width, front_counter_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                counter_opening_depth / 2.0 + front_counter_depth / 2.0,
                counter_z,
            )
        ),
        material=stainless,
        name="back_counter_strip",
    )

    left_door_axis_x = -cabinet_width / 2.0 - hinge_radius
    right_door_axis_x = cabinet_width / 2.0 + hinge_radius
    cabinet_body.visual(
        Box((0.006, 0.012, door_height)),
        origin=Origin(
            xyz=(left_door_axis_x + 0.003, door_axis_y + 0.006, door_center_z),
        ),
        material=handle_metal,
        name="left_hinge_pin",
    )
    cabinet_body.visual(
        Box((0.006, 0.012, door_height)),
        origin=Origin(
            xyz=(right_door_axis_x - 0.003, door_axis_y + 0.006, door_center_z),
        ),
        material=handle_metal,
        name="right_hinge_pin",
    )

    cooktop_panel = model.part("cooktop_panel")
    cooktop_panel.visual(
        Box((cooktop_width, cooktop_depth, cooktop_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height + cooktop_plate_thickness / 2.0)),
        material=stainless,
        name="stainless_top",
    )
    cooktop_panel.visual(
        Box((drop_box_width, drop_box_depth, drop_box_depth_z)),
        origin=Origin(
            xyz=(0.0, 0.0, cabinet_height - drop_box_depth_z / 2.0),
        ),
        material=stainless,
        name="drop_box",
    )
    cooktop_panel.visual(
        Box((0.16, 0.045, 0.03)),
        origin=Origin(
            xyz=(
                0.19,
                -(cooktop_depth / 2.0) + 0.045 / 2.0,
                cabinet_height + cooktop_plate_thickness + 0.015,
            )
        ),
        material=stainless,
        name="control_strip",
    )
    model.articulation(
        "body_to_cooktop",
        ArticulationType.FIXED,
        parent=cabinet_body,
        child=cooktop_panel,
        origin=Origin(),
    )

    burner_specs = {
        "front_left_burner": (-0.14, -0.11),
        "front_right_burner": (0.14, -0.11),
        "rear_left_burner": (-0.14, 0.11),
        "rear_right_burner": (0.14, 0.11),
    }
    burner_top_z = cabinet_height + cooktop_plate_thickness
    for burner_name, (bx, by) in burner_specs.items():
        burner = model.part(burner_name)
        burner.visual(
            Cylinder(radius=0.046, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, burner_top_z + 0.005)),
            material=burner_black,
            name="burner_ring",
        )
        burner.visual(
            Cylinder(radius=0.021, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, burner_top_z + 0.016)),
            material=burner_black,
            name="burner_cap",
        )
        burner.visual(
            Box((0.095, 0.012, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, burner_top_z + 0.014)),
            material=grate_dark,
            name="grate_cross_x",
        )
        burner.visual(
            Box((0.012, 0.095, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, burner_top_z + 0.014)),
            material=grate_dark,
            name="grate_cross_y",
        )
        model.articulation(
            f"cooktop_to_{burner_name}",
            ArticulationType.FIXED,
            parent=cooktop_panel,
            child=burner,
            origin=Origin(xyz=(bx, by, 0.0)),
        )

    knob_positions = {
        "left_knob": 0.14,
        "center_knob": 0.19,
        "right_knob": 0.24,
    }
    knob_axis_y = -(cooktop_depth / 2.0)
    knob_axis_z = cabinet_height + cooktop_plate_thickness + 0.024
    for knob_name, knob_x in knob_positions.items():
        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=0.022, length=0.026),
            origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.003, 0.018)),
            origin=Origin(xyz=(0.0, -0.0245, 0.008)),
            material=indicator_red,
            name="indicator",
        )
        model.articulation(
            f"cooktop_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=cooktop_panel,
            child=knob,
            origin=Origin(xyz=(knob_x, knob_axis_y, knob_axis_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
        )

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(hinge_offset + door_width / 2.0, door_local_center_y, 0.0)),
        material=cabinet_white,
        name="door_panel",
    )
    left_door.visual(
        Box((hinge_offset + 0.006, 0.006, door_height - 0.06)),
        origin=Origin(
            xyz=((hinge_offset + 0.006) / 2.0, door_local_center_y, 0.0),
        ),
        material=handle_metal,
        name="hinge_leaf",
    )
    left_door.visual(
        Cylinder(radius=hinge_radius, length=door_height),
        origin=Origin(xyz=(0.0, door_local_center_y, 0.0)),
        material=handle_metal,
        name="hinge_barrel",
    )
    left_door.visual(
        Box((0.012, 0.012, 0.22)),
        origin=Origin(
            xyz=(
                hinge_offset + door_width - 0.030,
                door_local_center_y - door_thickness / 2.0 - 0.006,
                0.0,
            )
        ),
        material=handle_metal,
        name="pull_handle",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(-(hinge_offset + door_width / 2.0), door_local_center_y, 0.0)),
        material=cabinet_white,
        name="door_panel",
    )
    right_door.visual(
        Box((hinge_offset + 0.006, 0.006, door_height - 0.06)),
        origin=Origin(
            xyz=((-hinge_offset - 0.006) / 2.0, door_local_center_y, 0.0),
        ),
        material=handle_metal,
        name="hinge_leaf",
    )
    right_door.visual(
        Cylinder(radius=hinge_radius, length=door_height),
        origin=Origin(xyz=(0.0, door_local_center_y, 0.0)),
        material=handle_metal,
        name="hinge_barrel",
    )
    right_door.visual(
        Box((0.012, 0.012, 0.22)),
        origin=Origin(
            xyz=(
                -(hinge_offset + door_width - 0.030),
                door_local_center_y - door_thickness / 2.0 - 0.006,
                0.0,
            )
        ),
        material=handle_metal,
        name="pull_handle",
    )

    model.articulation(
        "body_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=left_door,
        origin=Origin(xyz=(left_door_axis_x, door_axis_y, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=0.0, upper=1.95),
    )
    model.articulation(
        "body_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=right_door,
        origin=Origin(xyz=(right_door_axis_x, door_axis_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=0.0, upper=1.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet_body = object_model.get_part("cabinet_body")
    cooktop_panel = object_model.get_part("cooktop_panel")
    burners = [
        object_model.get_part("front_left_burner"),
        object_model.get_part("front_right_burner"),
        object_model.get_part("rear_left_burner"),
        object_model.get_part("rear_right_burner"),
    ]
    left_knob = object_model.get_part("left_knob")
    center_knob = object_model.get_part("center_knob")
    right_knob = object_model.get_part("right_knob")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")

    left_knob_joint = object_model.get_articulation("cooktop_to_left_knob")
    center_knob_joint = object_model.get_articulation("cooktop_to_center_knob")
    right_knob_joint = object_model.get_articulation("cooktop_to_right_knob")
    left_door_joint = object_model.get_articulation("body_to_left_door")
    right_door_joint = object_model.get_articulation("body_to_right_door")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
        left_door,
        cabinet_body,
        elem_a="hinge_barrel",
        elem_b="left_hinge_pin",
        reason="Left door shares a physical hinge pin with the cabinet body.",
    )
    ctx.allow_overlap(
        right_door,
        cabinet_body,
        elem_a="hinge_barrel",
        elem_b="right_hinge_pin",
        reason="Right door shares a physical hinge pin with the cabinet body.",
    )

    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(cooktop_panel, cabinet_body, name="cooktop_panel_supported")
    for burner in burners:
        ctx.expect_contact(burner, cooktop_panel, name=f"{burner.name}_mounted")
        ctx.expect_within(
            burner,
            cooktop_panel,
            axes="xy",
            margin=0.09,
            name=f"{burner.name}_within_cooktop",
        )

    ctx.expect_contact(
        left_door,
        cabinet_body,
        elem_a="hinge_barrel",
        elem_b="left_hinge_pin",
        name="left_door_hinge_contact",
    )
    ctx.expect_contact(
        right_door,
        cabinet_body,
        elem_a="hinge_barrel",
        elem_b="right_hinge_pin",
        name="right_door_hinge_contact",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.002,
        max_gap=0.006,
        name="door_center_seam_gap",
    )

    for knob in (left_knob, center_knob, right_knob):
        ctx.expect_contact(knob, cooktop_panel, name=f"{knob.name}_seated_on_strip")

    ctx.expect_origin_gap(
        center_knob,
        left_knob,
        axis="x",
        min_gap=0.045,
        max_gap=0.055,
        name="left_to_center_knob_spacing",
    )
    ctx.expect_origin_gap(
        right_knob,
        center_knob,
        axis="x",
        min_gap=0.045,
        max_gap=0.055,
        name="center_to_right_knob_spacing",
    )
    ctx.expect_origin_gap(
        left_knob,
        cooktop_panel,
        axis="x",
        min_gap=0.12,
        max_gap=0.16,
        name="knob_cluster_far_right",
    )

    front_left = object_model.get_part("front_left_burner")
    front_right = object_model.get_part("front_right_burner")
    rear_left = object_model.get_part("rear_left_burner")
    rear_right = object_model.get_part("rear_right_burner")
    ctx.expect_origin_gap(
        front_right,
        front_left,
        axis="x",
        min_gap=0.26,
        max_gap=0.30,
        name="front_burner_pair_spacing",
    )
    ctx.expect_origin_gap(
        rear_left,
        front_left,
        axis="y",
        min_gap=0.20,
        max_gap=0.24,
        name="left_burner_row_spacing",
    )
    ctx.expect_origin_gap(
        rear_right,
        front_right,
        axis="y",
        min_gap=0.20,
        max_gap=0.24,
        name="right_burner_row_spacing",
    )

    ctx.check(
        "left_door_axis_vertical",
        tuple(left_door_joint.axis) == (0.0, 0.0, -1.0),
        details=f"left door axis is {left_door_joint.axis}",
    )
    ctx.check(
        "right_door_axis_vertical",
        tuple(right_door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"right door axis is {right_door_joint.axis}",
    )
    for joint in (left_knob_joint, center_knob_joint, right_knob_joint):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_continuous_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"{joint.name} axis is {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_continuous_limits",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"{joint.name} limits are {limits}",
        )

    with ctx.pose({left_knob_joint: 1.5, center_knob_joint: 3.0, right_knob_joint: 4.4}):
        for knob in (left_knob, center_knob, right_knob):
            ctx.expect_contact(knob, cooktop_panel, name=f"{knob.name}_rotated_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_rotated_no_overlap")

    left_limits = left_door_joint.motion_limits
    right_limits = right_door_joint.motion_limits
    if (
        left_limits is not None
        and left_limits.lower is not None
        and left_limits.upper is not None
    ):
        with ctx.pose({left_door_joint: left_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_door_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="left_door_closed_no_floating")
        with ctx.pose({left_door_joint: left_limits.upper}):
            left_open_panel_aabb = ctx.part_element_world_aabb(left_door, elem="door_panel")
            ctx.fail_if_parts_overlap_in_current_pose(name="left_door_open_no_overlap")
            ctx.expect_contact(
                left_door,
                cabinet_body,
                elem_a="hinge_barrel",
                elem_b="left_hinge_pin",
                name="left_door_open_hinge_contact",
            )
            ctx.check(
                "left_door_swings_outward",
                left_open_panel_aabb is not None and left_open_panel_aabb[0][1] < -0.60,
                details=f"left door open panel aabb is {left_open_panel_aabb}",
            )

    if (
        right_limits is not None
        and right_limits.lower is not None
        and right_limits.upper is not None
    ):
        with ctx.pose({right_door_joint: right_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_door_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="right_door_closed_no_floating")
        with ctx.pose({right_door_joint: right_limits.upper}):
            right_open_panel_aabb = ctx.part_element_world_aabb(right_door, elem="door_panel")
            ctx.fail_if_parts_overlap_in_current_pose(name="right_door_open_no_overlap")
            ctx.expect_contact(
                right_door,
                cabinet_body,
                elem_a="hinge_barrel",
                elem_b="right_hinge_pin",
                name="right_door_open_hinge_contact",
            )
            ctx.check(
                "right_door_swings_outward",
                right_open_panel_aabb is not None and right_open_panel_aabb[0][1] < -0.60,
                details=f"right door open panel aabb is {right_open_panel_aabb}",
            )

    if (
        left_limits is not None
        and left_limits.upper is not None
        and right_limits is not None
        and right_limits.upper is not None
    ):
        with ctx.pose({left_door_joint: left_limits.upper, right_door_joint: right_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="both_doors_open_no_overlap")
            ctx.expect_contact(
                left_door,
                cabinet_body,
                elem_a="hinge_barrel",
                elem_b="left_hinge_pin",
                name="both_open_left_hinge_contact",
            )
            ctx.expect_contact(
                right_door,
                cabinet_body,
                elem_a="hinge_barrel",
                elem_b="right_hinge_pin",
                name="both_open_right_hinge_contact",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
