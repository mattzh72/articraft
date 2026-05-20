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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cooktop_cabinet_bay", assets=ASSETS)

    stone = model.material("counter_stone", rgba=(0.73, 0.74, 0.76, 1.0))
    cabinet_white = model.material("cabinet_white", rgba=(0.90, 0.90, 0.87, 1.0))
    cooktop_glass = model.material("cooktop_glass", rgba=(0.08, 0.08, 0.10, 1.0))
    zone_mark = model.material("zone_mark", rgba=(0.22, 0.22, 0.24, 1.0))
    control_dark = model.material("control_dark", rgba=(0.15, 0.15, 0.17, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.75, 0.77, 0.79, 1.0))

    counter_width = 0.90
    counter_depth = 0.62
    counter_thickness = 0.04
    counter_top_z = 0.90
    counter_center_z = counter_top_z - (counter_thickness / 2.0)

    cabinet_depth = 0.56
    cabinet_height = counter_top_z - counter_thickness
    side_thickness = 0.018
    back_thickness = 0.012
    shelf_thickness = 0.018
    rail_depth = 0.018
    rail_height = 0.04
    toe_kick_height = 0.10

    cooktop_width = 0.58
    cooktop_depth = 0.51
    glass_thickness = 0.008
    front_band_depth = 0.08
    opening_width = 0.544
    opening_depth = 0.474
    underbox_height = 0.09
    underbox_wall = 0.006

    small_zone_ring = mesh_from_geometry(
        TorusGeometry(radius=0.0788, tube=0.0012, radial_segments=14, tubular_segments=56),
        "assets/meshes/cooktop_zone_small.obj",
    )
    large_zone_ring = mesh_from_geometry(
        TorusGeometry(radius=0.0888, tube=0.0012, radial_segments=14, tubular_segments=56),
        "assets/meshes/cooktop_zone_large.obj",
    )
    cabinet = model.part("counter_cabinet")

    cabinet.visual(
        Box(((counter_width - opening_width) / 2.0, counter_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                -((counter_width + opening_width) / 4.0),
                0.0,
                counter_center_z,
            )
        ),
        material=stone,
        name="counter_left_strip",
    )
    cabinet.visual(
        Box(((counter_width - opening_width) / 2.0, counter_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                ((counter_width + opening_width) / 4.0),
                0.0,
                counter_center_z,
            )
        ),
        material=stone,
        name="counter_right_strip",
    )
    cabinet.visual(
        Box((opening_width, (counter_depth - opening_depth) / 2.0, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -((counter_depth + opening_depth) / 4.0),
                counter_center_z,
            )
        ),
        material=stone,
        name="counter_front_strip",
    )
    cabinet.visual(
        Box((opening_width, (counter_depth - opening_depth) / 2.0, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                ((counter_depth + opening_depth) / 4.0),
                counter_center_z,
            )
        ),
        material=stone,
        name="counter_back_strip",
    )

    cabinet.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                -(counter_width / 2.0) + (side_thickness / 2.0),
                0.0,
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                (counter_width / 2.0) - (side_thickness / 2.0),
                0.0,
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((counter_width - (2.0 * side_thickness), back_thickness, cabinet_height)),
        origin=Origin(
            xyz=(
                0.0,
                (cabinet_depth / 2.0) - (back_thickness / 2.0),
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="back_panel",
    )
    cabinet.visual(
        Box(
            (
                counter_width - (2.0 * side_thickness),
                cabinet_depth - 0.02,
                shelf_thickness,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.01,
                toe_kick_height + (shelf_thickness / 2.0),
            )
        ),
        material=cabinet_white,
        name="base_panel",
    )
    cabinet.visual(
        Box((counter_width - (2.0 * side_thickness), rail_depth, toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(cabinet_depth / 2.0) + (rail_depth / 2.0),
                toe_kick_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="toe_kick",
    )
    cabinet.visual(
        Box((counter_width - (2.0 * side_thickness), rail_depth, rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(cabinet_depth / 2.0) + (rail_depth / 2.0),
                cabinet_height - (rail_height / 2.0),
            )
        ),
        material=cabinet_white,
        name="front_rail",
    )
    cabinet.visual(
        Box((counter_width - (2.0 * side_thickness), rail_depth, rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                (cabinet_depth / 2.0) - (rail_depth / 2.0),
                cabinet_height - (rail_height / 2.0),
            )
        ),
        material=cabinet_white,
        name="back_rail",
    )

    cooktop = model.part("cooktop_body")
    cooktop_top_z = counter_top_z + glass_thickness
    glass_center_z = counter_top_z + (glass_thickness / 2.0)
    cooktop_front_y = -(cooktop_depth / 2.0)
    back_plate_depth = cooktop_depth - front_band_depth

    cooktop.visual(
        Box((cooktop_width, back_plate_depth, glass_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                cooktop_front_y + front_band_depth + (back_plate_depth / 2.0),
                glass_center_z,
            )
        ),
        material=cooktop_glass,
        name="glass_main",
    )

    band_y = cooktop_front_y + (front_band_depth / 2.0)
    control_segments = (
        ("left_outer_band", -0.2715, 0.037),
        ("left_inner_band", -0.2210, 0.012),
        ("center_control_band", 0.0000, 0.378),
        ("right_inner_band", 0.2210, 0.012),
        ("right_outer_band", 0.2715, 0.037),
    )
    for name, x_pos, width in control_segments:
        cooktop.visual(
            Box((width, front_band_depth, glass_thickness)),
            origin=Origin(xyz=(x_pos, band_y, glass_center_z)),
            material=cooktop_glass,
            name=name,
        )

    underbox_center_z = counter_top_z - (underbox_height / 2.0)
    cooktop.visual(
        Box((underbox_wall, opening_depth, underbox_height)),
        origin=Origin(xyz=(-(opening_width / 2.0) + (underbox_wall / 2.0), 0.0, underbox_center_z)),
        material=control_dark,
        name="underbox_left_wall",
    )
    cooktop.visual(
        Box((underbox_wall, opening_depth, underbox_height)),
        origin=Origin(xyz=((opening_width / 2.0) - (underbox_wall / 2.0), 0.0, underbox_center_z)),
        material=control_dark,
        name="underbox_right_wall",
    )
    cooktop.visual(
        Box((opening_width - (2.0 * underbox_wall), underbox_wall, underbox_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(opening_depth / 2.0) + (underbox_wall / 2.0),
                underbox_center_z,
            )
        ),
        material=control_dark,
        name="underbox_front_wall",
    )
    cooktop.visual(
        Box((opening_width - (2.0 * underbox_wall), underbox_wall, underbox_height)),
        origin=Origin(
            xyz=(
                0.0,
                (opening_depth / 2.0) - (underbox_wall / 2.0),
                underbox_center_z,
            )
        ),
        material=control_dark,
        name="underbox_back_wall",
    )
    cooktop.visual(
        Box(
            (
                opening_width - (2.0 * underbox_wall),
                opening_depth - (2.0 * underbox_wall),
                underbox_wall,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                counter_top_z - underbox_height + (underbox_wall / 2.0),
            )
        ),
        material=control_dark,
        name="underbox_bottom",
    )

    zone_specs = (
        ("zone_front_left", -0.145, -0.035, small_zone_ring),
        ("zone_front_right", 0.145, -0.035, small_zone_ring),
        ("zone_rear_left", -0.145, 0.145, large_zone_ring),
        ("zone_rear_right", 0.145, 0.145, large_zone_ring),
    )
    for name, x_pos, y_pos, zone_geometry in zone_specs:
        cooktop.visual(
            zone_geometry,
            origin=Origin(xyz=(x_pos, y_pos, cooktop_top_z + 0.0010)),
            material=zone_mark,
            name=name,
        )

    model.articulation(
        "cabinet_to_cooktop",
        ArticulationType.FIXED,
        parent=cabinet,
        child=cooktop,
        origin=Origin(),
    )

    button_size = 0.026
    button_height = 0.016
    button_top_proud = 0.004
    button_center_y = -0.210
    button_z_origin = cooktop_top_z
    button_specs = (
        ("button_left_outer", -0.240),
        ("button_left_inner", -0.202),
        ("button_right_inner", 0.202),
        ("button_right_outer", 0.240),
    )
    for part_name, x_pos in button_specs:
        button = model.part(part_name)
        button.visual(
            Box((button_size, button_size, button_height)),
            origin=Origin(xyz=(0.0, 0.0, -(button_height / 2.0) + button_top_proud)),
            material=control_dark,
            name="button_body",
        )
        model.articulation(
            f"cooktop_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=cooktop,
            child=button,
            origin=Origin(xyz=(x_pos, button_center_y, button_z_origin)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    knob = model.part("center_knob")
    knob.visual(
        Cylinder(radius=0.034, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=control_dark,
        name="knob_shell",
    )
    knob.visual(
        Cylinder(radius=0.022, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=brushed_metal,
        name="knob_cap",
    )
    knob.visual(
        Box((0.004, 0.016, 0.0015)),
        origin=Origin(xyz=(0.0, 0.016, 0.02325)),
        material=brushed_metal,
        name="knob_pointer",
    )
    model.articulation(
        "cooktop_to_center_knob",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child=knob,
        origin=Origin(xyz=(0.0, button_center_y, cooktop_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0),
    )

    door_gap = 0.004
    door_thickness = 0.019
    door_height = 0.72
    opening_clear_width = counter_width - (2.0 * side_thickness)
    door_width = (opening_clear_width - door_gap) / 2.0
    door_center_z = toe_kick_height + (door_height / 2.0)
    hinge_y = -(cabinet_depth / 2.0)

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(
            xyz=((side_thickness + (door_width / 2.0)), -(door_thickness / 2.0), 0.0)
        ),
        material=cabinet_white,
        name="door_panel",
    )
    left_door.visual(
        Cylinder(radius=0.007, length=0.220),
        origin=Origin(
            xyz=(
                side_thickness + door_width - 0.060,
                -(door_thickness + 0.007),
                0.0,
            )
        ),
        material=brushed_metal,
        name="door_handle",
    )
    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(-(counter_width / 2.0), hinge_y, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=1.92,
        ),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                -(side_thickness + (door_width / 2.0)),
                -(door_thickness / 2.0),
                0.0,
            )
        ),
        material=cabinet_white,
        name="door_panel",
    )
    right_door.visual(
        Cylinder(radius=0.007, length=0.220),
        origin=Origin(
            xyz=(
                -(side_thickness + door_width - 0.060),
                -(door_thickness + 0.007),
                0.0,
            )
        ),
        material=brushed_metal,
        name="door_handle",
    )
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=((counter_width / 2.0), hinge_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=1.92,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("counter_cabinet")
    cooktop = object_model.get_part("cooktop_body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    center_knob = object_model.get_part("center_knob")
    buttons = [
        object_model.get_part("button_left_outer"),
        object_model.get_part("button_left_inner"),
        object_model.get_part("button_right_inner"),
        object_model.get_part("button_right_outer"),
    ]
    button_joints = [
        object_model.get_articulation("cooktop_to_button_left_outer"),
        object_model.get_articulation("cooktop_to_button_left_inner"),
        object_model.get_articulation("cooktop_to_button_right_inner"),
        object_model.get_articulation("cooktop_to_button_right_outer"),
    ]
    knob_joint = object_model.get_articulation("cooktop_to_center_knob")
    door_joints = [
        object_model.get_articulation("cabinet_to_left_door"),
        object_model.get_articulation("cabinet_to_right_door"),
    ]

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
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_pose_no_floating")

    ctx.expect_contact(cooktop, cabinet, name="cooktop_contacts_counter_cutout")
    ctx.expect_contact(center_knob, cooktop, name="center_knob_is_seated")
    ctx.expect_overlap(center_knob, cooktop, axes="xy", min_overlap=0.06, name="center_knob_is_centered")

    for button in buttons:
        ctx.expect_contact(button, cooktop, name=f"{button.name}_guided_in_cooktop")
        ctx.expect_overlap(button, cooktop, axes="xy", min_overlap=0.024, name=f"{button.name}_stays_in_button_lane")

    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.004,
        max_gap=0.006,
        name="closed_doors_leave_center_reveal",
    )
    ctx.expect_overlap(left_door, cabinet, axes="z", min_overlap=0.70, name="left_door_spans_opening_height")
    ctx.expect_overlap(right_door, cabinet, axes="z", min_overlap=0.70, name="right_door_spans_opening_height")

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

    for joint, button in zip(button_joints, buttons):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
                ctx.expect_contact(button, cooktop, name=f"{joint.name}_lower_stays_guided")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
                ctx.expect_contact(button, cooktop, name=f"{joint.name}_upper_stays_guided")

    for joint in door_joints:
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    with ctx.pose({knob_joint: pi / 3.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="center_knob_spin_no_overlap")
        ctx.expect_contact(center_knob, cooktop, name="center_knob_spin_stays_seated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
