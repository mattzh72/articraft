from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="french_door_refrigerator", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    stainless_dark = model.material("stainless_dark", rgba=(0.60, 0.62, 0.64, 1.0))
    shadow_trim = model.material("shadow_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    liner = model.material("liner", rgba=(0.90, 0.91, 0.92, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.64, 0.65, 0.67, 1.0))

    width = 0.92
    depth = 0.72
    height = 1.83
    wall = 0.025
    corner_radius = 0.03
    header_height = 0.075
    freezer_height = 0.47
    seam_gap = 0.006
    top_gap = 0.006
    side_reveal = 0.008
    center_gap = 0.006
    front_clearance = 0.002
    door_thickness = 0.045
    drawer_front_thickness = 0.045

    axis_y = depth / 2.0 + front_clearance
    door_width = (width - (2.0 * side_reveal) - center_gap) / 2.0
    door_bottom_z = freezer_height + seam_gap / 2.0
    door_height = height - header_height - top_gap - door_bottom_z
    left_axis_x = -width / 2.0 + side_reveal
    right_axis_x = width / 2.0 - side_reveal
    drawer_bottom_z = 0.045
    drawer_front_height = freezer_height - seam_gap / 2.0 - drawer_bottom_z
    interior_half_width = width / 2.0 - wall

    hinge_leaf_width = 0.022
    hinge_leaf_depth = 0.004
    hinge_leaf_height = 0.055
    hinge_y = depth / 2.0 + hinge_leaf_depth / 2.0
    top_hinge_local_z = door_height - 0.06
    bottom_hinge_local_z = 0.06

    guide_plate_thickness = 0.004
    guide_flange_width = 0.016
    guide_flange_thickness = 0.004
    guide_height = 0.09
    guide_length = 0.58
    guide_center_y = 0.01
    guide_center_z = 0.19

    runner_thickness = 0.006
    runner_height = 0.04
    runner_length = 0.36

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=stainless_dark,
        name="left_side_shell",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=stainless_dark,
        name="right_side_shell",
    )
    cabinet.visual(
        Box((width - (2.0 * wall), depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=stainless_dark,
        name="top_shell",
    )
    cabinet.visual(
        Box((width - (2.0 * wall), depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=stainless_dark,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((width - (2.0 * wall), wall, height - (2.0 * wall))),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, height / 2.0)),
        material=shadow_trim,
        name="back_panel",
    )
    cabinet.visual(
        Cylinder(radius=corner_radius, length=height),
        origin=Origin(xyz=(-width / 2.0 + corner_radius, depth / 2.0 - corner_radius, height / 2.0)),
        material=stainless,
        name="left_front_radius",
    )
    cabinet.visual(
        Cylinder(radius=corner_radius, length=height),
        origin=Origin(xyz=(width / 2.0 - corner_radius, depth / 2.0 - corner_radius, height / 2.0)),
        material=stainless,
        name="right_front_radius",
    )
    cabinet.visual(
        Box((width - 0.14, 0.015, header_height - 0.012)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.0175, height - header_height / 2.0 - 0.006)),
        material=stainless_dark,
        name="top_header_panel",
    )
    guide_track_x = width / 2.0 - wall - 0.037
    left_guide_x = -guide_track_x
    right_guide_x = guide_track_x
    guide_flange_center_x = guide_track_x - (guide_flange_width - guide_plate_thickness) / 2.0
    guide_bridge_width = interior_half_width - (guide_track_x - guide_plate_thickness / 2.0)

    cabinet.visual(
        Box((guide_plate_thickness, guide_length, guide_height)),
        origin=Origin(xyz=(left_guide_x, guide_center_y, guide_center_z)),
        material=rail_metal,
        name="left_guide_plate",
    )
    cabinet.visual(
        Box((guide_plate_thickness, guide_length, guide_height)),
        origin=Origin(xyz=(right_guide_x, guide_center_y, guide_center_z)),
        material=rail_metal,
        name="right_guide_plate",
    )
    cabinet.visual(
        Box((guide_flange_width, guide_length, guide_flange_thickness)),
        origin=Origin(
            xyz=(-guide_flange_center_x, guide_center_y, guide_center_z + guide_height / 2.0 - guide_flange_thickness / 2.0)
        ),
        material=rail_metal,
        name="left_guide_top_flange",
    )
    cabinet.visual(
        Box((guide_flange_width, guide_length, guide_flange_thickness)),
        origin=Origin(
            xyz=(-guide_flange_center_x, guide_center_y, guide_center_z - guide_height / 2.0 + guide_flange_thickness / 2.0)
        ),
        material=rail_metal,
        name="left_guide_bottom_flange",
    )
    cabinet.visual(
        Box((guide_flange_width, guide_length, guide_flange_thickness)),
        origin=Origin(
            xyz=(guide_flange_center_x, guide_center_y, guide_center_z + guide_height / 2.0 - guide_flange_thickness / 2.0)
        ),
        material=rail_metal,
        name="right_guide_top_flange",
    )
    cabinet.visual(
        Box((guide_flange_width, guide_length, guide_flange_thickness)),
        origin=Origin(
            xyz=(guide_flange_center_x, guide_center_y, guide_center_z - guide_height / 2.0 + guide_flange_thickness / 2.0)
        ),
        material=rail_metal,
        name="right_guide_bottom_flange",
    )
    cabinet.visual(
        Box((guide_bridge_width, 0.06, 0.05)),
        origin=Origin(xyz=(-(interior_half_width + guide_track_x - guide_plate_thickness / 2.0) / 2.0, -0.17, guide_center_z)),
        material=rail_metal,
        name="left_guide_rear_mount",
    )
    cabinet.visual(
        Box((guide_bridge_width, 0.06, 0.05)),
        origin=Origin(xyz=(-(interior_half_width + guide_track_x - guide_plate_thickness / 2.0) / 2.0, 0.17, guide_center_z)),
        material=rail_metal,
        name="left_guide_front_mount",
    )
    cabinet.visual(
        Box((guide_bridge_width, 0.06, 0.05)),
        origin=Origin(xyz=((interior_half_width + guide_track_x - guide_plate_thickness / 2.0) / 2.0, -0.17, guide_center_z)),
        material=rail_metal,
        name="right_guide_rear_mount",
    )
    cabinet.visual(
        Box((guide_bridge_width, 0.06, 0.05)),
        origin=Origin(xyz=((interior_half_width + guide_track_x - guide_plate_thickness / 2.0) / 2.0, 0.17, guide_center_z)),
        material=rail_metal,
        name="right_guide_front_mount",
    )

    cabinet.visual(
        Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
        origin=Origin(xyz=(left_axis_x - hinge_leaf_width / 2.0, hinge_y, door_bottom_z + top_hinge_local_z)),
        material=rail_metal,
        name="left_top_hinge_leaf",
    )
    cabinet.visual(
        Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
        origin=Origin(xyz=(left_axis_x - hinge_leaf_width / 2.0, hinge_y, door_bottom_z + bottom_hinge_local_z)),
        material=rail_metal,
        name="left_bottom_hinge_leaf",
    )
    cabinet.visual(
        Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
        origin=Origin(xyz=(right_axis_x + hinge_leaf_width / 2.0, hinge_y, door_bottom_z + top_hinge_local_z)),
        material=rail_metal,
        name="right_top_hinge_leaf",
    )
    cabinet.visual(
        Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
        origin=Origin(xyz=(right_axis_x + hinge_leaf_width / 2.0, hinge_y, door_bottom_z + bottom_hinge_local_z)),
        material=rail_metal,
        name="right_bottom_hinge_leaf",
    )

    def add_door_bins(door_part, x_center: float, prefix: str) -> None:
        bin_width = door_width - 0.11
        back_thickness = 0.004
        shelf_depth = 0.07
        shelf_thickness = 0.004
        lip_thickness = 0.004
        lip_height = 0.035
        back_height = 0.08
        for index, bin_z in enumerate((0.24, 0.54, 0.84), start=1):
            door_part.visual(
                Box((bin_width, back_thickness, back_height)),
                origin=Origin(xyz=(x_center, -back_thickness / 2.0, bin_z)),
                material=liner,
                name=f"{prefix}_bin_{index}_back",
            )
            door_part.visual(
                Box((bin_width, shelf_depth, shelf_thickness)),
                origin=Origin(xyz=(x_center, -shelf_depth / 2.0, bin_z - 0.026)),
                material=liner,
                name=f"{prefix}_bin_{index}_shelf",
            )
            door_part.visual(
                Box((bin_width, lip_thickness, lip_height)),
                origin=Origin(
                    xyz=(x_center, -shelf_depth - lip_thickness / 2.0, bin_z - 0.013)
                ),
                material=liner,
                name=f"{prefix}_bin_{index}_lip",
            )

    def add_door(door_name: str, axis_x: float, side_sign: float, hinge_axis: tuple[float, float, float]) -> None:
        door = model.part(door_name)
        slab_center_x = side_sign * (door_width / 2.0)
        handle_x = side_sign * (door_width - 0.028)
        hinge_leaf_center_x = side_sign * (hinge_leaf_width / 2.0)

        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(slab_center_x, door_thickness / 2.0, door_height / 2.0)),
            material=stainless,
            name="door_panel",
        )
        door.visual(
            Box((0.024, 0.012, 0.78)),
            origin=Origin(xyz=(handle_x, door_thickness + 0.026, door_height * 0.56)),
            material=stainless_dark,
            name="leading_handle_bar",
        )
        door.visual(
            Box((0.020, 0.020, 0.06)),
            origin=Origin(xyz=(handle_x, door_thickness + 0.010, door_height * 0.37)),
            material=stainless_dark,
            name="upper_handle_mount",
        )
        door.visual(
            Box((0.020, 0.020, 0.06)),
            origin=Origin(xyz=(handle_x, door_thickness + 0.010, door_height * 0.75)),
            material=stainless_dark,
            name="lower_handle_mount",
        )
        door.visual(
            Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
            origin=Origin(xyz=(hinge_leaf_center_x, hinge_leaf_depth / 2.0, top_hinge_local_z)),
            material=rail_metal,
            name="top_hinge_leaf",
        )
        door.visual(
            Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
            origin=Origin(xyz=(hinge_leaf_center_x, hinge_leaf_depth / 2.0, bottom_hinge_local_z)),
            material=rail_metal,
            name="bottom_hinge_leaf",
        )

        add_door_bins(door, slab_center_x, door_name)

        model.articulation(
            f"{door_name}_hinge",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=door,
            origin=Origin(xyz=(axis_x, axis_y, door_bottom_z)),
            axis=hinge_axis,
            motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.7),
        )

    add_door("left_door", left_axis_x, 1.0, (0.0, 0.0, 1.0))
    add_door("right_door", right_axis_x, -1.0, (0.0, 0.0, -1.0))

    freezer_drawer = model.part("freezer_drawer")
    drawer_body_width = 0.78
    drawer_body_depth = 0.48
    drawer_side_height = 0.21
    drawer_wall_thickness = 0.012
    drawer_front_inset = 0.0
    runner_center_x = drawer_body_width / 2.0 + runner_thickness / 2.0
    drawer_origin_x = -guide_track_x
    drawer_origin_y = axis_y - runner_length / 2.0
    drawer_origin_z = guide_center_z
    drawer_x_shift = -drawer_origin_x
    drawer_y_shift = axis_y - drawer_origin_y
    drawer_z_shift = drawer_bottom_z - drawer_origin_z

    freezer_drawer.visual(
        Box((width - (2.0 * side_reveal), drawer_front_thickness, drawer_front_height)),
        origin=Origin(
            xyz=(drawer_x_shift, drawer_front_thickness / 2.0 + drawer_y_shift, drawer_front_height / 2.0 + drawer_z_shift)
        ),
        material=stainless,
        name="drawer_front",
    )
    freezer_drawer.visual(
        Box((0.60, 0.014, 0.030)),
        origin=Origin(
            xyz=(drawer_x_shift, drawer_front_thickness + 0.026 + drawer_y_shift, drawer_front_height * 0.58 + drawer_z_shift)
        ),
        material=stainless_dark,
        name="drawer_handle_bar",
    )
    freezer_drawer.visual(
        Box((0.05, 0.020, 0.025)),
        origin=Origin(
            xyz=(-0.22 + drawer_x_shift, drawer_front_thickness + 0.010 + drawer_y_shift, drawer_front_height * 0.58 + drawer_z_shift)
        ),
        material=stainless_dark,
        name="left_drawer_handle_mount",
    )
    freezer_drawer.visual(
        Box((0.05, 0.020, 0.025)),
        origin=Origin(
            xyz=(0.22 + drawer_x_shift, drawer_front_thickness + 0.010 + drawer_y_shift, drawer_front_height * 0.58 + drawer_z_shift)
        ),
        material=stainless_dark,
        name="right_drawer_handle_mount",
    )
    freezer_drawer.visual(
        Box((drawer_body_width, drawer_wall_thickness, drawer_side_height)),
        origin=Origin(
            xyz=(
                drawer_x_shift,
                -(drawer_front_inset + drawer_body_depth - drawer_wall_thickness / 2.0) + drawer_y_shift,
                drawer_side_height / 2.0 + drawer_z_shift,
            )
        ),
        material=liner,
        name="drawer_back_wall",
    )
    freezer_drawer.visual(
        Box((drawer_body_width, drawer_body_depth, drawer_wall_thickness)),
        origin=Origin(
            xyz=(drawer_x_shift, -(drawer_front_inset + drawer_body_depth / 2.0) + drawer_y_shift, drawer_wall_thickness / 2.0 + drawer_z_shift)
        ),
        material=liner,
        name="drawer_bottom",
    )
    freezer_drawer.visual(
        Box((drawer_wall_thickness, drawer_body_depth, drawer_side_height)),
        origin=Origin(
            xyz=(
                -(drawer_body_width / 2.0 - drawer_wall_thickness / 2.0) + drawer_x_shift,
                -(drawer_front_inset + drawer_body_depth / 2.0) + drawer_y_shift,
                drawer_side_height / 2.0 + drawer_z_shift,
            )
        ),
        material=liner,
        name="left_drawer_side",
    )
    freezer_drawer.visual(
        Box((drawer_wall_thickness, drawer_body_depth, drawer_side_height)),
        origin=Origin(
            xyz=(
                (drawer_body_width / 2.0 - drawer_wall_thickness / 2.0) + drawer_x_shift,
                -(drawer_front_inset + drawer_body_depth / 2.0) + drawer_y_shift,
                drawer_side_height / 2.0 + drawer_z_shift,
            )
        ),
        material=liner,
        name="right_drawer_side",
    )
    freezer_drawer.visual(
        Box((runner_thickness, runner_length, runner_height)),
        origin=Origin(
            xyz=(-runner_center_x + drawer_x_shift, -(drawer_front_inset + runner_length / 2.0) + drawer_y_shift, guide_center_z + drawer_z_shift)
        ),
        material=rail_metal,
        name="left_runner",
    )
    freezer_drawer.visual(
        Box((runner_thickness, runner_length, runner_height)),
        origin=Origin(
            xyz=(runner_center_x + drawer_x_shift, -(drawer_front_inset + runner_length / 2.0) + drawer_y_shift, guide_center_z + drawer_z_shift)
        ),
        material=rail_metal,
        name="right_runner",
    )

    model.articulation(
        "freezer_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=freezer_drawer,
        origin=Origin(xyz=(drawer_origin_x, drawer_origin_y, drawer_origin_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.8, lower=0.0, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    freezer_drawer = object_model.get_part("freezer_drawer")

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    drawer_slide = object_model.get_articulation("freezer_drawer_slide")

    header_panel = cabinet.get_visual("top_header_panel")
    left_top_cab_hinge = cabinet.get_visual("left_top_hinge_leaf")
    left_bottom_cab_hinge = cabinet.get_visual("left_bottom_hinge_leaf")
    right_top_cab_hinge = cabinet.get_visual("right_top_hinge_leaf")
    right_bottom_cab_hinge = cabinet.get_visual("right_bottom_hinge_leaf")
    left_guide_plate = cabinet.get_visual("left_guide_plate")
    right_guide_plate = cabinet.get_visual("right_guide_plate")

    left_panel = left_door.get_visual("door_panel")
    right_panel = right_door.get_visual("door_panel")
    left_handle = left_door.get_visual("leading_handle_bar")
    right_handle = right_door.get_visual("leading_handle_bar")
    left_top_door_hinge = left_door.get_visual("top_hinge_leaf")
    left_bottom_door_hinge = left_door.get_visual("bottom_hinge_leaf")
    right_top_door_hinge = right_door.get_visual("top_hinge_leaf")
    right_bottom_door_hinge = right_door.get_visual("bottom_hinge_leaf")
    left_bin_mid_lip = left_door.get_visual("left_door_bin_2_lip")
    right_bin_mid_lip = right_door.get_visual("right_door_bin_2_lip")

    drawer_front = freezer_drawer.get_visual("drawer_front")
    drawer_handle = freezer_drawer.get_visual("drawer_handle_bar")
    left_runner = freezer_drawer.get_visual("left_runner")
    right_runner = freezer_drawer.get_visual("right_runner")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_gap(
        left_door,
        freezer_drawer,
        axis="z",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem=left_panel,
        negative_elem=drawer_front,
    )
    ctx.expect_gap(
        right_door,
        freezer_drawer,
        axis="z",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem=right_panel,
        negative_elem=drawer_front,
    )
    ctx.expect_gap(
        left_door,
        cabinet,
        axis="y",
        min_gap=0.004,
        max_gap=0.020,
        positive_elem=left_panel,
        negative_elem=header_panel,
    )
    ctx.expect_gap(
        right_door,
        cabinet,
        axis="y",
        min_gap=0.004,
        max_gap=0.020,
        positive_elem=right_panel,
        negative_elem=header_panel,
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.005,
        max_gap=0.050,
        positive_elem=right_panel,
        negative_elem=left_handle,
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.005,
        max_gap=0.050,
        positive_elem=right_handle,
        negative_elem=left_panel,
    )
    ctx.expect_contact(left_door, cabinet, elem_a=left_top_door_hinge, elem_b=left_top_cab_hinge)
    ctx.expect_contact(left_door, cabinet, elem_a=left_bottom_door_hinge, elem_b=left_bottom_cab_hinge)
    ctx.expect_contact(right_door, cabinet, elem_a=right_top_door_hinge, elem_b=right_top_cab_hinge)
    ctx.expect_contact(right_door, cabinet, elem_a=right_bottom_door_hinge, elem_b=right_bottom_cab_hinge)
    ctx.expect_contact(freezer_drawer, cabinet, elem_a=left_runner, elem_b=left_guide_plate)
    ctx.expect_contact(freezer_drawer, cabinet, elem_a=right_runner, elem_b=right_guide_plate)
    ctx.expect_gap(
        left_door,
        left_door,
        axis="y",
        min_gap=0.060,
        max_gap=0.090,
        positive_elem=left_panel,
        negative_elem=left_bin_mid_lip,
    )
    ctx.expect_gap(
        right_door,
        right_door,
        axis="y",
        min_gap=0.060,
        max_gap=0.090,
        positive_elem=right_panel,
        negative_elem=right_bin_mid_lip,
    )
    ctx.expect_gap(
        freezer_drawer,
        freezer_drawer,
        axis="y",
        min_gap=0.018,
        max_gap=0.050,
        positive_elem=drawer_handle,
        negative_elem=drawer_front,
    )

    with ctx.pose({left_hinge: 1.15}):
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.20,
            positive_elem=right_panel,
            negative_elem=left_panel,
        )
    with ctx.pose({right_hinge: 1.15}):
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.20,
            positive_elem=right_panel,
            negative_elem=left_panel,
        )
    with ctx.pose({drawer_slide: 0.28}):
        ctx.expect_contact(freezer_drawer, cabinet, elem_a=left_runner, elem_b=left_guide_plate)
        ctx.expect_contact(freezer_drawer, cabinet, elem_a=right_runner, elem_b=right_guide_plate)
        ctx.expect_gap(
            freezer_drawer,
            cabinet,
            axis="y",
            min_gap=0.25,
            positive_elem=drawer_front,
            negative_elem=header_panel,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
