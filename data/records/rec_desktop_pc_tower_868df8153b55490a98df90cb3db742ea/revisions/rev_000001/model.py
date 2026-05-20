from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="sff_living_room_pc_case")

    shell = model.material("shell", rgba=(0.10, 0.11, 0.12, 1.0))
    trim = model.material("trim", rgba=(0.18, 0.19, 0.20, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    case_w = 0.34
    case_d = 0.29
    wall_h = 0.092
    wall_t = 0.003
    bottom_t = 0.003

    lid_t = 0.003
    lid_side_drop = 0.018
    lid_front_drop = 0.016
    lid_rear_setback = 0.005
    hinge_r = 0.005
    rear_hinge_x = 0.110

    door_w = 0.092
    door_h = 0.044
    door_t = 0.0025
    door_center_x = 0.104
    door_center_z = 0.053
    door_left = door_center_x - door_w / 2.0
    door_right = door_center_x + door_w / 2.0
    door_bottom = door_center_z - door_h / 2.0
    door_top = door_center_z + door_h / 2.0

    chassis = model.part("chassis")
    chassis.visual(
        Box((case_w, case_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=shell,
        name="bottom_plate",
    )
    chassis.visual(
        Box((wall_t, case_d, wall_h - bottom_t)),
        origin=Origin(
            xyz=(
                -(case_w / 2.0 - wall_t / 2.0),
                0.0,
                bottom_t + (wall_h - bottom_t) / 2.0,
            )
        ),
        material=shell,
        name="left_wall",
    )
    chassis.visual(
        Box((wall_t, case_d, wall_h - bottom_t)),
        origin=Origin(
            xyz=(
                case_w / 2.0 - wall_t / 2.0,
                0.0,
                bottom_t + (wall_h - bottom_t) / 2.0,
            )
        ),
        material=shell,
        name="right_wall",
    )
    chassis.visual(
        Box((case_w - 2.0 * wall_t, wall_t, wall_h - bottom_t)),
        origin=Origin(
            xyz=(
                0.0,
                case_d / 2.0 - wall_t / 2.0,
                bottom_t + (wall_h - bottom_t) / 2.0,
            )
        ),
        material=shell,
        name="rear_wall",
    )
    chassis.visual(
        Box((door_left + case_w / 2.0, wall_t, wall_h - bottom_t)),
        origin=Origin(
            xyz=(
                (-case_w / 2.0 + door_left) / 2.0,
                -(case_d / 2.0 - wall_t / 2.0),
                bottom_t + (wall_h - bottom_t) / 2.0,
            )
        ),
        material=shell,
        name="front_left_face",
    )
    chassis.visual(
        Box((case_w / 2.0 - door_left, wall_t, wall_h - door_top)),
        origin=Origin(
            xyz=(
                (door_left + case_w / 2.0) / 2.0,
                -(case_d / 2.0 - wall_t / 2.0),
                door_top + (wall_h - door_top) / 2.0,
            )
        ),
        material=shell,
        name="front_upper_face",
    )
    chassis.visual(
        Box((case_w / 2.0 - door_left, wall_t, door_bottom - bottom_t)),
        origin=Origin(
            xyz=(
                (door_left + case_w / 2.0) / 2.0,
                -(case_d / 2.0 - wall_t / 2.0),
                bottom_t + (door_bottom - bottom_t) / 2.0,
            )
        ),
        material=shell,
        name="front_lower_face",
    )
    chassis.visual(
        Box((case_w / 2.0 - door_right, wall_t, door_h)),
        origin=Origin(
            xyz=(
                (door_right + case_w / 2.0) / 2.0,
                -(case_d / 2.0 - wall_t / 2.0),
                door_center_z,
            )
        ),
        material=trim,
        name="front_hinge_jamb",
    )
    for suffix, x_pos in (("left", -rear_hinge_x), ("right", rear_hinge_x)):
        chassis.visual(
            Box((0.028, hinge_r, hinge_r)),
            origin=Origin(xyz=(x_pos, case_d / 2.0 + hinge_r / 2.0, wall_h - hinge_r)),
            material=trim,
            name=f"rear_hinge_bracket_{suffix}",
        )
    for suffix, x_pos, y_pos in (
        ("front_left", -0.122, -0.100),
        ("front_right", 0.122, -0.100),
        ("rear_left", -0.122, 0.100),
        ("rear_right", 0.122, 0.100),
    ):
        chassis.visual(
            Box((0.028, 0.028, 0.006)),
            origin=Origin(xyz=(x_pos, y_pos, -0.003)),
            material=rubber,
            name=f"foot_{suffix}",
        )

    lid = model.part("top_lid")
    lid_cover_d = case_d + wall_t
    lid_front_y = -lid_rear_setback - lid_cover_d
    lid_center_y = -lid_rear_setback - lid_cover_d / 2.0
    lid_w = case_w + 2.0 * wall_t
    lid.visual(
        Box((lid_w, lid_cover_d, lid_t)),
        origin=Origin(xyz=(0.0, lid_center_y, lid_t / 2.0)),
        material=shell,
        name="lid_top",
    )
    lid.visual(
        Box((wall_t, lid_cover_d, lid_side_drop)),
        origin=Origin(
            xyz=(
                -(lid_w / 2.0 - wall_t / 2.0),
                lid_center_y,
                -lid_side_drop / 2.0,
            )
        ),
        material=shell,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((wall_t, lid_cover_d, lid_side_drop)),
        origin=Origin(
            xyz=(
                lid_w / 2.0 - wall_t / 2.0,
                lid_center_y,
                -lid_side_drop / 2.0,
            )
        ),
        material=shell,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((lid_w, wall_t, lid_front_drop)),
        origin=Origin(
            xyz=(0.0, lid_front_y + wall_t / 2.0, -lid_front_drop / 2.0)
        ),
        material=trim,
        name="lid_front_lip",
    )
    for suffix, x_pos in (("left", -rear_hinge_x), ("right", rear_hinge_x)):
        lid.visual(
            Box((0.030, 0.008, 0.005)),
            origin=Origin(xyz=(x_pos, -0.001, 0.0015)),
            material=hinge_metal,
            name=f"lid_hinge_leaf_{suffix}",
        )
        lid.visual(
            Cylinder(radius=hinge_r, length=0.022),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=hinge_metal,
            name=f"lid_hinge_knuckle_{suffix}",
        )

    front_panel = model.part("front_io_panel")
    front_panel.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(-door_w / 2.0, -door_t / 2.0, 0.0)),
        material=trim,
        name="io_door",
    )
    front_panel.visual(
        Box((0.004, 0.006, 0.032)),
        origin=Origin(xyz=(-0.002, -0.003, 0.0)),
        material=hinge_metal,
        name="io_hinge_leaf",
    )
    front_panel.visual(
        Cylinder(radius=0.004, length=0.032),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=hinge_metal,
        name="io_hinge_knuckle",
    )
    front_panel.visual(
        Box((0.012, 0.003, 0.018)),
        origin=Origin(xyz=(-door_w + 0.010, -door_t - 0.0015, 0.0)),
        material=shell,
        name="io_pull_tab",
    )

    model.articulation(
        "chassis_to_top_lid",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lid,
        origin=Origin(xyz=(0.0, case_d / 2.0 + hinge_r, wall_h)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "chassis_to_front_io_panel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_panel,
        origin=Origin(xyz=(door_right, -case_d / 2.0, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.3,
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

    chassis = object_model.get_part("chassis")
    lid = object_model.get_part("top_lid")
    front_panel = object_model.get_part("front_io_panel")
    lid_hinge = object_model.get_articulation("chassis_to_top_lid")
    panel_hinge = object_model.get_articulation("chassis_to_front_io_panel")

    lid.get_visual("lid_hinge_knuckle_left")
    lid.get_visual("lid_hinge_knuckle_right")
    front_panel.get_visual("io_hinge_knuckle")

    ctx.check(
        "prompt-critical parts exist",
        chassis is not None and lid is not None and front_panel is not None,
        details="Expected chassis, top_lid, and front_io_panel parts.",
    )

    with ctx.pose({lid_hinge: 0.0, panel_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            chassis,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="lid_top",
            name="lid sits on the chassis top edge",
        )
        ctx.expect_overlap(
            lid,
            chassis,
            axes="xy",
            min_overlap=0.24,
            name="lid covers the chassis footprint",
        )
        ctx.expect_gap(
            chassis,
            front_panel,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="front_hinge_jamb",
            negative_elem="io_door",
            name="front I/O door closes flush to the front opening",
        )
        ctx.expect_overlap(
            front_panel,
            chassis,
            axes="xz",
            min_overlap=0.03,
            elem_a="io_door",
            name="front I/O door sits within the front opening envelope",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_front_lip")
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_front_lip")
        ctx.check(
            "top lid opens upward from the rear edge",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[0][2] > closed_lid_aabb[0][2] + 0.09,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    closed_panel_aabb = ctx.part_element_world_aabb(front_panel, elem="io_door")
    with ctx.pose({panel_hinge: 1.05}):
        open_panel_aabb = ctx.part_element_world_aabb(front_panel, elem="io_door")
        ctx.check(
            "front I/O panel swings outward on its side hinge",
            closed_panel_aabb is not None
            and open_panel_aabb is not None
            and open_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.03,
            details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
