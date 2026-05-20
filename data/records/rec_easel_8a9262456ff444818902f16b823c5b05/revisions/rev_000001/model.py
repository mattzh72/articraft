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
    model = ArticulatedObject(name="wall_mounted_fold_flat_display_easel")

    plate_black = model.material("plate_black", rgba=(0.14, 0.15, 0.16, 1.0))
    arm_black = model.material("arm_black", rgba=(0.10, 0.11, 0.12, 1.0))
    pad_charcoal = model.material("pad_charcoal", rgba=(0.20, 0.20, 0.21, 1.0))

    plate_width = 0.280
    plate_height = 0.420
    plate_thickness = 0.016
    arm_y_offset = 0.105
    arm_hinge_z = 0.105
    hinge_axis_x = 0.010
    arm_length = 0.230
    shelf_width = arm_y_offset * 2.0
    shelf_depth = 0.048

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((plate_thickness, plate_width, plate_height)),
        material=plate_black,
        name="plate_shell",
    )
    for side_name, side_y in (("left", -arm_y_offset), ("right", arm_y_offset)):
        for cheek_index, cheek_offset in enumerate((-0.008, 0.008)):
            wall_plate.visual(
                Box((0.008, 0.006, 0.020)),
                origin=Origin(xyz=(0.012, side_y + cheek_offset, arm_hinge_z)),
                material=plate_black,
                name=f"{side_name}_hinge_cheek_{cheek_index}",
            )
    for cheek_index, cheek_offset in enumerate((-0.019, 0.019)):
        wall_plate.visual(
            Box((0.008, 0.014, 0.018)),
            origin=Origin(xyz=(0.012, cheek_offset, 0.210)),
            material=plate_black,
            name=f"top_clip_cheek_{cheek_index}",
        )
    wall_plate.inertial = Inertial.from_geometry(
        Box((plate_thickness, plate_width, plate_height)),
        mass=1.8,
    )

    def _build_support_arm(part_name: str) -> None:
        arm = model.part(part_name)
        arm.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=arm_black,
            name="hinge_barrel",
        )
        arm.visual(
            Box((0.010, 0.018, arm_length)),
            origin=Origin(xyz=(0.005, 0.0, -arm_length / 2.0)),
            material=arm_black,
            name="arm_bar",
        )
        arm.visual(
            Box((0.014, 0.024, 0.016)),
            origin=Origin(xyz=(0.007, 0.0, -0.222)),
            material=arm_black,
            name="tip_block",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.014, 0.024, arm_length)),
            mass=0.22,
            origin=Origin(xyz=(0.007, 0.0, -arm_length / 2.0)),
        )

    _build_support_arm("left_arm")
    _build_support_arm("right_arm")

    ledge_shelf = model.part("ledge_shelf")
    ledge_shelf.visual(
        Box((0.010, shelf_width, shelf_depth)),
        origin=Origin(xyz=(0.005, shelf_width / 2.0, -shelf_depth / 2.0)),
        material=arm_black,
        name="deck_panel",
    )
    ledge_shelf.visual(
        Box((0.014, shelf_width, 0.006)),
        origin=Origin(xyz=(0.007, shelf_width / 2.0, -shelf_depth + 0.003)),
        material=arm_black,
        name="front_lip",
    )
    ledge_shelf.inertial = Inertial.from_geometry(
        Box((0.014, shelf_width, shelf_depth)),
        mass=0.28,
        origin=Origin(xyz=(0.007, shelf_width / 2.0, -shelf_depth / 2.0)),
    )

    top_clip = model.part("top_clip")
    top_clip.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_black,
        name="hinge_barrel",
    )
    top_clip.visual(
        Box((0.010, 0.022, 0.180)),
        origin=Origin(xyz=(0.005, 0.0, -0.090)),
        material=arm_black,
        name="clip_strip",
    )
    top_clip.visual(
        Box((0.018, 0.070, 0.012)),
        origin=Origin(xyz=(0.009, 0.0, -0.168)),
        material=pad_charcoal,
        name="pressure_pad",
    )
    top_clip.inertial = Inertial.from_geometry(
        Box((0.018, 0.050, 0.180)),
        mass=0.16,
        origin=Origin(xyz=(0.009, 0.0, -0.090)),
    )

    left_arm = model.get_part("left_arm")
    right_arm = model.get_part("right_arm")

    model.articulation(
        "wall_plate_to_left_arm",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=left_arm,
        origin=Origin(xyz=(hinge_axis_x, -arm_y_offset, arm_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "wall_plate_to_right_arm",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=right_arm,
        origin=Origin(xyz=(hinge_axis_x, arm_y_offset, arm_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "left_arm_to_ledge_shelf",
        ArticulationType.REVOLUTE,
        parent=left_arm,
        child=ledge_shelf,
        origin=Origin(xyz=(0.0, 0.0, -arm_length)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )
    model.articulation(
        "wall_plate_to_top_clip",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=top_clip,
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.210)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    ledge_shelf = object_model.get_part("ledge_shelf")
    top_clip = object_model.get_part("top_clip")

    left_arm_hinge = object_model.get_articulation("wall_plate_to_left_arm")
    right_arm_hinge = object_model.get_articulation("wall_plate_to_right_arm")
    shelf_hinge = object_model.get_articulation("left_arm_to_ledge_shelf")
    clip_hinge = object_model.get_articulation("wall_plate_to_top_clip")

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

    ctx.expect_origin_distance(
        left_arm,
        right_arm,
        axes="y",
        min_dist=0.20,
        max_dist=0.22,
        name="support arm hinges are spaced across the wall plate",
    )
    ctx.expect_gap(
        left_arm,
        wall_plate,
        axis="x",
        positive_elem="arm_bar",
        negative_elem="plate_shell",
        min_gap=0.0015,
        max_gap=0.003,
        name="left arm folds flat just off the plate face",
    )
    ctx.expect_gap(
        right_arm,
        wall_plate,
        axis="x",
        positive_elem="arm_bar",
        negative_elem="plate_shell",
        min_gap=0.0015,
        max_gap=0.003,
        name="right arm folds flat just off the plate face",
    )
    ctx.expect_gap(
        ledge_shelf,
        wall_plate,
        axis="x",
        positive_elem="deck_panel",
        negative_elem="plate_shell",
        min_gap=0.0015,
        max_gap=0.0035,
        name="shelf folds flat just off the plate face",
    )
    ctx.expect_gap(
        top_clip,
        wall_plate,
        axis="x",
        positive_elem="clip_strip",
        negative_elem="plate_shell",
        min_gap=0.0015,
        max_gap=0.003,
        name="top clip folds flat just off the plate face",
    )
    ctx.expect_contact(
        top_clip,
        wall_plate,
        elem_a="hinge_barrel",
        elem_b="top_clip_cheek_0",
        name="top clip hinge barrel is carried by the wall plate cheeks",
    )

    rest_left_tip = ctx.part_element_world_aabb(left_arm, elem="tip_block")
    rest_right_tip = ctx.part_element_world_aabb(right_arm, elem="tip_block")
    rest_clip_pad = ctx.part_element_world_aabb(top_clip, elem="pressure_pad")
    rest_shelf_deck = ctx.part_element_world_aabb(ledge_shelf, elem="deck_panel")

    deployed_arm_angle = math.radians(66.0)
    deployed_shelf_angle = (math.pi / 2.0) - deployed_arm_angle
    deployed_clip_angle = math.radians(20.0)

    with ctx.pose(
        {
            left_arm_hinge: deployed_arm_angle,
            right_arm_hinge: deployed_arm_angle,
            shelf_hinge: deployed_shelf_angle,
            clip_hinge: deployed_clip_angle,
        }
    ):
        ctx.expect_gap(
            ledge_shelf,
            wall_plate,
            axis="x",
            positive_elem="deck_panel",
            negative_elem="plate_shell",
            min_gap=0.19,
            name="open ledge projects forward to support displayed items",
        )
        ctx.expect_overlap(
            ledge_shelf,
            right_arm,
            axes="y",
            elem_a="deck_panel",
            elem_b="tip_block",
            min_overlap=0.010,
            name="ledge spans across to the right support arm",
        )

        open_left_tip = ctx.part_element_world_aabb(left_arm, elem="tip_block")
        open_right_tip = ctx.part_element_world_aabb(right_arm, elem="tip_block")
        open_clip_pad = ctx.part_element_world_aabb(top_clip, elem="pressure_pad")
        open_shelf_deck = ctx.part_element_world_aabb(ledge_shelf, elem="deck_panel")

    ctx.check(
        "left support arm swings forward when deployed",
        rest_left_tip is not None
        and open_left_tip is not None
        and open_left_tip[0][0] > rest_left_tip[0][0] + 0.16,
        details=f"rest={rest_left_tip}, open={open_left_tip}",
    )
    ctx.check(
        "right support arm swings forward when deployed",
        rest_right_tip is not None
        and open_right_tip is not None
        and open_right_tip[0][0] > rest_right_tip[0][0] + 0.16,
        details=f"rest={rest_right_tip}, open={open_right_tip}",
    )
    ctx.check(
        "top clip pad swings forward from the upper edge",
        rest_clip_pad is not None
        and open_clip_pad is not None
        and open_clip_pad[0][0] > rest_clip_pad[0][0] + 0.045,
        details=f"rest={rest_clip_pad}, open={open_clip_pad}",
    )
    ctx.check(
        "deployed shelf stays nearly level",
        rest_shelf_deck is not None
        and open_shelf_deck is not None
        and (open_shelf_deck[1][2] - open_shelf_deck[0][2]) < 0.020,
        details=f"rest={rest_shelf_deck}, open={open_shelf_deck}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
