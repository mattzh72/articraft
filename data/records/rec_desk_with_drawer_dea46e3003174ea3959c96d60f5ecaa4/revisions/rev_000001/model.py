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
    model = ArticulatedObject(name="sewing_cabinet_desk")

    wood = model.material("walnut_veneer", rgba=(0.45, 0.31, 0.19, 1.0))
    wood_dark = model.material("walnut_dark", rgba=(0.34, 0.22, 0.13, 1.0))
    steel = model.material("brushed_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    brass = model.material("aged_brass", rgba=(0.72, 0.61, 0.33, 1.0))

    body_width = 0.88
    body_depth = 0.54
    body_height = 0.79
    wall_t = 0.022
    top_t = 0.03
    bottom_t = 0.03
    back_t = 0.012
    shelf_t = 0.022

    front_x = body_depth / 2.0
    clear_width = body_width - 2.0 * wall_t
    case_depth = body_depth - back_t
    case_center_x = back_t / 2.0

    lower_open_low = 0.045
    drawer_open_h = 0.185
    upper_open_low = lower_open_low + drawer_open_h + shelf_t
    leaf_bottom_z = upper_open_low + drawer_open_h + shelf_t
    leaf_open_h = body_height - top_t - leaf_bottom_z - 0.008

    lower_open_center = lower_open_low + drawer_open_h / 2.0
    upper_open_center = upper_open_low + drawer_open_h / 2.0

    body = model.part("body")
    body.visual(
        Box((body_depth, wall_t, body_height - top_t)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall_t / 2.0, (body_height - top_t) / 2.0)),
        material=wood,
        name="left_side",
    )
    body.visual(
        Box((body_depth, wall_t, body_height - top_t)),
        origin=Origin(xyz=(0.0, -(body_width / 2.0 - wall_t / 2.0), (body_height - top_t) / 2.0)),
        material=wood,
        name="right_side",
    )
    body.visual(
        Box((body_depth, body_width, top_t)),
        origin=Origin(xyz=(0.0, 0.0, body_height - top_t / 2.0)),
        material=wood_dark,
        name="top",
    )
    body.visual(
        Box((case_depth, clear_width, bottom_t)),
        origin=Origin(xyz=(case_center_x, 0.0, bottom_t / 2.0)),
        material=wood,
        name="bottom",
    )
    body.visual(
        Box((back_t, clear_width, body_height - top_t - bottom_t)),
        origin=Origin(
            xyz=(
                -body_depth / 2.0 + back_t / 2.0,
                0.0,
                bottom_t + (body_height - top_t - bottom_t) / 2.0,
            )
        ),
        material=wood_dark,
        name="back_panel",
    )
    body.visual(
        Box((case_depth, clear_width, shelf_t)),
        origin=Origin(xyz=(case_center_x, 0.0, lower_open_low + drawer_open_h + shelf_t / 2.0)),
        material=wood,
        name="drawer_mid_shelf",
    )
    body.visual(
        Box((case_depth, clear_width, shelf_t)),
        origin=Origin(xyz=(case_center_x, 0.0, leaf_bottom_z - shelf_t / 2.0)),
        material=wood,
        name="machine_compartment_floor",
    )
    body.visual(
        Box((0.26, clear_width, 0.02)),
        origin=Origin(xyz=(-0.10, 0.0, 0.60)),
        material=wood_dark,
        name="machine_bed",
    )

    runner_len = 0.48
    runner_y = body_width / 2.0 - wall_t - 0.008
    runner_t = 0.016
    runner_h = 0.014
    runner_center_x = -0.01
    for visual_name, y_sign, z_center in (
        ("lower_left_runner", 1.0, lower_open_center),
        ("lower_right_runner", -1.0, lower_open_center),
        ("upper_left_runner", 1.0, upper_open_center),
        ("upper_right_runner", -1.0, upper_open_center),
    ):
        body.visual(
            Box((runner_len, runner_t, runner_h)),
            origin=Origin(xyz=(runner_center_x, y_sign * runner_y, z_center)),
            material=steel,
            name=visual_name,
        )

    leaf = model.part("leaf")
    leaf_width = clear_width - 0.012
    leaf_thickness = 0.024
    leaf.visual(
        Box((leaf_thickness, leaf_width, leaf_open_h)),
        origin=Origin(xyz=(-leaf_thickness / 2.0, 0.0, -leaf_open_h / 2.0)),
        material=wood_dark,
        name="leaf_panel",
    )
    leaf.visual(
        Cylinder(radius=0.006, length=leaf_width * 0.94),
        origin=Origin(xyz=(0.006, 0.0, -0.006), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="leaf_hinge_barrel",
    )
    leaf.visual(
        Box((0.008, leaf_width * 0.96, 0.012)),
        origin=Origin(xyz=(0.003, 0.0, -0.008)),
        material=brass,
        name="leaf_hinge_plate",
    )
    leaf.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(
            xyz=(0.009, 0.0, -leaf_open_h * 0.62),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=brass,
        name="leaf_pull",
    )

    drawer_depth = 0.41
    drawer_box_width = 0.776
    drawer_front_width = 0.796
    drawer_front_t = 0.018
    drawer_front_h = 0.176
    drawer_side_t = 0.012
    drawer_back_t = 0.012
    drawer_bottom_t = 0.01
    drawer_box_h = 0.14
    drawer_side_len = drawer_depth - drawer_front_t - drawer_back_t
    slide_member_len = 0.32
    slide_member_t = 0.014
    slide_member_h = 0.012

    def add_drawer(name: str) -> None:
        drawer = model.part(name)
        drawer.visual(
            Box((drawer_front_t, drawer_front_width, drawer_front_h)),
            origin=Origin(xyz=(-drawer_front_t / 2.0, 0.0, drawer_front_h / 2.0)),
            material=wood_dark,
            name="drawer_front",
        )
        drawer.visual(
            Box((drawer_side_len, drawer_side_t, drawer_box_h)),
            origin=Origin(
                xyz=(
                    -drawer_front_t - drawer_side_len / 2.0,
                    drawer_box_width / 2.0 - drawer_side_t / 2.0,
                    drawer_box_h / 2.0,
                )
            ),
            material=wood,
            name="left_side",
        )
        drawer.visual(
            Box((drawer_side_len, drawer_side_t, drawer_box_h)),
            origin=Origin(
                xyz=(
                    -drawer_front_t - drawer_side_len / 2.0,
                    -(drawer_box_width / 2.0 - drawer_side_t / 2.0),
                    drawer_box_h / 2.0,
                )
            ),
            material=wood,
            name="right_side",
        )
        drawer.visual(
            Box((drawer_back_t, drawer_box_width, drawer_box_h)),
            origin=Origin(
                xyz=(-drawer_depth + drawer_back_t / 2.0, 0.0, drawer_box_h / 2.0)
            ),
            material=wood,
            name="back",
        )
        drawer.visual(
            Box((drawer_side_len, drawer_box_width - 2.0 * drawer_side_t, drawer_bottom_t)),
            origin=Origin(
                xyz=(
                    -drawer_front_t - drawer_side_len / 2.0,
                    0.0,
                    drawer_bottom_t / 2.0,
                )
            ),
            material=wood,
            name="bottom",
        )
        drawer.visual(
            Box((slide_member_len, slide_member_t, slide_member_h)),
            origin=Origin(
                xyz=(
                    -0.25,
                    drawer_box_width / 2.0 + slide_member_t / 2.0,
                    0.092,
                )
            ),
            material=steel,
            name="left_slide_member",
        )
        drawer.visual(
            Box((slide_member_len, slide_member_t, slide_member_h)),
            origin=Origin(
                xyz=(
                    -0.25,
                    -(drawer_box_width / 2.0 + slide_member_t / 2.0),
                    0.092,
                )
            ),
            material=steel,
            name="right_slide_member",
        )
        drawer.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(
                xyz=(0.009, 0.0, drawer_front_h * 0.57),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
            name="pull_knob",
        )

    add_drawer("upper_drawer")
    add_drawer("lower_drawer")

    model.articulation(
        "body_to_leaf",
        ArticulationType.REVOLUTE,
        parent=body,
        child=leaf,
        origin=Origin(xyz=(front_x, 0.0, leaf_bottom_z + leaf_open_h)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.5,
            lower=0.0,
            upper=pi / 2.0,
        ),
    )
    model.articulation(
        "body_to_upper_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=model.get_part("upper_drawer"),
        origin=Origin(xyz=(front_x, 0.0, upper_open_low)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.28,
        ),
    )
    model.articulation(
        "body_to_lower_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=model.get_part("lower_drawer"),
        origin=Origin(xyz=(front_x, 0.0, lower_open_low)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.28,
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

    body = object_model.get_part("body")
    leaf = object_model.get_part("leaf")
    upper_drawer = object_model.get_part("upper_drawer")
    lower_drawer = object_model.get_part("lower_drawer")
    leaf_hinge = object_model.get_articulation("body_to_leaf")
    upper_slide = object_model.get_articulation("body_to_upper_drawer")
    lower_slide = object_model.get_articulation("body_to_lower_drawer")

    leaf_panel = leaf.get_visual("leaf_panel")
    upper_left_slide = upper_drawer.get_visual("left_slide_member")
    lower_left_slide = lower_drawer.get_visual("left_slide_member")
    upper_left_runner = body.get_visual("upper_left_runner")
    lower_left_runner = body.get_visual("lower_left_runner")

    ctx.expect_within(
        upper_drawer,
        body,
        axes="yz",
        margin=0.0,
        name="upper drawer stays inside cabinet width and height at rest",
    )
    ctx.expect_within(
        lower_drawer,
        body,
        axes="yz",
        margin=0.0,
        name="lower drawer stays inside cabinet width and height at rest",
    )
    ctx.expect_contact(
        upper_drawer,
        body,
        elem_a=upper_left_slide,
        elem_b=upper_left_runner,
        name="upper drawer is supported by its guide rail at rest",
    )
    ctx.expect_contact(
        lower_drawer,
        body,
        elem_a=lower_left_slide,
        elem_b=lower_left_runner,
        name="lower drawer is supported by its guide rail at rest",
    )

    leaf_panel_aabb = ctx.part_element_world_aabb(leaf, elem=leaf_panel)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "leaf panel closes flush with cabinet front",
        leaf_panel_aabb is not None
        and body_aabb is not None
        and abs(leaf_panel_aabb[1][0] - body_aabb[1][0]) <= 0.002,
        details=f"leaf_panel_aabb={leaf_panel_aabb}, body_aabb={body_aabb}",
    )

    upper_rest = ctx.part_world_position(upper_drawer)
    lower_rest = ctx.part_world_position(lower_drawer)
    closed_leaf_aabb = ctx.part_element_world_aabb(leaf, elem=leaf_panel)

    with ctx.pose({leaf_hinge: pi / 2.0}):
        open_leaf_aabb = ctx.part_element_world_aabb(leaf, elem=leaf_panel)
        ctx.check(
            "leaf folds forward into a work surface",
            closed_leaf_aabb is not None
            and open_leaf_aabb is not None
            and (open_leaf_aabb[1][0] - open_leaf_aabb[0][0]) > 0.22
            and (closed_leaf_aabb[1][2] - closed_leaf_aabb[0][2]) > 0.22
            and (open_leaf_aabb[1][2] - open_leaf_aabb[0][2]) < 0.04
            and open_leaf_aabb[1][0] > closed_leaf_aabb[1][0] + 0.20,
            details=f"closed={closed_leaf_aabb}, open={open_leaf_aabb}",
        )

    with ctx.pose({upper_slide: 0.28, lower_slide: 0.28}):
        upper_extended = ctx.part_world_position(upper_drawer)
        lower_extended = ctx.part_world_position(lower_drawer)
        ctx.expect_within(
            upper_drawer,
            body,
            axes="yz",
            margin=0.0,
            name="upper drawer stays aligned in the opening when extended",
        )
        ctx.expect_within(
            lower_drawer,
            body,
            axes="yz",
            margin=0.0,
            name="lower drawer stays aligned in the opening when extended",
        )
        ctx.expect_overlap(
            upper_drawer,
            body,
            axes="x",
            min_overlap=0.06,
            elem_a=upper_left_slide,
            elem_b=upper_left_runner,
            name="upper drawer retains insertion at full extension",
        )
        ctx.expect_overlap(
            lower_drawer,
            body,
            axes="x",
            min_overlap=0.06,
            elem_a=lower_left_slide,
            elem_b=lower_left_runner,
            name="lower drawer retains insertion at full extension",
        )
        ctx.check(
            "upper drawer slides outward",
            upper_rest is not None
            and upper_extended is not None
            and upper_extended[0] > upper_rest[0] + 0.25,
            details=f"rest={upper_rest}, extended={upper_extended}",
        )
        ctx.check(
            "lower drawer slides outward",
            lower_rest is not None
            and lower_extended is not None
            and lower_extended[0] > lower_rest[0] + 0.25,
            details=f"rest={lower_rest}, extended={lower_extended}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
