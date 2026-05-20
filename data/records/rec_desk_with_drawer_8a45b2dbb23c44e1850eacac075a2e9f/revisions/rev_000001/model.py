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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="campaign_desk")

    walnut = model.material("walnut", rgba=(0.38, 0.24, 0.13, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.28, 0.18, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.64, 0.33, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.19, 0.19, 0.20, 1.0))

    height = 0.76
    main_width = 0.72
    top_depth = 0.50
    top_thickness = 0.028
    leaf_width = 0.24
    leg_size = 0.045

    top_center_z = height - top_thickness / 2.0
    leg_height = height - top_thickness
    apron_height = 0.11
    apron_center_z = top_center_z - top_thickness / 2.0 - apron_height / 2.0

    base = model.part("desk_base")
    base.visual(
        Box((main_width, top_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_center_z)),
        material=walnut,
        name="main_top",
    )

    leg_x = 0.305
    leg_y = 0.205
    for name, x_pos, y_pos in (
        ("front_left_leg", -leg_x, leg_y),
        ("front_right_leg", leg_x, leg_y),
        ("rear_left_leg", -leg_x, -leg_y),
        ("rear_right_leg", leg_x, -leg_y),
    ):
        base.visual(
            Box((leg_size, leg_size, leg_height)),
            origin=Origin(xyz=(x_pos, y_pos, leg_height / 2.0)),
            material=dark_walnut,
            name=name,
        )
        base.visual(
            Box((leg_size + 0.008, leg_size + 0.008, 0.050)),
            origin=Origin(xyz=(x_pos, y_pos, leg_height - 0.025)),
            material=brass,
            name=f"{name}_cap",
        )

    front_frame_y = top_depth / 2.0 - 0.011
    back_frame_y = -front_frame_y
    opening_width = 0.392
    side_apron_width = 0.095
    hinge_axis_z = top_center_z - 0.020

    base.visual(
        Box((side_apron_width, 0.022, apron_height)),
        origin=Origin(
            xyz=(
                -(opening_width / 2.0 + side_apron_width / 2.0),
                front_frame_y,
                apron_center_z,
            )
        ),
        material=dark_walnut,
        name="front_left_apron",
    )
    base.visual(
        Box((side_apron_width, 0.022, apron_height)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 + side_apron_width / 2.0,
                front_frame_y,
                apron_center_z,
            )
        ),
        material=dark_walnut,
        name="front_right_apron",
    )
    base.visual(
        Box((opening_width, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, front_frame_y, height - 0.043)),
        material=dark_walnut,
        name="drawer_web",
    )
    base.visual(
        Box((0.565, 0.022, apron_height)),
        origin=Origin(xyz=(0.0, back_frame_y, apron_center_z)),
        material=dark_walnut,
        name="back_apron",
    )

    inner_frame_height = 0.080
    inner_frame_center_z = 0.662
    inner_frame_width = 0.022
    inner_frame_depth = 0.478
    inner_frame_x = opening_width / 2.0 + inner_frame_width / 2.0
    for name, x_pos in (("left_inner_frame", -inner_frame_x), ("right_inner_frame", inner_frame_x)):
        base.visual(
            Box((inner_frame_width, inner_frame_depth, inner_frame_height)),
            origin=Origin(xyz=(x_pos, -0.011, inner_frame_center_z)),
            material=dark_walnut,
            name=name,
        )

    guide_rail_width = 0.120
    guide_rail_height = 0.014
    guide_rail_depth = 0.456
    guide_rail_center_z = 0.623
    guide_rail_x = 0.115
    for name, x_pos in (("left_guide_rail", -guide_rail_x), ("right_guide_rail", guide_rail_x)):
        base.visual(
            Box((guide_rail_width, guide_rail_depth, guide_rail_height)),
            origin=Origin(xyz=(x_pos, 0.0, guide_rail_center_z)),
            material=blackened_steel,
            name=name,
        )

    hinge_radius = 0.006
    hinge_lengths = 0.070
    for side, x_pos in (("left", -main_width / 2.0), ("right", main_width / 2.0)):
        for idx, y_pos in enumerate((-0.090, 0.090), start=1):
            base.visual(
                Cylinder(radius=hinge_radius, length=hinge_lengths),
                origin=Origin(
                    xyz=(x_pos, y_pos, hinge_axis_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=brass,
                name=f"{side}_hinge_knuckle_{idx}",
            )

    base.inertial = Inertial.from_geometry(
        Box((1.24, 0.58, 0.80)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
    )

    left_leaf = model.part("left_leaf")
    left_leaf.visual(
        Box((leaf_width, top_depth, top_thickness)),
        origin=Origin(xyz=(-leaf_width / 2.0, 0.0, top_center_z - hinge_axis_z)),
        material=walnut,
        name="left_leaf_panel",
    )
    for idx, y_pos in enumerate((-0.180, 0.0, 0.180), start=1):
        left_leaf.visual(
            Cylinder(radius=hinge_radius, length=0.070),
            origin=Origin(
                xyz=(0.0, y_pos, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"left_leaf_hinge_{idx}",
        )
    left_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, top_depth, top_thickness)),
        mass=5.5,
        origin=Origin(xyz=(-leaf_width / 2.0, 0.0, top_center_z - hinge_axis_z)),
    )

    right_leaf = model.part("right_leaf")
    right_leaf.visual(
        Box((leaf_width, top_depth, top_thickness)),
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, top_center_z - hinge_axis_z)),
        material=walnut,
        name="right_leaf_panel",
    )
    for idx, y_pos in enumerate((-0.180, 0.0, 0.180), start=1):
        right_leaf.visual(
            Cylinder(radius=hinge_radius, length=0.070),
            origin=Origin(
                xyz=(0.0, y_pos, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"right_leaf_hinge_{idx}",
        )
    right_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, top_depth, top_thickness)),
        mass=5.5,
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, top_center_z - hinge_axis_z)),
    )

    drawer = model.part("center_drawer")
    drawer_front_width = 0.382
    drawer_front_height = 0.074
    drawer_front_thickness = 0.022
    drawer_outer_width = 0.336
    drawer_body_depth = 0.332
    drawer_wall_thickness = 0.012
    drawer_body_height = 0.060
    drawer_bottom_thickness = 0.006

    drawer.visual(
        Box((drawer_front_width, drawer_front_thickness, drawer_front_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=walnut,
        name="drawer_front",
    )
    drawer.visual(
        Box((drawer_outer_width, drawer_body_depth, drawer_bottom_thickness)),
        origin=Origin(xyz=(0.0, -0.177, -0.029)),
        material=dark_walnut,
        name="drawer_bottom",
    )
    for name, x_pos in (
        ("left_drawer_side", -(drawer_outer_width / 2.0 - drawer_wall_thickness / 2.0)),
        ("right_drawer_side", drawer_outer_width / 2.0 - drawer_wall_thickness / 2.0),
    ):
        drawer.visual(
            Box((drawer_wall_thickness, drawer_body_depth, drawer_body_height)),
            origin=Origin(xyz=(x_pos, -0.177, -0.004)),
            material=dark_walnut,
            name=name,
        )
    drawer.visual(
        Box((drawer_outer_width - 2.0 * drawer_wall_thickness, drawer_wall_thickness, drawer_body_height)),
        origin=Origin(
            xyz=(0.0, -(0.177 + drawer_body_depth / 2.0 + drawer_wall_thickness / 2.0 - 0.166), -0.004)
        ),
        material=dark_walnut,
        name="drawer_back",
    )
    for name, x_pos in (("left_knob", -0.105), ("right_knob", 0.105)):
        drawer.visual(
            Cylinder(radius=0.0075, length=0.018),
            origin=Origin(
                xyz=(x_pos, 0.020, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=name,
        )
        drawer.visual(
            Sphere(radius=0.010),
            origin=Origin(xyz=(x_pos, 0.032, 0.0)),
            material=brass,
            name=f"{name}_grip",
        )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_front_width, 0.370, drawer_front_height)),
        mass=3.5,
        origin=Origin(xyz=(0.0, -0.160, -0.004)),
    )

    model.articulation(
        "base_to_left_leaf",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_leaf,
        origin=Origin(xyz=(-main_width / 2.0, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.5,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "base_to_right_leaf",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_leaf,
        origin=Origin(xyz=(main_width / 2.0, 0.0, hinge_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.5,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "base_to_center_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(0.0, front_frame_y, 0.663)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.30,
            lower=0.0,
            upper=0.220,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("desk_base")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    drawer = object_model.get_part("center_drawer")

    left_hinge = object_model.get_articulation("base_to_left_leaf")
    right_hinge = object_model.get_articulation("base_to_right_leaf")
    drawer_slide = object_model.get_articulation("base_to_center_drawer")

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_gap(
            base,
            left_leaf,
            axis="x",
            positive_elem="main_top",
            negative_elem="left_leaf_panel",
            min_gap=0.0,
            max_gap=0.001,
            name="left leaf closes tightly to the main top",
        )
        ctx.expect_gap(
            right_leaf,
            base,
            axis="x",
            positive_elem="right_leaf_panel",
            negative_elem="main_top",
            min_gap=0.0,
            max_gap=0.001,
            name="right leaf closes tightly to the main top",
        )
        ctx.expect_overlap(
            drawer,
            base,
            axes="xy",
            elem_a="drawer_bottom",
            elem_b="left_guide_rail",
            min_overlap=0.030,
            name="drawer bottom stays aligned above the left guide rail",
        )
        ctx.expect_overlap(
            drawer,
            base,
            axes="xy",
            elem_a="drawer_bottom",
            elem_b="right_guide_rail",
            min_overlap=0.030,
            name="drawer bottom stays aligned above the right guide rail",
        )
        ctx.expect_gap(
            drawer,
            base,
            axis="z",
            positive_elem="drawer_bottom",
            negative_elem="left_guide_rail",
            min_gap=0.0005,
            max_gap=0.0020,
            name="drawer bottom clears the left guide rail with a shallow running gap",
        )
        ctx.expect_gap(
            drawer,
            base,
            axis="z",
            positive_elem="drawer_bottom",
            negative_elem="right_guide_rail",
            min_gap=0.0005,
            max_gap=0.0020,
            name="drawer bottom clears the right guide rail with a shallow running gap",
        )

    left_rest_aabb = ctx.part_element_world_aabb(left_leaf, elem="left_leaf_panel")
    right_rest_aabb = ctx.part_element_world_aabb(right_leaf, elem="right_leaf_panel")
    drawer_closed_pos = ctx.part_world_position(drawer)

    with ctx.pose({left_hinge: 1.45, right_hinge: 1.45, drawer_slide: 0.220}):
        ctx.expect_overlap(
            drawer,
            base,
            axes="y",
            elem_a="drawer_bottom",
            elem_b="left_guide_rail",
            min_overlap=0.100,
            name="extended drawer keeps retained insertion on the left rail",
        )
        ctx.expect_overlap(
            drawer,
            base,
            axes="y",
            elem_a="drawer_bottom",
            elem_b="right_guide_rail",
            min_overlap=0.100,
            name="extended drawer keeps retained insertion on the right rail",
        )

        left_folded_aabb = ctx.part_element_world_aabb(left_leaf, elem="left_leaf_panel")
        right_folded_aabb = ctx.part_element_world_aabb(right_leaf, elem="right_leaf_panel")
        drawer_open_pos = ctx.part_world_position(drawer)

    ctx.check(
        "left leaf folds downward beside the desk",
        left_rest_aabb is not None
        and left_folded_aabb is not None
        and left_folded_aabb[0][2] < left_rest_aabb[0][2] - 0.15,
        details=f"rest={left_rest_aabb}, folded={left_folded_aabb}",
    )
    ctx.check(
        "right leaf folds downward beside the desk",
        right_rest_aabb is not None
        and right_folded_aabb is not None
        and right_folded_aabb[0][2] < right_rest_aabb[0][2] - 0.15,
        details=f"rest={right_rest_aabb}, folded={right_folded_aabb}",
    )
    ctx.check(
        "drawer slides outward from the desk front",
        drawer_closed_pos is not None
        and drawer_open_pos is not None
        and drawer_open_pos[1] > drawer_closed_pos[1] + 0.18,
        details=f"closed={drawer_closed_pos}, open={drawer_open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
