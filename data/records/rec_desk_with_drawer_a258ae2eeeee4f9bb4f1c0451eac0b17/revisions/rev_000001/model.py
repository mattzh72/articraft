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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_drawer(
    part,
    *,
    body_width: float,
    front_width: float,
    height: float,
    body_depth: float,
    wood,
    metal,
) -> None:
    front_thickness = 0.02
    side_thickness = 0.012
    bottom_thickness = 0.01
    body_height = height - 0.025
    body_center_y = front_thickness + body_depth / 2.0
    side_center_x = body_width / 2.0 - side_thickness / 2.0
    side_center_z = -0.005
    bottom_center_z = -height / 2.0 + bottom_thickness / 2.0 + 0.005
    back_thickness = 0.012
    back_center_y = front_thickness + body_depth - back_thickness / 2.0

    part.visual(
        Box((front_width, front_thickness, height)),
        origin=Origin(xyz=(0.0, front_thickness / 2.0, 0.0)),
        material=wood,
        name="front_panel",
    )
    part.visual(
        Box((side_thickness, body_depth, body_height)),
        origin=Origin(xyz=(-side_center_x, body_center_y, side_center_z)),
        material=wood,
        name="outer_side",
    )
    part.visual(
        Box((side_thickness, body_depth, body_height)),
        origin=Origin(xyz=(side_center_x, body_center_y, side_center_z)),
        material=wood,
        name="inner_side",
    )
    part.visual(
        Box((body_width - 2.0 * side_thickness, body_depth - 0.02, bottom_thickness)),
        origin=Origin(xyz=(0.0, front_thickness + (body_depth - 0.02) / 2.0, bottom_center_z)),
        material=wood,
        name="bottom_panel",
    )
    part.visual(
        Box((body_width - 2.0 * side_thickness, back_thickness, body_height)),
        origin=Origin(xyz=(0.0, back_center_y, side_center_z)),
        material=wood,
        name="back_panel",
    )

    stem_radius = 0.004
    stem_length = 0.016
    handle_radius = 0.005
    handle_length = 0.10
    stem_y = -stem_length / 2.0
    handle_y = -0.018
    handle_x_offset = 0.042

    part.visual(
        Cylinder(radius=stem_radius, length=stem_length),
        origin=Origin(xyz=(-handle_x_offset, stem_y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_handle_stem",
    )
    part.visual(
        Cylinder(radius=stem_radius, length=stem_length),
        origin=Origin(xyz=(handle_x_offset, stem_y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_handle_stem",
    )
    part.visual(
        Cylinder(radius=handle_radius, length=handle_length),
        origin=Origin(xyz=(0.0, handle_y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="pull_bar",
    )

    part.inertial = Inertial.from_geometry(
        Box((front_width, front_thickness + body_depth, height)),
        mass=2.6,
        origin=Origin(xyz=(0.0, (front_thickness + body_depth) / 2.0, 0.0)),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vanity_dressing_table")

    wood = model.material("painted_wood", color=(0.90, 0.85, 0.78))
    wood_shadow = model.material("shadow_wood", color=(0.84, 0.78, 0.70))
    brass = model.material("brass", color=(0.76, 0.65, 0.39))
    rail_metal = model.material("rail_metal", color=(0.63, 0.65, 0.68))
    mirror_glass = model.material("mirror_glass", rgba=(0.82, 0.88, 0.93, 0.90))

    table = model.part("table")

    top_width = 1.20
    top_depth = 0.50
    top_thickness = 0.03
    top_height = 0.76
    top_center_z = top_height - top_thickness / 2.0

    table.visual(
        Box((top_width, top_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_center_z)),
        material=wood,
        name="top",
    )

    side_panel_width = 0.026
    side_panel_depth = 0.43
    side_panel_height = top_height - top_thickness
    side_panel_z = side_panel_height / 2.0
    outer_panel_x = 0.54
    inner_panel_x = 0.255

    for sign, name in ((-1.0, "left_outer_panel"), (1.0, "right_outer_panel")):
        table.visual(
            Box((side_panel_width, side_panel_depth, side_panel_height)),
            origin=Origin(xyz=(sign * outer_panel_x, -0.015, side_panel_z)),
            material=wood_shadow,
            name=name,
        )

    for sign, name in ((-1.0, "left_inner_panel"), (1.0, "right_inner_panel")):
        table.visual(
            Box((side_panel_width, side_panel_depth, side_panel_height)),
            origin=Origin(xyz=(sign * inner_panel_x, -0.015, side_panel_z)),
            material=wood_shadow,
            name=name,
        )

    bay_width = 0.259
    bay_depth = 0.34
    bay_floor_thickness = 0.018
    bay_floor_z = 0.555
    bay_center_y = -0.05
    bay_centers = {"left": -0.3975, "right": 0.3975}

    for side, x_center in bay_centers.items():
        table.visual(
            Box((bay_width, bay_depth, bay_floor_thickness)),
            origin=Origin(xyz=(x_center, bay_center_y, bay_floor_z)),
            material=wood_shadow,
            name=f"{side}_bay_floor",
        )
        table.visual(
            Box((bay_width, 0.018, 0.166)),
            origin=Origin(xyz=(x_center, 0.129, 0.647)),
            material=wood_shadow,
            name=f"{side}_bay_back",
        )

    apron_width = 0.484
    apron_depth = 0.03
    apron_height = 0.08
    apron_z = 0.69
    table.visual(
        Box((apron_width, apron_depth, apron_height)),
        origin=Origin(xyz=(0.0, -0.235, apron_z)),
        material=wood_shadow,
        name="center_front_apron",
    )
    table.visual(
        Box((apron_width, apron_depth, apron_height)),
        origin=Origin(xyz=(0.0, 0.235, apron_z)),
        material=wood_shadow,
        name="center_rear_apron",
    )

    guide_width = 0.012
    guide_length = 0.32
    guide_height = 0.018
    guide_y = -0.05
    guide_z = 0.645

    guide_positions = {
        "left_guide_outer": -0.521,
        "left_guide_inner": -0.274,
        "right_guide_inner": 0.274,
        "right_guide_outer": 0.521,
    }
    for guide_name, x_pos in guide_positions.items():
        table.visual(
            Box((guide_width, guide_length, guide_height)),
            origin=Origin(xyz=(x_pos, guide_y, guide_z)),
            material=rail_metal,
            name=guide_name,
        )

    post_width = 0.04
    post_depth = 0.045
    post_height = 0.40
    post_z = top_height + post_height / 2.0
    post_y = 0.18
    post_x = 0.49

    for sign, name in ((-1.0, "left_post"), (1.0, "right_post")):
        table.visual(
            Box((post_width, post_depth, post_height)),
            origin=Origin(xyz=(sign * post_x, post_y, post_z)),
            material=wood,
            name=name,
        )

    collar_radius = 0.014
    collar_length = 0.016
    collar_x = 0.478
    pivot_z = 1.08

    for sign, name in ((-1.0, "left_hinge_collar"), (1.0, "right_hinge_collar")):
        table.visual(
            Cylinder(radius=collar_radius, length=collar_length),
            origin=Origin(xyz=(sign * collar_x, post_y, pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=brass,
            name=name,
        )

    left_drawer = model.part("left_drawer")
    right_drawer = model.part("right_drawer")

    drawer_body_width = 0.235
    drawer_front_width = 0.243
    drawer_height = 0.145
    drawer_body_depth = 0.31

    _add_drawer(
        left_drawer,
        body_width=drawer_body_width,
        front_width=drawer_front_width,
        height=drawer_height,
        body_depth=drawer_body_depth,
        wood=wood,
        metal=brass,
    )
    _add_drawer(
        right_drawer,
        body_width=drawer_body_width,
        front_width=drawer_front_width,
        height=drawer_height,
        body_depth=drawer_body_depth,
        wood=wood,
        metal=brass,
    )

    model.articulation(
        "table_to_left_drawer",
        ArticulationType.PRISMATIC,
        parent=table,
        child=left_drawer,
        origin=Origin(xyz=(bay_centers["left"], -0.249, 0.643)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.18),
    )
    model.articulation(
        "table_to_right_drawer",
        ArticulationType.PRISMATIC,
        parent=table,
        child=right_drawer,
        origin=Origin(xyz=(bay_centers["right"], -0.249, 0.643)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.18),
    )

    mirror = model.part("mirror")

    mirror_width = 0.90
    mirror_height = 0.62
    mirror_thickness = 0.03
    frame_width = 0.05
    rail_height = 0.05
    stile_center_x = mirror_width / 2.0 - frame_width / 2.0
    rail_center_z = mirror_height / 2.0 - rail_height / 2.0

    mirror.visual(
        Box((frame_width, mirror_thickness, mirror_height)),
        origin=Origin(xyz=(-stile_center_x, 0.0, 0.0)),
        material=wood,
        name="left_stile",
    )
    mirror.visual(
        Box((frame_width, mirror_thickness, mirror_height)),
        origin=Origin(xyz=(stile_center_x, 0.0, 0.0)),
        material=wood,
        name="right_stile",
    )
    mirror.visual(
        Box((mirror_width - 2.0 * frame_width, mirror_thickness, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, rail_center_z)),
        material=wood,
        name="top_rail",
    )
    mirror.visual(
        Box((mirror_width - 2.0 * frame_width, mirror_thickness, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, -rail_center_z)),
        material=wood,
        name="bottom_rail",
    )
    mirror.visual(
        Box((mirror_width - 2.0 * frame_width + 0.004, 0.006, mirror_height - 2.0 * rail_height + 0.004)),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=mirror_glass,
        name="glass_panel",
    )

    trunnion_radius = 0.012
    trunnion_length = 0.02
    trunnion_x = 0.46

    for sign, name in ((-1.0, "left_trunnion"), (1.0, "right_trunnion")):
        mirror.visual(
            Cylinder(radius=trunnion_radius, length=trunnion_length),
            origin=Origin(xyz=(sign * trunnion_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=brass,
            name=name,
        )

    mirror.inertial = Inertial.from_geometry(
        Box((mirror_width, mirror_thickness, mirror_height)),
        mass=7.0,
        origin=Origin(),
    )

    model.articulation(
        "table_to_mirror",
        ArticulationType.REVOLUTE,
        parent=table,
        child=mirror,
        origin=Origin(xyz=(0.0, post_y, pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.40, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    table = object_model.get_part("table")
    mirror = object_model.get_part("mirror")
    left_drawer = object_model.get_part("left_drawer")
    right_drawer = object_model.get_part("right_drawer")

    left_slide = object_model.get_articulation("table_to_left_drawer")
    right_slide = object_model.get_articulation("table_to_right_drawer")
    mirror_hinge = object_model.get_articulation("table_to_mirror")

    top = table.get_visual("top")
    left_outer_guide = table.get_visual("left_guide_outer")
    left_inner_guide = table.get_visual("left_guide_inner")
    right_outer_guide = table.get_visual("right_guide_outer")
    right_inner_guide = table.get_visual("right_guide_inner")
    left_collar = table.get_visual("left_hinge_collar")
    right_collar = table.get_visual("right_hinge_collar")

    left_trunnion = mirror.get_visual("left_trunnion")
    right_trunnion = mirror.get_visual("right_trunnion")
    mirror_bottom_rail = mirror.get_visual("bottom_rail")

    left_outer_side = left_drawer.get_visual("outer_side")
    left_inner_side = left_drawer.get_visual("inner_side")
    right_outer_side = right_drawer.get_visual("outer_side")
    right_inner_side = right_drawer.get_visual("inner_side")

    ctx.expect_contact(
        mirror,
        table,
        elem_a=left_trunnion,
        elem_b=left_collar,
        contact_tol=0.0005,
        name="left mirror trunnion seats in the left hinge collar",
    )
    ctx.expect_contact(
        mirror,
        table,
        elem_a=right_trunnion,
        elem_b=right_collar,
        contact_tol=0.0005,
        name="right mirror trunnion seats in the right hinge collar",
    )
    ctx.expect_gap(
        mirror,
        table,
        axis="z",
        positive_elem=mirror_bottom_rail,
        negative_elem=top,
        min_gap=0.008,
        max_gap=0.03,
        name="mirror clears the tabletop at rest",
    )

    ctx.expect_overlap(
        left_drawer,
        table,
        axes="y",
        elem_a=left_outer_side,
        elem_b=left_outer_guide,
        min_overlap=0.28,
        name="left drawer stays engaged with the outer guide when closed",
    )
    ctx.expect_overlap(
        left_drawer,
        table,
        axes="y",
        elem_a=left_inner_side,
        elem_b=left_inner_guide,
        min_overlap=0.28,
        name="left drawer stays engaged with the inner guide when closed",
    )
    ctx.expect_overlap(
        right_drawer,
        table,
        axes="y",
        elem_a=right_outer_side,
        elem_b=right_outer_guide,
        min_overlap=0.28,
        name="right drawer stays engaged with the outer guide when closed",
    )
    ctx.expect_overlap(
        right_drawer,
        table,
        axes="y",
        elem_a=right_inner_side,
        elem_b=right_inner_guide,
        min_overlap=0.28,
        name="right drawer stays engaged with the inner guide when closed",
    )

    left_rest = ctx.part_world_position(left_drawer)
    right_rest = ctx.part_world_position(right_drawer)
    rest_top_center = _aabb_center(ctx.part_element_world_aabb(mirror, elem="top_rail"))

    with ctx.pose({left_slide: 0.18, right_slide: 0.18, mirror_hinge: 0.30}):
        ctx.expect_overlap(
            left_drawer,
            table,
            axes="y",
            elem_a=left_outer_side,
            elem_b=left_outer_guide,
            min_overlap=0.10,
            name="left drawer retains insertion on the outer guide when opened",
        )
        ctx.expect_overlap(
            right_drawer,
            table,
            axes="y",
            elem_a=right_inner_side,
            elem_b=right_inner_guide,
            min_overlap=0.10,
            name="right drawer retains insertion on the inner guide when opened",
        )

        left_open = ctx.part_world_position(left_drawer)
        right_open = ctx.part_world_position(right_drawer)
        tilted_top_center = _aabb_center(ctx.part_element_world_aabb(mirror, elem="top_rail"))

        ctx.check(
            "left drawer opens forward along its slide",
            left_rest is not None and left_open is not None and left_open[1] < left_rest[1] - 0.16,
            details=f"rest={left_rest}, open={left_open}",
        )
        ctx.check(
            "right drawer opens forward along its slide",
            right_rest is not None and right_open is not None and right_open[1] < right_rest[1] - 0.16,
            details=f"rest={right_rest}, open={right_open}",
        )
        ctx.check(
            "positive mirror angle tilts the top edge backward",
            rest_top_center is not None
            and tilted_top_center is not None
            and tilted_top_center[1] > rest_top_center[1] + 0.05,
            details=f"rest={rest_top_center}, tilted={tilted_top_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
