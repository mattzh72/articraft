from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_box(part, size, xyz, *, material, name):
    return part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_table")

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    black_steel = model.material("black_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    top_oak = model.material("top_oak", rgba=(0.78, 0.67, 0.49, 1.0))
    drawer_front = model.material("drawer_front", rgba=(0.73, 0.61, 0.43, 1.0))
    drawer_body = model.material("drawer_body", rgba=(0.56, 0.46, 0.34, 1.0))

    table_width = 1.20
    table_depth = 0.76
    top_width = 1.26
    top_depth = 0.82
    top_thickness = 0.028
    top_surface_z = 0.739
    frame_height = 0.03
    frame_top_z = top_surface_z - top_thickness - 0.002
    frame_center_z = frame_top_z - frame_height / 2.0

    leg_section = 0.045
    rail_width = 0.04
    lower_stretcher_section = 0.03

    half_base_x = table_width / 2.0
    half_base_y = table_depth / 2.0
    leg_x = half_base_x - leg_section / 2.0
    leg_y = half_base_y - leg_section / 2.0

    upper_span_x = table_width - 2.0 * leg_section
    upper_span_y = table_depth - 2.0 * leg_section

    base = model.part("base")

    # Upper frame and legs
    _add_box(
        base,
        (upper_span_x, rail_width, frame_height),
        (0.0, leg_y - rail_width / 2.0, frame_center_z),
        material=painted_steel,
        name="front_upper_rail",
    )
    _add_box(
        base,
        (upper_span_x, rail_width, frame_height),
        (0.0, -leg_y + rail_width / 2.0, frame_center_z),
        material=painted_steel,
        name="rear_upper_rail",
    )
    _add_box(
        base,
        (rail_width, upper_span_y, frame_height),
        (leg_x - rail_width / 2.0, 0.0, frame_center_z),
        material=painted_steel,
        name="left_upper_rail",
    )
    _add_box(
        base,
        (rail_width, upper_span_y, frame_height),
        (-leg_x + rail_width / 2.0, 0.0, frame_center_z),
        material=painted_steel,
        name="right_upper_rail",
    )

    for side_name, x in (("left", leg_x), ("right", -leg_x)):
        for end_name, y in (("front", leg_y), ("rear", -leg_y)):
            _add_box(
                base,
                (leg_section, leg_section, frame_top_z),
                (x, y, frame_top_z / 2.0),
                material=black_steel,
                name=f"{side_name}_{end_name}_leg",
            )

    side_stretcher_y = upper_span_y
    _add_box(
        base,
        (lower_stretcher_section, side_stretcher_y, lower_stretcher_section),
        (leg_x - rail_width / 2.0, 0.0, 0.18),
        material=painted_steel,
        name="left_lower_stretcher",
    )
    _add_box(
        base,
        (lower_stretcher_section, side_stretcher_y, lower_stretcher_section),
        (-leg_x + rail_width / 2.0, 0.0, 0.18),
        material=painted_steel,
        name="right_lower_stretcher",
    )
    _add_box(
        base,
        (upper_span_x - 0.025, lower_stretcher_section, lower_stretcher_section),
        (0.0, -0.31, 0.18),
        material=painted_steel,
        name="rear_lower_stretcher",
    )

    # Drawer guide structure mounted below the front half of the frame.
    guide_length = 0.44
    guide_z = 0.668
    guide_x = 0.32
    guide_y = 0.14
    guide_size = (0.02, guide_length, 0.02)
    _add_box(
        base,
        guide_size,
        (-guide_x, guide_y, guide_z),
        material=painted_steel,
        name="left_guide_rail",
    )
    _add_box(
        base,
        guide_size,
        (guide_x, guide_y, guide_z),
        material=painted_steel,
        name="right_guide_rail",
    )
    _add_box(
        base,
        (0.02, 0.02, 0.03),
        (-guide_x, 0.35, 0.693),
        material=painted_steel,
        name="left_guide_front_hanger",
    )
    _add_box(
        base,
        (0.02, 0.02, 0.03),
        (guide_x, 0.35, 0.693),
        material=painted_steel,
        name="right_guide_front_hanger",
    )
    _add_box(
        base,
        (0.71, 0.02, 0.02),
        (0.0, guide_y - guide_length / 2.0, guide_z),
        material=painted_steel,
        name="rear_guide_bridge",
    )

    base.inertial = Inertial.from_geometry(
        Box((table_width, table_depth, top_surface_z)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, top_surface_z / 2.0)),
    )

    tabletop = model.part("tabletop")
    _add_box(
        tabletop,
        (top_width, top_depth, top_thickness),
        (0.0, top_depth / 2.0, -top_thickness / 2.0),
        material=top_oak,
        name="top_panel",
    )
    _add_box(
        tabletop,
        (top_width - 0.08, 0.04, 0.018),
        (0.0, top_depth - 0.035, -0.037),
        material=drawer_front,
        name="front_stiffener",
    )
    tabletop.inertial = Inertial.from_geometry(
        Box((top_width, top_depth, top_thickness)),
        mass=13.0,
        origin=Origin(xyz=(0.0, top_depth / 2.0, -top_thickness / 2.0)),
    )

    drawer = model.part("drawer")
    drawer_outer_width = 0.62
    drawer_outer_depth = 0.44
    drawer_outer_height = 0.08
    wall_thickness = 0.012
    bottom_thickness = 0.012

    _add_box(
        drawer,
        (drawer_outer_width - 0.02, drawer_outer_depth, bottom_thickness),
        (0.0, 0.0, -drawer_outer_height / 2.0 + bottom_thickness / 2.0),
        material=drawer_body,
        name="drawer_bottom",
    )
    _add_box(
        drawer,
        (wall_thickness, drawer_outer_depth, drawer_outer_height - bottom_thickness),
        (
            drawer_outer_width / 2.0 - wall_thickness / 2.0,
            0.0,
            -bottom_thickness / 2.0,
        ),
        material=drawer_body,
        name="drawer_right_side",
    )
    _add_box(
        drawer,
        (wall_thickness, drawer_outer_depth, drawer_outer_height - bottom_thickness),
        (
            -drawer_outer_width / 2.0 + wall_thickness / 2.0,
            0.0,
            -bottom_thickness / 2.0,
        ),
        material=drawer_body,
        name="drawer_left_side",
    )
    _add_box(
        drawer,
        (drawer_outer_width - 0.02, wall_thickness, drawer_outer_height - bottom_thickness),
        (0.0, -drawer_outer_depth / 2.0 + wall_thickness / 2.0, -bottom_thickness / 2.0),
        material=drawer_body,
        name="drawer_back",
    )
    _add_box(
        drawer,
        (0.66, 0.018, 0.11),
        (0.0, drawer_outer_depth / 2.0 + 0.009, 0.0),
        material=drawer_front,
        name="drawer_front_panel",
    )
    _add_box(
        drawer,
        (0.16, 0.012, 0.012),
        (0.0, drawer_outer_depth / 2.0 + 0.032, 0.0),
        material=black_steel,
        name="drawer_pull_bar",
    )
    _add_box(
        drawer,
        (0.012, 0.016, 0.012),
        (-0.055, drawer_outer_depth / 2.0 + 0.022, 0.0),
        material=black_steel,
        name="drawer_pull_left_post",
    )
    _add_box(
        drawer,
        (0.012, 0.016, 0.012),
        (0.055, drawer_outer_depth / 2.0 + 0.022, 0.0),
        material=black_steel,
        name="drawer_pull_right_post",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.66, drawer_outer_depth + 0.04, 0.11)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.01, 0.0)),
    )

    model.articulation(
        "base_to_tabletop",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tabletop,
        origin=Origin(xyz=(0.0, -0.385, top_surface_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "base_to_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.141, 0.63)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.4,
            lower=0.0,
            upper=0.22,
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

    base = object_model.get_part("base")
    tabletop = object_model.get_part("tabletop")
    drawer = object_model.get_part("drawer")
    top_tilt = object_model.get_articulation("base_to_tabletop")
    drawer_slide = object_model.get_articulation("base_to_drawer")

    top_panel = tabletop.get_visual("top_panel")
    front_stiffener = tabletop.get_visual("front_stiffener")
    front_upper_rail = base.get_visual("front_upper_rail")
    left_guide_rail = base.get_visual("left_guide_rail")
    drawer_front_panel = drawer.get_visual("drawer_front_panel")
    drawer_bottom = drawer.get_visual("drawer_bottom")

    ctx.expect_gap(
        tabletop,
        base,
        axis="z",
        positive_elem=top_panel,
        max_gap=0.01,
        max_penetration=0.0,
        name="tabletop sits just above the support frame",
    )
    ctx.expect_gap(
        drawer,
        base,
        axis="y",
        positive_elem=drawer_front_panel,
        negative_elem=front_upper_rail,
        min_gap=0.002,
        max_gap=0.03,
        name="drawer front remains reachable from the front opening",
    )

    closed_front = ctx.part_element_world_aabb(tabletop, elem=front_stiffener)
    with ctx.pose({top_tilt: 0.9}):
        open_front = ctx.part_element_world_aabb(tabletop, elem=front_stiffener)
    ctx.check(
        "tabletop front edge lifts when tilted",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[0][2] + 0.22,
        details=f"closed={closed_front}, open={open_front}",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.22}):
        open_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            base,
            axes="y",
            elem_a=drawer_bottom,
            elem_b=left_guide_rail,
            min_overlap=0.18,
            name="drawer keeps meaningful insertion on its guide rail when open",
        )
    ctx.check(
        "drawer slides outward from the front",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[1] > closed_drawer_pos[1] + 0.18,
        details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
