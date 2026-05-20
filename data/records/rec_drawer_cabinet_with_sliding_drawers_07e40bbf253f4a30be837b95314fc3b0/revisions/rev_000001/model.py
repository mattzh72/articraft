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
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.40
BODY_DEPTH = 0.58
BODY_HEIGHT = 0.82
CARCASS_THICKNESS = 0.018
BACK_THICKNESS = 0.006
KICK_HEIGHT = 0.100
KICK_RECESS = 0.060

DRAWER_FRONT_THICKNESS = 0.019
DRAWER_FRONT_GAP = 0.003
DRAWER_FRONT_HEIGHT = 0.224
DRAWER_FRONT_WIDTH = 0.394

DRAWER_BOX_WIDTH = 0.318
DRAWER_BOX_DEPTH = 0.470
DRAWER_BOX_HEIGHT = 0.160
DRAWER_PANEL_THICKNESS = 0.012

OUTER_RAIL_THICKNESS = 0.012
OUTER_RAIL_LENGTH = 0.440
OUTER_RAIL_HEIGHT = 0.030
INNER_RAIL_THICKNESS = 0.006
INNER_RAIL_LENGTH = 0.400
INNER_RAIL_HEIGHT = 0.024
DRAWER_TRAVEL = 0.320
GUIDE_Y = 0.010


def _build_drawer_stack(
    model: ArticulatedObject,
    *,
    drawer_index: int,
    front_bottom_z: float,
    body_material,
    front_material,
    rail_material,
    handle_shadow_material,
):
    guide_z = front_bottom_z + 0.060
    box_bottom_z = front_bottom_z + 0.030
    box_center_z = box_bottom_z + DRAWER_BOX_HEIGHT * 0.5
    front_center_z = front_bottom_z + DRAWER_FRONT_HEIGHT * 0.5
    interior_width = BODY_WIDTH - 2.0 * CARCASS_THICKNESS
    front_center_local_z = front_center_z - guide_z
    front_face_y = BODY_DEPTH * 0.5 + DRAWER_FRONT_THICKNESS * 0.5 - GUIDE_Y

    handle_opening_width = 0.244
    handle_opening_height = 0.032
    handle_side_width = (DRAWER_FRONT_WIDTH - handle_opening_width) * 0.5
    handle_top_margin = 0.034
    handle_center_offset_from_front_center = DRAWER_FRONT_HEIGHT * 0.5 - handle_top_margin - handle_opening_height * 0.5
    handle_center_z = front_center_local_z + handle_center_offset_from_front_center

    upper_front_height = handle_top_margin
    lower_front_height = DRAWER_FRONT_HEIGHT - handle_top_margin - handle_opening_height
    upper_front_center_z = front_center_local_z + DRAWER_FRONT_HEIGHT * 0.5 - upper_front_height * 0.5
    lower_front_center_z = front_center_local_z - DRAWER_FRONT_HEIGHT * 0.5 + lower_front_height * 0.5

    handle_pocket_depth = 0.013
    handle_pocket_back_thickness = DRAWER_FRONT_THICKNESS - handle_pocket_depth
    grip_bar_width = 0.200
    grip_bar_height = 0.010
    grip_bar_depth = 0.007
    grip_bar_center_y = front_face_y - handle_pocket_depth + grip_bar_depth * 0.5
    grip_bar_center_z = handle_center_z + handle_opening_height * 0.5 - grip_bar_height * 0.65

    guide_part = model.part(f"drawer_{drawer_index}_outer_guides")
    guide_part.visual(
        Box((OUTER_RAIL_THICKNESS, OUTER_RAIL_LENGTH, OUTER_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                -interior_width * 0.5 + OUTER_RAIL_THICKNESS * 0.5,
                0.0,
                0.0,
            )
        ),
        material=rail_material,
        name="left_outer_rail",
    )
    guide_part.visual(
        Box((OUTER_RAIL_THICKNESS, OUTER_RAIL_LENGTH, OUTER_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                interior_width * 0.5 - OUTER_RAIL_THICKNESS * 0.5,
                0.0,
                0.0,
            )
        ),
        material=rail_material,
        name="right_outer_rail",
    )
    guide_part.visual(
        Box((interior_width - 2.0 * OUTER_RAIL_THICKNESS, 0.014, 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                -OUTER_RAIL_LENGTH * 0.5 + 0.007,
                -OUTER_RAIL_HEIGHT * 0.5 + 0.005,
            )
        ),
        material=rail_material,
        name="rear_bridge",
    )
    guide_part.visual(
        Box((OUTER_RAIL_THICKNESS, 0.034, 0.012)),
        origin=Origin(
            xyz=(
                -interior_width * 0.5 + OUTER_RAIL_THICKNESS * 0.5,
                -OUTER_RAIL_LENGTH * 0.5 + 0.028,
                -0.003,
            )
        ),
        material=rail_material,
        name="left_soft_close_block",
    )
    guide_part.visual(
        Box((OUTER_RAIL_THICKNESS, 0.034, 0.012)),
        origin=Origin(
            xyz=(
                interior_width * 0.5 - OUTER_RAIL_THICKNESS * 0.5,
                -OUTER_RAIL_LENGTH * 0.5 + 0.028,
                -0.003,
            )
        ),
        material=rail_material,
        name="right_soft_close_block",
    )
    guide_part.inertial = Inertial.from_geometry(
        Box((interior_width, OUTER_RAIL_LENGTH, OUTER_RAIL_HEIGHT)),
        mass=0.55,
    )

    drawer_part = model.part(f"drawer_{drawer_index}")
    drawer_part.visual(
        Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, lower_front_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y,
                lower_front_center_z,
            )
        ),
        material=front_material,
        name="front_lower_panel",
    )
    drawer_part.visual(
        Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, upper_front_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y,
                upper_front_center_z,
            )
        ),
        material=front_material,
        name="front_upper_panel",
    )
    drawer_part.visual(
        Box((handle_side_width, DRAWER_FRONT_THICKNESS, handle_opening_height)),
        origin=Origin(
            xyz=(
                -DRAWER_FRONT_WIDTH * 0.5 + handle_side_width * 0.5,
                front_face_y,
                handle_center_z,
            )
        ),
        material=front_material,
        name="front_left_stile",
    )
    drawer_part.visual(
        Box((handle_side_width, DRAWER_FRONT_THICKNESS, handle_opening_height)),
        origin=Origin(
            xyz=(
                DRAWER_FRONT_WIDTH * 0.5 - handle_side_width * 0.5,
                front_face_y,
                handle_center_z,
            )
        ),
        material=front_material,
        name="front_right_stile",
    )
    drawer_part.visual(
        Box((handle_opening_width, handle_pocket_back_thickness, handle_opening_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y - handle_pocket_depth * 0.5,
                handle_center_z,
            )
        ),
        material=handle_shadow_material,
        name="drawer_handle_pocket",
    )
    drawer_part.visual(
        Box((grip_bar_width, grip_bar_depth, grip_bar_height)),
        origin=Origin(
            xyz=(
                0.0,
                grip_bar_center_y,
                grip_bar_center_z,
            )
        ),
        material=rail_material,
        name="drawer_handle_bar",
    )
    drawer_part.visual(
        Box((DRAWER_BOX_WIDTH, DRAWER_BOX_DEPTH - DRAWER_PANEL_THICKNESS, DRAWER_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.049,
                box_bottom_z + DRAWER_PANEL_THICKNESS * 0.5 - guide_z,
            )
        ),
        material=body_material,
        name="drawer_bottom",
    )
    drawer_part.visual(
        Box((DRAWER_PANEL_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(
            xyz=(
                -DRAWER_BOX_WIDTH * 0.5 + DRAWER_PANEL_THICKNESS * 0.5,
                0.045,
                box_center_z - guide_z,
            )
        ),
        material=body_material,
        name="left_drawer_side",
    )
    drawer_part.visual(
        Box((DRAWER_PANEL_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(
            xyz=(
                DRAWER_BOX_WIDTH * 0.5 - DRAWER_PANEL_THICKNESS * 0.5,
                0.045,
                box_center_z - guide_z,
            )
        ),
        material=body_material,
        name="right_drawer_side",
    )
    drawer_part.visual(
        Box((DRAWER_BOX_WIDTH - 2.0 * DRAWER_PANEL_THICKNESS, DRAWER_PANEL_THICKNESS, DRAWER_BOX_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -0.184,
                box_center_z - guide_z,
            )
        ),
        material=body_material,
        name="drawer_back",
    )
    drawer_part.visual(
        Box((INNER_RAIL_THICKNESS, INNER_RAIL_LENGTH, INNER_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                -0.167,
                0.0,
                0.0,
            )
        ),
        material=rail_material,
        name="left_inner_rail",
    )
    drawer_part.visual(
        Box((INNER_RAIL_THICKNESS, INNER_RAIL_LENGTH, INNER_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.167,
                0.0,
                0.0,
            )
        ),
        material=rail_material,
        name="right_inner_rail",
    )
    for side_sign, prefix in ((-1.0, "left"), (1.0, "right")):
        bracket_x = side_sign * 0.1615
        for bracket_y, suffix in ((0.150, "front"), (-0.110, "rear")):
            drawer_part.visual(
                Box((0.005, 0.040, 0.030)),
                origin=Origin(xyz=(bracket_x, bracket_y, 0.015)),
                material=rail_material,
                name=f"{prefix}_rail_bracket_{suffix}",
            )
    drawer_part.inertial = Inertial.from_geometry(
        Box((DRAWER_FRONT_WIDTH, DRAWER_BOX_DEPTH + DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.070, front_center_z - guide_z)),
    )

    model.articulation(
        f"cabinet_to_drawer_{drawer_index}_guides",
        ArticulationType.FIXED,
        parent="cabinet_body",
        child=guide_part,
        origin=Origin(xyz=(0.0, GUIDE_Y, guide_z)),
    )
    model.articulation(
        f"drawer_{drawer_index}_slide",
        ArticulationType.PRISMATIC,
        parent=guide_part,
        child=drawer_part,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
        motion_properties=MotionProperties(damping=28.0, friction=7.0),
        meta={"soft_close": True},
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_counter_drawer_unit")

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.94, 0.92, 1.0))
    drawer_front_white = model.material("drawer_front_white", rgba=(0.97, 0.97, 0.95, 1.0))
    birch_interior = model.material("birch_interior", rgba=(0.76, 0.68, 0.54, 1.0))
    slide_metal = model.material("slide_metal", rgba=(0.66, 0.68, 0.71, 1.0))
    handle_shadow = model.material("handle_shadow", rgba=(0.18, 0.19, 0.20, 1.0))

    cabinet = model.part("cabinet_body")
    cabinet.visual(
        Box((CARCASS_THICKNESS, BODY_DEPTH, BODY_HEIGHT - KICK_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH * 0.5 + CARCASS_THICKNESS * 0.5,
                0.0,
                KICK_HEIGHT + (BODY_HEIGHT - KICK_HEIGHT) * 0.5,
            )
        ),
        material=cabinet_white,
        name="left_side",
    )
    cabinet.visual(
        Box((CARCASS_THICKNESS, BODY_DEPTH, BODY_HEIGHT - KICK_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_WIDTH * 0.5 - CARCASS_THICKNESS * 0.5,
                0.0,
                KICK_HEIGHT + (BODY_HEIGHT - KICK_HEIGHT) * 0.5,
            )
        ),
        material=cabinet_white,
        name="right_side",
    )
    cabinet.visual(
        Box((CARCASS_THICKNESS, BODY_DEPTH - KICK_RECESS, KICK_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH * 0.5 + CARCASS_THICKNESS * 0.5,
                -KICK_RECESS * 0.5,
                KICK_HEIGHT * 0.5,
            )
        ),
        material=cabinet_white,
        name="left_plinth_leg",
    )
    cabinet.visual(
        Box((CARCASS_THICKNESS, BODY_DEPTH - KICK_RECESS, KICK_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_WIDTH * 0.5 - CARCASS_THICKNESS * 0.5,
                -KICK_RECESS * 0.5,
                KICK_HEIGHT * 0.5,
            )
        ),
        material=cabinet_white,
        name="right_plinth_leg",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * CARCASS_THICKNESS, BODY_DEPTH - KICK_RECESS, CARCASS_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -KICK_RECESS * 0.5,
                KICK_HEIGHT + CARCASS_THICKNESS * 0.5,
            )
        ),
        material=cabinet_white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * CARCASS_THICKNESS, BODY_DEPTH, CARCASS_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - CARCASS_THICKNESS * 0.5)),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * CARCASS_THICKNESS, BACK_THICKNESS, BODY_HEIGHT - KICK_HEIGHT - CARCASS_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_DEPTH * 0.5 + BACK_THICKNESS * 0.5,
                KICK_HEIGHT + CARCASS_THICKNESS + (BODY_HEIGHT - KICK_HEIGHT - CARCASS_THICKNESS) * 0.5,
            )
        ),
        material=cabinet_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * CARCASS_THICKNESS, 0.018, KICK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 - KICK_RECESS - 0.009,
                KICK_HEIGHT * 0.5,
            )
        ),
        material=cabinet_white,
        name="toe_kick_board",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    first_front_bottom = KICK_HEIGHT + DRAWER_FRONT_GAP
    for drawer_index in range(1, 4):
        front_bottom_z = first_front_bottom + (drawer_index - 1) * (DRAWER_FRONT_HEIGHT + DRAWER_FRONT_GAP)
        _build_drawer_stack(
            model,
            drawer_index=drawer_index,
            front_bottom_z=front_bottom_z,
            body_material=birch_interior,
            front_material=drawer_front_white,
            rail_material=slide_metal,
            handle_shadow_material=handle_shadow,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    cabinet = object_model.get_part("cabinet_body")
    drawers = [object_model.get_part(f"drawer_{index}") for index in range(1, 4)]
    guides = [object_model.get_part(f"drawer_{index}_outer_guides") for index in range(1, 4)]
    slides = [object_model.get_articulation(f"drawer_{index}_slide") for index in range(1, 4)]

    ctx.check("cabinet_body_present", cabinet is not None, "Cabinet body part should exist.")
    for index, (drawer, guide, slide) in enumerate(zip(drawers, guides, slides), start=1):
        ctx.check(f"drawer_{index}_present", drawer is not None, f"drawer_{index} should exist.")
        ctx.check(
            f"drawer_{index}_guides_present",
            guide is not None,
            f"drawer_{index}_outer_guides should exist.",
        )
        limits = slide.motion_limits
        ctx.check(
            f"drawer_{index}_slide_axis",
            tuple(slide.axis) == (0.0, 1.0, 0.0),
            f"{slide.name} should translate outward on +Y.",
        )
        ctx.check(
            f"drawer_{index}_slide_limits",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and abs(limits.upper - DRAWER_TRAVEL) < 1e-9,
            f"{slide.name} should travel from fully closed to {DRAWER_TRAVEL:.3f} m open.",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=21,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=21,
        name="drawer_slides_clearance_sweep",
    )

    for index, (drawer, guide, slide) in enumerate(zip(drawers, guides, slides), start=1):
        limits = slide.motion_limits
        assert limits is not None and limits.upper is not None

        with ctx.pose({slide: limits.lower}):
            ctx.expect_contact(
                guide,
                cabinet,
                elem_a="left_outer_rail",
                elem_b="left_side",
                name=f"drawer_{index}_left_guide_mounted",
            )
            ctx.expect_contact(
                guide,
                cabinet,
                elem_a="right_outer_rail",
                elem_b="right_side",
                name=f"drawer_{index}_right_guide_mounted",
            )
            ctx.expect_contact(
                drawer,
                guide,
                elem_a="left_inner_rail",
                elem_b="left_outer_rail",
                name=f"drawer_{index}_left_slide_contact_closed",
            )
            ctx.expect_contact(
                drawer,
                guide,
                elem_a="right_inner_rail",
                elem_b="right_outer_rail",
                name=f"drawer_{index}_right_slide_contact_closed",
            )
            ctx.expect_gap(
                drawer,
                cabinet,
                axis="y",
                positive_elem="front_upper_panel",
                max_gap=0.0005,
                max_penetration=0.0,
                name=f"drawer_{index}_front_flush_closed",
            )
            ctx.expect_within(
                drawer,
                cabinet,
                axes="xz",
                margin=0.0,
                name=f"drawer_{index}_front_within_cabinet_face",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="xz",
                min_overlap=0.20,
                name=f"drawer_{index}_front_face_overlap_closed",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"drawer_{index}_closed_no_overlap")
            ctx.fail_if_isolated_parts(name=f"drawer_{index}_closed_no_floating")

        with ctx.pose({slide: limits.upper}):
            ctx.expect_contact(
                drawer,
                guide,
                elem_a="left_inner_rail",
                elem_b="left_outer_rail",
                name=f"drawer_{index}_left_slide_contact_open",
            )
            ctx.expect_contact(
                drawer,
                guide,
                elem_a="right_inner_rail",
                elem_b="right_outer_rail",
                name=f"drawer_{index}_right_slide_contact_open",
            )
            ctx.expect_gap(
                drawer,
                cabinet,
                axis="y",
                positive_elem="front_upper_panel",
                min_gap=DRAWER_TRAVEL - 0.001,
                max_gap=DRAWER_TRAVEL + 0.001,
                name=f"drawer_{index}_travel_distance_open",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"drawer_{index}_open_no_overlap")
            ctx.fail_if_isolated_parts(name=f"drawer_{index}_open_no_floating")

    front_aabbs = [ctx.part_world_aabb(drawer) for drawer in drawers]
    bottom_aabbs = [ctx.part_element_world_aabb(drawer, elem="drawer_bottom") for drawer in drawers]
    assert all(aabb is not None for aabb in front_aabbs)
    assert all(aabb is not None for aabb in bottom_aabbs)

    front_gaps = [
        front_aabbs[1][0][2] - front_aabbs[0][1][2],
        front_aabbs[2][0][2] - front_aabbs[1][1][2],
    ]
    ctx.check(
        "drawer_front_reveals_even",
        all(abs(gap - DRAWER_FRONT_GAP) < 1e-6 for gap in front_gaps),
        f"Drawer front vertical reveals should be {DRAWER_FRONT_GAP:.3f} m; got {front_gaps!r}.",
    )

    bottom_depths = [aabb[1][1] - aabb[0][1] for aabb in bottom_aabbs]
    ctx.check(
        "drawer_boxes_equal_depth",
        max(bottom_depths) - min(bottom_depths) < 1e-9,
        f"Drawer box depths should match; got {bottom_depths!r}.",
    )

    front_widths = [aabb[1][0] - aabb[0][0] for aabb in front_aabbs]
    ctx.check(
        "drawer_fronts_equal_width",
        max(front_widths) - min(front_widths) < 1e-9,
        f"Drawer front widths should match; got {front_widths!r}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
