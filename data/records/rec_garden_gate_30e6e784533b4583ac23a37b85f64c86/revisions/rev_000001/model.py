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


def _add_box(part, name, size, center, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_cylinder(part, name, radius, length, center, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _build_post(
    part,
    *,
    prefix,
    hinge_axis_offset,
    post_material,
    hardware_material,
    post_height,
    post_bottom,
):
    body_center_z = post_bottom + (post_height / 2.0)
    _add_box(
        part,
        f"{prefix}_body",
        (0.10, 0.12, post_height),
        (0.0, 0.0, body_center_z),
        post_material,
    )
    _add_box(
        part,
        f"{prefix}_cap",
        (0.12, 0.14, 0.025),
        (0.0, 0.0, post_bottom + post_height + 0.0125),
        post_material,
    )

    for hinge_name, hinge_z in (("lower_pin", 0.32), ("upper_pin", 0.92)):
        _add_box(
            part,
            f"{prefix}_{hinge_name}",
            (0.040, 0.024, 0.12),
            (hinge_axis_offset, 0.0, hinge_z),
            hardware_material,
        )


def _build_leaf(
    part,
    *,
    prefix,
    side_sign,
    frame_material,
    hardware_material,
    leaf_width,
    leaf_height,
    leaf_bottom,
    frame_thickness,
    depth,
    add_drop_bolt_guides=False,
):
    leaf_plane_y = -0.035
    leaf_mid_z = leaf_bottom + (leaf_height / 2.0)
    open_panel_height = leaf_height - (2.0 * frame_thickness)
    open_panel_mid_z = leaf_bottom + frame_thickness + (open_panel_height / 2.0)
    hinge_clearance = 0.025
    outer_stile_center_x = 0.024
    inner_stile_center_x = leaf_width - (frame_thickness / 2.0)
    rail_length = inner_stile_center_x - hinge_clearance
    rail_center_x = hinge_clearance + (rail_length / 2.0)

    _add_box(
        part,
        f"{prefix}_outer_stile",
        (frame_thickness, depth, leaf_height),
        (side_sign * outer_stile_center_x, leaf_plane_y, leaf_mid_z),
        frame_material,
    )
    _add_box(
        part,
        f"{prefix}_inner_stile",
        (frame_thickness, depth, leaf_height),
        (side_sign * inner_stile_center_x, leaf_plane_y, leaf_mid_z),
        frame_material,
    )
    _add_box(
        part,
        f"{prefix}_bottom_rail",
        (rail_length, depth, frame_thickness),
        (side_sign * rail_center_x, leaf_plane_y, leaf_bottom + (frame_thickness / 2.0)),
        frame_material,
    )
    _add_box(
        part,
        f"{prefix}_top_rail",
        (rail_length, depth, frame_thickness),
        (
            side_sign * rail_center_x,
            leaf_plane_y,
            leaf_bottom + leaf_height - (frame_thickness / 2.0),
        ),
        frame_material,
    )

    for index, x_pos in enumerate((0.20, 0.36, 0.52), start=1):
        _add_box(
            part,
            f"{prefix}_bar_{index}",
            (0.018, 0.018, open_panel_height),
            (side_sign * x_pos, leaf_plane_y, open_panel_mid_z),
            frame_material,
        )

    for hinge_name, hinge_z in (("lower_sleeve", 0.32), ("upper_sleeve", 0.92)):
        _add_box(
            part,
            f"{prefix}_{hinge_name}_strap",
            (0.024, 0.028, 0.12),
            (side_sign * 0.032, -0.028, hinge_z),
            frame_material,
        )
        _add_box(
            part,
            f"{prefix}_{hinge_name}",
            (0.040, 0.022, 0.12),
            (0.0, -0.003, hinge_z),
            hardware_material,
        )

    _add_box(
        part,
        f"{prefix}_latch_plate",
        (0.012, 0.020, 0.18),
        (side_sign * (leaf_width - 0.014), leaf_plane_y - 0.020, 0.68),
        hardware_material,
    )

    if add_drop_bolt_guides:
        guide_x = side_sign * (leaf_width - 0.020)
        guide_y = leaf_plane_y + (depth / 2.0) + 0.012
        _add_cylinder(
            part,
            f"{prefix}_upper_guide",
            0.012,
            0.080,
            (guide_x, guide_y, 0.78),
            hardware_material,
        )
        _add_cylinder(
            part,
            f"{prefix}_lower_guide",
            0.012,
            0.080,
            (guide_x, guide_y, 0.27),
            hardware_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_leaf_garden_gate")

    post_material = model.material("powder_coated_steel", rgba=(0.18, 0.24, 0.17, 1.0))
    hardware_material = model.material("galvanized_hardware", rgba=(0.70, 0.72, 0.74, 1.0))
    footing_material = model.material("concrete", rgba=(0.55, 0.55, 0.53, 1.0))

    post_bottom = -0.14
    post_height = 1.60
    leaf_bottom = 0.08
    leaf_height = 1.10
    leaf_width = 0.71
    frame_thickness = 0.04
    leaf_depth = 0.035
    left_post_x = -0.80
    right_post_x = 0.80
    left_hinge_x = -0.73
    right_hinge_x = 0.73
    drop_bolt_x = -(leaf_width - 0.020)
    drop_bolt_y = -0.0055
    open_angle = math.pi / 2.0

    foundation = model.part(
        "foundation",
        inertial=Inertial.from_geometry(
            Box((1.96, 0.24, 0.20)),
            mass=60.0,
            origin=Origin(xyz=(0.0, 0.0, -0.24)),
        ),
    )
    _add_box(
        foundation,
        "foundation_beam",
        (1.96, 0.24, 0.20),
        (0.0, 0.0, -0.24),
        footing_material,
    )
    _add_cylinder(
        foundation,
        "drop_socket",
        0.014,
        0.22,
        (0.042, drop_bolt_y, -0.06),
        hardware_material,
    )

    left_post = model.part(
        "left_post",
        inertial=Inertial.from_geometry(
            Box((0.10, 0.12, post_height)),
            mass=24.0,
            origin=Origin(xyz=(0.0, 0.0, post_bottom + (post_height / 2.0))),
        ),
    )
    _build_post(
        left_post,
        prefix="left_post",
        hinge_axis_offset=left_hinge_x - left_post_x,
        post_material=post_material,
        hardware_material=hardware_material,
        post_height=post_height,
        post_bottom=post_bottom,
    )

    right_post = model.part(
        "right_post",
        inertial=Inertial.from_geometry(
            Box((0.10, 0.12, post_height)),
            mass=24.0,
            origin=Origin(xyz=(0.0, 0.0, post_bottom + (post_height / 2.0))),
        ),
    )
    _build_post(
        right_post,
        prefix="right_post",
        hinge_axis_offset=right_hinge_x - right_post_x,
        post_material=post_material,
        hardware_material=hardware_material,
        post_height=post_height,
        post_bottom=post_bottom,
    )

    left_leaf = model.part(
        "left_leaf",
        inertial=Inertial.from_geometry(
            Box((leaf_width, leaf_depth, leaf_height)),
            mass=14.0,
            origin=Origin(xyz=((leaf_width / 2.0), 0.0, leaf_bottom + (leaf_height / 2.0))),
        ),
    )
    _build_leaf(
        left_leaf,
        prefix="left_leaf",
        side_sign=1.0,
        frame_material=post_material,
        hardware_material=hardware_material,
        leaf_width=leaf_width,
        leaf_height=leaf_height,
        leaf_bottom=leaf_bottom,
        frame_thickness=frame_thickness,
        depth=leaf_depth,
    )

    right_leaf = model.part(
        "right_leaf",
        inertial=Inertial.from_geometry(
            Box((leaf_width, leaf_depth, leaf_height)),
            mass=15.0,
            origin=Origin(xyz=(-(leaf_width / 2.0), 0.0, leaf_bottom + (leaf_height / 2.0))),
        ),
    )
    _build_leaf(
        right_leaf,
        prefix="right_leaf",
        side_sign=-1.0,
        frame_material=post_material,
        hardware_material=hardware_material,
        leaf_width=leaf_width,
        leaf_height=leaf_height,
        leaf_bottom=leaf_bottom,
        frame_thickness=frame_thickness,
        depth=leaf_depth,
        add_drop_bolt_guides=True,
    )

    drop_bolt = model.part(
        "drop_bolt",
        inertial=Inertial.from_geometry(
            Cylinder(radius=0.008, length=0.75),
            mass=1.2,
            origin=Origin(xyz=(0.0, 0.0, -0.375)),
        ),
    )
    _add_cylinder(
        drop_bolt,
        "bolt_rod",
        0.008,
        0.75,
        (0.0, 0.0, -0.375),
        hardware_material,
    )

    model.articulation(
        "foundation_to_left_post",
        ArticulationType.FIXED,
        parent=foundation,
        child=left_post,
        origin=Origin(xyz=(left_post_x, 0.0, 0.0)),
    )
    model.articulation(
        "foundation_to_right_post",
        ArticulationType.FIXED,
        parent=foundation,
        child=right_post,
        origin=Origin(xyz=(right_post_x, 0.0, 0.0)),
    )
    model.articulation(
        "left_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=left_post,
        child=left_leaf,
        origin=Origin(xyz=(left_hinge_x - left_post_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=open_angle,
        ),
    )
    model.articulation(
        "right_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=right_post,
        child=right_leaf,
        origin=Origin(xyz=(right_hinge_x - right_post_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-open_angle,
            upper=0.0,
        ),
    )
    model.articulation(
        "drop_bolt_slide",
        ArticulationType.PRISMATIC,
        parent=right_leaf,
        child=drop_bolt,
        origin=Origin(xyz=(drop_bolt_x, drop_bolt_y, 0.84)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.25,
            lower=0.0,
            upper=0.17,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foundation = object_model.get_part("foundation")
    left_post = object_model.get_part("left_post")
    right_post = object_model.get_part("right_post")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    drop_bolt = object_model.get_part("drop_bolt")

    left_hinge = object_model.get_articulation("left_leaf_hinge")
    right_hinge = object_model.get_articulation("right_leaf_hinge")
    bolt_slide = object_model.get_articulation("drop_bolt_slide")

    foundation_beam = foundation.get_visual("foundation_beam")
    drop_socket = foundation.get_visual("drop_socket")
    left_post_body = left_post.get_visual("left_post_body")
    right_post_body = right_post.get_visual("right_post_body")
    left_post_upper_pin = left_post.get_visual("left_post_upper_pin")
    left_post_lower_pin = left_post.get_visual("left_post_lower_pin")
    right_post_upper_pin = right_post.get_visual("right_post_upper_pin")
    right_post_lower_pin = right_post.get_visual("right_post_lower_pin")
    left_outer_stile = left_leaf.get_visual("left_leaf_outer_stile")
    left_inner_stile = left_leaf.get_visual("left_leaf_inner_stile")
    right_outer_stile = right_leaf.get_visual("right_leaf_outer_stile")
    right_inner_stile = right_leaf.get_visual("right_leaf_inner_stile")
    left_upper_sleeve = left_leaf.get_visual("left_leaf_upper_sleeve")
    left_lower_sleeve = left_leaf.get_visual("left_leaf_lower_sleeve")
    right_upper_sleeve = right_leaf.get_visual("right_leaf_upper_sleeve")
    right_lower_sleeve = right_leaf.get_visual("right_leaf_lower_sleeve")
    right_upper_guide = right_leaf.get_visual("right_leaf_upper_guide")
    right_lower_guide = right_leaf.get_visual("right_leaf_lower_guide")
    bolt_rod = drop_bolt.get_visual("bolt_rod")

    ctx.allow_overlap(
        left_leaf,
        left_post,
        elem_a=left_upper_sleeve,
        elem_b=left_post_upper_pin,
        reason="The leaf hinge knuckle wraps the fixed post pin.",
    )
    ctx.allow_overlap(
        left_leaf,
        left_post,
        elem_a=left_lower_sleeve,
        elem_b=left_post_lower_pin,
        reason="The leaf hinge knuckle wraps the fixed post pin.",
    )
    ctx.allow_overlap(
        right_leaf,
        right_post,
        elem_a=right_upper_sleeve,
        elem_b=right_post_upper_pin,
        reason="The leaf hinge knuckle wraps the fixed post pin.",
    )
    ctx.allow_overlap(
        right_leaf,
        right_post,
        elem_a=right_lower_sleeve,
        elem_b=right_post_lower_pin,
        reason="The leaf hinge knuckle wraps the fixed post pin.",
    )
    ctx.allow_overlap(
        drop_bolt,
        right_leaf,
        elem_a=bolt_rod,
        elem_b=right_upper_guide,
        reason="The drop bolt slides inside the welded guide tube.",
    )
    ctx.allow_overlap(
        drop_bolt,
        right_leaf,
        elem_a=bolt_rod,
        elem_b=right_lower_guide,
        reason="The drop bolt slides inside the welded guide tube.",
    )
    ctx.allow_overlap(
        drop_bolt,
        foundation,
        elem_a=bolt_rod,
        elem_b=drop_socket,
        reason="When locked, the drop bolt enters the ground socket.",
    )

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

    roots = object_model.root_parts()
    ctx.check(
        "single_root_foundation",
        len(roots) == 1 and roots[0].name == "foundation",
        details=f"root parts were {[part.name for part in roots]}",
    )
    ctx.check(
        "left_leaf_hinge_axis_vertical",
        tuple(left_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"left hinge axis was {left_hinge.axis}",
    )
    ctx.check(
        "right_leaf_hinge_axis_vertical",
        tuple(right_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"right hinge axis was {right_hinge.axis}",
    )
    ctx.check(
        "drop_bolt_axis_vertical",
        tuple(bolt_slide.axis) == (0.0, 0.0, -1.0),
        details=f"drop bolt axis was {bolt_slide.axis}",
    )

    left_limits = left_hinge.motion_limits
    right_limits = right_hinge.motion_limits
    bolt_limits = bolt_slide.motion_limits
    ctx.check(
        "left_leaf_open_range_realistic",
        left_limits is not None
        and left_limits.lower == 0.0
        and left_limits.upper is not None
        and 1.45 <= left_limits.upper <= 1.65,
        details=f"left hinge limits were {left_limits}",
    )
    ctx.check(
        "right_leaf_open_range_realistic",
        right_limits is not None
        and right_limits.upper == 0.0
        and right_limits.lower is not None
        and -1.65 <= right_limits.lower <= -1.45,
        details=f"right hinge limits were {right_limits}",
    )
    ctx.check(
        "drop_bolt_stroke_realistic",
        bolt_limits is not None
        and bolt_limits.lower == 0.0
        and bolt_limits.upper is not None
        and 0.14 <= bolt_limits.upper <= 0.20,
        details=f"bolt limits were {bolt_limits}",
    )

    ctx.expect_contact(
        left_post,
        foundation,
        elem_a=left_post_body,
        elem_b=foundation_beam,
        name="left_post_seated_on_foundation",
    )
    ctx.expect_contact(
        right_post,
        foundation,
        elem_a=right_post_body,
        elem_b=foundation_beam,
        name="right_post_seated_on_foundation",
    )
    ctx.expect_gap(
        left_leaf,
        left_post,
        axis="x",
        min_gap=0.015,
        max_gap=0.025,
        positive_elem=left_outer_stile,
        negative_elem=left_post_body,
        name="left_leaf_side_clearance",
    )
    ctx.expect_gap(
        right_post,
        right_leaf,
        axis="x",
        min_gap=0.015,
        max_gap=0.025,
        positive_elem=right_post_body,
        negative_elem=right_outer_stile,
        name="right_leaf_side_clearance",
    )
    ctx.expect_gap(
        right_leaf,
        left_leaf,
        axis="x",
        min_gap=0.035,
        max_gap=0.055,
        positive_elem=right_inner_stile,
        negative_elem=left_inner_stile,
        name="center_meeting_gap",
    )
    ctx.expect_overlap(
        drop_bolt,
        foundation,
        axes="xy",
        min_overlap=0.014,
        elem_a=bolt_rod,
        elem_b=drop_socket,
        name="drop_bolt_aligned_over_socket",
    )
    ctx.expect_overlap(
        drop_bolt,
        right_leaf,
        axes="xy",
        min_overlap=0.014,
        elem_a=bolt_rod,
        elem_b=right_upper_guide,
        name="drop_bolt_centered_in_upper_guide",
    )
    ctx.expect_overlap(
        drop_bolt,
        right_leaf,
        axes="xy",
        min_overlap=0.014,
        elem_a=bolt_rod,
        elem_b=right_lower_guide,
        name="drop_bolt_centered_in_lower_guide",
    )
    ctx.expect_gap(
        drop_bolt,
        foundation,
        axis="z",
        min_gap=0.03,
        positive_elem=bolt_rod,
        negative_elem=drop_socket,
        name="drop_bolt_retracted_above_socket",
    )

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

    if left_limits is not None and left_limits.lower is not None and left_limits.upper is not None:
        with ctx.pose({left_hinge: left_limits.lower, right_hinge: 0.0, bolt_slide: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="left_hinge_lower_no_floating")
        with ctx.pose({left_hinge: left_limits.upper, right_hinge: 0.0, bolt_slide: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="left_hinge_upper_no_floating")
            ctx.expect_gap(
                left_leaf,
                left_post,
                axis="y",
                min_gap=0.30,
                positive_elem=left_inner_stile,
                negative_elem=left_post_body,
                name="left_leaf_swings_open_toward_positive_y",
            )

    if right_limits is not None and right_limits.lower is not None and right_limits.upper is not None:
        with ctx.pose({left_hinge: 0.0, right_hinge: right_limits.upper, bolt_slide: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="right_hinge_upper_no_floating")
        with ctx.pose({left_hinge: 0.0, right_hinge: right_limits.lower, bolt_slide: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="right_hinge_lower_no_floating")
            ctx.expect_gap(
                right_leaf,
                right_post,
                axis="y",
                min_gap=0.30,
                positive_elem=right_inner_stile,
                negative_elem=right_post_body,
                name="right_leaf_swings_open_toward_positive_y",
            )

    if bolt_limits is not None and bolt_limits.lower is not None and bolt_limits.upper is not None:
        with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, bolt_slide: bolt_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="drop_bolt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="drop_bolt_lower_no_floating")
        with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, bolt_slide: bolt_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="drop_bolt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="drop_bolt_upper_no_floating")
            ctx.expect_gap(
                drop_bolt,
                foundation,
                axis="z",
                max_gap=0.0,
                max_penetration=0.14,
                positive_elem=bolt_rod,
                negative_elem=drop_socket,
                name="drop_bolt_enters_ground_socket",
            )

    if (
        left_limits is not None
        and left_limits.upper is not None
        and right_limits is not None
        and right_limits.lower is not None
    ):
        with ctx.pose({left_hinge: left_limits.upper, right_hinge: right_limits.lower, bolt_slide: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="both_leaves_open_no_overlap")
            ctx.fail_if_isolated_parts(name="both_leaves_open_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
