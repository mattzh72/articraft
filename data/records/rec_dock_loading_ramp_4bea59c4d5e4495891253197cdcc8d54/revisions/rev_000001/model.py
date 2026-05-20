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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, depth: float):
    half_w = width / 2.0
    half_d = depth / 2.0
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _square_tube_mesh(
    *,
    outer_width: float,
    inner_width: float,
    height: float,
    name: str,
):
    shell = ExtrudeWithHolesGeometry(
        _rect_profile(outer_width, outer_width),
        [_rect_profile(inner_width, inner_width)],
        height,
    )
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_deck_dock_plate")

    frame_paint = model.material("frame_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    support_yellow = model.material("support_yellow", rgba=(0.73, 0.66, 0.16, 1.0))
    rod_steel = model.material("rod_steel", rgba=(0.80, 0.81, 0.83, 1.0))

    ground_frame = model.part("ground_frame")
    platform = model.part("platform")
    left_support = model.part("left_support")
    right_support = model.part("right_support")

    # Ground frame: welded perimeter, hinge pedestals, and two support housings.
    ground_frame.visual(
        Box((1.90, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.82, 0.05)),
        material=frame_paint,
        name="left_side_rail",
    )
    ground_frame.visual(
        Box((1.90, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, -0.82, 0.05)),
        material=frame_paint,
        name="right_side_rail",
    )
    ground_frame.visual(
        Box((0.14, 1.78, 0.10)),
        origin=Origin(xyz=(-0.60, 0.0, 0.05)),
        material=frame_paint,
        name="rear_crossmember",
    )
    ground_frame.visual(
        Box((0.14, 1.78, 0.10)),
        origin=Origin(xyz=(0.60, 0.0, 0.05)),
        material=frame_paint,
        name="front_crossmember",
    )
    ground_frame.visual(
        Box((1.06, 0.20, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=frame_paint,
        name="center_spine",
    )
    ground_frame.visual(
        Box((0.14, 0.12, 0.10)),
        origin=Origin(xyz=(0.62, 0.52, 0.05)),
        material=frame_paint,
        name="left_support_inner_arm",
    )
    ground_frame.visual(
        Box((0.14, 0.12, 0.10)),
        origin=Origin(xyz=(0.94, 0.52, 0.05)),
        material=frame_paint,
        name="left_support_outer_arm",
    )
    ground_frame.visual(
        Box((0.14, 0.12, 0.10)),
        origin=Origin(xyz=(0.62, -0.52, 0.05)),
        material=frame_paint,
        name="right_support_inner_arm",
    )
    ground_frame.visual(
        Box((0.14, 0.12, 0.10)),
        origin=Origin(xyz=(0.94, -0.52, 0.05)),
        material=frame_paint,
        name="right_support_outer_arm",
    )
    ground_frame.visual(
        Box((0.16, 0.18, 0.185)),
        origin=Origin(xyz=(0.0, 0.71, 0.1925)),
        material=frame_paint,
        name="left_hinge_pedestal",
    )
    ground_frame.visual(
        Box((0.16, 0.18, 0.185)),
        origin=Origin(xyz=(0.0, -0.71, 0.1925)),
        material=frame_paint,
        name="right_hinge_pedestal",
    )
    ground_frame.visual(
        Box((0.26, 0.26, 0.02)),
        origin=Origin(xyz=(0.78, 0.52, 0.01)),
        material=frame_paint,
        name="left_support_base_plate",
    )
    ground_frame.visual(
        Box((0.26, 0.26, 0.02)),
        origin=Origin(xyz=(0.78, -0.52, 0.01)),
        material=frame_paint,
        name="right_support_base_plate",
    )
    ground_frame.visual(
        _square_tube_mesh(
            outer_width=0.16,
            inner_width=0.12,
            height=0.22,
            name="left_support_housing_mesh",
        ),
        origin=Origin(xyz=(0.78, 0.52, 0.13)),
        material=support_yellow,
        name="left_support_housing",
    )
    ground_frame.visual(
        _square_tube_mesh(
            outer_width=0.16,
            inner_width=0.12,
            height=0.22,
            name="right_support_housing_mesh",
        ),
        origin=Origin(xyz=(0.78, -0.52, 0.13)),
        material=support_yellow,
        name="right_support_housing",
    )

    # Platform: deck plate with welded beams, a central hinge barrel, and support shoes.
    platform.visual(
        Box((2.40, 2.00, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=deck_steel,
        name="deck_plate",
    )
    platform.visual(
        Box((2.32, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.94, 0.0)),
        material=frame_paint,
        name="left_side_beam",
    )
    platform.visual(
        Box((2.32, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.94, 0.0)),
        material=frame_paint,
        name="right_side_beam",
    )
    platform.visual(
        Box((0.12, 1.76, 0.12)),
        origin=Origin(xyz=(-1.14, 0.0, 0.0)),
        material=frame_paint,
        name="rear_edge_beam",
    )
    platform.visual(
        Box((0.12, 1.76, 0.12)),
        origin=Origin(xyz=(1.14, 0.0, 0.0)),
        material=frame_paint,
        name="front_edge_beam",
    )
    platform.visual(
        Box((0.22, 1.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=frame_paint,
        name="center_hinge_beam",
    )
    platform.visual(
        Box((0.92, 0.12, 0.10)),
        origin=Origin(xyz=(0.14, 0.52, -0.01)),
        material=frame_paint,
        name="left_stiffener",
    )
    platform.visual(
        Box((0.92, 0.12, 0.10)),
        origin=Origin(xyz=(0.14, -0.52, -0.01)),
        material=frame_paint,
        name="right_stiffener",
    )
    platform.visual(
        Box((0.22, 0.18, 0.03)),
        origin=Origin(xyz=(0.78, 0.52, -0.025)),
        material=frame_paint,
        name="left_support_pad",
    )
    platform.visual(
        Box((0.02, 0.16, 0.07)),
        origin=Origin(xyz=(0.68, 0.52, 0.025)),
        material=frame_paint,
        name="left_support_pad_inner_rib",
    )
    platform.visual(
        Box((0.02, 0.16, 0.07)),
        origin=Origin(xyz=(0.88, 0.52, 0.025)),
        material=frame_paint,
        name="left_support_pad_outer_rib",
    )
    platform.visual(
        Box((0.22, 0.18, 0.03)),
        origin=Origin(xyz=(0.78, -0.52, -0.025)),
        material=frame_paint,
        name="right_support_pad",
    )
    platform.visual(
        Box((0.02, 0.16, 0.07)),
        origin=Origin(xyz=(0.68, -0.52, 0.025)),
        material=frame_paint,
        name="right_support_pad_inner_rib",
    )
    platform.visual(
        Box((0.02, 0.16, 0.07)),
        origin=Origin(xyz=(0.88, -0.52, 0.025)),
        material=frame_paint,
        name="right_support_pad_outer_rib",
    )
    platform.visual(
        Cylinder(radius=0.055, length=1.40),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=deck_steel,
        name="hinge_barrel",
    )

    # Extending support legs: telescoping square columns with top contact shoes.
    for support_part in (left_support, right_support):
        support_part.visual(
            Box((0.10, 0.10, 0.24)),
            origin=Origin(xyz=(0.0, 0.0, -0.06)),
            material=rod_steel,
            name="inner_tube",
        )
        support_part.visual(
            Box((0.14, 0.14, 0.02)),
            origin=Origin(xyz=(0.0, 0.0, 0.01)),
            material=support_yellow,
            name="guide_collar",
        )
        support_part.visual(
            Box((0.18, 0.16, 0.02)),
            origin=Origin(xyz=(0.0, 0.0, 0.05)),
            material=rod_steel,
            name="head_pad",
        )

    model.articulation(
        "frame_to_platform",
        ArticulationType.REVOLUTE,
        parent=ground_frame,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.45,
            lower=0.0,
            upper=0.26,
        ),
    )
    model.articulation(
        "frame_to_left_support",
        ArticulationType.PRISMATIC,
        parent=ground_frame,
        child=left_support,
        origin=Origin(xyz=(0.78, 0.52, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.15,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "frame_to_right_support",
        ArticulationType.PRISMATIC,
        parent=ground_frame,
        child=right_support,
        origin=Origin(xyz=(0.78, -0.52, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.15,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_frame = object_model.get_part("ground_frame")
    platform = object_model.get_part("platform")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")
    platform_hinge = object_model.get_articulation("frame_to_platform")
    left_slide = object_model.get_articulation("frame_to_left_support")
    right_slide = object_model.get_articulation("frame_to_right_support")

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

    ctx.expect_contact(
        platform,
        ground_frame,
        elem_a="hinge_barrel",
        elem_b="left_hinge_pedestal",
        name="hinge barrel bears on left pedestal",
    )
    ctx.expect_contact(
        platform,
        ground_frame,
        elem_a="hinge_barrel",
        elem_b="right_hinge_pedestal",
        name="hinge barrel bears on right pedestal",
    )
    ctx.expect_contact(
        left_support,
        platform,
        elem_a="head_pad",
        elem_b="left_support_pad",
        name="left support roller contacts platform at rest",
    )
    ctx.expect_contact(
        right_support,
        platform,
        elem_a="head_pad",
        elem_b="right_support_pad",
        name="right support roller contacts platform at rest",
    )
    ctx.expect_contact(
        left_support,
        ground_frame,
        elem_a="guide_collar",
        elem_b="left_support_housing",
        name="left support collar seats on housing",
    )
    ctx.expect_contact(
        right_support,
        ground_frame,
        elem_a="guide_collar",
        elem_b="right_support_housing",
        name="right support collar seats on housing",
    )
    ctx.expect_within(
        left_support,
        ground_frame,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="left_support_housing",
        margin=0.0,
        name="left support rod stays centered in housing at rest",
    )
    ctx.expect_within(
        right_support,
        ground_frame,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="right_support_housing",
        margin=0.0,
        name="right support rod stays centered in housing at rest",
    )

    rest_front = ctx.part_element_world_aabb(platform, elem="front_edge_beam")
    with ctx.pose({platform_hinge: 0.23}):
        raised_front = ctx.part_element_world_aabb(platform, elem="front_edge_beam")
    ctx.check(
        "positive hinge rotation lifts front edge",
        rest_front is not None
        and raised_front is not None
        and raised_front[1][2] > rest_front[1][2] + 0.18,
        details=f"rest_front={rest_front}, raised_front={raised_front}",
    )

    rest_left_pos = ctx.part_world_position(left_support)
    with ctx.pose({left_slide: 0.18}):
        extended_left_pos = ctx.part_world_position(left_support)
    ctx.check(
        "left support extends upward with positive prismatic motion",
        rest_left_pos is not None
        and extended_left_pos is not None
        and extended_left_pos[2] > rest_left_pos[2] + 0.17,
        details=f"rest_left={rest_left_pos}, extended_left={extended_left_pos}",
    )

    with ctx.pose({platform_hinge: 0.2295, left_slide: 0.18, right_slide: 0.18}):
        ctx.expect_contact(
            left_support,
            platform,
            contact_tol=0.001,
            elem_a="head_pad",
            elem_b="left_support_pad",
            name="left support roller stays under tilted platform",
        )
        ctx.expect_contact(
            right_support,
            platform,
            contact_tol=0.001,
            elem_a="head_pad",
            elem_b="right_support_pad",
            name="right support roller stays under tilted platform",
        )
        ctx.expect_within(
            left_support,
            ground_frame,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="left_support_housing",
            margin=0.0,
            name="left support rod stays centered when extended",
        )
        ctx.expect_within(
            right_support,
            ground_frame,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="right_support_housing",
            margin=0.0,
            name="right support rod stays centered when extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
