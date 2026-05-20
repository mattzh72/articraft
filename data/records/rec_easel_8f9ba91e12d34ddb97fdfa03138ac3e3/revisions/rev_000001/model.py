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


UPRIGHT_X = 0.32
UPRIGHT_WIDTH = 0.05
UPRIGHT_DEPTH = 0.06
UPRIGHT_OUTER_X = UPRIGHT_X + UPRIGHT_WIDTH * 0.5


def _add_slider_carriages(part, *, prefix: str, carriage_height: float, material) -> None:
    shoe_width = 0.07
    shoe_depth = 0.03
    outer_plate_width = 0.02
    outer_plate_depth = 0.12
    front_y = 0.045
    back_y = -0.045
    outer_x = UPRIGHT_OUTER_X + outer_plate_width * 0.5

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        x_center = sign * UPRIGHT_X
        part.visual(
            Box((shoe_width, shoe_depth, carriage_height)),
            origin=Origin(xyz=(x_center, front_y, 0.0)),
            material=material,
            name=f"{prefix}_{side_name}_front_shoe",
        )
        part.visual(
            Box((shoe_width, shoe_depth, carriage_height)),
            origin=Origin(xyz=(x_center, back_y, 0.0)),
            material=material,
            name=f"{prefix}_{side_name}_back_shoe",
        )
        part.visual(
            Box((outer_plate_width, outer_plate_depth, carriage_height)),
            origin=Origin(xyz=(sign * outer_x, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_{side_name}_outer_plate",
        )


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_h_frame_easel")

    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.40, 0.42, 0.45, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    cork = model.material("cork", rgba=(0.63, 0.49, 0.31, 1.0))

    stand = model.part("stand")
    stand.inertial = Inertial.from_geometry(
        Box((0.74, 0.72, 1.92)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
    )
    stand.visual(
        Box((0.08, 0.72, 0.08)),
        origin=Origin(xyz=(-UPRIGHT_X, 0.0, 0.04)),
        material=aluminum,
        name="left_runner",
    )
    stand.visual(
        Box((0.08, 0.72, 0.08)),
        origin=Origin(xyz=(UPRIGHT_X, 0.0, 0.04)),
        material=aluminum,
        name="right_runner",
    )
    stand.visual(
        Box((0.74, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.27, 0.03)),
        material=aluminum,
        name="front_stabilizer",
    )
    stand.visual(
        Box((0.74, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, -0.27, 0.03)),
        material=aluminum,
        name="rear_stabilizer",
    )
    stand.visual(
        Box((UPRIGHT_WIDTH, UPRIGHT_DEPTH, 1.84)),
        origin=Origin(xyz=(-UPRIGHT_X, 0.0, 1.00)),
        material=aluminum,
        name="left_upright",
    )
    stand.visual(
        Box((UPRIGHT_WIDTH, UPRIGHT_DEPTH, 1.84)),
        origin=Origin(xyz=(UPRIGHT_X, 0.0, 1.00)),
        material=aluminum,
        name="right_upright",
    )
    stand.visual(
        Box((0.59, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=dark_aluminum,
        name="lower_stretcher",
    )
    stand.visual(
        Box((0.17, 0.05, 0.08)),
        origin=Origin(xyz=(-0.21, 0.0, 1.88)),
        material=dark_aluminum,
        name="left_top_shoulder",
    )
    stand.visual(
        Box((0.17, 0.05, 0.08)),
        origin=Origin(xyz=(0.21, 0.0, 1.88)),
        material=dark_aluminum,
        name="right_top_shoulder",
    )
    stand.visual(
        Box((0.09, 0.08, 0.12)),
        origin=Origin(xyz=(-0.25, 0.0, 1.06)),
        material=dark_aluminum,
        name="left_pivot_bracket",
    )
    stand.visual(
        Box((0.09, 0.08, 0.12)),
        origin=Origin(xyz=(0.25, 0.0, 1.06)),
        material=dark_aluminum,
        name="right_pivot_bracket",
    )

    top_grip_rail = model.part("top_grip_rail")
    top_grip_rail.inertial = Inertial.from_geometry(
        Box((0.74, 0.12, 0.14)),
        mass=2.8,
        origin=Origin(),
    )
    _add_slider_carriages(
        top_grip_rail,
        prefix="top_carriage",
        carriage_height=0.12,
        material=dark_aluminum,
    )
    top_grip_rail.visual(
        Box((0.05, 0.05, 0.06)),
        origin=Origin(xyz=(-0.275, 0.055, 0.0)),
        material=dark_aluminum,
        name="left_top_connector",
    )
    top_grip_rail.visual(
        Box((0.05, 0.05, 0.06)),
        origin=Origin(xyz=(0.275, 0.055, 0.0)),
        material=dark_aluminum,
        name="right_top_connector",
    )
    top_grip_rail.visual(
        Box((0.52, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        material=aluminum,
        name="top_grip_bar",
    )
    top_grip_rail.visual(
        Box((0.24, 0.035, 0.03)),
        origin=Origin(xyz=(0.0, 0.085, -0.035)),
        material=rubber,
        name="top_grip_pad",
    )
    top_grip_rail.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(0.0, 0.10, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_aluminum,
        name="top_grip_knob",
    )

    support_ledge = model.part("support_ledge")
    support_ledge.inertial = Inertial.from_geometry(
        Box((0.74, 0.30, 0.12)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.13, -0.01)),
    )
    _add_slider_carriages(
        support_ledge,
        prefix="support_carriage",
        carriage_height=0.10,
        material=dark_aluminum,
    )
    support_ledge.visual(
        Box((0.04, 0.04, 0.05)),
        origin=Origin(xyz=(-0.27, 0.06, 0.0)),
        material=dark_aluminum,
        name="left_support_connector",
    )
    support_ledge.visual(
        Box((0.04, 0.04, 0.05)),
        origin=Origin(xyz=(0.27, 0.06, 0.0)),
        material=dark_aluminum,
        name="right_support_connector",
    )
    support_ledge.visual(
        Box((0.52, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
        material=aluminum,
        name="support_beam",
    )
    support_ledge.visual(
        Box((0.56, 0.18, 0.02)),
        origin=Origin(xyz=(0.0, 0.17, -0.04)),
        material=aluminum,
        name="support_shelf",
    )
    support_ledge.visual(
        Box((0.54, 0.10, 0.006)),
        origin=Origin(xyz=(0.0, 0.17, -0.027)),
        material=cork,
        name="support_pad",
    )
    support_ledge.visual(
        Box((0.54, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.26, -0.02)),
        material=dark_aluminum,
        name="support_lip",
    )
    support_ledge.visual(
        Box((0.07, 0.14, 0.06)),
        origin=Origin(xyz=(-0.19, 0.13, -0.025)),
        material=dark_aluminum,
        name="left_shelf_brace",
    )
    support_ledge.visual(
        Box((0.07, 0.14, 0.06)),
        origin=Origin(xyz=(0.19, 0.13, -0.025)),
        material=dark_aluminum,
        name="right_shelf_brace",
    )

    tilt_post = model.part("tilt_post")
    tilt_post.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 1.18)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.08, 0.34)),
    )
    tilt_post.visual(
        Cylinder(radius=0.018, length=0.41),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_aluminum,
        name="tilt_axle",
    )
    tilt_post.visual(
        Box((0.16, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, -0.06, 0.0)),
        material=dark_aluminum,
        name="tilt_pivot_block",
    )
    tilt_post.visual(
        Box((0.10, 0.04, 1.14)),
        origin=Origin(xyz=(0.0, -0.14, 0.39)),
        material=aluminum,
        name="tilt_mast",
    )
    tilt_post.visual(
        Box((0.22, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, -0.14, 1.00)),
        material=dark_aluminum,
        name="tilt_post_head",
    )
    tilt_post.visual(
        Box((0.18, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, -0.14, -0.21)),
        material=dark_aluminum,
        name="tilt_post_foot",
    )

    model.articulation(
        "top_grip_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=top_grip_rail,
        origin=Origin(xyz=(0.0, 0.0, 1.54)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.22,
            lower=-0.32,
            upper=0.22,
        ),
    )
    model.articulation(
        "support_ledge_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=support_ledge,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.22,
            lower=-0.18,
            upper=0.28,
        ),
    )
    model.articulation(
        "tilt_post_hinge",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=tilt_post,
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.8,
            lower=-0.12,
            upper=0.38,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    top_grip_rail = object_model.get_part("top_grip_rail")
    support_ledge = object_model.get_part("support_ledge")
    tilt_post = object_model.get_part("tilt_post")

    top_grip_slide = object_model.get_articulation("top_grip_slide")
    support_ledge_slide = object_model.get_articulation("support_ledge_slide")
    tilt_post_hinge = object_model.get_articulation("tilt_post_hinge")

    top_grip_bar = top_grip_rail.get_visual("top_grip_bar")
    support_shelf = support_ledge.get_visual("support_shelf")
    tilt_post_head = tilt_post.get_visual("tilt_post_head")
    left_upright = stand.get_visual("left_upright")
    right_upright = stand.get_visual("right_upright")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.check(
        "top_grip_slide_axis",
        tuple(top_grip_slide.axis) == (0.0, 0.0, 1.0),
        f"Expected top_grip_slide axis (0, 0, 1), got {top_grip_slide.axis}",
    )
    ctx.check(
        "support_ledge_slide_axis",
        tuple(support_ledge_slide.axis) == (0.0, 0.0, 1.0),
        f"Expected support_ledge_slide axis (0, 0, 1), got {support_ledge_slide.axis}",
    )
    ctx.check(
        "tilt_post_hinge_axis",
        tuple(tilt_post_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected tilt_post_hinge axis (1, 0, 0), got {tilt_post_hinge.axis}",
    )

    stand_aabb = ctx.part_world_aabb(stand)
    left_upright_aabb = ctx.part_element_world_aabb(stand, elem=left_upright)
    right_upright_aabb = ctx.part_element_world_aabb(stand, elem=right_upright)
    assert stand_aabb is not None
    assert left_upright_aabb is not None
    assert right_upright_aabb is not None

    stand_width = stand_aabb[1][0] - stand_aabb[0][0]
    stand_depth = stand_aabb[1][1] - stand_aabb[0][1]
    stand_height = stand_aabb[1][2] - stand_aabb[0][2]
    left_upright_height = left_upright_aabb[1][2] - left_upright_aabb[0][2]
    upright_spacing = _aabb_center(right_upright_aabb)[0] - _aabb_center(left_upright_aabb)[0]

    ctx.check(
        "stand_realistic_width",
        0.70 <= stand_width <= 0.78,
        f"Stand width {stand_width:.3f} m should read as a full-size studio easel.",
    )
    ctx.check(
        "stand_realistic_depth",
        0.70 <= stand_depth <= 0.76,
        f"Stand depth {stand_depth:.3f} m should be stable for a tall easel.",
    )
    ctx.check(
        "stand_realistic_height",
        1.88 <= stand_height <= 1.95,
        f"Stand height {stand_height:.3f} m should match a studio H-frame easel.",
    )
    ctx.check(
        "uprights_are_tall",
        left_upright_height >= 1.80,
        f"Upright height {left_upright_height:.3f} m is too short for the requested easel.",
    )
    ctx.check(
        "uprights_evenly_spaced",
        0.62 <= upright_spacing <= 0.66,
        f"Upright spacing {upright_spacing:.3f} m should frame a central canvas area.",
    )

    ctx.expect_contact(top_grip_rail, stand, name="top_grip_contact_rest")
    ctx.expect_contact(support_ledge, stand, name="support_ledge_contact_rest")
    ctx.expect_contact(tilt_post, stand, name="tilt_post_contact_rest")
    ctx.expect_within(top_grip_rail, stand, axes="x", margin=0.0, name="top_grip_within_stand_width")
    ctx.expect_within(support_ledge, stand, axes="x", margin=0.0, name="support_ledge_within_stand_width")
    ctx.expect_within(tilt_post, stand, axes="x", margin=0.0, name="tilt_post_between_uprights")
    ctx.expect_gap(
        top_grip_rail,
        support_ledge,
        axis="z",
        positive_elem=top_grip_bar,
        negative_elem=support_shelf,
        min_gap=0.45,
        name="top_grip_above_support_ledge",
    )

    top_rest = ctx.part_world_position(top_grip_rail)
    support_rest = ctx.part_world_position(support_ledge)
    head_rest_aabb = ctx.part_element_world_aabb(tilt_post, elem=tilt_post_head)
    assert top_rest is not None
    assert support_rest is not None
    assert head_rest_aabb is not None
    head_rest_center = _aabb_center(head_rest_aabb)

    top_limits = top_grip_slide.motion_limits
    support_limits = support_ledge_slide.motion_limits
    tilt_limits = tilt_post_hinge.motion_limits
    assert top_limits is not None and top_limits.lower is not None and top_limits.upper is not None
    assert support_limits is not None and support_limits.lower is not None and support_limits.upper is not None
    assert tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None

    with ctx.pose({top_grip_slide: top_limits.upper}):
        top_upper = ctx.part_world_position(top_grip_rail)
        assert top_upper is not None
        ctx.check(
            "top_grip_moves_up",
            abs((top_upper[2] - top_rest[2]) - top_limits.upper) <= 1e-5,
            f"Top rail moved {top_upper[2] - top_rest[2]:.5f} m instead of {top_limits.upper:.5f} m upward.",
        )
        ctx.expect_contact(top_grip_rail, stand, name="top_grip_contact_upper")

    with ctx.pose({top_grip_slide: top_limits.lower}):
        top_lower = ctx.part_world_position(top_grip_rail)
        assert top_lower is not None
        ctx.check(
            "top_grip_moves_down",
            abs((top_lower[2] - top_rest[2]) - top_limits.lower) <= 1e-5,
            f"Top rail moved {top_lower[2] - top_rest[2]:.5f} m instead of {top_limits.lower:.5f} m downward.",
        )
        ctx.expect_contact(top_grip_rail, stand, name="top_grip_contact_lower")

    with ctx.pose({support_ledge_slide: support_limits.upper}):
        support_upper = ctx.part_world_position(support_ledge)
        assert support_upper is not None
        ctx.check(
            "support_ledge_moves_up",
            abs((support_upper[2] - support_rest[2]) - support_limits.upper) <= 1e-5,
            f"Support ledge moved {support_upper[2] - support_rest[2]:.5f} m instead of {support_limits.upper:.5f} m upward.",
        )
        ctx.expect_contact(support_ledge, stand, name="support_ledge_contact_upper")

    with ctx.pose({support_ledge_slide: support_limits.lower}):
        support_lower = ctx.part_world_position(support_ledge)
        assert support_lower is not None
        ctx.check(
            "support_ledge_moves_down",
            abs((support_lower[2] - support_rest[2]) - support_limits.lower) <= 1e-5,
            f"Support ledge moved {support_lower[2] - support_rest[2]:.5f} m instead of {support_limits.lower:.5f} m downward.",
        )
        ctx.expect_contact(support_ledge, stand, name="support_ledge_contact_lower")

    with ctx.pose({tilt_post_hinge: tilt_limits.upper}):
        head_back_aabb = ctx.part_element_world_aabb(tilt_post, elem=tilt_post_head)
        assert head_back_aabb is not None
        head_back_center = _aabb_center(head_back_aabb)
        ctx.check(
            "tilt_post_tilts_back",
            head_back_center[1] < head_rest_center[1] - 0.20,
            f"Tilt post head y moved to {head_back_center[1]:.3f} m, expected a clear rearward lean.",
        )
        ctx.expect_contact(tilt_post, stand, name="tilt_post_contact_upper")

    with ctx.pose({tilt_post_hinge: tilt_limits.lower}):
        head_forward_aabb = ctx.part_element_world_aabb(tilt_post, elem=tilt_post_head)
        assert head_forward_aabb is not None
        head_forward_center = _aabb_center(head_forward_aabb)
        ctx.check(
            "tilt_post_tilts_forward",
            head_forward_center[1] > head_rest_center[1] + 0.07,
            f"Tilt post head y moved to {head_forward_center[1]:.3f} m, expected a slight forward lean.",
        )
        ctx.expect_contact(tilt_post, stand, name="tilt_post_contact_lower")

    for joint in (top_grip_slide, support_ledge_slide, tilt_post_hinge):
        limits = joint.motion_limits
        assert limits is not None and limits.lower is not None and limits.upper is not None
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    with ctx.pose(
        {
            top_grip_slide: top_limits.lower,
            support_ledge_slide: support_limits.upper,
            tilt_post_hinge: tilt_limits.upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
