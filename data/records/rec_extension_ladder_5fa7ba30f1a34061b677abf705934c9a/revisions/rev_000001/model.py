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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


RAIL_CENTER_X = 0.170
UPPER_AXIS_SPAN = 1.300
LOWER_AXIS_SPAN = 1.200
UPPER_TOP_MARGIN = 0.085
UPPER_BOTTOM_MARGIN = 0.055
LOWER_TOP_MARGIN = 0.050
LOWER_BOTTOM_MARGIN = 0.090
UPPER_RAIL_LENGTH = UPPER_AXIS_SPAN - UPPER_TOP_MARGIN - UPPER_BOTTOM_MARGIN
LOWER_RAIL_LENGTH = LOWER_AXIS_SPAN - LOWER_TOP_MARGIN - LOWER_BOTTOM_MARGIN
DEPLOY_ANGLE = math.radians(16.0)


def _section_loop(width: float, depth: float, z: float, radius: float) -> list[tuple[float, float, float]]:
    safe_radius = min(radius, width * 0.45, depth * 0.45)
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, safe_radius, corner_segments=6)]


def _rail_mesh(
    name: str,
    *,
    length: float,
    top_width: float,
    bottom_width: float,
    top_depth: float,
    bottom_depth: float,
    radius: float,
):
    mid_width = (top_width + bottom_width) * 0.5
    mid_depth = (top_depth + bottom_depth) * 0.5
    rail_geom = section_loft(
        [
            _section_loop(top_width, top_depth, 0.0, radius),
            _section_loop(mid_width, mid_depth, -0.5 * length, radius),
            _section_loop(bottom_width, bottom_depth, -length, radius),
        ]
    )
    return mesh_from_geometry(rail_geom, name)


def _add_rung_set(
    part,
    *,
    prefix: str,
    rung_centers: list[float],
    width: float,
    depth: float,
    height: float,
    material,
) -> None:
    for index, center_z in enumerate(rung_centers):
        part.visual(
            Box((width, depth, height)),
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            material=material,
            name=f"{prefix}_rung_{index}",
        )


def _build_foot_pad(part, *, material_rubber, material_steel, name_prefix: str) -> None:
    part.visual(
        Box((0.024, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=material_steel,
        name=f"{name_prefix}_pad_bracket",
    )
    part.visual(
        Box((0.078, 0.052, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=material_rubber,
        name=f"{name_prefix}_pad_sole",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.078, 0.052, 0.050)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_loft_ladder")

    plate_paint = model.material("plate_paint", rgba=(0.93, 0.93, 0.91, 1.0))
    aluminum = model.material("aluminum", rgba=(0.79, 0.81, 0.83, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.11, 1.0))

    upper_rail = _rail_mesh(
        "upper_ladder_rail",
        length=UPPER_RAIL_LENGTH,
        top_width=0.045,
        bottom_width=0.055,
        top_depth=0.028,
        bottom_depth=0.030,
        radius=0.006,
    )
    lower_rail = _rail_mesh(
        "lower_ladder_rail",
        length=LOWER_RAIL_LENGTH,
        top_width=0.046,
        bottom_width=0.058,
        top_depth=0.028,
        bottom_depth=0.032,
        radius=0.006,
    )

    ceiling_plate = model.part("ceiling_plate")
    ceiling_plate.visual(
        Box((0.600, 0.280, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=plate_paint,
        name="ceiling_panel",
    )
    ceiling_plate.visual(
        Box((0.420, 0.100, 0.040)),
        origin=Origin(xyz=(0.0, 0.035, -0.005)),
        material=steel,
        name="backer_channel",
    )
    ceiling_plate.visual(
        Box((0.090, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, 0.122, -0.025)),
        material=steel,
        name="ceiling_hinge_lug",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.011, length=0.140),
        origin=Origin(xyz=(0.0, 0.122, -0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="ceiling_hinge_barrel",
    )
    ceiling_plate.inertial = Inertial.from_geometry(
        Box((0.600, 0.280, 0.080)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    upper_section = model.part("upper_section")
    upper_section.visual(
        Cylinder(radius=0.0105, length=0.060),
        origin=Origin(xyz=(-0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="upper_top_left_knuckle",
    )
    upper_section.visual(
        Cylinder(radius=0.0105, length=0.060),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="upper_top_right_knuckle",
    )
    upper_section.visual(
        Box((0.100, 0.018, 0.090)),
        origin=Origin(xyz=(-0.120, -0.012, -0.045)),
        material=steel,
        name="upper_top_left_link",
    )
    upper_section.visual(
        Box((0.100, 0.018, 0.090)),
        origin=Origin(xyz=(0.120, -0.012, -0.045)),
        material=steel,
        name="upper_top_right_link",
    )
    upper_section.visual(
        upper_rail,
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, -UPPER_TOP_MARGIN)),
        material=aluminum,
        name="upper_left_rail",
    )
    upper_section.visual(
        upper_rail,
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, -UPPER_TOP_MARGIN)),
        material=aluminum,
        name="upper_right_rail",
    )
    _add_rung_set(
        upper_section,
        prefix="upper",
        rung_centers=[-0.220, -0.430, -0.640, -0.850, -1.060],
        width=0.305,
        depth=0.032,
        height=0.022,
        material=aluminum,
    )
    upper_section.visual(
        Box((0.120, 0.020, 0.056)),
        origin=Origin(xyz=(-0.110, -0.010, -(UPPER_AXIS_SPAN - 0.028))),
        material=steel,
        name="upper_center_left_link",
    )
    upper_section.visual(
        Box((0.120, 0.020, 0.056)),
        origin=Origin(xyz=(0.110, -0.010, -(UPPER_AXIS_SPAN - 0.028))),
        material=steel,
        name="upper_center_right_link",
    )
    upper_section.inertial = Inertial.from_geometry(
        Box((0.410, 0.060, UPPER_AXIS_SPAN)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -UPPER_AXIS_SPAN * 0.5)),
    )

    lower_section = model.part("lower_section")
    lower_section.visual(
        Box((0.120, 0.020, 0.056)),
        origin=Origin(xyz=(-0.110, 0.010, -0.028)),
        material=steel,
        name="lower_top_left_link",
    )
    lower_section.visual(
        Box((0.120, 0.020, 0.056)),
        origin=Origin(xyz=(0.110, 0.010, -0.028)),
        material=steel,
        name="lower_top_right_link",
    )
    lower_section.visual(
        lower_rail,
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, -LOWER_TOP_MARGIN)),
        material=aluminum,
        name="lower_left_rail",
    )
    lower_section.visual(
        lower_rail,
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, -LOWER_TOP_MARGIN)),
        material=aluminum,
        name="lower_right_rail",
    )
    _add_rung_set(
        lower_section,
        prefix="lower",
        rung_centers=[-0.200, -0.430, -0.660, -0.890],
        width=0.305,
        depth=0.032,
        height=0.022,
        material=aluminum,
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        rail_x = side_sign * RAIL_CENTER_X
        lower_section.visual(
            Box((0.020, 0.010, 0.110)),
            origin=Origin(xyz=(rail_x, 0.0, -(LOWER_AXIS_SPAN - 0.055))),
            material=steel,
            name=f"{side_name}_foot_mount",
        )
    lower_section.inertial = Inertial.from_geometry(
        Box((0.420, 0.065, LOWER_AXIS_SPAN)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -LOWER_AXIS_SPAN * 0.5)),
    )

    left_foot_pad = model.part("left_foot_pad")
    _build_foot_pad(left_foot_pad, material_rubber=rubber, material_steel=steel, name_prefix="left")

    right_foot_pad = model.part("right_foot_pad")
    _build_foot_pad(right_foot_pad, material_rubber=rubber, material_steel=steel, name_prefix="right")

    model.articulation(
        "ceiling_hinge",
        ArticulationType.REVOLUTE,
        parent=ceiling_plate,
        child=upper_section,
        origin=Origin(xyz=(0.0, 0.122, -0.035), rpy=(DEPLOY_ANGLE, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(76.0),
        ),
    )
    model.articulation(
        "center_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_section,
        child=lower_section,
        origin=Origin(xyz=(0.0, 0.0, -UPPER_AXIS_SPAN)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "left_pad_swivel",
        ArticulationType.REVOLUTE,
        parent=lower_section,
        child=left_foot_pad,
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, -LOWER_AXIS_SPAN)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-math.radians(20.0),
            upper=math.radians(20.0),
        ),
    )
    model.articulation(
        "right_pad_swivel",
        ArticulationType.REVOLUTE,
        parent=lower_section,
        child=right_foot_pad,
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, -LOWER_AXIS_SPAN)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-math.radians(20.0),
            upper=math.radians(20.0),
        ),
    )

    return model


def _check_bounded_joint_extremes(ctx: TestContext, joint) -> None:
    limits = joint.motion_limits
    if limits is None or limits.lower is None or limits.upper is None:
        return
    with ctx.pose({joint: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
        ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
    with ctx.pose({joint: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
        ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ceiling_plate = object_model.get_part("ceiling_plate")
    upper_section = object_model.get_part("upper_section")
    lower_section = object_model.get_part("lower_section")
    left_foot_pad = object_model.get_part("left_foot_pad")
    right_foot_pad = object_model.get_part("right_foot_pad")

    ceiling_hinge = object_model.get_articulation("ceiling_hinge")
    center_hinge = object_model.get_articulation("center_hinge")
    left_pad_swivel = object_model.get_articulation("left_pad_swivel")
    right_pad_swivel = object_model.get_articulation("right_pad_swivel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        upper_section,
        lower_section,
        elem_a="upper_center_left_link",
        elem_b="lower_top_left_link",
        reason="Center hinge leaves interleave as simplified solid straps instead of drilled clevis plates.",
    )
    ctx.allow_overlap(
        upper_section,
        lower_section,
        elem_a="upper_center_right_link",
        elem_b="lower_top_right_link",
        reason="Center hinge leaves interleave as simplified solid straps instead of drilled clevis plates.",
    )
    ctx.allow_overlap(
        lower_section,
        left_foot_pad,
        elem_a="left_foot_mount",
        elem_b="left_pad_bracket",
        reason="Rubber foot yoke is modeled as a solid bracket around the swivel mount without a through-hole.",
    )
    ctx.allow_overlap(
        lower_section,
        right_foot_pad,
        elem_a="right_foot_mount",
        elem_b="right_pad_bracket",
        reason="Rubber foot yoke is modeled as a solid bracket around the swivel mount without a through-hole.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=40, name="articulation_clearance")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=40,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(ceiling_plate, upper_section, name="ceiling_plate_to_upper_contact")
    ctx.expect_contact(upper_section, lower_section, name="upper_to_lower_contact")
    ctx.expect_contact(lower_section, left_foot_pad, name="lower_to_left_pad_contact")
    ctx.expect_contact(lower_section, right_foot_pad, name="lower_to_right_pad_contact")

    ctx.expect_origin_distance(
        left_foot_pad,
        right_foot_pad,
        axes="x",
        min_dist=0.32,
        max_dist=0.38,
        name="foot_pad_spacing",
    )
    ctx.expect_origin_gap(
        ceiling_plate,
        left_foot_pad,
        axis="z",
        min_gap=2.25,
        max_gap=2.55,
        name="left_pad_drop",
    )
    ctx.expect_origin_gap(
        ceiling_plate,
        right_foot_pad,
        axis="z",
        min_gap=2.25,
        max_gap=2.55,
        name="right_pad_drop",
    )
    ctx.expect_origin_distance(
        upper_section,
        lower_section,
        axes="x",
        max_dist=0.001,
        name="deployed_section_centered",
    )

    upper_rest_aabb = ctx.part_world_aabb(upper_section)
    lower_rest_aabb = ctx.part_world_aabb(lower_section)
    left_pad_rest_aabb = ctx.part_world_aabb(left_foot_pad)
    right_pad_rest_aabb = ctx.part_world_aabb(right_foot_pad)
    assert upper_rest_aabb is not None
    assert lower_rest_aabb is not None
    assert left_pad_rest_aabb is not None
    assert right_pad_rest_aabb is not None
    upper_rest_center = _aabb_center(upper_rest_aabb)
    lower_rest_center = _aabb_center(lower_rest_aabb)

    ceiling_limits = ceiling_hinge.motion_limits
    assert ceiling_limits is not None and ceiling_limits.upper is not None
    with ctx.pose({ceiling_hinge: ceiling_limits.upper}):
        upper_folded_aabb = ctx.part_world_aabb(upper_section)
        assert upper_folded_aabb is not None
        upper_folded_center = _aabb_center(upper_folded_aabb)
        assert upper_folded_center[1] > upper_rest_center[1] + 0.35
        assert upper_folded_center[2] > upper_rest_center[2] + 0.55
        ctx.expect_contact(ceiling_plate, upper_section, name="ceiling_contact_folded")

    center_limits = center_hinge.motion_limits
    assert center_limits is not None and center_limits.upper is not None
    with ctx.pose({center_hinge: center_limits.upper}):
        lower_folded_aabb = ctx.part_world_aabb(lower_section)
        assert lower_folded_aabb is not None
        lower_folded_center = _aabb_center(lower_folded_aabb)
        assert lower_folded_center[1] > lower_rest_center[1] + 0.25
        assert lower_folded_center[2] > lower_rest_center[2] + 0.30
        ctx.expect_contact(upper_section, lower_section, name="center_hinge_contact_folded")

    with ctx.pose({left_pad_swivel: math.radians(18.0)}):
        left_pad_raised = ctx.part_world_aabb(left_foot_pad)
        assert left_pad_raised is not None
        assert left_pad_raised[1][1] > left_pad_rest_aabb[1][1] + 0.006
        ctx.expect_contact(lower_section, left_foot_pad, name="left_pad_contact_swiveled")

    with ctx.pose({right_pad_swivel: -math.radians(18.0)}):
        right_pad_raised = ctx.part_world_aabb(right_foot_pad)
        assert right_pad_raised is not None
        assert right_pad_raised[0][1] < right_pad_rest_aabb[0][1] - 0.006
        ctx.expect_contact(lower_section, right_foot_pad, name="right_pad_contact_swiveled")

    for joint in (ceiling_hinge, center_hinge, left_pad_swivel, right_pad_swivel):
        _check_bounded_joint_extremes(ctx, joint)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
