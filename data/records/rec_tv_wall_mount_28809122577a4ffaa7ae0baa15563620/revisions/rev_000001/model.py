from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WALL_PLATE_WIDTH = 0.095
WALL_PLATE_HEIGHT = 0.285
WALL_PLATE_THICKNESS = 0.010
WALL_AXIS_BACKSET = 0.028
WALL_SLOT_RADIUS = 0.0055

LINK_1_LENGTH = 0.265
LINK_2_LENGTH = 0.245
LINK_BODY_WIDTH = 0.050
LINK_BODY_HEIGHT = 0.030

VERT_JOINT_RADIUS = 0.021
VERT_TANG_HEIGHT = 0.010
VERT_EAR_HEIGHT = 0.012
VERT_STACK_HEIGHT = 2.0 * VERT_EAR_HEIGHT + VERT_TANG_HEIGHT

SWIVEL_NECK_LENGTH = 0.048
SWIVEL_BODY_WIDTH = 0.044
SWIVEL_BODY_HEIGHT = 0.028

TILT_RADIUS = 0.018
TILT_TANG_WIDTH = 0.014
TILT_EAR_WIDTH = 0.010
TILT_STACK_WIDTH = 2.0 * TILT_EAR_WIDTH + TILT_TANG_WIDTH

HEAD_PLATE_WIDTH = 0.160
HEAD_PLATE_HEIGHT = 0.120
HEAD_PLATE_THICKNESS = 0.006

VERT_SLOT_CLEAR_SPAN = 2.0 * VERT_JOINT_RADIUS + 0.010
TILT_SLOT_CLEAR_SPAN = 2.0 * TILT_RADIUS + 0.010
LINK_NECK_WIDTH = 0.032
LINK_BEAM_WIDTH = 0.040
LINK_BEAM_HEIGHT = 0.026


def _vertical_ears(center_x: float) -> cq.Workplane:
    ear_z = (VERT_TANG_HEIGHT + VERT_EAR_HEIGHT) / 2.0
    lower = _cyl_z(VERT_JOINT_RADIUS, VERT_EAR_HEIGHT, (center_x, 0.0, -ear_z))
    upper = _cyl_z(VERT_JOINT_RADIUS, VERT_EAR_HEIGHT, (center_x, 0.0, ear_z))
    return lower.union(upper)


def _tilt_ears(center_x: float) -> cq.Workplane:
    ear_y = (TILT_TANG_WIDTH + TILT_EAR_WIDTH) / 2.0
    left = _cyl_y(TILT_RADIUS, TILT_EAR_WIDTH, (center_x, -ear_y, 0.0))
    right = _cyl_y(TILT_RADIUS, TILT_EAR_WIDTH, (center_x, ear_y, 0.0))
    return left.union(right)


def _cyl_z(radius: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((center[0], center[1], center[2] - height / 2.0))
    )


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] - length / 2.0, center[2]))
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - length / 2.0, center[1], center[2]))
    )


def _filleted_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    selector: str = "|X",
    radius: float = 0.0,
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        solid = solid.edges(selector).fillet(radius)
    return solid.translate(center)


def _wall_plate_shape() -> cq.Workplane:
    plate_center_x = -(WALL_AXIS_BACKSET + WALL_PLATE_THICKNESS / 2.0)
    ear_z = (VERT_TANG_HEIGHT + VERT_EAR_HEIGHT) / 2.0
    plate = _filleted_box(
        (WALL_PLATE_THICKNESS, WALL_PLATE_WIDTH, WALL_PLATE_HEIGHT),
        (plate_center_x, 0.0, 0.0),
        selector="|Z",
        radius=0.003,
    )
    spine = _filleted_box(
        (WALL_AXIS_BACKSET, 0.060, 0.110),
        (-WALL_AXIS_BACKSET / 2.0, 0.0, 0.0),
        selector="|X",
        radius=0.004,
    )
    upper_lug = _filleted_box(
        (0.036, 0.050, 0.016),
        (-0.018, 0.0, ear_z),
        selector="|X",
        radius=0.003,
    )
    lower_lug = _filleted_box(
        (0.036, 0.050, 0.016),
        (-0.018, 0.0, -ear_z),
        selector="|X",
        radius=0.003,
    )
    ears = _vertical_ears(-VERT_JOINT_RADIUS)
    side_rib_1 = _filleted_box(
        (0.024, 0.012, 0.150),
        (-0.016, 0.024, 0.0),
        selector="|X",
        radius=0.0025,
    )
    side_rib_2 = _filleted_box(
        (0.024, 0.012, 0.150),
        (-0.016, -0.024, 0.0),
        selector="|X",
        radius=0.0025,
    )

    shape = plate.union(spine).union(upper_lug).union(lower_lug).union(ears).union(side_rib_1).union(side_rib_2)

    for y_pos in (-0.024, 0.024):
        for z_pos in (-0.088, 0.088):
            shape = shape.cut(
                _cyl_x(
                    WALL_SLOT_RADIUS,
                    WALL_PLATE_THICKNESS + 0.004,
                    (plate_center_x, y_pos, z_pos),
                )
            )
    return shape


def _link_shape(length: float) -> cq.Workplane:
    ear_z = (VERT_TANG_HEIGHT + VERT_EAR_HEIGHT) / 2.0
    beam_start = 0.064
    beam_end = length - 0.064
    beam = _filleted_box(
        (beam_end - beam_start, LINK_BEAM_WIDTH, LINK_BEAM_HEIGHT),
        ((beam_start + beam_end) / 2.0, 0.0, 0.0),
        selector="|X",
        radius=0.0045,
    )
    inner_tang = _cyl_z(VERT_JOINT_RADIUS, VERT_TANG_HEIGHT, (VERT_JOINT_RADIUS, 0.0, 0.0))
    inner_neck = _filleted_box(
        (0.044, LINK_NECK_WIDTH, VERT_TANG_HEIGHT),
        (0.022, 0.0, 0.0),
        selector="|X",
        radius=0.0025,
    )
    inner_transition = _filleted_box(
        (0.024, 0.034, 0.020),
        (0.054, 0.0, 0.0),
        selector="|X",
        radius=0.0025,
    )
    upper_lug = _filleted_box(
        (0.036, 0.036, VERT_EAR_HEIGHT),
        (length - 0.018, 0.0, ear_z),
        selector="|X",
        radius=0.003,
    )
    lower_lug = _filleted_box(
        (0.036, 0.036, VERT_EAR_HEIGHT),
        (length - 0.018, 0.0, -ear_z),
        selector="|X",
        radius=0.003,
    )
    outer_transition = _filleted_box(
        (0.024, 0.034, 0.020),
        (length - 0.054, 0.0, 0.0),
        selector="|X",
        radius=0.0025,
    )
    outer_ears = _vertical_ears(length - VERT_JOINT_RADIUS)
    return (
        beam.union(inner_tang)
        .union(inner_neck)
        .union(inner_transition)
        .union(upper_lug)
        .union(lower_lug)
        .union(outer_transition)
        .union(outer_ears)
    )


def _swivel_body_shape() -> cq.Workplane:
    ear_y = (TILT_TANG_WIDTH + TILT_EAR_WIDTH) / 2.0
    body = _filleted_box(
        (0.040, 0.034, 0.024),
        (0.028, 0.0, 0.0),
        selector="|X",
        radius=0.004,
    )
    inner_tang = _cyl_z(VERT_JOINT_RADIUS, VERT_TANG_HEIGHT, (VERT_JOINT_RADIUS, 0.0, 0.0))
    inner_neck = _filleted_box(
        (0.028, LINK_NECK_WIDTH, VERT_TANG_HEIGHT),
        (0.014, 0.0, 0.0),
        selector="|X",
        radius=0.0025,
    )
    left_arm = _filleted_box(
        (0.028, TILT_EAR_WIDTH, 0.030),
        (0.036, -ear_y, 0.0),
        selector="|X",
        radius=0.003,
    )
    right_arm = _filleted_box(
        (0.028, TILT_EAR_WIDTH, 0.030),
        (0.036, ear_y, 0.0),
        selector="|X",
        radius=0.003,
    )
    yoke = _tilt_ears(SWIVEL_NECK_LENGTH - TILT_RADIUS)
    return body.union(inner_tang).union(inner_neck).union(left_arm).union(right_arm).union(yoke)


def _head_shape() -> cq.Workplane:
    tilt_tang = _cyl_y(TILT_RADIUS, TILT_TANG_WIDTH, (TILT_RADIUS, 0.0, 0.0))
    support_block = _filleted_box(
        (0.022, TILT_TANG_WIDTH, 0.030),
        (0.011, 0.0, 0.0),
        selector="|X",
        radius=0.003,
    )
    center_boss = _filleted_box(
        (0.022, 0.030, 0.058),
        (0.025, 0.0, 0.0),
        selector="|X",
        radius=0.003,
    )
    spine = _filleted_box(
        (0.036, 0.052, 0.090),
        (0.040, 0.0, 0.0),
        selector="|X",
        radius=0.003,
    )
    plate = _filleted_box(
        (HEAD_PLATE_THICKNESS, HEAD_PLATE_WIDTH, HEAD_PLATE_HEIGHT),
        (0.056, 0.0, 0.0),
        selector="|Z",
        radius=0.002,
    )

    shape = tilt_tang.union(support_block).union(center_boss).union(spine).union(plate)

    for y_pos in (-0.050, 0.050):
        for z_pos in (-0.040, 0.040):
            shape = shape.cut(_cyl_x(0.0038, 0.016, (0.056, y_pos, z_pos)))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tv_wall_arm")

    model.material("powder_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("graphite", rgba=(0.21, 0.23, 0.25, 1.0))
    model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate"),
        material="powder_black",
        name="wall_plate_body",
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(_link_shape(LINK_1_LENGTH), "first_link"),
        material="dark_steel",
        name="first_link_body",
    )

    second_link = model.part("second_link")
    second_link.visual(
        mesh_from_cadquery(_link_shape(LINK_2_LENGTH), "second_link"),
        material="dark_steel",
        name="second_link_body",
    )

    swivel_body = model.part("swivel_body")
    swivel_body.visual(
        mesh_from_cadquery(_swivel_body_shape(), "swivel_body"),
        material="graphite",
        name="swivel_body_shell",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shape(), "head"),
        material="powder_black",
        name="vesa_head",
    )

    model.articulation(
        "wall_to_first",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=first_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.75, upper=1.75, effort=60.0, velocity=1.4),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.70, upper=2.70, effort=48.0, velocity=1.6),
    )
    model.articulation(
        "second_to_swivel",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=swivel_body,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.60, upper=1.60, effort=22.0, velocity=1.8),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel_body,
        child=head,
        origin=Origin(xyz=(SWIVEL_NECK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.25, effort=14.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    swivel_body = object_model.get_part("swivel_body")
    head = object_model.get_part("head")

    wall_to_first = object_model.get_articulation("wall_to_first")
    first_to_second = object_model.get_articulation("first_to_second")
    second_to_swivel = object_model.get_articulation("second_to_swivel")
    swivel_to_head = object_model.get_articulation("swivel_to_head")

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

    ctx.expect_contact(wall_plate, first_link, name="wall_hinge_contact")
    ctx.expect_contact(first_link, second_link, name="elbow_contact")
    ctx.expect_contact(second_link, swivel_body, name="head_swivel_contact")
    ctx.expect_contact(swivel_body, head, name="tilt_yoke_contact")
    ctx.expect_gap(head, wall_plate, axis="x", min_gap=0.42, name="head_reaches_out_from_wall")

    with ctx.pose({wall_to_first: 0.60}):
        ctx.expect_origin_gap(second_link, wall_plate, axis="y", min_gap=0.12, name="first_joint_positive_moves_arm_sideways")

    with ctx.pose({first_to_second: 0.80}):
        ctx.expect_origin_gap(swivel_body, second_link, axis="y", min_gap=0.14, name="second_joint_positive_folds_outer_link")

    with ctx.pose({second_to_swivel: 0.70}):
        ctx.expect_origin_gap(head, swivel_body, axis="y", min_gap=0.02, name="head_swivel_turns_head_sideways")

    ctx.check(
        "tilt_joint_axis_and_limits",
        swivel_to_head.axis == (0.0, -1.0, 0.0)
        and swivel_to_head.motion_limits is not None
        and swivel_to_head.motion_limits.lower is not None
        and swivel_to_head.motion_limits.upper is not None
        and swivel_to_head.motion_limits.lower < 0.0 < swivel_to_head.motion_limits.upper,
        details="Tilt joint should be a horizontal pitch axis with both down-tilt and slight up-tilt.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
