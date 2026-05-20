from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WALL_PLATE_WIDTH = 0.085
WALL_PLATE_HEIGHT = 0.160
WALL_PLATE_THICKNESS = 0.012
WALL_PLATE_X_CENTER = -0.028

YAW_JOINT_RADIUS = 0.016
PARENT_YAW_KNUCKLE = 0.018
CHILD_YAW_KNUCKLE = 0.016
YAW_KNUCKLE_OFFSET = 0.5 * (PARENT_YAW_KNUCKLE + CHILD_YAW_KNUCKLE)

PRIMARY_LINK_LENGTH = 0.215
SECONDARY_LINK_LENGTH = 0.190
ARM_WIDTH = 0.036
ARM_BODY_HEIGHT = 0.024
ARM_NECK_HEIGHT = 0.014
ARM_NECK_LENGTH = 0.032
FORK_LENGTH = 0.048
FORK_ARM_HEIGHT = 0.010
FORK_ARM_OFFSET_Z = 0.012

SWIVEL_BRACKET_LENGTH = 0.050
BRACKET_WIDTH = 0.028
BRACKET_HEIGHT = 0.020

TILT_JOINT_RADIUS = 0.010
PARENT_TILT_KNUCKLE = 0.013
CHILD_TILT_KNUCKLE = 0.016
TILT_KNUCKLE_OFFSET = 0.5 * (PARENT_TILT_KNUCKLE + CHILD_TILT_KNUCKLE)
TILT_YOKE_ARM_WIDTH = 0.008

HEAD_FRAME_OUTER_WIDTH = 0.140
HEAD_FRAME_OUTER_HEIGHT = 0.098
HEAD_FRAME_DEPTH = 0.012
HEAD_FRAME_RING = 0.014
HEAD_FRAME_CENTER_X = 0.045
HEAD_SPINE_LENGTH = 0.032
HEAD_SPINE_WIDTH = 0.026
HEAD_SPINE_HEIGHT = 0.032
HEAD_NECK_LENGTH = 0.016
HEAD_NECK_WIDTH = 0.020
HEAD_NECK_HEIGHT = 0.020


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XY").center(x, y).circle(radius).extrude(length, both=True).translate((0.0, 0.0, z))


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(length, both=True).translate((0.0, y, 0.0))


def _combine(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _keep_x_le(shape: cq.Workplane, x_max: float, size: float = 1.0) -> cq.Workplane:
    cutter = cq.Workplane("XY").box(size, size, size).translate((x_max - 0.5 * size, 0.0, 0.0))
    return shape.intersect(cutter)


def _keep_x_ge(shape: cq.Workplane, x_min: float, size: float = 1.0) -> cq.Workplane:
    cutter = cq.Workplane("XY").box(size, size, size).translate((x_min + 0.5 * size, 0.0, 0.0))
    return shape.intersect(cutter)


def _plate_with_mount_holes() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(WALL_PLATE_THICKNESS, WALL_PLATE_WIDTH, WALL_PLATE_HEIGHT)
        .translate((WALL_PLATE_X_CENTER, 0.0, 0.0))
        .edges("|X")
        .fillet(0.008)
    )
    for z in (-0.048, 0.048):
        hole = _y_cylinder(0.0045, WALL_PLATE_WIDTH + 0.010, (WALL_PLATE_X_CENTER, 0.0, z)).rotate(
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            90.0,
        )
        plate = plate.cut(hole)
    return plate


def make_wall_plate() -> cq.Workplane:
    plate = _plate_with_mount_holes()
    upper_tab = _box(0.024, 0.022, PARENT_YAW_KNUCKLE, (-0.012, 0.0, YAW_KNUCKLE_OFFSET))
    lower_tab = _box(0.024, 0.022, PARENT_YAW_KNUCKLE, (-0.012, 0.0, -YAW_KNUCKLE_OFFSET))
    upper_lug = _keep_x_le(_z_cylinder(YAW_JOINT_RADIUS, PARENT_YAW_KNUCKLE, (0.0, 0.0, YAW_KNUCKLE_OFFSET)), 0.0)
    lower_lug = _keep_x_le(_z_cylinder(YAW_JOINT_RADIUS, PARENT_YAW_KNUCKLE, (0.0, 0.0, -YAW_KNUCKLE_OFFSET)), 0.0)
    return _combine([plate, upper_tab, lower_tab, upper_lug, lower_lug])


def make_primary_link(length: float) -> cq.Workplane:
    split_start = length - FORK_LENGTH
    beam_center = (ARM_NECK_LENGTH + split_start) * 0.5
    beam_length = split_start - ARM_NECK_LENGTH
    shapes = [
        _keep_x_ge(_z_cylinder(YAW_JOINT_RADIUS, CHILD_YAW_KNUCKLE, (0.0, 0.0, 0.0)), 0.0),
        _box(ARM_NECK_LENGTH, ARM_WIDTH, ARM_NECK_HEIGHT, (0.5 * ARM_NECK_LENGTH, 0.0, 0.0)),
        _box(beam_length, ARM_WIDTH, ARM_BODY_HEIGHT, (beam_center, 0.0, 0.0)),
        _box(FORK_LENGTH, ARM_WIDTH, FORK_ARM_HEIGHT, (split_start + 0.5 * FORK_LENGTH, 0.0, FORK_ARM_OFFSET_Z)),
        _box(FORK_LENGTH, ARM_WIDTH, FORK_ARM_HEIGHT, (split_start + 0.5 * FORK_LENGTH, 0.0, -FORK_ARM_OFFSET_Z)),
        _keep_x_le(_z_cylinder(YAW_JOINT_RADIUS, PARENT_YAW_KNUCKLE, (length, 0.0, YAW_KNUCKLE_OFFSET)), length),
        _keep_x_le(_z_cylinder(YAW_JOINT_RADIUS, PARENT_YAW_KNUCKLE, (length, 0.0, -YAW_KNUCKLE_OFFSET)), length),
    ]
    return _combine(shapes)


def make_secondary_link(length: float) -> cq.Workplane:
    split_start = length - FORK_LENGTH
    beam_center = (ARM_NECK_LENGTH + split_start) * 0.5
    beam_length = split_start - ARM_NECK_LENGTH
    shapes = [
        _keep_x_ge(_z_cylinder(YAW_JOINT_RADIUS, CHILD_YAW_KNUCKLE, (0.0, 0.0, 0.0)), 0.0),
        _box(ARM_NECK_LENGTH, ARM_WIDTH, ARM_NECK_HEIGHT, (0.5 * ARM_NECK_LENGTH, 0.0, 0.0)),
        _box(beam_length, ARM_WIDTH * 0.95, ARM_BODY_HEIGHT * 0.95, (beam_center, 0.0, 0.0)),
        _box(FORK_LENGTH, ARM_WIDTH * 0.90, FORK_ARM_HEIGHT, (split_start + 0.5 * FORK_LENGTH, 0.0, FORK_ARM_OFFSET_Z)),
        _box(FORK_LENGTH, ARM_WIDTH * 0.90, FORK_ARM_HEIGHT, (split_start + 0.5 * FORK_LENGTH, 0.0, -FORK_ARM_OFFSET_Z)),
        _keep_x_le(_z_cylinder(YAW_JOINT_RADIUS, PARENT_YAW_KNUCKLE, (length, 0.0, YAW_KNUCKLE_OFFSET)), length),
        _keep_x_le(_z_cylinder(YAW_JOINT_RADIUS, PARENT_YAW_KNUCKLE, (length, 0.0, -YAW_KNUCKLE_OFFSET)), length),
    ]
    return _combine(shapes)


def make_tilt_bracket() -> cq.Workplane:
    body_end = SWIVEL_BRACKET_LENGTH - 0.022
    shapes = [
        _keep_x_ge(_z_cylinder(YAW_JOINT_RADIUS, CHILD_YAW_KNUCKLE, (0.0, 0.0, 0.0)), 0.0),
        _box(0.026, BRACKET_WIDTH, ARM_NECK_HEIGHT, (0.013, 0.0, 0.0)),
        _box(body_end - 0.026, BRACKET_WIDTH, BRACKET_HEIGHT, ((0.026 + body_end) * 0.5, 0.0, 0.0)),
        _box(0.022, TILT_YOKE_ARM_WIDTH, 0.026, (SWIVEL_BRACKET_LENGTH - 0.011, TILT_KNUCKLE_OFFSET, 0.0)),
        _box(0.022, TILT_YOKE_ARM_WIDTH, 0.026, (SWIVEL_BRACKET_LENGTH - 0.011, -TILT_KNUCKLE_OFFSET, 0.0)),
        _keep_x_le(_y_cylinder(TILT_JOINT_RADIUS, PARENT_TILT_KNUCKLE, (SWIVEL_BRACKET_LENGTH, TILT_KNUCKLE_OFFSET, 0.0)), SWIVEL_BRACKET_LENGTH),
        _keep_x_le(_y_cylinder(TILT_JOINT_RADIUS, PARENT_TILT_KNUCKLE, (SWIVEL_BRACKET_LENGTH, -TILT_KNUCKLE_OFFSET, 0.0)), SWIVEL_BRACKET_LENGTH),
    ]
    return _combine(shapes)


def make_head_frame() -> cq.Workplane:
    outer = _box(HEAD_FRAME_DEPTH, HEAD_FRAME_OUTER_WIDTH, HEAD_FRAME_OUTER_HEIGHT, (HEAD_FRAME_CENTER_X, 0.0, 0.0))
    inner = _box(
        HEAD_FRAME_DEPTH + 0.004,
        HEAD_FRAME_OUTER_WIDTH - 2.0 * HEAD_FRAME_RING,
        HEAD_FRAME_OUTER_HEIGHT - 2.0 * HEAD_FRAME_RING,
        (HEAD_FRAME_CENTER_X, 0.0, 0.0),
    )
    frame = outer.cut(inner)
    center_bar = _box(HEAD_FRAME_DEPTH, HEAD_FRAME_OUTER_WIDTH - 2.0 * HEAD_FRAME_RING, 0.012, (HEAD_FRAME_CENTER_X, 0.0, 0.0))
    spine = _box(HEAD_SPINE_LENGTH, HEAD_SPINE_WIDTH, HEAD_SPINE_HEIGHT, (0.5 * HEAD_SPINE_LENGTH, 0.0, 0.0))
    neck = _box(
        HEAD_NECK_LENGTH,
        HEAD_NECK_WIDTH,
        HEAD_NECK_HEIGHT,
        (HEAD_SPINE_LENGTH + 0.5 * HEAD_NECK_LENGTH, 0.0, 0.0),
    )
    lug = _keep_x_ge(_y_cylinder(TILT_JOINT_RADIUS, CHILD_TILT_KNUCKLE, (0.0, 0.0, 0.0)), 0.0)
    return _combine([frame, center_bar, spine, neck, lug])


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        0.5 * (lo[0] + hi[0]),
        0.5 * (lo[1] + hi[1]),
        0.5 * (lo[2] + hi[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_monitor_arm")

    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(make_wall_plate(), "wall_plate"),
        material=graphite,
        name="wall_plate_mesh",
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        mesh_from_cadquery(make_primary_link(PRIMARY_LINK_LENGTH), "primary_link"),
        material=dark_steel,
        name="primary_link_mesh",
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        mesh_from_cadquery(make_secondary_link(SECONDARY_LINK_LENGTH), "secondary_link"),
        material=dark_steel,
        name="secondary_link_mesh",
    )

    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.visual(
        mesh_from_cadquery(make_tilt_bracket(), "tilt_bracket"),
        material=satin_black,
        name="tilt_bracket_mesh",
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        mesh_from_cadquery(make_head_frame(), "head_frame"),
        material=graphite,
        name="head_frame_mesh",
    )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=primary_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-2.3, upper=2.3),
    )

    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(PRIMARY_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-2.5, upper=2.5),
    )

    model.articulation(
        "secondary_to_tilt_bracket",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=tilt_bracket,
        origin=Origin(xyz=(SECONDARY_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.9, upper=1.9),
    )

    model.articulation(
        "tilt_bracket_to_head_frame",
        ArticulationType.REVOLUTE,
        parent=tilt_bracket,
        child=head_frame,
        origin=Origin(xyz=(SWIVEL_BRACKET_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.75, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    tilt_bracket = object_model.get_part("tilt_bracket")
    head_frame = object_model.get_part("head_frame")

    wall_to_primary = object_model.get_articulation("wall_to_primary")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary")
    secondary_to_tilt_bracket = object_model.get_articulation("secondary_to_tilt_bracket")
    tilt_bracket_to_head_frame = object_model.get_articulation("tilt_bracket_to_head_frame")

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

    ctx.expect_contact(wall_plate, primary_link, contact_tol=0.001, name="wall plate supports primary link")
    ctx.expect_contact(primary_link, secondary_link, contact_tol=0.001, name="primary link supports secondary link")
    ctx.expect_contact(secondary_link, tilt_bracket, contact_tol=0.001, name="secondary link supports swivel bracket")
    ctx.expect_contact(tilt_bracket, head_frame, contact_tol=0.001, name="tilt bracket supports head frame")
    ctx.expect_origin_gap(head_frame, wall_plate, axis="x", min_gap=0.43, name="head frame projects outward from wall plate")

    with ctx.pose({wall_to_primary: 0.85, primary_to_secondary: 0.0, secondary_to_tilt_bracket: 0.0, tilt_bracket_to_head_frame: 0.0}):
        secondary_pos = ctx.part_world_position(secondary_link)
        ctx.check(
            "wall hinge swings arm laterally",
            secondary_pos is not None and secondary_pos[1] > 0.13 and abs(secondary_pos[2]) < 1e-4,
            f"secondary origin did not swing laterally as expected: {secondary_pos}",
        )

    with ctx.pose({wall_to_primary: 0.0, primary_to_secondary: 0.95, secondary_to_tilt_bracket: 0.0, tilt_bracket_to_head_frame: 0.0}):
        bracket_pos = ctx.part_world_position(tilt_bracket)
        ctx.check(
            "elbow hinge folds secondary link in plan",
            bracket_pos is not None and bracket_pos[1] > 0.12 and bracket_pos[0] < PRIMARY_LINK_LENGTH + SECONDARY_LINK_LENGTH,
            f"tilt bracket origin did not move in plan around elbow: {bracket_pos}",
        )

    with ctx.pose({wall_to_primary: 0.0, primary_to_secondary: 0.0, secondary_to_tilt_bracket: 0.75, tilt_bracket_to_head_frame: 0.0}):
        head_origin = ctx.part_world_position(head_frame)
        ctx.check(
            "swivel joint pans head assembly",
            head_origin is not None and head_origin[1] > 0.03,
            f"head origin did not pan laterally under swivel motion: {head_origin}",
        )

    rest_head_aabb = ctx.part_element_world_aabb(head_frame, elem="head_frame_mesh")
    rest_head_center = _aabb_center(rest_head_aabb)
    with ctx.pose({wall_to_primary: 0.0, primary_to_secondary: 0.0, secondary_to_tilt_bracket: 0.0, tilt_bracket_to_head_frame: 0.55}):
        tilted_head_aabb = ctx.part_element_world_aabb(head_frame, elem="head_frame_mesh")
        tilted_head_center = _aabb_center(tilted_head_aabb)
        ctx.check(
            "tilt joint lifts head frame on positive motion",
            (
                rest_head_center is not None
                and tilted_head_center is not None
                and rest_head_aabb is not None
                and tilted_head_aabb is not None
                and tilted_head_center[2] > rest_head_center[2] + 0.008
                and tilted_head_aabb[1][2] > rest_head_aabb[1][2] + 0.012
            ),
            f"head frame did not tilt upward enough: rest_center={rest_head_center}, tilted_center={tilted_head_center}, rest_aabb={rest_head_aabb}, tilted_aabb={tilted_head_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
