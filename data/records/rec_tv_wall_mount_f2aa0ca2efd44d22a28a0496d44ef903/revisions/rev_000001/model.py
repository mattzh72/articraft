from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WALL_PLATE_WIDTH = 0.090
WALL_PLATE_HEIGHT = 0.250
WALL_PLATE_THICKNESS = 0.006
WALL_BRACKET_FRONT_CLEAR = 0.028
WALL_SPINE_DEPTH = 0.022
WALL_SPINE_WIDTH = 0.060
WALL_SPINE_HEIGHT = 0.118
PIVOT_TOWER_DEPTH = 0.012
PIVOT_TOWER_WIDTH = 0.050
PIVOT_TOWER_HEIGHT = 0.040

VERT_HUB_RADIUS = 0.016
VERT_HUB_HEIGHT = 0.020
YAW_EAR_THICKNESS = 0.008
YAW_EAR_CENTER_Y = 0.020
YAW_EAR_LENGTH = 0.022
YAW_EAR_HEIGHT = 0.030

LINK_1_LENGTH = 0.260
LINK_2_LENGTH = 0.240
ARM_WIDTH = 0.054
ARM_HEIGHT = 0.028
ARM_FRONT_CLEAR = 0.030

HEAD_SUPPORT_LENGTH = 0.062
HEAD_BODY_WIDTH = 0.036
HEAD_BODY_HEIGHT = 0.026

TILT_HUB_RADIUS = 0.014
TILT_HUB_LENGTH = 0.024
TILT_EAR_THICKNESS = 0.010
TILT_EAR_HEIGHT = 0.010

FRAME_OFFSET_X = 0.055
FRAME_DEPTH = 0.010
FRAME_WIDTH = 0.078
FRAME_HEIGHT = 0.106
FRAME_RING = 0.010

YAW_PARENT_BLOCK_X = 0.012
YAW_CHILD_BLOCK_X = 0.012
YAW_PARENT_BLOCK_Y = 0.048
YAW_PARENT_BLOCK_Z = 0.034
YAW_CHILD_BLOCK_Y = 0.036
YAW_CHILD_BLOCK_Z = 0.030
LINK_RAIL_OFFSET_Y = 0.018
LINK_RAIL_Y = 0.010
LINK_RAIL_Z = 0.010
LINK_BRIDGE_X = 0.028
TILT_PLATE_X = 0.012
TILT_PLATE_Y = 0.030
TILT_PLATE_Z = 0.044


def _yaw_hub() -> cq.Workplane:
    return cq.Workplane("XY").circle(VERT_HUB_RADIUS).extrude(VERT_HUB_HEIGHT / 2.0, both=True)


def _yaw_clevis(center_x: float) -> cq.Workplane:
    upper_ear = (
        cq.Workplane("XY")
        .box(YAW_EAR_LENGTH, YAW_EAR_THICKNESS, YAW_EAR_HEIGHT)
        .translate((center_x, YAW_EAR_CENTER_Y, 0.0))
    )
    lower_ear = (
        cq.Workplane("XY")
        .box(YAW_EAR_LENGTH, YAW_EAR_THICKNESS, YAW_EAR_HEIGHT)
        .translate((center_x, -YAW_EAR_CENTER_Y, 0.0))
    )
    return upper_ear.union(lower_ear)


def _tilt_barrel() -> cq.Workplane:
    return cq.Workplane("XZ").circle(TILT_HUB_RADIUS).extrude(TILT_HUB_LENGTH / 2.0, both=True)


def _tilt_clevis(center_x: float) -> cq.Workplane:
    z_offset = TILT_HUB_RADIUS + TILT_EAR_THICKNESS / 2.0
    top_ear = (
        cq.Workplane("XY")
        .box(0.016, TILT_HUB_LENGTH + 0.010, TILT_EAR_THICKNESS)
        .translate((center_x, 0.0, z_offset))
    )
    bottom_ear = (
        cq.Workplane("XY")
        .box(0.016, TILT_HUB_LENGTH + 0.010, TILT_EAR_THICKNESS)
        .translate((center_x, 0.0, -z_offset))
    )
    return top_ear.union(bottom_ear)


def _wall_bracket_shape() -> cq.Workplane:
    plate_center_x = -(WALL_BRACKET_FRONT_CLEAR + WALL_PLATE_THICKNESS / 2.0)
    spine_center_x = -(WALL_BRACKET_FRONT_CLEAR + WALL_SPINE_DEPTH / 2.0)
    tower_center_x = -(WALL_BRACKET_FRONT_CLEAR + PIVOT_TOWER_DEPTH / 2.0)

    plate = (
        cq.Workplane("XY")
        .box(WALL_PLATE_THICKNESS, WALL_PLATE_WIDTH, WALL_PLATE_HEIGHT)
        .translate((plate_center_x, 0.0, 0.0))
    )
    spine = (
        cq.Workplane("XY")
        .box(WALL_SPINE_DEPTH, WALL_SPINE_WIDTH, WALL_SPINE_HEIGHT)
        .translate((spine_center_x, 0.0, 0.0))
    )
    tower = (
        cq.Workplane("XY")
        .box(PIVOT_TOWER_DEPTH, PIVOT_TOWER_WIDTH, PIVOT_TOWER_HEIGHT)
        .translate((tower_center_x, 0.0, 0.0))
    )
    top_rib = (
        cq.Workplane("XY")
        .box(0.016, 0.030, 0.050)
        .translate((-(WALL_BRACKET_FRONT_CLEAR + 0.008), 0.0, 0.052))
    )
    bottom_rib = (
        cq.Workplane("XY")
        .box(0.016, 0.030, 0.050)
        .translate((-(WALL_BRACKET_FRONT_CLEAR + 0.008), 0.0, -0.052))
    )
    pivot_neck = (
        cq.Workplane("XY")
        .box(0.026, 0.026, 0.036)
        .translate((-0.017, 0.0, 0.0))
    )
    body = (
        plate.union(spine)
        .union(tower)
        .union(top_rib)
        .union(bottom_rib)
        .union(pivot_neck)
        .union(_yaw_clevis(-0.010))
    )

    mounting_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.020, 0.080),
                (0.020, 0.080),
                (-0.020, -0.080),
                (0.020, -0.080),
            ]
        )
        .circle(0.0045)
        .extrude(0.040, both=True)
    )
    return body.cut(mounting_holes)


def _arm_link_shape(length: float) -> cq.Workplane:
    rail_start = 0.018
    rail_end = length - 0.012
    rail_length = rail_end - rail_start
    rail_center_x = rail_start + rail_length / 2.0
    rail_thickness = 0.010
    rail_offset_y = 0.017

    rear_bridge = (
        cq.Workplane("XY")
        .box(0.026, 0.034, 0.024)
        .translate((0.015, 0.0, 0.0))
    )
    upper_rail = (
        cq.Workplane("XY")
        .box(rail_length, rail_thickness, 0.022)
        .translate((rail_center_x, rail_offset_y, 0.0))
    )
    lower_rail = (
        cq.Workplane("XY")
        .box(rail_length, rail_thickness, 0.022)
        .translate((rail_center_x, -rail_offset_y, 0.0))
    )
    rear_hub = _yaw_hub()
    front_clevis = _yaw_clevis(length - 0.010)

    return (
        rear_hub.union(rear_bridge)
        .union(upper_rail)
        .union(lower_rail)
        .union(front_clevis)
    )


def _swivel_head_shape() -> cq.Workplane:
    rear_hub = _yaw_hub()
    main_body = (
        cq.Workplane("XY")
        .box(0.040, HEAD_BODY_WIDTH, HEAD_BODY_HEIGHT)
        .translate((0.018, 0.0, 0.0))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.022, 0.022, 0.020)
        .translate((0.046, 0.0, 0.0))
    )
    return rear_hub.union(main_body).union(neck).union(_tilt_clevis(HEAD_SUPPORT_LENGTH))


def _output_frame_shape() -> cq.Workplane:
    tilt_hub = _tilt_barrel()
    frame_ring = (
        cq.Workplane("YZ")
        .rect(FRAME_WIDTH, FRAME_HEIGHT)
        .rect(FRAME_WIDTH - 2.0 * FRAME_RING, FRAME_HEIGHT - 2.0 * FRAME_RING)
        .extrude(FRAME_DEPTH / 2.0, both=True)
        .translate((FRAME_OFFSET_X, 0.0, 0.0))
    )
    upper_web = (
        cq.Workplane("XY")
        .box(FRAME_OFFSET_X, 0.012, 0.030)
        .translate((FRAME_OFFSET_X / 2.0, 0.0, FRAME_HEIGHT / 2.0 - FRAME_RING - 0.012))
    )
    lower_web = (
        cq.Workplane("XY")
        .box(FRAME_OFFSET_X, 0.012, 0.030)
        .translate((FRAME_OFFSET_X / 2.0, 0.0, -(FRAME_HEIGHT / 2.0 - FRAME_RING - 0.012)))
    )

    return tilt_hub.union(upper_web).union(lower_web).union(frame_ring)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_tv_wall_mount")

    model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("dark_steel", rgba=(0.32, 0.33, 0.36, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((WALL_PLATE_THICKNESS, WALL_PLATE_WIDTH, WALL_PLATE_HEIGHT)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        material="graphite",
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.022, 0.060, 0.118)),
        origin=Origin(xyz=(-0.026, 0.0, 0.0)),
        material="graphite",
        name="spine_block",
    )
    wall_bracket.visual(
        Box((0.012, 0.052, 0.040)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0)),
        material="graphite",
        name="shoulder_block",
    )
    wall_bracket.visual(
        Box((0.012, YAW_EAR_THICKNESS, 0.030)),
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        material="dark_steel",
        name="upper_shoulder_ear",
    )
    wall_bracket.visual(
        Box((0.012, YAW_EAR_THICKNESS, 0.030)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material="dark_steel",
        name="lower_shoulder_ear",
    )

    inner_link = model.part("inner_link")
    inner_link.visual(
        Box((0.012, 0.036, 0.030)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material="dark_steel",
        name="rear_lug",
    )
    inner_link.visual(
        Box((0.024, 0.044, 0.026)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material="powder_black",
        name="rear_block",
    )
    inner_link.visual(
        Box((0.198, 0.010, 0.010)),
        origin=Origin(xyz=(0.131, 0.018, 0.0)),
        material="powder_black",
        name="upper_rail",
    )
    inner_link.visual(
        Box((0.198, 0.010, 0.010)),
        origin=Origin(xyz=(0.131, -0.018, 0.0)),
        material="powder_black",
        name="lower_rail",
    )
    inner_link.visual(
        Box((0.028, 0.052, 0.026)),
        origin=Origin(xyz=(0.244, 0.0, 0.0)),
        material="powder_black",
        name="front_block",
    )
    inner_link.visual(
        Box((0.012, YAW_EAR_THICKNESS, 0.030)),
        origin=Origin(xyz=(LINK_1_LENGTH, 0.022, 0.0)),
        material="dark_steel",
        name="upper_elbow_ear",
    )
    inner_link.visual(
        Box((0.012, YAW_EAR_THICKNESS, 0.030)),
        origin=Origin(xyz=(LINK_1_LENGTH, -0.022, 0.0)),
        material="dark_steel",
        name="lower_elbow_ear",
    )

    outer_link = model.part("outer_link")
    outer_link.visual(
        Box((0.012, 0.036, 0.030)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material="dark_steel",
        name="rear_lug",
    )
    outer_link.visual(
        Box((0.024, 0.044, 0.026)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material="powder_black",
        name="rear_block",
    )
    outer_link.visual(
        Box((0.178, 0.010, 0.010)),
        origin=Origin(xyz=(0.121, 0.018, 0.0)),
        material="powder_black",
        name="upper_rail",
    )
    outer_link.visual(
        Box((0.178, 0.010, 0.010)),
        origin=Origin(xyz=(0.121, -0.018, 0.0)),
        material="powder_black",
        name="lower_rail",
    )
    outer_link.visual(
        Box((0.028, 0.052, 0.026)),
        origin=Origin(xyz=(0.224, 0.0, 0.0)),
        material="powder_black",
        name="front_block",
    )
    outer_link.visual(
        Box((0.012, YAW_EAR_THICKNESS, 0.030)),
        origin=Origin(xyz=(LINK_2_LENGTH, 0.022, 0.0)),
        material="dark_steel",
        name="upper_head_ear",
    )
    outer_link.visual(
        Box((0.012, YAW_EAR_THICKNESS, 0.030)),
        origin=Origin(xyz=(LINK_2_LENGTH, -0.022, 0.0)),
        material="dark_steel",
        name="lower_head_ear",
    )

    swivel_head = model.part("swivel_head")
    swivel_head.visual(
        Box((0.012, 0.036, 0.030)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material="dark_steel",
        name="rear_lug",
    )
    swivel_head.visual(
        Box((0.028, 0.040, 0.026)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material="dark_steel",
        name="head_body",
    )
    swivel_head.visual(
        Box((0.026, 0.028, 0.018)),
        origin=Origin(xyz=(0.047, 0.0, 0.0)),
        material="dark_steel",
        name="tilt_neck",
    )
    swivel_head.visual(
        Box((0.014, 0.006, 0.030)),
        origin=Origin(xyz=(0.057, 0.016, 0.0)),
        material="dark_steel",
        name="upper_side_rib",
    )
    swivel_head.visual(
        Box((0.014, 0.006, 0.030)),
        origin=Origin(xyz=(0.057, -0.016, 0.0)),
        material="dark_steel",
        name="lower_side_rib",
    )
    swivel_head.visual(
        Box((0.016, 0.034, 0.010)),
        origin=Origin(xyz=(HEAD_SUPPORT_LENGTH, 0.0, 0.019)),
        material="dark_steel",
        name="tilt_top_plate",
    )
    swivel_head.visual(
        Box((0.016, 0.034, 0.010)),
        origin=Origin(xyz=(HEAD_SUPPORT_LENGTH, 0.0, -0.019)),
        material="dark_steel",
        name="tilt_bottom_plate",
    )

    output_frame = model.part("output_frame")
    output_frame.visual(
        Box((0.018, 0.024, 0.028)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material="dark_steel",
        name="tilt_block",
    )
    output_frame.visual(
        Box((0.020, 0.012, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material="dark_steel",
        name="center_neck",
    )
    output_frame.visual(
        Box((0.010, 0.012, 0.048)),
        origin=Origin(xyz=(0.015, 0.0, 0.024)),
        material="powder_black",
        name="upper_front_strut",
    )
    output_frame.visual(
        Box((0.010, 0.012, 0.048)),
        origin=Origin(xyz=(0.015, 0.0, -0.024)),
        material="powder_black",
        name="lower_front_strut",
    )
    output_frame.visual(
        Box((0.045, 0.012, 0.010)),
        origin=Origin(xyz=(0.038, 0.0, 0.048)),
        material="powder_black",
        name="upper_web",
    )
    output_frame.visual(
        Box((0.045, 0.012, 0.010)),
        origin=Origin(xyz=(0.038, 0.0, -0.048)),
        material="powder_black",
        name="lower_web",
    )
    output_frame.visual(
        Box((0.010, 0.010, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.060, 0.034, 0.0)),
        material="powder_black",
        name="right_side_bar",
    )
    output_frame.visual(
        Box((0.010, 0.010, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.060, -0.034, 0.0)),
        material="powder_black",
        name="left_side_bar",
    )
    output_frame.visual(
        Box((0.010, FRAME_WIDTH, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, 0.048)),
        material="powder_black",
        name="top_bar",
    )
    output_frame.visual(
        Box((0.010, FRAME_WIDTH, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, -0.048)),
        material="powder_black",
        name="bottom_bar",
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=inner_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=80.0, velocity=1.2),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=inner_link,
        child=outer_link,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.95, upper=1.95, effort=60.0, velocity=1.5),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=outer_link,
        child=swivel_head,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.60, upper=1.60, effort=25.0, velocity=1.8),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_head,
        child=output_frame,
        origin=Origin(xyz=(HEAD_SUPPORT_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=0.35, effort=18.0, velocity=1.5),
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

    wall_bracket = object_model.get_part("wall_bracket")
    inner_link = object_model.get_part("inner_link")
    outer_link = object_model.get_part("outer_link")
    swivel_head = object_model.get_part("swivel_head")
    output_frame = object_model.get_part("output_frame")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_yaw = object_model.get_articulation("elbow_yaw")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.expect_contact(wall_bracket, inner_link, name="wall bracket carries the first arm")
    ctx.expect_contact(inner_link, outer_link, name="first arm carries the second arm")
    ctx.expect_contact(outer_link, swivel_head, name="second arm carries the swivel head")
    ctx.expect_contact(swivel_head, output_frame, name="swivel head carries the tilt frame")

    yaw_axes_ok = (
        tuple(shoulder_yaw.axis) == (0.0, 0.0, 1.0)
        and tuple(elbow_yaw.axis) == (0.0, 0.0, 1.0)
        and tuple(head_swivel.axis) == (0.0, 0.0, 1.0)
    )
    ctx.check(
        "fold and swivel joints are vertical-axis revolutes",
        yaw_axes_ok,
        details=(
            f"shoulder={tuple(shoulder_yaw.axis)}, "
            f"elbow={tuple(elbow_yaw.axis)}, swivel={tuple(head_swivel.axis)}"
        ),
    )
    ctx.check(
        "head tilt is a horizontal revolute",
        tuple(head_tilt.axis) == (0.0, -1.0, 0.0),
        details=f"tilt axis is {tuple(head_tilt.axis)}",
    )

    def _x_extent(part) -> float | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return aabb[1][0] - aabb[0][0]

    def _center_z(part) -> float | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    inner_dx = _x_extent(inner_link)
    outer_dx = _x_extent(outer_link)
    head_dx = _x_extent(swivel_head)
    ctx.check(
        "arm links are distinctly longer than the head support",
        inner_dx is not None
        and outer_dx is not None
        and head_dx is not None
        and inner_dx > head_dx * 2.5
        and outer_dx > head_dx * 2.2,
        details=f"inner_dx={inner_dx}, outer_dx={outer_dx}, head_dx={head_dx}",
    )

    outer_rest = ctx.part_world_position(outer_link)
    with ctx.pose(shoulder_yaw=0.90):
        outer_swung = ctx.part_world_position(outer_link)
        ctx.check(
            "shoulder joint swings the first link away from the wall",
            outer_rest is not None and outer_swung is not None and outer_swung[1] > outer_rest[1] + 0.16,
            details=f"rest={outer_rest}, swung={outer_swung}",
        )

    head_rest = ctx.part_world_position(swivel_head)
    with ctx.pose(elbow_yaw=0.90):
        head_folded = ctx.part_world_position(swivel_head)
        ctx.check(
            "elbow joint folds the second link",
            head_rest is not None and head_folded is not None and head_folded[1] > head_rest[1] + 0.15,
            details=f"rest={head_rest}, folded={head_folded}",
        )

    frame_rest = ctx.part_world_position(output_frame)
    with ctx.pose(head_swivel=0.75):
        frame_swiveled = ctx.part_world_position(output_frame)
        ctx.check(
            "head swivel turns the compact output frame",
            frame_rest is not None and frame_swiveled is not None and frame_swiveled[1] > frame_rest[1] + 0.03,
            details=f"rest={frame_rest}, swiveled={frame_swiveled}",
        )

    frame_rest_z = _center_z(output_frame)
    with ctx.pose(head_tilt=0.32):
        frame_tilt_z = _center_z(output_frame)
        ctx.check(
            "head tilt lifts the frame face",
            frame_rest_z is not None and frame_tilt_z is not None and frame_tilt_z > frame_rest_z + 0.004,
            details=f"rest_center_z={frame_rest_z}, tilted_center_z={frame_tilt_z}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
