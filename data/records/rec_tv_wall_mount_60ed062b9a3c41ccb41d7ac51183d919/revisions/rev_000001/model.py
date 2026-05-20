from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WALL_JOINT_Y = 0.029
PRIMARY_ARM_LENGTH = 0.255
SECONDARY_ARM_LENGTH = 0.220
FRAME_TILT_Y = 0.060


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _add_mesh_visual(part, shape: cq.Workplane, name: str, material: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, name),
        material=material,
        name=f"{name}_shell",
    )


def _wall_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.190, 0.012, 0.320, centered=(True, False, True))
    center_rib = (
        cq.Workplane("XY")
        .box(0.070, 0.018, 0.170, centered=(True, False, True))
        .translate((0.0, 0.011, 0.0))
    )
    swivel_pad = (
        cq.Workplane("XY")
        .box(0.074, 0.018, 0.094, centered=(True, False, True))
        .translate((0.0, 0.011, 0.0))
    )
    top_flange = (
        cq.Workplane("XY")
        .box(0.110, 0.010, 0.032, centered=(True, False, True))
        .translate((0.0, 0.002, 0.112))
    )
    bottom_flange = (
        cq.Workplane("XY")
        .box(0.110, 0.010, 0.032, centered=(True, False, True))
        .translate((0.0, 0.002, -0.112))
    )
    front_cap = (
        cq.Workplane("XY")
        .box(0.052, 0.009, 0.066, centered=(True, False, True))
        .translate((0.0, 0.020, 0.0))
    )
    return _union_all(plate, center_rib, swivel_pad, top_flange, bottom_flange, front_cap)


def _primary_arm_shape() -> cq.Workplane:
    root_block = cq.Workplane("XY").box(0.062, 0.040, 0.050, centered=(True, False, True))
    main_beam = (
        cq.Workplane("XY")
        .box(0.042, 0.210, 0.026, centered=(True, False, True))
        .translate((0.0, 0.020, 0.0))
    )
    nose_block = (
        cq.Workplane("XY")
        .box(0.058, 0.025, 0.046, centered=(True, False, True))
        .translate((0.0, 0.230, 0.0))
    )
    return _union_all(root_block, main_beam, nose_block)


def _secondary_arm_shape() -> cq.Workplane:
    root_block = cq.Workplane("XY").box(0.046, 0.032, 0.036, centered=(True, False, True))
    main_beam = (
        cq.Workplane("XY")
        .box(0.030, 0.184, 0.020, centered=(True, False, True))
        .translate((0.0, 0.018, 0.0))
    )
    nose_block = (
        cq.Workplane("XY")
        .box(0.042, 0.018, 0.034, centered=(True, False, True))
        .translate((0.0, 0.202, 0.0))
    )
    return _union_all(root_block, main_beam, nose_block)


def _head_yoke_shape() -> cq.Workplane:
    rear_hub = cq.Workplane("XY").box(0.052, 0.024, 0.046, centered=(True, False, True))
    cross_tie = (
        cq.Workplane("XY")
        .box(0.180, 0.024, 0.028, centered=(True, False, True))
        .translate((0.0, 0.012, 0.0))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(0.024, 0.060, 0.160, centered=(True, False, True))
        .translate((-0.078, 0.022, 0.0))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.024, 0.060, 0.160, centered=(True, False, True))
        .translate((0.078, 0.022, 0.0))
    )
    top_bridge = (
        cq.Workplane("XY")
        .box(0.180, 0.024, 0.020, centered=(True, False, True))
        .translate((0.0, 0.058, 0.060))
    )
    bottom_bridge = (
        cq.Workplane("XY")
        .box(0.180, 0.024, 0.020, centered=(True, False, True))
        .translate((0.0, 0.058, -0.060))
    )
    return _union_all(rear_hub, cross_tie, left_cheek, right_cheek, top_bridge, bottom_bridge)


def _frame_shape() -> cq.Workplane:
    pivot_crossbar = (
        cq.Workplane("XY")
        .box(0.180, 0.020, 0.024, centered=(True, False, True))
        .translate((0.0, 0.022, 0.0))
    )
    left_tilt_ear = (
        cq.Workplane("XY")
        .box(0.024, 0.020, 0.110, centered=(True, False, True))
        .translate((-0.078, 0.022, 0.0))
    )
    right_tilt_ear = (
        cq.Workplane("XY")
        .box(0.024, 0.020, 0.110, centered=(True, False, True))
        .translate((0.078, 0.022, 0.0))
    )
    center_spine = (
        cq.Workplane("XY")
        .box(0.026, 0.100, 0.180, centered=(True, False, True))
        .translate((0.0, 0.022, 0.0))
    )
    outer = cq.Workplane("XY").box(0.400, 0.018, 0.280, centered=(True, True, True)).translate((0.0, 0.110, 0.0))
    inner = cq.Workplane("XY").box(0.322, 0.030, 0.202, centered=(True, True, True)).translate((0.0, 0.110, 0.0))
    ring = outer.cut(inner)
    top_link = (
        cq.Workplane("XY")
        .box(0.180, 0.088, 0.028, centered=(True, False, True))
        .translate((0.0, 0.022, 0.100))
    )
    bottom_link = (
        cq.Workplane("XY")
        .box(0.180, 0.088, 0.028, centered=(True, False, True))
        .translate((0.0, 0.022, -0.100))
    )
    return _union_all(pivot_crossbar, left_tilt_ear, right_tilt_ear, center_spine, ring, top_link, bottom_link)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_bay_tv_wall_bracket")

    model.material("powder_black", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("satin_steel", rgba=(0.58, 0.60, 0.63, 1.0))

    wall_plate = model.part("wall_plate")
    primary_arm = model.part("primary_arm")
    secondary_arm = model.part("secondary_arm")
    head_yoke = model.part("head_yoke")
    frame = model.part("mounting_frame")

    _add_mesh_visual(wall_plate, _wall_plate_shape(), "wall_plate", "powder_black")
    _add_mesh_visual(primary_arm, _primary_arm_shape(), "primary_arm", "graphite")
    _add_mesh_visual(secondary_arm, _secondary_arm_shape(), "secondary_arm", "powder_black")
    _add_mesh_visual(head_yoke, _head_yoke_shape(), "head_yoke", "graphite")
    _add_mesh_visual(frame, _frame_shape(), "mounting_frame", "powder_black")

    model.articulation(
        "wall_pan",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=primary_arm,
        origin=Origin(xyz=(0.0, WALL_JOINT_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-1.20, upper=1.20, effort=60.0, velocity=1.3),
    )
    model.articulation(
        "elbow_pan",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(0.0, PRIMARY_ARM_LENGTH, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-2.30, upper=2.30, effort=40.0, velocity=1.5),
    )
    model.articulation(
        "frame_swivel",
        ArticulationType.REVOLUTE,
        parent=secondary_arm,
        child=head_yoke,
        origin=Origin(xyz=(0.0, SECONDARY_ARM_LENGTH, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.10, effort=20.0, velocity=1.8),
    )
    model.articulation(
        "frame_tilt",
        ArticulationType.REVOLUTE,
        parent=head_yoke,
        child=frame,
        origin=Origin(xyz=(0.0, FRAME_TILT_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.40, upper=0.52, effort=16.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    primary_arm = object_model.get_part("primary_arm")
    secondary_arm = object_model.get_part("secondary_arm")
    head_yoke = object_model.get_part("head_yoke")
    frame = object_model.get_part("mounting_frame")

    wall_pan = object_model.get_articulation("wall_pan")
    elbow_pan = object_model.get_articulation("elbow_pan")
    frame_swivel = object_model.get_articulation("frame_swivel")
    frame_tilt = object_model.get_articulation("frame_tilt")

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

    ctx.expect_contact(wall_plate, primary_arm, name="wall_plate_supports_primary_arm")
    ctx.expect_contact(primary_arm, secondary_arm, name="primary_arm_supports_secondary_arm")
    ctx.expect_contact(secondary_arm, head_yoke, name="secondary_arm_supports_swivel_head")
    ctx.expect_contact(head_yoke, frame, name="yoke_supports_tilt_frame")

    ctx.check(
        "pan_joints_use_vertical_axes",
        wall_pan.axis == (0.0, 0.0, -1.0)
        and elbow_pan.axis == (0.0, 0.0, -1.0)
        and frame_swivel.axis == (0.0, 0.0, -1.0),
        details=f"axes: wall={wall_pan.axis}, elbow={elbow_pan.axis}, swivel={frame_swivel.axis}",
    )
    ctx.check(
        "tilt_joint_uses_horizontal_axis",
        frame_tilt.axis == (-1.0, 0.0, 0.0),
        details=f"frame_tilt axis was {frame_tilt.axis}",
    )

    rest_frame_pos = ctx.part_world_position(frame)
    rest_frame_aabb = ctx.part_world_aabb(frame)

    with ctx.pose({wall_pan: 0.70}):
        swung_frame_pos = ctx.part_world_position(frame)
    ctx.check(
        "wall_pan_swings_frame_sideways",
        rest_frame_pos is not None
        and swung_frame_pos is not None
        and swung_frame_pos[0] > rest_frame_pos[0] + 0.14,
        details=f"rest={rest_frame_pos}, swung={swung_frame_pos}",
    )

    with ctx.pose({elbow_pan: -1.35}):
        folded_frame_pos = ctx.part_world_position(frame)
    ctx.check(
        "elbow_joint_folds_linkage",
        rest_frame_pos is not None
        and folded_frame_pos is not None
        and folded_frame_pos[1] < rest_frame_pos[1] - 0.12,
        details=f"rest={rest_frame_pos}, folded={folded_frame_pos}",
    )

    with ctx.pose({frame_swivel: 0.70}):
        swiveled_frame_aabb = ctx.part_world_aabb(frame)
        swiveled_frame_pos = ctx.part_world_position(frame)
    ctx.check(
        "frame_swivel_turns_mounting_frame",
        rest_frame_aabb is not None
        and swiveled_frame_aabb is not None
        and rest_frame_pos is not None
        and swiveled_frame_pos is not None
        and abs(swiveled_frame_pos[0] - rest_frame_pos[0]) > 0.03
        and abs((swiveled_frame_aabb[1][1] - swiveled_frame_aabb[0][1]) - (rest_frame_aabb[1][1] - rest_frame_aabb[0][1])) > 0.10,
        details=(
            f"rest_pos={rest_frame_pos}, swiveled_pos={swiveled_frame_pos}, "
            f"rest_aabb={rest_frame_aabb}, swiveled_aabb={swiveled_frame_aabb}"
        ),
    )

    with ctx.pose({frame_tilt: 0.32}):
        tilted_frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame_tilt_kicks_top_edge_forward",
        rest_frame_aabb is not None
        and tilted_frame_aabb is not None
        and tilted_frame_aabb[1][1] > rest_frame_aabb[1][1] + 0.03,
        details=f"rest_aabb={rest_frame_aabb}, tilted_aabb={tilted_frame_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
