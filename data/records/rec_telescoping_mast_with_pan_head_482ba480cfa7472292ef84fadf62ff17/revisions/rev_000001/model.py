from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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


BASE_PLATE_LENGTH = 0.22
BASE_PLATE_WIDTH = 0.18
BASE_PLATE_THICKNESS = 0.016
MOUNT_HOLE_OFFSET_X = 0.082
MOUNT_HOLE_OFFSET_Y = 0.058
MOUNT_HOLE_DIAMETER = 0.012

OUTER_STAGE_OUTER = 0.09
OUTER_STAGE_WALL = 0.005
OUTER_STAGE_INNER = OUTER_STAGE_OUTER - 2.0 * OUTER_STAGE_WALL
OUTER_STAGE_HEIGHT = 0.39
OUTER_STAGE_BOTTOM_Z = BASE_PLATE_THICKNESS
OUTER_STAGE_TOP_Z = OUTER_STAGE_BOTTOM_Z + OUTER_STAGE_HEIGHT
OUTER_LOWER_SHROUD_SIZE = 0.12
OUTER_LOWER_SHROUD_HEIGHT = 0.05
OUTER_TOP_COLLAR_SIZE = 0.10
OUTER_TOP_COLLAR_HEIGHT = 0.025
OUTER_TOP_COLLAR_INNER = OUTER_STAGE_INNER

MIDDLE_STAGE_OUTER = 0.072
MIDDLE_STAGE_WALL = 0.004
MIDDLE_STAGE_INNER = MIDDLE_STAGE_OUTER - 2.0 * MIDDLE_STAGE_WALL
MIDDLE_STAGE_TOTAL_LENGTH = 0.56
MIDDLE_STAGE_BOTTOM_OFFSET = -0.36
MIDDLE_STAGE_TOP_Z = MIDDLE_STAGE_BOTTOM_OFFSET + MIDDLE_STAGE_TOTAL_LENGTH
MIDDLE_STAGE_TOP_COLLAR_SIZE = 0.082
MIDDLE_STAGE_TOP_COLLAR_HEIGHT = 0.028
MIDDLE_STAGE_TOP_COLLAR_INNER = MIDDLE_STAGE_INNER
MIDDLE_STAGE_TRAVEL = 0.22

TOP_STAGE_OUTER = 0.055
TOP_STAGE_WALL = 0.0035
TOP_STAGE_TUBE_LENGTH = 0.448
TOP_STAGE_BOTTOM_OFFSET = -0.32
TOP_STAGE_CAP_THICKNESS = 0.006
TOP_STAGE_BOSS_HEIGHT = 0.032
TOP_STAGE_TOP_Z = (
    TOP_STAGE_BOTTOM_OFFSET
    + TOP_STAGE_TUBE_LENGTH
    + TOP_STAGE_BOSS_HEIGHT
)
TOP_STAGE_BOSS_RADIUS = 0.022
TOP_STAGE_TRAVEL = 0.20

PAN_BASE_RADIUS = 0.03
PAN_BASE_HEIGHT = 0.014
PAN_DISK_RADIUS = 0.048
PAN_DISK_HEIGHT = 0.01
PAN_PLATE_LENGTH = 0.11
PAN_PLATE_WIDTH = 0.065
PAN_PLATE_HEIGHT = 0.012
PAN_SLOT_LENGTH = 0.05
PAN_SLOT_WIDTH = 0.012
PAN_POD_LENGTH = 0.03
PAN_POD_WIDTH = 0.024
PAN_POD_HEIGHT = 0.02
PAN_TOTAL_HEIGHT = PAN_BASE_HEIGHT + PAN_DISK_HEIGHT + PAN_PLATE_HEIGHT
PAN_YAW_LIMIT = pi


def _rect_ring(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        outer_x,
        outer_y,
        height,
        centered=(True, True, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            inner_x,
            inner_y,
            height + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(inner)


def _square_frame(outer_size: float, inner_size: float, length: float) -> cq.Workplane:
    wall = (outer_size - inner_size) / 2.0
    x_wall = cq.Workplane("XY").box(
        wall,
        outer_size,
        length,
        centered=(True, True, False),
    )
    y_wall = cq.Workplane("XY").box(
        inner_size,
        wall,
        length,
        centered=(True, True, False),
    )

    half_offset = outer_size / 2.0 - wall / 2.0
    frame = x_wall.translate((half_offset, 0.0, 0.0)).union(
        x_wall.translate((-half_offset, 0.0, 0.0))
    )
    frame = frame.union(y_wall.translate((0.0, half_offset, 0.0)))
    frame = frame.union(y_wall.translate((0.0, -half_offset, 0.0)))
    return frame


def _square_tube(size: float, wall: float, length: float) -> cq.Workplane:
    return _square_frame(size, size - 2.0 * wall, length)


def _guide_strip_cluster(
    base_size: float,
    pad_thickness: float,
    pad_width: float,
    pad_height: float,
    bottom_z: float,
) -> cq.Workplane:
    x_strip = cq.Workplane("XY").box(
        pad_thickness,
        pad_width,
        pad_height,
        centered=(True, True, False),
    )
    y_strip = cq.Workplane("XY").box(
        pad_width,
        pad_thickness,
        pad_height,
        centered=(True, True, False),
    )

    half_span = base_size / 2.0 + pad_thickness / 2.0
    cluster = x_strip.translate((half_span, 0.0, bottom_z)).union(
        x_strip.translate((-half_span, 0.0, bottom_z))
    )
    cluster = cluster.union(y_strip.translate((0.0, half_span, bottom_z)))
    cluster = cluster.union(y_strip.translate((0.0, -half_span, bottom_z)))
    return cluster


def _add_square_frame_visuals(
    part,
    *,
    prefix: str,
    outer_size: float,
    inner_size: float,
    height: float,
    bottom_z: float,
    material: str,
) -> None:
    wall = (outer_size - inner_size) / 2.0
    half_offset = outer_size / 2.0 - wall / 2.0
    center_z = bottom_z + height / 2.0

    part.visual(
        Box((wall, outer_size, height)),
        origin=Origin(xyz=(half_offset, 0.0, center_z)),
        material=material,
        name=f"{prefix}_wall_pos_x",
    )
    part.visual(
        Box((wall, outer_size, height)),
        origin=Origin(xyz=(-half_offset, 0.0, center_z)),
        material=material,
        name=f"{prefix}_wall_neg_x",
    )
    part.visual(
        Box((inner_size, wall, height)),
        origin=Origin(xyz=(0.0, half_offset, center_z)),
        material=material,
        name=f"{prefix}_wall_pos_y",
    )
    part.visual(
        Box((inner_size, wall, height)),
        origin=Origin(xyz=(0.0, -half_offset, center_z)),
        material=material,
        name=f"{prefix}_wall_neg_y",
    )


def _add_guide_visuals(
    part,
    *,
    prefix: str,
    parent_inner_size: float,
    thickness: float,
    width: float,
    height: float,
    bottom_z: float,
    material: str,
) -> None:
    half_offset = parent_inner_size / 2.0 - thickness / 2.0
    center_z = bottom_z + height / 2.0

    part.visual(
        Box((thickness, width, height)),
        origin=Origin(xyz=(half_offset, 0.0, center_z)),
        material=material,
        name=f"{prefix}_guide_pos_x",
    )
    part.visual(
        Box((thickness, width, height)),
        origin=Origin(xyz=(-half_offset, 0.0, center_z)),
        material=material,
        name=f"{prefix}_guide_neg_x",
    )
    part.visual(
        Box((width, thickness, height)),
        origin=Origin(xyz=(0.0, half_offset, center_z)),
        material=material,
        name=f"{prefix}_guide_pos_y",
    )
    part.visual(
        Box((width, thickness, height)),
        origin=Origin(xyz=(0.0, -half_offset, center_z)),
        material=material,
        name=f"{prefix}_guide_neg_y",
    )


def _base_sleeve_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(
            BASE_PLATE_LENGTH,
            BASE_PLATE_WIDTH,
            BASE_PLATE_THICKNESS,
            centered=(True, True, False),
        )
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-MOUNT_HOLE_OFFSET_X, -MOUNT_HOLE_OFFSET_Y),
                (-MOUNT_HOLE_OFFSET_X, MOUNT_HOLE_OFFSET_Y),
                (MOUNT_HOLE_OFFSET_X, -MOUNT_HOLE_OFFSET_Y),
                (MOUNT_HOLE_OFFSET_X, MOUNT_HOLE_OFFSET_Y),
            ]
        )
        .hole(MOUNT_HOLE_DIAMETER)
    )

    outer_tube = _square_tube(
        OUTER_STAGE_OUTER,
        OUTER_STAGE_WALL,
        OUTER_STAGE_HEIGHT,
    ).translate((0.0, 0.0, OUTER_STAGE_BOTTOM_Z))

    lower_shroud = _square_frame(
        OUTER_LOWER_SHROUD_SIZE,
        OUTER_STAGE_INNER,
        OUTER_LOWER_SHROUD_HEIGHT,
    ).translate((0.0, 0.0, OUTER_STAGE_BOTTOM_Z))

    top_collar = _square_frame(
        OUTER_TOP_COLLAR_SIZE,
        OUTER_TOP_COLLAR_INNER,
        OUTER_TOP_COLLAR_HEIGHT,
    ).translate((0.0, 0.0, OUTER_STAGE_TOP_Z - OUTER_TOP_COLLAR_HEIGHT))

    return plate.union(outer_tube).union(lower_shroud).union(top_collar)


def _middle_stage_shape() -> cq.Workplane:
    tube = _square_tube(
        MIDDLE_STAGE_OUTER,
        MIDDLE_STAGE_WALL,
        MIDDLE_STAGE_TOTAL_LENGTH,
    ).translate((0.0, 0.0, MIDDLE_STAGE_BOTTOM_OFFSET))
    guide_strips = _guide_strip_cluster(
        base_size=MIDDLE_STAGE_OUTER,
        pad_thickness=(OUTER_STAGE_INNER - MIDDLE_STAGE_OUTER) / 2.0,
        pad_width=0.018,
        pad_height=0.09,
        bottom_z=-0.345,
    )

    top_collar = _square_frame(
        MIDDLE_STAGE_TOP_COLLAR_SIZE,
        MIDDLE_STAGE_TOP_COLLAR_INNER,
        MIDDLE_STAGE_TOP_COLLAR_HEIGHT,
    ).translate((0.0, 0.0, MIDDLE_STAGE_TOP_Z - MIDDLE_STAGE_TOP_COLLAR_HEIGHT))

    return tube.union(top_collar).union(guide_strips)


def _top_stage_shape() -> cq.Workplane:
    tube = _square_tube(
        TOP_STAGE_OUTER,
        TOP_STAGE_WALL,
        TOP_STAGE_TUBE_LENGTH,
    ).translate((0.0, 0.0, TOP_STAGE_BOTTOM_OFFSET))
    guide_strips = _guide_strip_cluster(
        base_size=TOP_STAGE_OUTER,
        pad_thickness=(MIDDLE_STAGE_INNER - TOP_STAGE_OUTER) / 2.0,
        pad_width=0.014,
        pad_height=0.08,
        bottom_z=-0.30,
    )

    top_cap = cq.Workplane("XY").box(
        TOP_STAGE_OUTER,
        TOP_STAGE_OUTER,
        TOP_STAGE_CAP_THICKNESS,
        centered=(True, True, False),
    ).translate(
        (
            0.0,
            0.0,
            TOP_STAGE_BOTTOM_OFFSET + TOP_STAGE_TUBE_LENGTH - TOP_STAGE_CAP_THICKNESS,
        )
    )

    boss = (
        cq.Workplane("XY")
        .circle(TOP_STAGE_BOSS_RADIUS)
        .extrude(TOP_STAGE_BOSS_HEIGHT)
        .translate((0.0, 0.0, TOP_STAGE_BOTTOM_OFFSET + TOP_STAGE_TUBE_LENGTH))
    )

    return tube.union(top_cap).union(boss).union(guide_strips)


def _pan_plate_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(PAN_BASE_RADIUS).extrude(PAN_BASE_HEIGHT)
    disk = (
        cq.Workplane("XY")
        .circle(PAN_DISK_RADIUS)
        .extrude(PAN_DISK_HEIGHT)
        .translate((0.0, 0.0, PAN_BASE_HEIGHT))
    )

    top_plate = cq.Workplane("XY").box(
        PAN_PLATE_LENGTH,
        PAN_PLATE_WIDTH,
        PAN_PLATE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, PAN_BASE_HEIGHT + PAN_DISK_HEIGHT))

    slot_cutter = (
        cq.Workplane("XY")
        .slot2D(PAN_SLOT_LENGTH, PAN_SLOT_WIDTH)
        .extrude(PAN_PLATE_HEIGHT + 0.004)
        .translate((0.0, 0.0, PAN_BASE_HEIGHT + PAN_DISK_HEIGHT - 0.002))
    )

    motor_pod = cq.Workplane("XY").box(
        PAN_POD_LENGTH,
        PAN_POD_WIDTH,
        PAN_POD_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.042, 0.004))

    return base.union(disk).union(top_plate.cut(slot_cutter)).union(motor_pod)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_mast_pan_head")

    model.material("powder_black", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("dark_anodized", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("brushed_aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("machined_black", rgba=(0.11, 0.12, 0.13, 1.0))

    base_sleeve = model.part("base_sleeve")
    base_sleeve.visual(
        Box((BASE_PLATE_LENGTH, BASE_PLATE_WIDTH, BASE_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS / 2.0)),
        material="powder_black",
        name="base_plate",
    )
    _add_square_frame_visuals(
        base_sleeve,
        prefix="base_tube",
        outer_size=OUTER_STAGE_OUTER,
        inner_size=OUTER_STAGE_INNER,
        height=OUTER_STAGE_HEIGHT,
        bottom_z=OUTER_STAGE_BOTTOM_Z,
        material="powder_black",
    )
    _add_square_frame_visuals(
        base_sleeve,
        prefix="base_lower_shroud",
        outer_size=OUTER_LOWER_SHROUD_SIZE,
        inner_size=0.088,
        height=OUTER_LOWER_SHROUD_HEIGHT,
        bottom_z=OUTER_STAGE_BOTTOM_Z,
        material="powder_black",
    )
    _add_square_frame_visuals(
        base_sleeve,
        prefix="base_top_collar",
        outer_size=OUTER_TOP_COLLAR_SIZE,
        inner_size=0.088,
        height=OUTER_TOP_COLLAR_HEIGHT,
        bottom_z=OUTER_STAGE_TOP_Z - OUTER_TOP_COLLAR_HEIGHT,
        material="powder_black",
    )
    base_sleeve.visual(
        Cylinder(radius=0.02, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS + 0.006)),
        material="powder_black",
        name="base_spigot",
    )

    middle_stage = model.part("middle_stage")
    _add_square_frame_visuals(
        middle_stage,
        prefix="middle_tube",
        outer_size=MIDDLE_STAGE_OUTER,
        inner_size=MIDDLE_STAGE_INNER,
        height=MIDDLE_STAGE_TOTAL_LENGTH,
        bottom_z=MIDDLE_STAGE_BOTTOM_OFFSET,
        material="dark_anodized",
    )
    _add_square_frame_visuals(
        middle_stage,
        prefix="middle_top_collar",
        outer_size=MIDDLE_STAGE_TOP_COLLAR_SIZE,
        inner_size=0.068,
        height=MIDDLE_STAGE_TOP_COLLAR_HEIGHT,
        bottom_z=MIDDLE_STAGE_TOP_Z - MIDDLE_STAGE_TOP_COLLAR_HEIGHT,
        material="dark_anodized",
    )
    _add_guide_visuals(
        middle_stage,
        prefix="middle",
        parent_inner_size=OUTER_STAGE_INNER,
        thickness=0.005,
        width=0.018,
        height=0.09,
        bottom_z=-0.345,
        material="dark_anodized",
    )

    top_stage = model.part("top_stage")
    _add_square_frame_visuals(
        top_stage,
        prefix="top_tube",
        outer_size=TOP_STAGE_OUTER,
        inner_size=TOP_STAGE_OUTER - 2.0 * TOP_STAGE_WALL,
        height=TOP_STAGE_TUBE_LENGTH,
        bottom_z=TOP_STAGE_BOTTOM_OFFSET,
        material="brushed_aluminum",
    )
    _add_guide_visuals(
        top_stage,
        prefix="top",
        parent_inner_size=MIDDLE_STAGE_INNER,
        thickness=0.005,
        width=0.014,
        height=0.08,
        bottom_z=-0.30,
        material="brushed_aluminum",
    )
    top_stage.visual(
        Box((TOP_STAGE_OUTER, TOP_STAGE_OUTER, TOP_STAGE_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                TOP_STAGE_BOTTOM_OFFSET
                + TOP_STAGE_TUBE_LENGTH
                - TOP_STAGE_CAP_THICKNESS / 2.0,
            )
        ),
        material="brushed_aluminum",
        name="top_cap",
    )
    top_stage.visual(
        Cylinder(radius=TOP_STAGE_BOSS_RADIUS, length=TOP_STAGE_BOSS_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                TOP_STAGE_BOTTOM_OFFSET
                + TOP_STAGE_TUBE_LENGTH
                + TOP_STAGE_BOSS_HEIGHT / 2.0,
            )
        ),
        material="brushed_aluminum",
        name="top_boss",
    )

    pan_plate = model.part("pan_plate")
    pan_plate.visual(
        mesh_from_cadquery(_pan_plate_shape(), "pan_plate"),
        material="machined_black",
        name="pan_plate_body",
    )

    model.articulation(
        "sleeve_to_middle_stage",
        ArticulationType.PRISMATIC,
        parent=base_sleeve,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_STAGE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_STAGE_TRAVEL,
            effort=220.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "middle_to_top_stage",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_STAGE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TOP_STAGE_TRAVEL,
            effort=160.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "top_stage_to_pan_plate",
        ArticulationType.REVOLUTE,
        parent=top_stage,
        child=pan_plate,
        origin=Origin(xyz=(0.0, 0.0, TOP_STAGE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-PAN_YAW_LIMIT,
            upper=PAN_YAW_LIMIT,
            effort=20.0,
            velocity=1.4,
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

    base_sleeve = object_model.get_part("base_sleeve")
    middle_stage = object_model.get_part("middle_stage")
    top_stage = object_model.get_part("top_stage")
    pan_plate = object_model.get_part("pan_plate")

    sleeve_lift = object_model.get_articulation("sleeve_to_middle_stage")
    top_lift = object_model.get_articulation("middle_to_top_stage")
    pan_yaw = object_model.get_articulation("top_stage_to_pan_plate")

    ctx.check(
        "telescoping joints are vertical prisms",
        sleeve_lift.articulation_type == ArticulationType.PRISMATIC
        and top_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in sleeve_lift.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 6) for v in top_lift.axis) == (0.0, 0.0, 1.0),
        details=(
            f"sleeve axis={sleeve_lift.axis}, type={sleeve_lift.articulation_type}; "
            f"top axis={top_lift.axis}, type={top_lift.articulation_type}"
        ),
    )
    ctx.check(
        "pan joint yaws about mast centerline",
        pan_yaw.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in pan_yaw.axis) == (0.0, 0.0, 1.0)
        and pan_yaw.motion_limits is not None
        and pan_yaw.motion_limits.lower == -PAN_YAW_LIMIT
        and pan_yaw.motion_limits.upper == PAN_YAW_LIMIT,
        details=(
            f"axis={pan_yaw.axis}, type={pan_yaw.articulation_type}, "
            f"limits={pan_yaw.motion_limits}"
        ),
    )

    ctx.expect_within(
        middle_stage,
        base_sleeve,
        axes="xy",
        margin=0.002,
        name="middle stage stays centered in fixed sleeve",
    )
    ctx.expect_overlap(
        middle_stage,
        base_sleeve,
        axes="z",
        min_overlap=0.15,
        name="middle stage remains inserted in sleeve at rest",
    )
    ctx.expect_within(
        top_stage,
        middle_stage,
        axes="xy",
        margin=0.0025,
        name="top stage stays centered in middle stage",
    )
    ctx.expect_overlap(
        top_stage,
        middle_stage,
        axes="z",
        min_overlap=0.13,
        name="top stage remains inserted in middle stage at rest",
    )
    ctx.expect_gap(
        pan_plate,
        top_stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan plate seats on top stage",
    )
    ctx.expect_overlap(
        pan_plate,
        top_stage,
        axes="xy",
        min_overlap=0.04,
        name="pan plate stays centered over mast",
    )

    rest_middle_pos = ctx.part_world_position(middle_stage)
    rest_top_pos = ctx.part_world_position(top_stage)
    with ctx.pose({pan_yaw: 0.0}):
        rest_pan_pos = ctx.part_world_position(pan_plate)
    with ctx.pose({pan_yaw: pi / 2.0}):
        yaw_only_pan_pos = ctx.part_world_position(pan_plate)

    with ctx.pose(
        {
            sleeve_lift: MIDDLE_STAGE_TRAVEL,
            top_lift: TOP_STAGE_TRAVEL,
            pan_yaw: pi / 2.0,
        }
    ):
        ctx.expect_within(
            middle_stage,
            base_sleeve,
            axes="xy",
            margin=0.002,
            name="extended middle stage stays centered in sleeve",
        )
        ctx.expect_overlap(
            middle_stage,
            base_sleeve,
            axes="z",
            min_overlap=0.12,
            name="extended middle stage retains insertion in sleeve",
        )
        ctx.expect_within(
            top_stage,
            middle_stage,
            axes="xy",
            margin=0.0025,
            name="extended top stage stays centered in middle stage",
        )
        ctx.expect_overlap(
            top_stage,
            middle_stage,
            axes="z",
            min_overlap=0.10,
            name="extended top stage retains insertion in middle stage",
        )
        ctx.expect_gap(
            pan_plate,
            top_stage,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="yawed pan plate remains seated on top stage",
        )
        ctx.expect_origin_distance(
            pan_plate,
            top_stage,
            axes="xy",
            max_dist=1e-6,
            name="yaw axis remains on mast centerline",
        )

        extended_middle_pos = ctx.part_world_position(middle_stage)
        extended_top_pos = ctx.part_world_position(top_stage)

    ctx.check(
        "middle stage extends upward",
        rest_middle_pos is not None
        and extended_middle_pos is not None
        and extended_middle_pos[2] > rest_middle_pos[2] + 0.18,
        details=f"rest={rest_middle_pos}, extended={extended_middle_pos}",
    )
    ctx.check(
        "top stage extends upward through both telescoping joints",
        rest_top_pos is not None
        and extended_top_pos is not None
        and extended_top_pos[2] > rest_top_pos[2] + 0.38,
        details=f"rest={rest_top_pos}, extended={extended_top_pos}",
    )
    ctx.check(
        "pan plate origin stays fixed during yaw",
        rest_pan_pos is not None
        and yaw_only_pan_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pan_pos, yaw_only_pan_pos)) < 1e-6,
        details=f"rest={rest_pan_pos}, yawed={yaw_only_pan_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
