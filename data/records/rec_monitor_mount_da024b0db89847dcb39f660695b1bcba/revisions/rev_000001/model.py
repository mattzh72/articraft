from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BACKPLATE_WIDTH = 0.120
BACKPLATE_HEIGHT = 0.180
BACKPLATE_THICKNESS = 0.008
BACKPLATE_CORNER_RADIUS = 0.012
BACKPLATE_SPINE_WIDTH = 0.046
BACKPLATE_SPINE_DEPTH = 0.028
BACKPLATE_SPINE_HEIGHT = 0.126
SHOULDER_OFFSET_Y = 0.048

PIVOT_POST_RADIUS = 0.0105
PIVOT_POST_HEIGHT = 0.026
PIVOT_HUB_HEIGHT = 0.010
PIVOT_STACK_OFFSET_Z = 0.008
COLLAR_OUTER_RADIUS = 0.021
COLLAR_INNER_RADIUS = 0.0132
COLLAR_HEIGHT = 0.018
JOINT_BOSS_RADIUS = 0.018
JOINT_BOSS_THICKNESS = 0.016
HEAD_BOSS_RADIUS = 0.014
HEAD_BOSS_THICKNESS = 0.016
JOINT_FACE_GAP = 0.0

ARM_BODY_WIDTH = 0.028
ARM_BODY_HEIGHT = 0.020
ARM_NECK_WIDTH = 0.010
ARM_NECK_HEIGHT = 0.016
LOWER_ARM_LENGTH = 0.170
UPPER_ARM_LENGTH = 0.180

SWIVEL_BODY_WIDTH = 0.036
SWIVEL_BODY_DEPTH = 0.030
SWIVEL_BODY_HEIGHT = 0.032
TILT_AXIS_OFFSET_Y = 0.034
TILT_FORK_WIDTH = 0.052
TILT_FORK_DEPTH = 0.018
TILT_FORK_HEIGHT = 0.030
TILT_FORK_SLOT_WIDTH = 0.028
TILT_BARREL_RADIUS = 0.006
TILT_BARREL_LENGTH = 0.024

VESA_PLATE_WIDTH = 0.118
VESA_PLATE_HEIGHT = 0.118
VESA_PLATE_THICKNESS = 0.004
VESA_PLATE_OFFSET_Y = 0.022
VESA_REINFORCEMENT_WIDTH = 0.060
VESA_REINFORCEMENT_DEPTH = 0.010
VESA_REINFORCEMENT_HEIGHT = 0.060
VESA_HOLE_SPACING = 0.100
VESA_HOLE_RADIUS = 0.003

SHOULDER_LIMITS = (-1.45, 1.45)
ELBOW_LIMITS = (-2.45, 2.45)
SWIVEL_LIMITS = (-1.75, 1.75)
TILT_LIMITS = (-0.55, 0.70)


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _joint_boss(center_y: float, *, radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(thickness / 2.0, both=True)
        .translate((0.0, center_y, 0.0))
    )


def _arm_link_shape(length: float) -> cq.Workplane:
    start_boss_y = JOINT_BOSS_RADIUS + JOINT_FACE_GAP
    end_boss_y = length - JOINT_BOSS_RADIUS - JOINT_FACE_GAP
    beam_start_y = start_boss_y
    beam_length = end_boss_y - beam_start_y

    start_boss = _joint_boss(
        start_boss_y,
        radius=JOINT_BOSS_RADIUS,
        thickness=JOINT_BOSS_THICKNESS,
    )
    end_boss = _joint_boss(
        end_boss_y,
        radius=JOINT_BOSS_RADIUS,
        thickness=JOINT_BOSS_THICKNESS,
    )
    beam = (
        cq.Workplane("XY")
        .box(
            ARM_BODY_WIDTH,
            beam_length,
            ARM_BODY_HEIGHT,
            centered=(True, False, True),
        )
        .translate((0.0, beam_start_y, 0.0))
        .edges("|Y")
        .fillet(0.004)
    )

    return _union_all(start_boss, beam, end_boss).clean()


def _backplate_shape() -> cq.Workplane:
    shoulder_boss_y = SHOULDER_OFFSET_Y - JOINT_BOSS_RADIUS - JOINT_FACE_GAP
    plate = (
        cq.Workplane("XY")
        .box(BACKPLATE_WIDTH, BACKPLATE_THICKNESS, BACKPLATE_HEIGHT)
        .translate((0.0, BACKPLATE_THICKNESS / 2.0, 0.0))
        .edges("|Y")
        .fillet(BACKPLATE_CORNER_RADIUS)
    )
    spine = (
        cq.Workplane("XY")
        .box(BACKPLATE_SPINE_WIDTH, BACKPLATE_SPINE_DEPTH, BACKPLATE_SPINE_HEIGHT)
        .translate((0.0, BACKPLATE_THICKNESS + BACKPLATE_SPINE_DEPTH / 2.0, 0.0))
        .edges("|Y")
        .fillet(0.008)
    )
    support_arm = (
        cq.Workplane("XY")
        .box(
            0.024,
            SHOULDER_OFFSET_Y - BACKPLATE_THICKNESS - JOINT_BOSS_RADIUS,
            0.022,
            centered=(True, False, True),
        )
        .translate((0.0, BACKPLATE_THICKNESS, 0.0))
        .edges("|Y")
        .fillet(0.004)
    )
    shoulder_boss = _joint_boss(
        shoulder_boss_y,
        radius=JOINT_BOSS_RADIUS,
        thickness=JOINT_BOSS_THICKNESS,
    )

    return _union_all(plate, spine, support_arm, shoulder_boss).clean()


def _swivel_head_shape() -> cq.Workplane:
    start_boss_y = HEAD_BOSS_RADIUS + JOINT_FACE_GAP
    fork_arm_width = (TILT_FORK_WIDTH - TILT_FORK_SLOT_WIDTH) / 2.0
    body_start_y = 0.0
    body_length = 0.026
    start_boss = _joint_boss(
        start_boss_y,
        radius=HEAD_BOSS_RADIUS,
        thickness=HEAD_BOSS_THICKNESS,
    )
    body = (
        cq.Workplane("XY")
        .box(
            SWIVEL_BODY_WIDTH,
            body_length,
            SWIVEL_BODY_HEIGHT,
            centered=(True, False, True),
        )
        .translate((0.0, body_start_y, 0.0))
        .edges("|Y")
        .fillet(0.004)
    )
    left_fork = (
        cq.Workplane("XY")
        .box(
            fork_arm_width,
            TILT_FORK_DEPTH,
            TILT_FORK_HEIGHT,
            centered=(True, False, True),
        )
        .translate(
            (
                -(TILT_FORK_SLOT_WIDTH / 2.0 + fork_arm_width / 2.0),
                TILT_AXIS_OFFSET_Y - TILT_FORK_DEPTH,
                0.0,
            )
        )
    )
    right_fork = (
        cq.Workplane("XY")
        .box(
            fork_arm_width,
            TILT_FORK_DEPTH,
            TILT_FORK_HEIGHT,
            centered=(True, False, True),
        )
        .translate(
            (
                TILT_FORK_SLOT_WIDTH / 2.0 + fork_arm_width / 2.0,
                TILT_AXIS_OFFSET_Y - TILT_FORK_DEPTH,
                0.0,
            )
        )
    )
    hinge_hole = (
        cq.Workplane("XY")
        .circle(TILT_BARREL_RADIUS + 0.0008)
        .extrude((TILT_FORK_WIDTH + 0.010) / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.0, TILT_AXIS_OFFSET_Y, 0.0))
    )

    return _union_all(start_boss, body, left_fork, right_fork).cut(hinge_hole).clean()


def _tilt_head_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XY")
        .circle(TILT_BARREL_RADIUS)
        .extrude(TILT_BARREL_LENGTH / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )
    web = (
        cq.Workplane("XY")
        .box(0.028, 0.014, 0.050, centered=(True, False, True))
        .translate((0.0, 0.0, 0.0))
        .edges("|Y")
        .fillet(0.003)
    )
    reinforcement = (
        cq.Workplane("XY")
        .box(
            VESA_REINFORCEMENT_WIDTH,
            VESA_REINFORCEMENT_DEPTH,
            VESA_REINFORCEMENT_HEIGHT,
        )
        .translate((0.0, VESA_PLATE_OFFSET_Y - 0.004, 0.0))
        .edges("|Y")
        .fillet(0.004)
    )
    plate = (
        cq.Workplane("XY")
        .box(VESA_PLATE_WIDTH, VESA_PLATE_THICKNESS, VESA_PLATE_HEIGHT)
        .translate((0.0, VESA_PLATE_OFFSET_Y, 0.0))
        .edges("|Y")
        .fillet(0.008)
    )

    result = _union_all(barrel, web, reinforcement, plate)
    for x_pos in (-VESA_HOLE_SPACING / 2.0, VESA_HOLE_SPACING / 2.0):
        for z_pos in (-VESA_HOLE_SPACING / 2.0, VESA_HOLE_SPACING / 2.0):
            hole = (
                cq.Workplane("XY")
                .circle(VESA_HOLE_RADIUS)
                .extrude(0.030 / 2.0, both=True)
                .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
                .translate((x_pos, VESA_PLATE_OFFSET_Y, z_pos))
            )
            result = result.cut(hole)

    return result.clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backplate_monitor_arm")

    model.material("powder_black", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("satin_aluminum", rgba=(0.71, 0.74, 0.77, 1.0))
    model.material("graphite", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("matte_steel", rgba=(0.56, 0.59, 0.62, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_backplate_shape(), "backplate_shell"),
        material="powder_black",
        name="backplate_shell",
    )
    backplate.inertial = Inertial.from_geometry(
        Box((BACKPLATE_WIDTH, SHOULDER_OFFSET_Y, BACKPLATE_HEIGHT)),
        mass=2.6,
        origin=Origin(xyz=(0.0, SHOULDER_OFFSET_Y / 2.0, 0.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_arm_link_shape(LOWER_ARM_LENGTH), "lower_arm_link"),
        material="satin_aluminum",
        name="lower_arm_link",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((ARM_BODY_WIDTH, LOWER_ARM_LENGTH, ARM_BODY_HEIGHT)),
        mass=0.75,
        origin=Origin(xyz=(0.0, LOWER_ARM_LENGTH / 2.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_arm_link_shape(UPPER_ARM_LENGTH), "upper_arm_link"),
        material="satin_aluminum",
        name="upper_arm_link",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((ARM_BODY_WIDTH, UPPER_ARM_LENGTH, ARM_BODY_HEIGHT)),
        mass=0.72,
        origin=Origin(xyz=(0.0, UPPER_ARM_LENGTH / 2.0, 0.0)),
    )

    swivel_head = model.part("swivel_head")
    swivel_head.visual(
        mesh_from_cadquery(_swivel_head_shape(), "swivel_head_shell"),
        material="graphite",
        name="swivel_head_shell",
    )
    swivel_head.inertial = Inertial.from_geometry(
        Box((TILT_FORK_WIDTH, TILT_AXIS_OFFSET_Y + 0.010, SWIVEL_BODY_HEIGHT)),
        mass=0.34,
        origin=Origin(xyz=(0.0, (TILT_AXIS_OFFSET_Y + 0.010) / 2.0, 0.0)),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        mesh_from_cadquery(_tilt_head_shape(), "tilt_head_shell"),
        material="matte_steel",
        name="tilt_head_shell",
    )
    tilt_head.inertial = Inertial.from_geometry(
        Box((VESA_PLATE_WIDTH, VESA_PLATE_OFFSET_Y + 0.012, VESA_PLATE_HEIGHT)),
        mass=0.44,
        origin=Origin(xyz=(0.0, VESA_PLATE_OFFSET_Y / 2.0, 0.0)),
    )

    model.articulation(
        "backplate_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=lower_arm,
        origin=Origin(xyz=(0.0, SHOULDER_OFFSET_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
            effort=28.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.0, LOWER_ARM_LENGTH, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
            effort=24.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "upper_arm_to_swivel_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=swivel_head,
        origin=Origin(xyz=(0.0, UPPER_ARM_LENGTH, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=SWIVEL_LIMITS[0],
            upper=SWIVEL_LIMITS[1],
            effort=16.0,
            velocity=2.2,
        ),
    )
    model.articulation(
        "swivel_head_to_tilt_head",
        ArticulationType.REVOLUTE,
        parent=swivel_head,
        child=tilt_head,
        origin=Origin(xyz=(0.0, TILT_AXIS_OFFSET_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=TILT_LIMITS[0],
            upper=TILT_LIMITS[1],
            effort=12.0,
            velocity=2.5,
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

    backplate = object_model.get_part("backplate")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    swivel_head = object_model.get_part("swivel_head")
    tilt_head = object_model.get_part("tilt_head")

    shoulder = object_model.get_articulation("backplate_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    swivel = object_model.get_articulation("upper_arm_to_swivel_head")
    tilt = object_model.get_articulation("swivel_head_to_tilt_head")

    def aabb_center(aabb):
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.expect_gap(
        upper_arm,
        backplate,
        axis="y",
        min_gap=0.14,
        name="upper arm projects clearly forward from the backplate",
    )
    ctx.expect_gap(
        tilt_head,
        backplate,
        axis="y",
        min_gap=0.34,
        name="mounting head sits well forward of the wall plate",
    )

    rest_head_pos = ctx.part_world_position(tilt_head)
    with ctx.pose({shoulder: 0.80}):
        shoulder_head_pos = ctx.part_world_position(tilt_head)
    ctx.check(
        "shoulder hinge swings the arm laterally",
        rest_head_pos is not None
        and shoulder_head_pos is not None
        and shoulder_head_pos[0] > rest_head_pos[0] + 0.18,
        details=f"rest={rest_head_pos}, posed={shoulder_head_pos}",
    )

    with ctx.pose({elbow: 0.90}):
        elbow_head_pos = ctx.part_world_position(tilt_head)
    ctx.check(
        "elbow hinge folds the outer link",
        rest_head_pos is not None
        and elbow_head_pos is not None
        and elbow_head_pos[0] > rest_head_pos[0] + 0.10,
        details=f"rest={rest_head_pos}, posed={elbow_head_pos}",
    )

    with ctx.pose({swivel: 0.85}):
        swivel_head_pos = ctx.part_world_position(tilt_head)
    ctx.check(
        "head swivel rotates the compact mount body",
        rest_head_pos is not None
        and swivel_head_pos is not None
        and swivel_head_pos[0] > rest_head_pos[0] + 0.015,
        details=f"rest={rest_head_pos}, posed={swivel_head_pos}",
    )

    rest_head_aabb = ctx.part_world_aabb(tilt_head)
    with ctx.pose({tilt: 0.45}):
        tilted_head_aabb = ctx.part_world_aabb(tilt_head)
    rest_center = aabb_center(rest_head_aabb) if rest_head_aabb is not None else None
    tilted_center = aabb_center(tilted_head_aabb) if tilted_head_aabb is not None else None
    ctx.check(
        "tilt hinge pitches the VESA plate upward",
        rest_center is not None
        and tilted_center is not None
        and tilted_center[2] > rest_center[2] + 0.003,
        details=f"rest_center={rest_center}, tilted_center={tilted_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
