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


PLATE_T = 0.012
PLATE_W = 0.160
PLATE_H = 0.280
EAR_LEN = 0.028
EAR_W = 0.042
EAR_Z = 0.032
EAR_Z_OFFSET = 0.032
MOUNT_HOLE_D = 0.010

ARM_LUG_LEN = 0.030
ARM_LUG_W = 0.034
ARM_LUG_Z = 0.032
ARM_BODY_LEN = 0.230
ARM_BODY_W = 0.028
ARM_BODY_H = 0.018
ARM_FRONT_BLOCK_LEN = 0.045
ARM_FRONT_BLOCK_W = 0.032
ARM_FRONT_BLOCK_H = 0.028
RAIL_W = 0.008
RAIL_H = 0.012
RAIL_Y = 0.011

SLIDE_START = 0.270
GUIDE_LEN = 0.140
SLIDE_TRAVEL = 0.080

STAGE_LEN = 0.160
STAGE_W = 0.058
STAGE_H = 0.040
STAGE_PAD_LEN = 0.080
STAGE_PAD_W = 0.070
STAGE_PAD_H = 0.010
STAGE_FRONT_STOP = 0.012


def _make_side_plate() -> cq.Workplane:
    hole_pts = [
        (-PLATE_W * 0.28, PLATE_H * 0.32),
        (PLATE_W * 0.28, PLATE_H * 0.32),
        (-PLATE_W * 0.28, -PLATE_H * 0.32),
        (PLATE_W * 0.28, -PLATE_H * 0.32),
    ]

    plate = (
        cq.Workplane("XY")
        .box(PLATE_T, PLATE_W, PLATE_H)
        .translate((-PLATE_T / 2.0, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(hole_pts)
        .hole(MOUNT_HOLE_D)
    )

    upper_ear = (
        cq.Workplane("XY")
        .box(EAR_LEN, EAR_W, EAR_Z, centered=(False, True, True))
        .translate((0.0, 0.0, EAR_Z_OFFSET))
    )
    lower_ear = (
        cq.Workplane("XY")
        .box(EAR_LEN, EAR_W, EAR_Z, centered=(False, True, True))
        .translate((0.0, 0.0, -EAR_Z_OFFSET))
    )
    spine = (
        cq.Workplane("XY")
        .box(PLATE_T * 1.6, 0.070, 0.120)
        .translate((-PLATE_T * 0.20, 0.0, 0.0))
    )

    return plate.union(spine).union(upper_ear).union(lower_ear)


def _make_swing_arm() -> cq.Workplane:
    lug = (
        cq.Workplane("XY")
        .box(ARM_LUG_LEN, ARM_LUG_W, ARM_LUG_Z, centered=(False, True, True))
    )
    body = (
        cq.Workplane("XY")
        .box(ARM_BODY_LEN, ARM_BODY_W, ARM_BODY_H, centered=(False, True, True))
        .translate((ARM_LUG_LEN, 0.0, -0.007))
    )
    front_block = (
        cq.Workplane("XY")
        .box(
            ARM_FRONT_BLOCK_LEN,
            ARM_FRONT_BLOCK_W,
            ARM_FRONT_BLOCK_H,
            centered=(False, True, True),
        )
        .translate((SLIDE_START - 0.025, 0.0, -0.002))
    )
    left_rail = (
        cq.Workplane("XY")
        .box(GUIDE_LEN, RAIL_W, RAIL_H, centered=(False, True, True))
        .translate((SLIDE_START, RAIL_Y, 0.015))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(GUIDE_LEN, RAIL_W, RAIL_H, centered=(False, True, True))
        .translate((SLIDE_START, -RAIL_Y, 0.015))
    )

    return lug.union(body).union(front_block).union(left_rail).union(right_rail)


def _make_extension_stage() -> cq.Workplane:
    left_runner = (
        cq.Workplane("XY")
        .box(STAGE_LEN, 0.012, 0.012, centered=(False, True, True))
        .translate((0.0, RAIL_Y, 0.0))
    )
    right_runner = (
        cq.Workplane("XY")
        .box(STAGE_LEN, 0.012, 0.012, centered=(False, True, True))
        .translate((0.0, -RAIL_Y, 0.0))
    )
    top_bridge = (
        cq.Workplane("XY")
        .box(STAGE_LEN, 0.048, 0.012, centered=(False, True, True))
        .translate((0.0, 0.0, 0.012))
    )
    top_pad = (
        cq.Workplane("XY")
        .box(STAGE_PAD_LEN, STAGE_PAD_W, STAGE_PAD_H, centered=(False, True, True))
        .translate((0.056, 0.0, 0.023))
    )
    front_stop = (
        cq.Workplane("XY")
        .box(STAGE_FRONT_STOP, STAGE_W, 0.018, centered=(False, True, True))
        .translate((STAGE_LEN - STAGE_FRONT_STOP, 0.0, 0.015))
    )

    return left_runner.union(right_runner).union(top_bridge).union(top_pad).union(front_stop)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_swing_slide_mechanism")

    plate_mat = model.material("plate_steel", color=(0.42, 0.45, 0.48))
    arm_mat = model.material("arm_aluminum", color=(0.68, 0.70, 0.73))
    stage_mat = model.material("stage_dark", color=(0.18, 0.19, 0.21))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_make_side_plate(), "side_plate"),
        material=plate_mat,
        name="plate_body",
    )

    swing_arm = model.part("swing_arm")
    swing_arm.visual(
        mesh_from_cadquery(_make_swing_arm(), "swing_arm"),
        material=arm_mat,
        name="arm_body",
    )

    extension_stage = model.part("extension_stage")
    extension_stage.visual(
        mesh_from_cadquery(_make_extension_stage(), "extension_stage"),
        material=stage_mat,
        name="stage_body",
    )

    model.articulation(
        "plate_to_arm",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=swing_arm,
        origin=Origin(xyz=(EAR_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=0.0,
            upper=1.20,
        ),
    )

    model.articulation(
        "arm_to_stage",
        ArticulationType.PRISMATIC,
        parent=swing_arm,
        child=extension_stage,
        origin=Origin(xyz=(SLIDE_START, 0.0, 0.027)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    swing_arm = object_model.get_part("swing_arm")
    extension_stage = object_model.get_part("extension_stage")
    plate_to_arm = object_model.get_articulation("plate_to_arm")
    arm_to_stage = object_model.get_articulation("arm_to_stage")

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

    ctx.expect_contact(swing_arm, side_plate, name="arm_is_supported_by_side_plate")
    ctx.expect_contact(extension_stage, swing_arm, name="stage_is_carried_by_arm")

    ctx.check(
        "root_joint_is_vertical_revolute",
        plate_to_arm.articulation_type == ArticulationType.REVOLUTE
        and tuple(plate_to_arm.axis) == (0.0, 0.0, 1.0),
        f"type={plate_to_arm.articulation_type}, axis={plate_to_arm.axis}",
    )
    ctx.check(
        "carried_stage_is_forward_prismatic",
        arm_to_stage.articulation_type == ArticulationType.PRISMATIC
        and tuple(arm_to_stage.axis) == (1.0, 0.0, 0.0),
        f"type={arm_to_stage.articulation_type}, axis={arm_to_stage.axis}",
    )

    closed_stage_pos = ctx.part_world_position(extension_stage)
    with ctx.pose(arm_to_stage=SLIDE_TRAVEL):
        extended_stage_pos = ctx.part_world_position(extension_stage)
    ctx.check(
        "stage_extends_forward_by_slide_travel",
        closed_stage_pos is not None
        and extended_stage_pos is not None
        and 0.075 <= extended_stage_pos[0] - closed_stage_pos[0] <= 0.085,
        f"closed={closed_stage_pos}, extended={extended_stage_pos}",
    )

    with ctx.pose(plate_to_arm=0.90):
        swung_stage_pos = ctx.part_world_position(extension_stage)
    ctx.check(
        "positive_arm_rotation_swings_stage_outboard",
        swung_stage_pos is not None and swung_stage_pos[1] > 0.18,
        f"swung_stage_pos={swung_stage_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
