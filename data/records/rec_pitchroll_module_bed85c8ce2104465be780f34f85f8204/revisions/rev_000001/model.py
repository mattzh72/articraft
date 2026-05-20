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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_WIDTH = 0.26
BASE_DEPTH = 0.16
BASE_THICKNESS = 0.02

PEDESTAL_WIDTH = 0.21
PEDESTAL_DEPTH = 0.12
PEDESTAL_HEIGHT = 0.074

PITCH_AXIS_Z = 0.178

SUPPORT_CHEEK_THICKNESS = 0.03
SUPPORT_CHEEK_DEPTH = 0.10
SUPPORT_CHEEK_HEIGHT = 0.18
SUPPORT_CHEEK_CENTER_X = 0.105
SUPPORT_STUB_RADIUS = 0.022
SUPPORT_STUB_LENGTH = 0.01
SUPPORT_STUB_CENTER_X = 0.095

FRAME_SIDE_PLATE_THICKNESS = 0.024
FRAME_SIDE_PLATE_DEPTH = 0.10
FRAME_SIDE_PLATE_HEIGHT = 0.16
FRAME_SIDE_PLATE_CENTER_X = 0.061
FRAME_BRIDGE_WIDTH = 0.15
FRAME_BRIDGE_DEPTH = 0.026
FRAME_BRIDGE_HEIGHT = 0.026
FRAME_BRIDGE_CENTER_Z = 0.067
FRAME_CARRIER_WIDTH = 0.06
FRAME_CARRIER_DEPTH = 0.024
FRAME_CARRIER_HEIGHT = 0.032
FRAME_CARRIER_CENTER_Z = 0.04
FRAME_SHOE_WIDTH = 0.06
FRAME_SHOE_DEPTH = 0.022
FRAME_SHOE_HEIGHT = 0.01
FRAME_UPPER_SHOE_CENTER_Z = 0.024
FRAME_LOWER_SHOE_CENTER_Z = -0.024
FRAME_TRUNNION_RADIUS = 0.013
FRAME_TRUNNION_LENGTH = 0.018
FRAME_TRUNNION_CENTER_X = 0.081

SPINDLE_SHAFT_RADIUS = 0.019
SPINDLE_SHAFT_LENGTH = 0.112
SPINDLE_FRONT_FACE_RADIUS = 0.038
SPINDLE_FRONT_FACE_LENGTH = 0.016
SPINDLE_FRONT_FACE_CENTER_Y = 0.062
SPINDLE_REAR_COLLAR_RADIUS = 0.028
SPINDLE_REAR_COLLAR_LENGTH = 0.018
SPINDLE_REAR_COLLAR_CENTER_Y = -0.064
SPINDLE_LUG_WIDTH = 0.02
SPINDLE_LUG_DEPTH = 0.012
SPINDLE_LUG_HEIGHT = 0.018
SPINDLE_LUG_CENTER_Y = 0.062
SPINDLE_LUG_CENTER_Z = 0.033


def _vec_close(a: tuple[float, float, float], b: tuple[float, float, float], tol: float = 1e-9) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_spindle_pitch_roll_unit")

    model.material("support_finish", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("frame_finish", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("spindle_finish", rgba=(0.57, 0.58, 0.61, 1.0))

    support_base = model.part("support_base")
    support_base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="support_finish",
        name="base_plate",
    )
    support_base.visual(
        Box((PEDESTAL_WIDTH, PEDESTAL_DEPTH, PEDESTAL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="support_finish",
        name="pedestal",
    )
    support_base.visual(
        Box((SUPPORT_CHEEK_THICKNESS, SUPPORT_CHEEK_DEPTH, SUPPORT_CHEEK_HEIGHT)),
        origin=Origin(xyz=(-SUPPORT_CHEEK_CENTER_X, 0.0, PITCH_AXIS_Z)),
        material="support_finish",
        name="left_cheek",
    )
    support_base.visual(
        Box((SUPPORT_CHEEK_THICKNESS, SUPPORT_CHEEK_DEPTH, SUPPORT_CHEEK_HEIGHT)),
        origin=Origin(xyz=(SUPPORT_CHEEK_CENTER_X, 0.0, PITCH_AXIS_Z)),
        material="support_finish",
        name="right_cheek",
    )
    support_base.visual(
        Cylinder(radius=SUPPORT_STUB_RADIUS, length=SUPPORT_STUB_LENGTH),
        origin=Origin(
            xyz=(-SUPPORT_STUB_CENTER_X, 0.0, PITCH_AXIS_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="support_finish",
        name="left_stub",
    )
    support_base.visual(
        Cylinder(radius=SUPPORT_STUB_RADIUS, length=SUPPORT_STUB_LENGTH),
        origin=Origin(
            xyz=(SUPPORT_STUB_CENTER_X, 0.0, PITCH_AXIS_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="support_finish",
        name="right_stub",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        Box((FRAME_SIDE_PLATE_THICKNESS, FRAME_SIDE_PLATE_DEPTH, FRAME_SIDE_PLATE_HEIGHT)),
        origin=Origin(xyz=(-FRAME_SIDE_PLATE_CENTER_X, 0.0, 0.0)),
        material="frame_finish",
        name="left_side_plate",
    )
    pitch_frame.visual(
        Box((FRAME_SIDE_PLATE_THICKNESS, FRAME_SIDE_PLATE_DEPTH, FRAME_SIDE_PLATE_HEIGHT)),
        origin=Origin(xyz=(FRAME_SIDE_PLATE_CENTER_X, 0.0, 0.0)),
        material="frame_finish",
        name="right_side_plate",
    )
    pitch_frame.visual(
        Box((FRAME_BRIDGE_WIDTH, FRAME_BRIDGE_DEPTH, FRAME_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_BRIDGE_CENTER_Z)),
        material="frame_finish",
        name="top_bridge",
    )
    pitch_frame.visual(
        Box((FRAME_BRIDGE_WIDTH, FRAME_BRIDGE_DEPTH, FRAME_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_BRIDGE_CENTER_Z)),
        material="frame_finish",
        name="bottom_bridge",
    )
    pitch_frame.visual(
        Box((FRAME_CARRIER_WIDTH, FRAME_CARRIER_DEPTH, FRAME_CARRIER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_CARRIER_CENTER_Z)),
        material="frame_finish",
        name="upper_carrier",
    )
    pitch_frame.visual(
        Box((FRAME_CARRIER_WIDTH, FRAME_CARRIER_DEPTH, FRAME_CARRIER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_CARRIER_CENTER_Z)),
        material="frame_finish",
        name="lower_carrier",
    )
    pitch_frame.visual(
        Box((FRAME_SHOE_WIDTH, FRAME_SHOE_DEPTH, FRAME_SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_UPPER_SHOE_CENTER_Z)),
        material="frame_finish",
        name="upper_shoe",
    )
    pitch_frame.visual(
        Box((FRAME_SHOE_WIDTH, FRAME_SHOE_DEPTH, FRAME_SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_LOWER_SHOE_CENTER_Z)),
        material="frame_finish",
        name="lower_shoe",
    )
    pitch_frame.visual(
        Cylinder(radius=FRAME_TRUNNION_RADIUS, length=FRAME_TRUNNION_LENGTH),
        origin=Origin(
            xyz=(-FRAME_TRUNNION_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="frame_finish",
        name="left_trunnion",
    )
    pitch_frame.visual(
        Cylinder(radius=FRAME_TRUNNION_RADIUS, length=FRAME_TRUNNION_LENGTH),
        origin=Origin(
            xyz=(FRAME_TRUNNION_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="frame_finish",
        name="right_trunnion",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=SPINDLE_SHAFT_RADIUS, length=SPINDLE_SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="spindle_finish",
        name="shaft",
    )
    roll_spindle.visual(
        Cylinder(radius=SPINDLE_FRONT_FACE_RADIUS, length=SPINDLE_FRONT_FACE_LENGTH),
        origin=Origin(
            xyz=(0.0, SPINDLE_FRONT_FACE_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="spindle_finish",
        name="front_face",
    )
    roll_spindle.visual(
        Cylinder(radius=SPINDLE_REAR_COLLAR_RADIUS, length=SPINDLE_REAR_COLLAR_LENGTH),
        origin=Origin(
            xyz=(0.0, SPINDLE_REAR_COLLAR_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="spindle_finish",
        name="rear_collar",
    )
    roll_spindle.visual(
        Box((SPINDLE_LUG_WIDTH, SPINDLE_LUG_DEPTH, SPINDLE_LUG_HEIGHT)),
        origin=Origin(xyz=(0.0, SPINDLE_LUG_CENTER_Y, SPINDLE_LUG_CENTER_Z)),
        material="spindle_finish",
        name="front_lug",
    )

    model.articulation(
        "support_to_pitch",
        ArticulationType.REVOLUTE,
        parent=support_base,
        child=pitch_frame,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.9,
            upper=0.9,
        ),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
            lower=-pi,
            upper=pi,
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

    support_base = object_model.get_part("support_base")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")
    pitch_joint = object_model.get_articulation("support_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

    ctx.check("support base exists", support_base is not None)
    ctx.check("pitch frame exists", pitch_frame is not None)
    ctx.check("roll spindle exists", roll_spindle is not None)

    ctx.check(
        "pitch joint uses the trunnion axis",
        _vec_close(tuple(pitch_joint.axis), (1.0, 0.0, 0.0))
        and _vec_close(tuple(pitch_joint.origin.xyz), (0.0, 0.0, PITCH_AXIS_Z)),
        details=f"axis={pitch_joint.axis}, origin={pitch_joint.origin.xyz}",
    )
    ctx.check(
        "roll joint uses the spindle axis",
        _vec_close(tuple(roll_joint.axis), (0.0, 1.0, 0.0))
        and _vec_close(tuple(roll_joint.origin.xyz), (0.0, 0.0, 0.0)),
        details=f"axis={roll_joint.axis}, origin={roll_joint.origin.xyz}",
    )

    ctx.expect_origin_distance(
        pitch_frame,
        support_base,
        axes="xy",
        max_dist=1e-6,
        name="pitch frame is centered between the side supports",
    )
    ctx.expect_origin_distance(
        roll_spindle,
        pitch_frame,
        axes="xz",
        max_dist=1e-6,
        name="roll spindle is centered in the pitch frame",
    )
    ctx.expect_within(
        roll_spindle,
        pitch_frame,
        axes="xz",
        margin=0.002,
        name="roll spindle stays within the frame opening envelope",
    )
    ctx.expect_overlap(
        roll_spindle,
        pitch_frame,
        axes="y",
        min_overlap=0.10,
        name="roll spindle retains axial engagement through the pitch frame",
    )

    frame_aabb = ctx.part_world_aabb(pitch_frame)
    spindle_aabb = ctx.part_world_aabb(roll_spindle)
    ctx.check(
        "spindle nose projects forward of the pitch frame",
        frame_aabb is not None
        and spindle_aabb is not None
        and spindle_aabb[1][1] > frame_aabb[1][1] + 0.01,
        details=f"frame_aabb={frame_aabb}, spindle_aabb={spindle_aabb}",
    )

    with ctx.pose({pitch_joint: 0.65}):
        pitched_spindle_aabb = ctx.part_world_aabb(roll_spindle)
    ctx.check(
        "positive pitch lifts the spindle nose upward",
        spindle_aabb is not None
        and pitched_spindle_aabb is not None
        and pitched_spindle_aabb[1][2] > spindle_aabb[1][2] + 0.02,
        details=f"rest={spindle_aabb}, pitched={pitched_spindle_aabb}",
    )

    with ctx.pose({roll_joint: 1.2}):
        rolled_spindle_aabb = ctx.part_world_aabb(roll_spindle)
    ctx.check(
        "roll rotates the spindle front lug around the spindle axis",
        spindle_aabb is not None
        and rolled_spindle_aabb is not None
        and rolled_spindle_aabb[1][0] > spindle_aabb[1][0] + 0.01,
        details=f"rest={spindle_aabb}, rolled={rolled_spindle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
