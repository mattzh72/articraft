from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_X = 0.160
BASE_Y = 0.120
BASE_Z = 0.012

FRAME_OUTER_WIDTH = 0.126
FRAME_INNER_WIDTH = 0.090
ARM_THICKNESS = (FRAME_OUTER_WIDTH - FRAME_INNER_WIDTH) / 2.0
ARM_PLATE_X = 0.018
ARM_HEIGHT = 0.116
ARM_CENTER_X = -0.016
ARM_CENTER_Y = FRAME_INNER_WIDTH / 2.0 + ARM_THICKNESS / 2.0
ARM_CENTER_Z = BASE_Z + ARM_HEIGHT / 2.0

REAR_WEB_X = 0.024
REAR_WEB_HEIGHT = 0.076
REAR_WEB_CENTER_X = -0.032
REAR_WEB_CENTER_Z = BASE_Z + REAR_WEB_HEIGHT / 2.0

TOP_BRIDGE_X = 0.022
TOP_BRIDGE_Z = 0.016
TOP_BRIDGE_CENTER_X = -0.028
TOP_BRIDGE_CENTER_Z = 0.126

PIVOT_X = 0.0
PIVOT_Z = 0.102
PIVOT_PAD_RADIUS = 0.014
PIVOT_PAD_LENGTH = 0.005
PIVOT_PAD_CENTER_Y = FRAME_INNER_WIDTH / 2.0 - PIVOT_PAD_LENGTH / 2.0

HEAD_BODY_X = 0.048
HEAD_BODY_Y = 0.060
HEAD_BODY_Z = 0.050
HEAD_BODY_X_OFFSET = 0.020
HEAD_TRUNNION_RADIUS = 0.010
HEAD_TRUNNION_LENGTH = 0.011
HEAD_TRUNNION_CENTER_Y = HEAD_BODY_Y / 2.0 + HEAD_TRUNNION_LENGTH / 2.0 - 0.001

FRONT_PLATE_X = 0.004
FRONT_PLATE_Y = 0.050
FRONT_PLATE_Z = 0.038
FRONT_PLATE_CENTER_X = HEAD_BODY_X_OFFSET + HEAD_BODY_X / 2.0 + FRONT_PLATE_X / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_cradle_pitch_module")

    powder_gray = model.material("powder_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    face_black = model.material("face_black", rgba=(0.12, 0.13, 0.14, 1.0))

    fork_frame = model.part("fork_frame")
    fork_frame.visual(
        Box((BASE_X, BASE_Y, BASE_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z / 2.0)),
        material=powder_gray,
        name="frame_base",
    )
    fork_frame.visual(
        Box((ARM_PLATE_X, ARM_THICKNESS, ARM_HEIGHT)),
        origin=Origin(xyz=(ARM_CENTER_X, ARM_CENTER_Y, ARM_CENTER_Z)),
        material=powder_gray,
        name="left_arm",
    )
    fork_frame.visual(
        Box((ARM_PLATE_X, ARM_THICKNESS, ARM_HEIGHT)),
        origin=Origin(xyz=(ARM_CENTER_X, -ARM_CENTER_Y, ARM_CENTER_Z)),
        material=powder_gray,
        name="right_arm",
    )
    fork_frame.visual(
        Box((REAR_WEB_X, FRAME_OUTER_WIDTH, REAR_WEB_HEIGHT)),
        origin=Origin(xyz=(REAR_WEB_CENTER_X, 0.0, REAR_WEB_CENTER_Z)),
        material=powder_gray,
        name="rear_web",
    )
    fork_frame.visual(
        Box((TOP_BRIDGE_X, FRAME_OUTER_WIDTH, TOP_BRIDGE_Z)),
        origin=Origin(xyz=(TOP_BRIDGE_CENTER_X, 0.0, TOP_BRIDGE_CENTER_Z)),
        material=powder_gray,
        name="top_bridge",
    )
    fork_frame.visual(
        Cylinder(radius=PIVOT_PAD_RADIUS, length=PIVOT_PAD_LENGTH),
        origin=Origin(
            xyz=(PIVOT_X, PIVOT_PAD_CENTER_Y, PIVOT_Z),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=powder_gray,
        name="left_bearing",
    )
    fork_frame.visual(
        Cylinder(radius=PIVOT_PAD_RADIUS, length=PIVOT_PAD_LENGTH),
        origin=Origin(
            xyz=(PIVOT_X, -PIVOT_PAD_CENTER_Y, PIVOT_Z),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=powder_gray,
        name="right_bearing",
    )
    fork_frame.inertial = Inertial.from_geometry(
        Box((BASE_X, FRAME_OUTER_WIDTH, TOP_BRIDGE_CENTER_Z + TOP_BRIDGE_Z / 2.0)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (TOP_BRIDGE_CENTER_Z + TOP_BRIDGE_Z / 2.0) / 2.0)),
    )

    head = model.part("head")
    head.visual(
        Box((HEAD_BODY_X, HEAD_BODY_Y, HEAD_BODY_Z)),
        origin=Origin(xyz=(HEAD_BODY_X_OFFSET, 0.0, 0.0)),
        material=machine_gray,
        name="head_body",
    )
    head.visual(
        Cylinder(radius=HEAD_TRUNNION_RADIUS, length=HEAD_TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, HEAD_TRUNNION_CENTER_Y, 0.0),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=machine_gray,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=HEAD_TRUNNION_RADIUS, length=HEAD_TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, -HEAD_TRUNNION_CENTER_Y, 0.0),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=machine_gray,
        name="right_trunnion",
    )
    head.visual(
        Box((FRONT_PLATE_X, FRONT_PLATE_Y, FRONT_PLATE_Z)),
        origin=Origin(xyz=(FRONT_PLATE_CENTER_X, 0.0, 0.0)),
        material=face_black,
        name="head_front",
    )
    head.inertial = Inertial.from_geometry(
        Box((HEAD_BODY_X, HEAD_BODY_Y, HEAD_BODY_Z)),
        mass=0.85,
        origin=Origin(xyz=(HEAD_BODY_X_OFFSET, 0.0, 0.0)),
    )

    model.articulation(
        "fork_pitch",
        ArticulationType.REVOLUTE,
        parent=fork_frame,
        child=head,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.50, effort=22.0, velocity=1.8),
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

    fork_frame = object_model.get_part("fork_frame")
    head = object_model.get_part("head")
    pitch = object_model.get_articulation("fork_pitch")
    head_body = head.get_visual("head_body")
    head_front = head.get_visual("head_front")
    left_trunnion = head.get_visual("left_trunnion")
    right_trunnion = head.get_visual("right_trunnion")
    left_bearing = fork_frame.get_visual("left_bearing")
    right_bearing = fork_frame.get_visual("right_bearing")
    rear_web = fork_frame.get_visual("rear_web")
    top_bridge = fork_frame.get_visual("top_bridge")

    ctx.check("fork frame exists", fork_frame is not None)
    ctx.check("head exists", head is not None)
    ctx.check(
        "pitch axis is the supported horizontal cross-axis",
        tuple(round(value, 4) for value in pitch.axis) == (0.0, -1.0, 0.0),
        details=f"axis={pitch.axis}",
    )
    ctx.check(
        "pitch limits straddle level",
        pitch.motion_limits is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower < 0.0 < pitch.motion_limits.upper,
        details=f"limits={pitch.motion_limits}",
    )

    fork_aabb = ctx.part_world_aabb(fork_frame)
    ctx.check(
        "fork frame is grounded",
        fork_aabb is not None and abs(fork_aabb[0][2]) <= 1e-4,
        details=f"aabb={fork_aabb}",
    )

    ctx.expect_origin_distance(
        head,
        fork_frame,
        axes="y",
        max_dist=0.001,
        name="head stays centered between the fork arms",
    )
    ctx.expect_overlap(
        head,
        fork_frame,
        axes="yz",
        min_overlap=0.050,
        name="head sits inside the fork cradle silhouette",
    )
    ctx.expect_contact(
        head,
        fork_frame,
        elem_a=left_trunnion,
        elem_b=left_bearing,
        name="left trunnion seats on the left fork bearing",
    )
    ctx.expect_contact(
        head,
        fork_frame,
        elem_a=right_trunnion,
        elem_b=right_bearing,
        name="right trunnion seats on the right fork bearing",
    )

    rest_aabb = ctx.part_element_world_aabb(head, elem=head_front)
    with ctx.pose({pitch: pitch.motion_limits.upper}):
        ctx.expect_gap(
            head,
            fork_frame,
            axis="x",
            min_gap=0.001,
            positive_elem=head_body,
            negative_elem=top_bridge,
            name="head body clears the top bridge at upper pitch",
        )
        raised_aabb = ctx.part_element_world_aabb(head, elem=head_front)
    with ctx.pose({pitch: pitch.motion_limits.lower}):
        ctx.expect_gap(
            head,
            fork_frame,
            axis="x",
            min_gap=0.001,
            positive_elem=head_body,
            negative_elem=rear_web,
            name="head body clears the rear web at lower pitch",
        )
        lowered_aabb = ctx.part_element_world_aabb(head, elem=head_front)

    def z_center(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_z = z_center(rest_aabb)
    raised_z = z_center(raised_aabb)
    lowered_z = z_center(lowered_aabb)

    ctx.check(
        "positive pitch raises the head front face",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.015,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )
    ctx.check(
        "negative pitch lowers the head front face",
        rest_z is not None and lowered_z is not None and lowered_z < rest_z - 0.008,
        details=f"rest_z={rest_z}, lowered_z={lowered_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
