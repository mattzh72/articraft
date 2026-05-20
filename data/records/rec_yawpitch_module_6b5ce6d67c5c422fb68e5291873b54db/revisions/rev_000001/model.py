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

from math import pi


BASE_FOOT_RADIUS = 0.115
BASE_FOOT_HEIGHT = 0.018
BASE_BODY_RADIUS = 0.094
BASE_BODY_HEIGHT = 0.016
BASE_BEARING_RADIUS = 0.078
BASE_BEARING_HEIGHT = 0.004
BASE_TOTAL_HEIGHT = BASE_FOOT_HEIGHT + BASE_BODY_HEIGHT + BASE_BEARING_HEIGHT

TURNTABLE_RADIUS = 0.082
TURNTABLE_THICK = 0.012
CHEEK_LENGTH = 0.094
CHEEK_THICK = 0.012
CHEEK_HEIGHT = 0.110
CHEEK_CENTER_Y = 0.040
CHEEK_X_CENTER = 0.008
REAR_BRIDGE_LENGTH = 0.018
REAR_BRIDGE_WIDTH = 0.068
REAR_BRIDGE_HEIGHT = 0.028
REAR_BRIDGE_X = -0.034
REAR_BRIDGE_Z = 0.074

PIVOT_X = 0.010
PIVOT_Z = 0.070
TRUNNION_RADIUS = 0.009
TRUNNION_SPAN = 0.068

HEAD_BODY_LENGTH = 0.060
HEAD_BODY_WIDTH = 0.044
HEAD_BODY_HEIGHT = 0.050
HEAD_BODY_X = 0.024
HEAD_NOSE_RADIUS = 0.022
HEAD_NOSE_X = 0.040
HEAD_FACE_LENGTH = 0.018
HEAD_FACE_WIDTH = 0.036
HEAD_FACE_HEIGHT = 0.030
HEAD_FACE_X = 0.052

YAW_LOWER = -2.9
YAW_UPPER = 2.9
PITCH_LOWER = -0.45
PITCH_UPPER = 0.95


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_base_pitch_cradle")

    model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("anodized_gray", rgba=(0.45, 0.48, 0.52, 1.0))
    model.material("machined_silver", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_HEIGHT / 2.0)),
        name="base_shell",
        material="powder_black",
    )
    base.visual(
        Cylinder(radius=BASE_BODY_RADIUS, length=BASE_BODY_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_FOOT_HEIGHT + BASE_BODY_HEIGHT / 2.0),
        ),
        name="base_body",
        material="powder_black",
    )
    base.visual(
        Cylinder(radius=BASE_BEARING_RADIUS, length=BASE_BEARING_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_FOOT_HEIGHT + BASE_BODY_HEIGHT + BASE_BEARING_HEIGHT / 2.0),
        ),
        name="base_bearing_cap",
        material="machined_silver",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_TOTAL_HEIGHT),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT / 2.0)),
    )

    yoke = model.part("yaw_yoke")
    yoke.visual(
        Cylinder(radius=TURNTABLE_RADIUS, length=TURNTABLE_THICK),
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_THICK / 2.0)),
        name="turntable",
        material="anodized_gray",
    )
    yoke.visual(
        Box((CHEEK_LENGTH, CHEEK_THICK, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                CHEEK_X_CENTER,
                CHEEK_CENTER_Y,
                TURNTABLE_THICK + CHEEK_HEIGHT / 2.0,
            ),
        ),
        name="left_cheek",
        material="anodized_gray",
    )
    yoke.visual(
        Box((CHEEK_LENGTH, CHEEK_THICK, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                CHEEK_X_CENTER,
                -CHEEK_CENTER_Y,
                TURNTABLE_THICK + CHEEK_HEIGHT / 2.0,
            ),
        ),
        name="right_cheek",
        material="anodized_gray",
    )
    yoke.visual(
        Box((REAR_BRIDGE_LENGTH, REAR_BRIDGE_WIDTH, REAR_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(REAR_BRIDGE_X, 0.0, REAR_BRIDGE_Z)),
        name="rear_bridge",
        material="anodized_gray",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((CHEEK_LENGTH, 2.0 * CHEEK_CENTER_Y + CHEEK_THICK, CHEEK_HEIGHT)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
    )

    head = model.part("head")
    head.visual(
        Box((HEAD_BODY_LENGTH, HEAD_BODY_WIDTH, HEAD_BODY_HEIGHT)),
        origin=Origin(xyz=(HEAD_BODY_X, 0.0, 0.0)),
        name="head_body",
        material="machined_silver",
    )
    head.visual(
        Cylinder(radius=HEAD_NOSE_RADIUS, length=HEAD_BODY_WIDTH),
        origin=Origin(
            xyz=(HEAD_NOSE_X, 0.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        name="head_nose",
        material="machined_silver",
    )
    head.visual(
        Box((HEAD_FACE_LENGTH, HEAD_FACE_WIDTH, HEAD_FACE_HEIGHT)),
        origin=Origin(xyz=(HEAD_FACE_X, 0.0, 0.0)),
        name="sensor_face",
        material="powder_black",
    )
    head.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_SPAN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        name="trunnion_shaft",
        material="machined_silver",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.078, TRUNNION_SPAN, HEAD_BODY_HEIGHT)),
        mass=0.9,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=YAW_LOWER,
            upper=YAW_UPPER,
            effort=18.0,
            velocity=2.2,
        ),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
            effort=10.0,
            velocity=1.8,
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

    base = object_model.get_part("base")
    yoke = object_model.get_part("yaw_yoke")
    head = object_model.get_part("head")
    yaw = object_model.get_articulation("base_yaw")
    pitch = object_model.get_articulation("head_pitch")

    ctx.expect_contact(
        yoke,
        base,
        name="yaw turntable seats directly on the grounded base",
    )
    ctx.expect_within(
        head,
        yoke,
        axes="y",
        margin=0.0,
        name="head stays laterally between the fork cheeks",
    )
    ctx.expect_contact(
        head,
        yoke,
        name="head is mounted to the fork by the trunnion bearings",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({yaw: 0.8}):
        yawed_head_pos = ctx.part_world_position(head)
    ctx.check(
        "positive yaw sweeps the carried head toward +Y",
        rest_head_pos is not None
        and yawed_head_pos is not None
        and yawed_head_pos[1] > rest_head_pos[1] + 0.005,
        details=f"rest={rest_head_pos}, yawed={yawed_head_pos}",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({pitch: PITCH_UPPER}):
        pitched_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "positive pitch lifts the head envelope upward",
        rest_head_aabb is not None
        and pitched_head_aabb is not None
        and pitched_head_aabb[1][2] > rest_head_aabb[1][2] + 0.01,
        details=f"rest={rest_head_aabb}, pitched={pitched_head_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
