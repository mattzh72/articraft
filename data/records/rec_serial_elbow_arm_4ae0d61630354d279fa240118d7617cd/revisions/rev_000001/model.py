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


BASE_RADIUS = 0.11
BASE_THICKNESS = 0.024
COLUMN_RADIUS = 0.036
COLUMN_HEIGHT = 0.135
COLUMN_X = -0.020

SHOULDER_Z = 0.178
SHOULDER_OUTER_WIDTH = 0.082
SHOULDER_GAP = 0.058
SHOULDER_HEIGHT = 0.066
CHEEK_THICKNESS = 0.014
SHOULDER_CHEEK_Y = 0.034

LINK1_LENGTH = 0.285
LINK1_BARREL_RADIUS = 0.017
LINK1_BARREL_WIDTH = 0.054
LINK1_BEAM_WIDTH = 0.030
LINK1_BEAM_HEIGHT = 0.040
ELBOW_OUTER_WIDTH = 0.080
ELBOW_GAP = 0.056
ELBOW_YOKE_HEIGHT = 0.060
ELBOW_CHEEK_Y = 0.033

LINK2_BARREL_RADIUS = 0.0155
LINK2_BARREL_WIDTH = 0.052
LINK2_BEAM_WIDTH = 0.026
LINK2_BEAM_HEIGHT = 0.034
LINK2_BODY_LENGTH = 0.212
TOOL_FACE_THICKNESS = 0.008
TOOL_FACE_WIDTH = 0.052
TOOL_FACE_HEIGHT = 0.040
HALF_PI = 1.5707963267948966


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_two_link_elbow_arm")

    model.material("base_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("arm_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("tool_dark", rgba=(0.15, 0.16, 0.18, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="base_gray",
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(COLUMN_X, 0.0, BASE_THICKNESS + COLUMN_HEIGHT / 2.0)),
        material="base_gray",
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.018, SHOULDER_OUTER_WIDTH, SHOULDER_HEIGHT)),
        origin=Origin(xyz=(-0.028, 0.0, SHOULDER_Z)),
        material="base_gray",
        name="shoulder_bridge",
    )
    pedestal.visual(
        Box((0.040, CHEEK_THICKNESS, SHOULDER_HEIGHT)),
        origin=Origin(xyz=(0.0, SHOULDER_CHEEK_Y, SHOULDER_Z)),
        material="base_gray",
        name="shoulder_left_cheek",
    )
    pedestal.visual(
        Box((0.040, CHEEK_THICKNESS, SHOULDER_HEIGHT)),
        origin=Origin(xyz=(0.0, -SHOULDER_CHEEK_Y, SHOULDER_Z)),
        material="base_gray",
        name="shoulder_right_cheek",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS + COLUMN_HEIGHT),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + COLUMN_HEIGHT) / 2.0)),
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        Cylinder(radius=LINK1_BARREL_RADIUS, length=LINK1_BARREL_WIDTH),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material="arm_silver",
        name="shoulder_barrel",
    )
    proximal_link.visual(
        Box((0.215, LINK1_BEAM_WIDTH, LINK1_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.124, 0.0, 0.0)),
        material="arm_silver",
        name="proximal_beam",
    )
    proximal_link.visual(
        Box((0.140, 0.018, 0.010)),
        origin=Origin(xyz=(0.140, 0.0, 0.019)),
        material="arm_silver",
        name="top_rib",
    )
    proximal_link.visual(
        Box((0.040, LINK1_BEAM_WIDTH, 0.044)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material="arm_silver",
        name="elbow_stem",
    )
    proximal_link.visual(
        Box((0.017, ELBOW_OUTER_WIDTH, ELBOW_YOKE_HEIGHT)),
        origin=Origin(xyz=(0.258, 0.0, 0.0)),
        material="arm_silver",
        name="elbow_bridge",
    )
    proximal_link.visual(
        Box((0.037, CHEEK_THICKNESS, ELBOW_YOKE_HEIGHT)),
        origin=Origin(xyz=(LINK1_LENGTH, ELBOW_CHEEK_Y, 0.0)),
        material="arm_silver",
        name="elbow_left_cheek",
    )
    proximal_link.visual(
        Box((0.037, CHEEK_THICKNESS, ELBOW_YOKE_HEIGHT)),
        origin=Origin(xyz=(LINK1_LENGTH, -ELBOW_CHEEK_Y, 0.0)),
        material="arm_silver",
        name="elbow_right_cheek",
    )
    proximal_link.inertial = Inertial.from_geometry(
        Box((LINK1_LENGTH + 0.025, ELBOW_OUTER_WIDTH, ELBOW_YOKE_HEIGHT)),
        mass=2.3,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        Cylinder(radius=LINK2_BARREL_RADIUS, length=LINK2_BARREL_WIDTH),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material="arm_silver",
        name="elbow_barrel",
    )
    distal_link.visual(
        Box((0.160, LINK2_BEAM_WIDTH, LINK2_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0955, 0.0, 0.0)),
        material="arm_silver",
        name="distal_beam",
    )
    distal_link.visual(
        Box((0.038, 0.020, 0.024)),
        origin=Origin(xyz=(0.194, 0.0, 0.0)),
        material="arm_silver",
        name="tool_neck",
    )
    distal_link.visual(
        Box((TOOL_FACE_THICKNESS, TOOL_FACE_WIDTH, TOOL_FACE_HEIGHT)),
        origin=Origin(xyz=(0.2165, 0.0, 0.0)),
        material="tool_dark",
        name="tool_face",
    )
    distal_link.inertial = Inertial.from_geometry(
        Box((LINK2_BODY_LENGTH + TOOL_FACE_THICKNESS, TOOL_FACE_WIDTH, 0.050)),
        mass=1.5,
        origin=Origin(xyz=(0.5 * (LINK2_BODY_LENGTH + TOOL_FACE_THICKNESS), 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=proximal_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.75,
            upper=1.20,
            effort=90.0,
            velocity=1.0,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_link,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.35,
            upper=1.35,
            effort=60.0,
            velocity=1.3,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    proximal_link = object_model.get_part("proximal_link")
    distal_link = object_model.get_part("distal_link")
    distal_link.get_visual("tool_face")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")

    ctx.expect_origin_gap(
        proximal_link,
        pedestal,
        axis="z",
        min_gap=SHOULDER_Z - 0.001,
        max_gap=SHOULDER_Z + 0.001,
        name="shoulder joint sits at top of pedestal",
    )
    ctx.expect_origin_gap(
        distal_link,
        proximal_link,
        axis="x",
        min_gap=LINK1_LENGTH - 0.001,
        max_gap=LINK1_LENGTH + 0.001,
        name="elbow joint sits at the distal end of the proximal link",
    )
    ctx.expect_contact(
        pedestal,
        proximal_link,
        name="proximal link is physically supported by the pedestal yoke",
    )
    ctx.expect_contact(
        proximal_link,
        distal_link,
        name="distal link is physically supported by the elbow yoke",
    )
    ctx.check(
        "joint axes are parallel pitch axes",
        shoulder_pitch.axis == (0.0, -1.0, 0.0) and elbow_pitch.axis == shoulder_pitch.axis,
        details=f"shoulder={shoulder_pitch.axis}, elbow={elbow_pitch.axis}",
    )

    distal_rest = ctx.part_world_position(distal_link)
    with ctx.pose({shoulder_pitch: 0.85}):
        distal_raised = ctx.part_world_position(distal_link)
    ctx.check(
        "positive shoulder motion raises the chain",
        distal_rest is not None
        and distal_raised is not None
        and distal_raised[2] > distal_rest[2] + 0.12,
        details=f"rest={distal_rest}, raised={distal_raised}",
    )

    with ctx.pose({shoulder_pitch: 0.35}):
        tool_rest = ctx.part_element_world_aabb(distal_link, elem="tool_face")
    with ctx.pose({shoulder_pitch: 0.35, elbow_pitch: 0.95}):
        tool_folded = ctx.part_element_world_aabb(distal_link, elem="tool_face")
    ctx.check(
        "positive elbow motion lifts the tool face",
        tool_rest is not None
        and tool_folded is not None
        and tool_folded[0][2] > tool_rest[0][2] + 0.06,
        details=f"rest={tool_rest}, folded={tool_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
