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


BASE_RADIUS = 0.12
BASE_HEIGHT = 0.018
COLUMN_RADIUS = 0.055
COLUMN_HEIGHT = 0.17
TOP_FLANGE_RADIUS = 0.075
TOP_FLANGE_HEIGHT = 0.018
YAW_STAGE_GAP = 0.0

YAW_BODY_RADIUS = 0.078
YAW_BODY_HEIGHT = 0.052
YAW_YOKE_OUTER_WIDTH = 0.152
YAW_YOKE_DEPTH = 0.112
YAW_YOKE_HEIGHT = 0.128
YAW_YOKE_CENTER_Y = 0.086
YAW_YOKE_CENTER_Z = 0.112
YAW_WINDOW_WIDTH = 0.124
YAW_WINDOW_DEPTH = 0.090
YAW_WINDOW_HEIGHT = 0.084
YAW_WINDOW_CENTER_Y = 0.099
YAW_WINDOW_CENTER_Z = 0.106
PITCH_AXIS_Y = 0.082
PITCH_AXIS_Z = 0.108
YAW_BORE_RADIUS = 0.0125

PITCH_BODY_WIDTH = 0.096
PITCH_BODY_DEPTH = 0.072
PITCH_BODY_HEIGHT = 0.078
PITCH_BODY_CENTER_Y = 0.028
PITCH_FRONT_PLATE_WIDTH = 0.102
PITCH_FRONT_PLATE_DEPTH = 0.008
PITCH_FRONT_PLATE_HEIGHT = 0.080
PITCH_FRONT_PLATE_CENTER_Y = 0.068
PITCH_TRUNNION_RADIUS = 0.010
PITCH_TRUNNION_LENGTH = 0.156
PITCH_COLLAR_RADIUS = 0.017
PITCH_COLLAR_THICKNESS = 0.004
YAW_OUTER_FACE_X = YAW_YOKE_OUTER_WIDTH / 2.0

PEDESTAL_TOP_Z = BASE_HEIGHT + COLUMN_HEIGHT + TOP_FLANGE_HEIGHT
YAW_JOINT_Z = PEDESTAL_TOP_Z + YAW_STAGE_GAP


def _x_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _make_pedestal_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_HEIGHT)
    column = (
        cq.Workplane("XY")
        .circle(COLUMN_RADIUS)
        .extrude(COLUMN_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    top_flange = (
        cq.Workplane("XY")
        .circle(TOP_FLANGE_RADIUS)
        .extrude(TOP_FLANGE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT + COLUMN_HEIGHT))
    )
    return base.union(column).union(top_flange)


def _make_yaw_body_shape() -> cq.Workplane:
    drum = cq.Workplane("XY").circle(YAW_BODY_RADIUS).extrude(YAW_BODY_HEIGHT)
    cap = (
        cq.Workplane("XY")
        .rect(0.092, 0.078)
        .extrude(0.020)
        .translate((0.0, 0.0, YAW_BODY_HEIGHT - 0.004))
    )
    return drum.union(cap).edges("|Z").fillet(0.004)


def _make_yaw_yoke_shape() -> cq.Workplane:
    yoke_block = (
        cq.Workplane("XY")
        .box(
            YAW_YOKE_OUTER_WIDTH,
            YAW_YOKE_DEPTH,
            YAW_YOKE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, YAW_YOKE_CENTER_Y, 0.048))
    )
    window = (
        cq.Workplane("XY")
        .box(
            YAW_WINDOW_WIDTH,
            YAW_WINDOW_DEPTH,
            YAW_WINDOW_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, YAW_WINDOW_CENTER_Y, 0.064))
    )
    bore = _x_cylinder(YAW_BORE_RADIUS, YAW_YOKE_OUTER_WIDTH + 0.03).translate(
        (0.0, PITCH_AXIS_Y, PITCH_AXIS_Z)
    )
    gusset = (
        cq.Workplane("XZ")
        .moveTo(-0.030, YAW_BODY_HEIGHT)
        .lineTo(-0.058, YAW_BODY_HEIGHT)
        .lineTo(-0.070, 0.088)
        .lineTo(-0.070, 0.144)
        .lineTo(-0.030, 0.144)
        .close()
        .extrude(0.018)
        .translate((0.0, 0.016, 0.0))
    )
    mirrored_gusset = gusset.mirror("YZ")
    return (
        yoke_block.cut(window)
        .cut(bore)
        .union(gusset)
        .union(mirrored_gusset)
        .edges("|X")
        .fillet(0.003)
    )


def _make_pitch_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            PITCH_BODY_WIDTH,
            PITCH_BODY_DEPTH,
            PITCH_BODY_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, PITCH_BODY_CENTER_Y, 0.0))
    )
    return body.edges("|Z").fillet(0.005)


def _make_pitch_trunnion_shape() -> cq.Workplane:
    return _x_cylinder(PITCH_TRUNNION_RADIUS, PITCH_TRUNNION_LENGTH)


def _make_pitch_face_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            PITCH_FRONT_PLATE_WIDTH,
            PITCH_FRONT_PLATE_DEPTH,
            PITCH_FRONT_PLATE_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, PITCH_FRONT_PLATE_CENTER_Y, 0.0))
        .edges("|Z")
        .fillet(0.0025)
    )


def _make_pitch_side_collar_shape() -> cq.Workplane:
    collar = _x_cylinder(PITCH_COLLAR_RADIUS, PITCH_COLLAR_THICKNESS)
    left = collar.translate((-(YAW_OUTER_FACE_X + PITCH_COLLAR_THICKNESS / 2.0), 0.0, 0.0))
    right = collar.translate(((YAW_OUTER_FACE_X + PITCH_COLLAR_THICKNESS / 2.0), 0.0, 0.0))
    return left.union(right)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_inspection_head")

    model.material("pedestal_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    model.material("machine_gray", rgba=(0.46, 0.48, 0.50, 1.0))
    model.material("head_black", rgba=(0.13, 0.14, 0.15, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_make_pedestal_shape(), "pedestal_shell"),
        material="pedestal_gray",
        name="pedestal_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_body_shape(), "yaw_stage_body"),
        material="machine_gray",
        name="yaw_stage_body",
    )
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_yoke_shape(), "yaw_stage_yoke"),
        material="machine_gray",
        name="yaw_yoke",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_make_pitch_body_shape(), "pitch_frame_body"),
        material="head_black",
        name="pitch_body",
    )
    pitch_frame.visual(
        mesh_from_cadquery(_make_pitch_trunnion_shape(), "pitch_frame_trunnion"),
        material="machine_gray",
        name="pitch_trunnion_shaft",
    )
    pitch_frame.visual(
        mesh_from_cadquery(_make_pitch_side_collar_shape(), "pitch_frame_side_collars"),
        material="machine_gray",
        name="pitch_side_collars",
    )
    pitch_frame.visual(
        mesh_from_cadquery(_make_pitch_face_shape(), "pitch_output_face"),
        material="head_black",
        name="output_face",
    )

    model.articulation(
        "pedestal_to_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=1.6,
            lower=-2.7,
            upper=2.7,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(0.0, PITCH_AXIS_Y, PITCH_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=-0.95,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    yaw_joint = object_model.get_articulation("pedestal_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.check(
        "yaw joint is vertical revolute",
        yaw_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw_joint.articulation_type}, axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch joint is horizontal revolute",
        pitch_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(pitch_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={pitch_joint.articulation_type}, axis={pitch_joint.axis}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0}):
        ctx.expect_gap(
            yaw_stage,
            pedestal,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            name="yaw stage sits tightly on pedestal",
        )
        ctx.expect_overlap(
            yaw_stage,
            pedestal,
            axes="xy",
            min_overlap=0.12,
            name="yaw stage stays centered over pedestal",
        )
        ctx.expect_within(
            pitch_frame,
            yaw_stage,
            axes="x",
            inner_elem="pitch_body",
            outer_elem="yaw_yoke",
            margin=0.0,
            name="pitch frame remains laterally captured by yoke",
        )

    rest_pitch_origin = ctx.part_world_position(pitch_frame)
    with ctx.pose({yaw_joint: yaw_joint.motion_limits.upper, pitch_joint: 0.0}):
        yawed_pitch_origin = ctx.part_world_position(pitch_frame)

    ctx.check(
        "yaw sweeps the inspection head around the pedestal axis",
        rest_pitch_origin is not None
        and yawed_pitch_origin is not None
        and abs(yawed_pitch_origin[0] - rest_pitch_origin[0]) > 0.02
        and abs(yawed_pitch_origin[1] - rest_pitch_origin[1]) > 0.10,
        details=f"rest={rest_pitch_origin}, yawed={yawed_pitch_origin}",
    )

    rest_output_aabb = ctx.part_element_world_aabb(pitch_frame, elem="output_face")
    with ctx.pose({yaw_joint: 0.0, pitch_joint: pitch_joint.motion_limits.upper}):
        pitched_output_aabb = ctx.part_element_world_aabb(pitch_frame, elem="output_face")

    ctx.check(
        "positive pitch lifts the output face upward",
        rest_output_aabb is not None
        and pitched_output_aabb is not None
        and pitched_output_aabb[1][2] > rest_output_aabb[1][2] + 0.03,
        details=f"rest={rest_output_aabb}, pitched={pitched_output_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
