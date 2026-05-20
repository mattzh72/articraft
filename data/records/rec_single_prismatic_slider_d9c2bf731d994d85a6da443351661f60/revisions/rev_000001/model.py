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


BACKPLATE_WIDTH = 0.16
BACKPLATE_HEIGHT = 0.48
BACKPLATE_THICKNESS = 0.008
BACKPLATE_CORNER_RADIUS = 0.010
MOUNT_HOLE_DIAMETER = 0.006
MOUNT_HOLE_X = 0.052
MOUNT_HOLE_Z = 0.180

GUIDE_WIDTH = 0.050
GUIDE_HEIGHT = 0.340
GUIDE_DEPTH = 0.018
GUIDE_EDGE_RADIUS = 0.002
GUIDE_SINK_INTO_PLATE = 0.0008
GUIDE_CENTER_Y = BACKPLATE_THICKNESS / 2.0 + GUIDE_DEPTH / 2.0 - GUIDE_SINK_INTO_PLATE

FACEPLATE_SIZE = 0.110
FACEPLATE_THICKNESS = 0.010
FACEPLATE_CORNER_RADIUS = 0.007

SADDLE_SPACER_WIDTH = 0.070
SADDLE_SPACER_HEIGHT = 0.090
SADDLE_SPACER_DEPTH = 0.016

GUIDE_CLEARANCE = 0.0020
CHEEK_WIDTH = 0.016
CHEEK_HEIGHT = 0.094
CHEEK_DEPTH = 0.029
CHEEK_EDGE_RADIUS = 0.002
CHEEK_CENTER_X = GUIDE_WIDTH / 2.0 + GUIDE_CLEARANCE + CHEEK_WIDTH / 2.0
CHEEK_CENTER_Y = 0.0125

RUNNER_PAD_WIDTH = 0.014
RUNNER_PAD_HEIGHT = 0.024
RUNNER_PAD_DEPTH = 0.006
RUNNER_PAD_CENTER_Y = GUIDE_DEPTH / 2.0 + RUNNER_PAD_DEPTH / 2.0
RUNNER_PAD_CENTER_X = 0.013
RUNNER_PAD_CENTER_Z = 0.026

SADDLE_FRONT_Y = 0.0274
SADDLE_CENTER_Y = SADDLE_FRONT_Y - SADDLE_SPACER_DEPTH / 2.0
FACEPLATE_BACK_Y = 0.0262
FACEPLATE_CENTER_Y = FACEPLATE_BACK_Y + FACEPLATE_THICKNESS / 2.0

SLIDE_TRAVEL = 0.110


def _make_backplate_shape() -> cq.Workplane:
    hole_points = [
        (-MOUNT_HOLE_X, -MOUNT_HOLE_Z),
        (MOUNT_HOLE_X, -MOUNT_HOLE_Z),
        (-MOUNT_HOLE_X, MOUNT_HOLE_Z),
        (MOUNT_HOLE_X, MOUNT_HOLE_Z),
    ]
    return (
        cq.Workplane("XY")
        .box(BACKPLATE_WIDTH, BACKPLATE_THICKNESS, BACKPLATE_HEIGHT)
        .edges("|Y")
        .fillet(BACKPLATE_CORNER_RADIUS)
        .faces(">Y")
        .workplane()
        .pushPoints(hole_points)
        .circle(MOUNT_HOLE_DIAMETER / 2.0)
        .cutThruAll()
    )


def _make_guide_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)
        .edges("|Z")
        .fillet(GUIDE_EDGE_RADIUS)
        .translate((0.0, GUIDE_CENTER_Y, 0.0))
    )


def _make_faceplate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FACEPLATE_SIZE, FACEPLATE_THICKNESS, FACEPLATE_SIZE)
        .edges("|Y")
        .fillet(FACEPLATE_CORNER_RADIUS)
        .translate((0.0, FACEPLATE_CENTER_Y, 0.0))
    )


def _make_bridge_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(SADDLE_SPACER_WIDTH, SADDLE_SPACER_DEPTH, SADDLE_SPACER_HEIGHT)
        .edges("|Y")
        .fillet(0.003)
        .translate((0.0, SADDLE_CENTER_Y, 0.0))
    )


def _make_side_frames_shape() -> cq.Workplane:
    left_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_WIDTH, CHEEK_DEPTH, CHEEK_HEIGHT)
        .edges("|Z")
        .fillet(CHEEK_EDGE_RADIUS)
        .translate((CHEEK_CENTER_X, CHEEK_CENTER_Y, 0.0))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_WIDTH, CHEEK_DEPTH, CHEEK_HEIGHT)
        .edges("|Z")
        .fillet(CHEEK_EDGE_RADIUS)
        .translate((-CHEEK_CENTER_X, CHEEK_CENTER_Y, 0.0))
    )
    return (
        cq.Workplane("XY")
        .add(left_cheek.val())
        .add(right_cheek.val())
        .combine()
    )


def _make_runner_shape(*, x_sign: float, z_sign: float, name: str) -> cq.Workplane:
    upper_pad = (
        cq.Workplane("XY")
        .box(RUNNER_PAD_WIDTH, RUNNER_PAD_DEPTH, RUNNER_PAD_HEIGHT)
        .translate(
            (
                x_sign * RUNNER_PAD_CENTER_X,
                RUNNER_PAD_CENTER_Y,
                z_sign * RUNNER_PAD_CENTER_Z,
            )
        )
    )
    return upper_pad


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_shuttle_slide")

    model.material("backplate_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("guide_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("carriage_gray", rgba=(0.76, 0.78, 0.81, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_make_backplate_shape(), "backplate"),
        material="backplate_gray",
        name="backplate",
    )
    backplate.visual(
        mesh_from_cadquery(_make_guide_shape(), "guideway"),
        material="guide_steel",
        name="guideway",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_bridge_shape(), "carriage_bridge"),
        material="carriage_gray",
        name="bridge",
    )
    carriage.visual(
        mesh_from_cadquery(_make_side_frames_shape(), "carriage_side_frames"),
        material="carriage_gray",
        name="side_frames",
    )
    carriage.visual(
        mesh_from_cadquery(_make_runner_shape(x_sign=1.0, z_sign=1.0, name="upper_runner"), "upper_runner"),
        material="guide_steel",
        name="upper_runner",
    )
    carriage.visual(
        mesh_from_cadquery(_make_runner_shape(x_sign=-1.0, z_sign=-1.0, name="lower_runner"), "lower_runner"),
        material="guide_steel",
        name="lower_runner",
    )
    carriage.visual(
        mesh_from_cadquery(_make_faceplate_shape(), "carriage_faceplate"),
        material="carriage_gray",
        name="faceplate",
    )

    model.articulation(
        "backplate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=backplate,
        child=carriage,
        origin=Origin(xyz=(0.0, GUIDE_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.30,
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
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
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("backplate_to_carriage")

    plate_visual = backplate.get_visual("backplate")
    guide_visual = backplate.get_visual("guideway")
    faceplate_visual = carriage.get_visual("faceplate")
    side_frames_visual = carriage.get_visual("side_frames")
    upper_runner_visual = carriage.get_visual("upper_runner")
    lower_runner_visual = carriage.get_visual("lower_runner")

    ctx.allow_overlap(
        backplate,
        carriage,
        elem_a=guide_visual,
        elem_b=upper_runner_visual,
        reason="The upper runner pad is modeled in tangent sliding contact with the guide; mesh contact is backend-classified as overlap.",
    )
    ctx.allow_overlap(
        backplate,
        carriage,
        elem_a=guide_visual,
        elem_b=lower_runner_visual,
        reason="The lower runner pad is modeled in tangent sliding contact with the guide; mesh contact is backend-classified as overlap.",
    )

    ctx.expect_gap(
        carriage,
        backplate,
        axis="y",
        positive_elem=faceplate_visual,
        negative_elem=plate_visual,
        min_gap=0.032,
        max_gap=0.038,
        name="faceplate stands proud of the wall plate",
    )
    ctx.expect_within(
        carriage,
        backplate,
        axes="x",
        margin=0.0,
        name="carriage stays within the backplate width",
    )
    ctx.expect_overlap(
        carriage,
        backplate,
        axes="z",
        elem_a=side_frames_visual,
        elem_b=guide_visual,
        min_overlap=0.090,
        name="carriage side frames remain engaged on the guide at rest",
    )
    ctx.expect_contact(
        backplate,
        carriage,
        elem_a=guide_visual,
        elem_b=upper_runner_visual,
        contact_tol=1e-5,
        name="upper runner pad contacts the guide",
    )
    ctx.expect_contact(
        backplate,
        carriage,
        elem_a=guide_visual,
        elem_b=lower_runner_visual,
        contact_tol=1e-5,
        name="lower runner pad contacts the guide",
    )

    rest_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            backplate,
            axes="z",
            elem_a=side_frames_visual,
            elem_b=guide_visual,
            min_overlap=0.090,
            name="carriage side frames remain engaged at upper travel",
        )
        upper_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: -SLIDE_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            backplate,
            axes="z",
            elem_a=side_frames_visual,
            elem_b=guide_visual,
            min_overlap=0.090,
            name="carriage side frames remain engaged at lower travel",
        )
        lower_pos = ctx.part_world_position(carriage)

    ctx.check(
        "positive prismatic travel raises the carriage",
        rest_pos is not None and upper_pos is not None and upper_pos[2] > rest_pos[2] + 0.05,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )
    ctx.check(
        "negative prismatic travel lowers the carriage",
        rest_pos is not None and lower_pos is not None and lower_pos[2] < rest_pos[2] - 0.05,
        details=f"rest={rest_pos}, lower={lower_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
