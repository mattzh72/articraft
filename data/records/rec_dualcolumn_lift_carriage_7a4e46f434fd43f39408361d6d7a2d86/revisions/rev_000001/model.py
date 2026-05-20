from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_W = 0.60
BASE_D = 0.34
BASE_T = 0.05

PEDESTAL_W = 0.10
PEDESTAL_D = 0.18
PEDESTAL_H = 0.055

POST_X = 0.13
POST_R = 0.016

TOP_W = 0.40
TOP_D = 0.14
TOP_T = 0.06
TOP_BOTTOM_Z = 0.79

POST_BASE_Z = BASE_T + PEDESTAL_H
POST_L = TOP_BOTTOM_Z - POST_BASE_Z

CARRIAGE_SLEEVE_W = 0.056
CARRIAGE_D = 0.10
CARRIAGE_SLEEVE_H = 0.16
CARRIAGE_CENTER_H = 0.12
GUIDE_CHEEK_T = 0.014
GUIDE_BRIDGE_T = 0.018
GUIDE_BLOCK_W = (2.0 * POST_R) + (2.0 * GUIDE_CHEEK_T)
GUIDE_BLOCK_D = (2.0 * POST_R) + (2.0 * GUIDE_BRIDGE_T)
CARRIAGE_CENTER_W = 2.0 * (POST_X - POST_R - GUIDE_CHEEK_T)

OUTPUT_FACE_R = 0.052
OUTPUT_FACE_T = 0.016

CARRIAGE_REST_Z = 0.28
CARRIAGE_TRAVEL = 0.36


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="crosshead_guided_dual_post_lift")

    model.material("frame_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("post_steel", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("carriage_orange", rgba=(0.82, 0.40, 0.10, 1.0))
    model.material("face_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BASE_T)),
        material="frame_gray",
        name="base_plate",
    )
    frame.visual(
        Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)),
        origin=Origin(xyz=(POST_X, 0.0, BASE_T + (0.5 * PEDESTAL_H))),
        material="frame_gray",
        name="left_mount",
    )
    frame.visual(
        Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)),
        origin=Origin(xyz=(-POST_X, 0.0, BASE_T + (0.5 * PEDESTAL_H))),
        material="frame_gray",
        name="right_mount",
    )
    frame.visual(
        Cylinder(radius=POST_R, length=POST_L),
        origin=Origin(xyz=(POST_X, 0.0, POST_BASE_Z + (0.5 * POST_L))),
        material="post_steel",
        name="left_post",
    )
    frame.visual(
        Cylinder(radius=POST_R, length=POST_L),
        origin=Origin(xyz=(-POST_X, 0.0, POST_BASE_Z + (0.5 * POST_L))),
        material="post_steel",
        name="right_post",
    )
    frame.visual(
        Box((TOP_W, TOP_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, TOP_BOTTOM_Z + (0.5 * TOP_T))),
        material="frame_gray",
        name="top_tie",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, TOP_BOTTOM_Z + TOP_T)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (TOP_BOTTOM_Z + TOP_T))),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_CENTER_W, CARRIAGE_D, CARRIAGE_CENTER_H)),
        material="carriage_orange",
        name="center_block",
    )
    for side_name, x_sign in (("left", 1.0), ("right", -1.0)):
        x_post = x_sign * POST_X
        x_inner = x_sign * (POST_X - POST_R - (0.5 * GUIDE_CHEEK_T))
        x_outer = x_sign * (POST_X + POST_R + (0.5 * GUIDE_CHEEK_T))

        carriage.visual(
            Box((GUIDE_CHEEK_T, GUIDE_BLOCK_D, CARRIAGE_SLEEVE_H)),
            origin=Origin(xyz=(x_inner, 0.0, 0.0)),
            material="carriage_orange",
            name=f"{side_name}_inner_cheek",
        )
        carriage.visual(
            Box((GUIDE_CHEEK_T, GUIDE_BLOCK_D, CARRIAGE_SLEEVE_H)),
            origin=Origin(xyz=(x_outer, 0.0, 0.0)),
            material="carriage_orange",
            name=f"{side_name}_outer_cheek",
        )
        carriage.visual(
            Box((GUIDE_BLOCK_W, GUIDE_BRIDGE_T, CARRIAGE_SLEEVE_H)),
            origin=Origin(
                xyz=(x_post, POST_R + (0.5 * GUIDE_BRIDGE_T), 0.0),
            ),
            material="carriage_orange",
            name=f"{side_name}_front_bridge",
        )
        carriage.visual(
            Box((GUIDE_BLOCK_W, GUIDE_BRIDGE_T, CARRIAGE_SLEEVE_H)),
            origin=Origin(
                xyz=(x_post, -POST_R - (0.5 * GUIDE_BRIDGE_T), 0.0),
            ),
            material="carriage_orange",
            name=f"{side_name}_back_bridge",
        )
    carriage.visual(
        Cylinder(radius=OUTPUT_FACE_R, length=OUTPUT_FACE_T),
        origin=Origin(
            xyz=(0.0, (0.5 * CARRIAGE_D) + (0.5 * OUTPUT_FACE_T), 0.0),
            rpy=(0.5 * math.pi, 0.0, 0.0),
        ),
        material="face_steel",
        name="output_face",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((2.0 * (POST_X + (0.5 * CARRIAGE_SLEEVE_W)), CARRIAGE_D + OUTPUT_FACE_T, CARRIAGE_SLEEVE_H)),
        mass=12.0,
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.25,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
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

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")
    output_face = carriage.get_visual("output_face")

    limits = lift.motion_limits
    ctx.check(
        "lift uses one vertical prismatic joint",
        lift.articulation_type == ArticulationType.PRISMATIC
        and lift.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 0.30,
        details=f"type={lift.articulation_type}, axis={lift.axis}, limits={limits}",
    )
    ctx.check(
        "output face stays centered on the guide axis",
        abs(output_face.origin.xyz[0]) <= 1e-9 and abs(output_face.origin.xyz[2]) <= 1e-9,
        details=f"output_face_origin={output_face.origin.xyz}",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_origin_distance(
            carriage,
            frame,
            axes="xy",
            max_dist=1e-6,
            name="carriage is centered between the two posts at rest",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="center_block",
            negative_elem="base_plate",
            min_gap=0.12,
            name="carriage clears the broad base at rest",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_b="left_post",
            name="carriage is supported by the left guide post at rest",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_b="right_post",
            name="carriage is supported by the right guide post at rest",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        ctx.expect_origin_distance(
            carriage,
            frame,
            axes="xy",
            max_dist=1e-6,
            name="carriage remains centered while raised",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="top_tie",
            negative_elem="center_block",
            min_gap=0.05,
            name="carriage clears the top tie at full lift",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_b="left_post",
            name="carriage remains guided on the left post when raised",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_b="right_post",
            name="carriage remains guided on the right post when raised",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage travels upward along the shared guide axis",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.30
        and abs(raised_pos[0] - rest_pos[0]) <= 1e-6
        and abs(raised_pos[1] - rest_pos[1]) <= 1e-6,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
