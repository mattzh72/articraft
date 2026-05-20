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


HOUSING_LENGTH = 0.118
HOUSING_WIDTH = 0.078
BASE_HEIGHT = 0.014
MOUNT_BOSS_LENGTH = 0.052
MOUNT_BOSS_WIDTH = 0.058
MOUNT_BOSS_HEIGHT = 0.020

CHEEK_LENGTH = 0.114
CHEEK_THICKNESS = 0.010
CHEEK_HEIGHT = 0.028
CHEEK_CENTER_Y = HOUSING_WIDTH / 2.0 - CHEEK_THICKNESS / 2.0

SLIDE_SHOE_LENGTH = 0.042
SLIDE_SHOE_WIDTH = 0.042
SLIDE_SHOE_HEIGHT = 0.012
SLIDE_SHOE_CENTER_X = -0.007

CARRIAGE_BLOCK_LENGTH = 0.034
CARRIAGE_BLOCK_WIDTH = 0.034
CARRIAGE_BLOCK_HEIGHT = 0.010
CARRIAGE_BLOCK_CENTER_X = -0.007

FINGER_LENGTH = 0.012
FINGER_WIDTH = 0.018
FINGER_HEIGHT = 0.072
FINGER_CENTER_X = 0.008

LEFT_JAW_OPEN_X = -0.030
RIGHT_JAW_OPEN_X = 0.030
JAW_Z = BASE_HEIGHT
JAW_TRAVEL = 0.010


def _body_base_mesh():
    return (
        cq.Workplane("XY")
        .box(HOUSING_LENGTH, HOUSING_WIDTH, BASE_HEIGHT)
        .edges("|Z")
        .fillet(0.004)
    )


def _slide_shoe_mesh():
    return (
        cq.Workplane("XY")
        .box(SLIDE_SHOE_LENGTH, SLIDE_SHOE_WIDTH, SLIDE_SHOE_HEIGHT)
        .edges("|Z")
        .fillet(0.0022)
    )


def _carriage_block_mesh():
    return (
        cq.Workplane("XY")
        .box(CARRIAGE_BLOCK_LENGTH, CARRIAGE_BLOCK_WIDTH, CARRIAGE_BLOCK_HEIGHT)
        .edges("|Z")
        .fillet(0.002)
    )


def _finger_mesh():
    return (
        cq.Workplane("XY")
        .box(FINGER_LENGTH, FINGER_WIDTH, FINGER_HEIGHT)
        .faces(">Z")
        .edges("|Y")
        .fillet(0.003)
    )


def _add_jaw_visuals(part, *, inward_sign: float, material: str) -> None:
    part.visual(
        mesh_from_cadquery(_slide_shoe_mesh(), f"{part.name}_slide_shoe"),
        origin=Origin(xyz=(inward_sign * SLIDE_SHOE_CENTER_X, 0.0, SLIDE_SHOE_HEIGHT / 2.0)),
        material=material,
        name="slide_shoe",
    )
    part.visual(
        mesh_from_cadquery(_carriage_block_mesh(), f"{part.name}_carriage_block"),
        origin=Origin(
            xyz=(
                inward_sign * CARRIAGE_BLOCK_CENTER_X,
                0.0,
                SLIDE_SHOE_HEIGHT + CARRIAGE_BLOCK_HEIGHT / 2.0,
            )
        ),
        material=material,
        name="carriage_block",
    )
    part.visual(
        mesh_from_cadquery(_finger_mesh(), f"{part.name}_finger"),
        origin=Origin(
            xyz=(
                inward_sign * FINGER_CENTER_X,
                0.0,
                SLIDE_SHOE_HEIGHT + CARRIAGE_BLOCK_HEIGHT + FINGER_HEIGHT / 2.0,
            )
        ),
        material=material,
        name="finger",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cheek_guided_twin_slide_gripper")

    model.material("body_anodized", rgba=(0.33, 0.36, 0.40, 1.0))
    model.material("jaw_steel", rgba=(0.16, 0.18, 0.21, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_body_base_mesh(), "housing_body_base"),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material="body_anodized",
        name="body_base",
    )
    housing.visual(
        Box((MOUNT_BOSS_LENGTH, MOUNT_BOSS_WIDTH, MOUNT_BOSS_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -MOUNT_BOSS_HEIGHT / 2.0)),
        material="body_anodized",
        name="mount_boss",
    )
    housing.visual(
        Box((CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -CHEEK_CENTER_Y, BASE_HEIGHT + CHEEK_HEIGHT / 2.0)
        ),
        material="body_anodized",
        name="front_cheek",
    )
    housing.visual(
        Box((CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CHEEK_CENTER_Y, BASE_HEIGHT + CHEEK_HEIGHT / 2.0)
        ),
        material="body_anodized",
        name="rear_cheek",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_LENGTH, HOUSING_WIDTH, BASE_HEIGHT + MOUNT_BOSS_HEIGHT)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT - MOUNT_BOSS_HEIGHT) / 2.0)),
    )

    left_jaw = model.part("left_jaw")
    _add_jaw_visuals(left_jaw, inward_sign=1.0, material="jaw_steel")
    left_jaw.inertial = Inertial.from_geometry(
        Box((SLIDE_SHOE_LENGTH, SLIDE_SHOE_WIDTH, SLIDE_SHOE_HEIGHT + FINGER_HEIGHT)),
        mass=0.28,
        origin=Origin(
            xyz=(
                -0.007,
                0.0,
                (SLIDE_SHOE_HEIGHT + FINGER_HEIGHT) / 2.0,
            )
        ),
    )

    right_jaw = model.part("right_jaw")
    _add_jaw_visuals(right_jaw, inward_sign=-1.0, material="jaw_steel")
    right_jaw.inertial = Inertial.from_geometry(
        Box((SLIDE_SHOE_LENGTH, SLIDE_SHOE_WIDTH, SLIDE_SHOE_HEIGHT + FINGER_HEIGHT)),
        mass=0.28,
        origin=Origin(
            xyz=(
                0.007,
                0.0,
                (SLIDE_SHOE_HEIGHT + FINGER_HEIGHT) / 2.0,
            )
        ),
    )

    model.articulation(
        "housing_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_jaw,
        origin=Origin(xyz=(LEFT_JAW_OPEN_X, 0.0, JAW_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.15,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "housing_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_jaw,
        origin=Origin(xyz=(RIGHT_JAW_OPEN_X, 0.0, JAW_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.15,
            lower=0.0,
            upper=JAW_TRAVEL,
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

    housing = object_model.get_part("housing")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("housing_to_left_jaw")
    right_slide = object_model.get_articulation("housing_to_right_jaw")

    ctx.expect_gap(
        left_jaw,
        housing,
        axis="z",
        positive_elem="slide_shoe",
        negative_elem="body_base",
        min_gap=0.0,
        max_gap=0.0005,
        name="left jaw shoe rides on the housing deck",
    )
    ctx.expect_gap(
        right_jaw,
        housing,
        axis="z",
        positive_elem="slide_shoe",
        negative_elem="body_base",
        min_gap=0.0,
        max_gap=0.0005,
        name="right jaw shoe rides on the housing deck",
    )

    for jaw_name, jaw in (("left", left_jaw), ("right", right_jaw)):
        ctx.expect_gap(
            housing,
            jaw,
            axis="y",
            positive_elem="rear_cheek",
            negative_elem="slide_shoe",
            min_gap=0.007,
            max_gap=0.009,
            name=f"{jaw_name} jaw clears the rear cheek",
        )
        ctx.expect_gap(
            jaw,
            housing,
            axis="y",
            positive_elem="slide_shoe",
            negative_elem="front_cheek",
            min_gap=0.007,
            max_gap=0.009,
            name=f"{jaw_name} jaw clears the front cheek",
        )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_overlap(
            left_jaw,
            right_jaw,
            axes="yz",
            elem_a="finger",
            elem_b="finger",
            min_overlap=0.015,
            name="open fingers stay aligned across depth and height",
        )
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            positive_elem="finger",
            negative_elem="finger",
            min_gap=0.031,
            max_gap=0.033,
            name="open pose keeps a usable finger gap",
        )
        left_open_pos = ctx.part_world_position(left_jaw)
        right_open_pos = ctx.part_world_position(right_jaw)

    with ctx.pose({left_slide: JAW_TRAVEL, right_slide: JAW_TRAVEL}):
        ctx.expect_overlap(
            left_jaw,
            right_jaw,
            axes="yz",
            elem_a="finger",
            elem_b="finger",
            min_overlap=0.015,
            name="closed fingers remain opposed and parallel",
        )
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            positive_elem="finger",
            negative_elem="finger",
            min_gap=0.011,
            max_gap=0.013,
            name="closing stroke reduces the finger gap",
        )
        left_closed_pos = ctx.part_world_position(left_jaw)
        right_closed_pos = ctx.part_world_position(right_jaw)

    ctx.check(
        "left jaw moves inward along +x",
        left_open_pos is not None
        and left_closed_pos is not None
        and left_closed_pos[0] > left_open_pos[0] + 0.009,
        details=f"open={left_open_pos}, closed={left_closed_pos}",
    )
    ctx.check(
        "right jaw moves inward along -x",
        right_open_pos is not None
        and right_closed_pos is not None
        and right_closed_pos[0] < right_open_pos[0] - 0.009,
        details=f"open={right_open_pos}, closed={right_closed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
