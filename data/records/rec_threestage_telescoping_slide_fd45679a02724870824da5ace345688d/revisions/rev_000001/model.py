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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_DEPTH = 0.62
BODY_WIDTH = 0.48
BODY_HEIGHT = 0.24
BODY_WALL = 0.018
BODY_BACK = 0.020
BODY_FLOOR = 0.020
BODY_RAIL_WIDTH = 0.028
BODY_RAIL_HEIGHT = 0.012
BODY_RAIL_Y = BODY_WIDTH / 2.0 - BODY_WALL - BODY_RAIL_WIDTH / 2.0 - 0.004
BODY_RAIL_X0 = -BODY_DEPTH + BODY_BACK + 0.060
BODY_RAIL_X1 = -0.060

STAGE1_DEPTH = 0.50
STAGE1_WIDTH = 0.40
STAGE1_HEIGHT = 0.16
STAGE1_WALL = 0.014
STAGE1_BACK = 0.016
STAGE1_FLOOR = 0.014
STAGE1_FRONT = 0.018
STAGE1_ROLLER_RADIUS = 0.006
STAGE1_ROLLER_LENGTH = 0.030
STAGE1_ROLLER_Y = BODY_RAIL_Y
STAGE1_ROLLER_XS = (-0.370, -0.140)
STAGE1_INNER_RAIL_WIDTH = 0.020
STAGE1_INNER_RAIL_HEIGHT = 0.010
STAGE1_INNER_RAIL_Y = 0.160
STAGE1_INNER_RAIL_X0 = -0.400
STAGE1_INNER_RAIL_X1 = -0.080

STAGE2_DEPTH = 0.34
STAGE2_WIDTH = 0.32
STAGE2_HEIGHT = 0.10
STAGE2_WALL = 0.012
STAGE2_BACK = 0.014
STAGE2_FLOOR = 0.012
STAGE2_FRONT = 0.016
STAGE2_ROLLER_RADIUS = 0.005
STAGE2_ROLLER_LENGTH = 0.026
STAGE2_ROLLER_Y = STAGE1_INNER_RAIL_Y
STAGE2_ROLLER_XS = (-0.225, -0.095)

BODY_TO_STAGE1_HOME = -0.014
BODY_TO_STAGE1_TRAVEL = 0.24
STAGE1_TO_STAGE2_HOME = -0.090
STAGE1_TO_STAGE2_TRAVEL = 0.17

def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_y_roller(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_three_stage_slide")

    model.material("body_powdercoat", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("middle_stage", rgba=(0.46, 0.49, 0.53, 1.0))
    model.material("inner_stage", rgba=(0.73, 0.75, 0.78, 1.0))

    body = model.part("body")
    _add_box_visual(
        body,
        name="body_floor",
        size=(BODY_DEPTH, BODY_WIDTH, BODY_FLOOR),
        xyz=(-BODY_DEPTH / 2.0, 0.0, BODY_FLOOR / 2.0),
        material="body_powdercoat",
    )
    _add_box_visual(
        body,
        name="body_left_wall",
        size=(BODY_DEPTH - BODY_BACK, BODY_WALL, BODY_HEIGHT - BODY_FLOOR),
        xyz=(
            -(BODY_DEPTH - BODY_BACK) / 2.0,
            BODY_WIDTH / 2.0 - BODY_WALL / 2.0,
            BODY_FLOOR + (BODY_HEIGHT - BODY_FLOOR) / 2.0,
        ),
        material="body_powdercoat",
    )
    _add_box_visual(
        body,
        name="body_right_wall",
        size=(BODY_DEPTH - BODY_BACK, BODY_WALL, BODY_HEIGHT - BODY_FLOOR),
        xyz=(
            -(BODY_DEPTH - BODY_BACK) / 2.0,
            -BODY_WIDTH / 2.0 + BODY_WALL / 2.0,
            BODY_FLOOR + (BODY_HEIGHT - BODY_FLOOR) / 2.0,
        ),
        material="body_powdercoat",
    )
    _add_box_visual(
        body,
        name="body_back_wall",
        size=(BODY_BACK, BODY_WIDTH, BODY_HEIGHT),
        xyz=(-BODY_DEPTH + BODY_BACK / 2.0, 0.0, BODY_HEIGHT / 2.0),
        material="body_powdercoat",
    )
    _add_box_visual(
        body,
        name="body_left_runner",
        size=(BODY_RAIL_X1 - BODY_RAIL_X0, BODY_RAIL_WIDTH, BODY_RAIL_HEIGHT),
        xyz=(
            (BODY_RAIL_X0 + BODY_RAIL_X1) / 2.0,
            BODY_RAIL_Y,
            BODY_FLOOR + BODY_RAIL_HEIGHT / 2.0,
        ),
        material="body_powdercoat",
    )
    _add_box_visual(
        body,
        name="body_right_runner",
        size=(BODY_RAIL_X1 - BODY_RAIL_X0, BODY_RAIL_WIDTH, BODY_RAIL_HEIGHT),
        xyz=(
            (BODY_RAIL_X0 + BODY_RAIL_X1) / 2.0,
            -BODY_RAIL_Y,
            BODY_FLOOR + BODY_RAIL_HEIGHT / 2.0,
        ),
        material="body_powdercoat",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(-BODY_DEPTH / 2.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    stage1 = model.part("stage1")
    _add_box_visual(
        stage1,
        name="stage1_floor",
        size=(STAGE1_DEPTH, STAGE1_WIDTH, STAGE1_FLOOR),
        xyz=(-STAGE1_DEPTH / 2.0, 0.0, STAGE1_FLOOR / 2.0),
        material="middle_stage",
    )
    _add_box_visual(
        stage1,
        name="stage1_left_wall",
        size=(STAGE1_DEPTH - STAGE1_BACK, STAGE1_WALL, STAGE1_HEIGHT - STAGE1_FLOOR),
        xyz=(
            -(STAGE1_DEPTH - STAGE1_BACK) / 2.0,
            STAGE1_WIDTH / 2.0 - STAGE1_WALL / 2.0,
            STAGE1_FLOOR + (STAGE1_HEIGHT - STAGE1_FLOOR) / 2.0,
        ),
        material="middle_stage",
    )
    _add_box_visual(
        stage1,
        name="stage1_right_wall",
        size=(STAGE1_DEPTH - STAGE1_BACK, STAGE1_WALL, STAGE1_HEIGHT - STAGE1_FLOOR),
        xyz=(
            -(STAGE1_DEPTH - STAGE1_BACK) / 2.0,
            -STAGE1_WIDTH / 2.0 + STAGE1_WALL / 2.0,
            STAGE1_FLOOR + (STAGE1_HEIGHT - STAGE1_FLOOR) / 2.0,
        ),
        material="middle_stage",
    )
    _add_box_visual(
        stage1,
        name="stage1_back_wall",
        size=(STAGE1_BACK, STAGE1_WIDTH, STAGE1_HEIGHT),
        xyz=(-STAGE1_DEPTH + STAGE1_BACK / 2.0, 0.0, STAGE1_HEIGHT / 2.0),
        material="middle_stage",
    )
    _add_box_visual(
        stage1,
        name="stage1_front_face",
        size=(STAGE1_FRONT, STAGE1_WIDTH, STAGE1_HEIGHT),
        xyz=(STAGE1_FRONT / 2.0, 0.0, STAGE1_HEIGHT / 2.0),
        material="middle_stage",
    )
    _add_box_visual(
        stage1,
        name="stage1_left_inner_rail",
        size=(
            STAGE1_INNER_RAIL_X1 - STAGE1_INNER_RAIL_X0,
            STAGE1_INNER_RAIL_WIDTH,
            STAGE1_INNER_RAIL_HEIGHT,
        ),
        xyz=(
            (STAGE1_INNER_RAIL_X0 + STAGE1_INNER_RAIL_X1) / 2.0,
            STAGE1_INNER_RAIL_Y,
            STAGE1_FLOOR + STAGE1_INNER_RAIL_HEIGHT / 2.0,
        ),
        material="middle_stage",
    )
    _add_box_visual(
        stage1,
        name="stage1_right_inner_rail",
        size=(
            STAGE1_INNER_RAIL_X1 - STAGE1_INNER_RAIL_X0,
            STAGE1_INNER_RAIL_WIDTH,
            STAGE1_INNER_RAIL_HEIGHT,
        ),
        xyz=(
            (STAGE1_INNER_RAIL_X0 + STAGE1_INNER_RAIL_X1) / 2.0,
            -STAGE1_INNER_RAIL_Y,
            STAGE1_FLOOR + STAGE1_INNER_RAIL_HEIGHT / 2.0,
        ),
        material="middle_stage",
    )
    for index, x_center in enumerate(STAGE1_ROLLER_XS, start=1):
        _add_y_roller(
            stage1,
            name=f"stage1_left_roller_{index}",
            radius=STAGE1_ROLLER_RADIUS,
            length=STAGE1_ROLLER_LENGTH,
            xyz=(x_center, STAGE1_ROLLER_Y, STAGE1_ROLLER_RADIUS),
            material="middle_stage",
        )
        _add_y_roller(
            stage1,
            name=f"stage1_right_roller_{index}",
            radius=STAGE1_ROLLER_RADIUS,
            length=STAGE1_ROLLER_LENGTH,
            xyz=(x_center, -STAGE1_ROLLER_Y, STAGE1_ROLLER_RADIUS),
            material="middle_stage",
        )
    stage1.inertial = Inertial.from_geometry(
        Box((STAGE1_DEPTH + STAGE1_FRONT, STAGE1_WIDTH + 0.040, STAGE1_HEIGHT)),
        mass=8.5,
        origin=Origin(
            xyz=((-STAGE1_DEPTH + STAGE1_FRONT) / 2.0, 0.0, STAGE1_HEIGHT / 2.0)
        ),
    )

    stage2 = model.part("stage2")
    _add_box_visual(
        stage2,
        name="stage2_floor",
        size=(STAGE2_DEPTH, STAGE2_WIDTH, STAGE2_FLOOR),
        xyz=(-STAGE2_DEPTH / 2.0, 0.0, STAGE2_FLOOR / 2.0),
        material="inner_stage",
    )
    _add_box_visual(
        stage2,
        name="stage2_left_wall",
        size=(STAGE2_DEPTH - STAGE2_BACK, STAGE2_WALL, STAGE2_HEIGHT - STAGE2_FLOOR),
        xyz=(
            -(STAGE2_DEPTH - STAGE2_BACK) / 2.0,
            STAGE2_WIDTH / 2.0 - STAGE2_WALL / 2.0,
            STAGE2_FLOOR + (STAGE2_HEIGHT - STAGE2_FLOOR) / 2.0,
        ),
        material="inner_stage",
    )
    _add_box_visual(
        stage2,
        name="stage2_right_wall",
        size=(STAGE2_DEPTH - STAGE2_BACK, STAGE2_WALL, STAGE2_HEIGHT - STAGE2_FLOOR),
        xyz=(
            -(STAGE2_DEPTH - STAGE2_BACK) / 2.0,
            -STAGE2_WIDTH / 2.0 + STAGE2_WALL / 2.0,
            STAGE2_FLOOR + (STAGE2_HEIGHT - STAGE2_FLOOR) / 2.0,
        ),
        material="inner_stage",
    )
    _add_box_visual(
        stage2,
        name="stage2_back_wall",
        size=(STAGE2_BACK, STAGE2_WIDTH, STAGE2_HEIGHT),
        xyz=(-STAGE2_DEPTH + STAGE2_BACK / 2.0, 0.0, STAGE2_HEIGHT / 2.0),
        material="inner_stage",
    )
    _add_box_visual(
        stage2,
        name="stage2_front_face",
        size=(STAGE2_FRONT, STAGE2_WIDTH, STAGE2_HEIGHT),
        xyz=(STAGE2_FRONT / 2.0, 0.0, STAGE2_HEIGHT / 2.0),
        material="inner_stage",
    )
    for index, x_center in enumerate(STAGE2_ROLLER_XS, start=1):
        _add_y_roller(
            stage2,
            name=f"stage2_left_roller_{index}",
            radius=STAGE2_ROLLER_RADIUS,
            length=STAGE2_ROLLER_LENGTH,
            xyz=(x_center, STAGE2_ROLLER_Y, STAGE2_ROLLER_RADIUS),
            material="inner_stage",
        )
        _add_y_roller(
            stage2,
            name=f"stage2_right_roller_{index}",
            radius=STAGE2_ROLLER_RADIUS,
            length=STAGE2_ROLLER_LENGTH,
            xyz=(x_center, -STAGE2_ROLLER_Y, STAGE2_ROLLER_RADIUS),
            material="inner_stage",
        )
    stage2.inertial = Inertial.from_geometry(
        Box((STAGE2_DEPTH + STAGE2_FRONT, STAGE2_WIDTH + 0.030, STAGE2_HEIGHT)),
        mass=4.0,
        origin=Origin(
            xyz=((-STAGE2_DEPTH + STAGE2_FRONT) / 2.0, 0.0, STAGE2_HEIGHT / 2.0)
        ),
    )

    body_to_stage1 = model.articulation(
        "body_to_stage1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stage1,
        origin=Origin(xyz=(BODY_TO_STAGE1_HOME, 0.0, BODY_FLOOR + BODY_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=BODY_TO_STAGE1_TRAVEL,
            effort=150.0,
            velocity=0.30,
        ),
    )
    stage1_to_stage2 = model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(
            xyz=(
                STAGE1_TO_STAGE2_HOME,
                0.0,
                STAGE1_FLOOR + STAGE1_INNER_RAIL_HEIGHT,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE1_TO_STAGE2_TRAVEL,
            effort=100.0,
            velocity=0.35,
        ),
    )

    body_to_stage1.meta["role"] = "primary_stage_slide"
    stage1_to_stage2.meta["role"] = "secondary_stage_slide"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    body_to_stage1 = object_model.get_articulation("body_to_stage1")
    stage1_to_stage2 = object_model.get_articulation("stage1_to_stage2")

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

    ctx.expect_contact(
        stage1,
        body,
        name="stage1 is supported by the grounded body runners",
    )
    ctx.expect_contact(
        stage2,
        stage1,
        name="stage2 is supported by the stage1 runners",
    )
    ctx.expect_within(
        stage1,
        body,
        axes="yz",
        margin=0.0,
        name="stage1 stays laterally contained by the body",
    )
    ctx.expect_within(
        stage2,
        stage1,
        axes="yz",
        margin=0.0,
        name="stage2 stays laterally contained by stage1",
    )
    ctx.expect_overlap(
        stage1,
        body,
        axes="x",
        min_overlap=0.22,
        name="stage1 retains insertion in the body at rest",
    )
    ctx.expect_overlap(
        stage2,
        stage1,
        axes="x",
        min_overlap=0.14,
        name="stage2 retains insertion in stage1 at rest",
    )
    ctx.expect_origin_gap(
        stage1,
        stage2,
        axis="x",
        min_gap=0.08,
        max_gap=0.10,
        name="stage2 front starts recessed behind stage1",
    )

    stage1_rest = ctx.part_world_position(stage1)
    stage2_rest = ctx.part_world_position(stage2)
    with ctx.pose({body_to_stage1: BODY_TO_STAGE1_TRAVEL}):
        ctx.expect_contact(
            stage1,
            body,
            name="stage1 remains runner-supported at full first-stage extension",
        )
        ctx.expect_overlap(
            stage1,
            body,
            axes="x",
            min_overlap=0.10,
            name="stage1 keeps retained insertion at full first-stage extension",
        )
        stage1_extended = ctx.part_world_position(stage1)

    with ctx.pose(
        {
            body_to_stage1: BODY_TO_STAGE1_TRAVEL,
            stage1_to_stage2: STAGE1_TO_STAGE2_TRAVEL,
        }
    ):
        ctx.expect_contact(
            stage2,
            stage1,
            name="stage2 remains runner-supported at full extension",
        )
        ctx.expect_overlap(
            stage2,
            stage1,
            axes="x",
            min_overlap=0.06,
            name="stage2 keeps retained insertion at full extension",
        )
        stage2_extended = ctx.part_world_position(stage2)
        stage1_with_stage2_extended = ctx.part_world_position(stage1)

    ctx.check(
        "stage1 extends along +X",
        stage1_rest is not None
        and stage1_extended is not None
        and stage1_extended[0] > stage1_rest[0] + 0.20,
        details=f"rest={stage1_rest}, extended={stage1_extended}",
    )
    ctx.check(
        "stage2 extends farther forward than stage1",
        stage2_rest is not None
        and stage2_extended is not None
        and stage1_with_stage2_extended is not None
        and stage2_extended[0] > stage1_with_stage2_extended[0] + 0.06,
        details=(
            f"stage2_rest={stage2_rest}, stage1_full={stage1_with_stage2_extended}, "
            f"stage2_full={stage2_extended}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
