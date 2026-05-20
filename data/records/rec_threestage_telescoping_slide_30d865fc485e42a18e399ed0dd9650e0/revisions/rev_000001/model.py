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


OUTER_LENGTH = 0.460
OUTER_WIDTH = 0.060
OUTER_HEIGHT = 0.028
OUTER_WALL = 0.003

MIDDLE_LENGTH = 0.400
MIDDLE_WIDTH = 0.050
MIDDLE_HEIGHT = 0.020
MIDDLE_WALL = 0.003

INNER_LENGTH = 0.285
INNER_WIDTH = 0.040
INNER_HEIGHT = 0.012

TIP_PLATE_THICKNESS = 0.008
TIP_PLATE_WIDTH = 0.058
TIP_PLATE_HEIGHT = 0.026

MOUNT_PAD_LENGTH = 0.075
MOUNT_PAD_WIDTH = 0.074
MOUNT_PAD_THICKNESS = 0.004
REAR_PAD_START = 0.030
FRONT_PAD_START = 0.355

OUTER_TO_MIDDLE_HOME = OUTER_LENGTH - MIDDLE_LENGTH
OUTER_TO_MIDDLE_TRAVEL = 0.230
MIDDLE_TO_INNER_HOME = MIDDLE_LENGTH - INNER_LENGTH
MIDDLE_TO_INNER_TRAVEL = 0.180


def _rectangular_sleeve(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    name: str,
):
    outer_box = cq.Workplane("XY").box(
        length,
        width,
        height,
        centered=(False, True, True),
    )
    inner_void = (
        cq.Workplane("XY")
        .box(
            length + 0.002,
            width - 2.0 * wall,
            height - 2.0 * wall,
            centered=(False, True, True),
        )
        .translate((-0.001, 0.0, 0.0))
    )
    sleeve = outer_box.cut(inner_void)
    return mesh_from_cadquery(sleeve, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_nested_runner")

    model.material("outer_metal", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("middle_metal", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("inner_metal", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("tip_plate_finish", rgba=(0.18, 0.19, 0.22, 1.0))

    outer = model.part("outer_guide")
    outer.visual(
        _rectangular_sleeve(
            length=OUTER_LENGTH,
            width=OUTER_WIDTH,
            height=OUTER_HEIGHT,
            wall=OUTER_WALL,
            name="outer_sleeve",
        ),
        material="outer_metal",
        name="outer_sleeve",
    )
    outer.visual(
        Box((MOUNT_PAD_LENGTH, MOUNT_PAD_WIDTH, MOUNT_PAD_THICKNESS)),
        origin=Origin(
            xyz=(
                REAR_PAD_START + MOUNT_PAD_LENGTH / 2.0,
                0.0,
                -OUTER_HEIGHT / 2.0 - MOUNT_PAD_THICKNESS / 2.0,
            )
        ),
        material="outer_metal",
        name="outer_mount_pad_rear",
    )
    outer.visual(
        Box((MOUNT_PAD_LENGTH, MOUNT_PAD_WIDTH, MOUNT_PAD_THICKNESS)),
        origin=Origin(
            xyz=(
                FRONT_PAD_START + MOUNT_PAD_LENGTH / 2.0,
                0.0,
                -OUTER_HEIGHT / 2.0 - MOUNT_PAD_THICKNESS / 2.0,
            )
        ),
        material="outer_metal",
        name="outer_mount_pad_front",
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, MOUNT_PAD_WIDTH, OUTER_HEIGHT + MOUNT_PAD_THICKNESS)),
        mass=1.35,
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                0.0,
                -MOUNT_PAD_THICKNESS / 2.0,
            )
        ),
    )

    middle = model.part("middle_stage")
    middle.visual(
        _rectangular_sleeve(
            length=MIDDLE_LENGTH,
            width=MIDDLE_WIDTH,
            height=MIDDLE_HEIGHT,
            wall=MIDDLE_WALL,
            name="middle_sleeve",
        ),
        material="middle_metal",
        name="middle_sleeve",
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=0.72,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, 0.0)),
    )

    inner = model.part("inner_stage")
    inner.visual(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, 0.0)),
        material="inner_metal",
        name="inner_runner",
    )
    inner.visual(
        Box((TIP_PLATE_THICKNESS, TIP_PLATE_WIDTH, TIP_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(
                INNER_LENGTH + TIP_PLATE_THICKNESS / 2.0,
                0.0,
                0.0,
            )
        ),
        material="tip_plate_finish",
        name="tip_plate",
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH + TIP_PLATE_THICKNESS, TIP_PLATE_WIDTH, TIP_PLATE_HEIGHT)),
        mass=0.42,
        origin=Origin(
            xyz=((INNER_LENGTH + TIP_PLATE_THICKNESS) / 2.0, 0.0, 0.0)
        ),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=85.0,
            velocity=0.40,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
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
    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    outer_sleeve = outer.get_visual("outer_sleeve")
    middle_sleeve = middle.get_visual("middle_sleeve")
    inner_runner = inner.get_visual("inner_runner")
    tip_plate = inner.get_visual("tip_plate")

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        inner_elem=middle_sleeve,
        outer_elem=outer_sleeve,
        margin=0.002,
        name="middle sleeve stays centered inside outer guide",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a=middle_sleeve,
        elem_b=outer_sleeve,
        min_overlap=0.165,
        name="middle sleeve retains insertion inside outer guide",
    )

    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        inner_elem=inner_runner,
        outer_elem=middle_sleeve,
        margin=0.0015,
        name="inner runner stays centered inside middle sleeve",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a=inner_runner,
        elem_b=middle_sleeve,
        min_overlap=0.110,
        name="inner runner retains insertion inside middle sleeve",
    )
    ctx.expect_contact(
        inner,
        inner,
        elem_a=inner_runner,
        elem_b=tip_plate,
        name="tip plate is attached to the inner runner",
    )

    middle_rest = ctx.part_world_position(middle)
    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            inner_elem=middle_sleeve,
            outer_elem=outer_sleeve,
            margin=0.002,
            name="extended middle sleeve stays centered inside outer guide",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a=middle_sleeve,
            elem_b=outer_sleeve,
            min_overlap=0.165,
            name="extended middle sleeve still retains insertion",
        )
        middle_extended = ctx.part_world_position(middle)

    ctx.check(
        "middle stage extends along +X",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.20,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )

    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({middle_to_inner: MIDDLE_TO_INNER_TRAVEL}):
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            inner_elem=inner_runner,
            outer_elem=middle_sleeve,
            margin=0.0015,
            name="extended inner runner stays centered inside middle sleeve",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a=inner_runner,
            elem_b=middle_sleeve,
            min_overlap=0.100,
            name="extended inner runner still retains insertion",
        )
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "inner stage extends along +X",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.15,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
