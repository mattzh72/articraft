from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.420
OUTER_WIDTH = 0.076
OUTER_HEIGHT = 0.050
OUTER_WALL = 0.0035
OUTER_MOUNT_LENGTH = 0.120
OUTER_MOUNT_WIDTH = 0.104
OUTER_MOUNT_THICKNESS = 0.006
OUTER_GLIDE_START = OUTER_WALL + 0.008
OUTER_GLIDE_LENGTH = OUTER_LENGTH - OUTER_GLIDE_START - 0.018
OUTER_GLIDE_WIDTH = 0.024

MIDDLE_LENGTH = 0.340
MIDDLE_WIDTH = 0.066
MIDDLE_WALL = 0.0028
MIDDLE_HEIGHT = OUTER_HEIGHT - 2.0 * OUTER_WALL - 0.0040
MIDDLE_GLIDE_START = MIDDLE_WALL + 0.008
MIDDLE_GLIDE_LENGTH = MIDDLE_LENGTH - MIDDLE_GLIDE_START - 0.018
MIDDLE_GLIDE_WIDTH = 0.020

INNER_LENGTH = 0.280
INNER_WIDTH = 0.056
INNER_HEIGHT = MIDDLE_HEIGHT - 2.0 * MIDDLE_WALL - 0.0020
INNER_HEAD_LENGTH = 0.024
INNER_HEAD_WIDTH = 0.058
INNER_HEAD_HEIGHT = INNER_HEIGHT

OUTER_GLIDE_HEIGHT = (OUTER_HEIGHT - 2.0 * OUTER_WALL - MIDDLE_HEIGHT) / 2.0
MIDDLE_GLIDE_HEIGHT = (MIDDLE_HEIGHT - 2.0 * MIDDLE_WALL - INNER_HEIGHT) / 2.0

OUTER_HOME_OFFSET = OUTER_WALL + 0.0020
INNER_HOME_OFFSET = MIDDLE_WALL + 0.0020
OUTER_RETAINED_INSERTION = 0.120
INNER_RETAINED_INSERTION = 0.090
OUTER_TRAVEL = OUTER_LENGTH - OUTER_HOME_OFFSET - OUTER_RETAINED_INSERTION
INNER_TRAVEL = MIDDLE_LENGTH - INNER_HOME_OFFSET - INNER_RETAINED_INSERTION

OUTER_GLIDE_STRIP_NAME = "outer_glide_strip"
MIDDLE_BOTTOM_WALL_NAME = "middle_bottom_wall"
MIDDLE_GLIDE_STRIP_NAME = "middle_glide_strip"
INNER_BAR_NAME = "inner_bar"


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


def _add_channel_section(
    part,
    *,
    prefix: str,
    material: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    glide_start: float,
    glide_length: float,
    glide_width: float,
    glide_height: float,
) -> None:
    side_y = width / 2.0 - wall / 2.0
    top_z = height / 2.0 - wall / 2.0

    _add_box_visual(
        part,
        name=f"{prefix}_left_wall",
        size=(length, wall, height),
        xyz=(length / 2.0, side_y, 0.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_right_wall",
        size=(length, wall, height),
        xyz=(length / 2.0, -side_y, 0.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_top_wall",
        size=(length, width - 2.0 * wall, wall),
        xyz=(length / 2.0, 0.0, top_z),
        material=material,
    )
    _add_box_visual(
        part,
        name=MIDDLE_BOTTOM_WALL_NAME if prefix == "middle" else f"{prefix}_bottom_wall",
        size=(length, width - 2.0 * wall, wall),
        xyz=(length / 2.0, 0.0, -top_z),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_rear_wall",
        size=(wall, width - 2.0 * wall, height - 2.0 * wall),
        xyz=(wall / 2.0, 0.0, 0.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=(
            OUTER_GLIDE_STRIP_NAME
            if prefix == "outer"
            else MIDDLE_GLIDE_STRIP_NAME if prefix == "middle" else f"{prefix}_glide_strip"
        ),
        size=(glide_length, glide_width, glide_height),
        xyz=(
            glide_start + glide_length / 2.0,
            0.0,
            -height / 2.0 + wall + glide_height / 2.0,
        ),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_box_telescoping_slide")

    model.material("outer_dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("middle_galvanized", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("inner_aluminum", rgba=(0.82, 0.84, 0.87, 1.0))

    outer = model.part("outer_section")
    _add_channel_section(
        outer,
        prefix="outer",
        material="outer_dark_steel",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        glide_start=OUTER_GLIDE_START,
        glide_length=OUTER_GLIDE_LENGTH,
        glide_width=OUTER_GLIDE_WIDTH,
        glide_height=OUTER_GLIDE_HEIGHT,
    )
    _add_box_visual(
        outer,
        name="outer_mount_plate",
        size=(OUTER_MOUNT_LENGTH, OUTER_MOUNT_WIDTH, OUTER_MOUNT_THICKNESS),
        xyz=(
            OUTER_MOUNT_LENGTH / 2.0,
            0.0,
            -OUTER_HEIGHT / 2.0 - OUTER_MOUNT_THICKNESS / 2.0,
        ),
        material="outer_dark_steel",
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_MOUNT_WIDTH, OUTER_HEIGHT + OUTER_MOUNT_THICKNESS)),
        mass=2.2,
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                0.0,
                -OUTER_MOUNT_THICKNESS / 2.0,
            )
        ),
    )

    middle = model.part("middle_section")
    _add_channel_section(
        middle,
        prefix="middle",
        material="middle_galvanized",
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        glide_start=MIDDLE_GLIDE_START,
        glide_length=MIDDLE_GLIDE_LENGTH,
        glide_width=MIDDLE_GLIDE_WIDTH,
        glide_height=MIDDLE_GLIDE_HEIGHT,
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=1.3,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, 0.0)),
    )

    inner = model.part("inner_section")
    _add_box_visual(
        inner,
        name=INNER_BAR_NAME,
        size=(INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT),
        xyz=(INNER_LENGTH / 2.0, 0.0, 0.0),
        material="inner_aluminum",
    )
    _add_box_visual(
        inner,
        name="inner_head",
        size=(INNER_HEAD_LENGTH, INNER_HEAD_WIDTH, INNER_HEAD_HEIGHT),
        xyz=(INNER_LENGTH - INNER_HEAD_LENGTH / 2.0, 0.0, 0.0),
        material="inner_aluminum",
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_HEAD_WIDTH, INNER_HEAD_HEIGHT)),
        mass=0.8,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_HOME_OFFSET, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TRAVEL,
            effort=180.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(INNER_HOME_OFFSET, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TRAVEL,
            effort=120.0,
            velocity=0.40,
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

    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_section")
    inner = object_model.get_part("inner_section")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0,
        name="middle section stays centered in outer section at rest",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.300,
        name="middle section is deeply inserted at rest",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.0,
        name="inner section stays centered in middle section at rest",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.260,
        name="inner section is deeply inserted at rest",
    )
    ctx.expect_contact(
        outer,
        middle,
        elem_a=OUTER_GLIDE_STRIP_NAME,
        elem_b=MIDDLE_BOTTOM_WALL_NAME,
        name="middle section rests on outer glide strip at rest",
    )
    ctx.expect_contact(
        middle,
        inner,
        elem_a=MIDDLE_GLIDE_STRIP_NAME,
        elem_b=INNER_BAR_NAME,
        name="inner section rests on middle glide strip at rest",
    )

    middle_rest = ctx.part_world_position(middle)
    with ctx.pose({outer_to_middle: OUTER_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0,
            name="extended middle section stays guided in outer section",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=OUTER_RETAINED_INSERTION,
            name="extended middle section retains outer insertion",
        )
        ctx.expect_contact(
            outer,
            middle,
            elem_a=OUTER_GLIDE_STRIP_NAME,
            elem_b=MIDDLE_BOTTOM_WALL_NAME,
            name="extended middle section remains supported by outer glide strip",
        )
        middle_extended = ctx.part_world_position(middle)

    ctx.check(
        "outer_to_middle moves along +X",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + OUTER_TRAVEL * 0.95,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )

    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({middle_to_inner: INNER_TRAVEL}):
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="extended inner section stays guided in middle section",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=INNER_RETAINED_INSERTION - 0.001,
            name="extended inner section retains middle insertion",
        )
        ctx.expect_contact(
            middle,
            inner,
            elem_a=MIDDLE_GLIDE_STRIP_NAME,
            elem_b=INNER_BAR_NAME,
            name="extended inner section remains supported by middle glide strip",
        )
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "middle_to_inner moves along +X",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + INNER_TRAVEL * 0.95,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    with ctx.pose({outer_to_middle: OUTER_TRAVEL * 0.7, middle_to_inner: INNER_TRAVEL}):
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="serially extended inner section remains centered",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=INNER_RETAINED_INSERTION - 0.001,
            name="serially extended inner section keeps retained insertion",
        )
        ctx.expect_contact(
            middle,
            inner,
            elem_a=MIDDLE_GLIDE_STRIP_NAME,
            elem_b=INNER_BAR_NAME,
            name="serially extended inner section stays supported by middle glide strip",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
