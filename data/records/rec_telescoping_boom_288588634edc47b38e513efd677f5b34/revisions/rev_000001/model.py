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


BASE_LENGTH = 1.00
BASE_WIDTH = 0.72
BASE_THICKNESS = 0.05

SUPPORT_OUTER_ORIGIN = (0.02, 0.0, 0.68)

OUTER_LENGTH = 1.60
OUTER_WIDTH = 0.20
OUTER_HEIGHT = 0.16
OUTER_WALL = 0.012

MIDDLE_LENGTH = 1.42
MIDDLE_WIDTH = 0.164
MIDDLE_HEIGHT = 0.124
MIDDLE_WALL = 0.010

INNER1_LENGTH = 1.22
INNER1_WIDTH = 0.128
INNER1_HEIGHT = 0.094
INNER1_WALL = 0.008

INNER2_LENGTH = 1.05
INNER2_WIDTH = 0.094
INNER2_HEIGHT = 0.068
INNER2_WALL = 0.006

SLIDE_CLEARANCE = 0.002

OUTER_TO_MIDDLE_HOME = 0.30
OUTER_TO_MIDDLE_TRAVEL = 0.82

MIDDLE_TO_INNER1_HOME = 0.28
MIDDLE_TO_INNER1_TRAVEL = 0.70

INNER1_TO_INNER2_HOME = 0.26
INNER1_TO_INNER2_TRAVEL = 0.58


def _box_prism(
    length: float,
    width: float,
    height: float,
    *,
    x0: float,
    z0: float,
    y0: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        length,
        width,
        height,
        centered=(False, True, False),
    ).translate((x0, y0, z0))


def _tri_gusset(
    points: list[tuple[float, float]],
    *,
    thickness: float,
    y0: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, y0, 0.0))
    )


def _box_tube(
    length: float,
    width: float,
    height: float,
    wall: float,
    *,
    front_capped: bool = False,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        length,
        width,
        height,
        centered=(False, True, False),
    )
    inner_length = length - 2.0 * wall if front_capped else length - wall
    cavity = cq.Workplane("XY").box(
        inner_length,
        width - 2.0 * wall,
        height - 2.0 * wall,
        centered=(False, True, False),
    ).translate((wall, 0.0, wall))
    return outer.cut(cavity)


def _support_shape() -> cq.Workplane:
    support = _box_prism(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        x0=-0.18,
        z0=0.0,
    )
    support = support.union(_box_prism(0.30, 0.30, 0.42, x0=0.02, z0=BASE_THICKNESS))
    support = support.union(_box_prism(0.20, 0.24, 0.13, x0=0.07, z0=0.47))
    support = support.union(_box_prism(0.24, 0.18, 0.04, x0=0.08, z0=0.60))
    support = support.union(_box_prism(0.30, 0.05, 0.04, x0=0.04, z0=0.64, y0=0.055))
    support = support.union(_box_prism(0.30, 0.05, 0.04, x0=0.04, z0=0.64, y0=-0.055))
    support = support.union(_box_prism(0.18, 0.02, 0.17, x0=0.10, z0=0.47, y0=0.085))
    support = support.union(_box_prism(0.18, 0.02, 0.17, x0=0.10, z0=0.47, y0=-0.085))

    left_gusset = _tri_gusset(
        [(0.10, 0.47), (0.18, 0.47), (0.34, 0.64), (0.16, 0.64)],
        thickness=0.016,
        y0=0.090,
    )
    right_gusset = _tri_gusset(
        [(0.10, 0.47), (0.18, 0.47), (0.34, 0.64), (0.16, 0.64)],
        thickness=0.016,
        y0=-0.106,
    )
    support = support.union(left_gusset)
    support = support.union(right_gusset)
    return support


def _outer_section_shape() -> cq.Workplane:
    section = _box_tube(
        OUTER_LENGTH,
        OUTER_WIDTH,
        OUTER_HEIGHT,
        OUTER_WALL,
    )
    doubler = _box_prism(0.36, 0.12, 0.012, x0=0.08, z0=OUTER_HEIGHT)
    return section.union(doubler)


def _middle_section_shape() -> cq.Workplane:
    return _box_tube(
        MIDDLE_LENGTH,
        MIDDLE_WIDTH,
        MIDDLE_HEIGHT,
        MIDDLE_WALL,
    )


def _inner1_section_shape() -> cq.Workplane:
    return _box_tube(
        INNER1_LENGTH,
        INNER1_WIDTH,
        INNER1_HEIGHT,
        INNER1_WALL,
    )


def _inner2_section_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(
        INNER2_LENGTH,
        INNER2_WIDTH,
        INNER2_HEIGHT,
        centered=(False, True, False),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_telescoping_boom")

    model.material("support_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("outer_paint", rgba=(0.84, 0.63, 0.15, 1.0))
    model.material("middle_paint", rgba=(0.74, 0.55, 0.13, 1.0))
    model.material("inner_dark", rgba=(0.45, 0.46, 0.49, 1.0))
    model.material("inner_light", rgba=(0.67, 0.69, 0.72, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_support_shape(), "support_frame"),
        material="support_steel",
        name="support_frame",
    )

    outer = model.part("outer_section")
    outer.visual(
        mesh_from_cadquery(_outer_section_shape(), "outer_section_mesh"),
        material="outer_paint",
        name="outer_section",
    )

    middle = model.part("middle_section")
    middle.visual(
        mesh_from_cadquery(_middle_section_shape(), "middle_section_mesh"),
        material="middle_paint",
        name="middle_section",
    )

    inner_1 = model.part("inner_section_1")
    inner_1.visual(
        mesh_from_cadquery(_inner1_section_shape(), "inner_section_1_mesh"),
        material="inner_dark",
        name="inner_section_1",
    )

    inner_2 = model.part("inner_section_2")
    inner_2.visual(
        mesh_from_cadquery(_inner2_section_shape(), "inner_section_2_mesh"),
        material="inner_light",
        name="inner_section_2",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=support,
        child=outer,
        origin=Origin(xyz=SUPPORT_OUTER_ORIGIN),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(
            xyz=(
                OUTER_TO_MIDDLE_HOME,
                0.0,
                OUTER_WALL,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner_1",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner_1,
        origin=Origin(
            xyz=(
                MIDDLE_TO_INNER1_HOME,
                0.0,
                MIDDLE_WALL,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1000.0,
            velocity=0.40,
            lower=0.0,
            upper=MIDDLE_TO_INNER1_TRAVEL,
        ),
    )
    model.articulation(
        "inner_1_to_inner_2",
        ArticulationType.PRISMATIC,
        parent=inner_1,
        child=inner_2,
        origin=Origin(
            xyz=(
                INNER1_TO_INNER2_HOME,
                0.0,
                INNER1_WALL,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=700.0,
            velocity=0.45,
            lower=0.0,
            upper=INNER1_TO_INNER2_TRAVEL,
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

    support = object_model.get_part("support")
    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_section")
    inner_1 = object_model.get_part("inner_section_1")
    inner_2 = object_model.get_part("inner_section_2")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner_1 = object_model.get_articulation("middle_to_inner_1")
    inner_1_to_inner_2 = object_model.get_articulation("inner_1_to_inner_2")

    ctx.expect_contact(
        support,
        outer,
        name="outer section is seated on the grounded support",
    )

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0,
        name="middle section stays centered inside outer section at rest",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.50,
        name="middle section retains insertion inside outer section at rest",
    )
    ctx.expect_within(
        inner_1,
        middle,
        axes="yz",
        margin=0.0,
        name="first inner section stays centered inside middle section at rest",
    )
    ctx.expect_overlap(
        inner_1,
        middle,
        axes="x",
        min_overlap=0.45,
        name="first inner section retains insertion inside middle section at rest",
    )
    ctx.expect_within(
        inner_2,
        inner_1,
        axes="yz",
        margin=0.0,
        name="second inner section stays centered inside first inner section at rest",
    )
    ctx.expect_overlap(
        inner_2,
        inner_1,
        axes="x",
        min_overlap=0.40,
        name="second inner section retains insertion inside first inner section at rest",
    )

    middle_rest = ctx.part_world_position(middle)
    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0,
            name="middle section stays centered inside outer section when extended",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.45,
            name="middle section still retains insertion when extended",
        )
        middle_extended = ctx.part_world_position(middle)
    ctx.check(
        "middle section extends along +X",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.20,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )

    inner_1_rest = ctx.part_world_position(inner_1)
    with ctx.pose({middle_to_inner_1: MIDDLE_TO_INNER1_TRAVEL}):
        ctx.expect_within(
            inner_1,
            middle,
            axes="yz",
            margin=0.0,
            name="first inner section stays centered inside middle section when extended",
        )
        ctx.expect_overlap(
            inner_1,
            middle,
            axes="x",
            min_overlap=0.40,
            name="first inner section still retains insertion when extended",
        )
        inner_1_extended = ctx.part_world_position(inner_1)
    ctx.check(
        "first inner section extends along +X",
        inner_1_rest is not None
        and inner_1_extended is not None
        and inner_1_extended[0] > inner_1_rest[0] + 0.15,
        details=f"rest={inner_1_rest}, extended={inner_1_extended}",
    )

    inner_2_rest = ctx.part_world_position(inner_2)
    with ctx.pose({inner_1_to_inner_2: INNER1_TO_INNER2_TRAVEL}):
        ctx.expect_within(
            inner_2,
            inner_1,
            axes="yz",
            margin=0.0,
            name="second inner section stays centered inside first inner section when extended",
        )
        ctx.expect_overlap(
            inner_2,
            inner_1,
            axes="x",
            min_overlap=0.35,
            name="second inner section still retains insertion when extended",
        )
        inner_2_extended = ctx.part_world_position(inner_2)
    ctx.check(
        "second inner section extends along +X",
        inner_2_rest is not None
        and inner_2_extended is not None
        and inner_2_extended[0] > inner_2_rest[0] + 0.12,
        details=f"rest={inner_2_rest}, extended={inner_2_extended}",
    )

    tip_rest = ctx.part_world_position(inner_2)
    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner_1: MIDDLE_TO_INNER1_TRAVEL,
            inner_1_to_inner_2: INNER1_TO_INNER2_TRAVEL,
        }
    ):
        tip_extended = ctx.part_world_position(inner_2)
    ctx.check(
        "combined telescoping significantly lengthens the boom",
        tip_rest is not None
        and tip_extended is not None
        and tip_extended[0] > tip_rest[0] + 1.50,
        details=f"rest={tip_rest}, extended={tip_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
