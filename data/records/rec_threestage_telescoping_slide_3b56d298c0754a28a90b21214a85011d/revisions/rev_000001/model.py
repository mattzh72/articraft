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


OUTER_LEN = 0.40
OUTER_WIDTH = 0.18
OUTER_BASE_T = 0.004
OUTER_SUPPORT_W = 0.020
OUTER_SUPPORT_CENTER_Y = 0.080
OUTER_SUPPORT_T = 0.008
OUTER_LIP_W = 0.012
OUTER_LIP_CENTER_Y = 0.053
OUTER_LIP_H = 0.004

MIDDLE_LEN = 0.29
MIDDLE_WIDTH = 0.118
MIDDLE_DECK_T = 0.003
MIDDLE_GUIDE_W = 0.032
MIDDLE_GUIDE_CENTER_Y = 0.0
MIDDLE_GUIDE_H = 0.005

INNER_LEN = 0.18
INNER_WIDTH = 0.078
INNER_DECK_T = 0.003

OUTER_TO_MIDDLE_Z = OUTER_BASE_T + OUTER_SUPPORT_T
MIDDLE_TO_INNER_Z = MIDDLE_DECK_T + MIDDLE_GUIDE_H


def _stage_box(
    length: float,
    width: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, False))
        .translate((x, y, z))
    )


def make_outer_stage() -> cq.Workplane:
    outer = _stage_box(OUTER_LEN, OUTER_WIDTH, OUTER_BASE_T)
    for y in (-OUTER_SUPPORT_CENTER_Y, OUTER_SUPPORT_CENTER_Y):
        outer = outer.union(
            _stage_box(
                OUTER_LEN,
                OUTER_SUPPORT_W,
                OUTER_SUPPORT_T,
                y=y,
                z=OUTER_BASE_T,
            )
        )
    for y in (-OUTER_LIP_CENTER_Y, OUTER_LIP_CENTER_Y):
        outer = outer.union(
            _stage_box(
                OUTER_LEN,
                OUTER_LIP_W,
                OUTER_LIP_H,
                y=y,
                z=OUTER_BASE_T + OUTER_SUPPORT_T - OUTER_LIP_H,
            )
        )
    outer = outer.union(
        _stage_box(
            0.012,
            0.100,
            0.006,
            x=OUTER_LEN - 0.012,
            z=OUTER_BASE_T,
        )
    )
    return outer


def make_middle_stage() -> cq.Workplane:
    middle = _stage_box(MIDDLE_LEN, MIDDLE_WIDTH, MIDDLE_DECK_T)
    middle = middle.union(
        _stage_box(
            MIDDLE_LEN,
            MIDDLE_GUIDE_W,
            MIDDLE_GUIDE_H,
            y=MIDDLE_GUIDE_CENTER_Y,
            z=MIDDLE_DECK_T,
        )
    )
    middle = middle.union(
        _stage_box(
            0.012,
            0.064,
            0.004,
            x=MIDDLE_LEN - 0.012,
            z=MIDDLE_DECK_T,
        )
    )
    return middle


def make_inner_stage() -> cq.Workplane:
    inner = _stage_box(INNER_LEN, INNER_WIDTH, INNER_DECK_T)
    inner = inner.union(
        _stage_box(
            0.034,
            0.040,
            0.002,
            x=INNER_LEN - 0.034,
            z=INNER_DECK_T,
        )
    )
    inner = inner.union(
        _stage_box(
            0.036,
            0.048,
            0.002,
            x=INNER_LEN - 0.036,
            z=INNER_DECK_T,
        )
    )
    return inner


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_slide_module")

    outer_mat = model.material("outer_stage_finish", color=(0.25, 0.27, 0.30))
    middle_mat = model.material("middle_stage_finish", color=(0.60, 0.64, 0.69))
    inner_mat = model.material("inner_stage_finish", color=(0.79, 0.81, 0.84))

    outer = model.part("outer_stage")
    outer.visual(
        mesh_from_cadquery(make_outer_stage(), "outer_stage"),
        origin=Origin(),
        material=outer_mat,
        name="outer_stage_body",
    )

    middle = model.part("middle_stage")
    middle.visual(
        mesh_from_cadquery(make_middle_stage(), "middle_stage"),
        origin=Origin(),
        material=middle_mat,
        name="middle_stage_body",
    )

    inner = model.part("inner_stage")
    inner.visual(
        mesh_from_cadquery(make_inner_stage(), "inner_stage"),
        origin=Origin(),
        material=inner_mat,
        name="inner_stage_body",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.40,
            lower=0.0,
            upper=0.18,
        ),
    )

    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TO_INNER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.40,
            lower=0.0,
            upper=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_stage")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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

    ctx.check(
        "slide joints are prismatic on +X",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0)
        and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        details="Expected two serial prismatic joints aligned on the shared +X guide axis.",
    )

    ctx.expect_contact(middle, outer, name="middle stage is supported by outer stage")
    ctx.expect_contact(inner, middle, name="inner stage is supported by middle stage")
    ctx.expect_within(
        middle,
        outer,
        axes="xy",
        margin=0.0,
        name="middle stage nests within outer stage when retracted",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="xy",
        margin=0.0,
        name="inner stage nests within middle stage when retracted",
    )

    with ctx.pose({outer_to_middle: 0.14}):
        ctx.expect_contact(
            middle,
            outer,
            name="middle stage remains guided in representative extension",
        )
        ctx.expect_origin_gap(
            middle,
            outer,
            axis="x",
            min_gap=0.139,
            max_gap=0.141,
            name="outer-to-middle extension advances along +X",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="y",
            margin=0.0,
            name="middle stage stays laterally captured by outer stage",
        )

    with ctx.pose({outer_to_middle: 0.14, middle_to_inner: 0.09}):
        ctx.expect_contact(
            inner,
            middle,
            name="inner stage remains guided in representative extension",
        )
        ctx.expect_origin_gap(
            inner,
            middle,
            axis="x",
            min_gap=0.089,
            max_gap=0.091,
            name="middle-to-inner extension advances along +X",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="y",
            margin=0.0,
            name="inner stage stays laterally captured by middle stage",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
