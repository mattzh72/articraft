from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.18
BASE_DEPTH = 0.08
BASE_THICKNESS = 0.018

CHEEK_CENTER_X = 0.06
CHEEK_THICKNESS = 0.02
CHEEK_DEPTH = 0.065
CHEEK_BOTTOM_Z = -0.055
CHEEK_TOP_Z = 0.035
SHAFT_CENTER_HEIGHT = 0.055

MAIN_SHAFT_RADIUS = 0.014
MAIN_SHAFT_LENGTH = 0.10
JOURNAL_RADIUS = 0.008
JOURNAL_LENGTH = 0.021
JOURNAL_OVERLAP = 0.001
BEARING_HOLE_RADIUS = 0.0095
OUTER_BOSS_RADIUS = 0.018
OUTER_BOSS_LENGTH = 0.006


def _make_cheek_mesh(name: str, side: str):
    outer_sign = -1.0 if side == "left" else 1.0
    cheek_height = CHEEK_TOP_Z - CHEEK_BOTTOM_Z
    cheek_center_z = 0.5 * (CHEEK_TOP_Z + CHEEK_BOTTOM_Z)

    plate = (
        cq.Workplane("YZ")
        .center(0.0, cheek_center_z)
        .rect(CHEEK_DEPTH, cheek_height)
        .extrude(CHEEK_THICKNESS / 2.0, both=True)
    )

    boss_start_x = (
        -CHEEK_THICKNESS / 2.0 - OUTER_BOSS_LENGTH
        if outer_sign < 0.0
        else CHEEK_THICKNESS / 2.0
    )
    boss = (
        cq.Workplane("YZ")
        .circle(OUTER_BOSS_RADIUS)
        .extrude(OUTER_BOSS_LENGTH)
        .translate((boss_start_x, 0.0, 0.0))
    )

    hole = (
        cq.Workplane("YZ")
        .circle(BEARING_HOLE_RADIUS)
        .extrude(CHEEK_THICKNESS + 2.0 * OUTER_BOSS_LENGTH + 0.01, both=True)
    )

    cheek = (plate.union(boss)).cut(hole)
    cheek = cheek.edges("|X").fillet(0.003)
    return mesh_from_cadquery(cheek, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cheek_supported_rotary_shaft")

    model.material("painted_steel", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("shaft_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="painted_steel",
        name="base_block",
    )

    left_cheek = model.part("left_cheek")
    left_cheek.visual(
        _make_cheek_mesh("left_cheek_shell", side="left"),
        material="painted_steel",
        name="left_cheek_shell",
    )

    right_cheek = model.part("right_cheek")
    right_cheek.visual(
        _make_cheek_mesh("right_cheek_shell", side="right"),
        material="painted_steel",
        name="right_cheek_shell",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=MAIN_SHAFT_RADIUS, length=MAIN_SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material="shaft_steel",
        name="visible_shaft",
    )
    shaft.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=JOURNAL_LENGTH),
        origin=Origin(
            xyz=(
                -(
                    MAIN_SHAFT_LENGTH / 2.0
                    + JOURNAL_LENGTH / 2.0
                    - JOURNAL_OVERLAP / 2.0
                ),
                0.0,
                0.0,
            ),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material="shaft_steel",
        name="left_journal",
    )
    shaft.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=JOURNAL_LENGTH),
        origin=Origin(
            xyz=(
                MAIN_SHAFT_LENGTH / 2.0
                + JOURNAL_LENGTH / 2.0
                - JOURNAL_OVERLAP / 2.0,
                0.0,
                0.0,
            ),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material="shaft_steel",
        name="right_journal",
    )

    model.articulation(
        "base_to_left_cheek",
        ArticulationType.FIXED,
        parent=base,
        child=left_cheek,
        origin=Origin(xyz=(-CHEEK_CENTER_X, 0.0, BASE_THICKNESS + SHAFT_CENTER_HEIGHT)),
    )
    model.articulation(
        "base_to_right_cheek",
        ArticulationType.FIXED,
        parent=base,
        child=right_cheek,
        origin=Origin(xyz=(CHEEK_CENTER_X, 0.0, BASE_THICKNESS + SHAFT_CENTER_HEIGHT)),
    )
    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + SHAFT_CENTER_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
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
    base = object_model.get_part("base")
    left_cheek = object_model.get_part("left_cheek")
    right_cheek = object_model.get_part("right_cheek")
    shaft = object_model.get_part("shaft")
    spin = object_model.get_articulation("base_to_shaft")

    ctx.expect_gap(
        left_cheek,
        base,
        axis="z",
        positive_elem="left_cheek_shell",
        negative_elem="base_block",
        max_gap=0.001,
        max_penetration=1e-6,
        name="left cheek is seated on the base",
    )
    ctx.expect_gap(
        right_cheek,
        base,
        axis="z",
        positive_elem="right_cheek_shell",
        negative_elem="base_block",
        max_gap=0.001,
        max_penetration=1e-6,
        name="right cheek is seated on the base",
    )
    ctx.expect_origin_distance(
        shaft,
        left_cheek,
        axes="yz",
        max_dist=1e-6,
        name="shaft axis aligns with the left support cheek",
    )
    ctx.expect_origin_distance(
        shaft,
        right_cheek,
        axes="yz",
        max_dist=1e-6,
        name="shaft axis aligns with the right support cheek",
    )
    ctx.expect_gap(
        shaft,
        left_cheek,
        axis="x",
        positive_elem="visible_shaft",
        negative_elem="left_cheek_shell",
        max_gap=0.001,
        max_penetration=1e-6,
        name="left shaft end face sits at the inner left cheek plane",
    )
    ctx.expect_gap(
        right_cheek,
        shaft,
        axis="x",
        positive_elem="right_cheek_shell",
        negative_elem="visible_shaft",
        max_gap=0.001,
        max_penetration=1e-6,
        name="right shaft end face sits at the inner right cheek plane",
    )
    ctx.expect_overlap(
        shaft,
        left_cheek,
        axes="x",
        elem_a="left_journal",
        elem_b="left_cheek_shell",
        min_overlap=0.018,
        name="left journal remains inside the left support cheek",
    )
    ctx.expect_overlap(
        shaft,
        right_cheek,
        axes="x",
        elem_a="right_journal",
        elem_b="right_cheek_shell",
        min_overlap=0.018,
        name="right journal remains inside the right support cheek",
    )
    ctx.check(
        "shaft uses a continuous x-axis articulation",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (1.0, 0.0, 0.0)
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=(
            f"type={spin.articulation_type}, axis={spin.axis}, "
            f"limits={spin.motion_limits}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
