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


# Model dimensions are authored in millimeters for CadQuery robustness, then
# exported with unit_scale=0.001 so the SDK receives meters.
BASE_LENGTH = 24.0
BASE_WIDTH = 40.0
BASE_HEIGHT = 18.0
BASE_X_CENTER = -27.0

CHEEK_LENGTH = 20.0
CHEEK_THICKNESS = 7.0
CHEEK_HEIGHT = 28.0
INNER_GAP = 10.0
CHEEK_CENTER_Y = INNER_GAP / 2.0 + CHEEK_THICKNESS / 2.0

PIN_RADIUS = 4.0
PIN_LENGTH = INNER_GAP + 2.0 * CHEEK_THICKNESS + 2.0
ROOT_BARREL_RADIUS = 7.0

TAB_THICKNESS = 8.0
TAB_HEIGHT = 28.0
TAB_LENGTH = 58.0
TAB_TIP_RADIUS = 7.0
TAB_LIGHTENING_RADIUS = 5.0
CENTER_BARREL_RADIUS = 6.5
CENTER_BARREL_LENGTH = TAB_THICKNESS
PIN_BORE_RADIUS = 4.2


def _fork_root_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT).translate(
        (BASE_X_CENTER, 0.0, 0.0)
    )

    left_cheek = cq.Workplane("XY").box(
        CHEEK_LENGTH,
        CHEEK_THICKNESS,
        CHEEK_HEIGHT,
    ).translate((-7.0, -CHEEK_CENTER_Y, 0.0))
    right_cheek = cq.Workplane("XY").box(
        CHEEK_LENGTH,
        CHEEK_THICKNESS,
        CHEEK_HEIGHT,
    ).translate((-7.0, CHEEK_CENTER_Y, 0.0))

    left_barrel = (
        cq.Workplane("XZ")
        .circle(ROOT_BARREL_RADIUS)
        .extrude(CHEEK_THICKNESS / 2.0, both=True)
        .translate((0.0, -CHEEK_CENTER_Y, 0.0))
    )
    right_barrel = (
        cq.Workplane("XZ")
        .circle(ROOT_BARREL_RADIUS)
        .extrude(CHEEK_THICKNESS / 2.0, both=True)
        .translate((0.0, CHEEK_CENTER_Y, 0.0))
    )
    root = (
        base.union(left_cheek)
        .union(right_cheek)
        .union(left_barrel)
        .union(right_barrel)
    )

    mounting_holes = (
        cq.Workplane("XY")
        .pushPoints([(-29.0, -10.0), (-29.0, 10.0)])
        .circle(3.0)
        .extrude(BASE_HEIGHT + 4.0)
        .translate((0.0, 0.0, -BASE_HEIGHT / 2.0 - 2.0))
    )

    return root.cut(mounting_holes)


def _center_tab_shape() -> cq.Workplane:
    tab_body_length = TAB_LENGTH - TAB_TIP_RADIUS
    plate = cq.Workplane("XZ").box(
        tab_body_length,
        TAB_HEIGHT,
        TAB_THICKNESS,
    ).translate((tab_body_length / 2.0, 0.0, 0.0))
    tip_round = (
        cq.Workplane("XZ")
        .center(tab_body_length, 0.0)
        .circle(TAB_TIP_RADIUS)
        .extrude(TAB_THICKNESS / 2.0, both=True)
    )
    barrel = (
        cq.Workplane("XZ")
        .circle(CENTER_BARREL_RADIUS)
        .extrude(CENTER_BARREL_LENGTH / 2.0, both=True)
    )
    pin_bore = (
        cq.Workplane("XZ")
        .circle(PIN_BORE_RADIUS)
        .extrude(CENTER_BARREL_LENGTH / 2.0 + 1.0, both=True)
    )
    distal_hole = (
        cq.Workplane("XZ")
        .center(TAB_LENGTH - 16.0, 0.0)
        .circle(TAB_LIGHTENING_RADIUS)
        .extrude(TAB_THICKNESS / 2.0 + 1.0, both=True)
    )

    return plate.union(tip_round).union(barrel).cut(pin_bore).cut(distal_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_center_tab_hinge")

    model.material("powder_coat", rgba=(0.24, 0.27, 0.31, 1.0))
    model.material("zinc_pin", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("tab_finish", rgba=(0.84, 0.47, 0.16, 1.0))

    fork_root = model.part("fork_root")
    fork_root.visual(
        mesh_from_cadquery(_fork_root_shape(), "fork_root", unit_scale=0.001),
        material="powder_coat",
        name="fork_assembly",
    )
    fork_root.visual(
        mesh_from_cadquery(
            cq.Workplane("XZ").circle(PIN_RADIUS).extrude(PIN_LENGTH / 2.0, both=True),
            "through_pin",
            unit_scale=0.001,
        ),
        material="zinc_pin",
        name="pin_highlight",
    )

    center_tab = model.part("center_tab")
    center_tab.visual(
        mesh_from_cadquery(_center_tab_shape(), "center_tab", unit_scale=0.001),
        material="tab_finish",
        name="tab_body",
    )

    model.articulation(
        "fork_to_tab",
        ArticulationType.REVOLUTE,
        parent=fork_root,
        child=center_tab,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
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
    fork_root = object_model.get_part("fork_root")
    center_tab = object_model.get_part("center_tab")
    hinge = object_model.get_articulation("fork_to_tab")

    ctx.check(
        "fork root part exists",
        fork_root is not None,
        details="Expected grounded fork root part.",
    )
    ctx.check(
        "center tab part exists",
        center_tab is not None,
        details="Expected captured center tab part.",
    )
    ctx.check(
        "hinge axis follows the through-pin",
        hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "hinge has compact positive opening range",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and 1.0 <= hinge.motion_limits.upper <= 1.6,
        details=f"limits={hinge.motion_limits}",
    )

    ctx.allow_isolated_part(
        center_tab,
        reason=(
            "The center tab is intentionally carried by the revolute through-pin "
            "with a small running clearance in the bore, so the static rest pose "
            "does not need cheek contact."
        ),
    )

    ctx.expect_within(
        center_tab,
        fork_root,
        axes="y",
        inner_elem="tab_body",
        outer_elem="fork_assembly",
        margin=0.001,
        name="center tab remains captured between fork cheeks at rest",
    )
    ctx.expect_overlap(
        center_tab,
        fork_root,
        axes="y",
        elem_a="tab_body",
        elem_b="fork_assembly",
        min_overlap=0.008,
        name="tab stays engaged across the supported pin width at rest",
    )

    closed_aabb = ctx.part_element_world_aabb(center_tab, elem="tab_body")
    open_limit = hinge.motion_limits.upper if hinge.motion_limits is not None else 1.2
    with ctx.pose({hinge: open_limit}):
        ctx.expect_within(
            center_tab,
            fork_root,
            axes="y",
            inner_elem="tab_body",
            outer_elem="fork_assembly",
            margin=0.001,
            name="center tab remains captured between fork cheeks when opened",
        )
        ctx.expect_overlap(
            center_tab,
            fork_root,
            axes="y",
            elem_a="tab_body",
            elem_b="fork_assembly",
            min_overlap=0.008,
            name="tab stays engaged across the supported pin width when opened",
        )
        open_aabb = ctx.part_element_world_aabb(center_tab, elem="tab_body")

    ctx.check(
        "positive hinge motion lifts the tab",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.02,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
