from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_THICKNESS = 0.012
FOOT_THICKNESS = 0.012
AXIS_Z = 0.058
PAD_FRONT_Y = 0.007
BOSS_RADIUS = 0.018

JOURNAL_RADIUS = 0.0095
JOURNAL_LENGTH = 0.011
JOURNAL_CENTER_Y = PAD_FRONT_Y + JOURNAL_LENGTH / 2.0

HEAD_RADIUS = 0.0118
HEAD_LENGTH = 0.003
HEAD_CENTER_Y = PAD_FRONT_Y + JOURNAL_LENGTH + HEAD_LENGTH / 2.0

TAB_THICKNESS = 0.008
TAB_WIDTH = 0.038
TAB_BLADE_LENGTH = 0.135
TAB_EYE_OUTER_RADIUS = 0.016
TAB_BORE_RADIUS = 0.01025


def _support_body_shape() -> cq.Workplane:
    side_profile = [
        (-0.047, 0.0),
        (0.038, 0.0),
        (0.038, FOOT_THICKNESS),
        (0.019, FOOT_THICKNESS),
        (0.004, 0.032),
        (0.004, 0.090),
        (-0.029, 0.090),
        (-0.029, FOOT_THICKNESS),
        (-0.047, FOOT_THICKNESS),
    ]

    body = (
        cq.Workplane("XZ")
        .polyline(side_profile)
        .close()
        .extrude(SUPPORT_THICKNESS / 2.0, both=True)
    )

    mount_holes = (
        cq.Workplane("XZ")
        .pushPoints([(-0.029, FOOT_THICKNESS / 2.0), (0.023, FOOT_THICKNESS / 2.0)])
        .circle(0.0042)
        .extrude(SUPPORT_THICKNESS * 2.5, both=True)
    )

    boss_pad = (
        cq.Workplane("XZ")
        .center(0.0, AXIS_Z)
        .circle(BOSS_RADIUS)
        .extrude(-PAD_FRONT_Y)
    )

    return body.cut(mount_holes).union(boss_pad)


def _tab_shape() -> cq.Workplane:
    eye = cq.Workplane("XZ").circle(TAB_EYE_OUTER_RADIUS).extrude(TAB_THICKNESS / 2.0, both=True)
    blade = (
        cq.Workplane("XZ")
        .center(TAB_BLADE_LENGTH / 2.0, 0.0)
        .rect(TAB_BLADE_LENGTH, TAB_WIDTH)
        .extrude(TAB_THICKNESS / 2.0, both=True)
    )
    bore = cq.Workplane("XZ").circle(TAB_BORE_RADIUS).extrude(TAB_THICKNESS + 0.01, both=True)
    return eye.union(blade).cut(bore)


def _elem_center_z(ctx: TestContext, part_name: str, elem_name: str) -> float | None:
    aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_tab_hinge")

    support_finish = model.material("support_finish", rgba=(0.25, 0.28, 0.31, 1.0))
    tab_finish = model.material("tab_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    pin_finish = model.material("pin_finish", rgba=(0.63, 0.65, 0.68, 1.0))

    support = model.part("support_cheek")
    support.visual(
        mesh_from_cadquery(_support_body_shape(), "support_body"),
        material=support_finish,
        name="support_body",
    )
    support.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=JOURNAL_LENGTH),
        origin=Origin(xyz=(0.0, JOURNAL_CENTER_Y, AXIS_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_finish,
        name="journal",
    )
    support.visual(
        Cylinder(radius=HEAD_RADIUS, length=HEAD_LENGTH),
        origin=Origin(xyz=(0.0, HEAD_CENTER_Y, AXIS_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_finish,
        name="retaining_head",
    )

    tab = model.part("moving_tab")
    tab.visual(
        mesh_from_cadquery(_tab_shape(), "tab_body"),
        material=tab_finish,
        name="tab_body",
    )

    model.articulation(
        "support_to_tab",
        ArticulationType.REVOLUTE,
        parent=support,
        child=tab,
        origin=Origin(xyz=(0.0, JOURNAL_CENTER_Y, AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=6.0, velocity=2.0),
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

    support = object_model.get_part("support_cheek")
    tab = object_model.get_part("moving_tab")
    hinge = object_model.get_articulation("support_to_tab")

    ctx.allow_isolated_part(
        tab,
        reason="The tab is intentionally carried on a plain-bearing trunnion fit with modeled running clearance rather than fused contact.",
    )

    ctx.check(
        "hinge uses the trunnion axis",
        hinge.axis == (0.0, -1.0, 0.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper >= 1.2,
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            tab,
            support,
            axes="xyz",
            elem_a="tab_body",
            elem_b="journal",
            min_overlap=0.007,
            name="tab eye stays centered on the journal at rest",
        )
        ctx.expect_gap(
            tab,
            support,
            axis="y",
            positive_elem="tab_body",
            negative_elem="support_body",
            min_gap=0.001,
            max_gap=0.0025,
            name="tab stands proud of the support cheek",
        )
        rest_center_z = _elem_center_z(ctx, "moving_tab", "tab_body")

    with ctx.pose({hinge: 0.95}):
        ctx.expect_overlap(
            tab,
            support,
            axes="xyz",
            elem_a="tab_body",
            elem_b="journal",
            min_overlap=0.007,
            name="tab remains carried by the journal when raised",
        )
        raised_center_z = _elem_center_z(ctx, "moving_tab", "tab_body")

    ctx.check(
        "positive hinge motion lifts the tab",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.03,
        details=f"rest_center_z={rest_center_z}, raised_center_z={raised_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
