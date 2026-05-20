from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


FOOT_X = 0.240
FOOT_Y = 0.160
FOOT_Z = 0.032

PEDESTAL_BASE_X = 0.112
PEDESTAL_BASE_Y = 0.100
PEDESTAL_TOP_X = 0.076
PEDESTAL_TOP_Y = 0.086
PEDESTAL_H = 0.145
PEDESTAL_FOOT_OVERLAP = 0.003

CROSSHEAD_X = 0.126
CROSSHEAD_Y = 0.098
CROSSHEAD_Z = 0.026

CHEEK_T = 0.018
CHEEK_Y = 0.096
CHEEK_H = 0.074
CHEEK_SINK = 0.004
OUTER_SPAN_X = 0.118
CHEEK_CENTER_X = (OUTER_SPAN_X / 2.0) - (CHEEK_T / 2.0)
CHEEK_OUTER_FACE_X = OUTER_SPAN_X / 2.0

BEARING_BOSS_R = 0.028
BEARING_BOSS_LEN = 0.014
BORE_R = 0.0115

AXIS_Z = FOOT_Z + PEDESTAL_H + CROSSHEAD_Z + 0.038
CROSSHEAD_CENTER_Z = FOOT_Z + PEDESTAL_H + (CROSSHEAD_Z / 2.0) - 0.0015
CHEEK_CENTER_Z = (
    FOOT_Z
    + PEDESTAL_H
    + CROSSHEAD_Z
    + ((CHEEK_H + CHEEK_SINK) / 2.0)
    - (CHEEK_SINK / 2.0)
)

SHAFT_R = 0.0085
SHAFT_L = 0.152
HUB_R = 0.018
HUB_L = 0.054
FLANGE_R = 0.028
FLANGE_T = 0.012
PILOT_R = 0.011
PILOT_L = 0.006
RETAINER_R = 0.016
RETAINER_T = 0.006
SUPPORT_EXTENT_X = CHEEK_OUTER_FACE_X + BEARING_BOSS_LEN
RETAINER_CENTER_X = SUPPORT_EXTENT_X + (RETAINER_T / 2.0)

ROLL_LOWER = -2.4
ROLL_UPPER = 2.4


def _x_axis_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -(length / 2.0)))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )


def _base_tower_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_X, FOOT_Y, FOOT_Z)
        .edges("|Z")
        .fillet(0.016)
        .translate((0.0, 0.0, FOOT_Z / 2.0))
    )

    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=FOOT_Z - PEDESTAL_FOOT_OVERLAP)
        .rect(PEDESTAL_BASE_X, PEDESTAL_BASE_Y)
        .workplane(offset=PEDESTAL_H + PEDESTAL_FOOT_OVERLAP)
        .rect(PEDESTAL_TOP_X, PEDESTAL_TOP_Y)
        .loft(combine=False)
    )

    crosshead = (
        cq.Workplane("XY")
        .box(CROSSHEAD_X, CROSSHEAD_Y, CROSSHEAD_Z)
        .translate((0.0, 0.0, CROSSHEAD_CENTER_Z))
    )

    return foot.union(pedestal).union(crosshead)


def _cheek_shape(sign: float) -> cq.Workplane:
    cheek = (
        cq.Workplane("XY")
        .box(CHEEK_T, CHEEK_Y, CHEEK_H + CHEEK_SINK)
        .translate((sign * CHEEK_CENTER_X, 0.0, CHEEK_CENTER_Z))
    )

    boss = _x_axis_cylinder(BEARING_BOSS_R, BEARING_BOSS_LEN).translate(
        (sign * (CHEEK_OUTER_FACE_X + (BEARING_BOSS_LEN / 2.0)), 0.0, AXIS_Z)
    )

    full_bore = _x_axis_cylinder(BORE_R, 0.240).translate((0.0, 0.0, AXIS_Z))

    return cheek.union(boss).cut(full_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_roll_axis_spindle")

    model.material("support_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("machined_aluminum", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("spindle_steel", rgba=(0.56, 0.58, 0.61, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_base_tower_shape(), "support_base_tower"),
        material="support_dark",
        name="base_tower",
    )
    support.visual(
        mesh_from_cadquery(_cheek_shape(-1.0), "support_left_cheek"),
        material="machined_aluminum",
        name="left_cheek",
    )
    support.visual(
        mesh_from_cadquery(_cheek_shape(1.0), "support_right_cheek"),
        material="machined_aluminum",
        name="right_cheek",
    )
    support.inertial = Inertial.from_geometry(
        Box((FOOT_X, FOOT_Y, AXIS_Z + 0.050)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.0, (AXIS_Z + 0.050) / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_L),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="spindle_steel",
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=HUB_R, length=HUB_L),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="spindle_steel",
        name="hub_sleeve",
    )
    spindle.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_T),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_aluminum",
        name="output_flange",
    )
    spindle.visual(
        Cylinder(radius=PILOT_R, length=PILOT_L),
        origin=Origin(
            xyz=((FLANGE_T / 2.0) + (PILOT_L / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="machined_aluminum",
        name="pilot_boss",
    )
    spindle.visual(
        Cylinder(radius=RETAINER_R, length=RETAINER_T),
        origin=Origin(
            xyz=(-RETAINER_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="spindle_steel",
        name="left_retainer",
    )
    spindle.visual(
        Cylinder(radius=RETAINER_R, length=RETAINER_T),
        origin=Origin(
            xyz=(RETAINER_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="spindle_steel",
        name="right_retainer",
    )
    spindle.inertial = Inertial.from_geometry(
        Box((SHAFT_L, FLANGE_R * 2.0, FLANGE_R * 2.0)),
        mass=1.2,
        origin=Origin(),
    )

    model.articulation(
        "support_to_spindle",
        ArticulationType.REVOLUTE,
        parent=support,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=8.0,
            lower=ROLL_LOWER,
            upper=ROLL_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    spindle = object_model.get_part("spindle")
    roll = object_model.get_articulation("support_to_spindle")
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
        "roll articulation uses spindle shaft axis",
        tuple(roll.axis) == (1.0, 0.0, 0.0),
        f"axis={roll.axis}",
    )
    ctx.check(
        "roll articulation spans both directions",
        roll.motion_limits is not None
        and roll.motion_limits.lower is not None
        and roll.motion_limits.upper is not None
        and roll.motion_limits.lower < 0.0
        and roll.motion_limits.upper > 0.0,
        f"limits={roll.motion_limits}",
    )
    ctx.expect_contact(
        spindle,
        support,
        elem_a="left_retainer",
        elem_b="left_cheek",
        name="left retainer is captured by left support cheek",
    )
    ctx.expect_contact(
        spindle,
        support,
        elem_a="right_retainer",
        elem_b="right_cheek",
        name="right retainer is captured by right support cheek",
    )
    ctx.expect_within(
        spindle,
        support,
        axes="yz",
        inner_elem="output_flange",
        margin=0.0,
        name="output flange stays inside support package footprint",
    )

    with ctx.pose({roll: 1.0}):
        ctx.expect_contact(
            spindle,
            support,
            elem_a="left_retainer",
            elem_b="left_cheek",
            name="left retainer stays seated when rolled",
        )
        ctx.expect_contact(
            spindle,
            support,
            elem_a="right_retainer",
            elem_b="right_cheek",
            name="right retainer stays seated when rolled",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
