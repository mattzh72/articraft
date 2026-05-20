from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_FOOT_RADIUS = 0.170
BASE_BODY_RADIUS = 0.145
BASE_TOP_RADIUS = 0.118
BASE_FOOT_HEIGHT = 0.020
BASE_BODY_HEIGHT = 0.085
BASE_TOP_HEIGHT = 0.015
BASE_HEIGHT = BASE_FOOT_HEIGHT + BASE_BODY_HEIGHT + BASE_TOP_HEIGHT

STAGE_RADIUS = 0.145
STAGE_THICKNESS = 0.022
ARM_CENTER_X = 0.102
ARM_THICKNESS = 0.022
ARM_DEPTH = 0.120
ARM_HEIGHT = 0.146
TRUNNION_AXIS_Z = 0.124
TRUNNION_HOLE_RADIUS = 0.0115
REAR_BRIDGE_Y = 0.062
REAR_BRIDGE_THICKNESS = 0.014
REAR_BRIDGE_BOTTOM_Z = 0.078
REAR_BRIDGE_HEIGHT = 0.050

TABLE_WIDTH = 0.150
TABLE_DEPTH = 0.100
TABLE_THICKNESS = 0.016
TABLE_BOTTOM_Z = 0.028
TABLE_SLOT_WIDTH = 0.016
TABLE_SLOT_DEPTH = 0.004

TRUNNION_SHAFT_RADIUS = 0.010
TRUNNION_HALF_SPAN = ARM_CENTER_X + ARM_THICKNESS / 2.0
TRUNNION_COLLAR_RADIUS = 0.0205
TRUNNION_COLLAR_THICKNESS = 0.006

TABLE_HUB_WIDTH = 0.062
TABLE_HUB_DEPTH = 0.060
TABLE_HUB_BOTTOM_Z = -0.010
TABLE_HUB_HEIGHT = TABLE_BOTTOM_Z - TABLE_HUB_BOTTOM_Z


def _make_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_FOOT_RADIUS).extrude(BASE_FOOT_HEIGHT)
    base = base.union(
        cq.Workplane("XY")
        .workplane(offset=BASE_FOOT_HEIGHT)
        .circle(BASE_BODY_RADIUS)
        .extrude(BASE_BODY_HEIGHT)
    )
    base = base.union(
        cq.Workplane("XY")
        .workplane(offset=BASE_FOOT_HEIGHT + BASE_BODY_HEIGHT)
        .circle(BASE_TOP_RADIUS)
        .extrude(BASE_TOP_HEIGHT)
    )

    bearing_pocket = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .circle(0.070)
        .extrude(0.006, both=True)
    )
    pivot_clearance = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT - 0.010)
        .circle(0.028)
        .extrude(0.020)
    )
    return base.cut(bearing_pocket).cut(pivot_clearance)


def _make_lower_stage_shape() -> cq.Workplane:
    stage = (
        cq.Workplane("XY")
        .circle(STAGE_RADIUS)
        .extrude(STAGE_THICKNESS)
        .cut(
            cq.Workplane("XY")
            .workplane(offset=STAGE_THICKNESS)
            .circle(0.074)
            .extrude(0.005, both=True)
        )
    )

    yoke = stage
    for sign in (-1.0, 1.0):
        cheek = (
            cq.Workplane("XY")
            .box(
                ARM_THICKNESS,
                ARM_DEPTH,
                ARM_HEIGHT,
                centered=(True, True, False),
            )
            .translate((sign * ARM_CENTER_X, 0.0, STAGE_THICKNESS))
        )
        trunnion_bore = (
            cq.Workplane("YZ")
            .center(0.0, TRUNNION_AXIS_Z)
            .circle(TRUNNION_HOLE_RADIUS)
            .extrude(ARM_THICKNESS / 2.0, both=True)
            .translate((sign * ARM_CENTER_X, 0.0, 0.0))
        )
        cheek = cheek.cut(trunnion_bore)
        yoke = yoke.union(cheek)

    rear_bridge = (
        cq.Workplane("XY")
        .box(
            2.0 * ARM_CENTER_X + ARM_THICKNESS,
            REAR_BRIDGE_THICKNESS,
            REAR_BRIDGE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, REAR_BRIDGE_Y, REAR_BRIDGE_BOTTOM_Z))
    )
    return yoke.union(rear_bridge)


def _make_upper_table_shape() -> cq.Workplane:
    table = (
        cq.Workplane("XY")
        .box(
            TABLE_WIDTH,
            TABLE_DEPTH,
            TABLE_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, TABLE_BOTTOM_Z))
    )

    for slot_x in (-0.045, 0.0, 0.045):
        table = table.cut(
            cq.Workplane("XY")
            .box(
                TABLE_SLOT_WIDTH,
                TABLE_DEPTH - 0.028,
                TABLE_SLOT_DEPTH,
                centered=(True, True, False),
            )
            .translate(
                (
                    slot_x,
                    0.0,
                    TABLE_BOTTOM_Z + TABLE_THICKNESS - TABLE_SLOT_DEPTH,
                )
            )
        )

    hub = (
        cq.Workplane("XY")
        .box(
            TABLE_HUB_WIDTH,
            TABLE_HUB_DEPTH,
            TABLE_HUB_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, TABLE_HUB_BOTTOM_Z))
    )

    shaft = (
        cq.Workplane("YZ")
        .circle(TRUNNION_SHAFT_RADIUS)
        .extrude(TRUNNION_HALF_SPAN, both=True)
    )

    upper = table.union(hub).union(shaft)
    collar_center_x = TRUNNION_HALF_SPAN + TRUNNION_COLLAR_THICKNESS / 2.0
    for sign in (-1.0, 1.0):
        collar = (
            cq.Workplane("YZ")
            .circle(TRUNNION_COLLAR_RADIUS)
            .extrude(TRUNNION_COLLAR_THICKNESS / 2.0, both=True)
            .translate((sign * collar_center_x, 0.0, 0.0))
        )
        upper = upper.union(collar)

    return upper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_trunnion_table_module")

    base_color = model.material("base_color", rgba=(0.21, 0.23, 0.25, 1.0))
    yoke_color = model.material("yoke_color", rgba=(0.32, 0.34, 0.37, 1.0))
    table_color = model.material("table_color", rgba=(0.73, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_shell"),
        material=base_color,
        name="base_shell",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_make_lower_stage_shape(), "lower_stage_shell"),
        material=yoke_color,
        name="lower_stage_shell",
    )

    upper_table = model.part("upper_table")
    upper_table.visual(
        mesh_from_cadquery(_make_upper_table_shape(), "upper_table_shell"),
        material=table_color,
        name="upper_table_shell",
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=-pi,
            upper=pi,
        ),
    )

    model.articulation(
        "lower_stage_to_upper_table",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=upper_table,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.0,
            lower=-0.55,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    upper_table = object_model.get_part("upper_table")
    yaw_joint = object_model.get_articulation("base_to_lower_stage")
    tilt_joint = object_model.get_articulation("lower_stage_to_upper_table")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        lower_stage,
        upper_table,
        reason=(
            "Captured trunnion journals are represented as a nested bearing interface "
            "between the fork support and the tilting table, so the revolute support "
            "at the trunnion axis is intentionally allowed."
        ),
    )

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
        "yaw joint uses vertical axis",
        tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "tilt joint uses trunnion axis",
        tuple(tilt_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt_joint.axis}",
    )
    ctx.check(
        "yaw joint covers full rotary sweep",
        yaw_joint.motion_limits is not None
        and yaw_joint.motion_limits.lower is not None
        and yaw_joint.motion_limits.upper is not None
        and yaw_joint.motion_limits.lower <= -3.0
        and yaw_joint.motion_limits.upper >= 3.0,
        details=f"limits={yaw_joint.motion_limits}",
    )
    ctx.check(
        "tilt joint has asymmetric usable range",
        tilt_joint.motion_limits is not None
        and tilt_joint.motion_limits.lower is not None
        and tilt_joint.motion_limits.upper is not None
        and tilt_joint.motion_limits.lower < 0.0
        and tilt_joint.motion_limits.upper > 1.0,
        details=f"limits={tilt_joint.motion_limits}",
    )

    ctx.expect_origin_distance(
        lower_stage,
        base,
        axes="xy",
        max_dist=0.001,
        name="lower stage centered over base",
    )
    ctx.expect_origin_gap(
        lower_stage,
        base,
        axis="z",
        min_gap=BASE_HEIGHT - 0.001,
        max_gap=BASE_HEIGHT + 0.001,
        name="yaw axis sits at base top plane",
    )
    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="stage bearing seats cleanly on base",
    )
    ctx.expect_overlap(
        lower_stage,
        base,
        axes="xy",
        min_overlap=0.220,
        name="round stage covers base bearing footprint",
    )

    ctx.expect_origin_distance(
        upper_table,
        lower_stage,
        axes="xy",
        max_dist=0.001,
        name="table trunnion axis stays centered in yoke",
    )
    ctx.expect_origin_gap(
        upper_table,
        lower_stage,
        axis="z",
        min_gap=TRUNNION_AXIS_Z - 0.001,
        max_gap=TRUNNION_AXIS_Z + 0.001,
        name="table axis is carried at yoke height",
    )
    ctx.expect_contact(
        upper_table,
        lower_stage,
        contact_tol=0.0005,
        name="table trunnions bear in fork supports",
    )

    with ctx.pose({tilt_joint: 0.90}):
        ctx.expect_contact(
            upper_table,
            lower_stage,
            contact_tol=0.0005,
            name="tilted table stays supported on trunnion axis",
        )
        ctx.expect_gap(
            upper_table,
            base,
            axis="z",
            min_gap=0.080,
            name="tilted table clears fixed base housing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
