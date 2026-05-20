from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BODY_BASE_X = 0.26
BODY_BASE_Y = 0.19
BODY_BASE_H = 0.028
BODY_HOUSING_X = 0.18
BODY_HOUSING_Y = 0.13
BODY_HOUSING_H = 0.052
BODY_COLLAR_R = 0.052
BODY_COLLAR_H = 0.016

YAW_RING_R = 0.058
YAW_RING_H = 0.018
YAW_SADDLE_X = 0.072
YAW_SADDLE_Y = 0.058
YAW_SADDLE_H = 0.020
YAW_CHEEK_X = 0.080
YAW_CHEEK_T = 0.012
YAW_CHEEK_H = 0.070
YAW_CHEEK_Y = 0.038
YAW_CHEEK_Z = 0.038
YAW_REAR_BRIDGE_X = 0.018
YAW_REAR_BRIDGE_Y = 0.080
YAW_REAR_BRIDGE_H = 0.028
YAW_REAR_BRIDGE_CENTER_X = -0.027
PITCH_AXIS_Z = 0.077
TRUNNION_BORE_R = 0.0056

CRADLE_BACK_X = 0.012
CRADLE_BACK_Y = 0.034
CRADLE_BACK_Z = 0.032
CRADLE_BACK_CENTER_X = 0.006
CRADLE_RAIL_X = 0.044
CRADLE_RAIL_Y = 0.010
CRADLE_RAIL_Z = 0.008
CRADLE_RAIL_CENTER_X = 0.028
CRADLE_RAIL_OFFSET_Z = 0.018
CRADLE_FRONT_X = 0.008
CRADLE_FRONT_Y = 0.034
CRADLE_FRONT_Z = 0.036
CRADLE_FRONT_CENTER_X = 0.050
TRUNNION_SHAFT_R = 0.0046
TRUNNION_HALF_LENGTH = 0.048
TRUNNION_COLLAR_R = 0.008
TRUNNION_COLLAR_T = 0.002
TRUNNION_OUTER_FACE_Y = YAW_CHEEK_Y + (YAW_CHEEK_T * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_yaw_pitch_head")

    body_color = model.material("body_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    stage_color = model.material("stage_gray", rgba=(0.18, 0.19, 0.22, 1.0))
    cradle_color = model.material("cradle_gray", rgba=(0.42, 0.44, 0.47, 1.0))

    def union_all(*solids: cq.Workplane) -> cq.Workplane:
        result = solids[0]
        for solid in solids[1:]:
            result = result.union(solid)
        return result

    body_shape = union_all(
        cq.Workplane("XY")
        .box(BODY_BASE_X, BODY_BASE_Y, BODY_BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012),
        cq.Workplane("XY")
        .box(
            BODY_HOUSING_X,
            BODY_HOUSING_Y,
            BODY_HOUSING_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_BASE_H))
        .edges("|Z")
        .fillet(0.010),
        cq.Workplane("XY")
        .circle(BODY_COLLAR_R)
        .extrude(BODY_COLLAR_H)
        .translate((0.0, 0.0, BODY_BASE_H + BODY_HOUSING_H)),
    )

    yaw_stage_shape = union_all(
        cq.Workplane("XY").circle(YAW_RING_R).extrude(YAW_RING_H),
        cq.Workplane("XY")
        .box(
            YAW_SADDLE_X,
            YAW_SADDLE_Y,
            YAW_SADDLE_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, YAW_RING_H)),
        cq.Workplane("XY")
        .box(
            YAW_CHEEK_X,
            YAW_CHEEK_T,
            YAW_CHEEK_H,
            centered=(True, True, False),
        )
        .translate((0.0, YAW_CHEEK_Y, YAW_CHEEK_Z))
        .edges("|Z")
        .fillet(0.004),
        cq.Workplane("XY")
        .box(
            YAW_CHEEK_X,
            YAW_CHEEK_T,
            YAW_CHEEK_H,
            centered=(True, True, False),
        )
        .translate((0.0, -YAW_CHEEK_Y, YAW_CHEEK_Z))
        .edges("|Z")
        .fillet(0.004),
        cq.Workplane("XY")
        .box(
            YAW_REAR_BRIDGE_X,
            YAW_REAR_BRIDGE_Y,
            YAW_REAR_BRIDGE_H,
            centered=(True, True, False),
        )
        .translate((YAW_REAR_BRIDGE_CENTER_X, 0.0, 0.038)),
    ).cut(
        cq.Workplane("XZ")
        .center(0.0, PITCH_AXIS_Z)
        .circle(TRUNNION_BORE_R)
        .extrude(0.090, both=True)
    )

    cradle_back = (
        cq.Workplane("XY")
        .box(
            CRADLE_BACK_X,
            CRADLE_BACK_Y,
            CRADLE_BACK_Z,
            centered=(True, True, True),
        )
        .translate((CRADLE_BACK_CENTER_X, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )
    cradle_top_rail = (
        cq.Workplane("XY")
        .box(
            CRADLE_RAIL_X,
            CRADLE_RAIL_Y,
            CRADLE_RAIL_Z,
            centered=(True, True, True),
        )
        .translate((CRADLE_RAIL_CENTER_X, 0.0, CRADLE_RAIL_OFFSET_Z))
    )
    cradle_bottom_rail = (
        cq.Workplane("XY")
        .box(
            CRADLE_RAIL_X,
            CRADLE_RAIL_Y,
            CRADLE_RAIL_Z,
            centered=(True, True, True),
        )
        .translate((CRADLE_RAIL_CENTER_X, 0.0, -CRADLE_RAIL_OFFSET_Z))
    )
    cradle_front_bar = (
        cq.Workplane("XY")
        .box(
            CRADLE_FRONT_X,
            CRADLE_FRONT_Y,
            CRADLE_FRONT_Z,
            centered=(True, True, True),
        )
        .translate((CRADLE_FRONT_CENTER_X, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.003)
    )
    trunnion_shaft = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(TRUNNION_SHAFT_R)
        .extrude(TRUNNION_HALF_LENGTH, both=True)
    )
    cradle_shape = union_all(
        cradle_back,
        cradle_top_rail,
        cradle_bottom_rail,
        cradle_front_bar,
        trunnion_shaft,
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_shape, "body_shell"),
        material=body_color,
        name="body_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(yaw_stage_shape, "yaw_stage_shell"),
        material=stage_color,
        name="yaw_stage_shell",
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(cradle_shape, "cradle_shell"),
        material=cradle_color,
        name="cradle_shell",
    )
    cradle.visual(
        Cylinder(radius=TRUNNION_COLLAR_R, length=TRUNNION_COLLAR_T),
        origin=Origin(
            xyz=(0.0, TRUNNION_OUTER_FACE_Y + (TRUNNION_COLLAR_T * 0.5), 0.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material=cradle_color,
        name="right_thrust_collar",
    )
    cradle.visual(
        Cylinder(radius=TRUNNION_COLLAR_R, length=TRUNNION_COLLAR_T),
        origin=Origin(
            xyz=(0.0, -(TRUNNION_OUTER_FACE_Y + (TRUNNION_COLLAR_T * 0.5)), 0.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material=cradle_color,
        name="left_thrust_collar",
    )

    model.articulation(
        "body_to_yaw_stage",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yaw_stage,
        origin=Origin(
            xyz=(0.0, 0.0, BODY_BASE_H + BODY_HOUSING_H + BODY_COLLAR_H)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-2.5,
            upper=2.5,
        ),
    )

    model.articulation(
        "yaw_stage_to_cradle",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-0.75,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    yaw_stage = object_model.get_part("yaw_stage")
    cradle = object_model.get_part("cradle")
    yaw_joint = object_model.get_articulation("body_to_yaw_stage")
    pitch_joint = object_model.get_articulation("yaw_stage_to_cradle")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

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
        "yaw_axis_vertical",
        tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        f"expected yaw axis (0, 0, 1), got {yaw_joint.axis}",
    )
    ctx.check(
        "pitch_axis_opens_upward",
        tuple(pitch_joint.axis) == (0.0, -1.0, 0.0),
        f"expected pitch axis (0, -1, 0), got {pitch_joint.axis}",
    )

    ctx.expect_contact(
        yaw_stage,
        body,
        contact_tol=5e-4,
        name="yaw_stage_seated_on_body",
    )
    ctx.expect_contact(
        cradle,
        yaw_stage,
        contact_tol=5e-4,
        name="cradle_trunnions_supported",
    )

    closed_cradle_aabb = ctx.part_world_aabb(cradle)
    if closed_cradle_aabb is None:
        ctx.fail("closed_cradle_bounds_present", "cradle AABB missing in rest pose")
    else:
        closed_center = aabb_center(closed_cradle_aabb)
        with ctx.pose({pitch_joint: 0.9}):
            pitched_aabb = ctx.part_world_aabb(cradle)
        if pitched_aabb is None:
            ctx.fail(
                "pitched_cradle_bounds_present",
                "cradle AABB missing in pitched pose",
            )
        else:
            pitched_center = aabb_center(pitched_aabb)
            ctx.check(
                "pitch_lifts_cradle_mass",
                pitched_center[2] > closed_center[2] + 0.008,
                (
                    "positive pitch should raise the forward cradle; "
                    f"closed center {closed_center}, pitched center {pitched_center}"
                ),
            )

        with ctx.pose({yaw_joint: 1.0}):
            yawed_aabb = ctx.part_world_aabb(cradle)
        if yawed_aabb is None:
            ctx.fail(
                "yawed_cradle_bounds_present",
                "cradle AABB missing in yawed pose",
            )
        else:
            yawed_center = aabb_center(yawed_aabb)
            ctx.check(
                "yaw_swings_upper_head",
                abs(yawed_center[1]) > 0.012 and yawed_center[0] < closed_center[0] - 0.004,
                (
                    "positive yaw should sweep the upper head around the vertical axis; "
                    f"closed center {closed_center}, yawed center {yawed_center}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
