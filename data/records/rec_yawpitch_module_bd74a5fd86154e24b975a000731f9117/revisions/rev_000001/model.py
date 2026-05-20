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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_L = 0.240
FOOT_W = 0.180
FOOT_H = 0.018
PEDESTAL_L = 0.120
PEDESTAL_W = 0.100
PEDESTAL_H = 0.086
SEAT_R = 0.055
SEAT_H = 0.018
YAW_Z = FOOT_H + PEDESTAL_H + SEAT_H

STAGE_R = 0.064
STAGE_H = 0.028
TURRET_L = 0.086
TURRET_W = 0.082
TURRET_H = 0.030
REAR_WEB_L = 0.018
REAR_WEB_W = 0.096
REAR_WEB_H = 0.072
REAR_WEB_X = -0.024
YOKE_ARM_L = 0.034
YOKE_ARM_T = 0.010
YOKE_ARM_H = 0.076
YOKE_ARM_X = -0.008
YOKE_ARM_Y = 0.0525
YOKE_ARM_BOTTOM_Z = 0.056
TOP_BRIDGE_L = 0.018
TOP_BRIDGE_W = 0.105
TOP_BRIDGE_H = 0.012
TOP_BRIDGE_X = -0.020
TOP_BRIDGE_BOTTOM_Z = 0.120

PITCH_X = 0.0
PITCH_Z = 0.108
TRUNNION_R = 0.009
TRUNNION_LEN = 2.0 * (YOKE_ARM_Y - YOKE_ARM_T / 2.0)

NECK_L = 0.038
NECK_W = 0.024
NECK_H = 0.028
NECK_X = 0.020
HEAD_BODY_L = 0.060
HEAD_BODY_W = 0.054
HEAD_BODY_H = 0.064
HEAD_BODY_X = 0.062
BEZEL_L = 0.018
BEZEL_W = 0.066
BEZEL_H = 0.072
BEZEL_X = 0.101
OUTPUT_FACE_T = 0.004
OUTPUT_FACE_SIZE = 0.090
OUTPUT_FACE_X = 0.112


def _make_base_support_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(
        FOOT_L,
        FOOT_W,
        FOOT_H,
        centered=(True, True, False),
    )
    pedestal = (
        cq.Workplane("XY")
        .box(
            PEDESTAL_L,
            PEDESTAL_W,
            PEDESTAL_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, FOOT_H))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.086, 0.082, 0.020, centered=(True, True, False))
        .translate((0.0, 0.0, FOOT_H + PEDESTAL_H - 0.010))
    )
    seat = (
        cq.Workplane("XY")
        .circle(SEAT_R)
        .extrude(SEAT_H)
        .translate((0.0, 0.0, FOOT_H + PEDESTAL_H))
    )
    return foot.union(pedestal).union(neck).union(seat)


def _y_cylinder(
    radius: float,
    length: float,
    center_xyz: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center_xyz)
    )


def _make_yaw_stage_shape() -> cq.Workplane:
    disc = cq.Workplane("XY").circle(STAGE_R).extrude(STAGE_H)
    turret = (
        cq.Workplane("XY")
        .box(TURRET_L, TURRET_W, TURRET_H, centered=(True, True, False))
        .translate((0.0, 0.0, STAGE_H))
    )
    rear_web = (
        cq.Workplane("XY")
        .box(REAR_WEB_L, REAR_WEB_W, REAR_WEB_H, centered=(True, True, False))
        .translate((REAR_WEB_X, 0.0, YOKE_ARM_BOTTOM_Z))
    )
    right_arm = cq.Workplane("XY").box(
        YOKE_ARM_L, YOKE_ARM_T, YOKE_ARM_H, centered=(True, True, False)
    ).translate(
        (YOKE_ARM_X, YOKE_ARM_Y, YOKE_ARM_BOTTOM_Z)
    )
    left_arm = cq.Workplane("XY").box(
        YOKE_ARM_L, YOKE_ARM_T, YOKE_ARM_H, centered=(True, True, False)
    ).translate(
        (YOKE_ARM_X, -YOKE_ARM_Y, YOKE_ARM_BOTTOM_Z)
    )
    top_bridge = cq.Workplane("XY").box(
        TOP_BRIDGE_L, TOP_BRIDGE_W, TOP_BRIDGE_H, centered=(True, True, False)
    ).translate(
        (TOP_BRIDGE_X, 0.0, TOP_BRIDGE_BOTTOM_Z)
    )
    return disc.union(turret).union(rear_web).union(right_arm).union(left_arm).union(top_bridge)


def _make_pitch_head_shape() -> cq.Workplane:
    trunnion_shaft = _y_cylinder(TRUNNION_R, TRUNNION_LEN, (0.0, 0.0, 0.0))
    neck = cq.Workplane("XY").box(NECK_L, NECK_W, NECK_H).translate((NECK_X, 0.0, 0.0))
    main_body = cq.Workplane("XY").box(HEAD_BODY_L, HEAD_BODY_W, HEAD_BODY_H).translate(
        (HEAD_BODY_X, 0.0, 0.0)
    )
    bezel = cq.Workplane("XY").box(BEZEL_L, BEZEL_W, BEZEL_H).translate(
        (BEZEL_X, 0.0, 0.0)
    )
    return trunnion_shaft.union(neck).union(main_body).union(bezel)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_head")

    dark_base = model.material("dark_base", rgba=(0.19, 0.20, 0.22, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.13, 0.14, 1.0))
    machined_face = model.material("machined_face", rgba=(0.76, 0.78, 0.80, 1.0))

    base_support = model.part("base_support")
    base_support.visual(
        mesh_from_cadquery(_make_base_support_shape(), "base_support_shell"),
        material=dark_base,
        name="base_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=STAGE_R, length=STAGE_H),
        origin=Origin(xyz=(0.0, 0.0, STAGE_H / 2.0)),
        material=satin_black,
        name="yaw_disc",
    )
    yaw_stage.visual(
        Box((TURRET_L, TURRET_W, TURRET_H)),
        origin=Origin(xyz=(0.0, 0.0, STAGE_H + TURRET_H / 2.0)),
        material=satin_black,
        name="yaw_turret",
    )
    yaw_stage.visual(
        Box((REAR_WEB_L, REAR_WEB_W, REAR_WEB_H)),
        origin=Origin(
            xyz=(REAR_WEB_X, 0.0, YOKE_ARM_BOTTOM_Z + REAR_WEB_H / 2.0)
        ),
        material=satin_black,
        name="rear_web",
    )
    yaw_stage.visual(
        Box((YOKE_ARM_L, YOKE_ARM_T, YOKE_ARM_H)),
        origin=Origin(
            xyz=(YOKE_ARM_X, YOKE_ARM_Y, YOKE_ARM_BOTTOM_Z + YOKE_ARM_H / 2.0)
        ),
        material=satin_black,
        name="right_yoke_arm",
    )
    yaw_stage.visual(
        Box((YOKE_ARM_L, YOKE_ARM_T, YOKE_ARM_H)),
        origin=Origin(
            xyz=(YOKE_ARM_X, -YOKE_ARM_Y, YOKE_ARM_BOTTOM_Z + YOKE_ARM_H / 2.0)
        ),
        material=satin_black,
        name="left_yoke_arm",
    )
    yaw_stage.visual(
        Box((TOP_BRIDGE_L, TOP_BRIDGE_W, TOP_BRIDGE_H)),
        origin=Origin(
            xyz=(TOP_BRIDGE_X, 0.0, TOP_BRIDGE_BOTTOM_Z + TOP_BRIDGE_H / 2.0)
        ),
        material=satin_black,
        name="top_bridge",
    )

    pitch_head = model.part("pitch_head")
    pitch_head.visual(
        Cylinder(radius=TRUNNION_R, length=TRUNNION_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_base,
        name="trunnion_shaft",
    )
    pitch_head.visual(
        Box((NECK_L, NECK_W, NECK_H)),
        origin=Origin(xyz=(NECK_X, 0.0, 0.0)),
        material=dark_base,
        name="head_neck",
    )
    pitch_head.visual(
        Box((HEAD_BODY_L, HEAD_BODY_W, HEAD_BODY_H)),
        origin=Origin(xyz=(HEAD_BODY_X, 0.0, 0.0)),
        material=dark_base,
        name="head_shell",
    )
    pitch_head.visual(
        Box((BEZEL_L, BEZEL_W, BEZEL_H)),
        origin=Origin(xyz=(BEZEL_X, 0.0, 0.0)),
        material=dark_base,
        name="front_bezel",
    )
    pitch_head.visual(
        Box((OUTPUT_FACE_T, OUTPUT_FACE_SIZE, OUTPUT_FACE_SIZE)),
        origin=Origin(xyz=(OUTPUT_FACE_X, 0.0, 0.0)),
        material=machined_face,
        name="output_face",
    )

    model.articulation(
        "base_to_yaw_stage",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.5,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "yaw_stage_to_pitch_head",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_head,
        origin=Origin(xyz=(PITCH_X, 0.0, PITCH_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-0.85,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_head = object_model.get_part("pitch_head")
    yaw_joint = object_model.get_articulation("base_to_yaw_stage")
    pitch_joint = object_model.get_articulation("yaw_stage_to_pitch_head")
    output_face = pitch_head.get_visual("output_face")

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

    ctx.expect_gap(
        yaw_stage,
        base_support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="yaw stage seats on grounded support",
    )
    ctx.expect_overlap(
        yaw_stage,
        base_support,
        axes="xy",
        min_overlap=0.090,
        name="yaw stage stays centered over base support",
    )
    ctx.expect_contact(
        pitch_head,
        yaw_stage,
        contact_tol=0.0005,
        name="pitch head is supported by cradle trunnions",
    )
    ctx.expect_overlap(
        pitch_head,
        yaw_stage,
        axes="yz",
        min_overlap=0.030,
        name="head remains captured inside cradle envelope",
    )

    ctx.check(
        "yaw articulation axis is vertical",
        tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        f"got yaw axis {yaw_joint.axis}",
    )
    ctx.check(
        "pitch articulation axis is horizontal trunnion axis",
        tuple(round(v, 6) for v in pitch_joint.axis) == (0.0, -1.0, 0.0),
        f"got pitch axis {pitch_joint.axis}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0}):
        face_closed = ctx.part_element_world_aabb(pitch_head, elem=output_face.name)
    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.65}):
        face_pitched = ctx.part_element_world_aabb(pitch_head, elem=output_face.name)
    with ctx.pose({yaw_joint: pi / 2.0, pitch_joint: 0.0}):
        face_yawed = ctx.part_element_world_aabb(pitch_head, elem=output_face.name)

    if face_closed is None:
        ctx.fail("output face visual exists in world space", "could not resolve output face AABB")
    else:
        face_dy = face_closed[1][1] - face_closed[0][1]
        face_dz = face_closed[1][2] - face_closed[0][2]
        ctx.check(
            "output face is square",
            abs(face_dy - face_dz) <= 0.0015 and 0.086 <= face_dy <= 0.094,
            f"face extents were dy={face_dy:.4f}, dz={face_dz:.4f}",
        )

    if face_closed is None or face_pitched is None:
        ctx.fail(
            "positive pitch raises output face",
            "could not compare closed and pitched output-face poses",
        )
    else:
        closed_center = _aabb_center(face_closed)
        pitched_center = _aabb_center(face_pitched)
        ctx.check(
            "positive pitch raises output face",
            pitched_center[2] > closed_center[2] + 0.040,
            f"closed z={closed_center[2]:.4f}, pitched z={pitched_center[2]:.4f}",
        )

    if face_closed is None or face_yawed is None:
        ctx.fail(
            "positive yaw swings output face around base axis",
            "could not compare closed and yawed output-face poses",
        )
    else:
        closed_center = _aabb_center(face_closed)
        yawed_center = _aabb_center(face_yawed)
        ctx.check(
            "positive yaw swings output face around base axis",
            yawed_center[1] > closed_center[1] + 0.080,
            f"closed center={closed_center}, yawed center={yawed_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
