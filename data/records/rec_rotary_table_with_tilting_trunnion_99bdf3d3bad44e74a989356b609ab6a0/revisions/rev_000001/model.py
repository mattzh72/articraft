from __future__ import annotations

from math import pi

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_W = 0.18
BASE_D = 0.16
BASE_H = 0.06
BASE_PAD_R = 0.055
BASE_PAD_H = 0.006

STAGE_R = 0.07
STAGE_LOWER_H = 0.022
STAGE_UPPER_R = 0.062
STAGE_UPPER_H = 0.013
STAGE_H = STAGE_LOWER_H + STAGE_UPPER_H

FRAME_BASE_W = 0.11
FRAME_BASE_D = 0.095
FRAME_BASE_Y = -0.01
FRAME_BASE_T = 0.016
FRAME_CHEEK_X = 0.064
FRAME_CHEEK_T = 0.014
FRAME_CHEEK_D = 0.045
FRAME_CHEEK_Y = 0.022
FRAME_CHEEK_H = 0.095
FRAME_WINDOW_R = 0.029
FRAME_WEB_W = 0.12
FRAME_WEB_D = 0.01
FRAME_WEB_Y = -0.054
FRAME_WEB_H = 0.034
TRUNNION_Y = 0.022
TRUNNION_Z = 0.075
BEARING_BOSS_R = 0.024
BEARING_BOSS_L = 0.008
FRAME_INNER_FACE_X = FRAME_CHEEK_X - FRAME_CHEEK_T / 2.0
FRAME_OUTER_FACE_X = FRAME_CHEEK_X + FRAME_CHEEK_T / 2.0 + BEARING_BOSS_L
TRUNNION_HOLE_R = 0.013

FACEPLATE_R = 0.0475
FACEPLATE_T = 0.026
FACEPLATE_RING_R = 0.039
FACEPLATE_RING_INNER_R = 0.015
FACEPLATE_PCD_R = 0.028
FACEPLATE_PILOT_R = 0.011
FACEPLATE_PILOT_T = 0.004
BACK_HUB_R = 0.032
BACK_HUB_T = 0.018
TRUNNION_PIN_R = 0.012
TRUNNION_PIN_LEN = FRAME_OUTER_FACE_X - FACEPLATE_R
TRUNNION_COLLAR_R = 0.019
TRUNNION_COLLAR_T = 0.0035
TRUNNION_HUB_R = 0.017
TRUNNION_HUB_T = 0.005


def _make_base() -> cq.Workplane:
    body = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_H, centered=(True, True, False))
    body = body.edges("|Z").fillet(0.012)

    front_relief = (
        cq.Workplane("XZ")
        .box(BASE_W * 0.44, 0.012, BASE_H * 0.38, centered=(True, True, True))
        .translate((0.0, BASE_D / 2.0 - 0.006, BASE_H * 0.42))
    )
    body = body.cut(front_relief)

    bearing_pad = (
        cq.Workplane("XY")
        .circle(BASE_PAD_R)
        .extrude(BASE_PAD_H)
        .translate((0.0, 0.0, BASE_H))
    )
    return body.union(bearing_pad)


def _make_lower_stage() -> cq.Workplane:
    lower = cq.Workplane("XY").circle(STAGE_R).extrude(STAGE_LOWER_H)
    upper = (
        cq.Workplane("XY")
        .circle(STAGE_UPPER_R)
        .extrude(STAGE_UPPER_H)
        .translate((0.0, 0.0, STAGE_LOWER_H))
    )
    stage = lower.union(upper)

    stage = (
        stage.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(STAGE_UPPER_R * 0.78)
        .circle(0.017)
        .cutBlind(-0.002)
    )
    stage = (
        stage.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polarArray(0.038, 0.0, 360.0, 6)
        .hole(0.006, depth=0.005)
    )
    return stage


def _make_frame() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(FRAME_BASE_W, FRAME_BASE_D, FRAME_BASE_T, centered=(True, True, False))
        .translate((0.0, FRAME_BASE_Y, 0.0))
    )

    rear_web = (
        cq.Workplane("XY")
        .box(FRAME_WEB_W, FRAME_WEB_D, FRAME_WEB_H, centered=(True, True, False))
        .translate((0.0, FRAME_WEB_Y, FRAME_BASE_T))
    )

    window_len = FRAME_CHEEK_T + BEARING_BOSS_L + 0.006

    left_cheek = (
        cq.Workplane("XY")
        .box(FRAME_CHEEK_T, FRAME_CHEEK_D, FRAME_CHEEK_H, centered=(True, True, False))
        .translate((-FRAME_CHEEK_X, FRAME_CHEEK_Y, FRAME_BASE_T))
    )
    left_cheek = left_cheek.cut(
        cq.Workplane("YZ")
        .circle(FRAME_WINDOW_R)
        .extrude(window_len)
        .translate((-FRAME_OUTER_FACE_X - 0.003, TRUNNION_Y, TRUNNION_Z))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(FRAME_CHEEK_T, FRAME_CHEEK_D, FRAME_CHEEK_H, centered=(True, True, False))
        .translate((FRAME_CHEEK_X, FRAME_CHEEK_Y, FRAME_BASE_T))
    )
    right_cheek = right_cheek.cut(
        cq.Workplane("YZ")
        .circle(FRAME_WINDOW_R)
        .extrude(window_len)
        .translate((FRAME_INNER_FACE_X - 0.003, TRUNNION_Y, TRUNNION_Z))
    )

    left_boss = (
        cq.Workplane("YZ")
        .circle(BEARING_BOSS_R)
        .extrude(BEARING_BOSS_L)
        .translate(
            (
                -FRAME_CHEEK_X - FRAME_CHEEK_T / 2.0 - BEARING_BOSS_L,
                TRUNNION_Y,
                TRUNNION_Z,
            )
        )
    )
    right_boss = (
        cq.Workplane("YZ")
        .circle(BEARING_BOSS_R)
        .extrude(BEARING_BOSS_L)
        .translate(
            (
                FRAME_CHEEK_X + FRAME_CHEEK_T / 2.0,
                TRUNNION_Y,
                TRUNNION_Z,
            )
        )
    )

    frame = base_plate.union(rear_web).union(left_cheek).union(right_cheek)
    frame = frame.union(left_boss).union(right_boss)

    hole_len = BEARING_BOSS_L + FRAME_CHEEK_T
    left_hole = (
        cq.Workplane("YZ")
        .circle(TRUNNION_HOLE_R)
        .extrude(hole_len)
        .translate((-FRAME_OUTER_FACE_X, TRUNNION_Y, TRUNNION_Z))
    )
    right_hole = (
        cq.Workplane("YZ")
        .circle(TRUNNION_HOLE_R)
        .extrude(hole_len)
        .translate((FRAME_INNER_FACE_X, TRUNNION_Y, TRUNNION_Z))
    )
    return frame.cut(left_hole).cut(right_hole)


def _make_faceplate() -> cq.Workplane:
    plate = (
        cq.Workplane("XZ")
        .circle(FACEPLATE_R)
        .extrude(FACEPLATE_T)
        .translate((0.0, -FACEPLATE_T / 2.0, 0.0))
    )

    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .circle(FACEPLATE_RING_R)
        .circle(FACEPLATE_RING_INNER_R)
        .cutBlind(-0.002)
    )
    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .polarArray(FACEPLATE_PCD_R, 0.0, 360.0, 6)
        .hole(0.006, depth=0.007)
    )

    pilot = (
        cq.Workplane("XZ")
        .circle(FACEPLATE_PILOT_R)
        .extrude(FACEPLATE_PILOT_T)
        .translate((0.0, FACEPLATE_T / 2.0, 0.0))
    )
    back_hub = (
        cq.Workplane("XZ")
        .circle(BACK_HUB_R)
        .extrude(BACK_HUB_T)
        .translate((0.0, -FACEPLATE_T / 2.0 - BACK_HUB_T, 0.0))
    )
    left_hub = (
        cq.Workplane("YZ")
        .circle(TRUNNION_HUB_R)
        .extrude(TRUNNION_HUB_T)
        .translate((-(FACEPLATE_R + TRUNNION_HUB_T), 0.0, 0.0))
    )
    right_hub = (
        cq.Workplane("YZ")
        .circle(TRUNNION_HUB_R)
        .extrude(TRUNNION_HUB_T)
        .translate((FACEPLATE_R, 0.0, 0.0))
    )
    left_pin = (
        cq.Workplane("YZ")
        .circle(TRUNNION_PIN_R)
        .extrude(TRUNNION_PIN_LEN)
        .translate((-FRAME_OUTER_FACE_X, 0.0, 0.0))
    )
    right_pin = (
        cq.Workplane("YZ")
        .circle(TRUNNION_PIN_R)
        .extrude(TRUNNION_PIN_LEN)
        .translate((FACEPLATE_R, 0.0, 0.0))
    )
    left_collar = (
        cq.Workplane("YZ")
        .circle(TRUNNION_COLLAR_R)
        .extrude(TRUNNION_COLLAR_T)
        .translate((-(FRAME_OUTER_FACE_X + TRUNNION_COLLAR_T), 0.0, 0.0))
    )
    right_collar = (
        cq.Workplane("YZ")
        .circle(TRUNNION_COLLAR_R)
        .extrude(TRUNNION_COLLAR_T)
        .translate((FRAME_OUTER_FACE_X, 0.0, 0.0))
    )

    return (
        plate.union(back_hub)
        .union(left_hub)
        .union(right_hub)
        .union(left_pin)
        .union(right_pin)
        .union(left_collar)
        .union(right_collar)
        .union(pilot)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_cradle_indexer")

    body_dark = model.material("body_dark", rgba=(0.21, 0.23, 0.25, 1.0))
    body_mid = model.material("body_mid", rgba=(0.34, 0.36, 0.39, 1.0))
    stage_gray = model.material("stage_gray", rgba=(0.28, 0.30, 0.32, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base(), "indexer_base"),
        material=body_dark,
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_H + BASE_PAD_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_H + BASE_PAD_H) / 2.0)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_make_lower_stage(), "indexer_lower_stage"),
        material=stage_gray,
        name="stage_body",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=STAGE_R, length=STAGE_H),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, STAGE_H / 2.0)),
    )

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame(), "indexer_frame"),
        material=body_mid,
        name="frame_body",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.15, 0.10, FRAME_BASE_T + FRAME_CHEEK_H)),
        mass=7.5,
        origin=Origin(xyz=(0.0, -0.01, (FRAME_BASE_T + FRAME_CHEEK_H) / 2.0)),
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=FACEPLATE_R, length=FACEPLATE_T),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="faceplate_body",
    )
    faceplate.visual(
        Cylinder(radius=BACK_HUB_R, length=BACK_HUB_T),
        origin=Origin(
            xyz=(0.0, -(FACEPLATE_T / 2.0 + BACK_HUB_T / 2.0), 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="back_hub",
    )
    faceplate.visual(
        Cylinder(radius=FACEPLATE_PILOT_R, length=FACEPLATE_PILOT_T),
        origin=Origin(
            xyz=(0.0, FACEPLATE_T / 2.0 + FACEPLATE_PILOT_T / 2.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="pilot_nose",
    )
    faceplate.visual(
        Cylinder(radius=TRUNNION_HUB_R, length=TRUNNION_HUB_T),
        origin=Origin(
            xyz=(-(FACEPLATE_R + TRUNNION_HUB_T / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="left_trunnion_hub",
    )
    faceplate.visual(
        Cylinder(radius=TRUNNION_HUB_R, length=TRUNNION_HUB_T),
        origin=Origin(
            xyz=(FACEPLATE_R + TRUNNION_HUB_T / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="right_trunnion_hub",
    )
    faceplate.visual(
        Cylinder(radius=TRUNNION_PIN_R, length=TRUNNION_PIN_LEN),
        origin=Origin(
            xyz=(-(FRAME_OUTER_FACE_X + FACEPLATE_R) / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="left_trunnion_pin",
    )
    faceplate.visual(
        Cylinder(radius=TRUNNION_PIN_R, length=TRUNNION_PIN_LEN),
        origin=Origin(
            xyz=((FRAME_OUTER_FACE_X + FACEPLATE_R) / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="right_trunnion_pin",
    )
    faceplate.visual(
        Cylinder(radius=TRUNNION_COLLAR_R, length=TRUNNION_COLLAR_T),
        origin=Origin(
            xyz=(-(FRAME_OUTER_FACE_X + TRUNNION_COLLAR_T / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="left_retainer",
    )
    faceplate.visual(
        Cylinder(radius=TRUNNION_COLLAR_R, length=TRUNNION_COLLAR_T),
        origin=Origin(
            xyz=(FRAME_OUTER_FACE_X + TRUNNION_COLLAR_T / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="right_retainer",
    )
    faceplate.inertial = Inertial.from_geometry(
        Box((0.15, FACEPLATE_T + BACK_HUB_T + FACEPLATE_PILOT_T, 2.0 * FACEPLATE_R)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
    )

    model.articulation(
        "base_to_stage",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + BASE_PAD_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.5),
    )
    model.articulation(
        "stage_to_frame",
        ArticulationType.FIXED,
        parent=lower_stage,
        child=frame,
        origin=Origin(xyz=(0.0, 0.0, STAGE_H)),
    )
    model.articulation(
        "frame_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=faceplate,
        origin=Origin(xyz=(0.0, TRUNNION_Y, TRUNNION_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.45, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    frame = object_model.get_part("frame")
    faceplate = object_model.get_part("faceplate")
    base_to_stage = object_model.get_articulation("base_to_stage")
    frame_to_faceplate = object_model.get_articulation("frame_to_faceplate")

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
        "lower stage rotates on vertical axis",
        tuple(base_to_stage.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical C-axis, got {base_to_stage.axis}",
    )
    ctx.check(
        "upper faceplate rotates on horizontal axis",
        tuple(frame_to_faceplate.axis) == (1.0, 0.0, 0.0),
        details=f"expected horizontal trunnion axis, got {frame_to_faceplate.axis}",
    )
    ctx.check(
        "upper faceplate has realistic tilt limits",
        (
            frame_to_faceplate.motion_limits is not None
            and frame_to_faceplate.motion_limits.lower is not None
            and frame_to_faceplate.motion_limits.upper is not None
            and frame_to_faceplate.motion_limits.lower < 0.0 < frame_to_faceplate.motion_limits.upper
        ),
        details="expected bidirectional trunnion tilt limits around zero",
    )

    with ctx.pose({frame_to_faceplate: 0.0}):
        ctx.expect_contact(base, lower_stage, contact_tol=0.0015, name="stage is seated on base")
        ctx.expect_contact(lower_stage, frame, contact_tol=0.0015, name="frame is mounted on stage")
        ctx.expect_contact(faceplate, frame, contact_tol=0.0005, name="faceplate is carried by trunnion frame")
        ctx.expect_overlap(lower_stage, base, axes="xy", min_overlap=0.12, name="stage footprint stays over base")
        ctx.expect_origin_gap(faceplate, frame, axis="z", min_gap=0.07, max_gap=0.08, name="faceplate axis sits above frame base")
        ctx.expect_gap(faceplate, lower_stage, axis="z", min_gap=0.02, name="faceplate clears stage at neutral tilt")

    with ctx.pose({frame_to_faceplate: 1.2}):
        ctx.expect_gap(faceplate, lower_stage, axis="z", min_gap=0.015, name="faceplate clears stage when tilted")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
