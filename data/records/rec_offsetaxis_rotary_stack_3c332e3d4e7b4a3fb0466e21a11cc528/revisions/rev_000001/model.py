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


BASE_LENGTH = 0.42
BASE_WIDTH = 0.31
BASE_BODY_HEIGHT = 0.038
BASE_RING_HEIGHT = 0.018
BASE_LIP_HEIGHT = 0.019
MAIN_AXIS_HEIGHT = BASE_BODY_HEIGHT + BASE_RING_HEIGHT + BASE_LIP_HEIGHT

BASE_RING_RADIUS = 0.125
BASE_LIP_RADIUS = 0.105

LOWER_STAGE_RADIUS = 0.17
LOWER_STAGE_SKIRT_HEIGHT = 0.036
LOWER_STAGE_DECK_RADIUS = 0.146
LOWER_STAGE_DECK_HEIGHT = 0.012
LOWER_STAGE_CAP_RADIUS = 0.062
LOWER_STAGE_CAP_HEIGHT = 0.006

UPPER_AXIS_OFFSET = 0.145
SUPPORT_ARM_THICKNESS = 0.078
HEAD_BOSS_OUTER_RADIUS = 0.056
HEAD_BOSS_BORE_RADIUS = 0.032
HEAD_BOSS_BOTTOM_Z = 0.142
HEAD_BOSS_TOP_Z = 0.214

HEAD_FLANGE_RADIUS = 0.05
HEAD_FLANGE_HEIGHT = 0.01
HEAD_BODY_RADIUS = 0.058
HEAD_BODY_HEIGHT = 0.05
HEAD_FACEPLATE_RADIUS = 0.068
HEAD_FACEPLATE_HEIGHT = 0.012


def _make_plinth() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.02)
    )

    lower_ring = (
        cq.Workplane("XY")
        .circle(BASE_RING_RADIUS)
        .extrude(BASE_RING_HEIGHT)
        .translate((0.0, 0.0, BASE_BODY_HEIGHT))
    )

    upper_lip = (
        cq.Workplane("XY")
        .circle(BASE_LIP_RADIUS)
        .extrude(BASE_LIP_HEIGHT)
        .translate((0.0, 0.0, BASE_BODY_HEIGHT + BASE_RING_HEIGHT))
    )

    return body.union(lower_ring).union(upper_lip)


def _make_support_arm() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .moveTo(0.074, 0.018)
        .lineTo(0.126, 0.018)
        .lineTo(0.152, 0.052)
        .lineTo(0.17, 0.108)
        .lineTo(0.178, 0.148)
        .lineTo(0.178, 0.164)
        .threePointArc((0.152, 0.196), (0.112, 0.188))
        .lineTo(0.097, 0.124)
        .lineTo(0.082, 0.056)
        .close()
        .extrude(SUPPORT_ARM_THICKNESS, both=True)
    )

    window = (
        cq.Workplane("XZ")
        .moveTo(0.101, 0.058)
        .lineTo(0.121, 0.058)
        .lineTo(0.139, 0.09)
        .lineTo(0.147, 0.14)
        .lineTo(0.136, 0.165)
        .lineTo(0.118, 0.149)
        .lineTo(0.106, 0.106)
        .close()
        .extrude(SUPPORT_ARM_THICKNESS + 0.01, both=True)
    )

    top_relief = (
        cq.Workplane("XZ")
        .center(UPPER_AXIS_OFFSET, HEAD_BOSS_TOP_Z - 0.021)
        .rect(0.15, 0.072)
        .extrude(SUPPORT_ARM_THICKNESS + 0.01, both=True)
    )

    return outer.cut(window).cut(top_relief)


def _make_lower_stage() -> cq.Workplane:
    stage = cq.Workplane("XY").circle(LOWER_STAGE_RADIUS).extrude(LOWER_STAGE_SKIRT_HEIGHT)
    stage = (
        stage.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(LOWER_STAGE_DECK_RADIUS)
        .extrude(LOWER_STAGE_DECK_HEIGHT)
    )

    annular_groove = (
        cq.Workplane("XY")
        .circle(0.124)
        .circle(0.086)
        .extrude(0.004)
        .translate((0.0, 0.0, LOWER_STAGE_SKIRT_HEIGHT + LOWER_STAGE_DECK_HEIGHT - 0.004))
    )
    stage = stage.cut(annular_groove)

    center_cap = (
        cq.Workplane("XY")
        .circle(LOWER_STAGE_CAP_RADIUS)
        .extrude(LOWER_STAGE_CAP_HEIGHT)
        .translate((0.0, 0.0, LOWER_STAGE_SKIRT_HEIGHT + LOWER_STAGE_DECK_HEIGHT))
    )

    arm = _make_support_arm()
    head_boss = (
        cq.Workplane("XY")
        .circle(HEAD_BOSS_OUTER_RADIUS)
        .circle(HEAD_BOSS_BORE_RADIUS)
        .extrude(HEAD_BOSS_TOP_Z - HEAD_BOSS_BOTTOM_Z)
        .translate((UPPER_AXIS_OFFSET, 0.0, HEAD_BOSS_BOTTOM_Z))
    )

    return stage.union(center_cap).union(arm).union(head_boss)


def _make_upper_head() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(HEAD_FLANGE_RADIUS).extrude(HEAD_FLANGE_HEIGHT)

    body = (
        cq.Workplane("XY")
        .circle(HEAD_BODY_RADIUS)
        .extrude(HEAD_BODY_HEIGHT)
        .translate((0.0, 0.0, HEAD_FLANGE_HEIGHT))
    )
    waist_groove = (
        cq.Workplane("XY")
        .circle(0.049)
        .circle(0.043)
        .extrude(0.01)
        .translate((0.0, 0.0, HEAD_FLANGE_HEIGHT + 0.019))
    )
    body = body.cut(waist_groove)

    faceplate = (
        cq.Workplane("XY")
        .circle(HEAD_FACEPLATE_RADIUS)
        .extrude(HEAD_FACEPLATE_HEIGHT)
        .translate((0.0, 0.0, HEAD_FLANGE_HEIGHT + HEAD_BODY_HEIGHT))
    )

    head = flange.union(body).union(faceplate)

    top_cut_z = HEAD_FLANGE_HEIGHT + HEAD_BODY_HEIGHT + HEAD_FACEPLATE_HEIGHT - 0.005
    for angle_deg in (0.0, 120.0, 240.0):
        radial_slot = (
            cq.Workplane("XY")
            .center(0.024, 0.0)
            .slot2D(0.024, 0.01)
            .extrude(0.005)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
            .translate((0.0, 0.0, top_cut_z))
        )
        head = head.cut(radial_slot)

    center_recess = (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(0.008)
        .translate((0.0, 0.0, HEAD_FLANGE_HEIGHT + HEAD_BODY_HEIGHT + HEAD_FACEPLATE_HEIGHT - 0.008))
    )

    return head.cut(center_recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_indexing_head")

    plinth_mat = model.material("plinth_cast_iron", color=(0.18, 0.19, 0.2))
    lower_stage_mat = model.material("stage_machine_gray", color=(0.31, 0.33, 0.36))
    upper_head_mat = model.material("head_housing_gray", color=(0.56, 0.58, 0.61))

    plinth = model.part("plinth")
    plinth.visual(
        mesh_from_cadquery(_make_plinth(), "offset_indexing_head_plinth"),
        origin=Origin(),
        material=plinth_mat,
        name="plinth_shell",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_make_lower_stage(), "offset_indexing_head_lower_stage"),
        origin=Origin(),
        material=lower_stage_mat,
        name="lower_stage_shell",
    )

    upper_head = model.part("upper_head")
    upper_head.visual(
        mesh_from_cadquery(_make_upper_head(), "offset_indexing_head_upper_head"),
        origin=Origin(),
        material=upper_head_mat,
        name="upper_head_shell",
    )

    model.articulation(
        "main_rotation",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, MAIN_AXIS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5),
    )

    model.articulation(
        "head_rotation",
        ArticulationType.CONTINUOUS,
        parent=lower_stage,
        child=upper_head,
        origin=Origin(xyz=(UPPER_AXIS_OFFSET, 0.0, HEAD_BOSS_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    lower_stage = object_model.get_part("lower_stage")
    upper_head = object_model.get_part("upper_head")
    main_rotation = object_model.get_articulation("main_rotation")
    head_rotation = object_model.get_articulation("head_rotation")

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
        "key_parts_present",
        all((plinth, lower_stage, upper_head)),
        "Expected plinth, lower stage, and upper head parts.",
    )
    ctx.check(
        "vertical_parallel_axes",
        main_rotation.axis == (0.0, 0.0, 1.0) and head_rotation.axis == (0.0, 0.0, 1.0),
        "Both articulated stages should rotate about parallel upright axes.",
    )
    ctx.check(
        "continuous_rotation_stages",
        main_rotation.articulation_type == ArticulationType.CONTINUOUS
        and head_rotation.articulation_type == ArticulationType.CONTINUOUS,
        "The large lower stage and smaller upper stage should both be free-spinning rotary axes.",
    )

    ctx.expect_gap(
        lower_stage,
        plinth,
        axis="z",
        min_gap=0.0,
        max_gap=0.0015,
        name="lower_stage_seats_on_rotary_base",
    )
    ctx.expect_overlap(
        lower_stage,
        plinth,
        axes="xy",
        min_overlap=0.2,
        name="lower_stage_is_centered_over_plinth",
    )
    ctx.expect_contact(
        upper_head,
        lower_stage,
        contact_tol=0.0015,
        name="upper_head_is_mounted_to_side_support",
    )
    ctx.expect_origin_distance(
        upper_head,
        lower_stage,
        axes="xy",
        min_dist=UPPER_AXIS_OFFSET - 0.002,
        max_dist=UPPER_AXIS_OFFSET + 0.002,
        name="offset_axis_spacing",
    )

    with ctx.pose(main_rotation=1.1, head_rotation=-0.7):
        ctx.expect_origin_distance(
            upper_head,
            lower_stage,
            axes="xy",
            min_dist=UPPER_AXIS_OFFSET - 0.002,
            max_dist=UPPER_AXIS_OFFSET + 0.002,
            name="offset_spacing_persists_in_pose",
        )
        ctx.expect_contact(
            upper_head,
            lower_stage,
            contact_tol=0.0015,
            name="upper_head_remains_seated_while_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
