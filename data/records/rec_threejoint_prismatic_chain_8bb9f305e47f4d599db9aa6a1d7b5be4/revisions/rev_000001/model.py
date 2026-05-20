from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 1.05
BASE_WIDTH = 0.34
BASE_PLATE_T = 0.03
BASE_GUIDE_LEN = 0.97
BASE_GUIDE_WIDTH = 0.18
BASE_GUIDE_H = 0.055

STAGE1_LEN = 0.84
STAGE1_CARRIAGE_LEN = 0.26
STAGE1_WIDTH = 0.26
STAGE1_BEAM_WIDTH = 0.16
STAGE1_CARRIAGE_H = 0.035
STAGE1_BEAM_H = 0.045
STAGE1_RAIL_LEN = 0.68
STAGE1_RAIL_WIDTH = 0.11
STAGE1_RAIL_H = 0.022

STAGE2_LEN = 0.56
STAGE2_CARRIAGE_LEN = 0.19
STAGE2_WIDTH = 0.18
STAGE2_BEAM_WIDTH = 0.11
STAGE2_CARRIAGE_H = 0.028
STAGE2_BEAM_H = 0.036
STAGE2_RAIL_LEN = 0.42
STAGE2_RAIL_WIDTH = 0.078
STAGE2_RAIL_H = 0.018

STAGE3_CARRIAGE_LEN = 0.14
STAGE3_WIDTH = 0.13
STAGE3_CARRIAGE_H = 0.026
STAGE3_BODY_LEN = 0.09
STAGE3_BODY_WIDTH = 0.10
STAGE3_BODY_H = 0.088
STAGE3_FACE_T = 0.012
STAGE3_FACE_SIZE = 0.10

J1_X = 0.09
J1_Z = BASE_PLATE_T + BASE_GUIDE_H
J2_X = 0.13
J2_Z = STAGE1_CARRIAGE_H + STAGE1_BEAM_H + STAGE1_RAIL_H
J3_X = 0.10
J3_Z = STAGE2_CARRIAGE_H + STAGE2_BEAM_H + STAGE2_RAIL_H


def _box(length: float, width: float, height: float, *, x0: float = 0.0, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, False))
        .translate((x0, 0.0, z0))
    )


def _make_base_shape() -> cq.Workplane:
    plate = _box(BASE_LEN, BASE_WIDTH, BASE_PLATE_T).edges("|Z").fillet(0.006)
    guide = (
        _box(BASE_GUIDE_LEN, BASE_GUIDE_WIDTH, BASE_GUIDE_H, x0=0.04, z0=BASE_PLATE_T)
        .edges("|Z")
        .fillet(0.004)
    )
    side_rib_left = (
        _box(BASE_GUIDE_LEN - 0.12, 0.05, 0.02, x0=0.08, z0=BASE_PLATE_T)
        .translate((0.0, 0.105, 0.0))
        .edges("|Z")
        .fillet(0.003)
    )
    side_rib_right = (
        _box(BASE_GUIDE_LEN - 0.12, 0.05, 0.02, x0=0.08, z0=BASE_PLATE_T)
        .translate((0.0, -0.105, 0.0))
        .edges("|Z")
        .fillet(0.003)
    )
    return plate.union(guide).union(side_rib_left).union(side_rib_right)


def _make_stage1_shape() -> cq.Workplane:
    carriage = _box(STAGE1_CARRIAGE_LEN, STAGE1_WIDTH, STAGE1_CARRIAGE_H).edges("|Z").fillet(0.004)
    beam = (
        _box(
            STAGE1_LEN,
            STAGE1_BEAM_WIDTH,
            STAGE1_BEAM_H,
            x0=0.03,
            z0=STAGE1_CARRIAGE_H,
        )
        .edges("|Z")
        .fillet(0.003)
    )
    rail = (
        _box(
            STAGE1_RAIL_LEN,
            STAGE1_RAIL_WIDTH,
            STAGE1_RAIL_H,
            x0=0.12,
            z0=STAGE1_CARRIAGE_H + STAGE1_BEAM_H,
        )
        .edges("|Z")
        .fillet(0.0025)
    )
    nose = (
        _box(0.08, 0.18, 0.032, x0=0.79, z0=STAGE1_CARRIAGE_H)
        .edges("|Z")
        .fillet(0.003)
    )
    return carriage.union(beam).union(rail).union(nose)


def _make_stage2_shape() -> cq.Workplane:
    carriage = _box(STAGE2_CARRIAGE_LEN, STAGE2_WIDTH, STAGE2_CARRIAGE_H).edges("|Z").fillet(0.0035)
    beam = (
        _box(
            STAGE2_LEN,
            STAGE2_BEAM_WIDTH,
            STAGE2_BEAM_H,
            x0=0.03,
            z0=STAGE2_CARRIAGE_H,
        )
        .edges("|Z")
        .fillet(0.003)
    )
    rail = (
        _box(
            STAGE2_RAIL_LEN,
            STAGE2_RAIL_WIDTH,
            STAGE2_RAIL_H,
            x0=0.10,
            z0=STAGE2_CARRIAGE_H + STAGE2_BEAM_H,
        )
        .edges("|Z")
        .fillet(0.002)
    )
    nose = (
        _box(0.06, 0.125, 0.028, x0=0.53, z0=STAGE2_CARRIAGE_H)
        .edges("|Z")
        .fillet(0.0025)
    )
    return carriage.union(beam).union(rail).union(nose)


def _make_stage3_shape() -> cq.Workplane:
    carriage = _box(STAGE3_CARRIAGE_LEN, STAGE3_WIDTH, STAGE3_CARRIAGE_H).edges("|Z").fillet(0.003)
    body = (
        _box(
            STAGE3_BODY_LEN,
            STAGE3_BODY_WIDTH,
            STAGE3_BODY_H,
            x0=0.05,
            z0=STAGE3_CARRIAGE_H,
        )
        .edges("|Z")
        .fillet(0.003)
    )
    top_cap = (
        _box(0.045, 0.08, 0.016, x0=0.072, z0=STAGE3_CARRIAGE_H + STAGE3_BODY_H)
        .edges("|Z")
        .fillet(0.002)
    )
    return carriage.union(body).union(top_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_transfer_axis")

    base_mat = model.material("base_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    stage1_mat = model.material("stage1_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    stage2_mat = model.material("stage2_silver", rgba=(0.62, 0.65, 0.69, 1.0))
    stage3_mat = model.material("stage3_light", rgba=(0.78, 0.79, 0.81, 1.0))
    face_mat = model.material("tool_face_black", rgba=(0.15, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "transfer_axis_base"), material=base_mat, name="base_body")

    stage1 = model.part("stage1")
    stage1.visual(
        mesh_from_cadquery(_make_stage1_shape(), "transfer_axis_stage1"),
        material=stage1_mat,
        name="stage1_body",
    )

    stage2 = model.part("stage2")
    stage2.visual(
        mesh_from_cadquery(_make_stage2_shape(), "transfer_axis_stage2"),
        material=stage2_mat,
        name="stage2_body",
    )

    stage3 = model.part("stage3")
    stage3.visual(
        mesh_from_cadquery(_make_stage3_shape(), "transfer_axis_stage3"),
        material=stage3_mat,
        name="stage3_body",
    )
    stage3.visual(
        Box((STAGE3_FACE_T, STAGE3_FACE_SIZE, STAGE3_FACE_SIZE)),
        origin=Origin(
            xyz=(
                0.05 + STAGE3_BODY_LEN + (STAGE3_FACE_T / 2.0),
                0.0,
                (STAGE3_CARRIAGE_H + STAGE3_BODY_H) / 2.0,
            )
        ),
        material=face_mat,
        name="square_face",
    )

    model.articulation(
        "base_to_stage1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage1,
        origin=Origin(xyz=(J1_X, 0.0, J1_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.8,
            lower=0.0,
            upper=0.42,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(J2_X, 0.0, J2_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.7,
            lower=0.0,
            upper=0.27,
        ),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=(J3_X, 0.0, J3_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=700.0,
            velocity=0.6,
            lower=0.0,
            upper=0.14,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    stage3 = object_model.get_part("stage3")
    square_face = stage3.get_visual("square_face")

    joint1 = object_model.get_articulation("base_to_stage1")
    joint2 = object_model.get_articulation("stage1_to_stage2")
    joint3 = object_model.get_articulation("stage2_to_stage3")

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
        "all_primary_parts_present",
        all(part is not None for part in (base, stage1, stage2, stage3)),
        "Base and all three slide stages should exist.",
    )
    ctx.check(
        "all_joints_prismatic",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            for joint in (joint1, joint2, joint3)
        ),
        "The transfer axis should use three serial prismatic joints.",
    )
    ctx.check(
        "all_joint_axes_forward",
        all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in (joint1, joint2, joint3)),
        "Each stage should extend along +X.",
    )

    ctx.expect_gap(
        stage1,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        name="stage1_sits_on_base",
    )
    ctx.expect_overlap(
        stage1,
        base,
        axes="xy",
        min_overlap=0.16,
        name="stage1_has_supported_footprint",
    )
    ctx.expect_gap(
        stage2,
        stage1,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        name="stage2_sits_on_stage1",
    )
    ctx.expect_overlap(
        stage2,
        stage1,
        axes="xy",
        min_overlap=0.10,
        name="stage2_has_supported_footprint",
    )
    ctx.expect_gap(
        stage3,
        stage2,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        name="stage3_sits_on_stage2",
    )
    ctx.expect_overlap(
        stage3,
        stage2,
        axes="xy",
        min_overlap=0.07,
        name="stage3_has_supported_footprint",
    )

    with ctx.pose({joint1: joint1.motion_limits.upper}):
        ctx.expect_origin_gap(
            stage1,
            base,
            axis="x",
            min_gap=J1_X + joint1.motion_limits.upper - 0.005,
            name="stage1_extends_forward",
        )

    with ctx.pose({joint2: joint2.motion_limits.upper}):
        ctx.expect_origin_gap(
            stage2,
            stage1,
            axis="x",
            min_gap=J2_X + joint2.motion_limits.upper - 0.005,
            name="stage2_extends_forward",
        )

    with ctx.pose({joint3: joint3.motion_limits.upper}):
        ctx.expect_origin_gap(
            stage3,
            stage2,
            axis="x",
            min_gap=J3_X + joint3.motion_limits.upper - 0.005,
            name="stage3_extends_forward",
        )

    face_aabb = ctx.part_element_world_aabb(stage3, elem=square_face.name)
    square_ok = False
    details = "Square terminal face visual not found."
    if face_aabb is not None:
        (min_corner, max_corner) = face_aabb
        dy = max_corner[1] - min_corner[1]
        dz = max_corner[2] - min_corner[2]
        square_ok = dy > 0.095 and dz > 0.095 and abs(dy - dz) < 0.003
        details = f"Expected square terminal face, got dy={dy:.4f}, dz={dz:.4f}"
    ctx.check("terminal_face_is_square", square_ok, details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
