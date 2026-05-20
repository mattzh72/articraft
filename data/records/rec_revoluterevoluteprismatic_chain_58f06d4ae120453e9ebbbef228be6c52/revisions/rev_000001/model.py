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


BASE_LENGTH = 0.18
BASE_WIDTH = 0.12
BASE_THICKNESS = 0.016

SHOULDER_X = 0.055
SHOULDER_Z = 0.145

UPPER_LINK_LENGTH = 0.30
FOREARM_NOSE_X = 0.28
PROBE_STROKE = 0.15


def fused(*objects: object) -> cq.Workplane:
    wp = cq.Workplane("XY")
    for obj in objects:
        wp = wp.add(obj.val() if hasattr(obj, "val") else obj)
    return wp.combine()


def make_shoulder_block() -> cq.Workplane:
    mounting_holes = [
        (-0.055, -0.035),
        (-0.055, 0.035),
        (0.055, -0.035),
        (0.055, 0.035),
    ]

    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(mounting_holes)
        .hole(0.014)
        .translate((-0.035, 0.0, BASE_THICKNESS / 2.0))
    )

    pedestal = cq.Workplane("XY").box(0.07, 0.07, 0.09).translate((-0.025, 0.0, 0.061))
    spine = cq.Workplane("XY").box(0.035, 0.03, 0.04).translate((0.0125, 0.0, 0.11))
    shoulder_neck = cq.Workplane("XY").box(0.04, 0.024, 0.02).translate((0.02, 0.0, 0.126))

    clevis_outer = cq.Workplane("XY").box(0.02, 0.04, 0.06).translate((0.045, 0.0, SHOULDER_Z))
    clevis_slot = cq.Workplane("XY").box(0.024, 0.018, 0.038).translate((0.045, 0.0, SHOULDER_Z))
    clevis = clevis_outer.cut(clevis_slot)
    return fused(base, pedestal, spine, shoulder_neck, clevis)


def make_upper_link() -> cq.Workplane:
    shoulder_tab = cq.Workplane("XY").box(0.016, 0.016, 0.032).translate((0.008, 0.0, 0.0))
    beam_root = cq.Workplane("XY").box(0.09, 0.022, 0.028).translate((0.065, 0.0, 0.0))
    beam_mid = cq.Workplane("XY").box(0.11, 0.028, 0.032).translate((0.17, 0.0, 0.0))
    elbow_neck = cq.Workplane("XY").box(0.05, 0.024, 0.032).translate((0.245, 0.0, 0.0))

    elbow_outer = cq.Workplane("XY").box(0.03, 0.04, 0.048).translate((0.285, 0.0, 0.0))
    elbow_slot = cq.Workplane("XY").box(0.032, 0.018, 0.034).translate((0.285, 0.0, 0.0))
    elbow_clevis = elbow_outer.cut(elbow_slot)

    return fused(shoulder_tab, beam_root, beam_mid, elbow_neck, elbow_clevis)


def make_forearm_link() -> cq.Workplane:
    elbow_tab = cq.Workplane("XY").box(0.016, 0.016, 0.03).translate((0.008, 0.0, 0.0))
    beam_root = cq.Workplane("XY").box(0.08, 0.02, 0.026).translate((0.058, 0.0, 0.0))
    beam_mid = cq.Workplane("XY").box(0.11, 0.024, 0.028).translate((0.155, 0.0, 0.0))
    transition = cq.Workplane("XY").box(0.03, 0.026, 0.03).translate((0.225, 0.0, 0.0))

    nose_outer = cq.Workplane("YZ").circle(0.017).extrude(0.04).translate((0.24, 0.0, 0.0))
    nose_inner = cq.Workplane("YZ").circle(0.010).extrude(0.04).translate((0.24, 0.0, 0.0))
    nose_tube = nose_outer.cut(nose_inner)

    return fused(elbow_tab, beam_root, beam_mid, transition, nose_tube)


def make_probe_stage_body() -> cq.Workplane:
    return cq.Workplane("YZ").circle(0.014).extrude(0.02)


def make_probe_stage_tip() -> cq.Workplane:
    rod = cq.Workplane("YZ").circle(0.0055).extrude(0.1).translate((0.02, 0.0, 0.0))
    sensor_ball = cq.Workplane("XY").sphere(0.005).translate((0.125, 0.0, 0.0))
    return fused(rod, sensor_ball)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_probe_manipulator")

    base_dark = model.material("base_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    arm_alloy = model.material("arm_alloy", rgba=(0.76, 0.78, 0.8, 1.0))
    probe_black = model.material("probe_black", rgba=(0.12, 0.12, 0.14, 1.0))
    probe_steel = model.material("probe_steel", rgba=(0.82, 0.84, 0.86, 1.0))

    shoulder_block = model.part("shoulder_block")
    shoulder_block.visual(
        mesh_from_cadquery(make_shoulder_block(), "shoulder_block"),
        material=base_dark,
        name="shoulder_block_shell",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(make_upper_link(), "upper_link"),
        material=arm_alloy,
        name="upper_link_shell",
    )

    forearm_link = model.part("forearm_link")
    forearm_link.visual(
        mesh_from_cadquery(make_forearm_link(), "forearm_link"),
        material=arm_alloy,
        name="forearm_link_shell",
    )

    probe_stage = model.part("probe_stage")
    probe_stage.visual(
        mesh_from_cadquery(make_probe_stage_body(), "probe_stage_body"),
        material=probe_black,
        name="probe_stage_body",
    )
    probe_stage.visual(
        mesh_from_cadquery(make_probe_stage_tip(), "probe_stage_tip"),
        material=probe_steel,
        name="probe_stage_tip",
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_block,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=-1.15,
            upper=1.05,
        ),
    )

    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm_link,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.8,
            lower=-1.7,
            upper=0.35,
        ),
    )

    model.articulation(
        "probe_slide",
        ArticulationType.PRISMATIC,
        parent=forearm_link,
        child=probe_stage,
        origin=Origin(xyz=(FOREARM_NOSE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.35,
            lower=0.0,
            upper=PROBE_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    joint_clearance_tol = 0.0015

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=joint_clearance_tol)
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

    expected_parts = {"shoulder_block", "upper_link", "forearm_link", "probe_stage"}
    expected_joints = {"shoulder_pitch", "elbow_pitch", "probe_slide"}

    have_parts = ctx.check(
        "expected parts present",
        expected_parts.issubset({part.name for part in object_model.parts}),
        details=f"Missing one of {sorted(expected_parts)}",
    )
    have_joints = ctx.check(
        "expected articulations present",
        expected_joints.issubset({joint.name for joint in object_model.articulations}),
        details=f"Missing one of {sorted(expected_joints)}",
    )
    if not (have_parts and have_joints):
        return ctx.report()

    shoulder_block = object_model.get_part("shoulder_block")
    upper_link = object_model.get_part("upper_link")
    forearm_link = object_model.get_part("forearm_link")
    probe_stage = object_model.get_part("probe_stage")

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    probe_slide = object_model.get_articulation("probe_slide")

    def _close(a: float | None, b: float, tol: float = 1e-9) -> bool:
        return a is not None and abs(a - b) <= tol

    ctx.check(
        "serial articulation chain",
        shoulder_pitch.parent == shoulder_block.name
        and shoulder_pitch.child == upper_link.name
        and elbow_pitch.parent == upper_link.name
        and elbow_pitch.child == forearm_link.name
        and probe_slide.parent == forearm_link.name
        and probe_slide.child == probe_stage.name,
        details="Manipulator joints do not form shoulder -> upper -> forearm -> probe chain.",
    )
    ctx.check(
        "joint axes match manipulator layout",
        shoulder_pitch.axis == (0.0, 1.0, 0.0)
        and elbow_pitch.axis == (0.0, 1.0, 0.0)
        and probe_slide.axis == (1.0, 0.0, 0.0),
        details="Expected pitch-pitch-axial-slide articulation axes.",
    )
    ctx.check(
        "motion limits are realistic",
        shoulder_pitch.motion_limits is not None
        and elbow_pitch.motion_limits is not None
        and probe_slide.motion_limits is not None
        and _close(probe_slide.motion_limits.lower, 0.0)
        and _close(probe_slide.motion_limits.upper, PROBE_STROKE)
        and shoulder_pitch.motion_limits.lower is not None
        and shoulder_pitch.motion_limits.upper is not None
        and shoulder_pitch.motion_limits.lower < 0.0 < shoulder_pitch.motion_limits.upper
        and elbow_pitch.motion_limits.lower is not None
        and elbow_pitch.motion_limits.upper is not None
        and elbow_pitch.motion_limits.lower < 0.0 < elbow_pitch.motion_limits.upper,
        details="Expected bidirectional shoulder/elbow travel and a 150 mm probe stroke.",
    )

    ctx.expect_gap(
        upper_link,
        shoulder_block,
        axis="x",
        min_gap=0.0,
        max_gap=joint_clearance_tol,
        max_penetration=0.0,
        name="shoulder joint closes onto grounded block",
    )
    ctx.expect_overlap(
        shoulder_block,
        upper_link,
        axes="yz",
        min_overlap=0.018,
        name="shoulder joint tab sits within shoulder clevis span",
    )
    ctx.expect_gap(
        forearm_link,
        upper_link,
        axis="x",
        min_gap=0.0,
        max_gap=joint_clearance_tol,
        max_penetration=0.0,
        name="elbow joint closes between upper arm and forearm",
    )
    ctx.expect_overlap(
        upper_link,
        forearm_link,
        axes="yz",
        min_overlap=0.016,
        name="elbow tab sits within upper arm clevis span",
    )
    ctx.expect_contact(
        forearm_link,
        probe_stage,
        elem_b="probe_stage_body",
        name="forearm nose houses probe stage",
    )
    ctx.expect_overlap(
        probe_stage,
        forearm_link,
        axes="yz",
        elem_a="probe_stage_body",
        min_overlap=0.027,
        name="probe stage is coaxial with forearm nose",
    )

    with ctx.pose({probe_slide: PROBE_STROKE}):
        ctx.expect_origin_gap(
            probe_stage,
            forearm_link,
            axis="x",
            min_gap=FOREARM_NOSE_X + PROBE_STROKE - 0.005,
            max_gap=FOREARM_NOSE_X + PROBE_STROKE + 0.005,
            name="probe reaches full prismatic extension",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
