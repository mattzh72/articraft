from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_TOP_Z = 0.118
STAGE_AXIS_TO_FACEPLATE_X = 0.100
STAGE_AXIS_TO_FACEPLATE_Z = 0.362
FACEPLATE_FORWARD_OFFSET = 0.118


def _make_base() -> cq.Workplane:
    housing = cq.Workplane("XY").box(0.36, 0.27, 0.10, centered=(True, True, False))
    housing = housing.edges("|Z").fillet(0.010)
    housing = housing.faces(">Z").edges().fillet(0.005)

    turret = cq.Workplane("XY").circle(0.082).extrude(0.018).translate((0.0, 0.0, 0.10))
    base = housing.union(turret)

    center_well = (
        cq.Workplane("XY").circle(0.052).extrude(0.062).translate((0.0, 0.0, 0.056))
    )
    base = base.cut(center_well)

    front_pad = (
        cq.Workplane("XY")
        .box(0.16, 0.016, 0.040, centered=(True, True, False))
        .translate((0.0, -0.127, 0.022))
    )
    rear_pad = (
        cq.Workplane("XY")
        .box(0.12, 0.012, 0.030, centered=(True, True, False))
        .translate((0.0, 0.129, 0.028))
    )
    base = base.union(front_pad).union(rear_pad)
    return base


def _make_lower_stage() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.056).extrude(0.012)
    flange = cq.Workplane("XY").circle(0.078).extrude(0.010).translate((0.0, 0.0, 0.012))
    neck = cq.Workplane("XY").circle(0.086).extrude(0.014).translate((0.0, 0.0, 0.022))
    disk = cq.Workplane("XY").circle(0.126).extrude(0.024).translate((0.0, 0.0, 0.036))

    recess = cq.Workplane("XY").circle(0.056).extrude(0.006).translate((0.0, 0.0, 0.054))
    disk = disk.cut(recess)

    for angle_deg in range(0, 360, 45):
        angle = math.radians(angle_deg)
        x = 0.094 * math.cos(angle)
        y = 0.094 * math.sin(angle)
        bolt = cq.Workplane("XY").circle(0.005).extrude(0.012).translate((x, y, 0.046))
        disk = disk.cut(bolt)

    stage = hub.union(flange).union(neck).union(disk)

    support_arm = (
        cq.Workplane("XY")
        .box(0.060, 0.074, 0.020, centered=(False, True, False))
        .translate((0.0, 0.0, 0.070))
    )
    rear_column = (
        cq.Workplane("XY")
        .box(0.030, 0.092, 0.134, centered=(False, True, False))
        .translate((0.040, 0.0, 0.090))
    )
    side_tie = (
        cq.Workplane("XY")
        .box(0.024, 0.074, 0.056, centered=(False, True, False))
        .translate((0.050, 0.0, 0.210))
    )
    stage = stage.union(support_arm).union(rear_column).union(side_tie)

    for sign in (-1.0, 1.0):
        cheek = (
            cq.Workplane("XY")
            .box(0.046, 0.012, 0.194, centered=(False, True, False))
            .translate((0.058, sign * 0.056, 0.158))
        )
        window = (
            cq.Workplane("XZ")
            .ellipse(0.015, 0.056)
            .extrude(0.016, both=True)
            .translate((0.086, sign * 0.056, 0.238))
        )
        trunnion_hole = (
            cq.Workplane("XZ")
            .circle(0.0205)
            .extrude(0.016, both=True)
            .translate((STAGE_AXIS_TO_FACEPLATE_X, sign * 0.056, STAGE_AXIS_TO_FACEPLATE_Z))
        )
        cheek = cheek.cut(window).cut(trunnion_hole)

        bearing_cover = (
            cq.Workplane("XZ")
            .circle(0.032)
            .circle(0.022)
            .extrude(0.004)
            .translate((STAGE_AXIS_TO_FACEPLATE_X, sign * 0.064, STAGE_AXIS_TO_FACEPLATE_Z))
        )
        rear_gusset = (
            cq.Workplane("XY")
            .box(0.018, 0.012, 0.084, centered=(False, True, False))
            .translate((0.052, sign * 0.046, 0.148))
        )
        stop_lug_low = (
            cq.Workplane("XY")
            .box(0.012, 0.010, 0.024, centered=(False, True, False))
            .translate((0.096, sign * 0.074, 0.244))
        )
        stop_lug_high = (
            cq.Workplane("XY")
            .box(0.012, 0.010, 0.020, centered=(False, True, False))
            .translate((0.094, sign * 0.074, 0.348))
        )
        stage = stage.union(cheek).union(bearing_cover).union(rear_gusset).union(stop_lug_low).union(stop_lug_high)

    return stage


def _make_faceplate() -> cq.Workplane:
    shaft = cq.Workplane("XZ").circle(0.018).extrude(0.050, both=True)
    hub = cq.Workplane("XZ").circle(0.024).extrude(0.012, both=True)
    trunnion_rings = (
        cq.Workplane("XZ")
        .circle(0.026)
        .circle(0.018)
        .extrude(0.002, both=True)
        .translate((0.0, 0.050, 0.0))
    ).union(
        cq.Workplane("XZ")
        .circle(0.026)
        .circle(0.018)
        .extrude(0.002, both=True)
        .translate((0.0, -0.050, 0.0))
    )

    neck = (
        cq.Workplane("XY")
        .box(0.050, 0.022, 0.030, centered=(False, True, True))
        .translate((0.028, 0.0, 0.0))
    )
    clamp_ring = (
        cq.Workplane("XZ")
        .circle(0.030)
        .circle(0.018)
        .extrude(0.007, both=True)
        .translate((0.072, 0.0, 0.0))
    )
    plate = cq.Workplane("XZ").circle(0.070).extrude(0.016, both=True).translate(
        (FACEPLATE_FORWARD_OFFSET, 0.0, 0.0)
    )
    plate = plate.union(
        cq.Workplane("XZ")
        .circle(0.076)
        .circle(0.056)
        .extrude(0.005)
        .translate((FACEPLATE_FORWARD_OFFSET, 0.008, 0.0))
    )
    plate = plate.union(
        cq.Workplane("XZ")
        .circle(0.040)
        .extrude(0.010)
        .translate((FACEPLATE_FORWARD_OFFSET, -0.010, 0.0))
    )
    plate = plate.union(
        cq.Workplane("XZ")
        .circle(0.018)
        .extrude(0.005)
        .translate((FACEPLATE_FORWARD_OFFSET, 0.010, 0.0))
    )
    plate = plate.union(
        cq.Workplane("XZ")
        .circle(0.018)
        .extrude(0.004)
        .translate((FACEPLATE_FORWARD_OFFSET, 0.040, 0.0))
    )
    plate = plate.union(
        cq.Workplane("XZ")
        .circle(0.018)
        .extrude(0.004)
        .translate((FACEPLATE_FORWARD_OFFSET, -0.042, 0.0))
    )

    faceplate = shaft.union(hub).union(trunnion_rings).union(neck).union(clamp_ring).union(plate)

    cross_slot_x = (
        cq.Workplane("XY")
        .box(0.112, 0.004, 0.012, centered=(True, False, True))
        .translate((FACEPLATE_FORWARD_OFFSET, 0.008, 0.0))
    )
    cross_slot_z = (
        cq.Workplane("XY")
        .box(0.012, 0.004, 0.112, centered=(True, False, True))
        .translate((FACEPLATE_FORWARD_OFFSET, 0.008, 0.0))
    )
    faceplate = faceplate.cut(cross_slot_x).cut(cross_slot_z)

    for angle_deg in range(0, 360, 60):
        angle = math.radians(angle_deg)
        x = FACEPLATE_FORWARD_OFFSET + 0.052 * math.cos(angle)
        z = 0.052 * math.sin(angle)
        bolt = cq.Workplane("XZ").circle(0.004).extrude(0.005).translate((x, 0.008, z))
        faceplate = faceplate.cut(bolt)

    return faceplate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_cradle_indexer")

    base_gray = model.material("base_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    stage_gray = model.material("stage_gray", rgba=(0.53, 0.55, 0.58, 1.0))
    face_steel = model.material("face_steel", rgba=(0.74, 0.76, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base(), "base_shell"),
        material=base_gray,
        name="base_shell",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.056, length=0.012),
        material=stage_gray,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        name="lower_stage_body",
    )
    lower_stage.visual(
        Cylinder(radius=0.078, length=0.010),
        material=stage_gray,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        name="stage_flange",
    )
    lower_stage.visual(
        Cylinder(radius=0.126, length=0.024),
        material=stage_gray,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        name="stage_disk",
    )
    lower_stage.visual(
        Box((0.060, 0.074, 0.020)),
        material=stage_gray,
        origin=Origin(xyz=(0.030, 0.0, 0.056)),
        name="support_arm",
    )
    lower_stage.visual(
        Box((0.030, 0.132, 0.200)),
        material=stage_gray,
        origin=Origin(xyz=(0.055, 0.0, 0.146)),
        name="rear_column",
    )
    lower_stage.visual(
        Box((0.024, 0.090, 0.080)),
        material=stage_gray,
        origin=Origin(xyz=(0.062, 0.0, 0.286)),
        name="side_tie",
    )
    lower_stage.visual(
        Box((0.030, 0.126, 0.020)),
        material=stage_gray,
        origin=Origin(xyz=(0.095, 0.0, 0.494)),
        name="top_bridge",
    )

    for sign, side_name in ((1.0, "left"), (-1.0, "right")):
        lower_stage.visual(
            Box((0.046, 0.012, 0.274)),
            material=stage_gray,
            origin=Origin(xyz=(0.081, sign * 0.0645, 0.255)),
            name=f"{side_name}_lower_cheek",
        )
        lower_stage.visual(
            Box((0.046, 0.012, 0.098)),
            material=stage_gray,
            origin=Origin(xyz=(0.081, sign * 0.0645, 0.441)),
            name=f"{side_name}_upper_cheek",
        )
        lower_stage.visual(
            Box((0.018, 0.030, 0.110)),
            material=stage_gray,
            origin=Origin(xyz=(0.061, sign * 0.042, 0.111)),
            name=f"{side_name}_gusset",
        )
        lower_stage.visual(
            Cylinder(radius=0.032, length=0.006),
            material=stage_gray,
            origin=Origin(
                xyz=(STAGE_AXIS_TO_FACEPLATE_X, sign * 0.061, STAGE_AXIS_TO_FACEPLATE_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            name=f"{side_name}_bearing_cover",
        )
        lower_stage.visual(
            Box((0.012, 0.014, 0.024)),
            material=stage_gray,
            origin=Origin(xyz=(0.096, sign * 0.065, 0.286)),
            name=f"{side_name}_lower_stop",
        )
        lower_stage.visual(
            Box((0.012, 0.014, 0.020)),
            material=stage_gray,
            origin=Origin(xyz=(0.094, sign * 0.065, 0.412)),
            name=f"{side_name}_upper_stop",
        )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.018, length=0.104),
        material=face_steel,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="faceplate_body",
    )
    faceplate.visual(
        Cylinder(radius=0.024, length=0.012),
        material=face_steel,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="trunnion_hub",
    )
    faceplate.visual(
        Cylinder(radius=0.024, length=0.006),
        material=face_steel,
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="left_trunnion_collar",
    )
    faceplate.visual(
        Cylinder(radius=0.024, length=0.006),
        material=face_steel,
        origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="right_trunnion_collar",
    )
    faceplate.visual(
        Box((0.070, 0.022, 0.030)),
        material=face_steel,
        origin=Origin(xyz=(0.053, 0.0, 0.0)),
        name="faceplate_neck",
    )
    faceplate.visual(
        Cylinder(radius=0.030, length=0.014),
        material=face_steel,
        origin=Origin(xyz=(0.091, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="clamp_ring",
    )
    faceplate.visual(
        Cylinder(radius=0.064, length=0.016),
        material=face_steel,
        origin=Origin(xyz=(FACEPLATE_FORWARD_OFFSET, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="fixture_plate",
    )
    faceplate.visual(
        Cylinder(radius=0.040, length=0.022),
        material=face_steel,
        origin=Origin(
            xyz=(0.102, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        name="fixture_hub",
    )
    faceplate.visual(
        Cylinder(radius=0.018, length=0.006),
        material=face_steel,
        origin=Origin(
            xyz=(FACEPLATE_FORWARD_OFFSET, 0.040, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        name="upper_pad",
    )
    faceplate.visual(
        Cylinder(radius=0.018, length=0.006),
        material=face_steel,
        origin=Origin(
            xyz=(FACEPLATE_FORWARD_OFFSET, -0.040, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        name="lower_pad",
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.5),
    )
    model.articulation(
        "lower_stage_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=faceplate,
        origin=Origin(xyz=(STAGE_AXIS_TO_FACEPLATE_X, 0.0, STAGE_AXIS_TO_FACEPLATE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.1,
            lower=math.radians(-55.0),
            upper=math.radians(62.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    faceplate = object_model.get_part("faceplate")
    stage_joint = object_model.get_articulation("base_to_lower_stage")
    tilt_joint = object_model.get_articulation("lower_stage_to_faceplate")

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
        "stage rotation axis is vertical",
        tuple(stage_joint.axis) == (0.0, 0.0, 1.0),
        f"axis={stage_joint.axis}",
    )
    ctx.check(
        "faceplate tilt axis is horizontal",
        tuple(tilt_joint.axis) == (0.0, 1.0, 0.0),
        f"axis={tilt_joint.axis}",
    )
    tilt_limits = tilt_joint.motion_limits
    ctx.check(
        "faceplate tilt limits bracket zero",
        tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and tilt_limits.lower < 0.0 < tilt_limits.upper
        and tilt_limits.upper <= math.radians(75.0),
        f"limits={tilt_limits}",
    )

    ctx.expect_contact(base, lower_stage, name="lower stage is carried by base bearing")
    ctx.expect_contact(
        lower_stage,
        faceplate,
        name="faceplate is carried by trunnion bearing covers",
    )
    ctx.expect_gap(
        faceplate,
        base,
        axis="z",
        min_gap=0.070,
        name="faceplate clears base at rest",
    )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({stage_joint: 0.85, tilt_joint: tilt_limits.lower}):
            ctx.expect_gap(
                faceplate,
                base,
                axis="z",
                min_gap=0.070,
                name="faceplate clears base at negative tilt",
            )
        with ctx.pose({stage_joint: -0.95, tilt_joint: tilt_limits.upper}):
            ctx.expect_gap(
                faceplate,
                base,
                axis="z",
                min_gap=0.070,
                name="faceplate clears base at positive tilt",
            )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=18,
        name="articulation sweeps stay clear",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=18,
        ignore_adjacent=False,
        ignore_fixed=False,
        name="sampled pose model stays clear",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
