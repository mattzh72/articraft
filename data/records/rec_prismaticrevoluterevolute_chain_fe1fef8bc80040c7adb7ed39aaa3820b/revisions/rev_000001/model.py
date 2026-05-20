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


BASE_LEN = 0.62
BASE_W = 0.17
BASE_T = 0.018
BASE_MOUNT_L = 0.10
BASE_MOUNT_W = 0.118
BASE_MOUNT_H = 0.018
RAIL_BODY_L = 0.54
RAIL_BODY_W = 0.080
RAIL_BODY_H = 0.030
GUIDE_L = 0.50
GUIDE_W = 0.050
GUIDE_H = 0.014
STOP_L = 0.026
STOP_W = 0.074
STOP_H = 0.024
STOP_X = (GUIDE_L / 2.0) - (STOP_L / 2.0)
GUIDE_TOP_Z = BASE_T + RAIL_BODY_H + GUIDE_H

CARRIAGE_L = 0.23
CARRIAGE_W = 0.128
RUNNER_L = 0.196
RUNNER_W = 0.016
RUNNER_H = 0.018
RUNNER_Y = 0.012
DECK_T = 0.032
DECK_TOP = RUNNER_H + DECK_T
SKIRT_L = 0.188
SKIRT_T = 0.012
SKIRT_H = 0.034
SKIRT_Z = 0.008 + (SKIRT_H / 2.0)
SKIRT_Y = 0.050
PLINTH_X = 0.056
PLINTH_L = 0.102
PLINTH_W = 0.072
PLINTH_H = 0.028
PLINTH_TOP = DECK_TOP + PLINTH_H
PLINTH_Z = DECK_TOP + (PLINTH_H / 2.0)
SHOULDER_X = 0.087
SHOULDER_Z = 0.100
SHOULDER_HUB_R = 0.018
SHOULDER_HUB_W = 0.034
SHOULDER_EAR_L = 0.040
SHOULDER_EAR_T = 0.012
SHOULDER_EAR_H = 0.044
SHOULDER_EAR_Y = (SHOULDER_HUB_W / 2.0) + (SHOULDER_EAR_T / 2.0)
SHOULDER_CAP_T = 0.004
SHOULDER_CAP_R = 0.021
SHOULDER_CAP_Y = (SHOULDER_HUB_W / 2.0) + SHOULDER_EAR_T + (SHOULDER_CAP_T / 2.0)

UPPER_BASE_L = 0.038
UPPER_BASE_W = 0.022
UPPER_BASE_H = 0.020
UPPER_BEAM_L = 0.148
UPPER_BEAM_W = 0.024
UPPER_BEAM_H = 0.022
UPPER_ELBOW_X = 0.208
UPPER_MOUNT_L = 0.020
UPPER_MOUNT_W = 0.040
UPPER_MOUNT_H = 0.022
UPPER_RIB_L = 0.040
UPPER_RIB_W = 0.006
UPPER_RIB_H = 0.030
UPPER_RIB_Y = 0.012
ELBOW_HUB_R = 0.016
ELBOW_HUB_W = 0.030
ELBOW_CLEVIS_L = 0.038
ELBOW_CLEVIS_T = 0.010
ELBOW_CLEVIS_H = 0.044
ELBOW_CLEVIS_Y = (ELBOW_HUB_W / 2.0) + (ELBOW_CLEVIS_T / 2.0)
ELBOW_CAP_T = 0.004
ELBOW_CAP_R = 0.017
ELBOW_CAP_Y = (ELBOW_HUB_W / 2.0) + ELBOW_CLEVIS_T + (ELBOW_CAP_T / 2.0)

FOREARM_BASE_L = 0.032
FOREARM_BASE_W = 0.020
FOREARM_BASE_H = 0.020
FOREARM_BEAM_L = 0.132
FOREARM_BEAM_W = 0.022
FOREARM_BEAM_H = 0.020
FOREARM_NECK_L = 0.028
FOREARM_NECK_W = 0.028
FOREARM_NECK_H = 0.028
PAD_BACKER_T = 0.012
PAD_BACKER_W = 0.040
PAD_BACKER_H = 0.036
PAD_FACE_T = 0.010
PAD_FACE_W = 0.050
PAD_FACE_H = 0.044
PAD_BACKER_X = 0.170
PAD_FACE_X = PAD_BACKER_X + (PAD_BACKER_T / 2.0) + (PAD_FACE_T / 2.0)

SLIDE_LOWER = -0.10
SLIDE_UPPER = 0.10
SHOULDER_LOWER = 0.0
SHOULDER_UPPER = 1.10
ELBOW_LOWER = -0.35
ELBOW_UPPER = 1.45


def _base_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BASE_LEN, BASE_W, BASE_T, centered=(True, True, False))
    rail_body = cq.Workplane("XY").transformed(offset=(0.0, 0.0, BASE_T)).box(
        RAIL_BODY_L,
        RAIL_BODY_W,
        RAIL_BODY_H,
        centered=(True, True, False),
    )
    body = body.union(rail_body)

    for x_pos in (-0.20, 0.20):
        mount = cq.Workplane("XY").transformed(offset=(x_pos, 0.0, BASE_T)).box(
            BASE_MOUNT_L,
            BASE_MOUNT_W,
            BASE_MOUNT_H,
            centered=(True, True, False),
        )
        body = body.union(mount)

    for x_pos in (-STOP_X, STOP_X):
        stop = cq.Workplane("XY").transformed(offset=(x_pos, 0.0, BASE_T + RAIL_BODY_H)).box(
            STOP_L,
            STOP_W,
            STOP_H,
            centered=(True, True, False),
        )
        body = body.union(stop)

    for x_pos in (-0.22, 0.22):
        for y_pos in (-0.055, 0.055):
            slot = (
                cq.Workplane("XY")
                .transformed(offset=(x_pos, y_pos, 0.0))
                .slot2D(0.036, 0.014, angle=0.0)
                .extrude(BASE_T + 0.002)
            )
            body = body.cut(slot)

    center_relief = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.004)).box(
        0.16,
        0.108,
        0.010,
        centered=(True, True, False),
    )
    body = body.cut(center_relief)
    return body


def _carriage_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").transformed(offset=(0.0, 0.0, RUNNER_H)).box(
        CARRIAGE_L,
        CARRIAGE_W,
        DECK_T,
        centered=(True, True, False),
    )

    top_pocket = cq.Workplane("XY").transformed(
        offset=(-0.020, 0.0, DECK_TOP - 0.008),
    ).box(
        0.125,
        0.086,
        0.008,
        centered=(True, True, False),
    )
    body = body.cut(top_pocket)

    for y_pos in (-SKIRT_Y, SKIRT_Y):
        skirt = cq.Workplane("XY").transformed(offset=(0.0, y_pos, SKIRT_Z)).box(
            SKIRT_L,
            SKIRT_T,
            SKIRT_H,
            centered=(True, True, False),
        )
        body = body.union(skirt)

    plinth = cq.Workplane("XY").transformed(offset=(PLINTH_X, 0.0, DECK_TOP)).box(
        PLINTH_L,
        PLINTH_W,
        PLINTH_H,
        centered=(True, True, False),
    )
    nose = cq.Workplane("XY").transformed(offset=(0.090, 0.0, DECK_TOP)).box(
        0.026,
        0.042,
        0.018,
        centered=(True, True, False),
    )
    body = body.union(plinth).union(nose)

    for y_pos in (-0.030, 0.030):
        rib = cq.Workplane("XY").transformed(offset=(0.038, y_pos, DECK_TOP)).box(
            0.048,
            0.008,
            0.028,
            centered=(True, True, False),
        )
        body = body.union(rib)

    return body


def _upper_arm_frame_shape() -> cq.Workplane:
    root_block = cq.Workplane("XY").transformed(offset=(0.024, 0.0, 0.018)).box(
        0.048,
        0.024,
        0.012,
        centered=(True, True, True),
    )
    center_spine = cq.Workplane("XY").transformed(offset=(0.084, 0.0, 0.020)).box(
        0.094,
        0.014,
        0.014,
        centered=(True, True, True),
    )
    frame = root_block.union(center_spine)

    for x_pos in (0.050, 0.086):
        rib = cq.Workplane("XY").transformed(offset=(x_pos, 0.0, 0.014)).box(
            0.024,
            0.010,
            0.014,
            centered=(True, True, True),
        )
        frame = frame.union(rib)

    for y_pos in (-0.021, 0.021):
        split_rail = cq.Workplane("XY").transformed(offset=(0.154, y_pos, 0.022)).box(
            0.060,
            0.006,
            0.014,
            centered=(True, True, True),
        )
        elbow_stem = cq.Workplane("XY").transformed(offset=(0.186, y_pos, 0.014)).box(
            0.016,
            0.006,
            0.014,
            centered=(True, True, True),
        )
        frame = frame.union(split_rail).union(elbow_stem)

    return frame


def _forearm_frame_shape() -> cq.Workplane:
    root_block = cq.Workplane("XY").transformed(offset=(0.024, 0.0, -0.014)).box(
        0.040,
        0.012,
        0.016,
        centered=(True, True, True),
    )
    beam = cq.Workplane("XY").transformed(offset=(0.090, 0.0, -0.018)).box(
        0.122,
        0.012,
        0.014,
        centered=(True, True, True),
    )
    neck = cq.Workplane("XY").transformed(offset=(0.148, 0.0, 0.0)).box(
        FOREARM_NECK_L,
        0.020,
        0.024,
        centered=(True, True, True),
    )
    backer = cq.Workplane("XY").transformed(offset=(PAD_BACKER_X, 0.0, 0.0)).box(
        PAD_BACKER_T,
        PAD_BACKER_W,
        PAD_BACKER_H,
        centered=(True, True, True),
    )
    frame = root_block.union(beam).union(neck).union(backer)

    for y_pos in (-0.005, 0.005):
        rib = cq.Workplane("XY").transformed(offset=(0.046, y_pos, -0.010)).box(
            0.046,
            0.004,
            0.016,
            centered=(True, True, True),
        )
        frame = frame.union(rib)

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_positioning_module")

    model.material("base_steel", rgba=(0.33, 0.36, 0.40, 1.0))
    model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("carriage_dark", rgba=(0.19, 0.22, 0.25, 1.0))
    model.material("arm_silver", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("joint_steel", rgba=(0.58, 0.60, 0.64, 1.0))
    model.material("pad_rubber", rgba=(0.09, 0.10, 0.11, 1.0))

    base = model.part("base_rail")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "base_rail_body"),
        material="base_steel",
        name="base_body",
    )
    base.visual(
        Box((GUIDE_L, GUIDE_W, GUIDE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T + RAIL_BODY_H + (GUIDE_H / 2.0))),
        material="rail_steel",
        name="guide_strip",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_W, BASE_T + RAIL_BODY_H + STOP_H)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body_shape(), "carriage_body_mesh"),
        material="carriage_dark",
        name="carriage_body",
    )
    for name, y_pos in (("runner_left", RUNNER_Y), ("runner_right", -RUNNER_Y)):
        carriage.visual(
            Box((RUNNER_L, RUNNER_W, RUNNER_H)),
            origin=Origin(xyz=(0.0, y_pos, RUNNER_H / 2.0)),
            material="rail_steel",
            name=name,
        )
    for name, y_pos in (("shoulder_ear_left", SHOULDER_EAR_Y), ("shoulder_ear_right", -SHOULDER_EAR_Y)):
        carriage.visual(
            Box((SHOULDER_EAR_L, SHOULDER_EAR_T, SHOULDER_EAR_H)),
            origin=Origin(xyz=(SHOULDER_X, y_pos, SHOULDER_Z)),
            material="carriage_dark",
            name=name,
        )
    for name, y_pos in (("shoulder_cap_left", SHOULDER_CAP_Y), ("shoulder_cap_right", -SHOULDER_CAP_Y)):
        carriage.visual(
            Cylinder(radius=SHOULDER_CAP_R, length=SHOULDER_CAP_T),
            origin=Origin(xyz=(SHOULDER_X, y_pos, SHOULDER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material="joint_steel",
            name=name,
        )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_L, CARRIAGE_W, 0.11)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_frame_shape(), "upper_arm_frame_mesh"),
        material="arm_silver",
        name="upper_arm_frame",
    )
    upper_arm.visual(
        Cylinder(radius=SHOULDER_HUB_R, length=SHOULDER_HUB_W),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="joint_steel",
        name="shoulder_hub",
    )
    for name, y_pos in (("elbow_clevis_left", ELBOW_CLEVIS_Y), ("elbow_clevis_right", -ELBOW_CLEVIS_Y)):
        upper_arm.visual(
            Box((ELBOW_CLEVIS_L, ELBOW_CLEVIS_T, ELBOW_CLEVIS_H)),
            origin=Origin(xyz=(UPPER_ELBOW_X, y_pos, 0.0)),
            material="arm_silver",
            name=name,
        )
    for name, y_pos in (("elbow_cap_left", ELBOW_CAP_Y), ("elbow_cap_right", -ELBOW_CAP_Y)):
        upper_arm.visual(
            Cylinder(radius=ELBOW_CAP_R, length=ELBOW_CAP_T),
            origin=Origin(xyz=(UPPER_ELBOW_X, y_pos, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="joint_steel",
            name=name,
        )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.23, 0.06, 0.06)),
        mass=1.1,
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_frame_shape(), "forearm_frame_mesh"),
        material="arm_silver",
        name="forearm_frame",
    )
    forearm.visual(
        Cylinder(radius=ELBOW_HUB_R, length=ELBOW_HUB_W),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="joint_steel",
        name="elbow_hub",
    )
    forearm.visual(
        Box((PAD_FACE_T, PAD_FACE_W, PAD_FACE_H)),
        origin=Origin(xyz=(PAD_FACE_X, 0.0, 0.0)),
        material="pad_rubber",
        name="pad_face",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.20, 0.05, 0.05)),
        mass=0.85,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
        ),
    )
    model.articulation(
        "carriage_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=36.0,
            velocity=1.4,
            lower=SHOULDER_LOWER,
            upper=SHOULDER_UPPER,
        ),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ELBOW_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=26.0,
            velocity=1.8,
            lower=ELBOW_LOWER,
            upper=ELBOW_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")

    slide = object_model.get_articulation("base_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")

    ctx.check(
        "joint_axes_match_module_layout",
        slide.axis == (1.0, 0.0, 0.0)
        and shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0),
        details=f"axes={slide.axis}, {shoulder.axis}, {elbow.axis}",
    )

    ctx.expect_contact(base, carriage, elem_a="guide_strip", elem_b="runner_left", name="left_runner_on_guide")
    ctx.expect_contact(base, carriage, elem_a="guide_strip", elem_b="runner_right", name="right_runner_on_guide")
    ctx.expect_contact(
        carriage,
        upper_arm,
        elem_a="shoulder_ear_left",
        elem_b="shoulder_hub",
        name="left_shoulder_ear_contacts_hub",
    )
    ctx.expect_contact(
        carriage,
        upper_arm,
        elem_a="shoulder_ear_right",
        elem_b="shoulder_hub",
        name="right_shoulder_ear_contacts_hub",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        elem_a="elbow_clevis_left",
        elem_b="elbow_hub",
        name="left_elbow_clevis_contacts_hub",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        elem_a="elbow_clevis_right",
        elem_b="elbow_hub",
        name="right_elbow_clevis_contacts_hub",
    )

    with ctx.pose({slide: 0.0, shoulder: 0.0, elbow: 0.0}):
        ctx.expect_gap(
            upper_arm,
            carriage,
            axis="z",
            min_gap=0.008,
            positive_elem="upper_arm_frame",
            negative_elem="carriage_body",
            name="upper_arm_frame_clears_carriage_body_at_rest",
        )
        ctx.expect_gap(
            forearm,
            carriage,
            axis="x",
            min_gap=0.12,
            positive_elem="forearm_frame",
            negative_elem="carriage_body",
            name="forearm_starts_beyond_carriage_stage",
        )

    with ctx.pose({slide: SLIDE_UPPER, shoulder: SHOULDER_UPPER, elbow: 0.95}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_pose_slide_out_arm_up")

    with ctx.pose({slide: SLIDE_LOWER, shoulder: 0.25, elbow: ELBOW_LOWER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_pose_slide_in_low_elbow")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
