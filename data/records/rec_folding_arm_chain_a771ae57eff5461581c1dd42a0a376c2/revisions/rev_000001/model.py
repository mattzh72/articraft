from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.110
FOOT_WIDTH = 0.060
FOOT_HEIGHT = 0.014
SPINE_HEIGHT = 0.235
SPINE_THICKNESS_X = 0.020
SPINE_WIDTH_Y = 0.014
SHOULDER_X = 0.028
SHOULDER_Z = FOOT_HEIGHT + SPINE_HEIGHT

HUB_LENGTH = 0.012
CHEEK_THICKNESS = 0.005
OUTER_JOINT_WIDTH = HUB_LENGTH + 2.0 * CHEEK_THICKNESS
CLEVIS_DEPTH = 0.028
CLEVIS_BRIDGE = 0.010
CLEVIS_FORWARD = 0.016
CLEVIS_HEIGHT = 0.034
LINK_BODY_WIDTH = 0.010
LINK_BODY_HEIGHT = 0.018
ROOT_LUG_LENGTH = 0.014
ROOT_LUG_HEIGHT = 0.026

LINK_1_LENGTH = 0.220
LINK_2_LENGTH = 0.185
LINK_3_LENGTH = 0.145
PAD_LENGTH = 0.040
PAD_WIDTH = 0.024
PAD_HEIGHT = 0.014
PAD_CENTER_X = 0.185


def _root_lug() -> cq.Workplane:
    return cq.Workplane("XY").box(
        ROOT_LUG_LENGTH,
        HUB_LENGTH,
        ROOT_LUG_HEIGHT,
        centered=(False, True, True),
    )


def _clevis_tip(length: float) -> cq.Workplane:
    tip_block = (
        cq.Workplane("XY")
        .box(CLEVIS_DEPTH + CLEVIS_FORWARD, OUTER_JOINT_WIDTH, CLEVIS_HEIGHT, centered=(False, True, True))
        .translate((length - CLEVIS_DEPTH, 0.0, 0.0))
    )
    slot = (
        cq.Workplane("XY")
        .box(
            CLEVIS_DEPTH - CLEVIS_BRIDGE + CLEVIS_FORWARD + 0.006,
            HUB_LENGTH,
            CLEVIS_HEIGHT + 0.004,
            centered=(False, True, True),
        )
        .translate((length - CLEVIS_DEPTH + CLEVIS_BRIDGE, 0.0, 0.0))
    )
    return tip_block.cut(slot)


def _link_with_clevis(length: float) -> cq.Workplane:
    beam = cq.Workplane("XY").box(
        length - 0.010,
        LINK_BODY_WIDTH,
        LINK_BODY_HEIGHT,
        centered=(False, True, True),
    )
    root_reinforcement = (
        cq.Workplane("XY")
        .box(0.032, LINK_BODY_WIDTH, LINK_BODY_HEIGHT + 0.004, centered=(False, True, True))
        .translate((0.0, 0.0, 0.0))
    )
    return beam.union(root_reinforcement).union(_root_lug()).union(_clevis_tip(length))


def _terminal_link_body(length: float) -> cq.Workplane:
    beam = cq.Workplane("XY").box(
        length,
        LINK_BODY_WIDTH,
        LINK_BODY_HEIGHT,
        centered=(False, True, True),
    )
    neck = (
        cq.Workplane("XY")
        .box(PAD_CENTER_X - PAD_LENGTH / 2.0 - length, 0.008, 0.012, centered=(False, True, True))
        .translate((length, 0.0, 0.0))
    )
    return beam.union(neck).union(_root_lug())


def _end_pad_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PAD_LENGTH, PAD_WIDTH, PAD_HEIGHT, centered=(True, True, True))
        .edges("|Y")
        .fillet(0.003)
    )


def _spine_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(
        FOOT_LENGTH,
        FOOT_WIDTH,
        FOOT_HEIGHT,
        centered=(True, True, False),
    )
    spine = cq.Workplane("XY").box(
        SPINE_THICKNESS_X,
        SPINE_WIDTH_Y,
        SPINE_HEIGHT,
        centered=(True, True, False),
    )
    spine = spine.translate((0.0, 0.0, FOOT_HEIGHT))
    shoulder_neck = (
        cq.Workplane("XY")
        .box(SHOULDER_X + 0.010, LINK_BODY_WIDTH, LINK_BODY_HEIGHT + 0.006, centered=(False, True, True))
        .translate((-0.010, 0.0, SHOULDER_Z))
    )
    shoulder_clevis = _clevis_tip(SHOULDER_X)
    shoulder_clevis = shoulder_clevis.translate((0.0, 0.0, SHOULDER_Z))
    return foot.union(spine).union(shoulder_neck).union(shoulder_clevis)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_folding_arm_chain")

    model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("dark_hardware", rgba=(0.34, 0.35, 0.38, 1.0))
    model.material("rubber_pad", rgba=(0.12, 0.13, 0.14, 1.0))

    spine = model.part("ground_spine")
    spine.visual(
        mesh_from_cadquery(_spine_shape(), "ground_spine"),
        material="graphite",
        name="spine_body",
    )
    spine.inertial = Inertial.from_geometry(
        Box((FOOT_LENGTH, FOOT_WIDTH, SPINE_HEIGHT + FOOT_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (SPINE_HEIGHT + FOOT_HEIGHT) / 2.0)),
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_link_with_clevis(LINK_1_LENGTH), "link_1"),
        material="alloy",
        name="link_body",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_1_LENGTH, OUTER_JOINT_WIDTH, CLEVIS_HEIGHT)),
        mass=0.80,
        origin=Origin(xyz=(LINK_1_LENGTH / 2.0, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_link_with_clevis(LINK_2_LENGTH), "link_2"),
        material="dark_hardware",
        name="link_body",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_2_LENGTH, OUTER_JOINT_WIDTH, CLEVIS_HEIGHT)),
        mass=0.66,
        origin=Origin(xyz=(LINK_2_LENGTH / 2.0, 0.0, 0.0)),
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(_terminal_link_body(LINK_3_LENGTH), "link_3_body"),
        material="alloy",
        name="terminal_body",
    )
    link_3.visual(
        mesh_from_cadquery(_end_pad_shape(), "link_3_pad"),
        origin=Origin(xyz=(PAD_CENTER_X, 0.0, 0.0)),
        material="rubber_pad",
        name="end_pad",
    )
    link_3.inertial = Inertial.from_geometry(
        Box((PAD_CENTER_X + PAD_LENGTH / 2.0, PAD_WIDTH, PAD_HEIGHT)),
        mass=0.52,
        origin=Origin(xyz=((PAD_CENTER_X + PAD_LENGTH / 2.0) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "spine_to_link_1",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=link_1,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=26.0, velocity=1.2),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.20, upper=1.85, effort=20.0, velocity=1.5),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=1.90, effort=14.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("ground_spine")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    shoulder = object_model.get_articulation("spine_to_link_1")
    elbow = object_model.get_articulation("link_1_to_link_2")
    wrist = object_model.get_articulation("link_2_to_link_3")

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
    pivot_proxy_reason = (
        "Each revolute joint is represented as a compact captured clevis-and-lug pivot "
        "without explicit pin holes or clearance hardware, so the child lug is allowed "
        "to occupy the parent clevis envelope while exact joint support and motion are "
        "verified separately."
    )
    ctx.allow_overlap(spine, link_1, reason=pivot_proxy_reason)
    ctx.allow_overlap(link_1, link_2, reason=pivot_proxy_reason)
    ctx.allow_overlap(link_2, link_3, reason=pivot_proxy_reason)
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all arm joints use parallel pitch axes",
        shoulder.axis == elbow.axis == wrist.axis == (0.0, -1.0, 0.0),
        details=f"axes={(shoulder.axis, elbow.axis, wrist.axis)}",
    )

    ctx.expect_contact(link_1, spine, name="first link is supported by the grounded spine clevis")
    ctx.expect_contact(link_2, link_1, name="second link hub bears against the first link clevis")
    ctx.expect_contact(link_3, link_2, name="terminal link hub bears against the second link clevis")

    def pad_center_z() -> float | None:
        aabb = ctx.part_element_world_aabb(link_3, elem="end_pad")
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_pad_z = pad_center_z()
    with ctx.pose({shoulder: 0.95}):
        shoulder_pad_z = pad_center_z()
    with ctx.pose({elbow: 0.95}):
        elbow_pad_z = pad_center_z()
    with ctx.pose({wrist: 0.95}):
        wrist_pad_z = pad_center_z()

    ctx.check(
        "positive shoulder motion raises the end pad",
        rest_pad_z is not None and shoulder_pad_z is not None and shoulder_pad_z > rest_pad_z + 0.12,
        details=f"rest={rest_pad_z}, shoulder_up={shoulder_pad_z}",
    )
    ctx.check(
        "positive elbow motion raises the end pad",
        rest_pad_z is not None and elbow_pad_z is not None and elbow_pad_z > rest_pad_z + 0.09,
        details=f"rest={rest_pad_z}, elbow_up={elbow_pad_z}",
    )
    ctx.check(
        "positive wrist motion raises the end pad",
        rest_pad_z is not None and wrist_pad_z is not None and wrist_pad_z > rest_pad_z + 0.05,
        details=f"rest={rest_pad_z}, wrist_up={wrist_pad_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
