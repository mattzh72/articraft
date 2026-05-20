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


PALM_LENGTH = 0.075
PALM_WIDTH = 0.095
PALM_HEIGHT = 0.048
CHAIN_Y = 0.028
BASE_JOINT_X = PALM_LENGTH / 2.0 + 0.0035

JOINT_RADIUS = 0.005
ROOT_HUB_LEN = 0.008
END_HUB_LEN = 0.007

PROXIMAL_LENGTH = 0.040
MIDDLE_LENGTH = 0.032
DISTAL_LENGTH = 0.026

ROOT_NECK_LENGTH = 0.010
END_BLOCK_LENGTH = 0.007


def _y_cylinder(radius: float, length: float, *, x: float, y: float, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )

def _make_palm_shape() -> cq.Workplane:
    palm = cq.Workplane("XY").box(PALM_LENGTH, PALM_WIDTH, PALM_HEIGHT)

    front_relief = (
        cq.Workplane("XY")
        .box(0.020, 0.030, 0.028)
        .translate((PALM_LENGTH / 2.0 - 0.006, 0.0, 0.0))
    )
    top_relief = (
        cq.Workplane("XY")
        .box(0.028, 0.060, 0.010)
        .translate((0.006, 0.0, PALM_HEIGHT / 2.0 - 0.005))
    )
    palm = palm.cut(front_relief).cut(top_relief)

    arm_length = BASE_JOINT_X - PALM_LENGTH / 2.0
    arm_width = 0.018
    arm_height = 0.024
    hub_center_x = BASE_JOINT_X - JOINT_RADIUS

    for side in (-1.0, 1.0):
        y0 = side * CHAIN_Y
        arm = (
            cq.Workplane("XY")
            .box(arm_length, arm_width, arm_height)
            .translate((PALM_LENGTH / 2.0 + arm_length / 2.0, y0, 0.0))
        )
        hub = _y_cylinder(
            JOINT_RADIUS,
            ROOT_HUB_LEN,
            x=hub_center_x,
            y=y0,
        )
        cheek = (
            cq.Workplane("XY")
            .box(arm_length * 0.75, arm_width * 0.82, arm_height * 0.78)
            .translate((PALM_LENGTH / 2.0 + arm_length * 0.46, y0, -0.003))
        )
        palm = palm.union(arm).union(hub).union(cheek)

    return palm


def _make_link_shape(
    *,
    length: float,
    body_width: float,
    body_height: float,
    distal_clevis: bool,
) -> cq.Workplane:
    root_hub = _y_cylinder(
        JOINT_RADIUS,
        ROOT_HUB_LEN,
        x=JOINT_RADIUS,
        y=0.0,
    )
    root_neck = (
        cq.Workplane("XY")
        .box(ROOT_NECK_LENGTH, ROOT_HUB_LEN, body_height)
        .translate((ROOT_NECK_LENGTH / 2.0, 0.0, 0.0))
    )

    if distal_clevis:
        center_section_length = length - ROOT_NECK_LENGTH - END_BLOCK_LENGTH
    else:
        center_section_length = length - ROOT_NECK_LENGTH - 0.004

    center_section = (
        cq.Workplane("XY")
        .box(center_section_length, body_width, body_height)
        .translate((ROOT_NECK_LENGTH + center_section_length / 2.0, 0.0, 0.0))
    )

    top_taper = (
        cq.Workplane("XZ")
        .moveTo(ROOT_NECK_LENGTH * 0.55, -body_height / 2.0)
        .lineTo(length - 0.006, -body_height / 2.0)
        .lineTo(length - 0.001, 0.0)
        .lineTo(length - 0.006, body_height / 2.0)
        .lineTo(ROOT_NECK_LENGTH * 0.55, body_height / 2.0)
        .close()
        .extrude(body_width * 0.70 / 2.0, both=True)
    )

    link = root_hub.union(root_neck).union(center_section).union(top_taper)

    if distal_clevis:
        end_block = (
            cq.Workplane("XY")
            .box(END_BLOCK_LENGTH, body_width * 0.92, body_height * 0.92)
            .translate((length - END_BLOCK_LENGTH / 2.0, 0.0, 0.0))
        )
        end_hub = _y_cylinder(
            JOINT_RADIUS * 0.96,
            END_HUB_LEN,
            x=length - JOINT_RADIUS * 0.96,
            y=0.0,
        )
        link = link.union(end_block).union(end_hub)
    else:
        fingertip_cap = _y_cylinder(
            body_height * 0.42,
            body_width * 0.92,
            x=length - body_height * 0.42,
            y=0.0,
        )
        fingertip_pad = (
            cq.Workplane("XY")
            .box(0.010, body_width * 0.92, body_height * 0.68)
            .translate((length - 0.005, 0.0, -body_height * 0.10))
        )
        link = link.union(fingertip_cap).union(fingertip_pad)

    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_gripper_finger_pair")

    palm_mat = model.material("palm_charcoal", color=(0.18, 0.19, 0.22, 1.0))
    finger_mat = model.material("finger_aluminum", color=(0.73, 0.75, 0.78, 1.0))

    palm = model.part("palm")
    palm.visual(
        mesh_from_cadquery(_make_palm_shape(), "palm_body"),
        material=palm_mat,
        name="shell",
    )

    left_proximal = model.part("left_proximal")
    left_proximal.visual(
        mesh_from_cadquery(
            _make_link_shape(
                length=PROXIMAL_LENGTH,
                body_width=0.012,
                body_height=0.019,
                distal_clevis=True,
            ),
            "left_proximal",
        ),
        material=finger_mat,
        name="shell",
    )

    left_middle = model.part("left_middle")
    left_middle.visual(
        mesh_from_cadquery(
            _make_link_shape(
                length=MIDDLE_LENGTH,
                body_width=0.011,
                body_height=0.017,
                distal_clevis=True,
            ),
            "left_middle",
        ),
        material=finger_mat,
        name="shell",
    )

    left_distal = model.part("left_distal")
    left_distal.visual(
        mesh_from_cadquery(
            _make_link_shape(
                length=DISTAL_LENGTH,
                body_width=0.010,
                body_height=0.015,
                distal_clevis=False,
            ),
            "left_distal",
        ),
        material=finger_mat,
        name="shell",
    )

    right_proximal = model.part("right_proximal")
    right_proximal.visual(
        mesh_from_cadquery(
            _make_link_shape(
                length=PROXIMAL_LENGTH,
                body_width=0.012,
                body_height=0.019,
                distal_clevis=True,
            ),
            "right_proximal",
        ),
        material=finger_mat,
        name="shell",
    )

    right_middle = model.part("right_middle")
    right_middle.visual(
        mesh_from_cadquery(
            _make_link_shape(
                length=MIDDLE_LENGTH,
                body_width=0.011,
                body_height=0.017,
                distal_clevis=True,
            ),
            "right_middle",
        ),
        material=finger_mat,
        name="shell",
    )

    right_distal = model.part("right_distal")
    right_distal.visual(
        mesh_from_cadquery(
            _make_link_shape(
                length=DISTAL_LENGTH,
                body_width=0.010,
                body_height=0.015,
                distal_clevis=False,
            ),
            "right_distal",
        ),
        material=finger_mat,
        name="shell",
    )

    joint_limits_base = MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.20)
    joint_limits_mid = MotionLimits(effort=4.0, velocity=3.2, lower=0.0, upper=1.35)
    joint_limits_tip = MotionLimits(effort=3.0, velocity=3.4, lower=0.0, upper=1.20)

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(BASE_JOINT_X, CHAIN_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits_base,
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits_mid,
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits_tip,
    )

    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(BASE_JOINT_X, -CHAIN_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits_base,
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits_mid,
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits_tip,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    expected_parts = {
        "palm",
        "left_proximal",
        "left_middle",
        "left_distal",
        "right_proximal",
        "right_middle",
        "right_distal",
    }
    expected_joints = {
        "palm_to_left_proximal",
        "left_proximal_to_left_middle",
        "left_middle_to_left_distal",
        "palm_to_right_proximal",
        "right_proximal_to_right_middle",
        "right_middle_to_right_distal",
    }

    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_base = object_model.get_articulation("palm_to_left_proximal")
    left_mid = object_model.get_articulation("left_proximal_to_left_middle")
    left_tip = object_model.get_articulation("left_middle_to_left_distal")
    right_base = object_model.get_articulation("palm_to_right_proximal")
    right_mid = object_model.get_articulation("right_proximal_to_right_middle")
    right_tip = object_model.get_articulation("right_middle_to_right_distal")

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
        "expected parts present",
        {part.name for part in object_model.parts} == expected_parts,
        details=f"found parts: {sorted(part.name for part in object_model.parts)}",
    )
    ctx.check(
        "expected articulations present",
        {joint.name for joint in object_model.articulations} == expected_joints,
        details=f"found joints: {sorted(joint.name for joint in object_model.articulations)}",
    )

    ctx.expect_contact(left_proximal, palm, name="left proximal mounted to palm")
    ctx.expect_contact(left_middle, left_proximal, name="left middle mounted to proximal")
    ctx.expect_contact(left_distal, left_middle, name="left distal mounted to middle")
    ctx.expect_contact(right_proximal, palm, name="right proximal mounted to palm")
    ctx.expect_contact(right_middle, right_proximal, name="right middle mounted to proximal")
    ctx.expect_contact(right_distal, right_middle, name="right distal mounted to middle")

    ctx.expect_gap(
        left_proximal,
        right_proximal,
        axis="y",
        min_gap=0.030,
        name="proximal chains stay split across the centerline",
    )
    ctx.expect_gap(
        left_distal,
        right_distal,
        axis="y",
        min_gap=0.026,
        name="distal fingertips remain separate at rest",
    )
    ctx.expect_origin_gap(
        left_distal,
        palm,
        axis="x",
        min_gap=0.090,
        name="left distal chain reaches forward of the palm",
    )
    ctx.expect_origin_gap(
        right_distal,
        palm,
        axis="x",
        min_gap=0.090,
        name="right distal chain reaches forward of the palm",
    )

    curl_pose = {
        left_base: 0.85,
        left_mid: 0.80,
        left_tip: 0.65,
        right_base: 0.85,
        right_mid: 0.80,
        right_tip: 0.65,
    }
    with ctx.pose(curl_pose):
        ctx.expect_origin_gap(
            left_distal,
            palm,
            axis="z",
            min_gap=0.035,
            name="left chain curls upward for grasping",
        )
        ctx.expect_origin_gap(
            right_distal,
            palm,
            axis="z",
            min_gap=0.035,
            name="right chain curls upward for grasping",
        )
        ctx.expect_gap(
            left_distal,
            right_distal,
            axis="y",
            min_gap=0.020,
            name="finger chains stay uncoupled in the curled pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
