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


BASE_LENGTH = 0.34
BASE_WIDTH = 0.24
BASE_HEIGHT = 0.03

POST_RADIUS = 0.035
POST_HEIGHT = 0.86

SUPPORT_PAD_RADIUS = 0.04
SUPPORT_PAD_THICKNESS = 0.02
SUPPORT_PAD_CENTER_X = 0.165
SHOULDER_JOINT_X = SUPPORT_PAD_CENTER_X + SUPPORT_PAD_RADIUS
SUPPORT_PAD_TOP_Z = 0.76

LINK_PAD_THICKNESS = 0.02
JOINT_STANDOFF_HEIGHT = 0.04
JOINT_STANDOFF_RADIUS = 0.022
SHOULDER_ROOT_RADIUS = 0.04
SHOULDER_END_RADIUS = 0.05
FORELINK_ROOT_RADIUS = 0.046
FORELINK_END_RADIUS = 0.048
WRIST_ROOT_RADIUS = 0.043

SHOULDER_LENGTH = 0.38
FORELINK_LENGTH = 0.32


def _rounded_prism(length: float, width: float, height: float, fillet: float) -> cq.Workplane:
    prism = cq.Workplane("XY").rect(length, width).extrude(height)
    if fillet > 0.0:
        prism = prism.edges("|Z").fillet(fillet)
    return prism


def _make_support_shape() -> cq.Workplane:
    base = _rounded_prism(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, 0.012)

    post = (
        cq.Workplane("XY")
        .circle(POST_RADIUS)
        .extrude(POST_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )

    shoulder_arm = _rounded_prism(0.16, 0.06, 0.028, 0.008).translate((0.09, 0.0, 0.69))

    shoulder_riser = _rounded_prism(0.056, 0.062, 0.05, 0.006).translate((0.137, 0.0, 0.715))

    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (POST_RADIUS * 0.8, 0.585),
                (0.102, 0.585),
                (0.152, 0.725),
                (POST_RADIUS * 0.8, 0.725),
            ]
        )
        .close()
        .extrude(0.052)
        .translate((0.0, -0.026, 0.0))
    )

    support_pad = (
        cq.Workplane("XY")
        .circle(SUPPORT_PAD_RADIUS)
        .extrude(SUPPORT_PAD_THICKNESS)
        .translate(
            (
                SUPPORT_PAD_CENTER_X,
                0.0,
                SUPPORT_PAD_TOP_Z - SUPPORT_PAD_THICKNESS,
            )
        )
    )

    shoulder_mount = _rounded_prism(0.02, 0.078, 0.02, 0.004).translate(
        (SHOULDER_JOINT_X - 0.01, 0.0, SUPPORT_PAD_TOP_Z - 0.01)
    )

    cap = (
        cq.Workplane("XY")
        .circle(POST_RADIUS * 1.08)
        .extrude(0.012)
        .translate((0.0, 0.0, BASE_HEIGHT + POST_HEIGHT - 0.012))
    )

    return (
        base.union(post)
        .union(shoulder_arm)
        .union(shoulder_riser)
        .union(gusset)
        .union(support_pad)
        .union(shoulder_mount)
        .union(cap)
    )


def _make_shoulder_shape() -> cq.Workplane:
    root_plate = _rounded_prism(0.08, 0.078, LINK_PAD_THICKNESS, 0.006).translate((0.04, 0.0, 0.0))

    root_housing = (
        cq.Workplane("XY")
        .circle(0.03)
        .extrude(0.028)
        .translate((0.03, 0.0, 0.0))
    )

    beam = _rounded_prism(0.27, 0.07, 0.04, 0.007).translate((0.195, 0.0, 0.008))

    spine = _rounded_prism(0.22, 0.046, 0.014, 0.004).translate((0.205, 0.0, 0.028))

    end_disk = (
        cq.Workplane("XY")
        .circle(SHOULDER_END_RADIUS)
        .extrude(LINK_PAD_THICKNESS)
        .translate((SHOULDER_LENGTH, 0.0, 0.0))
    )

    end_web = _rounded_prism(0.05, 0.056, 0.024, 0.004).translate((0.355, 0.0, 0.012))

    end_boss = (
        cq.Workplane("XY")
        .circle(JOINT_STANDOFF_RADIUS)
        .extrude(JOINT_STANDOFF_HEIGHT)
        .translate((SHOULDER_LENGTH, 0.0, LINK_PAD_THICKNESS))
    )

    return root_plate.union(root_housing).union(beam).union(spine).union(end_disk).union(end_web).union(end_boss)


def _make_link_shape(
    *,
    length: float,
    beam_width: float,
    beam_height: float,
    root_radius: float,
    end_radius: float,
    root_hole_radius: float = 0.0,
) -> cq.Workplane:
    root_disk = cq.Workplane("XY").circle(root_radius).extrude(LINK_PAD_THICKNESS)
    if root_hole_radius > 0.0:
        root_disk = root_disk.cut(
            cq.Workplane("XY").circle(root_hole_radius).extrude(LINK_PAD_THICKNESS)
        )
    end_disk = (
        cq.Workplane("XY")
        .circle(end_radius)
        .extrude(LINK_PAD_THICKNESS)
        .translate((length, 0.0, 0.0))
    )

    beam_length = length - 0.45 * (root_radius + end_radius)
    beam = (
        _rounded_prism(beam_length, beam_width, beam_height, 0.007)
        .translate((length * 0.5, 0.0, 0.008))
    )

    top_cap = (
        _rounded_prism(beam_length * 0.88, beam_width * 0.72, 0.012, 0.004)
        .translate((length * 0.5, 0.0, LINK_PAD_THICKNESS + 0.016))
    )

    end_boss = (
        cq.Workplane("XY")
        .circle(JOINT_STANDOFF_RADIUS)
        .extrude(JOINT_STANDOFF_HEIGHT)
        .translate((length, 0.0, LINK_PAD_THICKNESS))
    )

    return root_disk.union(end_disk).union(beam).union(top_cap).union(end_boss)


def _make_wrist_face_shape() -> cq.Workplane:
    pad = cq.Workplane("XY").circle(WRIST_ROOT_RADIUS).extrude(LINK_PAD_THICKNESS)

    neck = (
        _rounded_prism(0.08, 0.05, 0.034, 0.005)
        .translate((0.048, 0.0, 0.01))
    )

    rib = (
        cq.Workplane("XZ")
        .polyline([(0.018, 0.0), (0.065, 0.0), (0.082, 0.042), (0.03, 0.042)])
        .close()
        .extrude(0.028)
        .translate((0.0, -0.014, LINK_PAD_THICKNESS))
    )

    plate = (
        cq.Workplane("YZ")
        .rect(0.14, 0.18)
        .extrude(0.016)
        .edges("|X")
        .fillet(0.01)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.04, -0.05), (0.04, -0.05), (-0.04, 0.05), (0.04, 0.05)])
        .hole(0.012)
        .translate((0.078, 0.0, 0.028))
    )

    return pad.union(neck).union(rib).union(plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_post_cantilever_arm")

    model.material("support_dark", rgba=(0.19, 0.2, 0.23, 1.0))
    model.material("arm_silver", rgba=(0.73, 0.75, 0.77, 1.0))
    model.material("wrist_gray", rgba=(0.57, 0.59, 0.62, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_make_support_shape(), "support_body"),
        material="support_dark",
        name="support_body",
    )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        mesh_from_cadquery(_make_shoulder_shape(), "shoulder_link_body"),
        material="arm_silver",
        name="shoulder_body",
    )

    forelink = model.part("forelink")
    forelink.visual(
        mesh_from_cadquery(
            _make_link_shape(
                length=FORELINK_LENGTH,
                beam_width=0.066,
                beam_height=0.038,
                root_radius=FORELINK_ROOT_RADIUS,
                end_radius=FORELINK_END_RADIUS,
            ),
            "forelink_body",
        ),
        material="arm_silver",
        name="forelink_body",
    )

    wrist_face = model.part("wrist_face")
    wrist_face.visual(
        mesh_from_cadquery(_make_wrist_face_shape(), "wrist_face_body"),
        material="wrist_gray",
        name="wrist_body",
    )

    model.articulation(
        "support_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=support,
        child=shoulder,
        origin=Origin(xyz=(SHOULDER_JOINT_X, 0.0, SUPPORT_PAD_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-1.0, upper=1.0),
    )

    model.articulation(
        "shoulder_to_forelink",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forelink,
        origin=Origin(xyz=(SHOULDER_LENGTH, 0.0, LINK_PAD_THICKNESS + JOINT_STANDOFF_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.7, lower=-1.0, upper=1.0),
    )

    model.articulation(
        "forelink_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=wrist_face,
        origin=Origin(xyz=(FORELINK_LENGTH, 0.0, LINK_PAD_THICKNESS + JOINT_STANDOFF_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-1.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    shoulder = object_model.get_part("shoulder_link")
    forelink = object_model.get_part("forelink")
    wrist_face = object_model.get_part("wrist_face")

    shoulder_joint = object_model.get_articulation("support_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_forelink")
    wrist_joint = object_model.get_articulation("forelink_to_wrist")

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

    ctx.expect_contact(shoulder, support, name="support_carries_shoulder")
    ctx.expect_contact(forelink, shoulder, name="shoulder_carries_forelink")
    ctx.expect_contact(wrist_face, forelink, name="forelink_carries_wrist_face")

    ctx.expect_origin_gap(shoulder, support, axis="x", min_gap=0.08, name="shoulder_is_side_mounted")
    ctx.expect_origin_gap(forelink, support, axis="x", min_gap=0.42, name="forelink_projects_outboard")
    ctx.expect_origin_gap(wrist_face, support, axis="x", min_gap=0.68, name="wrist_face_stays_outboard")

    axes_ok = all(
        tuple(float(value) for value in joint.axis) == (0.0, 0.0, 1.0)
        for joint in (shoulder_joint, elbow_joint, wrist_joint)
    )
    ctx.check(
        "serial_joints_are_vertical_revolutes",
        axes_ok,
        "Shoulder, elbow, and wrist should all yaw about the vertical axis.",
    )

    rest_forelink_pos = ctx.part_world_position(forelink)
    with ctx.pose({shoulder_joint: 0.55}):
        shoulder_swung_pos = ctx.part_world_position(forelink)
    shoulder_motion_ok = (
        rest_forelink_pos is not None
        and shoulder_swung_pos is not None
        and shoulder_swung_pos[1] > rest_forelink_pos[1] + 0.15
    )
    ctx.check(
        "positive_shoulder_rotation_swings_chain_ccw",
        shoulder_motion_ok,
        "Positive shoulder rotation should carry the downstream chain toward +Y.",
    )

    rest_wrist_origin = ctx.part_world_position(wrist_face)
    with ctx.pose({elbow_joint: 0.55}):
        elbow_swung_pos = ctx.part_world_position(wrist_face)
    elbow_motion_ok = (
        rest_wrist_origin is not None
        and elbow_swung_pos is not None
        and elbow_swung_pos[1] > rest_wrist_origin[1] + 0.14
    )
    ctx.check(
        "positive_elbow_rotation_swings_wrist_ccw",
        elbow_motion_ok,
        "Positive elbow rotation should sweep the wrist origin toward +Y.",
    )

    rest_wrist_aabb = ctx.part_world_aabb(wrist_face)
    with ctx.pose({wrist_joint: 0.7}):
        wrist_swung_aabb = ctx.part_world_aabb(wrist_face)
    wrist_motion_ok = (
        rest_wrist_aabb is not None
        and wrist_swung_aabb is not None
        and wrist_swung_aabb[1][1] > rest_wrist_aabb[1][1] + 0.03
    )
    ctx.check(
        "positive_wrist_rotation_turns_face_ccw",
        wrist_motion_ok,
        "Positive wrist rotation should turn the short wrist face toward +Y.",
    )

    def _all_parts_stay_right_of_post(pose_map: dict[object, float]) -> bool:
        with ctx.pose(pose_map):
            for part in (shoulder, forelink, wrist_face):
                aabb = ctx.part_world_aabb(part)
                if aabb is None or aabb[0][0] <= 0.02:
                    return False
        return True

    ctx.check(
        "upper_limit_pose_remains_on_one_side_of_support",
        _all_parts_stay_right_of_post(
            {
                shoulder_joint: shoulder_joint.motion_limits.upper,
                elbow_joint: elbow_joint.motion_limits.upper,
                wrist_joint: wrist_joint.motion_limits.upper,
            }
        ),
        "At the upper joint limits the arm should still project on the positive-X side of the post.",
    )

    ctx.check(
        "lower_limit_pose_remains_on_one_side_of_support",
        _all_parts_stay_right_of_post(
            {
                shoulder_joint: shoulder_joint.motion_limits.lower,
                elbow_joint: elbow_joint.motion_limits.lower,
                wrist_joint: wrist_joint.motion_limits.lower,
            }
        ),
        "At the lower joint limits the arm should still project on the positive-X side of the post.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
