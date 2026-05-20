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


VERT_CHILD_RADIUS = 0.0105
VERT_CHILD_THICKNESS = 0.014
VERT_EAR_RADIUS = 0.0135
VERT_EAR_THICKNESS = 0.006
VERT_EAR_CENTER_Z = (VERT_CHILD_THICKNESS / 2.0) + 0.001 + (VERT_EAR_THICKNESS / 2.0)

PITCH_AXIS_Y = 0.070
PITCH_ARM_X = 0.124
PITCH_ARM_THICKNESS = 0.008
PITCH_ARM_SPAN_Y = 0.026
PITCH_ARM_HEIGHT = 0.088
PITCH_TRUNNION_RADIUS = 0.010
PITCH_TRUNNION_LENGTH = 0.016
PITCH_TRUNNION_CENTER_X = 0.112


def _combine(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(radius: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height, both=True).translate(center)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length, both=True).translate(center)


def _vertical_child_knuckle() -> cq.Workplane:
    return _cyl_z(VERT_CHILD_RADIUS, VERT_CHILD_THICKNESS, (0.0, 0.0, 0.0)).intersect(
        _box((0.040, 0.040, 0.040), (0.0, 0.010, 0.0))
    )


def _vertical_parent_ears() -> cq.Workplane:
    trim = _box((0.040, 0.040, 0.040), (0.0, -0.010, 0.0))
    top_ear = _cyl_z(VERT_EAR_RADIUS, VERT_EAR_THICKNESS, (0.0, 0.0, VERT_EAR_CENTER_Z)).intersect(trim)
    bottom_ear = _cyl_z(VERT_EAR_RADIUS, VERT_EAR_THICKNESS, (0.0, 0.0, -VERT_EAR_CENTER_Z)).intersect(trim)
    return _combine(top_ear, bottom_ear)


def _pitch_child_trunnions() -> cq.Workplane:
    right_trunnion = _cyl_x(PITCH_TRUNNION_RADIUS, PITCH_TRUNNION_LENGTH, (PITCH_TRUNNION_CENTER_X, 0.0, 0.0))
    left_trunnion = _cyl_x(PITCH_TRUNNION_RADIUS, PITCH_TRUNNION_LENGTH, (-PITCH_TRUNNION_CENTER_X, 0.0, 0.0))
    return _combine(right_trunnion, left_trunnion)


def make_wall_bracket() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.115, 0.005, 0.180)
        .edges("|Z")
        .fillet(0.0018)
        .translate((0.0, -0.0335, 0.0))
    )
    cover = (
        cq.Workplane("XY")
        .box(0.070, 0.003, 0.122)
        .edges("|Z")
        .fillet(0.0010)
        .translate((0.0, -0.0290, 0.0))
    )
    base = (
        cq.Workplane("XY")
        .box(0.086, 0.020, 0.118)
        .edges("|Z")
        .fillet(0.0045)
        .translate((0.0, -0.013, 0.0))
    )
    shoulder = (
        cq.Workplane("XY")
        .box(0.054, 0.018, 0.072)
        .edges("|Z")
        .fillet(0.0040)
        .translate((0.0, -0.014, 0.0))
    )
    hinge_ears = _vertical_parent_ears()
    rear_block = _box((0.060, 0.014, 0.050), (0.0, -0.024, 0.0))
    left_web = _box((0.008, 0.018, 0.040), (-0.017, -0.016, 0.0))
    right_web = _box((0.008, 0.018, 0.040), (0.017, -0.016, 0.0))
    hinge = _combine(hinge_ears, rear_block, left_web, right_web)
    caps = []
    for x in (-0.030, 0.030):
        for z in (-0.056, 0.056):
            caps.append(cq.Workplane("XZ").circle(0.0075).extrude(0.003).translate((x, -0.031, z)))
    return _combine(plate, cover, base, shoulder, hinge, *caps)


def make_link(length: float, side_offset: float, clip_side: float) -> cq.Workplane:
    body_width = 0.020
    body_height = 0.014
    bridge_width = 0.022
    root_bridge = _box((abs(side_offset) + bridge_width, 0.016, body_height), (side_offset / 2.0, 0.022, 0.0))
    side_member_length = max(length - 0.064, 0.070)
    side_member = _box((body_width, side_member_length, body_height), (side_offset, length / 2.0, 0.0))
    distal_bridge = _box((abs(side_offset) + bridge_width, 0.016, body_height), (side_offset / 2.0, length - 0.022, 0.0))
    distal_ears = _vertical_parent_ears().translate((0.0, length, 0.0))

    clip_pad_center = (side_offset + (0.004 * clip_side), length * 0.50, (body_height / 2.0) + 0.0015)
    clip_shell_center = (clip_pad_center[0], clip_pad_center[1], (body_height / 2.0) + 0.007)
    clip_pad = _box((0.018, 0.014, 0.003), clip_pad_center)
    clip_shell = _box((0.018, 0.014, 0.012), clip_shell_center)
    clip_void = _box((0.010, 0.018, 0.008), (clip_shell_center[0], clip_shell_center[1], clip_shell_center[2] + 0.001))
    clip = clip_shell.cut(clip_void)

    return _combine(_vertical_child_knuckle(), root_bridge, side_member, distal_bridge, distal_ears, clip_pad, clip)


def make_head_yoke() -> cq.Workplane:
    spindle = _vertical_child_knuckle()
    base = (
        cq.Workplane("XY")
        .box(0.038, 0.016, 0.042)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.018, 0.0))
    )
    rear_tie = (
        cq.Workplane("XY")
        .box(0.094, 0.010, 0.018)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.024, 0.0))
    )
    left_cheek = _box((0.016, 0.048, 0.050), (-0.106, 0.048, 0.0))
    right_cheek = _box((0.016, 0.048, 0.050), (0.106, 0.048, 0.0))
    left_arm = _box((PITCH_ARM_THICKNESS, 0.020, PITCH_ARM_HEIGHT), (-PITCH_ARM_X, 0.060, 0.0))
    right_arm = _box((PITCH_ARM_THICKNESS, 0.020, PITCH_ARM_HEIGHT), (PITCH_ARM_X, 0.060, 0.0))
    left_boss = _cyl_x(0.016, PITCH_ARM_THICKNESS, (-PITCH_ARM_X, PITCH_AXIS_Y, 0.0))
    right_boss = _cyl_x(0.016, PITCH_ARM_THICKNESS, (PITCH_ARM_X, PITCH_AXIS_Y, 0.0))
    return _combine(
        spindle,
        base,
        rear_tie,
        left_cheek,
        right_cheek,
        left_arm,
        right_arm,
        left_boss,
        right_boss,
    )


def make_vesa_frame() -> cq.Workplane:
    ring = (
        cq.Workplane("XZ")
        .rect(0.240, 0.240)
        .extrude(0.006, both=True)
        .cut(cq.Workplane("XZ").rect(0.190, 0.190).extrude(0.010, both=True))
        .translate((0.0, 0.106, 0.0))
    )
    center_plate = cq.Workplane("XZ").rect(0.112, 0.112).extrude(0.006, both=True).translate((0.0, 0.106, 0.0))
    pitch_block = _box((0.028, 0.014, 0.040), (0.0, 0.076, 0.0))
    stem = _box((0.026, 0.072, 0.020), (0.0, 0.106, 0.0))
    top_rib = _box((0.032, 0.012, 0.040), (0.0, 0.106, 0.076))
    bottom_rib = _box((0.032, 0.012, 0.040), (0.0, 0.106, -0.076))
    left_rib = _box((0.040, 0.012, 0.032), (-0.076, 0.106, 0.0))
    right_rib = _box((0.040, 0.012, 0.032), (0.076, 0.106, 0.0))
    frame = _combine(ring, center_plate, pitch_block, stem, top_rib, bottom_rib, left_rib, right_rib, _pitch_child_trunnions())

    slot_points = [
        (-0.100, -0.100),
        (0.100, -0.100),
        (-0.100, 0.100),
        (0.100, 0.100),
        (-0.050, -0.050),
        (0.050, -0.050),
        (-0.050, 0.050),
        (0.050, 0.050),
    ]
    slot_cutters = cq.Workplane("XZ").pushPoints(slot_points).slot2D(0.014, 0.006, 90).extrude(0.020, both=True)
    return frame.cut(slot_cutters)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_motion_wall_mount")

    wall_color = model.material("wall_bracket_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    link_color = model.material("link_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    head_color = model.material("head_finish", rgba=(0.16, 0.16, 0.17, 1.0))
    frame_color = model.material("frame_finish", rgba=(0.09, 0.09, 0.10, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(mesh_from_cadquery(make_wall_bracket(), "wall_bracket"), material=wall_color, name="wall_bracket_shell")
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.120, 0.050, 0.180)),
        mass=1.60,
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
    )

    link_primary = model.part("link_primary")
    link_primary.visual(
        mesh_from_cadquery(make_link(0.165, 0.030, 1.0), "link_primary"),
        material=link_color,
        name="link_primary_shell",
    )
    link_primary.inertial = Inertial.from_geometry(
        Box((0.060, 0.180, 0.050)),
        mass=0.65,
        origin=Origin(xyz=(0.015, 0.082, 0.0)),
    )

    link_secondary = model.part("link_secondary")
    link_secondary.visual(
        mesh_from_cadquery(make_link(0.155, -0.030, -1.0), "link_secondary"),
        material=link_color,
        name="link_secondary_shell",
    )
    link_secondary.inertial = Inertial.from_geometry(
        Box((0.060, 0.170, 0.050)),
        mass=0.58,
        origin=Origin(xyz=(-0.015, 0.078, 0.0)),
    )

    head_yoke = model.part("head_yoke")
    head_yoke.visual(mesh_from_cadquery(make_head_yoke(), "head_yoke"), material=head_color, name="head_yoke_shell")
    head_yoke.inertial = Inertial.from_geometry(
        Box((0.270, 0.060, 0.120)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
    )

    vesa_frame = model.part("vesa_frame")
    vesa_frame.visual(
        mesh_from_cadquery(make_vesa_frame(), "vesa_frame"),
        material=frame_color,
        name="vesa_frame_shell",
    )
    vesa_frame.inertial = Inertial.from_geometry(
        Box((0.240, 0.014, 0.240)),
        mass=0.86,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "wall_to_link1",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=link_primary,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link_primary,
        child=link_secondary,
        origin=Origin(xyz=(0.0, 0.165, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-3.0, upper=3.0),
    )
    model.articulation(
        "link2_to_head_pan",
        ArticulationType.REVOLUTE,
        parent=link_secondary,
        child=head_yoke,
        origin=Origin(xyz=(0.0, 0.155, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=head_yoke,
        child=vesa_frame,
        origin=Origin(xyz=(0.0, PITCH_AXIS_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.35, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    link_primary = object_model.get_part("link_primary")
    link_secondary = object_model.get_part("link_secondary")
    head_yoke = object_model.get_part("head_yoke")
    vesa_frame = object_model.get_part("vesa_frame")

    wall_to_link1 = object_model.get_articulation("wall_to_link1")
    link1_to_link2 = object_model.get_articulation("link1_to_link2")
    link2_to_head_pan = object_model.get_articulation("link2_to_head_pan")
    head_pitch = object_model.get_articulation("head_pitch")

    ctx.allow_overlap(
        wall_bracket,
        link_primary,
        reason="Hidden bearing bore and axle clearance are omitted from the visualized wall-side revolute joint.",
    )
    ctx.allow_overlap(
        link_primary,
        link_secondary,
        reason="The elbow joint is represented as a compact journal pair without subtracting the concealed pin bores.",
    )
    ctx.allow_overlap(
        link_secondary,
        head_yoke,
        reason="The head pan joint omits internal bushings and bore voids, so the simplified hinge solids intentionally interpenetrate.",
    )
    ctx.allow_overlap(
        head_yoke,
        vesa_frame,
        reason="The pitch trunnion pockets are represented without internal cut bores, leaving only the exterior yoke and trunnion silhouettes.",
    )

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

    for part_name in ("wall_bracket", "link_primary", "link_secondary", "head_yoke", "vesa_frame"):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    ctx.check("wall_joint_axis_vertical", tuple(wall_to_link1.axis) == (0.0, 0.0, 1.0))
    ctx.check("elbow_joint_axis_vertical", tuple(link1_to_link2.axis) == (0.0, 0.0, 1.0))
    ctx.check("pan_joint_axis_vertical", tuple(link2_to_head_pan.axis) == (0.0, 0.0, 1.0))
    ctx.check("pitch_joint_axis_horizontal", tuple(head_pitch.axis) == (1.0, 0.0, 0.0))

    ctx.expect_contact(wall_bracket, link_primary, name="wall_joint_visibly_carried")
    ctx.expect_contact(link_primary, link_secondary, name="elbow_joint_visibly_carried")
    ctx.expect_contact(link_secondary, head_yoke, name="pan_joint_visibly_carried")
    ctx.expect_contact(head_yoke, vesa_frame, name="pitch_joint_visibly_carried")
    ctx.expect_origin_gap(vesa_frame, wall_bracket, axis="y", min_gap=0.32, name="deployed_reach_from_wall")

    with ctx.pose(
        {
            wall_to_link1: 0.0,
            link1_to_link2: 1.80,
            link2_to_head_pan: 0.0,
            head_pitch: 0.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_part_overlap")
        ctx.expect_origin_gap(
            vesa_frame,
            wall_bracket,
            axis="y",
            min_gap=0.08,
            max_gap=0.15,
            name="folded_package_keeps_compact_standoff",
        )
        ctx.expect_contact(link_primary, link_secondary, name="folded_elbow_support_contact")
        ctx.expect_contact(head_yoke, vesa_frame, name="folded_pitch_support_contact")

    with ctx.pose(
        {
            wall_to_link1: 1.15,
            link1_to_link2: -1.00,
            link2_to_head_pan: 0.85,
            head_pitch: -0.20,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="side_swept_pose_no_part_overlap")
        ctx.expect_gap(
            vesa_frame,
            wall_bracket,
            axis="y",
            min_gap=0.12,
            name="side_swept_pose_retains_standoff",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
