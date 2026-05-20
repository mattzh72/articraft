from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_RADIUS = 0.29
BASE_THICKNESS = 0.06
COLUMN_SIZE = (0.24, 0.20, 0.92)
SHOULDER_ORIGIN = (0.38, 0.0, 1.02)
SHOULDER_GAP = 0.070
SHOULDER_CHEEK_THICKNESS = 0.018
UPPER_LENGTH = 0.76
UPPER_ROOT_HUB_RADIUS = 0.060
UPPER_ROOT_HUB_WIDTH = SHOULDER_GAP
ELBOW_GAP = 0.060
ELBOW_CHEEK_WIDTH = 0.092

FOREARM_LENGTH = 0.55
FOREARM_ROOT_HUB_RADIUS = 0.052
FOREARM_ROOT_HUB_WIDTH = ELBOW_GAP
WRIST_GAP = 0.044
WRIST_FORK_WIDTH = 0.080


def box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def make_pedestal_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    plinth = cq.Workplane("XY").circle(0.17).extrude(0.04).translate((0.0, 0.0, BASE_THICKNESS))
    column = (
        cq.Workplane("XY")
        .box(*COLUMN_SIZE, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    shoulder_backbone = box_at((0.17, 0.11, 0.12), (0.16, 0.0, 0.95))
    underside_rib = (
        cq.Workplane("XZ")
        .polyline([(0.00, 0.87), (0.08, 0.87), (0.24, 0.95), (0.00, 1.01)])
        .close()
        .extrude(0.08, both=True)
    )
    cheek_y = SHOULDER_GAP / 2.0 + SHOULDER_CHEEK_THICKNESS / 2.0
    side_spine_left = box_at((0.17, SHOULDER_CHEEK_THICKNESS, 0.055), (0.235, cheek_y, 0.985))
    side_spine_right = box_at((0.17, SHOULDER_CHEEK_THICKNESS, 0.055), (0.235, -cheek_y, 0.985))
    cheek_left = box_at((0.070, SHOULDER_CHEEK_THICKNESS, 0.18), (0.345, cheek_y, SHOULDER_ORIGIN[2]))
    cheek_right = box_at((0.070, SHOULDER_CHEEK_THICKNESS, 0.18), (0.345, -cheek_y, SHOULDER_ORIGIN[2]))
    crown = box_at((0.08, 0.10, 0.05), (0.23, 0.0, 1.055))
    return (
        base
        .union(plinth)
        .union(column)
        .union(shoulder_backbone)
        .union(underside_rib)
        .union(side_spine_left)
        .union(side_spine_right)
        .union(cheek_left)
        .union(cheek_right)
        .union(crown)
    )


def make_upper_link_shape() -> cq.Workplane:
    root_hub = cylinder_y(0.055, UPPER_ROOT_HUB_WIDTH, (0.0, 0.0, 0.0))
    beam = box_at((0.45, 0.085, 0.145), (0.30, 0.0, 0.0))
    beam = beam.cut(box_at((0.24, 0.11, 0.060), (0.30, 0.0, 0.0)))
    mid_rib = box_at((0.17, 0.030, 0.022), (0.31, 0.0, 0.083))
    cheek_y = ELBOW_GAP / 2.0 + 0.016 / 2.0
    side_link_left = box_at((0.11, 0.016, 0.045), (0.655, cheek_y, 0.0))
    side_link_right = box_at((0.11, 0.016, 0.045), (0.655, -cheek_y, 0.0))
    elbow_cheek_left = box_at((0.060, 0.016, 0.16), (0.730, cheek_y, 0.0))
    elbow_cheek_right = box_at((0.060, 0.016, 0.16), (0.730, -cheek_y, 0.0))
    return (
        root_hub
        .union(beam)
        .union(mid_rib)
        .union(side_link_left)
        .union(side_link_right)
        .union(elbow_cheek_left)
        .union(elbow_cheek_right)
    )


def make_forearm_shape() -> cq.Workplane:
    root_hub = cylinder_y(0.048, FOREARM_ROOT_HUB_WIDTH, (0.0, 0.0, 0.0))
    beam = box_at((0.34, 0.070, 0.115), (0.24, 0.0, 0.0))
    beam = beam.cut(box_at((0.17, 0.095, 0.045), (0.24, 0.0, 0.0)))
    mid_rib = box_at((0.12, 0.026, 0.020), (0.24, 0.0, 0.066))
    cheek_y = WRIST_GAP / 2.0 + 0.018 / 2.0
    side_link_left = box_at((0.08, 0.018, 0.040), (0.460, cheek_y, 0.0))
    side_link_right = box_at((0.08, 0.018, 0.040), (0.460, -cheek_y, 0.0))
    wrist_cheek_left = box_at((0.056, 0.018, 0.10), (0.522, cheek_y, 0.0))
    wrist_cheek_right = box_at((0.056, 0.018, 0.10), (0.522, -cheek_y, 0.0))
    return (
        root_hub
        .union(beam)
        .union(mid_rib)
        .union(side_link_left)
        .union(side_link_right)
        .union(wrist_cheek_left)
        .union(wrist_cheek_right)
    )


def make_tool_flange_shape() -> cq.Workplane:
    wrist_hub = cylinder_x(0.022, WRIST_GAP, (0.0, 0.0, 0.0))
    shoulder = cylinder_x(0.020, 0.012, (0.028, 0.0, 0.0))
    flange = cylinder_x(0.055, 0.018, (0.060, 0.0, 0.0))
    pilot = cylinder_x(0.018, 0.012, (0.075, 0.0, 0.0))
    shape = wrist_hub.union(shoulder).union(flange).union(pilot)
    for y, z in ((0.026, 0.026), (0.026, -0.026), (-0.026, 0.026), (-0.026, -0.026)):
        shape = shape.cut(cylinder_x(0.0045, 0.024, (0.060, y, z)))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_service_arm")

    base_material = model.material("base_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    upper_material = model.material("upper_paint", rgba=(0.70, 0.72, 0.74, 1.0))
    forearm_material = model.material("forearm_paint", rgba=(0.60, 0.62, 0.66, 1.0))
    flange_material = model.material("flange_metal", rgba=(0.78, 0.79, 0.80, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(make_pedestal_shape(), "pedestal_service_arm_pedestal"),
        material=base_material,
        name="pedestal_shell",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(make_upper_link_shape(), "pedestal_service_arm_upper_link"),
        material=upper_material,
        name="upper_link_shell",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(make_forearm_shape(), "pedestal_service_arm_forearm"),
        material=forearm_material,
        name="forearm_shell",
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        mesh_from_cadquery(make_tool_flange_shape(), "pedestal_service_arm_tool_flange"),
        material=flange_material,
        name="tool_flange_shell",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_link,
        origin=Origin(xyz=SHOULDER_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(UPPER_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.8, lower=-2.10, upper=1.35),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=tool_flange,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=3.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    tool_flange = object_model.get_part("tool_flange")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

    ctx.allow_overlap(
        pedestal,
        upper_link,
        reason="Shoulder pivot is represented as a simplified hub captured between yoke cheeks without modeling separate bearing bores.",
    )
    ctx.allow_overlap(
        upper_link,
        forearm,
        reason="Elbow joint uses simplified captured hub geometry; the local overlap stands in for the unmodeled pin and bearing cartridge.",
    )
    ctx.allow_overlap(
        forearm,
        tool_flange,
        reason="Wrist flange root is simplified as a compact coaxial stub inside the wrist clevis rather than a fully bored pin assembly.",
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

    ctx.check(
        "expected parts present",
        {part.name for part in object_model.parts} == {"pedestal", "upper_link", "forearm", "tool_flange"},
        details=f"found parts: {[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "expected joints present",
        {joint.name for joint in object_model.articulations} == {"shoulder_joint", "elbow_joint", "wrist_joint"},
        details=f"found joints: {[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "all joints are revolute",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (shoulder_joint, elbow_joint, wrist_joint)
        ),
        details="shoulder, elbow, and wrist should all be separate revolute joints",
    )
    ctx.check(
        "shoulder and elbow axes are parallel horizontal axes",
        shoulder_joint.axis == (0.0, 1.0, 0.0) and elbow_joint.axis == (0.0, 1.0, 0.0),
        details=f"shoulder axis={shoulder_joint.axis}, elbow axis={elbow_joint.axis}",
    )
    ctx.check(
        "wrist axis follows the tool axis",
        wrist_joint.axis == (1.0, 0.0, 0.0),
        details=f"wrist axis={wrist_joint.axis}",
    )

    ctx.expect_contact(upper_link, pedestal, name="upper link seated in shoulder yoke")
    ctx.expect_contact(forearm, upper_link, name="forearm seated in elbow clevis")
    ctx.expect_contact(tool_flange, forearm, name="tool flange seated in wrist clevis")

    ctx.expect_origin_gap(upper_link, pedestal, axis="x", min_gap=0.37, max_gap=0.39, name="shoulder is offset ahead of column")
    ctx.expect_origin_gap(upper_link, pedestal, axis="z", min_gap=1.01, max_gap=1.03, name="shoulder sits near top of pedestal")
    ctx.expect_origin_gap(forearm, upper_link, axis="x", min_gap=0.75, max_gap=0.77, name="elbow lands at end of upper link")
    ctx.expect_origin_gap(tool_flange, forearm, axis="x", min_gap=0.54, max_gap=0.56, name="wrist lands at end of forearm")

    with ctx.pose({shoulder_joint: -0.45, elbow_joint: 0.75, wrist_joint: 1.1}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_working_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
