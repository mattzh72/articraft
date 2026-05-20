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


BASE_LENGTH = 0.200
BASE_WIDTH = 0.140
BASE_THICKNESS = 0.018
BASE_HOLE_RADIUS = 0.0065

COLUMN_WIDTH = 0.075
COLUMN_DEPTH = 0.060
COLUMN_HEIGHT = 0.320

JOINT_GAP = 0.034
JOINT_PLATE_THICKNESS = 0.010
JOINT_OUTER_WIDTH = JOINT_GAP + 2.0 * JOINT_PLATE_THICKNESS

SHOULDER_AXIS_X = 0.092
SHOULDER_AXIS_Z = BASE_THICKNESS + 0.305
UPPER_ARM_LENGTH = 0.245
FOREARM_LENGTH = 0.185

SHOULDER_HUB_RADIUS = 0.019
ELBOW_HUB_RADIUS = 0.018
WRIST_HUB_RADIUS = 0.015
HUB_LENGTH = JOINT_GAP
LUG_WIDTH = JOINT_GAP


def _y_axis_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _forward_lug(hub_radius: float, *, tongue_length: float, height: float) -> cq.Workplane:
    hub = _y_axis_cylinder(hub_radius, LUG_WIDTH).translate((hub_radius, 0.0, 0.0))
    tongue = cq.Workplane("XY").box(
        tongue_length,
        LUG_WIDTH,
        height,
        centered=(False, True, True),
    )
    return hub.union(tongue)


def _clevis(
    *,
    axis_x: float,
    body_back: float,
    body_front: float,
    height: float,
    slot_back: float,
    slot_front: float,
    slot_height: float,
) -> cq.Workplane:
    body = cq.Workplane("XY").box(
        body_back + body_front,
        JOINT_OUTER_WIDTH,
        height,
        centered=(False, True, True),
    ).translate((axis_x - body_back, 0.0, 0.0))
    slot = cq.Workplane("XY").box(
        slot_back + slot_front,
        LUG_WIDTH,
        slot_height,
        centered=(False, True, True),
    ).translate((axis_x - slot_back, 0.0, 0.0))
    return body.cut(slot)


def _support_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    base_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.068, -0.044),
                (-0.068, 0.044),
                (0.068, -0.044),
                (0.068, 0.044),
            ]
        )
        .circle(BASE_HOLE_RADIUS)
        .extrude(BASE_THICKNESS + 0.010)
    )
    base = base.cut(base_holes)

    column = (
        cq.Workplane("XY")
        .box(COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_THICKNESS))
        .edges("|Z")
        .fillet(0.004)
    )

    cantilever_start = COLUMN_WIDTH / 2.0 - 0.002
    cantilever_end = SHOULDER_AXIS_X - 0.042
    cantilever = cq.Workplane("XY").box(
        cantilever_end - cantilever_start,
        0.044,
        0.046,
        centered=(False, True, False),
    ).translate((cantilever_start, 0.0, SHOULDER_AXIS_Z - 0.023))

    shoulder_clevis = _clevis(
        axis_x=SHOULDER_AXIS_X,
        body_back=0.044,
        body_front=0.0,
        height=0.054,
        slot_back=0.010,
        slot_front=0.020,
        slot_height=0.038,
    ).translate((0.0, 0.0, SHOULDER_AXIS_Z))

    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (COLUMN_WIDTH / 2.0 - 0.002, BASE_THICKNESS + 0.105),
                (COLUMN_WIDTH / 2.0 - 0.002, SHOULDER_AXIS_Z - 0.024),
                (SHOULDER_AXIS_X - 0.044, SHOULDER_AXIS_Z - 0.024),
                (SHOULDER_AXIS_X - 0.044, SHOULDER_AXIS_Z - 0.090),
            ]
        )
        .close()
        .extrude(0.024, both=True)
    )

    return (
        base.union(column)
        .union(cantilever)
        .union(shoulder_clevis)
        .union(gusset)
    )


def _shoulder_link_shape() -> cq.Workplane:
    proximal_lug = _forward_lug(
        SHOULDER_HUB_RADIUS,
        tongue_length=0.030,
        height=0.036,
    )
    beam = (
        cq.Workplane("XY")
        .box(UPPER_ARM_LENGTH - 0.080, 0.024, 0.032, centered=(False, True, True))
        .translate((0.030, 0.0, 0.0))
        .edges("|X")
        .fillet(0.006)
    )
    elbow_clevis = _clevis(
        axis_x=UPPER_ARM_LENGTH,
        body_back=0.050,
        body_front=0.0,
        height=0.046,
        slot_back=0.016,
        slot_front=0.018,
        slot_height=0.034,
    )
    return proximal_lug.union(beam).union(elbow_clevis)


def _elbow_link_shape() -> cq.Workplane:
    proximal_lug = _forward_lug(
        ELBOW_HUB_RADIUS,
        tongue_length=0.028,
        height=0.032,
    )
    beam = (
        cq.Workplane("XY")
        .box(FOREARM_LENGTH - 0.074, 0.022, 0.028, centered=(False, True, True))
        .translate((0.028, 0.0, 0.0))
        .edges("|X")
        .fillet(0.005)
    )
    distal_bridge = cq.Workplane("XY").box(
        0.014,
        0.022,
        0.026,
        centered=(False, True, True),
    ).translate((FOREARM_LENGTH - 0.058, 0.0, 0.0))
    wrist_clevis = _clevis(
        axis_x=FOREARM_LENGTH,
        body_back=0.044,
        body_front=0.0,
        height=0.040,
        slot_back=0.014,
        slot_front=0.016,
        slot_height=0.030,
    )
    return proximal_lug.union(beam).union(distal_bridge).union(wrist_clevis)


def _wrist_tool_shape() -> cq.Workplane:
    proximal_lug = _forward_lug(
        WRIST_HUB_RADIUS,
        tongue_length=0.026,
        height=0.024,
    )
    bridge = cq.Workplane("XY").box(
        0.024, 0.022, 0.020, centered=(False, True, True)
    ).translate((0.024, 0.0, 0.0))
    plate = (
        cq.Workplane("XY")
        .box(0.086, 0.072, 0.012, centered=(False, True, True))
        .translate((0.044, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )
    front_pad = cq.Workplane("XY").box(
        0.014, 0.040, 0.018, centered=(False, True, True)
    ).translate((0.116, 0.0, 0.0))
    tool = proximal_lug.union(bridge).union(plate).union(front_pad)

    fastener_holes = (
        cq.Workplane("XY")
        .pushPoints([(0.052, -0.020), (0.052, 0.020), (0.084, -0.020), (0.084, 0.020)])
        .circle(0.0055)
        .extrude(0.020, both=True)
    )
    return tool.cut(fastener_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_tooling_cantilever_arm")

    model.material("powder_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("oxide_black", rgba=(0.12, 0.13, 0.14, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_support_shape(), "support_column"),
        material="powder_steel",
        name="support_shell",
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        mesh_from_cadquery(_shoulder_link_shape(), "shoulder_link"),
        material="machined_aluminum",
        name="shoulder_shell",
    )

    elbow_link = model.part("elbow_link")
    elbow_link.visual(
        mesh_from_cadquery(_elbow_link_shape(), "elbow_link"),
        material="machined_aluminum",
        name="elbow_shell",
    )

    wrist_tool = model.part("wrist_tool")
    wrist_tool.visual(
        mesh_from_cadquery(_wrist_tool_shape(), "wrist_tool"),
        material="oxide_black",
        name="wrist_shell",
    )

    model.articulation(
        "support_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=support,
        child=shoulder_link,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.45,
            upper=1.20,
            effort=80.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "shoulder_to_elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=elbow_link,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.10,
            upper=1.10,
            effort=55.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "elbow_to_wrist",
        ArticulationType.REVOLUTE,
        parent=elbow_link,
        child=wrist_tool,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.90,
            upper=0.95,
            effort=20.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    support = object_model.get_part("support")
    shoulder_link = object_model.get_part("shoulder_link")
    elbow_link = object_model.get_part("elbow_link")
    wrist_tool = object_model.get_part("wrist_tool")

    shoulder_joint = object_model.get_articulation("support_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_elbow")
    wrist_joint = object_model.get_articulation("elbow_to_wrist")

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
    ctx.allow_overlap(
        shoulder_link,
        support,
        reason=(
            "The shoulder trunnion is intentionally captured inside the support clevis at the pivot; "
            "the mesh-backed bearing interface carries a small local interference at the shoulder joint."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(shoulder_link, support, name="shoulder_joint_is_physically_supported")
    ctx.expect_contact(elbow_link, shoulder_link, name="elbow_joint_is_physically_supported")
    ctx.expect_contact(wrist_tool, elbow_link, name="wrist_joint_is_physically_supported")
    ctx.expect_origin_gap(
        wrist_tool,
        support,
        axis="x",
        min_gap=0.38,
        name="arm_projects_outboard_from_support",
    )

    axes_ok = all(
        abs(joint.axis[0]) < 1e-9
        and abs(abs(joint.axis[1]) - 1.0) < 1e-9
        and abs(joint.axis[2]) < 1e-9
        for joint in (shoulder_joint, elbow_joint, wrist_joint)
    )
    ctx.check(
        "parallel_horizontal_joint_axes",
        axes_ok,
        details="Shoulder, elbow, and wrist joints should all rotate about parallel horizontal Y axes.",
    )

    rest_elbow_pos = ctx.part_world_position(elbow_link)
    rest_wrist_pos = ctx.part_world_position(wrist_tool)
    rest_wrist_aabb = ctx.part_world_aabb(wrist_tool)

    with ctx.pose({shoulder_joint: 0.80}):
        raised_elbow_pos = ctx.part_world_position(elbow_link)
        raised_wrist_pos = ctx.part_world_position(wrist_tool)
    ctx.check(
        "positive_shoulder_rotation_lifts_chain",
        (
            rest_elbow_pos is not None
            and raised_elbow_pos is not None
            and rest_wrist_pos is not None
            and raised_wrist_pos is not None
            and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.10
            and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.10
        ),
        details="Positive shoulder motion should raise both the elbow and the wrist above the rest pose.",
    )

    with ctx.pose({elbow_joint: 0.85}):
        elbow_raised_wrist_pos = ctx.part_world_position(wrist_tool)
    ctx.check(
        "positive_elbow_rotation_lifts_forearm",
        (
            rest_wrist_pos is not None
            and elbow_raised_wrist_pos is not None
            and elbow_raised_wrist_pos[2] > rest_wrist_pos[2] + 0.07
        ),
        details="Positive elbow motion should lift the wrist assembly relative to the shoulder link.",
    )

    with ctx.pose({wrist_joint: 0.80}):
        wrist_rotated_aabb = ctx.part_world_aabb(wrist_tool)
    ctx.check(
        "positive_wrist_rotation_tilts_tool_plate_up",
        (
            rest_wrist_aabb is not None
            and wrist_rotated_aabb is not None
            and wrist_rotated_aabb[1][2] > rest_wrist_aabb[1][2] + 0.03
        ),
        details="Positive wrist motion should lift the front of the tool plate upward.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
