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


BASE_L = 0.160
BASE_W = 0.100
BASE_H = 0.018

COLUMN_L = 0.026
COLUMN_W = 0.030
COLUMN_H = 0.072

JOINT1_AXIS_Z = 0.111
PED_EAR_X = 0.016
PED_EAR_T = 0.010
PED_EAR_Z = 0.046
PED_EAR_CY = 0.015
PED_EAR_INNER_GAP = 0.020

LINK1_LEN = 0.220
LINK1_HEIGHT = 0.032
LINK1_BODY_W = 0.018
LINK1_BODY_X = 0.111
LINK1_BODY_L = 0.170
LINK1_WINDOW_L = 0.108
LINK1_WINDOW_H = 0.016
LINK1_ROOT_NECK_L = 0.024
LINK1_ROOT_NECK_H = 0.018
LINK1_ROOT_JOURNAL_R = 0.013
LINK1_ROOT_JOURNAL_LEN = PED_EAR_INNER_GAP
LINK1_TIP_YOKE_L = 0.026
LINK1_TIP_YOKE_W = 0.030
LINK1_TIP_YOKE_H = 0.030
LINK1_TIP_SLOT_W = 0.018

LINK2_HEIGHT = 0.028
LINK2_BODY_W = 0.016
LINK2_BODY_X = 0.089
LINK2_BODY_L = 0.118
LINK2_WINDOW_L = 0.060
LINK2_WINDOW_H = 0.012
LINK2_ROOT_NECK_L = 0.026
LINK2_ROOT_NECK_H = 0.016
LINK2_ROOT_JOURNAL_R = 0.011
LINK2_ROOT_JOURNAL_LEN = LINK1_TIP_SLOT_W
LINK2_NECK_L = 0.022
LINK2_NECK_W = 0.010
LINK2_NECK_H = 0.014
END_TAB_L = 0.022
END_TAB_T = 0.010
END_TAB_H = 0.018
END_TAB_HOLE_R = 0.0035
END_TAB_X = 0.168


def _pedestal_shape():
    base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_H, centered=(True, True, False))
    column = (
        cq.Workplane("XY")
        .box(COLUMN_L, COLUMN_W, COLUMN_H, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_H))
    )
    ear_pos = cq.Workplane("XY").box(PED_EAR_X, PED_EAR_T, PED_EAR_Z).translate(
        (0.0, PED_EAR_CY, JOINT1_AXIS_Z - 0.016)
    )
    ear_neg = cq.Workplane("XY").box(PED_EAR_X, PED_EAR_T, PED_EAR_Z).translate(
        (0.0, -PED_EAR_CY, JOINT1_AXIS_Z - 0.016)
    )

    return base.union(column).union(ear_pos).union(ear_neg)


def _link1_shape():
    body = cq.Workplane("XY").box(LINK1_BODY_L, LINK1_BODY_W, LINK1_HEIGHT).translate((LINK1_BODY_X, 0.0, 0.0))
    body_window = (
        cq.Workplane("XY")
        .box(LINK1_WINDOW_L, LINK1_BODY_W + 0.004, LINK1_WINDOW_H)
        .translate((0.112, 0.0, 0.0))
    )
    body = body.cut(body_window)

    root_journal = cq.Workplane("XZ").circle(LINK1_ROOT_JOURNAL_R).extrude(LINK1_ROOT_JOURNAL_LEN / 2.0, both=True)
    root_neck = cq.Workplane("XY").box(LINK1_ROOT_NECK_L, LINK1_BODY_W, LINK1_ROOT_NECK_H).translate(
        (0.021, 0.0, 0.0)
    )

    tip_yoke = cq.Workplane("XY").box(LINK1_TIP_YOKE_L, LINK1_TIP_YOKE_W, LINK1_TIP_YOKE_H).translate(
        (LINK1_LEN - LINK1_TIP_YOKE_L / 2.0, 0.0, 0.0)
    )
    tip_slot = cq.Workplane("XY").box(LINK1_TIP_YOKE_L + 0.002, LINK1_TIP_SLOT_W, LINK1_TIP_YOKE_H + 0.004).translate(
        (LINK1_LEN - LINK1_TIP_YOKE_L / 2.0, 0.0, 0.0)
    )
    tip_yoke = tip_yoke.cut(tip_slot)

    return body.union(root_journal).union(root_neck).union(tip_yoke)


def _link2_shape():
    body = cq.Workplane("XY").box(LINK2_BODY_L, LINK2_BODY_W, LINK2_HEIGHT).translate((LINK2_BODY_X, 0.0, 0.0))
    body_window = (
        cq.Workplane("XY")
        .box(LINK2_WINDOW_L, LINK2_BODY_W + 0.004, LINK2_WINDOW_H)
        .translate((0.088, 0.0, 0.0))
    )
    body = body.cut(body_window)

    root_journal = cq.Workplane("XZ").circle(LINK2_ROOT_JOURNAL_R).extrude(LINK2_ROOT_JOURNAL_LEN / 2.0, both=True)
    root_neck = cq.Workplane("XY").box(LINK2_ROOT_NECK_L, LINK2_BODY_W, LINK2_ROOT_NECK_H).translate(
        (0.018, 0.0, 0.0)
    )
    neck = cq.Workplane("XY").box(LINK2_NECK_L, LINK2_NECK_W, LINK2_NECK_H).translate(
        (0.152, 0.0, 0.0)
    )
    end_tab = (
        cq.Workplane("XZ")
        .center(END_TAB_X, 0.0)
        .slot2D(END_TAB_L, END_TAB_H, angle=0.0)
        .extrude(END_TAB_T / 2.0, both=True)
    )
    tab_hole = (
        cq.Workplane("XZ")
        .center(END_TAB_X, 0.0)
        .circle(END_TAB_HOLE_R)
        .extrude(END_TAB_T, both=True)
    )

    return body.union(root_journal).union(root_neck).union(neck).union(end_tab).cut(tab_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_revolute_chain")

    model.material("pedestal_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("link_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("link_aluminum_dark", rgba=(0.60, 0.63, 0.67, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal_shell"),
        material="pedestal_steel",
        name="pedestal_shell",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, JOINT1_AXIS_Z + PED_EAR_Z / 2.0)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, (JOINT1_AXIS_Z + PED_EAR_Z / 2.0) / 2.0)),
    )

    link1 = model.part("link1")
    link1.visual(
        mesh_from_cadquery(_link1_shape(), "link1_frame"),
        material="link_aluminum",
        name="link1_frame",
    )
    link1.inertial = Inertial.from_geometry(
        Box((LINK1_LEN + 0.018, 0.030, LINK1_HEIGHT)),
        mass=0.85,
        origin=Origin(xyz=((LINK1_LEN + 0.018) / 2.0, 0.0, 0.0)),
    )

    link2 = model.part("link2")
    link2.visual(
        mesh_from_cadquery(_link2_shape(), "link2_frame"),
        material="link_aluminum_dark",
        name="link2_frame",
    )
    link2.inertial = Inertial.from_geometry(
        Box((0.188, 0.024, LINK2_HEIGHT)),
        mass=0.48,
        origin=Origin(xyz=(0.094, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_link1",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, JOINT1_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.15, effort=30.0, velocity=1.3),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.00, upper=1.05, effort=20.0, velocity=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    shoulder = object_model.get_articulation("pedestal_to_link1")
    elbow = object_model.get_articulation("link1_to_link2")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        pedestal,
        link1,
        reason="Root revolute joint is authored as a nested clevis-and-journal abstraction without separate hole clearance solids.",
    )
    ctx.allow_overlap(
        link1,
        link2,
        reason="Elbow revolute joint is authored as a nested yoke-and-journal abstraction without a separately modeled hinge pin and bushing clearance.",
    )

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

    ctx.expect_contact(pedestal, link1, name="pedestal_supports_first_joint")
    ctx.expect_contact(link1, link2, name="first_link_supports_second_joint")

    ctx.check(
        "parallel_revolute_axes",
        tuple(shoulder.axis) == tuple(elbow.axis) == (0.0, -1.0, 0.0),
        details=f"shoulder axis={shoulder.axis}, elbow axis={elbow.axis}",
    )

    link1_aabb = ctx.part_world_aabb(link1)
    link2_aabb = ctx.part_world_aabb(link2)
    if link1_aabb is None or link2_aabb is None:
        ctx.fail("links_step_down_toward_tip", "missing AABB for one or more links")
    else:
        link1_size = tuple(link1_aabb[1][i] - link1_aabb[0][i] for i in range(3))
        link2_size = tuple(link2_aabb[1][i] - link2_aabb[0][i] for i in range(3))
        ctx.check(
            "links_step_down_toward_tip",
            link1_size[0] > link2_size[0] and link1_size[1] > link2_size[1] and link1_size[2] > link2_size[2],
            details=f"link1 size={link1_size}, link2 size={link2_size}",
        )

    with ctx.pose({shoulder: 0.55, elbow: 0.70}):
        link2_pos = ctx.part_world_position(link2)
        link2_aabb_open = ctx.part_world_aabb(link2)
        if link2_pos is None or link2_aabb_open is None:
            ctx.fail("positive_joint_motion_lifts_chain", "missing posed link2 measurement")
        else:
            ctx.check(
                "positive_joint_motion_lifts_chain",
                link2_pos[0] > 0.15 and link2_pos[2] > JOINT1_AXIS_Z + 0.08 and link2_aabb_open[1][2] > link2_pos[2] + 0.05,
                details=f"posed link2 position={link2_pos}, posed link2 aabb={link2_aabb_open}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
