from __future__ import annotations

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


BODY_WIDTH = 0.192
HEAD_WIDTH = 0.142
HINGE_X = -0.074
HINGE_Z = 0.305
BOWL_CENTER_X = 0.094
BOWL_SEAT_Z = 0.122
LOCK_X = -0.070
LOCK_Y = 0.059
LOCK_Z = 0.286


def _outline_solid(
    points: list[tuple[float, float]],
    *,
    width: float,
    fillet: float,
) -> cq.Workplane:
    solid = cq.Workplane("XZ").polyline(points).close().extrude(width / 2.0, both=True)
    if fillet <= 0.0:
        return solid
    return solid


def _body_shell() -> cq.Workplane:
    base = cq.Workplane("XY").center(0.012, 0.0).rect(0.304, 0.210).extrude(0.028)
    pedestal = cq.Workplane("XY").center(BOWL_CENTER_X, 0.0).ellipse(0.080, 0.062).extrude(0.112)
    bridge_points = [
        (-0.066, 0.028),
        (0.022, 0.028),
        (0.040, 0.070),
        (0.012, 0.104),
        (-0.030, 0.110),
        (-0.060, 0.086),
    ]
    bridge = _outline_solid(bridge_points, width=0.148, fillet=0.0)
    column_points = [
        (-0.118, 0.028),
        (-0.072, 0.028),
        (-0.062, 0.116),
        (-0.060, 0.192),
        (-0.066, 0.254),
        (-0.078, 0.304),
        (-0.072, 0.324),
        (-0.056, 0.318),
        (-0.054, 0.278),
        (-0.050, 0.214),
        (-0.048, 0.142),
        (-0.062, 0.072),
        (-0.094, 0.028),
    ]
    column = _outline_solid(column_points, width=0.118, fillet=0.0)
    return base.union(pedestal).union(bridge).union(column)


def _head_shell() -> cq.Workplane:
    head_points = [
        (0.012, 0.026),
        (0.024, 0.052),
        (0.055, 0.070),
        (0.108, 0.079),
        (0.158, 0.076),
        (0.194, 0.060),
        (0.214, 0.031),
        (0.219, 0.004),
        (0.209, -0.022),
        (0.181, -0.038),
        (0.132, -0.047),
        (0.078, -0.047),
        (0.030, -0.039),
        (0.012, -0.006),
    ]
    head = _outline_solid(head_points, width=HEAD_WIDTH, fillet=0.014)
    drive_fairing = cq.Workplane("XY").center(0.138, 0.0).circle(0.022).extrude(-0.030)
    return head.union(drive_fairing)


def _bowl_shell() -> cq.Workplane:
    profile = [
        (0.050, 0.016),
        (0.072, 0.032),
        (0.102, 0.087),
        (0.112, 0.147),
        (0.116, 0.166),
        (0.108, 0.166),
        (0.104, 0.154),
        (0.096, 0.090),
        (0.066, 0.036),
        (0.044, 0.024),
    ]
    return cq.Workplane("XZ").polyline(profile).close().revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def _beater_shape() -> cq.Workplane:
    outer = cq.Workplane("XZ").center(0.0, -0.068).rect(0.068, 0.084).extrude(0.005, both=True)
    inner = cq.Workplane("XZ").center(0.0, -0.074).rect(0.038, 0.046).extrude(0.006, both=True)
    spine = cq.Workplane("XZ").center(0.0, -0.054).rect(0.016, 0.060).extrude(0.0055, both=True)
    frame = outer.cut(inner).union(spine)
    return frame


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_mixer")

    enamel = model.material("enamel", rgba=(0.78, 0.16, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.86, 0.88, 0.91, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    trim = model.material("trim", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "mixer_body_shell"),
        material=enamel,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.068, length=0.012),
        origin=Origin(xyz=(BOWL_CENTER_X, 0.0, 0.116)),
        material=brushed_steel,
        name="seat_ring",
    )
    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell(), "mixer_head_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.023, length=0.028),
        origin=Origin(xyz=(0.170, 0.0, -0.034)),
        material=trim,
        name="drive_housing",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shell(), "mixer_bowl_shell"),
        material=steel,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.058, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=brushed_steel,
        name="bowl_foot",
    )
    for idx, angle_deg in enumerate((0.0, 120.0, 240.0)):
        angle = math.radians(angle_deg)
        bowl.visual(
            Box((0.022, 0.010, 0.008)),
            origin=Origin(
                xyz=(0.062 * math.cos(angle), 0.062 * math.sin(angle), 0.008),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"bowl_tab_{idx}",
        )

    rear_lock = model.part("rear_lock")
    rear_lock.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="rear_lock_pivot",
    )
    rear_lock.visual(
        Box((0.014, 0.008, 0.008)),
        origin=Origin(xyz=(-0.010, 0.0025, 0.004)),
        material=trim,
        name="rear_lock_bridge",
    )
    rear_lock.visual(
        Box((0.020, 0.009, 0.010)),
        origin=Origin(xyz=(-0.026, 0.0045, 0.006)),
        material=trim,
        name="rear_lock_handle",
    )
    rear_lock.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(-0.040, 0.0045, 0.010)),
        material=trim,
        name="rear_lock_tip",
    )

    attachment = model.part("attachment")
    attachment.visual(
        Cylinder(radius=0.006, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=steel,
        name="drive_shaft",
    )
    attachment.visual(
        mesh_from_cadquery(_beater_shape(), "mixer_beater_frame"),
        material=steel,
        name="beater_frame",
    )

    model.articulation(
        "body_to_head",
        ArticulationType.REVOLUTE,
        parent=body,
        child=head,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "body_to_bowl",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bowl,
        origin=Origin(xyz=(BOWL_CENTER_X, 0.0, BOWL_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(24.0),
        ),
    )
    model.articulation(
        "body_to_rear_lock",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_lock,
        origin=Origin(xyz=(LOCK_X, LOCK_Y, LOCK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "head_to_attachment",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=attachment,
        origin=Origin(xyz=(0.170, 0.0, -0.050)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    head = object_model.get_part("head")
    bowl = object_model.get_part("bowl")
    rear_lock = object_model.get_part("rear_lock")
    attachment = object_model.get_part("attachment")
    head_joint = object_model.get_articulation("body_to_head")
    bowl_joint = object_model.get_articulation("body_to_bowl")
    lock_joint = object_model.get_articulation("body_to_rear_lock")
    drive_joint = object_model.get_articulation("head_to_attachment")

    ctx.allow_overlap(
        body,
        head,
        elem_a="body_shell",
        elem_b="head_shell",
        reason="The rear tilt-head hinge is represented with simplified close-fitting fairings that nest around the pivot.",
    )
    ctx.allow_overlap(
        body,
        rear_lock,
        elem_a="body_shell",
        elem_b="rear_lock_pivot",
        reason="The rear lock lever uses a short captured pivot stem embedded in the pedestal boss.",
    )
    ctx.allow_overlap(
        body,
        rear_lock,
        elem_a="body_shell",
        elem_b="rear_lock_bridge",
        reason="The rear lock lever's hidden inner bridge is simplified as running inside the captured pedestal pocket.",
    )

    ctx.expect_gap(
        bowl,
        body,
        axis="z",
        positive_elem="bowl_foot",
        negative_elem="seat_ring",
        max_gap=0.0015,
        max_penetration=0.0,
        name="bowl foot sits on the bayonet seat",
    )
    ctx.expect_overlap(
        bowl,
        body,
        axes="xy",
        elem_a="bowl_foot",
        elem_b="seat_ring",
        min_overlap=0.100,
        name="bowl foot remains centered on the seat ring",
    )
    ctx.expect_within(
        attachment,
        bowl,
        axes="xy",
        inner_elem="beater_frame",
        outer_elem="bowl_shell",
        margin=0.010,
        name="beater footprint stays inside the bowl footprint",
    )

    rest_beater = _aabb_center(ctx.part_element_world_aabb(attachment, elem="beater_frame"))
    with ctx.pose({head_joint: head_joint.motion_limits.upper}):
        lifted_beater = _aabb_center(ctx.part_element_world_aabb(attachment, elem="beater_frame"))
    ctx.check(
        "head tilts the attachment upward from the bowl",
        rest_beater is not None
        and lifted_beater is not None
        and lifted_beater[2] > rest_beater[2] + 0.075,
        details=f"rest={rest_beater}, lifted={lifted_beater}",
    )

    rest_tab = _aabb_center(ctx.part_element_world_aabb(bowl, elem="bowl_tab_0"))
    with ctx.pose({bowl_joint: bowl_joint.motion_limits.upper}):
        turned_tab = _aabb_center(ctx.part_element_world_aabb(bowl, elem="bowl_tab_0"))
        ctx.expect_gap(
            bowl,
            body,
            axis="z",
            positive_elem="bowl_foot",
            negative_elem="seat_ring",
            max_gap=0.0015,
            max_penetration=0.0,
            name="bowl stays seated while twisted",
        )
    ctx.check(
        "bowl tab sweeps around the bayonet seat",
        rest_tab is not None
        and turned_tab is not None
        and abs(turned_tab[1] - rest_tab[1]) > 0.020,
        details=f"rest={rest_tab}, turned={turned_tab}",
    )

    rest_lock_tip = _aabb_center(ctx.part_element_world_aabb(rear_lock, elem="rear_lock_tip"))
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        moved_lock_tip = _aabb_center(ctx.part_element_world_aabb(rear_lock, elem="rear_lock_tip"))
    ctx.check(
        "rear lock lever rotates behind the hinge",
        rest_lock_tip is not None
        and moved_lock_tip is not None
        and moved_lock_tip[2] > rest_lock_tip[2] + 0.010,
        details=f"rest={rest_lock_tip}, moved={moved_lock_tip}",
    )

    drive_limits = drive_joint.motion_limits
    ctx.check(
        "attachment uses a continuous drive articulation",
        drive_joint.articulation_type == ArticulationType.CONTINUOUS
        and drive_limits is not None
        and drive_limits.lower is None
        and drive_limits.upper is None
        and abs(float(drive_joint.axis[2])) > 0.9,
        details=(
            f"type={drive_joint.articulation_type}, axis={drive_joint.axis}, "
            f"limits={drive_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
