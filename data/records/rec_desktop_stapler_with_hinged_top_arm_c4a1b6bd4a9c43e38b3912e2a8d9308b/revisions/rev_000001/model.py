from __future__ import annotations

import math

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


BASE_LENGTH = 0.168
BASE_WIDTH = 0.048
HINGE_X = -0.067
HINGE_Z = 0.032
ARM_LENGTH = 0.160
ARM_WIDTH = 0.052
FOLLOWER_TRAVEL = 0.016
ANVIL_PIVOT_X = 0.072
ANVIL_PIVOT_Z = -0.0005


def _make_base_shape() -> cq.Workplane:
    side_profile = [
        (-0.082, 0.003),
        (0.072, 0.003),
        (0.082, 0.004),
        (0.088, 0.0065),
        (0.084, 0.0095),
        (0.040, 0.0115),
        (-0.024, 0.0135),
        (-0.064, 0.0168),
        (-0.080, 0.0185),
        (-0.082, 0.0162),
    ]
    body = (
        cq.Workplane("XZ")
        .polyline(side_profile)
        .close()
        .extrude(BASE_WIDTH / 2.0, both=True)
        .edges("|Y")
        .fillet(0.0022)
    )

    foot_offset = 0.015
    foot = cq.Workplane("XY").box(0.148, 0.010, 0.003, centered=(True, True, False))
    body = body.union(foot.translate((0.0, foot_offset, 0.0)))
    body = body.union(foot.translate((0.0, -foot_offset, 0.0)))

    ear = cq.Workplane("XY").box(0.012, 0.006, 0.018, centered=(True, True, False))
    body = body.union(ear.translate((HINGE_X - 0.001, 0.018, 0.014)))
    body = body.union(ear.translate((HINGE_X - 0.001, -0.018, 0.014)))

    nose_pad = cq.Workplane("XY").box(0.022, 0.024, 0.003, centered=(True, True, False))
    body = body.union(nose_pad.translate((0.071, 0.0, 0.0082)))

    return body


def _make_arm_shape() -> cq.Workplane:
    outer_profile = [
        (0.014, 0.000),
        (0.055, -0.001),
        (0.118, -0.004),
        (0.152, -0.008),
        (0.160, -0.003),
        (0.156, 0.006),
        (0.126, 0.011),
        (0.062, 0.019),
        (0.022, 0.024),
        (0.014, 0.023),
    ]
    shell = (
        cq.Workplane("XZ")
        .polyline(outer_profile)
        .close()
        .extrude(ARM_WIDTH / 2.0, both=True)
        .edges("|Y")
        .fillet(0.0020)
    )

    bridge = cq.Workplane("XY").box(0.020, 0.018, 0.014, centered=(False, True, True)).translate(
        (0.0, 0.0, 0.006)
    )
    hinge_barrel = cq.Workplane("XZ").circle(0.006).extrude(0.015, both=True)
    nose_block = cq.Workplane("XY").box(0.016, 0.030, 0.010, centered=(False, True, True)).translate(
        (0.142, 0.0, 0.001)
    )

    channel_cut = cq.Workplane("XY").box(
        0.122,
        0.026,
        0.010,
        centered=(False, True, False),
    ).translate((0.026, 0.0, -0.006))
    exit_slot = cq.Workplane("XY").box(
        0.018,
        0.012,
        0.011,
        centered=(False, True, False),
    ).translate((0.138, 0.0, -0.008))

    shell = shell.union(bridge).union(hinge_barrel).union(nose_block)
    shell = shell.cut(channel_cut).cut(exit_slot)
    return shell


def _make_follower_shape() -> cq.Workplane:
    rod = cq.Workplane("XY").box(0.086, 0.006, 0.004, centered=(False, True, True)).translate(
        (0.006, 0.0, 0.0)
    )
    pusher = cq.Workplane("XY").box(0.012, 0.020, 0.006, centered=(False, True, True)).translate(
        (0.090, 0.0, 0.0)
    )
    thumb = cq.Workplane("XY").box(0.012, 0.020, 0.008, centered=(False, True, True)).translate(
        (-0.004, 0.0, 0.001)
    )
    cap = cq.Workplane("YZ").circle(0.004).extrude(0.006, both=True).translate((-0.001, 0.0, 0.001))
    return rod.union(pusher).union(thumb).union(cap)


def _make_anvil_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.018, 0.010, 0.0025, centered=(True, True, True)).translate(
        (0.002, 0.0, -0.0014)
    )
    pivot = cq.Workplane("XY").circle(0.0036).extrude(0.004, both=True).translate((0.0, 0.0, -0.0005))
    selector = cq.Workplane("XY").box(0.006, 0.004, 0.0025, centered=(False, True, True)).translate(
        (0.006, 0.006, -0.0014)
    )
    return plate.union(pivot).union(selector)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_stapler")

    model.material("die_cast_gray", rgba=(0.40, 0.42, 0.46, 1.0))
    model.material("satin_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("control_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("polished_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_shell"),
        material="die_cast_gray",
        name="base_shell",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_make_arm_shape(), "arm_shell"),
        material="satin_graphite",
        name="arm_shell",
    )

    follower = model.part("follower")
    follower.visual(
        mesh_from_cadquery(_make_follower_shape(), "follower_shell"),
        material="control_black",
        name="follower_shell",
    )

    anvil = model.part("anvil")
    anvil.visual(
        mesh_from_cadquery(_make_anvil_shape(), "anvil_shell"),
        material="polished_steel",
        name="anvil_shell",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )

    model.articulation(
        "arm_to_follower",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=follower,
        origin=Origin(xyz=(0.026, 0.0, -0.0015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=FOLLOWER_TRAVEL,
        ),
    )

    model.articulation(
        "base_to_anvil",
        ArticulationType.REVOLUTE,
        parent=base,
        child=anvil,
        origin=Origin(xyz=(ANVIL_PIVOT_X, 0.0, ANVIL_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    follower = object_model.get_part("follower")
    anvil = object_model.get_part("anvil")

    arm_joint = object_model.get_articulation("base_to_arm")
    follower_joint = object_model.get_articulation("arm_to_follower")
    anvil_joint = object_model.get_articulation("base_to_anvil")

    ctx.allow_overlap(
        base,
        arm,
        elem_a="base_shell",
        elem_b="arm_shell",
        reason="The rear hinge is modeled as a captured knuckle volume without separate pin holes, so the hinge ears and arm barrel intentionally share simplified hinge space.",
    )
    ctx.allow_overlap(
        arm,
        follower,
        elem_a="arm_shell",
        elem_b="follower_shell",
        reason="The staple magazine is simplified as an enclosed channel proxy around the translating follower assembly.",
    )

    with ctx.pose({arm_joint: 0.0, follower_joint: 0.0, anvil_joint: 0.0}):
        ctx.expect_overlap(
            arm,
            base,
            axes="xy",
            min_overlap=0.030,
            name="arm covers the base footprint when closed",
        )
        ctx.expect_within(
            follower,
            arm,
            axes="yz",
            margin=0.0015,
            name="follower stays centered in the staple channel",
        )
        ctx.expect_overlap(
            follower,
            arm,
            axes="x",
            min_overlap=0.100,
            name="follower remains inserted at rest",
        )
        ctx.expect_overlap(
            anvil,
            base,
            axes="xy",
            min_overlap=0.010,
            name="anvil sits under the stapling nose in closed clinch",
        )
        base_aabb = ctx.part_world_aabb(base)
        arm_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
        closed_height = None
        if base_aabb is not None and arm_aabb is not None:
            closed_height = max(base_aabb[1][2], arm_aabb[1][2]) - min(base_aabb[0][2], arm_aabb[0][2])
        ctx.check(
            "closed stapler keeps a dense desktop silhouette",
            closed_height is not None and 0.040 <= closed_height <= 0.058,
            details=f"closed_height={closed_height}",
        )

    arm_limits = arm_joint.motion_limits
    if arm_limits is not None and arm_limits.upper is not None:
        closed_arm_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
        with ctx.pose({arm_joint: arm_limits.upper}):
            open_arm_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
        ctx.check(
            "arm opens upward from the rear hinge",
            closed_arm_aabb is not None
            and open_arm_aabb is not None
            and open_arm_aabb[1][2] > closed_arm_aabb[1][2] + 0.035,
            details=f"closed={closed_arm_aabb}, open={open_arm_aabb}",
        )

    follower_limits = follower_joint.motion_limits
    if follower_limits is not None and follower_limits.upper is not None:
        rest_follower_pos = ctx.part_world_position(follower)
        with ctx.pose({follower_joint: follower_limits.upper}):
            ctx.expect_within(
                follower,
                arm,
                axes="yz",
                margin=0.0015,
                name="follower stays guided at maximum travel",
            )
            ctx.expect_overlap(
                follower,
                arm,
                axes="x",
                min_overlap=0.092,
                name="follower stays retained in the channel at maximum travel",
            )
            advanced_follower_pos = ctx.part_world_position(follower)
        ctx.check(
            "follower translates forward inside the arm",
            rest_follower_pos is not None
            and advanced_follower_pos is not None
            and advanced_follower_pos[0] > rest_follower_pos[0] + 0.012,
            details=f"rest={rest_follower_pos}, advanced={advanced_follower_pos}",
        )

    anvil_limits = anvil_joint.motion_limits
    if anvil_limits is not None and anvil_limits.upper is not None:
        with ctx.pose({anvil_joint: 0.0}):
            anvil_closed_aabb = ctx.part_element_world_aabb(anvil, elem="anvil_shell")
        with ctx.pose({anvil_joint: anvil_limits.upper}):
            ctx.expect_overlap(
                anvil,
                base,
                axes="xy",
                min_overlap=0.010,
                name="anvil stays under the nose in open clinch",
            )
            anvil_open_aabb = ctx.part_element_world_aabb(anvil, elem="anvil_shell")

        def _span(aabb, axis_index: int) -> float | None:
            if aabb is None:
                return None
            return float(aabb[1][axis_index] - aabb[0][axis_index])

        closed_x = _span(anvil_closed_aabb, 0)
        closed_y = _span(anvil_closed_aabb, 1)
        open_x = _span(anvil_open_aabb, 0)
        open_y = _span(anvil_open_aabb, 1)
        ctx.check(
            "anvil rotates between closed and open clinch orientations",
            closed_x is not None
            and closed_y is not None
            and open_x is not None
            and open_y is not None
            and closed_x > closed_y + 0.003
            and open_y > open_x + 0.003,
            details=(
                f"closed_xy=({closed_x}, {closed_y}), "
                f"open_xy=({open_x}, {open_y})"
            ),
        )

    return ctx.report()


object_model = build_object_model()
