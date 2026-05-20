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


def _add_mesh_visual(part, shape, mesh_name: str, material: str, visual_name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def _base_shape() -> cq.Workplane:
    base_profile = (
        cq.Workplane("XZ")
        .moveTo(-0.012, -0.028)
        .lineTo(0.118, -0.026)
        .lineTo(0.156, -0.023)
        .lineTo(0.159, -0.017)
        .lineTo(0.126, -0.016)
        .lineTo(0.040, -0.0155)
        .lineTo(0.004, -0.0150)
        .lineTo(-0.010, -0.0160)
        .close()
        .extrude(0.026, both=True)
    )
    hinge_cheeks = (
        cq.Workplane("XY")
        .box(0.020, 0.010, 0.015)
        .translate((0.004, 0.019, -0.0075))
        .union(
            cq.Workplane("XY")
            .box(0.020, 0.010, 0.015)
            .translate((0.004, -0.019, -0.0075))
        )
    )
    rear_bridge = cq.Workplane("XY").box(0.020, 0.040, 0.006).translate((0.002, 0.0, -0.018))
    return base_profile.union(hinge_cheeks).union(rear_bridge)


def _magazine_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .moveTo(0.002, -0.010)
        .lineTo(0.114, -0.0096)
        .lineTo(0.132, -0.0086)
        .lineTo(0.142, -0.0048)
        .lineTo(0.142, -0.0012)
        .lineTo(0.124, 0.0002)
        .lineTo(0.016, 0.0002)
        .lineTo(0.004, 0.0015)
        .close()
        .extrude(0.012, both=True)
        .translate((0.0, 0.0, -0.002))
    )
    channel_cut = cq.Workplane("XY").box(0.102, 0.016, 0.0076).translate((0.070, 0.0, -0.0022))
    barrel = cq.Workplane("YZ").circle(0.0046).extrude(0.018).translate((-0.009, 0.0, 0.0))
    ear_left = cq.Workplane("XY").box(0.012, 0.004, 0.015).translate((0.004, 0.009, 0.007))
    ear_right = cq.Workplane("XY").box(0.012, 0.004, 0.015).translate((0.004, -0.009, 0.007))
    journal_left = cq.Workplane("XY").box(0.008, 0.002, 0.006).translate((0.001, 0.013, 0.0))
    journal_right = cq.Workplane("XY").box(0.008, 0.002, 0.006).translate((0.001, -0.013, 0.0))
    nose_lower = cq.Workplane("XY").box(0.011, 0.018, 0.005).translate((0.138, 0.0, -0.009))
    return (
        outer.cut(channel_cut)
        .union(barrel)
        .union(ear_left)
        .union(ear_right)
        .union(journal_left)
        .union(journal_right)
        .union(nose_lower)
    )


def _arm_shape() -> cq.Workplane:
    arm_profile = (
        cq.Workplane("XZ")
        .moveTo(0.001, -0.0012)
        .lineTo(0.034, 0.0018)
        .lineTo(0.094, 0.0038)
        .lineTo(0.128, 0.0032)
        .lineTo(0.147, 0.0012)
        .lineTo(0.149, 0.0118)
        .lineTo(0.118, 0.0162)
        .lineTo(0.056, 0.0180)
        .lineTo(0.012, 0.0158)
        .lineTo(0.000, 0.0092)
        .close()
        .extrude(0.015, both=True)
    )
    rear_barrel = cq.Workplane("YZ").circle(0.0040).extrude(0.012).translate((-0.006, 0.0, 0.0))
    thumb_pad = cq.Workplane("XY").box(0.048, 0.024, 0.0022).translate((0.080, 0.0, 0.0065))
    nose = cq.Workplane("XY").box(0.016, 0.010, 0.006).translate((0.146, 0.0, 0.003))
    return arm_profile.union(rear_barrel).union(thumb_pad).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_stapler")

    model.material("body_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("brushed_metal", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("accent_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _base_shape(), "stapler_base", "body_dark", "base_shell")

    magazine = model.part("magazine")
    _add_mesh_visual(magazine, _magazine_shape(), "stapler_magazine", "brushed_metal", "magazine_body")

    arm = model.part("arm")
    _add_mesh_visual(arm, _arm_shape(), "stapler_arm", "body_dark", "arm_shell")

    follower = model.part("follower")
    follower.visual(
        Box((0.010, 0.013, 0.004)),
        origin=Origin(xyz=(0.005, 0.0, -0.002)),
        material="accent_black",
        name="follower_block",
    )
    follower.visual(
        Box((0.003, 0.006, 0.0055)),
        origin=Origin(xyz=(0.0015, 0.0, 0.00125)),
        material="accent_black",
        name="follower_tab",
    )

    anvil = model.part("anvil")
    anvil.visual(
        Box((0.014, 0.008, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.00075)),
        material="brushed_metal",
        name="anvil_plate",
    )
    anvil.visual(
        Cylinder(radius=0.0018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material="brushed_metal",
        name="anvil_pivot",
    )

    model.articulation(
        "base_to_magazine",
        ArticulationType.REVOLUTE,
        parent=base,
        child=magazine,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )
    model.articulation(
        "magazine_to_arm",
        ArticulationType.REVOLUTE,
        parent=magazine,
        child=arm,
        origin=Origin(xyz=(0.004, 0.0, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(56.0),
        ),
    )
    model.articulation(
        "magazine_to_follower",
        ArticulationType.PRISMATIC,
        parent=magazine,
        child=follower,
        origin=Origin(xyz=(0.018, 0.0, -0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.12,
            lower=0.0,
            upper=0.078,
        ),
    )
    model.articulation(
        "base_to_anvil",
        ArticulationType.REVOLUTE,
        parent=base,
        child=anvil,
        origin=Origin(xyz=(0.136, 0.0, -0.0154)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def _part_max_z(ctx: TestContext, part_name: str) -> float | None:
    aabb = ctx.part_world_aabb(part_name)
    if aabb is None:
        return None
    return aabb[1][2]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    magazine = object_model.get_part("magazine")
    arm = object_model.get_part("arm")
    follower = object_model.get_part("follower")
    anvil = object_model.get_part("anvil")
    magazine_hinge = object_model.get_articulation("base_to_magazine")
    arm_hinge = object_model.get_articulation("magazine_to_arm")
    follower_slide = object_model.get_articulation("magazine_to_follower")
    anvil_pivot = object_model.get_articulation("base_to_anvil")

    ctx.allow_overlap(
        base,
        magazine,
        reason="The rear base cheeks and magazine hinge journals are intentionally represented as an interleaved hinge assembly.",
    )
    ctx.allow_overlap(
        base,
        anvil,
        reason="The anvil pivot pin is intentionally represented as seated into the solid base proxy.",
    )

    ctx.expect_overlap(
        magazine,
        base,
        axes="x",
        min_overlap=0.110,
        name="closed magazine spans the base length",
    )
    ctx.expect_overlap(
        arm,
        magazine,
        axes="x",
        min_overlap=0.110,
        name="top arm spans the magazine channel",
    )
    ctx.expect_within(
        follower,
        magazine,
        axes="yz",
        margin=0.0015,
        name="follower stays centered in the magazine channel",
    )
    ctx.expect_overlap(
        follower,
        magazine,
        axes="x",
        min_overlap=0.010,
        name="follower remains inserted in the magazine",
    )

    magazine_limits = magazine_hinge.motion_limits
    arm_limits = arm_hinge.motion_limits
    follower_limits = follower_slide.motion_limits
    anvil_limits = anvil_pivot.motion_limits

    closed_magazine_top = _part_max_z(ctx, "magazine")
    if magazine_limits is not None and magazine_limits.upper is not None:
        with ctx.pose({magazine_hinge: magazine_limits.upper}):
            opened_magazine_top = _part_max_z(ctx, "magazine")
        ctx.check(
            "magazine opens upward from the base",
            closed_magazine_top is not None
            and opened_magazine_top is not None
            and opened_magazine_top > closed_magazine_top + 0.045,
            details=f"closed_top={closed_magazine_top}, open_top={opened_magazine_top}",
        )

    closed_arm_top = _part_max_z(ctx, "arm")
    if arm_limits is not None and arm_limits.upper is not None:
        with ctx.pose({arm_hinge: arm_limits.upper}):
            opened_arm_top = _part_max_z(ctx, "arm")
        ctx.check(
            "top arm opens upward from the magazine",
            closed_arm_top is not None
            and opened_arm_top is not None
            and opened_arm_top > closed_arm_top + 0.035,
            details=f"closed_top={closed_arm_top}, open_top={opened_arm_top}",
        )

    follower_rest = ctx.part_world_position(follower)
    if follower_limits is not None and follower_limits.upper is not None:
        with ctx.pose({follower_slide: follower_limits.upper}):
            follower_extended = ctx.part_world_position(follower)
            ctx.expect_within(
                follower,
                magazine,
                axes="yz",
                margin=0.0015,
                name="advanced follower stays centered in the channel",
            )
            ctx.expect_overlap(
                follower,
                magazine,
                axes="x",
                min_overlap=0.010,
                name="advanced follower remains inside the magazine",
            )
        ctx.check(
            "follower slides toward the stapler nose",
            follower_rest is not None
            and follower_extended is not None
            and follower_extended[0] > follower_rest[0] + 0.05,
            details=f"rest={follower_rest}, extended={follower_extended}",
        )

    anvil_rest = ctx.part_world_position(anvil)
    if anvil_limits is not None and anvil_limits.upper is not None:
        with ctx.pose({anvil_pivot: anvil_limits.upper / 2.0}):
            anvil_mid = ctx.part_world_position(anvil)
        ctx.check(
            "anvil pivots in place at the nose",
            anvil_rest is not None
            and anvil_mid is not None
            and abs(anvil_mid[0] - anvil_rest[0]) < 1e-6
            and abs(anvil_mid[1] - anvil_rest[1]) < 1e-6
            and abs(anvil_rest[0] - 0.136) < 0.01,
            details=f"rest={anvil_rest}, mid={anvil_mid}",
        )

    return ctx.report()


object_model = build_object_model()
