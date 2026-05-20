from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_arm_module")

    painted = model.material("painted_warm_gray", rgba=(0.63, 0.66, 0.67, 1.0))
    dark = model.material("dark_graphite", rgba=(0.06, 0.065, 0.07, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    accent = model.material("blue_slider_cover", rgba=(0.08, 0.25, 0.55, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.44, 0.32, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark,
        name="base_plate",
    )
    support.visual(
        Box((0.16, 0.15, 0.52)),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=painted,
        name="pedestal",
    )
    support.visual(
        Box((0.15, 0.18, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        material=painted,
        name="shoulder_bridge",
    )
    for y, name in ((-0.069, "shoulder_cheek_0"), (0.069, "shoulder_cheek_1")):
        support.visual(
            Box((0.16, 0.026, 0.24)),
            origin=Origin(xyz=(0.0, y, 0.68)),
            material=painted,
            name=name,
        )
        support.visual(
            Cylinder(radius=0.026, length=0.008),
            origin=Origin(xyz=(0.0, y * 1.02, 0.68), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"shoulder_bolt_{name[-1]}",
        )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.070, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shoulder_lug",
    )
    upper_link.visual(
        Box((0.46, 0.060, 0.074)),
        origin=Origin(xyz=(0.285, 0.0, 0.0)),
        material=painted,
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.13, 0.145, 0.072)),
        origin=Origin(xyz=(0.535, 0.0, 0.0)),
        material=painted,
        name="elbow_fork_bridge",
    )
    for y, name in ((-0.066, "elbow_fork_0"), (0.066, "elbow_fork_1")):
        upper_link.visual(
            Box((0.17, 0.026, 0.096)),
            origin=Origin(xyz=(0.680, y, 0.0)),
            material=painted,
            name=name,
        )
        upper_link.visual(
            Cylinder(radius=0.022, length=0.008),
            origin=Origin(xyz=(0.680, y * 1.02, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"elbow_bolt_{name[-1]}",
        )
    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.058, length=0.106),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_lug",
    )
    forelink.visual(
        Box((0.34, 0.056, 0.064)),
        origin=Origin(xyz=(0.210, 0.0, 0.0)),
        material=painted,
        name="fore_beam",
    )
    forelink.visual(
        Box((0.275, 0.090, 0.025)),
        origin=Origin(xyz=(0.4725, 0.0, 0.0445)),
        material=dark,
        name="guide_track",
    )
    for y, name in ((-0.056, "guide_rail_0"), (0.056, "guide_rail_1")):
        forelink.visual(
            Box((0.265, 0.022, 0.058)),
            origin=Origin(xyz=(0.4775, y, 0.063)),
            material=painted,
            name=name,
        )
    forelink.visual(
        Box((0.032, 0.125, 0.076)),
        origin=Origin(xyz=(0.343, 0.0, 0.043)),
        material=painted,
        name="guide_root_saddle",
    )
    end_stage = model.part("end_stage")
    end_stage.visual(
        Box((0.120, 0.064, 0.035)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=accent,
        name="carriage_block",
    )
    end_stage.visual(
        Box((0.060, 0.052, 0.042)),
        origin=Origin(xyz=(0.134, 0.0, 0.018)),
        material=dark,
        name="nose_block",
    )
    end_stage.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.176, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="end_pad",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.45, upper=1.15),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(0.680, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "terminal_slide",
        ArticulationType.PRISMATIC,
        parent=forelink,
        child=end_stage,
        origin=Origin(xyz=(0.430, 0.0, 0.0745)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.160),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    slide = object_model.get_articulation("terminal_slide")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    end_stage = object_model.get_part("end_stage")

    ctx.check(
        "two revolute joints and terminal slider",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"types={[shoulder.articulation_type, elbow.articulation_type, slide.articulation_type]}",
    )
    ctx.expect_contact(
        end_stage,
        forelink,
        elem_a="carriage_block",
        elem_b="guide_track",
        contact_tol=0.001,
        name="slider rides on guide track",
    )
    ctx.expect_overlap(
        end_stage,
        forelink,
        axes="xy",
        elem_a="carriage_block",
        elem_b="guide_track",
        min_overlap=0.055,
        name="slider is fully guided at rest",
    )

    rest_fore = ctx.part_world_position(forelink)
    rest_end = ctx.part_world_position(end_stage)
    with ctx.pose({shoulder: 0.55}):
        raised_fore = ctx.part_world_position(forelink)
    with ctx.pose({elbow: 0.55}):
        raised_end_by_elbow = ctx.part_world_position(end_stage)
    with ctx.pose({slide: 0.160}):
        extended_end = ctx.part_world_position(end_stage)
        ctx.expect_contact(
            end_stage,
            forelink,
            elem_a="carriage_block",
            elem_b="guide_track",
            contact_tol=0.001,
            name="extended slider remains on guide track",
        )
        ctx.expect_overlap(
            end_stage,
            forelink,
            axes="xy",
            elem_a="carriage_block",
            elem_b="guide_track",
            min_overlap=0.020,
            name="extended slider remains retained in guide",
        )

    ctx.check(
        "shoulder raises forelink",
        rest_fore is not None and raised_fore is not None and raised_fore[2] > rest_fore[2] + 0.18,
        details=f"rest={rest_fore}, raised={raised_fore}",
    )
    ctx.check(
        "elbow raises terminal stage",
        rest_end is not None
        and raised_end_by_elbow is not None
        and raised_end_by_elbow[2] > rest_end[2] + 0.12,
        details=f"rest={rest_end}, raised={raised_end_by_elbow}",
    )
    ctx.check(
        "terminal slider extends along arm",
        rest_end is not None and extended_end is not None and extended_end[0] > rest_end[0] + 0.14,
        details=f"rest={rest_end}, extended={extended_end}",
    )

    upper_aabb = ctx.part_world_aabb(upper_link)
    fore_aabb = ctx.part_world_aabb(forelink)
    end_aabb = ctx.part_world_aabb(end_stage)
    if upper_aabb is not None and fore_aabb is not None and end_aabb is not None:
        upper_len = upper_aabb[1][0] - upper_aabb[0][0]
        fore_len = fore_aabb[1][0] - fore_aabb[0][0]
        end_len = end_aabb[1][0] - end_aabb[0][0]
        end_height = end_aabb[1][2] - end_aabb[0][2]
        fore_height = fore_aabb[1][2] - fore_aabb[0][2]
        ctx.check(
            "end stage remains smaller than main links",
            end_len < min(upper_len, fore_len) * 0.45 and end_height < fore_height * 0.75,
            details=f"upper_len={upper_len}, fore_len={fore_len}, end_len={end_len}, fore_height={fore_height}, end_height={end_height}",
        )
    else:
        ctx.fail("end stage remains smaller than main links", "world AABB unavailable")

    return ctx.report()


object_model = build_object_model()
