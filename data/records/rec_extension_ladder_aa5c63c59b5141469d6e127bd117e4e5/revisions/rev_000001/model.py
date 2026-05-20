from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_section_extension_ladder")

    aluminum = Material("brushed_aluminium", rgba=(0.72, 0.76, 0.76, 1.0))
    worn_edge = Material("slightly_worn_aluminium", rgba=(0.86, 0.88, 0.86, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    latch_yellow = Material("safety_yellow", rgba=(0.95, 0.69, 0.10, 1.0))
    dark_pin = Material("dark_steel", rgba=(0.08, 0.08, 0.075, 1.0))

    base = model.part("base_section")

    base_rail_len = 2.50
    base_rail_center_z = 1.30
    base_x = 0.25
    base_depth = 0.045
    # Each side rail is drawn as an aluminium C-channel: an outside web with
    # front/back lips projecting inward, not just a solid rectangular post.
    for side, sign in (("left", -1.0), ("right", 1.0)):
        outer_x = sign * (base_x + 0.025)
        lip_center_x = sign * base_x
        base.visual(
            Box((0.010, base_depth, base_rail_len)),
            origin=Origin(xyz=(outer_x, 0.0, base_rail_center_z)),
            material=aluminum,
            name=f"{side}_rail_web",
        )
        for y_name, y in (("front", -0.0185), ("rear", 0.0185)):
            base.visual(
                Box((0.050, 0.008, base_rail_len)),
                origin=Origin(xyz=(lip_center_x, y, base_rail_center_z)),
                material=worn_edge,
                name=f"{side}_rail_{y_name}_lip",
            )
        base.visual(
            Box((0.105, 0.080, 0.050)),
            origin=Origin(xyz=(sign * base_x, 0.0, 0.025)),
            material=rubber,
            name=f"{side}_foot_pad",
        )

    for idx, z in enumerate((0.32, 0.62, 0.92, 1.22, 1.52, 1.82, 2.12, 2.42)):
        base.visual(
            Cylinder(radius=0.014, length=0.520),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name=f"base_rung_{idx}",
        )
        base.visual(
            Cylinder(radius=0.018, length=0.075),
            origin=Origin(xyz=(-0.250, 0.0, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=worn_edge,
            name=f"base_rung_collar_{idx}_0",
        )
        base.visual(
            Cylinder(radius=0.018, length=0.075),
            origin=Origin(xyz=(0.250, 0.0, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=worn_edge,
            name=f"base_rung_collar_{idx}_1",
        )

    # Fixed guide sleeves on the base section.  They are open toward the ladder
    # centerline so the fly rungs can pass; the outer web and front/rear cheeks
    # form channel pockets around the sliding fly rails.
    fly_x = 0.17
    fly_y = -0.055
    for level, zc in (("lower", 1.75), ("upper", 2.35)):
        for side, sign in (("left", -1.0), ("right", 1.0)):
            sleeve_outer_x = sign * (fly_x + 0.035)
            flange_center_x = sign * (fly_x + 0.006)
            bridge_center_x = sign * 0.215
            base.visual(
                Box((0.012, 0.066, 0.180)),
                origin=Origin(xyz=(sleeve_outer_x, fly_y, zc)),
                material=dark_pin,
                name=f"{side}_{level}_sleeve_web",
            )
            for y_name, y_off in (("front", -0.028), ("rear", 0.028)):
                base.visual(
                    Box((0.058, 0.010, 0.180)),
                    origin=Origin(xyz=(flange_center_x, fly_y + y_off, zc)),
                    material=dark_pin,
                    name=f"{side}_{level}_sleeve_{y_name}_cheek",
                )
            base.visual(
                Box((0.028, 0.072, 0.180)),
                origin=Origin(xyz=(bridge_center_x, -0.052, zc)),
                material=dark_pin,
                name=f"{side}_{level}_sleeve_bridge",
            )

    # Small fixed rope pulley and rung-lock catches make the assembly read like
    # an actual extension ladder without adding secondary articulations.
    base.visual(
        Cylinder(radius=0.042, length=0.020),
        origin=Origin(xyz=(0.285, -0.080, 2.52), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_pin,
        name="top_rope_pulley",
    )
    base.visual(
        Box((0.120, 0.018, 0.030)),
        origin=Origin(xyz=(0.285, -0.076, 2.52)),
        material=dark_pin,
        name="pulley_cross_pin",
    )
    base.visual(
        Box((0.035, 0.090, 0.160)),
        origin=Origin(xyz=(0.285, -0.040, 2.455)),
        material=dark_pin,
        name="pulley_yoke",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        base.visual(
            Box((0.055, 0.020, 0.085)),
            origin=Origin(xyz=(sign * 0.245, -0.075, 1.66), rpy=(0.0, 0.0, sign * 0.18)),
            material=latch_yellow,
            name=f"{side}_rung_lock",
        )
        base.visual(
            Box((0.035, 0.080, 0.200)),
            origin=Origin(xyz=(sign * 0.245, -0.035, 1.57)),
            material=dark_pin,
            name=f"{side}_lock_pivot_plate",
        )

    fly = model.part("fly_section")

    fly_rail_len = 2.45
    fly_rail_center_z = fly_rail_len / 2.0
    fly_depth = 0.035
    for side, sign in (("left", -1.0), ("right", 1.0)):
        outer_x = sign * (fly_x + 0.0245)
        lip_center_x = sign * fly_x
        fly.visual(
            Box((0.009, fly_depth, fly_rail_len)),
            origin=Origin(xyz=(outer_x, 0.0, fly_rail_center_z)),
            material=aluminum,
            name=f"{side}_rail_web",
        )
        for y_name, y in (("front", -0.0145), ("rear", 0.0145)):
            fly.visual(
                Box((0.040, 0.007, fly_rail_len)),
                origin=Origin(xyz=(lip_center_x, y, fly_rail_center_z)),
                material=worn_edge,
                name=f"{side}_rail_{y_name}_lip",
            )
        fly.visual(
            Box((0.050, 0.048, 0.050)),
            origin=Origin(xyz=(sign * fly_x, 0.0, fly_rail_len + 0.025)),
            material=rubber,
            name=f"{side}_top_cap",
        )

    for idx, z in enumerate((0.26, 0.56, 0.86, 1.16, 1.46, 1.76, 2.06, 2.36)):
        fly.visual(
            Cylinder(radius=0.0125, length=0.390),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name=f"fly_rung_{idx}",
        )
        fly.visual(
            Cylinder(radius=0.016, length=0.055),
            origin=Origin(xyz=(-0.170, 0.0, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=worn_edge,
            name=f"fly_rung_collar_{idx}_0",
        )
        fly.visual(
            Cylinder(radius=0.016, length=0.055),
            origin=Origin(xyz=(0.170, 0.0, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=worn_edge,
            name=f"fly_rung_collar_{idx}_1",
        )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        # The fly part frame is its lower end.  At q=0 the lower end is already
        # inserted through both base-mounted channel sleeves; positive travel
        # extends the fly section upward.
        origin=Origin(xyz=(0.0, fly_y, 0.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.40, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    slide = object_model.get_articulation("base_to_fly")

    ctx.check(
        "fly section uses vertical prismatic travel",
        slide.axis == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 1.0,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )

    for side in ("left", "right"):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            elem_a=f"{side}_rail_web",
            elem_b=f"{side}_upper_sleeve_web",
            min_overlap=0.12,
            name=f"{side} rail is retained in upper guide sleeve",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            elem_a=f"{side}_rail_web",
            elem_b=f"{side}_lower_sleeve_web",
            min_overlap=0.12,
            name=f"{side} rail is retained in lower guide sleeve",
        )

    ctx.expect_contact(
        fly,
        base,
        elem_a="left_rail_web",
        elem_b="left_upper_sleeve_web",
        name="left rail bears on upper sleeve web",
    )
    ctx.expect_contact(
        fly,
        base,
        elem_a="right_rail_web",
        elem_b="right_upper_sleeve_web",
        name="right rail bears on upper sleeve web",
    )

    rest_pos = ctx.part_world_position(fly)
    with ctx.pose({slide: 1.10}):
        for side in ("left", "right"):
            ctx.expect_overlap(
                fly,
                base,
                axes="z",
                elem_a=f"{side}_rail_web",
                elem_b=f"{side}_upper_sleeve_web",
                min_overlap=0.12,
                name=f"extended {side} rail remains in upper sleeve",
            )
            ctx.expect_overlap(
                fly,
                base,
                axes="z",
                elem_a=f"{side}_rail_web",
                elem_b=f"{side}_lower_sleeve_web",
                min_overlap=0.08,
                name=f"extended {side} rail remains in lower sleeve",
            )
        extended_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly section extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 1.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
