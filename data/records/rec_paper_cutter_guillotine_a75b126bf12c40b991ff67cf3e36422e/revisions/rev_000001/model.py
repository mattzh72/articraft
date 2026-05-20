from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="office_paper_cutter")

    base_color = model.material("base_body", color=(0.78, 0.80, 0.82))
    rail_color = model.material("rail_dark", color=(0.18, 0.19, 0.21))
    arm_color = model.material("arm_black", color=(0.14, 0.15, 0.16))
    steel_color = model.material("steel", color=(0.73, 0.75, 0.78))
    accent_color = model.material("accent_red", color=(0.76, 0.18, 0.14))

    base_w = 0.30
    base_d = 0.22
    base_t = 0.012
    hinge_r = 0.006
    hinge_x = -base_w / 2.0
    hinge_z = base_t + hinge_r
    rail_x = base_w / 2.0 - 0.018
    rail_len = 0.18
    rail_w = 0.012
    rail_h = 0.006

    base = model.part("base")
    base.visual(
        Box((base_w, base_d, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=base_color,
        name="board_panel",
    )
    base.visual(
        Box((0.184, 0.006, 0.001)),
        origin=Origin(xyz=(-0.058, -0.011, base_t + 0.0005)),
        material=steel_color,
        name="cut_strip",
    )
    base.visual(
        Box((rail_w, rail_len, rail_h)),
        origin=Origin(xyz=(rail_x, 0.0, base_t + rail_h / 2.0)),
        material=rail_color,
        name="side_rail",
    )
    base.visual(
        Box((0.014, 0.045, 0.018)),
        origin=Origin(xyz=(hinge_x + 0.007, -0.0775, base_t + 0.009)),
        material=base_color,
        name="hinge_support_lower",
    )
    base.visual(
        Box((0.014, 0.045, 0.018)),
        origin=Origin(xyz=(hinge_x + 0.007, 0.0775, base_t + 0.009)),
        material=base_color,
        name="hinge_support_upper",
    )
    base.visual(
        Cylinder(radius=hinge_r, length=0.045),
        origin=Origin(xyz=(hinge_x, -0.0775, hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel_color,
        name="hinge_barrel_lower",
    )
    base.visual(
        Cylinder(radius=hinge_r, length=0.045),
        origin=Origin(xyz=(hinge_x, 0.0775, hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel_color,
        name="hinge_barrel_upper",
    )

    arm = model.part("cutting_arm")
    arm_len = 0.195
    arm_w = 0.028
    arm_t = 0.012
    arm.visual(
        Box((0.022, 0.100, 0.008)),
        origin=Origin(xyz=(0.011, 0.0, -0.001)),
        material=steel_color,
        name="arm_hinge_leaf",
    )
    arm.visual(
        Cylinder(radius=0.005, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel_color,
        name="arm_hinge_barrel",
    )
    arm.visual(
        Box((arm_len, arm_w, arm_t)),
        origin=Origin(xyz=(arm_len / 2.0, 0.0, 0.001)),
        material=arm_color,
        name="arm_beam",
    )
    arm.visual(
        Box((0.175, 0.004, 0.0015)),
        origin=Origin(xyz=(0.090, -0.0115, -0.00405)),
        material=steel_color,
        name="blade_strip",
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.046),
        origin=Origin(xyz=(0.155, 0.0, 0.014), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=accent_color,
        name="handle_grip",
    )

    guide = model.part("paper_guide")
    guide.visual(
        Box((0.018, 0.036, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=rail_color,
        name="guide_carriage",
    )
    guide.visual(
        Box((0.008, 0.060, 0.040)),
        origin=Origin(xyz=(-0.005, 0.0, 0.025)),
        material=accent_color,
        name="paper_stop",
    )
    guide.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.050), rpy=(0.0, 0.0, 0.0)),
        material=steel_color,
        name="guide_knob",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "base_to_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(rail_x, 0.0, base_t + rail_h)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.25,
            lower=-0.070,
            upper=0.070,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    arm = object_model.get_part("cutting_arm")
    guide = object_model.get_part("paper_guide")
    arm_joint = object_model.get_articulation("base_to_arm")
    guide_joint = object_model.get_articulation("base_to_guide")

    ctx.check(
        "parts and joints exist",
        all(
            item is not None
            for item in (base, arm, guide, arm_joint, guide_joint)
        ),
        details="Expected base, cutting arm, paper guide, and both articulations.",
    )
    ctx.check(
        "arm hinge opens upward around base edge",
        arm_joint.axis == (0.0, -1.0, 0.0)
        and arm_joint.motion_limits is not None
        and arm_joint.motion_limits.lower == 0.0
        and arm_joint.motion_limits.upper is not None
        and arm_joint.motion_limits.upper >= 1.2,
        details=f"axis={arm_joint.axis}, limits={arm_joint.motion_limits}",
    )
    ctx.check(
        "paper guide slides along side rail",
        guide_joint.axis == (0.0, 1.0, 0.0)
        and guide_joint.motion_limits is not None
        and guide_joint.motion_limits.lower is not None
        and guide_joint.motion_limits.upper is not None
        and guide_joint.motion_limits.lower < 0.0 < guide_joint.motion_limits.upper,
        details=f"axis={guide_joint.axis}, limits={guide_joint.motion_limits}",
    )

    with ctx.pose({arm_joint: 0.0, guide_joint: 0.0}):
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="blade_strip",
            negative_elem="cut_strip",
            min_gap=0.0,
            max_gap=0.001,
            name="closed blade sits just above the cut strip",
        )
        ctx.expect_overlap(
            arm,
            base,
            axes="x",
            elem_a="arm_beam",
            elem_b="board_panel",
            min_overlap=0.16,
            name="closed arm spans the cutter base",
        )
        ctx.expect_gap(
            guide,
            base,
            axis="z",
            positive_elem="guide_carriage",
            negative_elem="side_rail",
            min_gap=0.0,
            max_gap=0.001,
            name="guide carriage rides directly on the side rail",
        )
        ctx.expect_within(
            guide,
            base,
            axes="y",
            inner_elem="guide_carriage",
            outer_elem="side_rail",
            margin=0.0,
            name="guide carriage stays within the rail travel at center",
        )

    guide_rest = ctx.part_world_position(guide)
    with ctx.pose({arm_joint: 1.0, guide_joint: 0.070}):
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="handle_grip",
            negative_elem="board_panel",
            min_gap=0.05,
            name="raised arm clears the base board",
        )
        ctx.expect_within(
            guide,
            base,
            axes="y",
            inner_elem="guide_carriage",
            outer_elem="side_rail",
            margin=0.0,
            name="guide carriage stays captured at max extension",
        )
        guide_extended = ctx.part_world_position(guide)

    ctx.check(
        "guide moves toward positive y when extended",
        guide_rest is not None
        and guide_extended is not None
        and guide_extended[1] > guide_rest[1] + 0.05,
        details=f"rest={guide_rest}, extended={guide_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
