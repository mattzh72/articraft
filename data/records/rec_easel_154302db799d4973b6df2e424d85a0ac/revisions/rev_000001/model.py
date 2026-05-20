from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pochade_box_watercolor_easel")

    walnut = model.material("walnut", rgba=(0.49, 0.33, 0.20, 1.0))
    birch = model.material("birch", rgba=(0.83, 0.77, 0.65, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.61, 0.28, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    graphite = model.material("graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.006, 0.020, 0.210)),
        origin=Origin(xyz=(0.0085, -0.124, 0.585)),
        material=graphite,
        name="left_sleeve",
    )
    support_frame.visual(
        Box((0.006, 0.020, 0.210)),
        origin=Origin(xyz=(0.0085, 0.124, 0.585)),
        material=graphite,
        name="right_sleeve",
    )
    support_frame.visual(
        Box((0.006, 0.020, 0.210)),
        origin=Origin(xyz=(0.0395, -0.124, 0.585)),
        material=graphite,
        name="left_outer_guide",
    )
    support_frame.visual(
        Box((0.006, 0.020, 0.210)),
        origin=Origin(xyz=(0.0395, 0.124, 0.585)),
        material=graphite,
        name="right_outer_guide",
    )
    support_frame.visual(
        Box((0.023, 0.012, 0.110)),
        origin=Origin(xyz=(-0.006, -0.118, 0.675)),
        material=graphite,
        name="left_plate",
    )
    support_frame.visual(
        Box((0.023, 0.012, 0.110)),
        origin=Origin(xyz=(-0.006, 0.118, 0.675)),
        material=graphite,
        name="right_plate",
    )
    support_frame.visual(
        Box((0.037, 0.020, 0.012)),
        origin=Origin(xyz=(0.024, -0.124, 0.696)),
        material=graphite,
        name="left_top_cap",
    )
    support_frame.visual(
        Box((0.037, 0.020, 0.012)),
        origin=Origin(xyz=(0.024, 0.124, 0.696)),
        material=graphite,
        name="right_top_cap",
    )
    support_frame.visual(
        Cylinder(radius=0.006, length=0.236),
        origin=Origin(xyz=(-0.018, 0.0, 0.705), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="rear_bridge",
    )
    support_frame.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(xyz=(0.045, -0.124, 0.640), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="left_clamp_knob",
    )
    support_frame.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(xyz=(0.045, 0.124, 0.640), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="right_clamp_knob",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.090, 0.290, 0.740)),
        mass=1.2,
        origin=Origin(xyz=(0.010, 0.0, 0.370)),
    )

    box_base = model.part("box_base")
    box_base.visual(
        Cylinder(radius=0.009, length=0.224),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="trunnion_bar",
    )
    box_base.visual(
        Box((0.024, 0.036, 0.024)),
        origin=Origin(xyz=(0.012, -0.090, 0.012)),
        material=walnut,
        name="left_mount_block",
    )
    box_base.visual(
        Box((0.024, 0.036, 0.024)),
        origin=Origin(xyz=(0.012, 0.090, 0.012)),
        material=walnut,
        name="right_mount_block",
    )
    box_base.visual(
        Box((0.352, 0.272, 0.008)),
        origin=Origin(xyz=(0.190, 0.0, 0.004)),
        material=walnut,
        name="bottom_panel",
    )
    box_base.visual(
        Box((0.342, 0.010, 0.046)),
        origin=Origin(xyz=(0.197, -0.136, 0.031)),
        material=walnut,
        name="left_wall",
    )
    box_base.visual(
        Box((0.342, 0.010, 0.046)),
        origin=Origin(xyz=(0.197, 0.136, 0.031)),
        material=walnut,
        name="right_wall",
    )
    box_base.visual(
        Box((0.012, 0.272, 0.046)),
        origin=Origin(xyz=(0.368, 0.0, 0.031)),
        material=walnut,
        name="front_wall",
    )
    box_base.visual(
        Box((0.012, 0.272, 0.046)),
        origin=Origin(xyz=(0.020, 0.0, 0.031)),
        material=walnut,
        name="rear_wall",
    )
    box_base.visual(
        Box((0.018, 0.258, 0.012)),
        origin=Origin(xyz=(0.356, 0.0, 0.048)),
        material=birch,
        name="front_ledge",
    )
    box_base.visual(
        Box((0.308, 0.232, 0.003)),
        origin=Origin(xyz=(0.195, 0.0, 0.0095)),
        material=birch,
        name="palette_floor",
    )
    box_base.visual(
        Box((0.006, 0.034, 0.028)),
        origin=Origin(xyz=(0.376, -0.086, 0.029)),
        material=brass,
        name="left_latch",
    )
    box_base.visual(
        Box((0.006, 0.034, 0.028)),
        origin=Origin(xyz=(0.376, 0.086, 0.029)),
        material=brass,
        name="right_latch",
    )
    box_base.inertial = Inertial.from_geometry(
        Box((0.385, 0.290, 0.070)),
        mass=1.9,
        origin=Origin(xyz=(0.192, 0.0, 0.035)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.356, 0.272, 0.008)),
        origin=Origin(xyz=(0.178, 0.0, 0.004)),
        material=walnut,
        name="lid_panel",
    )
    lid.visual(
        Box((0.340, 0.012, 0.014)),
        origin=Origin(xyz=(0.188, -0.135, 0.007)),
        material=walnut,
        name="left_rail",
    )
    lid.visual(
        Box((0.340, 0.012, 0.014)),
        origin=Origin(xyz=(0.188, 0.135, 0.007)),
        material=walnut,
        name="right_rail",
    )
    lid.visual(
        Box((0.012, 0.272, 0.014)),
        origin=Origin(xyz=(0.350, 0.0, 0.007)),
        material=walnut,
        name="front_rail",
    )
    lid.visual(
        Box((0.020, 0.272, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, 0.007)),
        material=walnut,
        name="rear_rail",
    )
    lid.visual(
        Box((0.312, 0.228, 0.003)),
        origin=Origin(xyz=(0.184, 0.0, 0.0095)),
        material=birch,
        name="painting_face",
    )
    lid.visual(
        Box((0.028, 0.010, 0.006)),
        origin=Origin(xyz=(0.315, -0.082, 0.014)),
        material=brass,
        name="left_clip",
    )
    lid.visual(
        Box((0.028, 0.010, 0.006)),
        origin=Origin(xyz=(0.315, 0.082, 0.014)),
        material=brass,
        name="right_clip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.370, 0.286, 0.020)),
        mass=0.9,
        origin=Origin(xyz=(0.185, 0.0, 0.010)),
    )

    left_leg = model.part("left_leg")
    left_leg.visual(
        Cylinder(radius=0.0125, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=aluminum,
        name="inner_post",
    )
    left_leg.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.654)),
        material=rubber,
        name="foot_pad",
    )
    left_leg.inertial = Inertial.from_geometry(
        Box((0.034, 0.034, 0.690)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.334)),
    )

    right_leg = model.part("right_leg")
    right_leg.visual(
        Cylinder(radius=0.0125, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=aluminum,
        name="inner_post",
    )
    right_leg.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.654)),
        material=rubber,
        name="foot_pad",
    )
    right_leg.inertial = Inertial.from_geometry(
        Box((0.034, 0.034, 0.690)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.334)),
    )

    model.articulation(
        "support_to_box_tilt",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=box_base,
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "box_to_lid",
        ArticulationType.REVOLUTE,
        parent=box_base,
        child=lid,
        origin=Origin(xyz=(0.020, 0.0, 0.054)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "support_to_left_leg",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=left_leg,
        origin=Origin(xyz=(0.024, -0.124, 0.690)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.12,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "support_to_right_leg",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=right_leg,
        origin=Origin(xyz=(0.024, 0.124, 0.690)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.12,
            lower=0.0,
            upper=0.080,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    box_base = object_model.get_part("box_base")
    lid = object_model.get_part("lid")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    tilt_joint = object_model.get_articulation("support_to_box_tilt")
    lid_hinge = object_model.get_articulation("box_to_lid")
    left_slide = object_model.get_articulation("support_to_left_leg")
    right_slide = object_model.get_articulation("support_to_right_leg")

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

    ctx.expect_contact(
        box_base,
        support_frame,
        elem_a="trunnion_bar",
        elem_b="left_plate",
        name="left tilt trunnion is captured by the support plate",
    )
    ctx.expect_contact(
        box_base,
        support_frame,
        elem_a="trunnion_bar",
        elem_b="right_plate",
        name="right tilt trunnion is captured by the support plate",
    )

    ctx.expect_gap(
        box_base,
        support_frame,
        axis="z",
        positive_elem="bottom_panel",
        negative_elem="rear_bridge",
        min_gap=0.0,
        max_gap=0.030,
        name="box body sits just above the leg top junction hardware",
    )

    ctx.expect_contact(
        left_leg,
        support_frame,
        elem_a="inner_post",
        elem_b="left_sleeve",
        name="left telescoping post is guided by the left sleeve",
    )
    ctx.expect_contact(
        right_leg,
        support_frame,
        elem_a="inner_post",
        elem_b="right_sleeve",
        name="right telescoping post is guided by the right sleeve",
    )
    ctx.expect_overlap(
        left_leg,
        support_frame,
        axes="z",
        elem_a="inner_post",
        elem_b="left_sleeve",
        min_overlap=0.180,
        name="left leg retains insertion at rest",
    )
    ctx.expect_overlap(
        right_leg,
        support_frame,
        axes="z",
        elem_a="inner_post",
        elem_b="right_sleeve",
        min_overlap=0.180,
        name="right leg retains insertion at rest",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            box_base,
            elem_a="front_rail",
            elem_b="front_wall",
            name="closed lid seats onto the front box rim",
        )
        ctx.expect_overlap(
            lid,
            box_base,
            axes="xy",
            min_overlap=0.240,
            name="closed lid covers the pochade box opening",
        )

    with ctx.pose({lid_hinge: math.radians(72.0)}):
        ctx.expect_gap(
            lid,
            box_base,
            axis="z",
            positive_elem="front_rail",
            negative_elem="front_wall",
            min_gap=0.180,
            name="opened lid lifts the painting face above the box",
        )

    left_rest = ctx.part_world_position(left_leg)
    right_rest = ctx.part_world_position(right_leg)
    with ctx.pose({left_slide: 0.080, right_slide: 0.080}):
        ctx.expect_contact(
            left_leg,
            support_frame,
            elem_a="inner_post",
            elem_b="left_sleeve",
            name="left leg remains guided when extended",
        )
        ctx.expect_contact(
            right_leg,
            support_frame,
            elem_a="inner_post",
            elem_b="right_sleeve",
            name="right leg remains guided when extended",
        )
        ctx.expect_overlap(
            left_leg,
            support_frame,
            axes="z",
            elem_a="inner_post",
            elem_b="left_sleeve",
            min_overlap=0.100,
            name="left leg keeps retained insertion when extended",
        )
        ctx.expect_overlap(
            right_leg,
            support_frame,
            axes="z",
            elem_a="inner_post",
            elem_b="right_sleeve",
            min_overlap=0.100,
            name="right leg keeps retained insertion when extended",
        )
        left_extended = ctx.part_world_position(left_leg)
        right_extended = ctx.part_world_position(right_leg)

    ctx.check(
        "left leg extends downward",
        left_rest is not None
        and left_extended is not None
        and left_extended[2] < left_rest[2] - 0.050,
        details=f"rest={left_rest}, extended={left_extended}",
    )
    ctx.check(
        "right leg extends downward",
        right_rest is not None
        and right_extended is not None
        and right_extended[2] < right_rest[2] - 0.050,
        details=f"rest={right_rest}, extended={right_extended}",
    )

    with ctx.pose({tilt_joint: math.radians(28.0)}):
        ctx.expect_gap(
            box_base,
            support_frame,
            axis="z",
            positive_elem="front_wall",
            negative_elem="rear_bridge",
            min_gap=0.140,
            name="tilt hinge raises the front edge for painting angle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
