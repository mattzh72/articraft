from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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
    model = ArticulatedObject(name="tabletop_mini_easel")

    beech = model.material("beech_wood", rgba=(0.76, 0.64, 0.46, 1.0))
    dark_wood = model.material("walnut_detail", rgba=(0.40, 0.28, 0.18, 1.0))
    brass = model.material("brass_pin", rgba=(0.77, 0.66, 0.31, 1.0))

    head_frame = model.part("head_frame")
    head_frame.visual(
        Box((0.090, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=beech,
        name="head_block",
    )
    head_frame.visual(
        Cylinder(radius=0.0035, length=0.102),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="top_pin",
    )
    head_frame.visual(
        Box((0.018, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.015, -0.012)),
        material=dark_wood,
        name="rear_brace_lug",
    )

    left_front_leg = model.part("left_front_leg")
    left_front_leg.visual(
        Box((0.018, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.006, -0.026)),
        material=dark_wood,
        name="left_leg_head",
    )
    left_front_leg.visual(
        Box((0.014, 0.008, 0.220)),
        origin=Origin(xyz=(0.0, 0.038, -0.125), rpy=(0.34, 0.16, 0.0)),
        material=beech,
        name="left_leg_rail",
    )

    right_front_leg = model.part("right_front_leg")
    right_front_leg.visual(
        Box((0.018, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.006, -0.026)),
        material=dark_wood,
        name="right_leg_head",
    )
    right_front_leg.visual(
        Box((0.014, 0.008, 0.220)),
        origin=Origin(xyz=(0.0, 0.038, -0.125), rpy=(0.34, -0.16, 0.0)),
        material=beech,
        name="right_leg_rail",
    )

    rear_brace = model.part("rear_brace")
    rear_brace.visual(
        Box((0.016, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.009, -0.012)),
        material=dark_wood,
        name="rear_brace_head",
    )
    rear_brace.visual(
        Box((0.012, 0.006, 0.284)),
        origin=Origin(xyz=(0.0, -0.120, -0.116), rpy=(-0.82, 0.0, 0.0)),
        material=beech,
        name="rear_brace_rail",
    )

    model.articulation(
        "head_to_left_leg",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=left_front_leg,
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.90,
            upper=0.20,
        ),
    )
    model.articulation(
        "head_to_right_leg",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=right_front_leg,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.90,
            upper=0.20,
        ),
    )
    model.articulation(
        "head_to_rear_brace",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=rear_brace,
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.10,
            upper=0.82,
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
    head_frame = object_model.get_part("head_frame")
    left_front_leg = object_model.get_part("left_front_leg")
    right_front_leg = object_model.get_part("right_front_leg")
    rear_brace = object_model.get_part("rear_brace")

    left_hinge = object_model.get_articulation("head_to_left_leg")
    right_hinge = object_model.get_articulation("head_to_right_leg")
    brace_hinge = object_model.get_articulation("head_to_rear_brace")

    ctx.check(
        "front leg hinges use the shared top-pin axis",
        left_hinge.axis == (1.0, 0.0, 0.0)
        and right_hinge.axis == (1.0, 0.0, 0.0)
        and isclose(left_hinge.origin.xyz[2], 0.0, abs_tol=1e-9)
        and isclose(right_hinge.origin.xyz[2], 0.0, abs_tol=1e-9),
        details=(
            f"left axis={left_hinge.axis}, right axis={right_hinge.axis}, "
            f"left origin={left_hinge.origin.xyz}, right origin={right_hinge.origin.xyz}"
        ),
    )
    ctx.check(
        "rear brace hinge mounts on the rear side of the top bracket",
        brace_hinge.axis == (1.0, 0.0, 0.0) and brace_hinge.origin.xyz[1] < -0.005,
        details=f"brace axis={brace_hinge.axis}, brace origin={brace_hinge.origin.xyz}",
    )

    ctx.expect_origin_gap(
        right_front_leg,
        left_front_leg,
        axis="x",
        min_gap=0.06,
        max_gap=0.08,
        name="front leg hinges are spaced across the head block",
    )
    ctx.expect_origin_gap(
        left_front_leg,
        rear_brace,
        axis="y",
        min_gap=0.009,
        name="rear brace starts behind the front leg hinge line",
    )

    ctx.expect_gap(
        head_frame,
        left_front_leg,
        axis="z",
        positive_elem="head_block",
        negative_elem="left_leg_head",
        max_gap=0.0,
        max_penetration=0.0,
        name="left front leg head seats directly against the underside of the head block",
    )
    ctx.expect_gap(
        head_frame,
        right_front_leg,
        axis="z",
        positive_elem="head_block",
        negative_elem="right_leg_head",
        max_gap=0.0,
        max_penetration=0.0,
        name="right front leg head seats directly against the underside of the head block",
    )
    ctx.expect_gap(
        head_frame,
        rear_brace,
        axis="y",
        positive_elem="rear_brace_lug",
        negative_elem="rear_brace_head",
        max_gap=0.0,
        max_penetration=0.0,
        name="rear brace head bears against the rear mounting lug",
    )

    left_rest_aabb = ctx.part_world_aabb(left_front_leg)
    brace_rest_aabb = ctx.part_world_aabb(rear_brace)

    with ctx.pose({left_hinge: left_hinge.motion_limits.lower, right_hinge: right_hinge.motion_limits.lower}):
        left_folded_aabb = ctx.part_world_aabb(left_front_leg)
        right_folded_aabb = ctx.part_world_aabb(right_front_leg)
        ctx.check(
            "front legs fold back toward the head when closed",
            left_rest_aabb is not None
            and left_folded_aabb is not None
            and left_folded_aabb[1][1] < left_rest_aabb[1][1] - 0.02,
            details=f"rest={left_rest_aabb}, folded={left_folded_aabb}",
        )
        ctx.check(
            "front legs remain mirrored when folded",
            left_folded_aabb is not None
            and right_folded_aabb is not None
            and left_folded_aabb[0][0] < right_folded_aabb[0][0]
            and left_folded_aabb[1][0] < right_folded_aabb[1][0],
            details=f"left={left_folded_aabb}, right={right_folded_aabb}",
        )

    with ctx.pose({brace_hinge: brace_hinge.motion_limits.upper}):
        brace_folded_aabb = ctx.part_world_aabb(rear_brace)
        ctx.check(
            "rear brace folds inward toward the head block",
            brace_rest_aabb is not None
            and brace_folded_aabb is not None
            and brace_folded_aabb[0][1] > brace_rest_aabb[0][1] + 0.05,
            details=f"rest={brace_rest_aabb}, folded={brace_folded_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
