from __future__ import annotations

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
    model = ArticulatedObject(name="wall_thermostat")

    housing_white = model.material("housing_white", rgba=(0.94, 0.95, 0.93, 1.0))
    trim_warm_gray = model.material("trim_warm_gray", rgba=(0.77, 0.78, 0.76, 1.0))
    display_dark = model.material("display_dark", rgba=(0.16, 0.18, 0.19, 1.0))
    smoked_cover = model.material("smoked_cover", rgba=(0.38, 0.41, 0.43, 0.45))
    dial_black = model.material("dial_black", rgba=(0.15, 0.15, 0.15, 1.0))
    dial_marker = model.material("dial_marker", rgba=(0.86, 0.87, 0.84, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.004, 0.112, 0.112)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=trim_warm_gray,
        name="backplate",
    )
    body.visual(
        Box((0.024, 0.108, 0.108)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=housing_white,
        name="housing_shell",
    )
    body.visual(
        Box((0.006, 0.086, 0.072)),
        origin=Origin(xyz=(0.031, -0.005, 0.002)),
        material=trim_warm_gray,
        name="face_bezel",
    )
    body.visual(
        Box((0.0015, 0.062, 0.028)),
        origin=Origin(xyz=(0.03475, -0.006, 0.014)),
        material=display_dark,
        name="display_face",
    )
    body.visual(
        Box((0.007, 0.090, 0.012)),
        origin=Origin(xyz=(0.0315, -0.005, 0.046)),
        material=housing_white,
        name="hinge_header",
    )
    body.visual(
        Box((0.005, 0.068, 0.026)),
        origin=Origin(xyz=(0.0315, -0.006, -0.026)),
        material=housing_white,
        name="service_frame",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.017, 0.056, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_warm_gray,
        name="dial_mount",
    )

    cover = model.part("cover")
    cover.visual(
        Cylinder(radius=0.0035, length=0.078),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_warm_gray,
        name="cover_barrel",
    )
    cover.visual(
        Box((0.003, 0.086, 0.086)),
        origin=Origin(xyz=(-0.0015, 0.0, -0.043)),
        material=smoked_cover,
        name="cover_panel",
    )
    cover.visual(
        Box((0.006, 0.036, 0.006)),
        origin=Origin(xyz=(-0.0005, 0.0, -0.083)),
        material=trim_warm_gray,
        name="cover_pull",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_warm_gray,
        name="shaft",
    )
    dial.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_wheel",
    )
    dial.visual(
        Box((0.005, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, 0.014, 0.011)),
        material=dial_marker,
        name="dial_marker",
    )

    cover_hinge = model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.038, -0.005, 0.046)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    dial_spin = model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.017, 0.058, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    body.meta["primary_controls"] = ["cover_hinge", "dial_spin"]
    cover_hinge.meta["qc_samples"] = [0.0, 0.8, 1.35]
    dial_spin.meta["qc_samples"] = [0.0, pi / 2.0, pi]

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    dial = object_model.get_part("dial")
    cover_hinge = object_model.get_articulation("cover_hinge")
    dial_spin = object_model.get_articulation("dial_spin")

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_overlap(
            cover,
            body,
            axes="yz",
            elem_a="cover_panel",
            elem_b="face_bezel",
            min_overlap=0.060,
            name="cover panel spans the thermostat face when closed",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem="cover_panel",
            negative_elem="face_bezel",
            min_gap=0.0003,
            max_gap=0.0060,
            name="closed cover sits just proud of the front bezel",
        )
        ctx.expect_gap(
            dial,
            body,
            axis="y",
            positive_elem="dial_wheel",
            negative_elem="housing_shell",
            min_gap=0.001,
            max_gap=0.020,
            name="dial wheel protrudes from the right side of the housing",
        )

    closed_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")
    with ctx.pose({cover_hinge: 1.20}):
        open_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")

    closed_cover_min = closed_cover[0] if closed_cover is not None else None
    open_cover_min = open_cover[0] if open_cover is not None else None
    open_cover_max = open_cover[1] if open_cover is not None else None
    closed_cover_max = closed_cover[1] if closed_cover is not None else None
    ctx.check(
        "cover swings upward and outward",
        closed_cover_min is not None
        and closed_cover_max is not None
        and open_cover_min is not None
        and open_cover_max is not None
        and open_cover_min[2] > closed_cover_min[2] + 0.025
        and open_cover_max[0] > closed_cover_max[0] + 0.030,
        details=f"closed={closed_cover}, open={open_cover}",
    )

    rest_marker = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
    with ctx.pose({dial_spin: pi / 2.0}):
        quarter_turn_marker = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))

    ctx.check(
        "dial marker moves when the dial rotates",
        rest_marker is not None
        and quarter_turn_marker is not None
        and quarter_turn_marker[0] > rest_marker[0] + 0.008
        and abs(quarter_turn_marker[2]) < abs(rest_marker[2]),
        details=f"rest={rest_marker}, quarter_turn={quarter_turn_marker}",
    )

    return ctx.report()


object_model = build_object_model()
