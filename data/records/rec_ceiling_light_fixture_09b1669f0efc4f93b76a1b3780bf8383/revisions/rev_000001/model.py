from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_ceiling_light")

    housing_white = model.material("housing_white", rgba=(0.94, 0.94, 0.92, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.97, 0.97, 0.95, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.80, 0.80, 0.80, 1.0))
    diffuser = model.material("diffuser", rgba=(0.95, 0.97, 1.0, 0.45))

    fixture_length = 1.22
    fixture_width = 0.305
    housing_depth = 0.090
    top_thickness = 0.006
    wall_thickness = 0.018
    lip_thickness = 0.012
    lip_width = 0.020

    opening_length = 1.184
    opening_width = 0.265

    housing = model.part("housing")
    housing.visual(
        Box((fixture_length, fixture_width, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=housing_white,
        name="top_pan",
    )
    housing.visual(
        Box((fixture_length, wall_thickness, 0.084)),
        origin=Origin(xyz=(0.0, 0.1435, -0.001)),
        material=housing_white,
        name="hinge_wall",
    )
    housing.visual(
        Box((fixture_length, wall_thickness, 0.084)),
        origin=Origin(xyz=(0.0, -0.1435, -0.001)),
        material=housing_white,
        name="latch_wall",
    )
    housing.visual(
        Box((wall_thickness, 0.269, 0.084)),
        origin=Origin(xyz=(0.601, 0.0, -0.001)),
        material=housing_white,
        name="end_cap_0",
    )
    housing.visual(
        Box((wall_thickness, 0.269, 0.084)),
        origin=Origin(xyz=(-0.601, 0.0, -0.001)),
        material=housing_white,
        name="end_cap_1",
    )
    housing.visual(
        Box((opening_length, lip_width, lip_thickness)),
        origin=Origin(xyz=(0.0, 0.1325, -0.036)),
        material=trim_gray,
        name="hinge_rail",
    )
    housing.visual(
        Box((opening_length, lip_width, lip_thickness)),
        origin=Origin(xyz=(0.0, -0.1325, -0.036)),
        material=trim_gray,
        name="latch_rail",
    )
    housing.visual(
        Box((wall_thickness, opening_width, lip_thickness)),
        origin=Origin(xyz=(0.601, 0.0, -0.036)),
        material=trim_gray,
        name="end_trim_0",
    )
    housing.visual(
        Box((wall_thickness, opening_width, lip_thickness)),
        origin=Origin(xyz=(-0.601, 0.0, -0.036)),
        material=trim_gray,
        name="end_trim_1",
    )
    housing.visual(
        Box((1.10, 0.190, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=reflector_white,
        name="reflector_tray",
    )

    lens = model.part("lens")
    lens.visual(
        Box((1.178, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.009, -0.008)),
        material=trim_gray,
        name="hinge_leaf",
    )
    lens.visual(
        Box((1.178, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, -0.251, -0.008)),
        material=trim_gray,
        name="latch_leaf",
    )
    lens.visual(
        Box((0.012, 0.259, 0.016)),
        origin=Origin(xyz=(0.583, -0.1295, -0.008)),
        material=trim_gray,
        name="end_frame_0",
    )
    lens.visual(
        Box((0.012, 0.259, 0.016)),
        origin=Origin(xyz=(-0.583, -0.1295, -0.008)),
        material=trim_gray,
        name="end_frame_1",
    )
    lens.visual(
        Box((1.154, 0.225, 0.004)),
        origin=Origin(xyz=(0.0, -0.1305, -0.010)),
        material=diffuser,
        name="diffuser_panel",
    )

    model.articulation(
        "lens_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lens,
        origin=Origin(xyz=(0.0, 0.1325, -0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lens = object_model.get_part("lens")
    hinge = object_model.get_articulation("lens_hinge")

    limits = hinge.motion_limits
    open_angle = 1.10
    if limits is not None and limits.upper is not None:
        open_angle = min(open_angle, limits.upper)

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lens,
            housing,
            axes="x",
            min_overlap=1.15,
            name="lens spans nearly the full fixture length",
        )
        ctx.expect_overlap(
            lens,
            housing,
            axes="y",
            min_overlap=0.24,
            name="lens covers the underside opening width",
        )
        ctx.expect_gap(
            housing,
            lens,
            axis="z",
            positive_elem="latch_rail",
            negative_elem="latch_leaf",
            min_gap=0.0,
            max_gap=0.006,
            name="closed lens aligns tightly under the housing trim",
        )

    with ctx.pose({hinge: open_angle}):
        ctx.expect_gap(
            housing,
            lens,
            axis="z",
            positive_elem="latch_rail",
            negative_elem="latch_leaf",
            min_gap=0.090,
            name="opened lens hangs well below the housing",
        )

    return ctx.report()


object_model = build_object_model()
