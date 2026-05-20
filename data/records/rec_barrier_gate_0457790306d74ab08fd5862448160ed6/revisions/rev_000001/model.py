from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_lift_beam_road_blocker")

    housing_paint = model.material("housing_paint", rgba=(0.22, 0.24, 0.26, 1.0))
    beam_paint = model.material("beam_paint", rgba=(0.86, 0.14, 0.12, 1.0))
    beam_tip_paint = model.material("beam_tip_paint", rgba=(0.95, 0.95, 0.95, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.30, 0.44, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=housing_paint,
        name="pedestal",
    )
    housing.visual(
        Box((0.06, 0.18, 1.82)),
        origin=Origin(xyz=(-0.09, -0.04, 1.03)),
        material=housing_paint,
        name="left_post",
    )
    housing.visual(
        Box((0.06, 0.18, 1.82)),
        origin=Origin(xyz=(0.09, -0.04, 1.03)),
        material=housing_paint,
        name="right_post",
    )
    housing.visual(
        Box((0.18, 0.02, 1.66)),
        origin=Origin(xyz=(0.0, -0.13, 0.95)),
        material=housing_paint,
        name="rear_panel",
    )
    housing.visual(
        Box((0.22, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.10, 1.92)),
        material=housing_paint,
        name="top_crossmember",
    )
    housing.visual(
        Box((0.014, 0.10, 1.54)),
        origin=Origin(xyz=(-0.053, -0.005, 0.89)),
        material=housing_paint,
        name="left_guide",
    )
    housing.visual(
        Box((0.014, 0.10, 1.54)),
        origin=Origin(xyz=(0.053, -0.005, 0.89)),
        material=housing_paint,
        name="right_guide",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.30, 0.44, 2.00)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
    )

    beam_arm = model.part("beam_arm")
    beam_arm.visual(
        Box((0.082, 0.08, 1.52)),
        origin=Origin(xyz=(0.0, -0.03, 0.76)),
        material=housing_paint,
        name="slider_mast",
    )
    beam_arm.visual(
        Box((0.10, 0.24, 0.20)),
        origin=Origin(xyz=(0.0, 0.07, 0.80)),
        material=housing_paint,
        name="carriage_head",
    )
    beam_arm.visual(
        Box((0.10, 3.20, 0.12)),
        origin=Origin(xyz=(0.0, 1.79, 0.80)),
        material=beam_paint,
        name="beam_panel",
    )
    beam_arm.visual(
        Box((0.12, 0.04, 0.16)),
        origin=Origin(xyz=(0.0, 3.41, 0.80)),
        material=beam_tip_paint,
        name="beam_tip",
    )
    beam_arm.inertial = Inertial.from_geometry(
        Box((0.12, 3.45, 1.52)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 1.685, 0.76)),
    )

    model.articulation(
        "housing_to_beam_arm",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=beam_arm,
        origin=Origin(xyz=(0.0, -0.03, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.35,
            lower=0.0,
            upper=1.05,
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

    housing = object_model.get_part("housing")
    beam_arm = object_model.get_part("beam_arm")
    lift = object_model.get_articulation("housing_to_beam_arm")
    limits = lift.motion_limits
    lower = 0.0 if limits is None or limits.lower is None else limits.lower
    upper = 1.05 if limits is None or limits.upper is None else limits.upper

    ctx.check(
        "road blocker parts and articulation exist",
        housing is not None and beam_arm is not None and lift is not None,
        details=f"housing={housing}, beam_arm={beam_arm}, lift={lift}",
    )

    with ctx.pose({lift: lower}):
        ctx.expect_gap(
            beam_arm,
            housing,
            axis="z",
            positive_elem="beam_panel",
            negative_elem="pedestal",
            min_gap=0.72,
            max_gap=0.90,
            name="closed beam sits at realistic blocking height above pedestal",
        )
        ctx.expect_gap(
            housing,
            beam_arm,
            axis="x",
            positive_elem="right_post",
            negative_elem="slider_mast",
            min_gap=0.015,
            name="slider mast clears right guide post at closed height",
        )
        ctx.expect_gap(
            beam_arm,
            housing,
            axis="x",
            positive_elem="slider_mast",
            negative_elem="left_post",
            min_gap=0.015,
            name="slider mast clears left guide post at closed height",
        )
        ctx.expect_overlap(
            beam_arm,
            housing,
            axes="z",
            elem_a="slider_mast",
            min_overlap=1.45,
            name="closed slider mast stays deeply engaged in the housing guides",
        )

    rest_pos = ctx.part_world_position(beam_arm)
    with ctx.pose({lift: upper}):
        ctx.expect_overlap(
            beam_arm,
            housing,
            axes="z",
            elem_a="slider_mast",
            min_overlap=0.68,
            name="raised slider mast keeps retained insertion inside the housing",
        )
        raised_pos = ctx.part_world_position(beam_arm)

    ctx.check(
        "beam arm lifts upward on the prismatic joint",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.95,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
