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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tire_swing_yoke")

    timber = model.material("timber", rgba=(0.56, 0.42, 0.28, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.16, 0.22, 0.28, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    tire_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.26, tube=0.08, radial_segments=22, tubular_segments=56),
        "tire_ring",
    )

    beam = model.part("beam")
    beam.visual(
        Box((1.60, 0.16, 0.14)),
        material=timber,
        name="top_beam",
    )
    beam.visual(
        Box((0.24, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=painted_steel,
        name="support_block",
    )
    beam.visual(
        Box((0.010, 0.024, 0.05)),
        origin=Origin(xyz=(-0.034, 0.0, -0.182)),
        material=painted_steel,
        name="left_pivot_hanger",
    )
    beam.visual(
        Box((0.010, 0.024, 0.05)),
        origin=Origin(xyz=(0.034, 0.0, -0.182)),
        material=painted_steel,
        name="right_pivot_hanger",
    )
    beam.visual(
        Cylinder(radius=0.010, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="beam_pivot_pin",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="top_sleeve",
    )
    yoke.visual(
        Box((0.048, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=painted_steel,
        name="top_hub",
    )
    yoke.visual(
        Box((0.52, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=painted_steel,
        name="spreader_bar",
    )
    yoke.visual(
        Box((0.035, 0.03, 0.22)),
        origin=Origin(xyz=(-0.24, 0.0, -0.195)),
        material=painted_steel,
        name="left_link",
    )
    yoke.visual(
        Box((0.035, 0.03, 0.22)),
        origin=Origin(xyz=(0.24, 0.0, -0.195)),
        material=painted_steel,
        name="right_link",
    )
    yoke.visual(
        Box((0.50, 0.028, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.258)),
        material=painted_steel,
        name="lower_tie",
    )
    yoke.visual(
        Cylinder(radius=0.012, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, -0.295), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="lower_axle",
    )

    tire = model.part("tire")
    tire.visual(
        Cylinder(radius=0.016, length=0.42),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hanger_tube",
    )
    tire.visual(
        tire_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=rubber,
        name="tire_ring",
    )
    tire.visual(
        Box((0.038, 0.032, 0.10)),
        origin=Origin(xyz=(-0.20, 0.0, -0.066)),
        material=dark_metal,
        name="left_hanger_strap",
    )
    tire.visual(
        Box((0.038, 0.032, 0.10)),
        origin=Origin(xyz=(0.20, 0.0, -0.066)),
        material=dark_metal,
        name="right_hanger_strap",
    )
    tire.visual(
        Box((0.44, 0.028, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=dark_metal,
        name="tire_bridge",
    )

    model.articulation(
        "beam_to_yoke",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.5,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "yoke_to_tire",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=tire,
        origin=Origin(xyz=(0.0, 0.0, -0.295)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=2.0,
            lower=-1.05,
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

    beam = object_model.get_part("beam")
    yoke = object_model.get_part("yoke")
    tire = object_model.get_part("tire")
    upper = object_model.get_articulation("beam_to_yoke")
    lower = object_model.get_articulation("yoke_to_tire")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower_corner, upper_corner = aabb
        return tuple((lower_corner[i] + upper_corner[i]) * 0.5 for i in range(3))

    ctx.allow_overlap(
        beam,
        yoke,
        elem_a="beam_pivot_pin",
        elem_b="top_sleeve",
        reason="The upper swing pivot is represented by a solid pin nested inside the yoke sleeve.",
    )
    ctx.allow_overlap(
        yoke,
        tire,
        elem_a="lower_axle",
        elem_b="hanger_tube",
        reason="The lower hanger pivot is represented by a solid axle nested through the tire's hanger tube.",
    )

    ctx.check(
        "upper pivot axis follows beam span",
        tuple(round(v, 6) for v in upper.axis) == (1.0, 0.0, 0.0),
        details=f"axis={upper.axis}",
    )
    ctx.check(
        "lower pivots follow beam span",
        tuple(round(v, 6) for v in lower.axis) == (1.0, 0.0, 0.0),
        details=f"axis={lower.axis}",
    )

    with ctx.pose({upper: 0.0, lower: 0.0}):
        ctx.expect_gap(
            beam,
            tire,
            axis="z",
            min_gap=0.18,
            name="tire seat sits clearly below the beam assembly",
        )
        ctx.expect_overlap(
            beam,
            yoke,
            axes="x",
            elem_a="beam_pivot_pin",
            elem_b="top_sleeve",
            min_overlap=0.045,
            name="upper pivot keeps the yoke captured on the beam pin",
        )
        ctx.expect_overlap(
            yoke,
            tire,
            axes="x",
            elem_a="lower_axle",
            elem_b="hanger_tube",
            min_overlap=0.36,
            name="lower hanger tube stays captured on the yoke axle",
        )

        rest_yoke = aabb_center(ctx.part_element_world_aabb(yoke, elem="lower_axle"))
        rest_tire = aabb_center(ctx.part_element_world_aabb(tire, elem="tire_ring"))

    with ctx.pose({upper: 0.45, lower: 0.0}):
        swung_yoke = aabb_center(ctx.part_element_world_aabb(yoke, elem="lower_axle"))
        swung_tire = aabb_center(ctx.part_element_world_aabb(tire, elem="tire_ring"))

    ctx.check(
        "yoke swings forward on the beam pivot",
        rest_yoke is not None
        and swung_yoke is not None
        and swung_yoke[1] > rest_yoke[1] + 0.08,
        details=f"rest={rest_yoke}, swung={swung_yoke}",
    )
    ctx.check(
        "tire follows the yoke when the upper pivot swings",
        rest_tire is not None
        and swung_tire is not None
        and swung_tire[1] > rest_tire[1] + 0.12,
        details=f"rest={rest_tire}, swung={swung_tire}",
    )

    with ctx.pose({upper: 0.0, lower: 0.55}):
        yoke_resting = aabb_center(ctx.part_element_world_aabb(yoke, elem="lower_axle"))
        pitched_tire = aabb_center(ctx.part_element_world_aabb(tire, elem="tire_ring"))

    ctx.check(
        "tire pitches independently beneath the yoke",
        yoke_resting is not None
        and rest_tire is not None
        and pitched_tire is not None
        and abs((yoke_resting[1] if yoke_resting is not None else 0.0) - (rest_yoke[1] if rest_yoke is not None else 0.0))
        < 1e-6
        and pitched_tire[1] > rest_tire[1] + 0.08,
        details=f"yoke={yoke_resting}, rest_tire={rest_tire}, pitched_tire={pitched_tire}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
