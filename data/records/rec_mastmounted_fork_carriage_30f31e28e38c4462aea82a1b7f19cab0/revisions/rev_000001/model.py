from __future__ import annotations

import math

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
    model = ArticulatedObject(name="mast_mounted_fork_carriage")

    mast_paint = Material("satin_black_painted_steel", rgba=(0.015, 0.017, 0.018, 1.0))
    rail_wear = Material("polished_guide_wear_strip", rgba=(0.58, 0.60, 0.58, 1.0))
    carriage_paint = Material("safety_yellow_carriage", rgba=(0.96, 0.68, 0.04, 1.0))
    fork_steel = Material("dark_forged_fork_steel", rgba=(0.11, 0.12, 0.12, 1.0))
    rubber = Material("dark_rubber_rollers", rgba=(0.02, 0.02, 0.018, 1.0))
    bolt_metal = Material("zinc_bolt_heads", rgba=(0.44, 0.45, 0.43, 1.0))

    mast = model.part("mast")

    def mast_box(name: str, size, xyz, material=mast_paint, rpy=(0.0, 0.0, 0.0)) -> None:
        mast.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # A floor-mounted mast base gives the assembly a grounded, bolted-down stance.
    mast_box("floor_anchor_plate", (0.48, 1.34, 0.070), (-0.08, 0.0, 0.035))
    mast_box("lower_crosshead", (0.18, 1.34, 0.14), (-0.01, 0.0, 0.145))
    mast_box("upper_crosshead", (0.18, 1.30, 0.14), (-0.01, 0.0, 3.115))
    mast_box("mid_tie_bar", (0.13, 1.24, 0.08), (-0.02, 0.0, 1.62))

    for y, side in ((0.55, "side_0"), (-0.55, "side_1")):
        mast_box(f"rail_{side}", (0.12, 0.10, 3.05), (0.0, y, 1.60))
        # Bright narrow wear strips mark the guide ways where the carriage rollers run.
        mast_box(
            f"inner_wear_strip_{side}",
            (0.035, 0.018, 2.86),
            (0.058, y - math.copysign(0.045, y), 1.60),
            material=rail_wear,
        )
        mast_box(
            f"rear_upright_{side}",
            (0.10, 0.08, 3.00),
            (-0.12, y, 1.56),
        )
        # Low triangular-looking braces are represented by rectangular tubes
        # angled from the base plate up into the mast uprights.
        mast_box(
            f"base_brace_{side}",
            (0.085, 0.070, 0.72),
            (-0.21, y, 0.47),
            rpy=(0.0, -0.36, 0.0),
        )
        for x, bolt_index in ((-0.26, 0), (0.10, 1)):
            mast.visual(
                Cylinder(radius=0.035, length=0.018),
                origin=Origin(xyz=(x, y, 0.079)),
                material=bolt_metal,
                name=f"anchor_bolt_{side}_{bolt_index}",
            )

    carriage = model.part("carriage")

    def carriage_box(name: str, size, xyz, material=carriage_paint, rpy=(0.0, 0.0, 0.0)) -> None:
        carriage.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Main moving apron: a welded front frame in front of the mast rails.
    carriage_box("lower_apron", (0.12, 1.02, 0.14), (0.02, 0.0, 0.15))
    carriage_box("upper_apron", (0.11, 1.02, 0.12), (0.02, 0.0, 0.90))
    for y, side in ((0.43, "side_0"), (-0.43, "side_1")):
        carriage_box(f"side_plate_{side}", (0.11, 0.08, 0.92), (0.02, y, 0.52))
        carriage_box(f"roller_bracket_lower_{side}", (0.105, 0.115, 0.07), (-0.055, y, 0.26))
        carriage_box(f"roller_bracket_upper_{side}", (0.105, 0.115, 0.07), (-0.055, y, 0.76))

        # Paired guide rollers are mounted on stub brackets and run just inside
        # the two mast rails. Their axes point side-to-side.
        for z, level in ((0.26, "lower"), (0.76, "upper")):
            carriage.visual(
                Cylinder(radius=0.055, length=0.060),
                origin=Origin(xyz=(-0.100, y + math.copysign(0.036, y), z), rpy=(math.pi / 2, 0.0, 0.0)),
                material=rubber,
                name=f"guide_roller_{level}_{side}",
            )

    # A tall load backrest keeps a pallet load from falling rearward.
    carriage_box("backrest_bottom_bar", (0.070, 1.04, 0.070), (0.055, 0.0, 0.98))
    carriage_box("backrest_top_bar", (0.070, 1.04, 0.070), (0.055, 0.0, 1.55))
    for y, post in ((0.48, "outer_0"), (0.24, "inner_0"), (0.0, "center"), (-0.24, "inner_1"), (-0.48, "outer_1")):
        carriage_box(f"backrest_post_{post}", (0.060, 0.050, 0.64), (0.055, y, 1.26))

    # Two forged forks project forward from hooked vertical shanks on the apron.
    for y, fork in ((0.29, "fork_0"), (-0.29, "fork_1")):
        carriage_box(f"{fork}_shank", (0.13, 0.13, 0.50), (0.13, y, 0.15), material=fork_steel)
        carriage_box(f"{fork}_top_hook", (0.16, 0.15, 0.12), (0.075, y, 0.42), material=fork_steel)
        carriage_box(f"{fork}_lower_hook", (0.17, 0.15, 0.10), (0.085, y, 0.03), material=fork_steel)
        carriage_box(f"{fork}_tine", (1.14, 0.12, 0.075), (0.72, y, -0.075), material=fork_steel)
        carriage_box(f"{fork}_beveled_tip", (0.13, 0.11, 0.045), (1.345, y, -0.060), material=fork_steel)

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.18, 0.0, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.45, lower=0.0, upper=1.45),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_lift")

    ctx.check(
        "single vertical carriage joint",
        len(object_model.articulations) == 1
        and lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in lift.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={len(object_model.articulations)}, type={lift.articulation_type}, axis={lift.axis}",
    )
    ctx.check(
        "realistic lift travel",
        lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper is not None
        and lift.motion_limits.upper > 1.2,
        details=f"limits={lift.motion_limits}",
    )

    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        min_gap=0.05,
        positive_elem="fork_0_tine",
        negative_elem="floor_anchor_plate",
        name="forks project forward of the grounded base",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        min_overlap=0.8,
        elem_a="side_plate_side_0",
        elem_b="rail_side_0",
        name="carriage side plate stays within mast rail height",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({lift: lift.motion_limits.upper}):
        raised_position = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            min_overlap=0.8,
            elem_a="side_plate_side_0",
            elem_b="rail_side_0",
            name="raised carriage remains engaged with guide rails",
        )

    ctx.check(
        "prismatic carriage moves upward",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 1.3,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
