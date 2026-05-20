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
    model = ArticulatedObject(name="car_sun_visor")

    bracket_plastic = model.material("bracket_plastic", rgba=(0.77, 0.77, 0.75, 1.0))
    visor_fabric = model.material("visor_fabric", rgba=(0.86, 0.84, 0.79, 1.0))
    trim_fabric = model.material("trim_fabric", rgba=(0.79, 0.77, 0.72, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.59, 0.61, 0.64, 1.0))
    dark_cap = model.material("dark_cap", rgba=(0.30, 0.29, 0.28, 1.0))

    roof_bracket = model.part("roof_bracket")
    roof_bracket.visual(
        Box((0.084, 0.044, 0.010)),
        origin=Origin(xyz=(0.012, 0.028, 0.016)),
        material=bracket_plastic,
        name="mount_pad",
    )
    roof_bracket.visual(
        Box((0.026, 0.020, 0.024)),
        origin=Origin(xyz=(-0.009, 0.010, 0.004)),
        material=bracket_plastic,
        name="pivot_housing",
    )
    roof_bracket.visual(
        Box((0.020, 0.012, 0.018)),
        origin=Origin(xyz=(-0.021, 0.006, -0.006)),
        material=dark_cap,
        name="pivot_base_fin",
    )
    roof_bracket.inertial = Inertial.from_geometry(
        Box((0.084, 0.044, 0.032)),
        mass=0.18,
        origin=Origin(xyz=(0.004, 0.022, 0.006)),
    )

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        Box((0.056, 0.012, 0.010)),
        origin=Origin(xyz=(0.032, 0.000, -0.004)),
        material=bracket_plastic,
        name="main_pivot_arm",
    )
    pivot_arm.visual(
        Cylinder(radius=0.0045, length=0.034),
        origin=Origin(xyz=(0.038, 0.000, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_metal,
        name="flip_hinge_rod_stub",
    )
    pivot_arm.visual(
        Box((0.012, 0.018, 0.014)),
        origin=Origin(xyz=(0.050, 0.000, -0.004)),
        material=bracket_plastic,
        name="hinge_knuckle",
    )
    pivot_arm.inertial = Inertial.from_geometry(
        Box((0.060, 0.018, 0.018)),
        mass=0.10,
        origin=Origin(xyz=(0.032, 0.000, -0.003)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        Box((0.334, 0.164, 0.018)),
        origin=Origin(xyz=(0.167, 0.082, -0.009)),
        material=visor_fabric,
        name="visor_body",
    )
    visor_panel.visual(
        Box((0.040, 0.024, 0.018)),
        origin=Origin(xyz=(0.022, 0.030, -0.009)),
        material=trim_fabric,
        name="hinge_reinforcement",
    )
    visor_panel.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.012, 0.000, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_cap,
        name="hinge_cap",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((0.334, 0.164, 0.020)),
        mass=0.62,
        origin=Origin(xyz=(0.167, 0.082, -0.009)),
    )

    model.articulation(
        "side_swing",
        ArticulationType.REVOLUTE,
        parent=roof_bracket,
        child=pivot_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.62,
        ),
    )
    model.articulation(
        "flip_down",
        ArticulationType.REVOLUTE,
        parent=pivot_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.056, 0.0, -0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=2.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_bracket = object_model.get_part("roof_bracket")
    pivot_arm = object_model.get_part("pivot_arm")
    visor_panel = object_model.get_part("visor_panel")
    side_swing = object_model.get_articulation("side_swing")
    flip_down = object_model.get_articulation("flip_down")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
        return (
            0.5 * (xmin + xmax),
            0.5 * (ymin + ymax),
            0.5 * (zmin + zmax),
        )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_gap(
        visor_panel,
        roof_bracket,
        axis="x",
        min_gap=0.001,
        max_gap=0.0045,
        name="closed visor panel sits just outboard of the roof bracket",
    )
    ctx.expect_contact(
        pivot_arm,
        visor_panel,
        elem_a="main_pivot_arm",
        elem_b="visor_body",
        name="visor panel is carried by the main pivot arm at rest",
    )

    rest_body_aabb = ctx.part_element_world_aabb(visor_panel, elem="visor_body")
    rest_body_center = aabb_center(rest_body_aabb)
    flip_upper = 0.0 if flip_down.motion_limits is None or flip_down.motion_limits.upper is None else flip_down.motion_limits.upper
    side_upper = 0.0 if side_swing.motion_limits is None or side_swing.motion_limits.upper is None else side_swing.motion_limits.upper

    with ctx.pose({flip_down: flip_upper}):
        flipped_body_center = aabb_center(ctx.part_element_world_aabb(visor_panel, elem="visor_body"))
        ctx.check(
            "positive flip joint drops the visor downward",
            rest_body_center is not None
            and flipped_body_center is not None
            and flipped_body_center[2] < rest_body_center[2] - 0.05
            and flipped_body_center[1] < rest_body_center[1] - 0.06,
            details=f"rest={rest_body_center}, flipped={flipped_body_center}",
        )

    with ctx.pose({side_swing: side_upper}):
        swung_body_center = aabb_center(ctx.part_element_world_aabb(visor_panel, elem="visor_body"))
        ctx.check(
            "positive side swing moves the visor toward the side window",
            rest_body_center is not None
            and swung_body_center is not None
            and swung_body_center[1] > rest_body_center[1] + 0.10
            and swung_body_center[0] < rest_body_center[0] - 0.10,
            details=f"rest={rest_body_center}, swung={swung_body_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
