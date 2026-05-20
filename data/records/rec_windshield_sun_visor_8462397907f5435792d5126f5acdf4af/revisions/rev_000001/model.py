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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(*, width: float, depth: float, thickness: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius, corner_segments=10),
            thickness,
            cap=True,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="car_windshield_sun_visor")

    bracket_plastic = model.material("bracket_plastic", rgba=(0.22, 0.23, 0.25, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.57, 0.58, 0.60, 1.0))
    visor_vinyl = model.material("visor_vinyl", rgba=(0.78, 0.74, 0.66, 1.0))
    visor_trim = model.material("visor_trim", rgba=(0.66, 0.62, 0.54, 1.0))

    visor_width = 0.340
    visor_depth = 0.155
    visor_thickness = 0.018
    visor_mesh = _rounded_panel_mesh(
        width=visor_width,
        depth=visor_depth,
        thickness=visor_thickness,
        radius=0.018,
        name="visor_panel_shell",
    )

    roof_bracket = model.part("roof_bracket")
    roof_bracket.visual(
        Box((0.092, 0.046, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=bracket_plastic,
        name="roof_plate",
    )
    roof_bracket.visual(
        Box((0.062, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.010, -0.011)),
        material=bracket_plastic,
        name="bracket_body",
    )
    roof_bracket.visual(
        Box((0.024, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, 0.024, -0.030)),
        material=bracket_plastic,
        name="pivot_stand",
    )
    roof_bracket.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, 0.032, -0.046)),
        material=arm_metal,
        name="side_pivot_socket",
    )
    roof_bracket.inertial = Inertial.from_geometry(
        Box((0.092, 0.046, 0.064)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.015, -0.021)),
    )

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=arm_metal,
        name="pivot_collar",
    )
    pivot_arm.visual(
        Box((0.022, 0.024, 0.030)),
        origin=Origin(xyz=(0.004, 0.014, -0.036)),
        material=arm_metal,
        name="inner_knuckle",
    )
    pivot_arm.visual(
        Box((0.074, 0.010, 0.012)),
        origin=Origin(
            xyz=(0.028, 0.050, -0.042),
            rpy=(0.0, 0.0, math.atan2(0.060, 0.040)),
        ),
        material=arm_metal,
        name="reach_rod",
    )
    pivot_arm.visual(
        Box((0.024, 0.022, 0.022)),
        origin=Origin(xyz=(0.048, 0.080, -0.042)),
        material=arm_metal,
        name="hinge_cheek",
    )
    pivot_arm.visual(
        Cylinder(radius=0.010, length=0.092),
        origin=Origin(xyz=(0.055, 0.090, -0.042), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_metal,
        name="flip_hinge_barrel",
    )
    pivot_arm.inertial = Inertial.from_geometry(
        Box((0.090, 0.120, 0.048)),
        mass=0.22,
        origin=Origin(xyz=(0.030, 0.050, -0.038)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        Box((0.048, 0.024, 0.020)),
        origin=Origin(xyz=(-0.052, 0.024, -0.010)),
        material=visor_trim,
        name="mount_block",
    )
    visor_panel.visual(
        Box((0.160, 0.016, 0.014)),
        origin=Origin(xyz=(0.020, 0.022, -0.017)),
        material=visor_trim,
        name="hinge_spine",
    )
    visor_panel.visual(
        visor_mesh,
        origin=Origin(xyz=(0.082, visor_depth / 2.0, -0.019)),
        material=visor_vinyl,
        name="panel_shell",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((visor_width, visor_depth, 0.026)),
        mass=0.58,
        origin=Origin(xyz=(0.082, visor_depth / 2.0, -0.019)),
    )

    model.articulation(
        "side_swing_pivot",
        ArticulationType.REVOLUTE,
        parent=roof_bracket,
        child=pivot_arm,
        origin=Origin(xyz=(0.0, 0.032, -0.046)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "flip_down_hinge",
        ArticulationType.REVOLUTE,
        parent=pivot_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.055, 0.090, -0.042)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
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

    roof_bracket = object_model.get_part("roof_bracket")
    pivot_arm = object_model.get_part("pivot_arm")
    visor_panel = object_model.get_part("visor_panel")
    side_swing = object_model.get_articulation("side_swing_pivot")
    flip_hinge = object_model.get_articulation("flip_down_hinge")

    ctx.expect_gap(
        roof_bracket,
        visor_panel,
        axis="z",
        positive_elem="pivot_stand",
        negative_elem="panel_shell",
        min_gap=0.040,
        max_gap=0.075,
        name="stowed visor hangs just below the pivot stand",
    )
    ctx.expect_overlap(
        pivot_arm,
        visor_panel,
        axes="x",
        elem_a="flip_hinge_barrel",
        elem_b="hinge_spine",
        min_overlap=0.060,
        name="flip hinge hardware lines up with the visor spine",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")
    with ctx.pose({flip_hinge: 1.20}):
        flipped_panel_aabb = ctx.part_element_world_aabb(visor_panel, elem="panel_shell")
    ctx.check(
        "positive flip rotates the visor downward",
        rest_panel_aabb is not None
        and flipped_panel_aabb is not None
        and flipped_panel_aabb[0][2] < rest_panel_aabb[0][2] - 0.080,
        details=f"rest={rest_panel_aabb}, flipped={flipped_panel_aabb}",
    )

    rest_origin = ctx.part_world_position(visor_panel)
    with ctx.pose({side_swing: 1.30}):
        swung_origin = ctx.part_world_position(visor_panel)
    ctx.check(
        "positive side swing carries the visor toward the side window direction",
        rest_origin is not None
        and swung_origin is not None
        and swung_origin[0] > rest_origin[0] + 0.035
        and swung_origin[1] < rest_origin[1] - 0.060,
        details=f"rest={rest_origin}, swung={swung_origin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
