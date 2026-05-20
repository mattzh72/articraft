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
    model = ArticulatedObject(name="rooftop_vent_tower")

    steel = model.material("galvanized_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    steel_dark = model.material("weathered_steel", rgba=(0.48, 0.50, 0.53, 1.0))
    interior = model.material("duct_shadow", rgba=(0.20, 0.21, 0.22, 1.0))
    hinge_finish = model.material("hinge_dark", rgba=(0.30, 0.31, 0.33, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.340, 0.240, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=steel_dark,
        name="roof_flashing",
    )
    tower.visual(
        Box((0.220, 0.120, 0.052)),
        origin=Origin(xyz=(0.000, 0.000, 0.036)),
        material=steel,
        name="lower_plenum",
    )
    tower.visual(
        Box((0.220, 0.010, 0.294)),
        origin=Origin(xyz=(0.000, -0.055, 0.209)),
        material=steel,
        name="back_wall",
    )
    tower.visual(
        Box((0.010, 0.120, 0.294)),
        origin=Origin(xyz=(-0.105, 0.000, 0.209)),
        material=steel,
        name="left_wall",
    )
    tower.visual(
        Box((0.010, 0.120, 0.294)),
        origin=Origin(xyz=(0.105, 0.000, 0.209)),
        material=steel,
        name="right_wall",
    )
    tower.visual(
        Box((0.220, 0.120, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.350)),
        material=steel,
        name="roof_cap",
    )
    tower.visual(
        Box((0.188, 0.012, 0.112)),
        origin=Origin(xyz=(0.000, 0.060, 0.117)),
        material=steel,
        name="front_apron",
    )
    tower.visual(
        Box((0.208, 0.012, 0.014)),
        origin=Origin(xyz=(0.000, 0.060, 0.178)),
        material=steel_dark,
        name="sill",
    )
    tower.visual(
        Box((0.014, 0.012, 0.158)),
        origin=Origin(xyz=(-0.090, 0.060, 0.252)),
        material=steel_dark,
        name="left_jamb",
    )
    tower.visual(
        Box((0.014, 0.012, 0.158)),
        origin=Origin(xyz=(0.090, 0.060, 0.252)),
        material=steel_dark,
        name="right_jamb",
    )
    tower.visual(
        Box((0.208, 0.012, 0.016)),
        origin=Origin(xyz=(0.000, 0.060, 0.328)),
        material=steel_dark,
        name="header",
    )
    tower.visual(
        Box((0.228, 0.018, 0.010)),
        origin=Origin(xyz=(0.000, 0.057, 0.350)),
        material=steel_dark,
        name="eyebrow",
    )
    tower.visual(
        Box((0.182, 0.006, 0.014)),
        origin=Origin(xyz=(0.000, 0.063, 0.338)),
        material=hinge_finish,
        name="hinge_mount",
    )
    tower.visual(
        Box((0.012, 0.010, 0.016)),
        origin=Origin(xyz=(-0.078, 0.060, 0.340)),
        material=hinge_finish,
        name="left_hinge_bracket",
    )
    tower.visual(
        Box((0.012, 0.010, 0.016)),
        origin=Origin(xyz=(0.078, 0.060, 0.340)),
        material=hinge_finish,
        name="right_hinge_bracket",
    )
    tower.visual(
        Box((0.170, 0.010, 0.150)),
        origin=Origin(xyz=(0.000, -0.045, 0.252)),
        material=interior,
        name="duct_baffle",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.340, 0.240, 0.362)),
        mass=8.5,
        origin=Origin(xyz=(0.000, 0.000, 0.181)),
    )

    flap = model.part("weather_flap")
    flap.visual(
        Cylinder(radius=0.005, length=0.172),
        origin=Origin(xyz=(0.000, 0.003, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=hinge_finish,
        name="flap_knuckle",
    )
    flap.visual(
        Box((0.176, 0.010, 0.014)),
        origin=Origin(xyz=(0.000, 0.008, -0.007)),
        material=steel_dark,
        name="top_return",
    )
    flap.visual(
        Box((0.176, 0.006, 0.146)),
        origin=Origin(xyz=(0.000, 0.008, -0.080)),
        material=steel,
        name="flap_plate",
    )
    flap.visual(
        Box((0.006, 0.012, 0.146)),
        origin=Origin(xyz=(-0.085, 0.009, -0.080)),
        material=steel_dark,
        name="left_hem",
    )
    flap.visual(
        Box((0.006, 0.012, 0.146)),
        origin=Origin(xyz=(0.085, 0.009, -0.080)),
        material=steel_dark,
        name="right_hem",
    )
    flap.visual(
        Box((0.176, 0.010, 0.010)),
        origin=Origin(xyz=(0.000, 0.009, -0.149)),
        material=steel_dark,
        name="bottom_return",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.176, 0.016, 0.156)),
        mass=0.85,
        origin=Origin(xyz=(0.000, 0.008, -0.078)),
    )

    model.articulation(
        "tower_to_weather_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.000, 0.068, 0.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
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

    tower = object_model.get_part("tower")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("tower_to_weather_flap")

    ctx.expect_gap(
        flap,
        tower,
        axis="y",
        min_gap=0.004,
        max_gap=0.012,
        positive_elem="flap_plate",
        negative_elem="hinge_mount",
        name="closed flap sits just ahead of the outlet frame",
    )
    ctx.expect_overlap(
        flap,
        tower,
        axes="x",
        min_overlap=0.160,
        elem_a="flap_plate",
        elem_b="header",
        name="flap spans the outlet width",
    )

    closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_plate")
    with ctx.pose({hinge: 1.10}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_plate")
        ctx.check(
            "flap opens upward and outward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.050
            and open_aabb[0][2] > closed_aabb[0][2] + 0.070,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
