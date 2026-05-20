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
    model = ArticulatedObject(name="rooftop_vent_tower")

    body_metal = model.material("body_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    flashing_metal = model.material("flashing_metal", rgba=(0.46, 0.47, 0.49, 1.0))
    frame_metal = model.material("frame_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    flap_metal = model.material("flap_metal", rgba=(0.41, 0.42, 0.44, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.72, 0.56, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=flashing_metal,
        name="roof_flashing",
    )
    housing.visual(
        Box((0.36, 0.28, 0.124)),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=body_metal,
        name="base_curb",
    )
    housing.visual(
        Box((0.32, 0.25, 0.346)),
        origin=Origin(xyz=(0.0, -0.005, 0.313)),
        material=body_metal,
        name="lower_tower",
    )
    housing.visual(
        Box((0.38, 0.042, 0.352)),
        origin=Origin(xyz=(0.0, -0.119, 0.660)),
        material=body_metal,
        name="back_wall",
    )
    housing.visual(
        Box((0.042, 0.282, 0.352)),
        origin=Origin(xyz=(-0.169, 0.020, 0.660)),
        material=body_metal,
        name="left_wall",
    )
    housing.visual(
        Box((0.042, 0.282, 0.352)),
        origin=Origin(xyz=(0.169, 0.020, 0.660)),
        material=body_metal,
        name="right_wall",
    )
    housing.visual(
        Box((0.38, 0.282, 0.044)),
        origin=Origin(xyz=(0.0, 0.020, 0.814)),
        material=body_metal,
        name="top_cap",
    )
    housing.visual(
        Box((0.38, 0.084, 0.060)),
        origin=Origin(xyz=(0.0, 0.181, 0.770)),
        material=frame_metal,
        name="frame_header",
    )
    housing.visual(
        Box((0.38, 0.102, 0.060)),
        origin=Origin(xyz=(0.0, 0.172, 0.490)),
        material=frame_metal,
        name="outlet_sill",
    )
    housing.visual(
        Box((0.042, 0.084, 0.228)),
        origin=Origin(xyz=(-0.169, 0.181, 0.628)),
        material=frame_metal,
        name="left_jamb",
    )
    housing.visual(
        Box((0.042, 0.084, 0.228)),
        origin=Origin(xyz=(0.169, 0.181, 0.628)),
        material=frame_metal,
        name="right_jamb",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.72, 0.56, 0.836)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.418)),
    )

    weather_flap = model.part("weather_flap")
    weather_flap.visual(
        Box((0.30, 0.018, 0.22)),
        origin=Origin(xyz=(0.0, 0.009, -0.11)),
        material=flap_metal,
        name="flap_panel",
    )
    weather_flap.visual(
        Box((0.34, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.012, -0.015)),
        material=flap_metal,
        name="top_hem",
    )
    weather_flap.visual(
        Box((0.020, 0.035, 0.200)),
        origin=Origin(xyz=(-0.140, 0.0175, -0.115)),
        material=flap_metal,
        name="left_return",
    )
    weather_flap.visual(
        Box((0.020, 0.035, 0.200)),
        origin=Origin(xyz=(0.140, 0.0175, -0.115)),
        material=flap_metal,
        name="right_return",
    )
    weather_flap.visual(
        Box((0.30, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.015, -0.210)),
        material=flap_metal,
        name="drip_lip",
    )
    weather_flap.inertial = Inertial.from_geometry(
        Box((0.34, 0.035, 0.22)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0175, -0.11)),
    )

    model.articulation(
        "housing_to_weather_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=weather_flap,
        origin=Origin(xyz=(0.0, 0.223, 0.740)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=0.0,
            upper=1.2,
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
    weather_flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("housing_to_weather_flap")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            weather_flap,
            housing,
            axis="y",
            positive_elem="flap_panel",
            negative_elem="frame_header",
            min_gap=0.0,
            max_gap=0.002,
            name="closed flap sits flush to the outlet frame",
        )
        ctx.expect_gap(
            weather_flap,
            housing,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="outlet_sill",
            min_gap=0.0,
            max_gap=0.010,
            name="closed flap lands near the sill",
        )
        ctx.expect_overlap(
            weather_flap,
            housing,
            axes="x",
            elem_a="flap_panel",
            elem_b="outlet_sill",
            min_overlap=0.28,
            name="flap spans the framed outlet width",
        )
        closed_aabb = ctx.part_element_world_aabb(weather_flap, elem="flap_panel")

    with ctx.pose({hinge: 1.0}):
        ctx.expect_gap(
            weather_flap,
            housing,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="outlet_sill",
            min_gap=0.080,
            name="open flap lifts well above the sill",
        )
        open_aabb = ctx.part_element_world_aabb(weather_flap, elem="flap_panel")

    flap_opens_upward_and_outward = False
    if closed_aabb is not None and open_aabb is not None:
        flap_opens_upward_and_outward = (
            open_aabb[0][2] > closed_aabb[0][2] + 0.08
            and open_aabb[1][1] > closed_aabb[1][1] + 0.08
        )
    ctx.check(
        "flap opens upward and outward",
        flap_opens_upward_and_outward,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
