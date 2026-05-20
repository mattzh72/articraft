from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_pan_tilt_unit")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("matte_black_sensor", rgba=(0.015, 0.016, 0.018, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    glass = model.material("blue_coated_glass", rgba=(0.08, 0.22, 0.34, 0.78))
    amber = model.material("status_amber", rgba=(1.0, 0.62, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.260, 0.200, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=anodized,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.061, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=steel,
        name="fixed_bearing_race",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(-0.085, 0.0, 0.033)),
        material=black,
        name="cable_grommet",
    )
    for index, (x, y) in enumerate(
        (
            (-0.100, -0.070),
            (0.100, -0.070),
            (-0.100, 0.070),
            (0.100, 0.070),
        )
    ):
        base.visual(
            Cylinder(radius=0.009, length=0.007),
            origin=Origin(xyz=(x, y, 0.0315)),
            material=steel,
            name=f"base_bolt_{index}",
        )
        base.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, y, -0.0015)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.058, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=anodized,
        name="turntable",
    )
    pan_stage.visual(
        Cylinder(radius=0.042, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=steel,
        name="rotating_bearing_cap",
    )
    pan_stage.visual(
        Box((0.150, 0.155, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=anodized,
        name="yoke_base",
    )
    pan_stage.visual(
        Box((0.046, 0.034, 0.040)),
        origin=Origin(xyz=(-0.046, 0.0, 0.085)),
        material=anodized,
        name="rear_rib",
    )
    pan_stage.visual(
        Box((0.026, 0.034, 0.170)),
        origin=Origin(xyz=(0.0, 0.088, 0.158)),
        material=anodized,
        name="yoke_arm_0",
    )
    pan_stage.visual(
        Box((0.026, 0.034, 0.170)),
        origin=Origin(xyz=(0.0, -0.088, 0.158)),
        material=anodized,
        name="yoke_arm_1",
    )
    pan_stage.visual(
        Cylinder(radius=0.029, length=0.016),
        origin=Origin(xyz=(0.0, 0.073, 0.225), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_bearing_0",
    )
    pan_stage.visual(
        Cylinder(radius=0.029, length=0.016),
        origin=Origin(xyz=(0.0, -0.073, 0.225), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_bearing_1",
    )
    pan_stage.visual(
        Box((0.018, 0.018, 0.150)),
        origin=Origin(xyz=(-0.044, 0.0, 0.135)),
        material=anodized,
        name="yoke_spine",
    )
    for index, (x, y) in enumerate(
        (
            (-0.050, -0.052),
            (0.050, -0.052),
            (-0.050, 0.052),
            (0.050, 0.052),
        )
    ):
        pan_stage.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(x, y, 0.078)),
            material=steel,
            name=f"pan_cap_screw_{index}",
        )
    pan_stage.visual(
        Box((0.010, 0.055, 0.016)),
        origin=Origin(xyz=(0.061, 0.0, 0.083)),
        material=steel,
        name="pan_stop_tab",
    )

    tilt_stage = model.part("tilt_stage")
    tilt_stage.visual(
        Box((0.136, 0.096, 0.070)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=black,
        name="sensor_body",
    )
    tilt_stage.visual(
        Box((0.142, 0.106, 0.012)),
        origin=Origin(xyz=(0.040, 0.0, -0.041)),
        material=anodized,
        name="tilt_cradle",
    )
    tilt_stage.visual(
        Cylinder(radius=0.013, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_axle",
    )
    tilt_stage.visual(
        Cylinder(radius=0.036, length=0.032),
        origin=Origin(xyz=(0.124, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="lens_barrel",
    )
    tilt_stage.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.142, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    tilt_stage.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(-0.033, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="rear_cable_gland",
    )
    tilt_stage.visual(
        Sphere(radius=0.005),
        origin=Origin(xyz=(0.090, -0.049, 0.022)),
        material=amber,
        name="status_led",
    )
    for index, (x, y) in enumerate(
        (
            (0.092, -0.039),
            (0.092, 0.039),
            (-0.010, -0.039),
            (-0.010, 0.039),
        )
    ):
        tilt_stage.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(x, y, 0.037)),
            material=steel,
            name=f"sensor_screw_{index}",
        )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-2.70, upper=2.70),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=tilt_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.75, upper=0.75),
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

    base = object_model.get_part("base")
    pan_stage = object_model.get_part("pan_stage")
    tilt_stage = object_model.get_part("tilt_stage")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")

    ctx.expect_gap(
        pan_stage,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="turntable",
        negative_elem="base_plate",
        name="pan turntable sits on base bearing plane",
    )
    ctx.expect_gap(
        tilt_stage,
        pan_stage,
        axis="z",
        min_gap=0.080,
        positive_elem="sensor_body",
        negative_elem="yoke_base",
        name="tilt sensor clears the pan yoke base",
    )
    ctx.expect_within(
        tilt_stage,
        pan_stage,
        axes="y",
        margin=0.0,
        inner_elem="sensor_body",
        outer_elem="yoke_base",
        name="sensor body fits between yoke sides",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(tilt_stage, elem="front_glass")
    rest_body_aabb = ctx.part_element_world_aabb(tilt_stage, elem="sensor_body")
    with ctx.pose({tilt_joint: 0.60}):
        up_lens_aabb = ctx.part_element_world_aabb(tilt_stage, elem="front_glass")
        up_body_aabb = ctx.part_element_world_aabb(tilt_stage, elem="sensor_body")
        ctx.expect_gap(
            tilt_stage,
            pan_stage,
            axis="z",
            min_gap=0.030,
            positive_elem="sensor_body",
            negative_elem="yoke_base",
            name="upward tilt remains above yoke base",
        )
    with ctx.pose({tilt_joint: -0.60}):
        ctx.expect_gap(
            tilt_stage,
            pan_stage,
            axis="z",
            min_gap=0.030,
            positive_elem="sensor_body",
            negative_elem="yoke_base",
            name="downward tilt remains above yoke base",
        )

    def _center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) / 2.0

    def _center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) / 2.0

    ctx.check(
        "positive tilt raises sensor front",
        rest_lens_aabb is not None
        and up_lens_aabb is not None
        and _center_z(up_lens_aabb) > _center_z(rest_lens_aabb) + 0.050,
        details=f"rest_lens_aabb={rest_lens_aabb}, up_lens_aabb={up_lens_aabb}",
    )
    ctx.check(
        "tilt axis passes through sensor housing",
        rest_body_aabb is not None
        and rest_body_aabb[0][2] < 0.255
        and rest_body_aabb[1][2] > 0.255
        and rest_body_aabb[0][1] < 0.0
        and rest_body_aabb[1][1] > 0.0,
        details=f"rest_body_aabb={rest_body_aabb}",
    )

    rest_sensor_aabb = ctx.part_element_world_aabb(tilt_stage, elem="sensor_body")
    with ctx.pose({pan_joint: 1.0}):
        panned_sensor_aabb = ctx.part_element_world_aabb(tilt_stage, elem="sensor_body")
    ctx.check(
        "pan joint sweeps sensor about vertical axis",
        rest_sensor_aabb is not None
        and panned_sensor_aabb is not None
        and _center_y(panned_sensor_aabb) > _center_y(rest_sensor_aabb) + 0.025,
        details=f"rest_sensor_aabb={rest_sensor_aabb}, panned_sensor_aabb={panned_sensor_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
