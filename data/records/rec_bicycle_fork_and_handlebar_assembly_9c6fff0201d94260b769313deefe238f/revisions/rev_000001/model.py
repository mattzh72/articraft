from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


HEAD_TILT = math.radians(7.0)


def _tube_mesh(points, radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=18,
            radial_segments=24,
            cap_ends=True,
        ),
        name,
    )


def _hollow_tube_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
    *,
    segments: int = 64,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
            [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=2,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recumbent_trike_front_fork")

    satin_black = Material("satin_black_paint", rgba=(0.015, 0.017, 0.018, 1.0))
    charcoal = Material("charcoal_head_tube", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = Material("brushed_threaded_steel", rgba=(0.66, 0.66, 0.62, 1.0))
    dark_rubber = Material("black_rubber_grips", rgba=(0.006, 0.006, 0.005, 1.0))
    bolt_steel = Material("pinch_bolt_steel", rgba=(0.78, 0.76, 0.70, 1.0))

    head_tube = model.part("head_tube")
    head_tube.visual(
        _hollow_tube_mesh(0.030, 0.020, 0.245, "head_shell_mesh"),
        origin=Origin(rpy=(0.0, HEAD_TILT, 0.0)),
        material=charcoal,
        name="head_shell",
    )
    head_tube.visual(
        _tube_mesh(
            [(-0.020, 0.0, 0.010), (-0.145, 0.0, -0.005), (-0.310, 0.0, -0.030)],
            0.024,
            "frame_boom_mesh",
        ),
        material=charcoal,
        name="frame_boom",
    )
    head_tube.visual(
        Box((0.084, 0.092, 0.010)),
        origin=Origin(xyz=(-0.060, 0.0, 0.065), rpy=(0.0, HEAD_TILT, 0.0)),
        material=charcoal,
        name="upper_gusset",
    )
    head_tube.visual(
        Box((0.080, 0.082, 0.010)),
        origin=Origin(xyz=(-0.066, 0.0, -0.065), rpy=(0.0, HEAD_TILT, 0.0)),
        material=charcoal,
        name="lower_gusset",
    )

    steering = model.part("steering_assembly")
    steering.visual(
        Cylinder(radius=0.014, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=brushed_steel,
        name="steerer",
    )
    for index, (x, y) in enumerate(((0.0166, 0.0), (-0.0166, 0.0), (0.0, 0.0166), (0.0, -0.0166))):
        steering.visual(
            Sphere(radius=0.0034),
            origin=Origin(xyz=(x, y, 0.0)),
            material=bolt_steel,
            name=f"bearing_ball_{index}",
        )
    for index, z in enumerate([0.225 + 0.011 * i for i in range(22)]):
        steering.visual(
            Cylinder(radius=0.0164, length=0.0032),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brushed_steel,
            name=f"thread_crest_{index}",
        )
    steering.visual(
        _hollow_tube_mesh(0.033, 0.0125, 0.032, "upper_bearing_cap_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.1462)),
        material=satin_black,
        name="upper_bearing_cap",
    )
    steering.visual(
        _hollow_tube_mesh(0.034, 0.0125, 0.034, "lower_bearing_cap_mesh"),
        origin=Origin(xyz=(0.0, 0.0, -0.143)),
        material=satin_black,
        name="lower_bearing_cap",
    )
    steering.visual(
        Box((0.082, 0.235, 0.078)),
        origin=Origin(xyz=(0.012, 0.0, -0.232)),
        material=satin_black,
        name="squared_crown",
    )
    steering.visual(
        Box((0.058, 0.135, 0.040)),
        origin=Origin(xyz=(0.007, 0.0, -0.184)),
        material=satin_black,
        name="crown_top_boss",
    )
    for suffix, y in (("0", 0.091), ("1", -0.091)):
        steering.visual(
            _tube_mesh(
                [
                    (0.004, y, -0.214),
                    (0.016, y, -0.390),
                    (0.045, y, -0.570),
                    (0.070, y, -0.735),
                ],
                0.0155,
                f"round_blade_{suffix}_mesh",
            ),
            material=satin_black,
            name=f"round_blade_{suffix}",
        )
        steering.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(0.073, y, -0.738), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name=f"axle_boss_{suffix}",
        )
        steering.visual(
            Box((0.032, 0.010, 0.060)),
            origin=Origin(xyz=(0.078, y, -0.742)),
            material=satin_black,
            name=f"dropout_plate_{suffix}",
        )
        steering.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.082, y, -0.738), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"axle_face_{suffix}",
        )

    steering.visual(
        Cylinder(radius=0.026, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=satin_black,
        name="stem_clamp",
    )
    steering.visual(
        _tube_mesh(
            [
                (0.000, 0.0, 0.345),
                (0.070, 0.0, 0.377),
                (0.185, 0.0, 0.410),
                (0.305, 0.0, 0.427),
            ],
            0.0175,
            "stem_extension_mesh",
        ),
        material=satin_black,
        name="stem_extension",
    )
    steering.visual(
        Cylinder(radius=0.036, length=0.188),
        origin=Origin(xyz=(0.315, 0.0, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="pinch_collar",
    )
    steering.visual(
        Box((0.005, 0.162, 0.050)),
        origin=Origin(xyz=(0.352, 0.0, 0.430)),
        material=dark_rubber,
        name="collar_split",
    )
    for suffix, y in (("0", 0.050), ("1", -0.050)):
        steering.visual(
            Cylinder(radius=0.0065, length=0.036),
            origin=Origin(xyz=(0.354, y, 0.450), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"pinch_bolt_{suffix}",
        )

    handlebar_paths = {
        "0": [
            (0.306, 0.054, 0.430),
            (0.350, 0.069, 0.427),
            (0.405, 0.083, 0.455),
            (0.438, 0.088, 0.512),
            (0.462, 0.087, 0.585),
        ],
        "1": [
            (0.306, -0.054, 0.430),
            (0.350, -0.069, 0.427),
            (0.405, -0.083, 0.455),
            (0.438, -0.088, 0.512),
            (0.462, -0.087, 0.585),
        ],
    }
    grip_paths = {
        "0": [(0.423, 0.087, 0.485), (0.442, 0.089, 0.535), (0.462, 0.087, 0.585)],
        "1": [(0.423, -0.087, 0.485), (0.442, -0.089, 0.535), (0.462, -0.087, 0.585)],
    }
    for suffix, path in handlebar_paths.items():
        steering.visual(
            _tube_mesh(path, 0.0115, f"bullhorn_bar_{suffix}_mesh"),
            material=satin_black,
            name=f"bullhorn_bar_{suffix}",
        )
        steering.visual(
            _tube_mesh(grip_paths[suffix], 0.0155, f"rubber_grip_{suffix}_mesh"),
            material=dark_rubber,
            name=f"rubber_grip_{suffix}",
        )

    model.articulation(
        "steering_pivot",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steering,
        origin=Origin(rpy=(0.0, HEAD_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-1.10, upper=1.10),
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

    head_tube = object_model.get_part("head_tube")
    steering = object_model.get_part("steering_assembly")
    pivot = object_model.get_articulation("steering_pivot")

    ctx.expect_overlap(
        steering,
        head_tube,
        axes="z",
        elem_a="steerer",
        elem_b="head_shell",
        min_overlap=0.20,
        name="steerer passes through head tube",
    )
    ctx.expect_within(
        steering,
        head_tube,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_shell",
        margin=0.030,
        name="steerer centered in head tube sleeve",
    )

    crown_box = ctx.part_element_world_aabb(steering, elem="squared_crown")
    blade_0 = ctx.part_element_world_aabb(steering, elem="round_blade_0")
    blade_1 = ctx.part_element_world_aabb(steering, elem="round_blade_1")
    if crown_box and blade_0 and blade_1:
        crown_width = crown_box[1][1] - crown_box[0][1]
        blade_track = abs((blade_0[0][1] + blade_0[1][1]) / 2.0 - (blade_1[0][1] + blade_1[1][1]) / 2.0)
        ctx.check(
            "wide crown spans blade pair",
            crown_width > blade_track + 0.020,
            details=f"crown_width={crown_width:.3f}, blade_track={blade_track:.3f}",
        )
    else:
        ctx.fail("wide crown spans blade pair", "could not measure crown or blade AABBs")

    with ctx.pose({pivot: 0.0}):
        rest_tip = ctx.part_element_world_aabb(steering, elem="pinch_collar")
    with ctx.pose({pivot: 0.70}):
        turned_tip = ctx.part_element_world_aabb(steering, elem="pinch_collar")
    if rest_tip and turned_tip:
        rest_y = (rest_tip[0][1] + rest_tip[1][1]) / 2.0
        turned_y = (turned_tip[0][1] + turned_tip[1][1]) / 2.0
        ctx.check(
            "steering pivot swings handlebar collar",
            turned_y > rest_y + 0.12,
            details=f"rest_y={rest_y:.3f}, turned_y={turned_y:.3f}",
        )
    else:
        ctx.fail("steering pivot swings handlebar collar", "could not measure collar pose")

    return ctx.report()


object_model = build_object_model()
