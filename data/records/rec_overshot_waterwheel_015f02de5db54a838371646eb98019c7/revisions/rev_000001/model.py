from __future__ import annotations

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_mill_waterwheel")

    timber = model.material("weathered_timber", color=(0.48, 0.29, 0.13, 1.0))
    dark_timber = model.material("dark_endgrain", color=(0.25, 0.14, 0.07, 1.0))
    wet_timber = model.material("wet_trough_wood", color=(0.32, 0.19, 0.09, 1.0))
    water = model.material("shallow_water", color=(0.12, 0.34, 0.55, 0.55))
    iron = model.material("blackened_iron", color=(0.04, 0.04, 0.035, 1.0))

    base = model.part("base")
    # Shallow mill race/trough. The top walls stay below the lower paddles so the
    # wheel visibly sits partly above the trough without colliding with it.
    base.visual(
        Box((2.10, 2.60, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=wet_timber,
        name="trough_floor",
    )
    base.visual(
        Box((2.10, 2.42, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=water,
        name="water_surface",
    )
    for side, x in enumerate((-0.99, 0.99)):
        base.visual(
            Box((0.12, 2.60, 0.32)),
            origin=Origin(xyz=(x, 0.0, 0.20)),
            material=wet_timber,
            name=f"trough_side_{side}",
        )
    for end, y in enumerate((-1.24, 1.24)):
        base.visual(
            Box((2.10, 0.12, 0.22)),
            origin=Origin(xyz=(0.0, y, 0.17)),
            material=wet_timber,
            name=f"trough_end_{end}",
        )

    # Twin timber bearing blocks, built as connected split pillow blocks with a
    # clear square window around the rotating axle.
    for side, x in enumerate((-0.62, 0.62)):
        base.visual(
            Box((0.26, 0.28, 1.43)),
            origin=Origin(xyz=(x, 0.0, 0.835)),
            material=timber,
            name=f"support_post_{side}",
        )
        base.visual(
            Box((0.36, 0.42, 0.12)),
            origin=Origin(xyz=(x, 0.0, 1.525)),
            material=dark_timber,
            name=f"bearing_saddle_{side}",
        )
        for cheek, y in enumerate((-0.19, 0.19)):
            base.visual(
                Box((0.36, 0.10, 0.44)),
                origin=Origin(xyz=(x, y, 1.65)),
                material=timber,
                name=f"bearing_cheek_{side}_{cheek}",
            )
        base.visual(
            Box((0.36, 0.42, 0.10)),
            origin=Origin(xyz=(x, 0.0, 1.84)),
            material=dark_timber,
            name=f"bearing_cap_{side}",
        )

    wheel = model.part("wheel")
    # Cylinders are native-Z aligned; rotate them so the real axle is horizontal
    # along the model X axis.
    axle_rot = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    wheel.visual(
        Cylinder(radius=0.065, length=1.52),
        origin=axle_rot,
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.20, length=0.74),
        origin=axle_rot,
        material=dark_timber,
        name="hub",
    )
    for collar, x in enumerate((-0.31, 0.31)):
        wheel.visual(
            Cylinder(radius=0.235, length=0.09),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=iron,
            name=f"hub_band_{collar}",
        )

    paddle_count = 16
    rim_radius = 1.24
    segment_angle = 2.0 * math.pi / paddle_count
    rim_segment_length = 2.0 * rim_radius * math.sin(segment_angle / 2.0) * 1.16
    for i in range(paddle_count):
        theta = i * segment_angle
        radial_y = math.cos(theta)
        radial_z = math.sin(theta)
        # Deep side rims: two polygonal timber hoops separated along the axle.
        for side, x in enumerate((-0.35, 0.35)):
            wheel.visual(
                Box((0.12, rim_segment_length, 0.12)),
                origin=Origin(
                    xyz=(x, rim_radius * radial_y, rim_radius * radial_z),
                    rpy=(theta + math.pi / 2.0, 0.0, 0.0),
                ),
                material=timber,
                name=f"rim_{side}_{i}",
            )

        # Cross-width paddles are evenly spaced and overlap the side hoops,
        # forming a deep bucket-like rim rather than a flat decorative wheel.
        wheel.visual(
            Box((0.82, 0.38, 0.055)),
            origin=Origin(
                xyz=(0.0, 1.23 * radial_y, 1.23 * radial_z),
                rpy=(theta, 0.0, 0.0),
            ),
            material=timber,
            name=f"paddle_{i}",
        )
        wheel.visual(
            Box((0.82, 0.055, 0.22)),
            origin=Origin(
                xyz=(
                    0.0,
                    1.34 * radial_y - 0.08 * math.sin(theta),
                    1.34 * radial_z + 0.08 * math.cos(theta),
                ),
                rpy=(theta, 0.0, 0.0),
            ),
            material=dark_timber,
            name=f"bucket_lip_{i}",
        )

    for i in range(8):
        theta = i * (2.0 * math.pi / 8.0)
        radial_y = math.cos(theta)
        radial_z = math.sin(theta)
        for side, x in enumerate((-0.27, 0.27)):
            wheel.visual(
                Box((0.10, 1.08, 0.075)),
                origin=Origin(
                    xyz=(x, 0.70 * radial_y, 0.70 * radial_z),
                    rpy=(theta, 0.0, 0.0),
                ),
                material=timber,
                name=f"spoke_{side}_{i}",
            )

    model.articulation(
        "axle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.65)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.2),
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
    wheel = object_model.get_part("wheel")
    axle = object_model.get_articulation("axle")
    paddle_front = wheel.get_visual(f"paddle_{0}")
    paddle_lower = wheel.get_visual(f"paddle_{12}")
    bearing_side = base.get_visual(f"bearing_saddle_{1}")

    ctx.check(
        "continuous horizontal axle",
        axle.articulation_type == ArticulationType.CONTINUOUS and tuple(axle.axis) == (1.0, 0.0, 0.0),
        details=f"type={axle.articulation_type}, axis={axle.axis}",
    )
    paddle_names = [visual.name for visual in wheel.visuals if visual.name and visual.name.startswith("paddle_")]
    ctx.check(
        "sixteen evenly named paddles",
        len(paddle_names) == 16,
        details=f"paddles={paddle_names}",
    )
    ctx.expect_gap(
        wheel,
        base,
        axis="z",
        positive_elem=paddle_lower,
        negative_elem="trough_floor",
        min_gap=0.02,
        max_gap=0.25,
        name="lower paddle clears trough floor",
    )
    ctx.expect_gap(
        base,
        wheel,
        axis="x",
        positive_elem=bearing_side,
        negative_elem=paddle_front,
        min_gap=0.025,
        name="side bearing clears paddle width",
    )

    start_box = ctx.part_element_world_aabb(wheel, elem=paddle_front)
    with ctx.pose({axle: math.pi / 2.0}):
        turned_box = ctx.part_element_world_aabb(wheel, elem=paddle_front)
    if start_box is not None and turned_box is not None:
        start_center = tuple((start_box[0][i] + start_box[1][i]) / 2.0 for i in range(3))
        turned_center = tuple((turned_box[0][i] + turned_box[1][i]) / 2.0 for i in range(3))
    else:
        start_center = None
        turned_center = None
    ctx.check(
        "paddle follows axle rotation",
        start_center is not None
        and turned_center is not None
        and turned_center[2] > start_center[2] + 0.9,
        details=f"start={start_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
