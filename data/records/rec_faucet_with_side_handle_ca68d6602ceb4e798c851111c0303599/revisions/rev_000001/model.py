from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_eyewash_faucet")

    chrome = model.material("chrome", rgba=(0.77, 0.79, 0.81, 1.0))
    stainless = model.material("stainless", rgba=(0.67, 0.69, 0.72, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    handle_polymer = model.material("handle_polymer", rgba=(0.14, 0.16, 0.18, 1.0))
    eyewash_green = model.material("eyewash_green", rgba=(0.30, 0.57, 0.20, 1.0))
    nozzle_gray = model.material("nozzle_gray", rgba=(0.36, 0.39, 0.42, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=stainless,
        name="deck_flange",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=chrome,
        name="valve_body",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=chrome,
        name="swivel_collar",
    )
    body.visual(
        Box((0.022, 0.028, 0.022)),
        origin=Origin(xyz=(0.011, 0.0, 0.032)),
        material=chrome,
        name="handle_mount",
    )
    for index, y_center in enumerate((-0.011, 0.011)):
        body.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(0.032, y_center, 0.038), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"pivot_ear_{index}",
        )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.017, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=chrome,
        name="spout_shank",
    )
    spout.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=stainless,
        name="spout_ring",
    )
    spout_tube = tube_from_spline_points(
        [
            (0.0, 0.0, 0.016),
            (0.0, 0.0, 0.110),
            (0.0, 0.045, 0.170),
            (0.0, 0.112, 0.202),
        ],
        radius=0.0125,
        samples_per_segment=18,
        radial_segments=24,
        cap_ends=True,
    )
    spout.visual(
        mesh_from_geometry(spout_tube, "spout_tube"),
        material=chrome,
        name="tube",
    )
    spout.visual(
        Box((0.074, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.145, 0.195)),
        material=chrome,
        name="head_body",
    )
    for index, x_center in enumerate((-0.022, 0.022)):
        spout.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(xyz=(x_center, 0.145, 0.209)),
            material=nozzle_gray,
            name=f"spray_nozzle_{index}",
        )
    for index, x_center in enumerate((-0.028, 0.028)):
        spout.visual(
            Box((0.012, 0.008, 0.012)),
            origin=Origin(xyz=(x_center, 0.113, 0.208)),
            material=stainless,
            name=f"hinge_tab_{index}",
        )
    for index, x_center in enumerate((-0.026, 0.026)):
        spout.visual(
            Cylinder(radius=0.004, length=0.010),
            origin=Origin(xyz=(x_center, 0.108, 0.214), rpy=(0.0, pi / 2.0, 0.0)),
            material=stainless,
            name=f"hinge_ear_{index}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="handle_hub",
    )
    handle.visual(
        Box((0.024, 0.012, 0.008)),
        origin=Origin(xyz=(0.012, 0.0, -0.008)),
        material=graphite,
        name="handle_bridge",
    )
    handle.visual(
        Box((0.052, 0.012, 0.012)),
        origin=Origin(xyz=(0.038, 0.0, -0.018)),
        material=handle_polymer,
        name="handle_arm",
    )
    handle.visual(
        Box((0.028, 0.028, 0.012)),
        origin=Origin(xyz=(0.067, 0.0, -0.022)),
        material=handle_polymer,
        name="handle_grip",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.081, 0.0, -0.022), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="handle_cap",
    )

    cover = model.part("cover")
    cover.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="cover_barrel",
    )
    cover.visual(
        Box((0.048, 0.007, 0.006)),
        origin=Origin(xyz=(0.0, 0.0035, 0.001)),
        material=eyewash_green,
        name="cover_bridge",
    )
    cover.visual(
        Box((0.078, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.033, 0.010)),
        material=eyewash_green,
        name="cover_shell",
    )
    cover.visual(
        Box((0.068, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.060, 0.007)),
        material=eyewash_green,
        name="front_lip",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.032, 0.0, 0.038)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=0.95),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=cover,
        origin=Origin(xyz=(0.0, 0.112, 0.214)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    handle = object_model.get_part("handle")
    cover = object_model.get_part("cover")

    spout_swivel = object_model.get_articulation("spout_swivel")
    handle_pivot = object_model.get_articulation("handle_pivot")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.expect_overlap(
        cover,
        spout,
        axes="xy",
        elem_a="cover_shell",
        elem_b="head_body",
        min_overlap=0.050,
        name="closed cover spans the eyewash head",
    )
    ctx.expect_gap(
        cover,
        spout,
        axis="z",
        positive_elem="cover_shell",
        negative_elem="head_body",
        min_gap=0.010,
        max_gap=0.020,
        name="closed cover sits just above the eyewash head",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="x",
        positive_elem="handle_grip",
        negative_elem="valve_body",
        min_gap=0.018,
        name="side handle projects outward from the body",
    )

    rest_head = _aabb_center(ctx.part_element_world_aabb(spout, elem="head_body"))
    with ctx.pose({spout_swivel: pi / 2.0}):
        quarter_head = _aabb_center(ctx.part_element_world_aabb(spout, elem="head_body"))
    ctx.check(
        "spout rotates around the vertical base axis",
        rest_head is not None
        and quarter_head is not None
        and abs(quarter_head[0] + rest_head[1]) < 0.012
        and abs(quarter_head[1] - rest_head[0]) < 0.012
        and abs(quarter_head[2] - rest_head[2]) < 0.004,
        details=f"rest={rest_head}, quarter_turn={quarter_head}",
    )

    rest_grip = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_grip"))
    with ctx.pose({handle_pivot: 0.85}):
        raised_grip = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_grip"))
    ctx.check(
        "side handle lifts upward on its pivot",
        rest_grip is not None
        and raised_grip is not None
        and raised_grip[2] > rest_grip[2] + 0.030,
        details=f"rest={rest_grip}, raised={raised_grip}",
    )

    rest_cover = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_shell"))
    with ctx.pose({cover_hinge: 1.30}):
        open_cover = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_shell"))
    ctx.check(
        "eyewash cover flips upward and back",
        rest_cover is not None
        and open_cover is not None
        and open_cover[2] > rest_cover[2] + 0.020
        and open_cover[1] < rest_cover[1] - 0.020,
        details=f"closed={rest_cover}, open={open_cover}",
    )

    return ctx.report()


object_model = build_object_model()
