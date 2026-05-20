from __future__ import annotations

from math import pi

import cadquery as cq
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
    mesh_from_cadquery,
)


BARREL_X = 0.070
BASE_TOP_Z = 0.035
BARREL_HEIGHT = 0.560
BARREL_TOP_Z = BASE_TOP_Z + BARREL_HEIGHT
SLIDE_ORIGIN_Z = BARREL_TOP_Z + 0.010
SLIDE_TRAVEL = 0.320


def _hollow_tube(outer_radius: float, inner_radius: float, height: float, name: str):
    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    return mesh_from_cadquery(tube, name, tolerance=0.0007, angular_tolerance=0.05)


def _rounded_box(size: tuple[float, float, float], fillet: float, name: str):
    shape = cq.Workplane("XY").box(*size).edges("|Z").fillet(fillet)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.06)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_barrel_floor_pump")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.004, 0.004, 0.004, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.73, 0.74, 1.0))
    polished_rod = model.material("polished_rod", rgba=(0.92, 0.94, 0.95, 1.0))
    pump_blue = model.material("pump_blue", rgba=(0.03, 0.16, 0.42, 1.0))
    gauge_white = model.material("gauge_white", rgba=(0.92, 0.90, 0.84, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_box((0.430, 0.190, 0.035), 0.030, "rounded_base"),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
        material=matte_black,
        name="base_plate",
    )
    body.visual(
        Box((0.245, 0.072, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + 0.029)),
        material=pump_blue,
        name="lower_manifold",
    )

    for index, x in enumerate((-BARREL_X, BARREL_X)):
        body.visual(
            _hollow_tube(0.028, 0.018, BARREL_HEIGHT, f"barrel_{index}_tube"),
            origin=Origin(xyz=(x, 0.0, BASE_TOP_Z)),
            material=brushed_steel,
            name=f"barrel_{index}",
        )
        body.visual(
            _hollow_tube(0.036, 0.0082, 0.045, f"top_collar_{index}_tube"),
            origin=Origin(xyz=(x, 0.0, BARREL_TOP_Z - 0.018)),
            material=pump_blue,
            name=f"top_collar_{index}",
        )
        body.visual(
            _hollow_tube(0.033, 0.018, 0.040, f"foot_socket_{index}_tube"),
            origin=Origin(xyz=(x, 0.0, BASE_TOP_Z + 0.012)),
            material=pump_blue,
            name=f"foot_socket_{index}",
        )

    # A small front pressure gauge and outlet boss make the static assembly read
    # like a floor pump without adding unrelated moving mechanisms.
    body.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, -0.102, 0.122), rpy=(pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="gauge_bezel",
    )
    body.visual(
        Cylinder(radius=0.029, length=0.020),
        origin=Origin(xyz=(0.0, -0.113, 0.122), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gauge_white,
        name="gauge_face",
    )
    body.visual(
        Box((0.036, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, -0.062, 0.079)),
        material=pump_blue,
        name="gauge_stem",
    )

    for x in (-0.145, 0.145):
        body.visual(
            Box((0.090, 0.038, 0.010)),
            origin=Origin(xyz=(x, -0.055, BASE_TOP_Z + 0.005)),
            material=rubber_black,
            name=f"foot_pad_{0 if x < 0 else 1}",
        )

    handle = model.part("handle")
    for index, x in enumerate((-BARREL_X, BARREL_X)):
        handle.visual(
            Cylinder(radius=0.0085, length=0.450),
            origin=Origin(xyz=(x, 0.0, -0.170)),
            material=polished_rod,
            name=f"rod_{index}",
        )
        handle.visual(
            Cylinder(radius=0.016, length=0.046),
            origin=Origin(xyz=(x, 0.0, 0.046)),
            material=pump_blue,
            name=f"rod_socket_{index}",
        )

    handle.visual(
        Cylinder(radius=0.023, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.075), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber_black,
        name="grip",
    )
    handle.visual(
        Box((0.185, 0.036, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=pump_blue,
        name="handle_bridge",
    )
    for index, x in enumerate((-0.205, 0.205)):
        handle.visual(
            Sphere(radius=0.023),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material=rubber_black,
            name=f"grip_end_{index}",
        )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=SLIDE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    slide = object_model.get_articulation("body_to_handle")

    for index in (0, 1):
        ctx.allow_overlap(
            body,
            handle,
            elem_a=f"top_collar_{index}",
            elem_b=f"rod_{index}",
            reason="Each sliding rod is intentionally captured in a tight top guide bushing at the barrel mouth.",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem=f"rod_{index}",
            outer_elem=f"barrel_{index}",
            margin=0.0,
            name=f"rod_{index} is centered within its barrel footprint",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a=f"rod_{index}",
            elem_b=f"barrel_{index}",
            min_overlap=0.300,
            name=f"rod_{index} has deep insertion at collapsed stroke",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a=f"rod_{index}",
            elem_b=f"top_collar_{index}",
            min_overlap=0.035,
            name=f"rod_{index} passes through its tight top guide",
        )

    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="grip",
        negative_elem="top_collar_0",
        min_gap=0.025,
        max_gap=0.055,
        name="shared grip clears fixed top collars when down",
    )

    rest_pos = ctx.part_world_position(handle)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_pos = ctx.part_world_position(handle)
        for index in (0, 1):
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                inner_elem=f"rod_{index}",
                outer_elem=f"barrel_{index}",
                margin=0.0,
                name=f"rod_{index} stays aligned at full stroke",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a=f"rod_{index}",
                elem_b=f"barrel_{index}",
                min_overlap=0.050,
                name=f"rod_{index} remains retained at full stroke",
            )

    ctx.check(
        "handle translates upward along the twin cylinder axes",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + SLIDE_TRAVEL - 0.005
        and abs(extended_pos[0] - rest_pos[0]) < 0.001
        and abs(extended_pos[1] - rest_pos[1]) < 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
