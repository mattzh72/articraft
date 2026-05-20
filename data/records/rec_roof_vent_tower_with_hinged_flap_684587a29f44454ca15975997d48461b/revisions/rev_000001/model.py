from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _make_housing_shell() -> cq.Workplane:
    width = 0.42
    depth = 0.30
    height = 0.76
    wall = 0.028
    base_z = 0.055
    center_z = base_z + height / 2.0

    outer = _box((width, depth, height), (0.0, 0.0, center_z))
    inner_void = _box((width - 2.0 * wall, depth - 2.0 * wall, height + 0.12), (0.0, 0.0, center_z))
    shell = outer.cut(inner_void)

    outlet_opening = _box((0.300, wall * 4.0, 0.390), (0.0, -depth / 2.0, 0.505))
    shell = shell.cut(outlet_opening)

    base_outer = _box((0.70, 0.52, base_z), (0.0, 0.0, base_z / 2.0))
    base_inner = _box((0.34, 0.22, base_z + 0.04), (0.0, 0.0, base_z / 2.0))
    base_ring = base_outer.cut(base_inner)

    curb_outer = _box((0.50, 0.38, 0.075), (0.0, 0.0, base_z + 0.0375))
    curb_inner = _box((0.34, 0.22, 0.095), (0.0, 0.0, base_z + 0.0375))
    raised_curb = curb_outer.cut(curb_inner)

    return base_ring.union(raised_curb).union(shell)


def _make_outlet_frame() -> cq.Workplane:
    y_center = -0.166
    depth = 0.040
    z_center = 0.505
    outer_w = 0.390
    outer_h = 0.470
    bar = 0.045
    side_h = outer_h

    frame = _box((outer_w, depth, bar), (0.0, y_center, z_center + outer_h / 2.0 - bar / 2.0))
    frame = frame.union(_box((outer_w, depth, bar), (0.0, y_center, z_center - outer_h / 2.0 + bar / 2.0)))
    frame = frame.union(_box((bar, depth, side_h), (outer_w / 2.0 - bar / 2.0, y_center, z_center)))
    frame = frame.union(_box((bar, depth, side_h), (-outer_w / 2.0 + bar / 2.0, y_center, z_center)))
    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized_sheet_metal", rgba=(0.62, 0.66, 0.66, 1.0))
    darker_metal = model.material("dark_hinge_metal", rgba=(0.18, 0.20, 0.21, 1.0))
    roof_black = model.material("black_roof_flashing", rgba=(0.035, 0.033, 0.030, 1.0))
    weathered_edge = model.material("weathered_edge_fold", rgba=(0.50, 0.53, 0.53, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_shell(), "housing_shell", tolerance=0.0012),
        material=galvanized,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_make_outlet_frame(), "outlet_frame", tolerance=0.0008),
        material=weathered_edge,
        name="outlet_frame",
    )

    for side, x in (("side_0", -0.162), ("side_1", 0.162)):
        housing.visual(
            Box((0.095, 0.026, 0.018)),
            origin=Origin(xyz=(x, -0.193, 0.729)),
            material=darker_metal,
            name=f"fixed_hinge_leaf_{side}",
        )
        housing.visual(
            Cylinder(radius=0.012, length=0.095),
            origin=Origin(xyz=(x, -0.205, 0.748), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_metal,
            name=f"fixed_hinge_knuckle_{side}",
        )

    flap = model.part("flap")
    flap.visual(
        Box((0.340, 0.014, 0.535)),
        origin=Origin(xyz=(0.0, 0.0, -0.2925)),
        material=galvanized,
        name="panel",
    )
    flap.visual(
        Cylinder(radius=0.011, length=0.205),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=darker_metal,
        name="center_hinge_knuckle",
    )
    flap.visual(
        Box((0.205, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.001, -0.015)),
        material=darker_metal,
        name="moving_hinge_leaf",
    )
    flap.visual(
        Box((0.340, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.002, -0.555)),
        material=weathered_edge,
        name="bottom_fold",
    )
    for side, x in (("side_0", -0.174), ("side_1", 0.174)):
        flap.visual(
            Box((0.014, 0.019, 0.505)),
            origin=Origin(xyz=(x, -0.001, -0.3025)),
            material=weathered_edge,
            name=f"side_fold_{side}",
        )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(0.0, -0.205, 0.748)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
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

    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("housing_to_flap")

    ctx.expect_gap(
        housing,
        flap,
        axis="y",
        positive_elem="outlet_frame",
        negative_elem="panel",
        min_gap=0.006,
        max_gap=0.030,
        name="closed flap sits just in front of outlet frame",
    )
    ctx.expect_overlap(
        housing,
        flap,
        axes="xz",
        elem_a="outlet_frame",
        elem_b="panel",
        min_overlap=0.31,
        name="long flap covers the framed outlet",
    )

    closed_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({hinge: 1.10}):
        open_aabb = ctx.part_world_aabb(flap)

    ctx.check(
        "flap opens outward and upward from top hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.28
        and open_aabb[0][2] > closed_aabb[0][2] + 0.16,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.check(
        "only the weather flap is articulated",
        len(object_model.articulations) == 1 and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )

    return ctx.report()


object_model = build_object_model()
