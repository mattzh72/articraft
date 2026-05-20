from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_housing_shape() -> cq.Workplane:
    """Low rounded service-module housing, authored in meters."""
    base = (
        cq.Workplane("XY")
        .box(0.220, 0.160, 0.036)
        .translate((0.0, 0.0, 0.018))
        .edges("|Z")
        .fillet(0.014)
        .edges(">Z")
        .chamfer(0.003)
    )

    top_boss = cq.Workplane("XY").circle(0.060).extrude(0.010).translate((0.0, 0.0, 0.036))
    pocket = cq.Workplane("XY").circle(0.040).extrude(0.005).translate((0.0, 0.0, 0.041))
    shape = base.union(top_boss).cut(pocket)

    # Small integrated screw pads in the top plate corners make the part read as
    # a compact replaceable service module rather than a plain block.
    for x in (-0.080, 0.080):
        for y in (-0.052, 0.052):
            pad = cq.Workplane("XY").circle(0.010).extrude(0.004).translate((x, y, 0.036))
            recess = cq.Workplane("XY").circle(0.004).extrude(0.002).translate((x, y, 0.039))
            shape = shape.union(pad).cut(recess)

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_joystick")

    dark_plastic = Material("dark_molded_plastic", rgba=(0.055, 0.060, 0.065, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    satin_metal = Material("satin_bushed_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    worn_collar = Material("brushed_root_collar", rgba=(0.42, 0.43, 0.42, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_rounded_housing_shape(), "rounded_housing", tolerance=0.0007),
        material=dark_plastic,
        name="rounded_housing",
    )
    fixed_yoke = TrunnionYokeGeometry(
        (0.150, 0.052, 0.088),
        span_width=0.092,
        trunnion_diameter=0.024,
        trunnion_center_z=0.065,
        base_thickness=0.018,
        corner_radius=0.004,
        center=False,
    )
    housing.visual(
        mesh_from_geometry(fixed_yoke, "fixed_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=satin_metal,
        name="fixed_yoke",
    )

    outer_yoke = model.part("outer_yoke")
    outer_yoke.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="trunnion_stub_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="trunnion_stub_1",
    )
    outer_yoke.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(-0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="trunnion_collar_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="trunnion_collar_1",
    )
    inner_yoke = TrunnionYokeGeometry(
        (0.106, 0.044, 0.078),
        span_width=0.056,
        trunnion_diameter=0.018,
        trunnion_center_z=0.040,
        base_thickness=0.014,
        corner_radius=0.003,
        center=False,
    )
    outer_yoke.visual(
        mesh_from_geometry(inner_yoke, "inner_yoke"),
        # Rotate the helper's trunnion axis from X to Y.  Its bore center is
        # shifted down so the two cardan axes intersect at this part frame.
        origin=Origin(xyz=(0.0, 0.0, -0.040), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=satin_metal,
        name="inner_yoke",
    )
    outer_yoke.visual(
        Box((0.070, 0.026, 0.021)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=satin_metal,
        name="central_saddle",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.0065, length=0.098),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_collar,
        name="pivot_pin",
    )
    lever.visual(
        Cylinder(radius=0.021, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=worn_collar,
        name="root_collar",
    )
    lever.visual(
        Cylinder(radius=0.0085, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=black_rubber,
        name="short_stick",
    )
    lever.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=black_rubber,
        name="tip_cap",
    )

    model.articulation(
        "housing_to_outer_yoke",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=0.45, effort=3.0, velocity=2.0),
    )
    model.articulation(
        "outer_yoke_to_lever",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=lever,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.50, upper=0.50, effort=2.0, velocity=2.5),
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
    outer_yoke = object_model.get_part("outer_yoke")
    lever = object_model.get_part("lever")
    first_axis = object_model.get_articulation("housing_to_outer_yoke")
    second_axis = object_model.get_articulation("outer_yoke_to_lever")

    ctx.check(
        "cardan pair has two orthogonal revolute axes",
        first_axis.articulation_type == ArticulationType.REVOLUTE
        and second_axis.articulation_type == ArticulationType.REVOLUTE
        and abs(sum(a * b for a, b in zip(first_axis.axis, second_axis.axis))) < 1e-6,
        details=f"first={first_axis.axis}, second={second_axis.axis}",
    )
    ctx.expect_overlap(
        outer_yoke,
        housing,
        axes="x",
        elem_a="trunnion_stub_1",
        elem_b="fixed_yoke",
        min_overlap=0.025,
        name="outer trunnion is carried by a fixed yoke bearing",
    )
    ctx.expect_within(
        lever,
        outer_yoke,
        axes="xy",
        inner_elem="root_collar",
        outer_elem="inner_yoke",
        margin=0.004,
        name="root collar sits inside the inner yoke window",
    )

    rest_tip = ctx.part_element_world_aabb(lever, elem="tip_cap")
    with ctx.pose({second_axis: second_axis.motion_limits.upper}):
        tipped_tip = ctx.part_element_world_aabb(lever, elem="tip_cap")
    ctx.check(
        "second cardan axis pitches the short lever",
        rest_tip is not None
        and tipped_tip is not None
        and (tipped_tip[0][0] + tipped_tip[1][0]) / 2.0 > (rest_tip[0][0] + rest_tip[1][0]) / 2.0 + 0.035,
        details=f"rest_tip={rest_tip}, tipped_tip={tipped_tip}",
    )

    with ctx.pose({first_axis: first_axis.motion_limits.upper}):
        rolled_tip = ctx.part_element_world_aabb(lever, elem="tip_cap")
    ctx.check(
        "first cardan axis rolls the yoke and lever as a unit",
        rest_tip is not None
        and rolled_tip is not None
        and abs((rolled_tip[0][1] + rolled_tip[1][1]) / 2.0 - (rest_tip[0][1] + rest_tip[1][1]) / 2.0) > 0.035,
        details=f"rest_tip={rest_tip}, rolled_tip={rolled_tip}",
    )

    return ctx.report()


object_model = build_object_model()
