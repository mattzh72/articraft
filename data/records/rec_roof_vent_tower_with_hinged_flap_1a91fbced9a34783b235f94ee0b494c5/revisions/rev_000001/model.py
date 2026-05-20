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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GALVANIZED = Material("galvanized_sheet_metal", rgba=(0.62, 0.66, 0.66, 1.0))
DARK_GALVANIZED = Material("dark_galvanized_edges", rgba=(0.38, 0.41, 0.42, 1.0))
BLACK_SHADOW = Material("dark_vent_interior", rgba=(0.035, 0.04, 0.045, 1.0))


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _union_all(shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _housing_mesh():
    """Connected sheet-metal vent tower with a raised framed side outlet."""
    shapes = [
        # Rooftop curb/flange and upright duct shell.
        _box_shape((0.66, 0.54, 0.035), (0.0, 0.0, 0.0175)),
        _box_shape((0.40, 0.035, 0.765), (0.0, 0.1425, 0.4175)),
        _box_shape((0.40, 0.035, 0.765), (0.0, -0.1425, 0.4175)),
        _box_shape((0.035, 0.32, 0.765), (-0.1825, 0.0, 0.4175)),
        _box_shape((0.40, 0.32, 0.035), (0.0, 0.0, 0.7825)),
        _box_shape((0.035, 0.32, 0.355), (0.1825, 0.0, 0.2125)),
        # Protruding outlet frame around the upper front opening.
        _box_shape((0.075, 0.42, 0.055), (0.2275, 0.0, 0.7725)),
        _box_shape((0.075, 0.42, 0.055), (0.2275, 0.0, 0.4275)),
        _box_shape((0.075, 0.055, 0.40), (0.2275, 0.1825, 0.60)),
        _box_shape((0.075, 0.055, 0.40), (0.2275, -0.1825, 0.60)),
        # Short raised seams make the shell read as folded sheet metal.
        _box_shape((0.012, 0.018, 0.67), (-0.005, 0.162, 0.405)),
        _box_shape((0.012, 0.018, 0.67), (-0.005, -0.162, 0.405)),
        _box_shape((0.44, 0.018, 0.012), (0.005, 0.162, 0.76)),
        _box_shape((0.44, 0.018, 0.012), (0.005, -0.162, 0.76)),
    ]
    return _union_all(shapes)


def _frame_face_mesh():
    """Thin front ring used as the crisp outlet lip and as a test target."""
    shapes = [
        _box_shape((0.006, 0.420, 0.055), (0.268, 0.0, 0.7725)),
        _box_shape((0.006, 0.420, 0.055), (0.268, 0.0, 0.4275)),
        _box_shape((0.006, 0.055, 0.400), (0.268, 0.1825, 0.600)),
        _box_shape((0.006, 0.055, 0.400), (0.268, -0.1825, 0.600)),
    ]
    return _union_all(shapes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower_weather_flap")

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_mesh(), "vent_housing"),
        material=GALVANIZED,
        name="outlet_frame",
    )
    housing.visual(
        mesh_from_cadquery(_frame_face_mesh(), "frame_face"),
        material=DARK_GALVANIZED,
        name="frame_face",
    )
    # A set-back, darkened duct interior is visible when the flap is raised,
    # but it does not cap the outlet at the front plane.
    housing.visual(
        Box((0.010, 0.295, 0.285)),
        origin=Origin(xyz=(0.172, 0.0, 0.600)),
        material=BLACK_SHADOW,
        name="interior_shadow",
    )

    hinge_x = 0.282
    hinge_z = 0.814
    for y, suffix in ((-0.105, "0"), (0.105, "1")):
        housing.visual(
            Box((0.026, 0.052, 0.025)),
            origin=Origin(xyz=(0.267, y, 0.798)),
            material=DARK_GALVANIZED,
            name=f"fixed_hinge_leaf_{suffix}",
        )
        housing.visual(
            Cylinder(radius=0.009, length=0.050),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=DARK_GALVANIZED,
            name=f"fixed_hinge_knuckle_{suffix}",
        )
    housing.visual(
        Cylinder(radius=0.004, length=0.270),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=DARK_GALVANIZED,
        name="hinge_pin",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.014, 0.360, 0.360)),
        origin=Origin(xyz=(0.016, 0.0, -0.195)),
        material=GALVANIZED,
        name="flap_panel",
    )
    flap.visual(
        Box((0.020, 0.120, 0.024)),
        origin=Origin(xyz=(0.011, 0.0, -0.017)),
        material=DARK_GALVANIZED,
        name="moving_hinge_leaf",
    )
    flap.visual(
        Cylinder(radius=0.009, length=0.105),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=DARK_GALVANIZED,
        name="moving_hinge_knuckle",
    )
    flap.visual(
        Box((0.018, 0.340, 0.018)),
        origin=Origin(xyz=(0.019, 0.0, -0.382)),
        material=DARK_GALVANIZED,
        name="bottom_hem",
    )
    flap.visual(
        Box((0.010, 0.018, 0.300)),
        origin=Origin(xyz=(0.024, 0.125, -0.210)),
        material=DARK_GALVANIZED,
        name="stiffening_rib_0",
    )
    flap.visual(
        Box((0.010, 0.018, 0.300)),
        origin=Origin(xyz=(0.024, -0.125, -0.210)),
        material=DARK_GALVANIZED,
        name="stiffening_rib_1",
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.20),
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

    ctx.allow_overlap(
        housing,
        flap,
        elem_a="hinge_pin",
        elem_b="moving_hinge_knuckle",
        reason="The short top hinge uses a captured pin passing through the flap knuckle.",
    )

    ctx.expect_gap(
        flap,
        housing,
        axis="x",
        min_gap=0.015,
        max_gap=0.040,
        positive_elem="flap_panel",
        negative_elem="frame_face",
        name="closed flap sits just proud of the framed outlet",
    )
    ctx.expect_overlap(
        flap,
        housing,
        axes="yz",
        min_overlap=0.300,
        elem_a="flap_panel",
        elem_b="frame_face",
        name="flap covers the outlet opening footprint",
    )
    ctx.expect_within(
        housing,
        flap,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="moving_hinge_knuckle",
        margin=0.001,
        name="hinge pin is captured inside the moving knuckle cross-section",
    )
    ctx.expect_overlap(
        housing,
        flap,
        axes="y",
        min_overlap=0.090,
        elem_a="hinge_pin",
        elem_b="moving_hinge_knuckle",
        name="moving knuckle retains length on the hinge pin",
    )

    closed_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({hinge: 1.05}):
        raised_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "flap swings outward and upward",
        closed_panel is not None
        and raised_panel is not None
        and raised_panel[1][0] > closed_panel[1][0] + 0.10
        and raised_panel[0][2] > closed_panel[0][2] + 0.08,
        details=f"closed={closed_panel}, raised={raised_panel}",
    )

    ctx.check(
        "hinge is a horizontal top-edge revolute joint",
        hinge.axis == (0.0, -1.0, 0.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper >= 1.0,
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
