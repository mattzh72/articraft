from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BASE_RADIUS = 0.135
BASE_HEIGHT = 0.025
STEM_RADIUS = 0.012
HINGE_X = 0.220
HINGE_Z = 1.480
SWITCH_Z = 0.900
SWITCH_X = 0.048


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.63, 0.54, 0.33, 1.0))
    warm_white = model.material("warm_white", rgba=(0.93, 0.91, 0.86, 1.0))
    switch_black = model.material("switch_black", rgba=(0.06, 0.06, 0.07, 1.0))

    base_stem = model.part("base_stem")
    base_stem.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
        material=matte_black,
        name="base_disk",
    )
    base_stem.visual(
        Cylinder(radius=0.023, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=satin_brass,
        name="stem_socket",
    )

    stem_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.018),
            (0.0, 0.0, 0.52),
            (0.020, 0.0, 1.02),
            (0.095, 0.0, 1.31),
            (0.198, 0.0, 1.468),
        ],
        radius=STEM_RADIUS,
        samples_per_segment=22,
        radial_segments=24,
        cap_ends=True,
    )
    base_stem.visual(
        _mesh("floor_lamp_stem", stem_geom),
        material=satin_brass,
        name="stem_tube",
    )

    base_stem.visual(
        Box((0.032, 0.024, 0.052)),
        origin=Origin(xyz=(0.024, 0.0, SWITCH_Z)),
        material=matte_black,
        name="switch_back",
    )
    base_stem.visual(
        Box((0.016, 0.024, 0.008)),
        origin=Origin(xyz=(0.040, 0.0, SWITCH_Z + 0.022)),
        material=matte_black,
        name="switch_top",
    )
    base_stem.visual(
        Box((0.016, 0.024, 0.008)),
        origin=Origin(xyz=(0.040, 0.0, SWITCH_Z - 0.022)),
        material=matte_black,
        name="switch_bottom",
    )
    base_stem.visual(
        Box((0.016, 0.004, 0.036)),
        origin=Origin(xyz=(0.040, 0.010, SWITCH_Z)),
        material=matte_black,
        name="switch_side_0",
    )
    base_stem.visual(
        Box((0.016, 0.004, 0.036)),
        origin=Origin(xyz=(0.040, -0.010, SWITCH_Z)),
        material=matte_black,
        name="switch_side_1",
    )

    base_stem.visual(
        Box((0.016, 0.058, 0.012)),
        origin=Origin(xyz=(HINGE_X - 0.016, 0.0, HINGE_Z)),
        material=matte_black,
        name="yoke_bridge",
    )
    base_stem.visual(
        Box((0.020, 0.016, 0.020)),
        origin=Origin(xyz=(0.198, 0.0, 1.472)),
        material=matte_black,
        name="yoke_post",
    )
    base_stem.visual(
        Box((0.012, 0.010, 0.020)),
        origin=Origin(xyz=(HINGE_X - 0.010, 0.020, HINGE_Z)),
        material=matte_black,
        name="yoke_cheek_0",
    )
    base_stem.visual(
        Box((0.012, 0.010, 0.020)),
        origin=Origin(xyz=(HINGE_X - 0.010, -0.020, HINGE_Z)),
        material=matte_black,
        name="yoke_cheek_1",
    )
    base_stem.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(HINGE_X, 0.019, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="yoke_collar_0",
    )
    base_stem.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(HINGE_X, -0.019, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="yoke_collar_1",
    )

    shade = model.part("shade")
    shade_geom = LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.000),
            (0.024, 0.010),
            (0.038, 0.028),
            (0.055, 0.060),
            (0.072, 0.105),
            (0.087, 0.150),
        ],
        [
            (0.000, 0.006),
            (0.017, 0.014),
            (0.030, 0.032),
            (0.046, 0.060),
            (0.063, 0.104),
            (0.079, 0.145),
        ],
        segments=72,
    ).rotate_y(math.pi / 2.0)
    shade.visual(
        _mesh("floor_lamp_shade", shade_geom),
        origin=Origin(xyz=(0.024, 0.0, -0.060)),
        material=warm_white,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.0055, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="hinge_barrel",
    )
    shade.visual(
        Box((0.030, 0.016, 0.040)),
        origin=Origin(xyz=(0.017, 0.0, -0.020)),
        material=matte_black,
        name="shade_strap",
    )

    rocker = model.part("rocker")
    rocker.visual(
        Box((0.007, 0.016, 0.030)),
        origin=Origin(xyz=(-0.0035, 0.0, 0.0)),
        material=switch_black,
        name="rocker_paddle",
    )

    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=base_stem,
        child=shade,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(0.0, -0.55, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=-0.15,
            upper=0.85,
        ),
    )
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=base_stem,
        child=rocker,
        origin=Origin(xyz=(SWITCH_X, 0.0, SWITCH_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-0.28,
            upper=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_stem = object_model.get_part("base_stem")
    shade = object_model.get_part("shade")
    rocker = object_model.get_part("rocker")
    shade_tilt = object_model.get_articulation("shade_tilt")
    rocker_pivot = object_model.get_articulation("rocker_pivot")

    ctx.expect_within(
        rocker,
        base_stem,
        axes="yz",
        inner_elem="rocker_paddle",
        outer_elem="switch_back",
        margin=0.004,
        name="rocker stays within the switch housing footprint",
    )

    rest_aabb = None
    open_aabb = None
    with ctx.pose({shade_tilt: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_tilt: shade_tilt.motion_limits.upper}):
        open_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")

    rest_center = _midpoint(rest_aabb)
    open_center = _midpoint(open_aabb)
    ctx.check(
        "shade lifts upward when tilted",
        rest_center is not None
        and open_center is not None
        and open_center[2] > rest_center[2] + 0.06,
        details=f"rest_center={rest_center}, open_center={open_center}",
    )
    ctx.check(
        "shade projects forward from the curved stem",
        rest_aabb is not None and rest_aabb[1][0] > 0.28,
        details=f"rest_aabb={rest_aabb}",
    )

    rocker_low = None
    rocker_high = None
    with ctx.pose({rocker_pivot: rocker_pivot.motion_limits.lower}):
        rocker_low = ctx.part_element_world_aabb(rocker, elem="rocker_paddle")
    with ctx.pose({rocker_pivot: rocker_pivot.motion_limits.upper}):
        rocker_high = ctx.part_element_world_aabb(rocker, elem="rocker_paddle")

    ctx.check(
        "rocker visibly pivots in the housing",
        rocker_low is not None
        and rocker_high is not None
        and abs(_midpoint(rocker_high)[2] - _midpoint(rocker_low)[2]) > 0.0015,
        details=f"rocker_low={rocker_low}, rocker_high={rocker_high}",
    )

    return ctx.report()


object_model = build_object_model()
