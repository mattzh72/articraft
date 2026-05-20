from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


FRAME_DEPTH = 0.004
LENS_CENTER_X = 0.035
LENS_OUTER_WIDTH = 0.056
LENS_OUTER_HEIGHT = 0.038
LENS_HOLE_WIDTH = 0.047
LENS_HOLE_HEIGHT = 0.029
OUTER_HINGE_X = 0.067


def _shift_profile(profile, *, x: float = 0.0, y: float = 0.0):
    return [(px + x, py + y) for px, py in profile]


def _box(size, center) -> MeshGeometry:
    return BoxGeometry(size).translate(*center)


def _front_frame_geometry(side: int) -> MeshGeometry:
    """One lens rim plus hinge leaves, authored in the front X/Z plane."""
    center_x = side * LENS_CENTER_X
    outer = _shift_profile(
        superellipse_profile(LENS_OUTER_WIDTH, LENS_OUTER_HEIGHT, 2.15, segments=72),
        x=center_x,
    )
    inner = _shift_profile(
        superellipse_profile(LENS_HOLE_WIDTH, LENS_HOLE_HEIGHT, 2.15, segments=72),
        x=center_x,
    )
    geom = ExtrudeWithHolesGeometry(outer, [inner], FRAME_DEPTH, center=True)
    # Profiles are drawn in X/Y; rotate extrusion depth onto the glasses' Y axis.
    geom.rotate_x(math.pi / 2.0)

    # Small metal leaf tabs connect the rim to the alternating bridge knuckles.
    if side > 0:
        tab_x = 0.007
        for z in (-0.0115, 0.0115):
            geom.merge(_box((0.010, 0.0036, 0.0060), (tab_x, 0.0, z)))
    else:
        geom.merge(_box((0.010, 0.0036, 0.0060), (-0.007, 0.0, 0.0)))

    # Outer temple-hinge leaves, split vertically so the moving middle knuckle clears.
    outer_tab_x = side * (OUTER_HINGE_X - 0.0035)
    for z in (-0.011, 0.011):
        geom.merge(_box((0.008, 0.0036, 0.0060), (outer_tab_x, 0.0, z)))

    # Nose-pad wire mount, overlapped into the inner lower rim.
    geom.merge(_box((0.012, 0.0080, 0.0020), (side * 0.014, -0.004, -0.012)))
    return geom


def _lens_geometry(side: int) -> MeshGeometry:
    profile = _shift_profile(
        superellipse_profile(0.049, 0.031, 2.05, segments=72),
        x=side * LENS_CENTER_X,
    )
    geom = ExtrudeGeometry(profile, 0.0013, center=True)
    geom.rotate_x(math.pi / 2.0)
    return geom


def _temple_geometry(side: int) -> MeshGeometry:
    """Short curved temple arm in a local frame whose origin is its hinge axis."""
    arm = tube_from_spline_points(
        [
            (0.0, -0.008, 0.000),
            (side * 0.003, -0.026, 0.001),
            (side * 0.006, -0.058, -0.001),
            (side * 0.005, -0.083, -0.009),
        ],
        radius=0.00145,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    # Hinge leaf tab sits behind the barrel, leaving the pin bore clear.
    arm.merge(_box((0.004, 0.006, 0.006), (0.0, -0.005, 0.0)))
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_reading_glasses")

    frame_mat = Material("matte_black_acetate", rgba=(0.015, 0.014, 0.012, 1.0))
    lens_mat = Material("clear_blue_lens", rgba=(0.62, 0.84, 1.0, 0.38))
    pin_mat = Material("brushed_steel_pins", rgba=(0.70, 0.70, 0.66, 1.0))
    pad_mat = Material("translucent_nose_pads", rgba=(0.95, 0.86, 0.70, 0.62))

    right_front = model.part("right_front")
    right_front.visual(
        mesh_from_geometry(_front_frame_geometry(1), "right_front_frame"),
        material=frame_mat,
        name="frame",
    )
    right_front.visual(
        mesh_from_geometry(_lens_geometry(1), "right_lens"),
        material=lens_mat,
        name="lens",
    )
    right_front.visual(
        Cylinder(radius=0.0010, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_mat,
        name="bridge_pin",
    )
    right_front.visual(
        Cylinder(radius=0.0023, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=frame_mat,
        name="bridge_barrel_upper",
    )
    right_front.visual(
        Cylinder(radius=0.0023, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.0115)),
        material=frame_mat,
        name="bridge_barrel_lower",
    )
    right_front.visual(
        Cylinder(radius=0.0010, length=0.034),
        origin=Origin(xyz=(OUTER_HINGE_X, 0.0, 0.0)),
        material=pin_mat,
        name="temple_pin",
    )
    for suffix, z in (("upper", 0.011), ("lower", -0.011)):
        right_front.visual(
            Cylinder(radius=0.0022, length=0.008),
            origin=Origin(xyz=(OUTER_HINGE_X, 0.0, z)),
            material=frame_mat,
            name=f"temple_barrel_{suffix}",
        )
    right_front.visual(
        Box((0.005, 0.002, 0.011)),
        origin=Origin(xyz=(0.012, -0.007, -0.012)),
        material=pad_mat,
        name="nose_pad",
    )

    left_front = model.part("left_front")
    left_front.visual(
        mesh_from_geometry(_front_frame_geometry(-1), "left_front_frame"),
        material=frame_mat,
        name="frame",
    )
    left_front.visual(
        mesh_from_geometry(_lens_geometry(-1), "left_lens"),
        material=lens_mat,
        name="lens",
    )
    left_front.visual(
        Cylinder(radius=0.0023, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=frame_mat,
        name="bridge_barrel",
    )
    left_front.visual(
        Cylinder(radius=0.0010, length=0.034),
        origin=Origin(xyz=(-OUTER_HINGE_X, 0.0, 0.0)),
        material=pin_mat,
        name="temple_pin",
    )
    for suffix, z in (("upper", 0.011), ("lower", -0.011)):
        left_front.visual(
            Cylinder(radius=0.0022, length=0.008),
            origin=Origin(xyz=(-OUTER_HINGE_X, 0.0, z)),
            material=frame_mat,
            name=f"temple_barrel_{suffix}",
        )
    left_front.visual(
        Box((0.005, 0.002, 0.011)),
        origin=Origin(xyz=(-0.012, -0.007, -0.012)),
        material=pad_mat,
        name="nose_pad",
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        mesh_from_geometry(_temple_geometry(1), "right_temple_arm"),
        material=frame_mat,
        name="arm",
    )
    right_temple.visual(
        Cylinder(radius=0.0022, length=0.010),
        material=frame_mat,
        name="hinge_barrel",
    )

    left_temple = model.part("left_temple")
    left_temple.visual(
        mesh_from_geometry(_temple_geometry(-1), "left_temple_arm"),
        material=frame_mat,
        name="arm",
    )
    left_temple.visual(
        Cylinder(radius=0.0022, length=0.010),
        material=frame_mat,
        name="hinge_barrel",
    )

    model.articulation(
        "bridge_hinge",
        ArticulationType.REVOLUTE,
        parent=right_front,
        child=left_front,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=0.0, upper=2.35),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=right_front,
        child=right_temple,
        origin=Origin(xyz=(OUTER_HINGE_X, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.5, lower=0.0, upper=1.8),
    )
    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=left_front,
        child=left_temple,
        origin=Origin(xyz=(-OUTER_HINGE_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.5, lower=0.0, upper=1.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    right_front = object_model.get_part("right_front")
    left_front = object_model.get_part("left_front")
    right_temple = object_model.get_part("right_temple")
    left_temple = object_model.get_part("left_temple")
    bridge = object_model.get_articulation("bridge_hinge")
    right_hinge = object_model.get_articulation("right_temple_hinge")
    left_hinge = object_model.get_articulation("left_temple_hinge")

    ctx.allow_overlap(
        right_front,
        left_front,
        elem_a="bridge_pin",
        elem_b="bridge_barrel",
        reason="The center bridge pin is intentionally captured inside the opposite hinge barrel.",
    )
    ctx.allow_overlap(
        right_front,
        right_temple,
        elem_a="temple_pin",
        elem_b="hinge_barrel",
        reason="The right temple hinge pin is intentionally captured inside the moving hinge barrel.",
    )
    ctx.allow_overlap(
        left_front,
        left_temple,
        elem_a="temple_pin",
        elem_b="hinge_barrel",
        reason="The left temple hinge pin is intentionally captured inside the moving hinge barrel.",
    )

    ctx.check(
        "three revolute folding hinges",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (bridge, right_hinge, left_hinge)),
        details="Bridge and both temple arms should be independently revolute.",
    )

    with ctx.pose({bridge: 0.0, right_hinge: 0.0, left_hinge: 0.0}):
        ctx.expect_within(
            right_front,
            left_front,
            axes="xy",
            inner_elem="bridge_pin",
            outer_elem="bridge_barrel",
            margin=0.0002,
            name="bridge pin is coaxial inside center barrel",
        )
        ctx.expect_overlap(
            right_front,
            left_front,
            axes="z",
            elem_a="bridge_pin",
            elem_b="bridge_barrel",
            min_overlap=0.009,
            name="bridge barrel is retained on the pin",
        )
        ctx.expect_within(
            right_front,
            right_temple,
            axes="xy",
            inner_elem="temple_pin",
            outer_elem="hinge_barrel",
            margin=0.0002,
            name="right temple pin is coaxial inside barrel",
        )
        ctx.expect_within(
            left_front,
            left_temple,
            axes="xy",
            inner_elem="temple_pin",
            outer_elem="hinge_barrel",
            margin=0.0002,
            name="left temple pin is coaxial inside barrel",
        )
        right_open = ctx.part_world_aabb(right_temple)
        left_open = ctx.part_world_aabb(left_temple)

    with ctx.pose({right_hinge: 1.55, left_hinge: 1.55}):
        right_folded = ctx.part_world_aabb(right_temple)
        left_folded = ctx.part_world_aabb(left_temple)

    ctx.check(
        "temples fold inward toward lenses",
        right_open is not None
        and left_open is not None
        and right_folded is not None
        and left_folded is not None
        and right_folded[0][0] < right_open[0][0] - 0.035
        and left_folded[1][0] > left_open[1][0] + 0.035,
        details=f"right_open={right_open}, right_folded={right_folded}, left_open={left_open}, left_folded={left_folded}",
    )

    with ctx.pose({bridge: 1.2}):
        folded_front = ctx.part_world_aabb(left_front)

    with ctx.pose({bridge: 0.0}):
        straight_front = ctx.part_world_aabb(left_front)

    ctx.check(
        "center bridge folds one half-front backward",
        straight_front is not None
        and folded_front is not None
        and folded_front[0][1] < straight_front[0][1] - 0.035,
        details=f"straight={straight_front}, folded={folded_front}",
    )

    return ctx.report()


object_model = build_object_model()
