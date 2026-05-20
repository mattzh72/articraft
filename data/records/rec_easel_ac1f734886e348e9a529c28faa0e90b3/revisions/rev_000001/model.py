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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


MAST_RADIUS = 0.0105
SLEEVE_OUTER_RADIUS = 0.0145
SLEEVE_INNER_RADIUS = 0.0112
SLEEVE_LENGTH = 0.30
SLEEVE_TOP_Z = 0.95
SLEEVE_CENTER_Z = SLEEVE_TOP_Z - SLEEVE_LENGTH / 2.0

LEFT_HINGE = (-0.055, 0.030, 0.985)
RIGHT_HINGE = (0.055, 0.030, 0.985)
REAR_HINGE = (0.0, -0.052, 0.990)

LEFT_FOOT = (-0.325, 0.245, 0.013)
RIGHT_FOOT = (0.325, 0.245, 0.013)
REAR_FOOT = (0.0, -0.395, 0.013)


def _unit(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vec))
    return (vec[0] / length, vec[1] / length, vec[2] / length)


def _rpy_for_z_axis(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    dx, dy, dz = _unit(direction)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return (0.0, pitch, yaw)


def _yaw_for_xy(direction: tuple[float, float, float]) -> float:
    return math.atan2(direction[1], direction[0])


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    shape = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )
    return mesh_from_cadquery(shape, name)


def _shelf_collar_mesh(name: str):
    collar = (
        cq.Workplane("XY")
        .box(0.056, 0.034, 0.084)
        .faces(">Z")
        .workplane()
        .circle(MAST_RADIUS)
        .cutThruAll()
        .edges("|Z")
        .fillet(0.004)
    )
    return mesh_from_cadquery(collar, name)


def _add_crown_lug(
    crown,
    *,
    radial: tuple[float, float, float],
    pivot: tuple[float, float, float],
    name_prefix: str,
    inset: float = 0.0,
    metal,
) -> None:
    yaw = _yaw_for_xy(radial)
    tangent = (-radial[1], radial[0], 0.0)
    lug_center = (
        pivot[0] - radial[0] * inset,
        pivot[1] - radial[1] * inset,
        pivot[2] - 0.010,
    )

    crown.visual(
        Box((0.014, 0.008, 0.028)),
        origin=Origin(xyz=lug_center, rpy=(0.0, 0.0, yaw)),
        material=metal,
        name=f"{name_prefix}_lug",
    )
    crown.visual(
        Cylinder(radius=0.0036, length=0.004),
        origin=Origin(xyz=pivot, rpy=_rpy_for_z_axis(tangent)),
        material=metal,
        name=f"{name_prefix}_pin",
    )


def _add_leg_part(
    model: ArticulatedObject,
    *,
    name: str,
    pivot: tuple[float, float, float],
    foot: tuple[float, float, float],
    metal,
    rubber,
) -> None:
    leg = model.part(name)

    radial = _unit((foot[0], foot[1], 0.0))
    tangent = (-radial[1], radial[0], 0.0)
    yaw = _yaw_for_xy(radial)
    tab_offset = tuple(component * 0.0085 for component in tangent)

    leg.visual(
        Box((0.016, 0.007, 0.030)),
        origin=Origin(xyz=(tab_offset[0], tab_offset[1], -0.010), rpy=(0.0, 0.0, yaw)),
        material=metal,
        name="tab_0",
    )
    leg.visual(
        Box((0.016, 0.007, 0.030)),
        origin=Origin(xyz=(-tab_offset[0], -tab_offset[1], -0.010), rpy=(0.0, 0.0, yaw)),
        material=metal,
        name="tab_1",
    )
    leg.visual(
        Box((0.020, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.036), rpy=(0.0, 0.0, yaw)),
        material=metal,
        name="hinge_block",
    )

    foot_vector = (foot[0] - pivot[0], foot[1] - pivot[1], foot[2] - pivot[2])
    direction = _unit(foot_vector)
    tube_length = math.sqrt(sum(component * component for component in foot_vector)) - 0.036
    tube_origin = tuple(direction[i] * (0.028 + tube_length / 2.0) for i in range(3))

    leg.visual(
        Cylinder(radius=0.0085, length=tube_length),
        origin=Origin(xyz=tube_origin, rpy=_rpy_for_z_axis(direction)),
        material=metal,
        name="leg_tube",
    )
    leg.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=foot_vector),
        material=rubber,
        name="foot_pad",
    )

    model.articulation(
        f"crown_to_{name}",
        ArticulationType.REVOLUTE,
        parent="crown",
        child=leg,
        origin=Origin(xyz=pivot),
        axis=tangent,
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.60,
            upper=0.15,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_artist_easel")

    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.67, 0.69, 0.72, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.69, 0.57, 0.40, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    crown = model.part("crown")
    crown.visual(
        Box((0.026, 0.040, 0.120)),
        origin=Origin(xyz=(-0.044, 0.0, 1.020)),
        material=graphite,
        name="head_0",
    )
    crown.visual(
        Box((0.026, 0.040, 0.120)),
        origin=Origin(xyz=(0.044, 0.0, 1.020)),
        material=graphite,
        name="head_1",
    )
    crown.visual(
        Box((0.102, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.024, 1.000)),
        material=graphite,
        name="front_bridge",
    )
    crown.visual(
        Box((0.114, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, -0.024, 1.068)),
        material=graphite,
        name="rear_bridge",
    )
    crown.visual(
        Box((0.030, 0.014, 0.110)),
        origin=Origin(xyz=(0.0, -0.034, 1.030)),
        material=graphite,
        name="rear_anchor",
    )
    crown.visual(
        Box((0.014, 0.030, 0.150)),
        origin=Origin(xyz=(-0.027, 0.0, 0.905)),
        material=graphite,
        name="rail_0",
    )
    crown.visual(
        Box((0.014, 0.030, 0.150)),
        origin=Origin(xyz=(0.027, 0.0, 0.905)),
        material=graphite,
        name="rail_1",
    )
    crown.visual(
        Box((0.008, 0.028, 0.090)),
        origin=Origin(xyz=(-0.018, 0.0, 0.915)),
        material=graphite,
        name="clamp_cheek_0",
    )
    crown.visual(
        Box((0.008, 0.028, 0.090)),
        origin=Origin(xyz=(0.018, 0.0, 0.915)),
        material=graphite,
        name="clamp_cheek_1",
    )
    crown.visual(
        _tube_mesh(
            outer_radius=SLEEVE_OUTER_RADIUS,
            inner_radius=SLEEVE_INNER_RADIUS,
            length=SLEEVE_LENGTH,
            name="easel_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_CENTER_Z)),
        material=satin_aluminum,
        name="sleeve",
    )
    crown.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(0.039, 0.0, 0.930), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="clamp_stem",
    )
    crown.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.053, 0.0, 0.930)),
        material=rubber,
        name="clamp_knob",
    )

    _add_crown_lug(
        crown,
        radial=_unit((LEFT_FOOT[0], LEFT_FOOT[1], 0.0)),
        pivot=LEFT_HINGE,
        name_prefix="left",
        metal=graphite,
    )
    _add_crown_lug(
        crown,
        radial=_unit((RIGHT_FOOT[0], RIGHT_FOOT[1], 0.0)),
        pivot=RIGHT_HINGE,
        name_prefix="right",
        metal=graphite,
    )
    _add_crown_lug(
        crown,
        radial=_unit((REAR_FOOT[0], REAR_FOOT[1], 0.0)),
        pivot=REAR_HINGE,
        name_prefix="rear",
        inset=0.004,
        metal=graphite,
    )

    _add_leg_part(
        model,
        name="left_leg",
        pivot=LEFT_HINGE,
        foot=LEFT_FOOT,
        metal=satin_aluminum,
        rubber=rubber,
    )
    _add_leg_part(
        model,
        name="right_leg",
        pivot=RIGHT_HINGE,
        foot=RIGHT_FOOT,
        metal=satin_aluminum,
        rubber=rubber,
    )
    _add_leg_part(
        model,
        name="rear_leg",
        pivot=REAR_HINGE,
        foot=REAR_FOOT,
        metal=satin_aluminum,
        rubber=rubber,
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=MAST_RADIUS, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=satin_aluminum,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.0175, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_graphite,
        name="mast_stop",
    )
    mast.visual(
        Box((0.018, 0.094, 0.020)),
        origin=Origin(xyz=(0.0, 0.050, 0.560)),
        material=graphite,
        name="top_arm",
    )
    mast.visual(
        Box((0.120, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.094, 0.552)),
        material=warm_wood,
        name="top_lip",
    )
    mast.visual(
        Box((0.046, 0.024, 0.072)),
        origin=Origin(xyz=(0.0, 0.020, 0.586)),
        material=graphite,
        name="top_block",
    )

    model.articulation(
        "crown_to_mast",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.18,
            lower=0.0,
            upper=0.22,
        ),
    )

    shelf = model.part("shelf")
    shelf.visual(
        Box((0.010, 0.034, 0.084)),
        origin=Origin(xyz=(-0.0155, 0.0, 0.0)),
        material=graphite,
        name="collar_0",
    )
    shelf.visual(
        Box((0.010, 0.034, 0.084)),
        origin=Origin(xyz=(0.0155, 0.0, 0.0)),
        material=graphite,
        name="collar_1",
    )
    shelf.visual(
        Box((0.041, 0.008, 0.084)),
        origin=Origin(xyz=(0.0, 0.0145, 0.0)),
        material=graphite,
        name="collar_2",
    )
    shelf.visual(
        Box((0.041, 0.008, 0.084)),
        origin=Origin(xyz=(0.0, -0.0145, 0.0)),
        material=graphite,
        name="collar_3",
    )
    shelf.visual(
        Box((0.022, 0.100, 0.020)),
        origin=Origin(xyz=(0.0, 0.066, -0.010)),
        material=graphite,
        name="arm",
    )
    shelf.visual(
        Box((0.290, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.142, -0.034)),
        material=warm_wood,
        name="tray",
    )
    shelf.visual(
        Box((0.290, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.172, -0.030)),
        material=warm_wood,
        name="front_lip",
    )
    shelf.visual(
        Box((0.024, 0.060, 0.022)),
        origin=Origin(xyz=(0.0, 0.106, -0.024), rpy=(0.30, 0.0, 0.0)),
        material=graphite,
        name="brace",
    )
    shelf.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(xyz=(0.031, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="shelf_stem",
    )
    shelf.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.043, 0.0, 0.006)),
        material=rubber,
        name="shelf_knob",
    )

    model.articulation(
        "mast_to_shelf",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=shelf,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.16,
            lower=0.0,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    mast = object_model.get_part("mast")
    shelf = object_model.get_part("shelf")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    rear_leg = object_model.get_part("rear_leg")

    mast_slide = object_model.get_articulation("crown_to_mast")
    shelf_slide = object_model.get_articulation("mast_to_shelf")
    left_hinge = object_model.get_articulation("crown_to_left_leg")
    right_hinge = object_model.get_articulation("crown_to_right_leg")
    rear_hinge = object_model.get_articulation("crown_to_rear_leg")

    ctx.expect_origin_distance(
        mast,
        crown,
        axes="xy",
        max_dist=0.001,
        name="mast stays centered in the sleeve axis",
    )
    ctx.expect_contact(
        mast,
        crown,
        elem_a="mast_stop",
        elem_b="sleeve",
        name="collapsed mast stop rests on the sleeve lip",
    )
    ctx.expect_overlap(
        mast,
        crown,
        axes="z",
        elem_a="mast_tube",
        elem_b="sleeve",
        min_overlap=0.09,
        name="collapsed mast remains inserted in the lower sleeve",
    )
    ctx.expect_origin_distance(
        shelf,
        mast,
        axes="xy",
        max_dist=0.001,
        name="shelf collar stays centered on the mast",
    )

    leg_aabbs = [ctx.part_world_aabb(part) for part in (left_leg, right_leg, rear_leg)]
    ctx.check(
        "deployed tripod feet reach the ground plane",
        all(aabb is not None and aabb[0][2] < 0.03 for aabb in leg_aabbs),
        details=f"aabbs={leg_aabbs}",
    )

    mast_rest = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: mast_slide.motion_limits.upper}):
        mast_extended = ctx.part_world_position(mast)
        ctx.expect_overlap(
            mast,
            crown,
            axes="z",
            elem_a="mast_tube",
            elem_b="sleeve",
            min_overlap=0.08,
            name="extended mast still retains insertion in the sleeve",
        )
    ctx.check(
        "mast extends upward",
        mast_rest is not None
        and mast_extended is not None
        and mast_extended[2] > mast_rest[2] + 0.18,
        details=f"rest={mast_rest}, extended={mast_extended}",
    )

    shelf_rest = ctx.part_world_position(shelf)
    with ctx.pose({shelf_slide: shelf_slide.motion_limits.upper}):
        shelf_raised = ctx.part_world_position(shelf)
        ctx.expect_origin_distance(
            shelf,
            mast,
            axes="xy",
            max_dist=0.001,
            name="raised shelf stays aligned to the mast",
        )
    ctx.check(
        "shelf slides upward on the mast",
        shelf_rest is not None
        and shelf_raised is not None
        and shelf_raised[2] > shelf_rest[2] + 0.18,
        details=f"rest={shelf_rest}, raised={shelf_raised}",
    )

    deployed_left = ctx.part_world_aabb(left_leg)
    with ctx.pose(
        {
            left_hinge: left_hinge.motion_limits.lower,
            right_hinge: right_hinge.motion_limits.lower,
            rear_hinge: rear_hinge.motion_limits.lower,
        }
    ):
        folded_left = ctx.part_world_aabb(left_leg)
    ctx.check(
        "legs fold upward toward the crown",
        deployed_left is not None
        and folded_left is not None
        and folded_left[0][2] > deployed_left[0][2] + 0.15,
        details=f"deployed={deployed_left}, folded={folded_left}",
    )

    return ctx.report()


object_model = build_object_model()
