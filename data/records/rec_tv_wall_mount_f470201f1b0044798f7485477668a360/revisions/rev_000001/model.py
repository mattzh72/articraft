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


FIRST_LINK_LENGTH = 0.46
SECOND_LINK_LENGTH = 0.40
TILT_AXIS_X = 0.145


def _plate_with_holes(
    *,
    thickness: float,
    width_y: float,
    height_z: float,
    holes: tuple[tuple[float, float], ...],
    hole_diameter: float,
):
    """A vertical plate extruded in +X with through holes in the YZ face."""

    plate = cq.Workplane("YZ").rect(width_y, height_z).extrude(thickness)
    if holes:
        plate = (
            plate.faces(">X")
            .workplane(centerOption="CenterOfMass")
            .pushPoints(list(holes))
            .hole(hole_diameter)
        )
    return plate


def _add_vertical_hub(
    part,
    *,
    name: str,
    x: float,
    z: float,
    radius: float,
    length: float,
    material: Material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, 0.0, z)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_tv_wall_arm")

    black = model.material("black_powder_coat", rgba=(0.015, 0.014, 0.013, 1.0))
    graphite = model.material("dark_graphite", rgba=(0.11, 0.115, 0.12, 1.0))
    steel = model.material("brushed_pin_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    shadow = model.material("hole_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(
            _plate_with_holes(
                thickness=0.018,
                width_y=0.220,
                height_z=0.460,
                holes=((-0.060, -0.155), (0.060, -0.155), (-0.060, 0.155), (0.060, 0.155)),
                hole_diameter=0.018,
            ),
            "wall_plate",
            tolerance=0.0008,
        ),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=black,
        name="wall_plate_shell",
    )
    # Two stacked hinge knuckles stand off the wall plate and leave a center gap
    # for the first link's knuckle.
    wall_plate.visual(
        Box((0.064, 0.054, 0.032)),
        origin=Origin(xyz=(-0.016, 0.0, 0.058)),
        material=black,
        name="wall_upper_ear",
    )
    wall_plate.visual(
        Cylinder(radius=0.047, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=black,
        name="wall_upper_boss",
    )
    wall_plate.visual(
        Box((0.064, 0.054, 0.032)),
        origin=Origin(xyz=(-0.016, 0.0, -0.058)),
        material=black,
        name="wall_lower_ear",
    )
    wall_plate.visual(
        Cylinder(radius=0.047, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=black,
        name="wall_lower_boss",
    )
    wall_plate.visual(
        Box((0.010, 0.060, 0.205)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0)),
        material=black,
        name="wall_center_rib",
    )

    first_link = model.part("first_link")
    first_link.visual(
        Box((FIRST_LINK_LENGTH - 0.075, 0.054, 0.040)),
        origin=Origin(xyz=(FIRST_LINK_LENGTH * 0.5, 0.0, 0.0)),
        material=graphite,
        name="first_arm_bar",
    )
    first_link.visual(
        Cylinder(radius=0.041, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="first_wall_knuckle",
    )
    first_link.visual(
        Cylinder(radius=0.041, length=0.084),
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        material=graphite,
        name="first_elbow_knuckle",
    )
    first_link.visual(
        Cylinder(radius=0.019, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="wall_pivot_pin_cap",
    )

    second_link = model.part("second_link")
    second_link.visual(
        Box((0.080, 0.044, 0.028)),
        origin=Origin(xyz=(0.034, 0.0, 0.056)),
        material=graphite,
        name="second_upper_ear",
    )
    second_link.visual(
        Cylinder(radius=0.040, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=graphite,
        name="second_upper_boss",
    )
    second_link.visual(
        Box((0.080, 0.044, 0.028)),
        origin=Origin(xyz=(0.034, 0.0, -0.056)),
        material=graphite,
        name="second_lower_ear",
    )
    second_link.visual(
        Cylinder(radius=0.040, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=graphite,
        name="second_lower_boss",
    )
    second_link.visual(
        Box((0.026, 0.048, 0.145)),
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=graphite,
        name="second_elbow_web",
    )
    second_link.visual(
        Box((SECOND_LINK_LENGTH - 0.100, 0.050, 0.038)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material=graphite,
        name="second_arm_bar",
    )
    second_link.visual(
        Cylinder(radius=0.038, length=0.082),
        origin=Origin(xyz=(SECOND_LINK_LENGTH, 0.0, 0.0)),
        material=graphite,
        name="second_swivel_knuckle",
    )
    for z, tag in ((0.056, "upper"), (-0.056, "lower")):
        second_link.visual(
            Cylinder(radius=0.017, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=steel,
            name=f"elbow_pin_cap_{tag}",
        )

    swivel_yoke = model.part("swivel_yoke")
    swivel_yoke.visual(
        Box((0.066, 0.042, 0.028)),
        origin=Origin(xyz=(0.030, 0.0, 0.055)),
        material=black,
        name="swivel_upper_ear",
    )
    swivel_yoke.visual(
        Cylinder(radius=0.038, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=black,
        name="swivel_upper_boss",
    )
    swivel_yoke.visual(
        Box((0.066, 0.042, 0.028)),
        origin=Origin(xyz=(0.030, 0.0, -0.055)),
        material=black,
        name="swivel_lower_ear",
    )
    swivel_yoke.visual(
        Cylinder(radius=0.038, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=black,
        name="swivel_lower_boss",
    )
    swivel_yoke.visual(
        Box((0.026, 0.046, 0.138)),
        origin=Origin(xyz=(0.062, 0.0, 0.0)),
        material=black,
        name="swivel_web",
    )
    swivel_yoke.visual(
        Box((0.065, 0.046, 0.040)),
        origin=Origin(xyz=(0.088, 0.0, 0.0)),
        material=black,
        name="tilt_stem",
    )
    swivel_yoke.visual(
        Box((0.022, 0.140, 0.030)),
        origin=Origin(xyz=(0.107, 0.0, 0.0)),
        material=black,
        name="tilt_yoke_bridge",
    )
    swivel_yoke.visual(
        Box((0.090, 0.018, 0.092)),
        origin=Origin(xyz=(TILT_AXIS_X, 0.061, 0.0)),
        material=black,
        name="tilt_yoke_side_0",
    )
    swivel_yoke.visual(
        Cylinder(radius=0.027, length=0.014),
        origin=Origin(xyz=(TILT_AXIS_X, 0.052, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_bushing_side_0",
    )
    swivel_yoke.visual(
        Box((0.090, 0.018, 0.092)),
        origin=Origin(xyz=(TILT_AXIS_X, -0.061, 0.0)),
        material=black,
        name="tilt_yoke_side_1",
    )
    swivel_yoke.visual(
        Cylinder(radius=0.027, length=0.014),
        origin=Origin(xyz=(TILT_AXIS_X, -0.052, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_bushing_side_1",
    )

    vesa_head = model.part("vesa_head")
    vesa_head.visual(
        Cylinder(radius=0.022, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_trunnion",
    )
    vesa_head.visual(
        Box((0.150, 0.052, 0.034)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=black,
        name="head_neck",
    )
    vesa_head.visual(
        mesh_from_cadquery(
            _plate_with_holes(
                thickness=0.018,
                width_y=0.240,
                height_z=0.180,
                holes=((-0.050, -0.050), (0.050, -0.050), (-0.050, 0.050), (0.050, 0.050)),
                hole_diameter=0.010,
            ),
            "vesa_head_plate",
            tolerance=0.0006,
        ),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=black,
        name="vesa_plate",
    )
    vesa_head.visual(
        Box((0.010, 0.150, 0.012)),
        origin=Origin(xyz=(0.165, 0.0, 0.075)),
        material=shadow,
        name="top_slot_shadow",
    )

    model.articulation(
        "wall_pivot",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.3, lower=-2.65, upper=2.65),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=swivel_yoke,
        origin=Origin(xyz=(SECOND_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.4, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_yoke,
        child=vesa_head,
        origin=Origin(xyz=(TILT_AXIS_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=-0.30, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_plate = object_model.get_part("wall_plate")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    swivel_yoke = object_model.get_part("swivel_yoke")
    vesa_head = object_model.get_part("vesa_head")

    wall_pivot = object_model.get_articulation("wall_pivot")
    elbow_pivot = object_model.get_articulation("elbow_pivot")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "four revolute user pivots",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details="Expected wall, elbow, head swivel, and head tilt revolutes.",
    )
    ctx.check(
        "single wall plate root",
        [part.name for part in object_model.root_parts()] == ["wall_plate"],
        details=f"roots={[part.name for part in object_model.root_parts()]}",
    )

    ctx.expect_contact(
        first_link,
        wall_plate,
        elem_a="first_wall_knuckle",
        elem_b="wall_upper_boss",
        contact_tol=0.001,
        name="first link is captured by wall hinge",
    )
    ctx.expect_contact(
        second_link,
        first_link,
        elem_a="second_upper_boss",
        elem_b="first_elbow_knuckle",
        contact_tol=0.001,
        name="second link is captured by elbow hinge",
    )
    ctx.expect_contact(
        swivel_yoke,
        second_link,
        elem_a="swivel_upper_boss",
        elem_b="second_swivel_knuckle",
        contact_tol=0.001,
        name="head swivel yoke is captured by arm end",
    )
    ctx.expect_contact(
        vesa_head,
        swivel_yoke,
        elem_a="tilt_trunnion",
        elem_b="tilt_bushing_side_0",
        contact_tol=0.001,
        name="tilt trunnion seats in side bushing",
    )

    plate_aabb = ctx.part_element_world_aabb(vesa_head, elem="vesa_plate")
    ctx.check(
        "VESA head is a broad rectangular plate",
        plate_aabb is not None
        and (plate_aabb[1][1] - plate_aabb[0][1]) > 0.220
        and (plate_aabb[1][2] - plate_aabb[0][2]) > 0.165,
        details=f"vesa_plate_aabb={plate_aabb}",
    )

    rest_elbow = ctx.part_world_position(second_link)
    with ctx.pose({wall_pivot: 0.90}):
        swung_elbow = ctx.part_world_position(second_link)
    ctx.check(
        "wall pivot swings the first link sideways",
        rest_elbow is not None
        and swung_elbow is not None
        and swung_elbow[1] > rest_elbow[1] + 0.30
        and swung_elbow[0] < rest_elbow[0] - 0.10,
        details=f"rest={rest_elbow}, swung={swung_elbow}",
    )

    rest_yoke = ctx.part_world_position(swivel_yoke)
    with ctx.pose({elbow_pivot: -1.30}):
        folded_yoke = ctx.part_world_position(swivel_yoke)
    ctx.check(
        "elbow pivot folds the second link across the reach",
        rest_yoke is not None
        and folded_yoke is not None
        and folded_yoke[1] < rest_yoke[1] - 0.30
        and folded_yoke[0] < rest_yoke[0] - 0.20,
        details=f"rest={rest_yoke}, folded={folded_yoke}",
    )

    rest_head = ctx.part_world_position(vesa_head)
    with ctx.pose({head_swivel: 0.80}):
        swiveled_head = ctx.part_world_position(vesa_head)
    ctx.check(
        "head swivel yaws the short yoke",
        rest_head is not None
        and swiveled_head is not None
        and swiveled_head[1] > rest_head[1] + 0.08,
        details=f"rest={rest_head}, swiveled={swiveled_head}",
    )

    rest_plate = ctx.part_element_world_aabb(vesa_head, elem="vesa_plate")
    with ctx.pose({head_tilt: 0.30}):
        tilted_plate = ctx.part_element_world_aabb(vesa_head, elem="vesa_plate")
    ctx.check(
        "head tilt pitches the VESA plate",
        rest_plate is not None
        and tilted_plate is not None
        and tilted_plate[0][2] < rest_plate[0][2] - 0.030,
        details=f"rest={rest_plate}, tilted={tilted_plate}",
    )

    return ctx.report()


object_model = build_object_model()
