from __future__ import annotations

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)


LEG_ANGLES = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
HINGE_RADIUS = 0.12
HINGE_HEIGHT = 1.00


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _tube_leg_mesh() -> object:
    return tube_from_spline_points(
        [
            (0.018, 0.0, -0.010),
            (0.082, 0.0, -0.165),
            (0.228, 0.0, -0.545),
            (0.440, 0.0, -0.980),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_binocular_viewer_tripod")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.54, 0.56, 0.58, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.20, 1.0))
    viewer_shell = model.material("viewer_shell", rgba=(0.70, 0.72, 0.74, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    lens_black = model.material("lens_black", rgba=(0.10, 0.10, 0.11, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.095, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        material=cast_aluminum,
        name="hub",
    )
    crown.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=dark_metal,
        name="underside_collar",
    )
    crown.visual(
        Cylinder(radius=0.032, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 1.100)),
        material=dark_metal,
        name="mast",
    )
    crown.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.185)),
        material=cast_aluminum,
        name="pan_socket",
    )

    for index, angle in enumerate(LEG_ANGLES):
        arm_x, arm_y = _polar_xy(0.045, angle)
        hinge_x, hinge_y = _polar_xy(0.100, angle)
        crown.visual(
            Box((0.090, 0.028, 0.018)),
            origin=Origin(xyz=(arm_x, arm_y, 0.992), rpy=(0.0, 0.0, angle)),
            material=cast_aluminum,
            name=f"spider_arm_{index}",
        )
        crown.visual(
            Box((0.040, 0.034, 0.028)),
            origin=Origin(xyz=(hinge_x, hinge_y, HINGE_HEIGHT), rpy=(0.0, 0.0, angle)),
            material=cast_aluminum,
            name=f"hinge_block_{index}",
        )

    for index in range(3):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.010, length=0.024),
            origin=Origin(xyz=(0.010, 0.0, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=cast_aluminum,
            name="crown_knuckle",
        )
        leg.visual(
            mesh_from_geometry(_tube_leg_mesh(), f"leg_tube_{index}"),
            material=dark_metal,
            name="tube",
        )
        leg.visual(
            Sphere(radius=0.020),
            origin=Origin(xyz=(0.440, 0.0, -0.980)),
            material=rubber,
            name="foot",
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_metal,
        name="bearing",
    )
    pan_head.visual(
        Cylinder(radius=0.038, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=dark_metal,
        name="body",
    )
    pan_head.visual(
        Cylinder(radius=0.055, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=cast_aluminum,
        name="top_plate",
    )
    pan_head.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.030, 0.038, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="handle_boss",
    )
    pan_head.visual(
        Cylinder(radius=0.007, length=0.080),
        origin=Origin(xyz=(0.075, 0.038, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )

    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.visual(
        Box((0.060, 0.080, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=cast_aluminum,
        name="base",
    )
    tilt_bracket.visual(
        Box((0.110, 0.190, 0.015)),
        origin=Origin(xyz=(0.055, 0.0, 0.020)),
        material=cast_aluminum,
        name="arm",
    )
    tilt_bracket.visual(
        Box((0.024, 0.012, 0.110)),
        origin=Origin(xyz=(0.102, 0.095, 0.065)),
        material=cast_aluminum,
        name="cheek_0",
    )
    tilt_bracket.visual(
        Box((0.024, 0.012, 0.110)),
        origin=Origin(xyz=(0.102, -0.095, 0.065)),
        material=cast_aluminum,
        name="cheek_1",
    )

    viewer = model.part("viewer")
    viewer.visual(
        Box((0.050, 0.156, 0.050)),
        origin=Origin(xyz=(0.025, 0.0, -0.012)),
        material=cast_aluminum,
        name="mount",
    )
    viewer.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, 0.078, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_0",
    )
    viewer.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, -0.078, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_1",
    )
    viewer.visual(
        Box((0.130, 0.150, 0.090)),
        origin=Origin(xyz=(0.090, 0.0, 0.020)),
        material=viewer_shell,
        name="body",
    )
    viewer.visual(
        Box((0.080, 0.090, 0.030)),
        origin=Origin(xyz=(0.090, 0.0, 0.080)),
        material=viewer_shell,
        name="prism_cap",
    )
    viewer.visual(
        Box((0.050, 0.100, 0.050)),
        origin=Origin(xyz=(-0.010, 0.0, 0.020)),
        material=viewer_shell,
        name="ocular_bridge",
    )
    viewer.visual(
        Cylinder(radius=0.037, length=0.280),
        origin=Origin(xyz=(0.190, 0.065, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=viewer_shell,
        name="barrel_0",
    )
    viewer.visual(
        Cylinder(radius=0.037, length=0.280),
        origin=Origin(xyz=(0.190, -0.065, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=viewer_shell,
        name="barrel_1",
    )
    viewer.visual(
        Cylinder(radius=0.047, length=0.070),
        origin=Origin(xyz=(0.350, 0.065, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="objective_0",
    )
    viewer.visual(
        Cylinder(radius=0.047, length=0.070),
        origin=Origin(xyz=(0.350, -0.065, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="objective_1",
    )
    viewer.visual(
        Cylinder(radius=0.022, length=0.090),
        origin=Origin(xyz=(-0.030, 0.038, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="eyepiece_0",
    )
    viewer.visual(
        Cylinder(radius=0.022, length=0.090),
        origin=Origin(xyz=(-0.030, -0.038, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="eyepiece_1",
    )
    viewer.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(-0.090, 0.038, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="eyecup_0",
    )
    viewer.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(-0.090, -0.038, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="eyecup_1",
    )
    viewer.visual(
        Cylinder(radius=0.016, length=0.028),
        origin=Origin(xyz=(0.055, 0.0, 0.082)),
        material=dark_metal,
        name="focus_knob",
    )

    for index, angle in enumerate(LEG_ANGLES):
        hinge_x, hinge_y = _polar_xy(HINGE_RADIUS, angle)
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=f"leg_{index}",
            origin=Origin(xyz=(hinge_x, hinge_y, HINGE_HEIGHT), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=1.2,
                lower=-0.20,
                upper=1.15,
            ),
        )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.5),
    )
    model.articulation(
        "pan_head_to_tilt_bracket",
        ArticulationType.FIXED,
        parent=pan_head,
        child=tilt_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
    )
    model.articulation(
        "tilt_bracket_to_viewer",
        ArticulationType.REVOLUTE,
        parent=tilt_bracket,
        child=viewer,
        origin=Origin(xyz=(0.104, 0.0, 0.085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=-0.45,
            upper=0.70,
        ),
    )

    return model


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    viewer = object_model.get_part("viewer")
    leg_0 = object_model.get_part("leg_0")

    pan = object_model.get_articulation("crown_to_pan_head")
    tilt = object_model.get_articulation("tilt_bracket_to_viewer")
    leg_hinge_0 = object_model.get_articulation("crown_to_leg_0")

    ctx.expect_gap(
        viewer,
        crown,
        axis="z",
        min_gap=0.15,
        name="viewer sits clearly above the tripod crown",
    )
    ctx.expect_gap(
        viewer,
        pan_head,
        axis="z",
        min_gap=0.025,
        name="viewer clears the pan head housing",
    )

    rest_viewer_pos = ctx.part_world_position(viewer)
    with ctx.pose({pan: math.pi / 2.0}):
        turned_viewer_pos = ctx.part_world_position(viewer)

    ctx.check(
        "pan head swings the viewer around the vertical axis",
        rest_viewer_pos is not None
        and turned_viewer_pos is not None
        and rest_viewer_pos[0] > 0.08
        and abs(turned_viewer_pos[0]) < 0.03
        and turned_viewer_pos[1] > 0.08,
        details=f"rest={rest_viewer_pos}, turned={turned_viewer_pos}",
    )

    rest_objective = ctx.part_element_world_aabb(viewer, elem="objective_0")
    with ctx.pose({tilt: 0.55}):
        raised_objective = ctx.part_element_world_aabb(viewer, elem="objective_0")

    rest_objective_z = _aabb_center_z(rest_objective)
    raised_objective_z = _aabb_center_z(raised_objective)
    ctx.check(
        "viewer tilt raises the objective end",
        rest_objective_z is not None
        and raised_objective_z is not None
        and raised_objective_z > rest_objective_z + 0.09,
        details=f"rest_z={rest_objective_z}, raised_z={raised_objective_z}",
    )

    rest_foot = ctx.part_element_world_aabb(leg_0, elem="foot")
    with ctx.pose({leg_hinge_0: 1.15}):
        folded_foot = ctx.part_element_world_aabb(leg_0, elem="foot")

    rest_foot_z = _aabb_center_z(rest_foot)
    folded_foot_z = _aabb_center_z(folded_foot)
    ctx.check(
        "tripod leg folds upward around the crown hinge",
        rest_foot_z is not None
        and folded_foot_z is not None
        and folded_foot_z > rest_foot_z + 0.15,
        details=f"rest_z={rest_foot_z}, folded_z={folded_foot_z}",
    )

    return ctx.report()


object_model = build_object_model()
