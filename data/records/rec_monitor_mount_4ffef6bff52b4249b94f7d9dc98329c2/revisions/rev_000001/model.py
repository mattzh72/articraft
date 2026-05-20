from __future__ import annotations

import math

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
)


X_CYL = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
Y_CYL = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _origin(x: float, y: float, z: float, *, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_monitor_arm")

    matte_black = model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.24, 1.0))
    screw_dark = model.material("recessed_screws", rgba=(0.005, 0.005, 0.006, 1.0))

    # Root frame: the first vertical fold axis, with +X pointing away from the wall.
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.030, 0.205, 0.340)),
        origin=_origin(-0.060, 0.0, 0.0),
        material=graphite,
        name="wall_backing_plate",
    )
    wall_plate.visual(
        Box((0.020, 0.150, 0.260)),
        origin=_origin(-0.050, 0.0, 0.0),
        material=dark_metal,
        name="raised_wall_plinth",
    )
    for i, (y, z) in enumerate(
        ((-0.068, -0.110), (0.068, -0.110), (-0.068, 0.110), (0.068, 0.110))
    ):
        wall_plate.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=_origin(-0.037, y, z, rpy=X_CYL.rpy),
            material=screw_dark,
            name=f"plate_screw_{i}",
        )
    # Upper and lower fixed knuckles leave a center gap for the primary-link barrel.
    for i, z in enumerate((-0.066, 0.066)):
        wall_plate.visual(
            Cylinder(radius=0.042, length=0.056),
            origin=_origin(0.0, 0.0, z),
            material=matte_black,
            name=f"wall_knuckle_{i}",
        )

    primary_link = model.part("primary_link")
    primary_link.visual(
        Cylinder(radius=0.034, length=0.076),
        origin=_origin(0.0, 0.0, 0.0),
        material=matte_black,
        name="wall_barrel",
    )
    for y in (-0.040, 0.040):
        primary_link.visual(
            Box((0.455, 0.014, 0.030)),
            origin=_origin(0.2275, y, 0.0),
            material=dark_metal,
            name=f"primary_side_bar_{'neg' if y < 0 else 'pos'}",
        )
    for i, z in enumerate((-0.058, 0.058)):
        primary_link.visual(
            Cylinder(radius=0.034, length=0.048),
            origin=_origin(0.460, 0.0, z),
            material=matte_black,
            name=f"elbow_yoke_{i}",
        )
    for y in (-0.040, 0.040):
        primary_link.visual(
            Box((0.046, 0.014, 0.122)),
            origin=_origin(0.460, y, 0.0),
            material=matte_black,
            name=f"elbow_cheek_{'neg' if y < 0 else 'pos'}",
        )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        Cylinder(radius=0.030, length=0.068),
        origin=_origin(0.0, 0.0, 0.0),
        material=matte_black,
        name="elbow_barrel",
    )
    for y in (-0.024, 0.024):
        secondary_link.visual(
            Box((0.180, 0.012, 0.028)),
            origin=_origin(0.090, y, 0.0),
            material=dark_metal,
            name=f"secondary_side_bar_{'neg' if y < 0 else 'pos'}",
        )
    for y in (-0.036, 0.036):
        side = "neg" if y < 0 else "pos"
        secondary_link.visual(
            Box((0.220, 0.012, 0.028)),
            origin=_origin(0.250, y, 0.0),
            material=dark_metal,
            name=f"secondary_outer_bar_{side}",
        )
        secondary_link.visual(
            Box((0.050, 0.036, 0.024)),
            origin=_origin(0.165, -0.030 if y < 0 else 0.030, 0.0),
            material=dark_metal,
            name=f"secondary_bar_splice_{side}",
        )
    for i, z in enumerate((-0.052, 0.052)):
        secondary_link.visual(
            Cylinder(radius=0.030, length=0.044),
            origin=_origin(0.360, 0.0, z),
            material=matte_black,
            name=f"wrist_yoke_{i}",
        )
    for y in (-0.036, 0.036):
        secondary_link.visual(
            Box((0.040, 0.012, 0.108)),
            origin=_origin(0.360, y, 0.0),
            material=matte_black,
            name=f"wrist_cheek_{'neg' if y < 0 else 'pos'}",
        )

    wrist_swivel = model.part("wrist_swivel")
    wrist_swivel.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=_origin(0.0, 0.0, 0.0),
        material=matte_black,
        name="swivel_barrel",
    )
    wrist_swivel.visual(
        Box((0.086, 0.052, 0.034)),
        origin=_origin(0.037, 0.0, 0.0),
        material=dark_metal,
        name="short_neck",
    )
    wrist_swivel.visual(
        Box((0.030, 0.116, 0.018)),
        origin=_origin(0.060, 0.0, 0.0),
        material=dark_metal,
        name="fork_bridge",
    )
    for y in (-0.052, 0.052):
        wrist_swivel.visual(
            Box((0.044, 0.012, 0.070)),
            origin=_origin(0.096, y, 0.0),
            material=matte_black,
            name=f"tilt_fork_{'neg' if y < 0 else 'pos'}",
        )
        wrist_swivel.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=_origin(0.104, y, 0.0, rpy=Y_CYL.rpy),
            material=matte_black,
            name=f"tilt_pivot_cap_{'neg' if y < 0 else 'pos'}",
        )

    head_frame = model.part("head_frame")
    head_frame.visual(
        Cylinder(radius=0.013, length=0.092),
        origin=_origin(0.0, 0.0, 0.0, rpy=Y_CYL.rpy),
        material=matte_black,
        name="tilt_barrel",
    )
    for y in (-0.038, 0.038):
        head_frame.visual(
            Box((0.070, 0.018, 0.022)),
            origin=_origin(0.030, y, 0.0),
            material=dark_metal,
            name=f"tilt_strap_{'neg' if y < 0 else 'pos'}",
        )
    # A screenless rectangular VESA-style head frame. The four bars overlap at
    # their corners, leaving the middle visibly open.
    head_frame.visual(
        Box((0.018, 0.300, 0.020)),
        origin=_origin(0.070, 0.0, 0.100),
        material=graphite,
        name="frame_top_bar",
    )
    head_frame.visual(
        Box((0.018, 0.300, 0.020)),
        origin=_origin(0.070, 0.0, -0.100),
        material=graphite,
        name="frame_bottom_bar",
    )
    for y in (-0.140, 0.140):
        head_frame.visual(
            Box((0.018, 0.020, 0.220)),
            origin=_origin(0.070, y, 0.0),
            material=graphite,
            name=f"frame_side_bar_{'neg' if y < 0 else 'pos'}",
        )
    head_frame.visual(
        Box((0.018, 0.300, 0.018)),
        origin=_origin(0.070, 0.0, 0.0),
        material=graphite,
        name="frame_center_bar",
    )
    for y in (-0.070, 0.070):
        head_frame.visual(
            Box((0.018, 0.018, 0.130)),
            origin=_origin(0.070, y, 0.0),
            material=dark_metal,
            name=f"vesa_rail_{'neg' if y < 0 else 'pos'}",
        )
    for i, (y, z) in enumerate(((-0.070, -0.055), (0.070, -0.055), (-0.070, 0.055), (0.070, 0.055))):
        head_frame.visual(
            Box((0.018, 0.030, 0.030)),
            origin=_origin(0.070, y, z),
            material=dark_metal,
            name=f"vesa_tab_{i}",
        )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=primary_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=_origin(0.460, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.5, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "secondary_to_wrist",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=wrist_swivel,
        origin=_origin(0.360, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "wrist_to_head",
        ArticulationType.REVOLUTE,
        parent=wrist_swivel,
        child=head_frame,
        origin=_origin(0.104, 0.0, 0.0),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.70, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_plate")
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    wrist = object_model.get_part("wrist_swivel")
    head = object_model.get_part("head_frame")

    wall_joint = object_model.get_articulation("wall_to_primary")
    elbow_joint = object_model.get_articulation("primary_to_secondary")
    swivel_joint = object_model.get_articulation("secondary_to_wrist")
    tilt_joint = object_model.get_articulation("wrist_to_head")

    ctx.check(
        "arm has four revolute mechanisms",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (wall_joint, elbow_joint, swivel_joint, tilt_joint)
        ),
    )
    ctx.expect_overlap(
        primary,
        wall,
        axes="xy",
        min_overlap=0.040,
        name="primary link is captured at wall hinge footprint",
    )
    ctx.expect_overlap(
        secondary,
        primary,
        axes="xy",
        min_overlap=0.040,
        name="secondary link is captured at elbow hinge footprint",
    )
    ctx.expect_overlap(
        wrist,
        secondary,
        axes="xy",
        min_overlap=0.030,
        name="head swivel is captured at wrist hinge footprint",
    )
    ctx.expect_overlap(
        head,
        wrist,
        axes="yz",
        min_overlap=0.020,
        name="tilt barrel is held in the short fork bracket",
    )

    head_aabb = ctx.part_world_aabb(head)
    if head_aabb is not None:
        head_min, head_max = head_aabb
        head_size = tuple(head_max[i] - head_min[i] for i in range(3))
        ctx.check(
            "head frame is a broad screenless rectangle",
            head_size[1] > 0.26 and head_size[2] > 0.18 and head_size[0] < 0.13,
            details=f"head_size={head_size}",
        )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({wall_joint: 1.0, elbow_joint: -1.8}):
        folded_head = ctx.part_world_position(head)
    ctx.check(
        "wall and elbow joints fold the arm inward",
        rest_head is not None
        and folded_head is not None
        and folded_head[0] < rest_head[0] - 0.25
        and abs(folded_head[1]) > 0.04,
        details=f"rest={rest_head}, folded={folded_head}",
    )

    rest_aabb = ctx.part_world_aabb(head)
    with ctx.pose({tilt_joint: 0.50}):
        tilted_aabb = ctx.part_world_aabb(head)
    if rest_aabb is not None and tilted_aabb is not None:
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
        tilted_center_z = (tilted_aabb[0][2] + tilted_aabb[1][2]) / 2.0
        ctx.check(
            "tilt revolute pitches the head frame",
            abs(tilted_center_z - rest_center_z) > 0.020,
            details=f"rest_z={rest_center_z}, tilted_z={tilted_center_z}",
        )

    return ctx.report()


object_model = build_object_model()
