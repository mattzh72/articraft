from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_cctv_mount")

    concrete = model.material("painted_concrete", rgba=(0.72, 0.72, 0.68, 1.0))
    bracket_white = model.material("powder_coated_white", rgba=(0.92, 0.91, 0.86, 1.0))
    dark_gasket = model.material("matte_black", rgba=(0.015, 0.015, 0.014, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.02, 0.05, 0.08, 0.72))
    screw_steel = model.material("brushed_screw_steel", rgba=(0.45, 0.46, 0.45, 1.0))

    wall = model.part("corner_wall")
    wall.visual(
        Box((0.70, 0.050, 1.20)),
        origin=Origin(xyz=(-0.290, -0.035, 0.600)),
        material=concrete,
        name="wall_leaf_x",
    )
    wall.visual(
        Box((0.050, 0.70, 1.20)),
        origin=Origin(xyz=(-0.035, -0.290, 0.600)),
        material=concrete,
        name="wall_leaf_y",
    )
    wall.visual(
        Box((0.120, 0.120, 1.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=concrete,
        name="corner_pillar",
    )

    bracket = model.part("l_bracket")
    bracket.visual(
        Box((0.012, 0.150, 0.235)),
        origin=Origin(xyz=(0.066, 0.012, 0.685)),
        material=bracket_white,
        name="plate_x",
    )
    bracket.visual(
        Box((0.150, 0.012, 0.235)),
        origin=Origin(xyz=(0.012, 0.066, 0.685)),
        material=bracket_white,
        name="plate_y",
    )
    # A square tube projects from the wall corner to carry the ball-head bearing.
    bracket.visual(
        Box((0.300, 0.050, 0.050)),
        origin=Origin(xyz=(0.235, 0.235, 0.720), rpy=(0.0, 0.0, pi / 4.0)),
        material=bracket_white,
        name="diagonal_arm",
    )
    bracket.visual(
        Box((0.078, 0.070, 0.050)),
        origin=Origin(xyz=(0.111, 0.090, 0.720)),
        material=bracket_white,
        name="corner_block",
    )
    bracket.visual(
        Box((0.240, 0.026, 0.026)),
        origin=Origin(xyz=(0.158, 0.158, 0.665), rpy=(0.52, 0.0, pi / 4.0)),
        material=bracket_white,
        name="lower_gusset",
    )
    bracket.visual(
        Cylinder(radius=0.057, length=0.028),
        origin=Origin(xyz=(0.320, 0.320, 0.756)),
        material=bracket_white,
        name="bearing_socket",
    )
    for idx, (x, y, face) in enumerate(
        (
            (0.073, -0.040, "x"),
            (0.073, 0.055, "x"),
            (-0.040, 0.073, "y"),
            (0.055, 0.073, "y"),
        )
    ):
        if face == "x":
            origin = Origin(xyz=(x, y, 0.735 if idx % 2 else 0.635), rpy=(0.0, pi / 2.0, 0.0))
        else:
            origin = Origin(xyz=(x, y, 0.735 if idx % 2 else 0.635), rpy=(pi / 2.0, 0.0, 0.0))
        bracket.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=origin,
            material=screw_steel,
            name=f"bolt_{idx}",
        )

    pan_head = model.part("ball_mount")
    pan_head.visual(
        Cylinder(radius=0.046, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=bracket_white,
        name="turntable",
    )
    pan_head.visual(
        Cylinder(radius=0.022, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=bracket_white,
        name="neck",
    )
    pan_head.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=bracket_white,
        name="ball",
    )
    pan_head.visual(
        Cylinder(radius=0.016, length=0.105),
        origin=Origin(xyz=(0.075, 0.064, 0.094), rpy=(0.0, pi / 2.0, 0.0)),
        material=bracket_white,
        name="side_link_0",
    )
    pan_head.visual(
        Cylinder(radius=0.016, length=0.105),
        origin=Origin(xyz=(0.075, -0.064, 0.094), rpy=(0.0, pi / 2.0, 0.0)),
        material=bracket_white,
        name="side_link_1",
    )
    pan_head.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(0.149, 0.064, 0.119)),
        material=bracket_white,
        name="upper_bridge_0",
    )
    pan_head.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(0.149, -0.064, 0.119)),
        material=bracket_white,
        name="upper_bridge_1",
    )
    pan_head.visual(
        Box((0.038, 0.012, 0.092)),
        origin=Origin(xyz=(0.160, 0.064, 0.094)),
        material=bracket_white,
        name="tilt_ear_0",
    )
    pan_head.visual(
        Box((0.038, 0.012, 0.092)),
        origin=Origin(xyz=(0.160, -0.064, 0.094)),
        material=bracket_white,
        name="tilt_ear_1",
    )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.044, length=0.220),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bracket_white,
        name="body",
    )
    camera.visual(
        Cylinder(radius=0.048, length=0.020),
        origin=Origin(xyz=(0.228, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gasket,
        name="front_bezel",
    )
    camera.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.242, 0.0, 0.0)),
        material=smoked_glass,
        name="front_lens",
    )
    camera.visual(
        Box((0.252, 0.112, 0.014)),
        origin=Origin(xyz=(0.126, 0.0, 0.051)),
        material=bracket_white,
        name="sun_shade",
    )
    camera.visual(
        Cylinder(radius=0.014, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=screw_steel,
        name="tilt_axle",
    )
    camera.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bracket_white,
        name="rear_cap",
    )

    model.articulation(
        "wall_to_bracket",
        ArticulationType.FIXED,
        parent=wall,
        child=bracket,
        origin=Origin(),
    )
    model.articulation(
        "bracket_to_pan",
        ArticulationType.CONTINUOUS,
        parent=bracket,
        child=pan_head,
        origin=Origin(xyz=(0.320, 0.320, 0.770), rpy=(0.0, 0.0, pi / 4.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5),
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.160, 0.0, 0.094)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("corner_wall")
    bracket = object_model.get_part("l_bracket")
    pan_head = object_model.get_part("ball_mount")
    camera = object_model.get_part("camera")
    pan = object_model.get_articulation("bracket_to_pan")
    tilt = object_model.get_articulation("pan_to_camera")

    ctx.allow_overlap(
        pan_head,
        camera,
        elem_a="tilt_ear_0",
        elem_b="tilt_axle",
        reason="The visible trunnion axle is intentionally captured inside the yoke ear proxy.",
    )
    ctx.allow_overlap(
        pan_head,
        camera,
        elem_a="tilt_ear_1",
        elem_b="tilt_axle",
        reason="The visible trunnion axle is intentionally captured inside the opposite yoke ear proxy.",
    )

    ctx.expect_contact(
        bracket,
        wall,
        elem_a="plate_x",
        elem_b="corner_pillar",
        contact_tol=0.001,
        name="first L-bracket leaf is fixed against the corner pillar",
    )
    ctx.expect_contact(
        bracket,
        wall,
        elem_a="plate_y",
        elem_b="corner_pillar",
        contact_tol=0.001,
        name="second L-bracket leaf is fixed against the corner pillar",
    )
    ctx.expect_contact(
        pan_head,
        bracket,
        elem_a="turntable",
        elem_b="bearing_socket",
        contact_tol=0.001,
        name="pan turntable sits on the bracket bearing socket",
    )
    ctx.expect_overlap(
        camera,
        pan_head,
        axes="xyz",
        elem_a="tilt_axle",
        elem_b="tilt_ear_0",
        min_overlap=0.010,
        name="tilt axle is retained by one yoke ear",
    )
    ctx.expect_overlap(
        camera,
        pan_head,
        axes="xyz",
        elem_a="tilt_axle",
        elem_b="tilt_ear_1",
        min_overlap=0.010,
        name="tilt axle is retained by the opposite yoke ear",
    )

    rest_pos = ctx.part_world_position(camera)
    with ctx.pose({pan: 1.0}):
        panned_pos = ctx.part_world_position(camera)
    ctx.check(
        "continuous pan bearing moves the camera around the vertical ball mount",
        rest_pos is not None
        and panned_pos is not None
        and ((panned_pos[0] - rest_pos[0]) ** 2 + (panned_pos[1] - rest_pos[1]) ** 2) ** 0.5
        > 0.08,
        details=f"rest={rest_pos}, panned={panned_pos}",
    )

    rest_lens = ctx.part_element_world_aabb(camera, elem="front_lens")
    with ctx.pose({tilt: 0.55}):
        raised_lens = ctx.part_element_world_aabb(camera, elem="front_lens")
    with ctx.pose({tilt: -0.55}):
        lowered_lens = ctx.part_element_world_aabb(camera, elem="front_lens")

    def z_center(aabb):
        return None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])

    rest_z = z_center(rest_lens)
    raised_z = z_center(raised_lens)
    lowered_z = z_center(lowered_lens)
    ctx.check(
        "revolute tilt joint raises and lowers the camera nose",
        rest_z is not None
        and raised_z is not None
        and lowered_z is not None
        and raised_z > rest_z + 0.05
        and lowered_z < rest_z - 0.05,
        details=f"rest_z={rest_z}, raised_z={raised_z}, lowered_z={lowered_z}",
    )

    return ctx.report()


object_model = build_object_model()
