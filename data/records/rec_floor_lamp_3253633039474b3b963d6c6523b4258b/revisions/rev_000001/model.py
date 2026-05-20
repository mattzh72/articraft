from __future__ import annotations

from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_THICKNESS = 0.045
LOWER_LINK_HEIGHT = 1.060
MIDDLE_LINK_LENGTH = 0.380
UPPER_LINK_LENGTH = 0.330


def _shade_shell_geometry(
    *,
    start_x: float = 0.060,
    length: float = 0.180,
    small_radius: float = 0.058,
    large_radius: float = 0.135,
    wall: float = 0.006,
    segments: int = 48,
) -> MeshGeometry:
    """Thin open conical shade shell, authored along local +X."""

    geom = MeshGeometry()
    outer_small = []
    outer_large = []
    inner_small = []
    inner_large = []

    for i in range(segments):
        angle = tau * i / segments
        c = cos(angle)
        s = sin(angle)
        outer_small.append(geom.add_vertex(start_x, small_radius * c, small_radius * s))
        outer_large.append(
            geom.add_vertex(start_x + length, large_radius * c, large_radius * s)
        )
        inner_small.append(
            geom.add_vertex(start_x, (small_radius - wall) * c, (small_radius - wall) * s)
        )
        inner_large.append(
            geom.add_vertex(
                start_x + length,
                (large_radius - wall) * c,
                (large_radius - wall) * s,
            )
        )

    for i in range(segments):
        j = (i + 1) % segments

        # Outer conical wall.
        geom.add_face(outer_small[i], outer_large[i], outer_large[j])
        geom.add_face(outer_small[i], outer_large[j], outer_small[j])

        # Inner conical wall, reversed normals.
        geom.add_face(inner_small[i], inner_large[j], inner_large[i])
        geom.add_face(inner_small[i], inner_small[j], inner_large[j])

        # Rolled rims connect inner and outer walls but leave the shade hollow.
        geom.add_face(outer_small[i], inner_small[j], inner_small[i])
        geom.add_face(outer_small[i], outer_small[j], inner_small[j])
        geom.add_face(outer_large[i], inner_large[i], inner_large[j])
        geom.add_face(outer_large[i], inner_large[j], outer_large[j])

    return geom


def _add_arm_link(part, *, length: float, metal: str, joint: str) -> None:
    """Add a double-rod link with a central input hub and forked output yoke."""

    # Input hinge barrel, centered between the parent fork ears.
    part.visual(
        Cylinder(radius=0.033, length=0.044),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint,
        name="start_hub",
    )

    # Two slender parallel tubes run most of the link length.
    rod_length = length - 0.050
    for sign, suffix in ((1.0, "upper_rod"), (-1.0, "lower_rod")):
        part.visual(
            Cylinder(radius=0.006, length=rod_length),
            origin=Origin(
                xyz=(0.020 + rod_length / 2.0, sign * 0.016, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=metal,
            name=suffix,
        )
        part.visual(
            Box((0.026, 0.012, 0.016)),
            origin=Origin(xyz=(length - 0.029, sign * 0.028, 0.0)),
            material=metal,
            name=f"{suffix}_web",
        )

    # Output fork ears.  The inner faces sit flush with the next central hub.
    part.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(length, 0.033, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint,
        name="upper_fork",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(length, -0.033, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint,
        name="lower_fork",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_reading_floor_lamp")

    model.material("matte_black", rgba=(0.04, 0.04, 0.045, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("brushed_brass", rgba=(0.70, 0.56, 0.30, 1.0))
    model.material("warm_cream", rgba=(0.92, 0.88, 0.76, 1.0))
    model.material("warm_light", rgba=(1.0, 0.76, 0.35, 0.72))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.205, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="rubber_black",
        name="rubber_foot",
    )
    base.visual(
        Cylinder(radius=0.190, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="matte_black",
        name="round_base",
    )

    lower_link = model.part("lower_link")
    lower_link.visual(
        Cylinder(radius=0.050, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="matte_black",
        name="base_collar",
    )
    lower_link.visual(
        Cylinder(radius=0.017, length=LOWER_LINK_HEIGHT - 0.073),
        origin=Origin(xyz=(0.0, 0.0, (LOWER_LINK_HEIGHT - 0.073) / 2.0 + 0.020)),
        material="brushed_brass",
        name="vertical_tube",
    )
    lower_link.visual(
        Box((0.070, 0.088, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_LINK_HEIGHT - 0.044)),
        material="matte_black",
        name="yoke_bridge",
    )
    lower_link.visual(
        Box((0.038, 0.022, 0.070)),
        origin=Origin(xyz=(0.0, 0.033, LOWER_LINK_HEIGHT - 0.018)),
        material="matte_black",
        name="upper_yoke_strap",
    )
    lower_link.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.0, 0.033, LOWER_LINK_HEIGHT), rpy=(pi / 2.0, 0.0, 0.0)),
        material="matte_black",
        name="upper_yoke_ear",
    )
    lower_link.visual(
        Box((0.038, 0.022, 0.070)),
        origin=Origin(xyz=(0.0, -0.033, LOWER_LINK_HEIGHT - 0.018)),
        material="matte_black",
        name="lower_yoke_strap",
    )
    lower_link.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.0, -0.033, LOWER_LINK_HEIGHT), rpy=(pi / 2.0, 0.0, 0.0)),
        material="matte_black",
        name="lower_yoke_ear",
    )

    middle_link = model.part("middle_link")
    _add_arm_link(
        middle_link,
        length=MIDDLE_LINK_LENGTH,
        metal="brushed_brass",
        joint="matte_black",
    )

    upper_link = model.part("upper_link")
    _add_arm_link(
        upper_link,
        length=UPPER_LINK_LENGTH,
        metal="brushed_brass",
        joint="matte_black",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.030, length=0.044),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="matte_black",
        name="tilt_hub",
    )
    shade.visual(
        Cylinder(radius=0.013, length=0.150),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_brass",
        name="neck_tube",
    )
    shade.visual(
        Cylinder(radius=0.052, length=0.030),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="warm_cream",
        name="shade_collar",
    )
    shade.visual(
        mesh_from_geometry(_shade_shell_geometry(), "shade_shell"),
        material="warm_cream",
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="matte_black",
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        material="warm_light",
        name="bulb",
    )

    model.articulation(
        "base_to_lower_link",
        ArticulationType.FIXED,
        parent=base,
        child=lower_link,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS)),
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=middle_link,
        origin=Origin(xyz=(0.0, 0.0, LOWER_LINK_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.5, lower=-0.90, upper=1.20),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=upper_link,
        origin=Origin(xyz=(MIDDLE_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.7, lower=-1.30, upper=1.35),
    )
    model.articulation(
        "upper_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=shade,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.4, lower=-1.10, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    revolute_names = [
        joint.name
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "three revolute adjustment joints",
        revolute_names == ["lower_to_middle", "middle_to_upper", "upper_to_shade"],
        details=f"revolute joints={revolute_names}",
    )

    base = object_model.get_part("base")
    lower = object_model.get_part("lower_link")
    middle = object_model.get_part("middle_link")
    upper = object_model.get_part("upper_link")
    shade = object_model.get_part("shade")

    ctx.expect_contact(
        lower,
        base,
        elem_a="base_collar",
        elem_b="round_base",
        name="vertical lower link is seated on round base",
    )
    ctx.expect_contact(
        lower,
        middle,
        elem_a="upper_yoke_ear",
        elem_b="start_hub",
        name="first elbow hub is captured by lower yoke",
    )
    ctx.expect_contact(
        middle,
        upper,
        elem_a="upper_fork",
        elem_b="start_hub",
        name="second elbow hub is captured by middle fork",
    )
    ctx.expect_contact(
        upper,
        shade,
        elem_a="upper_fork",
        elem_b="tilt_hub",
        name="shade tilt hub is captured at the tip",
    )

    middle_to_upper = object_model.get_articulation("middle_to_upper")
    upper_to_shade = object_model.get_articulation("upper_to_shade")

    shade_rest = ctx.part_world_position(shade)
    with ctx.pose({middle_to_upper: 0.70}):
        shade_raised = ctx.part_world_position(shade)
    ctx.check(
        "second elbow raises the tip",
        shade_rest is not None
        and shade_raised is not None
        and shade_raised[2] > shade_rest[2] + 0.10,
        details=f"rest={shade_rest}, raised={shade_raised}",
    )

    bulb_rest = ctx.part_element_world_aabb(shade, elem="bulb")
    with ctx.pose({upper_to_shade: -0.60}):
        bulb_tilted = ctx.part_element_world_aabb(shade, elem="bulb")
    rest_bulb_z = None if bulb_rest is None else (bulb_rest[0][2] + bulb_rest[1][2]) / 2.0
    tilted_bulb_z = (
        None if bulb_tilted is None else (bulb_tilted[0][2] + bulb_tilted[1][2]) / 2.0
    )
    ctx.check(
        "shade tilt aims the bulb downward",
        rest_bulb_z is not None
        and tilted_bulb_z is not None
        and tilted_bulb_z < rest_bulb_z - 0.04,
        details=f"rest_bulb_z={rest_bulb_z}, tilted_bulb_z={tilted_bulb_z}",
    )

    return ctx.report()


object_model = build_object_model()
