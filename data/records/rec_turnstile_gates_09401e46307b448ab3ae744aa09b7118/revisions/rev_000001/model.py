from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_mesh(name: str, outer_radius: float, inner_radius: float, height: float):
    """A vertical hollow cylinder, centered on its local origin."""
    shape = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.05)


def _radial_xyz(radius: float, yaw: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(yaw), radius * math.sin(yaw), z)


def _radial_origin(radius: float, yaw: float, z: float) -> Origin:
    # A cylinder's local +Z axis is turned into the horizontal radial direction.
    return Origin(xyz=_radial_xyz(radius, yaw, z), rpy=(0.0, math.pi / 2.0, yaw))


def _yawed_box_origin(radius: float, yaw: float, z: float) -> Origin:
    return Origin(xyz=_radial_xyz(radius, yaw, z), rpy=(0.0, 0.0, yaw))


def _cylinder_between(part, p0, p1, radius: float, *, material, name: str) -> None:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.acos(dz / length)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_turnstile_gate")

    cast_iron = model.material("aged_cast_iron", rgba=(0.08, 0.085, 0.08, 1.0))
    worn_black = model.material("worn_black_enamel", rgba=(0.015, 0.018, 0.018, 1.0))
    gunmetal = model.material("gunmetal_spindle", rgba=(0.32, 0.34, 0.34, 1.0))
    oiled_steel = model.material("oiled_steel", rgba=(0.55, 0.57, 0.55, 1.0))
    brass = model.material("aged_bronze_bearing", rgba=(0.72, 0.55, 0.23, 1.0))
    service_red = model.material("legacy_hatch_red", rgba=(0.45, 0.06, 0.035, 1.0))
    rubber = model.material("black_rubber_end_caps", rgba=(0.01, 0.01, 0.01, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((2.20, 2.10, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_iron,
        name="floor_threshold",
    )
    frame.visual(
        Cylinder(radius=0.245, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.1075)),
        material=cast_iron,
        name="flanged_plinth",
    )
    frame.visual(
        Cylinder(radius=0.060, length=1.46),
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        material=gunmetal,
        name="supported_spindle",
    )
    frame.visual(
        _annular_mesh("lower_bearing_cup", 0.175, 0.068, 0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.7425)),
        material=brass,
        name="lower_bearing_cup",
    )
    frame.visual(
        Cylinder(radius=0.070, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.7425)),
        material=gunmetal,
        name="lower_bearing_boss",
    )
    frame.visual(
        _annular_mesh("upper_bearing_cap", 0.178, 0.068, 0.095),
        origin=Origin(xyz=(0.0, 0.0, 1.2175)),
        material=brass,
        name="upper_bearing_cap",
    )
    frame.visual(
        Cylinder(radius=0.070, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 1.2175)),
        material=gunmetal,
        name="upper_bearing_boss",
    )
    frame.visual(
        Cylinder(radius=0.095, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=oiled_steel,
        name="spindle_nut",
    )

    for i, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        frame.visual(
            Box((0.36, 0.020, 0.30)),
            origin=_yawed_box_origin(0.13, yaw, 0.205),
            material=cast_iron,
            name=f"plinth_gusset_{i}",
        )

    # Bolted base circle, with every head seated into the cast flange.
    for i in range(12):
        yaw = i * math.tau / 12.0
        frame.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=_radial_xyz(0.205, yaw, 0.167)),
            material=oiled_steel,
            name=f"plinth_bolt_{i}",
        )

    # Old service hatches are proud on the spindle casing, with hinge barrels and screw heads.
    for side, yaw, suffix in ((-1.0, -math.pi / 2.0, "front"), (1.0, math.pi / 2.0, "rear")):
        y = side * 0.080
        frame.visual(
            Box((0.225, 0.040, 0.285)),
            origin=Origin(xyz=(0.0, y, 0.445)),
            material=service_red,
            name=f"{suffix}_service_hatch",
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.245),
            origin=Origin(xyz=(-0.122, side * 0.096, 0.445)),
            material=oiled_steel,
            name=f"{suffix}_hatch_hinge",
        )
        for j, (x, z) in enumerate(((-0.075, 0.345), (0.075, 0.345), (-0.075, 0.545), (0.075, 0.545))):
            frame.visual(
                Cylinder(radius=0.010, length=0.012),
                origin=Origin(xyz=(x, side * 0.103, z), rpy=(side * math.pi / 2.0, 0.0, 0.0)),
                material=oiled_steel,
                name=f"{suffix}_hatch_screw_{j}",
            )

    # Side lane rails are tied into the same threshold plate, so the guide work is not floating.
    for y, suffix in ((-0.93, "front"), (0.93, "rear")):
        for x, post_name in ((-0.93, "entry"), (0.93, "exit")):
            frame.visual(
                Cylinder(radius=0.045, length=0.95),
                origin=Origin(xyz=(x, y, 0.53)),
                material=worn_black,
                name=f"{suffix}_{post_name}_rail_post",
            )
            frame.visual(
                Cylinder(radius=0.070, length=0.028),
                origin=Origin(xyz=(x, y, 0.069)),
                material=cast_iron,
                name=f"{suffix}_{post_name}_post_foot",
            )
        for z, rail_name in ((0.55, "lower"), (0.91, "upper")):
            frame.visual(
                Cylinder(radius=0.026, length=1.86),
                origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=worn_black,
                name=f"{suffix}_{rail_name}_guard_rail",
            )

    rotor = model.part("rotor")
    rotor.visual(
        _annular_mesh("rotating_hub_sleeve", 0.145, 0.084, 0.380),
        origin=Origin(),
        material=oiled_steel,
        name="rotating_hub_sleeve",
    )
    rotor.visual(
        _annular_mesh("upper_adapter_flange", 0.205, 0.086, 0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=cast_iron,
        name="upper_adapter_flange",
    )
    rotor.visual(
        _annular_mesh("lower_adapter_flange", 0.205, 0.086, 0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.165)),
        material=cast_iron,
        name="lower_adapter_flange",
    )

    arm_length = 0.700
    arm_start = 0.130
    arm_radius = 0.032
    for i, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        rotor.visual(
            Box((0.155, 0.118, 0.112)),
            origin=_yawed_box_origin(0.195, yaw, 0.0),
            material=cast_iron,
            name=f"arm_adapter_{i}",
        )
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=_radial_origin(arm_start + arm_length / 2.0, yaw, 0.0),
            material=oiled_steel,
            name=f"radial_arm_{i}",
        )
        rotor.visual(
            Cylinder(radius=0.030, length=0.48),
            origin=Origin(xyz=_radial_xyz(arm_start + arm_length, yaw, -0.205)),
            material=oiled_steel,
            name=f"drop_bar_{i}",
        )
        rotor.visual(
            Cylinder(radius=0.040, length=0.055),
            origin=_radial_origin(arm_start + arm_length + 0.025, yaw, 0.0),
            material=rubber,
            name=f"arm_end_cap_{i}",
        )

        # Diagonal compression brace from the lower hub flange to each radial tube.
        p0 = _radial_xyz(0.160, yaw, -0.150)
        p1 = _radial_xyz(0.520, yaw, -0.030)
        _cylinder_between(rotor, p0, p1, 0.016, material=oiled_steel, name=f"arm_brace_{i}")

        for j, tangent in enumerate((-0.043, 0.043)):
            # Top-access bolts on the adapter saddle.
            x = 0.190 * math.cos(yaw) - tangent * math.sin(yaw)
            y = 0.190 * math.sin(yaw) + tangent * math.cos(yaw)
            rotor.visual(
                Cylinder(radius=0.011, length=0.014),
                origin=Origin(xyz=(x, y, 0.0625)),
                material=gunmetal,
                name=f"adapter_bolt_{i}_{j}",
            )

    model.articulation(
        "spindle_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.980)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.4),
        motion_properties=MotionProperties(damping=0.22, friction=0.08),
        meta={"description": "radial arm gate rotates about the supported central spindle"},
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("spindle_rotation")

    ctx.check(
        "rotation axis is vertical spindle",
        tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={joint.axis}",
    )
    ctx.expect_origin_distance(
        rotor,
        frame,
        axes="xy",
        max_dist=0.001,
        name="rotor is centered on frame spindle",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="rotating_hub_sleeve",
        outer_elem="supported_spindle",
        margin=0.150,
        name="bearing sleeve surrounds the spindle centerline",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="rotating_hub_sleeve",
        negative_elem="lower_bearing_cup",
        min_gap=0.0,
        max_gap=0.003,
        name="hub clears lower bearing cup",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="upper_bearing_cap",
        negative_elem="rotating_hub_sleeve",
        min_gap=0.0,
        max_gap=0.003,
        name="upper bearing cap clears rotating hub",
    )

    rest_arm = ctx.part_element_world_aabb(rotor, elem="radial_arm_0")
    with ctx.pose({joint: math.pi / 2.0}):
        turned_arm = ctx.part_element_world_aabb(rotor, elem="radial_arm_0")

    ctx.check(
        "radial arm visibly rotates around spindle",
        rest_arm is not None
        and turned_arm is not None
        and rest_arm[1][0] > 0.78
        and abs(rest_arm[1][1]) < 0.05
        and turned_arm[1][1] > 0.78
        and abs(turned_arm[1][0]) < 0.05,
        details=f"rest={rest_arm}, turned={turned_arm}",
    )

    return ctx.report()


object_model = build_object_model()
