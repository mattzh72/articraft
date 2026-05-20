from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_lamp")

    base_finish = model.material("base_finish", rgba=(0.16, 0.17, 0.19, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.70, 0.60, 0.33, 1.0))
    shade_finish = model.material("shade_finish", rgba=(0.86, 0.84, 0.77, 1.0))
    socket_finish = model.material("socket_finish", rgba=(0.22, 0.23, 0.24, 1.0))
    bulb_glass = model.material("bulb_glass", rgba=(0.94, 0.92, 0.82, 0.65))

    base_radius = 0.11
    base_thickness = 0.018
    stem_height = 0.140
    shoulder_z = 0.176
    lower_arm_length = 0.210
    upper_arm_length = 0.180

    base = model.part("base")
    base.visual(
        Cylinder(radius=base_radius, length=base_thickness),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=base_finish,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.018, length=stem_height),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + stem_height / 2.0)),
        material=base_finish,
        name="stem",
    )
    for y_pos, name in ((-0.018, "shoulder_ear_0"), (0.018, "shoulder_ear_1")):
        base.visual(
            Box((0.018, 0.010, 0.036)),
            origin=Origin(xyz=(0.0, y_pos, shoulder_z)),
            material=base_finish,
            name=name,
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=base_radius, length=base_thickness),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Box((lower_arm_length, 0.024, 0.012)),
        origin=Origin(xyz=(lower_arm_length / 2.0, 0.0, 0.0)),
        material=arm_finish,
        name="lower_beam",
    )
    lower_arm.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_finish,
        name="shoulder_barrel",
    )
    lower_arm.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(
            xyz=(lower_arm_length - 0.010, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=arm_finish,
        name="elbow_bridge",
    )
    for y_pos, name in ((-0.0175, "elbow_ear_0"), (0.0175, "elbow_ear_1")):
        lower_arm.visual(
            Box((0.024, 0.011, 0.030)),
            origin=Origin(xyz=(lower_arm_length - 0.012, y_pos, 0.0)),
            material=arm_finish,
            name=name,
        )
    lower_arm.inertial = Inertial.from_geometry(
        Box((lower_arm_length, 0.030, 0.030)),
        mass=0.42,
        origin=Origin(xyz=(lower_arm_length / 2.0, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((upper_arm_length, 0.024, 0.012)),
        origin=Origin(xyz=(upper_arm_length / 2.0, 0.0, 0.0)),
        material=arm_finish,
        name="upper_beam",
    )
    upper_arm.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(
            xyz=(upper_arm_length - 0.009, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=arm_finish,
        name="head_bridge",
    )
    for y_pos, name in ((-0.0175, "head_ear_0"), (0.0175, "head_ear_1")):
        upper_arm.visual(
            Box((0.024, 0.011, 0.028)),
            origin=Origin(xyz=(upper_arm_length - 0.012, y_pos, 0.0)),
            material=arm_finish,
            name=name,
        )
    upper_arm.inertial = Inertial.from_geometry(
        Box((upper_arm_length, 0.030, 0.030)),
        mass=0.35,
        origin=Origin(xyz=(upper_arm_length / 2.0, 0.0, 0.0)),
    )

    head = model.part("head")
    head.visual(
        Box((0.030, 0.020, 0.018)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=socket_finish,
        name="head_neck",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=socket_finish,
        name="socket_housing",
    )

    outer_profile = [(0.020, 0.0), (0.028, 0.010), (0.044, 0.070), (0.060, 0.140)]
    inner_profile = [(0.0, 0.010), (0.020, 0.018), (0.036, 0.074), (0.052, 0.132)]
    shade_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=48),
        "shade_shell",
    )
    head.visual(
        shade_shell,
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shade_finish,
        name="shade_shell",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.042),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=socket_finish,
        name="socket",
    )
    head.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.084, 0.0, 0.0)),
        material=bulb_glass,
        name="bulb",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.120)),
        mass=0.38,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=1.15, effort=18.0, velocity=1.6),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(lower_arm_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.25, upper=1.35, effort=14.0, velocity=1.8),
    )
    model.articulation(
        "upper_arm_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(upper_arm_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.80, upper=0.65, effort=6.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    head = object_model.get_part("head")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    wrist = object_model.get_articulation("upper_arm_to_head")

    ctx.expect_origin_gap(
        head,
        base,
        axis="x",
        min_gap=0.34,
        name="lamp head projects forward of the base",
    )
    ctx.expect_origin_gap(
        head,
        base,
        axis="z",
        min_gap=0.15,
        name="lamp head sits above the weighted base",
    )
    ctx.expect_origin_gap(
        upper_arm,
        lower_arm,
        axis="x",
        min_gap=0.16,
        name="upper arm starts ahead of the lower arm pivot",
    )

    rest_pos = ctx.part_world_position(head)
    with ctx.pose({shoulder: 0.80, elbow: 0.55, wrist: 0.20}):
        raised_pos = ctx.part_world_position(head)
        ctx.expect_origin_gap(
            head,
            base,
            axis="z",
            min_gap=0.33,
            name="raised pose lifts the lamp head high above the base",
        )

    ctx.check(
        "articulated arm raises the lamp head",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.18
        and raised_pos[0] > 0.15,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
