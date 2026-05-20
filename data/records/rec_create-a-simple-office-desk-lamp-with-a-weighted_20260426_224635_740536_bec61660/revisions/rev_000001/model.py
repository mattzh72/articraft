from __future__ import annotations

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
    TorusGeometry,
    mesh_from_geometry,
)


BASE_HINGE_Z = 0.115
LOWER_ARM_END = (0.240, 0.0, 0.320)
UPPER_ARM_END = (0.285, 0.0, -0.060)


def _cylinder_between(part, start, end, radius, material, name):
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # The lamp arms live in the XZ plane; this helper rotates the cylinder's
    # local +Z axis onto the requested strut direction.
    pitch = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def _y_axis_cylinder(part, xyz, radius, length, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _shade_mesh():
    shade = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.032, 0.000),
            (0.044, -0.020),
            (0.060, -0.052),
            (0.076, -0.088),
        ],
        inner_profile=[
            (0.026, -0.003),
            (0.038, -0.022),
            (0.054, -0.053),
            (0.070, -0.086),
        ],
        segments=56,
        start_cap="round",
        end_cap="round",
        lip_samples=6,
    )
    shade.merge(TorusGeometry(radius=0.073, tube=0.003, radial_segments=12, tubular_segments=56).translate(0.0, 0.0, -0.087))
    shade.translate(0.105, 0.0, 0.0)
    return shade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_desk_lamp")

    matte_black = model.material("matte_black", rgba=(0.02, 0.022, 0.026, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.12, 0.13, 0.15, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    warm_light = model.material("warm_light", rgba=(1.0, 0.82, 0.42, 0.82))
    rubber = model.material("rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.142, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=matte_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.150, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="rubber_foot",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=dark_grey,
        name="base_pedestal",
    )
    for i, y in enumerate((-0.052, 0.052)):
        base.visual(
            Box((0.032, 0.014, 0.092)),
            origin=Origin(xyz=(0.0, y, 0.079)),
            material=dark_grey,
            name=f"base_yoke_{i}",
        )
        _y_axis_cylinder(
            base,
            (0.0, y, BASE_HINGE_Z),
            radius=0.018,
            length=0.014,
            material=brushed_steel,
            name=f"base_bushing_{i}",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.142, length=0.035),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
    )

    lower_arm = model.part("lower_arm")
    _y_axis_cylinder(
        lower_arm,
        (0.0, 0.0, 0.0),
        radius=0.017,
        length=0.090,
        material=brushed_steel,
        name="base_hinge_barrel",
    )
    for i, y in enumerate((-0.025, 0.025)):
        _cylinder_between(
            lower_arm,
            (0.010, y, 0.010),
            (LOWER_ARM_END[0] - 0.010, y, LOWER_ARM_END[2] - 0.010),
            radius=0.006,
            material=brushed_steel,
            name=f"lower_strut_{i}",
        )
    _y_axis_cylinder(
        lower_arm,
        LOWER_ARM_END,
        radius=0.018,
        length=0.092,
        material=brushed_steel,
        name="elbow_hinge_barrel",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.30, 0.07, 0.34)),
        mass=0.34,
        origin=Origin(xyz=(0.120, 0.0, 0.160)),
    )

    upper_arm = model.part("upper_arm")
    for i, y in enumerate((-0.052, 0.052)):
        upper_arm.visual(
            Box((0.046, 0.012, 0.052)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_grey,
            name=f"elbow_yoke_{i}",
        )
        _cylinder_between(
            upper_arm,
            (0.010, y, -0.002),
            (UPPER_ARM_END[0] - 0.012, y, UPPER_ARM_END[2] + 0.006),
            radius=0.006,
            material=brushed_steel,
            name=f"upper_strut_{i}",
        )
        upper_arm.visual(
            Box((0.044, 0.012, 0.050)),
            origin=Origin(xyz=UPPER_ARM_END[0:1] + (y,) + UPPER_ARM_END[2:3]),
            material=dark_grey,
            name=f"wrist_yoke_{i}",
        )
    _y_axis_cylinder(
        upper_arm,
        (0.142, 0.0, -0.030),
        radius=0.005,
        length=0.116,
        material=brushed_steel,
        name="upper_cross_tie",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.32, 0.12, 0.09)),
        mass=0.30,
        origin=Origin(xyz=(0.145, 0.0, -0.030)),
    )

    lamp_head = model.part("lamp_head")
    _y_axis_cylinder(
        lamp_head,
        (0.0, 0.0, 0.0),
        radius=0.016,
        length=0.092,
        material=brushed_steel,
        name="wrist_hinge_barrel",
    )
    _cylinder_between(
        lamp_head,
        (0.012, 0.0, -0.002),
        (0.078, 0.0, -0.006),
        radius=0.007,
        material=brushed_steel,
        name="head_neck",
    )
    lamp_head.visual(
        Cylinder(radius=0.036, length=0.014),
        origin=Origin(xyz=(0.105, 0.0, -0.002)),
        material=dark_grey,
        name="shade_top_cap",
    )
    lamp_head.visual(
        mesh_from_geometry(_shade_mesh(), "shade_shell"),
        material=matte_black,
        name="shade_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(0.105, 0.0, -0.025)),
        material=dark_grey,
        name="lamp_socket",
    )
    lamp_head.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.105, 0.0, -0.058)),
        material=warm_light,
        name="bulb",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.20, 0.16, 0.12)),
        mass=0.38,
        origin=Origin(xyz=(0.090, 0.0, -0.040)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.70, upper=0.85),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=LOWER_ARM_END),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-1.20, upper=1.05),
    )
    model.articulation(
        "upper_arm_to_lamp_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lamp_head,
        origin=Origin(xyz=UPPER_ARM_END),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.8, lower=-0.90, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_arm")
    upper = object_model.get_part("upper_arm")
    head = object_model.get_part("lamp_head")

    ctx.expect_contact(lower, base, contact_tol=0.002, name="lower arm is seated in the base hinge")
    ctx.expect_contact(upper, lower, contact_tol=0.002, name="upper arm is seated in the elbow hinge")
    ctx.expect_contact(head, upper, contact_tol=0.002, name="lamp head is seated in the wrist hinge")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    wrist = object_model.get_articulation("upper_arm_to_lamp_head")
    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({shoulder: 0.65, elbow: 0.45}):
        raised_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "arm articulation raises the lamp head",
        rest_head_aabb is not None
        and raised_head_aabb is not None
        and float(raised_head_aabb[1][2] - rest_head_aabb[1][2]) > 0.08,
        details=f"rest={rest_head_aabb}, raised={raised_head_aabb}",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({wrist: 0.55}):
        tilted_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "wrist tilt visibly changes shade pose",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and abs(float(tilted_head_aabb[1][2] - rest_head_aabb[1][2])) > 0.050,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
