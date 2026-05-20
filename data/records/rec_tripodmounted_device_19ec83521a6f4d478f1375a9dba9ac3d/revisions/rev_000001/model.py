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
    Sphere,
    TestContext,
    TestReport,
)


def _radial_xy(radius: float, theta: float) -> tuple[float, float]:
    return (radius * math.cos(theta), radius * math.sin(theta))


def _radial_origin(radius: float, theta: float, z: float) -> Origin:
    x, y = _radial_xy(radius, theta)
    return Origin(xyz=(x, y, z), rpy=(0.0, 0.0, theta))


def _tangent_xy(offset: float, theta: float) -> tuple[float, float]:
    return (-offset * math.sin(theta), offset * math.cos(theta))


def _cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    material: Material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > 1e-9:
        raise ValueError("_cylinder_between is specialized for the leg XZ plane")
    pitch = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(
                (start[0] + end[0]) * 0.5,
                (start[1] + end[1]) * 0.5,
                (start[2] + end[2]) * 0.5,
            ),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_mounted_device")

    matte_black = model.material("matte_black", color=(0.015, 0.015, 0.018, 1.0))
    dark_metal = model.material("dark_metal", color=(0.06, 0.065, 0.07, 1.0))
    satin_aluminum = model.material("satin_aluminum", color=(0.68, 0.70, 0.70, 1.0))
    rubber = model.material("black_rubber", color=(0.005, 0.005, 0.004, 1.0))
    plate_gray = model.material("plate_gray", color=(0.12, 0.125, 0.13, 1.0))
    screw_metal = model.material("screw_metal", color=(0.85, 0.82, 0.72, 1.0))
    lens_glass = model.material("lens_glass", color=(0.05, 0.09, 0.12, 1.0))

    crown_z = 0.90
    pan_z = 0.94
    hinge_radius = 0.115

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, crown_z)),
        material=dark_metal,
        name="center_hub",
    )
    crown.visual(
        Cylinder(radius=0.068, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, pan_z - 0.009)),
        material=matte_black,
        name="top_collar",
    )

    leg_angles = (math.radians(90.0), math.radians(210.0), math.radians(330.0))
    for idx, theta in enumerate(leg_angles):
        crown.visual(
            Box((0.105, 0.030, 0.024)),
            origin=_radial_origin(0.052, theta, crown_z - 0.042),
            material=dark_metal,
            name=f"hinge_strut_{idx}",
        )
        crown.visual(
            Box((0.052, 0.108, 0.018)),
            origin=_radial_origin(0.054, theta, crown_z - 0.052),
            material=dark_metal,
            name=f"hinge_crossbar_{idx}",
        )
        for side, offset in enumerate((-0.043, 0.043)):
            rx, ry = _radial_xy(0.097, theta)
            tx, ty = _tangent_xy(offset, theta)
            crown.visual(
                Box((0.052, 0.020, 0.026)),
                origin=Origin(
                    xyz=(rx + tx, ry + ty, crown_z - 0.042),
                    rpy=(0.0, 0.0, theta),
                ),
                material=dark_metal,
                name=f"hinge_bridge_{idx}_{side}",
            )
            rx, ry = _radial_xy(hinge_radius, theta)
            tx, ty = _tangent_xy(offset, theta)
            crown.visual(
                Box((0.040, 0.016, 0.075)),
                origin=Origin(
                    xyz=(rx + tx, ry + ty, crown_z),
                    rpy=(0.0, 0.0, theta),
                ),
                material=dark_metal,
                name=f"hinge_cheek_{idx}_{side}",
            )

    legs = []
    for idx, theta in enumerate(leg_angles):
        leg = model.part(f"leg_{idx}")
        legs.append(leg)
        leg.visual(
            Cylinder(radius=0.021, length=0.070),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_barrel",
        )
        leg.visual(
            Sphere(radius=0.024),
            origin=Origin(xyz=(0.018, 0.0, -0.016)),
            material=dark_metal,
            name="leg_knuckle",
        )
        _cylinder_between(
            leg,
            (0.018, 0.0, -0.016),
            (0.285, 0.0, -0.455),
            0.017,
            material=matte_black,
            name="upper_tube",
        )
        _cylinder_between(
            leg,
            (0.250, 0.0, -0.400),
            (0.530, 0.0, -0.860),
            0.013,
            material=satin_aluminum,
            name="lower_tube",
        )
        leg.visual(
            Sphere(radius=0.026),
            origin=Origin(xyz=(0.530, 0.0, -0.860)),
            material=rubber,
            name="rubber_foot",
        )

        hx, hy = _radial_xy(hinge_radius, theta)
        model.articulation(
            f"crown_to_leg_{idx}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(hx, hy, crown_z), rpy=(0.0, 0.0, theta)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=35.0,
                velocity=1.5,
                lower=0.0,
                upper=1.20,
            ),
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.062, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=matte_black,
        name="pan_disk",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        material=dark_metal,
        name="short_neck",
    )
    head.visual(
        Box((0.070, 0.130, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_metal,
        name="yoke_base",
    )
    for side, y in enumerate((-0.061, 0.061)):
        head.visual(
            Box((0.040, 0.014, 0.085)),
            origin=Origin(xyz=(0.0, y, 0.135)),
            material=dark_metal,
            name=f"yoke_arm_{side}",
        )

    model.articulation(
        "crown_to_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, pan_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0),
    )

    plate = model.part("device_plate")
    plate.visual(
        Cylinder(radius=0.010, length=0.108),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=screw_metal,
        name="tilt_axle",
    )
    plate.visual(
        Box((0.035, 0.035, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=plate_gray,
        name="tilt_boss",
    )
    plate.visual(
        Box((0.170, 0.096, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=plate_gray,
        name="mounting_plate",
    )
    for idx, x in enumerate((-0.045, 0.045)):
        plate.visual(
            Box((0.045, 0.010, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.035)),
            material=rubber,
            name=f"grip_pad_{idx}",
        )
    plate.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=screw_metal,
        name="mounting_screw",
    )
    plate.visual(
        Box((0.120, 0.070, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=matte_black,
        name="device_body",
    )
    plate.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.074, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="device_lens",
    )

    model.articulation(
        "head_to_device_plate",
        ArticulationType.REVOLUTE,
        parent=head,
        child=plate,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    head = object_model.get_part("head")
    plate = object_model.get_part("device_plate")

    pan = object_model.get_articulation("crown_to_head")
    tilt = object_model.get_articulation("head_to_device_plate")
    leg_joints = [
        object_model.get_articulation(f"crown_to_leg_{idx}") for idx in range(3)
    ]

    ctx.check(
        "three folding leg hinges",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.parent == "crown"
            and joint.child == f"leg_{idx}"
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper >= 1.0
            for idx, joint in enumerate(leg_joints)
        ),
        details="Each leg must be a revolute child of the tripod crown with a folding range.",
    )
    for idx in range(3):
        ctx.expect_contact(
            f"leg_{idx}",
            crown,
            elem_a="hinge_barrel",
            contact_tol=1e-6,
            name=f"leg_{idx} hinge barrel is seated in the crown",
        )

    ctx.check(
        "pan joint is vertical and compact",
        pan.articulation_type == ArticulationType.CONTINUOUS
        and pan.axis == (0.0, 0.0, 1.0)
        and 0.0 <= pan.origin.xyz[2] - 0.90 <= 0.055,
        details=f"pan_axis={pan.axis}, pan_origin={pan.origin.xyz}",
    )
    ctx.expect_contact(
        head,
        crown,
        elem_a="pan_disk",
        elem_b="top_collar",
        contact_tol=1e-6,
        name="pan disk sits directly on the crown collar",
    )

    ctx.check(
        "device plate tilt axis is horizontal",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.axis == (0.0, 1.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.0,
        details=f"tilt_axis={tilt.axis}, tilt_limits={tilt.motion_limits}",
    )
    ctx.check(
        "tilt stage stays close to pan head",
        0.10 <= tilt.origin.xyz[2] <= 0.16,
        details=f"tilt_origin={tilt.origin.xyz}",
    )
    ctx.expect_contact(
        plate,
        head,
        elem_a="tilt_axle",
        contact_tol=1e-6,
        name="tilt axle is captured between the yoke arms",
    )

    rest_foot = ctx.part_element_world_aabb("leg_0", elem="rubber_foot")
    with ctx.pose({leg_joints[0]: 1.0}):
        folded_foot = ctx.part_element_world_aabb("leg_0", elem="rubber_foot")
    ctx.check(
        "leg hinge folds the foot upward",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[0][2] > rest_foot[1][2] + 0.20,
        details=f"rest_foot={rest_foot}, folded_foot={folded_foot}",
    )

    rest_plate = ctx.part_element_world_aabb(plate, elem="mounting_plate")
    with ctx.pose({tilt: 0.65}):
        tilted_plate = ctx.part_element_world_aabb(plate, elem="mounting_plate")
    ctx.check(
        "device plate visibly tilts",
        rest_plate is not None
        and tilted_plate is not None
        and tilted_plate[1][2] > rest_plate[1][2] + 0.025,
        details=f"rest_plate={rest_plate}, tilted_plate={tilted_plate}",
    )

    return ctx.report()


object_model = build_object_model()
