from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


LOWER_ARM_LENGTH = 0.38
UPPER_ARM_LENGTH = 0.36
SHOULDER_Z = 0.145


def _conical_shade_shell(
    *,
    length: float = 0.185,
    neck_radius: float = 0.040,
    mouth_radius: float = 0.105,
    wall: float = 0.004,
    segments: int = 72,
) -> MeshGeometry:
    """Thin hollow frustum along local +X, open at the mouth."""

    geom = MeshGeometry()
    outer_neck: list[int] = []
    outer_mouth: list[int] = []
    inner_neck: list[int] = []
    inner_mouth: list[int] = []

    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        outer_neck.append(geom.add_vertex(0.0, neck_radius * ca, neck_radius * sa))
        outer_mouth.append(geom.add_vertex(length, mouth_radius * ca, mouth_radius * sa))
        inner_neck.append(geom.add_vertex(0.003, (neck_radius - wall) * ca, (neck_radius - wall) * sa))
        inner_mouth.append(
            geom.add_vertex(length - 0.003, (mouth_radius - wall) * ca, (mouth_radius - wall) * sa)
        )

    for i in range(segments):
        j = (i + 1) % segments
        # Outside conical skin.
        geom.add_face(outer_neck[i], outer_mouth[i], outer_mouth[j])
        geom.add_face(outer_neck[i], outer_mouth[j], outer_neck[j])
        # Inside liner, reversed so its normals face inward toward the hollow.
        geom.add_face(inner_neck[i], inner_mouth[j], inner_mouth[i])
        geom.add_face(inner_neck[i], inner_neck[j], inner_mouth[j])
        # Small neck rim and rolled front lip close the sheet metal thickness
        # without capping the hollow openings.
        geom.add_face(outer_neck[i], outer_neck[j], inner_neck[j])
        geom.add_face(outer_neck[i], inner_neck[j], inner_neck[i])
        geom.add_face(outer_mouth[i], inner_mouth[i], inner_mouth[j])
        geom.add_face(outer_mouth[i], inner_mouth[j], outer_mouth[j])

    return geom


def _coil_spring(
    *,
    length: float,
    x0: float,
    z0: float,
    radius: float = 0.012,
    wire_radius: float = 0.0023,
    turns: int = 8,
) -> MeshGeometry:
    points = []
    samples = turns * 20
    for i in range(samples + 1):
        t = i / samples
        a = 2.0 * math.pi * turns * t
        points.append((x0 + length * t, radius * math.cos(a), z0 + radius * math.sin(a)))
    return tube_from_spline_points(
        points,
        radius=wire_radius,
        samples_per_segment=2,
        radial_segments=10,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _add_arm_segment(
    part,
    *,
    length: float,
    near_barrel_name: str,
    far_yoke_prefix: str,
    rod_material,
    joint_material,
    spring_material,
) -> None:
    # Main hinge barrel at the incoming pivot.  It sits between the parent yoke
    # cheeks and visually defines the child part's revolute axis.
    part.visual(
        Cylinder(radius=0.022, length=0.078),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_material,
        name=near_barrel_name,
    )

    for index, y in enumerate((-0.028, 0.028)):
        part.visual(
            Cylinder(radius=0.0065, length=length - 0.072),
            origin=Origin(xyz=(length / 2.0, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rod_material,
            name=f"rod_{index}",
        )

    # Web blocks and the far yoke make the pair of rods read as one rigid arm
    # segment rather than separate floating tubes.
    part.visual(
        Box((0.032, 0.070, 0.014)),
        origin=Origin(xyz=(0.027, 0.0, 0.0)),
        material=joint_material,
        name="near_web",
    )
    part.visual(
        Box((0.032, 0.090, 0.014)),
        origin=Origin(xyz=(length - 0.040, 0.0, 0.0)),
        material=joint_material,
        name="far_web",
    )
    for index, y in enumerate((-0.045, 0.045)):
        part.visual(
            Box((0.054, 0.012, 0.060)),
            origin=Origin(xyz=(length, y, 0.0)),
            material=joint_material,
            name=f"{far_yoke_prefix}_{index}",
        )

    spring_x0 = 0.055
    spring_len = length - 0.110
    spring_z = 0.043
    part.visual(
        mesh_from_geometry(
            _coil_spring(length=spring_len, x0=spring_x0, z0=spring_z),
            f"{part.name}_spring",
        ),
        material=spring_material,
        name="tension_spring",
    )
    for index, x in enumerate((spring_x0, spring_x0 + spring_len)):
        part.visual(
            Cylinder(radius=0.0045, length=spring_z),
            origin=Origin(xyz=(x, 0.0, spring_z / 2.0)),
            material=joint_material,
            name=f"spring_post_{index}",
        )
        part.visual(
            Cylinder(radius=0.004, length=0.055),
            origin=Origin(xyz=(x, 0.0, spring_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=joint_material,
            name=f"spring_hook_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architect_spring_arm_lamp")

    powder_black = model.material("powder_black", rgba=(0.015, 0.014, 0.013, 1.0))
    satin_black = model.material("satin_black", rgba=(0.055, 0.055, 0.052, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.68, 0.66, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.84, 0.84, 0.80, 1.0))
    warm_glass = model.material("warm_glass", rgba=(1.0, 0.82, 0.42, 0.65))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            # A low lathed disk with a soft bevel reads as the weighted cast base.
            MeshGeometry()
            .merge(LatheGeometry(
                [
                    (0.000, 0.000),
                    (0.135, 0.000),
                    (0.158, 0.009),
                    (0.160, 0.033),
                    (0.138, 0.045),
                    (0.000, 0.045),
                ],
                segments=80,
                closed=True,
            )),
            "weighted_base",
        ),
        material=powder_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=satin_black,
        name="base_boss",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=satin_black,
        name="upright_post",
    )
    base.visual(
        Box((0.048, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=satin_black,
        name="shoulder_bridge",
    )
    for index, y in enumerate((-0.045, 0.045)):
        base.visual(
            Box((0.058, 0.012, 0.070)),
            origin=Origin(xyz=(0.0, y, SHOULDER_Z)),
            material=satin_black,
            name=f"shoulder_yoke_{index}",
        )
        base.visual(
            Cylinder(radius=0.025, length=0.004),
            origin=Origin(xyz=(0.0, y * 1.13, SHOULDER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"shoulder_cap_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.045),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )

    lower_arm = model.part("lower_arm")
    _add_arm_segment(
        lower_arm,
        length=LOWER_ARM_LENGTH,
        near_barrel_name="shoulder_barrel",
        far_yoke_prefix="elbow_yoke",
        rod_material=brushed_steel,
        joint_material=satin_black,
        spring_material=spring_steel,
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH, 0.10, 0.08)),
        mass=0.42,
        origin=Origin(xyz=(LOWER_ARM_LENGTH / 2.0, 0.0, 0.015)),
    )

    upper_arm = model.part("upper_arm")
    _add_arm_segment(
        upper_arm,
        length=UPPER_ARM_LENGTH,
        near_barrel_name="elbow_barrel",
        far_yoke_prefix="shade_yoke",
        rod_material=brushed_steel,
        joint_material=satin_black,
        spring_material=spring_steel,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.10, 0.08)),
        mass=0.38,
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.015)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.019, length=0.078),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="shade_barrel",
    )
    shade.visual(
        Cylinder(radius=0.016, length=0.052),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="neck_collar",
    )
    shade.visual(
        mesh_from_geometry(_conical_shade_shell(), "conical_shade"),
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material=powder_black,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.041, length=0.008),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="shade_neck_ring",
    )
    shade.visual(
        Cylinder(radius=0.014, length=0.048),
        origin=Origin(xyz=(0.073, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="bulb_socket",
    )
    shade.visual(
        Sphere(radius=0.025),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=warm_glass,
        name="bulb",
    )
    shade.visual(
        Cylinder(radius=0.005, length=0.026),
        origin=Origin(xyz=(0.150, 0.0, 0.089), rpy=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="finger_stem",
    )
    shade.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.150, 0.0, 0.111)),
        material=dark_rubber,
        name="finger_knob",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.250, 0.220, 0.220)),
        mass=0.34,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z), rpy=(0.0, -0.80, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.65, upper=0.85),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 1.05, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.6, lower=-1.05, upper=1.10),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.72, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.85, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")
    shoulder = object_model.get_articulation("base_to_lower_arm")
    shade_pivot = object_model.get_articulation("upper_arm_to_shade")

    ctx.expect_contact(
        base,
        lower_arm,
        elem_a="shoulder_yoke_1",
        elem_b="shoulder_barrel",
        contact_tol=0.001,
        name="shoulder barrel is captured by the yoke",
    )
    ctx.expect_contact(
        lower_arm,
        upper_arm,
        elem_a="elbow_yoke_1",
        elem_b="elbow_barrel",
        contact_tol=0.001,
        name="elbow barrel is captured by the yoke",
    )
    ctx.expect_origin_gap(
        shade,
        base,
        axis="x",
        min_gap=0.45,
        name="lamp head reaches out over the desk",
    )
    ctx.expect_origin_gap(
        shade,
        base,
        axis="z",
        min_gap=0.12,
        name="articulated head is held above the weighted base",
    )

    rest_upper = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder: 0.45}):
        raised_upper = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder pivot raises the arm stack",
        rest_upper is not None and raised_upper is not None and raised_upper[2] > rest_upper[2] + 0.07,
        details=f"rest={rest_upper}, raised={raised_upper}",
    )

    with ctx.pose({shade_pivot: -0.55}):
        low_box = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_pivot: 0.55}):
        high_box = ctx.part_element_world_aabb(shade, elem="shade_shell")
    low_center_z = (low_box[0][2] + low_box[1][2]) / 2.0 if low_box is not None else None
    high_center_z = (high_box[0][2] + high_box[1][2]) / 2.0 if high_box is not None else None
    ctx.check(
        "shade yoke pivots the conical head",
        low_center_z is not None and high_center_z is not None and high_center_z > low_center_z + 0.06,
        details=f"low_center_z={low_center_z}, high_center_z={high_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
