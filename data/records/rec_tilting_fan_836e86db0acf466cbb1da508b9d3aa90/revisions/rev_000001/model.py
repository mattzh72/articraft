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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    plastic_dark = model.material("plastic_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    plastic_light = model.material("plastic_light", rgba=(0.78, 0.80, 0.82, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.74, 0.76, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=plastic_dark,
        name="base_foot",
    )
    stand.visual(
        Cylinder(radius=0.072, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=plastic_light,
        name="base_cap",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.255),
        origin=Origin(xyz=(-0.045, 0.0, 0.163)),
        material=metal,
        name="support_column",
    )
    stand.visual(
        Box((0.14, 0.162, 0.03)),
        origin=Origin(xyz=(0.010, 0.0, 0.248)),
        material=plastic_light,
        name="yoke_bridge",
    )
    for side, y in (("left", 0.075), ("right", -0.075)):
        stand.visual(
            Box((0.072, 0.012, 0.165)),
            origin=Origin(xyz=(0.050, y, 0.345)),
            material=plastic_light,
            name=f"{side}_bracket",
        )
    stand.inertial = Inertial.from_geometry(
        Box((0.29, 0.16, 0.43)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
    )

    head = model.part("head")
    outer_guard_ring = mesh_from_geometry(
        TorusGeometry(radius=0.118, tube=0.004, radial_segments=14, tubular_segments=56).rotate_y(
            math.pi / 2.0
        ),
        "outer_guard_ring",
    )
    middle_guard_ring = mesh_from_geometry(
        TorusGeometry(radius=0.100, tube=0.003, radial_segments=14, tubular_segments=56).rotate_y(
            math.pi / 2.0
        ),
        "middle_guard_ring",
    )
    inner_guard_ring = mesh_from_geometry(
        TorusGeometry(radius=0.082, tube=0.003, radial_segments=14, tubular_segments=56).rotate_y(
            math.pi / 2.0
        ),
        "inner_guard_ring",
    )
    head.visual(
        Cylinder(radius=0.060, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_light,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.040),
        origin=Origin(xyz=(-0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_dark,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_dark,
        name="front_axle_cap",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.0, 0.059, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.0, -0.059, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_trunnion",
    )
    head.visual(
        outer_guard_ring,
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
        material=metal,
        name="guard_outer_ring",
    )
    head.visual(
        middle_guard_ring,
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
        material=metal,
        name="guard_middle_ring",
    )
    head.visual(
        inner_guard_ring,
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
        material=metal,
        name="guard_inner_ring",
    )
    for index, angle in enumerate(
        (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi, 4.0 * math.pi / 3.0, 5.0 * math.pi / 3.0),
        start=1,
    ):
        y_inner = 0.082 * math.cos(angle)
        z_inner = 0.082 * math.sin(angle)
        y_outer = 0.118 * math.cos(angle)
        z_outer = 0.118 * math.sin(angle)
        head.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [(0.118, y_inner, z_inner), (0.118, y_outer, z_outer)],
                    radius=0.0023,
                    samples_per_segment=2,
                    radial_segments=12,
                    cap_ends=True,
                    up_hint=(1.0, 0.0, 0.0),
                ),
                f"guard_front_spoke_{index}",
            ),
            material=metal,
            name=f"guard_front_spoke_{index}",
        )
    for index, angle in enumerate((math.pi / 3.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0, 5.0 * math.pi / 3.0), start=1):
        y_start = 0.060 * math.cos(angle)
        z_start = 0.060 * math.sin(angle)
        y_mid = 0.082 * math.cos(angle)
        z_mid = 0.082 * math.sin(angle)
        y_end = 0.118 * math.cos(angle)
        z_end = 0.118 * math.sin(angle)
        head.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.036, y_start, z_start),
                        (0.060, y_mid, z_mid),
                        (0.092, y_end * 0.95, z_end * 0.95),
                        (0.118, y_end, z_end),
                    ],
                    radius=0.0032,
                    samples_per_segment=10,
                    radial_segments=14,
                    cap_ends=True,
                    up_hint=(1.0, 0.0, 0.0),
                ),
                f"guard_support_rib_{index}",
            ),
            material=metal,
            name=f"guard_support_rib_{index}",
        )
    head.inertial = Inertial.from_geometry(
        Box((0.16, 0.14, 0.14)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub",
    )
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        rotor.visual(
            Box((0.010, 0.160, 0.050)),
            origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(angle, 0.18, 0.0)),
            material=plastic_dark,
            name=f"blade_{idx}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.040),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "stand_to_head",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.085, 0.0, 0.345)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=math.radians(-20.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.079, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("stand_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    stand_pos = ctx.part_world_position(stand)
    rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "rotor axis is forward of base center",
        stand_pos is not None and rotor_pos is not None and rotor_pos[0] > stand_pos[0] + 0.05,
        details=f"stand={stand_pos}, rotor={rotor_pos}",
    )
    ctx.expect_gap(
        head,
        stand,
        axis="z",
        min_gap=0.18,
        negative_elem="base_cap",
        name="fan head sits clearly above the base",
    )

    front_rest = ctx.part_element_world_aabb(head, elem="front_axle_cap")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        front_tilted = ctx.part_element_world_aabb(head, elem="front_axle_cap")
        ctx.check(
            "positive tilt raises the fan nose",
            front_rest is not None
            and front_tilted is not None
            and front_tilted[1][2] > front_rest[1][2] + 0.02,
            details=f"rest={front_rest}, tilted={front_tilted}",
        )

    with ctx.pose({spin: 1.1}):
        ctx.expect_contact(
            rotor,
            head,
            elem_a="hub",
            elem_b="front_axle_cap",
            contact_tol=0.02,
            name="rotor hub remains mounted at the axle during spin",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
