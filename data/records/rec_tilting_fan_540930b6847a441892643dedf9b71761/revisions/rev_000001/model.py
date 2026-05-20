from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    painted_dark = model.material("painted_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    blade_black = model.material("blade_black", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.73, 0.76, 1.0))

    base = model.part("base")

    base_shell = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.23, 0.17, 0.042), 0.028),
        "base_shell",
    )
    base.visual(
        base_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=painted_dark,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.33),
        origin=Origin(xyz=(0.0, 0.0, 0.193)),
        material=warm_gray,
        name="mast_column",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=painted_dark,
        name="mast_collar",
    )
    base.visual(
        Box((0.050, 0.168, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.412)),
        material=warm_gray,
        name="yoke_body",
    )

    for side, sign in (("left", 1.0), ("right", -1.0)):
        base.visual(
            Box((0.018, 0.016, 0.052)),
            origin=Origin(xyz=(0.010, sign * 0.076, 0.436)),
            material=warm_gray,
            name=f"{side}_yoke_post",
        )
        base.visual(
            Box((0.040, 0.016, 0.018)),
            origin=Origin(xyz=(0.024, sign * 0.089, 0.429)),
            material=warm_gray,
            name=f"{side}_yoke_arm",
        )
        base.visual(
            Box((0.018, 0.024, 0.082)),
            origin=Origin(xyz=(0.040, sign * 0.104, 0.462)),
            material=warm_gray,
            name=f"{side}_bracket",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.23, 0.18, 0.49)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
    )

    head = model.part("head")

    head.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=painted_dark,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.052, length=0.092),
        origin=Origin(xyz=(0.051, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_dark,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.038, length=0.040),
        origin=Origin(xyz=(0.099, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_dark,
        name="motor_neck",
    )
    head.visual(
        Cylinder(radius=0.048, length=0.020),
        origin=Origin(xyz=(0.119, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.008, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_axle",
    )

    for side, sign in (("left", 1.0), ("right", -1.0)):
        head.visual(
            Cylinder(radius=0.017, length=0.040),
            origin=Origin(xyz=(0.0, sign * 0.072, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_gray,
            name=f"{side}_pivot_cap",
        )

    guard_geom = TorusGeometry(radius=0.118, tube=0.0036).rotate_y(math.pi / 2.0).translate(0.104, 0.0, 0.0)
    guard_geom.merge(
        TorusGeometry(radius=0.118, tube=0.0036).rotate_y(math.pi / 2.0).translate(0.172, 0.0, 0.0)
    )
    guard_geom.merge(
        TorusGeometry(radius=0.036, tube=0.0032).rotate_y(math.pi / 2.0).translate(0.172, 0.0, 0.0)
    )

    for angle in [i * math.pi / 4.0 for i in range(8)]:
        cy = math.cos(angle)
        sz = math.sin(angle)
        outer_r = 0.118
        guard_geom.merge(
            wire_from_points(
                [
                    (0.104, outer_r * cy, outer_r * sz),
                    (0.172, outer_r * cy, outer_r * sz),
                ],
                radius=0.0026,
                cap_ends=True,
            )
        )

    for angle in [i * math.pi / 3.0 for i in range(6)]:
        cy = math.cos(angle)
        sz = math.sin(angle)
        inner_r = 0.036
        outer_r = 0.118
        support_r = 0.046
        guard_geom.merge(
            wire_from_points(
                [
                    (0.172, inner_r * cy, inner_r * sz),
                    (0.172, outer_r * cy, outer_r * sz),
                ],
                radius=0.0024,
                cap_ends=True,
            )
        )
        guard_geom.merge(
            wire_from_points(
                [
                    (0.124, support_r * cy, support_r * sz),
                    (0.104, outer_r * cy, outer_r * sz),
                ],
                radius=0.0026,
                cap_ends=True,
            )
        )

    head.visual(
        mesh_from_geometry(guard_geom, "guard"),
        material=steel,
        name="guard",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.23, 0.25, 0.25)),
        mass=1.1,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="rotor_hub",
    )
    rotor.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="rotor_nose",
    )

    blade_geom = BoxGeometry((0.008, 0.032, 0.108)).translate(0.0, 0.0, 0.054)
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        rotor.visual(
            mesh_from_geometry(blade_geom.copy().rotate_x(angle), f"blade_{index}"),
            origin=Origin(rpy=(0.0, 0.0, 0.24)),
            material=blade_black,
            name=f"blade_{index}",
        )

    rotor.inertial = Inertial.from_geometry(Cylinder(radius=0.065, length=0.040), mass=0.18)

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.040, 0.0, 0.462)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.4,
            lower=math.radians(-28.0),
            upper=math.radians(42.0),
        ),
    )

    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")

    ctx.expect_gap(
        head,
        base,
        axis="z",
        positive_elem="guard",
        negative_elem="base_shell",
        min_gap=0.28,
        name="fan head sits clearly above the base",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        margin=0.01,
        name="rotor stays inside the guarded head footprint",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        raised_pos = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            head,
            axes="yz",
            margin=0.01,
            name="rotor stays inside the guard when tilted up",
        )

    ctx.check(
        "positive tilt raises the fan hub",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.035,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
