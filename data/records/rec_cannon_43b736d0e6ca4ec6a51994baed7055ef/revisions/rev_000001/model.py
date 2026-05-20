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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _carriage_cheek_mesh(name: str):
    cheek_profile = [
        (-1.02, 0.00),
        (0.98, 0.00),
        (0.98, 0.18),
        (0.68, 0.26),
        (0.26, 0.54),
        (0.14, 0.54),
        (0.14, 0.43),
        (-0.10, 0.43),
        (-0.10, 0.54),
        (-0.44, 0.54),
        (-0.86, 0.40),
        (-1.02, 0.24),
    ]
    cheek_geom = ExtrudeGeometry(cheek_profile, 0.10, center=True).rotate_x(math.pi / 2.0)
    return _mesh(name, cheek_geom)


def _quoin_mesh(name: str):
    wedge_profile = [
        (-0.22, 0.00),
        (0.22, 0.00),
        (0.22, 0.024),
        (-0.22, 0.092),
    ]
    wedge_geom = ExtrudeGeometry(wedge_profile, 0.26, center=True).rotate_x(math.pi / 2.0)
    return _mesh(name, wedge_geom)


def _barrel_mesh(name: str):
    barrel_profile = [
        (0.00, -0.64),
        (0.06, -0.62),
        (0.10, -0.58),
        (0.11, -0.53),
        (0.075, -0.49),
        (0.14, -0.45),
        (0.26, -0.39),
        (0.275, -0.24),
        (0.275, -0.06),
        (0.265, 0.14),
        (0.245, 0.52),
        (0.225, 1.04),
        (0.212, 1.36),
        (0.224, 1.52),
        (0.246, 1.64),
        (0.228, 1.75),
        (0.00, 1.79),
    ]
    barrel_geom = LatheGeometry(barrel_profile, segments=88).rotate_y(math.pi / 2.0)
    trunnion_geom = (
        LatheGeometry([(0.0, -0.43), (0.065, -0.43), (0.065, 0.43), (0.0, 0.43)], segments=40)
        .rotate_x(math.pi / 2.0)
    )
    barrel_geom.merge(trunnion_geom)
    return _mesh(name, barrel_geom)


def _wheel_tire_mesh(name: str):
    outer_profile = [
        (0.245, -0.06),
        (0.245, 0.06),
    ]
    inner_profile = [
        (0.225, -0.06),
        (0.225, 0.06),
    ]
    tire_geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
    ).rotate_x(math.pi / 2.0)
    return _mesh(name, tire_geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civil_war_garrison_siege_cannon")

    timber = model.material("timber", rgba=(0.56, 0.39, 0.22, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.43, 0.29, 0.16, 1.0))
    iron = model.material("iron", rgba=(0.22, 0.23, 0.25, 1.0))
    worn_iron = model.material("worn_iron", rgba=(0.34, 0.34, 0.36, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.19, 0.21, 0.23, 1.0))

    cheek_mesh = _carriage_cheek_mesh("carriage_cheek")
    barrel_mesh = _barrel_mesh("siege_barrel")
    quoin_mesh = _quoin_mesh("quoin_wedge")
    wheel_tire_mesh = _wheel_tire_mesh("truck_wheel_tire")

    carriage = model.part("carriage")
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, 0.37, 0.28)),
        material=dark_timber,
        name="left_cheek",
    )
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, -0.37, 0.28)),
        material=dark_timber,
        name="right_cheek",
    )
    carriage.visual(
        Box((1.68, 0.66, 0.08)),
        origin=Origin(xyz=(-0.18, 0.0, 0.34)),
        material=timber,
        name="bed_plank",
    )
    carriage.visual(
        Box((0.30, 0.74, 0.14)),
        origin=Origin(xyz=(0.76, 0.0, 0.35)),
        material=timber,
        name="front_bolster",
    )
    carriage.visual(
        Box((0.30, 0.74, 0.14)),
        origin=Origin(xyz=(-0.76, 0.0, 0.35)),
        material=timber,
        name="rear_bolster",
    )
    carriage.visual(
        Box((0.24, 0.84, 0.12)),
        origin=Origin(xyz=(0.74, 0.0, 0.37)),
        material=timber,
        name="front_transom",
    )
    carriage.visual(
        Box((0.24, 0.84, 0.12)),
        origin=Origin(xyz=(-0.86, 0.0, 0.39)),
        material=timber,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.34, 1.08, 0.12)),
        origin=Origin(xyz=(0.76, 0.0, 0.22)),
        material=dark_timber,
        name="front_axle_beam",
    )
    carriage.visual(
        Box((0.34, 1.08, 0.12)),
        origin=Origin(xyz=(-0.76, 0.0, 0.22)),
        material=dark_timber,
        name="rear_axle_beam",
    )
    for axle_x, axle_prefix in ((0.76, "front"), (-0.76, "rear")):
        for side_y, side_name in ((0.58, "left"), (-0.58, "right")):
            carriage.visual(
                Cylinder(radius=0.035, length=0.08),
                origin=Origin(xyz=(axle_x, side_y, 0.23), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=iron,
                name=f"{axle_prefix}_{side_name}_axle_stub",
            )
    carriage.visual(
        Box((0.20, 0.10, 0.05)),
        origin=Origin(xyz=(0.02, 0.37, 0.690)),
        material=iron,
        name="left_trunnion_saddle",
    )
    carriage.visual(
        Box((0.20, 0.10, 0.05)),
        origin=Origin(xyz=(0.02, -0.37, 0.690)),
        material=iron,
        name="right_trunnion_saddle",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((2.10, 1.10, 0.84)),
        mass=620.0,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        barrel_mesh,
        material=gunmetal,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.288, length=0.06),
        origin=Origin(xyz=(-0.30, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_iron,
        name="base_ring",
    )
    barrel.visual(
        Cylinder(radius=0.248, length=0.05),
        origin=Origin(xyz=(1.61, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_iron,
        name="muzzle_ring",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.28, length=2.43),
        mass=1680.0,
        origin=Origin(xyz=(0.56, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    quoin = model.part("quoin")
    quoin.visual(
        quoin_mesh,
        material=timber,
        name="quoin_body",
    )
    quoin.inertial = Inertial.from_geometry(
        Box((0.44, 0.26, 0.10)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    def add_wheel(part_name: str, joint_name: str, axle_x: float, axle_y: float, stub_name: str) -> None:
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.225, length=0.12),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=timber,
            name="wheel_body",
        )
        wheel.visual(
            wheel_tire_mesh,
            material=iron,
            name="iron_tire",
        )
        wheel.visual(
            Cylinder(radius=0.055, length=0.18),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_iron,
            name="hub_shell",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.245, length=0.12),
            mass=58.0,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(axle_x, axle_y, 0.23)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=280.0, velocity=18.0),
            meta={"axle_stub": stub_name},
        )

    add_wheel("front_left_wheel", "front_left_wheel_spin", 0.76, 0.71, "front_left_axle_stub")
    add_wheel("front_right_wheel", "front_right_wheel_spin", 0.76, -0.71, "front_right_axle_stub")
    add_wheel("rear_left_wheel", "rear_left_wheel_spin", -0.76, 0.71, "rear_left_axle_stub")
    add_wheel("rear_right_wheel", "rear_right_wheel_spin", -0.76, -0.71, "rear_right_axle_stub")

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.02, 0.0, 0.78)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.55,
            lower=-0.10,
            upper=0.22,
        ),
    )
    model.articulation(
        "carriage_to_quoin",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=quoin,
        origin=Origin(xyz=(-0.38, 0.0, 0.38)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.12,
            lower=0.0,
            upper=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    quoin = object_model.get_part("quoin")
    barrel_joint = object_model.get_articulation("carriage_to_barrel")
    quoin_joint = object_model.get_articulation("carriage_to_quoin")

    ctx.expect_gap(
        quoin,
        carriage,
        axis="z",
        positive_elem="quoin_body",
        negative_elem="bed_plank",
        min_gap=0.0,
        max_gap=0.004,
        name="quoin rests on the carriage bed",
    )
    ctx.expect_within(
        quoin,
        carriage,
        axes="y",
        inner_elem="quoin_body",
        outer_elem="bed_plank",
        margin=0.02,
        name="quoin stays between the cheeks",
    )

    for wheel_name, stub_name, sign in (
        ("front_left_wheel", "front_left_axle_stub", 1.0),
        ("rear_left_wheel", "rear_left_axle_stub", 1.0),
        ("front_right_wheel", "front_right_axle_stub", -1.0),
        ("rear_right_wheel", "rear_right_axle_stub", -1.0),
    ):
        wheel = object_model.get_part(wheel_name)
        if sign > 0.0:
            ctx.expect_gap(
                wheel,
                carriage,
                axis="y",
                positive_elem="hub_shell",
                negative_elem=stub_name,
                min_gap=0.0,
                max_gap=0.002,
                name=f"{wheel_name} bears on its fixed axle stub",
            )
        else:
            ctx.expect_gap(
                carriage,
                wheel,
                axis="y",
                positive_elem=stub_name,
                negative_elem="hub_shell",
                min_gap=0.0,
                max_gap=0.002,
                name=f"{wheel_name} bears on its fixed axle stub",
            )

    rest_barrel = ctx.part_world_aabb(barrel)
    with ctx.pose({barrel_joint: barrel_joint.motion_limits.upper}):
        raised_barrel = ctx.part_world_aabb(barrel)
    ctx.check(
        "barrel elevates upward at the trunnions",
        rest_barrel is not None
        and raised_barrel is not None
        and raised_barrel[1][2] > rest_barrel[1][2] + 0.14,
        details=f"rest={rest_barrel}, raised={raised_barrel}",
    )

    rest_quoin = ctx.part_world_position(quoin)
    with ctx.pose({quoin_joint: quoin_joint.motion_limits.upper}):
        advanced_quoin = ctx.part_world_position(quoin)
        ctx.expect_gap(
            quoin,
            carriage,
            axis="z",
            positive_elem="quoin_body",
            negative_elem="bed_plank",
            min_gap=0.0,
            max_gap=0.004,
            name="quoin stays seated on the bed when advanced",
        )
    ctx.check(
        "quoin slides forward beneath the breech",
        rest_quoin is not None
        and advanced_quoin is not None
        and advanced_quoin[0] > rest_quoin[0] + 0.16,
        details=f"rest={rest_quoin}, advanced={advanced_quoin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
