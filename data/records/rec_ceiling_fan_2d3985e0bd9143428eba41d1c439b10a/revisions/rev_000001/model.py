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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_canopy_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.024, 0.014),
            (0.024, -0.002),
            (0.024, -0.006),
            (0.040, -0.006),
            (0.056, -0.024),
            (0.078, -0.056),
            (0.084, -0.082),
        ],
        [
            (0.020, 0.014),
            (0.020, -0.002),
            (0.020, -0.006),
            (0.032, -0.006),
            (0.044, -0.022),
            (0.066, -0.052),
            (0.075, -0.076),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_ceiling_fan")

    hardware = model.material("hardware", rgba=(0.27, 0.21, 0.14, 1.0))
    iron_finish = model.material("iron_finish", rgba=(0.20, 0.18, 0.16, 1.0))
    wood = model.material("wood", rgba=(0.55, 0.37, 0.21, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_geometry(_build_canopy_mesh(), "canopy_shell"),
        material=hardware,
        name="canopy_shell",
    )
    canopy.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=hardware,
        name="mount_boss",
    )
    canopy.visual(
        Box((0.004, 0.010, 0.010)),
        origin=Origin(xyz=(0.019, 0.0, -0.002)),
        material=hardware,
        name="boss_rib_pos_x",
    )
    canopy.visual(
        Box((0.004, 0.010, 0.010)),
        origin=Origin(xyz=(-0.019, 0.0, -0.002)),
        material=hardware,
        name="boss_rib_neg_x",
    )
    canopy.visual(
        Box((0.010, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.019, -0.002)),
        material=hardware,
        name="boss_rib_pos_y",
    )
    canopy.visual(
        Box((0.010, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.019, -0.002)),
        material=hardware,
        name="boss_rib_neg_y",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.084, length=0.082),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
    )

    downrod = model.part("downrod")
    downrod.visual(
        Cylinder(radius=0.012, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, -0.063)),
        material=hardware,
        name="downrod_tube",
    )
    downrod.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=hardware,
        name="upper_coupler",
    )
    downrod.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.113)),
        material=hardware,
        name="lower_coupler",
    )
    downrod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.126),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.063)),
    )

    motor = model.part("motor_housing")
    motor.visual(
        Cylinder(radius=0.108, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=hardware,
        name="motor_barrel",
    )
    motor.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=hardware,
        name="upper_neck",
    )
    motor.visual(
        Cylinder(radius=0.064, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.179)),
        material=hardware,
        name="lower_switch_housing",
    )
    motor.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.207)),
        material=hardware,
        name="bottom_finial",
    )
    motor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.108, length=0.216),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, -0.108)),
    )

    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )
    model.articulation(
        "downrod_to_motor",
        ArticulationType.CONTINUOUS,
        parent=downrod,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, -0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )

    blade_pitch = -0.20
    blade_angles = [index * math.tau / 5.0 for index in range(5)]
    for index, angle in enumerate(blade_angles, start=1):
        iron = model.part(f"blade_iron_{index}")
        iron.visual(
            Box((0.044, 0.030, 0.012)),
            origin=Origin(xyz=(0.022, 0.0, -0.002)),
            material=iron_finish,
            name="root_hub",
        )
        iron.visual(
            Box((0.128, 0.020, 0.006)),
            origin=Origin(xyz=(0.083, 0.0, -0.006), rpy=(0.0, -0.10, 0.0)),
            material=iron_finish,
            name="iron_arm",
        )
        iron.visual(
            Box((0.072, 0.054, 0.010)),
            origin=Origin(xyz=(0.171, 0.0, -0.002)),
            material=iron_finish,
            name="blade_pad",
        )
        iron.inertial = Inertial.from_geometry(
            Box((0.210, 0.052, 0.020)),
            mass=0.35,
            origin=Origin(xyz=(0.105, 0.0, -0.005)),
        )

        blade = model.part(f"blade_{index}")
        blade.visual(
            Box((0.040, 0.056, 0.010)),
            origin=Origin(xyz=(0.020, 0.0, 0.005)),
            material=wood,
            name="blade_root",
        )
        blade.visual(
            Box((0.368, 0.132, 0.010)),
            origin=Origin(
                xyz=(0.223, 0.0, 0.005),
                rpy=(blade_pitch, 0.0, 0.0),
            ),
            material=wood,
            name="blade_panel",
        )
        blade.inertial = Inertial.from_geometry(
            Box((0.408, 0.132, 0.010)),
            mass=0.7,
            origin=Origin(xyz=(0.204, 0.0, 0.005)),
        )

        model.articulation(
            f"motor_to_blade_iron_{index}",
            ArticulationType.FIXED,
            parent=motor,
            child=iron,
            origin=Origin(
                xyz=(0.108 * math.cos(angle), 0.108 * math.sin(angle), -0.095),
                rpy=(0.0, 0.0, angle),
            ),
        )
        model.articulation(
            f"blade_iron_to_blade_{index}",
            ArticulationType.FIXED,
            parent=iron,
            child=blade,
            origin=Origin(xyz=(0.207, 0.0, 0.003)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    motor = object_model.get_part("motor_housing")
    spin = object_model.get_articulation("downrod_to_motor")

    ctx.expect_origin_distance(
        canopy,
        downrod,
        axes="xy",
        max_dist=0.001,
        name="downrod stays centered in the canopy",
    )
    ctx.expect_contact(
        motor,
        downrod,
        name="motor housing seats against the downrod coupler",
    )

    for index in range(1, 6):
        iron = object_model.get_part(f"blade_iron_{index}")
        blade = object_model.get_part(f"blade_{index}")
        ctx.expect_contact(
            iron,
            motor,
            name=f"blade iron {index} mounts to the motor housing",
        )
        ctx.expect_contact(
            blade,
            iron,
            name=f"blade {index} mounts to its blade iron",
        )

    limits = spin.motion_limits
    axis = spin.axis
    ctx.check(
        "rotor uses a continuous vertical joint",
        spin.joint_type == ArticulationType.CONTINUOUS
        and limits is not None
        and limits.lower is None
        and limits.upper is None
        and abs(axis[0]) < 1e-6
        and abs(axis[1]) < 1e-6
        and abs(abs(axis[2]) - 1.0) < 1e-6,
        details=f"type={spin.joint_type}, axis={axis}, limits={limits}",
    )

    blade_1 = object_model.get_part("blade_1")
    rest_position = ctx.part_world_position(blade_1)
    with ctx.pose({spin: math.tau / 10.0}):
        turned_position = ctx.part_world_position(blade_1)
    moved_xy = None
    moved_z = None
    if rest_position is not None and turned_position is not None:
        moved_xy = math.hypot(
            turned_position[0] - rest_position[0],
            turned_position[1] - rest_position[1],
        )
        moved_z = abs(turned_position[2] - rest_position[2])
    ctx.check(
        "continuous spin moves a blade around the vertical axis",
        moved_xy is not None and moved_z is not None and moved_xy > 0.15 and moved_z < 0.002,
        details=f"rest={rest_position}, turned={turned_position}, moved_xy={moved_xy}, moved_z={moved_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
