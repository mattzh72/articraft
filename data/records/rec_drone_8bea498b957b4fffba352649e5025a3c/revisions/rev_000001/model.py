from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

PLATE_RADIUS = 0.160
PLATE_THICKNESS = 0.012
ARM_HEIGHT = 0.018
ARM_LENGTH = 0.300
ARM_WIDTH = 0.034
PAD_LENGTH = 0.070
PAD_WIDTH = 0.054
PAD_CENTER_X = 0.010
BEAM_CENTER_X = 0.180
MOTOR_CENTER_X = 0.340
MOTOR_RADIUS = 0.026
MOTOR_CAP_HEIGHT = 0.006
MOTOR_HEIGHT = 0.028
PROP_HUB_RADIUS = 0.016
PROP_HUB_HEIGHT = 0.010
PROP_TIP_RADIUS = 0.175
PROP_BLADE_Z = 0.006


def _hexagon_profile(radius: float) -> list[tuple[float, float]]:
    return [
        (radius * cos(index * pi / 3.0), radius * sin(index * pi / 3.0))
        for index in range(6)
    ]


def _blade_section(
    x_pos: float,
    chord: float,
    thickness: float,
    pitch: float,
) -> list[tuple[float, float, float]]:
    section = rounded_rect_profile(
        chord,
        thickness,
        radius=min(thickness * 0.45, chord * 0.18),
        corner_segments=4,
    )
    c = cos(pitch)
    s = sin(pitch)
    return [
        (x_pos, y * c - z * s, y * s + z * c)
        for y, z in section
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hexacopter")

    carbon_black = model.material("carbon_black", rgba=(0.12, 0.12, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    prop_gray = model.material("prop_gray", rgba=(0.18, 0.18, 0.19, 1.0))
    fastener_silver = model.material("fastener_silver", rgba=(0.74, 0.76, 0.79, 1.0))

    center_plate = model.part("center_plate")
    plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_hexagon_profile(PLATE_RADIUS), PLATE_THICKNESS),
        "hexacopter_center_plate",
    )
    center_plate.visual(
        plate_mesh,
        material=carbon_black,
        name="center_plate_shell",
    )
    center_plate.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, PLATE_THICKNESS)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS / 2.0)),
    )

    prop_blade_mesh = mesh_from_geometry(
        section_loft(
            [
                _blade_section(0.012, 0.032, 0.0050, 0.42),
                _blade_section(0.090, 0.024, 0.0038, 0.22),
                _blade_section(0.175, 0.014, 0.0024, 0.08),
            ]
        ),
        "hexacopter_prop_blade",
    )

    for index in range(6):
        angle = index * pi / 3.0
        vx = PLATE_RADIUS * cos(angle)
        vy = PLATE_RADIUS * sin(angle)

        arm = model.part(f"arm_{index}")
        arm.visual(
            Box((PAD_LENGTH, PAD_WIDTH, ARM_HEIGHT)),
            origin=Origin(xyz=(PAD_CENTER_X, 0.0, ARM_HEIGHT / 2.0)),
            material=carbon_black,
            name="mount_pad",
        )
        arm.visual(
            Box((ARM_LENGTH, ARM_WIDTH, ARM_HEIGHT)),
            origin=Origin(xyz=(BEAM_CENTER_X, 0.0, ARM_HEIGHT / 2.0)),
            material=carbon_black,
            name="arm_beam",
        )
        arm.visual(
            Cylinder(radius=0.022, length=MOTOR_CAP_HEIGHT),
            origin=Origin(
                xyz=(MOTOR_CENTER_X, 0.0, ARM_HEIGHT + MOTOR_CAP_HEIGHT / 2.0)
            ),
            material=matte_black,
            name="motor_cap",
        )
        arm.visual(
            Cylinder(radius=MOTOR_RADIUS, length=MOTOR_HEIGHT),
            origin=Origin(
                xyz=(
                    MOTOR_CENTER_X,
                    0.0,
                    ARM_HEIGHT + MOTOR_CAP_HEIGHT + MOTOR_HEIGHT / 2.0,
                )
            ),
            material=motor_gray,
            name="motor_shell",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.40, 0.07, 0.060)),
            mass=0.28,
            origin=Origin(xyz=(0.19, 0.0, 0.030)),
        )

        model.articulation(
            f"center_to_arm_{index}",
            ArticulationType.FIXED,
            parent=center_plate,
            child=arm,
            origin=Origin(
                xyz=(vx, vy, PLATE_THICKNESS),
                rpy=(0.0, 0.0, angle),
            ),
        )

        propeller = model.part(f"prop_{index}")
        propeller.visual(
            Cylinder(radius=PROP_HUB_RADIUS, length=PROP_HUB_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, PROP_HUB_HEIGHT / 2.0)),
            material=prop_gray,
            name="hub_shell",
        )
        propeller.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, PROP_HUB_HEIGHT + 0.003)),
            material=fastener_silver,
            name="hub_cap",
        )
        propeller.visual(
            prop_blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, PROP_BLADE_Z)),
            material=prop_gray,
            name="blade_a",
        )
        propeller.visual(
            prop_blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, PROP_BLADE_Z), rpy=(0.0, 0.0, pi)),
            material=prop_gray,
            name="blade_b",
        )
        propeller.inertial = Inertial.from_geometry(
            Box((PROP_TIP_RADIUS * 2.0, 0.040, 0.020)),
            mass=0.050,
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
        )

        model.articulation(
            f"arm_{index}_to_prop_{index}",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=propeller,
            origin=Origin(
                xyz=(
                    MOTOR_CENTER_X,
                    0.0,
                    ARM_HEIGHT + MOTOR_CAP_HEIGHT + MOTOR_HEIGHT,
                )
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=220.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    center_plate = object_model.get_part("center_plate")
    expected_prop_radius = PLATE_RADIUS + MOTOR_CENTER_X

    for index in range(6):
        arm = object_model.get_part(f"arm_{index}")
        prop = object_model.get_part(f"prop_{index}")
        spin_joint = object_model.get_articulation(f"arm_{index}_to_prop_{index}")

        ctx.expect_contact(
            arm,
            center_plate,
            elem_a="mount_pad",
            elem_b="center_plate_shell",
            contact_tol=0.0005,
            name=f"arm {index} pad contacts the center plate",
        )
        ctx.expect_contact(
            prop,
            arm,
            elem_a="hub_shell",
            elem_b="motor_shell",
            contact_tol=0.0005,
            name=f"prop {index} hub contacts its motor",
        )

        limits = spin_joint.motion_limits
        ctx.check(
            f"prop {index} uses a vertical continuous spin joint",
            spin_joint.joint_type == ArticulationType.CONTINUOUS
            and spin_joint.axis == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=(
                f"type={spin_joint.joint_type}, axis={spin_joint.axis}, "
                f"limits={limits}"
            ),
        )

        prop_pos = ctx.part_world_position(prop)
        radial_distance = None
        if prop_pos is not None:
            radial_distance = sqrt(prop_pos[0] ** 2 + prop_pos[1] ** 2)
        ctx.check(
            f"prop {index} sits at the common rotor circle",
            radial_distance is not None
            and abs(radial_distance - expected_prop_radius) <= 0.003,
            details=f"radial_distance={radial_distance}, expected={expected_prop_radius}",
        )

    sample_joint = object_model.get_articulation("arm_0_to_prop_0")
    sample_prop = object_model.get_part("prop_0")
    sample_arm = object_model.get_part("arm_0")
    rest_pos = ctx.part_world_position(sample_prop)
    with ctx.pose({sample_joint: pi / 2.0}):
        spun_pos = ctx.part_world_position(sample_prop)
        ctx.expect_contact(
            sample_prop,
            sample_arm,
            elem_a="hub_shell",
            elem_b="motor_shell",
            contact_tol=0.0005,
            name="spun prop hub stays seated on its motor",
        )
    ctx.check(
        "spinning a propeller does not move its hub off axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) <= 1e-6
        and abs(rest_pos[1] - spun_pos[1]) <= 1e-6
        and abs(rest_pos[2] - spun_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
