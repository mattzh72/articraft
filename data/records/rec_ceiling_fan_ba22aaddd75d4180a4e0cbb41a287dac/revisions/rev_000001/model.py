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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


HUB_RADIUS = 0.26
ROTOR_ATTACH_Z = -0.18
ARM_LENGTH = 0.80
ARM_ROOT_LENGTH = 0.18
ARM_BEAM_LENGTH = 0.46
ARM_CLAMP_LENGTH = 0.16
BLADE_LENGTH = 2.30
BLADE_WIDTH = 0.30
BLADE_THICKNESS = 0.022


def _blade_panel_mesh():
    blade_profile = rounded_rect_profile(BLADE_LENGTH, BLADE_WIDTH, 0.055, corner_segments=10)
    blade_geom = ExtrudeGeometry(blade_profile, BLADE_THICKNESS, cap=True, center=True)
    return mesh_from_geometry(blade_geom, "hvls_blade_panel")


def _yz_rounded_rect_section(
    *,
    x_pos: float,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
):
    return [
        (x_pos, y_pos, z_center + z_pos)
        for y_pos, z_pos in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _arm_root_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _yz_rounded_rect_section(x_pos=0.0, width=0.16, height=0.10, radius=0.022),
                _yz_rounded_rect_section(x_pos=0.09, width=0.15, height=0.09, radius=0.020),
                _yz_rounded_rect_section(x_pos=ARM_ROOT_LENGTH, width=0.12, height=0.07, radius=0.017),
            ]
        ),
        "hvls_arm_root",
    )


def _arm_beam_mesh():
    x0 = ARM_ROOT_LENGTH
    x1 = ARM_ROOT_LENGTH + (0.55 * ARM_BEAM_LENGTH)
    x2 = ARM_ROOT_LENGTH + ARM_BEAM_LENGTH
    return mesh_from_geometry(
        section_loft(
            [
                _yz_rounded_rect_section(x_pos=x0, width=0.11, height=0.062, radius=0.016, z_center=0.018),
                _yz_rounded_rect_section(x_pos=x1, width=0.092, height=0.052, radius=0.014, z_center=0.018),
                _yz_rounded_rect_section(x_pos=x2, width=0.074, height=0.044, radius=0.012, z_center=0.018),
            ]
        ),
        "hvls_arm_beam",
    )


def _arm_clamp_mesh():
    x0 = ARM_ROOT_LENGTH + ARM_BEAM_LENGTH
    x1 = ARM_LENGTH
    return mesh_from_geometry(
        section_loft(
            [
                _yz_rounded_rect_section(x_pos=x0, width=0.18, height=0.08, radius=0.018),
                _yz_rounded_rect_section(x_pos=x0 + 0.07, width=0.17, height=0.075, radius=0.017),
                _yz_rounded_rect_section(x_pos=x1, width=0.14, height=0.060, radius=0.015),
            ]
        ),
        "hvls_arm_clamp",
    )


def _blade_cuff_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _yz_rounded_rect_section(x_pos=-0.004, width=0.22, height=0.052, radius=0.014),
                _yz_rounded_rect_section(x_pos=0.136, width=0.20, height=0.046, radius=0.013),
                _yz_rounded_rect_section(x_pos=0.316, width=0.15, height=0.034, radius=0.011),
            ]
        ),
        "hvls_blade_cuff",
    )


def _add_arm_and_blade(
    model: ArticulatedObject,
    rotor,
    *,
    index: int,
    yaw: float,
    arm_material,
    blade_material,
    blade_mesh,
    arm_root_mesh,
    arm_beam_mesh,
    arm_clamp_mesh,
    blade_cuff_mesh,
) -> None:
    arm = model.part(f"arm_{index}")
    arm.visual(
        arm_root_mesh,
        material=arm_material,
        name="arm_root_block",
    )
    arm.visual(
        arm_beam_mesh,
        material=arm_material,
        name="arm_beam",
    )
    arm.visual(
        Box((ARM_CLAMP_LENGTH, 0.14, 0.056)),
        origin=Origin(xyz=(ARM_ROOT_LENGTH + ARM_BEAM_LENGTH + (0.5 * ARM_CLAMP_LENGTH), 0.0, 0.0)),
        material=arm_material,
        name="arm_tip_clamp",
    )
    arm.inertial = Inertial.from_geometry(
        Box((ARM_LENGTH, 0.18, 0.10)),
        mass=9.0,
        origin=Origin(xyz=(0.40, 0.0, 0.01)),
    )

    model.articulation(
        f"rotor_to_arm_{index}",
        ArticulationType.FIXED,
        parent=rotor,
        child=arm,
        origin=Origin(
            xyz=(HUB_RADIUS * math.cos(yaw), HUB_RADIUS * math.sin(yaw), ROTOR_ATTACH_Z),
            rpy=(0.0, 0.0, yaw),
        ),
    )

    blade = model.part(f"blade_{index}")
    blade.visual(
        Box((ARM_CLAMP_LENGTH, 0.18, 0.040)),
        origin=Origin(xyz=(0.5 * ARM_CLAMP_LENGTH, 0.0, -0.048)),
        material=arm_material,
        name="blade_root_cuff",
    )
    blade.visual(
        blade_mesh,
        origin=Origin(xyz=(0.14 + (0.5 * BLADE_LENGTH), 0.0, -0.048)),
        material=blade_material,
        name="blade_panel",
    )
    blade.inertial = Inertial.from_geometry(
        Box((2.52, 0.32, 0.06)),
        mass=14.0,
        origin=Origin(xyz=(1.32, 0.0, -0.002)),
    )

    model.articulation(
        f"arm_{index}_to_blade_{index}",
        ArticulationType.FIXED,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(ARM_ROOT_LENGTH + ARM_BEAM_LENGTH, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_hvls_ceiling_fan")

    canopy_gray = model.material("canopy_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))

    blade_mesh = _blade_panel_mesh()
    arm_root_mesh = _arm_root_mesh()
    arm_beam_mesh = _arm_beam_mesh()
    arm_clamp_mesh = _arm_clamp_mesh()
    blade_cuff_mesh = _blade_cuff_mesh()

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Box((0.42, 0.20, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=steel_dark,
        name="ceiling_plate",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.17, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=canopy_gray,
        name="canopy_shell",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.04, length=0.68),
        origin=Origin(xyz=(0.0, 0.0, -0.49)),
        material=steel_dark,
        name="downrod_tube",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.17, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.92)),
        material=canopy_gray,
        name="upper_gearbox",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.21, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -1.10)),
        material=steel_dark,
        name="lower_gearbox",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.085, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -1.215)),
        material=steel_dark,
        name="bearing_collar",
    )
    ceiling_mount.inertial = Inertial.from_geometry(
        Box((0.42, 0.20, 1.26)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, -0.61)),
    )

    rotor_hub = model.part("rotor_hub")
    rotor_hub.visual(
        Cylinder(radius=0.075, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=steel_dark,
        name="rotor_neck",
    )
    rotor_hub.visual(
        Cylinder(radius=0.32, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=steel_dark,
        name="hub_flange",
    )
    rotor_hub.visual(
        Cylinder(radius=0.26, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, -0.25)),
        material=canopy_gray,
        name="hub_shell",
    )
    rotor_hub.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.41)),
        material=steel_dark,
        name="lower_cap",
    )
    rotor_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.32, length=0.45),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
    )

    model.articulation(
        "mount_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=ceiling_mount,
        child=rotor_hub,
        origin=Origin(xyz=(0.0, 0.0, -1.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.6),
    )

    for blade_index in range(3):
        _add_arm_and_blade(
            model,
            rotor_hub,
            index=blade_index + 1,
            yaw=(blade_index * math.tau) / 3.0,
            arm_material=steel_dark,
            blade_material=aluminum,
            blade_mesh=blade_mesh,
            arm_root_mesh=arm_root_mesh,
            arm_beam_mesh=arm_beam_mesh,
            arm_clamp_mesh=arm_clamp_mesh,
            blade_cuff_mesh=blade_cuff_mesh,
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

    mount = object_model.get_part("ceiling_mount")
    rotor = object_model.get_part("rotor_hub")
    spin = object_model.get_articulation("mount_to_rotor")
    blade_1 = object_model.get_part("blade_1")

    ctx.check(
        "rotor uses a vertical continuous joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_contact(
        rotor,
        mount,
        elem_a="rotor_neck",
        elem_b="bearing_collar",
        contact_tol=1e-5,
        name="rotor hangs from the bearing collar",
    )

    for index in range(1, 4):
        arm = object_model.get_part(f"arm_{index}")
        blade = object_model.get_part(f"blade_{index}")
        ctx.expect_contact(
            arm,
            rotor,
            elem_a="arm_root_block",
            elem_b="hub_shell",
            contact_tol=1e-5,
            name=f"arm {index} is mounted to the round hub",
        )
        ctx.expect_contact(
            blade,
            arm,
            elem_a="blade_root_cuff",
            elem_b="arm_tip_clamp",
            contact_tol=0.006,
            name=f"blade {index} is clamped to its arm",
        )

    rest_pos = ctx.part_world_position(blade_1)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(blade_1)

    rest_radius = None if rest_pos is None else math.hypot(rest_pos[0], rest_pos[1])
    turned_radius = None if turned_pos is None else math.hypot(turned_pos[0], turned_pos[1])
    ctx.check(
        "blade assembly rotates about the vertical shaft",
        rest_pos is not None
        and turned_pos is not None
        and rest_radius is not None
        and turned_radius is not None
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6
        and abs(rest_radius - turned_radius) < 0.01
        and abs(rest_pos[0] - turned_pos[0]) > 0.5,
        details=f"rest={rest_pos}, turned={turned_pos}, rest_radius={rest_radius}, turned_radius={turned_radius}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
