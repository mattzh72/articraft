from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent

PLATE_RADIUS = 0.065
PLATE_THICKNESS = 0.010
ARM_TIP_OFFSET = 0.145
ARM_ROOT_OFFSET = 0.050
ARM_LENGTH = ARM_TIP_OFFSET - ARM_ROOT_OFFSET
ARM_WIDTH = 0.024
ARM_THICKNESS = 0.012

LOWER_HOUSING_RADIUS = 0.008
LOWER_HOUSING_LENGTH = 0.018
LOWER_HOUSING_CENTER_Z = ARM_THICKNESS + LOWER_HOUSING_LENGTH / 2.0

COAXIAL_MAST_RADIUS = 0.0038
COAXIAL_MAST_BOTTOM_Z = ARM_THICKNESS + LOWER_HOUSING_LENGTH
COAXIAL_MAST_TOP_Z = 0.070
COAXIAL_MAST_LENGTH = COAXIAL_MAST_TOP_Z - COAXIAL_MAST_BOTTOM_Z
COAXIAL_MAST_CENTER_Z = COAXIAL_MAST_BOTTOM_Z + COAXIAL_MAST_LENGTH / 2.0

UPPER_HOUSING_RADIUS = 0.008
UPPER_HOUSING_LENGTH = 0.008
UPPER_HOUSING_CENTER_Z = COAXIAL_MAST_TOP_Z - UPPER_HOUSING_LENGTH / 2.0

ROTOR_HUB_RADIUS = 0.012
ROTOR_HUB_LENGTH = 0.006
LOWER_ROTOR_CENTER_Z = ARM_THICKNESS + LOWER_HOUSING_LENGTH + ROTOR_HUB_LENGTH / 2.0
UPPER_ROTOR_CENTER_Z = COAXIAL_MAST_TOP_Z + ROTOR_HUB_LENGTH / 2.0

BLADE_LENGTH = 0.060
BLADE_WIDTH = 0.013
BLADE_THICKNESS = 0.0018
BLADE_OFFSET = 0.036
LOWER_HUB_BORE_RADIUS = COAXIAL_MAST_RADIUS + 0.0012

ARM_TIPS = {
    "front": (0.0, ARM_TIP_OFFSET),
    "rear": (0.0, -ARM_TIP_OFFSET),
    "right": (ARM_TIP_OFFSET, 0.0),
    "left": (-ARM_TIP_OFFSET, 0.0),
}


def _arm_visual_origin(x: float, y: float) -> Origin:
    if abs(x) > 0.0:
        root_x = math.copysign(ARM_ROOT_OFFSET, x)
        return Origin(xyz=((x + root_x) / 2.0, 0.0, ARM_THICKNESS / 2.0))
    root_y = math.copysign(ARM_ROOT_OFFSET, y)
    return Origin(xyz=(0.0, (y + root_y) / 2.0, ARM_THICKNESS / 2.0))


def _arm_box(x: float, y: float) -> Box:
    if abs(x) > 0.0:
        return Box((ARM_LENGTH, ARM_WIDTH, ARM_THICKNESS))
    return Box((ARM_WIDTH, ARM_LENGTH, ARM_THICKNESS))


def _circle_profile(radius: float, *, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _build_lower_hub_mesh():
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(ROTOR_HUB_RADIUS),
            [_circle_profile(LOWER_HUB_BORE_RADIUS)],
            height=ROTOR_HUB_LENGTH,
            center=True,
        ),
        ASSETS.mesh_path("coaxial_lower_rotor_hub.obj"),
    )


def _build_blade_mesh():
    blade_profile = [
        (-0.006, 0.0),
        (-0.004, -0.0024),
        (0.000, -0.0044),
        (0.012, -0.0058),
        (0.030, -0.0062),
        (0.050, -0.0046),
        (0.058, -0.0024),
        (0.061, 0.0),
        (0.058, 0.0022),
        (0.050, 0.0041),
        (0.030, 0.0054),
        (0.012, 0.0048),
        (0.000, 0.0038),
        (-0.004, 0.0018),
    ]
    return mesh_from_geometry(
        ExtrudeGeometry(blade_profile, BLADE_THICKNESS, center=True),
        ASSETS.mesh_path("coaxial_prop_blade.obj"),
    )


def _add_rotor(
    model: ArticulatedObject,
    frame,
    *,
    arm_name: str,
    tier: str,
    x: float,
    y: float,
    z: float,
    axis_sign: float,
    hub_geometry,
    blade_geometry,
    hub_material,
    blade_material,
) -> None:
    rotor = model.part(f"{arm_name}_{tier}_rotor")
    rotor.visual(
        hub_geometry,
        material=hub_material,
        name="hub",
    )
    rotor.visual(
        blade_geometry,
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, 0.035, 0.10)),
        material=blade_material,
        name="blade_a",
    )
    rotor.visual(
        blade_geometry,
        origin=Origin(xyz=(-0.017, 0.0, 0.0), rpy=(0.0, -0.035, math.pi + 0.10)),
        material=blade_material,
        name="blade_b",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=ROTOR_HUB_RADIUS, length=ROTOR_HUB_LENGTH),
        mass=0.035,
    )

    model.articulation(
        f"{arm_name}_{tier}_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(x, y, z)),
        axis=(0.0, 0.0, axis_sign),
        motion_limits=MotionLimits(effort=0.3, velocity=180.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_quadrotor", assets=ASSETS)

    carbon = model.material("carbon", rgba=(0.13, 0.13, 0.14, 1.0))
    plate_mat = model.material("plate", rgba=(0.22, 0.23, 0.25, 1.0))
    motor_mat = model.material("motor", rgba=(0.56, 0.58, 0.61, 1.0))
    rotor_mat = model.material("rotor", rgba=(0.07, 0.07, 0.08, 1.0))
    lower_hub_mesh = _build_lower_hub_mesh()
    blade_mesh = _build_blade_mesh()

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=PLATE_RADIUS, length=PLATE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS / 2.0)),
        material=plate_mat,
        name="center_plate",
    )
    frame.visual(
        Cylinder(radius=0.023, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS + 0.012)),
        material=motor_mat,
        name="center_stack",
    )

    for arm_name, (x, y) in ARM_TIPS.items():
        frame.visual(
            _arm_box(x, y),
            origin=_arm_visual_origin(x, y),
            material=carbon,
            name=f"{arm_name}_arm",
        )
        frame.visual(
            Cylinder(radius=0.012, length=ARM_THICKNESS),
            origin=Origin(
                xyz=(
                    math.copysign(ARM_ROOT_OFFSET, x) if abs(x) > 0.0 else 0.0,
                    math.copysign(ARM_ROOT_OFFSET, y) if abs(y) > 0.0 else 0.0,
                    ARM_THICKNESS / 2.0,
                )
            ),
            material=carbon,
            name=f"{arm_name}_root_collar",
        )
        frame.visual(
            Cylinder(radius=LOWER_HOUSING_RADIUS, length=LOWER_HOUSING_LENGTH),
            origin=Origin(xyz=(x, y, LOWER_HOUSING_CENTER_Z)),
            material=motor_mat,
            name=f"{arm_name}_lower_axle_housing",
        )
        frame.visual(
            Cylinder(radius=COAXIAL_MAST_RADIUS, length=COAXIAL_MAST_LENGTH),
            origin=Origin(xyz=(x, y, COAXIAL_MAST_CENTER_Z)),
            material=motor_mat,
            name=f"{arm_name}_coaxial_mast",
        )
        frame.visual(
            Cylinder(radius=UPPER_HOUSING_RADIUS, length=UPPER_HOUSING_LENGTH),
            origin=Origin(xyz=(x, y, UPPER_HOUSING_CENTER_Z)),
            material=motor_mat,
            name=f"{arm_name}_upper_axle_housing",
        )

    frame.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATE_RADIUS, length=PLATE_THICKNESS),
        mass=1.45,
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS / 2.0)),
    )

    for arm_name, (x, y) in ARM_TIPS.items():
        _add_rotor(
            model,
            frame,
            arm_name=arm_name,
            tier="lower",
            x=x,
            y=y,
            z=LOWER_ROTOR_CENTER_Z,
            axis_sign=-1.0,
            hub_geometry=lower_hub_mesh,
            blade_geometry=blade_mesh,
            hub_material=motor_mat,
            blade_material=rotor_mat,
        )
        _add_rotor(
            model,
            frame,
            arm_name=arm_name,
            tier="upper",
            x=x,
            y=y,
            z=UPPER_ROTOR_CENTER_Z,
            axis_sign=1.0,
            hub_geometry=Cylinder(radius=ROTOR_HUB_RADIUS, length=ROTOR_HUB_LENGTH),
            blade_geometry=blade_mesh,
            hub_material=motor_mat,
            blade_material=rotor_mat,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    frame = object_model.get_part("frame")
    center_plate = frame.get_visual("center_plate")

    lower_rotors = {}
    upper_rotors = {}
    lower_spins = {}
    upper_spins = {}
    for arm_name in ARM_TIPS:
        lower_rotors[arm_name] = object_model.get_part(f"{arm_name}_lower_rotor")
        upper_rotors[arm_name] = object_model.get_part(f"{arm_name}_upper_rotor")
        lower_spins[arm_name] = object_model.get_articulation(f"{arm_name}_lower_spin")
        upper_spins[arm_name] = object_model.get_articulation(f"{arm_name}_upper_spin")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.check(
        "center_plate_is_round",
        isinstance(center_plate.geometry, Cylinder),
        "The quadrotor should use a round central plate.",
    )
    ctx.check(
        "eight_rotors_present",
        len(lower_rotors) == 4 and len(upper_rotors) == 4,
        "Expected four lower and four upper coaxial rotors.",
    )

    for arm_name in ARM_TIPS:
        lower_hub = lower_rotors[arm_name].get_visual("hub")
        upper_hub = upper_rotors[arm_name].get_visual("hub")
        lower_housing = frame.get_visual(f"{arm_name}_lower_axle_housing")
        upper_housing = frame.get_visual(f"{arm_name}_upper_axle_housing")
        lower_spin = lower_spins[arm_name]
        upper_spin = upper_spins[arm_name]

        ctx.check(
            f"{arm_name}_lower_joint_is_continuous",
            lower_spin.articulation_type == ArticulationType.CONTINUOUS,
            "Lower rotor should spin on a continuous revolute axle.",
        )
        ctx.check(
            f"{arm_name}_upper_joint_is_continuous",
            upper_spin.articulation_type == ArticulationType.CONTINUOUS,
            "Upper rotor should spin on a continuous revolute axle.",
        )
        ctx.check(
            f"{arm_name}_counter_rotates",
            lower_spin.axis[2] * upper_spin.axis[2] < 0.0,
            "The two coaxial rotor sets should counter-rotate.",
        )

        ctx.expect_origin_distance(
            lower_rotors[arm_name],
            frame,
            axes="xy",
            min_dist=ARM_TIP_OFFSET - 0.001,
            max_dist=ARM_TIP_OFFSET + 0.001,
            name=f"{arm_name}_arm_reaches_expected_tip_radius",
        )
        ctx.expect_contact(
            lower_rotors[arm_name],
            frame,
            elem_a=lower_hub,
            elem_b=lower_housing,
            name=f"{arm_name}_lower_rotor_seats_on_lower_axle",
        )
        ctx.expect_overlap(
            lower_rotors[arm_name],
            frame,
            axes="xy",
            min_overlap=0.014,
            elem_a=lower_hub,
            elem_b=lower_housing,
            name=f"{arm_name}_lower_rotor_is_centered_on_tip",
        )
        ctx.expect_contact(
            upper_rotors[arm_name],
            frame,
            elem_a=upper_hub,
            elem_b=upper_housing,
            name=f"{arm_name}_upper_rotor_seats_on_upper_axle",
        )
        ctx.expect_overlap(
            upper_rotors[arm_name],
            frame,
            axes="xy",
            min_overlap=0.014,
            elem_a=upper_hub,
            elem_b=upper_housing,
            name=f"{arm_name}_upper_rotor_is_centered_on_mast",
        )
        ctx.expect_origin_distance(
            upper_rotors[arm_name],
            lower_rotors[arm_name],
            axes="xy",
            max_dist=0.001,
            name=f"{arm_name}_upper_and_lower_rotors_share_one_axis",
        )
        ctx.expect_overlap(
            upper_rotors[arm_name],
            lower_rotors[arm_name],
            axes="xy",
            min_overlap=0.020,
            elem_a=upper_hub,
            elem_b=lower_hub,
            name=f"{arm_name}_rotor_hubs_are_coaxial",
        )
        ctx.expect_gap(
            upper_rotors[arm_name],
            lower_rotors[arm_name],
            axis="z",
            min_gap=0.028,
            name=f"{arm_name}_upper_rotor_clears_lower_rotor",
        )
        ctx.expect_gap(
            lower_rotors[arm_name],
            frame,
            axis="z",
            min_gap=0.020,
            positive_elem=lower_hub,
            negative_elem=center_plate,
            name=f"{arm_name}_lower_rotor_sits_above_center_plate",
        )

    spin_pose = {}
    for index, arm_name in enumerate(ARM_TIPS):
        spin_angle = (index + 1) * math.pi / 5.0
        spin_pose[lower_spins[arm_name]] = -spin_angle
        spin_pose[upper_spins[arm_name]] = spin_angle

    with ctx.pose(spin_pose):
        ctx.fail_if_isolated_parts(name="operating_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="operating_pose_no_overlap")
        for arm_name in ARM_TIPS:
            ctx.expect_contact(
                lower_rotors[arm_name],
                frame,
                elem_a=lower_rotors[arm_name].get_visual("hub"),
                elem_b=frame.get_visual(f"{arm_name}_lower_axle_housing"),
                name=f"{arm_name}_lower_rotor_remains_seated_while_spinning",
            )
            ctx.expect_contact(
                upper_rotors[arm_name],
                frame,
                elem_a=upper_rotors[arm_name].get_visual("hub"),
                elem_b=frame.get_visual(f"{arm_name}_upper_axle_housing"),
                name=f"{arm_name}_upper_rotor_remains_seated_while_spinning",
            )
            ctx.expect_origin_distance(
                upper_rotors[arm_name],
                lower_rotors[arm_name],
                axes="xy",
                max_dist=0.001,
                name=f"{arm_name}_coaxial_alignment_is_preserved_in_motion",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
