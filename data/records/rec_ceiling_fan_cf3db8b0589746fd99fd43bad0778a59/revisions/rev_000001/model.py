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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


CANOPY_RADIUS = 0.074
CANOPY_HEIGHT = 0.046
DROPROD_RADIUS = 0.011
DROPROD_LENGTH = 0.090
MOUNT_COLLAR_RADIUS = 0.038
MOUNT_COLLAR_LENGTH = 0.022
SPIN_ORIGIN_Z = -(CANOPY_HEIGHT + DROPROD_LENGTH + MOUNT_COLLAR_LENGTH)

HOUSING_RADIUS = 0.096
HOUSING_LENGTH = 0.162
HINGE_RADIUS = 0.108
HINGE_Z = -0.076
BLADE_PITCH = math.radians(13.0)
BLADE_FOLD_LIMIT = -1.33


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _canopy_mesh():
    return LatheGeometry(
        [
            (0.0, 0.0),
            (0.020, -0.002),
            (0.050, -0.010),
            (0.072, -0.022),
            (0.070, -0.036),
            (0.028, -0.046),
            (0.0, -0.046),
        ],
        segments=64,
    )


def _motor_housing_mesh():
    return LatheGeometry(
        [
            (0.0, 0.0),
            (0.028, -0.003),
            (0.058, -0.012),
            (0.084, -0.028),
            (0.096, -0.054),
            (0.094, -0.102),
            (0.078, -0.140),
            (0.046, -0.160),
            (0.0, -0.162),
        ],
        segments=80,
    )


def _blade_section(x_pos: float, chord: float, thickness: float) -> list[tuple[float, float, float]]:
    half_chord = chord * 0.5
    return [
        (x_pos, -1.00 * half_chord, 0.00 * thickness),
        (x_pos, -0.30 * half_chord, 0.72 * thickness),
        (x_pos, 0.46 * half_chord, 0.58 * thickness),
        (x_pos, 1.00 * half_chord, 0.10 * thickness),
        (x_pos, 0.64 * half_chord, -0.18 * thickness),
        (x_pos, -0.08 * half_chord, -0.34 * thickness),
    ]


def _blade_shell_mesh():
    sections = [
        _blade_section(0.078, 0.096, 0.0090),
        _blade_section(0.180, 0.118, 0.0084),
        _blade_section(0.322, 0.114, 0.0070),
        _blade_section(0.468, 0.098, 0.0054),
        _blade_section(0.565, 0.080, 0.0036),
    ]
    return repair_loft(section_loft(sections), repair="mesh")


def _blade_iron_section(
    x_pos: float,
    width: float,
    thickness: float,
    crown: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    half_thickness = thickness * 0.5
    return [
        (x_pos, -1.00 * half_width, -0.40 * half_thickness),
        (x_pos, -0.38 * half_width, half_thickness + crown),
        (x_pos, 0.38 * half_width, half_thickness + crown),
        (x_pos, 1.00 * half_width, -0.40 * half_thickness),
        (x_pos, 0.42 * half_width, -half_thickness),
        (x_pos, -0.42 * half_width, -half_thickness),
    ]


def _blade_iron_mesh():
    return repair_loft(
        section_loft(
            [
                _blade_iron_section(0.006, 0.030, 0.008),
                _blade_iron_section(0.050, 0.026, 0.007, crown=0.0005),
                _blade_iron_section(0.112, 0.024, 0.0065, crown=0.0005),
                _blade_iron_section(0.146, 0.056, 0.0075, crown=0.0003),
                _blade_iron_section(0.172, 0.056, 0.0075),
            ]
        ),
        repair="mesh",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_foldable_ceiling_fan")

    bronze = model.material("bronze", rgba=(0.22, 0.18, 0.15, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.12, 0.13, 1.0))
    blade_abs = model.material("blade_abs", rgba=(0.52, 0.43, 0.32, 1.0))

    canopy_mesh = _mesh(_canopy_mesh(), "ceiling_canopy")
    housing_mesh = _mesh(_motor_housing_mesh(), "motor_housing")
    blade_mesh = _mesh(_blade_shell_mesh(), "fan_blade_shell")
    blade_iron_mesh = _mesh(_blade_iron_mesh(), "fan_blade_iron")

    mount = model.part("mount")
    mount.visual(canopy_mesh, material=bronze, name="canopy_shell")
    mount.visual(
        Cylinder(radius=DROPROD_RADIUS, length=DROPROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -(CANOPY_HEIGHT + (DROPROD_LENGTH * 0.5)))),
        material=bronze,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -(CANOPY_HEIGHT + 0.010))),
        material=bronze,
        name="yoke_collar",
    )
    mount.visual(
        Cylinder(radius=MOUNT_COLLAR_RADIUS, length=MOUNT_COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, SPIN_ORIGIN_Z + (MOUNT_COLLAR_LENGTH * 0.5))),
        material=dark_metal,
        name="lower_bearing_collar",
    )
    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=CANOPY_RADIUS, length=CANOPY_HEIGHT + DROPROD_LENGTH + MOUNT_COLLAR_LENGTH),
        mass=2.0,
        origin=Origin(
            xyz=(0.0, 0.0, -0.5 * (CANOPY_HEIGHT + DROPROD_LENGTH + MOUNT_COLLAR_LENGTH))
        ),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.033, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_metal,
        name="top_bearing_cap",
    )
    rotor.visual(
        Cylinder(radius=0.052, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=bronze,
        name="upper_motor_collar",
    )
    rotor.visual(housing_mesh, material=bronze, name="motor_housing")
    rotor.visual(
        Cylinder(radius=0.086, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        material=bronze,
        name="rim_band",
    )
    for index in range(5):
        angle = index * (math.tau / 5.0)
        lug_radius = HINGE_RADIUS - 0.023
        rotor.visual(
            Box((0.022, 0.028, 0.028)),
            origin=Origin(
                xyz=(lug_radius * math.cos(angle), lug_radius * math.sin(angle), HINGE_Z),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"lug_{index + 1}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=0.17),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    model.articulation(
        "mount_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, SPIN_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )

    for index in range(5):
        blade = model.part(f"blade_{index + 1}")
        blade.visual(
            Cylinder(radius=0.012, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_barrel",
        )
        blade.visual(
            blade_iron_mesh,
            origin=Origin(rpy=(BLADE_PITCH, 0.0, 0.0)),
            material=dark_metal,
            name="blade_iron",
        )
        blade.visual(
            blade_mesh,
            origin=Origin(rpy=(BLADE_PITCH, 0.0, 0.0)),
            material=blade_abs,
            name="blade_shell",
        )
        blade.inertial = Inertial.from_geometry(
            Box((0.58, 0.13, 0.03)),
            mass=0.42,
            origin=Origin(xyz=(0.29, 0.0, 0.0)),
        )

        angle = index * (math.tau / 5.0)
        model.articulation(
            f"rotor_to_blade_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=rotor,
            child=blade,
            origin=Origin(
                xyz=(HINGE_RADIUS * math.cos(angle), HINGE_RADIUS * math.sin(angle), HINGE_Z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=2.5,
                lower=BLADE_FOLD_LIMIT,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("mount_to_rotor")
    blade_1 = object_model.get_part("blade_1")
    blade_hinges = [
        object_model.get_articulation(f"rotor_to_blade_{index}")
        for index in range(1, 6)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        mount,
        rotor,
        elem_a="lower_bearing_collar",
        elem_b="top_bearing_cap",
        name="rotor seats on the lower bearing collar",
    )

    for index in range(1, 6):
        blade = object_model.get_part(f"blade_{index}")
        joint = object_model.get_articulation(f"rotor_to_blade_{index}")
        ctx.expect_contact(
            blade,
            rotor,
            elem_a="hinge_barrel",
            elem_b=f"lug_{index}",
            name=f"blade {index} hinge barrel stays mounted to its lug",
        )
        ctx.check(
            f"blade {index} hinge uses a vertical folding pin",
            tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"blade {index} hinge permits folded storage and open running pose",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= -1.10
            and abs(limits.upper) <= 1e-9,
            details=f"limits={limits}",
        )

    spin_limits = spin.motion_limits
    ctx.check(
        "central axle is a continuous vertical spin joint",
        tuple(spin.axis) == (0.0, 0.0, 1.0)
        and spin_limits is not None
        and spin_limits.lower is None
        and spin_limits.upper is None,
        details=f"axis={spin.axis}, limits={spin_limits}",
    )

    open_aabb = ctx.part_element_world_aabb(blade_1, elem="blade_shell")
    folded_aabb = None
    with ctx.pose({blade_hinges[0]: BLADE_FOLD_LIMIT}):
        ctx.expect_contact(
            blade_1,
            rotor,
            elem_a="hinge_barrel",
            elem_b="lug_1",
            name="folded blade keeps hinge contact at the rotor rim",
        )
        folded_aabb = ctx.part_element_world_aabb(blade_1, elem="blade_shell")

    blade_retracts = False
    if open_aabb is not None and folded_aabb is not None:
        blade_retracts = (
            folded_aabb[1][0] < open_aabb[1][0] - 0.18
            and folded_aabb[0][1] < open_aabb[0][1] - 0.12
        )
    ctx.check(
        "first blade folds back toward storage around the housing",
        blade_retracts,
        details=f"open={open_aabb}, folded={folded_aabb}",
    )

    with ctx.pose({spin: 1.1}):
        ctx.expect_contact(
            mount,
            rotor,
            elem_a="lower_bearing_collar",
            elem_b="top_bearing_cap",
            name="rotor remains seated while spinning on the axle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
