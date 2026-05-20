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
    section_loft,
)
def _build_canopy_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.012, 0.000),
            (0.040, -0.008),
            (0.072, -0.026),
            (0.090, -0.052),
            (0.096, -0.076),
        ],
        [
            (0.000, -0.002),
            (0.028, -0.010),
            (0.060, -0.028),
            (0.078, -0.052),
            (0.084, -0.072),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_swivel_knuckle_mesh():
    return LatheGeometry(
        [
            (0.000, 0.012),
            (0.010, 0.010),
            (0.018, 0.004),
            (0.021, -0.008),
            (0.020, -0.020),
            (0.014, -0.032),
            (0.000, -0.038),
        ],
        segments=56,
    )


def _build_motor_housing_mesh():
    return LatheGeometry(
        [
            (0.000, -0.004),
            (0.048, -0.008),
            (0.082, -0.028),
            (0.112, -0.068),
            (0.126, -0.118),
            (0.118, -0.160),
            (0.088, -0.198),
            (0.040, -0.224),
            (0.000, -0.232),
        ],
        segments=84,
    )


def _blade_section(x_pos: float, chord: float, thickness: float, camber: float):
    return [
        (x_pos, -0.50 * chord, 0.26 * thickness + 0.60 * camber),
        (x_pos, -0.16 * chord, 0.50 * thickness + 0.95 * camber),
        (x_pos, 0.22 * chord, 0.28 * thickness + 0.15 * camber),
        (x_pos, 0.50 * chord, -0.18 * thickness - 0.72 * camber),
        (x_pos, 0.18 * chord, -0.48 * thickness - 0.56 * camber),
        (x_pos, -0.24 * chord, -0.20 * thickness + 0.04 * camber),
    ]


def _build_blade_mesh():
    return section_loft(
        [
            _blade_section(0.180, 0.162, 0.010, 0.010),
            _blade_section(0.320, 0.154, 0.0085, 0.009),
            _blade_section(0.480, 0.132, 0.0060, 0.007),
            _blade_section(0.655, 0.104, 0.0035, 0.005),
        ]
    )


def _arm_section(x_pos: float, width: float, thickness: float, crown: float):
    return [
        (x_pos, -0.50 * width, 0.28 * thickness + crown),
        (x_pos, -0.18 * width, 0.50 * thickness + 1.15 * crown),
        (x_pos, 0.26 * width, 0.34 * thickness + 0.72 * crown),
        (x_pos, 0.50 * width, -0.08 * thickness - 0.08 * crown),
        (x_pos, 0.18 * width, -0.50 * thickness - 0.36 * crown),
        (x_pos, -0.24 * width, -0.26 * thickness - 0.16 * crown),
    ]


def _build_arm_mesh():
    return section_loft(
        [
            _arm_section(0.042, 0.044, 0.013, 0.0015),
            _arm_section(0.115, 0.050, 0.011, 0.0010),
            _arm_section(0.188, 0.058, 0.009, 0.0006),
            _arm_section(0.226, 0.064, 0.007, 0.0003),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_ceiling_fan")

    white_powder = model.material("white_powder", rgba=(0.93, 0.94, 0.95, 1.0))
    satin_nickel = model.material("satin_nickel", rgba=(0.67, 0.70, 0.73, 1.0))
    wet_black = model.material("wet_black", rgba=(0.18, 0.19, 0.21, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.72, 0.75, 0.76, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_geometry(_build_canopy_mesh(), "canopy_shell"),
        material=white_powder,
        name="canopy_shell",
    )
    canopy.visual(
        Cylinder(radius=0.084, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        material=white_powder,
        name="canopy_insert",
    )
    canopy.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.077)),
        material=satin_nickel,
        name="hanger_neck",
    )
    canopy.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=satin_nickel,
        name="socket_collar",
    )
    canopy.visual(
        Cylinder(radius=0.042, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=satin_nickel,
        name="ceiling_plate",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.11)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )

    downrod = model.part("downrod")
    downrod.visual(
        mesh_from_geometry(_build_swivel_knuckle_mesh(), "swivel_knuckle"),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=satin_nickel,
        name="swivel_ball",
    )
    downrod.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=satin_nickel,
        name="swivel_collar",
    )
    downrod.visual(
        Cylinder(radius=0.011, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, -0.188)),
        material=satin_nickel,
        name="downrod_tube",
    )
    downrod.visual(
        Cylinder(radius=0.015, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=wet_black,
        name="rod_seal",
    )
    downrod.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.348)),
        material=satin_nickel,
        name="lower_coupler",
    )
    downrod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.390),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
    )

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=satin_nickel,
        name="motor_cap",
    )
    motor_housing.visual(
        mesh_from_geometry(_build_motor_housing_mesh(), "motor_body"),
        material=white_powder,
        name="motor_body",
    )
    motor_housing.visual(
        Cylinder(radius=0.100, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=satin_nickel,
        name="trim_band",
    )
    motor_housing.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.219)),
        material=wet_black,
        name="housing_nose",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.130, length=0.240),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
    )

    blade_mesh = mesh_from_geometry(_build_blade_mesh(), "fan_blade")
    arm_mesh = mesh_from_geometry(_build_arm_mesh(), "blade_arm")

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=wet_black,
        name="rotor_hub",
    )
    rotor.visual(
        Cylinder(radius=0.038, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=wet_black,
        name="hub_cap",
    )
    rotor.visual(
        arm_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=wet_black,
        name="arm_left",
    )
    rotor.visual(
        arm_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.016), rpy=(0.0, 0.0, math.pi)),
        material=wet_black,
        name="arm_right",
    )
    rotor.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=blade_gray,
        name="blade_left",
    )
    rotor.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.018), rpy=(0.0, 0.0, math.pi)),
        material=blade_gray,
        name="blade_right",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.680, length=0.050),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
    )

    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
    )
    model.articulation(
        "downrod_to_motor",
        ArticulationType.FIXED,
        parent=downrod,
        child=motor_housing,
        origin=Origin(xyz=(0.0, 0.0, -0.368)),
    )
    model.articulation(
        "motor_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=motor_housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.228)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
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
    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    motor_housing = object_model.get_part("motor_housing")
    rotor = object_model.get_part("rotor")
    spin_joint = object_model.get_articulation("motor_to_rotor")

    rotor.get_visual("arm_left")
    rotor.get_visual("arm_right")
    rotor.get_visual("blade_left")
    rotor.get_visual("blade_right")

    ctx.expect_origin_distance(
        canopy,
        downrod,
        axes="xy",
        max_dist=0.001,
        name="downrod stays centered beneath the canopy",
    )
    ctx.expect_origin_distance(
        canopy,
        motor_housing,
        axes="xy",
        max_dist=0.001,
        name="motor housing remains centered on the downrod axis",
    )
    ctx.expect_origin_gap(
        canopy,
        motor_housing,
        axis="z",
        min_gap=0.465,
        max_gap=0.478,
        name="motor housing hangs below the ceiling canopy on a downrod",
    )
    ctx.expect_contact(
        downrod,
        motor_housing,
        elem_a="lower_coupler",
        elem_b="motor_cap",
        contact_tol=0.0005,
        name="downrod lower coupler seats directly on the motor cap",
    )
    ctx.expect_contact(
        motor_housing,
        rotor,
        elem_a="housing_nose",
        elem_b="rotor_hub",
        contact_tol=0.0005,
        name="rotor hub mounts at the housing nose",
    )
    ctx.expect_overlap(
        rotor,
        motor_housing,
        axes="xy",
        elem_a="rotor_hub",
        elem_b="motor_body",
        min_overlap=0.060,
        name="rotor stays centered under the sealed motor housing",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((low + high) * 0.5 for low, high in zip(min_corner, max_corner))

    rest_blade_aabb = ctx.part_element_world_aabb(rotor, elem="blade_left")
    with ctx.pose({spin_joint: math.pi / 2.0}):
        quarter_turn_blade_aabb = ctx.part_element_world_aabb(rotor, elem="blade_left")

    rest_center = _center_from_aabb(rest_blade_aabb)
    quarter_turn_center = _center_from_aabb(quarter_turn_blade_aabb)
    ctx.check(
        "blade assembly rotates continuously about the vertical axis",
        rest_center is not None
        and quarter_turn_center is not None
        and rest_center[0] > 0.30
        and abs(rest_center[1]) < 0.05
        and abs(quarter_turn_center[0]) < 0.08
        and quarter_turn_center[1] > 0.30
        and abs(quarter_turn_center[2] - rest_center[2]) < 0.005,
        details=f"rest={rest_center}, quarter_turn={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
