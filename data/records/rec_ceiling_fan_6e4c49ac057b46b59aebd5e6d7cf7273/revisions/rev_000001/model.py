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


def build_object_model() -> ArticulatedObject:
    def blade_mesh():
        blade_outline = [
            (0.000, -0.062),
            (0.060, -0.068),
            (0.190, -0.077),
            (0.340, -0.074),
            (0.470, -0.064),
            (0.545, -0.046),
            (0.572, -0.020),
            (0.582, 0.000),
            (0.572, 0.020),
            (0.545, 0.046),
            (0.470, 0.064),
            (0.340, 0.074),
            (0.190, 0.077),
            (0.060, 0.068),
            (0.000, 0.062),
        ]
        return mesh_from_geometry(
            ExtrudeGeometry.from_z0(blade_outline, 0.012),
            "reclaimed_wood_blade",
        )

    def housing_mesh():
        housing_profile = [
            (-0.162, -0.102),
            (0.162, -0.102),
            (0.170, -0.094),
            (0.170, 0.094),
            (0.162, 0.102),
            (-0.162, 0.102),
            (-0.170, 0.094),
            (-0.170, -0.094),
        ]
        return mesh_from_geometry(
            ExtrudeGeometry.from_z0(housing_profile, 0.180),
            "galvanized_motor_housing",
        )

    def canopy_mesh():
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.022, 0.0),
                    (0.040, -0.004),
                    (0.072, -0.016),
                    (0.096, -0.034),
                    (0.102, -0.054),
                    (0.094, -0.064),
                    (0.082, -0.070),
                    (0.024, -0.076),
                ],
                [
                    (0.018, -0.002),
                    (0.032, -0.008),
                    (0.061, -0.018),
                    (0.084, -0.035),
                    (0.092, -0.053),
                    (0.086, -0.060),
                    (0.076, -0.064),
                    (0.026, -0.069),
                ],
                segments=56,
                start_cap="flat",
                end_cap="flat",
            ),
            "fan_canopy_shell",
        )

    model = ArticulatedObject(name="farmhouse_ceiling_fan")

    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.70, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.19, 0.19, 0.20, 1.0))
    aged_wood = model.material("aged_wood", rgba=(0.47, 0.35, 0.24, 1.0))
    darker_wood = model.material("darker_wood", rgba=(0.35, 0.25, 0.18, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        canopy_mesh(),
        origin=Origin(),
        material=galvanized,
        name="canopy_shell",
    )
    canopy.visual(
        Cylinder(radius=0.114, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=galvanized,
        name="ceiling_plate",
    )
    canopy.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=dark_iron,
        name="hanger_seat",
    )
    canopy.visual(
        Cylinder(radius=0.022, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=dark_iron,
        name="hanger_neck",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.114, length=0.060),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    downrod = model.part("downrod")
    downrod.visual(
        Cylinder(radius=0.013, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=dark_iron,
        name="downrod_tube",
    )
    downrod.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=dark_iron,
        name="upper_coupler",
    )
    downrod.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.227)),
        material=dark_iron,
        name="lower_coupler",
    )
    downrod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.240),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
    )

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Cylinder(radius=0.034, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_iron,
        name="mount_collar",
    )
    motor_housing.visual(
        housing_mesh(),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=galvanized,
        name="housing_body",
    )
    motor_housing.visual(
        Box((0.356, 0.236, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.214)),
        material=galvanized,
        name="bottom_band",
    )
    motor_housing.visual(
        Box((0.320, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.102, -0.090)),
        material=dark_iron,
        name="front_strap",
    )
    motor_housing.visual(
        Box((0.320, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.102, -0.090)),
        material=dark_iron,
        name="rear_strap",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Box((0.356, 0.236, 0.226)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, -0.113)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.050, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=dark_iron,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=dark_iron,
        name="switch_cap",
    )

    iron_center_radius = 0.130
    blade_root_radius = 0.210
    blade_specs = (
        ("blade_1", "iron_1", "mount_pad_1", 0.0, aged_wood),
        ("blade_2", "iron_2", "mount_pad_2", 2.0 * math.pi / 3.0, darker_wood),
        ("blade_3", "iron_3", "mount_pad_3", 4.0 * math.pi / 3.0, aged_wood),
    )
    for blade_name, iron_name, pad_name, angle, blade_material in blade_specs:
        c = math.cos(angle)
        s = math.sin(angle)
        rotor.visual(
            Box((0.172, 0.056, 0.004)),
            origin=Origin(
                xyz=(iron_center_radius * c, iron_center_radius * s, -0.024),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_iron,
            name=iron_name,
        )
        rotor.visual(
            Box((0.050, 0.048, 0.004)),
            origin=Origin(
                xyz=((blade_root_radius - 0.025) * c, (blade_root_radius - 0.025) * s, -0.024),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_iron,
            name=pad_name,
        )
        rotor.visual(
            blade_mesh(),
            origin=Origin(
                xyz=(blade_root_radius * c, blade_root_radius * s, -0.038),
                rpy=(0.0, 0.0, angle),
            ),
            material=blade_material,
            name=blade_name,
        )

    rotor.inertial = Inertial.from_geometry(
        Box((1.520, 1.520, 0.072)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
    )

    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
    )
    model.articulation(
        "downrod_to_motor_housing",
        ArticulationType.FIXED,
        parent=downrod,
        child=motor_housing,
        origin=Origin(xyz=(0.0, 0.0, -0.240)),
    )
    model.articulation(
        "motor_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=motor_housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    motor_housing = object_model.get_part("motor_housing")
    rotor = object_model.get_part("rotor")
    rotor_spin = object_model.get_articulation("motor_to_rotor")

    ctx.expect_contact(
        canopy,
        downrod,
        elem_a="hanger_seat",
        elem_b="upper_coupler",
        contact_tol=1e-6,
        name="downrod seats against canopy",
    )
    ctx.expect_gap(
        downrod,
        motor_housing,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        name="motor housing hangs from downrod",
    )
    ctx.expect_gap(
        motor_housing,
        rotor,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="housing_body",
        negative_elem="hub_shell",
        name="rotor hub meets housing bottom cleanly",
    )
    ctx.expect_gap(
        motor_housing,
        rotor,
        axis="z",
        min_gap=0.015,
        positive_elem="housing_body",
        negative_elem="blade_1",
        name="blade clears motor housing underside",
    )
    ctx.expect_overlap(
        motor_housing,
        rotor,
        axes="xy",
        elem_a="housing_body",
        elem_b="hub_shell",
        min_overlap=0.080,
        name="rotor stays centered under housing",
    )

    def elem_center(part_name: str, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        min_pt, max_pt = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))

    with ctx.pose({rotor_spin: 0.0}):
        rest_center = elem_center("rotor", "blade_1")
    with ctx.pose({rotor_spin: math.pi / 3.0}):
        turned_center = elem_center("rotor", "blade_1")

    ctx.check(
        "blade assembly rotates around the downrod axis",
        rest_center is not None
        and turned_center is not None
        and abs(rest_center[0] - turned_center[0]) > 0.18
        and abs(rest_center[1] - turned_center[1]) > 0.18,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
