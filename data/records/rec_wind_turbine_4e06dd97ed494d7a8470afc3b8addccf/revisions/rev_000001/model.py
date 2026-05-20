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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    tower_paint = model.material("tower_paint", rgba=(0.88, 0.90, 0.92, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.83, 0.85, 0.87, 1.0))
    blade_paint = model.material("blade_paint", rgba=(0.95, 0.96, 0.97, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.70, 0.73, 0.76, 1.0))

    tower_height = 82.0
    tower_diameter = 4.2
    nacelle_length = 12.0
    nacelle_width = 4.4
    nacelle_height = 4.6
    nacelle_rear_overhang = 1.8
    shaft_center_z = 2.35
    rotor_hub_radius = 1.65
    rotor_hub_length = 3.2
    blade_span = 26.0
    blade_chord = 3.1
    blade_thickness = 0.38

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=tower_diameter * 0.5, length=tower_height),
        origin=Origin(xyz=(0.0, 0.0, tower_height * 0.5)),
        material=tower_paint,
        name="tower_shaft",
    )
    tower.visual(
        Cylinder(radius=3.6, length=0.9),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=tower_paint,
        name="tower_base",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=tower_diameter * 0.5, length=tower_height),
        mass=180000.0,
        origin=Origin(xyz=(0.0, 0.0, tower_height * 0.5)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Box((nacelle_length, nacelle_width, nacelle_height)),
        origin=Origin(
            xyz=(
                (nacelle_length * 0.5) - nacelle_rear_overhang,
                0.0,
                nacelle_height * 0.5,
            )
        ),
        material=nacelle_paint,
        name="nacelle_body",
    )
    nacelle.visual(
        Cylinder(radius=1.05, length=1.35),
        origin=Origin(
            xyz=(nacelle_length - nacelle_rear_overhang - 0.475, 0.0, shaft_center_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=nacelle_paint,
        name="main_bearing_housing",
    )
    nacelle.visual(
        Cylinder(radius=0.72, length=0.65),
        origin=Origin(
            xyz=(nacelle_length - nacelle_rear_overhang + 0.525, 0.0, shaft_center_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hub_metal,
        name="main_shaft_nose",
    )
    nacelle.visual(
        Box((3.4, 2.8, 0.45)),
        origin=Origin(xyz=(-0.2, 0.0, 0.225)),
        material=hub_metal,
        name="yaw_bedplate",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((nacelle_length, nacelle_width, nacelle_height)),
        mass=56000.0,
        origin=Origin(
            xyz=(
                (nacelle_length * 0.5) - nacelle_rear_overhang,
                0.0,
                nacelle_height * 0.5,
            )
        ),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=rotor_hub_radius, length=rotor_hub_length),
        origin=Origin(xyz=(rotor_hub_length * 0.5, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hub_metal,
        name="hub_body",
    )
    rotor.visual(
        Sphere(radius=1.35),
        origin=Origin(xyz=(rotor_hub_length + 0.7, 0.0, 0.0)),
        material=hub_metal,
        name="spinner",
    )
    for blade_index, blade_angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        rotor.visual(
            Box((blade_thickness, blade_chord, blade_span)),
            origin=Origin(
                xyz=(0.2, 0.0, rotor_hub_radius + (blade_span * 0.5) - 0.15),
                rpy=(blade_angle, 0.0, 0.0),
            ),
            material=blade_paint,
            name=f"blade_{blade_index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=5.0, length=blade_span * 2.0),
        mass=39000.0,
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
    )

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, tower_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1500000.0, velocity=0.35),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(
            xyz=(nacelle_length - nacelle_rear_overhang + 0.85, 0.0, shaft_center_z)
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500000.0, velocity=2.5),
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

    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw_joint = object_model.get_articulation("tower_to_nacelle_yaw")
    spin_joint = object_model.get_articulation("nacelle_to_rotor_spin")

    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        positive_elem="hub_body",
        negative_elem="main_shaft_nose",
        min_gap=0.0,
        max_gap=1e-6,
        name="hub seats on the main shaft nose",
    )

    def element_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    shaft_nose_length = 0.65
    rotor_hub_length = 3.2
    expected_center_spacing = (shaft_nose_length * 0.5) + (rotor_hub_length * 0.5)

    def check_rotor_axis_alignment(name: str) -> None:
        hub_center = element_center("rotor", "hub_body")
        support_center = element_center("nacelle", "main_shaft_nose")
        center_spacing = None
        z_offset = None
        ok = (
            hub_center is not None
            and support_center is not None
            and ((z_offset := abs(hub_center[2] - support_center[2])) is not None)
            and z_offset <= 1e-4
            and (
                center_spacing := math.dist(hub_center, support_center)
            ) is not None
            and abs(center_spacing - expected_center_spacing) <= 1e-4
        )
        ctx.check(
            name,
            ok,
            details=(
                f"hub_center={hub_center}, support_center={support_center}, "
                f"z_offset={z_offset}, center_spacing={center_spacing}, "
                f"expected_center_spacing={expected_center_spacing}"
            ),
        )

    check_rotor_axis_alignment("rotor hub stays centered on nacelle shaft axis at rest")
    with ctx.pose({yaw_joint: 0.7, spin_joint: 1.1}):
        check_rotor_axis_alignment("rotor hub stays centered on nacelle shaft axis while yawed and spinning")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
