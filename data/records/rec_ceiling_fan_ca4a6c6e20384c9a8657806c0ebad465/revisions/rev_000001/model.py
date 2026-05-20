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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _blade_profile(
    *,
    root_x: float,
    length: float,
    root_chord: float,
    mid_chord: float,
    tip_chord: float,
) -> list[tuple[float, float]]:
    tip_x = root_x + length
    shoulder_x = root_x + 0.18 * length
    mid_x = root_x + 0.76 * length
    return [
        (root_x, -0.5 * root_chord),
        (shoulder_x, -0.5 * root_chord * 0.96),
        (mid_x, -0.5 * mid_chord),
        (tip_x - 0.018, -0.5 * tip_chord),
        (tip_x, 0.0),
        (tip_x - 0.018, 0.5 * tip_chord),
        (mid_x, 0.5 * mid_chord),
        (shoulder_x, 0.5 * root_chord * 0.96),
        (root_x, 0.5 * root_chord),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_blade_ceiling_fan")

    housing_white = model.material("housing_white", rgba=(0.95, 0.96, 0.97, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.57, 0.60, 0.64, 1.0))
    aluminum_blade = model.material("aluminum_blade", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.17, 0.19, 0.21, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=0.076, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=housing_white,
        name="canopy_shell",
    )
    canopy.visual(
        Cylinder(radius=0.084, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dark_hardware,
        name="ceiling_plate",
    )

    downrod = model.part("downrod")
    downrod.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=satin_steel,
        name="upper_collar",
    )
    downrod.visual(
        Cylinder(radius=0.011, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.0645)),
        material=satin_steel,
        name="rod_tube",
    )
    downrod.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.117)),
        material=dark_hardware,
        name="lower_collar",
    )

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_hardware,
        name="spindle_collar",
    )
    housing.visual(
        Cylinder(radius=0.092, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=housing_white,
        name="upper_cap",
    )
    housing.visual(
        Cylinder(radius=0.135, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, -0.114)),
        material=housing_white,
        name="motor_drum",
    )
    housing.visual(
        Sphere(radius=0.074),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=housing_white,
        name="lower_bulge",
    )
    housing.visual(
        Cylinder(radius=0.068, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.186)),
        material=dark_hardware,
        name="bottom_cap",
    )

    bracket_radius = 0.145
    blade_plane_z = -0.056
    bracket_size = (0.024, 0.072, 0.018)
    for index, angle in enumerate((0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi), start=1):
        bracket_x = bracket_radius * math.cos(angle)
        bracket_y = bracket_radius * math.sin(angle)
        housing.visual(
            Box(bracket_size),
            origin=Origin(
                xyz=(bracket_x, bracket_y, blade_plane_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_hardware,
            name=f"blade_bracket_{index}",
        )

    blade_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            _blade_profile(
                root_x=0.016,
                length=0.304,
                root_chord=0.088,
                mid_chord=0.070,
                tip_chord=0.054,
            ),
            0.004,
        ),
        "fan_blade_panel",
    )

    blade_names: list[str] = []
    for index in range(1, 5):
        blade = model.part(f"blade_{index}")
        blade_names.append(blade.name)
        blade.visual(
            Box((0.032, 0.060, 0.012)),
            origin=Origin(xyz=(0.016, 0.0, 0.0)),
            material=dark_hardware,
            name="root_cuff",
        )
        blade.visual(
            blade_panel_mesh,
            material=aluminum_blade,
            name="blade_panel",
        )

    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
    )
    model.articulation(
        "downrod_to_housing",
        ArticulationType.CONTINUOUS,
        parent=downrod,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, -0.127)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )

    hinge_radius = bracket_radius + 0.5 * bracket_size[0]
    for index, angle in enumerate((0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi), start=1):
        hinge_x = hinge_radius * math.cos(angle)
        hinge_y = hinge_radius * math.sin(angle)
        model.articulation(
            f"housing_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=blade_names[index - 1],
            origin=Origin(
                xyz=(hinge_x, hinge_y, blade_plane_z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.5,
                lower=0.0,
                upper=1.48,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    housing = object_model.get_part("housing")
    blades = [object_model.get_part(f"blade_{index}") for index in range(1, 5)]
    housing_spin = object_model.get_articulation("downrod_to_housing")
    blade_hinges = [
        object_model.get_articulation(f"housing_to_blade_{index}")
        for index in range(1, 5)
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

    ctx.check(
        "part_count",
        len(object_model.parts) == 7,
        f"Expected canopy, downrod, housing, and four blades; found {len(object_model.parts)} parts.",
    )
    ctx.check(
        "housing_spin_joint",
        housing_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(housing_spin.axis) == (0.0, 0.0, 1.0),
        "Housing should spin continuously about the vertical axle.",
    )
    ctx.check(
        "blade_hinges_axis_and_limits",
        all(
            hinge.articulation_type == ArticulationType.REVOLUTE
            and tuple(hinge.axis) == (0.0, 1.0, 0.0)
            and hinge.motion_limits is not None
            and hinge.motion_limits.lower == 0.0
            and hinge.motion_limits.upper is not None
            and 1.40 <= hinge.motion_limits.upper <= 1.55
            for hinge in blade_hinges
        ),
        "Each blade should fold on a local tangential hinge from extended to nearly vertical.",
    )

    ctx.expect_gap(
        canopy,
        downrod,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="canopy_seats_on_downrod",
    )
    ctx.expect_gap(
        downrod,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        name="downrod_seats_on_housing",
    )

    for index, blade in enumerate(blades, start=1):
        ctx.expect_contact(blade, housing, name=f"blade_{index}_mounted_to_housing")
        ctx.expect_origin_distance(
            blade,
            housing,
            axes="xy",
            min_dist=0.15,
            max_dist=0.17,
            name=f"blade_{index}_root_radius",
        )

    with ctx.pose({hinge: 1.48 for hinge in blade_hinges}):
        for index, blade in enumerate(blades, start=1):
            ctx.expect_within(
                blade,
                housing,
                axes="xy",
                margin=0.035,
                name=f"blade_{index}_stores_close_to_housing",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
