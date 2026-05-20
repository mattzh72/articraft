from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _build_arm_part(part, *, metal, glass, arm_mesh, cup_mesh) -> None:
    part.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="root_sleeve",
    )
    part.visual(arm_mesh, material=metal, name="arm_sweep")
    part.visual(
        cup_mesh,
        origin=Origin(xyz=(0.385, 0.0, 0.0)),
        material=metal,
        name="bulb_cup",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.385, 0.0, 0.026)),
        material=metal,
        name="bulb_neck",
    )
    part.visual(
        Sphere(radius=0.033),
        origin=Origin(xyz=(0.385, 0.0, 0.073)),
        material=glass,
        name="bulb_globe",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_arm_chandelier")

    dark_bronze = model.material("dark_bronze", rgba=(0.25, 0.20, 0.14, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.58, 0.48, 0.28, 1.0))
    opal_glass = model.material("opal_glass", rgba=(0.94, 0.91, 0.84, 0.72))

    canopy_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.070),
                (0.028, 0.070),
                (0.048, 0.064),
                (0.072, 0.044),
                (0.090, 0.014),
                (0.086, 0.000),
                (0.0, 0.000),
            ],
            segments=56,
        ),
        "ceiling_canopy",
    )
    hub_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.100),
                (0.042, 0.100),
                (0.066, 0.094),
                (0.088, 0.070),
                (0.095, 0.020),
                (0.088, -0.028),
                (0.070, -0.066),
                (0.046, -0.090),
                (0.0, -0.090),
            ],
            segments=64,
        ),
        "center_hub",
    )
    arm_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.022, 0.0, 0.000),
                (0.098, 0.0, -0.010),
                (0.205, 0.0, -0.032),
                (0.305, 0.0, -0.010),
                (0.388, 0.0, 0.004),
            ],
            radius=0.013,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        "arm_sweep",
    )
    cup_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.018, 0.000),
                (0.026, 0.006),
                (0.033, 0.022),
                (0.034, 0.045),
                (0.028, 0.051),
                (0.020, 0.051),
                (0.015, 0.043),
                (0.013, 0.010),
                (0.0, 0.008),
                (0.0, 0.000),
            ],
            segments=48,
        ),
        "bulb_cup",
    )

    hub_radius = 0.095
    boss_length = 0.020
    boss_radius = 0.010
    pivot_z = 0.018

    hub_canopy = model.part("hub_canopy")
    hub_canopy.visual(
        canopy_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=dark_bronze,
        name="ceiling_canopy",
    )
    hub_canopy.visual(
        Cylinder(radius=0.014, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=aged_brass,
        name="downrod",
    )
    hub_canopy.visual(hub_mesh, material=dark_bronze, name="center_hub")
    hub_canopy.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material=aged_brass,
        name="stem_collar",
    )
    hub_canopy.visual(
        Cylinder(radius=0.024, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.116)),
        material=aged_brass,
        name="lower_stem",
    )
    hub_canopy.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.154)),
        material=aged_brass,
        name="finial",
    )

    arm_specs = (
        ("arm_east", 0.0),
        ("arm_north", pi / 2.0),
        ("arm_west", pi),
        ("arm_south", -pi / 2.0),
    )
    for arm_name, yaw in arm_specs:
        hub_canopy.visual(
            Cylinder(radius=boss_radius, length=boss_length),
            origin=Origin(
                xyz=(
                    (hub_radius + boss_length * 0.5) * cos(yaw),
                    (hub_radius + boss_length * 0.5) * sin(yaw),
                    pivot_z,
                ),
                rpy=(0.0, pi / 2.0, yaw),
            ),
            material=aged_brass,
            name=f"{arm_name}_pivot_boss",
        )
        hub_canopy.visual(
            Box((0.020, 0.028, 0.016)),
            origin=Origin(
                xyz=(
                    (hub_radius + 0.010) * cos(yaw),
                    (hub_radius + 0.010) * sin(yaw),
                    pivot_z - 0.012,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=aged_brass,
            name=f"{arm_name}_mount_collar",
        )

    for arm_name, _yaw in arm_specs:
        arm = model.part(arm_name)
        _build_arm_part(
            arm,
            metal=aged_brass,
            glass=opal_glass,
            arm_mesh=arm_mesh,
            cup_mesh=cup_mesh,
        )

    for arm_name, yaw in arm_specs:
        model.articulation(
            f"hub_to_{arm_name}",
            ArticulationType.REVOLUTE,
            parent=hub_canopy,
            child=arm_name,
            origin=Origin(
                xyz=(
                    (hub_radius + boss_length) * cos(yaw),
                    (hub_radius + boss_length) * sin(yaw),
                    pivot_z,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.5,
                lower=-0.30,
                upper=0.45,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    hub_canopy = object_model.get_part("hub_canopy")
    arm_specs = (
        ("arm_east", 0.0),
        ("arm_north", pi / 2.0),
        ("arm_west", pi),
        ("arm_south", -pi / 2.0),
    )
    for arm_name, yaw in arm_specs:
        arm = object_model.get_part(arm_name)
        hinge = object_model.get_articulation(f"hub_to_{arm_name}")
        ctx.expect_contact(arm, hub_canopy, name=f"{arm_name}_is_mounted_to_hub")
        ctx.expect_origin_distance(
            arm,
            hub_canopy,
            axes="xy",
            min_dist=0.114,
            max_dist=0.116,
            name=f"{arm_name}_pivots_from_hub_perimeter",
        )
        ctx.check(
            f"{hinge.name}_axis_orientation",
            hinge.axis == (0.0, 1.0, 0.0),
            details=f"axis was {hinge.axis!r}",
        )
        ctx.check(
            f"{hinge.name}_yaw_layout",
            abs(hinge.origin.rpy[2] - yaw) < 1e-6,
            details=f"yaw was {hinge.origin.rpy[2]:.6f}, expected {yaw:.6f}",
        )
        limits = hinge.motion_limits
        ctx.check(
            f"{hinge.name}_motion_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= -0.25
            and 0.40 <= limits.upper <= 0.50,
            details=f"limits were {limits!r}",
        )
        with ctx.pose({hinge: 0.45}):
            ctx.expect_contact(arm, hub_canopy, name=f"{arm_name}_hinge_stays_seated")
            ctx.expect_gap(
                hub_canopy,
                arm,
                axis="z",
                positive_elem="ceiling_canopy",
                negative_elem="bulb_globe",
                min_gap=0.004,
                name=f"{arm_name}_bulb_clears_canopy_when_folded",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
