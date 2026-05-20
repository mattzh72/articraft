from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arc_floor_lamp")

    anthracite = model.material("anthracite", rgba=(0.14, 0.15, 0.16, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.24, 0.25, 0.27, 1.0))
    shade_white = model.material("shade_white", rgba=(0.93, 0.92, 0.89, 1.0))
    diffuser_warm = model.material("diffuser_warm", rgba=(0.97, 0.93, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.23, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=anthracite,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=dark_hardware,
        name="base_collar",
    )
    base.visual(
        Cylinder(radius=0.016, length=1.58),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=satin_steel,
        name="upright_column",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 1.64), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="pivot_crossbar",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(xyz=(0.0, 0.040, 1.69)),
        material=dark_hardware,
        name="pivot_riser_pos",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(xyz=(0.0, -0.040, 1.69)),
        material=dark_hardware,
        name="pivot_riser_neg",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.034, 1.74), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="pivot_cheek_pos",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, -0.034, 1.74), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="pivot_cheek_neg",
    )

    arm_assembly = model.part("arm_assembly")
    arm_assembly.visual(
        Cylinder(radius=0.025, length=0.058),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="pivot_collar",
    )
    arm_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.0),
                (0.10, 0.0, 0.08),
                (0.32, 0.0, 0.22),
                (0.66, 0.0, 0.39),
                (0.96, 0.0, 0.44),
                (1.15, 0.0, 0.31),
                (1.24, 0.0, 0.12),
                (1.24, 0.0, 0.0),
            ],
            radius=0.016,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "lamp_arm_tube",
    )
    arm_assembly.visual(
        arm_tube_mesh,
        material=satin_steel,
        name="arching_tube",
    )
    arm_assembly.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(1.24, 0.0, -0.045)),
        material=dark_hardware,
        name="shade_stem",
    )
    arm_assembly.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(1.24, 0.0, -0.084)),
        material=dark_hardware,
        name="shade_hub",
    )
    arm_assembly.visual(
        Cylinder(radius=0.21, length=0.018),
        origin=Origin(xyz=(1.24, 0.0, -0.099)),
        material=shade_white,
        name="shade_disc",
    )
    arm_assembly.visual(
        Cylinder(radius=0.17, length=0.006),
        origin=Origin(xyz=(1.24, 0.0, -0.111)),
        material=diffuser_warm,
        name="diffuser_disc",
    )

    model.articulation(
        "tilt_adjust",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_assembly,
        origin=Origin(xyz=(0.0, 0.0, 1.74)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=-0.28,
            upper=0.42,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm_assembly = object_model.get_part("arm_assembly")
    tilt_adjust = object_model.get_articulation("tilt_adjust")

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

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
        "parts_present",
        base is not None and arm_assembly is not None and tilt_adjust is not None,
        "Expected base, arm assembly, and tilt-adjust articulation.",
    )
    ctx.check(
        "tilt_joint_axis_and_limits",
        tilt_adjust.axis == (0.0, -1.0, 0.0)
        and tilt_adjust.motion_limits is not None
        and tilt_adjust.motion_limits.lower is not None
        and tilt_adjust.motion_limits.upper is not None
        and tilt_adjust.motion_limits.lower < 0.0 < tilt_adjust.motion_limits.upper,
        f"Unexpected tilt joint configuration: axis={tilt_adjust.axis}, limits={tilt_adjust.motion_limits}",
    )

    ctx.expect_gap(
        base,
        arm_assembly,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="pivot_cheek_pos",
        negative_elem="pivot_collar",
        name="pivot_collar_clearance_to_positive_cheek",
    )
    ctx.expect_gap(
        arm_assembly,
        base,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="pivot_collar",
        negative_elem="pivot_cheek_neg",
        name="pivot_collar_clearance_to_negative_cheek",
    )
    ctx.expect_origin_gap(
        arm_assembly,
        base,
        axis="z",
        min_gap=1.70,
        max_gap=1.78,
        name="arm_joint_sits_at_top_of_column",
    )

    base_disc_aabb = ctx.part_element_world_aabb(base, elem="weighted_base")
    base_on_floor = base_disc_aabb is not None and abs(base_disc_aabb[0][2]) <= 1e-6
    ctx.check(
        "weighted_base_sits_on_floor",
        base_on_floor,
        f"Weighted base expected to touch z=0 floor plane, got aabb={base_disc_aabb}",
    )

    shade_aabb = ctx.part_element_world_aabb(arm_assembly, elem="shade_disc")
    shade_center = _center_from_aabb(shade_aabb)
    shade_reads_correctly = (
        shade_center is not None
        and 1.10 <= shade_center[0] <= 1.35
        and abs(shade_center[1]) <= 0.02
        and 1.55 <= shade_center[2] <= 1.72
    )
    ctx.check(
        "shade_overhang_and_height",
        shade_reads_correctly,
        f"Expected disc shade to hang forward from the base column, got center={shade_center}",
    )

    with ctx.pose({tilt_adjust: 0.38}):
        raised_shade_center = _center_from_aabb(
            ctx.part_element_world_aabb(arm_assembly, elem="shade_disc")
        )
    articulation_reads_correctly = (
        shade_center is not None
        and raised_shade_center is not None
        and raised_shade_center[2] > shade_center[2] + 0.28
        and raised_shade_center[0] < shade_center[0] - 0.04
        and abs(raised_shade_center[1] - shade_center[1]) <= 0.02
    )
    ctx.check(
        "tilt_adjust_raises_arc_in_xz_plane",
        articulation_reads_correctly,
        f"Expected positive tilt to raise the shade mostly within the XZ plane, rest={shade_center}, raised={raised_shade_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
