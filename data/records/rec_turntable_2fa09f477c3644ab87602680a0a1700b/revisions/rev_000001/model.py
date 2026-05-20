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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def annulus_mesh(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    *,
    z_center: float = 0.0,
    segments: int = 64,
):
    geom = ExtrudeWithHolesGeometry(
        circle_profile(outer_radius, segments),
        [circle_profile(inner_radius, segments)],
        thickness,
    )
    geom.translate(0.0, 0.0, z_center)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_turntable")

    model.material("plinth_abs", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("platter_metal", rgba=(0.68, 0.69, 0.72, 1.0))
    model.material("mat_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("tonearm_finish", rgba=(0.60, 0.62, 0.66, 1.0))
    model.material("cartridge_dark", rgba=(0.10, 0.10, 0.10, 1.0))

    plinth_width = 0.45
    plinth_depth = 0.36
    plinth_height = 0.042

    platter_center = (-0.045, 0.0)
    platter_outer_radius = 0.152
    platter_hole_radius = 0.0048
    platter_support_z = 0.046

    tonearm_pivot = (0.165, 0.050, 0.060)
    tonearm_post_radius = 0.005
    tonearm_base_outer_radius = 0.020
    tonearm_base_hole_radius = 0.0065
    tonearm_angle = math.pi + 0.25
    tonearm_dir = (math.cos(tonearm_angle), math.sin(tonearm_angle))
    rear_dir = (-tonearm_dir[0], -tonearm_dir[1])

    plinth = model.part("plinth")
    plinth_shell = ExtrudeGeometry(
        rounded_rect_profile(plinth_width, plinth_depth, 0.020, corner_segments=8),
        plinth_height,
    )
    plinth.visual(
        mesh_from_geometry(plinth_shell, "plinth_shell"),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material="plinth_abs",
        name="plinth_shell",
    )
    plinth.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.044)),
        material="plinth_abs",
        name="bearing_pad",
    )
    plinth.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.044)),
        material="plinth_abs",
        name="bearing_collar",
    )
    plinth.visual(
        Cylinder(radius=0.003, length=0.012),
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.045)),
        material="platter_metal",
        name="spindle_post",
    )
    plinth.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], 0.051)),
        material="plinth_abs",
        name="tonearm_pedestal",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((plinth_width, plinth_depth, plinth_height)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
    )

    platter = model.part("platter")
    platter_body_geom = annulus_mesh(
        platter_outer_radius,
        platter_hole_radius,
        0.008,
        z_center=0.007,
    )
    platter_body_geom.merge(
        annulus_mesh(
            platter_outer_radius,
            0.135,
            0.006,
            z_center=0.011,
        )
    )
    platter_body_geom.merge(
        annulus_mesh(
            0.022,
            platter_hole_radius,
            0.006,
            z_center=0.003,
        )
    )
    platter_mat_geom = annulus_mesh(
        0.142,
        0.014,
        0.0025,
        z_center=0.01525,
    )
    platter.visual(
        mesh_from_geometry(platter_body_geom, "platter_body"),
        material="platter_metal",
        name="platter_hub",
    )
    platter.visual(
        mesh_from_geometry(platter_mat_geom, "platter_mat"),
        material="mat_rubber",
        name="platter_mat",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=platter_outer_radius, length=0.0165),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.00825)),
    )

    tonearm = model.part("tonearm")
    tonearm_base_geom = annulus_mesh(
        tonearm_base_outer_radius,
        tonearm_base_hole_radius,
        0.005,
        z_center=0.0025,
    )
    tonearm.visual(
        mesh_from_geometry(tonearm_base_geom, "tonearm_base"),
        material="tonearm_finish",
        name="tonearm_base",
    )
    tonearm.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material="tonearm_finish",
        name="pivot_housing",
    )
    tonearm.visual(
        Cylinder(radius=0.006, length=0.205),
        origin=Origin(
            xyz=(tonearm_dir[0] * 0.1025, tonearm_dir[1] * 0.1025, 0.018),
            rpy=(0.0, math.pi * 0.5, tonearm_angle),
        ),
        material="tonearm_finish",
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.024, 0.018, 0.006)),
        origin=Origin(
            xyz=(tonearm_dir[0] * 0.210, tonearm_dir[1] * 0.210, 0.014),
            rpy=(0.0, 0.0, tonearm_angle),
        ),
        material="tonearm_finish",
        name="headshell",
    )
    tonearm.visual(
        Box((0.014, 0.014, 0.010)),
        origin=Origin(
            xyz=(tonearm_dir[0] * 0.221, tonearm_dir[1] * 0.221, 0.015),
            rpy=(0.0, 0.0, tonearm_angle),
        ),
        material="cartridge_dark",
        name="cartridge_body",
    )
    tonearm.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(
            xyz=(rear_dir[0] * 0.020, rear_dir[1] * 0.020, 0.019),
            rpy=(0.0, math.pi * 0.5, tonearm_angle),
        ),
        material="tonearm_finish",
        name="counterweight",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.255, 0.050, 0.040)),
        mass=0.32,
        origin=Origin(xyz=(tonearm_dir[0] * 0.085, tonearm_dir[1] * 0.085, 0.018)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_center[0], platter_center[1], platter_support_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=12.0),
    )
    model.articulation(
        "tonearm_swivel",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=tonearm_pivot),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.0,
            lower=-0.30,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swivel = object_model.get_articulation("tonearm_swivel")

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
        "turntable_parts_present",
        all(part is not None for part in (plinth, platter, tonearm)),
        "Expected plinth, platter, and tonearm parts to exist.",
    )
    ctx.check(
        "platter_articulation_is_vertical_continuous",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(platter_spin.axis[0]) < 1e-9
        and abs(platter_spin.axis[1]) < 1e-9
        and abs(platter_spin.axis[2] - 1.0) < 1e-9
        and platter_spin.motion_limits is not None
        and platter_spin.motion_limits.lower is None
        and platter_spin.motion_limits.upper is None,
        "Platter should rotate as a continuous vertical spindle joint.",
    )
    ctx.check(
        "tonearm_articulation_is_vertical_revolute",
        tonearm_swivel.articulation_type == ArticulationType.REVOLUTE
        and abs(tonearm_swivel.axis[0]) < 1e-9
        and abs(tonearm_swivel.axis[1]) < 1e-9
        and abs(tonearm_swivel.axis[2] - 1.0) < 1e-9
        and tonearm_swivel.motion_limits is not None
        and tonearm_swivel.motion_limits.lower is not None
        and tonearm_swivel.motion_limits.upper is not None
        and tonearm_swivel.motion_limits.lower < 0.0
        and tonearm_swivel.motion_limits.upper > 0.0,
        "Tonearm should pivot on a bounded vertical stage joint.",
    )

    ctx.expect_contact(
        platter,
        plinth,
        elem_a="platter_hub",
        elem_b="bearing_collar",
        contact_tol=5e-4,
        name="platter_hub_supported_by_bearing_collar",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_hub",
        elem_b="bearing_collar",
        min_overlap=0.020,
        name="platter_hub_centered_on_bearing",
    )
    ctx.expect_contact(
        tonearm,
        plinth,
        elem_a="tonearm_base",
        elem_b="tonearm_pedestal",
        contact_tol=5e-4,
        name="tonearm_base_supported_by_pedestal",
    )
    ctx.expect_overlap(
        tonearm,
        plinth,
        axes="xy",
        elem_a="tonearm_base",
        elem_b="tonearm_pedestal",
        min_overlap=0.030,
        name="tonearm_base_centered_on_pivot_pedestal",
    )

    with ctx.pose({tonearm_swivel: 0.0}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="headshell",
            elem_b="platter_mat",
            min_overlap=0.010,
            name="headshell_reaches_record_area",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge_body",
            negative_elem="platter_mat",
            min_gap=0.004,
            max_gap=0.020,
            name="cartridge_clears_record_surface_at_nominal_pose",
        )

    with ctx.pose({tonearm_swivel: tonearm_swivel.motion_limits.upper}):
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge_body",
            negative_elem="platter_mat",
            min_gap=0.004,
            max_gap=0.020,
            name="cartridge_clears_record_surface_at_inner_pose",
        )

    with ctx.pose({tonearm_swivel: tonearm_swivel.motion_limits.lower}):
        low_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({tonearm_swivel: tonearm_swivel.motion_limits.upper}):
        high_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")

    low_y = None if low_aabb is None else 0.5 * (low_aabb[0][1] + low_aabb[1][1])
    high_y = None if high_aabb is None else 0.5 * (high_aabb[0][1] + high_aabb[1][1])
    ctx.check(
        "tonearm_sweeps_across_record_arc",
        low_y is not None and high_y is not None and high_y < low_y - 0.10,
        "Tonearm headshell should traverse a meaningful arc across the record area.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
