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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return superellipse_profile(
        radius * 2.0,
        radius * 2.0,
        exponent=2.0,
        segments=segments,
    )


def _ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 48,
):
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments)],
            thickness,
            center=True,
        ),
    )


def _build_glass_barrel_mesh():
    return _save_mesh(
        "glass_barrel_mesh",
        LatheGeometry.from_shell_profiles(
            [
                (0.0115, -0.053),
                (0.0115, 0.030),
            ],
            [
                (0.0093, -0.053),
                (0.0093, 0.030),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _build_nozzle_shell_mesh():
    return _save_mesh(
        "nozzle_shell_mesh",
        LatheGeometry.from_shell_profiles(
            [
                (0.0032, -0.092),
                (0.0038, -0.084),
                (0.0052, -0.076),
                (0.0080, -0.067),
                (0.0110, -0.058),
                (0.0115, -0.052),
            ],
            [
                (0.0012, -0.092),
                (0.0015, -0.084),
                (0.0023, -0.076),
                (0.0040, -0.067),
                (0.0082, -0.058),
                (0.0093, -0.052),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_syringe")

    glass = model.material("glass", rgba=(0.82, 0.90, 0.96, 0.34))
    brushed_steel = model.material("brushed_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.20, 0.22, 0.25, 1.0))
    adapter_bronze = model.material("adapter_bronze", rgba=(0.58, 0.50, 0.34, 1.0))
    seal_black = model.material("seal_black", rgba=(0.08, 0.09, 0.10, 1.0))

    barrel = model.part("barrel_assembly")
    barrel.visual(
        _build_glass_barrel_mesh(),
        material=glass,
        name="glass_barrel",
    )
    barrel.visual(
        _build_nozzle_shell_mesh(),
        material=brushed_steel,
        name="nozzle_tip",
    )
    barrel.visual(
        _ring_mesh(
            "front_adapter_ring",
            outer_radius=0.0145,
            inner_radius=0.0098,
            thickness=0.011,
            segments=52,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0575)),
        material=adapter_bronze,
        name="front_adapter",
    )
    barrel.visual(
        _ring_mesh(
            "front_stop_ring",
            outer_radius=0.0111,
            inner_radius=0.0022,
            thickness=0.006,
            segments=48,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0505)),
        material=brushed_steel,
        name="front_stop",
    )
    barrel.visual(
        _ring_mesh(
            "rear_service_sleeve",
            outer_radius=0.0138,
            inner_radius=0.0112,
            thickness=0.033,
            segments=52,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_oxide,
        name="rear_sleeve",
    )
    barrel.visual(
        _ring_mesh(
            "rear_cap_ring",
            outer_radius=0.0145,
            inner_radius=0.0034,
            thickness=0.007,
            segments=52,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=brushed_steel,
        name="rear_cap",
    )
    for sign, name in ((-1.0, "adapter_rib_left"), (1.0, "adapter_rib_right")):
        barrel.visual(
            Box((0.0042, 0.0052, 0.013)),
            origin=Origin(xyz=(sign * 0.0112, 0.0, -0.056)),
            material=adapter_bronze,
            name=name,
        )
    for sign, name in ((-1.0, "finger_wing_left"), (1.0, "finger_wing_right")):
        barrel.visual(
            Box((0.018, 0.010, 0.004)),
            origin=Origin(xyz=(sign * 0.0245, 0.0, 0.068)),
            material=brushed_steel,
            name=name,
        )
        barrel.visual(
            Box((0.006, 0.008, 0.012)),
            origin=Origin(xyz=(sign * 0.016, 0.0, 0.060)),
            material=dark_oxide,
            name=f"{name}_brace",
        )
    for sign, name in ((-1.0, "service_hatch_neg_y"), (1.0, "service_hatch_pos_y")):
        barrel.visual(
            Box((0.018, 0.0024, 0.016)),
            origin=Origin(xyz=(0.0, sign * 0.0149, 0.045)),
            material=dark_oxide,
            name=name,
        )
        for x_pos, z_pos, bolt_idx in ((-0.005, 0.040, 0), (0.005, 0.050, 1)):
            barrel.visual(
                Cylinder(radius=0.00115, length=0.005),
                origin=Origin(
                    xyz=(x_pos, sign * 0.0163, z_pos),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=brushed_steel,
                name=f"{name}_bolt_{bolt_idx}",
            )
    for x_pos, y_pos, bolt_name in (
        (0.0088, 0.0, "adapter_bolt_pos_x"),
        (-0.0088, 0.0, "adapter_bolt_neg_x"),
        (0.0, 0.0088, "adapter_bolt_pos_y"),
        (0.0, -0.0088, "adapter_bolt_neg_y"),
    ):
        barrel.visual(
            Cylinder(radius=0.0013, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, -0.065), rpy=(0.0, 0.0, 0.0)),
            material=brushed_steel,
            name=bolt_name,
        )
    for index in range(11):
        z_pos = -0.036 + index * 0.007
        if index in (0, 5, 10):
            tick_width = 0.0064
        elif index % 2 == 0:
            tick_width = 0.0049
        else:
            tick_width = 0.0036
        barrel.visual(
            Box((0.0008, tick_width, 0.0014)),
            origin=Origin(xyz=(0.0111, 0.0, z_pos)),
            material=dark_oxide,
            name=f"graduation_{index:02d}",
        )
    barrel.inertial = Inertial.from_geometry(
        Box((0.070, 0.045, 0.165)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0090, length=0.007),
        material=seal_black,
        name="piston_head",
    )
    plunger.visual(
        Cylinder(radius=0.0064, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=brushed_steel,
        name="piston_backup",
    )
    plunger.visual(
        Cylinder(radius=0.0026, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=brushed_steel,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0058, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_oxide,
        name="rear_stop_collar",
    )
    plunger.visual(
        Cylinder(radius=0.0046, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=dark_oxide,
        name="thumb_hub",
    )
    plunger.visual(
        Cylinder(radius=0.0026, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.126), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="thumb_crossbar",
    )
    plunger.visual(
        _save_mesh(
            "thumb_ring_mesh",
            TorusGeometry(
                radius=0.014,
                tube=0.0026,
                radial_segments=16,
                tubular_segments=48,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=brushed_steel,
        name="thumb_ring",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.145)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.18,
            lower=0.0,
            upper=0.086,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel_assembly")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")

    glass_barrel = barrel.get_visual("glass_barrel")
    front_stop = barrel.get_visual("front_stop")
    rear_cap = barrel.get_visual("rear_cap")
    piston_head = plunger.get_visual("piston_head")
    rear_stop_collar = plunger.get_visual("rear_stop_collar")

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
    ctx.warn_if_articulation_overlaps(max_pose_samples=8)

    ctx.expect_origin_distance(
        plunger,
        barrel,
        axes="xy",
        max_dist=0.0005,
        name="plunger_origin_stays_on_barrel_axis",
    )

    stroke = 0.086
    limits = slide.motion_limits
    if limits is None or limits.upper is None:
        ctx.fail("plunger_has_bounded_travel", "plunger_slide is missing an upper travel limit")
    else:
        stroke = limits.upper

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            plunger,
            barrel,
            axis="z",
            positive_elem=piston_head,
            negative_elem=front_stop,
            max_gap=0.0006,
            max_penetration=0.0,
            name="front_stop_seats_when_plunger_is_depressed",
        )
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            inner_elem=piston_head,
            outer_elem=glass_barrel,
            margin=0.0018,
            name="piston_stays_centered_in_barrel_when_depressed",
        )
        depressed_pos = ctx.part_world_position(plunger)

    with ctx.pose({slide: stroke}):
        ctx.expect_gap(
            barrel,
            plunger,
            axis="z",
            positive_elem=rear_cap,
            negative_elem=rear_stop_collar,
            max_gap=0.0006,
            max_penetration=0.0,
            name="rear_stop_collar_seats_against_cap_when_retracted",
        )
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            inner_elem=rear_stop_collar,
            outer_elem=glass_barrel,
            margin=0.0030,
            name="rear_stop_collar_remains_coaxial_when_retracted",
        )
        retracted_pos = ctx.part_world_position(plunger)

    if depressed_pos is None or retracted_pos is None:
        ctx.fail("plunger_pose_positions_resolve", "could not resolve plunger world positions")
    else:
        axial_delta = retracted_pos[2] - depressed_pos[2]
        lateral_error = math.hypot(
            retracted_pos[0] - depressed_pos[0],
            retracted_pos[1] - depressed_pos[1],
        )
        ctx.check(
            "plunger_motion_is_linear_coaxial_and_matches_stroke",
            lateral_error <= 1e-6 and abs(axial_delta - stroke) <= 5e-4,
            details=(
                f"lateral_error={lateral_error:.6f}, "
                f"axial_delta={axial_delta:.6f}, stroke={stroke:.6f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
