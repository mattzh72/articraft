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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_profile(radius: float, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _pipe_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [_circle_profile(inner_radius)],
            height=length,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pot_filler_faucet")

    brass = model.material("brass", rgba=(0.79, 0.68, 0.45, 1.0))
    bronze_shadow = model.material("bronze_shadow", rgba=(0.40, 0.31, 0.18, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.18, 1.0))

    stub_pipe_mesh = _pipe_mesh(
        outer_radius=0.0115,
        inner_radius=0.0085,
        length=0.050,
        name="pot_filler_stub_pipe",
    )
    arm_pipe_mesh = _pipe_mesh(
        outer_radius=0.0115,
        inner_radius=0.0085,
        length=0.146,
        name="pot_filler_arm_pipe",
    )
    spout_pipe_mesh = _pipe_mesh(
        outer_radius=0.0110,
        inner_radius=0.0080,
        length=0.125,
        name="pot_filler_spout_pipe",
    )
    nozzle_mesh = _pipe_mesh(
        outer_radius=0.0145,
        inner_radius=0.0095,
        length=0.024,
        name="pot_filler_nozzle_shell",
    )
    lever_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.005, 0.000, 0.000),
                (0.005, 0.016, -0.004),
                (0.006, 0.036, -0.011),
                (0.006, 0.061, -0.006),
            ],
            radius=0.0032,
            samples_per_segment=16,
            radial_segments=14,
        ),
        "pot_filler_valve_lever",
    )

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.255), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="escutcheon",
    )
    wall_bracket.visual(
        stub_pipe_mesh,
        origin=Origin(xyz=(0.033, 0.0, 0.255), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="wall_stub",
    )
    wall_bracket.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.058, 0.0, 0.255)),
        material=brass,
        name="elbow_body",
    )
    wall_bracket.visual(
        stub_pipe_mesh,
        origin=Origin(xyz=(0.058, 0.0, 0.280)),
        material=brass,
        name="vertical_riser",
    )
    wall_bracket.visual(
        Box((0.024, 0.008, 0.014)),
        origin=Origin(xyz=(0.057, 0.014, 0.305)),
        material=brass,
        name="knuckle_web_upper",
    )
    wall_bracket.visual(
        Box((0.024, 0.008, 0.014)),
        origin=Origin(xyz=(0.057, -0.014, 0.305)),
        material=brass,
        name="knuckle_web_lower",
    )
    wall_bracket.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.085, 0.014, 0.305), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="rear_yoke_upper",
    )
    wall_bracket.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.085, -0.014, 0.305), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="rear_yoke_lower",
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.100, 0.080, 0.120)),
        mass=1.6,
        origin=Origin(xyz=(0.050, 0.0, 0.275)),
    )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze_shadow,
        name="rear_boss",
    )
    inner_arm.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0215, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="rear_collar",
    )
    inner_arm.visual(
        arm_pipe_mesh,
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="arm_tube",
    )
    inner_arm.visual(
        Box((0.022, 0.008, 0.012)),
        origin=Origin(xyz=(0.166, 0.014, 0.0)),
        material=brass,
        name="front_web_upper",
    )
    inner_arm.visual(
        Box((0.022, 0.008, 0.012)),
        origin=Origin(xyz=(0.166, -0.014, 0.0)),
        material=brass,
        name="front_web_lower",
    )
    inner_arm.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.190, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="front_yoke_upper",
    )
    inner_arm.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.190, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="front_yoke_lower",
    )
    inner_arm.inertial = Inertial.from_geometry(
        Box((0.210, 0.060, 0.060)),
        mass=1.0,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze_shadow,
        name="rear_boss",
    )
    outer_arm.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0215, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="rear_collar",
    )
    outer_arm.visual(
        arm_pipe_mesh,
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="arm_tube",
    )
    outer_arm.visual(
        Cylinder(radius=0.014, length=0.032),
        origin=Origin(xyz=(0.174, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="tip_collar",
    )
    outer_arm.inertial = Inertial.from_geometry(
        Box((0.205, 0.060, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(0.096, 0.0, 0.0)),
    )

    spout_body = model.part("spout_body")
    spout_body.visual(
        Cylinder(radius=0.013, length=0.026),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="inlet_collar",
    )
    spout_body.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=brass,
        name="tee_body",
    )
    spout_body.visual(
        spout_pipe_mesh,
        origin=Origin(xyz=(0.026, 0.0, -0.0625)),
        material=brass,
        name="spout_tube",
    )
    spout_body.visual(
        nozzle_mesh,
        origin=Origin(xyz=(0.026, 0.0, -0.137)),
        material=brass,
        name="nozzle_shell",
    )
    spout_body.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(
            xyz=(0.021, 0.0135, -0.020),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bronze_shadow,
        name="lever_mount_boss",
    )
    spout_body.inertial = Inertial.from_geometry(
        Box((0.070, 0.060, 0.180)),
        mass=0.7,
        origin=Origin(xyz=(0.026, 0.0, -0.070)),
    )

    valve_lever = model.part("valve_lever")
    valve_lever.visual(
        Cylinder(radius=0.0055, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze_shadow,
        name="lever_hub",
    )
    valve_lever.visual(
        lever_mesh,
        material=brass,
        name="lever_arm",
    )
    valve_lever.visual(
        Cylinder(radius=0.0042, length=0.016),
        origin=Origin(xyz=(0.006, 0.068, -0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="lever_grip",
    )
    valve_lever.inertial = Inertial.from_geometry(
        Box((0.018, 0.090, 0.020)),
        mass=0.06,
        origin=Origin(xyz=(0.006, 0.040, -0.006)),
    )

    model.articulation(
        "bracket_to_inner_arm",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=inner_arm,
        origin=Origin(xyz=(0.085, 0.0, 0.305)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-1.25,
            upper=1.25,
        ),
    )
    model.articulation(
        "inner_to_outer_arm",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.7,
            lower=-1.75,
            upper=1.75,
        ),
    )
    model.articulation(
        "outer_arm_to_spout",
        ArticulationType.FIXED,
        parent=outer_arm,
        child=spout_body,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
    )
    model.articulation(
        "spout_to_valve_lever",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=valve_lever,
        origin=Origin(xyz=(0.026, 0.0135, -0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-0.55,
            upper=0.75,
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

    wall_bracket = object_model.get_part("wall_bracket")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    spout_body = object_model.get_part("spout_body")
    valve_lever = object_model.get_part("valve_lever")

    bracket_to_inner = object_model.get_articulation("bracket_to_inner_arm")
    inner_to_outer = object_model.get_articulation("inner_to_outer_arm")
    spout_to_lever = object_model.get_articulation("spout_to_valve_lever")

    ctx.expect_contact(wall_bracket, inner_arm, name="wall_bracket_contacts_inner_arm")
    ctx.expect_contact(inner_arm, outer_arm, name="inner_arm_contacts_outer_arm")
    ctx.expect_contact(outer_arm, spout_body, name="outer_arm_contacts_spout")
    ctx.expect_contact(spout_body, valve_lever, name="spout_contacts_valve_lever")

    ctx.expect_origin_gap(
        inner_arm,
        wall_bracket,
        axis="x",
        min_gap=0.080,
        max_gap=0.090,
        name="inner_arm_root_set_off_wall",
    )
    ctx.expect_origin_gap(
        outer_arm,
        inner_arm,
        axis="x",
        min_gap=0.185,
        max_gap=0.195,
        name="outer_arm_knuckle_spacing",
    )
    ctx.expect_origin_gap(
        spout_body,
        outer_arm,
        axis="x",
        min_gap=0.185,
        max_gap=0.195,
        name="spout_mounted_at_arm_tip",
    )

    def _axis_aligned(axis: tuple[float, float, float], target: tuple[float, float, float]) -> bool:
        return all(abs(abs(a) - abs(b)) < 1e-9 for a, b in zip(axis, target))

    ctx.check(
        "arm_knuckles_are_horizontal_revolutes",
        _axis_aligned(bracket_to_inner.axis, (0.0, 1.0, 0.0))
        and _axis_aligned(inner_to_outer.axis, (0.0, 1.0, 0.0)),
        details=f"axes={bracket_to_inner.axis!r}, {inner_to_outer.axis!r}",
    )
    ctx.check(
        "valve_lever_axis_is_side_pivot",
        _axis_aligned(spout_to_lever.axis, (1.0, 0.0, 0.0)),
        details=f"axis={spout_to_lever.axis!r}",
    )

    spout_rest = ctx.part_world_position(spout_body)
    outer_rest = ctx.part_world_position(outer_arm)
    lever_rest = ctx.part_element_world_aabb(valve_lever, elem="lever_grip")
    if spout_rest is None or outer_rest is None or lever_rest is None:
        ctx.fail("probeable_pose_data_available", "expected pose query data for spout, outer arm, and lever grip")
    else:
        with ctx.pose({bracket_to_inner: -0.75}):
            outer_raised = ctx.part_world_position(outer_arm)
            if outer_raised is None:
                ctx.fail("outer_arm_moves_with_base_knuckle", "outer arm world position unavailable in raised pose")
            else:
                ctx.check(
                    "outer_arm_moves_with_base_knuckle",
                    outer_raised[2] > outer_rest[2] + 0.11,
                    details=f"rest_z={outer_rest[2]:.4f}, raised_z={outer_raised[2]:.4f}",
                )

        with ctx.pose({inner_to_outer: 1.15}):
            spout_folded = ctx.part_world_position(spout_body)
            if spout_folded is None:
                ctx.fail("spout_moves_with_elbow_knuckle", "spout world position unavailable in folded pose")
            else:
                ctx.check(
                    "spout_moves_with_elbow_knuckle",
                    spout_folded[0] < spout_rest[0] - 0.09
                    and spout_folded[2] < spout_rest[2] - 0.13,
                    details=(
                        f"rest=({spout_rest[0]:.4f},{spout_rest[2]:.4f}), "
                        f"folded=({spout_folded[0]:.4f},{spout_folded[2]:.4f})"
                    ),
                )

        with ctx.pose({spout_to_lever: 0.60}):
            lever_open = ctx.part_element_world_aabb(valve_lever, elem="lever_grip")
            if lever_open is None:
                ctx.fail("valve_lever_moves", "lever grip AABB unavailable in open pose")
            else:
                ctx.check(
                    "valve_lever_moves",
                    lever_open[1][2] > lever_rest[1][2] + 0.015,
                    details=f"rest_top_z={lever_rest[1][2]:.4f}, open_top_z={lever_open[1][2]:.4f}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
