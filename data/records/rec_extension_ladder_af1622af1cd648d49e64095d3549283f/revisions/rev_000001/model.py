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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rung_positions(
    *,
    length: float,
    first_z: float,
    spacing: float,
    top_margin: float,
) -> list[float]:
    positions: list[float] = []
    z = first_z
    max_z = length - top_margin
    while z <= max_z + 1e-9:
        positions.append(z)
        z += spacing
    return positions


def _add_channel_rail(
    part,
    *,
    name_prefix: str,
    x_center: float,
    y_center: float,
    z0: float,
    length: float,
    width: float,
    depth: float,
    web_thickness: float,
    flange_thickness: float,
    outer_face_sign: float,
    material,
) -> dict[str, float]:
    web_x = x_center + outer_face_sign * (width * 0.5 - web_thickness * 0.5)
    rail_z = z0 + length * 0.5
    top_y = y_center + depth * 0.5 - flange_thickness * 0.5
    bottom_y = y_center - depth * 0.5 + flange_thickness * 0.5

    part.visual(
        Box((web_thickness, depth, length)),
        origin=Origin(xyz=(web_x, y_center, rail_z)),
        material=material,
        name=f"{name_prefix}_web",
    )
    part.visual(
        Box((width, flange_thickness, length)),
        origin=Origin(xyz=(x_center, top_y, rail_z)),
        material=material,
        name=f"{name_prefix}_top_flange",
    )
    part.visual(
        Box((width, flange_thickness, length)),
        origin=Origin(xyz=(x_center, bottom_y, rail_z)),
        material=material,
        name=f"{name_prefix}_bottom_flange",
    )

    return {
        "inner_x": x_center - outer_face_sign * width * 0.5,
        "outer_x": x_center + outer_face_sign * width * 0.5,
        "front_y": y_center + depth * 0.5,
        "back_y": y_center - depth * 0.5,
    }


def _add_rungs(
    part,
    *,
    name_prefix: str,
    rung_positions: list[float],
    rung_length: float,
    y_center: float,
    radius: float,
    material,
) -> None:
    for index, z in enumerate(rung_positions):
        part.visual(
            Cylinder(radius=radius, length=rung_length),
            origin=Origin(xyz=(0.0, y_center, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=f"{name_prefix}_{index:02d}",
        )


def _build_roof_hook_mesh(span: float):
    hook_radius = 0.010
    half_span = span * 0.5

    left_hook = tube_from_spline_points(
        [
            (-half_span, 0.030, 0.028),
            (-half_span, 0.038, 0.16),
            (-half_span, 0.060, 0.30),
            (-half_span, 0.102, 0.43),
            (-half_span, 0.172, 0.53),
            (-half_span, 0.252, 0.50),
            (-half_span, 0.272, 0.39),
        ],
        radius=hook_radius,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(1.0, 0.0, 0.0),
    )
    right_hook = tube_from_spline_points(
        [
            (half_span, 0.030, 0.028),
            (half_span, 0.038, 0.16),
            (half_span, 0.060, 0.30),
            (half_span, 0.102, 0.43),
            (half_span, 0.172, 0.53),
            (half_span, 0.252, 0.50),
            (half_span, 0.272, 0.39),
        ],
        radius=hook_radius,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(1.0, 0.0, 0.0),
    )
    upper_crossbar = (
        CylinderGeometry(radius=0.009, height=span + 0.06, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.20, 0.45)
    )
    mid_crossbar = (
        CylinderGeometry(radius=0.007, height=span + 0.04, radial_segments=16)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.10, 0.24)
    )

    return mesh_from_geometry(
        left_hook.merge(right_hook).merge(upper_crossbar).merge(mid_crossbar),
        "roof_hook_frame",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_hook_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.79, 0.80, 0.82, 1.0))
    aluminum_dark = model.material("aluminum_dark", rgba=(0.63, 0.65, 0.68, 1.0))
    polymer_black = model.material("polymer_black", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    base_length = 3.45
    fly_length = 3.25
    stored_fly_bottom = 0.88
    max_extension = 1.10

    base_rail_width = 0.046
    base_rail_depth = 0.030
    base_outer_width = 0.420
    base_rail_center = base_outer_width * 0.5 - base_rail_width * 0.5

    fly_rail_width = 0.040
    fly_rail_depth = 0.022
    fly_outer_width = 0.380
    fly_rail_center = fly_outer_width * 0.5 - fly_rail_width * 0.5

    hook_frame_mesh = _build_roof_hook_mesh(span=0.26)

    base = model.part("base_section")
    left_base_rail = _add_channel_rail(
        base,
        name_prefix="left_base_rail",
        x_center=-base_rail_center,
        y_center=0.0,
        z0=0.0,
        length=base_length,
        width=base_rail_width,
        depth=base_rail_depth,
        web_thickness=0.004,
        flange_thickness=0.003,
        outer_face_sign=-1.0,
        material=aluminum,
    )
    right_base_rail = _add_channel_rail(
        base,
        name_prefix="right_base_rail",
        x_center=base_rail_center,
        y_center=0.0,
        z0=0.0,
        length=base_length,
        width=base_rail_width,
        depth=base_rail_depth,
        web_thickness=0.004,
        flange_thickness=0.003,
        outer_face_sign=1.0,
        material=aluminum,
    )

    _add_rungs(
        base,
        name_prefix="base_rung",
        rung_positions=_rung_positions(length=base_length, first_z=0.34, spacing=0.28, top_margin=0.18),
        rung_length=right_base_rail["inner_x"] - left_base_rail["inner_x"],
        y_center=0.0,
        radius=0.016,
        material=aluminum_dark,
    )

    for sign in (-1.0, 1.0):
        base.visual(
            Box((0.056, 0.040, 0.020)),
            origin=Origin(xyz=(sign * base_rail_center, 0.0, 0.010)),
            material=rubber_black,
            name=f"foot_pad_{'left' if sign < 0 else 'right'}",
        )

    guide_block_y = base_rail_depth * 0.5 + 0.007
    for guide_index, z in enumerate((1.18, 2.12, 2.96)):
        for sign in (-1.0, 1.0):
            base.visual(
                Box((0.024, 0.014, 0.105)),
                origin=Origin(xyz=(sign * 0.175, guide_block_y, z)),
                material=polymer_black,
                name=f"guide_block_{guide_index}_{'left' if sign < 0 else 'right'}",
            )

    base.inertial = Inertial.from_geometry(
        Box((base_outer_width, 0.070, base_length)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, base_length * 0.5)),
    )

    fly = model.part("fly_section")
    left_fly_rail = _add_channel_rail(
        fly,
        name_prefix="left_fly_rail",
        x_center=-fly_rail_center,
        y_center=0.0,
        z0=0.0,
        length=fly_length,
        width=fly_rail_width,
        depth=fly_rail_depth,
        web_thickness=0.004,
        flange_thickness=0.003,
        outer_face_sign=-1.0,
        material=aluminum,
    )
    right_fly_rail = _add_channel_rail(
        fly,
        name_prefix="right_fly_rail",
        x_center=fly_rail_center,
        y_center=0.0,
        z0=0.0,
        length=fly_length,
        width=fly_rail_width,
        depth=fly_rail_depth,
        web_thickness=0.004,
        flange_thickness=0.003,
        outer_face_sign=1.0,
        material=aluminum,
    )

    _add_rungs(
        fly,
        name_prefix="fly_rung",
        rung_positions=_rung_positions(length=fly_length, first_z=0.27, spacing=0.28, top_margin=0.15),
        rung_length=right_fly_rail["inner_x"] - left_fly_rail["inner_x"],
        y_center=0.0,
        radius=0.015,
        material=aluminum_dark,
    )

    fly.visual(
        Box((0.342, 0.010, 0.048)),
        origin=Origin(xyz=(0.0, -0.017, fly_length - 0.090)),
        material=aluminum_dark,
        name="hook_mount_bar",
    )
    for sign in (-1.0, 1.0):
        fly.visual(
            Box((0.010, 0.036, 0.075)),
            origin=Origin(xyz=(sign * 0.176, 0.006, fly_length - 0.090)),
            material=aluminum_dark,
            name=f"hook_bracket_{'left' if sign < 0 else 'right'}",
        )

    fly.inertial = Inertial.from_geometry(
        Box((fly_outer_width, 0.060, fly_length)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, fly_length * 0.5)),
    )

    roof_hook = model.part("roof_hook")
    roof_hook.visual(
        Cylinder(radius=0.012, length=0.362),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum_dark,
        name="pivot_axle",
    )
    roof_hook.visual(
        Box((0.280, 0.024, 0.070)),
        origin=Origin(xyz=(0.0, 0.018, 0.040)),
        material=aluminum_dark,
        name="pivot_carrier",
    )
    roof_hook.visual(
        hook_frame_mesh,
        material=aluminum,
        name="hook_frame",
    )
    roof_hook.visual(
        Cylinder(radius=0.028, length=0.32),
        origin=Origin(xyz=(0.0, 0.16, 0.44), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polymer_black,
        name="roof_roller",
    )
    roof_hook.visual(
        Cylinder(radius=0.008, length=0.30),
        origin=Origin(xyz=(0.0, 0.16, 0.44), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum_dark,
        name="roller_axle",
    )
    for sign in (-1.0, 1.0):
        roof_hook.visual(
            Box((0.018, 0.024, 0.180)),
            origin=Origin(xyz=(sign * 0.130, 0.160, 0.400)),
            material=aluminum_dark,
            name=f"roller_bracket_{'left' if sign < 0 else 'right'}",
        )
    roof_hook.inertial = Inertial.from_geometry(
        Box((0.34, 0.28, 0.56)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.12, 0.26)),
    )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.040, stored_fly_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=0.45,
            lower=0.0,
            upper=max_extension,
        ),
    )
    model.articulation(
        "fly_to_roof_hook",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=roof_hook,
        origin=Origin(xyz=(0.0, 0.0, fly_length - 0.090)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=-0.15,
            upper=2.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    roof_hook = object_model.get_part("roof_hook")
    base_to_fly = object_model.get_articulation("base_to_fly")
    fly_to_roof_hook = object_model.get_articulation("fly_to_roof_hook")

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
    ctx.allow_overlap(
        fly,
        roof_hook,
        elem_a="hook_bracket_left",
        elem_b="pivot_axle",
        reason="Roof hook pivots on an axle passing through the left clevis plate.",
    )
    ctx.allow_overlap(
        fly,
        roof_hook,
        elem_a="hook_bracket_right",
        elem_b="pivot_axle",
        reason="Roof hook pivots on an axle passing through the right clevis plate.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "fly_joint_axis_is_vertical",
        tuple(base_to_fly.axis) == (0.0, 0.0, 1.0),
        f"Expected prismatic axis (0, 0, 1), got {base_to_fly.axis}.",
    )
    ctx.check(
        "roof_hook_axis_runs_across_ladder",
        tuple(abs(value) for value in fly_to_roof_hook.axis) == (1.0, 0.0, 0.0),
        f"Expected roof-hook axis to run along ladder width, got {fly_to_roof_hook.axis}.",
    )

    ctx.expect_contact(fly, base, contact_tol=0.002, name="fly_section_runs_in_base_guides")
    ctx.expect_within(fly, base, axes="x", margin=0.0, name="fly_width_stays_within_base_width")
    ctx.expect_overlap(fly, base, axes="x", min_overlap=0.30, name="sections_share_ladder_width")

    base_aabb = ctx.part_world_aabb(base)
    fly_rest_aabb = ctx.part_world_aabb(fly)
    hook_rest_aabb = ctx.part_world_aabb(roof_hook)
    assert base_aabb is not None
    assert fly_rest_aabb is not None
    assert hook_rest_aabb is not None

    with ctx.pose({base_to_fly: 1.0}):
        fly_extended_aabb = ctx.part_world_aabb(fly)
        assert fly_extended_aabb is not None
        ctx.check(
            "fly_extends_above_base",
            fly_extended_aabb[1][2] > base_aabb[1][2] + 0.75,
            (
                f"Expected fly top above base top by at least 0.75 m, got "
                f"{fly_extended_aabb[1][2] - base_aabb[1][2]:.3f} m."
            ),
        )
        ctx.expect_contact(fly, base, contact_tol=0.002, name="fly_remains_guided_when_extended")

    with ctx.pose({fly_to_roof_hook: 1.75}):
        hook_deployed_aabb = ctx.part_world_aabb(roof_hook)
        assert hook_deployed_aabb is not None
        ctx.check(
            "roof_hook_deploys_forward",
            hook_deployed_aabb[1][1] > hook_rest_aabb[1][1] + 0.22,
            (
                f"Expected deployed hook to swing forward by > 0.22 m, got "
                f"{hook_deployed_aabb[1][1] - hook_rest_aabb[1][1]:.3f} m."
            ),
        )

    with ctx.pose({base_to_fly: 1.0, fly_to_roof_hook: 1.75}):
        fly_extended_aabb = ctx.part_world_aabb(fly)
        hook_deployed_aabb = ctx.part_world_aabb(roof_hook)
        assert fly_extended_aabb is not None
        assert hook_deployed_aabb is not None
        ctx.check(
            "deployed_hook_projects_beyond_fly",
            hook_deployed_aabb[1][1] > fly_extended_aabb[1][1] + 0.18,
            (
                f"Expected deployed hook to project beyond fly front by > 0.18 m, got "
                f"{hook_deployed_aabb[1][1] - fly_extended_aabb[1][1]:.3f} m."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
