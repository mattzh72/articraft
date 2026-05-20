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
    rounded_rect_profile,
)


def _make_lathed_shell(name: str, outer_profile, inner_profile):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _make_rounded_slab(name: str, width: float, depth: float, height: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            height,
            cap=True,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soap_dispenser_pump_bottle")

    bottle_white = model.material("bottle_white", rgba=(0.95, 0.96, 0.94, 1.0))
    collar_metal = model.material("collar_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    pump_dark = model.material("pump_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    bottle = model.part("bottle")
    bottle_outer = [
        (0.014, 0.000),
        (0.034, 0.003),
        (0.037, 0.010),
        (0.037, 0.126),
        (0.035, 0.142),
        (0.028, 0.156),
        (0.020, 0.164),
        (0.0155, 0.168),
        (0.0155, 0.188),
    ]
    bottle_inner = [
        (0.000, 0.0035),
        (0.031, 0.0065),
        (0.034, 0.012),
        (0.034, 0.124),
        (0.032, 0.140),
        (0.025, 0.154),
        (0.017, 0.162),
        (0.011, 0.168),
        (0.011, 0.184),
    ]
    bottle.visual(
        _make_lathed_shell("bottle_shell_v2", bottle_outer, bottle_inner),
        material=bottle_white,
        name="bottle_shell",
    )
    bottle.visual(
        Cylinder(radius=0.0155, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=bottle_white,
        name="neck_land",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.188),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
    )

    collar = model.part("collar")
    collar_outer = [
        (0.020, 0.000),
        (0.020, 0.006),
        (0.018, 0.016),
        (0.013, 0.022),
        (0.012, 0.026),
    ]
    collar_inner = [
        (0.0155, 0.001),
        (0.0155, 0.010),
        (0.0065, 0.010),
        (0.0065, 0.026),
    ]
    collar.visual(
        _make_lathed_shell("pump_collar_v2", collar_outer, collar_inner),
        material=collar_metal,
        name="collar_shell",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.026),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )
    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
    )

    stem = model.part("plunger_stem")
    stem.visual(
        Cylinder(radius=0.0040, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=pump_dark,
        name="stem_shaft",
    )
    stem.visual(
        Cylinder(radius=0.0065, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=pump_dark,
        name="stem_guide_sleeve",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.016, 0.016, 0.036)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "collar_to_stem",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )

    spout_head = model.part("spout_head")
    head_hub_outer = [
        (0.010, 0.000),
        (0.010, 0.006),
    ]
    head_hub_inner = [
        (0.0040, 0.000),
        (0.0040, 0.006),
    ]
    spout_head.visual(
        _make_lathed_shell("spout_head_hub_v1", head_hub_outer, head_hub_inner),
        material=pump_dark,
        name="head_hub",
    )
    spout_head.visual(
        _make_rounded_slab("pump_top_pad_v1", 0.028, 0.018, 0.004, 0.004),
        origin=Origin(xyz=(-0.003, 0.0, 0.008)),
        material=pump_dark,
        name="top_pad",
    )
    spout_head.visual(
        Cylinder(radius=0.0048, length=0.044),
        origin=Origin(xyz=(0.022, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_dark,
        name="nozzle_tube",
    )
    spout_head.visual(
        Cylinder(radius=0.0036, length=0.012),
        origin=Origin(xyz=(0.041, 0.0, 0.002)),
        material=pump_dark,
        name="nozzle_tip",
    )
    spout_head.inertial = Inertial.from_geometry(
        Box((0.050, 0.022, 0.016)),
        mass=0.04,
        origin=Origin(xyz=(0.015, 0.0, 0.008)),
    )
    model.articulation(
        "stem_to_head",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=spout_head,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    stem = object_model.get_part("plunger_stem")
    spout_head = object_model.get_part("spout_head")

    collar_to_stem = object_model.get_articulation("collar_to_stem")
    stem_to_head = object_model.get_articulation("stem_to_head")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0015)
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
        collar,
        stem,
        reason="The plunger guide sleeve intentionally telescopes inside the collar bore during pump travel.",
    )
    ctx.allow_overlap(
        stem,
        spout_head,
        reason="The spout head is retained coaxially around the stem spindle at the rotating hub.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        collar,
        bottle,
        contact_tol=0.0015,
        name="collar_seats_on_bottle_neck",
    )
    ctx.expect_contact(stem, collar, elem_a="stem_guide_sleeve", name="stem_retained_by_collar")
    ctx.expect_contact(spout_head, stem, name="spout_head_mounted_on_stem")
    ctx.expect_overlap(collar, bottle, axes="xy", min_overlap=0.028, name="collar_over_bottle_neck")
    ctx.expect_overlap(spout_head, stem, axes="xy", min_overlap=0.010, name="spout_centered_over_stem")

    ctx.check(
        "stem_prismatic_axis_vertical",
        collar_to_stem.axis == (0.0, 0.0, -1.0),
        f"axis={collar_to_stem.axis}",
    )
    ctx.check(
        "head_revolute_axis_vertical",
        stem_to_head.axis == (0.0, 0.0, 1.0),
        f"axis={stem_to_head.axis}",
    )

    bottle_aabb = ctx.part_world_aabb(bottle)
    if bottle_aabb is not None:
        bottle_dx = bottle_aabb[1][0] - bottle_aabb[0][0]
        bottle_dy = bottle_aabb[1][1] - bottle_aabb[0][1]
        bottle_dz = bottle_aabb[1][2] - bottle_aabb[0][2]
        ctx.check("bottle_realistic_height", 0.17 <= bottle_dz <= 0.20, f"height={bottle_dz:.4f}")
        ctx.check(
            "bottle_realistic_diameter",
            0.07 <= max(bottle_dx, bottle_dy) <= 0.08,
            f"dims=({bottle_dx:.4f}, {bottle_dy:.4f}, {bottle_dz:.4f})",
        )

    nozzle_rest = ctx.part_element_world_aabb(spout_head, elem="nozzle_tube")
    stem_rest_pos = ctx.part_world_position(stem)

    with ctx.pose({collar_to_stem: 0.0, stem_to_head: 0.0}):
        ctx.expect_gap(
            spout_head,
            collar,
            axis="z",
            min_gap=-0.0005,
            max_gap=0.0005,
            name="rest_head_above_collar",
        )

    if collar_to_stem.motion_limits is not None:
        lower = collar_to_stem.motion_limits.lower
        upper = collar_to_stem.motion_limits.upper
        if lower is not None:
            with ctx.pose({collar_to_stem: lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name="pump_stroke_lower_no_overlap")
                ctx.expect_contact(
                    stem,
                    collar,
                    elem_a="stem_guide_sleeve",
                    name="pump_stroke_lower_contact",
                )
        if upper is not None:
            with ctx.pose({collar_to_stem: upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name="pump_stroke_upper_no_overlap")
                ctx.expect_gap(
                    spout_head,
                    collar,
                    axis="z",
                    min_gap=-0.0045,
                    max_gap=0.0005,
                    name="pressed_head_stays_above_collar",
                )
                stem_pressed_pos = ctx.part_world_position(stem)
                if stem_rest_pos is not None and stem_pressed_pos is not None:
                    ctx.check(
                        "stem_moves_down_when_pressed",
                        stem_pressed_pos[2] < stem_rest_pos[2] - 0.0035,
                        f"rest_z={stem_rest_pos[2]:.4f}, pressed_z={stem_pressed_pos[2]:.4f}",
                    )

    if stem_to_head.motion_limits is not None:
        lower = stem_to_head.motion_limits.lower
        upper = stem_to_head.motion_limits.upper
        if lower is not None:
            with ctx.pose({stem_to_head: lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name="head_swivel_lower_no_overlap")
                ctx.fail_if_isolated_parts(contact_tol=0.0015, name="head_swivel_lower_no_floating")
                ctx.expect_contact(spout_head, stem, name="head_swivel_lower_contact")
        if upper is not None:
            with ctx.pose({stem_to_head: upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name="head_swivel_upper_no_overlap")
                ctx.fail_if_isolated_parts(contact_tol=0.0015, name="head_swivel_upper_no_floating")
                ctx.expect_contact(spout_head, stem, name="head_swivel_upper_contact")
                nozzle_swiveled = ctx.part_element_world_aabb(spout_head, elem="nozzle_tube")
                if nozzle_rest is not None and nozzle_swiveled is not None:
                    rest_dx = nozzle_rest[1][0] - nozzle_rest[0][0]
                    rest_dy = nozzle_rest[1][1] - nozzle_rest[0][1]
                    swivel_dx = nozzle_swiveled[1][0] - nozzle_swiveled[0][0]
                    swivel_dy = nozzle_swiveled[1][1] - nozzle_swiveled[0][1]
                    ctx.check(
                        "spout_rotates_about_stem_axis",
                        rest_dx > rest_dy + 0.015 and swivel_dy > swivel_dx + 0.015,
                        (
                            "rest_spans="
                            f"({rest_dx:.4f}, {rest_dy:.4f}) "
                            "swivel_spans="
                            f"({swivel_dx:.4f}, {swivel_dy:.4f})"
                        ),
                    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
