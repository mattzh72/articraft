from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_bolt(
    part,
    *,
    xyz: tuple[float, float, float],
    axis: str = "z",
    radius: float = 0.007,
    length: float = 0.018,
    material: str = "fastener",
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_roof_vent_tower")

    model.material("steel", rgba=(0.37, 0.39, 0.41, 1.0))
    model.material("galvanized", rgba=(0.55, 0.57, 0.59, 1.0))
    model.material("fastener", rgba=(0.67, 0.68, 0.71, 1.0))
    model.material("safety_yellow", rgba=(0.80, 0.69, 0.14, 1.0))

    width = 0.68
    depth = 0.56
    base_height = 0.10
    post_size = 0.06
    sill_center_z = 0.21
    sill_height = 0.06
    header_center_z = 0.60
    header_height = 0.08
    cap_center_z = 0.69
    cap_height = 0.10
    hinge_y = -0.305
    hinge_z = 0.760
    barrel_radius = 0.017

    tower = model.part("tower")
    tower.visual(
        Box((0.76, 0.64, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material="steel",
        name="mounting_flange",
    )
    tower.visual(
        Box((width, depth, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material="steel",
        name="curb_base",
    )

    post_xy = (
        (width / 2.0 - post_size / 2.0, depth / 2.0 - post_size / 2.0),
        (-width / 2.0 + post_size / 2.0, depth / 2.0 - post_size / 2.0),
        (width / 2.0 - post_size / 2.0, -depth / 2.0 + post_size / 2.0),
        (-width / 2.0 + post_size / 2.0, -depth / 2.0 + post_size / 2.0),
    )
    for i, (x, y) in enumerate(post_xy, start=1):
        tower.visual(
            Box((post_size, post_size, 0.64)),
            origin=Origin(xyz=(x, y, 0.42)),
            material="steel",
            name=f"corner_post_{i}",
        )

    inner_width = width - 2.0 * post_size
    inner_depth = depth - 2.0 * post_size
    beam_y = depth / 2.0 - post_size / 2.0
    beam_x = width / 2.0 - post_size / 2.0

    for side, y in (("front", beam_y), ("rear", -beam_y)):
        tower.visual(
            Box((inner_width, post_size, sill_height)),
            origin=Origin(xyz=(0.0, y, sill_center_z)),
            material="steel",
            name=f"{side}_sill_beam",
        )
        tower.visual(
            Box((inner_width, post_size, header_height)),
            origin=Origin(xyz=(0.0, y, header_center_z)),
            material="steel",
            name=f"{side}_header_beam",
        )
        tower.visual(
            Box((width, post_size, cap_height)),
            origin=Origin(xyz=(0.0, y, cap_center_z)),
            material="steel",
            name=f"{side}_cap_beam",
        )

    for side, x in (("right", beam_x), ("left", -beam_x)):
        tower.visual(
            Box((post_size, inner_depth, sill_height)),
            origin=Origin(xyz=(x, 0.0, sill_center_z)),
            material="steel",
            name=f"{side}_sill_beam",
        )
        tower.visual(
            Box((post_size, inner_depth, header_height)),
            origin=Origin(xyz=(x, 0.0, header_center_z)),
            material="steel",
            name=f"{side}_header_beam",
        )
        tower.visual(
            Box((post_size, inner_depth, cap_height)),
            origin=Origin(xyz=(x, 0.0, cap_center_z)),
            material="steel",
            name=f"{side}_cap_beam",
        )

    guard_length = 0.32
    guard_center_z = 0.40
    for i, x in enumerate((-0.15, 0.0, 0.15), start=1):
        tower.visual(
            Cylinder(radius=0.011, length=guard_length),
            origin=Origin(xyz=(x, beam_y, guard_center_z)),
            material="galvanized",
            name=f"front_guard_bar_{i}",
        )
    for i, x in enumerate((-0.12, 0.12), start=1):
        tower.visual(
            Cylinder(radius=0.011, length=guard_length),
            origin=Origin(xyz=(x, -beam_y, guard_center_z)),
            material="galvanized",
            name=f"rear_guard_bar_{i}",
        )
    for side, x in (("right", beam_x), ("left", -beam_x)):
        for i, y in enumerate((-0.14, 0.0, 0.14), start=1):
            tower.visual(
                Cylinder(radius=0.011, length=guard_length),
                origin=Origin(xyz=(x, y, guard_center_z)),
                material="galvanized",
                name=f"{side}_guard_bar_{i}",
            )

    for side, x in (("right", 0.19), ("left", -0.19)):
        tower.visual(
            Box((0.10, 0.018, 0.12)),
            origin=Origin(xyz=(x, -0.295, 0.700)),
            material="steel",
            name=f"{side}_hinge_plate",
        )
        tower.visual(
            Cylinder(radius=barrel_radius, length=0.20),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=_axis_rpy("x")),
            material="galvanized",
            name=f"{side}_hinge_barrel",
        )
        _add_bolt(
            tower,
            xyz=(x - 0.025, -0.280, 0.662),
            axis="y",
            name=f"{side}_hinge_bolt_lower",
        )
        _add_bolt(
            tower,
            xyz=(x + 0.025, -0.280, 0.712),
            axis="y",
            name=f"{side}_hinge_bolt_upper",
        )

    for side, x in (("right", beam_x), ("left", -beam_x)):
        tower.visual(
            Box((0.04, 0.14, 0.128)),
            origin=Origin(
                xyz=(x - 0.010 if x > 0.0 else x + 0.010, -0.290, 0.704),
            ),
            material="steel",
            name=f"{side}_stop_riser",
        )
        tower.visual(
            Box((0.10, 0.04, 0.011)),
            origin=Origin(
                xyz=(x + 0.040 if x > 0.0 else x - 0.040, -0.347, 0.7735),
            ),
            material="safety_yellow",
            name=f"{side}_stop_block",
        )

    tower.visual(
        Box((0.09, 0.012, 0.06)),
        origin=Origin(xyz=(0.0, 0.274, 0.715)),
        material="safety_yellow",
        name="lockout_keeper",
    )
    _add_bolt(tower, xyz=(-0.025, 0.275, 0.695), axis="y", name="keeper_bolt_left")
    _add_bolt(tower, xyz=(0.025, 0.275, 0.695), axis="y", name="keeper_bolt_right")

    for side, x in (("right", beam_x), ("left", -beam_x)):
        tower.visual(
            Box((0.018, 0.03, 0.14)),
            origin=Origin(
                xyz=(x, -0.255, 0.60),
                rpy=(0.60 if side == "right" else -0.60, 0.0, 0.0),
            ),
            material="steel",
            name=f"{side}_rear_gusset",
        )

    flap = model.part("flap")
    flap.visual(
        Box((0.74, 0.61, 0.01)),
        origin=Origin(xyz=(0.0, 0.305, 0.025)),
        material="steel",
        name="roof_panel",
    )
    flap.visual(
        Box((0.70, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, 0.60, -0.01)),
        material="steel",
        name="front_lip",
    )
    for side, x in (("right", 0.348), ("left", -0.348)):
        flap.visual(
            Box((0.016, 0.46, 0.084)),
            origin=Origin(xyz=(x, 0.37, -0.022)),
            material="steel",
            name=f"{side}_side_skirt",
        )

    flap.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_axis_rpy("x")),
        material="galvanized",
        name="center_hinge_barrel",
    )
    flap.visual(
        Box((0.18, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.015, 0.03)),
        material="steel",
        name="center_hinge_bridge",
    )
    for side, x in (("right", 0.20), ("left", -0.20)):
        flap.visual(
            Box((0.08, 0.04, 0.04)),
            origin=Origin(xyz=(x, 0.05, 0.03)),
            material="steel",
            name=f"{side}_hinge_reinforcement",
        )
        _add_bolt(
            flap,
            xyz=(x - 0.02, 0.065, 0.028),
            axis="z",
            name=f"{side}_hinge_roof_bolt_a",
        )
        _add_bolt(
            flap,
            xyz=(x + 0.02, 0.035, 0.028),
            axis="z",
            name=f"{side}_hinge_roof_bolt_b",
        )

    for side, x in (("right", 0.20), ("left", -0.20)):
        flap.visual(
            Box((0.05, 0.24, 0.02)),
            origin=Origin(xyz=(x, 0.33, 0.01)),
            material="steel",
            name=f"{side}_under_rib",
        )
    flap.visual(
        Box((0.46, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, 0.46, 0.01)),
        material="steel",
        name="front_stiffener",
    )

    flap.visual(
        Box((0.05, 0.012, 0.12)),
        origin=Origin(xyz=(0.0, 0.594, -0.04)),
        material="safety_yellow",
        name="lockout_strap",
    )

    for side, lug_x, pad_x in (("right", 0.385, 0.385), ("left", -0.385, -0.385)):
        flap.visual(
            Box((0.06, 0.10, 0.03)),
            origin=Origin(xyz=(lug_x, 0.05, 0.04)),
            material="steel",
            name=f"{side}_stop_lug",
        )
        flap.visual(
            Box((0.05, 0.028, 0.03)),
            origin=Origin(xyz=(pad_x, 0.012, 0.055)),
            material="safety_yellow",
            name=f"{side}_stop_pad",
        )

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.8,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("tower_to_flap")

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

    limits = hinge.motion_limits
    ctx.check(
        "hinge_axis_is_rear_cross_shaft",
        tuple(round(v, 4) for v in hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected hinge axis (1, 0, 0), got {hinge.axis}",
    )
    ctx.check(
        "hinge_limits_match_heavy_duty_flap",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.95 <= limits.upper <= 1.10,
        details=f"unexpected hinge limits: {limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            tower,
            flap,
            elem_a="left_hinge_barrel",
            elem_b="center_hinge_barrel",
            name="left_hinge_barrels_touch",
        )
        ctx.expect_contact(
            tower,
            flap,
            elem_a="right_hinge_barrel",
            elem_b="center_hinge_barrel",
            name="right_hinge_barrels_touch",
        )
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            positive_elem="roof_panel",
            negative_elem="front_cap_beam",
            max_gap=0.041,
            max_penetration=0.0,
            name="closed_flap_weather_gap",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="xy",
            elem_a="roof_panel",
            elem_b="curb_base",
            min_overlap=0.50,
            name="closed_flap_covers_tower_plan",
        )

    with ctx.pose({hinge: limits.upper if limits is not None else 1.05}):
        ctx.expect_contact(
            flap,
            tower,
            elem_a="left_stop_pad",
            elem_b="left_stop_block",
            name="left_stop_engages_at_full_open",
        )
        ctx.expect_contact(
            flap,
            tower,
            elem_a="right_stop_pad",
            elem_b="right_stop_block",
            name="right_stop_engages_at_full_open",
        )
        lip_aabb = ctx.part_element_world_aabb(flap, elem="front_lip")
        tower_aabb = ctx.part_world_aabb(tower)
        lip_min_z = lip_aabb[0][2] if lip_aabb is not None else None
        tower_max_z = tower_aabb[1][2] if tower_aabb is not None else None
        ctx.check(
            "open_flap_front_edge_clears_exhaust_stream",
            lip_min_z is not None
            and tower_max_z is not None
            and lip_min_z >= tower_max_z + 0.18,
            details=(
                f"front lip min z {lip_min_z} must clear tower max z {tower_max_z} "
                "by at least 0.18 m"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
