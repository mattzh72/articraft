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
    TestContext,
    TestReport,
)


def _add_box(part, name, size, xyz, *, material=None, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder_x(part, name, radius, length, xyz, *, material=None):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_z(part, name, radius, length, xyz, *, material=None):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_rect_ring(
    part,
    prefix,
    *,
    outer_x,
    outer_y,
    inner_x,
    inner_y,
    center_z,
    height,
    material=None,
):
    ring_x = 0.5 * (outer_x - inner_x)
    ring_y = 0.5 * (outer_y - inner_y)
    y_center = 0.25 * (outer_y + inner_y)
    x_center = 0.25 * (outer_x + inner_x)
    _add_box(
        part,
        f"{prefix}_front",
        (outer_x, ring_y, height),
        (0.0, y_center, center_z),
        material=material,
    )
    _add_box(
        part,
        f"{prefix}_back",
        (outer_x, ring_y, height),
        (0.0, -y_center, center_z),
        material=material,
    )
    _add_box(
        part,
        f"{prefix}_left",
        (ring_x, inner_y, height),
        (-x_center, 0.0, center_z),
        material=material,
    )
    _add_box(
        part,
        f"{prefix}_right",
        (ring_x, inner_y, height),
        (x_center, 0.0, center_z),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="legacy_roof_vent_tower")

    galvanized = model.material("galvanized_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    aged_paint = model.material("aged_paint", rgba=(0.30, 0.35, 0.31, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.24, 1.0))

    tower = model.part("tower")

    # Roof adapter and lower curb.
    _add_rect_ring(
        tower,
        "base_adapter",
        outer_x=1.30,
        outer_y=0.96,
        inner_x=1.04,
        inner_y=0.70,
        center_z=0.02,
        height=0.04,
        material=dark_steel,
    )
    _add_rect_ring(
        tower,
        "adapter_collar",
        outer_x=1.00,
        outer_y=0.74,
        inner_x=0.86,
        inner_y=0.60,
        center_z=0.11,
        height=0.14,
        material=galvanized,
    )
    for idx, (x, y) in enumerate(
        (
            (-0.51, -0.37),
            (-0.51, 0.37),
            (0.51, -0.37),
            (0.51, 0.37),
        ),
        start=1,
    ):
        _add_box(
            tower,
            f"corner_reinforcement_{idx}",
            (0.08, 0.08, 0.16),
            (x, y, 0.11),
            material=dark_steel,
        )
    for idx, (x, y) in enumerate(
        (
            (-0.42, -0.36),
            (-0.16, -0.36),
            (0.16, -0.36),
            (0.42, -0.36),
            (-0.42, 0.36),
            (-0.16, 0.36),
            (0.16, 0.36),
            (0.42, 0.36),
        ),
        start=1,
    ):
        _add_cylinder_z(
            tower,
            f"adapter_bolt_{idx}",
            radius=0.012,
            length=0.014,
            xyz=(x, y, 0.047),
            material=dark_steel,
        )

    # Main tower frame with open front and rear vent faces.
    for x in (-0.40, 0.40):
        for y in (-0.28, 0.28):
            _add_box(
                tower,
                f"post_{'l' if x < 0 else 'r'}_{'rear' if y < 0 else 'front'}",
                (0.08, 0.08, 0.85),
                (x, y, 0.595),
                material=galvanized,
            )

    for side, y in (("front", 0.295), ("rear", -0.295)):
        for suffix, x in (("left", -0.34), ("right", 0.34)):
            _add_box(
                tower,
                f"{side}_jamb_{suffix}",
                (0.10, 0.05, 0.64),
                (x, y, 0.62),
                material=galvanized,
            )
        _add_box(
            tower,
            f"{side}_sill",
            (0.78, 0.05, 0.08),
            (0.0, y, 0.34),
            material=galvanized,
        )
        _add_box(
            tower,
            f"{side}_header",
            (0.78, 0.05, 0.08),
            (0.0, y, 0.90),
            material=galvanized,
        )
        _add_box(
            tower,
            f"{side}_opening_frame_outer",
            (0.86, 0.018, 0.76),
            (0.0, y + (0.012 if y > 0 else -0.012), 0.62),
            material=dark_steel,
        )

    _add_box(tower, "left_side_panel", (0.05, 0.48, 0.76), (-0.395, 0.0, 0.56), material=galvanized)
    _add_box(tower, "right_side_panel", (0.05, 0.48, 0.76), (0.395, 0.0, 0.56), material=galvanized)

    for x, side in ((-0.395, "left"), (0.395, "right")):
        _add_box(
            tower,
            f"{side}_service_hatch",
            (0.012, 0.34, 0.48),
            (x + (-0.019 if x < 0 else 0.019), 0.0, 0.57),
            material=aged_paint,
        )
        _add_box(
            tower,
            f"{side}_hatch_vertical_rib",
            (0.014, 0.06, 0.42),
            (x + (-0.019 if x < 0 else 0.019), 0.0, 0.57),
            material=dark_steel,
        )
        _add_box(
            tower,
            f"{side}_hatch_horizontal_rib",
            (0.014, 0.26, 0.06),
            (x + (-0.019 if x < 0 else 0.019), 0.0, 0.57),
            material=dark_steel,
        )
        for idx, (y, z) in enumerate(
            (
                (-0.13, 0.75),
                (0.13, 0.75),
                (-0.13, 0.57),
                (0.13, 0.57),
                (-0.13, 0.39),
                (0.13, 0.39),
            ),
            start=1,
        ):
            _add_cylinder_x(
                tower,
                f"{side}_hatch_bolt_{idx}",
                radius=0.010,
                length=0.016,
                xyz=(x + (-0.028 if x < 0 else 0.028), y, z),
                material=dark_steel,
            )

    _add_rect_ring(
        tower,
        "top_frame",
        outer_x=0.96,
        outer_y=0.70,
        inner_x=0.80,
        inner_y=0.54,
        center_z=1.00,
        height=0.08,
        material=galvanized,
    )
    _add_box(tower, "left_roof_rib", (0.08, 0.08, 0.18), (-0.44, -0.31, 0.97), material=dark_steel)
    _add_box(tower, "right_roof_rib", (0.08, 0.08, 0.18), (0.44, -0.31, 0.97), material=dark_steel)

    # Hinge-side tower barrels, support ears, and open-stop blocks.
    hinge_y = -0.35
    hinge_z = 1.10
    barrel_radius = 0.025
    tower_barrels = (
        ("tower_barrel_left", -0.31, 0.18),
        ("tower_barrel_center", 0.0, 0.16),
        ("tower_barrel_right", 0.31, 0.18),
    )
    for name, x, length in tower_barrels:
        _add_box(
            tower,
            f"{name}_plate",
            (max(length - 0.02, 0.08), 0.06, 0.07),
            (x, -0.38, 1.075),
            material=dark_steel,
        )
        _add_cylinder_x(
            tower,
            name,
            radius=barrel_radius,
            length=length,
            xyz=(x, hinge_y, hinge_z),
            material=dark_steel,
        )

    for side, x in (("left", -0.34), ("right", 0.34)):
        _add_box(
            tower,
            f"{side}_stop_strut",
            (0.03, 0.08, 0.12),
            (x, -0.40, 1.10),
            material=dark_steel,
        )
        _add_box(
            tower,
            f"{side}_stop_block",
            (0.07, 0.16, 0.08),
            (x, -0.32, 1.184),
            material=dark_steel,
        )

    flap = model.part("weather_flap")

    _add_box(flap, "flap_skin", (1.04, 0.72, 0.018), (0.0, 0.48, -0.048), material=aged_paint)
    _add_box(flap, "left_skirt", (0.022, 0.52, 0.09), (-0.511, 0.48, -0.088), material=aged_paint)
    _add_box(flap, "right_skirt", (0.022, 0.52, 0.09), (0.511, 0.48, -0.088), material=aged_paint)
    _add_box(flap, "front_drip", (0.94, 0.04, 0.08), (0.0, 0.86, -0.085), material=aged_paint)
    _add_box(flap, "center_rear_rib", (0.46, 0.14, 0.03), (0.0, 0.20, -0.036), material=dark_steel)
    _add_box(flap, "left_rear_stiffener", (0.06, 0.18, 0.03), (-0.25, 0.22, -0.038), material=dark_steel)
    _add_box(flap, "right_rear_stiffener", (0.06, 0.18, 0.03), (0.25, 0.22, -0.038), material=dark_steel)
    _add_box(flap, "left_hinge_strap", (0.12, 0.14, 0.05), (-0.15, 0.08, -0.010), material=dark_steel)
    _add_box(flap, "right_hinge_strap", (0.12, 0.14, 0.05), (0.15, 0.08, -0.010), material=dark_steel)
    _add_box(flap, "left_stop_lug", (0.06, 0.05, 0.07), (-0.34, 0.15, -0.010), material=dark_steel)
    _add_box(flap, "right_stop_lug", (0.06, 0.05, 0.07), (0.34, 0.15, -0.010), material=dark_steel)
    _add_cylinder_x(flap, "flap_barrel_left", radius=0.022, length=0.14, xyz=(-0.15, 0.0, 0.0), material=dark_steel)
    _add_cylinder_x(flap, "flap_barrel_right", radius=0.022, length=0.14, xyz=(0.15, 0.0, 0.0), material=dark_steel)

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    flap = object_model.get_part("weather_flap")
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

    ctx.check("tower_exists", tower is not None, "tower part missing")
    ctx.check("weather_flap_exists", flap is not None, "weather flap part missing")
    ctx.check("hinge_exists", hinge is not None, "flap hinge articulation missing")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            positive_elem="flap_skin",
            negative_elem="top_frame_front",
            min_gap=0.001,
            max_gap=0.035,
            name="closed_flap_seats_just_above_top_frame",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="xy",
            elem_a="flap_skin",
            elem_b="top_frame_front",
            min_overlap=0.05,
            name="closed_flap_covers_top_opening_front_band",
        )
        ctx.expect_contact(
            flap,
            tower,
            elem_a="flap_barrel_left",
            elem_b="tower_barrel_left",
            contact_tol=1e-6,
            name="left_hinge_barrels_touch",
        )
        ctx.expect_contact(
            flap,
            tower,
            elem_a="flap_barrel_right",
            elem_b="tower_barrel_right",
            contact_tol=1e-6,
            name="right_hinge_barrels_touch",
        )

    with ctx.pose({hinge: 1.10}):
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            positive_elem="front_drip",
            negative_elem="top_frame_front",
            min_gap=0.22,
            name="open_flap_front_edge_lifts_clear",
        )
        ctx.expect_contact(
            flap,
            tower,
            elem_a="left_stop_lug",
            elem_b="left_stop_block",
            contact_tol=0.015,
            name="left_stop_lug_meets_stop_block",
        )
        ctx.expect_contact(
            flap,
            tower,
            elem_a="right_stop_lug",
            elem_b="right_stop_block",
            contact_tol=0.015,
            name="right_stop_lug_meets_stop_block",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
