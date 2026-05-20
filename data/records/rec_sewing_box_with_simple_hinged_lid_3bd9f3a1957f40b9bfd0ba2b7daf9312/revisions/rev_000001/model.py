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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_sewing_box")

    body_color = model.material("body_polymer", rgba=(0.22, 0.28, 0.24, 1.0))
    lid_color = model.material("lid_polymer", rgba=(0.25, 0.32, 0.28, 1.0))
    gasket_color = model.material("seal_gasket", rgba=(0.08, 0.08, 0.08, 1.0))
    hardware_color = model.material("hardware_steel", rgba=(0.68, 0.71, 0.74, 1.0))

    outer_w = 0.42
    outer_d = 0.27
    body_h = 0.20
    wall_t = 0.008
    bottom_t = 0.008
    wall_h = body_h - bottom_t

    opening_w = outer_w - 2.0 * wall_t
    opening_d = outer_d - 2.0 * wall_t

    seal_band = 0.014
    seal_t = 0.004

    side_overhang = 0.011
    front_overhang = 0.016
    rear_hood = 0.018

    lid_w = outer_w + 2.0 * side_overhang
    lid_front_extent = outer_d + front_overhang
    lid_rear_extent = rear_hood

    top_t = 0.008
    skirt_t = 0.0045
    skirt_h = 0.042

    tongue_t = 0.005
    tongue_h = 0.018
    tongue_clearance = 0.0025

    gasket_w = 0.012
    gasket_h = 0.004

    hinge_axis_y = -outer_d / 2.0 - 0.006
    hinge_axis_z = body_h + 0.012
    barrel_r = 0.008

    body = model.part("body")
    body.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=body_color,
        name="floor",
    )
    body.visual(
        Box((wall_t, outer_d, wall_h)),
        origin=Origin(
            xyz=(-outer_w / 2.0 + wall_t / 2.0, 0.0, bottom_t + wall_h / 2.0)
        ),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d, wall_h)),
        origin=Origin(
            xyz=(outer_w / 2.0 - wall_t / 2.0, 0.0, bottom_t + wall_h / 2.0)
        ),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(
            xyz=(0.0, outer_d / 2.0 - wall_t / 2.0, bottom_t + wall_h / 2.0)
        ),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(
            xyz=(0.0, -outer_d / 2.0 + wall_t / 2.0, bottom_t + wall_h / 2.0)
        ),
        material=body_color,
        name="back_wall",
    )

    body.visual(
        Box((opening_w, seal_band, seal_t)),
        origin=Origin(
            xyz=(0.0, opening_d / 2.0 - seal_band / 2.0, body_h - seal_t / 2.0)
        ),
        material=body_color,
        name="seal_land_front",
    )
    body.visual(
        Box((opening_w, seal_band, seal_t)),
        origin=Origin(
            xyz=(0.0, -opening_d / 2.0 + seal_band / 2.0, body_h - seal_t / 2.0)
        ),
        material=body_color,
        name="seal_land_back",
    )
    body.visual(
        Box((seal_band, opening_d - 2.0 * seal_band, seal_t)),
        origin=Origin(
            xyz=(
                -opening_w / 2.0 + seal_band / 2.0,
                0.0,
                body_h - seal_t / 2.0,
            )
        ),
        material=body_color,
        name="seal_land_left",
    )
    body.visual(
        Box((seal_band, opening_d - 2.0 * seal_band, seal_t)),
        origin=Origin(
            xyz=(
                opening_w / 2.0 - seal_band / 2.0,
                0.0,
                body_h - seal_t / 2.0,
            )
        ),
        material=body_color,
        name="seal_land_right",
    )

    body_leaf_len = 0.125
    body_leaf_depth = 0.014
    body_leaf_h = 0.026
    body_leaf_z = body_h - 0.003
    body_leaf_y = -outer_d / 2.0 - body_leaf_depth / 2.0
    body_leaf_x = 0.120

    body.visual(
        Box((body_leaf_len, body_leaf_depth, body_leaf_h)),
        origin=Origin(xyz=(-body_leaf_x, body_leaf_y, body_leaf_z)),
        material=hardware_color,
        name="hinge_leaf_left",
    )
    body.visual(
        Box((body_leaf_len, body_leaf_depth, body_leaf_h)),
        origin=Origin(xyz=(body_leaf_x, body_leaf_y, body_leaf_z)),
        material=hardware_color,
        name="hinge_leaf_right",
    )
    body.visual(
        Cylinder(radius=barrel_r, length=body_leaf_len),
        origin=Origin(
            xyz=(-body_leaf_x, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hardware_color,
        name="hinge_barrel_left",
    )
    body.visual(
        Cylinder(radius=barrel_r, length=body_leaf_len),
        origin=Origin(
            xyz=(body_leaf_x, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hardware_color,
        name="hinge_barrel_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    lid = model.part("lid")

    lid_panel_start_y = 0.010
    lid_panel_depth = lid_front_extent - lid_panel_start_y
    lid_panel_center_y = (lid_panel_start_y + lid_front_extent) / 2.0
    lid_panel_z = -top_t / 2.0

    lid.visual(
        Box((lid_w, lid_panel_depth, top_t)),
        origin=Origin(xyz=(0.0, lid_panel_center_y, lid_panel_z)),
        material=lid_color,
        name="lid_top",
    )

    cap_depth = 0.038
    cap_center_y = 0.001
    cap_center_z = 0.016
    cap_web_depth = 0.008
    cap_web_h = 0.020
    cap_web_center_y = 0.014
    cap_web_center_z = 0.006

    lid.visual(
        Box((lid_w, cap_depth, top_t)),
        origin=Origin(xyz=(0.0, cap_center_y, cap_center_z)),
        material=lid_color,
        name="rear_weather_cap",
    )
    lid.visual(
        Box((lid_w, cap_web_depth, cap_web_h)),
        origin=Origin(xyz=(0.0, cap_web_center_y, cap_web_center_z)),
        material=lid_color,
        name="rear_cap_web",
    )

    side_skirt_depth = lid_front_extent - 0.006
    side_skirt_center_y = (0.006 + lid_front_extent) / 2.0
    side_skirt_z = -top_t - skirt_h / 2.0

    lid.visual(
        Box((skirt_t, side_skirt_depth, skirt_h)),
        origin=Origin(
            xyz=(-lid_w / 2.0 + skirt_t / 2.0, side_skirt_center_y, side_skirt_z)
        ),
        material=lid_color,
        name="left_skirt",
    )
    lid.visual(
        Box((skirt_t, side_skirt_depth, skirt_h)),
        origin=Origin(
            xyz=(lid_w / 2.0 - skirt_t / 2.0, side_skirt_center_y, side_skirt_z)
        ),
        material=lid_color,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_w - 2.0 * skirt_t, skirt_t, skirt_h)),
        origin=Origin(
            xyz=(
                0.0,
                lid_front_extent - skirt_t / 2.0,
                -top_t - skirt_h / 2.0,
            )
        ),
        material=lid_color,
        name="front_skirt",
    )

    tongue_side_len = opening_d - 2.0 * tongue_clearance - 2.0 * tongue_t
    tongue_front_center_world_y = opening_d / 2.0 - tongue_clearance - tongue_t / 2.0
    tongue_back_center_world_y = -opening_d / 2.0 + tongue_clearance + tongue_t / 2.0
    tongue_center_local_y = 0.0 - hinge_axis_y
    tongue_side_center_x = opening_w / 2.0 - tongue_clearance - tongue_t / 2.0
    tongue_z = -top_t - tongue_h / 2.0

    lid.visual(
        Box((opening_w - 2.0 * tongue_t, tongue_t, tongue_h)),
        origin=Origin(
            xyz=(0.0, tongue_front_center_world_y - hinge_axis_y, tongue_z)
        ),
        material=lid_color,
        name="tongue_front",
    )
    lid.visual(
        Box((opening_w - 2.0 * tongue_t, tongue_t, tongue_h)),
        origin=Origin(
            xyz=(0.0, tongue_back_center_world_y - hinge_axis_y, tongue_z)
        ),
        material=lid_color,
        name="tongue_back",
    )
    lid.visual(
        Box((tongue_t, tongue_side_len, tongue_h)),
        origin=Origin(
            xyz=(-tongue_side_center_x, tongue_center_local_y, tongue_z)
        ),
        material=lid_color,
        name="tongue_left",
    )
    lid.visual(
        Box((tongue_t, tongue_side_len, tongue_h)),
        origin=Origin(
            xyz=(tongue_side_center_x, tongue_center_local_y, tongue_z)
        ),
        material=lid_color,
        name="tongue_right",
    )

    gasket_z = -top_t - gasket_h / 2.0
    gasket_front_world_y = opening_d / 2.0 - seal_band / 2.0
    gasket_back_world_y = -opening_d / 2.0 + seal_band / 2.0
    gasket_side_x = opening_w / 2.0 - seal_band / 2.0

    lid.visual(
        Box((opening_w, gasket_w, gasket_h)),
        origin=Origin(xyz=(0.0, gasket_front_world_y - hinge_axis_y, gasket_z)),
        material=gasket_color,
        name="gasket_front",
    )
    lid.visual(
        Box((opening_w, gasket_w, gasket_h)),
        origin=Origin(xyz=(0.0, gasket_back_world_y - hinge_axis_y, gasket_z)),
        material=gasket_color,
        name="gasket_back",
    )
    lid.visual(
        Box((gasket_w, opening_d - 2.0 * seal_band, gasket_h)),
        origin=Origin(xyz=(-gasket_side_x, -hinge_axis_y, gasket_z)),
        material=gasket_color,
        name="gasket_left",
    )
    lid.visual(
        Box((gasket_w, opening_d - 2.0 * seal_band, gasket_h)),
        origin=Origin(xyz=(gasket_side_x, -hinge_axis_y, gasket_z)),
        material=gasket_color,
        name="gasket_right",
    )

    lid_barrel_len = 0.107
    lid.visual(
        Box((lid_barrel_len, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, -0.002, 0.004)),
        material=hardware_color,
        name="hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=barrel_r, length=lid_barrel_len),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_color,
        name="hinge_barrel_center",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_front_extent + lid_rear_extent, 0.062)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.14, -0.010)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

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
        "hinge_limits_are_practical",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and 1.2 <= hinge.motion_limits.upper <= 1.5,
        details="Rear weather cap and hinge knuckles should constrain the lid to a realistic opening range.",
    )

    ctx.expect_contact(
        lid,
        body,
        elem_a="gasket_front",
        elem_b="seal_land_front",
        contact_tol=0.001,
        name="front_gasket_seats_on_seal_land",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="gasket_left",
        elem_b="seal_land_left",
        contact_tol=0.001,
        name="left_gasket_seats_on_seal_land",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="gasket_right",
        elem_b="seal_land_right",
        contact_tol=0.001,
        name="right_gasket_seats_on_seal_land",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.20,
        name="lid_covers_box_plan",
    )

    open_angle = 1.1
    if hinge.motion_limits is not None and hinge.motion_limits.upper is not None:
        open_angle = hinge.motion_limits.upper

    with ctx.pose({hinge: open_angle}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="seal_land_front",
            min_gap=0.10,
            name="front_edge_lifts_clear_when_opened",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
