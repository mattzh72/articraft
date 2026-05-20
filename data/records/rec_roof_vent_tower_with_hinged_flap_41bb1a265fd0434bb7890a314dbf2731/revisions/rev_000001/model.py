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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_vent_tower_with_hinged_weather_flap")

    galvanized = model.material("galvanized_steel", rgba=(0.66, 0.69, 0.71, 1.0))
    coated = model.material("powder_coat_gray", rgba=(0.40, 0.43, 0.46, 1.0))
    gasket = model.material("epdm_gasket", rgba=(0.08, 0.08, 0.08, 1.0))
    hardware = model.material("stainless_hardware", rgba=(0.74, 0.76, 0.78, 1.0))

    tower = model.part("tower")
    flap = model.part("flap")

    curb_size = 0.78
    flashing_size = 0.92
    curb_height = 0.16
    wall_height = 0.76
    tower_width = 0.60
    tower_depth = 0.60
    wall_thickness = 0.04

    opening_width = 0.42
    opening_height = 0.52
    opening_bottom = 0.25
    opening_top = opening_bottom + opening_height
    mid_opening_z = (opening_bottom + opening_top) / 2.0

    front_wall_y = tower_depth / 2.0 - wall_thickness / 2.0
    side_wall_x = tower_width / 2.0 - wall_thickness / 2.0
    rear_wall_y = -tower_depth / 2.0 + wall_thickness / 2.0
    wall_center_z = curb_height + wall_height / 2.0
    tower_top_z = curb_height + wall_height

    hinge_axis_y = 0.322
    hinge_axis_z = 0.80
    barrel_radius = 0.011

    # Roof curb and flashing apron.
    tower.visual(
        Box((flashing_size, flashing_size, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=galvanized,
        name="roof_flashing",
    )
    tower.visual(
        Box((curb_size, curb_size, curb_height)),
        origin=Origin(xyz=(0.0, 0.0, curb_height / 2.0)),
        material=galvanized,
        name="curb_shell",
    )

    # Hollow tower body with framed front opening.
    tower.visual(
        Box((wall_thickness, tower_depth, wall_height)),
        origin=Origin(xyz=(-side_wall_x, 0.0, wall_center_z)),
        material=galvanized,
        name="left_wall",
    )
    tower.visual(
        Box((wall_thickness, tower_depth, wall_height)),
        origin=Origin(xyz=(side_wall_x, 0.0, wall_center_z)),
        material=galvanized,
        name="right_wall",
    )
    tower.visual(
        Box((tower_width - 2.0 * wall_thickness, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, rear_wall_y, wall_center_z)),
        material=galvanized,
        name="rear_wall",
    )

    tower.visual(
        Box((0.09, wall_thickness, opening_height)),
        origin=Origin(xyz=(-0.255, front_wall_y, mid_opening_z)),
        material=galvanized,
        name="front_jamb_left",
    )
    tower.visual(
        Box((0.09, wall_thickness, opening_height)),
        origin=Origin(xyz=(0.255, front_wall_y, mid_opening_z)),
        material=galvanized,
        name="front_jamb_right",
    )
    tower.visual(
        Box((opening_width, wall_thickness, opening_bottom - curb_height)),
        origin=Origin(
            xyz=(0.0, front_wall_y, curb_height + (opening_bottom - curb_height) / 2.0)
        ),
        material=galvanized,
        name="front_sill_structure",
    )
    tower.visual(
        Box((opening_width, wall_thickness, tower_top_z - opening_top)),
        origin=Origin(
            xyz=(0.0, front_wall_y, opening_top + (tower_top_z - opening_top) / 2.0)
        ),
        material=galvanized,
        name="front_header_structure",
    )

    # Closure land for the weather flap to seal against.
    tower.visual(
        Box((0.46, 0.014, 0.02)),
        origin=Origin(xyz=(0.0, 0.307, 0.78)),
        material=galvanized,
        name="closure_header",
    )
    tower.visual(
        Box((0.46, 0.014, 0.02)),
        origin=Origin(xyz=(0.0, 0.307, 0.24)),
        material=galvanized,
        name="closure_sill",
    )
    tower.visual(
        Box((0.012, 0.014, 0.56)),
        origin=Origin(xyz=(-0.194, 0.307, 0.51)),
        material=galvanized,
        name="closure_left",
    )
    tower.visual(
        Box((0.012, 0.014, 0.56)),
        origin=Origin(xyz=(0.194, 0.307, 0.51)),
        material=galvanized,
        name="closure_right",
    )

    # Overhanging weather hood and drip control around the hinge zone.
    tower.visual(
        Box((0.64, 0.64, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
        material=galvanized,
        name="cap_riser",
    )
    tower.visual(
        Box((0.74, 0.78, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.9725)),
        material=coated,
        name="cap_plate",
    )
    tower.visual(
        Box((0.05, 0.68, 0.09)),
        origin=Origin(xyz=(-0.345, 0.0, 0.915)),
        material=galvanized,
        name="left_drip_skirt",
    )
    tower.visual(
        Box((0.05, 0.68, 0.09)),
        origin=Origin(xyz=(0.345, 0.0, 0.915)),
        material=galvanized,
        name="right_drip_skirt",
    )
    tower.visual(
        Box((0.66, 0.05, 0.09)),
        origin=Origin(xyz=(0.0, -0.345, 0.915)),
        material=galvanized,
        name="rear_drip_skirt",
    )
    tower.visual(
        Box((0.58, 0.13, 0.10)),
        origin=Origin(xyz=(0.0, 0.325, 0.91)),
        material=coated,
        name="hood_brow",
    )
    tower.visual(
        Box((0.06, 0.13, 0.10)),
        origin=Origin(xyz=(-0.27, 0.325, 0.91)),
        material=coated,
        name="hood_cheek_left",
    )
    tower.visual(
        Box((0.06, 0.13, 0.10)),
        origin=Origin(xyz=(0.27, 0.325, 0.91)),
        material=coated,
        name="hood_cheek_right",
    )

    # Hinge support rail, barrels, and hard stop blocks on the fixed tower.
    tower.visual(
        Box((0.12, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.284, 0.81)),
        material=galvanized,
        name="hinge_rail",
    )
    tower.visual(
        Box((0.07, 0.026, 0.058)),
        origin=Origin(xyz=(-0.11, 0.298, 0.806)),
        material=hardware,
        name="tower_plate_left",
    )
    tower.visual(
        Box((0.07, 0.026, 0.058)),
        origin=Origin(xyz=(0.11, 0.298, 0.806)),
        material=hardware,
        name="tower_plate_right",
    )
    tower.visual(
        Cylinder(radius=barrel_radius, length=0.06),
        origin=Origin(xyz=(-0.11, hinge_axis_y, hinge_axis_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="tower_knuckle_left",
    )
    tower.visual(
        Cylinder(radius=barrel_radius, length=0.06),
        origin=Origin(xyz=(0.11, hinge_axis_y, hinge_axis_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="tower_knuckle_right",
    )
    tower.visual(
        Box((0.024, 0.02, 0.025)),
        origin=Origin(xyz=(-0.292, 0.362, 0.848)),
        material=hardware,
        name="stop_block_left",
    )
    tower.visual(
        Box((0.024, 0.02, 0.025)),
        origin=Origin(xyz=(0.292, 0.362, 0.848)),
        material=hardware,
        name="stop_block_right",
    )

    # Hinged weather flap, rebuilt as a hemmed sheet-metal leaf with a dropped
    # main panel below the hinge barrels so the hinge line stays protected.
    flap.visual(
        Box((0.46, 0.028, 0.49)),
        origin=Origin(xyz=(0.0, 0.006, -0.301)),
        material=coated,
        name="panel_core",
    )
    flap.visual(
        Box((0.54, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.035, -0.035)),
        material=coated,
        name="top_cap",
    )
    flap.visual(
        Box((0.018, 0.04, 0.47)),
        origin=Origin(xyz=(-0.221, 0.002, -0.293)),
        material=coated,
        name="side_flange_left",
    )
    flap.visual(
        Box((0.018, 0.04, 0.47)),
        origin=Origin(xyz=(0.221, 0.002, -0.293)),
        material=coated,
        name="side_flange_right",
    )
    flap.visual(
        Box((0.50, 0.05, 0.024)),
        origin=Origin(xyz=(0.0, 0.018, -0.558)),
        material=coated,
        name="bottom_drip",
    )
    flap.visual(
        Box((0.08, 0.04, 0.44)),
        origin=Origin(xyz=(0.0, -0.004, -0.312)),
        material=galvanized,
        name="back_stiffener",
    )

    # Gasketed closure interface on the back face of the flap.
    flap.visual(
        Box((0.42, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, 0.006, -0.035)),
        material=gasket,
        name="seal_header",
    )
    flap.visual(
        Box((0.42, 0.014, 0.02)),
        origin=Origin(xyz=(0.0, -0.001, -0.539)),
        material=gasket,
        name="seal_bottom",
    )
    flap.visual(
        Box((0.02, 0.014, 0.46)),
        origin=Origin(xyz=(-0.21, -0.001, -0.292)),
        material=gasket,
        name="seal_left",
    )
    flap.visual(
        Box((0.02, 0.014, 0.46)),
        origin=Origin(xyz=(0.21, -0.001, -0.292)),
        material=gasket,
        name="seal_right",
    )

    # Explicit hinge barrels and flap-side mounting plates.
    flap.visual(
        Box((0.16, 0.038, 0.11)),
        origin=Origin(xyz=(0.0, 0.018, -0.048)),
        material=hardware,
        name="flap_plate_center",
    )
    flap.visual(
        Cylinder(radius=barrel_radius, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="flap_knuckle_center",
    )

    # Upstream stop noses that approach the fixed blocks near full-open travel.
    flap.visual(
        Box((0.05, 0.018, 0.024)),
        origin=Origin(xyz=(-0.258, 0.04, -0.03)),
        material=hardware,
        name="stop_nose_left",
    )
    flap.visual(
        Box((0.05, 0.018, 0.024)),
        origin=Origin(xyz=(0.258, 0.04, -0.03)),
        material=hardware,
        name="stop_nose_right",
    )

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.08,
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

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            flap,
            tower,
            elem_a="seal_header",
            elem_b="closure_header",
            contact_tol=0.003,
            name="closed_flap_seats_on_top_seal",
        )

        panel_aabb = ctx.part_element_world_aabb(flap, elem="panel_core")
        closure_left = ctx.part_element_world_aabb(tower, elem="closure_left")
        closure_right = ctx.part_element_world_aabb(tower, elem="closure_right")
        closure_sill = ctx.part_element_world_aabb(tower, elem="closure_sill")
        closure_header = ctx.part_element_world_aabb(tower, elem="closure_header")

        if (
            panel_aabb is None
            or closure_left is None
            or closure_right is None
            or closure_sill is None
            or closure_header is None
        ):
            ctx.fail("framed_opening_aabbs_available", "expected framed opening visuals were not measurable")
        else:
            frame_min_x = closure_left[0][0]
            frame_max_x = closure_right[1][0]
            frame_min_z = closure_sill[0][2]
            frame_max_z = closure_header[1][2]
            flap_aabb = ctx.part_world_aabb(flap)
            if flap_aabb is None:
                ctx.fail("closed_flap_aabb_available", "flap AABB was not measurable in the closed pose")
            else:
                ctx.check(
                    "flap_covers_framed_opening",
                    flap_aabb[0][0] <= frame_min_x
                    and flap_aabb[1][0] >= frame_max_x
                    and flap_aabb[0][2] <= frame_min_z
                    and flap_aabb[1][2] >= frame_max_z,
                    details=(
                        f"flap_aabb={flap_aabb}, "
                        f"frame_extents=(({frame_min_x:.3f}, {frame_min_z:.3f}), "
                        f"({frame_max_x:.3f}, {frame_max_z:.3f}))"
                    ),
                )

    closed_bottom = None
    open_bottom = None
    with ctx.pose({hinge: 0.0}):
        closed_bottom = ctx.part_element_world_aabb(flap, elem="bottom_drip")

    with ctx.pose({hinge: 1.08}):
        ctx.fail_if_parts_overlap_in_current_pose(name="service_open_pose_clear")
        ctx.expect_gap(
            flap,
            tower,
            axis="y",
            positive_elem="bottom_drip",
            negative_elem="closure_header",
            min_gap=0.16,
            name="bottom_edge_swings_forward_when_open",
        )
        ctx.expect_contact(
            flap,
            tower,
            elem_a="stop_nose_left",
            elem_b="stop_block_left",
            contact_tol=0.012,
            name="left_stop_nears_engagement_at_open_limit",
        )
        ctx.expect_contact(
            flap,
            tower,
            elem_a="stop_nose_right",
            elem_b="stop_block_right",
            contact_tol=0.012,
            name="right_stop_nears_engagement_at_open_limit",
        )
        open_bottom = ctx.part_element_world_aabb(flap, elem="bottom_drip")

    if closed_bottom is None or open_bottom is None:
        ctx.fail("bottom_edge_motion_aabbs_available", "bottom drip AABBs were not available in one or more poses")
    else:
        ctx.check(
            "bottom_edge_lifts_as_flap_opens",
            open_bottom[0][2] > closed_bottom[1][2] + 0.20,
            details=f"closed_bottom_max_z={closed_bottom[1][2]:.3f}, open_bottom_min_z={open_bottom[0][2]:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
