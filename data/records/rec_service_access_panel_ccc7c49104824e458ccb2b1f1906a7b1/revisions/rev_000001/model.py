from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_service_access_panel")

    # Realistic outdoor service-panel proportions.
    opening_w = 0.600
    opening_h = 0.850
    frame_border = 0.060
    outer_w = opening_w + 2.0 * frame_border
    outer_h = opening_h + 2.0 * frame_border

    sleeve_depth = 0.056
    flange_thick = 0.004
    gasket_thick = 0.003

    cover_w = 0.650
    cover_h = 0.900
    cover_inset_from_hinge = 0.010
    pan_w = 0.584
    pan_h = 0.834
    pan_depth = 0.034

    hinge_axis_x = -(cover_inset_from_hinge + cover_w / 2.0)

    frame_color = model.material("frame_powdercoat", rgba=(0.30, 0.33, 0.36, 1.0))
    panel_color = model.material("panel_powdercoat", rgba=(0.77, 0.79, 0.80, 1.0))
    gasket_color = model.material("epdm_gasket", rgba=(0.08, 0.08, 0.08, 1.0))
    hardware_color = model.material("stainless_hardware", rgba=(0.70, 0.73, 0.76, 1.0))

    frame = model.part("frame")

    # Recessed sleeve around the service opening.
    frame.visual(
        Box((frame_border, sleeve_depth, outer_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 + frame_border / 2.0), -0.026, 0.0)),
        material=frame_color,
        name="left_jamb",
    )
    frame.visual(
        Box((frame_border, sleeve_depth, outer_h)),
        origin=Origin(xyz=((opening_w / 2.0 + frame_border / 2.0), -0.026, 0.0)),
        material=frame_color,
        name="right_jamb",
    )
    frame.visual(
        Box((opening_w, sleeve_depth, frame_border)),
        origin=Origin(xyz=(0.0, -0.026, opening_h / 2.0 + frame_border / 2.0)),
        material=frame_color,
        name="top_sleeve",
    )
    frame.visual(
        Box((opening_w, sleeve_depth, frame_border)),
        origin=Origin(xyz=(0.0, -0.026, -(opening_h / 2.0 + frame_border / 2.0))),
        material=frame_color,
        name="bottom_sleeve",
    )

    # Face flange for mounting and sealing surface.
    frame.visual(
        Box((frame_border, flange_thick, outer_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 + frame_border / 2.0), 0.002, 0.0)),
        material=frame_color,
        name="left_flange",
    )
    frame.visual(
        Box((frame_border, flange_thick, outer_h)),
        origin=Origin(xyz=((opening_w / 2.0 + frame_border / 2.0), 0.002, 0.0)),
        material=frame_color,
        name="right_flange",
    )
    frame.visual(
        Box((opening_w, flange_thick, frame_border)),
        origin=Origin(xyz=(0.0, 0.002, opening_h / 2.0 + frame_border / 2.0)),
        material=frame_color,
        name="top_flange",
    )
    frame.visual(
        Box((opening_w, flange_thick, frame_border)),
        origin=Origin(xyz=(0.0, 0.002, -(opening_h / 2.0 + frame_border / 2.0))),
        material=frame_color,
        name="bottom_flange",
    )

    # Compression gasket under the cover plate for a sealed outdoor closure.
    gasket_outer_w = 0.620
    gasket_outer_h = 0.870
    gasket_band = 0.018
    frame.visual(
        Box((gasket_band, gasket_thick, gasket_outer_h)),
        origin=Origin(xyz=(-(gasket_outer_w / 2.0 - gasket_band / 2.0), 0.0035, 0.0)),
        material=gasket_color,
        name="gasket_left",
    )
    frame.visual(
        Box((gasket_band, gasket_thick, gasket_outer_h)),
        origin=Origin(xyz=((gasket_outer_w / 2.0 - gasket_band / 2.0), 0.0035, 0.0)),
        material=gasket_color,
        name="gasket_right",
    )
    frame.visual(
        Box((gasket_outer_w, gasket_thick, gasket_band)),
        origin=Origin(xyz=(0.0, 0.0035, gasket_outer_h / 2.0 - gasket_band / 2.0)),
        material=gasket_color,
        name="gasket_top",
    )
    frame.visual(
        Box((gasket_outer_w, gasket_thick, gasket_band)),
        origin=Origin(xyz=(0.0, 0.0035, -(gasket_outer_h / 2.0 - gasket_band / 2.0))),
        material=gasket_color,
        name="gasket_bottom",
    )

    # Weather hood with a drip nose to keep runoff off the seal line.
    frame.visual(
        Box((outer_w, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.004, outer_h / 2.0 + 0.015)),
        material=frame_color,
        name="hood_backer",
    )
    frame.visual(
        Box((0.780, 0.072, 0.004)),
        origin=Origin(xyz=(0.0, 0.036, outer_h / 2.0 + 0.030)),
        material=frame_color,
        name="hood_plate",
    )
    frame.visual(
        Box((0.780, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.069, outer_h / 2.0 + 0.022)),
        material=frame_color,
        name="drip_lip",
    )
    frame.visual(
        Box((0.020, 0.055, 0.030)),
        origin=Origin(xyz=(-0.350, 0.028, outer_h / 2.0 + 0.015)),
        material=frame_color,
        name="hood_cheek_left",
    )
    frame.visual(
        Box((0.020, 0.055, 0.030)),
        origin=Origin(xyz=(0.350, 0.028, outer_h / 2.0 + 0.015)),
        material=frame_color,
        name="hood_cheek_right",
    )

    # Protected hinge side and latch-side keeper hardware.
    frame.visual(
        Cylinder(radius=0.010, length=0.820),
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.0)),
        material=hardware_color,
        name="hinge_barrel",
    )
    frame.visual(
        Box((0.014, 0.030, cover_h)),
        origin=Origin(xyz=(hinge_axis_x - 0.017, 0.015, 0.0)),
        material=frame_color,
        name="hinge_guard",
    )
    frame.visual(
        Box((0.012, 0.020, 0.140)),
        origin=Origin(xyz=(0.306, -0.006, 0.0)),
        material=hardware_color,
        name="striker_block",
    )

    panel = model.part("service_panel")

    panel.visual(
        Box((cover_w, 0.0025, cover_h)),
        origin=Origin(xyz=(cover_inset_from_hinge + cover_w / 2.0, 0.00625, 0.0)),
        material=panel_color,
        name="cover_plate",
    )
    panel.visual(
        Box((pan_w, pan_depth, pan_h)),
        origin=Origin(xyz=(cover_inset_from_hinge + cover_w / 2.0, -0.012, 0.0)),
        material=panel_color,
        name="pan_body",
    )
    panel.visual(
        Box((0.028, 0.004, 0.780)),
        origin=Origin(xyz=(0.028, 0.003, 0.0)),
        material=hardware_color,
        name="hinge_leaf",
    )
    panel.visual(
        Box((0.050, 0.008, 0.190)),
        origin=Origin(xyz=(0.605, 0.010, 0.0)),
        material=hardware_color,
        name="latch_escutcheon",
    )
    panel.visual(
        Cylinder(radius=0.009, length=0.115),
        origin=Origin(xyz=(0.616, 0.037, 0.0)),
        material=hardware_color,
        name="handle_grip",
    )
    panel.visual(
        Box((0.012, 0.024, 0.012)),
        origin=Origin(xyz=(0.613, 0.025, 0.036)),
        material=hardware_color,
        name="handle_post_top",
    )
    panel.visual(
        Box((0.012, 0.024, 0.012)),
        origin=Origin(xyz=(0.613, 0.025, -0.036)),
        material=hardware_color,
        name="handle_post_bottom",
    )
    panel.visual(
        Box((0.026, 0.018, 0.140)),
        origin=Origin(xyz=(0.620, -0.005, 0.0)),
        material=hardware_color,
        name="latch_housing",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("service_panel")
    hinge = object_model.get_articulation("frame_to_panel")

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
            panel,
            frame,
            elem_a="cover_plate",
            elem_b="gasket_top",
            contact_tol=0.001,
            name="cover_plate_seals_against_gasket",
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="x",
            positive_elem="pan_body",
            negative_elem="left_jamb",
            min_gap=0.006,
            max_gap=0.010,
            name="hinge_side_pan_clearance",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="x",
            positive_elem="right_jamb",
            negative_elem="pan_body",
            min_gap=0.006,
            max_gap=0.010,
            name="latch_side_pan_clearance",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="z",
            positive_elem="top_sleeve",
            negative_elem="pan_body",
            min_gap=0.006,
            max_gap=0.010,
            name="top_pan_clearance",
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="pan_body",
            negative_elem="bottom_sleeve",
            min_gap=0.006,
            max_gap=0.010,
            name="bottom_pan_clearance",
        )

        hinge_barrel_aabb = ctx.part_element_world_aabb(frame, elem="hinge_barrel")
        latch_aabb = ctx.part_element_world_aabb(panel, elem="latch_escutcheon")
        if hinge_barrel_aabb is None or latch_aabb is None:
            ctx.fail("hardware_aabbs_present", "Expected hinge barrel and latch escutcheon visuals.")
        else:
            hinge_x = 0.5 * (hinge_barrel_aabb[0][0] + hinge_barrel_aabb[1][0])
            latch_x = 0.5 * (latch_aabb[0][0] + latch_aabb[1][0])
            ctx.check(
                "latch_is_opposite_hinge",
                latch_x - hinge_x > 0.55,
                details=f"Expected latch hardware on opposite side of hinge; dx={latch_x - hinge_x:.3f} m",
            )

    with ctx.pose({hinge: 1.10}):
        opened_cover_aabb = ctx.part_element_world_aabb(panel, elem="cover_plate")
        if opened_cover_aabb is None:
            ctx.fail("opened_cover_aabb_present", "Expected cover plate AABB in open pose.")
        else:
            opened_y = opened_cover_aabb[1][1]
            ctx.check(
                "panel_swings_outward",
                opened_y > 0.25,
                details=f"Expected the free edge to swing outward; opened max y={opened_y:.3f} m",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
