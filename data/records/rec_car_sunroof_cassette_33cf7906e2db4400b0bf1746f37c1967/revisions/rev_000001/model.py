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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soft_top_sunroof_cassette")

    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    anodized = model.material("anodized", rgba=(0.34, 0.36, 0.39, 1.0))
    rail_liner = model.material("rail_liner", rgba=(0.12, 0.13, 0.14, 1.0))
    canvas = model.material("canvas", rgba=(0.14, 0.15, 0.16, 1.0))
    polymer = model.material("polymer", rgba=(0.18, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))

    frame = model.part("frame")
    frame_ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(1.18, 0.96, 0.03, corner_segments=8),
        [rounded_rect_profile(0.82, 0.62, 0.02, corner_segments=8)],
        0.016,
        center=False,
    )
    frame.visual(
        _save_mesh("sunroof_frame_ring", frame_ring),
        material=aluminum,
        name="base_ring",
    )
    frame.visual(
        Box((1.18, 0.072, 0.018)),
        origin=Origin(xyz=(0.0, -0.443, 0.025)),
        material=anodized,
        name="front_header",
    )
    frame.visual(
        Box((1.18, 0.164, 0.010)),
        origin=Origin(xyz=(0.0, 0.392, 0.013)),
        material=anodized,
        name="rear_housing_base",
    )
    frame.visual(
        Box((1.18, 0.028, 0.064)),
        origin=Origin(xyz=(0.0, 0.454, 0.046)),
        material=anodized,
        name="rear_wall",
    )
    frame.visual(
        Box((0.120, 0.830, 0.003)),
        origin=Origin(xyz=(-0.502, 0.018, 0.0175)),
        material=rail_liner,
        name="left_track_floor",
    )
    frame.visual(
        Box((0.120, 0.830, 0.003)),
        origin=Origin(xyz=(0.502, 0.018, 0.0175)),
        material=rail_liner,
        name="right_track_floor",
    )
    frame.visual(
        Box((0.028, 0.860, 0.064)),
        origin=Origin(xyz=(-0.576, 0.015, 0.046)),
        material=aluminum,
        name="left_outer_wall",
    )
    frame.visual(
        Box((0.028, 0.860, 0.064)),
        origin=Origin(xyz=(0.576, 0.015, 0.046)),
        material=aluminum,
        name="right_outer_wall",
    )
    frame.visual(
        Box((0.024, 0.824, 0.056)),
        origin=Origin(xyz=(-0.454, 0.018, 0.042)),
        material=anodized,
        name="left_track_inner",
    )
    frame.visual(
        Box((0.024, 0.824, 0.056)),
        origin=Origin(xyz=(0.454, 0.018, 0.042)),
        material=anodized,
        name="right_track_inner",
    )
    frame.visual(
        Box((0.050, 0.120, 0.064)),
        origin=Origin(xyz=(-0.515, 0.352, 0.046)),
        material=aluminum,
        name="left_drum_cheek",
    )
    frame.visual(
        Box((0.050, 0.120, 0.064)),
        origin=Origin(xyz=(0.515, 0.352, 0.046)),
        material=aluminum,
        name="right_drum_cheek",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.18, 0.96, 0.09)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    rear_drum = model.part("rear_drum")
    rear_drum.visual(
        Cylinder(radius=0.022, length=0.780),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="drum_shell",
    )
    rear_drum.visual(
        Cylinder(radius=0.028, length=0.760),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=canvas,
        name="rolled_canvas",
    )
    rear_drum.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(-0.389, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_end_cap",
    )
    rear_drum.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.389, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_end_cap",
    )
    rear_drum.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(-0.406, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_axle",
    )
    rear_drum.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(0.406, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_axle",
    )
    rear_drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.780),
        mass=2.6,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    sliding_panel = model.part("sliding_panel")
    sliding_panel.visual(
        Box((0.810, 0.055, 0.024)),
        origin=Origin(xyz=(0.0, 0.028, 0.014)),
        material=anodized,
        name="front_bow",
    )
    sliding_panel.visual(
        Box((0.790, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.060, 0.028)),
        material=canvas,
        name="front_seal",
    )
    sliding_panel.visual(
        Box((0.020, 0.054, 0.034)),
        origin=Origin(xyz=(-0.408, 0.058, 0.013)),
        material=anodized,
        name="left_bracket",
    )
    sliding_panel.visual(
        Box((0.020, 0.054, 0.034)),
        origin=Origin(xyz=(0.408, 0.058, 0.013)),
        material=anodized,
        name="right_bracket",
    )
    sliding_panel.visual(
        Box((0.038, 0.110, 0.018)),
        origin=Origin(xyz=(-0.517, 0.055, -0.001)),
        material=polymer,
        name="left_shoe",
    )
    sliding_panel.visual(
        Box((0.038, 0.110, 0.018)),
        origin=Origin(xyz=(0.517, 0.055, -0.001)),
        material=polymer,
        name="right_shoe",
    )
    sliding_panel.visual(
        Box((0.016, 0.028, 0.045)),
        origin=Origin(xyz=(-0.498, 0.055, 0.0295)),
        material=anodized,
        name="left_shoe_riser",
    )
    sliding_panel.visual(
        Box((0.084, 0.022, 0.009)),
        origin=Origin(xyz=(-0.458, 0.055, 0.0525)),
        material=anodized,
        name="left_shoe_bridge",
    )
    sliding_panel.visual(
        Box((0.028, 0.022, 0.024)),
        origin=Origin(xyz=(-0.426, 0.055, 0.040)),
        material=anodized,
        name="left_shoe_drop",
    )
    sliding_panel.visual(
        Box((0.016, 0.028, 0.045)),
        origin=Origin(xyz=(0.498, 0.055, 0.0295)),
        material=anodized,
        name="right_shoe_riser",
    )
    sliding_panel.visual(
        Box((0.084, 0.022, 0.009)),
        origin=Origin(xyz=(0.458, 0.055, 0.0525)),
        material=anodized,
        name="right_shoe_bridge",
    )
    sliding_panel.visual(
        Box((0.028, 0.022, 0.024)),
        origin=Origin(xyz=(0.426, 0.055, 0.040)),
        material=anodized,
        name="right_shoe_drop",
    )
    sliding_panel.visual(
        Box((0.800, 0.634, 0.0025)),
        origin=Origin(xyz=(0.0, 0.373, 0.031)),
        material=canvas,
        name="canvas_sheet",
    )
    sliding_panel.visual(
        Box((0.020, 0.634, 0.006)),
        origin=Origin(xyz=(-0.400, 0.373, 0.031)),
        material=canvas,
        name="left_edge_bead",
    )
    sliding_panel.visual(
        Box((0.020, 0.634, 0.006)),
        origin=Origin(xyz=(0.400, 0.373, 0.031)),
        material=canvas,
        name="right_edge_bead",
    )
    sliding_panel.visual(
        Box((0.760, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.682, 0.030)),
        material=canvas,
        name="rear_hem",
    )
    sliding_panel.inertial = Inertial.from_geometry(
        Box((1.06, 0.70, 0.07)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.350, 0.020)),
    )

    model.articulation(
        "frame_to_rear_drum",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_drum,
        origin=Origin(xyz=(0.0, 0.342, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_panel_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sliding_panel,
        origin=Origin(xyz=(0.0, -0.392, 0.029)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=0.0,
            upper=0.520,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rear_drum = object_model.get_part("rear_drum")
    sliding_panel = object_model.get_part("sliding_panel")
    drum_spin = object_model.get_articulation("frame_to_rear_drum")
    panel_slide = object_model.get_articulation("frame_to_panel_slide")

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

    drum_limits = drum_spin.motion_limits
    ctx.check(
        "rear drum uses continuous width-wise rotation",
        drum_spin.articulation_type == ArticulationType.CONTINUOUS
        and drum_spin.axis == (1.0, 0.0, 0.0)
        and drum_limits is not None
        and drum_limits.lower is None
        and drum_limits.upper is None,
        details=(
            f"type={drum_spin.articulation_type}, axis={drum_spin.axis}, "
            f"limits={drum_limits}"
        ),
    )

    slide_limits = panel_slide.motion_limits
    ctx.check(
        "panel carriage retracts rearward on a prismatic guide",
        panel_slide.articulation_type == ArticulationType.PRISMATIC
        and panel_slide.axis == (0.0, 1.0, 0.0)
        and slide_limits is not None
        and slide_limits.lower == 0.0
        and slide_limits.upper is not None
        and slide_limits.upper >= 0.45,
        details=(
            f"type={panel_slide.articulation_type}, axis={panel_slide.axis}, "
            f"limits={slide_limits}"
        ),
    )

    ctx.expect_gap(
        sliding_panel,
        frame,
        axis="y",
        positive_elem="front_bow",
        negative_elem="front_header",
        min_gap=0.008,
        max_gap=0.025,
        name="front bow closes near the front header",
    )
    ctx.expect_gap(
        rear_drum,
        sliding_panel,
        axis="y",
        positive_elem="rolled_canvas",
        negative_elem="rear_hem",
        min_gap=0.0,
        max_gap=0.020,
        name="canvas tail approaches the rolled drum",
    )

    for side in ("left", "right"):
        if side == "left":
            ctx.expect_gap(
                sliding_panel,
                frame,
                axis="x",
                positive_elem="left_shoe",
                negative_elem="left_outer_wall",
                min_gap=0.015,
                max_gap=0.040,
                name="left shoe clears outer rail wall",
            )
            ctx.expect_gap(
                frame,
                sliding_panel,
                axis="x",
                positive_elem="left_track_inner",
                negative_elem="left_shoe",
                min_gap=0.020,
                max_gap=0.050,
                name="left shoe clears inner rail wall",
            )
            ctx.expect_contact(
                sliding_panel,
                frame,
                elem_a="left_shoe",
                elem_b="left_track_floor",
                name="left shoe is supported by the left track floor",
            )
        else:
            ctx.expect_gap(
                frame,
                sliding_panel,
                axis="x",
                positive_elem="right_outer_wall",
                negative_elem="right_shoe",
                min_gap=0.015,
                max_gap=0.040,
                name="right shoe clears outer rail wall",
            )
            ctx.expect_gap(
                sliding_panel,
                frame,
                axis="x",
                positive_elem="right_shoe",
                negative_elem="right_track_inner",
                min_gap=0.020,
                max_gap=0.050,
                name="right shoe clears inner rail wall",
            )
            ctx.expect_contact(
                sliding_panel,
                frame,
                elem_a="right_shoe",
                elem_b="right_track_floor",
                name="right shoe is supported by the right track floor",
            )

    rest_pos = ctx.part_world_position(sliding_panel)
    slide_upper = 0.0 if slide_limits is None or slide_limits.upper is None else slide_limits.upper
    with ctx.pose({panel_slide: slide_upper}):
        ctx.expect_contact(
            sliding_panel,
            frame,
            elem_a="left_shoe",
            elem_b="left_track_floor",
            name="left shoe stays supported on the left rail floor when retracted",
        )
        ctx.expect_contact(
            sliding_panel,
            frame,
            elem_a="right_shoe",
            elem_b="right_track_floor",
            name="right shoe stays supported on the right rail floor when retracted",
        )
        open_pos = ctx.part_world_position(sliding_panel)

    ctx.check(
        "panel carriage moves rearward as it retracts",
        rest_pos is not None and open_pos is not None and open_pos[1] > rest_pos[1] + 0.45,
        details=f"rest={rest_pos}, retracted={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
