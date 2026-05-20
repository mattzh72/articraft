from __future__ import annotations

from math import cos, pi, sin, tan

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


def _add_handwheel_visuals(part, *, wheel_color, hub_color) -> None:
    wheel_plane_y = 0.025
    rim_radius = 0.27
    rim_thickness = 0.026
    segment_count = 8
    segment_length = 2.0 * rim_radius * tan(pi / segment_count) + rim_thickness * 0.95
    segment_center_radius = rim_radius * cos(pi / segment_count)

    for index in range(segment_count):
        mid_angle = 2.0 * pi * index / segment_count + pi / segment_count
        tangent_angle = mid_angle + pi / 2.0
        part.visual(
            Box((segment_length, rim_thickness, rim_thickness)),
            origin=Origin(
                xyz=(
                    segment_center_radius * cos(mid_angle),
                    wheel_plane_y,
                    segment_center_radius * sin(mid_angle),
                ),
                rpy=(0.0, tangent_angle, 0.0),
            ),
            material=wheel_color,
            name=f"rim_{index}",
        )

    spoke_length = 0.50
    for index, angle in enumerate((0.0, pi / 4.0, pi / 2.0, -pi / 4.0)):
        part.visual(
            Box((spoke_length, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, wheel_plane_y, 0.0), rpy=(0.0, angle, 0.0)),
            material=wheel_color,
            name=f"spoke_{index}",
        )

    part.visual(
        Cylinder(radius=0.056, length=0.050),
        origin=Origin(xyz=(0.0, wheel_plane_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_color,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_color,
        name="shaft_stub",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.19, wheel_plane_y + 0.01, -0.02), rpy=(0.0, 0.0, pi / 2.0)),
        material=hub_color,
        name="grip_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stormwater_sluice_gate")

    coated_steel = model.material("coated_steel", rgba=(0.29, 0.33, 0.36, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    galvanized = model.material("galvanized", rgba=(0.72, 0.75, 0.77, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.08, 0.09, 0.10, 1.0))
    safety_red = model.material("safety_red", rgba=(0.69, 0.12, 0.10, 1.0))
    pawl_yellow = model.material("pawl_yellow", rgba=(0.83, 0.68, 0.13, 1.0))

    opening_width = 1.80
    guide_width = 0.22
    frame_depth = 0.24
    guide_height = 2.95
    panel_offset_y = -0.035
    panel_width = 1.74
    panel_height = 2.10
    panel_thickness = 0.055

    frame = model.part("frame")
    half_opening = opening_width * 0.5
    guide_center_x = half_opening + guide_width * 0.5

    for side, sign in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            Box((0.16, frame_depth, guide_height)),
            origin=Origin(xyz=(sign * (half_opening + 0.08), 0.0, guide_height * 0.5 - 0.10)),
            material=coated_steel,
            name=f"{side}_guide_web",
        )
        frame.visual(
            Box((0.06, 0.05, 2.45)),
            origin=Origin(xyz=(sign * (half_opening - 0.025), 0.095, 1.125)),
            material=galvanized,
            name=f"{side}_retainer",
        )
        frame.visual(
            Box((0.04, 0.06, 2.40)),
            origin=Origin(xyz=(sign * (half_opening - 0.040), -0.090, 1.100)),
            material=seal_rubber,
            name=f"{side}_seal",
        )
        frame.visual(
            Box((0.06, 0.12, guide_height)),
            origin=Origin(xyz=(sign * (guide_center_x + 0.09), -0.02, guide_height * 0.5 - 0.10)),
            material=dark_steel,
            name=f"{side}_back_leg",
        )

    frame.visual(
        Box((opening_width + 2.0 * guide_width, 0.28, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=dark_steel,
        name="sill",
    )
    frame.visual(
        Box((opening_width + 2.0 * guide_width, frame_depth, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 2.58)),
        material=coated_steel,
        name="top_beam",
    )
    frame.visual(
        Box((opening_width + 0.10, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.080, 1.85)),
        material=galvanized,
        name="headseal_bar",
    )
    frame.visual(
        Box((opening_width + 0.28, 0.10, 0.30)),
        origin=Origin(xyz=(0.0, 0.085, 2.28)),
        material=coated_steel,
        name="bonnet_cover",
    )

    panel = model.part("panel")
    panel.visual(
        Box((panel_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(0.0, 0.0, panel_height * 0.5)),
        material=galvanized,
        name="panel_skin",
    )
    for index, rib_z in enumerate((0.36, 0.86, 1.36, 1.86)):
        panel.visual(
            Box((panel_width - 0.12, 0.065, 0.11)),
            origin=Origin(xyz=(0.0, 0.030, rib_z)),
            material=coated_steel,
            name=f"stiffener_{index}",
        )
    panel.visual(
        Box((0.58, 0.10, 0.16)),
        origin=Origin(xyz=(0.0, 0.015, 2.02)),
        material=coated_steel,
        name="top_rail",
    )
    panel.visual(
        Box((0.040, 0.040, panel_height)),
        origin=Origin(xyz=(-panel_width * 0.5 + 0.080, 0.028, panel_height * 0.5)),
        material=coated_steel,
        name="left_stile",
    )
    panel.visual(
        Box((0.040, 0.040, panel_height)),
        origin=Origin(xyz=(panel_width * 0.5 - 0.080, 0.028, panel_height * 0.5)),
        material=coated_steel,
        name="right_stile",
    )
    panel.visual(
        Box((panel_width - 0.10, 0.060, 0.08)),
        origin=Origin(xyz=(0.0, -0.004, 0.04)),
        material=seal_rubber,
        name="bottom_seal",
    )

    gearbox = model.part("gearbox")
    gearbox.visual(
        Box((0.24, 0.22, 0.10)),
        origin=Origin(xyz=(0.0, 0.02, 0.05)),
        material=dark_steel,
        name="mount_pad",
    )
    gearbox.visual(
        Box((0.42, 0.28, 0.32)),
        origin=Origin(xyz=(0.0, 0.14, 0.21)),
        material=coated_steel,
        name="housing",
    )
    gearbox.visual(
        Box((0.28, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, 0.10, 0.41)),
        material=dark_steel,
        name="service_cover",
    )
    gearbox.visual(
        Cylinder(radius=0.050, length=0.080),
        origin=Origin(xyz=(0.0, 0.320, 0.18), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="shaft_boss",
    )
    gearbox.visual(
        Box((0.18, 0.06, 0.12)),
        origin=Origin(xyz=(0.27, 0.29, 0.35)),
        material=dark_steel,
        name="pawl_support",
    )
    gearbox.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.33, 0.340, 0.35), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pawl_bracket",
    )

    handwheel = model.part("handwheel")
    _add_handwheel_visuals(handwheel, wheel_color=safety_red, hub_color=galvanized)

    pawl = model.part("pawl")
    pawl.visual(
        Cylinder(radius=0.017, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pawl_hub",
    )
    pawl.visual(
        Box((0.045, 0.016, 0.170)),
        origin=Origin(xyz=(-0.004, 0.020, -0.082)),
        material=pawl_yellow,
        name="pawl_arm",
    )
    pawl.visual(
        Box((0.020, 0.016, 0.045)),
        origin=Origin(xyz=(-0.026, 0.020, -0.158)),
        material=dark_steel,
        name="pawl_tooth",
    )
    pawl.visual(
        Box((0.030, 0.016, 0.052)),
        origin=Origin(xyz=(0.020, 0.020, -0.030)),
        material=pawl_yellow,
        name="pawl_handle",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, panel_offset_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.20, lower=0.0, upper=1.85),
    )
    model.articulation(
        "frame_to_gearbox",
        ArticulationType.FIXED,
        parent=frame,
        child=gearbox,
        origin=Origin(xyz=(0.66, 0.0, 2.73)),
    )
    model.articulation(
        "gearbox_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=gearbox,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.36, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=6.0),
    )
    model.articulation(
        "gearbox_to_pawl",
        ArticulationType.REVOLUTE,
        parent=gearbox,
        child=pawl,
        origin=Origin(xyz=(0.33, 0.36, 0.35)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.0, lower=-0.65, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    gearbox = object_model.get_part("gearbox")
    handwheel = object_model.get_part("handwheel")
    pawl = object_model.get_part("pawl")
    panel_slide = object_model.get_articulation("frame_to_panel")

    ctx.expect_gap(
        panel,
        frame,
        axis="x",
        positive_elem="panel_skin",
        negative_elem="left_guide_web",
        min_gap=0.02,
        max_gap=0.05,
        name="panel clears left guide web",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="x",
        positive_elem="right_guide_web",
        negative_elem="panel_skin",
        min_gap=0.02,
        max_gap=0.05,
        name="panel clears right guide web",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="y",
        positive_elem="left_retainer",
        negative_elem="panel_skin",
        min_gap=0.06,
        max_gap=0.14,
        name="panel stays behind the guide retainers",
    )
    ctx.expect_contact(
        handwheel,
        gearbox,
        elem_a="hub",
        elem_b="shaft_boss",
        contact_tol=1e-5,
        name="handwheel mounts on the gearbox shaft boss",
    )
    ctx.expect_contact(
        pawl,
        gearbox,
        elem_a="pawl_hub",
        elem_b="pawl_bracket",
        contact_tol=1e-5,
        name="pawl pivots from a gearbox-mounted bracket",
    )
    ctx.expect_origin_gap(
        handwheel,
        gearbox,
        axis="y",
        min_gap=0.30,
        max_gap=0.40,
        name="handwheel sits forward of the gearbox housing",
    )
    ctx.expect_origin_distance(
        pawl,
        handwheel,
        axes="xz",
        min_dist=0.18,
        max_dist=0.40,
        name="pawl sits beside the handwheel",
    )

    limits = panel_slide.motion_limits
    if limits is not None and limits.upper is not None:
        closed_pos = ctx.part_world_position(panel)
        with ctx.pose({panel_slide: limits.upper}):
            ctx.expect_overlap(
                panel,
                frame,
                axes="z",
                elem_a="panel_skin",
                elem_b="left_guide_web",
                min_overlap=0.75,
                name="lifted panel retains guide engagement",
            )
            ctx.expect_gap(
                panel,
                frame,
                axis="z",
                positive_elem="panel_skin",
                negative_elem="sill",
                min_gap=1.70,
                name="lifted panel clears the channel sill",
            )
            open_pos = ctx.part_world_position(panel)
        ctx.check(
            "panel slides upward through the guides",
            closed_pos is not None and open_pos is not None and open_pos[2] > closed_pos[2] + 1.70,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    return ctx.report()


object_model = build_object_model()
