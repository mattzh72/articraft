from __future__ import annotations

from math import radians

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
    model = ArticulatedObject(name="vertical_service_panel_axis")

    painted = model.material("charcoal_powder_coat", rgba=(0.08, 0.085, 0.09, 1.0))
    rib = model.material("dark_cast_rib", rgba=(0.045, 0.05, 0.055, 1.0))
    rail = model.material("brushed_linear_rail", rgba=(0.62, 0.64, 0.62, 1.0))
    beam_blue = model.material("anodized_beam_blue", rgba=(0.05, 0.18, 0.34, 1.0))
    beam_dark = model.material("dark_carriage_block", rgba=(0.06, 0.065, 0.07, 1.0))
    slide = model.material("ground_slide_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    sensor = model.material("amber_sensor_plastic", rgba=(0.95, 0.60, 0.12, 1.0))
    rubber = model.material("black_rubber_stop", rgba=(0.01, 0.01, 0.012, 1.0))

    back_frame = model.part("back_frame")
    back_frame.visual(
        Box((1.36, 0.040, 1.60)),
        origin=Origin(xyz=(0.0, -0.025, 0.80)),
        material=painted,
        name="back_plate",
    )
    for x, name in ((-0.68, "side_post_0"), (0.68, "side_post_1")):
        back_frame.visual(
            Box((0.08, 0.080, 1.60)),
            origin=Origin(xyz=(x, 0.005, 0.80)),
            material=rib,
            name=name,
        )
    for z, name in ((0.04, "bottom_crossmember"), (1.56, "top_crossmember")):
        back_frame.visual(
            Box((1.36, 0.080, 0.08)),
            origin=Origin(xyz=(0.0, 0.005, z)),
            material=rib,
            name=name,
        )
    back_frame.visual(
        Box((0.05, 0.060, 1.36)),
        origin=Origin(xyz=(0.0, -0.002, 0.80)),
        material=rib,
        name="center_stiffener",
    )
    for angle, name in ((-0.73, "diagonal_rib_0"), (0.73, "diagonal_rib_1")):
        back_frame.visual(
            Box((1.50, 0.032, 0.036)),
            origin=Origin(xyz=(0.0, -0.002, 0.80), rpy=(0.0, angle, 0.0)),
            material=rib,
            name=name,
        )

    back_frame.visual(
        Box((1.16, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, 0.013, 0.95)),
        material=painted,
        name="x_rail_base_lower",
    )
    back_frame.visual(
        Box((1.16, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.043, 0.95)),
        material=rail,
        name="x_rail_lower",
    )
    back_frame.visual(
        Box((1.16, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, 0.013, 1.23)),
        material=painted,
        name="x_rail_base_upper",
    )
    back_frame.visual(
        Box((1.16, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.043, 1.23)),
        material=rail,
        name="x_rail_upper",
    )
    for z, rail_name in ((0.95, "lower"), (1.23, "upper")):
        for x in (-0.48, -0.24, 0.0, 0.24, 0.48):
            back_frame.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(x, 0.030, z), rpy=(-radians(90.0), 0.0, 0.0)),
                material=rail,
                name=f"rail_bolt_{rail_name}_{x:+.2f}",
            )

    for x, name in ((-0.63, "travel_stop_0"), (0.63, "travel_stop_1")):
        back_frame.visual(
            Box((0.050, 0.082, 0.36)),
            origin=Origin(xyz=(x, 0.040, 1.09)),
            material=rubber,
            name=name,
        )
    for x, name in ((-0.64, "sensor_tab_0"), (0.64, "sensor_tab_1")):
        back_frame.visual(
            Box((0.020, 0.030, 0.110)),
            origin=Origin(xyz=(x, 0.070, 1.285)),
            material=painted,
            name=f"{name}_stem",
        )
        back_frame.visual(
            Box((0.050, 0.036, 0.080)),
            origin=Origin(xyz=(x, 0.088, 1.355)),
            material=sensor,
            name=name,
        )

    beam = model.part("beam")
    beam.visual(
        Box((0.60, 0.055, 0.46)),
        origin=Origin(xyz=(0.0, 0.095, 0.0)),
        material=beam_dark,
        name="rear_web",
    )
    beam.visual(
        Box((0.60, 0.120, 0.22)),
        origin=Origin(xyz=(0.0, 0.165, 0.02)),
        material=beam_blue,
        name="beam_body",
    )
    for z, name in ((0.125, "top_flange"), (-0.105, "bottom_flange")):
        beam.visual(
            Box((0.64, 0.140, 0.045)),
            origin=Origin(xyz=(0.0, 0.170, z)),
            material=beam_blue,
            name=name,
        )
    beam.visual(
        Box((0.160, 0.036, 0.065)),
        origin=Origin(xyz=(-0.20, 0.0785, 0.14)),
        material=rail,
        name="guide_upper_0",
    )
    beam.visual(
        Box((0.160, 0.036, 0.065)),
        origin=Origin(xyz=(0.20, 0.0785, 0.14)),
        material=rail,
        name="guide_upper_1",
    )
    beam.visual(
        Box((0.160, 0.036, 0.065)),
        origin=Origin(xyz=(-0.20, 0.0785, -0.14)),
        material=rail,
        name="guide_lower_0",
    )
    beam.visual(
        Box((0.160, 0.036, 0.065)),
        origin=Origin(xyz=(0.20, 0.0785, -0.14)),
        material=rail,
        name="guide_lower_1",
    )
    for x, name in ((-0.15, "front_pad_0"), (0.15, "front_pad_1")):
        beam.visual(
            Box((0.16, 0.022, 0.075)),
            origin=Origin(xyz=(x, 0.236, 0.035)),
            material=beam_dark,
            name=name,
        )
        for dz in (-0.018, 0.018):
            beam.visual(
                Cylinder(radius=0.010, length=0.010),
                origin=Origin(
                    xyz=(x, 0.252, 0.035 + dz),
                    rpy=(-radians(90.0), 0.0, 0.0),
                ),
                material=rail,
                name=f"{name}_screw_{dz:+.2f}",
            )

    beam.visual(
        Box((0.038, 0.110, 0.46)),
        origin=Origin(xyz=(-0.095, 0.285, -0.30)),
        material=beam_dark,
        name="z_guide_rail_0",
    )
    beam.visual(
        Box((0.038, 0.110, 0.46)),
        origin=Origin(xyz=(0.095, 0.285, -0.30)),
        material=beam_dark,
        name="z_guide_rail_1",
    )
    for x, suffix in ((-0.105, "0"), (0.105, "1")):
        beam.visual(
            Box((0.050, 0.055, 0.15)),
            origin=Origin(xyz=(x, 0.240, -0.105)),
            material=beam_dark,
            name=f"z_guide_mount_{suffix}",
        )
        beam.visual(
            Box((0.052, 0.115, 0.042)),
            origin=Origin(xyz=(x, 0.285, -0.548)),
            material=rubber,
            name=f"z_bottom_stop_{suffix}",
        )
    beam.visual(
        Box((0.040, 0.018, 0.145)),
        origin=Origin(xyz=(0.285, 0.070, 0.235)),
        material=sensor,
        name="sensor_flag",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((0.105, 0.060, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, -0.24)),
        material=slide,
        name="slide_spine",
    )
    z_slide.visual(
        Box((0.152, 0.035, 0.58)),
        origin=Origin(xyz=(0.0, 0.035, -0.32)),
        material=slide,
        name="front_cover",
    )
    z_slide.visual(
        Box((0.220, 0.080, 0.080)),
        origin=Origin(xyz=(0.0, 0.020, -0.660)),
        material=beam_dark,
        name="tool_plate",
    )
    z_slide.visual(
        Box((0.050, 0.026, 0.120)),
        origin=Origin(xyz=(-0.075, 0.090, -0.435)),
        material=sensor,
        name="lower_sensor_flag",
    )
    z_slide.visual(
        Box((0.018, 0.054, 0.100)),
        origin=Origin(xyz=(-0.062, 0.064, -0.435)),
        material=beam_dark,
        name="lower_sensor_stem",
    )

    model.articulation(
        "frame_to_beam",
        ArticulationType.PRISMATIC,
        parent=back_frame,
        child=beam,
        origin=Origin(xyz=(-0.28, 0.0, 1.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.45, lower=0.0, upper=0.56),
    )
    model.articulation(
        "beam_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=z_slide,
        origin=Origin(xyz=(0.0, 0.285, -0.12)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.30, lower=0.0, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_frame = object_model.get_part("back_frame")
    beam = object_model.get_part("beam")
    z_slide = object_model.get_part("z_slide")
    frame_to_beam = object_model.get_articulation("frame_to_beam")
    beam_to_z_slide = object_model.get_articulation("beam_to_z_slide")

    for q, label in ((0.0, "left"), (0.56, "right")):
        with ctx.pose({frame_to_beam: q, beam_to_z_slide: 0.0}):
            ctx.expect_gap(
                beam,
                back_frame,
                axis="y",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem="guide_upper_0",
                negative_elem="x_rail_upper",
                name=f"upper guide rides rail at {label} travel",
            )
            ctx.expect_gap(
                beam,
                back_frame,
                axis="y",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem="guide_lower_0",
                negative_elem="x_rail_lower",
                name=f"lower guide rides rail at {label} travel",
            )
            ctx.expect_within(
                beam,
                back_frame,
                axes="x",
                inner_elem="guide_upper_0",
                outer_elem="x_rail_upper",
                margin=0.0,
                name=f"left carriage pad stays on rail at {label} travel",
            )
            ctx.expect_within(
                beam,
                back_frame,
                axes="x",
                inner_elem="guide_upper_1",
                outer_elem="x_rail_upper",
                margin=0.0,
                name=f"right carriage pad stays on rail at {label} travel",
            )
            ctx.expect_overlap(
                beam,
                back_frame,
                axes="z",
                min_overlap=0.030,
                elem_a="guide_upper_0",
                elem_b="x_rail_upper",
                name=f"upper guide remains vertically engaged at {label} travel",
            )

    rest_z = ctx.part_world_position(z_slide)
    with ctx.pose({beam_to_z_slide: 0.30}):
        ctx.expect_overlap(
            z_slide,
            beam,
            axes="z",
            min_overlap=0.18,
            elem_a="slide_spine",
            elem_b="z_guide_rail_0",
            name="vertical slide retains guide overlap at full drop",
        )
        ctx.expect_gap(
            beam,
            z_slide,
            axis="x",
            max_gap=0.001,
            max_penetration=0.00001,
            positive_elem="z_guide_rail_1",
            negative_elem="front_cover",
            name="right vertical guide face supports front cover",
        )
        ctx.expect_gap(
            z_slide,
            beam,
            axis="x",
            max_gap=0.001,
            max_penetration=0.00001,
            positive_elem="front_cover",
            negative_elem="z_guide_rail_0",
            name="left vertical guide face supports front cover",
        )
        ctx.expect_gap(
            beam,
            z_slide,
            axis="x",
            min_gap=0.015,
            positive_elem="z_guide_rail_1",
            negative_elem="slide_spine",
            name="right vertical rail clears slide spine",
        )
        ctx.expect_gap(
            z_slide,
            beam,
            axis="x",
            min_gap=0.015,
            positive_elem="slide_spine",
            negative_elem="z_guide_rail_0",
            name="left vertical rail clears slide spine",
        )
        drop_z = ctx.part_world_position(z_slide)

    ctx.check(
        "z slide moves downward",
        rest_z is not None and drop_z is not None and drop_z[2] < rest_z[2] - 0.25,
        details=f"rest={rest_z}, dropped={drop_z}",
    )

    return ctx.report()


object_model = build_object_model()
