from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_drawer_shuttle")

    powder_dark = model.material("charcoal_powder_coat", rgba=(0.08, 0.09, 0.10, 1.0))
    powder_mid = model.material("blue_gray_powder_coat", rgba=(0.26, 0.34, 0.40, 1.0))
    powder_light = model.material("pale_zinc_tray", rgba=(0.56, 0.63, 0.66, 1.0))
    platform_mat = model.material("safety_orange_platform", rgba=(0.95, 0.42, 0.12, 1.0))
    rail_mat = model.material("black_acetal_slides", rgba=(0.015, 0.014, 0.013, 1.0))
    stop_mat = model.material("red_stop_tabs", rgba=(0.85, 0.08, 0.045, 1.0))
    fastener_mat = model.material("brushed_fasteners", rgba=(0.72, 0.70, 0.66, 1.0))

    def add_box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def add_side_fasteners(part, x_positions, y_abs, z, prefix):
        for i, x in enumerate(x_positions):
            add_box(part, f"{prefix}_fastener_{i}", (0.018, 0.004, 0.018), (x, y_abs, z), fastener_mat)
            add_box(part, f"{prefix}_fastener_{i + len(x_positions)}", (0.018, 0.004, 0.018), (x, -y_abs, z), fastener_mat)

    outer_tray = model.part("outer_tray")
    add_box(outer_tray, "outer_floor", (0.720, 0.460, 0.022), (0.360, 0.000, 0.011), powder_dark)
    add_box(outer_tray, "outer_wall_0", (0.720, 0.025, 0.140), (0.360, 0.2175, 0.070), powder_dark)
    add_box(outer_tray, "outer_wall_1", (0.720, 0.025, 0.140), (0.360, -0.2175, 0.070), powder_dark)
    add_box(outer_tray, "outer_rear_lip", (0.035, 0.460, 0.095), (0.0175, 0.000, 0.069), powder_dark)
    add_box(outer_tray, "outer_front_bridge", (0.030, 0.115, 0.026), (0.705, 0.000, 0.0345), powder_dark)
    # Folded outer lips turn outward from the tray wall so they visibly stiffen the
    # tray without intruding into the sliding envelope.
    add_box(outer_tray, "outer_folded_lip_0", (0.705, 0.030, 0.018), (0.367, 0.245, 0.142), powder_dark)
    add_box(outer_tray, "outer_folded_lip_1", (0.705, 0.030, 0.018), (0.367, -0.245, 0.142), powder_dark)
    add_box(outer_tray, "outer_rail_0", (0.580, 0.040, 0.012), (0.340, 0.185, 0.034), rail_mat)
    add_box(outer_tray, "outer_rail_1", (0.580, 0.040, 0.012), (0.340, -0.185, 0.034), rail_mat)
    add_box(outer_tray, "outer_stop_0", (0.040, 0.020, 0.026), (0.668, 0.232, 0.096), stop_mat)
    add_box(outer_tray, "outer_stop_1", (0.040, 0.020, 0.026), (0.668, -0.232, 0.096), stop_mat)
    add_box(outer_tray, "outer_stop_2", (0.040, 0.020, 0.026), (0.090, 0.232, 0.096), stop_mat)
    add_box(outer_tray, "outer_stop_3", (0.040, 0.020, 0.026), (0.090, -0.232, 0.096), stop_mat)
    add_side_fasteners(outer_tray, (0.135, 0.360, 0.585), 0.232, 0.107, "outer")

    middle_tray = model.part("middle_tray")
    add_box(middle_tray, "middle_floor", (0.640, 0.300, 0.018), (0.330, 0.000, 0.021), powder_mid)
    add_box(middle_tray, "middle_wall_0", (0.660, 0.020, 0.070), (0.330, 0.160, 0.047), powder_mid)
    add_box(middle_tray, "middle_wall_1", (0.660, 0.020, 0.070), (0.330, -0.160, 0.047), powder_mid)
    add_box(middle_tray, "middle_front_lip", (0.030, 0.340, 0.070), (0.645, 0.000, 0.047), powder_mid)
    add_box(middle_tray, "middle_rear_tie", (0.020, 0.250, 0.040), (0.020, 0.000, 0.040), powder_mid)
    add_box(middle_tray, "middle_folded_lip_0", (0.635, 0.024, 0.014), (0.330, 0.182, 0.088), powder_mid)
    add_box(middle_tray, "middle_folded_lip_1", (0.635, 0.024, 0.014), (0.330, -0.182, 0.088), powder_mid)
    add_box(middle_tray, "middle_runner_0", (0.580, 0.014, 0.013), (0.280, 0.173, 0.0065), rail_mat)
    add_box(middle_tray, "middle_runner_1", (0.580, 0.014, 0.013), (0.280, -0.173, 0.0065), rail_mat)
    add_box(middle_tray, "middle_rail_0", (0.480, 0.030, 0.010), (0.305, 0.135, 0.043), rail_mat)
    add_box(middle_tray, "middle_rail_1", (0.480, 0.030, 0.010), (0.305, -0.135, 0.043), rail_mat)
    add_box(middle_tray, "middle_stop_0", (0.032, 0.018, 0.022), (0.590, 0.143, 0.065), stop_mat)
    add_box(middle_tray, "middle_stop_1", (0.032, 0.018, 0.022), (0.590, -0.143, 0.065), stop_mat)
    add_box(middle_tray, "middle_stop_2", (0.032, 0.018, 0.022), (0.085, 0.143, 0.065), stop_mat)
    add_box(middle_tray, "middle_stop_3", (0.032, 0.018, 0.022), (0.085, -0.143, 0.065), stop_mat)
    add_side_fasteners(middle_tray, (0.155, 0.355, 0.555), 0.169, 0.058, "middle")

    inner_tray = model.part("inner_tray")
    add_box(inner_tray, "inner_floor", (0.520, 0.214, 0.016), (0.270, 0.000, 0.018), powder_light)
    add_box(inner_tray, "inner_wall_0", (0.540, 0.018, 0.055), (0.270, 0.116, 0.0375), powder_light)
    add_box(inner_tray, "inner_wall_1", (0.540, 0.018, 0.055), (0.270, -0.116, 0.0375), powder_light)
    add_box(inner_tray, "inner_front_lip_0", (0.025, 0.042, 0.055), (0.5275, 0.104, 0.0375), powder_light)
    add_box(inner_tray, "inner_front_lip_1", (0.025, 0.042, 0.055), (0.5275, -0.104, 0.0375), powder_light)
    add_box(inner_tray, "inner_rear_tie", (0.018, 0.180, 0.032), (0.018, 0.000, 0.032), powder_light)
    add_box(inner_tray, "inner_folded_lip_0", (0.515, 0.020, 0.012), (0.270, 0.133, 0.070), powder_light)
    add_box(inner_tray, "inner_folded_lip_1", (0.515, 0.020, 0.012), (0.270, -0.133, 0.070), powder_light)
    add_box(inner_tray, "inner_runner_0", (0.470, 0.012, 0.010), (0.230, 0.127, 0.005), rail_mat)
    add_box(inner_tray, "inner_runner_1", (0.470, 0.012, 0.010), (0.230, -0.127, 0.005), rail_mat)
    add_box(inner_tray, "inner_rail_0", (0.320, 0.020, 0.010), (0.240, 0.097, 0.032), rail_mat)
    add_box(inner_tray, "inner_rail_1", (0.320, 0.020, 0.010), (0.240, -0.097, 0.032), rail_mat)
    add_box(inner_tray, "inner_stop_0", (0.026, 0.016, 0.020), (0.478, 0.101, 0.052), stop_mat)
    add_box(inner_tray, "inner_stop_1", (0.026, 0.016, 0.020), (0.478, -0.101, 0.052), stop_mat)
    add_box(inner_tray, "inner_stop_2", (0.026, 0.016, 0.020), (0.080, 0.108, 0.058), stop_mat)
    add_box(inner_tray, "inner_stop_3", (0.026, 0.016, 0.020), (0.080, -0.108, 0.058), stop_mat)
    add_side_fasteners(inner_tray, (0.145, 0.315, 0.465), 0.127, 0.046, "inner")

    terminal_platform = model.part("terminal_platform")
    add_box(terminal_platform, "platform_deck", (0.340, 0.160, 0.016), (0.180, 0.000, 0.018), platform_mat)
    add_box(terminal_platform, "platform_lip_0", (0.340, 0.012, 0.035), (0.180, 0.086, 0.0275), platform_mat)
    add_box(terminal_platform, "platform_lip_1", (0.340, 0.012, 0.035), (0.180, -0.086, 0.0275), platform_mat)
    add_box(terminal_platform, "platform_front_lip", (0.026, 0.160, 0.040), (0.347, 0.000, 0.030), platform_mat)
    add_box(terminal_platform, "platform_handle_pad", (0.055, 0.105, 0.012), (0.365, 0.000, 0.055), rail_mat)
    add_box(terminal_platform, "platform_runner_0", (0.260, 0.010, 0.010), (0.135, 0.093, 0.005), rail_mat)
    add_box(terminal_platform, "platform_runner_1", (0.260, 0.010, 0.010), (0.135, -0.093, 0.005), rail_mat)
    add_box(terminal_platform, "platform_stop_0", (0.022, 0.013, 0.018), (0.306, 0.084, 0.047), stop_mat)
    add_box(terminal_platform, "platform_stop_1", (0.022, 0.013, 0.018), (0.306, -0.084, 0.047), stop_mat)

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_tray,
        child=middle_tray,
        origin=Origin(xyz=(0.047, 0.000, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.28, lower=0.0, upper=0.280),
        motion_properties=MotionProperties(damping=8.0, friction=1.5),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_tray,
        child=inner_tray,
        origin=Origin(xyz=(0.045, 0.000, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.25, lower=0.0, upper=0.220),
        motion_properties=MotionProperties(damping=6.0, friction=1.2),
    )
    model.articulation(
        "inner_to_platform",
        ArticulationType.PRISMATIC,
        parent=inner_tray,
        child=terminal_platform,
        origin=Origin(xyz=(0.035, 0.000, 0.037)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.22, lower=0.0, upper=0.160),
        motion_properties=MotionProperties(damping=4.0, friction=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_tray")
    middle = object_model.get_part("middle_tray")
    inner = object_model.get_part("inner_tray")
    platform = object_model.get_part("terminal_platform")
    outer_slide = object_model.get_articulation("outer_to_middle")
    middle_slide = object_model.get_articulation("middle_to_inner")
    inner_slide = object_model.get_articulation("inner_to_platform")

    def expect_stage_supported(child, parent, child_runner, parent_rail, min_x_overlap, name):
        ctx.expect_within(
            child,
            parent,
            axes="y",
            inner_elem=child_runner,
            outer_elem=parent_rail,
            margin=0.001,
            name=f"{name} runner centered over rail",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="z",
            positive_elem=child_runner,
            negative_elem=parent_rail,
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{name} runner seated on rail",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="x",
            elem_a=child_runner,
            elem_b=parent_rail,
            min_overlap=min_x_overlap,
            name=f"{name} retained rail insertion",
        )

    def expect_side_clearance(child, parent, pos_child_wall, neg_child_wall, pos_parent_wall, neg_parent_wall, min_gap, name):
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            positive_elem=pos_parent_wall,
            negative_elem=pos_child_wall,
            min_gap=min_gap,
            name=f"{name} positive side wall clearance",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            positive_elem=neg_child_wall,
            negative_elem=neg_parent_wall,
            min_gap=min_gap,
            name=f"{name} negative side wall clearance",
        )

    with ctx.pose({outer_slide: 0.0, middle_slide: 0.0, inner_slide: 0.0}):
        expect_stage_supported(middle, outer, "middle_runner_0", "outer_rail_0", 0.50, "middle closed")
        expect_stage_supported(middle, outer, "middle_runner_1", "outer_rail_1", 0.50, "middle closed lower side")
        expect_stage_supported(inner, middle, "inner_runner_0", "middle_rail_0", 0.40, "inner closed")
        expect_stage_supported(inner, middle, "inner_runner_1", "middle_rail_1", 0.40, "inner closed lower side")
        expect_stage_supported(platform, inner, "platform_runner_0", "inner_rail_0", 0.22, "platform closed")
        expect_stage_supported(platform, inner, "platform_runner_1", "inner_rail_1", 0.22, "platform closed lower side")
        expect_side_clearance(middle, outer, "middle_wall_0", "middle_wall_1", "outer_wall_0", "outer_wall_1", 0.020, "middle closed")
        expect_side_clearance(inner, middle, "inner_wall_0", "inner_wall_1", "middle_wall_0", "middle_wall_1", 0.014, "inner closed")
        expect_side_clearance(platform, inner, "platform_lip_0", "platform_lip_1", "inner_wall_0", "inner_wall_1", 0.008, "platform closed")
        ctx.expect_gap(
            middle,
            outer,
            axis="z",
            positive_elem="middle_front_lip",
            negative_elem="outer_front_bridge",
            min_gap=0.002,
            name="closed middle front lip clears outer bridge",
        )
        closed_positions = {
            "middle": ctx.part_world_position(middle),
            "inner": ctx.part_world_position(inner),
            "platform": ctx.part_world_position(platform),
        }

    with ctx.pose({outer_slide: 0.280, middle_slide: 0.220, inner_slide: 0.160}):
        expect_stage_supported(middle, outer, "middle_runner_0", "outer_rail_0", 0.30, "middle extended")
        expect_stage_supported(middle, outer, "middle_runner_1", "outer_rail_1", 0.30, "middle extended lower side")
        expect_stage_supported(inner, middle, "inner_runner_0", "middle_rail_0", 0.24, "inner extended")
        expect_stage_supported(inner, middle, "inner_runner_1", "middle_rail_1", 0.24, "inner extended lower side")
        expect_stage_supported(platform, inner, "platform_runner_0", "inner_rail_0", 0.11, "platform extended")
        expect_stage_supported(platform, inner, "platform_runner_1", "inner_rail_1", 0.11, "platform extended lower side")
        expect_side_clearance(middle, outer, "middle_wall_0", "middle_wall_1", "outer_wall_0", "outer_wall_1", 0.020, "middle extended")
        expect_side_clearance(inner, middle, "inner_wall_0", "inner_wall_1", "middle_wall_0", "middle_wall_1", 0.014, "inner extended")
        expect_side_clearance(platform, inner, "platform_lip_0", "platform_lip_1", "inner_wall_0", "inner_wall_1", 0.008, "platform extended")
        ctx.expect_gap(
            inner,
            platform,
            axis="y",
            positive_elem="inner_front_lip_0",
            negative_elem="platform_front_lip",
            min_gap=0.002,
            name="platform clears positive inner front stop",
        )
        ctx.expect_gap(
            platform,
            inner,
            axis="y",
            positive_elem="platform_front_lip",
            negative_elem="inner_front_lip_1",
            min_gap=0.002,
            name="platform clears negative inner front stop",
        )
        extended_positions = {
            "middle": ctx.part_world_position(middle),
            "inner": ctx.part_world_position(inner),
            "platform": ctx.part_world_position(platform),
        }

    for name, required_delta in (("middle", 0.27), ("inner", 0.48), ("platform", 0.63)):
        closed = closed_positions[name]
        extended = extended_positions[name]
        ctx.check(
            f"{name} moves outward on serial slides",
            closed is not None and extended is not None and extended[0] > closed[0] + required_delta,
            details=f"closed={closed}, extended={extended}",
        )

    return ctx.report()


object_model = build_object_model()
