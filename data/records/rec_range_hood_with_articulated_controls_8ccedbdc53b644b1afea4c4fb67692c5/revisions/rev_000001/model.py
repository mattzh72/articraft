from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_out_visor_range_hood")

    painted_steel = model.material("painted_steel", rgba=(0.90, 0.91, 0.93, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    slider_black = model.material("slider_black", rgba=(0.07, 0.07, 0.08, 1.0))
    lamp_lens = model.material("lamp_lens", rgba=(0.96, 0.93, 0.80, 0.55))
    smoked_glass = model.material("smoked_glass", rgba=(0.30, 0.34, 0.36, 0.45))

    hood_width = 0.60
    hood_depth = 0.30
    hood_height = 0.17
    wall_thickness = 0.018
    top_thickness = 0.020
    inner_width = hood_width - 2.0 * wall_thickness

    housing = model.part("housing")
    housing.visual(
        Box((hood_width, hood_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, hood_height - top_thickness / 2.0)),
        material=painted_steel,
        name="top_shell",
    )
    housing.visual(
        Box((wall_thickness, 0.272, hood_height - top_thickness)),
        origin=Origin(
            xyz=(
                -hood_width / 2.0 + wall_thickness / 2.0,
                -0.014,
                (hood_height - top_thickness) / 2.0,
            )
        ),
        material=painted_steel,
        name="left_side",
    )
    housing.visual(
        Box((wall_thickness, 0.272, hood_height - top_thickness)),
        origin=Origin(
            xyz=(
                hood_width / 2.0 - wall_thickness / 2.0,
                -0.014,
                (hood_height - top_thickness) / 2.0,
            )
        ),
        material=painted_steel,
        name="right_side",
    )
    housing.visual(
        Box((inner_width, 0.020, hood_height - top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -hood_depth / 2.0 + 0.010,
                (hood_height - top_thickness) / 2.0,
            )
        ),
        material=painted_steel,
        name="back_wall",
    )
    housing.visual(
        Box((inner_width, 0.028, 0.078)),
        origin=Origin(
            xyz=(
                0.0,
                hood_depth / 2.0 - 0.014,
                hood_height - top_thickness - 0.039,
            )
        ),
        material=painted_steel,
        name="front_header",
    )
    housing.visual(
        Box((inner_width, 0.180, 0.008)),
        origin=Origin(xyz=(0.0, -0.040, 0.004)),
        material=charcoal,
        name="filter_panel",
    )
    housing.visual(
        Box((0.022, 0.190, 0.012)),
        origin=Origin(xyz=(-0.271, 0.030, 0.086)),
        material=charcoal,
        name="left_track",
    )
    housing.visual(
        Box((0.022, 0.190, 0.012)),
        origin=Origin(xyz=(0.271, 0.030, 0.086)),
        material=charcoal,
        name="right_track",
    )
    housing.visual(
        Box((0.065, 0.035, 0.004)),
        origin=Origin(xyz=(-0.120, 0.015, 0.002)),
        material=lamp_lens,
        name="left_lamp",
    )
    housing.visual(
        Box((0.065, 0.035, 0.004)),
        origin=Origin(xyz=(0.120, 0.015, 0.002)),
        material=lamp_lens,
        name="right_lamp",
    )
    housing.inertial = Inertial.from_geometry(
        Box((hood_width, hood_depth, hood_height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, hood_height / 2.0)),
    )

    visor = model.part("visor")
    visor.visual(
        Box((0.560, 0.230, 0.010)),
        origin=Origin(xyz=(0.0, -0.115, 0.0)),
        material=brushed_metal,
        name="visor_tray",
    )
    visor.visual(
        Box((0.560, 0.028, 0.046)),
        origin=Origin(xyz=(0.0, -0.014, -0.023)),
        material=brushed_metal,
        name="visor_front_lip",
    )
    visor.visual(
        Box((0.500, 0.140, 0.004)),
        origin=Origin(xyz=(0.0, -0.105, -0.007)),
        material=smoked_glass,
        name="visor_glass",
    )
    visor.visual(
        Box((0.018, 0.180, 0.024)),
        origin=Origin(xyz=(-0.271, -0.100, 0.017)),
        material=charcoal,
        name="left_guide_tongue",
    )
    visor.visual(
        Box((0.018, 0.180, 0.024)),
        origin=Origin(xyz=(0.271, -0.100, 0.017)),
        material=charcoal,
        name="right_guide_tongue",
    )
    visor.visual(
        Box((0.003, 0.040, 0.018)),
        origin=Origin(xyz=(0.2785, -0.014, -0.018)),
        material=charcoal,
        name="power_slot",
    )
    visor.inertial = Inertial.from_geometry(
        Box((0.560, 0.230, 0.055)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.115, -0.012)),
    )

    model.articulation(
        "housing_to_visor",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=visor,
        origin=Origin(xyz=(0.0, hood_depth / 2.0, 0.047)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.15,
            lower=0.0,
            upper=0.120,
        ),
    )

    power_slider = model.part("power_slider")
    power_slider.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=slider_black,
        name="slider_knob",
    )
    power_slider.visual(
        Box((0.004, 0.008, 0.002)),
        origin=Origin(xyz=(0.014, 0.0, 0.002)),
        material=slider_black,
        name="slider_ridge",
    )
    power_slider.inertial = Inertial.from_geometry(
        Box((0.018, 0.018, 0.010)),
        mass=0.05,
        origin=Origin(xyz=(0.009, 0.0, 0.001)),
    )

    model.articulation(
        "visor_to_power_slider",
        ArticulationType.PRISMATIC,
        parent=visor,
        child=power_slider,
        origin=Origin(xyz=(0.280, -0.014, -0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=-0.008,
            upper=0.008,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    visor = object_model.get_part("visor")
    power_slider = object_model.get_part("power_slider")
    visor_slide = object_model.get_articulation("housing_to_visor")
    power_slider_slide = object_model.get_articulation("visor_to_power_slider")

    with ctx.pose({visor_slide: 0.0, power_slider_slide: 0.0}):
        ctx.expect_within(
            visor,
            housing,
            axes="x",
            inner_elem="visor_tray",
            outer_elem="top_shell",
            margin=0.0,
            name="visor tray fits between the housing sides",
        )
        ctx.expect_overlap(
            visor,
            housing,
            axes="y",
            elem_a="visor_tray",
            elem_b="top_shell",
            min_overlap=0.220,
            name="retracted visor remains deeply nested in the hood body",
        )
        ctx.expect_within(
            power_slider,
            visor,
            axes="yz",
            inner_elem="slider_knob",
            outer_elem="power_slot",
            margin=0.0,
            name="power slider sits within its side slot at rest",
        )
        visor_rest_pos = ctx.part_world_position(visor)

    with ctx.pose({visor_slide: 0.120, power_slider_slide: 0.0}):
        ctx.expect_overlap(
            visor,
            housing,
            axes="y",
            elem_a="visor_tray",
            elem_b="top_shell",
            min_overlap=0.100,
            name="extended visor keeps retained insertion inside the housing",
        )
        visor_extended_pos = ctx.part_world_position(visor)

    ctx.check(
        "visor translates forward",
        visor_rest_pos is not None
        and visor_extended_pos is not None
        and visor_extended_pos[1] > visor_rest_pos[1] + 0.10,
        details=f"rest={visor_rest_pos}, extended={visor_extended_pos}",
    )

    with ctx.pose({power_slider_slide: -0.008}):
        slider_rear_pos = ctx.part_world_position(power_slider)
        ctx.expect_within(
            power_slider,
            visor,
            axes="yz",
            inner_elem="slider_knob",
            outer_elem="power_slot",
            margin=0.0,
            name="power slider stays captured at the rear of the slot",
        )

    with ctx.pose({power_slider_slide: 0.008}):
        slider_front_pos = ctx.part_world_position(power_slider)
        ctx.expect_within(
            power_slider,
            visor,
            axes="yz",
            inner_elem="slider_knob",
            outer_elem="power_slot",
            margin=0.0,
            name="power slider stays captured at the front of the slot",
        )

    ctx.check(
        "power slider moves along the side slot",
        slider_rear_pos is not None
        and slider_front_pos is not None
        and slider_front_pos[1] > slider_rear_pos[1] + 0.012,
        details=f"rear={slider_rear_pos}, front={slider_front_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
