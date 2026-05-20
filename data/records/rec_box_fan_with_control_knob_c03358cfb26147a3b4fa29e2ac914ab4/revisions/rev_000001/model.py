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


MAIN_WIDTH = 0.64
BODY_HEIGHT = 0.30
BODY_DEPTH = 0.10
FRAME_THICKNESS = 0.016
DIVIDER_THICKNESS = 0.020
CHAMBER_HEIGHT = BODY_HEIGHT - 2.0 * FRAME_THICKNESS
CHAMBER_WIDTH = (MAIN_WIDTH - 2.0 * FRAME_THICKNESS - DIVIDER_THICKNESS) / 2.0
FAN_CENTER_X = DIVIDER_THICKNESS / 2.0 + CHAMBER_WIDTH / 2.0
FAN_CENTER_LEFT = -FAN_CENTER_X
FAN_CENTER_RIGHT = FAN_CENTER_X

EXTENSION_WIDTH = 0.18
EXTENSION_HEIGHT = 0.28
EXTENSION_DEPTH = 0.016
EXTENSION_TRAVEL = 0.12
GUIDE_LENGTH = 0.18
GUIDE_CENTER_X = MAIN_WIDTH / 2.0 + 0.07
GUIDE_Y = BODY_HEIGHT / 2.0 + 0.008


def _add_propeller(part, blade_material, hub_material) -> None:
    part.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=hub_material,
        name="hub_cap",
    )

    blade_size = (0.118, 0.034, 0.006)
    blade_offset = 0.060
    blade_pitch = 0.30
    for idx, yaw in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        part.visual(
            Box(blade_size),
            origin=Origin(
                xyz=(blade_offset, 0.0, 0.004),
                rpy=(0.0, blade_pitch, yaw),
            ),
            material=blade_material,
            name=f"blade_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_window_fan")

    housing_color = model.material("housing", rgba=(0.90, 0.91, 0.88, 1.0))
    fan_blade_color = model.material("fan_blade", rgba=(0.24, 0.26, 0.28, 1.0))
    fan_hub_color = model.material("fan_hub", rgba=(0.16, 0.17, 0.19, 1.0))
    knob_color = model.material("knob", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_color = model.material("accent", rgba=(0.73, 0.76, 0.78, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((MAIN_WIDTH, FRAME_THICKNESS, BODY_DEPTH)),
        origin=Origin(xyz=(0.0, BODY_HEIGHT / 2.0 - FRAME_THICKNESS / 2.0, 0.0)),
        material=housing_color,
        name="top_frame",
    )
    housing.visual(
        Box((MAIN_WIDTH, FRAME_THICKNESS, BODY_DEPTH)),
        origin=Origin(xyz=(0.0, -BODY_HEIGHT / 2.0 + FRAME_THICKNESS / 2.0, 0.0)),
        material=housing_color,
        name="bottom_frame",
    )
    housing.visual(
        Box((FRAME_THICKNESS, CHAMBER_HEIGHT, BODY_DEPTH)),
        origin=Origin(xyz=(-MAIN_WIDTH / 2.0 + FRAME_THICKNESS / 2.0, 0.0, 0.0)),
        material=housing_color,
        name="left_frame",
    )
    housing.visual(
        Box((FRAME_THICKNESS, CHAMBER_HEIGHT, BODY_DEPTH)),
        origin=Origin(xyz=(MAIN_WIDTH / 2.0 - FRAME_THICKNESS / 2.0, 0.0, 0.0)),
        material=housing_color,
        name="right_frame",
    )
    housing.visual(
        Box((DIVIDER_THICKNESS, CHAMBER_HEIGHT, BODY_DEPTH)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=housing_color,
        name="center_divider",
    )
    housing.visual(
        Box((0.11, 0.06, 0.022)),
        origin=Origin(xyz=(0.0, -0.090, -BODY_DEPTH / 2.0 + 0.011)),
        material=housing_color,
        name="control_pod",
    )

    guide_sleeve_size = (GUIDE_LENGTH, 0.014, 0.012)
    bracket_size = (0.012, 0.020, 0.012)
    for suffix, guide_y, bracket_y in (
        ("top", GUIDE_Y, BODY_HEIGHT / 2.0 + 0.001),
        ("bottom", -GUIDE_Y, -BODY_HEIGHT / 2.0 - 0.001),
    ):
        housing.visual(
            Box(guide_sleeve_size),
            origin=Origin(xyz=(GUIDE_CENTER_X, guide_y, 0.0)),
            material=accent_color,
            name=f"{suffix}_guide_sleeve",
        )
        housing.visual(
            Box((GUIDE_LENGTH, 0.010, 0.024)),
            origin=Origin(xyz=(GUIDE_CENTER_X, guide_y, BODY_DEPTH / 2.0 - 0.032)),
            material=accent_color,
            name=f"{suffix}_guide_rear_web",
        )
        for side, x in (("inner", MAIN_WIDTH / 2.0 - 0.006), ("outer", GUIDE_CENTER_X + GUIDE_LENGTH / 2.0 - 0.018)):
            housing.visual(
                Box(bracket_size),
                origin=Origin(xyz=(x, bracket_y, BODY_DEPTH / 2.0 - 0.014)),
                material=housing_color,
                name=f"{suffix}_guide_bracket_{side}",
            )

    for fan_name, fan_x in (("left", FAN_CENTER_LEFT), ("right", FAN_CENTER_RIGHT)):
        housing.visual(
            Cylinder(radius=0.034, length=0.014),
            origin=Origin(xyz=(fan_x, 0.0, -0.021)),
            material=accent_color,
            name=f"{fan_name}_motor_pod",
        )
        housing.visual(
            Box((0.118, 0.010, 0.006)),
            origin=Origin(xyz=(fan_x - 0.090, 0.0, -0.023)),
            material=accent_color,
            name=f"{fan_name}_brace_left",
        )
        housing.visual(
            Box((0.118, 0.010, 0.006)),
            origin=Origin(xyz=(fan_x + 0.090, 0.0, -0.023)),
            material=accent_color,
            name=f"{fan_name}_brace_right",
        )
        housing.visual(
            Box((0.010, 0.118, 0.006)),
            origin=Origin(xyz=(fan_x, 0.090, -0.023)),
            material=accent_color,
            name=f"{fan_name}_brace_top",
        )
        housing.visual(
            Box((0.010, 0.118, 0.006)),
            origin=Origin(xyz=(fan_x, -0.090, -0.023)),
            material=accent_color,
            name=f"{fan_name}_brace_bottom",
        )
        for idx, slat_offset in enumerate((-0.063, -0.021, 0.021, 0.063)):
            housing.visual(
                Box((0.007, CHAMBER_HEIGHT, 0.005)),
                origin=Origin(xyz=(fan_x + slat_offset, 0.0, 0.032)),
                material=housing_color,
                name=f"{fan_name}_front_slat_{idx}",
            )

    left_propeller = model.part("left_propeller")
    _add_propeller(left_propeller, fan_blade_color, fan_hub_color)

    right_propeller = model.part("right_propeller")
    _add_propeller(right_propeller, fan_blade_color, fan_hub_color)

    extension_panel = model.part("extension_panel")
    extension_half_h = EXTENSION_HEIGHT / 2.0
    panel_inner_center_x = -0.064
    panel_outer_center_x = 0.104
    panel_span_center_x = (panel_inner_center_x + panel_outer_center_x) / 2.0
    panel_span_width = (panel_outer_center_x - panel_inner_center_x) + 0.012
    extension_panel.visual(
        Box((panel_span_width, 0.012, EXTENSION_DEPTH)),
        origin=Origin(xyz=(panel_span_center_x, extension_half_h - 0.006, 0.0)),
        material=housing_color,
        name="top_rail",
    )
    extension_panel.visual(
        Box((panel_span_width, 0.012, EXTENSION_DEPTH)),
        origin=Origin(xyz=(panel_span_center_x, -extension_half_h + 0.006, 0.0)),
        material=housing_color,
        name="bottom_rail",
    )
    extension_panel.visual(
        Box((0.012, EXTENSION_HEIGHT - 0.024, EXTENSION_DEPTH)),
        origin=Origin(xyz=(panel_inner_center_x, 0.0, 0.0)),
        material=housing_color,
        name="left_rail",
    )
    extension_panel.visual(
        Box((0.012, EXTENSION_HEIGHT - 0.024, EXTENSION_DEPTH)),
        origin=Origin(xyz=(panel_outer_center_x, 0.0, 0.0)),
        material=housing_color,
        name="right_rail",
    )
    for idx, slat_x in enumerate((-0.020, 0.025, 0.070)):
        extension_panel.visual(
            Box((0.009, EXTENSION_HEIGHT - 0.024, 0.010)),
            origin=Origin(xyz=(slat_x, 0.0, 0.0)),
            material=housing_color,
            name=f"panel_slat_{idx}",
        )
    extension_panel.visual(
        Box((0.170, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, GUIDE_Y - 0.002, 0.0)),
        material=accent_color,
        name="top_tongue",
    )
    extension_panel.visual(
        Box((0.170, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -GUIDE_Y + 0.002, 0.0)),
        material=accent_color,
        name="bottom_tongue",
    )
    for idx, connector_x in enumerate((-0.040, 0.040)):
        extension_panel.visual(
            Box((0.016, 0.011, 0.008)),
            origin=Origin(xyz=(connector_x, extension_half_h + 0.0055, 0.0)),
            material=accent_color,
            name=f"top_connector_{idx}",
        )
        extension_panel.visual(
            Box((0.016, 0.011, 0.008)),
            origin=Origin(xyz=(connector_x, -extension_half_h - 0.0055, 0.0)),
            material=accent_color,
            name=f"bottom_connector_{idx}",
        )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=knob_color,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=fan_hub_color,
        name="knob_base",
    )
    speed_knob.visual(
        Box((0.018, 0.005, 0.004)),
        origin=Origin(xyz=(0.012, 0.0, -0.022)),
        material=accent_color,
        name="indicator_ridge",
    )

    model.articulation(
        "housing_to_left_propeller",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_propeller,
        origin=Origin(xyz=(FAN_CENTER_LEFT, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=28.0),
    )
    model.articulation(
        "housing_to_right_propeller",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_propeller,
        origin=Origin(xyz=(FAN_CENTER_RIGHT, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=28.0),
    )
    model.articulation(
        "housing_to_extension_panel",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=extension_panel,
        origin=Origin(xyz=(GUIDE_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.25, lower=0.0, upper=EXTENSION_TRAVEL),
    )
    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(0.0, -0.090, -BODY_DEPTH / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=4.71),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    extension_panel = object_model.get_part("extension_panel")
    speed_knob = object_model.get_part("speed_knob")

    left_spin = object_model.get_articulation("housing_to_left_propeller")
    right_spin = object_model.get_articulation("housing_to_right_propeller")
    slide = object_model.get_articulation("housing_to_extension_panel")
    knob_joint = object_model.get_articulation("housing_to_speed_knob")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        housing,
        extension_panel,
        elem_a="top_guide_sleeve",
        elem_b="top_tongue",
        reason="The panel tongue is intentionally represented as sliding inside a solid top guide sleeve proxy.",
    )
    ctx.allow_overlap(
        housing,
        extension_panel,
        elem_a="bottom_guide_sleeve",
        elem_b="bottom_tongue",
        reason="The panel tongue is intentionally represented as sliding inside a solid bottom guide sleeve proxy.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_propeller,
        housing,
        elem_a="hub",
        elem_b="left_motor_pod",
        name="left propeller hub seats on the motor pod",
    )
    ctx.expect_contact(
        right_propeller,
        housing,
        elem_a="hub",
        elem_b="right_motor_pod",
        name="right propeller hub seats on the motor pod",
    )
    ctx.expect_contact(
        extension_panel,
        housing,
        elem_a="left_rail",
        elem_b="right_frame",
        name="extension panel rests flush against the housing at minimum width",
    )
    ctx.expect_contact(
        speed_knob,
        housing,
        elem_a="knob_body",
        elem_b="control_pod",
        name="speed knob mounts on the rear control pod",
    )

    ctx.expect_within(
        extension_panel,
        housing,
        axes="yz",
        inner_elem="top_tongue",
        outer_elem="top_guide_sleeve",
        margin=0.003,
        name="top tongue stays centered in the upper guide sleeve",
    )
    ctx.expect_within(
        extension_panel,
        housing,
        axes="yz",
        inner_elem="bottom_tongue",
        outer_elem="bottom_guide_sleeve",
        margin=0.003,
        name="bottom tongue stays centered in the lower guide sleeve",
    )
    ctx.expect_overlap(
        extension_panel,
        housing,
        axes="x",
        elem_a="top_tongue",
        elem_b="top_guide_sleeve",
        min_overlap=0.10,
        name="top guide tongue has substantial retained insertion at rest",
    )
    ctx.expect_overlap(
        extension_panel,
        housing,
        axes="x",
        elem_a="bottom_tongue",
        elem_b="bottom_guide_sleeve",
        min_overlap=0.10,
        name="bottom guide tongue has substantial retained insertion at rest",
    )

    rest_panel_pos = ctx.part_world_position(extension_panel)
    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else None
    with ctx.pose({slide: slide_upper if slide_upper is not None else 0.0}):
        ctx.expect_within(
            extension_panel,
            housing,
            axes="yz",
            inner_elem="top_tongue",
            outer_elem="top_guide_sleeve",
            margin=0.003,
            name="top tongue stays centered when extended",
        )
        ctx.expect_within(
            extension_panel,
            housing,
            axes="yz",
            inner_elem="bottom_tongue",
            outer_elem="bottom_guide_sleeve",
            margin=0.003,
            name="bottom tongue stays centered when extended",
        )
        ctx.expect_overlap(
            extension_panel,
            housing,
            axes="x",
            elem_a="top_tongue",
            elem_b="top_guide_sleeve",
            min_overlap=0.05,
            name="top tongue retains insertion at maximum extension",
        )
        ctx.expect_overlap(
            extension_panel,
            housing,
            axes="x",
            elem_a="bottom_tongue",
            elem_b="bottom_guide_sleeve",
            min_overlap=0.05,
            name="bottom tongue retains insertion at maximum extension",
        )
        extended_panel_pos = ctx.part_world_position(extension_panel)

    ctx.check(
        "extension panel slides outward along +X",
        rest_panel_pos is not None
        and extended_panel_pos is not None
        and extended_panel_pos[0] > rest_panel_pos[0] + 0.10,
        details=f"rest={rest_panel_pos}, extended={extended_panel_pos}",
    )

    ctx.check(
        "dual propellers use continuous front-to-back spin axes",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in left_spin.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 3) for v in right_spin.axis) == (0.0, 0.0, 1.0),
        details=f"left={left_spin.axis}, right={right_spin.axis}",
    )
    ctx.check(
        "shared rear knob uses a bounded rotary control joint",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in knob_joint.axis) == (0.0, 0.0, -1.0)
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.upper is not None
        and knob_joint.motion_limits.upper >= 4.0,
        details=f"axis={knob_joint.axis}, limits={knob_joint.motion_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
