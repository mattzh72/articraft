from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.190
BODY_WIDTH = 0.260
BIN_TRAVEL = 0.095
BIN_CLOSED_X = 0.014


def _box_on_floor(x: float, y: float, z: float):
    return cq.Workplane("XY").box(x, y, z, centered=(True, True, False))


def _build_bin_shape():
    outer = _box_on_floor(0.152, 0.216, 0.188)
    front_fascia = _box_on_floor(0.008, 0.224, 0.154).translate((0.080, 0.0, 0.014))
    inner = _box_on_floor(0.142, 0.206, 0.184).translate((-0.001, 0.0, 0.010))
    handle_recess = _box_on_floor(0.016, 0.088, 0.030).translate((0.078, 0.0, 0.124))

    return outer.union(front_fascia).cut(inner).cut(handle_recess)


def _add_rear_wheel(model: ArticulatedObject, name: str, side_sign: float, tire, hub, body) -> None:
    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub,
        name="hub",
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(
            xyz=(
                -BODY_DEPTH * 0.5 - 0.029,
                side_sign * (BODY_WIDTH * 0.5 - 0.027),
                0.036,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )


def _add_cutter_drum(
    model: ArticulatedObject,
    name: str,
    x_pos: float,
    axis_sign: float,
    drum_material,
    shaft_material,
    body,
) -> None:
    drum = model.part(name)
    drum.visual(
        Cylinder(radius=0.009, length=0.182),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_material,
        name="cutter_body",
    )
    drum.visual(
        Cylinder(radius=0.0035, length=0.224),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shaft_material,
        name="shaft",
    )
    drum.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.098, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shaft_material,
        name="end_cap_0",
    )
    drum.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, -0.098, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shaft_material,
        name="end_cap_1",
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(x_pos, 0.0, 0.318)),
        axis=(0.0, axis_sign, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_shredder")

    body_plastic = model.material("body_plastic", rgba=(0.15, 0.16, 0.17, 1.0))
    bin_plastic = model.material("bin_plastic", rgba=(0.19, 0.20, 0.22, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.46, 0.47, 0.50, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    control_grey = model.material("control_grey", rgba=(0.33, 0.34, 0.36, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.014, 0.236, 0.218)),
        origin=Origin(xyz=(-0.088, 0.0, 0.117)),
        material=body_plastic,
        name="lower_back",
    )
    body.visual(
        Box((0.176, 0.012, 0.218)),
        origin=Origin(xyz=(0.003, 0.118, 0.117)),
        material=body_plastic,
        name="lower_left_wall",
    )
    body.visual(
        Box((0.176, 0.012, 0.218)),
        origin=Origin(xyz=(0.003, -0.118, 0.117)),
        material=body_plastic,
        name="lower_right_wall",
    )
    body.visual(
        Box((0.176, 0.236, 0.028)),
        origin=Origin(xyz=(0.003, 0.0, 0.240)),
        material=body_plastic,
        name="bin_roof",
    )
    body.visual(
        Box((0.176, 0.012, 0.150)),
        origin=Origin(xyz=(0.000, 0.118, 0.321)),
        material=body_plastic,
        name="head_left_cheek",
    )
    body.visual(
        Box((0.176, 0.012, 0.150)),
        origin=Origin(xyz=(0.000, -0.118, 0.321)),
        material=body_plastic,
        name="head_right_cheek",
    )
    body.visual(
        Box((0.014, 0.236, 0.150)),
        origin=Origin(xyz=(-0.081, 0.0, 0.321)),
        material=body_plastic,
        name="head_back",
    )
    body.visual(
        Box((0.024, 0.236, 0.086)),
        origin=Origin(xyz=(0.070, 0.0, 0.289)),
        material=body_plastic,
        name="head_front",
    )
    body.visual(
        Box((0.108, 0.236, 0.022)),
        origin=Origin(xyz=(-0.034, 0.0, 0.385)),
        material=body_plastic,
        name="top_rear",
    )
    body.visual(
        Box((0.054, 0.236, 0.022)),
        origin=Origin(xyz=(0.061, 0.0, 0.385)),
        material=body_plastic,
        name="top_front",
    )
    body.visual(
        Box((0.062, 0.212, 0.006)),
        origin=Origin(xyz=(0.061, 0.0, 0.399)),
        material=trim_dark,
        name="control_pad",
    )
    body.visual(
        Box((0.020, 0.066, 0.001)),
        origin=Origin(xyz=(0.061, 0.005, 0.4025)),
        material=control_grey,
        name="slider_track",
    )
    body.visual(
        Box((0.022, 0.030, 0.064)),
        origin=Origin(xyz=(-0.084, 0.102, 0.032)),
        material=body_plastic,
        name="rear_left_boss",
    )
    body.visual(
        Box((0.022, 0.030, 0.064)),
        origin=Origin(xyz=(-0.084, -0.102, 0.032)),
        material=body_plastic,
        name="rear_right_boss",
    )
    body.visual(
        Box((0.032, 0.036, 0.008)),
        origin=Origin(xyz=(0.073, 0.100, 0.004)),
        material=body_plastic,
        name="front_left_foot",
    )
    body.visual(
        Box((0.032, 0.036, 0.008)),
        origin=Origin(xyz=(0.073, -0.100, 0.004)),
        material=body_plastic,
        name="front_right_foot",
    )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(_build_bin_shape(), "shredder_bin"),
        material=bin_plastic,
        name="bin_shell",
    )

    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(BIN_CLOSED_X, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.28, lower=0.0, upper=BIN_TRAVEL),
    )

    _add_cutter_drum(model, "rear_drum", 0.010, 1.0, metal_dark, wheel_hub, body)
    _add_cutter_drum(model, "front_drum", 0.029, -1.0, metal_dark, wheel_hub, body)

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.024, 0.014, 0.006)),
        material=control_grey,
        name="rocker_cap",
    )
    power_rocker.visual(
        Box((0.006, 0.014, 0.0015)),
        origin=Origin(xyz=(0.009, 0.0, 0.00375)),
        material=trim_dark,
        name="rocker_front",
    )
    power_rocker.visual(
        Box((0.006, 0.014, 0.0015)),
        origin=Origin(xyz=(-0.009, 0.0, 0.00375)),
        material=trim_dark,
        name="rocker_rear",
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=Origin(xyz=(0.061, -0.072, 0.405)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=-0.24, upper=0.24),
    )

    mode_slider = model.part("mode_slider")
    mode_slider.visual(
        Box((0.018, 0.010, 0.005)),
        material=control_grey,
        name="slider_body",
    )
    mode_slider.visual(
        Box((0.010, 0.012, 0.0025)),
        origin=Origin(xyz=(0.0, 0.0, 0.00375)),
        material=trim_dark,
        name="slider_grip",
    )
    model.articulation(
        "body_to_mode_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_slider,
        origin=Origin(xyz=(0.061, 0.005, 0.4055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.10, lower=-0.028, upper=0.028),
    )

    reverse_button = model.part("reverse_button")
    reverse_button.visual(
        Box((0.012, 0.012, 0.004)),
        material=control_grey,
        name="button_cap",
    )
    model.articulation(
        "body_to_reverse_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=reverse_button,
        origin=Origin(xyz=(0.061, 0.078, 0.404)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.0022),
    )

    _add_rear_wheel(model, "rear_left_wheel", 1.0, trim_dark, wheel_hub, body)
    _add_rear_wheel(model, "rear_right_wheel", -1.0, trim_dark, wheel_hub, body)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    power_rocker = object_model.get_part("power_rocker")
    mode_slider = object_model.get_part("mode_slider")
    reverse_button = object_model.get_part("reverse_button")
    bin_slide = object_model.get_articulation("body_to_bin")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    slider_joint = object_model.get_articulation("body_to_mode_slider")
    reverse_joint = object_model.get_articulation("body_to_reverse_button")

    closed_pos = ctx.part_world_position(bin_part)
    with ctx.pose({bin_slide: 0.0}):
        ctx.expect_within(
            bin_part,
            body,
            axes="yz",
            margin=0.002,
            name="bin stays laterally nested in the body when closed",
        )

    limits = bin_slide.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({bin_slide: limits.upper}):
            ctx.expect_within(
                bin_part,
                body,
                axes="yz",
                margin=0.004,
                name="bin stays aligned in the opening when extended",
            )
            ctx.expect_overlap(
                bin_part,
                body,
                axes="x",
                min_overlap=0.055,
                name="bin retains insertion at full extension",
            )
            open_pos = ctx.part_world_position(bin_part)
        ctx.check(
            "bin slides forward",
            closed_pos is not None and open_pos is not None and open_pos[0] > closed_pos[0] + 0.08,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    slider_limits = slider_joint.motion_limits
    if slider_limits is not None and slider_limits.lower is not None and slider_limits.upper is not None:
        with ctx.pose({slider_joint: slider_limits.lower}):
            slider_low = ctx.part_world_position(mode_slider)
        with ctx.pose({slider_joint: slider_limits.upper}):
            slider_high = ctx.part_world_position(mode_slider)
        ctx.check(
            "mode slider travels laterally along the strip",
            slider_low is not None and slider_high is not None and slider_high[1] > slider_low[1] + 0.05,
            details=f"low={slider_low}, high={slider_high}",
        )

    reverse_limits = reverse_joint.motion_limits
    if reverse_limits is not None and reverse_limits.upper is not None:
        reverse_rest = ctx.part_world_position(reverse_button)
        with ctx.pose({reverse_joint: reverse_limits.upper}):
            reverse_pressed = ctx.part_world_position(reverse_button)
        ctx.check(
            "reverse button presses downward",
            reverse_rest is not None and reverse_pressed is not None and reverse_pressed[2] < reverse_rest[2] - 0.0015,
            details=f"rest={reverse_rest}, pressed={reverse_pressed}",
        )

    rocker_limits = rocker_joint.motion_limits
    if rocker_limits is not None and rocker_limits.lower is not None and rocker_limits.upper is not None:
        with ctx.pose({rocker_joint: rocker_limits.lower}):
            rocker_low = ctx.part_element_world_aabb(power_rocker, elem="rocker_front")
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            rocker_high = ctx.part_element_world_aabb(power_rocker, elem="rocker_front")
        ctx.check(
            "power rocker pivots on its short axis",
            rocker_low is not None
            and rocker_high is not None
            and rocker_high[1][2] > rocker_low[1][2] + 0.003,
            details=f"low={rocker_low}, high={rocker_high}",
        )

    return ctx.report()


object_model = build_object_model()
