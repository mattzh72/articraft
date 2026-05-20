from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_drum_visuals(part, *, material) -> None:
    drum_spin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.015, length=0.30),
        origin=drum_spin,
        material=material,
        name="shaft",
    )
    for index, y in enumerate((-0.08, -0.04, 0.0, 0.04, 0.08)):
        part.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"cutter_{index}",
        )


def _add_button_visuals(part, *, material, light_material) -> None:
    part.visual(
        Box((0.007, 0.028, 0.012)),
        origin=Origin(xyz=(-0.0005, 0.0, 0.0)),
        material=light_material,
        name="guide",
    )
    part.visual(
        Box((0.010, 0.036, 0.014)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=material,
        name="cap",
    )
    part.visual(
        Box((0.003, 0.020, 0.006)),
        origin=Origin(xyz=(0.0125, 0.0, 0.0)),
        material=light_material,
        name="indicator",
    )


def _add_caster(model, body, *, index: int, x: float, y: float, z: float, steel, wheel_core, rubber) -> None:
    caster = model.part(f"caster_{index}")
    caster.visual(
        Cylinder(radius=0.0075, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=steel,
        name="stem",
    )
    caster.visual(
        Box((0.022, 0.044, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=steel,
        name="bridge",
    )
    caster.visual(
        Box((0.012, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, 0.016, -0.036)),
        material=steel,
        name="fork_0",
    )
    caster.visual(
        Box((0.012, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, -0.016, -0.036)),
        material=steel,
        name="fork_1",
    )

    wheel = model.part(f"wheel_{index}")
    wheel.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_core,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.003, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="spindle",
    )

    model.articulation(
        f"body_to_caster_{index}",
        ArticulationType.FIXED,
        parent=body,
        child=caster,
        origin=Origin(xyz=(x, y, z)),
    )
    model.articulation(
        f"caster_{index}_to_wheel_{index}",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.047)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=20.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_shredder")

    body_gray = model.material("body_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    body_black = model.material("body_black", rgba=(0.11, 0.12, 0.13, 1.0))
    panel_black = model.material("panel_black", rgba=(0.07, 0.08, 0.09, 1.0))
    bin_black = model.material("bin_black", rgba=(0.14, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.71, 0.74, 1.0))
    cutter_steel = model.material("cutter_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    start_green = model.material("start_green", rgba=(0.18, 0.50, 0.24, 1.0))
    reverse_red = model.material("reverse_red", rgba=(0.62, 0.16, 0.16, 1.0))
    amber = model.material("amber", rgba=(0.86, 0.58, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.40, 0.015, 0.48)),
        origin=Origin(xyz=(0.0, 0.1525, 0.314)),
        material=body_gray,
        name="side_0",
    )
    body.visual(
        Box((0.40, 0.015, 0.48)),
        origin=Origin(xyz=(0.0, -0.1525, 0.314)),
        material=body_gray,
        name="side_1",
    )
    body.visual(
        Box((0.015, 0.29, 0.48)),
        origin=Origin(xyz=(-0.1925, 0.0, 0.314)),
        material=body_gray,
        name="rear",
    )
    body.visual(
        Box((0.364, 0.32, 0.014)),
        origin=Origin(xyz=(-0.018, 0.0, 0.081)),
        material=body_black,
        name="floor",
    )
    body.visual(
        Box((0.260, 0.025, 0.030)),
        origin=Origin(xyz=(0.000, 0.080, 0.103)),
        material=panel_black,
        name="runner_0",
    )
    body.visual(
        Box((0.260, 0.025, 0.030)),
        origin=Origin(xyz=(0.000, -0.080, 0.103)),
        material=panel_black,
        name="runner_1",
    )
    body.visual(
        Box((0.40, 0.32, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.562)),
        material=body_black,
        name="top",
    )
    body.visual(
        Box((0.040, 0.32, 0.042)),
        origin=Origin(xyz=(0.180, 0.0, 0.095)),
        material=body_gray,
        name="toe_kick",
    )
    body.visual(
        Box((0.025, 0.32, 0.050)),
        origin=Origin(xyz=(0.1875, 0.0, 0.529)),
        material=body_gray,
        name="upper_bezel",
    )
    body.visual(
        Box((0.020, 0.25, 0.012)),
        origin=Origin(xyz=(0.176, 0.0, 0.546)),
        material=panel_black,
        name="bin_stop",
    )

    bin_part = model.part("bin")
    bin_part.visual(
        Box((0.310, 0.270, 0.010)),
        origin=Origin(xyz=(-0.147, 0.0, 0.005)),
        material=bin_black,
        name="bottom",
    )
    bin_part.visual(
        Box((0.310, 0.008, 0.360)),
        origin=Origin(xyz=(-0.147, 0.139, 0.185)),
        material=bin_black,
        name="side_0",
    )
    bin_part.visual(
        Box((0.310, 0.008, 0.360)),
        origin=Origin(xyz=(-0.147, -0.139, 0.185)),
        material=bin_black,
        name="side_1",
    )
    bin_part.visual(
        Box((0.014, 0.270, 0.360)),
        origin=Origin(xyz=(-0.303, 0.0, 0.185)),
        material=bin_black,
        name="rear",
    )
    bin_part.visual(
        Box((0.018, 0.286, 0.380)),
        origin=Origin(xyz=(0.009, 0.0, 0.190)),
        material=body_black,
        name="front",
    )
    bin_part.visual(
        Box((0.032, 0.190, 0.034)),
        origin=Origin(xyz=(0.024, 0.0, 0.300)),
        material=body_gray,
        name="handle",
    )

    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.158, 0.0, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.22),
    )

    head = model.part("head")
    head.visual(
        Box((0.42, 0.012, 0.17)),
        origin=Origin(xyz=(0.0, 0.164, 0.095)),
        material=body_black,
        name="side_0",
    )
    head.visual(
        Box((0.42, 0.012, 0.17)),
        origin=Origin(xyz=(0.0, -0.164, 0.095)),
        material=body_black,
        name="side_1",
    )
    head.visual(
        Box((0.012, 0.328, 0.17)),
        origin=Origin(xyz=(-0.204, 0.0, 0.095)),
        material=body_black,
        name="rear",
    )
    head.visual(
        Box((0.018, 0.328, 0.16)),
        origin=Origin(xyz=(0.201, 0.0, 0.082)),
        material=body_black,
        name="front",
    )
    head.visual(
        Box((0.24, 0.34, 0.018)),
        origin=Origin(xyz=(-0.090, 0.0, 0.181)),
        material=body_black,
        name="top_rear",
    )
    head.visual(
        Box((0.10, 0.34, 0.018)),
        origin=Origin(xyz=(0.160, 0.0, 0.181)),
        material=body_black,
        name="top_front",
    )
    head.visual(
        Box((0.38, 0.30, 0.012)),
        origin=Origin(xyz=(-0.010, 0.0, 0.006)),
        material=body_black,
        name="mount",
    )
    head.visual(
        Box((0.10, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.154, 0.126)),
        material=steel,
        name="bearing_0",
    )
    head.visual(
        Box((0.10, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, -0.154, 0.126)),
        material=steel,
        name="bearing_1",
    )
    head.visual(
        Box((0.008, 0.24, 0.10)),
        origin=Origin(xyz=(0.209, 0.0, 0.085)),
        material=panel_black,
        name="control_panel",
    )

    model.articulation(
        "body_to_head",
        ArticulationType.FIXED,
        parent=body,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.570)),
    )

    for drum_name, drum_x in (("drum_0", -0.024), ("drum_1", 0.024)):
        drum = model.part(drum_name)
        _add_drum_visuals(drum, material=cutter_steel)
        model.articulation(
            f"head_to_{drum_name}",
            ArticulationType.CONTINUOUS,
            parent=head,
            child=drum,
            origin=Origin(xyz=(drum_x, 0.0, 0.126)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=20.0),
        )

    reverse_button = model.part("reverse_button")
    _add_button_visuals(reverse_button, material=reverse_red, light_material=amber)
    model.articulation(
        "head_to_reverse_button",
        ArticulationType.PRISMATIC,
        parent=head,
        child=reverse_button,
        origin=Origin(xyz=(0.217, -0.065, 0.074)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    start_button = model.part("start_button")
    _add_button_visuals(start_button, material=start_green, light_material=amber)
    model.articulation(
        "head_to_start_button",
        ArticulationType.PRISMATIC,
        parent=head,
        child=start_button,
        origin=Origin(xyz=(0.217, 0.065, 0.074)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    dial = model.part("jam_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.024,
                body_style="skirted",
                top_diameter=0.040,
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "jam_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_gray,
        name="knob",
    )
    model.articulation(
        "head_to_jam_dial",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=dial,
        origin=Origin(xyz=(0.213, 0.0, 0.126)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    caster_z = 0.074
    for index, (caster_x, caster_y) in enumerate(
        (
            (0.135, 0.110),
            (0.135, -0.110),
            (-0.135, 0.110),
            (-0.135, -0.110),
        )
    ):
        _add_caster(
            model,
            body,
            index=index,
            x=caster_x,
            y=caster_y,
            z=caster_z,
            steel=steel,
            wheel_core=body_gray,
            rubber=rubber,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    head = object_model.get_part("head")
    bin_part = object_model.get_part("bin")
    drum_0 = object_model.get_part("drum_0")
    drum_1 = object_model.get_part("drum_1")
    reverse_button = object_model.get_part("reverse_button")
    start_button = object_model.get_part("start_button")

    bin_joint = object_model.get_articulation("body_to_bin")
    reverse_joint = object_model.get_articulation("head_to_reverse_button")
    start_joint = object_model.get_articulation("head_to_start_button")

    bin_upper = 0.22
    button_press = 0.004

    with ctx.pose({bin_joint: 0.0}):
        ctx.expect_within(
            bin_part,
            body,
            axes="yz",
            margin=0.0,
            name="bin stays centered in cabinet at rest",
        )
        rest_bin_pos = ctx.part_world_position(bin_part)

    with ctx.pose({bin_joint: bin_upper}):
        ctx.expect_within(
            bin_part,
            body,
            axes="yz",
            margin=0.0,
            name="bin stays centered while extended",
        )
        ctx.expect_overlap(
            bin_part,
            body,
            axes="x",
            min_overlap=0.12,
            name="bin keeps retained insertion at full pullout",
        )
        extended_bin_pos = ctx.part_world_position(bin_part)

    ctx.check(
        "bin slides forward",
        rest_bin_pos is not None
        and extended_bin_pos is not None
        and extended_bin_pos[0] > rest_bin_pos[0] + 0.18,
        details=f"rest={rest_bin_pos}, extended={extended_bin_pos}",
    )

    ctx.expect_within(
        drum_0,
        head,
        axes="yz",
        margin=0.0,
        name="rear cutter drum stays inside the motor head",
    )
    ctx.expect_within(
        drum_1,
        head,
        axes="yz",
        margin=0.0,
        name="front cutter drum stays inside the motor head",
    )

    reverse_rest = ctx.part_world_position(reverse_button)
    start_rest = ctx.part_world_position(start_button)
    with ctx.pose({reverse_joint: button_press, start_joint: button_press}):
        reverse_pressed = ctx.part_world_position(reverse_button)
        start_pressed = ctx.part_world_position(start_button)

    ctx.check(
        "reverse button presses inward",
        reverse_rest is not None
        and reverse_pressed is not None
        and reverse_pressed[0] < reverse_rest[0] - 0.003,
        details=f"rest={reverse_rest}, pressed={reverse_pressed}",
    )
    ctx.check(
        "start button presses inward",
        start_rest is not None
        and start_pressed is not None
        and start_pressed[0] < start_rest[0] - 0.003,
        details=f"rest={start_rest}, pressed={start_pressed}",
    )

    wheel_bottoms = []
    for index in range(4):
        wheel_aabb = ctx.part_world_aabb(object_model.get_part(f"wheel_{index}"))
        wheel_bottoms.append(None if wheel_aabb is None else wheel_aabb[0][2])

    ctx.check(
        "casters sit on the floor plane",
        all(bottom is not None and abs(bottom) <= 0.005 for bottom in wheel_bottoms),
        details=f"wheel_bottoms={wheel_bottoms}",
    )

    return ctx.report()


object_model = build_object_model()
