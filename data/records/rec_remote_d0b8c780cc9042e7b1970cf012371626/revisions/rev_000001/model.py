from __future__ import annotations

import math

import cadquery as cq

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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="media_center_remote")

    body_plastic = model.material("satin_graphite_plastic", rgba=(0.075, 0.078, 0.082, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.013, 1.0))
    dark_trim = model.material("dark_recess_trim", rgba=(0.025, 0.027, 0.030, 1.0))
    battery_plastic = model.material("battery_door_plastic", rgba=(0.105, 0.110, 0.118, 1.0))
    icon_grey = model.material("soft_grey_print", rgba=(0.55, 0.57, 0.60, 1.0))
    lens = model.material("smoked_ir_lens", rgba=(0.02, 0.01, 0.012, 0.75))

    body = model.part("body")
    body_width = 0.052
    body_length = 0.178
    body_thickness = 0.018

    body_shape = (
        cq.Workplane("XY")
        .box(body_width, body_length, body_thickness)
        .edges()
        .fillet(0.006)
    )
    wheel_slot = cq.Workplane("XY").box(0.050, 0.036, 0.040).translate((0.0, 0.038, 0.0))
    body_shape = body_shape.cut(wheel_slot)
    body.visual(
        mesh_from_cadquery(body_shape, "rounded_remote_body", tolerance=0.0007, angular_tolerance=0.08),
        material=body_plastic,
        name="rounded_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_length, body_thickness)),
        mass=0.105,
    )

    bezel_outer = rounded_rect_profile(0.052, 0.040, 0.007, corner_segments=8)
    bezel_inner = rounded_rect_profile(0.048, 0.034, 0.004, corner_segments=8)
    body.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(bezel_outer, [bezel_inner], 0.0018, center=True),
            "scroll_wheel_bezel",
        ),
        origin=Origin(xyz=(0.0, 0.038, 0.0094)),
        material=dark_trim,
        name="wheel_bezel",
    )
    for side_index, side_x in enumerate((-0.024, 0.024)):
        body.visual(
            Box((0.0040, 0.018, 0.010)),
            origin=Origin(xyz=(side_x, 0.038, 0.0134)),
            material=dark_trim,
            name=f"wheel_cheek_{side_index}",
        )

    body.visual(
        Box((0.031, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.082, 0.0095)),
        material=lens,
        name="front_ir_window",
    )
    body.visual(
        Box((0.018, 0.0016, 0.0008)),
        origin=Origin(xyz=(0.0, -0.010, 0.00925)),
        material=icon_grey,
        name="play_pause_mark",
    )
    body.visual(
        Box((0.0016, 0.013, 0.0008)),
        origin=Origin(xyz=(-0.004, -0.010, 0.00925)),
        material=icon_grey,
        name="pause_mark_0",
    )
    body.visual(
        Box((0.0016, 0.013, 0.0008)),
        origin=Origin(xyz=(0.004, -0.010, 0.00925)),
        material=icon_grey,
        name="pause_mark_1",
    )
    for rail_index, rail_y in enumerate((-0.0715, -0.0085)):
        body.visual(
            Box((0.050, 0.0032, 0.0030)),
            origin=Origin(xyz=(0.0, rail_y, -0.0105)),
            material=dark_trim,
            name=f"battery_rail_{rail_index}",
        )

    wheel_carriage = model.part("wheel_carriage")
    wheel_carriage.visual(
        Box((0.036, 0.006, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
        material=dark_trim,
        name="click_cradle",
    )
    wheel_carriage.inertial = Inertial.from_geometry(
        Box((0.036, 0.006, 0.0020)),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
    )

    scroll_wheel = model.part("scroll_wheel")
    scroll_wheel.visual(
        Cylinder(radius=0.0110, length=0.038),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="rubber_wheel",
    )
    scroll_wheel.visual(
        Cylinder(radius=0.0030, length=0.040),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="axle_stub",
    )
    tread_ring = mesh_from_geometry(TorusGeometry(radius=0.0112, tube=0.00055), "scroll_wheel_tread_ring")
    for ring_index, ring_x in enumerate((-0.011, 0.0, 0.011)):
        scroll_wheel.visual(
            tread_ring,
            origin=Origin(xyz=(ring_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_trim,
            name=f"tread_ring_{ring_index}",
        )
    scroll_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.038),
        mass=0.006,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    battery_door = model.part("battery_door")
    door_profile = rounded_rect_profile(0.037, 0.060, 0.004, corner_segments=6)
    battery_door.visual(
        mesh_from_geometry(ExtrudeWithHolesGeometry(door_profile, [], 0.0026, center=True), "battery_door_panel"),
        origin=Origin(xyz=(0.0, 0.0, -0.0013)),
        material=battery_plastic,
        name="sliding_panel",
    )
    battery_door.visual(
        Box((0.018, 0.0012, 0.0007)),
        origin=Origin(xyz=(0.0, -0.012, -0.00275)),
        material=icon_grey,
        name="slide_arrow_mark",
    )
    battery_door.visual(
        Box((0.005, 0.0012, 0.0007)),
        origin=Origin(xyz=(0.0085, -0.010, -0.00275), rpy=(0.0, 0.0, 0.55)),
        material=icon_grey,
        name="slide_arrow_head_0",
    )
    battery_door.visual(
        Box((0.005, 0.0012, 0.0007)),
        origin=Origin(xyz=(0.0085, -0.014, -0.00275), rpy=(0.0, 0.0, -0.55)),
        material=icon_grey,
        name="slide_arrow_head_1",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.037, 0.060, 0.0026)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, -0.0013)),
    )

    model.articulation(
        "body_to_wheel_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=wheel_carriage,
        origin=Origin(xyz=(0.0, 0.038, 0.0130)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=0.04, lower=0.0, upper=0.0020),
    )
    model.articulation(
        "carriage_to_scroll_wheel",
        ArticulationType.CONTINUOUS,
        parent=wheel_carriage,
        child=scroll_wheel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.18, velocity=18.0),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, -0.040, -0.0090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.18, lower=0.0, upper=0.020),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("wheel_carriage")
    wheel = object_model.get_part("scroll_wheel")
    door = object_model.get_part("battery_door")
    click_joint = object_model.get_articulation("body_to_wheel_carriage")
    wheel_joint = object_model.get_articulation("carriage_to_scroll_wheel")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.check(
        "scroll wheel has a click travel",
        click_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={click_joint.articulation_type}",
    )
    ctx.check(
        "scroll wheel click depresses into the front face",
        tuple(round(v, 6) for v in click_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={click_joint.axis}",
    )
    ctx.check(
        "scroll wheel uses continuous rotation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_joint.articulation_type}",
    )
    ctx.check(
        "scroll wheel axis is horizontal across the remote",
        tuple(round(v, 6) for v in wheel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={wheel_joint.axis}",
    )
    ctx.expect_within(
        wheel,
        body,
        axes="x",
        inner_elem="rubber_wheel",
        outer_elem="wheel_bezel",
        margin=0.001,
        name="wheel fits between the side cheeks of the front bezel",
    )
    ctx.expect_overlap(
        wheel,
        body,
        axes="y",
        elem_a="rubber_wheel",
        elem_b="wheel_bezel",
        min_overlap=0.020,
        name="wheel is centered in the front-face scroll slot",
    )
    unclicked_position = ctx.part_world_position(carriage)
    with ctx.pose({click_joint: 0.0020}):
        clicked_position = ctx.part_world_position(carriage)
    ctx.check(
        "scroll wheel click moves downward",
        unclicked_position is not None
        and clicked_position is not None
        and clicked_position[2] < unclicked_position[2] - 0.0015,
        details=f"unclicked={unclicked_position}, clicked={clicked_position}",
    )

    ctx.check(
        "battery door uses a prismatic slide",
        door_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={door_joint.articulation_type}",
    )
    ctx.check(
        "battery door slides along the short body axis",
        tuple(round(v, 6) for v in door_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.expect_gap(
        body,
        door,
        axis="z",
        positive_elem="rounded_shell",
        negative_elem="sliding_panel",
        max_gap=0.0008,
        max_penetration=0.0002,
        name="battery door sits on the rear face",
    )
    ctx.expect_within(
        door,
        body,
        axes="y",
        inner_elem="sliding_panel",
        outer_elem="rounded_shell",
        margin=0.003,
        name="battery door remains within the rear battery bay length",
    )

    closed_position = ctx.part_world_position(door)
    with ctx.pose({door_joint: 0.020}):
        slid_position = ctx.part_world_position(door)
        ctx.expect_within(
            door,
            body,
            axes="y",
            inner_elem="sliding_panel",
            outer_elem="rounded_shell",
            margin=0.003,
            name="slid battery door stays guided between rear rails",
        )
    ctx.check(
        "battery door moves along +X when opened",
        closed_position is not None
        and slid_position is not None
        and slid_position[0] > closed_position[0] + 0.018,
        details=f"closed={closed_position}, slid={slid_position}",
    )

    return ctx.report()


object_model = build_object_model()
