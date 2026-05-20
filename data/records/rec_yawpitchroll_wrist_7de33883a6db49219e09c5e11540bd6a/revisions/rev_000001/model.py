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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").cylinder(length, radius).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").cylinder(length, radius).translate(center)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").cylinder(length, radius).translate(center)


def _ring_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_z(outer_radius, length, center).cut(
        _cyl_z(inner_radius, length + 0.004, center)
    )


def _ring_y(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_y(outer_radius, length, center).cut(
        _cyl_y(inner_radius, length + 0.004, center)
    )


def _ring_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return _cyl_x(outer_radius, length, center).cut(
        _cyl_x(inner_radius, length + 0.004, center)
    )


def _try_chamfer(shape: cq.Workplane, amount: float) -> cq.Workplane:
    try:
        return shape.edges().chamfer(amount)
    except Exception:
        return shape


def _pedestal_shape() -> cq.Workplane:
    base = _try_chamfer(_box((0.210, 0.165, 0.026), (0.0, 0.0, 0.013)), 0.004)
    plinth = _cyl_z(0.073, 0.070, (0.0, 0.0, 0.061))
    pedestal = base.union(plinth)
    pedestal = pedestal.union(_cyl_z(0.082, 0.014, (0.0, 0.0, 0.103)))
    pedestal = pedestal.union(_cyl_z(0.052, 0.036, (0.0, 0.0, 0.128)))
    pedestal = pedestal.union(_cyl_z(0.071, 0.010, (0.0, 0.0, 0.148)))

    # Four low, machined foot pads read as bolted mounting pads while remaining
    # part of the stationary pedestal casting.
    for x in (-0.075, 0.075):
        for y in (-0.055, 0.055):
            pedestal = pedestal.union(_cyl_z(0.014, 0.006, (x, y, 0.029)))

    return _try_chamfer(pedestal, 0.002)


def _yaw_carriage_shape() -> cq.Workplane:
    pitch_x = 0.185
    pitch_z = 0.142

    carriage = _cyl_z(0.067, 0.030, (0.0, 0.0, 0.015))
    carriage = carriage.union(_ring_z(0.076, 0.055, 0.010, (0.0, 0.0, 0.034)))

    # Offset machined boom from yaw stack to the pitch trunnion station.
    carriage = carriage.union(_try_chamfer(_box((0.205, 0.060, 0.045), (0.095, 0.0, 0.073)), 0.004))
    carriage = carriage.union(_box((0.040, 0.048, 0.110), (0.010, 0.0, 0.083)))
    carriage = carriage.union(_box((0.120, 0.020, 0.080), (0.110, 0.0, 0.115)))
    carriage = carriage.union(_box((0.074, 0.174, 0.026), (0.158, 0.0, 0.073)))

    # Twin side trunnion supports, each with a real through-bore for the pitch
    # shafts.  The web blocks connect them back into the yaw boom.
    for side in (-1.0, 1.0):
        y = side * 0.078
        carriage = carriage.union(_box((0.074, 0.018, 0.142), (pitch_x, y, pitch_z - 0.020)))
        carriage = carriage.union(_ring_y(0.046, 0.024, 0.023, (pitch_x, y, pitch_z)))
        carriage = carriage.union(_box((0.095, 0.016, 0.035), (0.135, y, 0.092)))
        carriage = carriage.union(_box((0.050, 0.016, 0.070), (0.165, y, 0.112)))

    # Hard stops that are fixed to the yaw carriage and clear the pitching cage.
    for side in (-1.0, 1.0):
        carriage = carriage.union(_box((0.022, 0.018, 0.030), (0.137, side * 0.099, 0.186)))

    return _try_chamfer(carriage, 0.0025)


def _pitch_frame_shape() -> cq.Workplane:
    # The pitch part frame is at the trunnion axis.  The roll bore is offset
    # forward in +X, giving the terminal wrist its compact offset geometry.
    roll_x = 0.070
    length_x = 0.152
    outer_y = 0.124
    outer_z = 0.124
    wall = 0.016

    frame = _cyl_y(0.020, 0.196, (0.0, 0.0, 0.0))
    for y in (-0.058, 0.058):
        frame = frame.union(_cyl_y(0.029, 0.012, (0.0, y, 0.0)))

    # Four connected rails make the roll cartridge opening visibly hollow.
    frame = frame.union(_box((length_x, wall, outer_z), (roll_x, outer_y / 2.0 - wall / 2.0, 0.0)))
    frame = frame.union(_box((length_x, wall, outer_z), (roll_x, -outer_y / 2.0 + wall / 2.0, 0.0)))
    frame = frame.union(_box((length_x, outer_y, wall), (roll_x, 0.0, outer_z / 2.0 - wall / 2.0)))
    frame = frame.union(_box((length_x, outer_y, wall), (roll_x, 0.0, -outer_z / 2.0 + wall / 2.0)))

    # Bearing collars are rings along the roll axis.  Their inner bore is larger
    # than the nose cartridge barrel, preserving real wall thickness and running
    # clearance rather than hiding an overlap.
    frame = frame.union(_ring_x(0.058, 0.043, 0.024, (roll_x - 0.054, 0.0, 0.0)))
    frame = frame.union(_ring_x(0.058, 0.043, 0.024, (roll_x + 0.054, 0.0, 0.0)))

    # Short webs tie the pitch trunnion tube into the roll cage.
    frame = frame.union(_box((0.044, 0.038, 0.026), (0.024, 0.0, 0.026)))
    frame = frame.union(_box((0.044, 0.038, 0.026), (0.024, 0.0, -0.026)))

    return _try_chamfer(frame, 0.002)


def _nose_shape() -> cq.Workplane:
    nose = _cyl_x(0.027, 0.160, (0.020, 0.0, 0.0))
    nose = nose.union(_cyl_x(0.031, 0.014, (-0.046, 0.0, 0.0)))
    nose = nose.union(_cyl_x(0.031, 0.014, (0.046, 0.0, 0.0)))
    nose = nose.union(_cyl_x(0.039, 0.022, (0.109, 0.0, 0.0)))

    # A small keyed tool socket is cut into the front face of the roll nose.
    socket_cutter = _cyl_x(0.017, 0.032, (0.124, 0.0, 0.0))
    keyway = _box((0.034, 0.028, 0.009), (0.124, 0.0, 0.0))
    nose = nose.cut(socket_cutter.union(keyway))

    return _try_chamfer(nose, 0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_robotic_wrist")

    aluminum = model.material("satin_machined_aluminum", rgba=(0.70, 0.72, 0.72, 1.0))
    dark = model.material("dark_anodized_covers", rgba=(0.07, 0.075, 0.080, 1.0))
    steel = model.material("brushed_steel_spindle", rgba=(0.52, 0.55, 0.57, 1.0))
    bearing = model.material("black_bearing_seals", rgba=(0.015, 0.017, 0.018, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal_body", tolerance=0.0008),
        material=aluminum,
        name="pedestal_body",
    )
    for i, (x, y) in enumerate(((-0.075, -0.055), (-0.075, 0.055), (0.075, -0.055), (0.075, 0.055))):
        pedestal.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(x, y, 0.032)),
            material=dark,
            name=f"base_bolt_{i}",
        )
    pedestal.visual(
        Cylinder(radius=0.056, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        material=bearing,
        name="yaw_seal",
    )

    yaw_carriage = model.part("yaw_carriage")
    yaw_carriage.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=bearing,
        name="turntable_underseal",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.067, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark,
        name="turntable",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.076, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=dark,
        name="turntable_lip",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.046, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=dark,
        name="yaw_hub",
    )
    yaw_carriage.visual(
        mesh_from_cadquery(_try_chamfer(_box((0.175, 0.060, 0.042), (0.080, 0.0, 0.071)), 0.004), "offset_boom", tolerance=0.0008),
        material=dark,
        name="offset_boom",
    )
    yaw_carriage.visual(
        Box((0.040, 0.048, 0.110)),
        origin=Origin(xyz=(0.010, 0.0, 0.083)),
        material=dark,
        name="hub_web",
    )
    yaw_carriage.visual(
        Box((0.074, 0.174, 0.026)),
        origin=Origin(xyz=(0.158, 0.0, 0.046)),
        material=dark,
        name="trunnion_saddle",
    )
    for side in (-1.0, 1.0):
        y = side * 0.078
        yaw_carriage.visual(
            Box((0.090, 0.018, 0.060)),
            origin=Origin(xyz=(0.185, y, 0.089)),
            material=dark,
            name=f"trunnion_lower_{'pos' if side > 0 else 'neg'}",
        )
        yaw_carriage.visual(
            Box((0.090, 0.018, 0.034)),
            origin=Origin(xyz=(0.185, y, 0.197)),
            material=dark,
            name=f"trunnion_upper_{'pos' if side > 0 else 'neg'}",
        )
        yaw_carriage.visual(
            Box((0.016, 0.018, 0.112)),
            origin=Origin(xyz=(0.135, y, 0.142)),
            material=dark,
            name=f"trunnion_rear_{'pos' if side > 0 else 'neg'}",
        )
        yaw_carriage.visual(
            Box((0.016, 0.018, 0.112)),
            origin=Origin(xyz=(0.235, y, 0.142)),
            material=dark,
            name=f"trunnion_front_{'pos' if side > 0 else 'neg'}",
        )
        yaw_carriage.visual(
            Box((0.022, 0.026, 0.030)),
            origin=Origin(xyz=(0.137, side * 0.091, 0.186)),
            material=dark,
            name=f"pitch_stop_{'pos' if side > 0 else 'neg'}",
        )

    pitch_frame = model.part("pitch_frame")
    # Split primitive rails leave real open space around the roll cartridge; the
    # part is still a single connected frame through overlapping corners, webs,
    # and trunnion shaft.
    pitch_frame.visual(
        Cylinder(radius=0.020, length=0.196),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="pitch_shaft",
    )
    for side, y in (("neg", -0.058), ("pos", 0.058)):
        pitch_frame.visual(
            Cylinder(radius=0.029, length=0.022),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=f"pitch_collar_{side}",
        )
    for name, y in (("rail_side_neg", -0.054), ("rail_side_pos", 0.054)):
        pitch_frame.visual(
            Box((0.152, 0.016, 0.124)),
            origin=Origin(xyz=(0.070, y, 0.0)),
            material=aluminum,
            name=name,
        )
    for name, z in (("rail_lower", -0.054), ("rail_upper", 0.054)):
        pitch_frame.visual(
            Box((0.152, 0.124, 0.016)),
            origin=Origin(xyz=(0.070, 0.0, z)),
            material=aluminum,
            name=name,
        )
    for x, label in ((0.016, "rear"), (0.124, "front")):
        pitch_frame.visual(
            Box((0.024, 0.106, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.052)),
            material=aluminum,
            name=f"{label}_bearing_upper",
        )
        pitch_frame.visual(
            Box((0.024, 0.106, 0.012)),
            origin=Origin(xyz=(x, 0.0, -0.052)),
            material=aluminum,
            name=f"{label}_bearing_lower",
        )
        pitch_frame.visual(
            Box((0.024, 0.012, 0.106)),
            origin=Origin(xyz=(x, 0.052, 0.0)),
            material=aluminum,
            name=f"{label}_bearing_side_pos",
        )
        pitch_frame.visual(
            Box((0.024, 0.012, 0.106)),
            origin=Origin(xyz=(x, -0.052, 0.0)),
            material=aluminum,
            name=f"{label}_bearing_side_neg",
        )
    for x, label in ((0.070, "mid"), (0.124, "front")):
        pitch_frame.visual(
            Box((0.030, 0.036, 0.0195)),
            origin=Origin(xyz=(x, 0.0, 0.04025)),
            material=bearing,
            name=f"{label}_bearing_pad_upper",
        )
        pitch_frame.visual(
            Box((0.030, 0.036, 0.0195)),
            origin=Origin(xyz=(x, 0.0, -0.04025)),
            material=bearing,
            name=f"{label}_bearing_pad_lower",
        )
        pitch_frame.visual(
            Box((0.030, 0.0195, 0.036)),
            origin=Origin(xyz=(x, 0.04025, 0.0)),
            material=bearing,
            name=f"{label}_bearing_pad_pos",
        )
        pitch_frame.visual(
            Box((0.030, 0.0195, 0.036)),
            origin=Origin(xyz=(x, -0.04025, 0.0)),
            material=bearing,
            name=f"{label}_bearing_pad_neg",
        )
    roll_nose = model.part("roll_nose")
    roll_nose.visual(
        Cylinder(radius=0.027, length=0.130),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="nose_barrel",
    )
    for x, label in ((0.000, "rear"), (0.054, "front")):
        roll_nose.visual(
            Cylinder(radius=0.031, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"{label}_spindle_collar",
        )
    roll_nose.visual(
        Cylinder(radius=0.039, length=0.022),
        origin=Origin(xyz=(0.136, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tool_flange",
    )
    roll_nose.visual(
        Cylinder(radius=0.017, length=0.005),
        origin=Origin(xyz=(0.1495, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="tool_socket",
    )

    yaw_joint = model.articulation(
        "pedestal_to_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    pitch_joint = model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_carriage,
        child=pitch_frame,
        origin=Origin(xyz=(0.185, 0.0, 0.142)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.0, lower=-0.80, upper=0.80),
    )
    roll_joint = model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_nose,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0, lower=-math.pi, upper=math.pi),
    )

    # Keep handles for tests/readability without forcing users to remember names.
    model.meta["primary_axes"] = (yaw_joint.name, pitch_joint.name, roll_joint.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_carriage = object_model.get_part("yaw_carriage")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_nose = object_model.get_part("roll_nose")
    yaw_joint = object_model.get_articulation("pedestal_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

    ctx.expect_contact(
        pedestal,
        yaw_carriage,
        elem_a="yaw_seal",
        elem_b="turntable_underseal",
        contact_tol=0.0015,
        name="yaw turntable is seated on pedestal bearing",
    )
    ctx.expect_within(
        roll_nose,
        pitch_frame,
        axes="yz",
        margin=0.004,
        name="roll nose remains inside pitch-frame bore envelope",
    )

    rest_roll_position = ctx.part_world_position(roll_nose)
    with ctx.pose({pitch_joint: 0.72}):
        raised_roll_position = ctx.part_world_position(roll_nose)
        ctx.expect_within(
            roll_nose,
            pitch_frame,
            axes="y",
            margin=0.004,
            name="pitched nose stays captured in frame envelope",
        )

    ctx.check(
        "pitch axis moves the offset roll cartridge",
        rest_roll_position is not None
        and raised_roll_position is not None
        and raised_roll_position[2] < rest_roll_position[2] - 0.025,
        details=f"rest={rest_roll_position}, pitched={raised_roll_position}",
    )

    rest_pitch_position = ctx.part_world_position(pitch_frame)
    with ctx.pose({yaw_joint: math.radians(65.0), roll_joint: math.radians(110.0)}):
        yawed_pitch_position = ctx.part_world_position(pitch_frame)

    ctx.check(
        "yaw axis swings the side-supported wrist about the pedestal",
        rest_pitch_position is not None
        and yawed_pitch_position is not None
        and abs(yawed_pitch_position[1] - rest_pitch_position[1]) > 0.12,
        details=f"rest={rest_pitch_position}, yawed={yawed_pitch_position}",
    )

    return ctx.report()


object_model = build_object_model()
