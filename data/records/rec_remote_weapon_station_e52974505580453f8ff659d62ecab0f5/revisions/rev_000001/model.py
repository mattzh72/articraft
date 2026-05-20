from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _armored_plate_mesh(name: str):
    """Sloped side armor plate, drawn in local X/Z and extruded through Y."""

    # ExtrudeGeometry builds a 2-D XY profile through local Z.  Rotating the
    # visual +90 deg about X maps the profile's Y coordinate to vertical Z and
    # the extrusion depth to side-plate thickness.
    side_profile = [
        (-0.34, -0.21),
        (0.24, -0.21),
        (0.34, -0.06),
        (0.27, 0.22),
        (-0.22, 0.22),
        (-0.36, 0.04),
    ]
    return mesh_from_geometry(ExtrudeGeometry(side_profile, 0.050), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_remote_weapon_station")

    armor = model.material("matte_olive_armor", rgba=(0.22, 0.26, 0.18, 1.0))
    dark_armor = model.material("dark_olive_shadow", rgba=(0.12, 0.15, 0.11, 1.0))
    gunmetal = model.material("phosphated_gunmetal", rgba=(0.06, 0.065, 0.06, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.011, 0.010, 1.0))
    glass = model.material("blue_black_glass", rgba=(0.02, 0.08, 0.12, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.018, 0.018, 0.016, 1.0))

    # Fixed low pedestal: compact deck mount with a round bearing seat.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.76, 0.62, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_armor,
        name="floor_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.185, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=armor,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.260, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=dark_armor,
        name="fixed_bearing",
    )
    for ix, x in enumerate((-0.27, 0.27)):
        for iy, y in enumerate((-0.20, 0.20)):
            pedestal.visual(
                Cylinder(radius=0.026, length=0.030),
                origin=Origin(xyz=(x, y, 0.075)),
                material=gunmetal,
                name=f"anchor_bolt_{ix}_{iy}",
            )

    # Continuously yawing turret base: turntable, compact electronics deck, and
    # cheek armor plates forming a protective trunnion yoke.
    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        Cylinder(radius=0.278, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_armor,
        name="turntable_disk",
    )
    yaw_base.visual(
        Cylinder(radius=0.225, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=armor,
        name="rotating_bearing",
    )
    yaw_base.visual(
        Box((0.62, 0.61, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=armor,
        name="armored_deck",
    )
    yaw_base.visual(
        Box((0.22, 0.42, 0.160)),
        origin=Origin(xyz=(-0.20, 0.0, 0.300)),
        material=dark_armor,
        name="rear_equipment_hump",
    )
    plate_mesh = _armored_plate_mesh("sloped_side_armor")
    for i, y in enumerate((-0.305, 0.305)):
        yaw_base.visual(
            plate_mesh,
            origin=Origin(xyz=(0.0, y, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=armor,
            name=f"side_plate_{i}",
        )
    for i, y in enumerate((-0.345, 0.345)):
        yaw_base.visual(
            Cylinder(radius=0.092, length=0.040),
            origin=Origin(xyz=(0.0, y, 0.430), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"bearing_boss_{i}",
        )
    yaw_base.visual(
        Box((0.42, 0.660, 0.075)),
        origin=Origin(xyz=(-0.10, 0.0, 0.625)),
        material=dark_armor,
        name="top_bridge",
    )

    # Pitching cradle: weapon receiver and barrel on one side, EO/IR sensor pod
    # on the other, tied together by the trunnion tube and transverse beam.
    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.055, length=0.566),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_tube",
    )
    cradle.visual(
        Box((0.30, 0.42, 0.075)),
        origin=Origin(xyz=(0.09, 0.0, -0.015)),
        material=dark_armor,
        name="cross_beam",
    )
    cradle.visual(
        Box((0.325, 0.150, 0.155)),
        origin=Origin(xyz=(0.185, 0.095, 0.010)),
        material=gunmetal,
        name="weapon_receiver",
    )
    cradle.visual(
        Box((0.135, 0.115, 0.170)),
        origin=Origin(xyz=(0.170, 0.217, -0.080)),
        material=dark_armor,
        name="feed_box",
    )
    cradle.visual(
        Cylinder(radius=0.024, length=0.690),
        origin=Origin(xyz=(0.650, 0.095, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="main_barrel",
    )
    cradle.visual(
        Cylinder(radius=0.013, length=0.455),
        origin=Origin(xyz=(0.570, 0.095, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="gas_tube",
    )
    for i, x in enumerate((0.395, 0.710)):
        cradle.visual(
            Box((0.030, 0.044, 0.050)),
            origin=Origin(xyz=(x, 0.095, 0.060)),
            material=gunmetal,
            name=f"barrel_clamp_{i}",
        )
    cradle.visual(
        Cylinder(radius=0.034, length=0.105),
        origin=Origin(xyz=(1.035, 0.095, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="muzzle_brake",
    )
    cradle.visual(
        Box((0.250, 0.165, 0.205)),
        origin=Origin(xyz=(0.170, -0.165, 0.005)),
        material=dark_armor,
        name="sensor_body",
    )
    cradle.visual(
        Box((0.170, 0.180, 0.040)),
        origin=Origin(xyz=(0.230, -0.165, 0.125)),
        material=armor,
        name="sensor_sun_brow",
    )
    cradle.visual(
        Cylinder(radius=0.050, length=0.022),
        origin=Origin(xyz=(0.304, -0.165, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="primary_lens",
    )
    cradle.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.302, -0.165, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="secondary_lens",
    )
    cradle.visual(
        Box((0.055, 0.150, 0.085)),
        origin=Origin(xyz=(-0.025, -0.165, 0.005)),
        material=rubber,
        name="sensor_mount_pad",
    )

    model.articulation(
        "yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=yaw_base,
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.6),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=95.0,
            velocity=1.2,
            lower=math.radians(-20.0),
            upper=math.radians(60.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_base = object_model.get_part("yaw_base")
    cradle = object_model.get_part("cradle")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")

    ctx.check(
        "yaw is continuous vertical",
        yaw.articulation_type == ArticulationType.CONTINUOUS and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "pitch limits match weapon cradle",
        pitch.motion_limits is not None
        and abs(pitch.motion_limits.lower - math.radians(-20.0)) < 1e-6
        and abs(pitch.motion_limits.upper - math.radians(60.0)) < 1e-6
        and tuple(pitch.axis) == (0.0, -1.0, 0.0),
        details=f"limits={pitch.motion_limits}, axis={pitch.axis}",
    )

    ctx.expect_gap(
        yaw_base,
        pedestal,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="fixed_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on pedestal bearing",
    )
    ctx.expect_within(
        cradle,
        yaw_base,
        axes="y",
        inner_elem="cross_beam",
        outer_elem="top_bridge",
        margin=0.0,
        name="cradle beam is inside armored side opening",
    )

    rest_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    with ctx.pose({pitch: math.radians(60.0)}):
        high_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    with ctx.pose({pitch: math.radians(-20.0)}):
        low_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    ctx.check(
        "pitch raises and depresses barrel",
        rest_muzzle is not None
        and high_muzzle is not None
        and low_muzzle is not None
        and high_muzzle[1][2] > rest_muzzle[1][2] + 0.45
        and low_muzzle[0][2] < rest_muzzle[0][2] - 0.12,
        details=f"rest={rest_muzzle}, high={high_muzzle}, low={low_muzzle}",
    )

    rest_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    if rest_muzzle is not None and yawed_muzzle is not None:
        rest_center_x = 0.5 * (rest_muzzle[0][0] + rest_muzzle[1][0])
        yawed_center_y = 0.5 * (yawed_muzzle[0][1] + yawed_muzzle[1][1])
    else:
        rest_center_x = yawed_center_y = None
    ctx.check(
        "yaw rotates forward weapon around pedestal",
        rest_center_x is not None
        and yawed_center_y is not None
        and rest_center_x > 0.9
        and yawed_center_y > 0.9,
        details=f"rest_x={rest_center_x}, yawed_y={yawed_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
