from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


KINGPIN_X = 0.76
HUB_X = 0.30


def _x_cylinder_origin(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Origin:
    """Orient a URDF cylinder so its length runs along local +X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def _add_beam_end(beam, side: float, cast_iron, pin_steel) -> None:
    prefix = "left" if side > 0.0 else "right"

    beam.visual(
        Box((0.08, 0.16, 0.24)),
        origin=Origin(xyz=(side * 0.665, 0.0, 0.0)),
        material=cast_iron,
        name=f"{prefix}_yoke_web",
    )
    beam.visual(
        Box((0.18, 0.14, 0.045)),
        origin=Origin(xyz=(side * KINGPIN_X, 0.0, 0.095)),
        material=cast_iron,
        name=f"{prefix}_upper_lug",
    )
    beam.visual(
        Box((0.18, 0.14, 0.045)),
        origin=Origin(xyz=(side * KINGPIN_X, 0.0, -0.095)),
        material=cast_iron,
        name=f"{prefix}_lower_lug",
    )
    beam.visual(
        Cylinder(radius=0.014, length=0.286),
        origin=Origin(xyz=(side * KINGPIN_X, 0.0, 0.0)),
        material=pin_steel,
        name=f"{prefix}_kingpin_pin",
    )
    beam.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(side * KINGPIN_X, 0.0, 0.129)),
        material=pin_steel,
        name=f"{prefix}_kingpin_head",
    )
    beam.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(side * KINGPIN_X, 0.0, -0.129)),
        material=pin_steel,
        name=f"{prefix}_kingpin_nut",
    )


def _add_stub_visuals(stub, cast_iron, pin_steel, bronze, side_name: str) -> None:
    arm_y_sign = 1.0 if side_name == "left" else -1.0

    stub.visual(
        Cylinder(radius=0.035, length=0.120),
        origin=Origin(),
        material=bronze,
        name="kingpin_sleeve",
    )
    stub.visual(
        Box((0.120, 0.100, 0.080)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=cast_iron,
        name="knuckle_body",
    )
    stub.visual(
        Cylinder(radius=0.045, length=0.100),
        origin=_x_cylinder_origin(0.120, 0.0, 0.0),
        material=cast_iron,
        name="bearing_shoulder",
    )
    stub.visual(
        Cylinder(radius=0.028, length=0.280),
        origin=_x_cylinder_origin(0.200, 0.0, 0.0),
        material=pin_steel,
        name="spindle_shaft",
    )
    stub.visual(
        Box((0.105, 0.205, 0.038)),
        origin=Origin(xyz=(0.035, arm_y_sign * 0.125, 0.0)),
        material=cast_iron,
        name="steering_arm",
    )
    stub.visual(
        Cylinder(radius=0.023, length=0.048),
        origin=Origin(xyz=(0.030, arm_y_sign * 0.230, 0.0)),
        material=pin_steel,
        name="tie_rod_eye",
    )
    stub.inertial = Inertial.from_geometry(
        Box((0.36, 0.28, 0.16)),
        mass=8.0,
        origin=Origin(xyz=(0.14, 0.04, 0.0)),
    )
    stub.meta["description"] = f"{side_name} steerable stub spindle and kingpin sleeve"


def _add_hub_visuals(hub, hub_steel, dark_steel) -> None:
    hub.visual(
        Cylinder(radius=0.055, length=0.160),
        origin=_x_cylinder_origin(0.0, 0.0, 0.0),
        material=dark_steel,
        name="hub_bearing_barrel",
    )
    hub.visual(
        Cylinder(radius=0.082, length=0.040),
        origin=_x_cylinder_origin(-0.055, 0.0, 0.0),
        material=hub_steel,
        name="inner_bearing_collar",
    )
    hub.visual(
        Cylinder(radius=0.122, length=0.035),
        origin=_x_cylinder_origin(0.095, 0.0, 0.0),
        material=hub_steel,
        name="wheel_flange",
    )
    hub.visual(
        Cylinder(radius=0.043, length=0.050),
        origin=_x_cylinder_origin(0.127, 0.0, 0.0),
        material=dark_steel,
        name="dust_cap",
    )

    for i in range(5):
        angle = 2.0 * pi * i / 5.0
        y = 0.084 * cos(angle)
        z = 0.084 * sin(angle)
        hub.visual(
            Cylinder(radius=0.008, length=0.050),
            origin=_x_cylinder_origin(0.132, y, z),
            material=dark_steel,
            name=f"wheel_stud_{i}",
        )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.22),
        mass=6.0,
        origin=_x_cylinder_origin(0.035, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_stub_axle_kingpin_assembly")

    axle_paint = model.material("axle_paint", rgba=(0.10, 0.11, 0.12, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    hub_steel = model.material("hub_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.08, 0.085, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.65, 0.45, 0.22, 1.0))

    beam = model.part("beam_axle")
    beam.visual(
        Box((1.28, 0.120, 0.105)),
        origin=Origin(),
        material=axle_paint,
        name="rectangular_beam",
    )
    beam.visual(
        Box((0.20, 0.135, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cast_iron,
        name="center_pad",
    )
    beam.visual(
        Box((1.02, 0.025, 0.020)),
        origin=Origin(xyz=(0.0, 0.073, 0.030)),
        material=pin_steel,
        name="front_weld_bead",
    )
    beam.visual(
        Box((1.02, 0.025, 0.020)),
        origin=Origin(xyz=(0.0, -0.073, -0.030)),
        material=pin_steel,
        name="rear_weld_bead",
    )
    _add_beam_end(beam, 1.0, cast_iron, pin_steel)
    _add_beam_end(beam, -1.0, cast_iron, pin_steel)
    beam.inertial = Inertial.from_geometry(
        Box((1.64, 0.18, 0.30)),
        mass=45.0,
        origin=Origin(),
    )

    left_stub = model.part("left_stub")
    _add_stub_visuals(left_stub, cast_iron, pin_steel, bearing_bronze, "left")

    right_stub = model.part("right_stub")
    _add_stub_visuals(right_stub, cast_iron, pin_steel, bearing_bronze, "right")

    left_hub = model.part("left_hub")
    _add_hub_visuals(left_hub, hub_steel, dark_steel)

    right_hub = model.part("right_hub")
    _add_hub_visuals(right_hub, hub_steel, dark_steel)

    model.articulation(
        "left_kingpin",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=left_stub,
        origin=Origin(xyz=(KINGPIN_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.8, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "right_kingpin",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=right_stub,
        origin=Origin(xyz=(-KINGPIN_X, 0.0, 0.0), rpy=(0.0, 0.0, pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.8, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=left_stub,
        child=left_hub,
        origin=Origin(xyz=(HUB_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=35.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=right_stub,
        child=right_hub,
        origin=Origin(xyz=(HUB_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam_axle")
    left_stub = object_model.get_part("left_stub")
    right_stub = object_model.get_part("right_stub")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")
    left_kingpin = object_model.get_articulation("left_kingpin")
    right_kingpin = object_model.get_articulation("right_kingpin")
    left_spin = object_model.get_articulation("left_hub_spin")
    right_spin = object_model.get_articulation("right_hub_spin")

    for stub, hub, side in (
        (left_stub, left_hub, "left"),
        (right_stub, right_hub, "right"),
    ):
        ctx.allow_overlap(
            stub,
            hub,
            elem_a="spindle_shaft",
            elem_b="hub_bearing_barrel",
            reason="The rotating hub bearing is intentionally represented as a solid sleeve captured around the fixed spindle shaft.",
        )
        ctx.allow_overlap(
            hub,
            stub,
            elem_a="inner_bearing_collar",
            elem_b="spindle_shaft",
            reason="The inboard bearing collar is part of the hub sleeve and intentionally surrounds the spindle shoulder area.",
        )
        ctx.expect_within(
            stub,
            hub,
            axes="yz",
            inner_elem="spindle_shaft",
            outer_elem="hub_bearing_barrel",
            margin=0.002,
            name=f"{side} spindle is centered in bearing sleeve",
        )
        ctx.expect_overlap(
            stub,
            hub,
            axes="x",
            elem_a="spindle_shaft",
            elem_b="hub_bearing_barrel",
            min_overlap=0.080,
            name=f"{side} bearing retains spindle insertion",
        )
        ctx.expect_within(
            stub,
            hub,
            axes="yz",
            inner_elem="spindle_shaft",
            outer_elem="inner_bearing_collar",
            margin=0.002,
            name=f"{side} spindle is centered in inner bearing collar",
        )
        ctx.expect_overlap(
            stub,
            hub,
            axes="x",
            elem_a="spindle_shaft",
            elem_b="inner_bearing_collar",
            min_overlap=0.030,
            name=f"{side} inner bearing collar surrounds spindle",
        )

    for stub, side in ((left_stub, "left"), (right_stub, "right")):
        ctx.allow_overlap(
            beam,
            stub,
            elem_a=f"{side}_kingpin_pin",
            elem_b="kingpin_sleeve",
            reason="The kingpin shank is intentionally captured inside the bronze sleeve so the stub can pivot on the beam yoke.",
        )
        ctx.expect_within(
            beam,
            stub,
            axes="xy",
            inner_elem=f"{side}_kingpin_pin",
            outer_elem="kingpin_sleeve",
            margin=0.002,
            name=f"{side} kingpin pin is centered in sleeve",
        )
        ctx.expect_overlap(
            beam,
            stub,
            axes="z",
            elem_a=f"{side}_kingpin_pin",
            elem_b="kingpin_sleeve",
            min_overlap=0.100,
            name=f"{side} kingpin pin passes through sleeve",
        )
        ctx.expect_gap(
            beam,
            stub,
            axis="z",
            positive_elem=f"{side}_upper_lug",
            negative_elem="kingpin_sleeve",
            min_gap=0.006,
            max_gap=0.025,
            name=f"{side} sleeve clears upper yoke lug",
        )
        ctx.expect_gap(
            stub,
            beam,
            axis="z",
            positive_elem="kingpin_sleeve",
            negative_elem=f"{side}_lower_lug",
            min_gap=0.006,
            max_gap=0.025,
            name=f"{side} sleeve clears lower yoke lug",
        )
        ctx.expect_overlap(
            stub,
            beam,
            axes="xy",
            elem_a="kingpin_sleeve",
            elem_b=f"{side}_upper_lug",
            min_overlap=0.045,
            name=f"{side} kingpin sleeve sits inside yoke footprint",
        )

    ctx.check(
        "both kingpins are limited steering pivots",
        left_kingpin.motion_limits is not None
        and right_kingpin.motion_limits is not None
        and left_kingpin.motion_limits.lower <= -0.60
        and left_kingpin.motion_limits.upper >= 0.60
        and right_kingpin.motion_limits.lower <= -0.60
        and right_kingpin.motion_limits.upper >= 0.60,
        details=f"left={left_kingpin.motion_limits}, right={right_kingpin.motion_limits}",
    )

    left_rest = ctx.part_world_position(left_hub)
    right_rest = ctx.part_world_position(right_hub)
    with ctx.pose({left_kingpin: 0.50, right_kingpin: 0.50}):
        left_steered = ctx.part_world_position(left_hub)
        right_steered = ctx.part_world_position(right_hub)
    ctx.check(
        "stub spindles sweep about vertical kingpins",
        left_rest is not None
        and right_rest is not None
        and left_steered is not None
        and right_steered is not None
        and left_steered[1] > left_rest[1] + 0.10
        and right_steered[1] < right_rest[1] - 0.10,
        details=f"left rest={left_rest}, left steered={left_steered}, right rest={right_rest}, right steered={right_steered}",
    )

    with ctx.pose({left_spin: 1.70, right_spin: -2.10}):
        left_spun = ctx.part_world_position(left_hub)
        right_spun = ctx.part_world_position(right_hub)
    ctx.check(
        "hub spin is coaxial without translating hub centers",
        left_rest is not None
        and right_rest is not None
        and left_spun is not None
        and right_spun is not None
        and max(abs(left_spun[i] - left_rest[i]) for i in range(3)) < 1e-6
        and max(abs(right_spun[i] - right_rest[i]) for i in range(3)) < 1e-6,
        details=f"left rest={left_rest}, spun={left_spun}; right rest={right_rest}, spun={right_spun}",
    )

    return ctx.report()


object_model = build_object_model()
