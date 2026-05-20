from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _bar_pitch(dx: float, dz: float) -> float:
    """Pitch a local +X box/cylinder segment through a world X/Z rise."""
    return -math.atan2(dz, dx)


def _add_scissor_bar(
    model: ArticulatedObject,
    *,
    name: str,
    parent,
    anchor_xyz: tuple[float, float, float],
    dx: float,
    dz: float,
    source_joint: str | None,
    mimic_multiplier: float,
    lower: float,
    upper: float,
    pin_length: float,
    material: str,
    pin_material: str,
):
    length = math.hypot(dx, dz)
    pitch = _bar_pitch(dx, dz)
    bar = model.part(name)
    bar.visual(
        Box((length, 0.018, 0.034)),
        origin=Origin(xyz=(length * 0.5, 0.0, 0.0)),
        material=material,
        name="strap",
    )
    for i, x in enumerate((0.0, length)):
        bar.visual(
            Cylinder(radius=0.026, length=pin_length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pin_material,
            name=f"pin_{i}",
        )
    model.articulation(
        f"{parent.name}_to_{name}",
        ArticulationType.REVOLUTE,
        parent=parent,
        child=bar,
        origin=Origin(xyz=anchor_xyz, rpy=(0.0, pitch, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.0, lower=lower, upper=upper),
        mimic=Mimic(source_joint, multiplier=mimic_multiplier) if source_joint else None,
    )
    return bar


def _add_gate_stile(part, *, material: str, roller_material: str) -> None:
    part.visual(
        Box((0.035, 0.035, 1.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=material,
        name="vertical_stile",
    )
    part.visual(
        Box((0.020, 0.105, 0.020)),
        origin=Origin(xyz=(0.0, 0.045, 1.21)),
        material=material,
        name="hanger_neck",
    )
    part.visual(
        Cylinder(radius=0.025, length=0.050),
        origin=Origin(xyz=(0.0, 0.065, 1.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_material,
        name="top_roller",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_elevator")

    model.material("galvanized_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    model.material("dark_steel", rgba=(0.16, 0.17, 0.17, 1.0))
    model.material("brushed_door", rgba=(0.42, 0.44, 0.46, 1.0))
    model.material("safety_yellow", rgba=(0.95, 0.72, 0.10, 1.0))
    model.material("rubber_black", rgba=(0.02, 0.02, 0.018, 1.0))

    shaft = model.part("shaft")
    shaft.visual(
        Box((2.05, 1.65, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material="dark_steel",
        name="pit_slab",
    )
    for x in (-0.96, 0.96):
        for y in (-0.75, 0.75):
            shaft.visual(
                Box((0.08, 0.08, 3.55)),
                origin=Origin(xyz=(x, y, 1.895)),
                material="dark_steel",
                name=f"corner_post_{x}_{y}",
            )
    for y in (-0.75, 0.75):
        shaft.visual(
            Box((2.00, 0.08, 0.10)),
            origin=Origin(xyz=(0.0, y, 3.70)),
            material="dark_steel",
            name=f"top_beam_y_{y}",
        )
    for x in (-0.96, 0.96):
        shaft.visual(
            Box((0.08, 1.58, 0.10)),
            origin=Origin(xyz=(x, 0.0, 3.70)),
            material="dark_steel",
            name=f"top_beam_x_{x}",
        )
    for x in (-0.84, 0.84):
        shaft.visual(
            Box((0.045, 0.09, 3.46)),
            origin=Origin(xyz=(x, -0.06, 1.85)),
            material="galvanized_steel",
            name=f"guide_rail_{x}",
        )
    shaft.visual(
        Box((1.35, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, -0.82, 0.17)),
        material="safety_yellow",
        name="landing_sill",
    )

    car = model.part("car")
    car.visual(
        Box((1.46, 1.22, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="galvanized_steel",
        name="floor_plate",
    )
    for x in (-0.73, 0.73):
        car.visual(
            Box((0.08, 1.22, 1.90)),
            origin=Origin(xyz=(x, 0.0, 1.00)),
            material="galvanized_steel",
            name=f"side_wall_{x}",
        )
    car.visual(
        Box((1.46, 0.07, 1.90)),
        origin=Origin(xyz=(0.0, 0.61, 1.00)),
        material="galvanized_steel",
        name="back_wall",
    )
    car.visual(
        Box((1.46, 1.22, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.96)),
        material="galvanized_steel",
        name="roof_plate",
    )
    for x in (-0.73, 0.73):
        car.visual(
            Box((0.08, 0.08, 1.90)),
            origin=Origin(xyz=(x, -0.61, 1.00)),
            material="dark_steel",
            name=f"front_post_{x}",
        )
    car.visual(
        Box((1.46, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, -0.61, 1.87)),
        material="dark_steel",
        name="front_header",
    )
    car.visual(
        Box((1.46, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, -0.61, 0.11)),
        material="dark_steel",
        name="front_sill",
    )
    car.visual(
        Box((0.06, 0.06, 1.62)),
        origin=Origin(xyz=(-0.65, -0.49, 1.11)),
        material="dark_steel",
        name="door_hinge_jamb",
    )
    for z in (1.765, 0.39):
        car.visual(
            Box((1.40, 0.06, 0.06)),
            origin=Origin(xyz=(0.0, -0.715, z)),
            material="dark_steel",
            name=f"gate_track_{z}",
        )
        for x in (-0.67, 0.67):
            car.visual(
                Box((0.08, 0.13, 0.08)),
                origin=Origin(xyz=(x, -0.66, z)),
                material="dark_steel",
                name=f"track_standoff_{x}_{z}",
            )
    car.visual(
        Box((0.035, 0.035, 1.25)),
        origin=Origin(xyz=(-0.55, -0.78, 1.075)),
        material="safety_yellow",
        name="gate_anchor_stile",
    )
    car.visual(
        Box((0.045, 0.12, 0.06)),
        origin=Origin(xyz=(-0.55, -0.735, 1.705)),
        material="safety_yellow",
        name="gate_anchor_bridge",
    )
    for x in (-0.785, 0.785):
        car.visual(
            Box((0.065, 0.12, 0.16)),
            origin=Origin(xyz=(x, -0.06, 1.20)),
            material="dark_steel",
            name=f"guide_shoe_{x}",
        )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.55, lower=0.0, upper=1.45),
    )

    inner_door = model.part("inner_door")
    inner_door.visual(
        Box((1.16, 0.045, 1.55)),
        origin=Origin(xyz=(0.58, 0.0, 0.775)),
        material="brushed_door",
        name="solid_panel",
    )
    for z in (0.22, 0.78, 1.34):
        inner_door.visual(
            Cylinder(radius=0.018, length=0.34),
            origin=Origin(xyz=(0.0, -0.030, z), rpy=(0.0, 0.0, 0.0)),
            material="dark_steel",
            name=f"hinge_barrel_{z}",
        )
    inner_door.visual(
        Box((0.08, 0.07, 0.035)),
        origin=Origin(xyz=(1.05, -0.048, 0.86)),
        material="dark_steel",
        name="pull_handle",
    )
    model.articulation(
        "car_to_inner_door",
        ArticulationType.REVOLUTE,
        parent=car,
        child=inner_door,
        origin=Origin(xyz=(-0.602, -0.49, 0.30)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    gate_stile_1 = model.part("gate_stile_1")
    gate_stile_2 = model.part("gate_stile_2")
    gate_lead = model.part("gate_lead")
    for stile in (gate_stile_1, gate_stile_2, gate_lead):
        _add_gate_stile(stile, material="safety_yellow", roller_material="rubber_black")

    model.articulation(
        "car_to_gate_lead",
        ArticulationType.PRISMATIC,
        parent=car,
        child=gate_lead,
        origin=Origin(xyz=(0.55, -0.78, 0.45)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.7, lower=0.0, upper=1.0),
    )
    model.articulation(
        "car_to_gate_stile_1",
        ArticulationType.PRISMATIC,
        parent=car,
        child=gate_stile_1,
        origin=Origin(xyz=(-0.18, -0.78, 0.45)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=0.0, upper=0.37),
        mimic=Mimic("car_to_gate_lead", multiplier=0.37),
    )
    model.articulation(
        "car_to_gate_stile_2",
        ArticulationType.PRISMATIC,
        parent=car,
        child=gate_stile_2,
        origin=Origin(xyz=(0.18, -0.78, 0.45)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=0.0, upper=0.73),
        mimic=Mimic("car_to_gate_lead", multiplier=0.73),
    )

    _add_scissor_bar(
        model,
        name="scissor_bar_0a",
        parent=car,
        anchor_xyz=(-0.55, -0.82, 0.60),
        dx=0.37,
        dz=0.70,
        source_joint=None,
        mimic_multiplier=0.0,
        lower=-0.45,
        upper=0.0,
        pin_length=0.045,
        material="dark_steel",
        pin_material="galvanized_steel",
    )
    _add_scissor_bar(
        model,
        name="scissor_bar_0b",
        parent=car,
        anchor_xyz=(-0.55, -0.85, 1.30),
        dx=0.37,
        dz=-0.70,
        source_joint="car_to_scissor_bar_0a",
        mimic_multiplier=-1.0,
        lower=0.0,
        upper=0.45,
        pin_length=0.105,
        material="dark_steel",
        pin_material="galvanized_steel",
    )
    _add_scissor_bar(
        model,
        name="scissor_bar_1a",
        parent=gate_stile_1,
        anchor_xyz=(0.0, -0.04, 0.15),
        dx=0.36,
        dz=0.70,
        source_joint="car_to_scissor_bar_0a",
        mimic_multiplier=1.0,
        lower=-0.45,
        upper=0.0,
        pin_length=0.045,
        material="dark_steel",
        pin_material="galvanized_steel",
    )
    _add_scissor_bar(
        model,
        name="scissor_bar_1b",
        parent=gate_stile_1,
        anchor_xyz=(0.0, -0.07, 0.85),
        dx=0.36,
        dz=-0.70,
        source_joint="car_to_scissor_bar_0a",
        mimic_multiplier=-1.0,
        lower=0.0,
        upper=0.45,
        pin_length=0.105,
        material="dark_steel",
        pin_material="galvanized_steel",
    )
    _add_scissor_bar(
        model,
        name="scissor_bar_2a",
        parent=gate_stile_2,
        anchor_xyz=(0.0, -0.04, 0.15),
        dx=0.37,
        dz=0.70,
        source_joint="car_to_scissor_bar_0a",
        mimic_multiplier=1.0,
        lower=-0.45,
        upper=0.0,
        pin_length=0.045,
        material="dark_steel",
        pin_material="galvanized_steel",
    )
    _add_scissor_bar(
        model,
        name="scissor_bar_2b",
        parent=gate_stile_2,
        anchor_xyz=(0.0, -0.07, 0.85),
        dx=0.37,
        dz=-0.70,
        source_joint="car_to_scissor_bar_0a",
        mimic_multiplier=-1.0,
        lower=0.0,
        upper=0.45,
        pin_length=0.105,
        material="dark_steel",
        pin_material="galvanized_steel",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    inner_door = object_model.get_part("inner_door")
    gate_stile_1 = object_model.get_part("gate_stile_1")
    gate_stile_2 = object_model.get_part("gate_stile_2")
    gate_lead = object_model.get_part("gate_lead")
    lift = object_model.get_articulation("shaft_to_car")
    door_hinge = object_model.get_articulation("car_to_inner_door")
    gate_slide = object_model.get_articulation("car_to_gate_lead")

    shared_scissor_pins = (
        ("scissor_bar_0a", "scissor_bar_1b", "pin_1", "pin_0"),
        ("scissor_bar_0a", "scissor_bar_1b", "strap", "pin_0"),
        ("scissor_bar_0b", "scissor_bar_1a", "pin_1", "pin_0"),
        ("scissor_bar_0b", "scissor_bar_1a", "pin_1", "strap"),
        ("scissor_bar_1a", "scissor_bar_2b", "pin_1", "pin_0"),
        ("scissor_bar_1a", "scissor_bar_2b", "strap", "pin_0"),
        ("scissor_bar_1b", "scissor_bar_2a", "pin_1", "pin_0"),
        ("scissor_bar_1b", "scissor_bar_2a", "pin_1", "strap"),
    )
    for link_a, link_b, elem_a, elem_b in shared_scissor_pins:
        ctx.allow_overlap(
            link_a,
            link_b,
            elem_a=elem_a,
            elem_b=elem_b,
            reason="The crossing accordion straps share a single coaxial scissor pivot pin at this joint.",
        )
        ctx.expect_overlap(
            link_a,
            link_b,
            elem_a=elem_a,
            elem_b=elem_b,
            axes="xyz",
            min_overlap=0.010,
            name=f"{link_a} and {link_b} share a pivot pin",
        )

    ctx.expect_within(
        car,
        shaft,
        axes="xy",
        margin=0.02,
        name="car rides between the shaft guide rails",
    )

    car_rest = ctx.part_world_position(car)
    with ctx.pose({lift: 1.20}):
        car_raised = ctx.part_world_position(car)
    ctx.check(
        "service car travels vertically",
        car_rest is not None
        and car_raised is not None
        and car_raised[2] > car_rest[2] + 1.15,
        details=f"rest={car_rest}, raised={car_raised}",
    )

    lead_rest = ctx.part_world_position(gate_lead)
    with ctx.pose({gate_slide: 0.90}):
        lead_folded = ctx.part_world_position(gate_lead)
        stile_positions = [
            ctx.part_world_position(gate_stile_1),
            ctx.part_world_position(gate_stile_2),
            ctx.part_world_position(gate_lead),
        ]
    xs = [p[0] for p in stile_positions if p is not None]
    ctx.check(
        "accordion gate collapses toward one jamb",
        lead_rest is not None
        and lead_folded is not None
        and lead_folded[0] < lead_rest[0] - 0.80
        and len(xs) == 3
        and max(xs) - min(xs) < 0.18,
        details=f"lead_rest={lead_rest}, lead_folded={lead_folded}, stile_xs={xs}",
    )

    door_rest_aabb = ctx.part_world_aabb(inner_door)
    with ctx.pose({door_hinge: 1.10}):
        door_open_aabb = ctx.part_world_aabb(inner_door)
    ctx.check(
        "inner door swings outward on its hinge",
        door_rest_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[0][1] < door_rest_aabb[0][1] - 0.45,
        details=f"rest_aabb={door_rest_aabb}, open_aabb={door_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
