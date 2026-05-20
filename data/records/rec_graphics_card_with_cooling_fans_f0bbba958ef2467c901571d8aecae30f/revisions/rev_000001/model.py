from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, center: tuple[float, float] = (0.0, 0.0), segments: int = 64):
    cx, cy = center
    return [
        (cx + radius * math.cos(2.0 * math.pi * i / segments), cy + radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_short_graphics_card")

    pcb_green = model.material("pcb_green", rgba=(0.02, 0.18, 0.08, 1.0))
    black = model.material("matte_black", rgba=(0.006, 0.007, 0.008, 1.0))
    dark_grey = model.material("dark_anodized_aluminum", rgba=(0.10, 0.105, 0.11, 1.0))
    fin_grey = model.material("graphite_fin_edges", rgba=(0.28, 0.29, 0.30, 1.0))
    bracket_metal = model.material("brushed_slot_bracket", rgba=(0.72, 0.70, 0.65, 1.0))
    copper = model.material("copper_heatpipe", rgba=(0.85, 0.42, 0.16, 1.0))
    gold = model.material("gold_edge_contacts", rgba=(1.0, 0.72, 0.18, 1.0))
    rubber = model.material("rubber_pad", rgba=(0.015, 0.015, 0.014, 1.0))

    fan_centers = [(-0.038, 0.017), (0.044, -0.017)]

    card = model.part("card")
    card.visual(
        Box((0.195, 0.105, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pcb_green,
        name="pcb",
    )
    card.visual(
        Box((0.060, 0.010, 0.0015)),
        origin=Origin(xyz=(0.030, -0.057, -0.00025)),
        material=gold,
        name="edge_contacts",
    )
    card.visual(
        Box((0.006, 0.128, 0.046)),
        origin=Origin(xyz=(-0.1005, 0.0, 0.022)),
        material=bracket_metal,
        name="bracket_plate",
    )
    card.visual(
        Box((0.004, 0.024, 0.012)),
        origin=Origin(xyz=(-0.097, 0.025, 0.024)),
        material=black,
        name="display_port_cutout",
    )
    card.visual(
        Box((0.004, 0.030, 0.010)),
        origin=Origin(xyz=(-0.097, -0.020, 0.024)),
        material=black,
        name="hdmi_cutout",
    )
    card.visual(
        Box((0.160, 0.090, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, 0.008)),
        material=dark_grey,
        name="heatsink_block",
    )

    for idx, x in enumerate([-0.055, -0.035, -0.015, 0.005, 0.025, 0.045, 0.065]):
        card.visual(
            Box((0.006, 0.084, 0.009)),
            origin=Origin(xyz=(x, 0.0, 0.0195)),
            material=fin_grey,
            name=f"heatsink_fin_{idx}",
        )

    for idx, y in enumerate((-0.023, 0.023)):
        card.visual(
            Cylinder(radius=0.004, length=0.146),
            origin=Origin(xyz=(0.010, y, 0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=copper,
            name=f"heatpipe_{idx}",
        )

    shroud_solid = cq.Workplane("XY").box(0.174, 0.102, 0.006).edges("|Z").fillet(0.006)
    shroud_cutters = cq.Workplane("XY").pushPoints(fan_centers).circle(0.033).extrude(0.020).translate((0.0, 0.0, -0.010))
    shroud_mesh = mesh_from_cadquery(
        shroud_solid.cut(shroud_cutters),
        "stubby_dual_fan_shroud",
    )
    card.visual(
        shroud_mesh,
        origin=Origin(xyz=(0.010, 0.0, 0.027)),
        material=black,
        name="stubby_shroud",
    )
    card.visual(
        Box((0.176, 0.006, 0.015)),
        origin=Origin(xyz=(0.010, 0.054, 0.0185)),
        material=black,
        name="top_shroud_lip",
    )
    card.visual(
        Box((0.176, 0.006, 0.015)),
        origin=Origin(xyz=(0.010, -0.054, 0.0185)),
        material=black,
        name="bottom_shroud_lip",
    )

    for fan_idx, (fx, fy) in enumerate(fan_centers):
        card.visual(
            Cylinder(radius=0.006, length=0.009),
            origin=Origin(xyz=(fx + 0.010, fy, 0.0198)),
            material=dark_grey,
            name=f"fan_{fan_idx}_stator_boss",
        )
        for screw_idx, angle in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
            card.visual(
                Cylinder(radius=0.0024, length=0.0022),
                origin=Origin(
                    xyz=(
                        fx + 0.010 + 0.039 * math.cos(angle),
                        fy + 0.039 * math.sin(angle),
                        0.0309,
                    )
                ),
                material=bracket_metal,
                name=f"fan_{fan_idx}_screw_{screw_idx}",
            )

    card.visual(
        Box((0.014, 0.018, 0.014)),
        origin=Origin(xyz=(-0.101, -0.066, 0.013)),
        material=bracket_metal,
        name="hinge_block",
    )
    card.visual(
        Cylinder(radius=0.0022, length=0.020),
        origin=Origin(xyz=(-0.101, -0.066, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="hinge_pin",
    )

    rotor_geometry = FanRotorGeometry(
        0.0285,
        0.0088,
        9,
        thickness=0.006,
        blade_pitch_deg=32.0,
        blade_sweep_deg=25.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.12, tip_clearance=0.001),
        hub=FanRotorHub(style="domed", rear_collar_height=0.0015, rear_collar_radius=0.007),
    )

    for fan_idx, (fx, fy) in enumerate(fan_centers):
        rotor = model.part(f"fan_{fan_idx}")
        rotor.visual(
            mesh_from_geometry(rotor_geometry, f"fan_{fan_idx}_rotor"),
            origin=Origin(),
            material=black,
            name="rotor",
        )
        rotor.visual(
            Cylinder(radius=0.003, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=dark_grey,
            name="hub_shaft",
        )
        model.articulation(
            f"card_to_fan_{fan_idx}",
            ArticulationType.CONTINUOUS,
            parent=card,
            child=rotor,
            origin=Origin(xyz=(fx + 0.010, fy, 0.027)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.08, velocity=80.0),
        )

    brace = model.part("support_brace")
    brace.visual(
        Cylinder(radius=0.0055, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_metal,
        name="hinge_knuckle",
    )
    brace_length = 0.185
    brace_angle = 0.18
    brace.visual(
        Box((brace_length, 0.006, 0.006)),
        origin=Origin(
            xyz=(
                0.5 * brace_length * math.cos(brace_angle),
                0.0,
                -0.5 * brace_length * math.sin(brace_angle),
            ),
            rpy=(0.0, brace_angle, 0.0),
        ),
        material=bracket_metal,
        name="brace_arm",
    )
    brace.visual(
        Box((0.016, 0.014, 0.005)),
        origin=Origin(
            xyz=(
                brace_length * math.cos(brace_angle) + 0.005,
                0.0,
                -brace_length * math.sin(brace_angle) - 0.003,
            )
        ),
        material=rubber,
        name="brace_foot",
    )
    model.articulation(
        "card_to_support_brace",
        ArticulationType.REVOLUTE,
        parent=card,
        child=brace,
        origin=Origin(xyz=(-0.101, -0.066, 0.013)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=1.5, lower=0.0, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card = object_model.get_part("card")
    fan_0 = object_model.get_part("fan_0")
    fan_1 = object_model.get_part("fan_1")
    brace = object_model.get_part("support_brace")
    fan_joint_0 = object_model.get_articulation("card_to_fan_0")
    fan_joint_1 = object_model.get_articulation("card_to_fan_1")
    brace_joint = object_model.get_articulation("card_to_support_brace")

    ctx.check(
        "two axial fan spin joints are normal to the card face",
        fan_joint_0.axis == (0.0, 0.0, 1.0) and fan_joint_1.axis == (0.0, 0.0, 1.0),
        details=f"fan axes: {fan_joint_0.axis}, {fan_joint_1.axis}",
    )
    ctx.expect_within(fan_0, card, axes="xy", inner_elem="rotor", outer_elem="stubby_shroud", margin=0.002)
    ctx.expect_within(fan_1, card, axes="xy", inner_elem="rotor", outer_elem="stubby_shroud", margin=0.002)
    ctx.expect_gap(
        fan_0,
        card,
        axis="z",
        positive_elem="rotor",
        negative_elem="fan_0_stator_boss",
        max_gap=0.001,
        max_penetration=0.001,
        name="front fan rotor rides on its stator boss",
    )
    ctx.expect_gap(
        fan_1,
        card,
        axis="z",
        positive_elem="rotor",
        negative_elem="fan_1_stator_boss",
        max_gap=0.001,
        max_penetration=0.001,
        name="rear fan rotor rides on its stator boss",
    )
    for fan, boss_name in ((fan_0, "fan_0_stator_boss"), (fan_1, "fan_1_stator_boss")):
        ctx.allow_overlap(
            card,
            fan,
            elem_a=boss_name,
            elem_b="hub_shaft",
            reason="The fan shaft is intentionally captured inside its fixed stator boss as a bearing.",
        )
        ctx.expect_within(
            fan,
            card,
            axes="xy",
            inner_elem="hub_shaft",
            outer_elem=boss_name,
            margin=0.0005,
            name=f"{fan.name} shaft is centered in its stator boss",
        )
        ctx.expect_overlap(
            fan,
            card,
            axes="z",
            elem_a="hub_shaft",
            elem_b=boss_name,
            min_overlap=0.004,
            name=f"{fan.name} shaft remains inserted in its stator boss",
        )

    ctx.allow_overlap(
        card,
        brace,
        elem_a="hinge_block",
        elem_b="hinge_knuckle",
        reason="The support brace knuckle is intentionally captured by the small bracket hinge block.",
    )
    ctx.expect_overlap(
        card,
        brace,
        axes="yz",
        elem_a="hinge_block",
        elem_b="hinge_knuckle",
        min_overlap=0.004,
        name="brace hinge knuckle is captured in the bracket block",
    )

    stowed_aabb = ctx.part_element_world_aabb(brace, elem="brace_foot")
    with ctx.pose({brace_joint: 0.65}):
        deployed_aabb = ctx.part_element_world_aabb(brace, elem="brace_foot")
    ctx.check(
        "support brace folds down from the bracket hinge",
        stowed_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[0][2] < stowed_aabb[0][2] - 0.025,
        details=f"rest={stowed_aabb}, folded={deployed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
