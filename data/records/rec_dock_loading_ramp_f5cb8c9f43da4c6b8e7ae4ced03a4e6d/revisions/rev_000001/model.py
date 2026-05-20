from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="container_dock_leveler")

    concrete = Material("poured_concrete", rgba=(0.48, 0.46, 0.42, 1.0))
    dark_steel = Material("dark_painted_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    ribbed_steel = Material("ribbed_galvanized_steel", rgba=(0.42, 0.45, 0.46, 1.0))
    worn_edges = Material("worn_bright_edges", rgba=(0.66, 0.69, 0.68, 1.0))
    warning_yellow = Material("safety_yellow", rgba=(0.95, 0.70, 0.10, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    for material in (concrete, dark_steel, ribbed_steel, worn_edges, warning_yellow, rubber):
        model.material(material.name, rgba=material.rgba)

    dock = model.part("dock_face")
    dock.visual(
        Box((0.12, 2.50, 1.45)),
        origin=Origin(xyz=(-0.16, 0.0, -0.62)),
        material=concrete,
        name="dock_face_panel",
    )
    dock.visual(
        Box((0.36, 2.50, 0.12)),
        origin=Origin(xyz=(-0.24, 0.0, 0.11)),
        material=concrete,
        name="dock_deck_sill",
    )
    dock.visual(
        Box((3.10, 2.50, 0.08)),
        origin=Origin(xyz=(1.28, 0.0, -1.08)),
        material=concrete,
        name="pit_floor",
    )
    dock.visual(
        Box((0.08, 2.42, 0.34)),
        origin=Origin(xyz=(-0.14, 0.0, -0.04)),
        material=dark_steel,
        name="pivot_backplate",
    )
    for suffix, y in (("0", -1.16), ("1", 1.16)):
        dock.visual(
            Box((0.24, 0.12, 0.38)),
            origin=Origin(xyz=(0.02, y, -0.04)),
            material=dark_steel,
            name=f"pivot_cheek_{suffix}",
        )
        dock.visual(
            Cylinder(radius=0.055, length=0.16),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=worn_edges,
            name=f"base_hinge_pin_{suffix}",
        )

    platform = model.part("platform")
    platform.visual(
        Box((2.30, 2.05, 0.10)),
        origin=Origin(xyz=(1.17, 0.0, 0.0)),
        material=ribbed_steel,
        name="deck_plate",
    )
    platform.visual(
        Box((2.30, 0.08, 0.16)),
        origin=Origin(xyz=(1.17, -1.01, 0.0)),
        material=dark_steel,
        name="side_beam_0",
    )
    platform.visual(
        Box((2.30, 0.08, 0.16)),
        origin=Origin(xyz=(1.17, 1.01, 0.0)),
        material=dark_steel,
        name="side_beam_1",
    )
    platform.visual(
        Box((0.10, 1.86, 0.12)),
        origin=Origin(xyz=(0.12, 0.0, -0.09)),
        material=dark_steel,
        name="rear_torque_tube",
    )
    for i, x in enumerate((0.55, 1.15, 1.75, 2.20)):
        platform.visual(
            Box((0.08, 1.86, 0.12)),
            origin=Origin(xyz=(x, 0.0, -0.105)),
            material=dark_steel,
            name=f"crossmember_{i}",
        )
    for i, y in enumerate((-0.82, -0.60, -0.38, -0.16, 0.06, 0.28, 0.50, 0.72)):
        platform.visual(
            Box((2.06, 0.055, 0.035)),
            origin=Origin(xyz=(1.21, y, 0.066)),
            material=worn_edges,
            name=f"deck_rib_{i}",
        )
    platform.visual(
        Cylinder(radius=0.055, length=2.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=worn_edges,
        name="rear_hinge_barrel",
    )
    platform.visual(
        Box((0.18, 1.84, 0.08)),
        origin=Origin(xyz=(0.06, 0.0, -0.055)),
        material=dark_steel,
        name="rear_hinge_leaf",
    )

    leg_mounts = (("0", -0.78), ("1", 0.78))
    for suffix, y in leg_mounts:
        x = 1.42
        platform.visual(
            Box((0.025, 0.20, 0.44)),
            origin=Origin(xyz=(x - 0.0875, y, -0.26)),
            material=dark_steel,
            name=f"leg_{suffix}_sleeve_rear",
        )
        platform.visual(
            Box((0.025, 0.20, 0.44)),
            origin=Origin(xyz=(x + 0.0875, y, -0.26)),
            material=dark_steel,
            name=("leg_0_sleeve_front" if suffix == "0" else "leg_1_sleeve_front"),
        )
        platform.visual(
            Box((0.20, 0.025, 0.44)),
            origin=Origin(xyz=(x, y - 0.0875, -0.26)),
            material=dark_steel,
            name=f"leg_{suffix}_sleeve_side_0",
        )
        platform.visual(
            Box((0.20, 0.025, 0.44)),
            origin=Origin(xyz=(x, y + 0.0875, -0.26)),
            material=dark_steel,
            name=f"leg_{suffix}_sleeve_side_1",
        )
        platform.visual(
            Box((0.28, 0.035, 0.06)),
            origin=Origin(xyz=(x, y - 0.1175, -0.49)),
            material=dark_steel,
            name=f"leg_{suffix}_collar_y_0",
        )
        platform.visual(
            Box((0.28, 0.035, 0.06)),
            origin=Origin(xyz=(x, y + 0.1175, -0.49)),
            material=dark_steel,
            name=f"leg_{suffix}_collar_y_1",
        )
        platform.visual(
            Box((0.035, 0.28, 0.06)),
            origin=Origin(xyz=(x - 0.1175, y, -0.49)),
            material=dark_steel,
            name=f"leg_{suffix}_collar_x_0",
        )
        platform.visual(
            Box((0.035, 0.28, 0.06)),
            origin=Origin(xyz=(x + 0.1175, y, -0.49)),
            material=dark_steel,
            name=f"leg_{suffix}_collar_x_1",
        )

    for i, y in enumerate((-0.72, -0.36, 0.0, 0.36, 0.72)):
        platform.visual(
            Cylinder(radius=0.035, length=0.16),
            origin=Origin(xyz=(2.38, y, -0.08), rpy=(pi / 2.0, 0.0, 0.0)),
            material=worn_edges,
            name=f"front_hinge_knuckle_{i}",
        )
    platform.visual(
        Box((0.18, 1.92, 0.055)),
        origin=Origin(xyz=(2.255, 0.0, -0.055)),
        material=dark_steel,
        name="front_hinge_leaf",
    )
    platform.visual(
        Cylinder(radius=0.018, length=1.76),
        origin=Origin(xyz=(2.38, 0.0, -0.08), rpy=(pi / 2.0, 0.0, 0.0)),
        material=worn_edges,
        name="front_hinge_pin",
    )

    lip = model.part("approach_lip")
    lip.visual(
        Box((0.60, 2.02, 0.07)),
        origin=Origin(xyz=(0.38, 0.0, 0.08)),
        material=ribbed_steel,
        name="lip_plate",
    )
    lip.visual(
        Box((0.08, 1.96, 0.08)),
        origin=Origin(xyz=(0.075, 0.0, 0.04)),
        material=dark_steel,
        name="lip_hinge_leaf",
    )
    lip.visual(
        Box((0.07, 2.02, 0.03)),
        origin=Origin(xyz=(0.67, 0.0, 0.095)),
        material=warning_yellow,
        name="lip_nose_edge",
    )
    for i, y in enumerate((-0.54, -0.18, 0.18, 0.54)):
        lip.visual(
            Cylinder(radius=0.035, length=0.16),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=worn_edges,
            name=f"lip_hinge_knuckle_{i}",
        )
    for i, x in enumerate((0.22, 0.38, 0.54)):
        lip.visual(
            Box((0.035, 1.86, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.1275)),
            material=worn_edges,
            name=f"lip_cross_rib_{i}",
        )

    for suffix, y in leg_mounts:
        leg = model.part(f"support_leg_{suffix}")
        leg.visual(
            Box((0.150, 0.150, 0.78)),
            origin=Origin(xyz=(0.0, 0.0, -0.45)),
            material=worn_edges,
            name="inner_tube",
        )
        leg.visual(
            Box((0.28, 0.28, 0.05)),
            origin=Origin(xyz=(0.0, 0.0, -0.865)),
            material=dark_steel,
            name="foot_plate",
        )
        leg.visual(
            Box((0.23, 0.23, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.899)),
            material=rubber,
            name="rubber_pad",
        )

    model.articulation(
        "dock_to_platform",
        ArticulationType.REVOLUTE,
        parent=dock,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.25, lower=-0.10, upper=0.35),
    )
    model.articulation(
        "platform_to_lip",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=lip,
        origin=Origin(xyz=(2.38, 0.0, -0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3000.0, velocity=0.50, lower=-0.05, upper=0.75),
    )
    for suffix, y in leg_mounts:
        model.articulation(
            f"platform_to_leg_{suffix}",
            ArticulationType.PRISMATIC,
            parent=platform,
            child=f"support_leg_{suffix}",
            origin=Origin(xyz=(1.42, y, -0.06)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=6000.0, velocity=0.18, lower=0.0, upper=0.32),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dock = object_model.get_part("dock_face")
    platform = object_model.get_part("platform")
    lip = object_model.get_part("approach_lip")
    leg_0 = object_model.get_part("support_leg_0")
    leg_1 = object_model.get_part("support_leg_1")

    dock_hinge = object_model.get_articulation("dock_to_platform")
    lip_hinge = object_model.get_articulation("platform_to_lip")
    leg_slide_0 = object_model.get_articulation("platform_to_leg_0")
    leg_slide_1 = object_model.get_articulation("platform_to_leg_1")

    for i in range(4):
        ctx.allow_overlap(
            lip,
            platform,
            elem_a=f"lip_hinge_knuckle_{i}",
            elem_b="front_hinge_pin",
            reason="The steel hinge pin intentionally passes through the lip knuckle bore.",
        )
        ctx.expect_within(
            platform,
            lip,
            axes="xz",
            inner_elem="front_hinge_pin",
            outer_elem=f"lip_hinge_knuckle_{i}",
            margin=0.0,
            name=f"front hinge pin is inside lip knuckle {i}",
        )
        ctx.expect_overlap(
            lip,
            platform,
            axes="y",
            min_overlap=0.10,
            elem_a=f"lip_hinge_knuckle_{i}",
            elem_b="front_hinge_pin",
            name=f"front hinge pin spans lip knuckle {i}",
        )

    ctx.check(
        "wide ribbed platform has raised ribs",
        len([v for v in platform.visuals if v.name and v.name.startswith("deck_rib_")]) >= 8,
        details="The platform should read as a broad ribbed steel deck.",
    )
    ctx.check(
        "primary mechanisms are articulated",
        dock_hinge.articulation_type == ArticulationType.REVOLUTE
        and lip_hinge.articulation_type == ArticulationType.REVOLUTE
        and leg_slide_0.articulation_type == ArticulationType.PRISMATIC
        and leg_slide_1.articulation_type == ArticulationType.PRISMATIC,
        details="Expected rear platform hinge, front lip hinge, and two telescoping prismatic legs.",
    )
    ctx.expect_gap(
        platform,
        dock,
        axis="x",
        min_gap=0.08,
        max_gap=0.18,
        positive_elem="deck_plate",
        negative_elem="dock_face_panel",
        name="deck clears the vertical dock face",
    )
    ctx.expect_gap(
        lip,
        platform,
        axis="x",
        min_gap=0.05,
        max_gap=0.20,
        positive_elem="lip_plate",
        negative_elem="deck_plate",
        name="approach lip plate sits forward of platform",
    )
    for suffix, leg in (("0", leg_0), ("1", leg_1)):
        ctx.expect_within(
            leg,
            platform,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="deck_plate",
            margin=0.0,
            name=f"support leg {suffix} sits below deck footprint",
        )
        ctx.expect_overlap(
            leg,
            platform,
            axes="z",
            min_overlap=0.25,
            elem_a="inner_tube",
            elem_b=f"leg_{suffix}_sleeve_front",
            name=f"support leg {suffix} remains inserted in sleeve",
        )

    rest_deck_aabb = ctx.part_element_world_aabb(platform, elem="deck_plate")
    with ctx.pose({dock_hinge: 0.30}):
        raised_deck_aabb = ctx.part_element_world_aabb(platform, elem="deck_plate")
    ctx.check(
        "platform hinge raises front edge",
        rest_deck_aabb is not None
        and raised_deck_aabb is not None
        and raised_deck_aabb[1][2] > rest_deck_aabb[1][2] + 0.45,
        details=f"rest={rest_deck_aabb}, raised={raised_deck_aabb}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_plate")
    with ctx.pose({lip_hinge: 0.60}):
        lowered_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_plate")
    ctx.check(
        "approach lip hinges downward",
        rest_lip_aabb is not None
        and lowered_lip_aabb is not None
        and lowered_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.25,
        details=f"rest={rest_lip_aabb}, lowered={lowered_lip_aabb}",
    )

    rest_leg_pos = ctx.part_world_position(leg_0)
    with ctx.pose({leg_slide_0: 0.25, leg_slide_1: 0.25}):
        extended_leg_pos = ctx.part_world_position(leg_0)
        ctx.expect_overlap(
            leg_0,
            platform,
            axes="z",
            min_overlap=0.08,
            elem_a="inner_tube",
            elem_b="leg_0_sleeve_front",
            name="extended leg retains sleeve insertion",
        )
    ctx.check(
        "prismatic legs extend downward",
        rest_leg_pos is not None
        and extended_leg_pos is not None
        and extended_leg_pos[2] < rest_leg_pos[2] - 0.20,
        details=f"rest={rest_leg_pos}, extended={extended_leg_pos}",
    )

    return ctx.report()


object_model = build_object_model()
