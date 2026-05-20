from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_assembly")

    weathered_steel = model.material("weathered_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    coated_gate = model.material("coated_gate", rgba=(0.18, 0.33, 0.48, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.20, 1.0))
    wheel_red = model.material("wheel_red", rgba=(0.60, 0.16, 0.14, 1.0))

    opening_w = 1.32
    opening_h = 1.50
    frame_depth = 0.26
    post_w = 0.16
    sill_h = 0.18
    post_h = 2.65
    guide_depth = 0.05
    guide_width = 0.05
    guide_h = 1.95
    lintel_depth = 0.05
    deck_depth = 0.05
    lintel_h = 0.12
    deck_h = 0.08
    head_w = 0.56
    head_depth = 0.22
    head_h = 0.38
    cheek_w = 0.10
    head_base_z = sill_h + guide_h + deck_h
    total_w = opening_w + 2.0 * post_w
    guide_x = opening_w / 2.0 - guide_width / 2.0 + 0.005
    guide_y = 0.101

    frame = model.part("frame")
    frame.visual(
        Box((total_w, frame_depth, sill_h)),
        origin=Origin(xyz=(0.0, 0.0, sill_h / 2.0)),
        material=weathered_steel,
        name="sill",
    )
    for idx, x_sign in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((post_w, frame_depth, post_h)),
            origin=Origin(xyz=(x_sign * (opening_w / 2.0 + post_w / 2.0), 0.0, post_h / 2.0)),
            material=weathered_steel,
            name=f"side_post_{idx}",
        )
    for prefix, y_sign, z_center in (
        ("lintel", 1.0, sill_h + opening_h + lintel_h / 2.0),
        ("lintel", -1.0, sill_h + opening_h + lintel_h / 2.0),
        ("deck", 1.0, sill_h + guide_h + deck_h / 2.0),
        ("deck", -1.0, sill_h + guide_h + deck_h / 2.0),
    ):
        depth = lintel_depth if prefix == "lintel" else deck_depth
        height = lintel_h if prefix == "lintel" else deck_h
        frame.visual(
            Box((total_w, depth, height)),
            origin=Origin(xyz=(0.0, y_sign * (frame_depth / 2.0 - depth / 2.0), z_center)),
            material=weathered_steel,
            name=f"{prefix}_{0 if y_sign > 0 else 1}",
        )
    for idx, y_sign in enumerate((1.0, -1.0)):
        for x_sign in (-1.0, 1.0):
            frame.visual(
                Box((guide_width, guide_depth, guide_h)),
                origin=Origin(
                    xyz=(x_sign * guide_x, y_sign * guide_y, sill_h + guide_h / 2.0),
                ),
                material=weathered_steel,
                name=f"guide_{idx * 2 + (0 if x_sign < 0 else 1)}",
            )
    for idx, x_sign in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((cheek_w, head_depth, head_h)),
            origin=Origin(
                xyz=(
                    x_sign * (head_w / 2.0 - cheek_w / 2.0),
                    0.0,
                    head_base_z + head_h / 2.0,
                )
            ),
            material=weathered_steel,
            name=f"head_cheek_{idx}",
        )
    frame.visual(
        Box((head_w, 0.04, head_h - 0.08)),
        origin=Origin(xyz=(0.0, -(head_depth / 2.0 - 0.02), head_base_z + (head_h - 0.08) / 2.0)),
        material=weathered_steel,
        name="head_back",
    )
    head_top_z = head_base_z + head_h + 0.03
    frame.visual(
        Box((head_w, 0.055, 0.06)),
        origin=Origin(xyz=(0.0, head_depth / 2.0 - 0.0275, head_top_z)),
        material=weathered_steel,
        name="head_top",
    )
    frame.visual(
        Box((head_w, 0.055, 0.06)),
        origin=Origin(xyz=(0.0, -(head_depth / 2.0 - 0.0275), head_top_z)),
        material=weathered_steel,
        name="head_top_rear",
    )
    for idx, x_sign in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.15, 0.11, 0.06)),
            origin=Origin(xyz=(x_sign * 0.205, 0.0, head_top_z)),
            material=weathered_steel,
            name=f"head_top_side_{idx}",
        )
    frame.visual(
        Box((0.03, 0.04, head_h)),
        origin=Origin(xyz=(0.055, head_depth / 2.0 - 0.02, head_base_z + head_h / 2.0)),
        material=weathered_steel,
        name="hatch_jamb",
    )

    panel_w = 1.26
    panel_t = 0.074
    panel_h = 1.50
    stem_r = 0.035
    stem_len = 1.05
    panel = model.part("panel")
    panel.visual(
        Box((panel_w, panel_t, panel_h)),
        origin=Origin(xyz=(0.0, 0.0, panel_h / 2.0)),
        material=coated_gate,
        name="panel_plate",
    )
    for idx, x in enumerate((-0.34, 0.34)):
        panel.visual(
            Box((0.10, 0.03, 1.18)),
            origin=Origin(xyz=(x, panel_t / 2.0 + 0.015, 0.72)),
            material=coated_gate,
            name=f"panel_rib_{idx}",
        )
    panel.visual(
        Box((0.90, 0.03, 0.10)),
        origin=Origin(xyz=(0.0, panel_t / 2.0 + 0.015, 0.94)),
        material=coated_gate,
        name="panel_crossbar",
    )
    panel.visual(
        Box((0.18, 0.11, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, panel_h + 0.07)),
        material=coated_gate,
        name="stem_collar",
    )
    panel.visual(
        Cylinder(radius=stem_r, length=stem_len),
        origin=Origin(xyz=(0.0, 0.0, panel_h + stem_len / 2.0)),
        material=dark_metal,
        name="stem",
    )

    wheel_radius = 0.235
    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.05, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="hub",
    )
    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        wheel.visual(
            Box((0.18, 0.024, 0.024)),
            origin=Origin(
                xyz=(0.13 * math.cos(angle), 0.13 * math.sin(angle), 0.03),
                rpy=(0.0, 0.0, angle),
            ),
            material=wheel_red,
            name=f"spoke_{idx}",
        )
    for idx in range(8):
        angle = idx * (math.pi / 4.0)
        wheel.visual(
            Box((0.20, 0.03, 0.03)),
            origin=Origin(
                xyz=(wheel_radius * math.cos(angle), wheel_radius * math.sin(angle), 0.03),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=wheel_red,
            name=f"rim_{idx}",
        )
    wheel.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(xyz=(wheel_radius, 0.0, 0.095)),
        material=dark_metal,
        name="knob",
    )

    hatch_w = 0.14
    hatch_t = 0.018
    hatch_h = 0.30
    hatch = model.part("hatch")
    hatch.visual(
        Box((hatch_w, hatch_t, hatch_h)),
        origin=Origin(xyz=(hatch_w / 2.0, hatch_t / 2.0, hatch_h / 2.0)),
        material=weathered_steel,
        name="hatch_panel",
    )
    hatch.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(
            xyz=(0.105, hatch_t + 0.0175, hatch_h / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="hatch_handle",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, sill_h)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.18, lower=0.0, upper=0.72),
    )
    model.articulation(
        "panel_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, panel_h + stem_len)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=4.0),
    )
    model.articulation(
        "frame_to_hatch",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hatch,
        origin=Origin(xyz=(0.07, head_depth / 2.0, head_base_z + 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    wheel = object_model.get_part("wheel")
    hatch = object_model.get_part("hatch")

    panel_slide = object_model.get_articulation("frame_to_panel")
    wheel_spin = object_model.get_articulation("panel_to_wheel")
    hatch_hinge = object_model.get_articulation("frame_to_hatch")

    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="panel_plate",
        negative_elem="sill",
        max_gap=0.002,
        max_penetration=1e-5,
        name="panel seats on the sill in the closed pose",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem="hub",
        negative_elem="head_top",
        min_gap=0.05,
        max_gap=0.16,
        name="handwheel sits just above the drive head",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="x",
        inner_elem="hub",
        outer_elem="head_top",
        margin=0.02,
        name="wheel hub stays centered over the drive head",
    )

    rest_panel_pos = ctx.part_world_position(panel)
    rest_wheel_pos = ctx.part_world_position(wheel)
    rest_knob_aabb = ctx.part_element_world_aabb(wheel, elem="knob")
    closed_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")

    with ctx.pose({panel_slide: panel_slide.motion_limits.upper}):
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="panel_plate",
            negative_elem="sill",
            min_gap=0.70,
            name="panel lifts well clear of the sill when opened",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="panel_plate",
            elem_b="guide_0",
            min_overlap=1.10,
            name="panel remains engaged in the guides at full lift",
        )
        raised_panel_pos = ctx.part_world_position(panel)
        raised_wheel_pos = ctx.part_world_position(wheel)

    ctx.check(
        "panel travel moves upward",
        rest_panel_pos is not None
        and raised_panel_pos is not None
        and raised_panel_pos[2] > rest_panel_pos[2] + 0.70,
        details=f"rest={rest_panel_pos}, raised={raised_panel_pos}",
    )
    ctx.check(
        "rising stem lifts the handwheel with the panel",
        rest_wheel_pos is not None
        and raised_wheel_pos is not None
        and raised_wheel_pos[2] > rest_wheel_pos[2] + 0.70,
        details=f"rest={rest_wheel_pos}, raised={raised_wheel_pos}",
    )

    with ctx.pose({wheel_spin: math.pi / 2.0}):
        turned_knob_aabb = ctx.part_element_world_aabb(wheel, elem="knob")

    ctx.check(
        "handwheel rotation carries the grip around the rim",
        rest_knob_aabb is not None
        and turned_knob_aabb is not None
        and turned_knob_aabb[1][1] > rest_knob_aabb[1][1] + 0.18,
        details=f"rest={rest_knob_aabb}, turned={turned_knob_aabb}",
    )

    with ctx.pose({hatch_hinge: 1.10}):
        open_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")

    ctx.check(
        "service hatch swings outward from the drive head",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][1] > closed_hatch_aabb[1][1] + 0.10,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
