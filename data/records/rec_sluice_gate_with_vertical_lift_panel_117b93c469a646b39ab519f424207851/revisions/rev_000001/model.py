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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canal_sluice_assembly")

    concrete = model.material("aged_concrete", rgba=(0.56, 0.55, 0.50, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.46, 0.50, 0.52, 1.0))
    dark_steel = model.material("dark_oxide_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    gate_blue = model.material("weathered_blue_gate", rgba=(0.05, 0.18, 0.32, 1.0))
    grease_black = model.material("grease_black", rgba=(0.015, 0.014, 0.012, 1.0))
    water_shadow = model.material("shadowed_channel", rgba=(0.02, 0.04, 0.055, 1.0))
    warning_yellow = model.material("worn_yellow", rgba=(0.75, 0.56, 0.10, 1.0))

    frame = model.part("frame")

    # Civil-scale concrete sill and side abutments around the channel mouth.
    frame.visual(
        Box((3.20, 0.50, 0.16)),
        origin=Origin(xyz=(0.0, 0.12, 0.08)),
        material=concrete,
        name="base_sill",
    )
    for x in (-1.45, 1.45):
        frame.visual(
            Box((0.28, 0.50, 1.90)),
            origin=Origin(xyz=(x, 0.12, 0.95)),
            material=concrete,
            name=f"side_pier_{0 if x < 0 else 1}",
        )
    frame.visual(
        Box((3.20, 0.50, 0.24)),
        origin=Origin(xyz=(0.0, 0.32, 1.86)),
        material=concrete,
        name="top_lintel",
    )
    frame.visual(
        Box((2.30, 0.035, 1.25)),
        origin=Origin(xyz=(0.0, 0.36, 0.78)),
        material=water_shadow,
        name="channel_shadow",
    )

    # Steel guide channels flanking the sliding gate leaf.
    for x in (-1.12, 1.12):
        frame.visual(
            Box((0.08, 0.10, 1.82)),
            origin=Origin(xyz=(x, -0.04, 0.91)),
            material=dark_steel,
            name=f"guide_channel_{0 if x < 0 else 1}",
        )
        frame.visual(
            Box((0.13, 0.07, 0.16)),
            origin=Origin(xyz=(x, -0.02, 0.22)),
            material=dark_steel,
            name=f"guide_foot_{0 if x < 0 else 1}",
        )

    # Gantry and drive head above the opening, built from simple structural
    # members with a clear central passage for the rising stem.
    for x in (-0.56, 0.56):
        frame.visual(
            Box((0.14, 0.22, 0.94)),
            origin=Origin(xyz=(x, 0.16, 2.33)),
            material=galvanized,
            name=f"drive_post_{0 if x < 0 else 1}",
        )
    frame.visual(
        Box((1.28, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, 0.16, 2.75)),
        material=galvanized,
        name="drive_base_beam",
    )
    for x in (-0.28, 0.28):
        frame.visual(
            Box((0.10, 0.46, 0.30)),
            origin=Origin(xyz=(x, 0.0, 2.94)),
            material=galvanized,
            name=f"drive_cheek_{0 if x < 0 else 1}",
        )
    for y in (-0.23, 0.23):
        frame.visual(
            Box((0.62, 0.08, 0.30)),
            origin=Origin(xyz=(0.0, y, 2.94)),
            material=galvanized,
            name=f"drive_crossbar_{0 if y < 0 else 1}",
        )

    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.105, tube=0.026, radial_segments=24, tubular_segments=48),
        "stem_bearing_ring",
    )
    frame.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 3.08)),
        material=dark_steel,
        name="bearing_ring",
    )
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        frame.visual(
            Box((0.18, 0.045, 0.055)),
            origin=Origin(
                xyz=(0.15 * math.cos(angle), 0.15 * math.sin(angle), 3.08),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"bearing_spoke_{int(round(angle * 100))}",
        )

    # Fixed hinge hardware for the narrow inspection hatch on the drive head.
    frame.visual(
        Box((0.050, 0.070, 0.42)),
        origin=Origin(xyz=(-0.22, -0.255, 2.94)),
        material=dark_steel,
        name="hatch_hinge_leaf",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.46),
        origin=Origin(xyz=(-0.22, -0.285, 2.94)),
        material=dark_steel,
        name="hatch_hinge_pin",
    )

    gate_panel = model.part("gate_panel")
    gate_panel.visual(
        Box((2.05, 0.12, 1.45)),
        origin=Origin(xyz=(0.0, -0.04, 0.885)),
        material=gate_blue,
        name="gate_leaf",
    )
    for z in (0.34, 0.80, 1.26):
        gate_panel.visual(
            Box((2.12, 0.08, 0.08)),
            origin=Origin(xyz=(0.0, -0.115, z)),
            material=dark_steel,
            name=f"horizontal_rib_{int(z * 100)}",
        )
    for x in (-0.54, 0.54):
        gate_panel.visual(
            Box((0.08, 0.075, 1.36)),
            origin=Origin(xyz=(x, -0.115, 0.91)),
            material=dark_steel,
            name=f"vertical_rib_{0 if x < 0 else 1}",
        )
    gate_panel.visual(
        Box((0.42, 0.16, 0.16)),
        origin=Origin(xyz=(0.0, -0.035, 1.52)),
        material=dark_steel,
        name="stem_yoke",
    )
    gate_panel.visual(
        Cylinder(radius=0.045, length=2.10),
        origin=Origin(xyz=(0.0, 0.0, 2.50)),
        material=dark_steel,
        name="rising_stem",
    )
    gate_panel.visual(
        Cylinder(radius=0.070, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
        material=dark_steel,
        name="stem_collar",
    )

    model.articulation(
        "frame_to_gate_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=0.12, lower=0.0, upper=0.90),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.32, tube=0.030, radial_segments=24, tubular_segments=72),
            "drive_handwheel_rim",
        ),
        material=warning_yellow,
        name="wheel_rim",
    )
    handwheel.visual(
        Cylinder(radius=0.080, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_hub",
    )
    for i in range(6):
        angle = i * math.tau / 6.0
        handwheel.visual(
            Box((0.53, 0.035, 0.035)),
            origin=Origin(
                xyz=(0.14 * math.cos(angle), 0.14 * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=warning_yellow,
            name=f"wheel_spoke_{i}",
        )
    handwheel.visual(
        Cylinder(radius=0.060, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=dark_steel,
        name="top_nut",
    )

    model.articulation(
        "gate_panel_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=gate_panel,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.0, 3.63)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=3.5),
    )

    service_hatch = model.part("service_hatch")
    service_hatch.visual(
        Box((0.36, 0.030, 0.36)),
        origin=Origin(xyz=(0.205, -0.050, 0.0)),
        material=galvanized,
        name="hatch_panel",
    )
    service_hatch.visual(
        Box((0.055, 0.022, 0.32)),
        origin=Origin(xyz=(0.040, -0.0265, 0.0)),
        material=dark_steel,
        name="hatch_leaf",
    )
    service_hatch.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(0.320, -0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grease_black,
        name="hatch_pull",
    )

    model.articulation(
        "frame_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=service_hatch,
        origin=Origin(xyz=(-0.22, -0.285, 2.94)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate_panel = object_model.get_part("gate_panel")
    handwheel = object_model.get_part("handwheel")
    service_hatch = object_model.get_part("service_hatch")

    gate_slide = object_model.get_articulation("frame_to_gate_panel")
    wheel_spin = object_model.get_articulation("gate_panel_to_handwheel")
    hatch_hinge = object_model.get_articulation("frame_to_service_hatch")

    ctx.allow_overlap(
        frame,
        service_hatch,
        elem_a="hatch_hinge_pin",
        elem_b="hatch_leaf",
        reason=(
            "The hatch hinge leaf is intentionally captured against the fixed hinge pin with a "
            "tiny hidden interference so the service hatch is physically retained."
        ),
    )
    ctx.expect_gap(
        frame,
        service_hatch,
        axis="y",
        positive_elem="hatch_hinge_pin",
        negative_elem="hatch_leaf",
        max_gap=0.001,
        max_penetration=0.005,
        name="hatch hinge leaf is retained on the pin",
    )

    ctx.expect_gap(
        gate_panel,
        frame,
        axis="z",
        positive_elem="gate_leaf",
        negative_elem="base_sill",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed gate leaf seats on the sill",
    )
    ctx.expect_overlap(
        gate_panel,
        frame,
        axes="xz",
        elem_a="gate_leaf",
        elem_b="channel_shadow",
        min_overlap=1.20,
        name="gate leaf covers the channel mouth",
    )
    ctx.expect_within(
        gate_panel,
        frame,
        axes="xy",
        inner_elem="rising_stem",
        outer_elem="bearing_ring",
        margin=0.0,
        name="rising stem is centered in the drive-head bearing",
    )
    ctx.expect_overlap(
        gate_panel,
        frame,
        axes="z",
        elem_a="rising_stem",
        elem_b="bearing_ring",
        min_overlap=0.045,
        name="rising stem passes through the drive head",
    )
    ctx.expect_gap(
        handwheel,
        gate_panel,
        axis="z",
        positive_elem="wheel_hub",
        negative_elem="rising_stem",
        max_gap=0.002,
        max_penetration=0.002,
        name="handwheel hub sits on the stem top",
    )

    rest_gate_pos = ctx.part_world_position(gate_panel)
    with ctx.pose({gate_slide: 0.90}):
        raised_gate_pos = ctx.part_world_position(gate_panel)
        ctx.expect_within(
            gate_panel,
            frame,
            axes="xy",
            inner_elem="rising_stem",
            outer_elem="bearing_ring",
            margin=0.0,
            name="raised stem remains in the bearing",
        )
        ctx.expect_overlap(
            gate_panel,
            frame,
            axes="z",
            elem_a="rising_stem",
            elem_b="bearing_ring",
            min_overlap=0.045,
            name="raised stem still passes through the drive head",
        )
    ctx.check(
        "gate panel translates upward",
        rest_gate_pos is not None
        and raised_gate_pos is not None
        and raised_gate_pos[2] > rest_gate_pos[2] + 0.85,
        details=f"rest={rest_gate_pos}, raised={raised_gate_pos}",
    )

    rest_wheel_pos = ctx.part_world_position(handwheel)
    with ctx.pose({wheel_spin: 2.4}):
        spun_wheel_pos = ctx.part_world_position(handwheel)
    ctx.check(
        "handwheel spins about its stem axis",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and abs(spun_wheel_pos[0] - rest_wheel_pos[0]) < 1e-6
        and abs(spun_wheel_pos[1] - rest_wheel_pos[1]) < 1e-6
        and abs(spun_wheel_pos[2] - rest_wheel_pos[2]) < 1e-6,
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    rest_hatch_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_panel")
    with ctx.pose({hatch_hinge: 1.25}):
        open_hatch_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_panel")
    ctx.check(
        "service hatch swings outward on a vertical hinge",
        rest_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[0][1] < rest_hatch_aabb[0][1] - 0.10,
        details=f"rest_aabb={rest_hatch_aabb}, open_aabb={open_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
