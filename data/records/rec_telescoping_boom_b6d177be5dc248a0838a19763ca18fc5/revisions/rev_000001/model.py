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


MEDIUM_TRAVEL = 0.38
TERMINAL_TRAVEL = 0.30


def _add_xz_bar(part, name, x0, z0, x1, z1, *, y, thickness, material) -> None:
    """Add a square tube whose local +X axis runs between two XZ points."""
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    pitch = -math.atan2(dz, dx)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, y, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_support_boom")

    dark_anodized = model.material("dark_anodized_aluminum", rgba=(0.06, 0.07, 0.08, 1.0))
    graphite = model.material("graphite_sleeve", rgba=(0.17, 0.18, 0.19, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    machined = model.material("machined_plate", rgba=(0.55, 0.57, 0.58, 1.0))

    root = model.part("root_housing")
    # Large rear mounting plate and open trussed cage around the first slide.
    root.visual(
        Box((0.08, 0.44, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_anodized,
        name="rear_bulkhead",
    )
    for y, suffix in ((0.17, "upper_side"), (-0.17, "lower_side")):
        root.visual(
            Box((0.09, 0.04, 0.29)),
            origin=Origin(xyz=(0.58, y, 0.0)),
            material=dark_anodized,
            name=f"front_post_{suffix}",
        )

    root.visual(
        Box((0.10, 0.31, 0.035)),
        origin=Origin(xyz=(0.58, 0.0, 0.103)),
        material=dark_anodized,
        name="front_collar_top",
    )
    root.visual(
        Box((0.10, 0.31, 0.035)),
        origin=Origin(xyz=(0.58, 0.0, -0.103)),
        material=dark_anodized,
        name="front_collar_bottom",
    )
    root.visual(
        Box((0.08, 0.40, 0.035)),
        origin=Origin(xyz=(0.02, 0.0, 0.145)),
        material=dark_anodized,
        name="rear_top_tie",
    )
    root.visual(
        Box((0.08, 0.40, 0.035)),
        origin=Origin(xyz=(0.02, 0.0, -0.145)),
        material=dark_anodized,
        name="rear_bottom_tie",
    )
    for y, side in ((0.17, "side_0"), (-0.17, "side_1")):
        _add_xz_bar(root, f"{side}_top_longeron", 0.02, 0.145, 0.60, 0.105, y=y, thickness=0.032, material=dark_anodized)
        _add_xz_bar(root, f"{side}_bottom_longeron", 0.02, -0.145, 0.60, -0.105, y=y, thickness=0.032, material=dark_anodized)
        _add_xz_bar(root, f"{side}_rising_brace", 0.02, -0.130, 0.58, 0.095, y=y, thickness=0.026, material=dark_anodized)
        _add_xz_bar(root, f"{side}_falling_brace", 0.02, 0.130, 0.58, -0.095, y=y, thickness=0.026, material=dark_anodized)

    medium = model.part("medium_stage")
    # A medium rectangular tube: four walls leave a visible clear bore for the terminal stage.
    medium.visual(
        Box((1.50, 0.16, 0.020)),
        origin=Origin(xyz=(0.20, 0.0, 0.060)),
        material=graphite,
        name="medium_top_wall",
    )
    medium.visual(
        Box((1.50, 0.16, 0.020)),
        origin=Origin(xyz=(0.20, 0.0, -0.060)),
        material=graphite,
        name="medium_bottom_wall",
    )
    medium.visual(
        Box((1.50, 0.020, 0.120)),
        origin=Origin(xyz=(0.20, 0.075, 0.0)),
        material=graphite,
        name="medium_side_wall_0",
    )
    medium.visual(
        Box((1.50, 0.020, 0.120)),
        origin=Origin(xyz=(0.20, -0.075, 0.0)),
        material=graphite,
        name="medium_side_wall_1",
    )
    medium.visual(
        Box((0.070, 0.10, 0.016)),
        origin=Origin(xyz=(-0.04, 0.0, 0.0775)),
        material=black,
        name="upper_slide_shoe",
    )
    medium.visual(
        Box((0.070, 0.10, 0.016)),
        origin=Origin(xyz=(-0.04, 0.0, -0.0775)),
        material=black,
        name="lower_slide_shoe",
    )
    medium.visual(
        Box((0.045, 0.18, 0.025)),
        origin=Origin(xyz=(0.94, 0.0, 0.0675)),
        material=dark_anodized,
        name="medium_front_band_top",
    )
    medium.visual(
        Box((0.045, 0.18, 0.025)),
        origin=Origin(xyz=(0.94, 0.0, -0.0675)),
        material=dark_anodized,
        name="medium_front_band_bottom",
    )
    medium.visual(
        Box((0.045, 0.025, 0.14)),
        origin=Origin(xyz=(0.94, 0.0775, 0.0)),
        material=dark_anodized,
        name="medium_front_band_0",
    )
    medium.visual(
        Box((0.045, 0.025, 0.14)),
        origin=Origin(xyz=(0.94, -0.0775, 0.0)),
        material=dark_anodized,
        name="medium_front_band_1",
    )

    terminal = model.part("terminal_stage")
    terminal.visual(
        Box((1.11, 0.065, 0.045)),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=dark_anodized,
        name="terminal_bar",
    )
    terminal.visual(
        Box((0.16, 0.045, 0.030)),
        origin=Origin(xyz=(-0.32, 0.0, 0.035)),
        material=black,
        name="terminal_upper_runner",
    )
    terminal.visual(
        Box((0.16, 0.045, 0.030)),
        origin=Origin(xyz=(-0.32, 0.0, -0.035)),
        material=black,
        name="terminal_lower_runner",
    )
    terminal.visual(
        Box((0.035, 0.16, 0.12)),
        origin=Origin(xyz=(0.775, 0.0, 0.0)),
        material=machined,
        name="face_plate",
    )
    terminal.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.796, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="camera_boss",
    )
    for y in (-0.048, 0.048):
        for z in (-0.036, 0.036):
            terminal.visual(
                Box((0.010, 0.026, 0.018)),
                origin=Origin(xyz=(0.795, y, z)),
                material=black,
                name=f"rubber_pad_{'p' if y > 0 else 'n'}_{'p' if z > 0 else 'n'}",
            )

    model.articulation(
        "root_to_medium",
        ArticulationType.PRISMATIC,
        parent=root,
        child=medium,
        origin=Origin(xyz=(0.60, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=MEDIUM_TRAVEL),
    )
    model.articulation(
        "medium_to_terminal",
        ArticulationType.PRISMATIC,
        parent=medium,
        child=terminal,
        origin=Origin(xyz=(0.93, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.16, lower=0.0, upper=TERMINAL_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_housing")
    medium = object_model.get_part("medium_stage")
    terminal = object_model.get_part("terminal_stage")
    root_slide = object_model.get_articulation("root_to_medium")
    terminal_slide = object_model.get_articulation("medium_to_terminal")

    ctx.expect_within(
        medium,
        root,
        axes="yz",
        margin=0.002,
        name="medium stage fits within root housing bore",
    )
    ctx.expect_overlap(
        medium,
        root,
        axes="x",
        min_overlap=0.45,
        name="medium stage is deeply retained in root housing",
    )
    ctx.expect_within(
        terminal,
        medium,
        axes="yz",
        inner_elem="terminal_bar",
        margin=0.002,
        name="terminal bar fits through medium tube bore",
    )
    ctx.expect_overlap(
        terminal,
        medium,
        axes="x",
        elem_a="terminal_bar",
        min_overlap=0.30,
        name="terminal stage is retained inside medium tube",
    )

    medium_rest = ctx.part_world_position(medium)
    with ctx.pose({root_slide: MEDIUM_TRAVEL}):
        ctx.expect_within(
            medium,
            root,
            axes="yz",
            margin=0.002,
            name="extended medium stays centered in root housing",
        )
        ctx.expect_overlap(
            medium,
            root,
            axes="x",
            min_overlap=0.15,
            name="extended medium remains inserted in root housing",
        )
        medium_extended = ctx.part_world_position(medium)
    ctx.check(
        "medium slide extends away from base",
        medium_rest is not None and medium_extended is not None and medium_extended[0] > medium_rest[0] + 0.30,
        details=f"rest={medium_rest}, extended={medium_extended}",
    )

    terminal_rest = ctx.part_world_position(terminal)
    with ctx.pose({terminal_slide: TERMINAL_TRAVEL}):
        ctx.expect_within(
            terminal,
            medium,
            axes="yz",
            inner_elem="terminal_bar",
            margin=0.002,
            name="extended terminal stays centered in medium tube",
        )
        ctx.expect_overlap(
            terminal,
            medium,
            axes="x",
            elem_a="terminal_bar",
            min_overlap=0.06,
            name="extended terminal remains inserted in medium tube",
        )
        terminal_extended = ctx.part_world_position(terminal)
    ctx.check(
        "terminal slide extends away from base",
        terminal_rest is not None and terminal_extended is not None and terminal_extended[0] > terminal_rest[0] + 0.20,
        details=f"rest={terminal_rest}, extended={terminal_extended}",
    )

    return ctx.report()


object_model = build_object_model()
