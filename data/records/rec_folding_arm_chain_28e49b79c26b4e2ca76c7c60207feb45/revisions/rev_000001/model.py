from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(length: float, height: float, segments: int = 16) -> list[tuple[float, float]]:
    """Rounded bar outline in the local X/Z plane before extrusion through Y."""
    radius = height / 2.0
    pts: list[tuple[float, float]] = []
    for i in range(segments + 1):
        angle = -math.pi / 2.0 + math.pi * i / segments
        pts.append((length + radius * math.cos(angle), radius * math.sin(angle)))
    for i in range(segments + 1):
        angle = math.pi / 2.0 + math.pi * i / segments
        pts.append((radius * math.cos(angle), radius * math.sin(angle)))
    return pts


def _rounded_link_plate(
    length: float,
    height: float,
    thickness: float,
    hole_centers: tuple[float, ...],
    hole_radius: float,
):
    holes = [_circle_profile(x, 0.0, hole_radius, 28) for x in hole_centers]
    geom = ExtrudeWithHolesGeometry(
        _capsule_profile(length, height, 18),
        holes,
        thickness,
        center=True,
    )
    return geom.rotate_x(-math.pi / 2.0)


def _y_cylinder(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hatch_support_arm")

    zinc = model.material("brushed_zinc", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_bushed_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    oxide = model.material("black_oxide_fasteners", rgba=(0.02, 0.022, 0.020, 1.0))
    pad_finish = model.material("slightly_worn_pad", rgba=(0.48, 0.50, 0.47, 1.0))

    first_len = 0.225
    center_len = 0.520
    terminal_len = 0.200
    pin_origin = _y_cylinder(0.0085, 0.096)[1]

    base = model.part("base_lug")
    base.visual(
        Box((0.190, 0.105, 0.012)),
        origin=Origin(xyz=(-0.045, 0.0, -0.064)),
        material=zinc,
        name="mounting_foot",
    )
    base.visual(
        Box((0.048, 0.008, 0.086)),
        origin=Origin(xyz=(0.000, 0.039, -0.014)),
        material=zinc,
        name="clevis_cheek_0",
    )
    base.visual(
        Box((0.048, 0.008, 0.086)),
        origin=Origin(xyz=(0.000, -0.039, -0.014)),
        material=zinc,
        name="clevis_cheek_1",
    )
    base.visual(
        Box((0.024, 0.094, 0.018)),
        origin=Origin(xyz=(-0.024, 0.0, -0.049)),
        material=zinc,
        name="clevis_bridge",
    )
    base.visual(
        Box((0.028, 0.072, 0.030)),
        origin=Origin(xyz=(0.054, 0.0, -0.049)),
        material=dark_steel,
        name="lower_stop_tab",
    )
    pin_geom, pin_rot = _y_cylinder(0.0085, 0.096)
    base.visual(
        pin_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=pin_rot.rpy),
        material=oxide,
        name="base_pin",
    )
    bolt_geom, bolt_rot = _y_cylinder(0.006, 0.010)
    for i, x in enumerate((-0.102, 0.012)):
        for j, y in enumerate((-0.032, 0.032)):
            base.visual(
                bolt_geom,
                origin=Origin(xyz=(x, y, -0.055), rpy=bolt_rot.rpy),
                material=oxide,
                name=f"foot_bolt_{i}_{j}",
            )

    first = model.part("first_link")
    first_plate_mesh = mesh_from_geometry(
        _rounded_link_plate(first_len, 0.038, 0.0045, (0.0, first_len), 0.0115),
        "first_link_side_plate",
    )
    for suffix, y in (("0", 0.023), ("1", -0.023)):
        first.visual(
            first_plate_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=zinc,
            name=f"side_plate_{suffix}",
        )
    first.visual(
        Box((0.036, 0.042, 0.012)),
        origin=Origin(xyz=(first_len * 0.48, 0.0, -0.019)),
        material=zinc,
        name="pressed_spacer_web",
    )
    bushing_geom, bushing_rot = _y_cylinder(0.0135, 0.054)
    first.visual(
        bushing_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=bushing_rot.rpy),
        material=dark_steel,
        name="root_bushing",
    )
    spacer_geom, spacer_rot = _y_cylinder(0.0105, 0.010)
    first.visual(
        spacer_geom,
        origin=Origin(xyz=(first_len, 0.023, 0.0), rpy=spacer_rot.rpy),
        material=dark_steel,
        name="distal_spacer_0",
    )
    first.visual(
        spacer_geom,
        origin=Origin(xyz=(first_len, -0.023, 0.0), rpy=spacer_rot.rpy),
        material=dark_steel,
        name="distal_spacer_1",
    )
    first.visual(
        pin_geom,
        origin=Origin(xyz=(first_len, 0.0, 0.0), rpy=pin_origin.rpy),
        material=oxide,
        name="distal_pin",
    )
    first.visual(
        Box((0.018, 0.044, 0.010)),
        origin=Origin(xyz=(first_len - 0.032, 0.0, -0.018)),
        material=dark_steel,
        name="fold_stop_tab",
    )

    center = model.part("center_link")
    center_main_mesh = mesh_from_geometry(
        _rounded_link_plate(center_len - 0.090, 0.032, 0.008, (), 0.0115),
        "center_link_main_plate",
    )
    center.visual(
        center_main_mesh,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=zinc,
        name="main_web",
    )
    root_bushing_geom, root_bushing_rot = _y_cylinder(0.014, 0.026)
    center.visual(
        root_bushing_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=root_bushing_rot.rpy),
        material=dark_steel,
        name="root_bushing",
    )
    center.visual(
        Box((0.030, 0.008, 0.006)),
        origin=Origin(xyz=(0.016, 0.0, 0.013)),
        material=zinc,
        name="upper_root_neck",
    )
    center.visual(
        Box((0.030, 0.008, 0.006)),
        origin=Origin(xyz=(0.016, 0.0, -0.013)),
        material=zinc,
        name="lower_root_neck",
    )
    center_fork_mesh = mesh_from_geometry(
        _rounded_link_plate(0.110, 0.030, 0.0045, (0.055,), 0.0105),
        "center_distal_fork_plate",
    )
    for suffix, y in (("0", 0.021), ("1", -0.021)):
        center.visual(
            center_fork_mesh,
            origin=Origin(xyz=(center_len - 0.055, y, 0.0)),
            material=zinc,
            name=f"distal_cheek_{suffix}",
        )
    center.visual(
        Box((0.038, 0.052, 0.020)),
        origin=Origin(xyz=(center_len - 0.075, 0.0, -0.003)),
        material=zinc,
        name="formed_fork_bridge",
    )
    center.visual(
        Box((0.020, 0.042, 0.012)),
        origin=Origin(xyz=(center_len - 0.035, 0.0, -0.014)),
        material=dark_steel,
        name="fork_stop_tab",
    )
    small_pin_geom, small_pin_rot = _y_cylinder(0.0075, 0.064)
    center.visual(
        small_pin_geom,
        origin=Origin(xyz=(center_len, 0.0, 0.0), rpy=small_pin_rot.rpy),
        material=oxide,
        name="terminal_pin",
    )

    terminal = model.part("terminal_link")
    terminal_mesh = mesh_from_geometry(
        _rounded_link_plate(terminal_len - 0.030, 0.026, 0.007, (), 0.0105),
        "terminal_narrow_plate",
    )
    terminal.visual(
        terminal_mesh,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=zinc,
        name="narrow_leaf",
    )
    terminal_bushing_geom, terminal_bushing_rot = _y_cylinder(0.0115, 0.020)
    terminal.visual(
        terminal_bushing_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=terminal_bushing_rot.rpy),
        material=dark_steel,
        name="root_bushing",
    )
    terminal.visual(
        Box((0.030, 0.007, 0.006)),
        origin=Origin(xyz=(0.016, 0.0, 0.012)),
        material=zinc,
        name="upper_pivot_neck",
    )
    terminal.visual(
        Box((0.030, 0.007, 0.006)),
        origin=Origin(xyz=(0.016, 0.0, -0.012)),
        material=zinc,
        name="lower_pivot_neck",
    )
    terminal.visual(
        Box((0.026, 0.030, 0.028)),
        origin=Origin(xyz=(terminal_len - 0.008, 0.0, 0.002)),
        material=zinc,
        name="formed_neck",
    )
    terminal.visual(
        Box((0.074, 0.040, 0.010)),
        origin=Origin(xyz=(terminal_len + 0.028, 0.0, 0.016)),
        material=pad_finish,
        name="tip_pad",
    )
    pad_bolt_geom, pad_bolt_rot = _y_cylinder(0.0048, 0.007)
    for i, x in enumerate((terminal_len + 0.006, terminal_len + 0.050)):
        terminal.visual(
            pad_bolt_geom,
            origin=Origin(xyz=(x, 0.023, 0.016), rpy=pad_bolt_rot.rpy),
            material=oxide,
            name=f"pad_bolt_{i}",
        )

    model.articulation(
        "base_to_first",
        ArticulationType.REVOLUTE,
        parent=base,
        child=first,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=0.95),
    )
    model.articulation(
        "first_to_center",
        ArticulationType.REVOLUTE,
        parent=first,
        child=center,
        origin=Origin(xyz=(first_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=38.0, velocity=1.2, lower=0.0, upper=0.72),
    )
    model.articulation(
        "center_to_terminal",
        ArticulationType.REVOLUTE,
        parent=center,
        child=terminal,
        origin=Origin(xyz=(center_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.4, lower=0.0, upper=0.62),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_lug")
    first = object_model.get_part("first_link")
    center = object_model.get_part("center_link")
    terminal = object_model.get_part("terminal_link")
    j0 = object_model.get_articulation("base_to_first")
    j1 = object_model.get_articulation("first_to_center")
    j2 = object_model.get_articulation("center_to_terminal")

    ctx.allow_overlap(
        base,
        first,
        elem_a="base_pin",
        elem_b="root_bushing",
        reason="The black-oxide base pin is intentionally captured inside the first-link bushing.",
    )
    ctx.allow_overlap(
        first,
        center,
        elem_a="distal_pin",
        elem_b="root_bushing",
        reason="The first-link pivot pin is intentionally represented as nested in the center-link bushing.",
    )
    ctx.allow_overlap(
        center,
        terminal,
        elem_a="terminal_pin",
        elem_b="root_bushing",
        reason="The terminal pivot pin is intentionally captured inside the narrow terminal-link bushing.",
    )

    ctx.expect_overlap(
        base,
        first,
        axes="xyz",
        elem_a="base_pin",
        elem_b="root_bushing",
        min_overlap=0.010,
        name="base pivot has captured pin overlap",
    )
    ctx.expect_overlap(
        first,
        center,
        axes="xyz",
        elem_a="distal_pin",
        elem_b="root_bushing",
        min_overlap=0.010,
        name="middle pivot has captured pin overlap",
    )
    ctx.expect_overlap(
        center,
        terminal,
        axes="xyz",
        elem_a="terminal_pin",
        elem_b="root_bushing",
        min_overlap=0.009,
        name="terminal pivot has captured pin overlap",
    )

    ctx.expect_gap(
        first,
        base,
        axis="z",
        min_gap=-0.001,
        positive_elem="fold_stop_tab",
        negative_elem="lower_stop_tab",
        name="first link stop tab does not bury into base stop",
    )

    rest_tip = ctx.part_element_world_aabb(terminal, elem="tip_pad")
    with ctx.pose({j0: 0.82, j1: 0.58, j2: 0.44}):
        ctx.expect_gap(
            center,
            base,
            axis="z",
            min_gap=0.035,
            positive_elem="main_web",
            negative_elem="mounting_foot",
            name="raised center link clears the mounting foot",
        )
        raised_tip = ctx.part_element_world_aabb(terminal, elem="tip_pad")

    ctx.check(
        "tip pad sweeps upward at the joint limits",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[0][2] > rest_tip[0][2] + 0.20,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
