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


def _rounded_box(sx: float, sy: float, sz: float, radius: float) -> cq.Workplane:
    """Small CadQuery helper for upholstered/chromed rounded rectangular solids."""
    return cq.Workplane("XY").box(sx, sy, sz).edges().fillet(radius)


def _coord(vec, index: int) -> float:
    """Read a coordinate from SDK Vec3-like values or plain tuples."""
    if hasattr(vec, "__getitem__"):
        return float(vec[index])
    return float((vec.x, vec.y, vec.z)[index])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_lounge_chair")

    chrome = model.material("brushed_chrome", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_shell = model.material("satin_black_shell", rgba=(0.035, 0.033, 0.032, 1.0))
    leather = model.material("warm_cognac_leather", rgba=(0.62, 0.34, 0.16, 1.0))
    seam = model.material("dark_seam_shadow", rgba=(0.10, 0.06, 0.035, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.42, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=chrome,
        name="floor_disk",
    )
    base.visual(
        Cylinder(radius=0.34, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=dark_shell,
        name="rubber_trim",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.07, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=chrome,
        name="center_column",
    )
    pedestal.visual(
        Cylinder(radius=0.14, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=chrome,
        name="lower_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.20, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=chrome,
        name="top_plate",
    )

    seat_shell = model.part("seat_shell")
    seat_shell_shape = (
        _rounded_box(0.82, 0.78, 0.065, 0.025)
        .translate((0.0, 0.0, 0.045))
        .union(_rounded_box(0.70, 0.085, 0.13, 0.025).translate((-0.02, 0.355, 0.105)))
        .union(_rounded_box(0.70, 0.085, 0.13, 0.025).translate((-0.02, -0.355, 0.105)))
        .union(_rounded_box(0.085, 0.66, 0.12, 0.025).translate((0.36, 0.0, 0.098)))
        .union(_rounded_box(0.06, 0.62, 0.095, 0.018).translate((-0.37, 0.0, 0.095)))
    )
    seat_shell.visual(
        mesh_from_cadquery(seat_shell_shape, "seat_shell"),
        material=dark_shell,
        name="molded_shell",
    )
    seat_cushion_shape = _rounded_box(0.62, 0.56, 0.055, 0.025).translate((0.03, 0.0, 0.128))
    seat_shell.visual(
        mesh_from_cadquery(seat_cushion_shape, "seat_cushion"),
        material=leather,
        name="seat_cushion",
    )
    seat_shell.visual(
        Box((0.56, 0.012, 0.01)),
        origin=Origin(xyz=(0.04, 0.0, 0.158)),
        material=seam,
        name="center_seam",
    )
    seat_shell.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=chrome,
        name="swivel_socket",
    )
    # Yoke cheeks at the rear backrest hinge; the backrest barrel sits between them.
    for i, y in enumerate((-0.40, 0.40)):
        seat_shell.visual(
            Box((0.08, 0.05, 0.12)),
            origin=Origin(xyz=(-0.40, y, 0.18)),
            material=chrome,
            name=f"rear_bracket_{i}",
        )
    # Matching front yoke cheeks for the stowed/folding leg panel.
    for i, y in enumerate((-0.40, 0.40)):
        seat_shell.visual(
            Box((0.08, 0.08, 0.075)),
            origin=Origin(xyz=(0.445, y, 0.035)),
            material=chrome,
            name=f"front_bracket_{i}",
        )

    backrest = model.part("backrest")
    back_angle = -0.25
    back_span = 0.76
    back_center = (
        math.sin(back_angle) * (back_span / 2.0 + 0.035),
        0.0,
        math.cos(back_angle) * (back_span / 2.0 + 0.035),
    )
    back_shell_shape = _rounded_box(0.085, 0.74, back_span, 0.025)
    backrest.visual(
        mesh_from_cadquery(back_shell_shape, "back_shell"),
        origin=Origin(xyz=back_center, rpy=(0.0, back_angle, 0.0)),
        material=dark_shell,
        name="back_shell",
    )
    front_offset = 0.042
    front_vec = (math.cos(back_angle), 0.0, -math.sin(back_angle))
    cushion_center = (
        back_center[0] + front_vec[0] * front_offset,
        0.0,
        back_center[2] + front_vec[2] * front_offset,
    )
    back_cushion_shape = _rounded_box(0.038, 0.64, 0.64, 0.022)
    backrest.visual(
        mesh_from_cadquery(back_cushion_shape, "back_cushion"),
        origin=Origin(xyz=cushion_center, rpy=(0.0, back_angle, 0.0)),
        material=leather,
        name="back_cushion",
    )
    backrest.visual(
        Box((0.012, 0.58, 0.012)),
        origin=Origin(xyz=(cushion_center[0] + 0.016, 0.0, cushion_center[2] + 0.01), rpy=(0.0, back_angle, 0.0)),
        material=seam,
        name="back_seam",
    )
    backrest.visual(
        Cylinder(radius=0.03, length=0.75),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    for i, y in enumerate((-0.27, 0.27)):
        backrest.visual(
            Box((0.048, 0.055, 0.085)),
            origin=Origin(xyz=(-0.012, y, 0.04)),
            material=chrome,
            name=f"hinge_strap_{i}",
        )

    leg_panel = model.part("leg_panel")
    leg_panel_shape = _rounded_box(0.07, 0.64, 0.35, 0.022)
    leg_panel.visual(
        mesh_from_cadquery(leg_panel_shape, "leg_panel"),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=leather,
        name="padded_panel",
    )
    leg_panel.visual(
        Box((0.012, 0.56, 0.012)),
        origin=Origin(xyz=(0.038, 0.0, -0.18)),
        material=seam,
        name="leg_seam",
    )
    leg_panel.visual(
        Cylinder(radius=0.026, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )

    model.articulation(
        "base_to_pedestal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pedestal_to_seat",
        ArticulationType.FIXED,
        parent=pedestal,
        child=seat_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=backrest,
        origin=Origin(xyz=(-0.455, 0.0, 0.18)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.20, upper=0.75),
    )
    model.articulation(
        "seat_to_leg_panel",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=leg_panel,
        origin=Origin(xyz=(0.455, 0.0, 0.005)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat_shell = object_model.get_part("seat_shell")
    backrest = object_model.get_part("backrest")
    leg_panel = object_model.get_part("leg_panel")
    swivel = object_model.get_articulation("base_to_pedestal")
    back_hinge = object_model.get_articulation("seat_to_backrest")
    leg_hinge = object_model.get_articulation("seat_to_leg_panel")

    ctx.check(
        "pedestal has vertical swivel",
        tuple(round(v, 3) for v in swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and swivel.motion_limits.lower <= -3.0
        and swivel.motion_limits.upper >= 3.0,
        details=f"axis={swivel.axis}, limits={swivel.motion_limits}",
    )
    ctx.expect_gap(
        seat_shell,
        pedestal,
        axis="z",
        positive_elem="swivel_socket",
        negative_elem="top_plate",
        max_gap=0.002,
        max_penetration=0.001,
        name="seat shell sits on pedestal plate",
    )

    rest_back = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 0.70}):
        reclined_back = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest reclines rearward",
        rest_back is not None
        and reclined_back is not None
        and _coord(reclined_back[0], 0) < _coord(rest_back[0], 0) - 0.12,
        details=f"rest={rest_back}, reclined={reclined_back}",
    )

    rest_leg = ctx.part_world_aabb(leg_panel)
    with ctx.pose({leg_hinge: 1.45}):
        deployed_leg = ctx.part_world_aabb(leg_panel)
    ctx.check(
        "leg panel folds forward",
        rest_leg is not None
        and deployed_leg is not None
        and _coord(deployed_leg[1], 0) > _coord(rest_leg[1], 0) + 0.25,
        details=f"rest={rest_leg}, deployed={deployed_leg}",
    )

    return ctx.report()


object_model = build_object_model()
