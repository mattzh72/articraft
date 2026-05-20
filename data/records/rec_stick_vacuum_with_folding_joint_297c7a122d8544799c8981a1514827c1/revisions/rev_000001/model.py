from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> object:
    """Small CadQuery helper for premium molded/plated rectangular solids."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_folding_stick_vacuum")

    pearl = Material("warm_pearl_polymer", rgba=(0.86, 0.84, 0.78, 1.0))
    graphite = Material("soft_graphite_polymer", rgba=(0.075, 0.080, 0.085, 1.0))
    satin = Material("satin_painted_metal", rgba=(0.56, 0.58, 0.57, 1.0))
    elastomer = Material("matte_black_elastomer", rgba=(0.012, 0.012, 0.011, 1.0))
    smoked = Material("smoked_clear_bin", rgba=(0.45, 0.52, 0.55, 0.38))
    accent = Material("muted_champagne_accent", rgba=(0.82, 0.62, 0.36, 1.0))

    # Root: a low, wide floor nozzle.  Dimensions are consumer-stick-vacuum scale
    # (roughly 30 cm cleaning width) with softly radiused polymer surfacing.
    floor_head = model.part("floor_head")
    floor_head.visual(
        mesh_from_cadquery(_rounded_box((0.105, 0.315, 0.040), 0.010), "floor_head_shell"),
        origin=Origin(xyz=(0.005, 0.0, 0.035)),
        material=pearl,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.018, 0.302, 0.020)),
        origin=Origin(xyz=(0.064, 0.0, 0.032)),
        material=elastomer,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.012, 0.275, 0.012)),
        origin=Origin(xyz=(-0.052, 0.0, 0.024)),
        material=graphite,
        name="rear_wear_strip",
    )
    floor_head.visual(
        Cylinder(radius=0.011, length=0.278),
        origin=Origin(xyz=(0.030, 0.0, 0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="brush_window",
    )
    # Bracketed nozzle pitch joint: two cheeks tied into a rear bridge on the
    # nozzle shell, with the wand tongue and pin captured between the cheeks.
    floor_head.visual(
        Box((0.040, 0.014, 0.070)),
        origin=Origin(xyz=(-0.030, -0.044, 0.085)),
        material=graphite,
        name="nozzle_cheek_0",
    )
    floor_head.visual(
        Box((0.040, 0.014, 0.070)),
        origin=Origin(xyz=(-0.030, 0.044, 0.085)),
        material=graphite,
        name="nozzle_cheek_1",
    )
    floor_head.visual(
        Box((0.048, 0.102, 0.016)),
        origin=Origin(xyz=(-0.030, 0.0, 0.055)),
        material=graphite,
        name="nozzle_bridge",
    )
    floor_head.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(-0.030, -0.058, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="nozzle_pin_cap_0",
    )
    floor_head.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(-0.030, 0.058, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="nozzle_pin_cap_1",
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.020, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="nozzle_pivot_barrel",
    )
    lower_wand.visual(
        Cylinder(radius=0.006, length=0.104),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="nozzle_pin",
    )
    lower_wand.visual(
        Cylinder(radius=0.014, length=0.660),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=satin,
        name="metal_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.020, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=graphite,
        name="lower_socket",
    )
    lower_wand.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=graphite,
        name="fold_collar",
    )
    lower_wand.visual(
        Box((0.050, 0.014, 0.084)),
        origin=Origin(xyz=(0.0, -0.044, 0.760)),
        material=graphite,
        name="fold_cheek_0",
    )
    lower_wand.visual(
        Box((0.050, 0.014, 0.084)),
        origin=Origin(xyz=(0.0, 0.044, 0.760)),
        material=graphite,
        name="fold_cheek_1",
    )
    lower_wand.visual(
        Box((0.055, 0.102, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.712)),
        material=graphite,
        name="fold_bridge",
    )
    lower_wand.visual(
        Box((0.018, 0.078, 0.022)),
        origin=Origin(xyz=(0.026, 0.0, 0.690)),
        material=accent,
        name="fold_release_key",
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        Cylinder(radius=0.023, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="fold_pivot_barrel",
    )
    upper_body.visual(
        Cylinder(radius=0.0065, length=0.106),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="fold_pin",
    )
    upper_body.visual(
        Cylinder(radius=0.014, length=0.405),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=satin,
        name="upper_tube",
    )
    upper_body.visual(
        Cylinder(radius=0.023, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=graphite,
        name="fold_socket",
    )
    upper_body.visual(
        Cylinder(radius=0.024, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=graphite,
        name="body_socket",
    )
    upper_body.visual(
        Box((0.070, 0.046, 0.072)),
        origin=Origin(xyz=(0.024, 0.0, 0.480)),
        material=graphite,
        name="airpath_bridge",
    )
    upper_body.visual(
        mesh_from_cadquery(_rounded_box((0.155, 0.112, 0.135), 0.030), "motor_shroud"),
        origin=Origin(xyz=(0.054, 0.0, 0.785)),
        material=pearl,
        name="motor_shroud",
    )
    upper_body.visual(
        Cylinder(radius=0.047, length=0.210),
        origin=Origin(xyz=(0.047, 0.0, 0.610)),
        material=smoked,
        name="dust_bin",
    )
    upper_body.visual(
        Cylinder(radius=0.049, length=0.018),
        origin=Origin(xyz=(0.047, 0.0, 0.505)),
        material=graphite,
        name="bin_lower_ring",
    )
    upper_body.visual(
        Cylinder(radius=0.050, length=0.022),
        origin=Origin(xyz=(0.047, 0.0, 0.722)),
        material=graphite,
        name="bin_upper_ring",
    )
    upper_body.visual(
        mesh_from_cadquery(_rounded_box((0.130, 0.090, 0.064), 0.017), "rear_battery_pack"),
        origin=Origin(xyz=(-0.062, 0.0, 0.770)),
        material=graphite,
        name="battery_pack",
    )
    upper_body.visual(
        Box((0.055, 0.052, 0.028)),
        origin=Origin(xyz=(-0.004, 0.0, 0.770)),
        material=graphite,
        name="battery_bridge",
    )
    upper_body.visual(
        mesh_from_cadquery(_rounded_box((0.044, 0.050, 0.245), 0.018), "handle_grip"),
        origin=Origin(xyz=(-0.130, 0.0, 0.675)),
        material=elastomer,
        name="handle_grip",
    )
    upper_body.visual(
        Box((0.088, 0.050, 0.040)),
        origin=Origin(xyz=(-0.094, 0.0, 0.805)),
        material=graphite,
        name="handle_top_bridge",
    )
    upper_body.visual(
        Box((0.100, 0.050, 0.036)),
        origin=Origin(xyz=(-0.082, 0.0, 0.552)),
        material=graphite,
        name="handle_lower_bridge",
    )
    upper_body.visual(
        Box((0.028, 0.030, 0.070)),
        origin=Origin(xyz=(-0.108, 0.0, 0.636)),
        material=accent,
        name="trigger_paddle",
    )
    upper_body.visual(
        Box((0.008, 0.070, 0.030)),
        origin=Origin(xyz=(0.132, 0.0, 0.805)),
        material=graphite,
        name="exhaust_slot_band",
    )

    model.articulation(
        "nozzle_pitch",
        ArticulationType.REVOLUTE,
        parent=floor_head,
        child=lower_wand,
        origin=Origin(xyz=(-0.030, 0.0, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.2, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_body,
        origin=Origin(xyz=(0.0, 0.0, 0.760)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=2.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    floor_head = object_model.get_part("floor_head")
    lower_wand = object_model.get_part("lower_wand")
    upper_body = object_model.get_part("upper_body")
    nozzle_pitch = object_model.get_articulation("nozzle_pitch")
    fold_hinge = object_model.get_articulation("fold_hinge")

    # Captured hinge pins are intentionally seated through bracket cheeks.  The
    # allowances are scoped to the named pin/cheek interfaces and paired with
    # exact checks proving the pins remain within their brackets.
    for cheek in ("nozzle_cheek_0", "nozzle_cheek_1"):
        ctx.allow_overlap(
            floor_head,
            lower_wand,
            elem_a=cheek,
            elem_b="nozzle_pin",
            reason="The nozzle hinge pin is intentionally captured through the bracket cheek bore.",
        )
    for cheek in ("fold_cheek_0", "fold_cheek_1"):
        ctx.allow_overlap(
            lower_wand,
            upper_body,
            elem_a=cheek,
            elem_b="fold_pin",
            reason="The folding hinge pin is intentionally captured through the wand clevis cheek bore.",
        )

    ctx.expect_within(
        lower_wand,
        floor_head,
        axes="y",
        inner_elem="nozzle_pivot_barrel",
        outer_elem="nozzle_bridge",
        margin=0.006,
        name="nozzle pivot barrel sits between floor-head cheeks",
    )
    ctx.expect_overlap(
        lower_wand,
        floor_head,
        axes="z",
        elem_a="nozzle_pivot_barrel",
        elem_b="nozzle_cheek_0",
        min_overlap=0.018,
        name="nozzle pivot aligns vertically with cheek bores",
    )
    ctx.expect_within(
        upper_body,
        lower_wand,
        axes="y",
        inner_elem="fold_pivot_barrel",
        outer_elem="fold_bridge",
        margin=0.006,
        name="fold pivot barrel sits between wand clevis cheeks",
    )
    ctx.expect_overlap(
        upper_body,
        lower_wand,
        axes="z",
        elem_a="fold_pivot_barrel",
        elem_b="fold_cheek_0",
        min_overlap=0.020,
        name="folding pivot aligns vertically with cheek bores",
    )

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return (mn[0] + mx[0]) * 0.5

    upright_shroud_x = _aabb_center_x(ctx.part_element_world_aabb(upper_body, elem="motor_shroud"))
    with ctx.pose({fold_hinge: 1.85}):
        folded_shroud_x = _aabb_center_x(
            ctx.part_element_world_aabb(upper_body, elem="motor_shroud")
        )

    ctx.check(
        "folding joint changes upper-body pose",
        upright_shroud_x is not None
        and folded_shroud_x is not None
        and folded_shroud_x > upright_shroud_x + 0.20,
        details=f"upright_shroud_x={upright_shroud_x}, folded_shroud_x={folded_shroud_x}",
    )

    return ctx.report()


object_model = build_object_model()
