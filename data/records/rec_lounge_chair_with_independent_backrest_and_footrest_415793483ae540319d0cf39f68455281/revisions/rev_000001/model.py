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


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded upholstery block, authored in meters."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (ca * x + sa * z, y, -sa * x + ca * z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="padded_chaise_recliner")

    fabric = Material("warm_taupe_fabric", rgba=(0.58, 0.47, 0.35, 1.0))
    seam = Material("darker_fabric_welting", rgba=(0.34, 0.27, 0.21, 1.0))
    shadow_fabric = Material("shadowed_base_fabric", rgba=(0.25, 0.21, 0.18, 1.0))
    metal = Material("blackened_steel", rgba=(0.03, 0.03, 0.028, 1.0))

    # Fixed recliner tub: broad arms, low plinth, and the seat cushion are one
    # rigid base frame.  X is forward, Y is the transverse hinge direction, and
    # Z is up.
    base = model.part("base")
    base.visual(
        Box((1.06, 0.98, 0.11)),
        origin=Origin(xyz=(0.39, 0.0, 0.405)),
        material=shadow_fabric,
        name="lower_plinth",
    )
    base.visual(
        mesh_from_cadquery(_rounded_box((0.80, 0.60, 0.13), 0.045), "seat_cushion"),
        origin=Origin(xyz=(0.39, 0.0, 0.515)),
        material=fabric,
        name="seat_cushion",
    )
    for y, suffix in ((0.40, "0"), (-0.40, "1")):
        base.visual(
            mesh_from_cadquery(_rounded_box((1.08, 0.18, 0.32), 0.055), f"side_arm_{suffix}"),
            origin=Origin(xyz=(0.38, y, 0.58)),
            material=fabric,
            name=f"side_arm_{suffix}",
        )

    # Subtle padded seams on the fixed seat, slightly proud of the top cushion.
    for x, suffix in ((0.17, "0"), (0.44, "1"), (0.69, "2")):
        base.visual(
            Box((0.018, 0.56, 0.010)),
            origin=Origin(xyz=(x, 0.0, 0.577)),
            material=seam,
            name=f"seat_welt_{suffix}",
        )

    cyl_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    base.visual(
        Cylinder(radius=0.027, length=0.050),
        origin=Origin(xyz=(-0.10, 0.305, 0.565), rpy=cyl_y.rpy),
        material=metal,
        name="back_socket_0",
    )
    base.visual(
        Cylinder(radius=0.027, length=0.050),
        origin=Origin(xyz=(-0.10, -0.305, 0.565), rpy=cyl_y.rpy),
        material=metal,
        name="back_socket_1",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.82, 0.305, 0.515), rpy=cyl_y.rpy),
        material=metal,
        name="front_socket_0",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.82, -0.305, 0.515), rpy=cyl_y.rpy),
        material=metal,
        name="front_socket_1",
    )

    # Backrest child frame is exactly on the rear transverse hinge line.
    backrest = model.part("backrest")
    back_tilt = -0.36
    back_height = 0.82
    back_thickness = 0.14
    backrest.visual(
        mesh_from_cadquery(
            _rounded_box((back_thickness, 0.52, back_height), 0.045),
            "back_panel",
        ),
        origin=Origin(xyz=_rotate_y((0.0, 0.0, 0.08 + back_height / 2.0), back_tilt), rpy=(0.0, back_tilt, 0.0)),
        material=fabric,
        name="back_panel",
    )
    backrest.visual(
        Cylinder(radius=0.024, length=0.61),
        origin=Origin(rpy=cyl_y.rpy),
        material=metal,
        name="back_barrel",
    )
    backrest.visual(
        Box((0.060, 0.44, 0.090)),
        origin=Origin(xyz=_rotate_y((0.015, 0.0, 0.045), back_tilt), rpy=(0.0, back_tilt, 0.0)),
        material=metal,
        name="back_hinge_leaf",
    )
    for z, suffix in ((0.28, "0"), (0.52, "1")):
        backrest.visual(
            Box((0.010, 0.50, 0.014)),
            origin=Origin(
                xyz=_rotate_y((back_thickness / 2.0 + 0.004, 0.0, z), back_tilt),
                rpy=(0.0, back_tilt, 0.0),
            ),
            material=seam,
            name=f"back_welt_{suffix}",
        )

    # Leg rest child frame is on the front hinge axis; its panel extends
    # forward from the hinge as a long chaise-style pad.
    legrest = model.part("legrest")
    legrest.visual(
        mesh_from_cadquery(_rounded_box((0.78, 0.52, 0.11), 0.040), "legrest_panel"),
        origin=Origin(xyz=(0.44, 0.0, 0.0)),
        material=fabric,
        name="legrest_panel",
    )
    legrest.visual(
        Cylinder(radius=0.023, length=0.61),
        origin=Origin(rpy=cyl_y.rpy),
        material=metal,
        name="legrest_barrel",
    )
    legrest.visual(
        Box((0.110, 0.43, 0.032)),
        origin=Origin(xyz=(0.055, 0.0, -0.030)),
        material=metal,
        name="legrest_hinge_leaf",
    )
    for x, suffix in ((0.23, "0"), (0.52, "1")):
        legrest.visual(
            Box((0.016, 0.50, 0.010)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=seam,
            name=f"legrest_welt_{suffix}",
        )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=Origin(xyz=(-0.10, 0.0, 0.565)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=0.80),
    )
    model.articulation(
        "legrest_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=legrest,
        origin=Origin(xyz=(0.82, 0.0, 0.515)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    backrest = object_model.get_part("backrest")
    legrest = object_model.get_part("legrest")
    back_hinge = object_model.get_articulation("back_hinge")
    legrest_hinge = object_model.get_articulation("legrest_hinge")

    for socket_name in ("back_socket_0", "back_socket_1"):
        ctx.allow_overlap(
            backrest,
            base,
            elem_a="back_barrel",
            elem_b=socket_name,
            reason="The backrest hinge pin is intentionally captured inside the side-arm socket.",
        )
    for socket_name in ("front_socket_0", "front_socket_1"):
        ctx.allow_overlap(
            legrest,
            base,
            elem_a="legrest_barrel",
            elem_b=socket_name,
            reason="The leg-rest hinge pin is intentionally captured inside the front side-arm socket.",
        )

    ctx.check(
        "independent reclining panels",
        len(object_model.parts) == 3 and len(object_model.articulations) == 2,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )
    ctx.expect_overlap(
        backrest,
        base,
        axes="xz",
        elem_a="back_barrel",
        elem_b="back_socket_0",
        min_overlap=0.030,
        name="back hinge barrels share the rear transverse axis",
    )
    ctx.expect_overlap(
        backrest,
        base,
        axes="xz",
        elem_a="back_barrel",
        elem_b="back_socket_1",
        min_overlap=0.030,
        name="back hinge uses paired side-arm sockets",
    )
    ctx.expect_overlap(
        legrest,
        base,
        axes="xz",
        elem_a="legrest_barrel",
        elem_b="front_socket_0",
        min_overlap=0.030,
        name="legrest hinge barrels share the front transverse axis",
    )
    ctx.expect_overlap(
        legrest,
        base,
        axes="xz",
        elem_a="legrest_barrel",
        elem_b="front_socket_1",
        min_overlap=0.030,
        name="legrest hinge uses paired front sockets",
    )
    ctx.expect_within(
        backrest,
        base,
        axes="y",
        inner_elem="back_panel",
        outer_elem="lower_plinth",
        margin=0.0,
        name="back panel sits between the broad side arms",
    )
    ctx.expect_within(
        legrest,
        base,
        axes="y",
        inner_elem="legrest_panel",
        outer_elem="lower_plinth",
        margin=0.0,
        name="leg rest stays centered between the arms",
    )

    back_rest_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 0.65}):
        back_reclined_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest reclines rearward",
        back_rest_aabb is not None
        and back_reclined_aabb is not None
        and back_reclined_aabb[0][0] < back_rest_aabb[0][0] - 0.08,
        details=f"rest={back_rest_aabb}, reclined={back_reclined_aabb}",
    )

    leg_rest_aabb = ctx.part_world_aabb(legrest)
    with ctx.pose({legrest_hinge: 0.55}):
        leg_lowered_aabb = ctx.part_world_aabb(legrest)
    ctx.check(
        "leg rest pivots downward independently",
        leg_rest_aabb is not None
        and leg_lowered_aabb is not None
        and leg_lowered_aabb[0][2] < leg_rest_aabb[0][2] - 0.18,
        details=f"rest={leg_rest_aabb}, lowered={leg_lowered_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
