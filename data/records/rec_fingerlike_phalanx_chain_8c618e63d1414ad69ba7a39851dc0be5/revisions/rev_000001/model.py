from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HINGE_RPY = (math.pi / 2.0, 0.0, 0.0)


def _spatulate_pad_profile() -> list[tuple[float, float]]:
    """Flat probing pad outline in local X/Y, extruded through Z."""
    neck_half = 0.013
    pad_half = 0.026
    nose_center_x = 0.195
    nose_radius = 0.026
    points: list[tuple[float, float]] = [
        (0.138, -neck_half),
        (0.158, -pad_half),
        (nose_center_x, -pad_half),
    ]
    for i in range(1, 14):
        theta = -math.pi / 2.0 + math.pi * i / 13.0
        points.append(
            (
                nose_center_x + nose_radius * math.cos(theta),
                pad_half * math.sin(theta),
            )
        )
    points.extend(
        [
            (0.158, pad_half),
            (0.138, neck_half),
        ]
    )
    return points


def _add_hinge_cylinder(part, *, x: float, y: float, z: float, radius: float, length: float, name: str, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=HINGE_RPY),
        material=material,
        name=name,
    )


def _add_fork_knuckle(part, *, prefix: str, x: float, link_material: Material, steel: Material) -> None:
    """Side cheek pair, bridge, visible barrel sleeves, and pin-head caps."""
    cheek_y = 0.024
    _add_hinge_cylinder(
        part,
        x=x,
        y=0.0,
        z=0.0,
        radius=0.0055,
        length=0.072,
        name=f"{prefix}_pin",
        material=steel,
    )
    for suffix, y in (("pos", cheek_y), ("neg", -cheek_y)):
        part.visual(
            Box((0.038, 0.010, 0.040)),
            origin=Origin(xyz=(x, y, 0.0)),
            material=link_material,
            name=f"{prefix}_cheek_{suffix}",
        )
        _add_hinge_cylinder(
            part,
            x=x,
            y=y,
            z=0.0,
            radius=0.0115,
            length=0.010,
            name=f"{prefix}_barrel_{suffix}",
            material=link_material,
        )
        _add_hinge_cylinder(
            part,
            x=x,
            y=0.031 if y > 0.0 else -0.031,
            z=0.0,
            radius=0.014,
            length=0.005,
            name=f"{prefix}_pin_cap_{suffix}",
            material=steel,
        )

    # Rear bridge ties both cheeks back into the phalanx without filling the
    # child barrel clearance at the hinge axis.
    part.visual(
        Box((0.034, 0.058, 0.016)),
        origin=Origin(xyz=(x - 0.034, 0.0, -0.002)),
        material=link_material,
        name=f"{prefix}_bridge",
    )


def _add_proximal_boss(part, *, steel: Material, bronze: Material) -> None:
    """Central barrel and thin spacer collars that sit between fork cheeks."""
    _add_hinge_cylinder(
        part,
        x=0.0,
        y=0.0,
        z=0.0,
        radius=0.010,
        length=0.020,
        name="proximal_barrel",
        material=steel,
    )
    _add_hinge_cylinder(
        part,
        x=0.0,
        y=0.0118,
        z=0.0,
        radius=0.012,
        length=0.004,
        name="proximal_spacer_pos",
        material=bronze,
    )
    _add_hinge_cylinder(
        part,
        x=0.0,
        y=-0.0118,
        z=0.0,
        radius=0.012,
        length=0.004,
        name="proximal_spacer_neg",
        material=bronze,
    )


def _add_phalanx(part, *, length: float, has_distal_fork: bool, link_material: Material, steel: Material, bronze: Material) -> None:
    _add_proximal_boss(part, steel=steel, bronze=bronze)
    body_end = length - (0.024 if has_distal_fork else 0.012)
    body_length = body_end - 0.009
    part.visual(
        Box((body_length, 0.016, 0.014)),
        origin=Origin(xyz=(0.009 + body_length / 2.0, 0.0, 0.0)),
        material=link_material,
        name="spine",
    )
    part.visual(
        Box((body_length * 0.80, 0.006, 0.020)),
        origin=Origin(xyz=(0.030 + body_length * 0.40, 0.0, 0.0)),
        material=link_material,
        name="center_rib",
    )
    # Small countersunk pads on the top face make the narrow metal strip read as
    # serviceable hardware rather than a plain bar.
    for idx, x in enumerate((0.070, max(0.100, body_end - 0.055))):
        part.visual(
            Cylinder(radius=0.006, length=0.0025),
            origin=Origin(xyz=(x, 0.0, 0.0082)),
            material=steel,
            name=f"top_fastener_{idx}",
        )
    if has_distal_fork:
        _add_fork_knuckle(part, prefix="distal", x=length, link_material=link_material, steel=steel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_probing_finger_chain")

    dark_anodized = model.material("dark_anodized", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    bronze = model.material("bronze_spacer", rgba=(0.62, 0.44, 0.22, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.018, 0.018, 0.016, 1.0))
    hard_stop = model.material("black_stop_blocks", rgba=(0.05, 0.05, 0.045, 1.0))

    base = model.part("base_mount")
    base.visual(
        Box((0.125, 0.078, 0.010)),
        origin=Origin(xyz=(-0.030, 0.0, -0.031)),
        material=dark_anodized,
        name="base_plate",
    )
    base.visual(
        Box((0.036, 0.060, 0.024)),
        origin=Origin(xyz=(-0.028, 0.0, -0.014)),
        material=dark_anodized,
        name="root_pedestal",
    )
    _add_fork_knuckle(base, prefix="root", x=0.0, link_material=dark_anodized, steel=steel)
    for suffix, y in (("pos", 0.036), ("neg", -0.036)):
        base.visual(
            Box((0.024, 0.014, 0.032)),
            origin=Origin(xyz=(0.034, y, -0.003)),
            material=hard_stop,
            name=f"root_stop_{suffix}",
        )
        base.visual(
            Box((0.044, 0.012, 0.014)),
            origin=Origin(xyz=(0.006, y, -0.013)),
            material=hard_stop,
            name=f"stop_web_{suffix}",
        )
        base.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(xyz=(0.034, y, 0.013)),
            material=steel,
            name=f"stop_screw_{suffix}",
        )

    phalanx_0 = model.part("phalanx_0")
    phalanx_1 = model.part("phalanx_1")
    phalanx_2 = model.part("phalanx_2")

    _add_phalanx(
        phalanx_0,
        length=0.240,
        has_distal_fork=True,
        link_material=dark_anodized,
        steel=steel,
        bronze=bronze,
    )
    _add_phalanx(
        phalanx_1,
        length=0.205,
        has_distal_fork=True,
        link_material=dark_anodized,
        steel=steel,
        bronze=bronze,
    )
    _add_phalanx(
        phalanx_2,
        length=0.170,
        has_distal_fork=False,
        link_material=dark_anodized,
        steel=steel,
        bronze=bronze,
    )
    phalanx_2.visual(
        Box((0.052, 0.020, 0.006)),
        origin=Origin(xyz=(0.143, 0.0, -0.004)),
        material=steel,
        name="tip_backing",
    )
    tip_pad = ExtrudeGeometry(_spatulate_pad_profile(), 0.010, cap=True, center=True)
    phalanx_2.visual(
        mesh_from_geometry(tip_pad, "spatulate_tip_pad"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber,
        name="tip_pad",
    )

    limits = MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.05)
    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=phalanx_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "knuckle_0",
        ArticulationType.REVOLUTE,
        parent=phalanx_0,
        child=phalanx_1,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "knuckle_1",
        ArticulationType.REVOLUTE,
        parent=phalanx_1,
        child=phalanx_2,
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_mount")
    phalanx_0 = object_model.get_part("phalanx_0")
    phalanx_1 = object_model.get_part("phalanx_1")
    phalanx_2 = object_model.get_part("phalanx_2")
    base_hinge = object_model.get_articulation("base_hinge")
    knuckle_0 = object_model.get_articulation("knuckle_0")
    knuckle_1 = object_model.get_articulation("knuckle_1")

    for joint in (base_hinge, knuckle_0, knuckle_1):
        ctx.check(
            f"{joint.name} is a limited revolute hinge",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and 1.0 <= joint.motion_limits.upper <= 1.1,
            details=str(joint),
        )

    pin_interfaces = (
        (base, phalanx_0, "root_pin"),
        (phalanx_0, phalanx_1, "distal_pin"),
        (phalanx_1, phalanx_2, "distal_pin"),
    )
    for parent, child, pin_name in pin_interfaces:
        for bearing_name in ("proximal_barrel", "proximal_spacer_pos", "proximal_spacer_neg"):
            ctx.allow_overlap(
                parent,
                child,
                elem_a=pin_name,
                elem_b=bearing_name,
                reason=(
                    "The visible hinge pin is intentionally represented as a solid shaft "
                    "running through the child bearing sleeve and spacer collars."
                ),
            )
            ctx.expect_overlap(
                parent,
                child,
                axes="xyz",
                min_overlap=0.003,
                elem_a=pin_name,
                elem_b=bearing_name,
                name=f"{parent.name} {pin_name} captures {child.name} {bearing_name}",
            )

    ctx.expect_origin_gap(
        phalanx_1,
        phalanx_0,
        axis="x",
        min_gap=0.235,
        max_gap=0.245,
        name="first link length sets second knuckle",
    )
    ctx.expect_origin_gap(
        phalanx_2,
        phalanx_1,
        axis="x",
        min_gap=0.200,
        max_gap=0.210,
        name="second link length sets third knuckle",
    )

    clearance_pairs = (
        (base, phalanx_0, "root"),
        (phalanx_0, phalanx_1, "distal"),
        (phalanx_1, phalanx_2, "distal"),
    )
    for parent, child, prefix in clearance_pairs:
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            min_gap=0.003,
            positive_elem=f"{prefix}_barrel_pos",
            negative_elem="proximal_spacer_pos",
            name=f"{parent.name} positive cheek clears spacer",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            min_gap=0.003,
            positive_elem="proximal_spacer_neg",
            negative_elem=f"{prefix}_barrel_neg",
            name=f"{parent.name} negative cheek clears spacer",
        )

    rest_tip = ctx.part_world_aabb(phalanx_2)
    with ctx.pose({base_hinge: 1.05}):
        raised_mid = ctx.part_world_position(phalanx_1)
    with ctx.pose({knuckle_0: 1.05}):
        raised_distal = ctx.part_world_position(phalanx_2)
    with ctx.pose({knuckle_1: 1.05}):
        raised_tip = ctx.part_world_aabb(phalanx_2)
    with ctx.pose({base_hinge: 1.05, knuckle_0: 1.05, knuckle_1: 1.05}):
        ctx.expect_gap(
            phalanx_1,
            base,
            axis="z",
            min_gap=0.050,
            name="curled middle link clears root hardware",
        )
        ctx.expect_gap(
            phalanx_2,
            phalanx_0,
            axis="z",
            min_gap=0.050,
            name="curled fingertip clears proximal link",
        )
        ctx.expect_gap(
            phalanx_2,
            base,
            axis="z",
            min_gap=0.120,
            name="curled fingertip clears base mount",
        )
    ctx.check(
        "base hinge bends chain upward",
        raised_mid is not None and raised_mid[2] > 0.19,
        details=f"raised_mid={raised_mid}",
    )
    ctx.check(
        "middle hinge bends distal link upward",
        raised_distal is not None and raised_distal[2] > 0.17,
        details=f"raised_distal={raised_distal}",
    )
    ctx.check(
        "tip hinge lifts spatulate pad",
        rest_tip is not None and raised_tip is not None and raised_tip[1][2] > rest_tip[1][2] + 0.13,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
