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


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 28):
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(x0: float, x1: float, width: float, segments: int = 18):
    """A flat-ended link plate outline with rounded pivot ends in local XY."""
    r = width * 0.5
    pts = []
    # Traverse the right end from lower edge to upper edge, then the left end.
    for i in range(segments + 1):
        a = -0.5 * math.pi + math.pi * i / segments
        pts.append((x1 + r * math.cos(a), r * math.sin(a)))
    for i in range(segments + 1):
        a = 0.5 * math.pi + math.pi * i / segments
        pts.append((x0 + r * math.cos(a), r * math.sin(a)))
    return pts


def _link_plate_mesh(
    *,
    length: float,
    width: float,
    thickness: float,
    pivot_radius: float,
    slot_length: float,
    slot_width: float,
    name: str,
):
    outer = _capsule_profile(0.0, length, width)
    holes = [
        _circle_profile(0.0, 0.0, pivot_radius),
        _circle_profile(length, 0.0, pivot_radius),
        _capsule_profile(
            0.5 * (length - slot_length),
            0.5 * (length + slot_length),
            slot_width,
            segments=12,
        ),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, thickness, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_folding_arm")

    powder_coat = model.material("powder_coat", rgba=(0.11, 0.13, 0.14, 1.0))
    link_finish = model.material("link_finish", rgba=(0.72, 0.76, 0.76, 1.0))
    dark_rail = model.material("dark_rail", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("rubber_pad", rgba=(0.015, 0.015, 0.012, 1.0))
    fastener = model.material("fastener_heads", rgba=(0.03, 0.032, 0.035, 1.0))

    # A compact, wall-mounted vertical base with a projecting root bearing.
    wall = model.part("wall_mount")
    wall.visual(
        Box((0.035, 0.260, 0.340)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        material=powder_coat,
        name="wall_plate",
    )
    wall.visual(
        Box((0.130, 0.095, 0.064)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=powder_coat,
        name="bearing_block",
    )
    wall.visual(
        Cylinder(radius=0.022, length=0.112),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material=fastener,
        name="root_pin",
    )
    for z in (-0.064, 0.064):
        wall.visual(
            Cylinder(radius=0.034, length=0.014),
            origin=Origin(xyz=(0.065, 0.0, z * 0.95)),
            material=fastener,
            name=f"root_cap_{'lower' if z < 0 else 'upper'}",
        )
    for y in (-0.085, 0.085):
        for z in (-0.115, 0.115):
            wall.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(
                    xyz=(-0.024, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=fastener,
                name=f"screw_{'neg' if y < 0 else 'pos'}_{'low' if z < 0 else 'high'}",
            )

    l1 = 0.480
    l2 = 0.420

    proximal = model.part("proximal_link")
    proximal_plate = _link_plate_mesh(
        length=l1,
        width=0.100,
        thickness=0.008,
        pivot_radius=0.027,
        slot_length=0.210,
        slot_width=0.028,
        name="proximal_plate",
    )
    for z, label in ((-0.039, "lower"), (0.039, "upper")):
        proximal.visual(
            proximal_plate,
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=link_finish,
            name=f"{label}_plate",
        )
    for x in (0.150, 0.330):
        for y in (-0.037, 0.037):
            proximal.visual(
                Cylinder(radius=0.010, length=0.088),
                origin=Origin(xyz=(x, y, 0.0)),
                material=fastener,
                name=f"spacer_{int(x * 1000)}_{'neg' if y < 0 else 'pos'}",
            )
    for z in (-0.047, 0.047):
        proximal.visual(
            Cylinder(radius=0.030, length=0.008),
            origin=Origin(xyz=(l1, 0.0, z)),
            material=fastener,
            name=f"elbow_cap_{'lower' if z < 0 else 'upper'}",
        )
    proximal.visual(
        Cylinder(radius=0.018, length=0.104),
        origin=Origin(xyz=(l1, 0.0, 0.0)),
        material=fastener,
        name="elbow_pin",
    )

    distal = model.part("distal_link")
    distal_plate = _link_plate_mesh(
        length=l2,
        width=0.090,
        thickness=0.008,
        pivot_radius=0.026,
        slot_length=0.180,
        slot_width=0.024,
        name="distal_plate",
    )
    for z, label in ((-0.019, "lower"), (0.019, "upper")):
        distal.visual(
            distal_plate,
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=link_finish,
            name=f"{label}_plate",
        )
    for x in (0.135, 0.300):
        for y in (-0.036, 0.036):
            distal.visual(
                Cylinder(radius=0.008, length=0.046),
                origin=Origin(xyz=(x, y, 0.0)),
                material=fastener,
                name=f"spacer_{int(x * 1000)}_{'neg' if y < 0 else 'pos'}",
            )
    distal.visual(
        Box((0.175, 0.018, 0.052)),
        origin=Origin(xyz=(l2 - 0.080, -0.053, 0.0)),
        material=dark_rail,
        name="nose_rail_neg",
    )
    distal.visual(
        Box((0.175, 0.018, 0.052)),
        origin=Origin(xyz=(l2 - 0.080, 0.053, 0.0)),
        material=dark_rail,
        name="nose_rail_pos",
    )

    carriage = model.part("nose_carriage")
    carriage.visual(
        Box((0.400, 0.046, 0.030)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
        material=dark_rail,
        name="slide_bar",
    )
    carriage.visual(
        Box((0.105, 0.070, 0.040)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=link_finish,
        name="nose_block",
    )
    carriage.visual(
        Box((0.012, 0.074, 0.044)),
        origin=Origin(xyz=(0.198, 0.0, 0.0)),
        material=rubber,
        name="front_pad",
    )
    for y in (-0.018, 0.018):
        carriage.visual(
            Cylinder(radius=0.006, length=0.005),
            origin=Origin(xyz=(0.205, y, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener,
            name=f"front_bolt_{'neg' if y < 0 else 'pos'}",
        )

    model.articulation(
        "root_yaw",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=proximal,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=distal,
        # At zero the second link is folded back diagonally; its lower stop
        # straightens the two plate-built links into one long support arm.
        origin=Origin(xyz=(l1, 0.0, 0.0), rpy=(0.0, 0.0, 2.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-2.35, upper=0.35),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=distal,
        child=carriage,
        origin=Origin(xyz=(l2, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=0.200),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_mount")
    proximal = object_model.get_part("proximal_link")
    distal = object_model.get_part("distal_link")
    carriage = object_model.get_part("nose_carriage")
    elbow = object_model.get_articulation("elbow_yaw")
    slide = object_model.get_articulation("nose_slide")

    for plate in ("lower_plate", "upper_plate"):
        ctx.allow_overlap(
            wall,
            proximal,
            elem_a="root_pin",
            elem_b=plate,
            reason="The fixed root hinge pin is intentionally captured through the proximal link plate bore.",
        )
        ctx.expect_overlap(
            wall,
            proximal,
            axes="z",
            elem_a="root_pin",
            elem_b=plate,
            min_overlap=0.006,
            name=f"root pin crosses {plate}",
        )
        ctx.allow_overlap(
            proximal,
            distal,
            elem_a="elbow_pin",
            elem_b=plate,
            reason="The elbow hinge pin is intentionally captured through the distal link plate bore.",
        )
        ctx.expect_overlap(
            proximal,
            distal,
            axes="z",
            elem_a="elbow_pin",
            elem_b=plate,
            min_overlap=0.006,
            name=f"elbow pin crosses distal {plate}",
        )

    with ctx.pose({elbow: -2.35, slide: 0.0}):
        ctx.expect_within(
            carriage,
            distal,
            axes="yz",
            inner_elem="slide_bar",
            margin=0.002,
            name="slide bar is captured between distal plates",
        )
        ctx.expect_overlap(
            carriage,
            distal,
            axes="x",
            elem_a="slide_bar",
            min_overlap=0.180,
            name="collapsed nose carriage remains inserted",
        )

    with ctx.pose({elbow: -2.35, slide: 0.200}):
        ctx.expect_within(
            carriage,
            distal,
            axes="yz",
            inner_elem="slide_bar",
            margin=0.002,
            name="extended slide stays in the distal guide",
        )
        ctx.expect_overlap(
            carriage,
            distal,
            axes="x",
            elem_a="slide_bar",
            min_overlap=0.060,
            name="extended nose carriage retains insertion",
        )

    with ctx.pose({elbow: -2.35, slide: 0.0}):
        straight_tip = ctx.part_element_world_aabb(carriage, elem="front_pad")
        distal_aabb = ctx.part_world_aabb(distal)
    with ctx.pose({elbow: -2.35, slide: 0.200}):
        extended_tip = ctx.part_element_world_aabb(carriage, elem="front_pad")
    ctx.check(
        "nose slide adds reach at straight pose",
        straight_tip is not None
        and extended_tip is not None
        and distal_aabb is not None
        and extended_tip[1][0] > straight_tip[1][0] + 0.18
        and extended_tip[1][0] > distal_aabb[1][0] + 0.25,
        details=f"straight_tip={straight_tip}, extended_tip={extended_tip}, distal={distal_aabb}",
    )

    with ctx.pose({elbow: -2.35}):
        straight_distal = ctx.part_world_aabb(distal)
        straight_proximal = ctx.part_world_aabb(proximal)
    ctx.check(
        "elbow lower stop straightens the links",
        straight_distal is not None
        and straight_proximal is not None
        and straight_distal[1][0] > straight_proximal[1][0] + 0.32,
        details=f"proximal={straight_proximal}, distal={straight_distal}",
    )

    return ctx.report()


object_model = build_object_model()
