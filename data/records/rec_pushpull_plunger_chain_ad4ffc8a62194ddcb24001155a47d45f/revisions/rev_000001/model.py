from __future__ import annotations

from math import pi, sqrt

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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _tube_x(
    length: float,
    outer_radius: float,
    inner_radius: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length * 0.5, both=True)
        .translate(center)
    )


def _cylinder_x(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length * 0.5, both=True).translate(center)


def _bar_xz(
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    thickness: float,
) -> cq.Workplane:
    sx, sz = start
    ex, ez = end
    dx = ex - sx
    dz = ez - sz
    length = sqrt(dx * dx + dz * dz)
    nx = -dz / length * width * 0.5
    nz = dx / length * width * 0.5
    points = [
        (sx + nx, sz + nz),
        (ex + nx, ez + nz),
        (ex - nx, ez - nz),
        (sx - nx, sz - nz),
    ]
    return cq.Workplane("XZ").polyline(points).close().extrude(thickness * 0.5, both=True)


def _support_frame_mesh() -> cq.Workplane:
    """One continuous rear frame, fork yoke, and base with a plunger clearance bore."""
    body = _box((0.52, 0.18, 0.024), (0.01, 0.0, 0.012))
    body = body.union(_box((0.030, 0.18, 0.230), (-0.190, 0.0, 0.139)))
    body = body.union(_box((0.228, 0.016, 0.168), (0.092, 0.055, 0.109)))
    body = body.union(_box((0.228, 0.016, 0.168), (0.092, -0.055, 0.109)))
    body = body.union(_box((0.032, 0.128, 0.022), (-0.012, 0.0, 0.204)))
    body = body.union(_box((0.045, 0.128, 0.038), (0.036, 0.0, 0.043)))
    body = body.union(_box((0.020, 0.145, 0.055), (-0.045, 0.0, 0.052)))

    # The rear plate is actually drilled; the plunger shaft passes through this
    # bore and the sleeve wall ties into the remaining annulus.
    body = body.cut(_cylinder_x(0.42, 0.019, (-0.080, 0.0, 0.105)))
    return body


def _guide_sleeve_mesh() -> cq.Workplane:
    sleeve = _tube_x(0.250, 0.027, 0.017, (-0.085, 0.0, 0.105))
    front_collar = _tube_x(0.036, 0.038, 0.018, (0.034, 0.0, 0.105))
    rear_flange = _tube_x(0.018, 0.034, 0.018, (-0.202, 0.0, 0.105))
    return sleeve.union(front_collar).union(rear_flange)


def _lever_plate_mesh() -> cq.Workplane:
    plate = _bar_xz((0.0, 0.0), (0.066, 0.124), 0.030, 0.026)
    plate = plate.union(_bar_xz((0.0, 0.0), (-0.044, -0.040), 0.028, 0.026))
    plate = plate.union(cq.Workplane("XZ").circle(0.028).extrude(0.013, both=True))
    plate = plate.union(
        cq.Workplane("XZ").center(0.066, 0.124).circle(0.020).extrude(0.013, both=True)
    )
    plate = plate.union(
        cq.Workplane("XZ").center(-0.044, -0.040).circle(0.014).extrude(0.013, both=True)
    )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_plunger_chain")

    dark_frame = Material("dark phosphate frame", color=(0.10, 0.11, 0.12, 1.0))
    worn_steel = Material("brushed steel", color=(0.62, 0.64, 0.62, 1.0))
    oiled_steel = Material("oiled steel", color=(0.34, 0.36, 0.35, 1.0))
    black = Material("black oxide", color=(0.015, 0.015, 0.014, 1.0))
    brass = Material("brass bearing", color=(0.76, 0.58, 0.25, 1.0))

    support = model.part("support_frame")
    support.visual(
        mesh_from_cadquery(_support_frame_mesh(), "support_frame"),
        material=dark_frame,
        name="frame",
    )
    support.visual(
        mesh_from_cadquery(_guide_sleeve_mesh(), "guide_sleeve"),
        material=oiled_steel,
        name="guide_sleeve",
    )
    for idx, y in enumerate((0.066, -0.066)):
        support.visual(
            Cylinder(radius=0.025, length=0.012),
            origin=Origin(xyz=(0.135, y, 0.145), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"pivot_bushing_{idx}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.017, length=0.305),
        origin=Origin(xyz=(-0.0925, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=worn_steel,
        name="shaft",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=worn_steel,
        name="front_tip",
    )
    plunger.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(-0.252, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="rear_stop",
    )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_lever_plate_mesh(), "lever_plate"),
        material=black,
        name="lever_plate",
    )
    lever.visual(
        Cylinder(radius=0.023, length=0.094),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="pivot_hub",
    )
    lever.visual(
        Box((0.022, 0.064, 0.035)),
        origin=Origin(xyz=(-0.0535, 0.0, -0.040)),
        material=brass,
        name="drive_pad",
    )

    model.articulation(
        "sleeve_to_plunger",
        ArticulationType.PRISMATIC,
        parent=support,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.025),
    )
    model.articulation(
        "fork_to_lever",
        ArticulationType.REVOLUTE,
        parent=support,
        child=lever,
        origin=Origin(xyz=(0.135, 0.0, 0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=0.48),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    plunger = object_model.get_part("plunger")
    lever = object_model.get_part("lever")
    slide = object_model.get_articulation("sleeve_to_plunger")
    hinge = object_model.get_articulation("fork_to_lever")

    ctx.check(
        "one prismatic plunger and one revolute lever",
        len(object_model.articulations) == 2
        and slide.articulation_type == ArticulationType.PRISMATIC
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={[a.name for a in object_model.articulations]}",
    )
    ctx.allow_overlap(
        plunger,
        support,
        elem_a="shaft",
        elem_b="guide_sleeve",
        reason=(
            "The plunger shaft is intentionally represented as a close sliding "
            "fit inside the guide sleeve proxy."
        ),
    )
    ctx.allow_overlap(
        lever,
        support,
        elem_a="pivot_hub",
        elem_b="frame",
        reason=(
            "The lever hub is intentionally captured through the fork cheek "
            "bearing proxy at the revolute pivot."
        ),
    )

    ctx.expect_within(
        plunger,
        support,
        axes="yz",
        inner_elem="shaft",
        outer_elem="guide_sleeve",
        margin=0.002,
        name="shaft is centered through guide sleeve",
    )
    ctx.expect_overlap(
        plunger,
        support,
        axes="x",
        elem_a="shaft",
        elem_b="guide_sleeve",
        min_overlap=0.20,
        name="plunger remains captured in sleeve at rest",
    )
    ctx.expect_gap(
        lever,
        plunger,
        axis="x",
        positive_elem="drive_pad",
        negative_elem="front_tip",
        min_gap=0.0,
        max_gap=0.001,
        name="plunger tip is poised at lever root",
    )
    ctx.expect_within(
        lever,
        support,
        axes="y",
        inner_elem="pivot_hub",
        outer_elem="frame",
        margin=0.0,
        name="lever hub sits between fork cheeks",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.025}):
        ctx.expect_overlap(
            plunger,
            support,
            axes="x",
            elem_a="shaft",
            elem_b="guide_sleeve",
            min_overlap=0.20,
            name="extended plunger remains guided",
        )
        extended_pos = ctx.part_world_position(plunger)
    ctx.check(
        "prismatic joint drives plunger forward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.020,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_pad = ctx.part_element_world_aabb(lever, elem="drive_pad")
    with ctx.pose({hinge: 0.36}):
        swung_pad = ctx.part_element_world_aabb(lever, elem="drive_pad")
    ctx.check(
        "positive lever rotation moves the driven root forward",
        rest_pad is not None and swung_pad is not None and swung_pad[0][0] > rest_pad[0][0] + 0.004,
        details=f"rest_pad={rest_pad}, swung_pad={swung_pad}",
    )

    return ctx.report()


object_model = build_object_model()
