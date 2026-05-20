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


MAIN_X = 0.150
MAIN_Y = 0.055
MAIN_Z = 0.066
SIDE_Y = -MAIN_Y / 2.0


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _lens_ring_mesh() -> cq.Workplane:
    """Short hollow knurled ring, centered on the local X rotation axis."""

    outer_radius = 0.024
    # Slightly smaller than the fixed barrel radius; the authored overlap is
    # a hidden bearing/capture proxy and is explicitly allowed in tests.
    inner_radius = 0.0165
    length = 0.018
    ring = (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length, both=True)
    )

    # Raised rubber ribs around the outside so the rotating ring reads as a
    # gripped focus/zoom ring rather than a plain sleeve.
    for idx in range(18):
        angle = idx * 360.0 / 18.0
        rib = (
            cq.Workplane("XY")
            .box(length * 1.02, 0.0031, 0.0042)
            .translate((0.0, 0.0, outer_radius + 0.0017))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle)
        )
        ring = ring.union(rib)
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="palm_camcorder")

    body_mat = model.material("warm_dark_graphite", rgba=(0.09, 0.095, 0.10, 1.0))
    grip_mat = model.material("soft_rubber_black", rgba=(0.015, 0.014, 0.013, 1.0))
    trim_mat = model.material("satin_black_trim", rgba=(0.025, 0.027, 0.030, 1.0))
    screen_mat = model.material("inactive_lcd_glass", rgba=(0.02, 0.08, 0.12, 1.0))
    glass_mat = model.material("coated_lens_glass", rgba=(0.04, 0.12, 0.16, 0.82))
    label_mat = model.material("tiny_white_labels", rgba=(0.82, 0.86, 0.84, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_box((MAIN_X, MAIN_Y, MAIN_Z), 0.010), "rounded_body"),
        material=body_mat,
        name="rounded_body",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box((0.105, 0.030, 0.052), 0.010), "rounded_handgrip"),
        origin=Origin(xyz=(-0.010, 0.040, -0.004)),
        material=grip_mat,
        name="rounded_handgrip",
    )
    body.visual(
        Box((0.076, 0.004, 0.036)),
        origin=Origin(xyz=(-0.010, 0.056, -0.004)),
        material=trim_mat,
        name="grip_strap_pad",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.007),
        origin=Origin(xyz=(MAIN_X / 2.0 + 0.0035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_mat,
        name="lens_shoulder",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(MAIN_X / 2.0 + 0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_mat,
        name="lens_barrel",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=(MAIN_X / 2.0 + 0.0515, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_mat,
        name="front_glass",
    )
    body.visual(
        Box((0.010, 0.0065, 0.064)),
        origin=Origin(xyz=(-0.055, SIDE_Y - 0.00325, 0.0)),
        material=trim_mat,
        name="monitor_hinge_pedestal",
    )
    body.visual(
        Box((0.046, 0.0033, 0.005)),
        origin=Origin(xyz=(-0.012, SIDE_Y - 0.00065, -0.026)),
        material=trim_mat,
        name="port_hinge_ledge",
    )
    body.visual(
        Box((0.020, 0.0015, 0.006)),
        origin=Origin(xyz=(-0.012, SIDE_Y - 0.00075, -0.006)),
        material=label_mat,
        name="port_label",
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        mesh_from_cadquery(_lens_ring_mesh(), "knurled_lens_ring", tolerance=0.0006),
        material=grip_mat,
        name="knurled_ring",
    )

    monitor = model.part("side_monitor")
    monitor.visual(
        Cylinder(radius=0.0035, length=0.058),
        origin=Origin(rpy=(0.0, 0.0, 0.0)),
        material=trim_mat,
        name="monitor_hinge_barrel",
    )
    monitor.visual(
        Box((0.011, 0.004, 0.052)),
        origin=Origin(xyz=(0.0055, -0.0040, 0.0)),
        material=trim_mat,
        name="monitor_hinge_leaf",
    )
    monitor.visual(
        Box((0.074, 0.006, 0.056)),
        origin=Origin(xyz=(0.037, -0.0080, 0.0)),
        material=body_mat,
        name="monitor_frame",
    )
    monitor.visual(
        Box((0.060, 0.001, 0.042)),
        origin=Origin(xyz=(0.041, -0.0115, 0.0)),
        material=screen_mat,
        name="lcd_screen",
    )

    port_cover = model.part("port_cover")
    port_cover.visual(
        Cylinder(radius=0.0022, length=0.044),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_mat,
        name="cover_hinge_barrel",
    )
    port_cover.visual(
        Box((0.040, 0.0030, 0.0060)),
        origin=Origin(xyz=(0.0, -0.0027, 0.0035)),
        material=trim_mat,
        name="cover_hinge_leaf",
    )
    port_cover.visual(
        Box((0.040, 0.0040, 0.0220)),
        origin=Origin(xyz=(0.0, -0.0050, 0.0132)),
        material=body_mat,
        name="cover_panel",
    )
    port_cover.visual(
        Box((0.014, 0.0010, 0.0050)),
        origin=Origin(xyz=(0.0, -0.0075, 0.0150)),
        material=grip_mat,
        name="cover_pull_lip",
    )

    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(MAIN_X / 2.0 + 0.037, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )
    model.articulation(
        "body_to_side_monitor",
        ArticulationType.REVOLUTE,
        parent=body,
        child=monitor,
        origin=Origin(xyz=(-0.055, SIDE_Y - 0.0100, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.5, lower=0.0, upper=1.62),
    )
    model.articulation(
        "body_to_port_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=port_cover,
        origin=Origin(xyz=(-0.012, SIDE_Y - 0.0045, -0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens_ring = object_model.get_part("lens_ring")
    monitor = object_model.get_part("side_monitor")
    port_cover = object_model.get_part("port_cover")
    ring_joint = object_model.get_articulation("body_to_lens_ring")
    monitor_joint = object_model.get_articulation("body_to_side_monitor")
    cover_joint = object_model.get_articulation("body_to_port_cover")

    ctx.allow_overlap(
        body,
        lens_ring,
        elem_a="lens_barrel",
        elem_b="knurled_ring",
        reason="The rotating ring is intentionally captured on a simplified solid barrel by a hidden bearing fit.",
    )

    ctx.expect_within(
        body,
        lens_ring,
        axes="yz",
        inner_elem="lens_barrel",
        outer_elem="knurled_ring",
        margin=0.0025,
        name="lens ring surrounds the fixed barrel radially",
    )
    ctx.expect_overlap(
        lens_ring,
        body,
        axes="x",
        elem_a="knurled_ring",
        elem_b="lens_barrel",
        min_overlap=0.014,
        name="lens ring sits on the short barrel length",
    )

    ctx.expect_gap(
        body,
        monitor,
        axis="y",
        positive_elem="monitor_hinge_pedestal",
        negative_elem="monitor_hinge_barrel",
        max_gap=0.001,
        max_penetration=0.0005,
        name="side monitor barrel seats against its body pedestal",
    )
    ctx.expect_overlap(
        monitor,
        body,
        axes="z",
        elem_a="monitor_hinge_barrel",
        elem_b="monitor_hinge_pedestal",
        min_overlap=0.050,
        name="side monitor uses a tall vertical hinge line",
    )

    ctx.expect_gap(
        body,
        port_cover,
        axis="y",
        positive_elem="port_hinge_ledge",
        negative_elem="cover_hinge_barrel",
        max_gap=0.001,
        max_penetration=0.0005,
        name="port cover barrel is carried by the side-wall ledge",
    )
    ctx.expect_overlap(
        port_cover,
        body,
        axes="x",
        elem_a="cover_hinge_barrel",
        elem_b="port_hinge_ledge",
        min_overlap=0.038,
        name="port cover hinge runs along the cover bottom edge",
    )

    rest_monitor_aabb = ctx.part_world_aabb(monitor)
    rest_cover_aabb = ctx.part_world_aabb(port_cover)
    with ctx.pose({monitor_joint: 1.25, cover_joint: 1.0, ring_joint: math.pi}):
        open_monitor_aabb = ctx.part_world_aabb(monitor)
        open_cover_aabb = ctx.part_world_aabb(port_cover)

    ctx.check(
        "side monitor rotates outward away from the body side",
        rest_monitor_aabb is not None
        and open_monitor_aabb is not None
        and open_monitor_aabb[0][1] < rest_monitor_aabb[0][1] - 0.025,
        details=f"rest={rest_monitor_aabb}, open={open_monitor_aabb}",
    )
    ctx.check(
        "bottom hinged port cover swings outward and downward",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][1] < rest_cover_aabb[0][1] - 0.006
        and open_cover_aabb[1][2] < rest_cover_aabb[1][2] - 0.004,
        details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
