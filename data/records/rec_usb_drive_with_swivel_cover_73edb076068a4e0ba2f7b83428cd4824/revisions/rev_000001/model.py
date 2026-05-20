from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    """Small meter-scale rounded rectangular solid."""
    solid = cq.Workplane("XY").box(length, width, height)
    try:
        solid = solid.edges("|Z").fillet(radius)
        solid = solid.edges(">Z or <Z").fillet(min(radius * 0.35, height * 0.18))
    except Exception:
        # Very small geometry can make cosmetic fillets fragile; the base box is
        # still dimensionally correct if CadQuery refuses a fillet.
        pass
    return solid


def _usb_connector_shell(length: float, width: float, height: float, wall: float) -> cq.Workplane:
    """A hollow USB-A metal sleeve, open at the plug face."""
    outer = cq.Workplane("XY").box(length, width, height)
    inner = cq.Workplane("XY").box(length + 0.003, width - 2.0 * wall, height - 2.0 * wall)
    shell = outer.cut(inner)
    try:
        shell = shell.edges("|X").fillet(0.00035)
    except Exception:
        pass
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_drive_swivel_cover")

    body_black = model.material("body_black", rgba=(0.030, 0.034, 0.040, 1.0))
    grip_dark = model.material("grip_dark", rgba=(0.075, 0.080, 0.090, 1.0))
    label_blue = model.material("label_blue", rgba=(0.10, 0.26, 0.62, 1.0))
    stamped_steel = model.material("stamped_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    brushed_pin = model.material("brushed_pin", rgba=(0.82, 0.80, 0.76, 1.0))
    connector_metal = model.material("connector_metal", rgba=(0.78, 0.78, 0.74, 1.0))
    port_plastic = model.material("port_plastic", rgba=(0.03, 0.04, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_box(0.052, 0.024, 0.010, 0.0030), "usb_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0060)),
        material=body_black,
        name="body_shell",
    )
    body.visual(
        Box((0.034, 0.018, 0.0012)),
        origin=Origin(xyz=(-0.0005, 0.0, 0.01135)),
        material=label_blue,
        name="top_label",
    )
    body.visual(
        Box((0.022, 0.0011, 0.0022)),
        origin=Origin(xyz=(-0.004, 0.01225, 0.0070)),
        material=grip_dark,
        name="side_grip_0",
    )
    body.visual(
        Box((0.022, 0.0011, 0.0022)),
        origin=Origin(xyz=(-0.004, -0.01225, 0.0070)),
        material=grip_dark,
        name="side_grip_1",
    )
    body.visual(
        Box((0.012, 0.004, 0.0010)),
        origin=Origin(xyz=(-0.023, 0.0, 0.0010)),
        material=grip_dark,
        name="lanyard_slot",
    )

    body.visual(
        mesh_from_cadquery(_usb_connector_shell(0.022, 0.0140, 0.0058, 0.00075), "usb_connector_shell"),
        origin=Origin(xyz=(0.0365, 0.0, 0.0061)),
        material=connector_metal,
        name="connector_shell",
    )
    body.visual(
        Box((0.023, 0.0067, 0.0012)),
        origin=Origin(xyz=(0.0365, 0.0, 0.0047)),
        material=port_plastic,
        name="connector_tongue",
    )
    body.visual(
        Cylinder(radius=0.00255, length=0.040),
        origin=Origin(xyz=(-0.0255, 0.0, 0.0060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0015),
        origin=Origin(xyz=(-0.0255, 0.0193, 0.0060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="pin_head_0",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0015),
        origin=Origin(xyz=(-0.0255, -0.0193, 0.0060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="pin_head_1",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.075, 0.028, 0.016)),
        mass=0.018,
        origin=Origin(xyz=(0.010, 0.0, 0.0065)),
    )

    cover = model.part("cover")
    cover.visual(
        Box((0.0770, 0.0340, 0.0030)),
        origin=Origin(xyz=(0.0415, 0.0, 0.0085)),
        material=stamped_steel,
        name="top_sheet",
    )
    cover.visual(
        Box((0.0750, 0.0030, 0.0115)),
        origin=Origin(xyz=(0.0435, 0.0155, 0.0017)),
        material=stamped_steel,
        name="side_rail_0",
    )
    cover.visual(
        Box((0.0750, 0.0030, 0.0115)),
        origin=Origin(xyz=(0.0435, -0.0155, 0.0017)),
        material=stamped_steel,
        name="side_rail_1",
    )
    cover.visual(
        Box((0.0055, 0.0340, 0.0150)),
        origin=Origin(xyz=(0.0790, 0.0, 0.0017)),
        material=stamped_steel,
        name="front_plate",
    )
    cover.visual(
        Cylinder(radius=0.0043, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0155, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stamped_steel,
        name="pivot_bushing_0",
    )
    cover.visual(
        Box((0.0090, 0.0030, 0.0012)),
        origin=Origin(xyz=(0.0045, 0.0155, 0.0042)),
        material=stamped_steel,
        name="pivot_ear_0",
    )
    cover.visual(
        Cylinder(radius=0.0043, length=0.0030),
        origin=Origin(xyz=(0.0, -0.0155, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stamped_steel,
        name="pivot_bushing_1",
    )
    cover.visual(
        Box((0.0090, 0.0030, 0.0012)),
        origin=Origin(xyz=(0.0045, -0.0155, 0.0042)),
        material=stamped_steel,
        name="pivot_ear_1",
    )
    cover.visual(
        Box((0.018, 0.018, 0.0008)),
        origin=Origin(xyz=(0.0740, 0.0, 0.01035)),
        material=brushed_pin,
        name="front_stamp",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.085, 0.035, 0.016)),
        mass=0.006,
        origin=Origin(xyz=(0.040, 0.0, 0.004)),
    )

    model.articulation(
        "cover_pivot",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.0255, 0.0, 0.0060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=10.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    pivot = object_model.get_articulation("cover_pivot")

    for bushing_name in ("pivot_bushing_0", "pivot_bushing_1"):
        ctx.allow_overlap(
            body,
            cover,
            elem_a="pivot_pin",
            elem_b=bushing_name,
            reason="The swivel cover bushing is intentionally captured around the side pivot pin.",
        )
        ctx.expect_contact(
            body,
            cover,
            elem_a="pivot_pin",
            elem_b=bushing_name,
            name=f"{bushing_name} is carried by the pivot pin",
        )

    ctx.check(
        "cover uses continuous swivel joint",
        pivot.articulation_type == ArticulationType.CONTINUOUS and tuple(pivot.axis) == (0.0, -1.0, 0.0),
        details=f"type={pivot.articulation_type}, axis={pivot.axis}",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        min_overlap=0.020,
        name="u cover wraps over the compact body footprint",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="x",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem="front_plate",
        negative_elem="connector_shell",
        name="front cover bridge sits just beyond the connector",
    )

    rest_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({pivot: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(cover)

    ctx.check(
        "cover swings away from front face",
        rest_aabb is not None
        and turned_aabb is not None
        and turned_aabb[1][2] > rest_aabb[1][2] + 0.030,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
