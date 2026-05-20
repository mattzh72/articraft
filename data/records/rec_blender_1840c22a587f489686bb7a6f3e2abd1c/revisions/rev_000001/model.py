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


STEEL = Material("brushed_steel", rgba=(0.72, 0.74, 0.73, 1.0))
DARK = Material("soft_black", rgba=(0.01, 0.012, 0.014, 1.0))
RUBBER = Material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
GLASS = Material("smoky_clear_jar", rgba=(0.72, 0.90, 1.0, 0.36))
BUTTON = Material("satin_button", rgba=(0.18, 0.19, 0.20, 1.0))
BLADE = Material("polished_blade", rgba=(0.86, 0.87, 0.84, 1.0))


def _tapered_base() -> cq.Workplane:
    """Lofted square-to-square blender motor base, authored in meters."""
    return (
        cq.Workplane("XY")
        .rect(0.34, 0.30)
        .workplane(offset=0.32)
        .rect(0.24, 0.22)
        .loft(combine=True)
    )


def _jar_shell() -> cq.Workplane:
    """Open, hollow cylindrical jar shell with a thick transparent bottom."""
    outer_radius = 0.140
    inner_radius = 0.124
    height = 0.480
    bottom_thickness = 0.026
    shell = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner_void = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.004)
        .translate((0.0, 0.0, bottom_thickness))
    )
    return shell.cut(inner_void)


def _vented_lid() -> cq.Workplane:
    """Low black lid with a central filler opening and eight radial vents."""
    lid = cq.Workplane("XY").circle(0.148).extrude(0.040)
    center_cut = (
        cq.Workplane("XY")
        .circle(0.040)
        .extrude(0.060)
        .translate((0.0, 0.0, -0.010))
    )
    lid = lid.cut(center_cut)

    for i in range(8):
        angle = i * 45.0
        slot = (
            cq.Workplane("XY")
            .box(0.058, 0.011, 0.060)
            .translate((0.090, 0.0, 0.020))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        lid = lid.cut(slot)
    return lid


def _six_blade_star() -> cq.Workplane:
    """Six pitched stainless blades welded into a central hub."""
    star = cq.Workplane("XY").circle(0.028).extrude(0.030).translate((0.0, 0.0, -0.015))
    star = star.union(cq.Workplane("XY").circle(0.015).extrude(0.018).translate((0.0, 0.0, 0.015)))

    for i in range(6):
        angle = i * 60.0
        pitch = 16.0 if i % 2 == 0 else -13.0
        z_lift = 0.004 if i % 2 == 0 else -0.004
        blade = (
            cq.Workplane("XY")
            .polyline(
                [
                    (0.018, -0.010),
                    (0.096, -0.020),
                    (0.108, 0.000),
                    (0.096, 0.020),
                    (0.018, 0.010),
                ]
            )
            .close()
            .extrude(0.004)
            .translate((0.0, 0.0, -0.002 + z_lift))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), pitch)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        star = star.union(blade)

    return star


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_performance_blender")

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tapered_base(), "tapered_base", tolerance=0.001),
        material=STEEL,
        name="tapered_base",
    )
    body.visual(
        Cylinder(radius=0.130, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=DARK,
        name="jar_collar",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.388)),
        material=STEEL,
        name="drive_socket",
    )
    body.visual(
        mesh_from_cadquery(_jar_shell(), "hollow_jar", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=GLASS,
        name="jar_shell",
    )
    body.visual(
        Cylinder(radius=0.145, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.8525)),
        material=RUBBER,
        name="lid_gasket",
    )
    body.visual(
        mesh_from_cadquery(_vented_lid(), "vented_lid", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.865)),
        material=RUBBER,
        name="lid_shell",
    )
    body.visual(
        Box((0.030, 0.025, 0.265)),
        origin=Origin(xyz=(0.0, 0.190, 0.610)),
        material=DARK,
        name="rear_grip",
    )
    body.visual(
        Box((0.032, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, 0.158, 0.735)),
        material=DARK,
        name="upper_handle_mount",
    )
    body.visual(
        Box((0.032, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, 0.158, 0.485)),
        material=DARK,
        name="lower_handle_mount",
    )
    body.visual(
        Box((0.180, 0.036, 0.150)),
        origin=Origin(xyz=(0.0, -0.148, 0.155)),
        material=DARK,
        name="control_panel",
    )
    for x in (-0.115, 0.115):
        for y in (-0.100, 0.100):
            body.visual(
                Cylinder(radius=0.030, length=0.018),
                origin=Origin(xyz=(x, y, -0.007)),
                material=RUBBER,
                name=f"rubber_foot_{x}_{y}",
            )
    for x in (-0.039, 0.039):
        body.visual(
            Box((0.018, 0.026, 0.008)),
            origin=Origin(xyz=(x, 0.065, 0.909)),
            material=RUBBER,
            name=f"cap_hinge_pad_{x}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.018),
            origin=Origin(xyz=(x, 0.047, 0.914), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=RUBBER,
            name=f"cap_hinge_lug_{x}",
        )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_six_blade_star(), "six_blade_star", tolerance=0.0005),
        material=BLADE,
        name="blade_star",
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=500.0),
    )

    cap = model.part("center_cap")
    cap.visual(
        Cylinder(radius=0.045, length=0.016),
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        material=DARK,
        name="cap_disk",
    )
    cap.visual(
        Cylinder(radius=0.007, length=0.052),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=DARK,
        name="cap_barrel",
    )
    cap.visual(
        Box((0.034, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, -0.018, 0.004)),
        material=DARK,
        name="cap_strap",
    )
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.047, 0.913)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.65),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.034, length=0.035),
        origin=Origin(xyz=(0.0, -0.0175, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BUTTON,
        name="dial_face",
    )
    dial.visual(
        Box((0.006, 0.009, 0.034)),
        origin=Origin(xyz=(0.0, -0.037, 0.0)),
        material=STEEL,
        name="dial_pointer",
    )
    model.articulation(
        "dial_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, -0.166, 0.172)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-2.4, upper=2.4),
    )

    for i, x in enumerate((-0.055, 0.055)):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.042, 0.020, 0.028)),
            origin=Origin(xyz=(0.0, -0.010, 0.0)),
            material=BUTTON,
            name="button_cap",
        )
        model.articulation(
            f"button_press_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.166, 0.090)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    blade = object_model.get_part("blade")
    cap = object_model.get_part("center_cap")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")

    blade_spin = object_model.get_articulation("blade_spin")
    cap_hinge = object_model.get_articulation("cap_hinge")
    button_press = object_model.get_articulation("button_press_0")

    ctx.expect_within(
        blade,
        body,
        axes="xy",
        inner_elem="blade_star",
        outer_elem="jar_shell",
        margin=0.010,
        name="six blade assembly fits inside jar radius",
    )
    ctx.expect_overlap(
        body,
        cap,
        axes="xy",
        elem_a="lid_shell",
        elem_b="cap_disk",
        min_overlap=0.030,
        name="center cap covers filler opening",
    )
    ctx.expect_gap(
        cap,
        body,
        axis="z",
        positive_elem="cap_disk",
        negative_elem="lid_shell",
        max_gap=0.001,
        max_penetration=0.001,
        name="center cap rests on lid",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_face",
        elem_b="control_panel",
        contact_tol=0.002,
        name="speed dial is mounted on front panel",
    )

    closed_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_disk")
    with ctx.pose({cap_hinge: 1.20}):
        open_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_disk")
    ctx.check(
        "flip cap opens upward",
        closed_cap_aabb is not None
        and open_cap_aabb is not None
        and open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.040,
        details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
    )

    rest_button_pos = ctx.part_world_position(button_0)
    with ctx.pose({button_press: 0.006}):
        pressed_button_pos = ctx.part_world_position(button_0)
    ctx.check(
        "front button travels inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.004,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    ctx.check(
        "blade joint is vertical continuous spin",
        tuple(round(v, 3) for v in blade_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={blade_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
