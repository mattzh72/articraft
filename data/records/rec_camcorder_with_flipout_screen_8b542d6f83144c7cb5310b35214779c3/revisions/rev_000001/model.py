from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _circle_profile(radius: float, segments: int = 80) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_recording_camcorder")

    charcoal = model.material("charcoal_magnesium", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    dark_gray = model.material("dark_graphite_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    satin = model.material("satin_black_ring", rgba=(0.02, 0.022, 0.024, 1.0))
    glass = model.material("coated_blue_glass", rgba=(0.03, 0.08, 0.12, 0.72))
    screen = model.material("glossy_inactive_lcd", rgba=(0.01, 0.015, 0.018, 1.0))
    label = model.material("soft_gray_markings", rgba=(0.62, 0.64, 0.66, 1.0))

    body_length = 0.160
    body_width = 0.075
    body_height = 0.085
    front_x = body_length / 2.0
    rear_x = -body_length / 2.0
    side_y = body_width / 2.0

    body = model.part("body")
    body_shell = cq.Workplane("XY").box(body_length, body_width, body_height).edges().fillet(0.006)
    body.visual(
        mesh_from_cadquery(body_shell, "camcorder_body_shell", tolerance=0.0007),
        material=charcoal,
        name="main_shell",
    )
    body.visual(
        Box((0.118, 0.020, 0.012)),
        origin=Origin(xyz=(-0.006, -0.010, body_height / 2.0 + 0.0045)),
        material=dark_gray,
        name="top_handgrip",
    )
    body.visual(
        Box((0.070, 0.0025, 0.052)),
        origin=Origin(xyz=(-0.010, side_y + 0.0008, 0.0)),
        material=dark_gray,
        name="display_recess",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(front_x + 0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(front_x + 0.0015, -0.026, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_sensor_window",
    )

    eyecup = BezelGeometry(
        (0.024, 0.016),
        (0.046, 0.033),
        0.018,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.004,
        outer_corner_radius=0.008,
        center=False,
    )
    body.visual(
        mesh_from_geometry(eyecup, "rear_rubber_eyecup"),
        origin=Origin(xyz=(rear_x, -0.006, 0.004), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=black,
        name="rear_eyecup",
    )

    hinge_x = 0.052
    hinge_y = side_y + 0.0025
    body.visual(
        Box((0.010, 0.006, 0.070)),
        origin=Origin(xyz=(hinge_x, side_y + 0.0012, 0.0)),
        material=charcoal,
        name="side_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.070),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        material=satin,
        name="side_hinge_barrel",
    )

    collar = model.part("zoom_collar")
    collar_length = 0.026
    collar_ring = ExtrudeWithHolesGeometry(
        _circle_profile(0.031, 96),
        (_circle_profile(0.020, 96),),
        collar_length,
        center=True,
    )
    collar.visual(
        mesh_from_geometry(collar_ring, "knurled_zoom_collar_ring"),
        origin=Origin(xyz=(collar_length / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="collar_ring",
    )
    rib_count = 18
    for i in range(rib_count):
        theta = 2.0 * math.pi * i / rib_count
        radial = 0.0315
        collar.visual(
            Box((0.023, 0.0032, 0.0042)),
            origin=Origin(
                xyz=(collar_length / 2.0, radial * math.cos(theta), radial * math.sin(theta)),
                rpy=(theta - math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_gray,
            name=f"grip_rib_{i}",
        )
    collar.visual(
        Cylinder(radius=0.0205, length=0.0024),
        origin=Origin(xyz=(collar_length + 0.0004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens_glass",
    )
    collar.visual(
        Cylinder(radius=0.012, length=0.0015),
        origin=Origin(xyz=(collar_length + 0.0018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="deep_lens_core",
    )
    model.articulation(
        "body_to_zoom_collar",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=collar,
        origin=Origin(xyz=(front_x + 0.008, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    display = model.part("display_door")
    display.visual(
        Box((0.106, 0.006, 0.064)),
        origin=Origin(xyz=(-0.053, 0.004, 0.0)),
        material=dark_gray,
        name="door_panel",
    )
    display.visual(
        Box((0.087, 0.0012, 0.047)),
        origin=Origin(xyz=(-0.056, 0.0076, 0.001)),
        material=screen,
        name="lcd_screen",
    )
    display.visual(
        Box((0.092, 0.0014, 0.052)),
        origin=Origin(xyz=(-0.056, 0.0068, 0.001)),
        material=black,
        name="lcd_bezel",
    )
    display.visual(
        Cylinder(radius=0.0034, length=0.066),
        origin=Origin(xyz=(-0.0005, 0.004, 0.0)),
        material=satin,
        name="door_hinge_barrel",
    )
    display.visual(
        Box((0.012, 0.0035, 0.048)),
        origin=Origin(xyz=(-0.006, 0.003, 0.0)),
        material=dark_gray,
        name="door_hinge_leaf",
    )
    display.visual(
        Box((0.030, 0.0010, 0.0022)),
        origin=Origin(xyz=(-0.056, 0.0084, -0.021)),
        material=label,
        name="screen_status_mark",
    )
    model.articulation(
        "body_to_display_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.2, lower=0.0, upper=1.75),
    )

    latch = model.part("battery_latch")
    latch.visual(
        Box((0.0030, 0.026, 0.010)),
        origin=Origin(xyz=(-0.0015, 0.011, 0.0)),
        material=dark_gray,
        name="latch_plate",
    )
    latch.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(-0.0020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="latch_pivot_cap",
    )
    latch.visual(
        Box((0.0008, 0.014, 0.0014)),
        origin=Origin(xyz=(-0.0033, 0.012, 0.0025)),
        material=label,
        name="latch_mark",
    )
    model.articulation(
        "body_to_battery_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(rear_x, 0.021, -0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    display = object_model.get_part("display_door")
    collar = object_model.get_part("zoom_collar")
    latch = object_model.get_part("battery_latch")
    display_hinge = object_model.get_articulation("body_to_display_door")
    latch_pivot = object_model.get_articulation("body_to_battery_latch")

    ctx.allow_overlap(
        body,
        display,
        elem_a="side_hinge_barrel",
        elem_b="door_hinge_barrel",
        reason="The LCD-door hinge barrel is intentionally captured around the side hinge pin proxy.",
    )
    ctx.expect_overlap(
        body,
        display,
        axes="z",
        elem_a="side_hinge_barrel",
        elem_b="door_hinge_barrel",
        min_overlap=0.055,
        name="display hinge barrels share a vertical pin span",
    )

    ctx.expect_gap(
        display,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="main_shell",
        min_gap=0.0003,
        max_gap=0.004,
        name="closed display door sits just proud of the side shell",
    )
    ctx.expect_overlap(
        display,
        body,
        axes="xz",
        elem_a="door_panel",
        elem_b="display_recess",
        min_overlap=0.045,
        name="display door covers the broad side recess",
    )
    ctx.expect_gap(
        collar,
        body,
        axis="x",
        positive_elem="collar_ring",
        negative_elem="lens_mount",
        max_gap=0.0015,
        max_penetration=0.000001,
        name="zoom collar seats on the front lens mount",
    )
    ctx.expect_gap(
        body,
        latch,
        axis="x",
        positive_elem="main_shell",
        negative_elem="latch_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="battery latch plate rests on the rear body face",
    )

    closed_display_aabb = ctx.part_element_world_aabb(display, elem="door_panel")
    with ctx.pose({display_hinge: 1.35}):
        open_display_aabb = ctx.part_element_world_aabb(display, elem="door_panel")
    ctx.check(
        "display door rotates outward from side",
        closed_display_aabb is not None
        and open_display_aabb is not None
        and open_display_aabb[1][1] > closed_display_aabb[1][1] + 0.045,
        details=f"closed={closed_display_aabb}, open={open_display_aabb}",
    )

    closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_plate")
    with ctx.pose({latch_pivot: 0.55}):
        open_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_plate")
    ctx.check(
        "battery release latch swings on rear pivot",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][2] > closed_latch_aabb[1][2] + 0.003,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
