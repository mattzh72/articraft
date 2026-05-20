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


BODY_X = 0.46
BODY_Y = 0.32
BODY_Z = 0.075
HINGE_X = BODY_X / 2 - 0.012
HINGE_Z = BODY_Z + 0.010
SLOT_X = -BODY_X / 2
SLOT_Z = 0.036


def _scanner_body_shell() -> cq.Workplane:
    """Plastic scanner base with a shallow glass recess and an open front tray slot."""
    shell = cq.Workplane("XY").box(BODY_X, BODY_Y, BODY_Z).translate((0.0, 0.0, BODY_Z / 2))

    # A through-tunnel from the front face for the film holder tray.
    slot = cq.Workplane("XY").box(0.410, 0.264, 0.026).translate((-0.032, 0.0, SLOT_Z))
    shell = shell.cut(slot)

    # Recess for the top scan glass, leaving a molded rim around it.
    glass_recess = cq.Workplane("XY").box(0.356, 0.236, 0.010).translate((-0.020, 0.0, BODY_Z - 0.002))
    shell = shell.cut(glass_recess)

    return shell


def _film_tray_frame() -> cq.Workplane:
    """A single connected film strip holder tray with six rectangular exposure windows."""
    tray = cq.Workplane("XY").box(0.380, 0.220, 0.006)
    for x in (-0.1275, -0.0765, -0.0255, 0.0255, 0.0765, 0.1275):
        window = cq.Workplane("XY").box(0.040, 0.132, 0.014).translate((x, 0.0, 0.0))
        tray = tray.cut(window)
    return tray


def _backlight_bezel_frame() -> cq.Workplane:
    """Thin raised frame around the translucent backlight diffuser inside the lid."""
    frame = cq.Workplane("XY").box(0.360, 0.244, 0.004)
    opening = cq.Workplane("XY").box(0.330, 0.214, 0.010)
    return frame.cut(opening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="transparency_backlight_scanner")

    warm_gray = model.material("warm_gray_plastic", rgba=(0.72, 0.71, 0.67, 1.0))
    dark_gray = model.material("dark_gray_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    glass_blue = model.material("slightly_blue_glass", rgba=(0.56, 0.78, 0.95, 0.42))
    light_panel = model.material("warm_backlight_diffuser", rgba=(1.0, 0.93, 0.62, 0.72))
    film_amber = model.material("amber_film", rgba=(0.82, 0.36, 0.08, 0.65))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_scanner_body_shell(), "body_shell", tolerance=0.001),
        material=warm_gray,
        name="body_shell",
    )
    body.visual(
        Box((0.344, 0.224, 0.004)),
        origin=Origin(xyz=(-0.020, 0.0, 0.070)),
        material=glass_blue,
        name="scan_glass",
    )
    body.visual(
        Box((0.350, 0.012, 0.010)),
        origin=Origin(xyz=(-0.025, 0.105, 0.028)),
        material=dark_gray,
        name="guide_rail_0",
    )
    body.visual(
        Box((0.350, 0.012, 0.010)),
        origin=Origin(xyz=(-0.025, -0.105, 0.028)),
        material=dark_gray,
        name="guide_rail_1",
    )
    body.visual(
        Box((0.250, 0.006, 0.005)),
        origin=Origin(xyz=(SLOT_X - 0.002, 0.0, 0.050)),
        material=black,
        name="slot_shadow_lip",
    )
    body.visual(
        Box((0.022, 0.062, 0.012)),
        origin=Origin(xyz=(HINGE_X, -0.126, BODY_Z + 0.003)),
        material=dark_gray,
        name="hinge_mount_0",
    )
    body.visual(
        Box((0.022, 0.062, 0.012)),
        origin=Origin(xyz=(HINGE_X, 0.126, BODY_Z + 0.003)),
        material=dark_gray,
        name="hinge_mount_1",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.062),
        origin=Origin(xyz=(HINGE_X, -0.126, HINGE_Z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_gray,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.062),
        origin=Origin(xyz=(HINGE_X, 0.126, HINGE_Z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_gray,
        name="hinge_barrel_1",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.450, 0.340, 0.028)),
        # The lid part frame is on the rear hinge axis; the closed lid extends forward along local -X.
        origin=Origin(xyz=(-0.235, 0.0, 0.013)),
        material=dark_gray,
        name="lid_shell",
    )
    lid.visual(
        Box((0.030, 0.176, 0.006)),
        origin=Origin(xyz=(-0.015, 0.0, 0.003)),
        material=dark_gray,
        name="hinge_leaf",
    )
    lid.visual(
        Box((0.340, 0.224, 0.002)),
        origin=Origin(xyz=(-0.218, 0.0, -0.002)),
        material=light_panel,
        name="backlight_panel",
    )
    lid.visual(
        mesh_from_cadquery(_backlight_bezel_frame(), "backlight_bezel_frame", tolerance=0.001),
        origin=Origin(xyz=(-0.218, 0.0, -0.003)),
        material=black,
        name="backlight_bezel",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_gray,
        name="hinge_barrel",
    )

    tray = model.part("film_tray")
    tray.visual(
        mesh_from_cadquery(_film_tray_frame(), "film_tray_frame", tolerance=0.001),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=black,
        name="tray_frame",
    )
    tray.visual(
        Box((0.332, 0.056, 0.001)),
        origin=Origin(xyz=(0.170, 0.0, 0.0035)),
        material=film_amber,
        name="film_strip",
    )
    tray.visual(
        Box((0.024, 0.180, 0.014)),
        origin=Origin(xyz=(-0.032, 0.0, 0.001)),
        material=dark_gray,
        name="front_pull",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_film_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(SLOT_X, 0.0, SLOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("film_tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_slide = object_model.get_articulation("body_to_film_tray")

    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.20, elem_a="backlight_panel", elem_b="scan_glass")
    ctx.expect_gap(lid, body, axis="z", min_gap=0.001, max_gap=0.010, positive_elem="backlight_panel", negative_elem="scan_glass")
    ctx.expect_within(tray, body, axes="y", inner_elem="tray_frame", outer_elem="body_shell", margin=0.0)
    ctx.expect_overlap(tray, body, axes="x", min_overlap=0.12, elem_a="tray_frame", elem_b="body_shell")

    closed_lid_aabb = ctx.part_world_aabb(lid)
    closed_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({lid_hinge: 1.05, tray_slide: 0.150}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        extended_tray_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(tray, body, axes="x", min_overlap=0.08, elem_a="tray_frame", elem_b="body_shell")
        ctx.expect_within(tray, body, axes="y", inner_elem="tray_frame", outer_elem="body_shell", margin=0.0)

    ctx.check(
        "lid opens upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.20,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "film tray slides outward from front slot",
        closed_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] < closed_tray_pos[0] - 0.12,
        details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
