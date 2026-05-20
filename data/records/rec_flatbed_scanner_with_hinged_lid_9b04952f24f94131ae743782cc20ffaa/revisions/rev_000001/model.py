from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inkjet_printer_scanner")

    warm_white = model.material("warm_white_plastic", color=(0.82, 0.84, 0.82, 1.0))
    dark_gray = model.material("dark_gray_plastic", color=(0.10, 0.11, 0.12, 1.0))
    black = model.material("matte_black", color=(0.015, 0.016, 0.018, 1.0))
    glass = model.material("smoked_scanner_glass", color=(0.06, 0.16, 0.18, 0.70))
    rubber = model.material("black_rubber", color=(0.01, 0.01, 0.01, 1.0))

    body = model.part("body")
    # A wide desktop all-in-one printer body: about 64 cm wide, 45 cm deep.
    body.visual(
        Box((0.64, 0.45, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=warm_white,
        name="body_shell",
    )
    # Raised flatbed surround and recessed glass on the top deck.
    body.visual(
        Box((0.58, 0.035, 0.014)),
        origin=Origin(xyz=(0.0, -0.185, 0.187)),
        material=warm_white,
        name="front_scanner_rim",
    )
    body.visual(
        Box((0.58, 0.035, 0.014)),
        origin=Origin(xyz=(0.0, 0.185, 0.187)),
        material=warm_white,
        name="rear_scanner_rim",
    )
    body.visual(
        Box((0.035, 0.37, 0.014)),
        origin=Origin(xyz=(-0.2725, 0.0, 0.187)),
        material=warm_white,
        name="scanner_rim_0",
    )
    body.visual(
        Box((0.035, 0.37, 0.014)),
        origin=Origin(xyz=(0.2725, 0.0, 0.187)),
        material=warm_white,
        name="scanner_rim_1",
    )
    body.visual(
        Box((0.48, 0.31, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.183)),
        material=glass,
        name="scanner_glass",
    )
    # Front output path details.
    body.visual(
        Box((0.46, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, -0.228, 0.145)),
        material=black,
        name="output_slot",
    )
    body.visual(
        Box((0.22, 0.007, 0.030)),
        origin=Origin(xyz=(0.18, -0.2285, 0.104)),
        material=dark_gray,
        name="status_window",
    )
    # Rear hinge knuckles mounted on protruding lugs, leaving the middle knuckle to the lid.
    for suffix, x in (("0", -0.23), ("1", 0.23)):
        body.visual(
            Box((0.12, 0.020, 0.020)),
            origin=Origin(xyz=(x, 0.235, 0.188)),
            material=warm_white,
            name=f"rear_hinge_lug_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.12),
            origin=Origin(xyz=(x, 0.245, 0.198), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_gray,
            name=f"rear_hinge_barrel_{suffix}",
        )
    # Lower front tray hinge brackets tied into the body face.
    body.visual(
        Box((0.12, 0.025, 0.024)),
        origin=Origin(xyz=(-0.21, -0.2375, 0.025)),
        material=warm_white,
        name="front_hinge_lug_0",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.12),
        origin=Origin(xyz=(-0.21, -0.250, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="front_hinge_barrel_0",
    )
    body.visual(
        Box((0.12, 0.025, 0.024)),
        origin=Origin(xyz=(0.21, -0.2375, 0.025)),
        material=warm_white,
        name="front_hinge_lug_1",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.12),
        origin=Origin(xyz=(0.21, -0.250, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="front_hinge_barrel_1",
    )

    lid = model.part("scanner_lid")
    lid.visual(
        Box((0.60, 0.42, 0.028)),
        # The lid frame is on the rear hinge axis; the closed panel extends forward.
        origin=Origin(xyz=(0.0, -0.23, 0.014)),
        material=warm_white,
        name="lid_panel",
    )
    lid.visual(
        Box((0.22, 0.025, 0.018)),
        origin=Origin(xyz=(0.0, -0.012, 0.008)),
        material=warm_white,
        name="lid_hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.18, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.440, 0.005)),
        material=dark_gray,
        name="lid_front_grip",
    )
    lid.visual(
        Box((0.040, 0.025, 0.004)),
        origin=Origin(xyz=(-0.21, -0.430, -0.002)),
        material=rubber,
        name="lid_pad_0",
    )
    lid.visual(
        Box((0.040, 0.025, 0.004)),
        origin=Origin(xyz=(0.21, -0.430, -0.002)),
        material=rubber,
        name="lid_pad_1",
    )

    tray = model.part("output_tray")
    tray.visual(
        Box((0.50, 0.012, 0.110)),
        # In the closed pose the tray is a vertical panel just proud of the lower front.
        origin=Origin(xyz=(0.0, -0.006, 0.105)),
        material=warm_white,
        name="tray_panel",
    )
    tray.visual(
        Box((0.30, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -0.006, 0.025)),
        material=warm_white,
        name="tray_hinge_web",
    )
    tray.visual(
        Cylinder(radius=0.010, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="tray_hinge_barrel",
    )
    for suffix, x in (("0", -0.225), ("1", 0.225)):
        tray.visual(
            Box((0.018, 0.012, 0.125)),
            origin=Origin(xyz=(x, 0.006, 0.090)),
            material=warm_white,
            name=f"tray_side_rail_{suffix}",
        )
    tray.visual(
        Box((0.42, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.007, 0.155)),
        material=warm_white,
        name="tray_end_lip",
    )
    tray.visual(
        Box((0.14, 0.003, 0.024)),
        origin=Origin(xyz=(0.0, -0.0135, 0.122)),
        material=dark_gray,
        name="tray_grip_recess",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.245, 0.198)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.250, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("scanner_lid")
    tray = object_model.get_part("output_tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_hinge = object_model.get_articulation("body_to_tray")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="scanner_glass",
        min_overlap=0.25,
        name="closed lid covers the flatbed glass",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_pad_0",
        negative_elem="front_scanner_rim",
        min_gap=0.0,
        max_gap=0.001,
        name="lid rubber pad seats on scanner rim",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="tray_hinge_barrel",
        elem_b="front_hinge_barrel_0",
        contact_tol=0.001,
        name="tray hinge contacts one body knuckle",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="tray_hinge_barrel",
        elem_b="front_hinge_barrel_1",
        contact_tol=0.001,
        name="tray hinge contacts opposite body knuckle",
    )
    ctx.expect_gap(
        body,
        tray,
        axis="y",
        positive_elem="body_shell",
        negative_elem="tray_panel",
        min_gap=0.015,
        max_gap=0.040,
        name="closed output tray sits proud of lower front face",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    closed_tray_aabb = ctx.part_element_world_aabb(tray, elem="tray_panel")
    with ctx.pose({lid_hinge: 1.20, tray_hinge: 1.45}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        open_tray_aabb = ctx.part_element_world_aabb(tray, elem="tray_panel")

    ctx.check(
        "scanner lid opens upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.25,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "output tray folds outward and down",
        closed_tray_aabb is not None
        and open_tray_aabb is not None
        and open_tray_aabb[0][1] < closed_tray_aabb[0][1] - 0.10
        and open_tray_aabb[1][2] < closed_tray_aabb[1][2] - 0.08,
        details=f"closed={closed_tray_aabb}, open={open_tray_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
