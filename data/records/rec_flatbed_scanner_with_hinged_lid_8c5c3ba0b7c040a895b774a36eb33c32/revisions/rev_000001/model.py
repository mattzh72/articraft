from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="document_management_scanner")

    warm_gray = Material("warm_gray_plastic", rgba=(0.70, 0.72, 0.72, 1.0))
    dark_gray = Material("dark_gray_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    black = Material("black_rubber", rgba=(0.015, 0.015, 0.018, 1.0))
    glass = Material("slightly_blue_scanner_glass", rgba=(0.62, 0.82, 0.94, 0.55))
    mirror = Material("reflective_white_lid_liner", rgba=(0.93, 0.95, 0.93, 1.0))
    metal = Material("brushed_hinge_metal", rgba=(0.55, 0.56, 0.58, 1.0))
    button_blue = Material("blue_start_button", rgba=(0.08, 0.36, 0.78, 1.0))
    button_dark = Material("dark_mode_button", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.80, 0.45, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=warm_gray,
        name="chassis",
    )
    body.visual(
        Box((0.62, 0.38, 0.004)),
        origin=Origin(xyz=(-0.075, -0.005, 0.102)),
        material=dark_gray,
        name="scanner_bed_bezel",
    )
    body.visual(
        Box((0.53, 0.30, 0.003)),
        origin=Origin(xyz=(-0.075, -0.020, 0.1035)),
        material=glass,
        name="glass",
    )
    body.visual(
        Box((0.16, 0.085, 0.006)),
        origin=Origin(xyz=(0.305, -0.160, 0.103)),
        material=dark_gray,
        name="control_panel",
    )
    body.visual(
        Box((0.56, 0.004, 0.020)),
        origin=Origin(xyz=(-0.030, -0.226, 0.075)),
        material=black,
        name="feed_slot_panel",
    )
    body.visual(
        Box((0.56, 0.010, 0.018)),
        origin=Origin(xyz=(-0.030, -0.228, 0.089)),
        material=dark_gray,
        name="front_tray_hinge_mount",
    )

    for i, x in enumerate((-0.255, 0.105)):
        body.visual(
            Box((0.120, 0.068, 0.006)),
            origin=Origin(xyz=(x, 0.211, 0.1015)),
            material=metal,
            name=f"rear_hinge_leaf_{i}",
        )
        body.visual(
            Box((0.070, 0.012, 0.015)),
            origin=Origin(xyz=(x, 0.247, 0.1075)),
            material=metal,
            name=f"rear_hinge_bracket_{i}",
        )

    for i, x in enumerate((-0.315, 0.315)):
        body.visual(
            Box((0.080, 0.060, 0.014)),
            origin=Origin(xyz=(x, 0.170, 0.007)),
            material=black,
            name=f"rubber_foot_{i}",
        )
        body.visual(
            Box((0.080, 0.060, 0.014)),
            origin=Origin(xyz=(x, -0.170, 0.007)),
            material=black,
            name=f"rubber_foot_{i + 2}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.62, 0.43, 0.026)),
        origin=Origin(xyz=(-0.075, -0.218, 0.0)),
        material=warm_gray,
        name="lid_shell",
    )
    lid.visual(
        Box((0.53, 0.34, 0.002)),
        origin=Origin(xyz=(-0.075, -0.218, -0.014)),
        material=mirror,
        name="reflective_panel",
    )
    lid.visual(
        Box((0.62, 0.014, 0.030)),
        origin=Origin(xyz=(-0.075, -0.010, -0.001)),
        material=dark_gray,
        name="rear_lid_rib",
    )
    for i, x in enumerate((-0.255, 0.105)):
        lid.visual(
            Cylinder(radius=0.011, length=0.115),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name=f"rear_barrel_{i}",
        )
        lid.visual(
            Box((0.080, 0.020, 0.010)),
            origin=Origin(xyz=(x, -0.014, -0.003)),
            material=metal,
            name=f"lid_hinge_leaf_{i}",
        )

    feed_tray = model.part("feed_tray")
    feed_tray.visual(
        Box((0.56, 0.30, 0.010)),
        origin=Origin(xyz=(-0.030, -0.160, 0.005)),
        material=warm_gray,
        name="tray_panel",
    )
    feed_tray.visual(
        Cylinder(radius=0.010, length=0.58),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="tray_hinge_barrel",
    )
    for i, x in enumerate((-0.285, 0.225)):
        feed_tray.visual(
            Box((0.014, 0.250, 0.016)),
            origin=Origin(xyz=(x, -0.170, 0.018)),
            material=dark_gray,
            name=f"paper_guide_{i}",
        )
    feed_tray.visual(
        Box((0.54, 0.014, 0.026)),
        origin=Origin(xyz=(-0.030, -0.305, 0.023)),
        material=dark_gray,
        name="paper_stop_lip",
    )
    feed_tray.visual(
        Box((0.42, 0.003, 0.010)),
        origin=Origin(xyz=(-0.030, -0.055, 0.015)),
        material=black,
        name="paper_entry_marker",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.040, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=button_blue,
        name="button_cap",
    )

    mode_button = model.part("mode_button")
    mode_button.visual(
        Box((0.030, 0.026, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=button_dark,
        name="button_cap",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.240, 0.122)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "tray_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=feed_tray,
        origin=Origin(xyz=(0.0, -0.241, 0.095)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.35, upper=0.0),
    )
    model.articulation(
        "start_button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(0.275, -0.160, 0.106)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.004),
    )
    model.articulation(
        "mode_button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button,
        origin=Origin(xyz=(0.340, -0.160, 0.106)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.05, lower=0.0, upper=0.003),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    feed_tray = object_model.get_part("feed_tray")
    start_button = object_model.get_part("start_button")
    lid_hinge = object_model.get_articulation("lid_hinge")
    tray_hinge = object_model.get_articulation("tray_hinge")
    start_press = object_model.get_articulation("start_button_press")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="reflective_panel",
        negative_elem="glass",
        name="reflective lid liner clears the scanner glass when closed",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.20,
        elem_a="lid_shell",
        elem_b="scanner_bed_bezel",
        name="closed lid covers the scanner bed footprint",
    )
    ctx.expect_overlap(
        feed_tray,
        body,
        axes="x",
        min_overlap=0.45,
        elem_a="tray_hinge_barrel",
        elem_b="feed_slot_panel",
        name="feed tray hinge spans the front feed slot",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.0}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward around the rear barrel hinge",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.15,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    open_tray_aabb = ctx.part_world_aabb(feed_tray)
    body_aabb = ctx.part_world_aabb(body)
    with ctx.pose({tray_hinge: -1.2}):
        folded_tray_aabb = ctx.part_world_aabb(feed_tray)
        ctx.check(
            "feed tray folds inward toward the front face",
            open_tray_aabb is not None
            and folded_tray_aabb is not None
            and body_aabb is not None
            and open_tray_aabb[0][1] < body_aabb[0][1] - 0.25
            and folded_tray_aabb[0][1] > open_tray_aabb[0][1] + 0.16,
            details=f"open={open_tray_aabb}, folded={folded_tray_aabb}, body={body_aabb}",
        )

    button_rest = ctx.part_world_position(start_button)
    with ctx.pose({start_press: 0.003}):
        button_pressed = ctx.part_world_position(start_button)
        ctx.check(
            "start button depresses into the control panel",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[2] < button_rest[2] - 0.002,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
