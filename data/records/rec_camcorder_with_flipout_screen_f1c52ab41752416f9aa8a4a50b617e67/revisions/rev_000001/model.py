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


BODY_LENGTH = 0.108
BODY_WIDTH = 0.056
BODY_HEIGHT = 0.064
HALF_PI = math.pi / 2.0

LENS_AXIS_Z = 0.004


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handheld_camcorder")

    graphite = model.material("graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    satin = model.material("satin", rgba=(0.34, 0.35, 0.37, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.08, 0.10, 0.12, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.10, 0.13, 0.15, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.84, 0.84, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)),
        material=graphite,
        name="main_shell",
    )
    body.visual(
        Box((0.064, 0.034, 0.016)),
        origin=Origin(xyz=(-0.006, 0.0, BODY_HEIGHT / 2.0 + 0.008)),
        material=charcoal,
        name="top_hump",
    )
    body.visual(
        Box((0.022, 0.040, 0.040)),
        origin=Origin(xyz=(-0.040, 0.0, -0.004)),
        material=charcoal,
        name="rear_bulge",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.008),
        origin=Origin(
            xyz=(BODY_LENGTH / 2.0 - 0.004, 0.0, LENS_AXIS_Z),
            rpy=(0.0, HALF_PI, 0.0),
        ),
        material=satin,
        name="mount_collar",
    )
    body.visual(
        Box((0.018, 0.026, 0.016)),
        origin=Origin(xyz=(-BODY_LENGTH / 2.0 - 0.009, -0.008, 0.010)),
        material=charcoal,
        name="eyepiece_block",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(
            xyz=(-BODY_LENGTH / 2.0 - 0.014, -0.008, 0.010),
            rpy=(0.0, HALF_PI, 0.0),
        ),
        material=rubber,
        name="eyecup",
    )

    lens_barrel = model.part("lens_barrel")
    lens_barrel.visual(
        Cylinder(radius=0.012, length=0.066),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=charcoal,
        name="inner_core",
    )
    lens_barrel.visual(
        Cylinder(radius=0.019, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=graphite,
        name="rear_section",
    )
    lens_barrel.visual(
        Cylinder(radius=0.017, length=0.022),
        origin=Origin(xyz=(0.049, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=charcoal,
        name="front_section",
    )
    lens_barrel.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.063, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=satin,
        name="front_bezel",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0135, length=0.002),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=lens_glass,
        name="front_glass",
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring_shape = (
        cq.Workplane("YZ")
        .circle(0.0215)
        .circle(0.0186)
        .extrude(0.007, both=True)
        .union(
            cq.Workplane("YZ", origin=(-0.0045, 0.0, 0.0))
            .circle(0.0225)
            .circle(0.0186)
            .extrude(0.00125, both=True)
        )
        .union(
            cq.Workplane("YZ", origin=(0.0045, 0.0, 0.0))
            .circle(0.0225)
            .circle(0.0186)
            .extrude(0.00125, both=True)
        )
    )
    zoom_ring.visual(
        mesh_from_cadquery(zoom_ring_shape, "zoom_ring"),
        material=rubber,
        name="ring_body",
    )

    lcd_panel = model.part("lcd_panel")
    lcd_panel.visual(
        Cylinder(radius=0.0028, length=0.046),
        origin=Origin(xyz=(0.0, -0.0025, 0.0)),
        material=satin,
        name="hinge_barrel",
    )
    lcd_panel.visual(
        Box((0.068, 0.005, 0.046)),
        origin=Origin(xyz=(0.034, -0.0025, 0.0)),
        material=graphite,
        name="panel_shell",
    )
    lcd_panel.visual(
        Box((0.055, 0.0012, 0.039)),
        origin=Origin(xyz=(0.036, -0.0007, 0.0)),
        material=screen_glass,
        name="screen",
    )

    hand_strap = model.part("hand_strap")
    hand_strap.visual(
        Box((0.072, 0.0025, 0.030)),
        material=rubber,
        name="strap_band",
    )
    hand_strap.visual(
        Box((0.014, 0.010, 0.032)),
        origin=Origin(xyz=(0.029, -0.004, 0.0)),
        material=charcoal,
        name="front_anchor",
    )
    hand_strap.visual(
        Box((0.014, 0.010, 0.032)),
        origin=Origin(xyz=(-0.022, -0.004, 0.0)),
        material=charcoal,
        name="rear_anchor",
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        Cylinder(radius=0.011, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=satin,
        name="dial_body",
    )
    mode_dial.visual(
        Cylinder(radius=0.0125, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.00425)),
        material=graphite,
        name="dial_rim",
    )
    mode_dial.visual(
        Box((0.006, 0.0015, 0.0012)),
        origin=Origin(xyz=(0.007, 0.0, 0.0052)),
        material=dial_mark,
        name="pointer",
    )

    model.articulation(
        "body_to_lens_barrel",
        ArticulationType.FIXED,
        parent=body,
        child=lens_barrel,
        origin=Origin(xyz=(BODY_LENGTH / 2.0, 0.0, LENS_AXIS_Z)),
    )
    model.articulation(
        "lens_barrel_to_zoom_ring",
        ArticulationType.CONTINUOUS,
        parent=lens_barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.031, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=10.0),
    )
    model.articulation(
        "body_to_lcd_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lcd_panel,
        origin=Origin(xyz=(-0.022, -BODY_WIDTH / 2.0, 0.004)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.5,
            lower=0.0,
            upper=2.2,
        ),
    )
    model.articulation(
        "body_to_hand_strap",
        ArticulationType.FIXED,
        parent=body,
        child=hand_strap,
        origin=Origin(xyz=(-0.006, BODY_WIDTH / 2.0 + 0.009, 0.002)),
    )
    model.articulation(
        "body_to_mode_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.030, 0.014, BODY_HEIGHT / 2.0 + 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lens_barrel = object_model.get_part("lens_barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    lcd_panel = object_model.get_part("lcd_panel")
    hand_strap = object_model.get_part("hand_strap")
    mode_dial = object_model.get_part("mode_dial")

    lcd_hinge = object_model.get_articulation("body_to_lcd_panel")
    zoom_spin = object_model.get_articulation("lens_barrel_to_zoom_ring")
    dial_spin = object_model.get_articulation("body_to_mode_dial")

    ctx.expect_gap(
        lens_barrel,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rear_section",
        negative_elem="mount_collar",
        name="lens barrel seats flush on the front collar",
    )
    ctx.expect_overlap(
        lens_barrel,
        body,
        axes="yz",
        min_overlap=0.035,
        elem_a="rear_section",
        elem_b="mount_collar",
        name="lens barrel stays centered on the front mount",
    )

    with ctx.pose({lcd_hinge: 0.0}):
        ctx.expect_gap(
            body,
            lcd_panel,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="main_shell",
            negative_elem="panel_shell",
            name="closed lcd sits flush against the left body side",
        )
        ctx.expect_overlap(
            lcd_panel,
            body,
            axes="xz",
            min_overlap=0.040,
            elem_a="panel_shell",
            elem_b="main_shell",
            name="closed lcd covers the left side face",
        )
        lcd_closed_aabb = ctx.part_world_aabb(lcd_panel)

    with ctx.pose({lcd_hinge: lcd_hinge.motion_limits.upper}):
        lcd_open_aabb = ctx.part_world_aabb(lcd_panel)

    ctx.check(
        "lcd swings outward from the body",
        lcd_closed_aabb is not None
        and lcd_open_aabb is not None
        and lcd_open_aabb[0][1] < lcd_closed_aabb[0][1] - 0.020,
        details=f"closed={lcd_closed_aabb}, open={lcd_open_aabb}",
    )

    ctx.expect_origin_distance(
        zoom_ring,
        lens_barrel,
        axes="yz",
        max_dist=0.0005,
        name="zoom ring stays concentric with the optical axis",
    )
    ctx.expect_origin_gap(
        zoom_ring,
        lens_barrel,
        axis="x",
        min_gap=0.026,
        max_gap=0.036,
        name="zoom ring sits around the front half of the barrel",
    )
    ctx.check(
        "zoom ring uses continuous rotation",
        zoom_spin.motion_limits is not None
        and zoom_spin.motion_limits.lower is None
        and zoom_spin.motion_limits.upper is None,
        details=f"limits={zoom_spin.motion_limits}",
    )

    ctx.expect_gap(
        hand_strap,
        body,
        axis="y",
        min_gap=0.005,
        max_gap=0.012,
        positive_elem="strap_band",
        negative_elem="main_shell",
        name="hand strap stands proud for a gripping gap",
    )
    ctx.expect_gap(
        hand_strap,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="front_anchor",
        negative_elem="main_shell",
        name="front strap anchor lands on the body side",
    )
    ctx.expect_gap(
        hand_strap,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rear_anchor",
        negative_elem="main_shell",
        name="rear strap anchor lands on the body side",
    )

    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="dial_body",
        negative_elem="top_hump",
        name="mode dial sits on the top rear corner pad",
    )
    mode_pos = ctx.part_world_position(mode_dial)
    ctx.check(
        "mode dial is mounted on the rear top corner",
        mode_pos is not None and mode_pos[0] < -0.015 and mode_pos[1] > 0.008 and mode_pos[2] > 0.045,
        details=f"mode_pos={mode_pos}",
    )
    ctx.check(
        "mode dial uses continuous rotation",
        dial_spin.motion_limits is not None
        and dial_spin.motion_limits.lower is None
        and dial_spin.motion_limits.upper is None,
        details=f"limits={dial_spin.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
