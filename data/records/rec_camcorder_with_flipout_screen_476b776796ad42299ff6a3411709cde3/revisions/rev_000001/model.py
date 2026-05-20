from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="handheld_camcorder")

    body_dark = model.material("body_dark", color=(0.17, 0.18, 0.20))
    body_accent = model.material("body_accent", color=(0.24, 0.25, 0.28))
    strap_rubber = model.material("strap_rubber", color=(0.09, 0.09, 0.10))
    lens_black = model.material("lens_black", color=(0.06, 0.06, 0.07))
    metal_trim = model.material("metal_trim", color=(0.45, 0.46, 0.48))
    display_shell = model.material("display_shell", color=(0.20, 0.20, 0.22))
    screen_black = model.material("screen_black", color=(0.03, 0.04, 0.05))

    body = model.part("body")
    body.visual(
        Box((0.110, 0.064, 0.068)),
        material=body_dark,
        name="main_shell",
    )
    body.visual(
        Box((0.014, 0.048, 0.052)),
        origin=Origin(xyz=(0.058, 0.0, 0.008)),
        material=body_accent,
        name="front_block",
    )
    body.visual(
        Box((0.048, 0.028, 0.014)),
        origin=Origin(xyz=(-0.005, 0.0, 0.041)),
        material=body_accent,
        name="top_hump",
    )
    body.visual(
        Box((0.022, 0.046, 0.058)),
        origin=Origin(xyz=(-0.062, 0.0, -0.002)),
        material=body_accent,
        name="rear_battery",
    )
    body.visual(
        Box((0.010, 0.004, 0.052)),
        origin=Origin(xyz=(-0.020, -0.030, 0.0)),
        material=body_accent,
        name="display_hinge_mount",
    )
    body.visual(
        Box((0.018, 0.008, 0.026)),
        origin=Origin(xyz=(0.030, 0.032, 0.003)),
        material=strap_rubber,
        name="strap_front_anchor",
    )
    body.visual(
        Box((0.018, 0.008, 0.030)),
        origin=Origin(xyz=(-0.034, 0.032, 0.006)),
        material=strap_rubber,
        name="strap_rear_anchor",
    )
    body.visual(
        Box((0.074, 0.006, 0.028)),
        origin=Origin(xyz=(-0.002, 0.035, 0.005)),
        material=strap_rubber,
        name="hand_strap",
    )
    body.visual(
        Box((0.026, 0.004, 0.020)),
        origin=Origin(xyz=(-0.004, 0.038, 0.005)),
        material=body_accent,
        name="strap_pad",
    )

    lens = model.part("lens_barrel")
    lens.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_trim,
        name="rear_mount",
    )
    lens.visual(
        Cylinder(radius=0.021, length=0.022),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_black,
        name="zoom_ring",
    )
    lens.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.042, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_black,
        name="front_barrel",
    )
    lens.visual(
        Cylinder(radius=0.022, length=0.005),
        origin=Origin(xyz=(0.0545, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_trim,
        name="front_trim",
    )
    lens.visual(
        Cylinder(radius=0.015, length=0.002),
        origin=Origin(xyz=(0.0575, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=screen_black,
        name="front_glass",
    )
    lens.visual(
        Box((0.012, 0.004, 0.003)),
        origin=Origin(xyz=(0.020, 0.0, 0.021)),
        material=metal_trim,
        name="zoom_rib",
    )

    display = model.part("display_panel")
    display.visual(
        Cylinder(radius=0.0035, length=0.050),
        material=body_accent,
        name="hinge_barrel",
    )
    display.visual(
        Box((0.012, 0.007, 0.040)),
        origin=Origin(xyz=(0.006, 0.0005, 0.0)),
        material=body_accent,
        name="hinge_arm",
    )
    display.visual(
        Box((0.072, 0.0045, 0.050)),
        origin=Origin(xyz=(0.046, 0.00175, 0.0)),
        material=display_shell,
        name="panel_back",
    )
    display.visual(
        Box((0.056, 0.0012, 0.038)),
        origin=Origin(xyz=(0.046, 0.0031, 0.0)),
        material=screen_black,
        name="screen",
    )

    model.articulation(
        "body_to_lens",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens,
        origin=Origin(xyz=(0.065, 0.0, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )
    model.articulation(
        "body_to_display",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(-0.020, -0.036, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens = object_model.get_part("lens_barrel")
    display = object_model.get_part("display_panel")
    lens_joint = object_model.get_articulation("body_to_lens")
    display_joint = object_model.get_articulation("body_to_display")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((a + b) * 0.5 for a, b in zip(lo, hi))

    ctx.expect_gap(
        body,
        display,
        axis="y",
        positive_elem="main_shell",
        negative_elem="panel_back",
        max_gap=0.002,
        max_penetration=1e-6,
        name="closed display sits just off the left body side",
    )
    ctx.expect_overlap(
        body,
        display,
        axes="xz",
        elem_a="main_shell",
        elem_b="panel_back",
        min_overlap=0.040,
        name="closed display covers the camcorder side panel area",
    )
    ctx.expect_gap(
        lens,
        body,
        axis="x",
        positive_elem="rear_mount",
        negative_elem="front_block",
        max_gap=0.002,
        max_penetration=0.0,
        name="lens barrel mounts tightly to the front face",
    )
    ctx.expect_overlap(
        lens,
        body,
        axes="yz",
        elem_a="rear_mount",
        elem_b="front_block",
        min_overlap=0.040,
        name="lens mount overlaps the front bezel footprint",
    )

    closed_panel_center = aabb_center(ctx.part_element_world_aabb(display, elem="panel_back"))
    with ctx.pose({display_joint: 1.6}):
        open_panel_center = aabb_center(ctx.part_element_world_aabb(display, elem="panel_back"))
        ctx.expect_gap(
            body,
            display,
            axis="y",
            positive_elem="main_shell",
            negative_elem="panel_back",
            min_gap=0.010,
            name="opened display swings clear of the body side",
        )
    ctx.check(
        "display swings outward toward negative Y",
        closed_panel_center is not None
        and open_panel_center is not None
        and open_panel_center[1] < closed_panel_center[1] - 0.010,
        details=f"closed_center={closed_panel_center}, open_center={open_panel_center}",
    )

    rib_center_rest = aabb_center(ctx.part_element_world_aabb(lens, elem="zoom_rib"))
    with ctx.pose({lens_joint: 1.1}):
        rib_center_spin = aabb_center(ctx.part_element_world_aabb(lens, elem="zoom_rib"))
    ctx.check(
        "lens barrel rotates about its viewing axis",
        rib_center_rest is not None
        and rib_center_spin is not None
        and abs(rib_center_spin[0] - rib_center_rest[0]) < 0.002
        and (
            abs(rib_center_spin[1] - rib_center_rest[1]) > 0.006
            or abs(rib_center_spin[2] - rib_center_rest[2]) > 0.006
        ),
        details=f"rest_center={rib_center_rest}, spun_center={rib_center_spin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
