from __future__ import annotations

import math

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
    model = ArticulatedObject(name="a3_flatbed_scanner")

    warm_gray = Material("warm_light_gray", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_gray = Material("dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    black = Material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    glass = Material("slightly_blue_glass", rgba=(0.35, 0.68, 0.82, 0.45))
    white = Material("white_document_pad", rgba=(0.93, 0.93, 0.88, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    button_blue = Material("blue_button", rgba=(0.08, 0.20, 0.42, 1.0))
    button_green = Material("green_button", rgba=(0.04, 0.36, 0.16, 1.0))
    button_gray = Material("button_gray", rgba=(0.25, 0.26, 0.27, 1.0))

    base = model.part("scanner_body")
    base.visual(
        Box((0.70, 0.52, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=warm_gray,
        name="wide_body_shell",
    )
    base.visual(
        Box((0.485, 0.355, 0.004)),
        origin=Origin(xyz=(0.0, -0.035, 0.087)),
        material=dark_gray,
        name="platen_recess",
    )
    base.visual(
        Box((0.455, 0.325, 0.006)),
        origin=Origin(xyz=(0.0, -0.035, 0.091)),
        material=glass,
        name="glass_panel",
    )
    base.visual(
        Box((0.030, 0.330, 0.004)),
        origin=Origin(xyz=(-0.235, -0.035, 0.094)),
        material=black,
        name="registration_ruler",
    )
    base.visual(
        Box((0.455, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.196, 0.094)),
        material=black,
        name="front_scan_mark",
    )
    base.visual(
        Box((0.255, 0.036, 0.022)),
        origin=Origin(xyz=(0.145, -0.258, 0.096)),
        material=dark_gray,
        name="front_control_panel",
    )
    base.visual(
        Box((0.082, 0.019, 0.003)),
        origin=Origin(xyz=(0.055, -0.258, 0.1085)),
        material=black,
        name="status_display",
    )

    # Rear hinge hardware: two fixed brackets carry an exposed steel pin.
    for index, x in enumerate((-0.235, 0.235)):
        base.visual(
            Box((0.095, 0.026, 0.033)),
            origin=Origin(xyz=(x, 0.251, 0.1015)),
            material=dark_gray,
            name=f"rear_hinge_mount_{index}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.090),
            origin=Origin(xyz=(x, 0.252, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_gray,
            name=f"fixed_hinge_barrel_{index}",
        )
    base.visual(
        Cylinder(radius=0.0032, length=0.620),
        origin=Origin(xyz=(0.0, 0.252, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="hinge_pin",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.670, 0.466, 0.028)),
        origin=Origin(xyz=(0.0, -0.245, 0.0)),
        material=warm_gray,
        name="lid_panel",
    )
    lid.visual(
        Box((0.515, 0.350, 0.006)),
        origin=Origin(xyz=(0.0, -0.290, -0.017)),
        material=white,
        name="white_pressure_pad",
    )
    lid.visual(
        Box((0.615, 0.135, 0.052)),
        origin=Origin(xyz=(0.0, -0.076, 0.040)),
        material=dark_gray,
        name="roller_housing",
    )
    lid.visual(
        Box((0.540, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, -0.151, 0.034)),
        material=black,
        name="feed_slot_shadow",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.555),
        origin=Origin(xyz=(0.0, -0.151, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="feed_roller",
    )
    lid.visual(
        Box((0.575, 0.120, 0.010)),
        origin=Origin(xyz=(0.0, -0.195, 0.060)),
        material=warm_gray,
        name="paper_guide_lip",
    )
    for index, x in enumerate((-0.115, 0.0, 0.115)):
        lid.visual(
            Cylinder(radius=0.0105, length=0.060),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_gray,
            name=f"lid_knuckle_{index}",
        )
        lid.visual(
            Box((0.058, 0.018, 0.012)),
            origin=Origin(xyz=(x, -0.015, -0.010)),
            material=dark_gray,
            name=f"knuckle_lug_{index}",
        )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.252, 0.115)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    button_specs = (
        ("power_button", 0.155, button_green),
        ("scan_button", 0.195, button_blue),
        ("copy_button", 0.235, button_gray),
        ("menu_button", 0.267, button_gray),
    )
    for name, x, material in button_specs:
        button = model.part(name)
        button.visual(
            Box((0.030, 0.018, 0.006)),
            origin=Origin(),
            material=material,
            name="button_cap",
        )
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.258, 0.110)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("scanner_body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("rear_lid_hinge")

    for index in range(3):
        knuckle = f"lid_knuckle_{index}"
        ctx.allow_overlap(
            body,
            lid,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="The fixed hinge pin intentionally passes through the lid hinge knuckle.",
        )
        ctx.expect_within(
            body,
            lid,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem=knuckle,
            name=f"hinge pin is captured in knuckle {index}",
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="x",
            elem_a="hinge_pin",
            elem_b=knuckle,
            min_overlap=0.045,
            name=f"hinge pin spans knuckle {index}",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="glass_panel",
            min_gap=0.006,
            max_gap=0.020,
            name="closed lid clears the platen glass",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="glass_panel",
            min_overlap=0.30,
            name="closed lid covers the A3 glass bed",
        )

    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.20}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "rear hinge opens the lid upward",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.20,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )

    scan_button = object_model.get_part("scan_button")
    scan_joint = object_model.get_articulation("scan_button_press")
    rest_button_aabb = ctx.part_world_aabb(scan_button)
    with ctx.pose({scan_joint: 0.004}):
        pressed_button_aabb = ctx.part_world_aabb(scan_button)
    ctx.check(
        "scan button depresses into the control panel",
        rest_button_aabb is not None
        and pressed_button_aabb is not None
        and pressed_button_aabb[1][2] < rest_button_aabb[1][2] - 0.003,
        details=f"rest={rest_button_aabb}, pressed={pressed_button_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
