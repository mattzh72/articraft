from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="air_fryer")

    # Dimensions
    housing_w, housing_d, housing_h = 0.30, 0.35, 0.35
    
    # 1. Housing
    housing_body = (
        cq.Workplane("XY")
        .box(housing_w, housing_d, housing_h)
        .edges("|Z").fillet(0.05)
        .edges(">Z").fillet(0.05)
        .translate((0, 0, housing_h / 2))
    )
    
    chamber_cut = (
        cq.Workplane("XY")
        .box(0.26, 0.30, 0.22)
        .edges("|Z").fillet(0.03)
        .translate((0, -0.05, 0.13))
    )
    housing_body = housing_body.cut(chamber_cut)

    preset_cavities = (
        cq.Workplane("XY")
        .box(0.022, 0.017, 0.01)
        .translate((-0.06, 0.05, 0.345))
        .union(
            cq.Workplane("XY")
            .box(0.022, 0.017, 0.01)
            .translate((0.06, 0.05, 0.345))
        )
    )
    housing_body = housing_body.cut(preset_cavities)

    housing = model.part("housing")
    housing.visual(mesh_from_cadquery(housing_body, "housing_body"))

    # 2. Drawer
    drawer_front = (
        cq.Workplane("XY")
        .box(0.256, 0.03, 0.216)
        .edges("|Y").fillet(0.028)
        .translate((0, -0.16, 0.13))
    )
    notch = (
        cq.Workplane("XY")
        .box(0.06, 0.05, 0.06)
        .translate((0, -0.16, 0.22))
    )
    drawer_front = drawer_front.cut(notch)

    drawer_tub = (
        cq.Workplane("XY")
        .box(0.25, 0.235, 0.20)
        .edges("|Z").fillet(0.02)
        .translate((0, -0.0275, 0.12))
    )
    drawer_tub_cut = (
        cq.Workplane("XY")
        .box(0.24, 0.225, 0.20)
        .edges("|Z").fillet(0.015)
        .translate((0, -0.0275, 0.13))
    )
    drawer_tub = drawer_tub.cut(drawer_tub_cut)
    drawer_body = drawer_front.union(drawer_tub)

    drawer = model.part("drawer")
    drawer.visual(mesh_from_cadquery(drawer_body, "drawer_body"))

    # 3. Basket
    basket_tub = (
        cq.Workplane("XY")
        .box(0.23, 0.215, 0.19)
        .edges("|Z").fillet(0.01)
        .translate((0, -0.0275, 0.125))
    )
    basket_tub_cut = (
        cq.Workplane("XY")
        .box(0.22, 0.205, 0.19)
        .edges("|Z").fillet(0.008)
        .translate((0, -0.0275, 0.135))
    )
    basket_tub = basket_tub.cut(basket_tub_cut)

    perf_cut_bottom = (
        cq.Workplane("XY")
        .workplane(offset=0.03)
        .rarray(0.02, 0.02, 10, 9)
        .rect(0.01, 0.01)
        .extrude(0.02)
    )
    basket_tub = basket_tub.cut(perf_cut_bottom)

    handle_horiz = (
        cq.Workplane("XY")
        .box(0.05, 0.08, 0.02)
        .translate((0, -0.175, 0.21))
    )
    handle_vert = (
        cq.Workplane("XY")
        .box(0.05, 0.03, 0.08)
        .edges("|X").fillet(0.01)
        .translate((0, -0.20, 0.24))
    )
    basket_handle = handle_horiz.union(handle_vert)
    basket_body = basket_tub.union(basket_handle)

    button_cavity = (
        cq.Workplane("XY")
        .box(0.03, 0.02, 0.015)
        .translate((0, -0.20, 0.2725))
    )
    basket_body = basket_body.cut(button_cavity)

    basket = model.part("basket")
    basket.visual(mesh_from_cadquery(basket_body, "basket_body"))

    # 4. Basket Button
    button_body = (
        cq.Workplane("XY")
        .box(0.026, 0.016, 0.01)
        .translate((0, 0, -0.005))
        .union(
            cq.Workplane("XY")
            .box(0.01, 0.01, 0.02)
            .translate((0, 0, -0.01))
        )
    )
    
    basket_button = model.part("basket_button")
    basket_button.visual(mesh_from_cadquery(button_body, "basket_button_body"))

    # 5. Controls
    dial_temp = cq.Workplane("XY").circle(0.02).extrude(0.015)
    dial_time = cq.Workplane("XY").circle(0.02).extrude(0.015)
    
    preset_1 = (
        cq.Workplane("XY")
        .box(0.02, 0.015, 0.005)
        .union(
            cq.Workplane("XY")
            .box(0.01, 0.01, 0.015)
            .translate((0, 0, -0.0075))
        )
    )
    preset_2 = (
        cq.Workplane("XY")
        .box(0.02, 0.015, 0.005)
        .union(
            cq.Workplane("XY")
            .box(0.01, 0.01, 0.015)
            .translate((0, 0, -0.0075))
        )
    )

    dial_temp_part = model.part("dial_temp")
    dial_temp_part.visual(mesh_from_cadquery(dial_temp, "dial_temp_body"))

    dial_time_part = model.part("dial_time")
    dial_time_part.visual(mesh_from_cadquery(dial_time, "dial_time_body"))

    preset_1_part = model.part("preset_1")
    preset_1_part.visual(mesh_from_cadquery(preset_1, "preset_1_body"))

    preset_2_part = model.part("preset_2")
    preset_2_part.visual(mesh_from_cadquery(preset_2, "preset_2_body"))

    # Articulations
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(lower=0.0, upper=0.25)
    )

    model.articulation(
        "basket_lift",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=0.20)
    )

    model.articulation(
        "basket_button_press",
        ArticulationType.PRISMATIC,
        parent=basket,
        child=basket_button,
        origin=Origin(xyz=(0, -0.20, 0.28)),
        axis=(0, 0, -1),
        motion_limits=MotionLimits(lower=0.0, upper=0.005)
    )

    model.articulation(
        "dial_temp_turn",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial_temp_part,
        origin=Origin(xyz=(-0.06, 0.0, 0.35)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0)
    )

    model.articulation(
        "dial_time_turn",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial_time_part,
        origin=Origin(xyz=(0.06, 0.0, 0.35)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0)
    )

    model.articulation(
        "preset_1_press",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=preset_1_part,
        origin=Origin(xyz=(-0.06, 0.05, 0.35)),
        axis=(0, 0, -1),
        motion_limits=MotionLimits(lower=0.0, upper=0.004)
    )

    model.articulation(
        "preset_2_press",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=preset_2_part,
        origin=Origin(xyz=(0.06, 0.05, 0.35)),
        axis=(0, 0, -1),
        motion_limits=MotionLimits(lower=0.0, upper=0.004)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    housing = object_model.get_part("housing")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    basket_button = object_model.get_part("basket_button")
    preset_1 = object_model.get_part("preset_1")
    preset_2 = object_model.get_part("preset_2")
    
    ctx.allow_overlap(drawer, housing, reason="Drawer fits into housing chamber")
    ctx.allow_overlap(basket, drawer, reason="Basket fits into drawer tub")
    ctx.allow_overlap(basket_button, basket, reason="Button fits into basket handle cavity")
    ctx.allow_overlap(preset_1, housing, reason="Preset button fits into housing cavity")
    ctx.allow_overlap(preset_2, housing, reason="Preset button fits into housing cavity")
    
    ctx.expect_overlap(drawer, housing, axes="xz", min_overlap=0.1)
    ctx.expect_overlap(basket, drawer, axes="xy", min_overlap=0.1)
    
    return ctx.report()

object_model = build_object_model()
