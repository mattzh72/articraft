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


def _rounded_box(x: float, y: float, z: float, radius: float) -> cq.Workplane:
    """CadQuery rounded box centered on the local origin."""
    base = cq.Workplane("XY").box(x, y, z)
    try:
        return base.edges().fillet(radius)
    except Exception:
        return base


def _front_plate(depth: float, width: float, height: float, radius: float) -> cq.Workplane:
    """A rounded rectangular plate whose broad face is in the YZ plane."""
    base = cq.Workplane("XY").box(depth, width, height)
    try:
        return base.edges("|X").fillet(radius)
    except Exception:
        return base


def _body_shell() -> cq.Workplane:
    """Rounded outer appliance shell with a real top well for the inner pot."""
    shell = _rounded_box(0.38, 0.32, 0.22, 0.045).translate((0.0, 0.0, 0.11))
    top_well = cq.Workplane("XY").circle(0.118).extrude(0.13).translate((0.015, 0.0, 0.145))
    return shell.cut(top_well)


def _inner_pot() -> cq.Workplane:
    """A seated metal cooking pot: open cup wall plus rolled top rim."""
    cup = cq.Workplane("XY").circle(0.106).extrude(0.058).translate((0.015, 0.0, 0.155))
    hollow = cq.Workplane("XY").circle(0.096).extrude(0.060).translate((0.015, 0.0, 0.162))
    cup = cup.cut(hollow)
    rim = cq.Workplane("XY").circle(0.126).circle(0.098).extrude(0.011).translate((0.015, 0.0, 0.207))
    return cup.union(rim)


def _top_gasket() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.148).circle(0.127).extrude(0.004).translate((0.015, 0.0, 0.220))


def _lid_shell() -> cq.Workplane:
    """Low domed lid lofted from a larger rounded lower section."""
    try:
        lid = (
            cq.Workplane("XY")
            .rect(0.365, 0.295)
            .vertices()
            .fillet(0.120)
            .workplane(offset=0.040)
            .rect(0.315, 0.245)
            .vertices()
            .fillet(0.105)
            .loft(combine=True)
        )
        return lid.translate((0.195, 0.0, -0.006))
    except Exception:
        return _rounded_box(0.365, 0.295, 0.040, 0.035).translate((0.195, 0.0, 0.014))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domestic_rice_cooker")

    warm_white = model.material("warm_white", rgba=(0.92, 0.89, 0.82, 1.0))
    seam_black = model.material("seam_black", rgba=(0.015, 0.014, 0.012, 1.0))
    charcoal = model.material("charcoal_panel", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    button_gray = model.material("button_gray", rgba=(0.62, 0.64, 0.64, 1.0))
    latch_red = model.material("latch_red", rgba=(0.78, 0.18, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell", tolerance=0.0015),
        material=warm_white,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_inner_pot(), "inner_pot", tolerance=0.001),
        material=satin_steel,
        name="inner_pot",
    )
    body.visual(
        mesh_from_cadquery(_top_gasket(), "top_gasket", tolerance=0.001),
        material=seam_black,
        name="top_gasket",
    )
    body.visual(
        mesh_from_cadquery(
            _front_plate(0.008, 0.190, 0.130, 0.018).translate((0.194, 0.0, 0.095)),
            "control_panel",
            tolerance=0.001,
        ),
        material=charcoal,
        name="control_panel",
    )
    body.visual(
        mesh_from_cadquery(
            _front_plate(0.006, 0.108, 0.023, 0.006).translate((0.199, 0.0, 0.162)),
            "latch_slot",
            tolerance=0.001,
        ),
        material=seam_black,
        name="latch_slot",
    )
    body.visual(
        mesh_from_cadquery(
            _front_plate(0.006, 0.095, 0.026, 0.006).translate((0.199, 0.0, 0.118)),
            "status_window",
            tolerance=0.001,
        ),
        material=seam_black,
        name="status_window",
    )
    # Rear clevis-style hinge supports, attached to the rounded shell.
    for idx, y in enumerate((-0.095, 0.095)):
        body.visual(
            Box((0.032, 0.034, 0.038)),
            origin=Origin(xyz=(-0.188, y, 0.211)),
            material=warm_white,
            name=f"hinge_support_{idx}",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.058),
            origin=Origin(xyz=(-0.205, y, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_white,
            name=f"hinge_knuckle_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(
            _front_plate(0.375, 0.305, 0.006, 0.035).translate((0.195, 0.0, -0.006)),
            "lid_seam_band",
            tolerance=0.001,
        ),
        material=seam_black,
        name="lid_seam_band",
    )
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell", tolerance=0.001),
        material=warm_white,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_white,
        name="center_hinge_knuckle",
    )
    lid.visual(
        Box((0.032, 0.112, 0.016)),
        origin=Origin(xyz=(0.015, 0.0, 0.001)),
        material=warm_white,
        name="hinge_bridge",
    )
    lid.visual(
        Box((0.020, 0.090, 0.020)),
        origin=Origin(xyz=(0.365, 0.0, 0.000)),
        material=warm_white,
        name="front_lip",
    )
    lid.visual(
        Cylinder(radius=0.017, length=0.005),
        origin=Origin(xyz=(0.125, 0.080, 0.033)),
        material=charcoal,
        name="steam_vent",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.205, 0.0, 0.235)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    def add_button(
        part_name: str,
        joint_name: str,
        center_y: float,
        center_z: float,
        width: float,
        height: float,
        material: Material,
        travel: float,
    ) -> None:
        button = model.part(part_name)
        button.visual(
            mesh_from_cadquery(
                _front_plate(0.014, width, height, min(width, height) * 0.28).translate((0.007, 0.0, 0.0)),
                f"{part_name}_cap",
                tolerance=0.0008,
            ),
            material=material,
            name="button_cap",
        )
        # A short hidden stem makes the cap read as captured in the control panel.
        button.visual(
            Box((0.010, width * 0.55, height * 0.45)),
            origin=Origin(xyz=(0.001, 0.0, 0.0)),
            material=button_gray,
            name="button_stem",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.202, center_y, center_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.08, lower=0.0, upper=travel),
        )

    add_button("latch_button", "body_to_latch_button", 0.0, 0.155, 0.080, 0.026, latch_red, 0.008)
    add_button("menu_button_0", "body_to_menu_button_0", -0.044, 0.074, 0.050, 0.024, button_gray, 0.006)
    add_button("menu_button_1", "body_to_menu_button_1", 0.044, 0.074, 0.050, 0.024, button_gray, 0.006)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch_button")
    menu_0 = object_model.get_part("menu_button_0")
    menu_1 = object_model.get_part("menu_button_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_slide = object_model.get_articulation("body_to_latch_button")
    menu_slide_0 = object_model.get_articulation("body_to_menu_button_0")
    menu_slide_1 = object_model.get_articulation("body_to_menu_button_1")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.014,
        positive_elem="lid_seam_band",
        negative_elem="body_shell",
        name="closed lid sits just above rounded body seam",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="inner_pot",
        min_overlap=0.16,
        name="lid covers cooking pot opening",
    )
    for button, check_name in (
        (latch, "latch button is proud of front panel"),
        (menu_0, "menu button 0 is proud of front panel"),
        (menu_1, "menu button 1 is proud of front panel"),
    ):
        ctx.expect_gap(
            button,
            body,
            axis="x",
            min_gap=0.0,
            max_gap=0.006,
            positive_elem="button_cap",
            negative_elem="control_panel",
            name=check_name,
        )

    front_lip_closed = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_hinge: 1.10}):
        front_lip_open = ctx.part_element_world_aabb(lid, elem="front_lip")
    ctx.check(
        "rear hinge lifts the front of the lid upward",
        front_lip_closed is not None
        and front_lip_open is not None
        and front_lip_open[1][2] > front_lip_closed[1][2] + 0.20,
        details=f"closed={front_lip_closed}, open={front_lip_open}",
    )

    for button, joint, label in (
        (latch, latch_slide, "latch button translates into body"),
        (menu_0, menu_slide_0, "menu button 0 depresses independently"),
        (menu_1, menu_slide_1, "menu button 1 depresses independently"),
    ):
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            label,
            rest is not None and pressed is not None and pressed[0] < rest[0] - 0.004,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
