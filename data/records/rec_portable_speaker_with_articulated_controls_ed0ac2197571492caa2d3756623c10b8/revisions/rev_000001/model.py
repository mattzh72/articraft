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
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


ENCLOSURE_WIDTH = 0.180
ENCLOSURE_DEPTH = 0.070
ENCLOSURE_HEIGHT = 0.105
REAR_Y = ENCLOSURE_DEPTH * 0.5
FRONT_Y = -ENCLOSURE_DEPTH * 0.5
BUTTON_XS = (-0.032, 0.032)
BUTTON_Y = 0.000
BUTTON_TRAVEL = 0.005


def _build_enclosure_shell() -> cq.Workplane:
    """Rounded compact speaker case with two real top counterbore openings."""
    body = (
        cq.Workplane("XY")
        .box(ENCLOSURE_WIDTH, ENCLOSURE_DEPTH, ENCLOSURE_HEIGHT)
        .translate((0.0, 0.0, ENCLOSURE_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.010)
    )

    for x in BUTTON_XS:
        shaft_cut = (
            cq.Workplane("XY")
            .circle(0.0047)
            .extrude(0.030)
            .translate((x, BUTTON_Y, ENCLOSURE_HEIGHT - 0.027))
        )
        counterbore_cut = (
            cq.Workplane("XY")
            .circle(0.0145)
            .extrude(0.013)
            .translate((x, BUTTON_Y, ENCLOSURE_HEIGHT - 0.011))
        )
        body = body.cut(shaft_cut).cut(counterbore_cut)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="personal_speaker_kickstand")

    body_mat = model.material("soft_charcoal", rgba=(0.075, 0.080, 0.085, 1.0))
    grille_mat = model.material("black_perforated_grille", rgba=(0.010, 0.011, 0.012, 1.0))
    button_mat = model.material("matte_button_rubber", rgba=(0.150, 0.155, 0.160, 1.0))
    icon_mat = model.material("pale_button_icons", rgba=(0.82, 0.84, 0.82, 1.0))
    hinge_mat = model.material("brushed_pin_metal", rgba=(0.62, 0.63, 0.60, 1.0))
    stand_mat = model.material("dark_stand_panel", rgba=(0.050, 0.052, 0.055, 1.0))
    foot_mat = model.material("dark_rubber_feet", rgba=(0.020, 0.020, 0.020, 1.0))

    enclosure = model.part("enclosure")
    enclosure.visual(
        mesh_from_cadquery(_build_enclosure_shell(), "rounded_enclosure"),
        material=body_mat,
        name="case_shell",
    )
    enclosure.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.145, 0.076),
                0.004,
                hole_diameter=0.0045,
                pitch=(0.0090, 0.0090),
                frame=0.008,
                corner_radius=0.006,
                stagger=True,
            ),
            "speaker_grille",
        ),
        origin=Origin(
            xyz=(0.0, FRONT_Y - 0.002, 0.056),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_mat,
        name="front_grille",
    )
    enclosure.visual(
        Box((0.130, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0035, 0.017)),
        material=grille_mat,
        name="lower_acoustic_slot",
    )
    enclosure.visual(
        Cylinder(radius=0.0048, length=0.160),
        origin=Origin(xyz=(0.0, REAR_Y + 0.0065, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="hinge_pin",
    )
    for suffix, x in (("0", -0.060), ("1", 0.060)):
        enclosure.visual(
            Box((0.030, 0.007, 0.018)),
            origin=Origin(xyz=(x, REAR_Y + 0.0035, 0.020)),
            material=body_mat,
            name=f"rear_hinge_leaf_{suffix}",
        )
        enclosure.visual(
            Cylinder(radius=0.0052, length=0.028),
            origin=Origin(xyz=(x, REAR_Y + 0.0065, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"fixed_knuckle_{suffix}",
        )
    for suffix, x in (("0", -0.058), ("1", 0.058)):
        enclosure.visual(
            Box((0.030, 0.005, 0.004)),
            origin=Origin(xyz=(x, REAR_Y + 0.0025, 0.004)),
            material=foot_mat,
            name=f"rear_foot_{suffix}",
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.0045, length=0.048),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="stand_knuckle",
    )
    kickstand.visual(
        Box((0.094, 0.006, 0.074)),
        origin=Origin(xyz=(0.0, 0.0048, 0.037)),
        material=stand_mat,
        name="stand_panel",
    )
    kickstand.visual(
        Box((0.076, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, 0.0088, 0.071)),
        material=foot_mat,
        name="rubber_tip",
    )
    model.articulation(
        "enclosure_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=kickstand,
        origin=Origin(xyz=(0.0, REAR_Y + 0.0065, 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.6, lower=0.0, upper=1.35),
    )

    for index, x in enumerate(BUTTON_XS):
        button = model.part(f"top_button_{index}")
        button.visual(
            Cylinder(radius=0.0120, length=0.0060),
            origin=Origin(xyz=(0.0, 0.0, 0.0030)),
            material=button_mat,
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.0050, length=0.0180),
            origin=Origin(xyz=(0.0, 0.0, -0.0090)),
            material=button_mat,
            name="plunger",
        )
        if index == 0:
            button.visual(
                Box((0.011, 0.0020, 0.0008)),
                origin=Origin(xyz=(0.0, 0.0, 0.0064)),
                material=icon_mat,
                name="minus_icon",
            )
        else:
            button.visual(
                Box((0.011, 0.0020, 0.0008)),
                origin=Origin(xyz=(0.0, 0.0, 0.0064)),
                material=icon_mat,
                name="plus_bar",
            )
            button.visual(
                Box((0.0020, 0.011, 0.0008)),
                origin=Origin(xyz=(0.0, 0.0, 0.0065)),
                material=icon_mat,
                name="plus_stem",
            )
        model.articulation(
            f"enclosure_to_top_button_{index}",
            ArticulationType.PRISMATIC,
            parent=enclosure,
            child=button,
            origin=Origin(xyz=(x, BUTTON_Y, ENCLOSURE_HEIGHT + 0.0008)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    kickstand = object_model.get_part("kickstand")
    stand_joint = object_model.get_articulation("enclosure_to_kickstand")

    ctx.allow_overlap(
        enclosure,
        kickstand,
        elem_a="hinge_pin",
        elem_b="stand_knuckle",
        reason="The metal hinge pin is intentionally captured through the kickstand knuckle.",
    )
    ctx.expect_within(
        enclosure,
        kickstand,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="stand_knuckle",
        margin=0.0004,
        name="hinge pin is captured concentrically by the stand knuckle",
    )
    ctx.expect_overlap(
        enclosure,
        kickstand,
        axes="x",
        elem_a="hinge_pin",
        elem_b="stand_knuckle",
        min_overlap=0.040,
        name="hinge pin spans the kickstand knuckle",
    )

    rest_stand_aabb = ctx.part_element_world_aabb(kickstand, elem="rubber_tip")
    with ctx.pose({stand_joint: 1.35}):
        open_stand_aabb = ctx.part_element_world_aabb(kickstand, elem="rubber_tip")
    ctx.check(
        "kickstand swings rearward",
        rest_stand_aabb is not None
        and open_stand_aabb is not None
        and open_stand_aabb[1][1] > rest_stand_aabb[1][1] + 0.040,
        details=f"rest={rest_stand_aabb}, open={open_stand_aabb}",
    )

    for index in range(2):
        button = object_model.get_part(f"top_button_{index}")
        joint = object_model.get_articulation(f"enclosure_to_top_button_{index}")
        ctx.allow_overlap(
            enclosure,
            button,
            elem_a="case_shell",
            elem_b="plunger",
            reason="The short button plunger is intentionally captured in a simplified guide bore inside the case.",
        )
        ctx.expect_gap(
            button,
            enclosure,
            axis="z",
            positive_elem="cap",
            negative_elem="case_shell",
            min_gap=0.0003,
            max_gap=0.0020,
            name=f"top button {index} cap rests just proud of the case",
        )
        ctx.expect_within(
            button,
            enclosure,
            axes="xy",
            inner_elem="plunger",
            outer_elem="case_shell",
            margin=0.0,
            name=f"top button {index} plunger is inside the top opening footprint",
        )
        ctx.expect_overlap(
            button,
            enclosure,
            axes="z",
            elem_a="plunger",
            elem_b="case_shell",
            min_overlap=0.012,
            name=f"top button {index} plunger remains inserted in the case guide",
        )
        rest_cap_aabb = ctx.part_element_world_aabb(button, elem="cap")
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_cap_aabb = ctx.part_element_world_aabb(button, elem="cap")
        ctx.check(
            f"top button {index} plunges downward",
            rest_cap_aabb is not None
            and pressed_cap_aabb is not None
            and pressed_cap_aabb[0][2] < rest_cap_aabb[0][2] - 0.004,
            details=f"rest={rest_cap_aabb}, pressed={pressed_cap_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
