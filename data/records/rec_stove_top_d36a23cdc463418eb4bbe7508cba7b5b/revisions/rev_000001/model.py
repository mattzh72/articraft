from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_zone_induction_cooktop")

    glass = model.material("smoked_glass", rgba=(0.01, 0.012, 0.014, 0.92))
    black = model.material("matte_black", rgba=(0.02, 0.02, 0.022, 1.0))
    trim = model.material("dark_brushed_trim", rgba=(0.16, 0.16, 0.15, 1.0))
    print_gray = model.material("soft_gray_print", rgba=(0.74, 0.78, 0.78, 1.0))
    button_mat = model.material("satin_black_buttons", rgba=(0.035, 0.037, 0.04, 1.0))
    bushing_mat = model.material("black_plastic_bushing", rgba=(0.04, 0.04, 0.043, 1.0))

    cooktop = model.part("cooktop")

    housing_shape = (
        cq.Workplane("XY")
        .box(0.46, 0.32, 0.035)
        .edges("|Z")
        .fillet(0.018)
    )
    cooktop.visual(
        mesh_from_cadquery(housing_shape, "rounded_housing", tolerance=0.0007),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=black,
        name="rounded_housing",
    )

    glass_shape = (
        cq.Workplane("XY")
        .box(0.43, 0.265, 0.006)
        .edges("|Z")
        .fillet(0.014)
        .edges(">Z")
        .fillet(0.0012)
    )
    cooktop.visual(
        mesh_from_cadquery(glass_shape, "glass_surface", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.018, 0.038)),
        material=glass,
        name="glass_surface",
    )

    trim_shape = (
        cq.Workplane("XY")
        .box(0.43, 0.052, 0.010)
        .edges("|Z")
        .fillet(0.011)
        .edges(">Z")
        .fillet(0.001)
    )
    cooktop.visual(
        mesh_from_cadquery(trim_shape, "lower_trim", tolerance=0.0005),
        origin=Origin(xyz=(0.0, -0.141, 0.03985)),
        material=trim,
        name="lower_trim",
    )

    # Printed induction zone graphics, slightly sunk into the glass as applied ink.
    cooktop.visual(
        mesh_from_geometry(
            TorusGeometry(0.062, 0.00075, radial_segments=12, tubular_segments=72),
            "zone_ring_0",
        ),
        origin=Origin(xyz=(-0.105, 0.040, 0.0415)),
        material=print_gray,
        name="zone_ring_0",
    )
    cooktop.visual(
        mesh_from_geometry(
            TorusGeometry(0.038, 0.00055, radial_segments=10, tubular_segments=64),
            "inner_ring_0",
        ),
        origin=Origin(xyz=(-0.105, 0.040, 0.04135)),
        material=print_gray,
        name="inner_ring_0",
    )
    cooktop.visual(
        mesh_from_geometry(
            TorusGeometry(0.056, 0.00075, radial_segments=12, tubular_segments=72),
            "zone_ring_1",
        ),
        origin=Origin(xyz=(0.105, 0.040, 0.0415)),
        material=print_gray,
        name="zone_ring_1",
    )
    cooktop.visual(
        mesh_from_geometry(
            TorusGeometry(0.034, 0.00055, radial_segments=10, tubular_segments=64),
            "inner_ring_1",
        ),
        origin=Origin(xyz=(0.105, 0.040, 0.04135)),
        material=print_gray,
        name="inner_ring_1",
    )

    # Small front button wells printed/recessed into the lower trim.
    button_shape = (
        cq.Workplane("XY")
        .box(0.038, 0.024, 0.007)
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.0012)
    )
    well_shape = (
        cq.Workplane("XY")
        .box(0.052, 0.034, 0.001)
        .edges("|Z")
        .fillet(0.007)
    )
    button_xs = (-0.060, 0.0, 0.060)
    for i, x in enumerate(button_xs):
        cooktop.visual(
            mesh_from_cadquery(well_shape, f"button_well_{i}", tolerance=0.0004),
            origin=Origin(xyz=(x, -0.141, 0.04525)),
            material=black,
            name=f"button_well_{i}",
        )

    # A side bushing anchors the visible horizontal shaft of the rotary dial.
    cooktop.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.236, -0.056, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bushing_mat,
        name="side_bushing",
    )

    dial = model.part("side_dial")
    dial.visual(
        Cylinder(radius=0.008, length=0.019),
        origin=Origin(xyz=(0.0095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bushing_mat,
        name="dial_shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.022,
                body_style="faceted",
                base_diameter=0.050,
                top_diameter=0.044,
                edge_radius=0.0009,
                grip=KnobGrip(style="ribbed", count=18, depth=0.00075, width=0.0016),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "side_dial_cap",
        ),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_mat,
        name="dial_cap",
    )
    model.articulation(
        "cooktop_to_side_dial",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child=dial,
        origin=Origin(xyz=(0.242, -0.056, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=6.0),
    )

    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        button.visual(
            mesh_from_cadquery(button_shape, f"button_cap_{i}", tolerance=0.0004),
            origin=Origin(xyz=(0.0, 0.0, 0.0035)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"cooktop_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cooktop,
            child=button,
            origin=Origin(xyz=(x, -0.141, 0.04575)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cooktop = object_model.get_part("cooktop")
    dial = object_model.get_part("side_dial")
    dial_joint = object_model.get_articulation("cooktop_to_side_dial")
    buttons = [object_model.get_part(f"button_{i}") for i in range(3)]
    button_joints = [object_model.get_articulation(f"cooktop_to_button_{i}") for i in range(3)]

    ctx.check(
        "side dial is a continuous rotary control",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"joint={dial_joint}",
    )
    ctx.expect_contact(
        dial,
        cooktop,
        elem_a="dial_shaft",
        elem_b="side_bushing",
        contact_tol=0.001,
        name="dial shaft seats against the side bushing",
    )
    ctx.expect_overlap(
        dial,
        cooktop,
        axes="yz",
        min_overlap=0.010,
        elem_a="dial_shaft",
        elem_b="side_bushing",
        name="dial shaft is centered in the side bushing",
    )

    for i, (button, joint) in enumerate(zip(buttons, button_joints)):
        ctx.check(
            f"button {i} is an independent prismatic push control",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.motion_limits is not None
            and joint.motion_limits.upper == 0.004,
            details=f"joint={joint}",
        )
        ctx.expect_gap(
            button,
            cooktop,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="button_cap",
            negative_elem=f"button_well_{i}",
            name=f"button {i} rests on its trim well",
        )
        ctx.expect_overlap(
            button,
            cooktop,
            axes="xy",
            min_overlap=0.020,
            elem_a="button_cap",
            elem_b=f"button_well_{i}",
            name=f"button {i} is contained by its trim well",
        )

    rest_positions = [ctx.part_world_position(button) for button in buttons]
    with ctx.pose({button_joints[0]: 0.004}):
        pushed = ctx.part_world_position(buttons[0])
        unchanged = ctx.part_world_position(buttons[1])
    ctx.check(
        "front buttons move independently",
        rest_positions[0] is not None
        and pushed is not None
        and rest_positions[1] is not None
        and unchanged is not None
        and pushed[2] < rest_positions[0][2] - 0.0035
        and abs(unchanged[2] - rest_positions[1][2]) < 0.0005,
        details=f"rest={rest_positions}, pushed={pushed}, unchanged={unchanged}",
    )

    ctx.expect_within(
        cooktop,
        cooktop,
        axes="xy",
        inner_elem="zone_ring_0",
        outer_elem="glass_surface",
        margin=0.001,
        name="left induction zone is printed on the glass",
    )
    ctx.expect_within(
        cooktop,
        cooktop,
        axes="xy",
        inner_elem="zone_ring_1",
        outer_elem="glass_surface",
        margin=0.001,
        name="right induction zone is printed on the glass",
    )

    return ctx.report()


object_model = build_object_model()
