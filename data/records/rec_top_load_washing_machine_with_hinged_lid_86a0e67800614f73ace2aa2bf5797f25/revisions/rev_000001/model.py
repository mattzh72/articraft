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


def _annular_cylinder(outer_radius: float, inner_radius: float, depth: float):
    """CadQuery annular cylinder centered on its local Z axis."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(depth)
        .translate((0.0, 0.0, -0.5 * depth))
    )


def _washer_top_deck(width: float, depth: float, thickness: float, opening_radius: float):
    blank = cq.Workplane("XY").box(width, depth, thickness)
    cutter = (
        cq.Workplane("XY")
        .circle(opening_radius)
        .extrude(thickness + 0.04)
        .translate((0.0, -0.03, -0.5 * (thickness + 0.04)))
    )
    return blank.cut(cutter)


def _wash_tub(outer_radius: float, inner_radius: float, height: float, bottom_thickness: float):
    shell = cq.Workplane("XY").circle(outer_radius).extrude(-height)
    hollow_cut = cq.Workplane("XY").circle(inner_radius).extrude(-(height - bottom_thickness))
    shell = shell.cut(hollow_cut)
    rolled_lip = cq.Workplane("XY").circle(outer_radius + 0.018).circle(inner_radius - 0.004).extrude(0.026)
    return shell.union(rolled_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_top_load_washer")

    enamel = model.material("warm_white_enamel", rgba=(0.92, 0.93, 0.90, 1.0))
    shadow = model.material("cavity_shadow", rgba=(0.06, 0.07, 0.08, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_plastic = model.material("charcoal_plastic", rgba=(0.04, 0.045, 0.05, 1.0))
    pale_gray = model.material("soft_gray_control", rgba=(0.72, 0.75, 0.76, 1.0))
    glass = model.material("smoked_lid_window", rgba=(0.16, 0.23, 0.30, 0.38))
    chrome = model.material("satin_chrome", rgba=(0.62, 0.64, 0.64, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(Box((0.72, 0.035, 0.86)), origin=Origin(xyz=(0.0, -0.3425, 0.43)), material=enamel, name="front_panel")
    cabinet.visual(Box((0.72, 0.035, 0.86)), origin=Origin(xyz=(0.0, 0.3425, 0.43)), material=enamel, name="rear_panel")
    cabinet.visual(Box((0.035, 0.72, 0.86)), origin=Origin(xyz=(-0.3425, 0.0, 0.43)), material=enamel, name="side_panel_0")
    cabinet.visual(Box((0.035, 0.72, 0.86)), origin=Origin(xyz=(0.3425, 0.0, 0.43)), material=enamel, name="side_panel_1")
    cabinet.visual(Box((0.72, 0.72, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.0275)), material=enamel, name="base_plinth")
    cabinet.visual(
        Cylinder(radius=0.050, length=0.200),
        origin=Origin(xyz=(0.0, -0.030, 0.155)),
        material=shadow,
        name="bearing_pedestal",
    )
    cabinet.visual(
        mesh_from_cadquery(_washer_top_deck(0.72, 0.72, 0.050, 0.300), "top_deck"),
        origin=Origin(xyz=(0.0, 0.0, 0.875)),
        material=enamel,
        name="top_deck",
    )
    cabinet.visual(Box((0.72, 0.090, 0.160)), origin=Origin(xyz=(0.0, 0.405, 0.980)), material=enamel, name="backsplash")
    cabinet.visual(Box((0.54, 0.006, 0.115)), origin=Origin(xyz=(0.06, 0.357, 0.995)), material=pale_gray, name="control_plate")
    cabinet.visual(Box((0.70, 0.025, 0.045)), origin=Origin(xyz=(0.0, 0.355, 0.878)), material=enamel, name="rear_top_land")

    for x in (-0.335, 0.335):
        cabinet.visual(
            Cylinder(radius=0.014, length=0.050),
            origin=Origin(xyz=(x, 0.250, 0.918), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"hinge_barrel_{0 if x < 0 else 1}",
        )
        cabinet.visual(
            Box((0.052, 0.034, 0.034)),
            origin=Origin(xyz=(x, 0.250, 0.896)),
            material=enamel,
            name=f"hinge_block_{0 if x < 0 else 1}",
        )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_wash_tub(0.252, 0.214, 0.550, 0.045), "hollow_wash_tub"),
        origin=Origin(),
        material=stainless,
        name="tub_shell",
    )
    tub.visual(Box((0.010, 0.020, 0.230)), origin=Origin(xyz=(0.252, 0.0, -0.215)), material=shadow, name="tub_index_mark")

    agitator_cap = model.part("agitator_cap")
    agitator_cap.visual(Cylinder(radius=0.034, length=0.300), origin=Origin(xyz=(0.0, 0.0, 0.150)), material=pale_gray, name="agitator_stem")
    agitator_cap.visual(Cylinder(radius=0.073, length=0.052), origin=Origin(xyz=(0.0, 0.0, 0.326)), material=enamel, name="cap_crown")
    agitator_cap.visual(Box((0.060, 0.012, 0.006)), origin=Origin(xyz=(0.027, 0.0, 0.355)), material=dark_plastic, name="cap_pointer")

    lid = model.part("lid")
    lid.visual(Box((0.600, 0.056, 0.030)), origin=Origin(xyz=(0.0, -0.028, 0.019)), material=enamel, name="rear_rail")
    lid.visual(Box((0.600, 0.056, 0.030)), origin=Origin(xyz=(0.0, -0.502, 0.019)), material=enamel, name="front_rail")
    lid.visual(Box((0.058, 0.420, 0.030)), origin=Origin(xyz=(-0.271, -0.265, 0.019)), material=enamel, name="side_rail_0")
    lid.visual(Box((0.058, 0.420, 0.030)), origin=Origin(xyz=(0.271, -0.265, 0.019)), material=enamel, name="side_rail_1")
    lid.visual(Box((0.492, 0.394, 0.010)), origin=Origin(xyz=(0.0, -0.265, 0.026)), material=glass, name="window")
    lid.visual(
        Cylinder(radius=0.015, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )

    dial = model.part("ring_dial")
    dial.visual(
        mesh_from_cadquery(_annular_cylinder(0.064, 0.038, 0.018), "ring_dial"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="dial_ring",
    )
    dial.visual(Box((0.014, 0.004, 0.030)), origin=Origin(xyz=(0.0, -0.010, 0.052)), material=chrome, name="dial_pointer")

    button_specs = [
        ("button_0", -0.020),
        ("button_1", 0.060),
        ("button_2", 0.140),
        ("button_3", 0.220),
    ]
    for button_name, x in button_specs:
        button = model.part(button_name)
        button.visual(Box((0.052, 0.018, 0.052)), origin=Origin(xyz=(0.0, -0.009, 0.0)), material=dark_plastic, name="button_cap")
        button.visual(Box((0.030, 0.006, 0.030)), origin=Origin(xyz=(0.0, -0.021, 0.0)), material=pale_gray, name="button_face")
        model.articulation(
            f"backsplash_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, 0.354, 0.988)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.10, lower=0.0, upper=0.012),
        )

    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, -0.030, 0.805)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=8.0),
    )
    model.articulation(
        "tub_to_agitator_cap",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=agitator_cap,
        origin=Origin(xyz=(0.0, 0.0, -0.505)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=10.0),
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.250, 0.9149)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "backsplash_to_ring_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(-0.205, 0.345, 0.995)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    tub = object_model.get_part("tub")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("ring_dial")
    cap = object_model.get_part("agitator_cap")

    lid_joint = object_model.get_articulation("cabinet_to_lid")
    tub_joint = object_model.get_articulation("cabinet_to_tub")
    dial_joint = object_model.get_articulation("backsplash_to_ring_dial")
    cap_joint = object_model.get_articulation("tub_to_agitator_cap")

    ctx.check("tub is a continuous vertical rotor", tub_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(tub_joint.axis) == (0.0, 0.0, 1.0))
    ctx.check("ring dial is a continuous backsplash rotor", dial_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_joint.axis) == (0.0, -1.0, 0.0))
    ctx.check("agitator cap is a continuous vertical rotor", cap_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(cap_joint.axis) == (0.0, 0.0, 1.0))

    ctx.allow_overlap(
        cap,
        tub,
        elem_a="agitator_stem",
        elem_b="tub_shell",
        reason="The agitator stem is intentionally centered inside the modeled hollow tub cavity and seated at the tub-bottom bearing.",
    )

    ctx.expect_gap(lid, cabinet, axis="z", positive_elem="window", negative_elem="top_deck", min_gap=0.010, max_gap=0.040, name="closed lid sits just above the top deck")
    ctx.expect_overlap(lid, cabinet, axes="xy", elem_a="window", elem_b="top_deck", min_overlap=0.35, name="lid window covers the washer opening")
    ctx.expect_within(tub, cabinet, axes="xy", inner_elem="tub_shell", outer_elem="top_deck", margin=0.10, name="wash tub is centered under the deck opening")
    ctx.expect_contact(cap, tub, elem_a="agitator_stem", elem_b="tub_shell", contact_tol=0.002, name="agitator stem seats on the tub bottom")
    ctx.expect_within(cap, tub, axes="xy", inner_elem="agitator_stem", outer_elem="tub_shell", margin=0.0, name="agitator stem stays centered in the tub cavity")

    tub_aabb = ctx.part_element_world_aabb(tub, elem="tub_shell")
    ctx.check(
        "wash tub reads as a deep hollow cavity",
        tub_aabb is not None and (tub_aabb[1][2] - tub_aabb[0][2]) > 0.52,
        details=f"tub_aabb={tub_aabb}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.20}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge lifts the lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.30,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    for i in range(4):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"backsplash_to_button_{i}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.012}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} pushes inward",
            joint.articulation_type == ArticulationType.PRISMATIC
            and rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.010,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )
        ctx.expect_gap(cabinet, button, axis="y", positive_elem="control_plate", negative_elem="button_cap", max_gap=0.001, max_penetration=0.001, name=f"button_{i} cap is seated on the control plate")

    ctx.expect_gap(cabinet, dial, axis="y", positive_elem="control_plate", negative_elem="dial_ring", max_gap=0.002, max_penetration=0.001, name="ring dial seats on the control plate")

    return ctx.report()


object_model = build_object_model()
