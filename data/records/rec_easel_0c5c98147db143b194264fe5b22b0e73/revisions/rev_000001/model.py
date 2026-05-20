from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)
import cadquery as cq
import math


def _bearing_sleeve(outer_radius: float, inner_radius: float, length: float):
    """A clearanced vertical sliding sleeve for round guide rails."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length, both=True)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(length * 1.15, both=True)
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbalanced_studio_easel")

    beech = Material("varnished_beech", rgba=(0.62, 0.39, 0.18, 1.0))
    dark_wood = Material("dark_end_grain", rgba=(0.30, 0.18, 0.08, 1.0))
    black = Material("blackened_hardware", rgba=(0.02, 0.022, 0.025, 1.0))
    steel = Material("polished_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    canvas = Material("warm_canvas", rgba=(0.86, 0.80, 0.66, 1.0))
    lead = Material("counterweight_gray", rgba=(0.22, 0.23, 0.24, 1.0))

    guide_sleeve_mesh = mesh_from_cadquery(
        _bearing_sleeve(0.030, 0.018, 0.18),
        "guide_bearing_sleeve",
        tolerance=0.0008,
        angular_tolerance=0.05,
    )
    weight_sleeve_mesh = mesh_from_cadquery(
        _bearing_sleeve(0.023, 0.014, 0.13),
        "counterweight_bearing_sleeve",
        tolerance=0.0008,
        angular_tolerance=0.05,
    )

    frame = model.part("frame")

    # Wide floor frame: four overlapping wooden rails make a continuous footprint.
    frame.visual(Box((1.18, 0.075, 0.055)), origin=Origin(xyz=(0.0, -0.32, 0.0275)), material=beech, name="front_foot")
    frame.visual(Box((1.18, 0.075, 0.055)), origin=Origin(xyz=(0.0, 0.32, 0.0275)), material=beech, name="rear_foot")
    frame.visual(Box((0.075, 0.70, 0.055)), origin=Origin(xyz=(-0.5525, 0.0, 0.0275)), material=beech, name="side_foot_0")
    frame.visual(Box((0.075, 0.70, 0.055)), origin=Origin(xyz=(0.5525, 0.0, 0.0275)), material=beech, name="side_foot_1")
    frame.visual(Box((0.080, 0.70, 0.055)), origin=Origin(xyz=(-0.36, 0.0, 0.0275)), material=beech, name="mast_foot_0")
    frame.visual(Box((0.080, 0.70, 0.055)), origin=Origin(xyz=(0.36, 0.0, 0.0275)), material=beech, name="mast_foot_1")

    # Tall studio-easel uprights with cross ties.
    frame.visual(Box((0.060, 0.070, 1.64)), origin=Origin(xyz=(-0.36, 0.0, 0.86)), material=beech, name="upright_0")
    frame.visual(Box((0.060, 0.070, 1.64)), origin=Origin(xyz=(0.36, 0.0, 0.86)), material=beech, name="upright_1")
    frame.visual(Box((0.82, 0.075, 0.055)), origin=Origin(xyz=(0.0, 0.0, 1.67)), material=beech, name="top_crossbar")
    frame.visual(Box((0.78, 0.065, 0.050)), origin=Origin(xyz=(0.0, 0.0, 0.44)), material=beech, name="lower_crossbar")
    frame.visual(Box((0.76, 0.052, 0.045)), origin=Origin(xyz=(0.0, -0.06, 1.28)), material=beech, name="upper_cross_tie")
    frame.visual(Box((0.42, 0.24, 0.040)), origin=Origin(xyz=(0.0, 0.105, 0.42)), material=black, name="rear_lower_tie")
    frame.visual(Box((0.42, 0.24, 0.040)), origin=Origin(xyz=(0.0, 0.105, 1.67)), material=black, name="rear_upper_tie")

    # Front linear guide rails carried on small stand-off brackets from the uprights.
    for i, x in enumerate((-0.275, 0.275)):
        frame.visual(Cylinder(radius=0.012, length=1.27), origin=Origin(xyz=(x, -0.082, 1.055)), material=steel, name=f"guide_rail_{i}")
        bracket_x = x - 0.040 if x < 0 else x + 0.040
        frame.visual(Box((0.105, 0.035, 0.040)), origin=Origin(xyz=(bracket_x, -0.055, 0.43)), material=black, name=f"lower_rail_bracket_{i}")
        frame.visual(Box((0.105, 0.035, 0.040)), origin=Origin(xyz=(bracket_x, -0.055, 1.68)), material=black, name=f"upper_rail_bracket_{i}")

    # Rear counterweight guide rods.
    for i, x in enumerate((-0.135, 0.135)):
        frame.visual(Cylinder(radius=0.009, length=1.28), origin=Origin(xyz=(x, 0.235, 1.05)), material=steel, name=f"weight_rail_{i}")
        frame.visual(Box((0.050, 0.065, 0.040)), origin=Origin(xyz=(x, 0.205, 0.42)), material=black, name=f"weight_lower_bracket_{i}")
        frame.visual(Box((0.050, 0.065, 0.040)), origin=Origin(xyz=(x, 0.205, 1.67)), material=black, name=f"weight_upper_bracket_{i}")

    # Pulleys and visible counterbalance cable over the head of the easel.
    for i, x in enumerate((-0.275, 0.275)):
        frame.visual(
            Cylinder(radius=0.046, length=0.026),
            origin=Origin(xyz=(x, -0.100, 1.755), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"top_pulley_{i}",
        )
        frame.visual(Box((0.055, 0.120, 0.055)), origin=Origin(xyz=(x, -0.052, 1.705)), material=black, name=f"pulley_standoff_{i}")
        frame.visual(
            Cylinder(radius=0.018, length=0.034),
            origin=Origin(xyz=(x, -0.100, 1.755), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"pulley_hub_{i}",
        )
        cable = wire_from_points(
            [
                (x, -0.108, 1.52),
                (x, -0.108, 1.755),
                (0.0, 0.200, 1.755),
                (0.0, 0.200, 1.54),
            ],
            radius=0.003,
            radial_segments=10,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.025,
            corner_segments=8,
        )
        frame.visual(mesh_from_geometry(cable, f"counterbalance_cable_{i}"), material=black, name=f"counterbalance_cable_{i}")

    cradle = model.part("canvas_cradle")
    # Child frame is the carriage center; at q=0 it sits at world z=0.62.
    cradle.visual(Box((0.70, 0.095, 0.050)), origin=Origin(xyz=(0.0, -0.205, -0.015)), material=beech, name="canvas_shelf")
    cradle.visual(Box((0.72, 0.030, 0.110)), origin=Origin(xyz=(0.0, -0.255, 0.030)), material=dark_wood, name="front_lip")
    cradle.visual(Box((0.64, 0.025, 0.670)), origin=Origin(xyz=(0.0, -0.232, 0.365)), material=canvas, name="sample_canvas")
    cradle.visual(Box((0.68, 0.040, 0.050)), origin=Origin(xyz=(0.0, -0.150, 0.080)), material=beech, name="lower_carriage_bar")
    cradle.visual(Box((0.035, 0.040, 0.560)), origin=Origin(xyz=(-0.315, -0.170, 0.335)), material=beech, name="side_stile_0")
    cradle.visual(Box((0.035, 0.040, 0.560)), origin=Origin(xyz=(0.315, -0.170, 0.335)), material=beech, name="side_stile_1")
    cradle.visual(Box((0.64, 0.034, 0.045)), origin=Origin(xyz=(0.0, -0.150, 0.585)), material=beech, name="top_canvas_clamp")
    cradle.visual(Box((0.070, 0.050, 0.070)), origin=Origin(xyz=(-0.285, -0.150, 0.585)), material=black, name="clamp_block_0")
    cradle.visual(Box((0.070, 0.050, 0.070)), origin=Origin(xyz=(0.285, -0.150, 0.585)), material=black, name="clamp_block_1")

    for i, x in enumerate((-0.275, 0.275)):
        cradle.visual(guide_sleeve_mesh, origin=Origin(xyz=(x, -0.082, 0.100)), material=steel, name=f"guide_sleeve_{i}")
        cradle.visual(Box((0.014, 0.008, 0.125)), origin=Origin(xyz=(x, -0.098, 0.100)), material=black, name=f"guide_pad_{i}")
        cradle.visual(Box((0.040, 0.070, 0.040)), origin=Origin(xyz=(x, -0.145, 0.100)), material=black, name=f"sleeve_arm_{i}")
        cradle.visual(Box((0.038, 0.075, 0.100)), origin=Origin(xyz=(x, -0.170, 0.040)), material=black, name=f"shelf_gusset_{i}")

    counterweight = model.part("counterweight")
    counterweight.visual(Box((0.210, 0.050, 0.280)), origin=Origin(xyz=(0.0, -0.055, 0.0)), material=lead, name="weight_block")
    counterweight.visual(Box((0.170, 0.020, 0.035)), origin=Origin(xyz=(0.0, -0.030, 0.155)), material=black, name="cable_yoke")
    for i, x in enumerate((-0.135, 0.135)):
        counterweight.visual(weight_sleeve_mesh, origin=Origin(xyz=(x, 0.0, 0.0)), material=steel, name=f"weight_sleeve_{i}")
        counterweight.visual(Box((0.011, 0.007, 0.090)), origin=Origin(xyz=(x, -0.0125, 0.0)), material=black, name=f"weight_guide_pad_{i}")
        counterweight.visual(Box((0.050, 0.025, 0.030)), origin=Origin(xyz=(x * 0.82, -0.026, 0.0)), material=black, name=f"weight_sleeve_arm_{i}")

    slide = model.articulation(
        "cradle_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.66),
        motion_properties=MotionProperties(damping=8.0, friction=1.5),
    )
    model.articulation(
        "counterweight_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=counterweight,
        origin=Origin(xyz=(0.0, 0.235, 1.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=-0.66, upper=0.0),
        motion_properties=MotionProperties(damping=8.0, friction=1.5),
        mimic=Mimic(joint=slide.name, multiplier=-1.0, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    cradle = object_model.get_part("canvas_cradle")
    counterweight = object_model.get_part("counterweight")
    slide = object_model.get_articulation("cradle_slide")

    # Each carriage sleeve is centered around its matching rail in projection and
    # retains a long insertion at both ends of the intended travel.
    for i in range(2):
        ctx.expect_within(
            frame,
            cradle,
            axes="xy",
            inner_elem=f"guide_rail_{i}",
            outer_elem=f"guide_sleeve_{i}",
            margin=0.002,
            name=f"rail {i} is captured by sliding sleeve at rest",
        )
        ctx.expect_overlap(
            frame,
            cradle,
            axes="z",
            elem_a=f"guide_rail_{i}",
            elem_b=f"guide_sleeve_{i}",
            min_overlap=0.16,
            name=f"rail {i} overlaps sleeve length at rest",
        )

    rest_cradle = ctx.part_world_position(cradle)
    rest_weight = ctx.part_world_position(counterweight)
    with ctx.pose({slide: 0.60}):
        for i in range(2):
            ctx.expect_within(
                frame,
                cradle,
                axes="xy",
                inner_elem=f"guide_rail_{i}",
                outer_elem=f"guide_sleeve_{i}",
                margin=0.002,
                name=f"rail {i} remains captured by raised sleeve",
            )
            ctx.expect_overlap(
                frame,
                cradle,
                axes="z",
                elem_a=f"guide_rail_{i}",
                elem_b=f"guide_sleeve_{i}",
                min_overlap=0.16,
                name=f"rail {i} overlaps sleeve length when raised",
            )
        raised_cradle = ctx.part_world_position(cradle)
        raised_weight = ctx.part_world_position(counterweight)

    ctx.check(
        "canvas cradle slides upward on the uprights",
        rest_cradle is not None and raised_cradle is not None and raised_cradle[2] > rest_cradle[2] + 0.55,
        details=f"rest={rest_cradle}, raised={raised_cradle}",
    )
    ctx.check(
        "counterweight travels opposite the cradle",
        rest_weight is not None and raised_weight is not None and raised_weight[2] < rest_weight[2] - 0.55,
        details=f"rest={rest_weight}, raised={raised_weight}",
    )

    return ctx.report()


object_model = build_object_model()
