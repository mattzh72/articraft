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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    brushed_steel = Material("brushed_steel", rgba=(0.42, 0.43, 0.42, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    black = Material("black_enamel", rgba=(0.01, 0.012, 0.014, 1.0))
    brass = Material("aged_brass", rgba=(0.72, 0.55, 0.24, 1.0))
    white = Material("engraved_white", rgba=(0.92, 0.92, 0.86, 1.0))
    wall_paint = Material("matte_wall_paint", rgba=(0.70, 0.68, 0.62, 1.0))

    for material in (brushed_steel, dark_steel, black, brass, white, wall_paint):
        model.materials.append(material)

    frame = model.part("frame")
    # A shallow wall plane and a welded safe frame surround the recessed opening.
    # The wall is split into four connected fields so the open safe reads as hollow.
    frame.visual(
        Box((0.130, 0.018, 0.980)),
        origin=Origin(xyz=(-0.385, -0.016, 0.0)),
        material=wall_paint,
        name="wall_left_field",
    )
    frame.visual(
        Box((0.130, 0.018, 0.980)),
        origin=Origin(xyz=(0.385, -0.016, 0.0)),
        material=wall_paint,
        name="wall_right_field",
    )
    frame.visual(
        Box((0.900, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, -0.016, 0.430)),
        material=wall_paint,
        name="wall_top_field",
    )
    frame.visual(
        Box((0.900, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, -0.016, -0.430)),
        material=wall_paint,
        name="wall_bottom_field",
    )
    frame.visual(
        Box((0.080, 0.060, 0.800)),
        origin=Origin(xyz=(-0.290, 0.012, 0.0)),
        material=brushed_steel,
        name="frame_left_jamb",
    )
    frame.visual(
        Box((0.080, 0.060, 0.800)),
        origin=Origin(xyz=(0.290, 0.012, 0.0)),
        material=brushed_steel,
        name="frame_right_jamb",
    )
    frame.visual(
        Box((0.660, 0.060, 0.080)),
        origin=Origin(xyz=(0.0, 0.012, 0.360)),
        material=brushed_steel,
        name="frame_top_rail",
    )
    frame.visual(
        Box((0.660, 0.060, 0.080)),
        origin=Origin(xyz=(0.0, 0.012, -0.360)),
        material=brushed_steel,
        name="frame_bottom_rail",
    )
    # Dark return walls and back plate make the safe read as a recessed box, not a flat panel.
    frame.visual(
        Box((0.035, 0.200, 0.670)),
        origin=Origin(xyz=(-0.252, -0.088, 0.0)),
        material=dark_steel,
        name="cavity_left_wall",
    )
    frame.visual(
        Box((0.035, 0.200, 0.670)),
        origin=Origin(xyz=(0.252, -0.088, 0.0)),
        material=dark_steel,
        name="cavity_right_wall",
    )
    frame.visual(
        Box((0.520, 0.200, 0.035)),
        origin=Origin(xyz=(0.0, -0.088, 0.318)),
        material=dark_steel,
        name="cavity_top_wall",
    )
    frame.visual(
        Box((0.520, 0.200, 0.035)),
        origin=Origin(xyz=(0.0, -0.088, -0.318)),
        material=dark_steel,
        name="cavity_bottom_wall",
    )
    frame.visual(
        Box((0.520, 0.020, 0.670)),
        origin=Origin(xyz=(0.0, -0.190, 0.0)),
        material=dark_steel,
        name="safe_back_plate",
    )
    frame.visual(
        Box((0.038, 0.032, 0.560)),
        origin=Origin(xyz=(0.278, 0.058, 0.0)),
        material=brushed_steel,
        name="fixed_hinge_leaf",
    )

    hinge_x = 0.235
    door_width = 0.455
    door_height = 0.600
    door_thickness = 0.075

    door = model.part("door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(-0.236, 0.040, 0.0)),
        material=dark_steel,
        name="door_slab",
    )
    door.visual(
        Box((0.395, 0.012, 0.525)),
        origin=Origin(xyz=(-0.236, 0.082, 0.0)),
        material=brushed_steel,
        name="raised_front_plate",
    )
    door.visual(
        Box((0.380, 0.006, 0.018)),
        origin=Origin(xyz=(-0.236, 0.090, 0.236)),
        material=dark_steel,
        name="upper_recess_line",
    )
    door.visual(
        Box((0.380, 0.006, 0.018)),
        origin=Origin(xyz=(-0.236, 0.090, -0.236)),
        material=dark_steel,
        name="lower_recess_line",
    )
    door.visual(
        Box((0.018, 0.006, 0.500)),
        origin=Origin(xyz=(-0.425, 0.090, 0.0)),
        material=dark_steel,
        name="left_recess_line",
    )
    door.visual(
        Box((0.018, 0.006, 0.500)),
        origin=Origin(xyz=(-0.047, 0.090, 0.0)),
        material=dark_steel,
        name="right_recess_line",
    )
    door.visual(
        Box((0.028, 0.060, 0.540)),
        origin=Origin(xyz=(-0.014, 0.040, 0.0)),
        material=brushed_steel,
        name="moving_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.570),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.75),
    )

    dial = model.part("dial")
    # Annular dial mesh leaves a real central bore for the handle shaft.
    dial_body = cq.Workplane("XZ").circle(0.082).circle(0.022).extrude(0.024)
    dial.visual(
        mesh_from_cadquery(dial_body, "combination_dial"),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=black,
        name="dial_annulus",
    )
    # Raised tick marks sit just proud of the dial face and rotate with it.
    for i in range(36):
        angle = (2.0 * math.pi * i) / 36.0
        major = i % 3 == 0
        length = 0.020 if major else 0.012
        thickness = 0.004 if major else 0.0024
        radius = 0.069 if major else 0.073
        dial.visual(
            Box((length, 0.003, thickness)),
            origin=Origin(
                xyz=(radius * math.cos(angle), 0.0235, radius * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=white if major else brass,
            name=f"dial_tick_{i}",
        )
    dial.visual(
        Box((0.045, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.0235, 0.050), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=white,
        name="dial_index_line",
    )

    door_center = (-0.236, 0.088, 0.0)
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=door_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="handle_shaft",
    )
    handle.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="handle_hub",
    )
    for i in range(3):
        angle = (2.0 * math.pi * i) / 3.0
        handle.visual(
            Box((0.150, 0.018, 0.020)),
            origin=Origin(
                xyz=(0.070 * math.cos(angle), 0.060, 0.070 * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=brass,
            name=f"handle_spoke_{i}",
        )
        handle.visual(
            Sphere(radius=0.022),
            origin=Origin(xyz=(0.150 * math.cos(angle), 0.060, 0.150 * math.sin(angle))),
            material=brass,
            name=f"handle_knob_{i}",
        )

    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=door_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-1.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    hinge = object_model.get_articulation("frame_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    handle_turn = object_model.get_articulation("door_to_handle")

    ctx.expect_gap(
        dial,
        door,
        axis="y",
        positive_elem="dial_annulus",
        negative_elem="raised_front_plate",
        max_gap=0.001,
        max_penetration=0.0002,
        name="combination dial seats on the door face",
    )
    ctx.expect_gap(
        handle,
        door,
        axis="y",
        positive_elem="handle_shaft",
        negative_elem="raised_front_plate",
        max_gap=0.001,
        max_penetration=0.0002,
        name="handle shaft seats on the same door center",
    )
    ctx.expect_origin_distance(
        dial,
        handle,
        axes="xyz",
        max_dist=0.0005,
        name="dial and three-spoke handle are coaxial",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "right-side hinge swings the door outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.25,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    with ctx.pose({dial_spin: math.pi, handle_turn: 0.75}):
        ctx.expect_origin_distance(
            dial,
            handle,
            axes="xyz",
            max_dist=0.0005,
            name="coaxial controls keep their centers while rotating",
        )

    return ctx.report()


object_model = build_object_model()
