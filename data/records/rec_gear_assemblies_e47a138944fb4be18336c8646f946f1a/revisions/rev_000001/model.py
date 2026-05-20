from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_gear_train_bench")

    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_steel = Material("dark_burnished_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    brass = Material("oiled_bronze_gears", rgba=(0.82, 0.58, 0.22, 1.0))
    black = Material("black_handwheel", rgba=(0.03, 0.035, 0.035, 1.0))

    shaft_z = 0.32
    input_x = 0.0
    idler_x = 0.259
    output_x = 0.439
    side_y = 0.185
    gear_width = 0.055

    frame = model.part("side_frame")
    # A welded open bench: two side cheeks connected by a base tray and cross rails.
    frame.visual(Box((0.68, 0.48, 0.13)), origin=Origin(xyz=(0.20, 0.0, 0.065)), material=steel, name="base_tray")
    for y in (-side_y, side_y):
        frame.visual(Box((0.72, 0.025, 0.035)), origin=Origin(xyz=(0.20, y, 0.13)), material=steel, name=f"lower_rail_{y:+.2f}")
        frame.visual(Box((0.72, 0.025, 0.030)), origin=Origin(xyz=(0.20, y, 0.51)), material=steel, name=f"top_rail_{y:+.2f}")
        for x in (-0.16, 0.56):
            frame.visual(Box((0.035, 0.025, 0.57)), origin=Origin(xyz=(x, y, 0.30)), material=steel, name=f"end_post_{x:+.2f}_{y:+.2f}")
        for x in (input_x, idler_x, output_x):
            # Bearing bosses are open toroidal rings, with small vertical webs tying them to the rails.
            frame.visual(mesh_from_geometry(TorusGeometry(0.038, 0.010, radial_segments=18, tubular_segments=36), f"bearing_ring_{x:.2f}_{y:+.2f}"),
                         origin=Origin(xyz=(x, y, shaft_z), rpy=(math.pi / 2, 0.0, 0.0)), material=dark_steel, name=f"bearing_ring_{x:.2f}_{y:+.2f}")
            frame.visual(Box((0.026, 0.022, 0.132)), origin=Origin(xyz=(x, y, 0.431)), material=steel, name=f"upper_web_{x:.2f}_{y:+.2f}")
            frame.visual(Box((0.026, 0.022, 0.132)), origin=Origin(xyz=(x, y, 0.209)), material=steel, name=f"lower_web_{x:.2f}_{y:+.2f}")
    # Front and rear spacers make the frame read as one rigid bench assembly.
    for x in (-0.13, 0.52):
        frame.visual(Box((0.040, 0.42, 0.035)), origin=Origin(xyz=(x, 0.0, 0.13)), material=steel, name=f"cross_spacer_{x:+.0f}")

    def gear_mesh(teeth: int, root_radius: float, outer_radius: float, width: float, name: str):
        pts = []
        for i in range(teeth * 4):
            a = 2.0 * math.pi * i / (teeth * 4)
            # broad roots and flatter tooth tips give a readable spur-gear silhouette.
            r = outer_radius if i % 4 in (1, 2) else root_radius
            pts.append((r * math.cos(a), r * math.sin(a)))
        return mesh_from_geometry(ExtrudeGeometry.centered(pts, width), name)

    def add_shaft_part(part_name: str, x: float, teeth: int, pitch_radius: float, outer_radius: float, phase: float, handwheel: bool = False, mimic: Mimic | None = None):
        part = model.part(part_name)
        part.visual(Cylinder(radius=0.014, length=0.58), origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=dark_steel, name="shaft")
        # Retaining collars touch the bearing races so the revolving shafts are visibly supported.
        part.visual(Cylinder(radius=0.043, length=0.020), origin=Origin(xyz=(0.0, -0.2045, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=dark_steel, name="bearing_collar_0")
        part.visual(Cylinder(radius=0.043, length=0.020), origin=Origin(xyz=(0.0, 0.2045, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=dark_steel, name="bearing_collar_1")
        part.visual(Cylinder(radius=0.030, length=0.075), origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=dark_steel, name="gear_hub")
        part.visual(
            gear_mesh(teeth, pitch_radius - 0.010, outer_radius, gear_width, f"{part_name}_teeth"),
            origin=Origin(rpy=(math.pi / 2, 0.0, phase)),
            material=brass,
            name="spur_gear",
        )
        if handwheel:
            wheel_y = -0.275
            part.visual(mesh_from_geometry(TorusGeometry(0.080, 0.008, radial_segments=16, tubular_segments=48), "handwheel_rim"),
                        origin=Origin(xyz=(0.0, wheel_y, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=black, name="handwheel_rim")
            part.visual(Cylinder(radius=0.020, length=0.020), origin=Origin(xyz=(0.0, wheel_y, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=black, name="handwheel_hub")
            part.visual(Box((0.150, 0.010, 0.010)), origin=Origin(xyz=(0.0, wheel_y, 0.0)), material=black, name="handwheel_spoke_x")
            part.visual(Box((0.010, 0.010, 0.150)), origin=Origin(xyz=(0.0, wheel_y, 0.0)), material=black, name="handwheel_spoke_z")
            part.visual(Cylinder(radius=0.010, length=0.055), origin=Origin(xyz=(0.0, wheel_y - 0.035, 0.080), rpy=(math.pi / 2, 0.0, 0.0)), material=black, name="rim_handle")
        model.articulation(
            f"frame_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=part,
            origin=Origin(xyz=(x, 0.0, shaft_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0, lower=-math.pi, upper=math.pi),
            mimic=mimic,
        )
        return part

    # Odd tooth counts let the idler show a tooth between the two neighboring gear valleys.
    add_shaft_part("input_shaft", input_x, teeth=37, pitch_radius=0.148, outer_radius=0.156, phase=math.pi / (37 * 2), handwheel=True)
    add_shaft_part("idler_shaft", idler_x, teeth=23, pitch_radius=0.092, outer_radius=0.100, phase=math.pi, mimic=Mimic("frame_to_input_shaft", multiplier=-37.0 / 23.0))
    add_shaft_part("output_shaft", output_x, teeth=17, pitch_radius=0.068, outer_radius=0.076, phase=math.pi, mimic=Mimic("frame_to_input_shaft", multiplier=37.0 / 17.0))
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("side_frame")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")
    input_joint = object_model.get_articulation("frame_to_input_shaft")
    idler_joint = object_model.get_articulation("frame_to_idler_shaft")
    output_joint = object_model.get_articulation("frame_to_output_shaft")

    for joint in (input_joint, idler_joint, output_joint):
        ctx.check(
            f"{joint.name} is a shaft revolute joint",
            tuple(round(v, 3) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )
    ctx.check(
        "idler counter-rotates from input",
        idler_joint.mimic is not None and idler_joint.mimic.joint == "frame_to_input_shaft" and idler_joint.mimic.multiplier < 0.0,
        details=f"mimic={idler_joint.mimic}",
    )
    ctx.check(
        "output follows two gear meshes",
        output_joint.mimic is not None and output_joint.mimic.joint == "frame_to_input_shaft" and output_joint.mimic.multiplier > 0.0,
        details=f"mimic={output_joint.mimic}",
    )

    ctx.expect_overlap(input_shaft, idler_shaft, axes="z", elem_a="spur_gear", elem_b="spur_gear", min_overlap=0.12, name="large gear and idler share gear plane")
    ctx.expect_overlap(idler_shaft, output_shaft, axes="z", elem_a="spur_gear", elem_b="spur_gear", min_overlap=0.12, name="idler and output gear share gear plane")
    ctx.expect_gap(idler_shaft, input_shaft, axis="x", positive_elem="spur_gear", negative_elem="spur_gear", min_gap=0.0, max_gap=0.010, name="large gear nearly meshes with idler")
    ctx.expect_gap(output_shaft, idler_shaft, axis="x", positive_elem="spur_gear", negative_elem="spur_gear", min_gap=0.0, max_gap=0.012, name="idler nearly meshes with output gear")

    with ctx.pose({input_joint: 0.5}):
        ctx.expect_within(input_shaft, frame, axes="z", inner_elem="shaft", outer_elem="base_tray", margin=0.40, name="input shaft remains in bench height envelope")

    return ctx.report()


object_model = build_object_model()
