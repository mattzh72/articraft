from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shallow_gas_cooktop_on_open_shelving")

    wood = model.material("oiled_wood", rgba=(0.62, 0.43, 0.25, 1.0))
    dark_frame = model.material("dark_powder_coated_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.73, 1.0))
    cast_iron = model.material("matte_cast_iron", rgba=(0.015, 0.015, 0.014, 1.0))
    burner_black = model.material("black_enamel", rgba=(0.02, 0.02, 0.018, 1.0))
    brass = model.material("warm_brass", rgba=(0.86, 0.58, 0.25, 1.0))
    knob_black = model.material("black_knob_plastic", rgba=(0.025, 0.025, 0.023, 1.0))
    pointer_white = model.material("white_pointer_fill", rgba=(0.92, 0.90, 0.84, 1.0))

    frame = model.part("worktop_shelving")

    # A real-height narrow counter: an open shelf frame supports a thin worktop,
    # with no closed appliance cabinet or base body below the cooktop.
    top_z = 0.835
    top_thickness = 0.055
    top_surface_z = top_z + top_thickness / 2.0
    frame.visual(
        Box((1.18, 0.54, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material=wood,
        name="worktop",
    )

    for x in (-0.55, 0.55):
        for y in (-0.24, 0.24):
            frame.visual(
                Box((0.045, 0.045, 0.815)),
                origin=Origin(xyz=(x, y, 0.4075)),
                material=dark_frame,
                name=f"post_{x:+.0f}_{y:+.0f}",
            )

    for z, name in ((0.18, "lower_shelf"), (0.47, "middle_shelf")):
        frame.visual(
            Box((1.08, 0.44, 0.035)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=wood,
            name=name,
        )

    for y, name in ((-0.24, "front_upper_rail"), (0.24, "rear_upper_rail")):
        frame.visual(
            Box((1.15, 0.035, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.725)),
            material=dark_frame,
            name=name,
        )
    for x, name in ((-0.55, "left_upper_rail"), (0.55, "right_upper_rail")):
        frame.visual(
            Box((0.035, 0.49, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.725)),
            material=dark_frame,
            name=name,
        )

    # Thin stainless cooktop sheet proud of the wood, plus only a shallow front
    # control lip.  It reads as an inset top, not as a range body.
    cooktop_panel = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.78, 0.43, 0.035, corner_segments=10),
            0.012,
            cap=True,
            center=True,
        ),
        "rounded_cooktop_panel",
    )
    panel_z = top_surface_z + 0.002
    frame.visual(
        cooktop_panel,
        origin=Origin(xyz=(0.0, 0.025, panel_z)),
        material=stainless,
        name="stainless_top",
    )

    lip_y = -0.276
    lip_front_y = lip_y - 0.018 / 2.0
    knob_z = 0.815
    frame.visual(
        Box((0.86, 0.018, 0.085)),
        origin=Origin(xyz=(0.0, lip_y, 0.8175)),
        material=stainless,
        name="front_lip",
    )

    burner_ring = mesh_from_geometry(
        TorusGeometry(radius=0.045, tube=0.0055, radial_segments=14, tubular_segments=36),
        "brass_burner_ring",
    )
    burner_positions = (
        (-0.235, -0.060),
        (0.235, -0.060),
        (-0.235, 0.175),
        (0.235, 0.175),
    )
    panel_top_z = panel_z + 0.006
    for index, (x, y) in enumerate(burner_positions):
        frame.visual(
            Cylinder(radius=0.070, length=0.006),
            origin=Origin(xyz=(x, y, panel_top_z + 0.0025)),
            material=burner_black,
            name=f"burner_base_{index}",
        )
        frame.visual(
            burner_ring,
            origin=Origin(xyz=(x, y, panel_top_z + 0.0085)),
            material=brass,
            name=f"burner_ring_{index}",
        )
        frame.visual(
            Cylinder(radius=0.037, length=0.012),
            origin=Origin(xyz=(x, y, panel_top_z + 0.011)),
            material=burner_black,
            name=f"burner_cap_{index}",
        )

        grate_z = panel_top_z + 0.039
        frame.visual(
            Box((0.175, 0.012, 0.014)),
            origin=Origin(xyz=(x, y, grate_z)),
            material=cast_iron,
            name=f"grate_cross_x_{index}",
        )
        frame.visual(
            Box((0.012, 0.175, 0.014)),
            origin=Origin(xyz=(x, y, grate_z)),
            material=cast_iron,
            name=f"grate_cross_y_{index}",
        )
        for side_y in (-0.072, 0.072):
            frame.visual(
                Box((0.168, 0.012, 0.014)),
                origin=Origin(xyz=(x, y + side_y, grate_z)),
                material=cast_iron,
                name=f"grate_side_x_{index}_{side_y:+.3f}",
            )
        for side_x in (-0.072, 0.072):
            frame.visual(
                Box((0.012, 0.168, 0.014)),
                origin=Origin(xyz=(x + side_x, y, grate_z)),
                material=cast_iron,
                name=f"grate_side_y_{index}_{side_x:+.3f}",
            )
        for fx in (-0.072, 0.072):
            for fy in (-0.072, 0.072):
                frame.visual(
                    Box((0.014, 0.014, 0.034)),
                    origin=Origin(xyz=(x + fx, y + fy, panel_top_z + 0.017)),
                    material=cast_iron,
                    name=f"grate_foot_{index}_{fx:+.0f}_{fy:+.0f}",
                )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.034,
            body_style="skirted",
            top_diameter=0.040,
            skirt=KnobSkirt(0.060, 0.006, flare=0.06, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            bore=KnobBore(style="d_shaft", diameter=0.007, flat_depth=0.001),
            center=False,
        ),
        "front_lip_knob",
    )

    knob_locations = {
        "left_knob_0": -0.345,
        "left_knob_1": -0.265,
        "right_knob_0": 0.265,
        "right_knob_1": 0.345,
    }
    for name, x in knob_locations.items():
        knob = model.part(name)
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.006, 0.0025, 0.024)),
            origin=Origin(xyz=(0.0, -0.0352, 0.007)),
            material=pointer_white,
            name="pointer_line",
        )
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=knob,
            origin=Origin(xyz=(x, lip_front_y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    knob_names = ("left_knob_0", "left_knob_1", "right_knob_0", "right_knob_1")
    joints = [object_model.get_articulation(f"{name}_spin") for name in knob_names]

    ctx.check(
        "only the four knobs articulate",
        len(object_model.articulations) == 4,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    for joint in joints:
        axis = tuple(joint.axis)
        ctx.check(
            f"{joint.name} is continuous front-to-back",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and abs(axis[0]) < 1e-6
            and abs(abs(axis[1]) - 1.0) < 1e-6
            and abs(axis[2]) < 1e-6,
            details=f"type={joint.articulation_type}, axis={axis}",
        )

    centers = [ctx.part_world_position(object_model.get_part(name)) for name in knob_names]
    ctx.check(
        "front center lip remains clear",
        all(center is not None and abs(center[0]) > 0.22 for center in centers),
        details=f"knob_centers={centers}",
    )

    with ctx.pose({"left_knob_0_spin": math.pi / 2.0, "right_knob_1_spin": -math.pi / 2.0}):
        moved = [
            ctx.part_world_position(object_model.get_part("left_knob_0")),
            ctx.part_world_position(object_model.get_part("right_knob_1")),
        ]
        ctx.check(
            "knob spin keeps pivots mounted",
            all(pos is not None and abs(pos[1] + 0.285) < 0.002 for pos in moved),
            details=f"posed_positions={moved}",
        )

    return ctx.report()


object_model = build_object_model()
