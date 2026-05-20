from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    shadow = model.material("dark_filter", rgba=(0.035, 0.038, 0.040, 1.0))
    black = model.material("black_control", rgba=(0.010, 0.011, 0.012, 1.0))
    knob_material = model.material("dark_brushed_knob", rgba=(0.16, 0.16, 0.15, 1.0))

    hood = model.part("hood")

    # Coordinate frame: +X across the hood, +Y from the front lip toward the wall,
    # +Z upward.  The front skirt is intentionally deep, as on chimney hoods
    # that hide the grease-filter cavity and controls behind a tall fascia.
    hood.visual(
        Box((0.92, 0.050, 0.180)),
        origin=Origin(xyz=(0.0, -0.285, 0.140)),
        material=stainless,
        name="deep_front_skirt",
    )
    hood.visual(
        Box((0.92, 0.500, 0.036)),
        origin=Origin(xyz=(0.0, -0.060, 0.038)),
        material=stainless,
        name="lower_shell",
    )
    hood.visual(
        Box((0.62, 0.300, 0.012)),
        origin=Origin(xyz=(-0.075, -0.070, 0.016)),
        material=shadow,
        name="recessed_filter",
    )
    hood.visual(
        Box((0.92, 0.445, 0.026)),
        origin=Origin(xyz=(0.0, -0.055, 0.265), rpy=(0.34, 0.0, 0.0)),
        material=stainless,
        name="sloped_canopy",
    )
    hood.visual(
        Box((0.030, 0.500, 0.210)),
        origin=Origin(xyz=(-0.445, -0.055, 0.140)),
        material=stainless,
        name="side_cheek_0",
    )
    hood.visual(
        Box((0.030, 0.500, 0.210)),
        origin=Origin(xyz=(0.445, -0.055, 0.140)),
        material=stainless,
        name="side_cheek_1",
    )
    hood.visual(
        Box((0.92, 0.060, 0.250)),
        origin=Origin(xyz=(0.0, 0.170, 0.160)),
        material=stainless,
        name="rear_wall_lip",
    )
    hood.visual(
        Box((0.360, 0.340, 0.080)),
        origin=Origin(xyz=(0.0, 0.075, 0.335)),
        material=stainless,
        name="chimney_collar",
    )
    hood.visual(
        Box((0.295, 0.295, 0.780)),
        origin=Origin(xyz=(0.0, 0.075, 0.765)),
        material=stainless,
        name="square_chimney",
    )
    hood.visual(
        Box((0.095, 0.006, 0.166)),
        origin=Origin(xyz=(0.365, -0.313, 0.140)),
        material=black,
        name="control_strip",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.026,
            body_style="skirted",
            top_diameter=0.034,
            edge_radius=0.0009,
            skirt=KnobSkirt(0.052, 0.006, flare=0.07, chamfer=0.0010),
            grip=KnobGrip(style="fluted", count=18, depth=0.0011),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        "range_hood_knob",
    )

    knob_x = 0.365
    knob_y = -0.316
    for index, knob_z in enumerate((0.200, 0.140, 0.080)):
        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_mesh,
            # KnobGeometry is built along local +Z.  Rotate it so the cap projects
            # outward from the front face along -Y while its rotation axis remains
            # the front-to-back Y axis of the hood.
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_material,
            name="cap",
        )
        model.articulation(
            f"hood_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=hood,
            child=knob,
            origin=Origin(xyz=(knob_x, knob_y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = list(object_model.articulations)
    ctx.check("only three controls articulate", len(joints) == 3)
    for index, joint in enumerate(joints):
        ctx.check(
            f"knob_{index} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint.name} type={joint.articulation_type}",
        )
        ctx.check(
            f"knob_{index} axis is front-to-back",
            abs(joint.axis[1]) > 0.99 and abs(joint.axis[0]) < 1e-6 and abs(joint.axis[2]) < 1e-6,
            details=f"{joint.name} axis={joint.axis}",
        )

    hood = object_model.get_part("hood")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(3)]
    for index, knob in enumerate(knobs):
        ctx.expect_gap(
            hood,
            knob,
            axis="y",
            max_gap=0.004,
            max_penetration=0.001,
            name=f"knob_{index} seats on the front control strip",
        )
        ctx.expect_overlap(
            knob,
            hood,
            axes="xz",
            min_overlap=0.020,
            elem_a="cap",
            elem_b="control_strip",
            name=f"knob_{index} footprint lies on the right front strip",
        )

    z_positions = [joint.origin.xyz[2] for joint in joints]
    ctx.check(
        "knobs form one vertical column",
        z_positions[0] > z_positions[1] > z_positions[2]
        and max(abs(joint.origin.xyz[0] - joints[0].origin.xyz[0]) for joint in joints) < 1e-6
        and max(abs(joint.origin.xyz[1] - joints[0].origin.xyz[1]) for joint in joints) < 1e-6,
        details=f"origins={[joint.origin.xyz for joint in joints]}",
    )

    with ctx.pose({"hood_to_knob_0": math.pi / 2.0, "hood_to_knob_1": -math.pi / 3.0, "hood_to_knob_2": math.pi}):
        for index, knob in enumerate(knobs):
            ctx.expect_gap(
                hood,
                knob,
                axis="y",
                max_gap=0.004,
                max_penetration=0.001,
                name=f"rotated knob_{index} remains seated",
            )

    return ctx.report()


object_model = build_object_model()
