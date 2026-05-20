from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    brushed_steel = model.material(
        "brushed_stainless_steel", rgba=(0.72, 0.74, 0.72, 1.0)
    )
    darker_steel = model.material("shadowed_stainless", rgba=(0.45, 0.47, 0.46, 1.0))
    black_glass = model.material("black_glass_strip", rgba=(0.02, 0.025, 0.03, 1.0))
    grease_filter = model.material("dark_grease_filter", rgba=(0.12, 0.13, 0.13, 1.0))
    knob_plastic = model.material("satin_black_knob", rgba=(0.015, 0.015, 0.018, 1.0))
    white_mark = Material("white_pointer_mark", rgba=(0.92, 0.92, 0.86, 1.0))

    hood = model.part("hood")

    # A broad, rectangular stainless lower canopy with subtly eased edges.
    lower_canopy_shape = (
        cq.Workplane("XY")
        .box(0.54, 0.90, 0.16)
        .edges()
        .fillet(0.008)
        .translate((0.270, 0.0, 0.080))
    )
    hood.visual(
        mesh_from_cadquery(lower_canopy_shape, "lower_canopy"),
        material=brushed_steel,
        name="lower_canopy",
    )

    # The tall chimney cover sits toward the wall side and nests slightly into the
    # canopy top, as a real two-piece cover would.
    chimney_shape = (
        cq.Workplane("XY")
        .box(0.24, 0.32, 0.84)
        .edges()
        .fillet(0.006)
        .translate((0.135, 0.0, 0.565))
    )
    hood.visual(
        mesh_from_cadquery(chimney_shape, "upper_chimney_cover"),
        material=brushed_steel,
        name="upper_chimney_cover",
    )

    # A thin dark rear wall plate visually anchors both the chimney and canopy.
    hood.visual(
        Box((0.018, 0.54, 0.96)),
        origin=Origin(xyz=(0.002, 0.0, 0.500)),
        material=darker_steel,
        name="rear_mounting_plate",
    )

    # Front control strip proud of the canopy face.  The three rotary knobs mount
    # on this surface and are the only articulated parts.
    control_strip_x = 0.558
    hood.visual(
        Box((0.018, 0.62, 0.070)),
        origin=Origin(xyz=(0.549, 0.0, 0.096)),
        material=black_glass,
        name="control_strip",
    )

    # Underside grease-filter panels and baffle ribs make the object read as a
    # kitchen range hood rather than a plain box.
    for i, y in enumerate((-0.205, 0.205)):
        hood.visual(
            Box((0.260, 0.330, 0.008)),
            origin=Origin(xyz=(0.300, y, -0.002)),
            material=grease_filter,
            name=f"filter_panel_{i}",
        )
        for j, x in enumerate((0.205, 0.270, 0.335, 0.400)):
            hood.visual(
                Box((0.012, 0.300, 0.010)),
                origin=Origin(xyz=(x, y, -0.006)),
                material=darker_steel,
                name=f"filter_baffle_{i}_{j}",
            )

    hood.visual(
        Box((0.028, 0.88, 0.030)),
        origin=Origin(xyz=(0.526, 0.0, 0.025)),
        material=brushed_steel,
        name="front_lip",
    )

    knob_geometry = KnobGeometry(
        0.044,
        0.026,
        body_style="skirted",
        top_diameter=0.035,
        edge_radius=0.001,
        skirt=KnobSkirt(0.050, 0.006, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0010),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
        center=False,
    )
    knob_mesh = mesh_from_geometry(knob_geometry, "fluted_rotary_knob")

    for i, y in enumerate((-0.18, 0.0, 0.18)):
        knob = model.part(f"knob_{i}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=knob_plastic,
            name="knob_cap",
        )
        # A raised pointer mark on the front face rotates with the knob, making
        # the continuous front-to-back rotation legible.
        knob.visual(
            Box((0.003, 0.005, 0.022)),
            origin=Origin(xyz=(0.0270, 0.0, 0.008)),
            material=white_mark,
            name="pointer_mark",
        )
        model.articulation(
            f"hood_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=hood,
            child=knob,
            origin=Origin(xyz=(control_strip_x, y, 0.096)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(3)]
    joints = [object_model.get_articulation(f"hood_to_knob_{i}") for i in range(3)]

    ctx.check(
        "only three articulated knobs",
        len(object_model.articulations) == 3
        and all(j.child == f"knob_{i}" for i, j in enumerate(joints)),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "knobs rotate continuously about front-to-back axes",
        all(
            j.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in j.axis) == (1.0, 0.0, 0.0)
            for j in joints
        ),
        details=f"axes={[j.axis for j in joints]}",
    )

    y_positions = [j.origin.xyz[1] for j in joints]
    ctx.check(
        "knobs are evenly spaced across the control strip",
        abs((y_positions[1] - y_positions[0]) - (y_positions[2] - y_positions[1]))
        < 1e-6
        and abs(y_positions[1]) < 1e-6,
        details=f"y_positions={y_positions}",
    )

    for i, knob in enumerate(knobs):
        ctx.expect_gap(
            knob,
            hood,
            axis="x",
            positive_elem="knob_cap",
            negative_elem="control_strip",
            max_gap=0.002,
            max_penetration=0.0005,
            name=f"knob_{i} sits on the front control strip",
        )
        ctx.expect_within(
            knob,
            hood,
            axes="z",
            inner_elem="knob_cap",
            outer_elem="control_strip",
            margin=0.001,
            name=f"knob_{i} fits within strip height",
        )

    with ctx.pose({"hood_to_knob_1": math.pi / 2.0}):
        ctx.expect_gap(
            knobs[1],
            hood,
            axis="x",
            positive_elem="knob_cap",
            negative_elem="control_strip",
            max_gap=0.002,
            max_penetration=0.0005,
            name="center knob stays seated after quarter turn",
        )

    return ctx.report()


object_model = build_object_model()
