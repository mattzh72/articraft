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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


AXIS_Z = 0.72


def _bearing_ring_mesh():
    """Annular bearing housing: local Z is the trunnion/bearing axis."""
    return mesh_from_cadquery(
        cq.Workplane("XY")
        .circle(0.145)
        .circle(0.064)
        .extrude(0.06, both=True),
        "drawbridge_bearing_ring",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("weathered_concrete", rgba=(0.55, 0.55, 0.51, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.10, 0.12, 0.14, 1.0))
    worn_steel = model.material("worn_grey_steel", rgba=(0.42, 0.44, 0.45, 1.0))
    asphalt = model.material("dark_asphalt", rgba=(0.055, 0.057, 0.055, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))
    rust = model.material("rust_brown", rgba=(0.46, 0.20, 0.10, 1.0))

    bearing_ring = _bearing_ring_mesh()

    shore_frame = model.part("shore_frame")

    # Concrete shore abutment and machinery sill.  The small forward nose stops
    # just short of the moving leaf so the long deck reads as a separate panel.
    shore_frame.visual(
        Box((0.72, 1.18, 0.18)),
        origin=Origin(xyz=(-0.30, 0.0, 0.09)),
        material=concrete,
        name="foundation_slab",
    )
    shore_frame.visual(
        Box((0.42, 1.06, 0.34)),
        origin=Origin(xyz=(-0.23, 0.0, 0.35)),
        material=concrete,
        name="abutment_wall",
    )
    shore_frame.visual(
        Box((0.08, 0.95, 0.06)),
        origin=Origin(xyz=(0.015, 0.0, 0.55)),
        material=concrete,
        name="shore_sill",
    )

    # Two side bearing pedestals and a cross head make the fixed frame one
    # continuous supported assembly.
    for side, y in (("bearing_0", -0.56), ("bearing_1", 0.56)):
        shore_frame.visual(
            Box((0.22, 0.15, 0.50)),
            origin=Origin(xyz=(0.0, y, 0.35)),
            material=dark_steel,
            name=f"{side}_pedestal",
        )
        shore_frame.visual(
            bearing_ring,
            origin=Origin(xyz=(0.0, y, AXIS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"{side}_ring",
        )
        shore_frame.visual(
            Box((0.11, 0.17, 0.09)),
            origin=Origin(xyz=(-0.105, y, AXIS_Z)),
            material=dark_steel,
            name=f"{side}_rear_lug",
        )

        # Heavy cap bolts on the outboard faces of the bearing rings.
        face_y = y + (0.044 if y > 0 else -0.044)
        for strap_i, strap_x in enumerate((-0.090, 0.090)):
            shore_frame.visual(
                Box((0.045, 0.022, 0.30)),
                origin=Origin(xyz=(strap_x, face_y, AXIS_Z - 0.020)),
                material=dark_steel,
                name=f"{side}_cap_strap_{strap_i}",
            )
        for i, (bx, bz) in enumerate(
            ((0.090, AXIS_Z + 0.070), (-0.090, AXIS_Z + 0.070), (0.090, AXIS_Z - 0.070), (-0.090, AXIS_Z - 0.070))
        ):
            shore_frame.visual(
                Cylinder(radius=0.012, length=0.026),
                origin=Origin(xyz=(bx, face_y, bz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rust,
                name=f"{side}_bolt_{i}",
            )

    shore_frame.visual(
        Box((0.24, 1.30, 0.09)),
        origin=Origin(xyz=(-0.03, 0.0, 0.885)),
        material=dark_steel,
        name="crosshead_beam",
    )
    shore_frame.visual(
        Box((0.16, 1.16, 0.055)),
        origin=Origin(xyz=(-0.05, 0.0, 0.615)),
        material=dark_steel,
        name="bearing_tie_beam",
    )

    bridge_leaf = model.part("bridge_leaf")

    # The leaf frame is the hinge line.  All deck geometry extends along +X,
    # making it long and thin relative to the compact shore frame.
    bridge_leaf.visual(
        Cylinder(radius=0.045, length=1.40),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_pin",
    )
    bridge_leaf.visual(
        Box((0.10, 0.78, 0.09)),
        origin=Origin(xyz=(0.050, 0.0, -0.030)),
        material=dark_steel,
        name="hinge_web",
    )
    bridge_leaf.visual(
        Box((2.36, 0.88, 0.060)),
        origin=Origin(xyz=(1.26, 0.0, -0.075)),
        material=worn_steel,
        name="deck_panel",
    )
    bridge_leaf.visual(
        Box((2.22, 0.70, 0.010)),
        origin=Origin(xyz=(1.30, 0.0, -0.040)),
        material=asphalt,
        name="road_surface",
    )

    for y in (-0.465, 0.465):
        bridge_leaf.visual(
            Box((2.25, 0.070, 0.080)),
            origin=Origin(xyz=(1.245, y, -0.005)),
            material=safety_yellow,
            name=f"side_curb_{0 if y < 0 else 1}",
        )
        bridge_leaf.visual(
            Box((2.32, 0.065, 0.125)),
            origin=Origin(xyz=(1.255, y * 0.93, -0.135)),
            material=dark_steel,
            name=f"side_girder_{0 if y < 0 else 1}",
        )

    for i, x in enumerate((0.36, 0.78, 1.20, 1.62, 2.04, 2.38)):
        bridge_leaf.visual(
            Box((0.045, 0.78, 0.085)),
            origin=Origin(xyz=(x, 0.0, -0.130)),
            material=dark_steel,
            name=f"floor_beam_{i}",
        )

    bridge_leaf.visual(
        Box((0.080, 0.82, 0.105)),
        origin=Origin(xyz=(2.47, 0.0, -0.072)),
        material=dark_steel,
        name="toe_nose",
    )

    model.articulation(
        "leaf_pivot",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    pivot = object_model.get_articulation("leaf_pivot")

    ctx.check(
        "single revolute drawbridge leaf",
        len(object_model.parts) == 2
        and len(object_model.articulations) == 1
        and pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot.axis == (0.0, -1.0, 0.0),
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}, axis={pivot.axis}",
    )

    with ctx.pose({pivot: 0.0}):
        ctx.expect_within(
            bridge_leaf,
            shore_frame,
            axes="xz",
            inner_elem="trunnion_pin",
            outer_elem="bearing_0_ring",
            margin=0.002,
            name="trunnion sits in first side bearing",
        )
        ctx.expect_within(
            bridge_leaf,
            shore_frame,
            axes="xz",
            inner_elem="trunnion_pin",
            outer_elem="bearing_1_ring",
            margin=0.002,
            name="trunnion sits in second side bearing",
        )
        ctx.expect_overlap(
            bridge_leaf,
            shore_frame,
            axes="y",
            elem_a="trunnion_pin",
            elem_b="bearing_0_ring",
            min_overlap=0.08,
            name="first bearing captures trunnion span",
        )
        ctx.expect_overlap(
            bridge_leaf,
            shore_frame,
            axes="y",
            elem_a="trunnion_pin",
            elem_b="bearing_1_ring",
            min_overlap=0.08,
            name="second bearing captures trunnion span",
        )

        deck_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="deck_panel")
        frame_aabb = ctx.part_world_aabb(shore_frame)
        if deck_aabb is not None and frame_aabb is not None:
            deck_dx = deck_aabb[1][0] - deck_aabb[0][0]
            deck_dz = deck_aabb[1][2] - deck_aabb[0][2]
            frame_dx = frame_aabb[1][0] - frame_aabb[0][0]
            ctx.check(
                "bridge leaf is long and thin",
                deck_dx > 2.20 and deck_dz < 0.075 and deck_dx > frame_dx * 2.5,
                details=f"deck_dx={deck_dx:.3f}, deck_dz={deck_dz:.3f}, frame_dx={frame_dx:.3f}",
            )
        else:
            ctx.fail("bridge leaf is long and thin", "missing deck or frame AABB")

        closed_toe = ctx.part_element_world_aabb(bridge_leaf, elem="toe_nose")

    with ctx.pose({pivot: 1.10}):
        raised_toe = ctx.part_element_world_aabb(bridge_leaf, elem="toe_nose")
        ctx.check(
            "positive pivot raises the free end",
            closed_toe is not None
            and raised_toe is not None
            and raised_toe[1][2] > closed_toe[1][2] + 1.4,
            details=f"closed_toe={closed_toe}, raised_toe={raised_toe}",
        )

    return ctx.report()


object_model = build_object_model()
