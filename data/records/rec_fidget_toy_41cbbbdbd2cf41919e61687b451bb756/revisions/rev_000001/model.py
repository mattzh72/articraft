from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


FACE_SIZE = 0.150
FACE_THICKNESS = 0.006
HINGE_OFFSET = 0.006
PANEL_Z = 0.006
PANEL_TOP_Z = PANEL_Z + FACE_THICKNESS / 2.0

BUTTON_RADIUS = 0.011
BUTTON_STEM_RADIUS = 0.006
BUTTON_STEM_LENGTH = 0.008
BUTTON_TRAVEL = 0.006
HOLE_RADIUS = 0.013
RING_OUTER_RADIUS = 0.016
RING_HEIGHT = 0.0025

BUTTON_XS = (
    HINGE_OFFSET + 0.030,
    HINGE_OFFSET + 0.075,
    HINGE_OFFSET + 0.120,
)
BUTTON_YS = (-0.0525, -0.0175, 0.0175, 0.0525)


def _button_mesh() -> MeshGeometry:
    """One-piece lathed pop dome with a narrow hidden stem."""
    radial_segments = 40
    dome_segments = 12
    geom = MeshGeometry()

    rings: list[list[int]] = []

    def add_ring(radius: float, z: float) -> list[int]:
        ring = []
        for i in range(radial_segments):
            angle = 2.0 * math.pi * i / radial_segments
            ring.append(geom.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z))
        rings.append(ring)
        return ring

    bottom_ring = add_ring(BUTTON_STEM_RADIUS, -BUTTON_STEM_LENGTH)
    stem_top_ring = add_ring(BUTTON_STEM_RADIUS, 0.0)
    dome_base_ring = add_ring(BUTTON_RADIUS, 0.0)
    dome_rings = [dome_base_ring]
    for i in range(1, dome_segments):
        theta = (math.pi / 2.0) * i / dome_segments
        dome_rings.append(add_ring(BUTTON_RADIUS * math.cos(theta), BUTTON_RADIUS * math.sin(theta)))

    bottom_center = geom.add_vertex(0.0, 0.0, -BUTTON_STEM_LENGTH)
    dome_tip = geom.add_vertex(0.0, 0.0, BUTTON_RADIUS)

    def stitch(a: list[int], b: list[int]) -> None:
        for i in range(radial_segments):
            j = (i + 1) % radial_segments
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])

    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        geom.add_face(bottom_center, bottom_ring[j], bottom_ring[i])

    stitch(bottom_ring, stem_top_ring)
    stitch(stem_top_ring, dome_base_ring)
    for lower, upper in zip(dome_rings, dome_rings[1:]):
        stitch(lower, upper)

    top_ring = dome_rings[-1]
    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        geom.add_face(top_ring[i], top_ring[j], dome_tip)

    return geom


def _face_panel_shape() -> cq.Workplane:
    """Rounded square silicone face with through holes and raised collars."""
    panel = (
        cq.Workplane("XY")
        .box(FACE_SIZE, FACE_SIZE, FACE_THICKNESS)
        .translate((HINGE_OFFSET + FACE_SIZE / 2.0, 0.0, PANEL_Z))
    )

    for x in BUTTON_XS:
        for y in BUTTON_YS:
            cutter = (
                cq.Workplane("XY")
                .center(x, y)
                .circle(HOLE_RADIUS)
                .extrude(FACE_THICKNESS * 3.0)
                .translate((0.0, 0.0, PANEL_Z - 1.5 * FACE_THICKNESS))
            )
            panel = panel.cut(cutter)

    top_z = PANEL_Z + FACE_THICKNESS / 2.0
    for x in BUTTON_XS:
        for y in BUTTON_YS:
            collar = (
                cq.Workplane("XY")
                .center(x, y)
                .circle(RING_OUTER_RADIUS)
                .circle(HOLE_RADIUS)
                .extrude(RING_HEIGHT)
                .translate((0.0, 0.0, top_z))
            )
            panel = panel.union(collar)

    # Low raised edge strips make the toy read as a molded square cube face.
    border_w = 0.004
    border_h = 0.002
    cx = HINGE_OFFSET + FACE_SIZE / 2.0
    half = FACE_SIZE / 2.0
    for y in (-half + border_w / 2.0, half - border_w / 2.0):
        rail = (
            cq.Workplane("XY")
            .box(FACE_SIZE, border_w, border_h)
            .translate((cx, y, top_z + border_h / 2.0))
        )
        panel = panel.union(rail)
    for x in (HINGE_OFFSET + border_w / 2.0, HINGE_OFFSET + FACE_SIZE - border_w / 2.0):
        rail = (
            cq.Workplane("XY")
            .box(border_w, FACE_SIZE, border_h)
            .translate((x, 0.0, top_z + border_h / 2.0))
        )
        panel = panel.union(rail)

    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_pop_it_cube_face")

    teal = model.material("teal_silicone", rgba=(0.07, 0.63, 0.74, 1.0))
    dark_teal = model.material("dark_teal_hinge", rgba=(0.04, 0.36, 0.43, 1.0))
    side_mat = model.material("aqua_side", rgba=(0.10, 0.52, 0.62, 1.0))
    rainbow = (
        model.material("button_red", rgba=(0.95, 0.18, 0.20, 1.0)),
        model.material("button_orange", rgba=(0.98, 0.48, 0.12, 1.0)),
        model.material("button_yellow", rgba=(0.98, 0.84, 0.16, 1.0)),
        model.material("button_green", rgba=(0.22, 0.70, 0.28, 1.0)),
        model.material("button_blue", rgba=(0.12, 0.36, 0.86, 1.0)),
        model.material("button_violet", rgba=(0.55, 0.22, 0.78, 1.0)),
    )

    side_face = model.part("side_face")
    side_face.visual(
        Box((0.006, FACE_SIZE, FACE_SIZE)),
        origin=Origin(xyz=(-0.006, 0.0, FACE_SIZE / 2.0)),
        material=side_mat,
        name="fixed_square_face",
    )

    # Two stationary hinge knuckles and leaves at the cube edge.
    root_knuckles = ((-0.054, 0.040), (0.054, 0.040))
    for index, (y_center, length) in enumerate(root_knuckles):
        side_face.visual(
            Cylinder(radius=0.0035, length=length),
            origin=Origin(xyz=(0.0, y_center, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_teal,
            name=f"hinge_barrel_{index}",
        )
        side_face.visual(
            Box((0.007, length, 0.003)),
            origin=Origin(xyz=(-0.0035, y_center, 0.0025)),
            material=dark_teal,
            name=f"hinge_leaf_{index}",
        )

    face = model.part("face")
    face.visual(
        mesh_from_cadquery(_face_panel_shape(), "pop_it_square_face", tolerance=0.0008),
        material=teal,
        name="molded_panel",
    )
    face.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(xyz=(0.0005, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_teal,
        name="center_hinge_barrel",
    )
    face.visual(
        Box((0.012, 0.040, 0.003)),
        origin=Origin(xyz=(0.006, 0.0, 0.0025)),
        material=dark_teal,
        name="center_hinge_leaf",
    )

    model.articulation(
        "side_face_to_face",
        ArticulationType.REVOLUTE,
        parent=side_face,
        child=face,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.57),
    )

    button_mesh = mesh_from_geometry(_button_mesh(), "pop_button_dome")
    for row, y in enumerate(BUTTON_YS):
        for col, x in enumerate(BUTTON_XS):
            index = row * len(BUTTON_XS) + col
            button = model.part(f"button_{row}_{col}")
            button.visual(
                button_mesh,
                material=rainbow[row % len(rainbow)],
                name="dome",
            )
            button.visual(
                Cylinder(radius=0.015, length=0.001),
                origin=Origin(xyz=(0.0, 0.0, RING_HEIGHT + 0.0005)),
                material=rainbow[row % len(rainbow)],
                name="seating_flange",
            )
            model.articulation(
                f"face_to_button_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=face,
                child=button,
                origin=Origin(xyz=(x, y, PANEL_TOP_Z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=0.35,
                    velocity=0.05,
                    lower=0.0,
                    upper=BUTTON_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    face = object_model.get_part("face")
    hinge = object_model.get_articulation("side_face_to_face")

    button_parts = [object_model.get_part(f"button_{r}_{c}") for r in range(4) for c in range(3)]
    button_joints = [
        object_model.get_articulation(f"face_to_button_{r}_{c}") for r in range(4) for c in range(3)
    ]

    ctx.check("twelve_pop_buttons", all(part is not None for part in button_parts), "Expected 12 dome buttons.")
    ctx.check(
        "twelve_prismatic_button_joints",
        all(
            joint is not None
            and joint.articulation_type == ArticulationType.PRISMATIC
            and joint.motion_limits is not None
            and joint.motion_limits.upper == BUTTON_TRAVEL
            for joint in button_joints
        ),
        "Each dome should have a short prismatic compression joint.",
    )

    for button in button_parts:
        if button is not None:
            ctx.expect_within(
                button,
                face,
                axes="xy",
                margin=0.001,
                name=f"{button.name}_within_square_face",
            )

    sample_button = object_model.get_part("button_1_1")
    sample_joint = object_model.get_articulation("face_to_button_1_1")
    rest_pos = ctx.part_world_position(sample_button)
    with ctx.pose({sample_joint: BUTTON_TRAVEL}):
        compressed_pos = ctx.part_world_position(sample_button)
    ctx.check(
        "sample_button_compresses_down",
        rest_pos is not None
        and compressed_pos is not None
        and compressed_pos[2] < rest_pos[2] - 0.004,
        details=f"rest={rest_pos}, compressed={compressed_pos}",
    )

    rest_face_aabb = ctx.part_world_aabb(face)
    with ctx.pose({hinge: 1.20}):
        folded_face_aabb = ctx.part_world_aabb(face)
    ctx.check(
        "face_folds_up_on_edge_hinge",
        rest_face_aabb is not None
        and folded_face_aabb is not None
        and folded_face_aabb[1][2] > rest_face_aabb[1][2] + 0.08,
        details=f"rest={rest_face_aabb}, folded={folded_face_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
