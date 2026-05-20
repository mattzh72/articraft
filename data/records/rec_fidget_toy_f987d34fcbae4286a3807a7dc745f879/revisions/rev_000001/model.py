from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CUBE_SIZE = 0.080
WELL_RADIUS = 0.030
WELL_DEPTH = 0.010
BEARING_RADIUS = 0.0055
BEARING_DEPTH = 0.016
PANEL_SIDE = 0.040
PANEL_THICKNESS = 0.005
PANEL_BOSS_RADIUS = 0.006
PANEL_BOSS_HEIGHT = 0.002
PANEL_STEM_RADIUS = 0.004
PANEL_STEM_LENGTH = BEARING_DEPTH - WELL_DEPTH / 2.0 - PANEL_THICKNESS / 2.0
POINTER_THICKNESS = 0.001


def _cube_body_geometry() -> cq.Workplane:
    """One-piece cube body with six circular recessed wells and center bearings."""
    body = cq.Workplane("XY").box(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE)
    body = body.edges().chamfer(0.003)

    for selector in (">Z", "<Z", ">X", "<X", ">Y", "<Y"):
        body = (
            body.faces(selector)
            .workplane(centerOption="CenterOfMass")
            .circle(BEARING_RADIUS)
            .cutBlind(-BEARING_DEPTH)
        )
        body = (
            body.faces(selector)
            .workplane(centerOption="CenterOfMass")
            .circle(WELL_RADIUS)
            .cutBlind(-WELL_DEPTH)
        )

    return body


def _panel_geometry() -> cq.Workplane:
    """A rounded square dial tile with a raised center boss and hidden stem."""
    plate = cq.Workplane("XY").box(PANEL_SIDE, PANEL_SIDE, PANEL_THICKNESS)
    plate = plate.edges("|Z").fillet(0.003)

    boss = (
        cq.Workplane("XY")
        .circle(PANEL_BOSS_RADIUS)
        .extrude(PANEL_BOSS_HEIGHT)
        .translate((0.0, 0.0, PANEL_THICKNESS / 2.0))
    )
    stem = (
        cq.Workplane("XY")
        .circle(PANEL_STEM_RADIUS)
        .extrude(PANEL_STEM_LENGTH)
        .translate((0.0, 0.0, -PANEL_THICKNESS / 2.0 - PANEL_STEM_LENGTH))
    )

    return plate.union(boss).union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="six_face_dial_fidget_cube")

    body_material = Material("warm_white_plastic", color=(0.86, 0.84, 0.78, 1.0))
    pointer_material = Material("engraved_black", color=(0.02, 0.02, 0.018, 1.0))
    panel_materials = [
        Material("panel_coral", color=(0.92, 0.22, 0.18, 1.0)),
        Material("panel_blue", color=(0.12, 0.39, 0.86, 1.0)),
        Material("panel_yellow", color=(0.96, 0.73, 0.13, 1.0)),
        Material("panel_green", color=(0.18, 0.62, 0.35, 1.0)),
        Material("panel_orange", color=(0.93, 0.43, 0.12, 1.0)),
        Material("panel_purple", color=(0.49, 0.23, 0.78, 1.0)),
    ]

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_cube_body_geometry(), "cube_body", tolerance=0.0006),
        material=body_material,
        name="cube_shell",
    )

    panel_mesh = mesh_from_cadquery(_panel_geometry(), "dial_panel", tolerance=0.0005)
    panel_center = CUBE_SIZE / 2.0 - WELL_DEPTH / 2.0
    face_mounts = [
        ((0.0, 0.0, panel_center), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, -panel_center), (math.pi, 0.0, 0.0)),
        ((panel_center, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
        ((-panel_center, 0.0, 0.0), (0.0, -math.pi / 2.0, 0.0)),
        ((0.0, panel_center, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
        ((0.0, -panel_center, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ]

    for index, (xyz, rpy) in enumerate(face_mounts):
        panel = model.part(f"face_panel_{index}")
        panel.visual(
            panel_mesh,
            material=panel_materials[index],
            name="dial_tile",
        )
        panel.visual(
            Box((0.0035, 0.023, POINTER_THICKNESS)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.009,
                    PANEL_THICKNESS / 2.0
                    + PANEL_BOSS_HEIGHT
                    + POINTER_THICKNESS / 2.0
                    - 0.0002,
                )
            ),
            material=pointer_material,
            name="pointer_mark",
        )

        model.articulation(
            f"body_to_panel_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=panel,
            origin=Origin(xyz=xyz, rpy=rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.35,
                velocity=12.0,
                lower=-math.pi,
                upper=math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    joints = [object_model.get_articulation(f"body_to_panel_{i}") for i in range(6)]
    panels = [object_model.get_part(f"face_panel_{i}") for i in range(6)]

    ctx.check("six independent face dials", len(joints) == 6 and len(panels) == 6)
    for i, joint in enumerate(joints):
        limits = joint.motion_limits
        ctx.check(
            f"panel_{i}_full_turn_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower <= -math.pi + 1e-6
            and limits.upper >= math.pi - 1e-6,
        )

    # Each square dial is smaller than the round well that surrounds it, so its
    # corners clear the pocket wall through a full in-plane spin.
    clear_radius = PANEL_SIDE * math.sqrt(2.0) / 2.0
    ctx.check(
        "square_panels_clear_round_wells",
        clear_radius + 0.001 < WELL_RADIUS,
        details=f"corner radius={clear_radius:.4f}, well radius={WELL_RADIUS:.4f}",
    )

    with ctx.pose({joints[0]: math.pi / 4.0}):
        ctx.expect_within(
            panels[0],
            body,
            axes="xy",
            margin=0.001,
            inner_elem="dial_tile",
            outer_elem="cube_shell",
            name="rotated_panel_0_stays_in_face_well",
        )

    return ctx.report()


object_model = build_object_model()
