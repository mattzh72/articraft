from __future__ import annotations

import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    # 1. Mast (Base)
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.1, height=1.0),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        name="mast_column",
    )
    # Add a ground base plate
    mast.visual(
        Cylinder(radius=0.3, height=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="ground_plate",
    )

    # 2. Yoke (Pan stage)
    yoke = model.part("yoke")
    # Yoke dimensions
    yoke_width = 0.4
    yoke_depth = 0.2
    yoke_height = 0.3
    yoke_span = 0.3
    trunnion_dia = 0.04
    trunnion_z = 0.2
    base_thick = 0.05

    yoke_geom = TrunnionYokeGeometry(
        (yoke_width, yoke_depth, yoke_height),
        span_width=yoke_span,
        trunnion_diameter=trunnion_dia,
        trunnion_center_z=trunnion_z,
        base_thickness=base_thick,
        center=False,
    )
    yoke.visual(
        mesh_from_geometry(yoke_geom, "yoke_geom"),
        origin=Origin(), # Yoke part frame is at Z=1.0
        name="yoke_bracket",
    )

    # Pan articulation
    model.articulation(
        "mast_to_yoke",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )

    # 3. Lamp Head
    lamp = model.part("lamp")
    
    # Build hollow lamp head with CadQuery
    lamp_radius = 0.14
    lamp_length = 0.3
    bezel_depth = 0.05
    wall_thickness = 0.01

    # Main body along Y axis
    lamp_cq = (
        cq.Workplane("XZ")
        .cylinder(lamp_length, lamp_radius)
        .faces(">Y")
        .workplane()
        .circle(lamp_radius - wall_thickness)
        .extrude(-bezel_depth, combine="cut")
    )
    
    # Trunnions along X axis
    trunnion_radius = trunnion_dia / 2 + 0.001 # slightly larger for intentional overlap
    trunnion_length = yoke_width + 0.002 # slightly longer than yoke width for overlap
    trunnions_cq = cq.Workplane("YZ").cylinder(trunnion_length, trunnion_radius)
    
    lamp_cq = lamp_cq.union(trunnions_cq)

    # The lamp center is at (0, 0, trunnion_z) relative to the yoke base.
    # The yoke base part frame is at Z=1.0.
    lamp.visual(
        mesh_from_cadquery(lamp_cq, "lamp_body"),
        origin=Origin(),
        name="lamp_head",
    )

    # Tilt articulation
    model.articulation(
        "yoke_to_lamp",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, trunnion_z)),
        axis=(1.0, 0.0, 0.0), # Tilt around X axis
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-math.pi/4, upper=math.pi/3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Scoped allowance for trunnions inside the yoke bores
    ctx.allow_overlap(
        "lamp",
        "yoke",
        elem_a="lamp_head",
        elem_b="yoke_bracket",
        reason="Trunnions are captured inside the yoke cheek bores.",
    )
    
    ctx.expect_contact("yoke", "mast", name="Yoke sits on mast")
    ctx.expect_overlap("lamp", "yoke", axes="x", min_overlap=0.3, name="Lamp trunnions overlap yoke arms")

    return ctx.report()


object_model = build_object_model()
