from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_bike_steering")
    
    # -------------------------------------------------------------------------
    # FORK
    # -------------------------------------------------------------------------
    fork = model.part("fork")
    
    # Steerer: round tube, radius 14.3mm, length 0.25m, centered at origin in XY, goes up Z.
    steerer_cq = cq.Workplane("XY").circle(0.0143).extrude(0.25)
    
    # Crown: connects steerer to legs.
    crown_cq = cq.Workplane("XY").rect(0.04, 0.14).extrude(-0.03).edges("|Z").fillet(0.015)
    
    # Left leg: sweeps from crown down to dropout
    left_path = cq.Workplane("XZ", origin=(0, 0.06, 0)).moveTo(0, -0.03).lineTo(0.04, -0.4)
    left_profile = cq.Workplane("XY", origin=(0, 0.06, -0.03)).circle(0.012)
    left_leg_cq = left_profile.sweep(left_path, isFrenet=True)
    
    # Right leg:
    right_path = cq.Workplane("XZ", origin=(0, -0.06, 0)).moveTo(0, -0.03).lineTo(0.04, -0.4)
    right_profile = cq.Workplane("XY", origin=(0, -0.06, -0.03)).circle(0.012)
    right_leg_cq = right_profile.sweep(right_path, isFrenet=True)
    
    fork_cq = steerer_cq.union(crown_cq).union(left_leg_cq).union(right_leg_cq)
    fork.visual(mesh_from_cadquery(fork_cq, "fork_geom"))

    # -------------------------------------------------------------------------
    # STEM
    # -------------------------------------------------------------------------
    stem = model.part("stem")
    
    # Stem clamps around the steerer. Clamp from Z=0 to Z=0.04 in local coordinates.
    # We will place the stem at Z=0.20 on the fork.
    stem_clamp_cq = cq.Workplane("XY").circle(0.02).extrude(0.04)
    stem_clamp_cq = stem_clamp_cq.faces("<Z").workplane().circle(0.0143).cutThruAll()
    
    # Stem extension: lofts forward and up.
    stem_ext_cq = (
        cq.Workplane("YZ", origin=(0.015, 0, 0.02))
        .rect(0.03, 0.03)
        .workplane(offset=0.065)  # Moves to X=0.08
        .center(0, 0.04)          # Moves local Y (world Z) up by 0.04
        .rect(0.025, 0.025)
        .loft()
    )
    
    # Handlebar clamp at the end of the extension: X=0.08, Z=0.06
    bar_clamp_cq = cq.Workplane("XZ", origin=(0.08, 0, 0.06)).circle(0.018).extrude(0.02, both=True)
    bar_clamp_cq = bar_clamp_cq.faces("<Y").workplane().circle(0.0127).cutThruAll()
    
    stem_cq = stem_clamp_cq.union(stem_ext_cq).union(bar_clamp_cq)
    stem.visual(mesh_from_cadquery(stem_cq, "stem_geom"))
    
    # Articulation: steerer_to_stem (steering yaw, Z axis)
    model.articulation(
        "steerer_to_stem",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0, 0, 0.20)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5)
    )

    # -------------------------------------------------------------------------
    # HANDLEBAR
    # -------------------------------------------------------------------------
    handlebar = model.part("handlebar")
    
    # Swept-back handlebar path
    path = cq.Workplane("XY").spline([
        (-0.15, -0.25),
        (-0.05, -0.15),
        (0, -0.05),
        (0, 0.05),
        (-0.05, 0.15),
        (-0.15, 0.25)
    ])
    profile = cq.Workplane("XZ").circle(0.011)
    bar_cq = profile.sweep(path, isFrenet=True)
    
    # Center clamp area (thicker, radius 12.7mm)
    center_clamp_cq = cq.Workplane("XZ").circle(0.0127).extrude(0.02, both=True)
    bar_cq = bar_cq.union(center_clamp_cq)
    
    handlebar.visual(mesh_from_cadquery(bar_cq, "handlebar_geom"))
    
    # Articulation: stem_to_handlebar (pitch, Y axis)
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.08, 0, 0.06)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-0.5, upper=0.5)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    
    # Intentional embeddings for clamps
    ctx.allow_overlap(stem, fork, reason="Stem clamps tightly around steerer tube.")
    ctx.allow_overlap(handlebar, stem, reason="Handlebar is clamped tightly inside stem.")
    
    # Check that stem is within fork footprint radially
    ctx.expect_within(stem, fork, axes="xy", margin=0.10, name="stem stays over steerer footprint")
    
    # Check that stem revolves around Z
    steerer_joint = object_model.get_articulation("steerer_to_stem")
    rest_stem = ctx.part_world_position(stem)
    with ctx.pose({steerer_joint: 1.0}):
        turned_stem = ctx.part_world_position(stem)
        ctx.check("stem_origin_stable", rest_stem == turned_stem)
    
    # Check that handlebar revolves around Y
    bar_joint = object_model.get_articulation("stem_to_handlebar")
    rest_bar = ctx.part_world_position(handlebar)
    with ctx.pose({bar_joint: 0.5}):
        pitched_bar = ctx.part_world_position(handlebar)
        ctx.check("bar_origin_stable", rest_bar == pitched_bar)
    
    return ctx.report()


object_model = build_object_model()