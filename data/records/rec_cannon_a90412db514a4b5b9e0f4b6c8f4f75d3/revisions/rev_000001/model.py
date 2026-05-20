from __future__ import annotations

import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WheelGeometry,
    WheelSpokes,
    WheelRim,
    WheelHub,
    WheelBore,
    mesh_from_geometry,
    mesh_from_cadquery,
)

def make_barrel():
    # Build along Z, from -0.5 to 0.7
    barrel = cq.Workplane("XY").workplane(offset=-0.5).circle(0.08).extrude(1.2, taper=1.43)
    cascabel = cq.Workplane("XY").workplane(offset=-0.52).sphere(0.05)
    barrel = barrel.union(cascabel)
    trunnions = cq.Workplane("YZ").workplane(offset=-0.2).center(0, 0).circle(0.041).extrude(0.4)
    barrel = barrel.union(trunnions)
    
    # Cut bore last
    barrel = barrel.faces(">Z").workplane().circle(0.035).cutBlind(-1.1)
    
    barrel = barrel.rotate((0,0,0), (0,0,1), 90).rotate((0,0,0), (0,1,0), 90)
    return barrel

def make_carriage():
    pts = [
        (0.2, -0.05),
        (0.2, 0.35),
        (-0.2, 0.35),
        (-1.6, -0.3),
        (-1.6, -0.48),
        (-0.2, -0.05)
    ]
    # Left cheek
    left_cheek = cq.Workplane("XZ").workplane(offset=0.09).polyline(pts).close().extrude(0.06)
    # Right cheek
    right_cheek = cq.Workplane("XZ").workplane(offset=-0.15).polyline(pts).close().extrude(0.06)
    
    carriage = left_cheek.union(right_cheek)
    
    # Front transom
    front_transom = cq.Workplane("XY").workplane(offset=-0.05).center(0.1, 0).rect(0.1, 0.18).extrude(0.3)
    # Center transom
    center_transom = cq.Workplane("XY").workplane(offset=-0.15).center(-0.8, 0).rect(0.2, 0.18).extrude(0.1)
    # Rear transom
    rear_transom = cq.Workplane("XY").workplane(offset=-0.4).center(-1.5, 0).rect(0.15, 0.18).extrude(0.1)
    
    carriage = carriage.union(front_transom).union(center_transom).union(rear_transom)
    
    # Axletree
    axletree = cq.Workplane("XZ").workplane(offset=-0.35).center(0, 0).circle(0.04).extrude(0.7)
    carriage = carriage.union(axletree)
    
    # Cutout for trunnion
    trunnion_cut = cq.Workplane("XZ").workplane(offset=-0.2).center(-0.1, 0.35).circle(0.04).extrude(0.4)
    carriage = carriage.cut(trunnion_cut)
    
    return carriage

def make_trail_handle():
    handle = cq.Workplane("YZ").workplane(offset=-1.5).center(0, -0.375).circle(0.02).extrude(-0.3)
    return handle

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swedish_regimental_field_gun")

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage(), "carriage_mesh"),
        name="carriage_visual",
        color=(0.5, 0.3, 0.1, 1.0) # Wood color
    )

    trail_handle = model.part("trail_handle")
    trail_handle.visual(
        mesh_from_cadquery(make_trail_handle(), "trail_handle_mesh"),
        name="trail_handle_visual",
        color=(0.4, 0.2, 0.1, 1.0)
    )
    model.articulation(
        "carriage_to_handle",
        ArticulationType.FIXED,
        parent=carriage,
        child=trail_handle,
        origin=Origin()
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(make_barrel(), "barrel_mesh"),
        name="barrel_visual",
        color=(0.2, 0.2, 0.2, 1.0) # Cast iron
    )
    model.articulation(
        "barrel_trunnion",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(-0.1, 0.0, 0.35)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.1, upper=0.5)
    )

    wheel_geom = WheelGeometry(
        radius=0.5,
        width=0.08,
        rim=WheelRim(inner_radius=0.45, flange_height=0.02, flange_thickness=0.01, bead_seat_depth=0.0),
        hub=WheelHub(radius=0.08, width=0.1, cap_style="flat"),
        spokes=WheelSpokes(count=12, thickness=0.04, style="straight"),
        bore=WheelBore(diameter=0.078, style="round")
    )
    
    wheel_left = model.part("wheel_left")
    wheel_left.visual(
        mesh_from_geometry(wheel_geom, "wheel_mesh_left"),
        name="wheel_left_visual",
        color=(0.5, 0.3, 0.1, 1.0)
    )
    model.articulation(
        "wheel_left_axle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wheel_left,
        origin=Origin(xyz=(0.0, 0.25, 0.0), rpy=(0.0, 0.0, math.pi/2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=5.0, lower=-1e4, upper=1e4)
    )

    wheel_right = model.part("wheel_right")
    wheel_right.visual(
        mesh_from_geometry(wheel_geom, "wheel_mesh_right"),
        name="wheel_right_visual",
        color=(0.5, 0.3, 0.1, 1.0)
    )
    model.articulation(
        "wheel_right_axle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wheel_right,
        origin=Origin(xyz=(0.0, -0.25, 0.0), rpy=(0.0, 0.0, -math.pi/2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=5.0, lower=-1e4, upper=1e4)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allowances for captured trunnions and axles
    ctx.allow_overlap(
        "carriage", "barrel",
        reason="Trunnions sit inside the carriage cutouts."
    )
    ctx.allow_overlap(
        "carriage", "wheel_left",
        reason="Axletree sits inside the wheel hub."
    )
    ctx.allow_overlap(
        "carriage", "wheel_right",
        reason="Axletree sits inside the wheel hub."
    )
    ctx.allow_overlap(
        "carriage", "trail_handle",
        reason="Trail handle is rigidly attached to the carriage."
    )

    return ctx.report()

object_model = build_object_model()
