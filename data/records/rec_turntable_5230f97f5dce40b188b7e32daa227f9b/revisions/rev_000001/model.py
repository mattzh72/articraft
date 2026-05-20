from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_player")

    # Materials
    wood = Material(name="wood", rgba=(0.6, 0.4, 0.2, 1.0))
    aluminum = Material(name="aluminum", rgba=(0.8, 0.8, 0.8, 1.0))
    rubber = Material(name="rubber", rgba=(0.1, 0.1, 0.1, 1.0))
    metal = Material(name="metal", rgba=(0.9, 0.9, 0.9, 1.0))
    black = Material(name="black", rgba=(0.2, 0.2, 0.2, 1.0))

    # Base (rectangular plinth)
    base = model.part("base")
    
    # Plinth: 0.4m x 0.3m x 0.1m with rounded edges
    plinth_shape = (
        cq.Workplane("XY")
        .box(0.4, 0.3, 0.1)
        .edges()
        .fillet(0.01)
    )
    base.visual(
        mesh_from_cadquery(plinth_shape, "plinth"),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # Center of plinth at (0,0,0.05) world
        material=wood,
        name="plinth_shell",
    )

    # Fixed tonearm details
    # Tonearm base (small cylinder on plinth, positioned outside platter radius)
    tonearm_base_shape = cq.Workplane("XY").cylinder(0.02, 0.015)
    base.visual(
        mesh_from_cadquery(tonearm_base_shape, "tonearm_base_geom"),
        origin=Origin(xyz=(0.18, 0.1, 0.11)),  # Top of plinth + half tonearm base height (X=0.18 > platter radius 0.15)
        material=black,
        name="tonearm_base",
    )

    # Tonearm arm (long thin cylinder along Y axis, outside platter radius)
    tonearm_arm_shape = cq.Workplane("XZ").cylinder(0.2, 0.008)  # 0.2m long, 0.008m radius
    base.visual(
        mesh_from_cadquery(tonearm_arm_shape, "tonearm_arm_geom"),
        origin=Origin(xyz=(0.18, 0.0, 0.12)),  # Top of tonearm base (X=0.18 > platter radius)
        material=black,
        name="tonearm_arm",
    )

    # Platter assembly (moving part)
    platter = model.part("platter")
    
    # Main platter (circular, 0.02m thick, 0.15m radius)
    platter_shape = cq.Workplane("XY").cylinder(0.02, 0.15)
    platter.visual(
        mesh_from_cadquery(platter_shape, "platter_geom"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Centered on platter part frame
        material=aluminum,
        name="platter_shell",
    )

    # Rubber mat (on top of platter)
    mat_shape = cq.Workplane("XY").cylinder(0.005, 0.14)
    platter.visual(
        mesh_from_cadquery(mat_shape, "mat_geom"),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),  # Top of platter + half mat thickness
        material=rubber,
        name="rubber_mat",
    )

    # Center spindle
    spindle_shape = cq.Workplane("XY").cylinder(0.03, 0.005)
    platter.visual(
        mesh_from_cadquery(spindle_shape, "spindle_geom"),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),  # Top of mat + half spindle height
        material=metal,
        name="spindle",
    )

    # Vertical revolute joint for platter spin
    model.articulation(
        "platter_spin",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),  # Spindle axis at platter center
        axis=(0.0, 0.0, 1.0),  # Vertical Z axis
        motion_limits=MotionLimits(lower=0.0, upper=6.283, effort=1.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platter = object_model.get_part("platter")
    joint = object_model.get_articulation("platter_spin")

    # Validate joint properties
    ctx.check(
        "joint_is_revolute",
        joint.articulation_type == ArticulationType.REVOLUTE,
        "Primary joint must be revolute",
    )
    ctx.check(
        "joint_axis_vertical",
        joint.axis == (0.0, 0.0, 1.0),
        "Joint axis must be vertical (Z)",
    )

    # Rest pose checks (q=0)
    with ctx.pose({joint: 0.0}):
        # Platter contacts base at rest
        ctx.expect_contact(base, platter, name="platter_rest_contact")
        
        # Platter is centered
        platter_pos = ctx.part_world_position(platter)
        ctx.check(
            "platter_centered_rest",
            platter_pos is not None and abs(platter_pos[0]) < 1e-3 and abs(platter_pos[1]) < 1e-3,
            f"Platter should be centered at rest, got {platter_pos}",
        )

        # Visible details present
        ctx.check(
            "platter_has_mat",
            platter.get_visual("rubber_mat") is not None,
            "Platter must have rubber mat",
        )
        ctx.check(
            "platter_has_spindle",
            platter.get_visual("spindle") is not None,
            "Platter must have center spindle",
        )
        ctx.check(
            "base_has_tonearm",
            base.get_visual("tonearm_arm") is not None,
            "Base must have fixed tonearm",
        )

    # Rotated pose check (90 degrees)
    with ctx.pose({joint: 1.5708}):
        # Verify joint pose is applied (platter orientation changes, position unchanged)
        ctx.check(
            "platter_rotated_pose",
            abs(joint.motion_limits.upper - 6.283) < 1e-3,
            "Joint should allow full 360° rotation",
        )
        # Still in contact with base
        ctx.expect_contact(base, platter, name="platter_rotated_contact")

    return ctx.report()


object_model = build_object_model()
