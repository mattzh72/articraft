import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Box,
    Cylinder,
    Material,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="friction_hinge_display")

    # --------------------------
    # Root part: Base plate
    # --------------------------
    base_plate = model.part("base_plate")

    # Base plate: thick plate
    bp_size = (0.10, 0.05, 0.01)
    base_plate.visual(
        Box(bp_size),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="base_plate_shell",
        material=Material("dark_gray_metal", color=(0.2, 0.2, 0.2)),
    )

    # Pivot barrel: large cylinder along Y axis (3cm diameter)
    barrel_radius = 0.015
    barrel_length = 0.08
    base_plate.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.02), rpy=(math.pi/2, 0.0, 0.0)),
        name="pivot_barrel",
        material=Material("silver_metal", color=(0.8, 0.8, 0.8)),
    )

    # Tension knob: knurled cylinder using CadQuery
    knob_radius = 0.0125
    knob_length = 0.01
    # Create cylinder along Z, add grooves for knurling, then rotate to Y
    knob_wp = cq.Workplane("XY").cylinder(knob_length, knob_radius)
    # Add circumferential grooves
    num_grooves = 8
    groove_depth = 0.001
    for i in range(num_grooves):
        z_pos = -knob_length/2 + (i + 1) * knob_length / (num_grooves + 1)
        cutter = cq.Workplane("XY").workplane(offset=z_pos - 0.0003).cylinder(0.0006, knob_radius)
        inner = cq.Workplane("XY").workplane(offset=z_pos - 0.0003).cylinder(0.0006, knob_radius - groove_depth)
        groove = cq.Workplane("XY").add(cutter).cut(inner)
        knob_wp = knob_wp.cut(groove)
    # Rotate to align with Y axis
    knob_solid = knob_wp.rotate((0,0,0), (1,0,0), 90).val()
    base_plate.visual(
        mesh_from_cadquery(knob_solid, "tension_knob"),
        origin=Origin(xyz=(0.0, 0.045, 0.02)),
        name="tension_knob",
        material=Material("black_plastic", color=(0.1, 0.1, 0.1)),
    )

    # --------------------------
    # Child part: Movable plate with pivot hole
    # --------------------------
    movable_plate = model.part("movable_plate")

    # Create plate with hole using CadQuery
    mp_length, mp_width, mp_thickness = 0.10, 0.05, 0.01
    plate_wp = cq.Workplane("XY").box(mp_length, mp_width, mp_thickness)
    # Cut hole for barrel (cylinder along Y)
    hole = cq.Workplane("XY").cylinder(mp_width + 0.02, barrel_radius)
    hole_rotated = hole.rotate((0,0,0), (1,0,0), 90)  # Rotate to Y axis
    plate_with_hole = plate_wp.cut(hole_rotated).val()
    movable_plate.visual(
        mesh_from_cadquery(plate_with_hole, "movable_plate_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="movable_plate_shell",
        material=Material("light_gray_metal", color=(0.5, 0.5, 0.5)),
    )

    # Side washers: thin disks on both sides
    washer_thickness = 0.001
    washer_radius = 0.02
    movable_plate.visual(
        Cylinder(radius=washer_radius, length=washer_thickness),
        origin=Origin(xyz=(0.0, -0.0255, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="washer_neg_y",
        material=Material("washer_silver", color=(0.8, 0.8, 0.8)),
    )
    movable_plate.visual(
        Cylinder(radius=washer_radius, length=washer_thickness),
        origin=Origin(xyz=(0.0, 0.0255, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="washer_pos_y",
        material=Material("washer_silver", color=(0.8, 0.8, 0.8)),
    )

    # --------------------------
    # Articulation: Revolute hinge
    # --------------------------
    model.articulation(
        "plate_hinge",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child=movable_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.0,
            lower=0.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    movable = object_model.get_part("movable_plate")
    hinge = object_model.get_articulation("plate_hinge")

    # 1. Main mechanism checks
    ctx.check("hinge axis is Y", hinge.axis == (0.0, 1.0, 0.0))
    ctx.check(
        "hinge has tight motion limits",
        hinge.motion_limits.lower == 0.0 and hinge.motion_limits.upper == 1.0,
    )

    # 2. Support/contact checks with overlap allowance
    ctx.allow_overlap(
        "base_plate", "movable_plate",
        elem_a="pivot_barrel",
        elem_b="movable_plate_shell",
        reason="Pivot barrel passes through movable plate hole for hinge mechanism",
    )
    ctx.expect_contact(
        movable, base,
        elem_a="movable_plate_shell", elem_b="pivot_barrel",
        contact_tol=0.005,
        name="movable plate contacts pivot barrel",
    )

    # 3. Visible detail checks
    ctx.check(
        "tension knob exists",
        base.get_visual("tension_knob") is not None,
    )
    ctx.check(
        "washers exist",
        movable.get_visual("washer_neg_y") is not None and
        movable.get_visual("washer_pos_y") is not None,
    )

    # Color contrast
    base_color = base.get_visual("base_plate_shell").material.rgba[:3]
    movable_color = movable.get_visual("movable_plate_shell").material.rgba[:3]
    ctx.check(
        "color contrast between plates",
        base_color != movable_color,
        details=f"Base: {base_color}, Movable: {movable_color}",
    )

    # 4. Pose validation
    rest_aabb = ctx.part_world_aabb(movable)
    with ctx.pose({hinge: 1.0}):
        posed_aabb = ctx.part_world_aabb(movable)
        ctx.check(
            "movable plate rotates",
            rest_aabb != posed_aabb,
        )

    return ctx.report()


object_model = build_object_model()
