import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer")

    mat_red = Material(name="red", rgba=(0.8, 0.15, 0.15, 1.0))
    mat_metal = Material(name="metal", rgba=(0.85, 0.85, 0.9, 1.0))
    mat_dark = Material(name="dark", rgba=(0.2, 0.2, 0.2, 1.0))
    mat_whisk = Material(name="whisk_metal", rgba=(0.75, 0.75, 0.8, 1.0))

    # BASE
    base = model.part("base")
    
    # Lower base
    lower_base_cq = cq.Workplane("XY").box(0.24, 0.36, 0.05, centered=(True, True, False)).edges("|Z").fillet(0.04).edges(">Z").fillet(0.01)
    base.visual(
        mesh_from_cadquery(lower_base_cq, "base_lower"),
        name="base_lower_vis",
        material=mat_red
    )
    
    # Column
    col_cq = cq.Workplane("XY").box(0.14, 0.16, 0.28, centered=(True, True, False)).edges("|Z").fillet(0.04).edges(">Z").fillet(0.01)
    base.visual(
        mesh_from_cadquery(col_cq, "base_column"),
        origin=Origin(xyz=(0, -0.1, 0)),
        name="base_column_vis",
        material=mat_red
    )

    # BOWL CARRIAGE
    bowl_carriage = model.part("bowl_carriage")
    
    bracket_cq = cq.Workplane("XY").box(0.16, 0.16, 0.02, centered=(True, True, False)).edges("|Z").fillet(0.02)
    bowl_carriage.visual(
        mesh_from_cadquery(bracket_cq, "carriage_bracket"),
        origin=Origin(xyz=(0, 0.08, 0)),
        name="bracket_vis",
        material=mat_dark
    )
    
    bowl_cq = cq.Workplane("XY").circle(0.11).extrude(0.14).faces(">Z").shell(-0.005)
    bowl_carriage.visual(
        mesh_from_cadquery(bowl_cq, "mixing_bowl"),
        origin=Origin(xyz=(0, 0.12, 0.02)),
        name="bowl_vis",
        material=mat_metal
    )
    
    model.articulation(
        "bowl_carriage_joint",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0, -0.02, 0.05)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=0.04)
    )

    # TILT HEAD
    head = model.part("head")
    
    head_cq = cq.Workplane("XY").box(0.14, 0.34, 0.12, centered=(True, True, False)).edges("|Z").fillet(0.04).edges(">Z").fillet(0.04).edges("<Z").fillet(0.01)
    head.visual(
        mesh_from_cadquery(head_cq, "tilt_head"),
        origin=Origin(xyz=(0, 0.09, 0)),
        name="head_vis",
        material=mat_red
    )
    
    model.articulation(
        "head_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0, -0.1, 0.28)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=0.0, upper=0.8)
    )

    # WHISK
    whisk = model.part("whisk")
    
    wp = cq.Workplane("XY")
    shaft = wp.circle(0.005).extrude(-0.15)
    wire1 = wp.workplane(offset=-0.08).box(0.08, 0.004, 0.12, centered=(True, True, True))
    wire2 = wp.workplane(offset=-0.08).box(0.004, 0.08, 0.12, centered=(True, True, True))
    whisk_cq = shaft.union(wire1).union(wire2)
    
    whisk.visual(
        mesh_from_cadquery(whisk_cq, "whisk_tool"),
        name="whisk_vis",
        material=mat_whisk
    )
    
    model.articulation(
        "whisk_spin_joint",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0, 0.2, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0)
    )

    # SPEED LEVER
    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Box((0.02, 0.04, 0.01)),
        origin=Origin(xyz=(0.0, 0.02, 0)),
        name="lever_vis",
        material=mat_dark
    )
    
    model.articulation(
        "speed_lever_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(0.07, -0.1, 0.22)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=-0.5, upper=0.5)
    )

    # LOCK BUTTON
    lock_button = model.part("lock_button")
    lock_button.visual(
        Cylinder(radius=0.012, length=0.02),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, 1.5708, 0)),
        name="button_vis",
        material=mat_dark
    )
    
    model.articulation(
        "lock_button_joint",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_button,
        origin=Origin(xyz=(-0.08, -0.1, 0.15)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=0.0, upper=0.01)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap(
        "bowl_carriage", "whisk",
        elem_a="bowl_vis", elem_b="whisk_vis",
        reason="The whisk operates inside the mixing bowl."
    )
    
    ctx.allow_overlap(
        "head", "base",
        elem_a="head_vis", elem_b="base_column_vis",
        reason="The tilt head sits flush on the base column at the hinge."
    )
    
    ctx.allow_overlap(
        "bowl_carriage", "base",
        elem_a="bracket_vis", elem_b="base_column_vis",
        reason="The bowl carriage wraps or touches the base column to slide."
    )

    ctx.allow_overlap(
        "speed_lever", "base",
        reason="The lever pivot is embedded in the base."
    )
    
    ctx.allow_overlap(
        "lock_button", "base",
        reason="The lock button translates into the base."
    )

    ctx.expect_within(
        "whisk", "bowl_carriage", axes="xy",
        inner_elem="whisk_vis", outer_elem="bowl_vis",
        name="Whisk operates within the mixing bowl footprint"
    )

    with ctx.pose(bowl_carriage_joint=0.0):
        ctx.expect_overlap(
            "bowl_carriage", "whisk", axes="z",
            elem_a="bowl_vis", elem_b="whisk_vis",
            min_overlap=0.05,
            name="Whisk is inside the bowl when carriage is lowered"
        )
    
    with ctx.pose(head_tilt_joint=0.8):
        ctx.expect_gap(
            "whisk", "bowl_carriage", axis="z",
            positive_elem="whisk_vis", negative_elem="bowl_vis",
            min_gap=0.05,
            name="Whisk clears the bowl when head is tilted up"
        )

    with ctx.pose(bowl_carriage_joint=0.04):
        ctx.expect_overlap(
            "bowl_carriage", "whisk", axes="z",
            elem_a="bowl_vis", elem_b="whisk_vis",
            min_overlap=0.1,
            name="Whisk is deep inside the bowl when carriage is raised"
        )

    return ctx.report()

object_model = build_object_model()