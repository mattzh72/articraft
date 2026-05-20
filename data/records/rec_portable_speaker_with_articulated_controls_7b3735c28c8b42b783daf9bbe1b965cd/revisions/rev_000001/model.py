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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pa_speaker")
    
    enclosure = model.part("enclosure")
    
    width = 0.25
    depth = 0.25
    height = 0.40
    
    # Enclosure body (centered at 0,0,0)
    body_cq = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(0.02)
        # Recess for grille
        .faces("<Y")
        .workplane()
        .center(0, -0.05)
        .rect(0.21, 0.26)
        .cutBlind(-0.01)
        # Recess for control strip
        .center(0, 0.18)
        .rect(0.21, 0.08)
        .cutBlind(-0.01)
        # Mount ears on the back
        .faces(">Y")
        .workplane()
        .center(0, -0.15)
        .rect(0.12, 0.04)
        .extrude(0.02)
        .faces(">Y")
        .workplane()
        .rect(0.08, 0.04)
        .cutBlind(-0.02)
    )
    
    # Handle on top
    handle_cq = (
        cq.Workplane("XY", origin=(0, 0, height/2))
        .rect(0.15, 0.04)
        .extrude(0.04)
        .faces("<Y")
        .workplane()
        .rect(0.11, 0.02)
        .cutThruAll()
    )
    
    enclosure_mesh = body_cq.union(handle_cq)
    enclosure.visual(
        mesh_from_cadquery(enclosure_mesh, "enclosure_mesh"),
        origin=Origin(xyz=(0, 0, height/2)),
        name="enclosure_shell"
    )

    grille = model.part("grille")
    grille.visual(
        Box((0.208, 0.012, 0.258)),
        origin=Origin(xyz=(0, -0.119, 0.15)), 
        name="grille_panel"
    )
    model.articulation(
        "enclosure_to_grille",
        ArticulationType.FIXED,
        parent=enclosure,
        child=grille,
        origin=Origin()
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((0.208, 0.012, 0.078)),
        origin=Origin(xyz=(0, -0.119, 0.33)),
        name="control_panel"
    )
    model.articulation(
        "enclosure_to_control_strip",
        ArticulationType.FIXED,
        parent=enclosure,
        child=control_strip,
        origin=Origin()
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.02, length=0.015),
        origin=Origin(xyz=(0, -0.015/2, 0), rpy=(1.5708, 0, 0)),
        name="knob_body"
    )
    knob.visual(
        Cylinder(radius=0.005, length=0.01),
        origin=Origin(xyz=(0, 0.005, 0), rpy=(1.5708, 0, 0)),
        name="knob_stem"
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=enclosure,
        child=knob,
        origin=Origin(xyz=(-0.05, -0.124, 0.33)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0)
    )

    for i in range(3):
        btn = model.part(f"button_{i}")
        btn.visual(
            Box((0.015, 0.01, 0.015)),
            origin=Origin(xyz=(0, -0.005, 0)),
            name=f"button_{i}_body"
        )
        model.articulation(
            f"button_{i}_push",
            ArticulationType.PRISMATIC,
            parent=enclosure,
            child=btn,
            origin=Origin(xyz=(0.02 + i*0.03, -0.120, 0.33)),
            axis=(0, 1, 0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=0.005)
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Box((0.076, 0.015, 0.15)),
        origin=Origin(xyz=(0, 0.015/2, 0.15/2)), 
        name="kickstand_body"
    )
    kickstand.visual(
        Cylinder(radius=0.005, length=0.124),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, 1.5708, 0)),
        name="hinge_pin"
    )
    model.articulation(
        "kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=kickstand,
        origin=Origin(xyz=(0, 0.135, 0.05)),
        axis=(-1, 0, 0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=1.0)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    enclosure = object_model.get_part("enclosure")
    grille = object_model.get_part("grille")
    control_strip = object_model.get_part("control_strip")
    knob = object_model.get_part("knob")
    kickstand = object_model.get_part("kickstand")
    
    ctx.allow_overlap(enclosure, grille, reason="Grille is seated in the front recess")
    ctx.allow_overlap(enclosure, control_strip, reason="Control strip is seated in the front recess")
    
    ctx.allow_overlap(enclosure, knob, reason="Knob stem embeds into the enclosure")
    ctx.allow_overlap(control_strip, knob, reason="Knob stem passes through the control strip")
    
    for i in range(3):
        btn = object_model.get_part(f"button_{i}")
        ctx.allow_overlap(enclosure, btn, reason="Button embeds into the enclosure")
        ctx.allow_overlap(control_strip, btn, reason="Button passes through the control strip")
        
        btn_joint = object_model.get_articulation(f"button_{i}_push")
        with ctx.pose({btn_joint: 0.005}):
            ctx.expect_within(btn, control_strip, axes="xz", name=f"button_{i} stays within control strip bounds")
            
    ctx.allow_overlap(enclosure, kickstand, reason="Kickstand hinge pin embeds into the clevis ears")
    ctx.allow_isolated_part(kickstand, reason="Kickstand is connected via hinge pin which is fully embedded in the clevis ears")
    
    kickstand_joint = object_model.get_articulation("kickstand_hinge")
    with ctx.pose({kickstand_joint: 1.0}):
        aabb = ctx.part_world_aabb(kickstand)
        ctx.check("kickstand swings outward", aabb is not None and aabb[1][1] > 0.20)

    return ctx.report()


object_model = build_object_model()
