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
    model = ArticulatedObject(name="lab_blender")

    # 1. Housing (Root)
    housing = model.part("housing")
    # Base pad: Y from -0.04 to 0.11, Z from 0.0 to 0.10
    housing.visual(
        Box((0.20, 0.15, 0.10)),
        origin=Origin(xyz=(0.0, 0.035, 0.05)),
        name="base_pad"
    )
    # Tower: Y from -0.16 to -0.04, Z from 0.0 to 0.28
    housing.visual(
        Box((0.20, 0.12, 0.28)),
        origin=Origin(xyz=(0.0, -0.10, 0.14)),
        name="tower"
    )
    # Control panel on the front
    housing.visual(
        Box((0.10, 0.01, 0.04)),
        origin=Origin(xyz=(0.0, 0.115, 0.05)),
        name="control_panel"
    )

    # 2. Cup
    cup = model.part("cup")
    # Hollow cup using CadQuery
    cup_body_cq = (
        cq.Workplane("XY")
        .circle(0.06)
        .extrude(0.18)
        .faces(">Z")
        .shell(-0.005)
    )
    cup.visual(
        mesh_from_cadquery(cup_body_cq, "cup_body"),
        origin=Origin(xyz=(0.0, -0.06, 0.0)),
        name="cup_shell"
    )

    # Cup tilts forward on the front edge of the base pad
    # Base pad front is Y=0.11, Z=0.10
    model.articulation(
        "cup_tilt",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cup,
        origin=Origin(xyz=(0.0, 0.11, 0.10)),
        axis=(-1.0, 0.0, 0.0), # Positive q tilts top forward
        motion_limits=MotionLimits(lower=0.0, upper=1.0)
    )

    # 3. Top Clamp
    top_clamp = model.part("top_clamp")
    # Clamp body: covers the cup. Local center (0.0, 0.085, 0.01) -> World (0.0, 0.045, 0.29)
    top_clamp.visual(
        Box((0.16, 0.17, 0.02)),
        origin=Origin(xyz=(0.0, 0.085, 0.01)),
        name="clamp_body"
    )
    # Clamp lip: hangs down in front of the cup. Local center (0.0, 0.16, 0.0) -> World (0.0, 0.12, 0.28)
    top_clamp.visual(
        Box((0.16, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.16, 0.0)),
        name="clamp_lip"
    )

    # Top clamp hinges at the top front edge of the tower
    # Tower top front is Y=-0.04, Z=0.28
    model.articulation(
        "clamp_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=top_clamp,
        origin=Origin(xyz=(0.0, -0.04, 0.28)),
        axis=(1.0, 0.0, 0.0), # Positive q rotates upward
        motion_limits=MotionLimits(lower=0.0, upper=1.5)
    )

    # 4. Blade
    blade = model.part("blade")
    # Blade local frame is at world (0.0, 0.05, 0.105)
    blade.visual(
        Cylinder(radius=0.005, height=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="blade_shaft"
    )
    blade.visual(
        Box((0.08, 0.01, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="blade_x"
    )
    blade.visual(
        Box((0.01, 0.08, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="blade_y"
    )

    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=cup,
        child=blade,
        origin=Origin(xyz=(0.0, -0.06, 0.005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    housing = object_model.get_part("housing")
    cup = object_model.get_part("cup")
    top_clamp = object_model.get_part("top_clamp")
    blade = object_model.get_part("blade")

    ctx.allow_overlap(blade, cup, reason="blade shaft is seated in cup bottom")

    # In closed pose, clamp rests on cup and housing
    ctx.expect_contact(top_clamp, cup, elem_a="clamp_body", elem_b="cup_shell", name="clamp rests on cup")
    ctx.expect_contact(cup, housing, elem_a="cup_shell", elem_b="base_pad", name="cup rests on housing")
    
    # Blade is inside cup
    ctx.expect_within(blade, cup, axes="xy", name="blade within cup xy")
    ctx.expect_within(blade, cup, axes="z", name="blade within cup z")

    # Test open clamp
    clamp_hinge = object_model.get_articulation("clamp_hinge")
    cup_tilt = object_model.get_articulation("cup_tilt")
    
    with ctx.pose({clamp_hinge: 1.5}):
        ctx.expect_gap(top_clamp, cup, axis="z", min_gap=0.0, name="clamp clears cup when open")
        
        # Test cup tilt while clamp is open
        with ctx.pose({cup_tilt: 1.0}):
            ctx.expect_gap(cup, housing, negative_elem="base_pad", axis="z", min_gap=0.0, name="tilted cup clears base pad")
            # The cup should have moved forward
            cup_pos = ctx.part_world_position(cup)
            ctx.check("cup tilted forward", cup_pos is not None and cup_pos[1] > 0.05)

    return ctx.report()

object_model = build_object_model()
