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
    place_on_surface,
    place_on_face
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="propeller_airplane")

    # Fuselage
    fuselage = model.part("fuselage")
    fuselage_cq = (
        cq.Workplane("YZ")
        .workplane(offset=-0.3)
        .circle(0.02)
        .workplane(offset=0.15)
        .circle(0.06)
        .workplane(offset=0.3)
        .circle(0.06)
        .workplane(offset=0.15)
        .circle(0.02)
        .loft()
    )
    fuselage.visual(mesh_from_cadquery(fuselage_cq, "fuselage_mesh"), name="fuselage_body")

    # Main Wings
    fuselage.visual(
        Box((0.15, 0.6, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        name="main_wings"
    )

    # Horizontal Stabilizer (Tail Wings)
    fuselage.visual(
        Box((0.08, 0.25, 0.01)),
        origin=Origin(xyz=(-0.25, 0.0, 0.01)),
        name="horizontal_stabilizer"
    )

    # Vertical Stabilizer
    fuselage.visual(
        Box((0.08, 0.01, 0.1)),
        origin=Origin(xyz=(-0.25, 0.0, 0.065)),
        name="vertical_stabilizer"
    )

    # Display Peg / Landing Gear Base
    display_base = model.part("display_base")
    display_base.visual(
        Box((0.15, 0.15, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.2)),
        name="base_plate"
    )
    display_base.visual(
        Cylinder(radius=0.01, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, -0.115)), # from -0.19 to -0.04
        name="stand_peg"
    )
    # Secondary support feature
    display_base.visual(
        Box((0.02, 0.15, 0.15)),
        origin=Origin(xyz=(-0.085, 0.0, -0.135)),
        name="wall_plate"
    )

    # Attach fuselage to display_base rigidly
    model.articulation(
        "base_to_fuselage",
        ArticulationType.FIXED,
        parent=display_base,
        child=fuselage,
        origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # Propeller
    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.025, length=0.04),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        name="prop_hub"
    )
    propeller.visual(
        Box((0.01, 0.3, 0.02)),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        name="prop_blades"
    )

    # Propeller joint
    model.articulation(
        "fuselage_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.3, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    propeller = object_model.get_part("propeller")
    display_base = object_model.get_part("display_base")
    
    ctx.allow_overlap(
        display_base,
        fuselage,
        elem_a="stand_peg",
        elem_b="fuselage_body",
        reason="The stand peg embeds into the fuselage body to support the plane."
    )
    
    ctx.expect_overlap(fuselage, display_base, axes="xy", name="fuselage over base")
    ctx.expect_gap(propeller, fuselage, axis="x", min_gap=-0.01, max_gap=0.01, name="propeller mounts to nose")

    return ctx.report()


object_model = build_object_model()
