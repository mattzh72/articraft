import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
from sdk import (
    KnobGeometry,
    KnobSkirt,
    KnobGrip,
    KnobIndicator,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject("gas_stove_top")

    # Materials
    base_mat = Material("stainless_steel", rgba=(0.7, 0.7, 0.7, 1.0))
    burner_mat = Material("cast_iron", rgba=(0.1, 0.1, 0.1, 1.0))
    knob_mat = Material("knob_plastic", rgba=(0.15, 0.15, 0.15, 1.0))

    # Base
    base = model.part("base")

    # Base shape: 0.76w x 0.52d x 0.10h
    # Translated so Z goes from 0 to 0.10.
    # Top face has a 0.02m deep inset for the burner field.
    base_shape = (
        cq.Workplane("XY")
        .box(0.76, 0.52, 0.10)
        .translate((0, 0, 0.05))
        .edges("|Z")
        .fillet(0.01)
        .faces(">Z")
        .workplane()
        .rect(0.72, 0.48)
        .cutBlind(-0.02)
    )

    base.visual(
        mesh_from_cadquery(base_shape, "base_shell"),
        origin=Origin(),
        material=base_mat,
        name="base_visual",
    )

    # Burners
    # The inset floor is at Z = 0.08.
    # We embed them slightly to ensure robust contact and avoid exact coplanar ambiguity.
    burner_radius = 0.05
    burner_length = 0.015
    burner_z_base = 0.0795  # 0.5 mm embed

    burner_positions = [
        (-0.22, -0.12),  # Front Left
        (0.22, -0.12),   # Front Right
        (-0.22, 0.14),   # Rear Left
        (0.22, 0.14),    # Rear Right
    ]

    for i, (bx, by) in enumerate(burner_positions):
        b_name = f"burner_{i}"
        burner = model.part(b_name)
        
        # Burner visual
        burner.visual(
            Cylinder(radius=burner_radius, length=burner_length),
            origin=Origin(xyz=(0.0, 0.0, burner_length / 2)),
            material=burner_mat,
            name=f"{b_name}_visual",
        )
        
        model.articulation(
            f"base_to_{b_name}",
            ArticulationType.FIXED,
            parent=base,
            child=burner,
            origin=Origin(xyz=(bx, by, burner_z_base)),
        )

    # Knobs
    # 5 knobs on the front face (Y = -0.26).
    # We embed them slightly by 0.5 mm to ensure robust seating.
    knob_geom = KnobGeometry(
        diameter=0.04,
        height=0.02,
        body_style="skirted",
        top_diameter=0.03,
        skirt=KnobSkirt(diameter=0.045, height=0.005, flare=0.05),
        grip=KnobGrip(style="fluted", count=15, depth=0.001),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.001),
        center=False,  # Base at Z=0, extends to Z=0.02
    )
    knob_mesh = mesh_from_geometry(knob_geom, "knob_mesh")

    knob_x_positions = [-0.20, -0.10, 0.0, 0.10, 0.20]
    
    for i, kx in enumerate(knob_x_positions):
        k_name = f"knob_{i}"
        knob = model.part(k_name)
        
        knob.visual(
            knob_mesh,
            origin=Origin(),
            material=knob_mat,
            name=f"{k_name}_visual",
        )
        
        # Orient articulation so local +Z points to parent -Y.
        # Rx(pi/2) maps +Y to +Z, and +Z to -Y.
        model.articulation(
            f"base_to_{k_name}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=knob,
            origin=Origin(
                xyz=(kx, -0.2595, 0.05),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    
    # Check burners
    for i in range(4):
        burner = object_model.get_part(f"burner_{i}")
        
        # Allow the intended embed
        ctx.allow_overlap(
            burner,
            base,
            elem_a=f"burner_{i}_visual",
            elem_b="base_visual",
            reason="Burners are seated in the cooktop inset with a slight embed.",
        )
        
        # Prove they are seated
        ctx.expect_contact(
            burner,
            base,
            elem_a=f"burner_{i}_visual",
            elem_b="base_visual",
            name=f"burner_{i}_seated_on_base",
        )
        
    # Check knobs
    for i in range(5):
        knob = object_model.get_part(f"knob_{i}")
        joint = object_model.get_articulation(f"base_to_knob_{i}")
        
        # Allow the intended embed
        ctx.allow_overlap(
            knob,
            base,
            elem_a=f"knob_{i}_visual",
            elem_b="base_visual",
            reason="Knobs are seated against the front face with a slight embed.",
        )
        
        # Prove they are seated against the front face (-Y)
        # Knob is on the negative side of the front face. Wait!
        # The base is from Y=-0.26 to +0.26. The knob is from Y=-0.28 to -0.26.
        # So knob is on the negative side of base along Y.
        ctx.expect_gap(
            base,
            knob,
            axis="y",
            max_penetration=0.001,
            positive_elem="base_visual",
            negative_elem=f"knob_{i}_visual",
            name=f"knob_{i}_seated_against_base",
        )
        
        # Test rotation
        with ctx.pose({joint: math.pi / 2}):
            ctx.expect_gap(
                base,
                knob,
                axis="y",
                max_penetration=0.001,
                positive_elem="base_visual",
                negative_elem=f"knob_{i}_visual",
                name=f"knob_{i}_seated_against_base_rotated",
            )

    return ctx.report()


object_model = build_object_model()
