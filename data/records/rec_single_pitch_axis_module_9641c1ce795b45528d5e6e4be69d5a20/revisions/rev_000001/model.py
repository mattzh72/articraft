from __future__ import annotations

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
    ClevisBracketGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_actuator_module")

    dark_metal = Material("dark_metal", rgba=(0.2, 0.2, 0.2, 1.0))
    steel = Material("steel", rgba=(0.8, 0.8, 0.8, 1.0))
    aluminum = Material("aluminum", rgba=(0.6, 0.6, 0.6, 1.0))

    base = model.part("base")
    clevis = ClevisBracketGeometry(
        (0.12, 0.08, 0.10),
        gap_width=0.06,
        bore_diameter=0.015,
        bore_center_z=0.07,
        base_thickness=0.015,
        center=False,
    )
    base.visual(
        mesh_from_geometry(clevis, "support_cheeks"),
        material=dark_metal,
        name="support_cheeks",
    )

    payload = model.part("payload_bracket")
    
    # Horizontal hinge shaft
    payload.visual(
        Cylinder(radius=0.0075, length=0.12),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, 1.570796, 0)),
        name="shaft",
        material=steel,
    )
    
    # Rectangular bracket body extending upwards
    payload.visual(
        Box((0.058, 0.02, 0.06)),
        origin=Origin(xyz=(0, 0, 0.03)),
        name="bracket_body",
        material=aluminum,
    )
    
    # Payload mounting plate
    payload.visual(
        Box((0.08, 0.08, 0.01)),
        origin=Origin(xyz=(0, 0, 0.065)),
        name="payload_plate",
        material=aluminum,
    )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=payload,
        origin=Origin(xyz=(0, 0, 0.07)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    payload = object_model.get_part("payload_bracket")
    
    ctx.allow_overlap(
        payload, base, 
        elem_a="shaft", elem_b="support_cheeks", 
        reason="Shaft is captured in the support cheeks bore"
    )
    
    # Verify the payload bracket fits inside the cheeks
    ctx.expect_within(
        payload, base,
        axes="x",
        inner_elem="bracket_body",
        outer_elem="support_cheeks",
        margin=0.002,
        name="bracket body fits within the cheeks"
    )
    
    # Verify the shaft overlaps the cheeks (it extends through them)
    ctx.expect_overlap(
        payload, base,
        axes="x",
        elem_a="shaft",
        elem_b="support_cheeks",
        min_overlap=0.02,
        name="shaft extends into the cheeks"
    )

    return ctx.report()


object_model = build_object_model()
