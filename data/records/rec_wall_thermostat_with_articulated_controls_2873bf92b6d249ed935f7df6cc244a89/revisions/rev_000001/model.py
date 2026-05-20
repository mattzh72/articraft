import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_thermostat")

    base = model.part("base")
    base.visual(
        Box((0.10, 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="body_shell",
        color=(0.9, 0.9, 0.9, 1.0)
    )
    
    base.visual(
        Box((0.06, 0.04, 0.002)),
        origin=Origin(xyz=(0.0, 0.02, 0.02)),
        name="screen",
        color=(0.1, 0.1, 0.1, 1.0)
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.012, length=0.01),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="dial_knob",
        color=(0.8, 0.8, 0.8, 1.0)
    )
    dial.visual(
        Cylinder(radius=0.005, length=0.005),
        origin=Origin(xyz=(-0.0025, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="dial_shaft",
        color=(0.5, 0.5, 0.5, 1.0)
    )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.05, 0.0, 0.01)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0)
    )

    button_xs = [-0.03, -0.01, 0.01, 0.03]
    for i, x in enumerate(button_xs):
        btn = model.part(f"button_{i}")
        btn.visual(
            Box((0.012, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            name=f"button_{i}_cap",
            color=(0.7, 0.7, 0.7, 1.0)
        )
        
        btn.visual(
            Box((0.006, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            name=f"button_{i}_shaft",
            color=(0.5, 0.5, 0.5, 1.0)
        )

        model.articulation(
            f"button_{i}_press",
            ArticulationType.PRISMATIC,
            parent=base,
            child=btn,
            origin=Origin(xyz=(x, -0.02, 0.02)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=0.003)
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    dial = object_model.get_part("dial")
    
    ctx.allow_overlap(
        base, dial,
        elem_a="body_shell",
        elem_b="dial_shaft",
        reason="Dial shaft is housed inside the base."
    )
    ctx.expect_contact(dial, base, elem_a="dial_knob", elem_b="body_shell", name="dial_rests_on_base")

    for i in range(4):
        btn = object_model.get_part(f"button_{i}")
        ctx.allow_overlap(
            base, btn,
            elem_a="body_shell",
            elem_b=f"button_{i}_shaft",
            reason="Button shaft is housed inside the base."
        )
        ctx.expect_contact(btn, base, elem_a=f"button_{i}_cap", elem_b="body_shell", name=f"button_{i}_rests_on_base")

    return ctx.report()

object_model = build_object_model()
