from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    base = model.part("base")
    
    # Roof mount flange
    base.visual(
        Box((0.40, 0.20, 0.01)),
        origin=Origin((0.0, 0.0, -0.005)),
        name="flange",
    )
    
    # Left wall
    base.visual(
        Box((0.02, 0.10, 0.30)),
        origin=Origin((-0.14, 0.0, 0.15)),
        name="left_wall",
    )
    
    # Right wall
    base.visual(
        Box((0.02, 0.10, 0.30)),
        origin=Origin((0.14, 0.0, 0.15)),
        name="right_wall",
    )
    
    # Back wall
    base.visual(
        Box((0.26, 0.02, 0.30)),
        origin=Origin((0.0, -0.04, 0.15)),
        name="back_wall",
    )
    
    # Top wall
    base.visual(
        Box((0.26, 0.08, 0.02)),
        origin=Origin((0.0, 0.01, 0.29)),
        name="top_wall",
    )
    
    # Bottom lip (front frame)
    base.visual(
        Box((0.26, 0.02, 0.04)),
        origin=Origin((0.0, 0.04, 0.02)),
        name="bottom_lip",
    )

    flap = model.part("flap")
    
    # The flap covers the front opening and rests flush against the front face (Y=0.05)
    flap.visual(
        Box((0.30, 0.02, 0.28)),
        origin=Origin((0.0, 0.01, -0.14)),
        name="panel",
    )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=flap,
        origin=Origin((0.0, 0.05, 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.5),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("flap_hinge")

    # At rest, the flap should be flush against the base housing
    ctx.expect_contact(flap, base, contact_tol=1e-4, name="flap rests against housing")
    
    # Flap should overlap the opening to provide a weather seal
    # Opening is bounded by left/right walls and top/bottom lips.
    ctx.expect_overlap(flap, base, axes="xz", min_overlap=0.01, name="flap covers opening in XZ projection")

    # When opened, the flap swings outward (+Y) and upward (+Z)
    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({hinge: 1.5}):
        flap_aabb = ctx.part_world_aabb(flap)
        if rest_aabb and flap_aabb:
            # Flap bottom edge (min Z) should move up, and outer edge (max Y) should move out
            ctx.check(
                "flap swings outward and upward",
                flap_aabb[1][1] > rest_aabb[1][1] + 0.1 and flap_aabb[0][2] > rest_aabb[0][2] + 0.1,
                details=f"rest_aabb={rest_aabb}, open_aabb={flap_aabb}"
            )
        
    return ctx.report()

object_model = build_object_model()
