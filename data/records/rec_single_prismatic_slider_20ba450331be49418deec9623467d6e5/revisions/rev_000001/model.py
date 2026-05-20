from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
)


# Realistic bench vise dimensions (meters)
BASE_LENGTH = 0.20      # 200 mm
BASE_WIDTH = 0.08       # 80 mm
BASE_HEIGHT = 0.05     # 50 mm
JAW_THICKNESS = 0.05    # 50 mm (moving jaw thickness along slide)
JAW_HEIGHT = 0.05       # 50 mm
MAX_OPENING = 0.10      # 100 mm max jaw opening
GUIDE_WIDTH = 0.03      # 30 mm (guide width)
GUIDE_HEIGHT = 0.02     # 20 mm (guide height)
GUIDE_LENGTH = 0.18     # 180 mm (guide length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_vise_jaw_slide")
    
    # Fixed base (root part) - merge all base visuals into one CadQuery shape
    base = model.part("base")
    
    # Combine base elements using CadQuery
    import cadquery as cq
    base_shape = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
    )
    # Add guide tongue
    guide = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .box(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT, centered=(True, True, False))
    )
    base_shape = base_shape.union(guide)
    # Add fixed jaw
    fixed_jaw = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .center(-BASE_LENGTH/2 + 0.01, 0)
        .box(0.02, BASE_WIDTH, JAW_HEIGHT, centered=(False, True, False))
    )
    base_shape = base_shape.union(fixed_jaw)
    # Add screw shaft
    screw = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT/2)
        .center(0, -BASE_WIDTH/2 + 0.01)
        .cylinder(GUIDE_LENGTH, 0.008)
        .rotate((0,0,0), (0,1,0), 90)
    )
    base_shape = base_shape.union(screw)
    
    # Export as single visual
    from sdk import mesh_from_cadquery
    base_visual = mesh_from_cadquery(base_shape, "base_shell")
    base.visual(
        base_visual,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=Material(name="cast_iron", color=(0.4, 0.4, 0.4)),
        name="base_shell"
    )
    
    # Moving jaw (child part)
    moving_jaw = model.part("moving_jaw")
    # Main jaw body - origin at its center (will be set by joint)
    jaw_body = Box((JAW_THICKNESS, BASE_WIDTH, JAW_HEIGHT))
    moving_jaw.visual(
        jaw_body,
        material=Material(name="machined_steel", color=(0.6, 0.6, 0.6)),
        name="jaw_body"
    )
    
    # Grippy jaw pad (thin box on jaw face)
    jaw_pad = Box((0.002, BASE_WIDTH - 0.01, JAW_HEIGHT))
    moving_jaw.visual(
        jaw_pad,
        origin=Origin(xyz=(JAW_THICKNESS/2 + 0.001, 0.0, 0.0)),  # Front face of jaw
        material=Material(name="grippy_rubber", color=(0.1, 0.1, 0.1)),
        name="jaw_pad"
    )
    
    # Prismatic joint for jaw slide
    # Joint origin at closed position: jaw center so jaw face touches fixed jaw
    # Fixed jaw X max = (-BASE_LENGTH/2 + 0.01) + 0.01 = -0.08
    # Jaw X min = joint_origin_x - JAW_THICKNESS/2 = -0.08 (touching)
    joint_origin_x = -0.08 + JAW_THICKNESS/2  # -0.08 + 0.025 = -0.055
    joint_origin_z = BASE_HEIGHT + GUIDE_HEIGHT + JAW_HEIGHT/2  # 0.05 + 0.02 + 0.025 = 0.095
    model.articulation(
        "base_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=base,
        child=moving_jaw,
        origin=Origin(xyz=(joint_origin_x, 0.0, joint_origin_z)),
        axis=(1.0, 0.0, 0.0),  # Slide along X axis
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.1,
            lower=0.0,
            upper=MAX_OPENING,
        ),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jaw = object_model.get_part("moving_jaw")
    joint = object_model.get_articulation("base_to_moving_jaw")
    
    # Test 1: Moving jaw stays within base footprint in XY
    ctx.expect_within(
        jaw, base,
        axes="xy",
        margin=0.005,
        name="moving jaw stays within base footprint"
    )
    
    # Test 2: At closed position, jaw should be near fixed jaw
    with ctx.pose({joint: 0.0}):
        jaw_pos = ctx.part_world_position(jaw)
        base_pos = ctx.part_world_position(base)
        ctx.check(
            "jaw at closed position",
            jaw_pos is not None and jaw_pos[0] < base_pos[0] + 0.05,
            details=f"jaw_x={jaw_pos[0] if jaw_pos else None}"
        )
    
    # Test 3: At max opening, jaw should have moved along +X
    with ctx.pose({joint: MAX_OPENING}):
        jaw_pos = ctx.part_world_position(jaw)
        ctx.check(
            "max opening reaches target",
            jaw_pos is not None and jaw_pos[0] > -BASE_LENGTH/4,
            details=f"jaw_x={jaw_pos[0] if jaw_pos else None}"
        )
    
    # Test 4: Allow intentional overlap between jaw and guide (sliding fit)
    ctx.allow_overlap(
        "base", "moving_jaw",
        reason="Jaw slides on guide, intentional seated overlap",
        elem_a="base_shell",
        elem_b="jaw_body"
    )
    # Prove the overlap is valid with contact check at rest
    with ctx.pose({joint: 0.0}):
        ctx.expect_contact(
            base, jaw,
            contact_tol=0.01,
            elem_a="base_shell",
            elem_b="jaw_body",
            name="jaw seated on guide at rest"
        )
    
    # Test 5: Jaw pad exists as separate visual
    jaw_pad = jaw.get_visual("jaw_pad")
    ctx.check(
        "jaw pad exists as separate visual",
        jaw_pad is not None,
        details="Jaw pad visual missing"
    )
    
    return ctx.report()


object_model = build_object_model()
