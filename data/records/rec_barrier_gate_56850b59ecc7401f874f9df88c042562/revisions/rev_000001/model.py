from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_road_barrier")

    # --- Root part: Ground base ---
    ground_base = model.part("ground_base")
    ground_base.visual(
        Box((6.0, 0.5, 0.1)),  # 6m long, 0.5m wide, 0.1m thick
        origin=Origin(xyz=(1.5, 0.0, 0.05)),  # Center at (1.5, 0, 0.05) world, top face at Z=0.10
        name="base_slab",
        material=Material(name="concrete_gray", color=(0.5, 0.5, 0.5)),  # Concrete gray
    )

    # --- Fixed hinge post ---
    hinge_post = model.part("hinge_post")
    hinge_post.visual(
        Box((0.1, 0.1, 1.0)),  # 0.1x0.1m cross section, 1m tall
        origin=Origin(xyz=(0.0, 0.0, 0.5)),  # Center at Z=0.5 relative to post, top at Z=1.0
        name="post_body",
        material=Material(name="steel_gray", color=(0.3, 0.3, 0.3)),  # Steel gray
    )
    # Attach to ground base via FIXED articulation
    model.articulation(
        "base_to_hinge_post",
        ArticulationType.FIXED,
        parent=ground_base,
        child=hinge_post,
        origin=Origin(xyz=(0.0, 0.0, 0.1)),  # World position (0, 0, 0.1)
        axis=(0.0, 0.0, 1.0),
    )

    # --- Fixed latch receiver post ---
    latch_receiver = model.part("latch_receiver_post")
    latch_receiver.visual(
        Box((0.1, 0.1, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        name="receiver_post_body",
        material=Material(name="steel_gray", color=(0.3, 0.3, 0.3)),
    )
    # Latch hook on top
    latch_receiver.visual(
        Box((0.15, 0.15, 0.1)),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),  # Top of post + 0.05m
        name="latch_hook",
        material=Material(name="yellow", color=(1.0, 1.0, 0.0)),  # Yellow
    )
    # Attach to ground base via FIXED articulation
    model.articulation(
        "base_to_latch_receiver",
        ArticulationType.FIXED,
        parent=ground_base,
        child=latch_receiver,
        origin=Origin(xyz=(3.0, 0.0, 0.1)),  # World position (3.0, 0, 0.1)
        axis=(0.0, 0.0, 1.0),
    )

    # --- Moving beam assembly ---
    beam_assembly = model.part("beam_assembly")

    # Main wooden beam with chamfered edges
    beam_cq = (
        cq.Workplane("XY")
        .rect(3.0, 0.2)  # 3m long, 0.2m wide
        .extrude(0.1)  # 0.1m thick
        .edges("|Z")  # Chamfer long edges
        .chamfer(0.01)  # 1cm chamfer
    )
    beam_assembly.visual(
        mesh_from_cadquery(beam_cq, "main_beam"),
        origin=Origin(xyz=(1.5, 0.0, 0.0)),  # Center at X=1.5 (spans 0-3m X)
        name="main_beam",
        material=Material(name="wood_brown", color=(0.6, 0.3, 0.1)),  # Wood brown
    )

    # Diagonal brace (part of beam assembly)
    brace_cq = (
        cq.Workplane("XY")
        .rect(0.05, 0.05)  # 5cm x 5cm cross section
        .extrude(0.5)  # 0.5m long
        .edges("|Z")
        .chamfer(0.005)
    )
    beam_assembly.visual(
        mesh_from_cadquery(brace_cq, "diagonal_brace"),
        origin=Origin(xyz=(0.2, 0.0, 0.05), rpy=(0.0, 0.785, 0.0)),  # 45° pitch
        name="diagonal_brace",
        material=Material(name="steel_gray", color=(0.3, 0.3, 0.3)),  # Steel gray
    )

    # Striped paint (alternating yellow/black stripes)
    for i in range(14):  # 14 stripes every 0.2m from 0.2-2.8m X
        x_pos = 0.2 + i * 0.2
        color = (1.0, 1.0, 0.0) if i % 2 == 0 else (0.0, 0.0, 0.0)
        beam_assembly.visual(
            Box((0.05, 0.2, 0.1)),  # Stripe: 5cm long, full width/thickness
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            name=f"stripe_{i}",
            material=Material(name="stripe_yellow" if i % 2 == 0 else "stripe_black", color=color),
        )

    # --- Pitch hinge (revolute around Y axis) ---
    model.articulation(
        "hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_post,
        child=beam_assembly,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),  # Top of hinge post (in post frame)
        axis=(0.0, -1.0, 0.0),  # Y axis (pitch rotation, positive lifts beam)
        motion_limits=MotionLimits(
            lower=0.0,  # Closed (horizontal)
            upper=1.5708,  # ~90° (vertical)
            effort=10.0,
            velocity=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hinge_post = object_model.get_part("hinge_post")
    beam = object_model.get_part("beam_assembly")
    latch = object_model.get_part("latch_receiver_post")
    hinge = object_model.get_articulation("hinge")

    # Allow intentional overlaps with proof checks
    ctx.allow_overlap(
        "beam_assembly",
        "hinge_post",
        reason="Hinge pin captured in hinge post barrel",
    )
    ctx.expect_contact(
        "beam_assembly",
        "hinge_post",
        name="hinge contact",
    )

    # Closed pose tests (q=0)
    with ctx.pose({hinge: 0.0}):
        ctx.allow_overlap(
            "beam_assembly",
            "latch_receiver_post",
            reason="Beam rests on latch receiver when closed",
        )
        ctx.expect_contact(
            "beam_assembly",
            "latch_receiver_post",
            name="closed beam contacts latch",
        )
        main_beam_aabb = ctx.part_element_world_aabb(beam, elem="main_beam")
        ctx.check(
            "closed beam horizontal",
            abs(main_beam_aabb[1][2] - 1.20) < 0.05,
            details=f"Main beam max Z: {main_beam_aabb[1][2]:.2f} (expected ~1.20)",
        )

    # Open pose tests (q=π/2)
    with ctx.pose({hinge: 1.5708}):
        main_beam_aabb = ctx.part_element_world_aabb(beam, elem="main_beam")
        ctx.check(
            "open beam raised",
            main_beam_aabb[1][2] > 3.0,
            details=f"Open main beam max Z: {main_beam_aabb[1][2]:.2f} (expected >3.0)",
        )

    # Visible details checks
    beam_visuals = beam.visuals
    stripe_count = sum(1 for v in beam_visuals if v.name.startswith("stripe_"))
    ctx.check(
        "striped paint present",
        stripe_count >= 10,
        details=f"Found {stripe_count} stripes (expected >=10)",
    )

    brace_visuals = [v for v in beam_visuals if v.name == "diagonal_brace"]
    ctx.check(
        "diagonal brace present",
        len(brace_visuals) == 1,
        details="Diagonal brace missing",
    )

    # Mechanism checks
    ctx.check(
        "hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"Hinge type: {hinge.articulation_type}",
    )
    ctx.check(
        "hinge axis is Y (pitch)",
        hinge.axis == (0.0, -1.0, 0.0),
        details=f"Hinge axis: {hinge.axis}",
    )

    return ctx.report()


object_model = build_object_model()
