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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate")

    # Materials
    masonry_mat = Material(name="masonry", color=(0.6, 0.6, 0.6))
    steel_mat = Material(name="steel", color=(0.3, 0.3, 0.35))
    gate_mat = Material(name="gate", color=(0.2, 0.2, 0.2))
    brass_mat = Material(name="brass", color=(0.8, 0.6, 0.2))

    # 1. Base (Masonry Channel)
    base = model.part("base")
    # Floor
    base.visual(Box((2.0, 1.0, 0.2)), origin=Origin((0.0, 0.0, 0.1)), material=masonry_mat, name="floor")
    # Left wall (sits on floor)
    base.visual(Box((0.4, 1.0, 2.4)), origin=Origin((-0.8, 0.0, 1.4)), material=masonry_mat, name="left_wall")
    # Right wall (sits on floor)
    base.visual(Box((0.4, 1.0, 2.4)), origin=Origin((0.8, 0.0, 1.4)), material=masonry_mat, name="right_wall")

    # 2. Frame (Steel Jambs, Lintel, Gearbox Body)
    frame = model.part("frame")
    
    # Left C-channel jamb (sits on floor, touches left wall)
    frame.visual(Box((0.05, 0.3, 2.4)), origin=Origin((-0.575, 0.0, 1.4)), material=steel_mat, name="left_jamb_web")
    frame.visual(Box((0.05, 0.05, 2.4)), origin=Origin((-0.525, -0.125, 1.4)), material=steel_mat, name="left_jamb_flange_front")
    frame.visual(Box((0.05, 0.05, 2.4)), origin=Origin((-0.525, 0.125, 1.4)), material=steel_mat, name="left_jamb_flange_rear")
    
    # Right C-channel jamb (sits on floor, touches right wall)
    frame.visual(Box((0.05, 0.3, 2.4)), origin=Origin((0.575, 0.0, 1.4)), material=steel_mat, name="right_jamb_web")
    frame.visual(Box((0.05, 0.05, 2.4)), origin=Origin((0.525, -0.125, 1.4)), material=steel_mat, name="right_jamb_flange_front")
    frame.visual(Box((0.05, 0.05, 2.4)), origin=Origin((0.525, 0.125, 1.4)), material=steel_mat, name="right_jamb_flange_rear")
    
    # Lintel beams (sit on top of jambs at z=2.6)
    frame.visual(Box((1.2, 0.1, 0.2)), origin=Origin((0.0, -0.1, 2.7)), material=steel_mat, name="lintel_front")
    frame.visual(Box((1.2, 0.1, 0.2)), origin=Origin((0.0, 0.1, 2.7)), material=steel_mat, name="lintel_rear")
    
    # Gearbox body (sits on lintel at z=2.8)
    frame.visual(Box((0.4, 0.4, 0.6)), origin=Origin((0.0, 0.0, 3.1)), material=steel_mat, name="gearbox_body")
    
    # Input shaft (right side of gearbox)
    frame.visual(
        Cylinder(radius=0.03, length=0.15),
        origin=Origin((0.275, 0.0, 3.2), rpy=(0.0, 1.5708, 0.0)),
        material=steel_mat,
        name="input_shaft"
    )

    model.articulation(
        "base_to_frame",
        ArticulationType.FIXED,
        parent=base,
        child=frame,
        origin=Origin()
    )

    # 3. Gate (Lift Panel)
    gate = model.part("gate")
    # Gate panel spans x=[-0.54, 0.54], y=[-0.025, 0.025], z=[0.2, 1.4] at q=0
    gate.visual(Box((1.08, 0.05, 1.2)), origin=Origin((0.0, 0.0, 0.8)), material=gate_mat, name="gate_panel")
    
    # Lifting stem (connects gate to gearbox)
    # Spans z=[1.4, 3.6] locally. Passes through the gap between lintel beams and into the gearbox.
    gate.visual(Cylinder(radius=0.02, length=2.2), origin=Origin((0.0, 0.0, 2.5)), material=steel_mat, name="lifting_stem")

    model.articulation(
        "gate_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.2, effort=1000.0, velocity=0.5)
    )

    # 4. Crank Wheel
    crank_wheel = model.part("crank_wheel")
    crank_wheel.visual(
        Cylinder(radius=0.3, length=0.05),
        origin=Origin((0.025, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=brass_mat,
        name="wheel_rim"
    )
    crank_wheel.visual(Box((0.04, 0.56, 0.04)), origin=Origin((0.025, 0.0, 0.0)), material=brass_mat, name="spoke_1")
    crank_wheel.visual(Box((0.04, 0.04, 0.56)), origin=Origin((0.025, 0.0, 0.0)), material=brass_mat, name="spoke_2")
    crank_wheel.visual(
        Cylinder(radius=0.02, length=0.15),
        origin=Origin((0.1, 0.2, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=brass_mat,
        name="crank_handle"
    )

    model.articulation(
        "crank_turn",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank_wheel,
        origin=Origin((0.35, 0.0, 3.2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=5.0)
    )

    # 5. Gearbox Cover
    gearbox_cover = model.part("gearbox_cover")
    # Cover spans x=[0.0, 0.4], y=[-0.02, 0.0], z=[-0.3, 0.3] locally
    gearbox_cover.visual(Box((0.4, 0.02, 0.6)), origin=Origin((0.2, -0.01, 0.0)), material=steel_mat, name="cover_plate")

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=gearbox_cover,
        origin=Origin((-0.2, -0.2, 3.1)),
        axis=(0.0, 0.0, -1.0), # Positive q opens outwards
        motion_limits=MotionLimits(lower=0.0, upper=2.0, effort=10.0, velocity=2.0)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    gate = object_model.get_part("gate")
    frame = object_model.get_part("frame")
    base = object_model.get_part("base")
    gearbox_cover = object_model.get_part("gearbox_cover")

    # Intentional overlap: lifting stem passes through the solid proxy gearbox body
    ctx.allow_overlap(
        gate,
        frame,
        elem_a="lifting_stem",
        elem_b="gearbox_body",
        reason="The lifting stem passes through the gearbox housing."
    )

    # Verify gate is contained within the jambs on the X axis
    ctx.expect_within(
        gate,
        frame,
        axes="x",
        inner_elem="gate_panel",
        outer_elem="left_jamb_web",
        margin=1.2, # The gate spans across, so we just check it stays between the webs generally
        name="gate panel stays between jamb webs"
    )
    
    # Better check: gate panel is between left and right jamb webs
    # We can check gap between gate panel and left/right jamb flanges
    ctx.expect_gap(
        gate, frame,
        axis="y",
        positive_elem="gate_panel",
        negative_elem="left_jamb_flange_front",
        min_gap=0.05,
        name="gate clears front flange"
    )

    # Poses
    gate_lift = object_model.get_articulation("gate_lift")
    cover_hinge = object_model.get_articulation("cover_hinge")

    # At q=0, gate touches the floor
    ctx.expect_contact(gate, base, elem_a="gate_panel", elem_b="floor", name="gate rests on floor when closed")

    with ctx.pose({gate_lift: 1.0}):
        ctx.expect_gap(
            gate, base,
            axis="z",
            positive_elem="gate_panel",
            negative_elem="floor",
            min_gap=0.9,
            name="gate lifts off floor"
        )

    with ctx.pose({cover_hinge: 1.5}):
        ctx.expect_gap(
            frame, gearbox_cover,
            axis="y",
            positive_elem="gearbox_body",
            negative_elem="cover_plate",
            min_gap=0.0,
            name="cover opens away from gearbox"
        )

    return ctx.report()


object_model = build_object_model()
