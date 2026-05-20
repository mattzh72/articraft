import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    SpurGear,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixie_crankset")

    # 1. Bottom Bracket (Root)
    bottom_bracket = model.part("bottom_bracket")
    bb_cq = cq.Workplane("XZ").cylinder(0.068, 0.0175)
    bottom_bracket.visual(mesh_from_cadquery(bb_cq, "bb_mesh"), name="bb_shell")

    # 2. Spindle
    spindle = model.part("spindle")
    spindle_cq = cq.Workplane("XZ").cylinder(0.105, 0.01)
    spindle.visual(mesh_from_cadquery(spindle_cq, "spindle_mesh"), name="spindle_body")

    model.articulation(
        "bb_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0),
    )

    # Crank arm builder
    def make_crank_arm(side_y: float, angle_deg: float, has_spider: bool = False):
        length = 0.165
        thickness = 0.015
        
        # Arm body
        arm = cq.Workplane("XY").box(0.03, thickness, length, centered=(True, True, False)).translate((0, 0, -length))
        # Boss at spindle
        boss1 = cq.Workplane("XZ").cylinder(thickness, 0.015)
        # Boss at pedal
        boss2 = cq.Workplane("XZ").center(0, -length).cylinder(thickness, 0.015)
        
        res = arm.union(boss1).union(boss2)
        
        if has_spider:
            # Spider disk to mount chainring
            spider = cq.Workplane("XZ").cylinder(0.0025, 0.065).translate((0, -0.00875, 0))
            res = res.union(spider)
            
        res = res.translate((0, side_y, 0))
        
        if angle_deg != 0:
            res = res.rotate((0, 0, 0), (0, 1, 0), angle_deg)
            
        return res

    # 3. Right Crank Arm
    right_crank_arm = model.part("right_crank_arm")
    right_crank_cq = make_crank_arm(side_y=0.06, angle_deg=0, has_spider=True)
    right_crank_arm.visual(mesh_from_cadquery(right_crank_cq, "right_crank_mesh"), name="right_crank_body")

    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # 4. Left Crank Arm
    left_crank_arm = model.part("left_crank_arm")
    left_crank_cq = make_crank_arm(side_y=-0.06, angle_deg=180, has_spider=False)
    left_crank_arm.visual(mesh_from_cadquery(left_crank_cq, "left_crank_mesh"), name="left_crank_body")

    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # 5. Chainring
    chainring = model.part("chainring")
    chainring_cq = (
        SpurGear(module=0.003, teeth_number=48, width=0.003, bore_d=0.120)
        .build()
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0, 0.05, 0))
    )
    chainring.visual(mesh_from_cadquery(chainring_cq, "chainring_mesh"), name="chainring_body")

    model.articulation(
        "right_crank_to_chainring",
        ArticulationType.FIXED,
        parent=right_crank_arm,
        child=chainring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Pedal builder
    def make_pedal(direction: int):
        axle_len = 0.02
        axle = cq.Workplane("XZ").cylinder(axle_len, 0.006).translate((0, direction * axle_len / 2, 0))
        
        body = cq.Workplane("XY").box(0.08, 0.09, 0.02)
        body = body.translate((0, direction * (axle_len + 0.09 / 2), 0))
        
        return axle.union(body)

    # 6. Right Pedal
    right_pedal = model.part("right_pedal")
    right_pedal_cq = make_pedal(direction=1)
    right_pedal.visual(mesh_from_cadquery(right_pedal_cq, "right_pedal_mesh"), name="right_pedal_body")

    model.articulation(
        "right_crank_to_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_crank_arm,
        child=right_pedal,
        origin=Origin(xyz=(0.0, 0.0675, -0.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0),
    )

    # 7. Left Pedal
    left_pedal = model.part("left_pedal")
    left_pedal_cq = make_pedal(direction=-1)
    left_pedal.visual(mesh_from_cadquery(left_pedal_cq, "left_pedal_mesh"), name="left_pedal_body")

    model.articulation(
        "left_crank_to_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank_arm,
        child=left_pedal,
        origin=Origin(xyz=(0.0, -0.0675, 0.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottom_bracket = object_model.get_part("bottom_bracket")
    spindle = object_model.get_part("spindle")
    right_crank_arm = object_model.get_part("right_crank_arm")
    left_crank_arm = object_model.get_part("left_crank_arm")
    chainring = object_model.get_part("chainring")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    # Allow overlap for the spindle inside the bottom bracket sleeve
    ctx.allow_overlap(
        spindle,
        bottom_bracket,
        reason="The spindle passes completely through the bottom bracket shell.",
    )

    ctx.expect_within(
        spindle,
        bottom_bracket,
        axes="xz",
        margin=0.0,
        name="Spindle is radially centered inside the BB shell",
    )

    ctx.expect_contact(
        spindle,
        right_crank_arm,
        name="Spindle touches right crank arm",
    )
    
    ctx.expect_contact(
        spindle,
        left_crank_arm,
        name="Spindle touches left crank arm",
    )

    ctx.expect_contact(
        right_crank_arm,
        chainring,
        name="Chainring touches right crank arm",
    )

    ctx.expect_contact(
        right_crank_arm,
        right_pedal,
        name="Right pedal touches right crank arm",
    )

    ctx.expect_contact(
        left_crank_arm,
        left_pedal,
        name="Left pedal touches left crank arm",
    )

    # Pose check: pedals rotate
    with ctx.pose({"right_crank_to_pedal": 1.57}):
        ctx.expect_contact(right_crank_arm, right_pedal)

    with ctx.pose({"bb_to_spindle": 1.57}):
        # Entire crank assembly rotates
        ctx.expect_contact(spindle, right_crank_arm)

    return ctx.report()


object_model = build_object_model()
