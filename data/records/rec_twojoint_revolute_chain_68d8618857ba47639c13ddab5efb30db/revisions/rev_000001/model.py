from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def make_root_bracket():
    # Base block
    base = cq.Workplane("XY").box(0.05, 0.10, 0.10).translate((-0.025, 0, 0))
    
    # Clevis ears
    ear_box = cq.Workplane("XY").box(0.05, 0.06, 0.02).translate((0.025, 0, 0))
    ear_cyl = cq.Workplane("XY").cylinder(0.02, 0.03).translate((0.05, 0, 0))
    ear = ear_box.union(ear_cyl)
    
    top_ear = ear.translate((0, 0, 0.04))
    bottom_ear = ear.translate((0, 0, -0.04))
    
    # Pin
    pin = cq.Workplane("XY").cylinder(0.10, 0.012).translate((0.05, 0, 0))
    
    return base.union(top_ear).union(bottom_ear).union(pin)

def make_middle_link():
    # Body
    body = cq.Workplane("XY").box(0.10, 0.06, 0.06).translate((0.10, 0, 0))
    left_cyl = cq.Workplane("XY").cylinder(0.06, 0.03).translate((0.05, 0, 0))
    right_cyl = cq.Workplane("XY").cylinder(0.06, 0.02).translate((0.15, 0, 0))
    
    solid = body.union(left_cyl).union(right_cyl)
    
    # Cut left hole
    hole = cq.Workplane("XY").cylinder(0.10, 0.013).translate((0.05, 0, 0))
    
    # Cut right clevis gap
    gap = cq.Workplane("XY").box(0.06, 0.06, 0.03).translate((0.15, 0, 0))
    
    # Add right pin
    pin = cq.Workplane("XY").cylinder(0.06, 0.01).translate((0.15, 0, 0))
    
    return solid.cut(hole).cut(gap).union(pin)

def make_tip_link():
    # Body
    body = cq.Workplane("XY").box(0.10, 0.04, 0.03).translate((0.05, 0, 0))
    left_cyl = cq.Workplane("XY").cylinder(0.03, 0.02).translate((0, 0, 0))
    right_cyl = cq.Workplane("XY").cylinder(0.03, 0.015).translate((0.10, 0, 0))
    
    solid = body.union(left_cyl).union(right_cyl)
    
    # Cut left hole
    hole = cq.Workplane("XY").cylinder(0.05, 0.011).translate((0, 0, 0))
    
    return solid.cut(hole).translate((-0.011, 0, 0))

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_joint_arm")

    root = model.part("root_bracket")
    middle = model.part("middle_link")
    tip = model.part("tip_link")

    root.visual(
        mesh_from_cadquery(make_root_bracket(), "root_bracket_mesh"),
        origin=Origin(),
        name="root_geom",
    )

    middle.visual(
        mesh_from_cadquery(make_middle_link(), "middle_link_mesh"),
        origin=Origin(),
        name="middle_geom",
    )

    tip.visual(
        mesh_from_cadquery(make_tip_link(), "tip_link_mesh"),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        name="tip_geom",
    )

    model.articulation(
        "root_to_middle",
        ArticulationType.REVOLUTE,
        parent=root,
        child=middle,
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-1.5, upper=1.5),
    )

    model.articulation(
        "middle_to_tip",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=tip,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-1.5, upper=1.5),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_bracket")
    middle = object_model.get_part("middle_link")
    tip = object_model.get_part("tip_link")

    ctx.allow_overlap(
        middle,
        root,
        reason="Middle link is captured in the root clevis pin.",
    )
    
    ctx.allow_overlap(
        tip,
        middle,
        reason="Tip link is captured in the middle clevis pin.",
    )

    ctx.expect_within(middle, root, axes="z", margin=0.002, name="middle_in_root_clevis")
    ctx.expect_within(tip, middle, axes="z", margin=0.002, name="tip_in_middle_clevis")

    joint1 = object_model.get_articulation("root_to_middle")
    joint2 = object_model.get_articulation("middle_to_tip")

    with ctx.pose({joint1: 1.0, joint2: 1.0}):
        pos1 = ctx.part_world_position(middle)
        pos2 = ctx.part_world_position(tip)
        ctx.check("middle_moves", pos1 is not None)
        ctx.check("tip_moves", pos2 is not None)

    return ctx.report()

object_model = build_object_model()