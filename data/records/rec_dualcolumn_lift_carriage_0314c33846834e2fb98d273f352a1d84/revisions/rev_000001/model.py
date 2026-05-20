from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)
import cadquery as cq

def create_base():
    rear = cq.Workplane("XY").box(0.3, 0.9, 0.1).translate((-0.05, 0, 0.05))
    left_leg = cq.Workplane("XY").box(0.6, 0.1, 0.1).translate((0.4, -0.3, 0.05))
    right_leg = cq.Workplane("XY").box(0.6, 0.1, 0.1).translate((0.4, 0.3, 0.05))
    return rear.union(left_leg).union(right_leg)

def create_columns():
    left_col = cq.Workplane("XY", origin=(0, -0.3, 0.05)).circle(0.02).extrude(1.1)
    right_col = cq.Workplane("XY", origin=(0, 0.3, 0.05)).circle(0.02).extrude(1.1)
    return left_col.union(right_col)

def create_top_bracket():
    return cq.Workplane("XY").box(0.2, 0.9, 0.05).translate((0, 0, 1.125))

def create_carriage():
    crosshead = cq.Workplane("XY").box(0.15, 0.48, 0.15)
    left_bearing = cq.Workplane("XY", origin=(0, -0.3, 0)).box(0.12, 0.12, 0.25)
    right_bearing = cq.Workplane("XY", origin=(0, 0.3, 0)).box(0.12, 0.12, 0.25)
    backplate = cq.Workplane("XY", origin=(0.1, 0, -0.05)).box(0.05, 0.4, 0.15)
    left_fork = cq.Workplane("XY", origin=(0.375, -0.15, -0.1)).box(0.6, 0.05, 0.05)
    right_fork = cq.Workplane("XY", origin=(0.375, 0.15, -0.1)).box(0.6, 0.05, 0.05)
    
    res = crosshead.union(left_bearing).union(right_bearing).union(backplate).union(left_fork).union(right_fork)
    
    holes = (
        cq.Workplane("XY", origin=(0, -0.3, -0.5)).circle(0.022).extrude(2.0)
        .union(
            cq.Workplane("XY", origin=(0, 0.3, -0.5)).circle(0.022).extrude(2.0)
        )
    )
    return res.cut(holes)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift")
    
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(create_base(), "base"),
        material=Material(name="base_mat", color=(0.15, 0.15, 0.15)),
        name="base_vis"
    )
    frame.visual(
        mesh_from_cadquery(create_columns(), "columns"),
        material=Material(name="columns_mat", color=(0.8, 0.8, 0.8)),
        name="columns_vis"
    )
    frame.visual(
        mesh_from_cadquery(create_top_bracket(), "top_bracket"),
        material=Material(name="top_bracket_mat", color=(0.15, 0.15, 0.15)),
        name="top_bracket_vis"
    )
    
    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(create_carriage(), "carriage"),
        material=Material(name="carriage_mat", color=(0.9, 0.5, 0.0)),
        name="carriage_vis"
    )
    
    model.articulation(
        "lift_joint",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0, 0, 0.25)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=0.7, effort=5000.0, velocity=0.5)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("lift_joint")
    
    ctx.allow_isolated_part(carriage, reason="Carriage slides on linear bearings with a small realistic clearance.")
    
    ctx.expect_gap(carriage, frame, positive_elem="carriage_vis", negative_elem="base_vis", axis="z", min_gap=0.01, name="carriage_clears_base_at_rest")
    
    with ctx.pose({lift: 0.7}):
        ctx.expect_gap(frame, carriage, positive_elem="top_bracket_vis", negative_elem="carriage_vis", axis="z", min_gap=0.01, name="carriage_clears_top_bracket")
        
        pos = ctx.part_world_position(carriage)
        ctx.check("carriage_moves_up", pos is not None and pos[2] > 0.8)
        
    return ctx.report()

object_model = build_object_model()
