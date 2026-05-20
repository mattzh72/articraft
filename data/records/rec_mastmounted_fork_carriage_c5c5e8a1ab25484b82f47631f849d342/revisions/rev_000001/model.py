from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
)
import cadquery as cq

def make_mast_cq():
    mast_width = 0.8
    col_width = 0.12
    col_depth = 0.16
    mast_height = 2.0
    
    # Left C-channel
    left_col = (
        cq.Workplane("XY")
        .box(col_width, col_depth, mast_height)
        .translate((-0.34, 0, mast_height/2))
    )
    left_cut = (
        cq.Workplane("XY")
        .box(0.08, col_depth - 0.04, mast_height)
        .translate((-0.32, 0, mast_height/2))
    )
    left_col = left_col.cut(left_cut)
    
    # Right C-channel
    right_col = (
        cq.Workplane("XY")
        .box(col_width, col_depth, mast_height)
        .translate((0.34, 0, mast_height/2))
    )
    right_cut = (
        cq.Workplane("XY")
        .box(0.08, col_depth - 0.04, mast_height)
        .translate((0.32, 0, mast_height/2))
    )
    right_col = right_col.cut(right_cut)
    
    # Crossbars
    top_bar = (
        cq.Workplane("XY")
        .box(mast_width, 0.04, 0.1)
        .translate((0, -col_depth/2 + 0.02, mast_height - 0.05))
    )
    bottom_bar = (
        cq.Workplane("XY")
        .box(mast_width, 0.04, 0.1)
        .translate((0, -col_depth/2 + 0.02, 0.05))
    )
    
    return left_col.union(right_col).union(top_bar).union(bottom_bar)

def make_carriage_cq():
    c_width = 0.54
    c_height = 0.6
    c_depth = 0.06
    
    plate = (
        cq.Workplane("XY")
        .box(c_width, c_depth, c_height)
        .translate((0, 0, c_height/2))
    )
    
    rollers = cq.Workplane("XY")
    for sign in [-1, 1]:
        for z in [0.1, c_height - 0.1]:
            axle = (
                cq.Workplane("YZ")
                .workplane(offset=sign * 0.27)
                .center(0, z)
                .circle(0.02)
                .extrude(sign * 0.05)
            )
            roller = (
                cq.Workplane("YZ")
                .workplane(offset=sign * 0.30)
                .center(0, z)
                .circle(0.06)
                .extrude(sign * 0.06)
            )
            rollers = rollers.union(axle).union(roller)
            
    top_bar = (
        cq.Workplane("XY")
        .box(c_width, 0.04, 0.08)
        .translate((0, 0.05, c_height - 0.04))
    )
    bottom_bar = (
        cq.Workplane("XY")
        .box(c_width, 0.04, 0.08)
        .translate((0, 0.05, 0.04))
    )
    
    return plate.union(rollers).union(top_bar).union(bottom_bar)

def make_fork_cq():
    fork_width = 0.1
    fork_thickness = 0.04
    fork_length = 1.0
    fork_height = 0.6
    
    return (
        cq.Workplane("YZ")
        .moveTo(0, 0)
        .lineTo(fork_length, 0)
        .lineTo(fork_length, fork_thickness)
        .lineTo(fork_thickness, fork_thickness)
        .lineTo(fork_thickness, fork_height)
        .lineTo(0, fork_height)
        .close()
        .extrude(fork_width/2, both=True)
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_carriage")

    mat_mast = Material("mast_mat", rgba=(0.2, 0.2, 0.2, 1.0))
    mat_carriage = Material("carriage_mat", rgba=(0.8, 0.2, 0.1, 1.0))
    mat_fork = Material("fork_mat", rgba=(0.1, 0.1, 0.1, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(make_mast_cq(), "mast_mesh"),
        name="mast_body",
        material=mat_mast
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_cq(), "carriage_mesh"),
        name="carriage_body",
        material=mat_carriage
    )

    model.articulation(
        "lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0, 0, 0.1)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=1.3, effort=1000.0, velocity=1.0)
    )

    fork_shape = make_fork_cq()
    for i, x_offset in enumerate([-0.2, 0.2]):
        fork = model.part(f"fork_{i}")
        fork.visual(
            mesh_from_cadquery(fork_shape, f"fork_mesh_{i}"),
            name=f"fork_body_{i}",
            material=mat_fork
        )
        model.articulation(
            f"carriage_to_fork_{i}",
            ArticulationType.FIXED,
            parent=carriage,
            child=fork,
            origin=Origin(xyz=(x_offset, 0.07, 0))
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("lift")
    
    ctx.allow_overlap(
        carriage,
        mast,
        reason="The carriage rollers are intentionally captured inside the mast C-channels."
    )
    
    ctx.expect_within(carriage, mast, axes="xy", margin=0.001, name="carriage fits within mast footprint")
    
    with ctx.pose({lift: 1.3}):
        ctx.expect_within(carriage, mast, axes="xy", margin=0.001, name="carriage fits within mast footprint at max lift")
        
    return ctx.report()

object_model = build_object_model()