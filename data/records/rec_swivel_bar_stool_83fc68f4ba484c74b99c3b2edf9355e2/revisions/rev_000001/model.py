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

Chrome = Material("chrome", rgba=(0.9, 0.9, 0.92, 1.0))
BlackPlastic = Material("black_plastic", rgba=(0.1, 0.1, 0.1, 1.0))
Wood = Material("wood", rgba=(0.6, 0.4, 0.2, 1.0))
Leather = Material("leather", rgba=(0.2, 0.2, 0.2, 1.0))

def make_base():
    disc = cq.Workplane("XY").cylinder(0.01, 0.25).translate((0, 0, 0.005))
    cone = (
        cq.Workplane("XZ")
        .moveTo(0, 0)
        .lineTo(0.25, 0)
        .lineTo(0.04, 0.06)
        .lineTo(0, 0.06)
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    ).translate((0, 0, 0.01))
    tube = cq.Workplane("XY").cylinder(0.21, 0.03).translate((0, 0, 0.175))
    collar = cq.Workplane("XY").cylinder(0.02, 0.035).translate((0, 0, 0.27))
    base_solid = disc.union(cone).union(tube).union(collar)
    
    hole = cq.Workplane("XY").cylinder(0.3, 0.025).translate((0, 0, 0.15))
    return base_solid.cut(hole)

def make_inner_tube():
    return cq.Workplane("XY").cylinder(0.60, 0.025).translate((0, 0, 0.05))

def make_seat_shell():
    pan = cq.Workplane("XY").cylinder(0.04, 0.18).translate((0, 0, 0.02))
    backrest = cq.Workplane("XY").cylinder(0.20, 0.18).translate((0, 0, 0.14))
    inner_cut = cq.Workplane("XY").cylinder(0.25, 0.16).translate((0, 0, 0.14))
    backrest = backrest.cut(inner_cut)
    front_cut = cq.Workplane("XY").cylinder(0.3, 0.25).translate((0, 0.20, 0.14))
    backrest = backrest.cut(front_cut)
    shell = pan.union(backrest)
    try:
        shell = shell.edges().fillet(0.005)
    except:
        pass
    return shell

def make_back_cushion():
    backrest = cq.Workplane("XY").cylinder(0.18, 0.16).translate((0, 0, 0.14))
    inner_cut = cq.Workplane("XY").cylinder(0.25, 0.14).translate((0, 0, 0.14))
    backrest = backrest.cut(inner_cut)
    front_cut = cq.Workplane("XY").cylinder(0.3, 0.25).translate((0, 0.20, 0.14))
    backrest = backrest.cut(front_cut)
    try:
        backrest = backrest.edges().fillet(0.005)
    except:
        pass
    return backrest

def make_cushion():
    cushion = cq.Workplane("XY").cylinder(0.04, 0.16).translate((0, 0, 0.05))
    try:
        cushion = cushion.edges("|Z").fillet(0.01)
        cushion = cushion.edges(">Z").fillet(0.015)
    except:
        pass
    return cushion

def make_footrest():
    hub = cq.Workplane("XY").cylinder(0.04, 0.035)
    path = cq.Workplane("XY").circle(0.18)
    ring = cq.Workplane("XZ").center(0.18, 0).circle(0.01).sweep(path)
    spoke_x = cq.Workplane("YZ").cylinder(0.36, 0.01)
    spoke_y = cq.Workplane("XZ").cylinder(0.36, 0.01)
    return hub.union(ring).union(spoke_x).union(spoke_y).translate((0, 0, 0.05))

def make_lever():
    rod = cq.Workplane("YZ").cylinder(0.09, 0.005).translate((0.095, 0, 0))
    paddle = cq.Workplane("XY").box(0.04, 0.02, 0.005).translate((0.16, 0, 0))
    return rod.union(paddle)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base(), "base_geom"),
        name="base_visual",
        material=Chrome
    )

    lift_slider = model.part("lift_slider")
    # Dummy part for prismatic joint

    seat_assembly = model.part("seat_assembly")
    seat_assembly.visual(
        mesh_from_cadquery(make_inner_tube(), "inner_tube_geom"),
        name="inner_tube",
        material=Chrome
    )
    seat_assembly.visual(
        mesh_from_cadquery(cq.Workplane("XY").box(0.12, 0.12, 0.04), "mechanism_box_geom"),
        origin=Origin(xyz=(0, 0, 0.33)),
        name="mechanism_box",
        material=BlackPlastic
    )
    seat_assembly.visual(
        mesh_from_cadquery(make_seat_shell(), "seat_shell_geom"),
        origin=Origin(xyz=(0, 0, 0.35)),
        name="seat_shell",
        material=Wood
    )
    seat_assembly.visual(
        mesh_from_cadquery(make_cushion(), "seat_cushion_geom"),
        origin=Origin(xyz=(0, 0, 0.35)),
        name="seat_cushion",
        material=Leather
    )
    seat_assembly.visual(
        mesh_from_cadquery(make_back_cushion(), "back_cushion_geom"),
        origin=Origin(xyz=(0, 0, 0.35)),
        name="back_cushion",
        material=Leather
    )
    seat_assembly.visual(
        mesh_from_cadquery(make_footrest(), "footrest_geom"),
        origin=Origin(xyz=(0, 0, 0.0)),
        name="footrest",
        material=Chrome
    )
    seat_assembly.visual(
        mesh_from_cadquery(make_lever(), "lever_geom"),
        origin=Origin(xyz=(0, 0, 0.33), rpy=(0, 0, 0.5)),
        name="lever",
        material=BlackPlastic
    )

    model.articulation(
        "height_adjustment",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_slider,
        origin=Origin(xyz=(0, 0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=0.20),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.REVOLUTE,
        parent=lift_slider,
        child=seat_assembly,
        origin=Origin(xyz=(0, 0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-3.14159, upper=3.14159),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    seat_assembly = object_model.get_part("seat_assembly")
    
    height_joint = object_model.get_articulation("height_adjustment")
    
    ctx.allow_overlap(
        seat_assembly,
        base,
        elem_a="inner_tube",
        elem_b="base_visual",
        reason="The inner tube slides and swivels inside the outer base tube."
    )
    
    ctx.expect_within(
        seat_assembly,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="base_visual",
        margin=0.001,
        name="inner tube stays centered in base"
    )
    
    ctx.expect_overlap(
        seat_assembly,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="base_visual",
        min_overlap=0.10,
        name="inner tube retains insertion at lowest position"
    )
    
    with ctx.pose({height_joint: 0.20}):
        ctx.expect_overlap(
            seat_assembly,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="base_visual",
            min_overlap=0.04,
            name="inner tube retains insertion at highest position"
        )
        
    ctx.expect_gap(
        seat_assembly,
        base,
        axis="z",
        positive_elem="footrest",
        negative_elem="base_visual",
        min_gap=0.01,
        name="footrest clears base at lowest position"
    )

    return ctx.report()

object_model = build_object_model()
