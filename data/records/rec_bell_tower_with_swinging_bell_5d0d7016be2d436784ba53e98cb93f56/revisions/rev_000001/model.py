from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bell_tower")
    
    # Materials
    stone_mat = Material(name="stone", color=(0.4, 0.4, 0.42))
    brick_mat = Material(name="brick", color=(0.6, 0.5, 0.4))
    wood_mat = Material(name="wood", color=(0.4, 0.25, 0.15))
    bronze_mat = Material(name="bronze", color=(0.8, 0.5, 0.2))
    roof_mat = Material(name="roof", color=(0.2, 0.2, 0.25))
    iron_mat = Material(name="iron", color=(0.15, 0.15, 0.15))
    
    # Structure
    tower = model.part("tower")
    
    # Base
    tower.visual(Box((5.0, 5.0, 1.0)), origin=Origin(xyz=(0, 0, 0.5)), material=stone_mat, name="base")
    
    # Body
    tower.visual(Box((4.0, 4.0, 10.0)), origin=Origin(xyz=(0, 0, 1.0 + 5.0)), material=brick_mat, name="body")
    
    # Belfry floor
    tower.visual(Box((4.4, 4.4, 0.4)), origin=Origin(xyz=(0, 0, 11.0 + 0.2)), material=stone_mat, name="belfry_floor")
    
    # Pillars
    for x in [-1.8, 1.8]:
        for y in [-1.8, 1.8]:
            tower.visual(
                Box((0.6, 0.6, 4.0)),
                origin=Origin(xyz=(x, y, 11.4 + 2.0)),
                material=stone_mat,
                name=f"pillar_{x}_{y}".replace("-", "m").replace(".", "p")
            )
            
    # Belfry walls (low walls between pillars)
    for x, y, dx, dy in [
        (0, 1.8, 3.0, 0.4),
        (0, -1.8, 3.0, 0.4),
        (1.8, 0, 0.4, 3.0),
        (-1.8, 0, 0.4, 3.0),
    ]:
        tower.visual(
            Box((dx, dy, 1.0)),
            origin=Origin(xyz=(x, y, 11.4 + 0.5)),
            material=brick_mat,
            name=f"belfry_wall_{x}_{y}".replace("-", "m").replace(".", "p")
        )
        
    # Roof
    roof_solid = (
        cq.Workplane("XY")
        .rect(4.8, 4.8)
        .workplane(offset=3.0)
        .rect(0.2, 0.2)
        .loft(combine=True)
    )
    tower.visual(
        mesh_from_cadquery(roof_solid, "roof_mesh"),
        origin=Origin(xyz=(0, 0, 15.4)),
        material=roof_mat,
        name="roof"
    )
    
    # Yoke supports
    tower.visual(Box((0.4, 3.0, 0.6)), origin=Origin(xyz=(-1.3, 0, 14.5)), material=wood_mat, name="yoke_support_left")
    tower.visual(Box((0.4, 3.0, 0.6)), origin=Origin(xyz=(1.3, 0, 14.5)), material=wood_mat, name="yoke_support_right")
    
    # Yoke Part
    yoke = model.part("yoke")
    yoke.visual(Box((1.8, 0.6, 0.6)), origin=Origin(xyz=(0, 0, 0)), material=wood_mat, name="yoke_beam")
    yoke.visual(Cylinder(radius=0.4, length=1.2), origin=Origin(xyz=(0, 0, 0.4), rpy=(0, 1.5708, 0)), material=wood_mat, name="yoke_headstock")
    yoke.visual(Cylinder(radius=0.1, length=2.8), origin=Origin(xyz=(0, 0, 0), rpy=(0, 1.5708, 0)), material=iron_mat, name="yoke_axle")
    
    model.articulation(
        "yoke_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yoke,
        origin=Origin(xyz=(0, 0, 14.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1000.0, velocity=5.0, lower=-1.5, upper=1.5)
    )
    
    # Bell Part
    bell_solid = (
        cq.Workplane("XZ")
        .moveTo(0, 0)
        .lineTo(0.3, 0)
        .spline([(0.45, -0.5), (0.6, -1.0), (0.75, -1.3)], includeCurrent=True)
        .spline([(0.8, -1.4), (0.8, -1.5)], includeCurrent=True)
        .lineTo(0.7, -1.5)
        .spline([(0.7, -1.4), (0.65, -1.3)], includeCurrent=True)
        .spline([(0.5, -1.0), (0.35, -0.5), (0.2, -0.1)], includeCurrent=True)
        .lineTo(0, -0.1)
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    
    bell = model.part("bell")
    bell.visual(
        mesh_from_cadquery(bell_solid, "bell_mesh"),
        origin=Origin(xyz=(0, 0, 0)),
        material=bronze_mat,
        name="bell_body"
    )
    
    model.articulation(
        "yoke_to_bell",
        ArticulationType.FIXED,
        parent=yoke,
        child=bell,
        origin=Origin(xyz=(0, 0, -0.3))
    )
    
    # Clapper
    clapper = model.part("clapper")
    clapper.visual(Cylinder(radius=0.03, length=1.2), origin=Origin(xyz=(0, 0, -0.6)), material=iron_mat, name="clapper_shaft")
    clapper.visual(Cylinder(radius=0.1, length=0.15), origin=Origin(xyz=(0, 0, -1.2)), material=iron_mat, name="clapper_bob")
    clapper.visual(Cylinder(radius=0.05, length=0.1), origin=Origin(xyz=(0, 0, -0.05), rpy=(0, 1.5708, 0)), material=iron_mat, name="clapper_hinge")
    
    model.articulation(
        "clapper_swing",
        ArticulationType.REVOLUTE,
        parent=bell,
        child=clapper,
        origin=Origin(xyz=(0, 0, -0.1)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=10.0, lower=-0.5, upper=0.5)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    tower = object_model.get_part("tower")
    yoke = object_model.get_part("yoke")
    bell = object_model.get_part("bell")
    clapper = object_model.get_part("clapper")
    
    ctx.allow_overlap(yoke, tower, elem_a="yoke_axle", elem_b="yoke_support_left", reason="Axle is captured in the left bearing.")
    ctx.allow_overlap(yoke, tower, elem_a="yoke_axle", elem_b="yoke_support_right", reason="Axle is captured in the right bearing.")
    ctx.allow_overlap(clapper, bell, reason="Clapper is suspended from the inner top of the bell.")
    ctx.allow_overlap(yoke, bell, reason="Bell is bolted to the bottom of the yoke beam.")
    
    ctx.expect_overlap(yoke, tower, axes="x", elem_a="yoke_axle", elem_b="yoke_support_left", min_overlap=0.1, name="Axle is retained in left support")
    ctx.expect_overlap(yoke, tower, axes="x", elem_a="yoke_axle", elem_b="yoke_support_right", min_overlap=0.1, name="Axle is retained in right support")
    
    yoke_swing = object_model.get_articulation("yoke_swing")
    with ctx.pose({yoke_swing: 1.0}):
        ctx.expect_overlap(yoke, tower, axes="x", elem_a="yoke_axle", elem_b="yoke_support_left", min_overlap=0.1, name="Axle stays in left support when swung")
        ctx.expect_overlap(yoke, tower, axes="x", elem_a="yoke_axle", elem_b="yoke_support_right", min_overlap=0.1, name="Axle stays in right support when swung")
        
    return ctx.report()

object_model = build_object_model()
