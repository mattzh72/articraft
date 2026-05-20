import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lazy_susan_table")

    table_wood = Material(color=(0.40, 0.22, 0.10), name="table_wood")
    susan_wood = Material(color=(0.45, 0.25, 0.12), name="susan_wood")
    
    # Table Base (Root)
    table = model.part("table")
    
    wp = cq.Workplane("XY")
    
    # Base tiers
    tier1 = wp.circle(0.4).extrude(0.04).edges(">Z").chamfer(0.01)
    tier2 = wp.workplane(offset=0.04).circle(0.3).extrude(0.04).edges(">Z").chamfer(0.01)
    
    # Column
    column = wp.workplane(offset=0.08).circle(0.12).extrude(0.57)
    
    # Top support (mounting plate for the tabletop)
    support = wp.workplane(offset=0.65).circle(0.35).extrude(0.05).edges("<Z").chamfer(0.02)
    
    # Tabletop
    tabletop = wp.workplane(offset=0.70).circle(0.75).extrude(0.04).edges(">Z").fillet(0.01)
    
    # Bearing base (stationary part of lazy susan mechanism)
    bearing_base = wp.workplane(offset=0.74).circle(0.15).extrude(0.005)
    
    table_geom = tier1.union(tier2).union(column).union(support).union(tabletop).union(bearing_base)
    
    table.visual(
        mesh_from_cadquery(table_geom, "table_geom"),
        material=table_wood,
        name="table_visual"
    )

    # Lazy Susan (Child)
    lazy_susan = model.part("lazy_susan")
    
    ls_wp = cq.Workplane("XY")
    # Bearing top (rotates)
    bearing_top = ls_wp.circle(0.15).extrude(0.005)
    # Tray
    tray = ls_wp.workplane(offset=0.005).circle(0.35).extrude(0.02).edges(">Z").fillet(0.005)
    
    ls_geom = bearing_top.union(tray)
    
    lazy_susan.visual(
        mesh_from_cadquery(ls_geom, "lazy_susan_geom"),
        material=susan_wood,
        name="lazy_susan_visual"
    )

    # Articulation
    model.articulation(
        "table_to_lazy_susan",
        ArticulationType.CONTINUOUS,
        parent=table,
        child=lazy_susan,
        origin=Origin(xyz=(0.0, 0.0, 0.745)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    table = object_model.get_part("table")
    lazy_susan = object_model.get_part("lazy_susan")
    
    # Check that they are not overlapping and touch correctly
    ctx.expect_contact(lazy_susan, table, name="lazy_susan_rests_on_table_bearing")
    
    # Check that lazy susan is within the tabletop bounds in XY
    ctx.expect_within(lazy_susan, table, axes="xy", margin=0.0, name="lazy_susan_within_table_bounds")

    # Check rotation
    joint = object_model.get_articulation("table_to_lazy_susan")
    with ctx.pose({joint: 1.57}):
        ctx.expect_contact(lazy_susan, table, name="contact_maintained_during_rotation")
        ctx.expect_within(lazy_susan, table, axes="xy", margin=0.0, name="within_bounds_during_rotation")
        
    return ctx.report()

object_model = build_object_model()
