from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bicycle_fork_assembly")

    mat_frame = Material(name="frame", color=(0.2, 0.2, 0.2))
    mat_fork = Material(name="fork", color=(0.1, 0.1, 0.1))
    mat_stem = Material(name="stem", color=(0.05, 0.05, 0.05))
    mat_bar = Material(name="bar_tape", color=(0.1, 0.1, 0.1))
    mat_hood = Material(name="hood", color=(0.02, 0.02, 0.02))
    mat_lever = Material(name="lever", color=(0.8, 0.8, 0.8))

    # 1. Head tube (Root)
    head_tube = model.part("head_tube")
    ht_geom = (
        cq.Workplane("XY")
        .circle(0.025)
        .circle(0.02)
        .extrude(0.15)
    )
    head_tube.visual(
        mesh_from_cadquery(ht_geom, "head_tube_mesh"),
        material=mat_frame,
        name="head_tube_mesh"
    )

    # 2. Fork Assembly
    fork = model.part("fork")
    
    # Steerer tube (hollow)
    steerer_geom = (
        cq.Workplane("XY").workplane(offset=-0.01)
        .circle(0.0205)
        .circle(0.015)
        .extrude(0.26)
    )
    fork.visual(
        mesh_from_cadquery(steerer_geom, "steerer"),
        material=mat_fork,
        name="steerer"
    )
    
    # Crown
    crown_geom = (
        cq.Workplane("XY").transformed(offset=(0, 0, -0.02))
        .box(0.04, 0.09, 0.04)
        .edges("|Z").fillet(0.01)
    )
    fork.visual(
        mesh_from_cadquery(crown_geom, "crown"),
        material=mat_fork,
        name="crown"
    )
    
    # Left blade
    blade_l_geom = (
        cq.Workplane("XY").transformed(offset=(0.0, 0.035, -0.04))
        .circle(0.012)
        .workplane(offset=-0.15).center(0.01, 0)
        .circle(0.009)
        .workplane(offset=-0.20).center(0.02, 0)
        .circle(0.005)
        .loft()
    )
    fork.visual(
        mesh_from_cadquery(blade_l_geom, "blade_l"),
        material=mat_fork,
        name="blade_l"
    )
    
    # Right blade
    blade_r_geom = (
        cq.Workplane("XY").transformed(offset=(0.0, -0.035, -0.04))
        .circle(0.012)
        .workplane(offset=-0.15).center(0.01, 0)
        .circle(0.009)
        .workplane(offset=-0.20).center(0.02, 0)
        .circle(0.005)
        .loft()
    )
    fork.visual(
        mesh_from_cadquery(blade_r_geom, "blade_r"),
        material=mat_fork,
        name="blade_r"
    )

    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=fork,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-1.5, upper=1.5, effort=10.0, velocity=5.0),
    )

    # 3. Stem
    stem = model.part("stem")
    stem_clamp = cq.Workplane("XY").circle(0.025).circle(0.0205).extrude(0.04)
    stem_ext = cq.Workplane("YZ").center(0, 0.02).workplane(offset=0.02).circle(0.015).extrude(0.06)
    stem_bar_clamp = cq.Workplane("XZ").center(0.08, 0.02).workplane(offset=-0.025).circle(0.02).circle(0.015).extrude(0.05)
    
    stem_geom = stem_clamp.union(stem_ext).union(stem_bar_clamp)
    stem.visual(
        mesh_from_cadquery(stem_geom, "stem_mesh"),
        material=mat_stem,
        name="stem_mesh"
    )
    
    model.articulation(
        "fork_to_stem",
        ArticulationType.FIXED,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0, 0, 0.20))
    )

    # 4. Handlebar
    handlebar = model.part("handlebar")
    
    right_points = [
        (0.0, 0.0, 0.0),
        (0.0, -0.10, 0.0),
        (0.05, -0.15, 0.0),
        (0.08, -0.18, -0.05),
        (0.08, -0.18, -0.10),
        (0.04, -0.18, -0.14),
        (-0.02, -0.18, -0.14),
    ]
    left_points = [(x, -y, z) for x, y, z in right_points[::-1]]
    bar_points = left_points[:-1] + right_points
    
    bar_geom = tube_from_spline_points(
        bar_points,
        radius=0.012,
        samples_per_segment=10,
        cap_ends=True,
    )
    handlebar.visual(
        mesh_from_geometry(bar_geom, "bar_mesh"),
        material=mat_bar,
        name="bar_mesh"
    )
    
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.FIXED,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.08, 0, 0.02))
    )

    # 5. Brake Levers
    def add_brake(side, y_sign):
        hood = model.part(f"{side}_hood")
        lever = model.part(f"{side}_lever")
        
        hood_geom = cq.Workplane("XY").box(0.06, 0.03, 0.04).edges("|Y").fillet(0.01)
        hood.visual(
            mesh_from_cadquery(hood_geom, f"{side}_hood_mesh"),
            material=mat_hood,
            name="hood_mesh"
        )
        
        lever_geom = cq.Workplane("XY").box(0.015, 0.015, 0.08).edges("|Y").fillet(0.005)
        lever.visual(
            mesh_from_cadquery(lever_geom, f"{side}_lever_mesh"),
            origin=Origin(xyz=(0.0, 0.0, -0.04)),
            material=mat_lever,
            name="lever_mesh"
        )
        
        model.articulation(
            f"bar_to_{side}_hood",
            ArticulationType.FIXED,
            parent=handlebar,
            child=hood,
            origin=Origin(xyz=(0.07, y_sign * 0.17, -0.02))
        )
        
        model.articulation(
            f"{side}_hood_to_lever",
            ArticulationType.REVOLUTE,
            parent=hood,
            child=lever,
            origin=Origin(xyz=(0.03, 0.0, 0.01)),
            axis=(0.0, y_sign * 1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.3, effort=5.0, velocity=2.0)
        )

    add_brake("left", 1)
    add_brake("right", -1)

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap(
        "head_tube", "fork",
        elem_a="head_tube_mesh",
        elem_b="steerer",
        reason="Steerer tube intentionally rotates inside the head tube."
    )
    ctx.allow_overlap(
        "fork", "stem",
        reason="Stem clamps onto the steerer tube."
    )
    ctx.allow_overlap(
        "stem", "handlebar",
        reason="Stem clamps onto the handlebar."
    )
    ctx.allow_overlap(
        "handlebar", "left_hood",
        reason="Brake hood wraps onto the handlebar."
    )
    ctx.allow_overlap(
        "handlebar", "right_hood",
        reason="Brake hood wraps onto the handlebar."
    )
    ctx.allow_overlap(
        "left_hood", "left_lever",
        reason="Brake lever pivots inside the hood."
    )
    ctx.allow_overlap(
        "right_hood", "right_lever",
        reason="Brake lever pivots inside the hood."
    )

    ht = object_model.get_part("head_tube")
    fk = object_model.get_part("fork")
    steering = object_model.get_articulation("steering")

    ctx.expect_within(fk, ht, axes="xy", inner_elem="steerer", outer_elem="head_tube_mesh", margin=0.001)
    
    with ctx.pose({steering: 1.0}):
        ctx.expect_within(fk, ht, axes="xy", inner_elem="steerer", outer_elem="head_tube_mesh", margin=0.001)

    return ctx.report()

object_model = build_object_model()
