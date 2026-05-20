from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bell_tower")

    # --- Tower Geometry ---
    tower_base = cq.Workplane("XY").box(2.4, 2.4, 12, centered=(True, True, False))

    # Arch cutout
    tower_cut = (
        tower_base
        .faces(">Y")
        .workplane(centerOption="CenterOfBoundBox")
        .center(0, 3.5)
        .moveTo(0.8, -1.5)
        .lineTo(0.8, 0.7)
        .threePointArc((0, 1.5), (-0.8, 0.7))
        .lineTo(-0.8, -1.5)
        .close()
        .cutThruAll()
    )

    # Mounts
    mount1 = cq.Workplane("XY").workplane(offset=9.8).center(0.75, 0).box(0.1, 0.4, 0.4)
    mount2 = cq.Workplane("XY").workplane(offset=9.8).center(-0.75, 0).box(0.1, 0.4, 0.4)
    tower_with_mounts = tower_cut.union(mount1).union(mount2)

    # Roof
    roof_cq = (
        cq.Workplane("XY")
        .workplane(offset=12.0)
        .rect(2.6, 2.6)
        .workplane(offset=1.5)
        .rect(0.1, 0.1)
        .loft()
    )
    tower_with_roof = tower_with_mounts.union(roof_cq)

    # Windows
    tower_final = (
        tower_with_roof
        .faces(">Y").workplane(centerOption="CenterOfBoundBox").center(0, -2.0).rect(0.4, 1.0).cutThruAll()
        .faces(">X").workplane(centerOption="CenterOfBoundBox").center(0, -2.0).rect(0.4, 1.0).cutThruAll()
    )

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(tower_final, "tower_body"),
        name="body",
        material=Material(name="concrete", color=(0.6, 0.6, 0.6))
    )

    # --- Bell and Yoke Geometry ---
    # Yoke
    yoke_beam = cq.Workplane("XY").box(1.38, 0.3, 0.2)
    yoke_sides = (
        cq.Workplane("XY")
        .workplane(offset=-0.3)
        .center(0.59, 0)
        .box(0.2, 0.3, 0.4, centered=(True, True, False))
        .center(-1.18, 0)
        .box(0.2, 0.3, 0.4, centered=(True, True, False))
    )
    yoke_center = cq.Workplane("XY").workplane(offset=-0.2).box(0.4, 0.3, 0.1, centered=(True, True, False))
    yoke_shaft = cq.Workplane("YZ").cylinder(1.6, 0.05)

    yoke_cq = yoke_beam.union(yoke_sides).union(yoke_center).union(yoke_shaft)

    # Bell
    bell_outer = (
        cq.Workplane("XY")
        .workplane(offset=-0.2).circle(0.2)
        .workplane(offset=-0.6).circle(0.5)
        .workplane(offset=-0.2).circle(0.65)
        .loft()
    )
    bell_inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.25).circle(0.15)
        .workplane(offset=-0.55).circle(0.45)
        .workplane(offset=-0.21).circle(0.61)
        .loft()
    )
    bell_hollow = bell_outer.cut(bell_inner)

    clapper_rod = cq.Workplane("XY").workplane(offset=-1.0).cylinder(0.8, 0.03, centered=(True, True, False))
    clapper_ball = cq.Workplane("XY").workplane(offset=-0.95).sphere(0.08)
    bell_final = bell_hollow.union(clapper_rod).union(clapper_ball)

    bell_yoke = model.part("bell_yoke")
    # Yoke visual
    bell_yoke.visual(
        mesh_from_cadquery(yoke_cq, "yoke_body"),
        name="yoke",
        material=Material(name="steel", color=(0.2, 0.2, 0.2))
    )
    # Bell visual
    bell_yoke.visual(
        mesh_from_cadquery(bell_final, "bell_body"),
        name="bell",
        material=Material(name="bronze", color=(0.8, 0.6, 0.2))
    )

    # --- Articulation ---
    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell_yoke,
        origin=Origin(xyz=(0, 0, 9.8)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=5.0, lower=-1.5, upper=1.5)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    bell_yoke = object_model.get_part("bell_yoke")

    ctx.allow_overlap(bell_yoke, tower, elem_a="yoke", elem_b="body", reason="Yoke shaft is captured in tower mounts.")

    # Check bell is centered in X
    ctx.expect_within(bell_yoke, tower, axes="x", inner_elem="yoke", outer_elem="body", margin=0.0)

    # Check bell swings along Y
    rest_aabb = ctx.part_world_aabb(bell_yoke)

    swing_joint = object_model.get_articulation("bell_swing")
    with ctx.pose({swing_joint: 1.0}):
        swung_aabb = ctx.part_world_aabb(bell_yoke)

    if rest_aabb and swung_aabb:
        rest_y_span = rest_aabb[1][1] - rest_aabb[0][1]
        swung_y_span = swung_aabb[1][1] - swung_aabb[0][1]
        ctx.check("bell swings along Y axis", swung_y_span > rest_y_span + 0.1)

    return ctx.report()

object_model = build_object_model()