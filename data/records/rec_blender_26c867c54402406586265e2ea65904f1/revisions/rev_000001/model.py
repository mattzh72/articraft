import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Box,
    Cylinder,
    place_on_face,
    proud_for_flush_mount,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blender")

    # 1. Base
    base = model.part("base")

    base_cq = cq.Workplane("XY").rect(0.16, 0.16).extrude(0.18).edges("|Z").fillet(0.03)
    
    # Base collar for jar
    collar = cq.Workplane("XY").workplane(offset=0.18).circle(0.06).extrude(0.02)
    base_cq = base_cq.union(collar)
    
    # Hole in collar
    collar_hole = cq.Workplane("XY").workplane(offset=0.18).circle(0.05).extrude(0.02)
    base_cq = base_cq.cut(collar_hole)

    base.visual(
        mesh_from_cadquery(base_cq, "base_mesh"),
        origin=Origin(xyz=(0, 0, 0)),
        name="motor_base",
        color=(0.15, 0.15, 0.15),
    )

    # Feet
    for dx in [-0.06, 0.06]:
        for dy in [-0.06, 0.06]:
            base.visual(
                Cylinder(radius=0.015, length=0.01),
                origin=Origin(xyz=(dx, dy, -0.005)),
                name=f"foot_{dx}_{dy}",
                color=(0.05, 0.05, 0.05),
            )

    # 2. Controls
    dial = model.part("dial")
    dial_cq = cq.Workplane("XY").circle(0.025).extrude(0.015).edges(">Z").fillet(0.005)
    indicator = cq.Workplane("XY").rect(0.005, 0.04).extrude(0.016)
    dial_cq = dial_cq.union(indicator)

    dial.visual(
        mesh_from_cadquery(dial_cq, "dial_mesh"),
        origin=Origin(),
        name="dial_knob",
        color=(0.8, 0.8, 0.8),
    )

    dial_origin = place_on_face(
        base,
        "-y",
        face_pos=(0, 0.12),
        proud=-0.001
    )

    model.articulation(
        "base_to_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial,
        origin=dial_origin,
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-2.0, upper=2.0),
    )

    for i, dx in enumerate([-0.03, 0, 0.03]):
        btn = model.part(f"button_{i}")
        btn.visual(
            Box((0.02, 0.01, 0.01)),
            origin=Origin(),
            name=f"btn_vis_{i}",
            color=(0.8, 0.8, 0.8),
        )
        btn_origin = place_on_face(
            base,
            "-y",
            face_pos=(dx, 0.06),
            proud=0.004
        )
        model.articulation(
            f"base_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=btn,
            origin=btn_origin,
            axis=(0, 0, -1),
            motion_limits=MotionLimits(lower=0, upper=0.005),
        )

    # 3. Jar
    jar = model.part("jar")

    jar_body = cq.Workplane("XY").circle(0.055).extrude(0.24, taper=-2)
    jar_body = jar_body.faces(">Z").shell(-0.004)

    handle_outer = cq.Workplane("XZ").center(0.07, 0.12).rect(0.06, 0.16).extrude(0.015, both=True).edges().fillet(0.005)
    handle_inner = cq.Workplane("XZ").center(0.08, 0.12).rect(0.03, 0.12).extrude(0.02, both=True)
    handle_cq = handle_outer.cut(handle_inner)

    jar_cq = jar_body.union(handle_cq)

    jar_base = cq.Workplane("XY").workplane(offset=-0.02).circle(0.048).extrude(0.02)
    jar_cq = jar_cq.union(jar_base)

    jar.visual(
        mesh_from_cadquery(jar_cq, "jar_mesh"),
        origin=Origin(xyz=(0, 0, 0)),
        name="pitcher",
        color=(0.8, 0.9, 1.0, 0.4),
    )

    model.articulation(
        "base_to_jar",
        ArticulationType.PRISMATIC,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0, 0, 0.20)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0, upper=0.30),
    )

    # 4. Blade
    blade = model.part("blade")
    blade_cq = cq.Workplane("XY").rect(0.07, 0.015).extrude(0.002)
    blade_cq = blade_cq.union(cq.Workplane("XY").rect(0.015, 0.07).extrude(0.002))
    blade_stem = cq.Workplane("XY").workplane(offset=-0.02).circle(0.006).extrude(0.022)
    blade_cq = blade_cq.union(blade_stem)

    blade.visual(
        mesh_from_cadquery(blade_cq, "blade_mesh"),
        origin=Origin(xyz=(0, 0, 0)),
        name="blade_metal",
        color=(0.7, 0.7, 0.7),
    )

    model.articulation(
        "jar_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade,
        origin=Origin(xyz=(0, 0, 0.005)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    # 5. Lid
    lid = model.part("lid")

    lid_cq = cq.Workplane("XY").circle(0.065).extrude(0.01)
    lid_plug = (
        cq.Workplane("XY")
        .workplane(offset=-0.015)
        .circle(0.05)
        .extrude(0.015)
    )
    lid_cq = lid_cq.union(lid_plug)

    cap = cq.Workplane("XY").workplane(offset=0.01).circle(0.03).extrude(0.01)
    lid_cq = lid_cq.union(cap)

    lid.visual(
        mesh_from_cadquery(lid_cq, "lid_mesh"),
        origin=Origin(xyz=(0, 0, 0)),
        name="lid_cap",
        color=(0.1, 0.1, 0.1),
    )

    model.articulation(
        "jar_to_lid",
        ArticulationType.PRISMATIC,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0, 0, 0.24)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0, upper=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    lid = object_model.get_part("lid")
    blade = object_model.get_part("blade")

    ctx.allow_overlap(jar, base, reason="Jar base fits inside the base collar.")
    ctx.allow_overlap(jar, lid, reason="Lid plug fits inside the jar opening.")
    ctx.allow_overlap(jar, blade, reason="Blade stem connects through the jar bottom.")

    with ctx.pose(base_to_jar=0.05):
        ctx.expect_gap(jar, base, axis="z", min_gap=0.02)

    with ctx.pose(jar_to_lid=0.05):
        ctx.expect_gap(lid, jar, axis="z", min_gap=0.02)

    return ctx.report()


object_model = build_object_model()