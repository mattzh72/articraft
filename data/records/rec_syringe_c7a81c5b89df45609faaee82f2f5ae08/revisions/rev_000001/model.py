from __future__ import annotations

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
    model = ArticulatedObject(name="hand_syringe")

    # Dimensions
    barrel_outer_radius = 0.01
    barrel_inner_radius = 0.009
    barrel_length = 0.08
    flange_radius = 0.015
    flange_thickness = 0.002
    nozzle_length = 0.015
    nozzle_tip_radius_out = 0.003
    nozzle_tip_radius_in = 0.0015
    nozzle_tip_length = 0.01

    thumb_rest_radius = 0.015
    thumb_rest_thickness = 0.002
    rod_length = 0.078
    rod_width = 0.012
    rod_thickness = 0.002
    head_radius = 0.0088
    head_length = 0.005

    # Build barrel
    barrel_outer = (
        cq.Workplane("XY")
        .cylinder(flange_thickness, flange_radius, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .cylinder(barrel_length - flange_thickness, barrel_outer_radius, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .circle(barrel_outer_radius)
        .workplane(offset=nozzle_length)
        .circle(nozzle_tip_radius_out)
        .loft(combine=True)
        .faces(">Z")
        .workplane()
        .cylinder(nozzle_tip_length, nozzle_tip_radius_out, centered=(True, True, False))
    )

    barrel_inner = (
        cq.Workplane("XY")
        .cylinder(barrel_length, barrel_inner_radius, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .circle(barrel_inner_radius)
        .workplane(offset=nozzle_length)
        .circle(nozzle_tip_radius_in)
        .loft(combine=True)
        .faces(">Z")
        .workplane()
        .cylinder(nozzle_tip_length, nozzle_tip_radius_in, centered=(True, True, False))
    )

    barrel_cq = barrel_outer.cut(barrel_inner)

    # Build plunger
    rod = (
        cq.Workplane("XY")
        .rect(rod_width, rod_thickness)
        .extrude(rod_length)
        .union(cq.Workplane("XY").rect(rod_thickness, rod_width).extrude(rod_length))
    )

    plunger_cq = (
        cq.Workplane("XY")
        .cylinder(thumb_rest_thickness, thumb_rest_radius, centered=(True, True, False))
        .union(rod.translate((0, 0, thumb_rest_thickness)))
        .union(
            cq.Workplane("XY", origin=(0, 0, thumb_rest_thickness + rod_length)).cylinder(
                head_length, head_radius, centered=(True, True, False)
            )
        )
    )

    # Add to model
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(barrel_cq, "barrel_mesh"),
        name="barrel_visual",
        material=Material(name="barrel_mat", rgba=(0.8, 0.8, 0.9, 0.5)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        mesh_from_cadquery(plunger_cq, "plunger_mesh"),
        name="plunger_visual",
        material=Material(name="plunger_mat", rgba=(0.1, 0.1, 0.1, 1.0)),
    )

    # Articulation
    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.07, effort=10.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")

    ctx.allow_isolated_part(plunger, reason="Plunger slides inside the barrel with clearance")

    ctx.expect_within(
        plunger,
        barrel,
        axes="xy",
        margin=0.001,
        name="plunger stays centered inside barrel",
    )

    with ctx.pose(plunger_slide=0.07):
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            margin=0.001,
            name="extended plunger stays centered inside barrel",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            min_overlap=0.005,
            name="extended plunger remains inserted in the barrel",
        )

    return ctx.report()


object_model = build_object_model()