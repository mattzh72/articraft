from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cylinder_along_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _cylinder_along_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vacuum_articulated_wand")

    plastic_red = Material("red_plastic", rgba=(0.72, 0.04, 0.035, 1.0))
    dark = Material("dark_graphite", rgba=(0.035, 0.038, 0.043, 1.0))
    rubber = Material("matte_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    metal = Material("brushed_aluminum", rgba=(0.72, 0.76, 0.77, 1.0))
    trim = Material("light_grey_trim", rgba=(0.78, 0.80, 0.80, 1.0))
    bristle = Material("bristle_strip", rgba=(0.018, 0.018, 0.016, 1.0))

    # Root: compact canister body with two fixed side cheeks around the first
    # wand hinge.  The cheeks are deliberately separate visible plates, but
    # each overlaps the molded shell so the root reads as one supported frame.
    body = model.part("body")
    body_shell = cq.Workplane("XY").box(0.62, 0.29, 0.26).edges("|Z").fillet(0.055)
    body.visual(
        mesh_from_cadquery(body_shell, "rounded_vacuum_body"),
        origin=Origin(xyz=(-0.03, 0.0, 0.20)),
        material=plastic_red,
        name="main_shell",
    )
    body.visual(
        Box((0.22, 0.18, 0.045)),
        origin=Origin(xyz=(-0.03, 0.0, 0.355)),
        material=dark,
        name="top_grip",
    )
    body.visual(
        Box((0.040, 0.055, 0.090)),
        origin=Origin(xyz=(-0.145, 0.0, 0.315)),
        material=dark,
        name="rear_grip_post",
    )
    body.visual(
        Box((0.040, 0.055, 0.090)),
        origin=Origin(xyz=(0.085, 0.0, 0.315)),
        material=dark,
        name="front_grip_post",
    )
    body.visual(
        Box((0.16, 0.035, 0.16)),
        origin=Origin(xyz=(0.32, 0.105, 0.35)),
        material=trim,
        name="cheek_positive",
    )
    body.visual(
        Box((0.16, 0.035, 0.16)),
        origin=Origin(xyz=(0.32, -0.105, 0.35)),
        material=trim,
        name="cheek_negative",
    )
    port_geom, port_rot = _cylinder_along_x(0.058, 0.055)
    body.visual(
        port_geom,
        origin=Origin(xyz=(0.285, 0.0, 0.245), rpy=port_rot.rpy),
        material=dark,
        name="front_suction_port",
    )
    axle_geom, axle_rot = _cylinder_along_y(0.014, 0.36)
    body.visual(
        axle_geom,
        origin=Origin(xyz=(-0.17, 0.0, 0.090), rpy=axle_rot.rpy),
        material=dark,
        name="rear_axle",
    )
    wheel_geom, wheel_rot = _cylinder_along_y(0.090, 0.055)
    body.visual(
        wheel_geom,
        origin=Origin(xyz=(-0.17, 0.170, 0.090), rpy=wheel_rot.rpy),
        material=rubber,
        name="wheel_0",
    )
    body.visual(
        wheel_geom,
        origin=Origin(xyz=(-0.17, -0.170, 0.090), rpy=wheel_rot.rpy),
        material=rubber,
        name="wheel_1",
    )

    wand_0 = model.part("wand_0")
    hub_geom, hub_rot = _cylinder_along_y(0.052, 0.175)
    wand_0.visual(
        hub_geom,
        origin=Origin(rpy=hub_rot.rpy),
        material=dark,
        name="rear_hinge_barrel",
    )
    tube_geom, tube_rot = _cylinder_along_x(0.024, 0.470)
    wand_0.visual(
        tube_geom,
        origin=Origin(xyz=(0.255, 0.0, 0.0), rpy=tube_rot.rpy),
        material=metal,
        name="tube",
    )
    wand_0.visual(
        Box((0.13, 0.13, 0.045)),
        origin=Origin(xyz=(0.475, 0.0, 0.0)),
        material=dark,
        name="front_yoke_bridge",
    )
    wand_0.visual(
        Box((0.12, 0.025, 0.090)),
        origin=Origin(xyz=(0.57, 0.067, 0.0)),
        material=dark,
        name="front_yoke_positive",
    )
    wand_0.visual(
        Box((0.12, 0.025, 0.090)),
        origin=Origin(xyz=(0.57, -0.067, 0.0)),
        material=dark,
        name="front_yoke_negative",
    )

    wand_1 = model.part("wand_1")
    mid_hub_geom, mid_hub_rot = _cylinder_along_y(0.038, 0.109)
    wand_1.visual(
        mid_hub_geom,
        origin=Origin(rpy=mid_hub_rot.rpy),
        material=dark,
        name="rear_hinge_barrel",
    )
    tube_1_geom, tube_1_rot = _cylinder_along_x(0.022, 0.470)
    wand_1.visual(
        tube_1_geom,
        origin=Origin(xyz=(0.255, 0.0, 0.0), rpy=tube_1_rot.rpy),
        material=metal,
        name="tube",
    )
    wand_1.visual(
        Box((0.13, 0.13, 0.045)),
        origin=Origin(xyz=(0.475, 0.0, 0.0)),
        material=dark,
        name="front_yoke_bridge",
    )
    wand_1.visual(
        Box((0.12, 0.025, 0.090)),
        origin=Origin(xyz=(0.57, 0.067, 0.0)),
        material=dark,
        name="front_yoke_positive",
    )
    wand_1.visual(
        Box((0.12, 0.025, 0.090)),
        origin=Origin(xyz=(0.57, -0.067, 0.0)),
        material=dark,
        name="front_yoke_negative",
    )

    nozzle = model.part("nozzle")
    nozzle_hub_geom, nozzle_hub_rot = _cylinder_along_y(0.040, 0.109)
    nozzle.visual(
        nozzle_hub_geom,
        origin=Origin(rpy=nozzle_hub_rot.rpy),
        material=dark,
        name="pitch_barrel",
    )
    nozzle.visual(
        Box((0.115, 0.105, 0.085)),
        origin=Origin(xyz=(0.050, 0.0, -0.045)),
        material=dark,
        name="neck",
    )
    nozzle.visual(
        Box((0.300, 0.620, 0.055)),
        origin=Origin(xyz=(0.170, 0.0, -0.098)),
        material=dark,
        name="floor_head",
    )
    nozzle.visual(
        Box((0.040, 0.610, 0.020)),
        origin=Origin(xyz=(0.305, 0.0, -0.135)),
        material=bristle,
        name="front_bristles",
    )
    nozzle.visual(
        Box((0.040, 0.500, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, -0.128)),
        material=rubber,
        name="rear_squeegee",
    )
    nozzle.visual(
        Box((0.250, 0.035, 0.018)),
        origin=Origin(xyz=(0.150, 0.327, -0.098)),
        material=trim,
        name="side_bumper_0",
    )
    nozzle.visual(
        Box((0.250, 0.035, 0.018)),
        origin=Origin(xyz=(0.150, -0.327, -0.098)),
        material=trim,
        name="side_bumper_1",
    )

    downward_pitch = 0.20
    link_len = 0.58
    model.articulation(
        "body_to_wand_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand_0,
        origin=Origin(xyz=(0.34, 0.0, 0.35), rpy=(0.0, downward_pitch, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.55, upper=0.70),
    )
    model.articulation(
        "wand_0_to_wand_1",
        ArticulationType.REVOLUTE,
        parent=wand_0,
        child=wand_1,
        origin=Origin(xyz=(link_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "wand_1_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=wand_1,
        child=nozzle,
        # The nozzle hinge compensates for the wand's resting downward slope so
        # the floor head sits level at q=0, then pitches about its horizontal pin.
        origin=Origin(xyz=(link_len, 0.0, 0.0), rpy=(0.0, -downward_pitch, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand_0 = object_model.get_part("wand_0")
    wand_1 = object_model.get_part("wand_1")
    nozzle = object_model.get_part("nozzle")
    first_elbow = object_model.get_articulation("body_to_wand_0")
    mid_elbow = object_model.get_articulation("wand_0_to_wand_1")
    nozzle_pitch = object_model.get_articulation("wand_1_to_nozzle")

    ctx.expect_gap(
        body,
        wand_0,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="cheek_positive",
        negative_elem="rear_hinge_barrel",
        name="positive root cheek clears first barrel",
    )
    ctx.expect_gap(
        wand_0,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="rear_hinge_barrel",
        negative_elem="cheek_negative",
        name="negative root cheek clears first barrel",
    )
    ctx.expect_overlap(
        wand_0,
        body,
        axes="xz",
        min_overlap=0.060,
        elem_a="rear_hinge_barrel",
        elem_b="cheek_positive",
        name="first barrel is framed by root cheek",
    )
    ctx.expect_overlap(
        wand_1,
        wand_0,
        axes="xz",
        min_overlap=0.050,
        elem_a="rear_hinge_barrel",
        elem_b="front_yoke_positive",
        name="middle barrel sits inside wand yoke",
    )
    ctx.expect_overlap(
        nozzle,
        wand_1,
        axes="xz",
        min_overlap=0.050,
        elem_a="pitch_barrel",
        elem_b="front_yoke_positive",
        name="nozzle pitch barrel sits inside wand yoke",
    )

    rest_nozzle_aabb = ctx.part_world_aabb(nozzle)
    with ctx.pose({mid_elbow: 0.45}):
        bent_nozzle_aabb = ctx.part_world_aabb(nozzle)
    ctx.check(
        "middle elbow visibly bends wand chain",
        rest_nozzle_aabb is not None
        and bent_nozzle_aabb is not None
        and abs(bent_nozzle_aabb[0][2] - rest_nozzle_aabb[0][2]) > 0.045,
        details=f"rest={rest_nozzle_aabb}, bent={bent_nozzle_aabb}",
    )

    with ctx.pose({nozzle_pitch: 0.35}):
        pitched_nozzle_aabb = ctx.part_world_aabb(nozzle)
    ctx.check(
        "floor nozzle pitches on its hinge",
        rest_nozzle_aabb is not None
        and pitched_nozzle_aabb is not None
        and abs(pitched_nozzle_aabb[0][2] - rest_nozzle_aabb[0][2]) > 0.025,
        details=f"rest={rest_nozzle_aabb}, pitched={pitched_nozzle_aabb}",
    )

    with ctx.pose({first_elbow: -0.35, mid_elbow: 0.35, nozzle_pitch: 0.20}):
        ctx.expect_overlap(
            nozzle,
            wand_1,
            axes="xz",
            min_overlap=0.040,
            elem_a="pitch_barrel",
            elem_b="front_yoke_positive",
            name="pitched nozzle remains in yoke",
        )

    return ctx.report()


object_model = build_object_model()
