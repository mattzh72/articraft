from __future__ import annotations

import math

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
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_contractor_toolbox")

    safety_red = model.material("powder_coated_red", rgba=(0.78, 0.05, 0.03, 1.0))
    dark_lid = model.material("matte_black_lid", rgba=(0.02, 0.022, 0.024, 1.0))
    black = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    dark_metal = model.material("dark_anodized_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    bright_metal = model.material("zinc_plated_steel", rgba=(0.64, 0.66, 0.62, 1.0))

    def hollow_tube(outer_radius: float, inner_radius: float, length: float):
        return (
            cq.Workplane("XY")
            .circle(outer_radius)
            .circle(inner_radius)
            .extrude(length)
            .translate((0.0, 0.0, -length / 2.0))
        )

    guide_mesh = mesh_from_cadquery(hollow_tube(0.022, 0.015, 0.36), "recessed_guide_tube")
    stage_mesh = mesh_from_cadquery(hollow_tube(0.015, 0.0074, 0.48), "outer_handle_tube")

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.110,
            0.060,
            inner_radius=0.074,
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "toolbox_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.076,
            0.050,
            rim=WheelRim(inner_radius=0.052, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(radius=0.026, width=0.044, cap_style="flat"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.030),
        ),
        "toolbox_wheel_rim",
    )

    body = model.part("body")
    body.visual(Box((0.72, 0.42, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.120)), material=safety_red, name="body_floor")
    body.visual(Box((0.72, 0.035, 0.420)), origin=Origin(xyz=(0.0, -0.1925, 0.350)), material=safety_red, name="front_wall")
    body.visual(Box((0.72, 0.035, 0.420)), origin=Origin(xyz=(0.0, 0.1925, 0.350)), material=safety_red, name="rear_wall")
    body.visual(Box((0.035, 0.42, 0.420)), origin=Origin(xyz=(-0.3425, 0.0, 0.350)), material=safety_red, name="side_wall_0")
    body.visual(Box((0.035, 0.42, 0.420)), origin=Origin(xyz=(0.3425, 0.0, 0.350)), material=safety_red, name="side_wall_1")
    body.visual(Box((0.70, 0.035, 0.035)), origin=Origin(xyz=(0.0, -0.1925, 0.5425)), material=dark_metal, name="front_lip")
    body.visual(Box((0.70, 0.035, 0.035)), origin=Origin(xyz=(0.0, 0.1925, 0.5425)), material=dark_metal, name="rear_lip")
    body.visual(Box((0.035, 0.40, 0.035)), origin=Origin(xyz=(-0.3425, 0.0, 0.5425)), material=dark_metal, name="side_lip_0")
    body.visual(Box((0.035, 0.40, 0.035)), origin=Origin(xyz=(0.3425, 0.0, 0.5425)), material=dark_metal, name="side_lip_1")
    body.visual(Box((0.18, 0.018, 0.060)), origin=Origin(xyz=(0.0, -0.215, 0.445)), material=dark_metal, name="front_latch_strike")

    axle_rpy = (0.0, math.pi / 2.0, 0.0)
    body.visual(Cylinder(radius=0.015, length=0.88), origin=Origin(xyz=(0.0, 0.205, 0.115), rpy=axle_rpy), material=bright_metal, name="rear_axle")
    for x in (-0.350, 0.350):
        body.visual(Box((0.040, 0.045, 0.080)), origin=Origin(xyz=(x, 0.205, 0.145)), material=dark_metal, name=f"axle_hanger_{0 if x < 0 else 1}")

    for x in (-0.235, 0.235):
        tube_name = "guide_tube_0" if x < 0 else "guide_tube_1"
        body.visual(guide_mesh, origin=Origin(xyz=(x, 0.265, 0.715)), material=dark_metal, name=tube_name)
        for z in (0.530, 0.840):
            for side in (-1.0, 1.0):
                body.visual(
                    Box((0.014, 0.075, 0.035)),
                    origin=Origin(xyz=(x + side * 0.024, 0.237, z)),
                    material=dark_metal,
                    name=f"guide_tab_{0 if x < 0 else 1}_{0 if z < 0.7 else 1}_{0 if side < 0 else 1}",
                )

    for x, length in ((-0.280, 0.140), (0.000, 0.120), (0.280, 0.140)):
        body.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(x, 0.230, 0.560), rpy=axle_rpy),
            material=bright_metal,
            name=f"body_hinge_{x:+.2f}",
        )
        body.visual(
            Box((length, 0.040, 0.012)),
            origin=Origin(xyz=(x, 0.213, 0.552)),
            material=bright_metal,
            name=f"body_hinge_leaf_{x:+.2f}",
        )

    lid = model.part("lid")
    lid.visual(Box((0.760, 0.410, 0.040)), origin=Origin(xyz=(0.0, -0.225, 0.020)), material=dark_lid, name="lid_panel")
    lid.visual(Box((0.500, 0.025, 0.020)), origin=Origin(xyz=(0.0, -0.225, 0.049)), material=dark_metal, name="recessed_grip")
    lid.visual(Box((0.160, 0.018, 0.030)), origin=Origin(xyz=(0.0, -0.410, 0.054)), material=dark_metal, name="front_latch")
    for x, length in ((-0.135, 0.130), (0.135, 0.130)):
        lid.visual(
            Cylinder(radius=0.017, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=axle_rpy),
            material=bright_metal,
            name=f"lid_hinge_{0 if x < 0 else 1}",
        )
        lid.visual(
            Box((length, 0.035, 0.012)),
            origin=Origin(xyz=(x, -0.015, 0.006)),
            material=bright_metal,
            name=f"lid_hinge_leaf_{0 if x < 0 else 1}",
        )

    stage_0 = model.part("handle_stage_0")
    for x in (-0.235, 0.235):
        stage_0.visual(stage_mesh, origin=Origin(xyz=(x, 0.0, -0.180)), material=dark_metal, name=f"stage0_leg_{0 if x < 0 else 1}")
    stage_0.visual(Box((0.500, 0.012, 0.024)), origin=Origin(xyz=(0.0, -0.014, 0.055)), material=dark_metal, name="stage0_bridge")

    stage_1 = model.part("handle_stage_1")
    for x in (-0.235, 0.235):
        stage_1.visual(Cylinder(radius=0.0074, length=0.420), origin=Origin(xyz=(x, 0.0, -0.180)), material=bright_metal, name=f"stage1_leg_{0 if x < 0 else 1}")
    stage_1.visual(Box((0.560, 0.035, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.038)), material=black, name="pull_grip")

    wheel_positions = [(-0.405, 0.205, 0.115), (0.405, 0.205, 0.115)]
    for i, xyz in enumerate(wheel_positions):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(wheel_mesh, material=bright_metal, name="rim")
        model.articulation(
            f"body_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=20.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.230, 0.560)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stage_0,
        origin=Origin(xyz=(0.0, 0.265, 0.880)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.30, lower=0.0, upper=0.24),
    )
    model.articulation(
        "stage_0_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.25, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    stage_0 = object_model.get_part("handle_stage_0")
    stage_1 = object_model.get_part("handle_stage_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    stage0_slide = object_model.get_articulation("body_to_stage_0")
    stage1_slide = object_model.get_articulation("stage_0_to_stage_1")

    for i in (0, 1):
        ctx.allow_overlap(
            body,
            stage_0,
            elem_a=f"guide_tube_{i}",
            elem_b=f"stage0_leg_{i}",
            reason="The solid guide-tube proxy intentionally captures the outer telescoping handle leg.",
        )
        ctx.allow_overlap(
            stage_0,
            stage_1,
            elem_a=f"stage0_leg_{i}",
            elem_b=f"stage1_leg_{i}",
            reason="The inner telescoping leg is intentionally represented sliding inside the hollow outer-stage proxy.",
        )
        ctx.allow_overlap(
            body,
            f"wheel_{i}",
            elem_a="rear_axle",
            elem_b="rim",
            reason="The fixed axle is intentionally captured through the wheel hub bore.",
        )

    wheel_joints = [object_model.get_articulation("body_to_wheel_0"), object_model.get_articulation("body_to_wheel_1")]
    ctx.check(
        "rear wheels use continuous axle joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS and tuple(j.axis) == (1.0, 0.0, 0.0) for j in wheel_joints),
        details="Both rear wheels should spin continuously around the fixed horizontal axle.",
    )

    ctx.expect_gap(lid, body, axis="z", max_gap=0.002, max_penetration=0.0, positive_elem="lid_panel", negative_elem="front_lip", name="closed lid seats on top lip")
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.34, elem_a="lid_panel", elem_b="body_floor", name="lid covers the deep body footprint")

    for i in (0, 1):
        ctx.expect_within(stage_0, body, axes="xy", inner_elem=f"stage0_leg_{i}", outer_elem=f"guide_tube_{i}", margin=0.004, name=f"outer handle leg {i} centered in guide tube")
        ctx.expect_overlap(stage_0, body, axes="z", min_overlap=0.15, elem_a=f"stage0_leg_{i}", elem_b=f"guide_tube_{i}", name=f"outer handle leg {i} retained in guide tube")
        ctx.expect_within(stage_1, stage_0, axes="xy", inner_elem=f"stage1_leg_{i}", outer_elem=f"stage0_leg_{i}", margin=0.002, name=f"inner handle leg {i} centered in outer stage")
        ctx.expect_overlap(stage_1, stage_0, axes="z", min_overlap=0.25, elem_a=f"stage1_leg_{i}", elem_b=f"stage0_leg_{i}", name=f"inner handle leg {i} retained in outer stage")
        ctx.expect_within(body, f"wheel_{i}", axes="yz", inner_elem="rear_axle", outer_elem="rim", margin=0.0, name=f"axle {i} passes through wheel hub")
        ctx.expect_overlap(f"wheel_{i}", body, axes="x", min_overlap=0.040, elem_a="rim", elem_b="rear_axle", name=f"wheel {i} retained on fixed axle")

    rest_stage0 = ctx.part_world_position(stage_0)
    rest_stage1 = ctx.part_world_position(stage_1)
    with ctx.pose({stage0_slide: 0.24, stage1_slide: 0.22}):
        extended_stage0 = ctx.part_world_position(stage_0)
        extended_stage1 = ctx.part_world_position(stage_1)
        for i in (0, 1):
            ctx.expect_overlap(stage_0, body, axes="z", min_overlap=0.06, elem_a=f"stage0_leg_{i}", elem_b=f"guide_tube_{i}", name=f"outer handle leg {i} remains captured when extended")
            ctx.expect_overlap(stage_1, stage_0, axes="z", min_overlap=0.12, elem_a=f"stage1_leg_{i}", elem_b=f"stage0_leg_{i}", name=f"inner handle leg {i} remains captured when extended")
    ctx.check(
        "two handle stages extend upward",
        rest_stage0 is not None
        and extended_stage0 is not None
        and rest_stage1 is not None
        and extended_stage1 is not None
        and extended_stage0[2] > rest_stage0[2] + 0.20
        and extended_stage1[2] > rest_stage1[2] + 0.42,
        details=f"stage0 rest={rest_stage0} extended={extended_stage0}; stage1 rest={rest_stage1} extended={extended_stage1}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    closed_top = closed_lid_aabb[1][2] if closed_lid_aabb is not None else None
    open_top = open_lid_aabb[1][2] if open_lid_aabb is not None else None
    ctx.check(
        "lid hinge opens upward",
        closed_top is not None and open_top is not None and open_top > closed_top + 0.20,
        details=f"closed top={closed_top}, open top={open_top}",
    )

    return ctx.report()


object_model = build_object_model()
