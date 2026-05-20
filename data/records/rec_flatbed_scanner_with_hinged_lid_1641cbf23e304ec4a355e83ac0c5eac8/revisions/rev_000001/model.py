from __future__ import annotations

import math

import cadquery as cq

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_flatbed_scanner")

    matte_body = model.material("warm_matte_plastic", rgba=(0.76, 0.76, 0.72, 1.0))
    dark_trim = model.material("charcoal_trim", rgba=(0.03, 0.035, 0.04, 1.0))
    hinge_black = model.material("black_hinge_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    glass_blue = model.material("smoky_scanner_glass", rgba=(0.18, 0.32, 0.42, 0.46))
    lid_white = model.material("off_white_lid", rgba=(0.88, 0.88, 0.84, 1.0))
    guide_gray = model.material("paper_guide_gray", rgba=(0.48, 0.51, 0.52, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    body_w = 0.398
    body_d = 0.296
    body_h = 0.042
    hinge_y = 0.153
    hinge_z = 0.058
    cyl_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    body = model.part("body")
    body_shell = (
        cq.Workplane("XY")
        .box(body_w, body_d, body_h)
        .edges("|Z")
        .fillet(0.018)
    )
    body.visual(
        mesh_from_cadquery(body_shell, "scanner_body_shell", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
        material=matte_body,
        name="body_shell",
    )
    body.visual(
        Box((0.330, 0.210, 0.003)),
        origin=Origin(xyz=(0.0, -0.006, 0.0445)),
        material=glass_blue,
        name="glass_pane",
    )
    body.visual(
        Box((0.348, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.116, 0.0440)),
        material=dark_trim,
        name="front_bezel",
    )
    body.visual(
        Box((0.348, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.104, 0.0440)),
        material=dark_trim,
        name="rear_bezel",
    )
    for x, name in ((-0.174, "side_bezel_0"), (0.174, "side_bezel_1")):
        body.visual(
            Box((0.010, 0.220, 0.004)),
            origin=Origin(xyz=(x, -0.006, 0.0440)),
            material=dark_trim,
            name=name,
        )
    body.visual(
        Box((0.300, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.144, 0.0435)),
        material=dark_trim,
        name="front_grip_strip",
    )
    for x, name in ((-0.165, "lid_stop_pad_0"), (0.165, "lid_stop_pad_1")):
        body.visual(
            Box((0.032, 0.014, 0.008)),
            origin=Origin(xyz=(x, -0.124, 0.0460)),
            material=rubber,
            name=name,
        )
    body.visual(
        Box((0.365, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, 0.145, 0.0450)),
        material=dark_trim,
        name="rear_hinge_spine",
    )
    for i, x in enumerate((-0.125, 0.125)):
        body.visual(
            Box((0.024, 0.020, 0.014)),
            origin=Origin(xyz=(x, 0.145, 0.0510)),
            material=hinge_black,
            name=f"lid_hinge_saddle_{i}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.022),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=cyl_x.rpy),
            material=hinge_black,
            name=f"lid_hinge_base_barrel_{i}",
        )
    body.visual(
        Box((0.030, 0.010, 0.003)),
        origin=Origin(xyz=(-0.165, -0.153, 0.010)),
        material=rubber,
        name="front_foot_0",
    )
    body.visual(
        Box((0.030, 0.010, 0.003)),
        origin=Origin(xyz=(0.165, -0.153, 0.010)),
        material=rubber,
        name="front_foot_1",
    )

    lid = model.part("lid")
    lid_shell = (
        cq.Workplane("XY")
        .box(0.374, 0.262, 0.016)
        .edges("|Z")
        .fillet(0.014)
    )
    lid.visual(
        mesh_from_cadquery(lid_shell, "scanner_lid_shell", tolerance=0.001),
        origin=Origin(xyz=(0.0, -0.155, 0.0)),
        material=lid_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.330, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.276, 0.010)),
        material=dark_trim,
        name="front_lid_lip",
    )
    for i, base_x in enumerate((-0.125, 0.125)):
        for side, dx in enumerate((-0.020, 0.020)):
            x = base_x + dx
            lid.visual(
                Cylinder(radius=0.008, length=0.011),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=cyl_x.rpy),
                material=hinge_black,
                name=f"lid_hinge_leaf_barrel_{i}_{side}",
            )
            lid.visual(
                Box((0.011, 0.024, 0.006)),
                origin=Origin(xyz=(x, -0.012, -0.003)),
                material=hinge_black,
                name=f"lid_hinge_leaf_{i}_{side}",
            )

    guide_hinge_y = -0.070
    guide_hinge_z = 0.014
    for i, x in enumerate((-0.060, 0.060)):
        lid.visual(
            Box((0.026, 0.016, 0.006)),
            origin=Origin(xyz=(x, guide_hinge_y, 0.011)),
            material=hinge_black,
            name=f"guide_hinge_pedestal_{i}",
        )
        lid.visual(
            Cylinder(radius=0.0045, length=0.022),
            origin=Origin(xyz=(x, guide_hinge_y, guide_hinge_z), rpy=cyl_x.rpy),
            material=hinge_black,
            name=f"guide_hinge_fixed_barrel_{i}",
        )

    paper_guide = model.part("paper_guide")
    paper_guide.visual(
        Box((0.260, 0.126, 0.004)),
        origin=Origin(xyz=(0.0, -0.074, -0.004)),
        material=guide_gray,
        name="guide_plate",
    )
    paper_guide.visual(
        Box((0.260, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.140, 0.005)),
        material=guide_gray,
        name="paper_stop_lip",
    )
    for x, name in ((-0.124, "side_rail_0"), (0.124, "side_rail_1")):
        paper_guide.visual(
            Box((0.006, 0.112, 0.010)),
            origin=Origin(xyz=(x, -0.081, 0.003)),
            material=guide_gray,
            name=name,
        )
    for x, length, name in (
        (0.0, 0.070, "guide_hinge_barrel_center"),
        (-0.105, 0.030, "guide_hinge_barrel_0"),
        (0.105, 0.030, "guide_hinge_barrel_1"),
    ):
        paper_guide.visual(
            Cylinder(radius=0.0045, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=cyl_x.rpy),
            material=hinge_black,
            name=name,
        )
        paper_guide.visual(
            Box((length, 0.024, 0.005)),
            origin=Origin(xyz=(x, -0.012, -0.0025)),
            material=hinge_black,
            name=f"{name}_leaf",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "lid_to_paper_guide",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=paper_guide,
        origin=Origin(xyz=(0.0, guide_hinge_y, guide_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    paper_guide = object_model.get_part("paper_guide")
    lid_hinge = object_model.get_articulation("body_to_lid")
    guide_hinge = object_model.get_articulation("lid_to_paper_guide")

    ctx.check(
        "lid uses two compact rear hinge sets",
        len([v for v in body.visuals if v.name and v.name.startswith("lid_hinge_base_barrel")]) == 2
        and len([v for v in lid.visuals if v.name and v.name.startswith("lid_hinge_leaf_barrel")]) == 4,
        details="Expected two rear base knuckles and four interleaved lid knuckles.",
    )
    ctx.check(
        "paper guide has a secondary hinge",
        guide_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"paper guide joint type is {guide_hinge.articulation_type}",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="glass_pane",
        min_overlap=0.18,
        name="closed lid covers the scanner glass",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="glass_pane",
        min_gap=0.002,
        max_gap=0.008,
        name="closed lid clears the glass platen",
    )
    ctx.expect_gap(
        paper_guide,
        lid,
        axis="z",
        positive_elem="guide_plate",
        negative_elem="lid_shell",
        min_gap=0.0,
        max_gap=0.0015,
        name="folded paper guide rests just proud of the lid",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid hinge opens upward from the rear edge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    folded_guide_aabb = ctx.part_element_world_aabb(paper_guide, elem="guide_plate")
    with ctx.pose({guide_hinge: 1.35}):
        raised_guide_aabb = ctx.part_element_world_aabb(paper_guide, elem="guide_plate")
    ctx.check(
        "paper guide folds upward on its own hinge",
        folded_guide_aabb is not None
        and raised_guide_aabb is not None
        and raised_guide_aabb[1][2] > folded_guide_aabb[1][2] + 0.06,
        details=f"folded={folded_guide_aabb}, raised={raised_guide_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
