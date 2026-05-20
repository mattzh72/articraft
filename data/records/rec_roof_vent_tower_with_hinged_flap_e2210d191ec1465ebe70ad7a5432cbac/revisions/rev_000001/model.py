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


def _housing_shell() -> cq.Workplane:
    """A compact rectangular metal tower with a real through-opening."""
    depth = 0.160
    width = 0.340
    height = 0.420
    opening_width = 0.240
    opening_height = 0.240
    opening_center_z = 0.045

    outer = cq.Workplane("XY").box(depth, width, height)
    vent_void = (
        cq.Workplane("XY")
        .box(depth + 0.060, opening_width, opening_height)
        .translate((0.0, 0.0, opening_center_z))
    )
    return outer.cut(vent_void).edges("|Z").fillet(0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized_zinc", rgba=(0.58, 0.62, 0.60, 1.0))
    dark_metal = model.material("dark_inner_shadow", rgba=(0.03, 0.035, 0.035, 1.0))
    flap_metal = model.material("darker_weathered_metal", rgba=(0.38, 0.42, 0.41, 1.0))
    rubber = model.material("black_edge_gasket", rgba=(0.01, 0.012, 0.012, 1.0))

    housing = model.part("housing")

    # Roof flashing and curb make the vent read as a rooftop-mounted tower.
    housing.visual(
        Box((0.360, 0.520, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=galvanized,
        name="roof_flashing",
    )
    housing.visual(
        Box((0.210, 0.310, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=galvanized,
        name="raised_curb",
    )
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "hollow_vent_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=galvanized,
        name="hollow_shell",
    )

    # A short, shallow framed outlet on the front face (+X).  Four bars leave
    # the actual vent hole open, rather than painting a hole on a solid box.
    front_x = 0.080
    frame_depth = 0.034
    frame_x = front_x + frame_depth / 2.0 - 0.006
    opening_width = 0.240
    opening_height = 0.240
    opening_center_z = 0.335
    opening_top = opening_center_z + opening_height / 2.0
    opening_bottom = opening_center_z - opening_height / 2.0
    frame_wall = 0.035
    outer_width = opening_width + 2.0 * frame_wall

    housing.visual(
        Box((frame_depth, frame_wall, opening_height + 2.0 * frame_wall)),
        origin=Origin(xyz=(frame_x, -(opening_width + frame_wall) / 2.0, opening_center_z)),
        material=galvanized,
        name="side_frame_0",
    )
    housing.visual(
        Box((frame_depth, frame_wall, opening_height + 2.0 * frame_wall)),
        origin=Origin(xyz=(frame_x, (opening_width + frame_wall) / 2.0, opening_center_z)),
        material=galvanized,
        name="side_frame_1",
    )
    housing.visual(
        Box((frame_depth, outer_width, frame_wall)),
        origin=Origin(xyz=(frame_x, 0.0, opening_top + frame_wall / 2.0)),
        material=galvanized,
        name="top_frame",
    )
    housing.visual(
        Box((frame_depth, outer_width, frame_wall)),
        origin=Origin(xyz=(frame_x, 0.0, opening_bottom - frame_wall / 2.0)),
        material=galvanized,
        name="bottom_frame",
    )
    housing.visual(
        Box((0.006, opening_width + 0.020, opening_height + 0.020)),
        origin=Origin(xyz=(-0.003, 0.0, opening_center_z)),
        material=dark_metal,
        name="recess_shadow",
    )
    housing.visual(
        Box((0.006, outer_width, 0.010)),
        origin=Origin(xyz=(front_x + 0.020, 0.0, opening_top + 0.010)),
        material=rubber,
        name="top_gasket",
    )
    housing.visual(
        Box((0.024, 0.034, 0.030)),
        origin=Origin(xyz=(front_x + 0.030, -0.142, opening_top + 0.020)),
        material=galvanized,
        name="hinge_lug_0",
    )
    housing.visual(
        Box((0.024, 0.034, 0.030)),
        origin=Origin(xyz=(front_x + 0.030, 0.142, opening_top + 0.020)),
        material=galvanized,
        name="hinge_lug_1",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.018, 0.268, 0.265)),
        origin=Origin(xyz=(0.013, 0.0, -0.1325)),
        material=flap_metal,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.010, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=flap_metal,
        name="hinge_curl",
    )
    flap.visual(
        Box((0.010, 0.248, 0.006)),
        origin=Origin(xyz=(0.003, 0.0, -0.263)),
        material=rubber,
        name="lower_drip_edge",
    )

    hinge_x = front_x + 0.050
    hinge_z = opening_top + 0.017
    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("housing_to_flap")

    ctx.expect_overlap(
        flap,
        housing,
        axes="yz",
        elem_a="flap_panel",
        elem_b="recess_shadow",
        min_overlap=0.18,
        name="flap covers the framed outlet",
    )
    ctx.expect_gap(
        flap,
        housing,
        axis="x",
        min_gap=0.004,
        max_gap=0.030,
        positive_elem="flap_panel",
        negative_elem="top_frame",
        name="closed flap sits proud of frame",
    )

    closed_pos = ctx.part_world_position(flap)
    with ctx.pose({hinge: 1.0}):
        ctx.expect_gap(
            flap,
            housing,
            axis="x",
            min_gap=0.020,
            positive_elem="flap_panel",
            negative_elem="top_frame",
            name="opened flap swings outward from outlet",
        )
        opened_pos = ctx.part_world_position(flap)

    ctx.check(
        "flap origin stays on top hinge line",
        closed_pos is not None
        and opened_pos is not None
        and abs(closed_pos[1] - opened_pos[1]) < 1e-6
        and abs(closed_pos[2] - opened_pos[2]) < 1e-6,
        details=f"closed={closed_pos}, opened={opened_pos}",
    )

    return ctx.report()


object_model = build_object_model()
