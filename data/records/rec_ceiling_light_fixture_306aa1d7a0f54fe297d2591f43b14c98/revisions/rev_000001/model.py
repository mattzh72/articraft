from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FIXTURE_LENGTH = 1.22
FIXTURE_WIDTH = 0.23
FIXTURE_HEIGHT = 0.095
HINGE_AXIS_Y = -FIXTURE_WIDTH / 2.0 - 0.009
HINGE_AXIS_Z = -FIXTURE_HEIGHT / 2.0 + 0.008


def _housing_shape() -> cq.Workplane:
    wall = 0.0035
    top_thickness = 0.004
    lip_thickness = 0.0025
    lip_depth = 0.012
    raceway_height = 0.018
    hinge_radius = 0.0045
    hinge_leaf_width = 0.011
    hinge_leaf_height = 0.016
    hinge_length = FIXTURE_LENGTH - 0.10

    shell = cq.Workplane("XY").box(FIXTURE_LENGTH, FIXTURE_WIDTH, FIXTURE_HEIGHT)
    cavity = (
        cq.Workplane("XY")
        .box(FIXTURE_LENGTH - 2.0 * wall, FIXTURE_WIDTH - 2.0 * wall, FIXTURE_HEIGHT)
        .translate((0.0, 0.0, -top_thickness))
    )
    shell = shell.cut(cavity)

    return_ring = (
        cq.Workplane("XY")
        .box(
            FIXTURE_LENGTH - 2.0 * wall,
            FIXTURE_WIDTH - 2.0 * wall,
            lip_thickness,
        )
        .translate((0.0, 0.0, -FIXTURE_HEIGHT / 2.0 + lip_thickness / 2.0))
    )
    opening = (
        cq.Workplane("XY")
        .box(
            FIXTURE_LENGTH - 2.0 * (wall + lip_depth),
            FIXTURE_WIDTH - 2.0 * (wall + lip_depth),
            lip_thickness + 0.004,
        )
        .translate((0.0, 0.0, -FIXTURE_HEIGHT / 2.0 + lip_thickness / 2.0))
    )
    shell = shell.union(return_ring.cut(opening))

    raceway = (
        cq.Workplane("XY")
        .box(FIXTURE_LENGTH * 0.62, FIXTURE_WIDTH * 0.44, raceway_height)
        .translate((0.0, 0.0, FIXTURE_HEIGHT / 2.0 + raceway_height / 2.0))
    )
    shell = shell.union(raceway)

    rib_height = 0.014
    rib_width = 0.032
    rib_z = FIXTURE_HEIGHT / 2.0 - top_thickness - rib_height / 2.0
    for rib_y in (-0.052, 0.052):
        rib = (
            cq.Workplane("XY")
            .box(FIXTURE_LENGTH - 0.10, rib_width, rib_height)
            .translate((0.0, rib_y, rib_z))
        )
        shell = shell.union(rib)

    hinge_leaf = (
        cq.Workplane("XY")
        .box(hinge_length, hinge_leaf_width, hinge_leaf_height)
        .translate(
            (
                0.0,
                -FIXTURE_WIDTH / 2.0 - hinge_leaf_width / 2.0,
                HINGE_AXIS_Z - 0.003,
            )
        )
    )
    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(hinge_radius)
        .extrude(hinge_length)
        .translate((-hinge_length / 2.0, HINGE_AXIS_Y, HINGE_AXIS_Z))
    )
    shell = shell.union(hinge_leaf).union(hinge_barrel)

    return shell


def _diffuser_frame_shape() -> cq.Workplane:
    frame_length = FIXTURE_LENGTH - 0.04
    frame_width = FIXTURE_WIDTH - 0.014
    frame_thickness = 0.015
    frame_front_offset = 0.006
    panel_top = -0.010
    frame_center_y = frame_front_offset + frame_width / 2.0
    frame_center_z = panel_top - frame_thickness / 2.0

    inner_length = frame_length - 0.10
    inner_width = frame_width - 0.060

    frame = (
        cq.Workplane("XY")
        .box(frame_length, frame_width, frame_thickness)
        .translate((0.0, frame_center_y, frame_center_z))
    )
    frame_opening = (
        cq.Workplane("XY")
        .box(inner_length, inner_width, frame_thickness + 0.010)
        .translate((0.0, frame_center_y, frame_center_z))
    )
    frame = frame.cut(frame_opening)

    hinge_leaf = (
        cq.Workplane("XY")
        .box(frame_length, 0.016, 0.010)
        .translate((0.0, 0.008, -0.014))
    )
    frame = frame.union(hinge_leaf)

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_ceiling_light")

    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    paint = model.material("paint", rgba=(0.90, 0.91, 0.92, 1.0))
    lens_material = model.material("lens", rgba=(0.96, 0.97, 0.99, 0.55))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing_shell"),
        material=steel,
        name="housing_shell",
    )

    diffuser = model.part("diffuser")
    diffuser.visual(
        mesh_from_cadquery(_diffuser_frame_shape(), "diffuser_frame"),
        material=paint,
        name="diffuser_frame",
    )

    lens_length = FIXTURE_LENGTH - 0.14
    lens_width = FIXTURE_WIDTH - 0.074
    lens_thickness = 0.0045
    diffuser.visual(
        Box((lens_length, lens_width, lens_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.006 + (FIXTURE_WIDTH - 0.014) / 2.0,
                -0.013,
            )
        ),
        material=lens_material,
        name="lens",
    )

    model.articulation(
        "housing_to_diffuser",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=diffuser,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=1.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    diffuser = object_model.get_part("diffuser")
    hinge = object_model.get_articulation("housing_to_diffuser")
    upper = 1.80
    if hinge.motion_limits is not None and hinge.motion_limits.upper is not None:
        upper = hinge.motion_limits.upper

    ctx.expect_gap(
        housing,
        diffuser,
        axis="z",
        positive_elem="housing_shell",
        negative_elem="diffuser_frame",
        max_penetration=0.003,
        max_gap=0.006,
        name="closed diffuser sits just below the housing rim",
    )
    ctx.expect_overlap(
        diffuser,
        housing,
        axes="xy",
        elem_a="diffuser_frame",
        elem_b="housing_shell",
        min_overlap=0.15,
        name="closed diffuser covers the light opening",
    )

    closed_lens_aabb = ctx.part_element_world_aabb(diffuser, elem="lens")
    open_lens_aabb = None
    with ctx.pose({hinge: upper}):
        ctx.expect_gap(
            housing,
            diffuser,
            axis="z",
            positive_elem="housing_shell",
            negative_elem="lens",
            min_gap=0.015,
            name="open diffuser hangs clearly below the housing",
        )
        open_lens_aabb = ctx.part_element_world_aabb(diffuser, elem="lens")

    ctx.check(
        "diffuser rotates downward from the hinge side",
        closed_lens_aabb is not None
        and open_lens_aabb is not None
        and open_lens_aabb[0][2] < closed_lens_aabb[0][2] - 0.12
        and open_lens_aabb[1][1] < closed_lens_aabb[1][1] - 0.15,
        details=f"closed={closed_lens_aabb}, open={open_lens_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
