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


def _cylinder_x(radius: float, length: float, *, x0: float = 0.0):
    solid = cq.Workplane("YZ").circle(radius).extrude(length)
    if abs(x0) > 1e-9:
        solid = solid.translate((x0, 0.0, 0.0))
    return solid


def _shell_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    x0: float = 0.0,
):
    return (
        cq.Workplane("YZ")
        .workplane(offset=x0)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _centered_shell_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
):
    return _shell_x(outer_radius, inner_radius, length, x0=-length * 0.5)


def _build_outer_barrel_shape():
    barrel = _shell_x(0.0305, 0.0252, 0.060, x0=0.004)
    barrel = barrel.union(_shell_x(0.0318, 0.0270, 0.010, x0=-0.004))
    barrel = barrel.union(_shell_x(0.0310, 0.0254, 0.014, x0=0.058))
    barrel = barrel.union(_shell_x(0.0313, 0.0294, 0.020, x0=0.020))
    barrel = barrel.union(_shell_x(0.0309, 0.0296, 0.004, x0=0.022))
    barrel = barrel.union(_shell_x(0.0309, 0.0296, 0.004, x0=0.030))
    barrel = barrel.union(_shell_x(0.0309, 0.0296, 0.004, x0=0.038))
    return barrel


def _build_inner_tube_shape():
    tube = _shell_x(0.0236, 0.0190, 0.052, x0=-0.032)
    tube = tube.union(_shell_x(0.0252, 0.0210, 0.007, x0=-0.029))
    tube = tube.union(_shell_x(0.0244, 0.0188, 0.010, x0=0.020))
    tube = tube.union(_shell_x(0.0250, 0.0184, 0.005, x0=0.030))
    return tube


def _build_focus_ring_shape():
    ring = _centered_shell_x(0.0287, 0.0236, 0.014)
    ring = ring.union(_shell_x(0.0293, 0.0236, 0.0017, x0=-0.0059))
    ring = ring.union(_shell_x(0.0293, 0.0236, 0.0017, x0=-0.0018))
    ring = ring.union(_shell_x(0.0293, 0.0236, 0.0017, x0=0.0023))
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_zoom_camera_lens")

    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.33, 0.34, 0.36, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.18, 0.24, 0.30, 0.45))

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        mesh_from_cadquery(_build_outer_barrel_shape(), "outer_barrel_shell"),
        material=satin_black,
        name="barrel_shell",
    )
    outer_barrel.visual(
        Box((0.022, 0.0032, 0.015)),
        origin=Origin(xyz=(0.036, 0.0310, 0.0)),
        material=trim_gray,
        name="control_land",
    )
    outer_barrel.visual(
        Cylinder(radius=0.0026, length=0.0036),
        origin=Origin(xyz=(0.036, 0.0342, 0.0037)),
        material=trim_gray,
        name="hinge_block_0",
    )
    outer_barrel.visual(
        Cylinder(radius=0.0026, length=0.0036),
        origin=Origin(xyz=(0.036, 0.0342, -0.0037)),
        material=trim_gray,
        name="hinge_block_1",
    )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        mesh_from_cadquery(_build_inner_tube_shape(), "inner_tube_shell"),
        material=satin_black,
        name="tube_shell",
    )
    inner_tube.visual(
        Cylinder(radius=0.0193, length=0.0018),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_smoke,
        name="front_glass",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_build_focus_ring_shape(), "focus_ring_shell"),
        material=rubber_black,
        name="focus_ring_shell",
    )

    zoom_rocker = model.part("zoom_rocker")
    zoom_rocker.visual(
        Cylinder(radius=0.0017, length=0.0042),
        material=trim_gray,
        name="rocker_hub",
    )
    zoom_rocker.visual(
        Box((0.0055, 0.0020, 0.0030)),
        origin=Origin(xyz=(0.0, 0.0007, 0.0)),
        material=trim_gray,
        name="rocker_web",
    )
    zoom_rocker.visual(
        Box((0.014, 0.0022, 0.0036)),
        origin=Origin(xyz=(0.0, -0.0001, 0.0)),
        material=trim_gray,
        name="rocker_pad",
    )
    zoom_rocker.visual(
        Cylinder(radius=0.0018, length=0.0022),
        origin=Origin(xyz=(0.007, -0.0001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="rocker_tip_0",
    )
    zoom_rocker.visual(
        Cylinder(radius=0.0018, length=0.0022),
        origin=Origin(xyz=(-0.007, -0.0001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="rocker_tip_1",
    )

    tube_extension = model.articulation(
        "tube_extension",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=inner_tube,
        origin=Origin(xyz=(0.071, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )
    tube_extension.meta["axis_story"] = "Positive travel extends the inner optical tube forward."

    focus_spin = model.articulation(
        "focus_spin",
        ArticulationType.CONTINUOUS,
        parent=inner_tube,
        child=focus_ring,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=14.0,
        ),
    )
    focus_spin.meta["axis_story"] = "The focus ring spins continuously around the optical axis."

    rocker_hinge = model.articulation(
        "rocker_hinge",
        ArticulationType.REVOLUTE,
        parent=outer_barrel,
        child=zoom_rocker,
        origin=Origin(xyz=(0.036, 0.0342, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=6.0,
            lower=-0.24,
            upper=0.24,
        ),
    )
    rocker_hinge.meta["axis_story"] = "The zoom rocker teeters fore and aft about its short side-wall hinge."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_barrel = object_model.get_part("outer_barrel")
    inner_tube = object_model.get_part("inner_tube")
    focus_ring = object_model.get_part("focus_ring")
    zoom_rocker = object_model.get_part("zoom_rocker")

    tube_extension = object_model.get_articulation("tube_extension")
    focus_spin = object_model.get_articulation("focus_spin")
    rocker_hinge = object_model.get_articulation("rocker_hinge")

    ctx.allow_overlap(
        outer_barrel,
        inner_tube,
        elem_a="barrel_shell",
        elem_b="tube_shell",
        reason=(
            "The extending optical tube is intentionally represented as a telescoping "
            "coaxial sleeve nested inside the barrel shell."
        ),
    )
    ctx.allow_overlap(
        inner_tube,
        focus_ring,
        elem_a="tube_shell",
        elem_b="focus_ring_shell",
        reason=(
            "The focus ring is intentionally represented as a concentric rotating sleeve "
            "wrapped around the front tube guide land."
        ),
    )

    ctx.expect_within(
        inner_tube,
        outer_barrel,
        axes="yz",
        elem_a="tube_shell",
        elem_b="barrel_shell",
        margin=0.008,
        name="inner tube stays coaxial inside outer barrel",
    )
    ctx.expect_overlap(
        inner_tube,
        outer_barrel,
        axes="x",
        elem_a="tube_shell",
        elem_b="barrel_shell",
        min_overlap=0.030,
        name="collapsed inner tube remains deeply inserted",
    )
    ctx.expect_overlap(
        focus_ring,
        inner_tube,
        axes="yz",
        elem_a="focus_ring_shell",
        elem_b="tube_shell",
        min_overlap=0.045,
        name="focus ring encircles the front tube",
    )
    ctx.expect_gap(
        focus_ring,
        outer_barrel,
        axis="x",
        positive_elem="focus_ring_shell",
        negative_elem="barrel_shell",
        min_gap=0.002,
        max_gap=0.008,
        name="focus ring sits ahead of the outer barrel front",
    )
    ctx.expect_gap(
        zoom_rocker,
        outer_barrel,
        axis="y",
        positive_elem="rocker_pad",
        negative_elem="control_land",
        min_gap=0.0001,
        max_gap=0.0010,
        name="zoom rocker stands slightly proud of its barrel land",
    )
    ctx.check(
        "focus ring articulation is continuous",
        focus_spin.articulation_type == ArticulationType.CONTINUOUS
        and focus_spin.motion_limits is not None
        and focus_spin.motion_limits.lower is None
        and focus_spin.motion_limits.upper is None,
        details=f"type={focus_spin.articulation_type}, limits={focus_spin.motion_limits}",
    )

    rest_tube_pos = ctx.part_world_position(inner_tube)
    rest_ring_pos = ctx.part_world_position(focus_ring)
    rest_rocker_aabb = ctx.part_element_world_aabb(zoom_rocker, elem="rocker_pad")

    extension_upper = 0.0
    if tube_extension.motion_limits is not None and tube_extension.motion_limits.upper is not None:
        extension_upper = tube_extension.motion_limits.upper

    rocker_upper = 0.0
    if rocker_hinge.motion_limits is not None and rocker_hinge.motion_limits.upper is not None:
        rocker_upper = rocker_hinge.motion_limits.upper

    with ctx.pose({tube_extension: extension_upper, rocker_hinge: rocker_upper}):
        ctx.expect_within(
            inner_tube,
            outer_barrel,
            axes="yz",
            elem_a="tube_shell",
            elem_b="barrel_shell",
            margin=0.008,
            name="extended inner tube stays coaxial inside outer barrel",
        )
        ctx.expect_overlap(
            inner_tube,
            outer_barrel,
            axes="x",
            elem_a="tube_shell",
            elem_b="barrel_shell",
            min_overlap=0.014,
            name="extended inner tube keeps retained insertion",
        )
        extended_tube_pos = ctx.part_world_position(inner_tube)
        extended_ring_pos = ctx.part_world_position(focus_ring)
        tilted_rocker_aabb = ctx.part_element_world_aabb(zoom_rocker, elem="rocker_pad")

    ctx.check(
        "inner tube extends forward",
        rest_tube_pos is not None
        and extended_tube_pos is not None
        and extended_tube_pos[0] > rest_tube_pos[0] + 0.015,
        details=f"rest={rest_tube_pos}, extended={extended_tube_pos}",
    )
    ctx.check(
        "focus ring rides with extending tube",
        rest_tube_pos is not None
        and extended_tube_pos is not None
        and rest_ring_pos is not None
        and extended_ring_pos is not None
        and abs((extended_ring_pos[0] - rest_ring_pos[0]) - (extended_tube_pos[0] - rest_tube_pos[0])) < 1e-6,
        details=(
            f"tube_rest={rest_tube_pos}, tube_extended={extended_tube_pos}, "
            f"ring_rest={rest_ring_pos}, ring_extended={extended_ring_pos}"
        ),
    )
    ctx.check(
        "zoom rocker visibly tilts about its hinge",
        rest_rocker_aabb is not None
        and tilted_rocker_aabb is not None
        and tilted_rocker_aabb[1][1] > rest_rocker_aabb[1][1] + 0.0002,
        details=f"rest={rest_rocker_aabb}, tilted={tilted_rocker_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
