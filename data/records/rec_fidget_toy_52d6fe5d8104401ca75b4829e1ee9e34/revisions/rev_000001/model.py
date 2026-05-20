from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(
    length: float,
    width: float,
    height: float,
    *,
    radius: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    """A small rounded rectangular solid, authored in meters."""
    shape = cq.Workplane("XY").box(length, width, height)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape.translate(center)


def _housing_body_shape() -> cq.Workplane:
    length = 0.160
    width = 0.052
    base_h = 0.008
    rail_h = 0.014
    rail_w = 0.006
    end_w = 0.012

    base = _rounded_box(
        length,
        width,
        base_h,
        radius=0.006,
        center=(0.0, 0.0, base_h / 2.0),
    )
    body = base

    for y in (-width / 2.0 + rail_w / 2.0, width / 2.0 - rail_w / 2.0):
        rail = _rounded_box(
            length - end_w,
            rail_w,
            rail_h,
            radius=0.0015,
            center=(0.0, y, base_h + rail_h / 2.0),
        )
        body = body.union(rail)

    for x in (-length / 2.0 + end_w / 2.0, length / 2.0 - end_w / 2.0):
        end_stop = _rounded_box(
            end_w,
            width,
            rail_h,
            radius=0.003,
            center=(x, 0.0, base_h + rail_h / 2.0),
        )
        body = body.union(end_stop)

    return body


def _thumb_pad_shape() -> cq.Workplane:
    """Raised slider cap with transverse grip ribs."""
    cap = _rounded_box(
        0.047,
        0.032,
        0.010,
        radius=0.004,
        center=(0.0, 0.0, 0.003 + 0.010 / 2.0),
    )

    body = cap
    for x in (-0.016, -0.0095, -0.003, 0.0035, 0.010, 0.0165):
        rib = _rounded_box(
            0.0022,
            0.025,
            0.0015,
            radius=0.0008,
            center=(x, 0.0, 0.003 + 0.010 + 0.0015 / 2.0),
        )
        body = body.union(rib)

    return body


def _detent_plate_shape() -> cq.Workplane:
    """A short flat pawl with a pointed tooth and a clearance hole at the pivot."""
    plate_h = 0.0035
    outer_r = 0.0049
    inner_r = 0.0028
    hub = cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(plate_h)

    arm_points = [
        (0.0044, -0.0022),
        (0.0170, -0.0022),
        (0.0215, -0.0050),  # inward click tooth
        (0.0240, -0.0010),
        (0.0200, 0.0024),
        (0.0044, 0.0024),
    ]
    arm = cq.Workplane("XY").polyline(arm_points).close().extrude(plate_h)
    return hub.union(arm)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clicking_slider_fidget")

    housing_mat = model.material("matte_ink_housing", rgba=(0.055, 0.060, 0.090, 1.0))
    track_mat = model.material("low_friction_black_track", rgba=(0.010, 0.012, 0.016, 1.0))
    slider_mat = model.material("warm_orange_slider", rgba=(0.95, 0.34, 0.08, 1.0))
    runner_mat = model.material("dark_orange_runner", rgba=(0.55, 0.16, 0.05, 1.0))
    steel_mat = model.material("brushed_spring_steel", rgba=(0.72, 0.72, 0.68, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_body_shape(), "housing_body", tolerance=0.0007),
        material=housing_mat,
        name="housing_body",
    )
    housing.visual(
        Box((0.120, 0.023, 0.0007)),
        origin=Origin(xyz=(0.0, 0.0, 0.00835)),
        material=track_mat,
        name="track_floor",
    )

    # Steel pivot posts stand on the housing rails and pass through the pawl holes.
    pivot_specs = (
        ("pivot_post_0", (-0.060, 0.0215, 0.0255)),
        ("pivot_post_1", (0.060, -0.0215, 0.0255)),
    )
    for post_name, xyz in pivot_specs:
        housing.visual(
            Cylinder(radius=0.0022, length=0.0070),
            origin=Origin(xyz=xyz),
            material=steel_mat,
            name=post_name,
        )

    thumb_slider = model.part("thumb_slider")
    thumb_slider.visual(
        Box((0.065, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=runner_mat,
        name="slider_runner",
    )
    thumb_slider.visual(
        mesh_from_cadquery(_thumb_pad_shape(), "thumb_pad", tolerance=0.00045),
        material=slider_mat,
        name="thumb_pad",
    )

    detent_mesh = mesh_from_cadquery(_detent_plate_shape(), "detent_plate", tolerance=0.00035)
    for detent_name in ("detent_0", "detent_1"):
        detent = model.part(detent_name)
        detent.visual(
            detent_mesh,
            material=steel_mat,
            name="detent_plate",
        )

    slide_joint = model.articulation(
        "housing_to_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=thumb_slider,
        origin=Origin(xyz=(0.0, 0.0, 0.0087)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=0.18, lower=-0.034, upper=0.034),
        motion_properties=MotionProperties(damping=0.05, friction=0.25),
    )
    slide_joint.meta["description"] = "Thumb pad slides along the long axis between click stops."

    for name, xyz, yaw in (
        ("housing_to_detent_0", (-0.060, 0.0215, 0.0220), 0.0),
        ("housing_to_detent_1", (0.060, -0.0215, 0.0220), math.pi),
    ):
        detent_name = name.replace("housing_to_", "")
        joint = model.articulation(
            name,
            ArticulationType.REVOLUTE,
            parent=housing,
            child=detent_name,
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.18, velocity=8.0, lower=-0.32, upper=0.32),
            motion_properties=MotionProperties(damping=0.02, friction=0.04),
        )
        joint.meta["description"] = "Short spring detent pawl rocks as the slider tooth clicks past."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    slider = object_model.get_part("thumb_slider")
    detent_0 = object_model.get_part("detent_0")
    detent_1 = object_model.get_part("detent_1")
    slide = object_model.get_articulation("housing_to_slider")
    detent_joint_0 = object_model.get_articulation("housing_to_detent_0")
    detent_joint_1 = object_model.get_articulation("housing_to_detent_1")

    ctx.check(
        "thumb slider is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type is {slide.articulation_type}",
    )
    ctx.check(
        "both click detents are revolute",
        detent_joint_0.articulation_type == ArticulationType.REVOLUTE
        and detent_joint_1.articulation_type == ArticulationType.REVOLUTE,
        details=f"types: {detent_joint_0.articulation_type}, {detent_joint_1.articulation_type}",
    )

    ctx.expect_within(
        slider,
        housing,
        axes="y",
        inner_elem="slider_runner",
        outer_elem="track_floor",
        margin=0.0005,
        name="runner is laterally captured in the center track",
    )
    ctx.expect_overlap(
        slider,
        housing,
        axes="x",
        elem_a="slider_runner",
        elem_b="track_floor",
        min_overlap=0.055,
        name="centered slider remains retained by the long track",
    )
    ctx.expect_gap(
        slider,
        housing,
        axis="z",
        positive_elem="slider_runner",
        negative_elem="track_floor",
        max_gap=0.0001,
        max_penetration=0.0,
        name="slider runner rests on the low-friction track",
    )

    rest_pos = ctx.part_world_position(slider)
    with ctx.pose({slide: 0.034}):
        upper_pos = ctx.part_world_position(slider)
        ctx.expect_overlap(
            slider,
            housing,
            axes="x",
            elem_a="slider_runner",
            elem_b="track_floor",
            min_overlap=0.025,
            name="slider is still retained at the forward click stop",
        )
        ctx.expect_overlap(
            detent_1,
            slider,
            axes="xz",
            elem_a="detent_plate",
            elem_b="thumb_pad",
            min_overlap=0.001,
            name="forward detent tooth overlaps the slider end in projection",
        )
        ctx.expect_gap(
            slider,
            detent_1,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0,
            name="forward detent tooth sits at the slider side",
        )

    with ctx.pose({slide: -0.034}):
        lower_pos = ctx.part_world_position(slider)
        ctx.expect_overlap(
            slider,
            housing,
            axes="x",
            elem_a="slider_runner",
            elem_b="track_floor",
            min_overlap=0.025,
            name="slider is still retained at the rear click stop",
        )
        ctx.expect_overlap(
            detent_0,
            slider,
            axes="xz",
            elem_a="detent_plate",
            elem_b="thumb_pad",
            min_overlap=0.001,
            name="rear detent tooth overlaps the slider end in projection",
        )
        ctx.expect_gap(
            detent_0,
            slider,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0,
            name="rear detent tooth sits at the slider side",
        )

    ctx.check(
        "thumb slider travel follows the long axis",
        rest_pos is not None
        and upper_pos is not None
        and lower_pos is not None
        and upper_pos[0] > rest_pos[0] + 0.030
        and lower_pos[0] < rest_pos[0] - 0.030,
        details=f"rest={rest_pos}, upper={upper_pos}, lower={lower_pos}",
    )

    return ctx.report()


object_model = build_object_model()
