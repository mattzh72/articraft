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


def _annular_tube(outer_radius: float, inner_radius: float, length: float):
    """CadQuery tube extruded upward from local z=0."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _square_plate_with_hole(width: float, depth: float, hole_radius: float, thickness: float):
    """Rectangular ground plate extruded upward from local z=0 with a round sleeve opening."""
    return cq.Workplane("XY").rect(width, depth).circle(hole_radius).extrude(thickness)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_security_post")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.10, 0.11, 0.11, 1.0))
    yellow = model.material("yellow_powder_coat", rgba=(1.0, 0.73, 0.05, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    white_reflector = model.material("white_reflector", rgba=(0.92, 0.92, 0.82, 1.0))
    amber = model.material("amber_lens", rgba=(1.0, 0.55, 0.02, 0.70))
    smoked = model.material("smoked_clear_cover", rgba=(0.42, 0.48, 0.45, 0.38))
    concrete = model.material("dark_concrete", rgba=(0.23, 0.23, 0.22, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_square_plate_with_hole(0.74, 0.74, 0.142, 0.045), "pavement_plate_hole"),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=concrete,
        name="flush_pavement_plate",
    )
    sleeve.visual(
        mesh_from_cadquery(_annular_tube(0.135, 0.105, 0.80), "recessed_sleeve_tube"),
        origin=Origin(xyz=(0.0, 0.0, -0.70)),
        material=dark_steel,
        name="sleeve_tube",
    )
    sleeve.visual(
        mesh_from_cadquery(_annular_tube(0.185, 0.105, 0.10), "raised_sleeve_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="collar_ring",
    )
    sleeve.visual(
        mesh_from_cadquery(_annular_tube(0.205, 0.105, 0.012), "collar_lip_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=galvanized,
        name="collar_lip",
    )
    sleeve.visual(
        Box((0.015, 0.040, 0.115)),
        origin=Origin(xyz=(0.1025, 0.0, 0.055)),
        material=black,
        name="guide_pad_0",
    )
    sleeve.visual(
        Box((0.015, 0.040, 0.115)),
        origin=Origin(xyz=(-0.1025, 0.0, 0.055)),
        material=black,
        name="guide_pad_1",
    )
    sleeve.visual(
        Box((0.040, 0.015, 0.115)),
        origin=Origin(xyz=(0.0, 0.1025, 0.055)),
        material=black,
        name="guide_pad_2",
    )
    sleeve.visual(
        Box((0.040, 0.015, 0.115)),
        origin=Origin(xyz=(0.0, -0.1025, 0.055)),
        material=black,
        name="guide_pad_3",
    )

    bollard = model.part("bollard")
    bollard.visual(
        Cylinder(radius=0.095, length=1.50),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=yellow,
        name="upper_cylinder",
    )
    bollard.visual(
        Cylinder(radius=0.098, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=black,
        name="lower_grip_band",
    )
    bollard.visual(
        Cylinder(radius=0.098, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        material=white_reflector,
        name="reflective_band",
    )
    bollard.visual(
        Cylinder(radius=0.103, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.105)),
        material=yellow,
        name="crown_cap",
    )
    bollard.visual(
        Cylinder(radius=0.042, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.140)),
        material=amber,
        name="warning_lamp",
    )
    bollard.visual(
        Box((0.012, 0.075, 0.075)),
        origin=Origin(xyz=(0.099, 0.0, 0.920)),
        material=dark_steel,
        name="service_boss",
    )
    bollard.visual(
        Box((0.052, 0.100, 0.022)),
        origin=Origin(xyz=(0.064, 0.0, 1.139)),
        material=dark_steel,
        name="hinge_saddle",
    )
    bollard.visual(
        Box((0.028, 0.012, 0.034)),
        origin=Origin(xyz=(0.078, -0.044, 1.165)),
        material=dark_steel,
        name="hinge_yoke_0",
    )
    bollard.visual(
        Box((0.028, 0.012, 0.034)),
        origin=Origin(xyz=(0.078, 0.044, 1.165)),
        material=dark_steel,
        name="hinge_yoke_1",
    )

    service_cap = model.part("service_cap")
    service_cap.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="cap_disk",
    )
    service_cap.visual(
        Box((0.0025, 0.008, 0.026)),
        origin=Origin(xyz=(0.0152, 0.0, 0.0)),
        material=black,
        name="key_slot",
    )

    light_cover = model.part("light_cover")
    light_cover.visual(
        Cylinder(radius=0.006, length=0.104),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="cover_hinge_knuckle",
    )
    light_cover.visual(
        Box((0.040, 0.050, 0.006)),
        origin=Origin(xyz=(-0.020, 0.0, 0.008)),
        material=galvanized,
        name="cover_leaf",
    )
    light_cover.visual(
        Box((0.014, 0.046, 0.030)),
        origin=Origin(xyz=(-0.034, 0.0, 0.020)),
        material=galvanized,
        name="cover_bridge",
    )
    light_cover.visual(
        Cylinder(radius=0.065, length=0.024),
        origin=Origin(xyz=(-0.064, 0.0, 0.035)),
        material=smoked,
        name="cover_lens",
    )

    slide = model.articulation(
        "sleeve_to_bollard",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=bollard,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.20, lower=0.0, upper=0.28),
    )
    slide.meta["description"] = "Vertical telescoping travel with retained insertion in the recessed collar."

    model.articulation(
        "bollard_to_service_cap",
        ArticulationType.CONTINUOUS,
        parent=bollard,
        child=service_cap,
        origin=Origin(xyz=(0.105, 0.0, 0.920)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    model.articulation(
        "bollard_to_light_cover",
        ArticulationType.REVOLUTE,
        parent=bollard,
        child=light_cover,
        origin=Origin(xyz=(0.078, 0.0, 1.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    bollard = object_model.get_part("bollard")
    service_cap = object_model.get_part("service_cap")
    light_cover = object_model.get_part("light_cover")
    slide = object_model.get_articulation("sleeve_to_bollard")
    cap_joint = object_model.get_articulation("bollard_to_service_cap")
    cover_hinge = object_model.get_articulation("bollard_to_light_cover")

    ctx.allow_overlap(
        light_cover,
        bollard,
        elem_a="cover_hinge_knuckle",
        elem_b="hinge_yoke_0",
        reason="The tiny hinge pin/knuckle is intentionally captured through the crown yoke bore.",
    )
    ctx.allow_overlap(
        light_cover,
        bollard,
        elem_a="cover_hinge_knuckle",
        elem_b="hinge_yoke_1",
        reason="The tiny hinge pin/knuckle is intentionally captured through the crown yoke bore.",
    )

    ctx.check(
        "post slides vertically in sleeve",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "service cap rotates continuously on radial axis",
        cap_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(cap_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={cap_joint.articulation_type}, axis={cap_joint.axis}",
    )
    ctx.check(
        "warning cover has upward hinge range",
        cover_hinge.motion_limits is not None
        and cover_hinge.motion_limits.lower == 0.0
        and cover_hinge.motion_limits.upper is not None
        and cover_hinge.motion_limits.upper > 1.0,
        details=f"limits={cover_hinge.motion_limits}",
    )

    ctx.expect_within(
        bollard,
        sleeve,
        axes="xy",
        inner_elem="upper_cylinder",
        outer_elem="collar_ring",
        margin=0.0,
        name="upper post is laterally captured by collar",
    )
    ctx.expect_overlap(
        bollard,
        sleeve,
        axes="z",
        elem_a="upper_cylinder",
        elem_b="collar_ring",
        min_overlap=0.09,
        name="post passes through sleeve collar at rest",
    )
    ctx.expect_contact(
        bollard,
        sleeve,
        elem_a="upper_cylinder",
        elem_b="guide_pad_0",
        contact_tol=0.001,
        name="collar guide pad bears on upper post",
    )
    ctx.expect_contact(
        service_cap,
        bollard,
        elem_a="cap_disk",
        elem_b="service_boss",
        contact_tol=0.001,
        name="service cap seats on welded boss",
    )
    ctx.expect_overlap(
        light_cover,
        bollard,
        axes="y",
        elem_a="cover_hinge_knuckle",
        elem_b="hinge_yoke_0",
        min_overlap=0.002,
        name="cover hinge knuckle aligns with crown yoke",
    )
    ctx.expect_overlap(
        light_cover,
        bollard,
        axes="y",
        elem_a="cover_hinge_knuckle",
        elem_b="hinge_yoke_1",
        min_overlap=0.002,
        name="cover hinge knuckle spans second yoke",
    )

    rest_pos = ctx.part_world_position(bollard)
    upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0
    with ctx.pose({slide: upper}):
        ctx.expect_within(
            bollard,
            sleeve,
            axes="xy",
            inner_elem="upper_cylinder",
            outer_elem="collar_ring",
            margin=0.0,
            name="extended post remains centered in collar",
        )
        ctx.expect_overlap(
            bollard,
            sleeve,
            axes="z",
            elem_a="upper_cylinder",
            elem_b="sleeve_tube",
            min_overlap=0.10,
            name="extended post retains insertion in sleeve",
        )
        extended_pos = ctx.part_world_position(bollard)

    ctx.check(
        "upper post moves upward at extension",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    closed_aabb = ctx.part_world_aabb(light_cover)
    with ctx.pose({cover_hinge: 1.1}):
        open_aabb = ctx.part_world_aabb(light_cover)
    ctx.check(
        "warning cover opens upward from crown",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.030,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
