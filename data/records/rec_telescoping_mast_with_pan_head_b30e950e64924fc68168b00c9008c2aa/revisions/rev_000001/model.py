from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_mesh(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 64,
) -> MeshGeometry:
    """Closed annular tube with a real through-hole, local z in [0, height]."""
    geom = MeshGeometry()
    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []

    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, 0.0))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, height))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, 0.0))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, height))

    for i in range(segments):
        j = (i + 1) % segments

        # Outer wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])

        # Inner wall, wound opposite the outer wall.
        geom.add_face(inner_bottom[j], inner_bottom[i], inner_top[i])
        geom.add_face(inner_bottom[j], inner_top[i], inner_top[j])

        # Top and bottom annular lips.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        geom.add_face(outer_bottom[j], outer_bottom[i], inner_bottom[i])
        geom.add_face(outer_bottom[j], inner_bottom[i], inner_bottom[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_pan_head")

    matte_black = Material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    anodized = Material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    sleeve_metal = Material("brushed_sleeve_metal", rgba=(0.45, 0.47, 0.48, 1.0))
    lower_metal = Material("satin_stage_aluminum", rgba=(0.62, 0.64, 0.63, 1.0))
    upper_metal = Material("light_stage_aluminum", rgba=(0.78, 0.79, 0.77, 1.0))
    rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    screw_metal = Material("polished_camera_screw", rgba=(0.82, 0.78, 0.66, 1.0))
    red_mark = Material("red_index_mark", rgba=(0.85, 0.04, 0.02, 1.0))

    fixed_sleeve_mesh = mesh_from_geometry(_tube_mesh(0.045, 0.036, 0.705), "fixed_sleeve")
    lower_tube_mesh = mesh_from_geometry(_tube_mesh(0.031, 0.024, 0.880), "lower_tube")
    lower_guide_mesh = mesh_from_geometry(_tube_mesh(0.036, 0.0305, 0.026), "lower_guide")
    lower_collar_mesh = mesh_from_geometry(_tube_mesh(0.039, 0.030, 0.075), "lower_collar")
    top_tube_mesh = mesh_from_geometry(_tube_mesh(0.022, 0.015, 0.720), "top_tube")
    top_guide_mesh = mesh_from_geometry(_tube_mesh(0.024, 0.0215, 0.022), "top_guide")
    top_bearing_mesh = mesh_from_geometry(_tube_mesh(0.036, 0.014, 0.052), "top_bearing")
    sleeve_collar_mesh = mesh_from_geometry(_tube_mesh(0.057, 0.044, 0.080), "sleeve_collar")

    mast_sleeve = model.part("mast_sleeve")
    mast_sleeve.visual(
        Cylinder(radius=0.145, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=matte_black,
        name="base_disk",
    )
    mast_sleeve.visual(
        Box((0.31, 0.055, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=matte_black,
        name="base_foot_x",
    )
    mast_sleeve.visual(
        Box((0.055, 0.31, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=matte_black,
        name="base_foot_y",
    )
    mast_sleeve.visual(
        Cylinder(radius=0.054, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=matte_black,
        name="base_boss",
    )
    mast_sleeve.visual(
        fixed_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=sleeve_metal,
        name="fixed_sleeve",
    )
    mast_sleeve.visual(
        sleeve_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.655)),
        material=anodized,
        name="sleeve_collar",
    )
    mast_sleeve.visual(
        Cylinder(radius=0.007, length=0.075),
        origin=Origin(xyz=(0.074, 0.0, 0.695), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screw_metal,
        name="sleeve_lock_screw",
    )
    mast_sleeve.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.120, 0.0, 0.695), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="sleeve_lock_knob",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        lower_tube_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.580)),
        material=lower_metal,
        name="lower_tube",
    )
    lower_stage.visual(
        lower_guide_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        material=rubber,
        name="lower_guide",
    )
    lower_stage.visual(
        lower_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=anodized,
        name="lower_collar",
    )
    lower_stage.visual(
        Cylinder(radius=0.0055, length=0.060),
        origin=Origin(xyz=(-0.064, 0.0, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screw_metal,
        name="lower_lock_screw",
    )
    lower_stage.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.102, 0.0, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lower_lock_knob",
    )

    top_stage = model.part("top_stage")
    top_stage.visual(
        top_tube_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        material=upper_metal,
        name="top_tube",
    )
    top_stage.visual(
        top_guide_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.440)),
        material=rubber,
        name="top_guide",
    )
    top_stage.visual(
        top_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        material=anodized,
        name="top_bearing",
    )
    top_stage.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        material=matte_black,
        name="bearing_seal",
    )

    rotary_plate = model.part("rotary_plate")
    rotary_plate.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
        name="yaw_disk",
    )
    rotary_plate.visual(
        Box((0.132, 0.076, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=anodized,
        name="camera_plate",
    )
    rotary_plate.visual(
        Box((0.104, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.022, 0.033)),
        material=rubber,
        name="rubber_pad_0",
    )
    rotary_plate.visual(
        Box((0.104, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, -0.022, 0.033)),
        material=rubber,
        name="rubber_pad_1",
    )
    rotary_plate.visual(
        Cylinder(radius=0.0075, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=screw_metal,
        name="camera_screw",
    )
    rotary_plate.visual(
        Box((0.026, 0.004, 0.004)),
        origin=Origin(xyz=(0.057, 0.0, 0.033)),
        material=red_mark,
        name="index_mark",
    )

    model.articulation(
        "sleeve_to_lower_stage",
        ArticulationType.PRISMATIC,
        parent=mast_sleeve,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.750)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.25, lower=0.0, upper=0.450),
    )
    model.articulation(
        "lower_to_top_stage",
        ArticulationType.PRISMATIC,
        parent=lower_stage,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.22, lower=0.0, upper=0.360),
    )
    model.articulation(
        "top_stage_to_plate",
        ArticulationType.REVOLUTE,
        parent=top_stage,
        child=rotary_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast_sleeve")
    lower = object_model.get_part("lower_stage")
    top = object_model.get_part("top_stage")
    plate = object_model.get_part("rotary_plate")
    lower_slide = object_model.get_articulation("sleeve_to_lower_stage")
    top_slide = object_model.get_articulation("lower_to_top_stage")
    yaw = object_model.get_articulation("top_stage_to_plate")

    ctx.allow_overlap(
        lower,
        mast,
        elem_a="lower_guide",
        elem_b="fixed_sleeve",
        reason=(
            "The lower-stage bushing is intentionally seated against the inside "
            "of the fixed sleeve as a local sliding guide proxy."
        ),
    )
    ctx.allow_overlap(
        top,
        lower,
        elem_a="top_guide",
        elem_b="lower_tube",
        reason=(
            "The top-stage bushing is intentionally seated against the lower "
            "stage tube as a local sliding guide proxy."
        ),
    )

    ctx.check(
        "serial vertical prismatic stages",
        lower_slide.articulation_type == ArticulationType.PRISMATIC
        and top_slide.articulation_type == ArticulationType.PRISMATIC
        and lower_slide.axis == (0.0, 0.0, 1.0)
        and top_slide.axis == (0.0, 0.0, 1.0),
        details=f"lower={lower_slide.articulation_type}/{lower_slide.axis}, top={top_slide.articulation_type}/{top_slide.axis}",
    )
    ctx.check(
        "plate yaws about mast centerline",
        yaw.articulation_type == ArticulationType.REVOLUTE and yaw.axis == (0.0, 0.0, 1.0),
        details=f"yaw={yaw.articulation_type}/{yaw.axis}",
    )

    ctx.expect_within(
        lower,
        mast,
        axes="xy",
        inner_elem="lower_tube",
        outer_elem="fixed_sleeve",
        margin=0.001,
        name="lower stage nests inside fixed sleeve",
    )
    ctx.expect_overlap(
        lower,
        mast,
        axes="z",
        elem_a="lower_tube",
        elem_b="fixed_sleeve",
        min_overlap=0.450,
        name="lower stage retained in fixed sleeve at rest",
    )
    ctx.expect_within(
        lower,
        mast,
        axes="xy",
        inner_elem="lower_guide",
        outer_elem="fixed_sleeve",
        margin=0.001,
        name="lower guide is captured by fixed sleeve",
    )
    ctx.expect_overlap(
        lower,
        mast,
        axes="z",
        elem_a="lower_guide",
        elem_b="fixed_sleeve",
        min_overlap=0.020,
        name="lower guide has seated sleeve engagement",
    )
    ctx.expect_within(
        top,
        lower,
        axes="xy",
        inner_elem="top_tube",
        outer_elem="lower_tube",
        margin=0.001,
        name="top stage nests inside lower stage",
    )
    ctx.expect_overlap(
        top,
        lower,
        axes="z",
        elem_a="top_tube",
        elem_b="lower_tube",
        min_overlap=0.450,
        name="top stage retained in lower stage at rest",
    )
    ctx.expect_within(
        top,
        lower,
        axes="xy",
        inner_elem="top_guide",
        outer_elem="lower_tube",
        margin=0.001,
        name="top guide is captured by lower stage",
    )
    ctx.expect_overlap(
        top,
        lower,
        axes="z",
        elem_a="top_guide",
        elem_b="lower_tube",
        min_overlap=0.018,
        name="top guide has seated lower-stage engagement",
    )
    ctx.expect_gap(
        plate,
        top,
        axis="z",
        positive_elem="yaw_disk",
        negative_elem="bearing_seal",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotary plate sits on bearing seal",
    )

    rest_lower = ctx.part_world_position(lower)
    rest_top = ctx.part_world_position(top)
    with ctx.pose({lower_slide: 0.450, top_slide: 0.360}):
        ctx.expect_overlap(
            lower,
            mast,
            axes="z",
            elem_a="lower_tube",
            elem_b="fixed_sleeve",
            min_overlap=0.120,
            name="extended lower stage keeps insertion",
        )
        ctx.expect_overlap(
            top,
            lower,
            axes="z",
            elem_a="top_tube",
            elem_b="lower_tube",
            min_overlap=0.120,
            name="extended top stage keeps insertion",
        )
        extended_lower = ctx.part_world_position(lower)
        extended_top = ctx.part_world_position(top)

    ctx.check(
        "mast stages extend upward",
        rest_lower is not None
        and rest_top is not None
        and extended_lower is not None
        and extended_top is not None
        and extended_lower[2] > rest_lower[2] + 0.40
        and extended_top[2] > rest_top[2] + 0.75,
        details=f"lower {rest_lower}->{extended_lower}, top {rest_top}->{extended_top}",
    )

    at_zero = ctx.part_element_world_aabb(plate, elem="camera_plate")
    with ctx.pose({yaw: math.pi / 2.0}):
        at_quarter = ctx.part_element_world_aabb(plate, elem="camera_plate")
    if at_zero is None or at_quarter is None:
        ctx.fail("rotary plate yaw visibly changes deck orientation", "camera_plate AABB unavailable")
    else:
        zero_size = (
            at_zero[1][0] - at_zero[0][0],
            at_zero[1][1] - at_zero[0][1],
        )
        quarter_size = (
            at_quarter[1][0] - at_quarter[0][0],
            at_quarter[1][1] - at_quarter[0][1],
        )
        ctx.check(
            "rotary plate yaw visibly changes deck orientation",
            zero_size[0] > zero_size[1] and quarter_size[1] > quarter_size[0],
            details=f"zero={zero_size}, quarter_turn={quarter_size}",
        )

    return ctx.report()


object_model = build_object_model()
