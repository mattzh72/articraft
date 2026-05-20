from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_orchestra_stand")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    metal = model.material("metal", rgba=(0.42, 0.43, 0.45, 1.0))

    hub_bottom = 0.082
    hub_height = 0.050
    sleeve_bottom = hub_bottom + hub_height - 0.003
    sleeve_height = 0.348
    collar_height = 0.055
    sleeve_outer_radius = 0.0165
    sleeve_inner_radius = 0.0134
    collar_outer_radius = 0.028
    sleeve_top = sleeve_bottom + sleeve_height
    collar_top = sleeve_top + collar_height
    boss_radius = 0.0105
    boss_length = 0.018
    boss_z = sleeve_top + collar_height * 0.55
    boss_outer_face_x = collar_outer_radius + boss_length

    tripod_hub = model.part("tripod_hub")

    hub_casting_shape = (
        cq.Workplane("XY")
        .circle(0.055)
        .extrude(hub_height)
        .translate((0.0, 0.0, hub_bottom))
    )
    tripod_hub.visual(
        mesh_from_cadquery(hub_casting_shape, "tripod_hub_casting"),
        material=powder_black,
        name="hub_casting",
    )

    outer_sleeve_shape = (
        cq.Workplane("XY")
        .circle(sleeve_outer_radius)
        .circle(sleeve_inner_radius)
        .extrude(sleeve_height)
        .translate((0.0, 0.0, sleeve_bottom))
    )
    outer_sleeve_shape = outer_sleeve_shape.union(
        cq.Workplane("XY")
        .circle(collar_outer_radius)
        .circle(sleeve_inner_radius)
        .extrude(collar_height)
        .translate((0.0, 0.0, sleeve_top))
    )
    outer_sleeve_shape = outer_sleeve_shape.union(
        cq.Workplane("YZ")
        .circle(boss_radius)
        .extrude(boss_length)
        .translate((collar_outer_radius, 0.0, boss_z))
    )
    tripod_hub.visual(
        mesh_from_cadquery(outer_sleeve_shape, "tripod_outer_sleeve"),
        material=graphite,
        name="outer_sleeve",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.024 * c, 0.024 * s, 0.110),
                (0.155 * c, 0.155 * s, 0.072),
                (0.355 * c, 0.355 * s, 0.015),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        tripod_hub.visual(
            mesh_from_geometry(leg_mesh, f"tripod_leg_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        tripod_hub.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.355 * c, 0.355 * s, 0.015)),
            material=rubber,
            name=f"foot_{index}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0115, length=0.920),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=graphite,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=powder_black,
        name="guide_collar",
    )
    mast.visual(
        Cylinder(radius=0.0155, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.539)),
        material=powder_black,
        name="head_post",
    )

    model.articulation(
        "hub_to_mast",
        ArticulationType.PRISMATIC,
        parent=tripod_hub,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, collar_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.240,
        ),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Box((0.062, 0.032, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=powder_black,
        name="receiver_block",
    )
    tilt_head.visual(
        Box((0.010, 0.024, 0.052)),
        origin=Origin(xyz=(-0.029, 0.0, 0.0)),
        material=powder_black,
        name="left_cheek",
    )
    tilt_head.visual(
        Box((0.010, 0.024, 0.052)),
        origin=Origin(xyz=(0.029, 0.0, 0.0)),
        material=powder_black,
        name="right_cheek",
    )
    tilt_head.visual(
        Box((0.068, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.012, 0.001)),
        material=graphite,
        name="front_bridge",
    )
    tilt_head.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(-0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="left_washer",
    )
    tilt_head.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="right_washer",
    )

    model.articulation(
        "mast_to_head",
        ArticulationType.FIXED,
        parent=mast,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.0, 0.601)),
    )

    desk = model.part("desk")
    desk_rest_tilt = 0.30
    desk.visual(
        Cylinder(radius=0.0075, length=0.044),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    desk.visual(
        Box((0.046, 0.028, 0.026)),
        origin=Origin(xyz=(0.0, -0.030, -0.010)),
        material=powder_black,
        name="hinge_block",
    )
    desk.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, -0.008, -0.010),
                    (0.0, -0.032, 0.058),
                    (0.0, -0.084, 0.146),
                    (0.0, -0.132, 0.214),
                ],
                radius=0.009,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "music_stand_support_arm",
        ),
        material=powder_black,
        name="support_arm",
    )
    desk.visual(
        Box((0.560, 0.006, 0.360)),
        origin=Origin(xyz=(0.0, -0.148, 0.228), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="panel",
    )
    desk.visual(
        Box((0.108, 0.020, 0.245)),
        origin=Origin(xyz=(0.0, -0.142, 0.198), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=powder_black,
        name="center_spine",
    )
    desk.visual(
        Box((0.014, 0.024, 0.360)),
        origin=Origin(xyz=(-0.273, -0.145, 0.228), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="flange_0",
    )
    desk.visual(
        Box((0.014, 0.024, 0.360)),
        origin=Origin(xyz=(0.273, -0.145, 0.228), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="flange_1",
    )
    desk.visual(
        Box((0.500, 0.046, 0.024)),
        origin=Origin(xyz=(0.0, -0.086, 0.078), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="lower_tray",
    )
    desk.visual(
        Box((0.500, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.056, 0.078), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="retaining_lip",
    )

    model.articulation(
        "head_to_desk",
        ArticulationType.REVOLUTE,
        parent=tilt_head,
        child=desk,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-0.22,
            upper=0.58,
        ),
    )

    collar_knob = model.part("collar_knob")
    collar_knob.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="knob_stem",
    )
    collar_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.018,
                body_style="mushroom",
                top_diameter=0.036,
                edge_radius=0.002,
                center=False,
            ),
            "collar_knob_body",
        ),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )

    model.articulation(
        "hub_to_knob",
        ArticulationType.CONTINUOUS,
        parent=tripod_hub,
        child=collar_knob,
        origin=Origin(xyz=(boss_outer_face_x, 0.0, boss_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod_hub = object_model.get_part("tripod_hub")
    mast = object_model.get_part("mast")
    tilt_head = object_model.get_part("tilt_head")
    desk = object_model.get_part("desk")
    collar_knob = object_model.get_part("collar_knob")

    mast_slide = object_model.get_articulation("hub_to_mast")
    desk_hinge = object_model.get_articulation("head_to_desk")
    knob_spin = object_model.get_articulation("hub_to_knob")

    ctx.expect_within(
        mast,
        tripod_hub,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="mast stays centered in the sleeve at rest",
    )
    ctx.expect_overlap(
        mast,
        tripod_hub,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.300,
        name="rest pose keeps long mast insertion inside the sleeve",
    )
    ctx.expect_contact(
        collar_knob,
        tripod_hub,
        elem_a="knob_stem",
        elem_b="outer_sleeve",
        contact_tol=1e-4,
        name="collar knob stem seats on the clamp boss",
    )
    ctx.check(
        "knob uses continuous articulation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.motion_limits is not None
        and knob_spin.motion_limits.lower is None
        and knob_spin.motion_limits.upper is None,
        details=f"type={knob_spin.articulation_type}, limits={knob_spin.motion_limits}",
    )

    slide_limits = mast_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        rest_pos = ctx.part_world_position(mast)
        with ctx.pose({mast_slide: slide_limits.upper}):
            ctx.expect_within(
                mast,
                tripod_hub,
                axes="xy",
                inner_elem="inner_tube",
                outer_elem="outer_sleeve",
                margin=0.004,
                name="fully extended mast stays centered in the sleeve",
            )
            ctx.expect_overlap(
                mast,
                tripod_hub,
                axes="z",
                elem_a="inner_tube",
                elem_b="outer_sleeve",
                min_overlap=0.120,
                name="fully extended mast still retains insertion in the sleeve",
            )
            extended_pos = ctx.part_world_position(mast)

        ctx.check(
            "mast extends upward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.20,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    hinge_limits = desk_hinge.motion_limits
    if hinge_limits is not None and hinge_limits.lower is not None and hinge_limits.upper is not None:
        with ctx.pose({desk_hinge: hinge_limits.lower}):
            low_panel = ctx.part_element_world_aabb(desk, elem="panel")
        with ctx.pose({desk_hinge: hinge_limits.upper}):
            high_panel = ctx.part_element_world_aabb(desk, elem="panel")

        ctx.check(
            "desk reaches a steeper angle at the upper hinge limit",
            low_panel is not None
            and high_panel is not None
            and high_panel[1][2] > low_panel[1][2] + 0.06
            and abs(high_panel[0][1] - low_panel[0][1]) > 0.10,
            details=f"lower={low_panel}, upper={high_panel}",
        )

    ctx.expect_gap(
        tilt_head,
        mast,
        axis="z",
        positive_elem="receiver_block",
        negative_elem="head_post",
        max_gap=0.001,
        max_penetration=1e-6,
        name="tilt head seats directly on the mast post",
    )

    return ctx.report()


object_model = build_object_model()
