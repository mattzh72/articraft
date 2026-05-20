from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    model.material("matte_black", rgba=(0.025, 0.027, 0.030, 1.0))
    model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("painted_yellow", rgba=(0.95, 0.66, 0.16, 1.0))
    model.material("galvanized_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    model.material("concrete_gray", rgba=(0.55, 0.55, 0.52, 1.0))
    model.material("blue_glass", rgba=(0.35, 0.66, 0.95, 0.55))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.72, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="concrete_gray",
        name="foundation_plinth",
    )
    base.visual(
        Cylinder(radius=0.065, length=1.70),
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        material="galvanized_metal",
        name="mast_tube",
    )
    brace_mesh = mesh_from_geometry(
        wire_from_points(
            [(0.28, 0.28, 0.08), (0.055, 0.055, 1.32)],
            radius=0.016,
            radial_segments=14,
            cap_ends=True,
            corner_mode="miter",
        ),
        "mast_diagonal_brace",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        base.visual(
            brace_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material="dark_steel",
            name=f"brace_{index}",
        )
    base.visual(
        Cylinder(radius=0.17, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.74)),
        material="dark_steel",
        name="fixed_bearing_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 1.78)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.19, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="matte_black",
        name="turntable_platter",
    )
    pan_stage.visual(
        Cylinder(radius=0.09, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material="dark_steel",
        name="rotating_collar",
    )
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.54, 0.34, 0.72),
            span_width=0.36,
            trunnion_diameter=0.095,
            trunnion_center_z=0.46,
            base_thickness=0.10,
            corner_radius=0.018,
            center=False,
        ),
        "searchlight_yoke",
    )
    pan_stage.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.22), rpy=(0.0, 0.0, math.pi / 2.0)),
        material="painted_yellow",
        name="yoke_frame",
    )
    pan_stage.inertial = Inertial.from_geometry(
        Box((0.38, 0.58, 0.95)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.155, length=0.44),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_black",
        name="barrel_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.112, length=0.055),
        origin=Origin(xyz=(-0.165, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="rear_cap",
    )
    front_bezel_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.152, tube=0.022, radial_segments=18, tubular_segments=56),
        "front_bezel",
    )
    lamp_head.visual(
        front_bezel_mesh,
        origin=Origin(xyz=(0.305, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="painted_yellow",
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.126, length=0.026),
        origin=Origin(xyz=(0.300, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="blue_glass",
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.048, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="galvanized_metal",
        name="trunnion_shaft",
    )
    lamp_head.visual(
        Cylinder(radius=0.064, length=0.028),
        origin=Origin(xyz=(0.0, 0.158, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="trunnion_boss_0",
    )
    lamp_head.visual(
        Cylinder(radius=0.064, length=0.028),
        origin=Origin(xyz=(0.0, -0.158, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="trunnion_boss_1",
    )
    handle_mesh = mesh_from_geometry(
        wire_from_points(
            [(-0.055, 0.0, 0.145), (0.02, 0.0, 0.238), (0.18, 0.0, 0.238), (0.255, 0.0, 0.145)],
            radius=0.010,
            radial_segments=14,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.035,
        ),
        "top_grab_handle",
    )
    lamp_head.visual(
        handle_mesh,
        material="galvanized_metal",
        name="top_handle",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.52, 0.33, 0.34)),
        mass=14.0,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.78)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.7,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=65.0,
            velocity=0.8,
            lower=-math.radians(40.0),
            upper=math.radians(65.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pan_stage = object_model.get_part("pan_stage")
    lamp_head = object_model.get_part("lamp_head")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    ctx.check(
        "two user-facing revolute joints",
        len(object_model.articulations) == 2
        and pan_axis.articulation_type == ArticulationType.REVOLUTE
        and tilt_axis.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "pan axis is vertical",
        tuple(round(v, 6) for v in pan_axis.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan_axis.axis}",
    )
    ctx.check(
        "tilt axis is horizontal yoke axis",
        tuple(round(v, 6) for v in tilt_axis.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tilt_axis.axis}",
    )

    ctx.expect_gap(
        pan_stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_platter",
        negative_elem="fixed_bearing_cap",
        name="turntable is seated on high mast bearing",
    )
    ctx.expect_within(
        lamp_head,
        pan_stage,
        axes="y",
        margin=0.0,
        inner_elem="barrel_shell",
        outer_elem="yoke_frame",
        name="lamp barrel sits between yoke arms",
    )
    ctx.allow_overlap(
        lamp_head,
        pan_stage,
        elem_a="trunnion_shaft",
        elem_b="yoke_frame",
        reason="The tilt trunnion shaft is intentionally captured in the yoke bearing bores.",
    )
    ctx.expect_contact(
        lamp_head,
        pan_stage,
        elem_a="trunnion_shaft",
        elem_b="yoke_frame",
        contact_tol=0.002,
        name="trunnion shaft seats in yoke bearings",
    )
    ctx.expect_overlap(
        lamp_head,
        pan_stage,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="yoke_frame",
        min_overlap=0.50,
        name="trunnion shaft spans both yoke arms",
    )

    base_pos = ctx.part_world_position(base)
    pan_pos = ctx.part_world_position(pan_stage)
    ctx.check(
        "rotating stage is high above base",
        base_pos is not None and pan_pos is not None and pan_pos[2] - base_pos[2] > 1.6,
        details=f"base={base_pos}, pan_stage={pan_pos}",
    )

    rest_lens = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    with ctx.pose({tilt_axis: math.radians(45.0)}):
        raised_lens = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    ctx.check(
        "positive tilt raises the searchlight beam",
        rest_lens is not None
        and raised_lens is not None
        and (raised_lens[0][2] + raised_lens[1][2]) / 2.0
        > (rest_lens[0][2] + rest_lens[1][2]) / 2.0
        + 0.12,
        details=f"rest={rest_lens}, raised={raised_lens}",
    )

    with ctx.pose({pan_axis: math.pi / 2.0}):
        panned_lens = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    ctx.check(
        "pan joint turns the beam around the mast",
        rest_lens is not None
        and panned_lens is not None
        and abs((panned_lens[0][1] + panned_lens[1][1]) / 2.0)
        > abs((rest_lens[0][1] + rest_lens[1][1]) / 2.0)
        + 0.20,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    return ctx.report()


object_model = build_object_model()
