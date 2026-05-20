from __future__ import annotations

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
    model = ArticulatedObject(name="camera_nod_head")

    # Define materials
    metal = model.material("metal", rgba=(0.2, 0.2, 0.2, 1.0))
    aluminum = model.material("aluminum", rgba=(0.7, 0.7, 0.7, 1.0))
    plastic = model.material("plastic", rgba=(0.1, 0.1, 0.1, 1.0))
    glass = model.material("glass", rgba=(0.3, 0.3, 0.3, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    # Root part: fixed vertical support
    vertical_support = model.part("vertical_support")
    # Base plate
    vertical_support.visual(
        Box((0.2, 0.2, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
        material=metal,
    )
    # Vertical post (SDK Cylinder defaults to +Z axis)
    vertical_support.visual(
        Cylinder(radius=0.025, length=0.3),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        name="vertical_post",
        material=metal,
    )

    # Side fork (fixed to vertical support)
    side_fork = model.part("side_fork")
    # Mounting plate on top of vertical post
    side_fork.visual(
        Box((0.1, 0.1, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="mounting_plate",
        material=aluminum,
    )
    # Horizontal arm extending forward (+X)
    side_fork.visual(
        Box((0.20, 0.05, 0.02)),
        origin=Origin(xyz=(0.10, 0.0, 0.0)),  # x from 0.0 to 0.20 in side_fork frame
        name="horizontal_arm",
        material=aluminum,
    )
    # Fork prongs (two, along Y axis, at end of horizontal arm)
    # Prongs extend upward from arm (Z direction), centered at y=±0.05, z=0.05
    # Right prong (y=+0.05, extends in Z from 0.0 to 0.10)
    side_fork.visual(
        Box((0.02, 0.10, 0.10)),
        origin=Origin(xyz=(0.20, 0.05, 0.05)),
        name="right_prong",
        material=aluminum,
    )
    # Left prong (y=-0.05, extends in Z from 0.0 to 0.10)
    side_fork.visual(
        Box((0.02, 0.10, 0.10)),
        origin=Origin(xyz=(0.20, -0.05, 0.05)),
        name="left_prong",
        material=aluminum,
    )

    # Fixed articulation between vertical support and side fork
    model.articulation(
        "support_to_fork",
        ArticulationType.FIXED,
        parent=vertical_support,
        child=side_fork,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),  # top of vertical post (world coordinates)
        axis=(0.0, 0.0, 1.0),
    )

    # Camera body (pitch articulation)
    camera_body = model.part("camera_body")
    # Chamfered camera cube - positioned so back face is at axle (x=0 in camera_body frame)
    camera_cube_shape = (
        cq.Workplane("XY")
        .box(0.1, 0.15, 0.1)  # dx=0.1 (X), dy=0.15 (Y), dz=0.1 (Z)
        .edges("|X or |Y or |Z")
        .chamfer(0.005)
    )
    camera_body.visual(
        mesh_from_cadquery(camera_cube_shape, "camera_cube"),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),  # cube goes from x=0.0 to 0.1 (front)
        name="camera_cube",
        material=plastic,
    )
    # Lens cylinder (front face, +X axis)
    lens_shape = (
        cq.Workplane("YZ")  # workplane normal is X, so extrusion is along X
        .circle(0.03)
        .extrude(0.08)  # length 0.08 along X
        .faces("+X")
        .chamfer(0.002)
    )
    camera_body.visual(
        mesh_from_cadquery(lens_shape, "lens"),
        origin=Origin(xyz=(0.10, 0.0, 0.0)),  # at front face of cube (x=0.1)
        name="lens",
        material=glass,
    )
    # Axle with knobs (single mesh for connectivity)
    axle_knobs_shape = (
        cq.Workplane("XZ")  # workplane normal is Y, extrusion along Y
        .circle(0.005)
        .extrude(0.30)  # axle, length 0.30 along Y
        .union(
            cq.Workplane("XZ")
            .workplane(origin=(0, -0.13, 0))
            .circle(0.015)
            .extrude(0.02)  # left knob
        )
        .union(
            cq.Workplane("XZ")
            .workplane(origin=(0, 0.13, 0))
            .circle(0.015)
            .extrude(0.02)  # right knob
        )
    )
    camera_body.visual(
        mesh_from_cadquery(axle_knobs_shape, "axle_knobs"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="axle_knobs",
        material=metal,  # knobs will be metal instead of rubber for connectivity
    )

    # Pitch joint (revolute, Y axis) - at prongs location, axle at z=0.05
    # Negative Y axis so positive q pitches camera up (lens moves upward)
    model.articulation(
        "fork_to_camera_pitch",
        ArticulationType.REVOLUTE,
        parent=side_fork,
        child=camera_body,
        origin=Origin(xyz=(0.20, 0.0, 0.05)),  # at prongs, axle at z=0.05 above arm
        axis=(0.0, -1.0, 0.0),  # negative Y so positive q pitches up
        motion_limits=MotionLimits(lower=-0.5, upper=0.5, effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    vertical_support = object_model.get_part("vertical_support")
    side_fork = object_model.get_part("side_fork")
    camera_body = object_model.get_part("camera_body")
    pitch_joint = object_model.get_articulation("fork_to_camera_pitch")

    # Allow intentional overlaps
    # 1. Axle+knobs seated inside fork prong holes for pitch rotation
    ctx.allow_overlap(
        "camera_body",
        "side_fork",
        reason="Axle intentionally seated inside fork prong holes for pitch rotation",
        elem_a="axle_knobs",
        elem_b="left_prong",
    )
    ctx.allow_overlap(
        "camera_body",
        "side_fork",
        reason="Axle intentionally seated inside fork prong holes for pitch rotation",
        elem_a="axle_knobs",
        elem_b="right_prong",
    )
    # 2. Camera cube between prongs (intentional, cube is in front of prongs)
    ctx.allow_overlap(
        "camera_body",
        "side_fork",
        reason="Camera cube positioned between fork prongs as part of nod head design",
        elem_a="camera_cube",
        elem_b="left_prong",
    )
    ctx.allow_overlap(
        "camera_body",
        "side_fork",
        reason="Camera cube positioned between fork prongs as part of nod head design",
        elem_a="camera_cube",
        elem_b="right_prong",
    )
    # 3. Mounting plate on top of vertical post (intentional contact)
    ctx.allow_overlap(
        "side_fork",
        "vertical_support",
        reason="Mounting plate sits on top of vertical post",
        elem_a="mounting_plate",
        elem_b="vertical_post",
    )
    # 4. Horizontal arm overlaps vertical post (intentional, arm mounted on post)
    ctx.allow_overlap(
        "side_fork",
        "vertical_support",
        reason="Horizontal arm overlaps vertical post at mounting point",
        elem_a="horizontal_arm",
        elem_b="vertical_post",
    )

    # Proof: axle contacts both prongs
    ctx.expect_contact(
        camera_body,
        side_fork,
        elem_a="axle_knobs",
        elem_b="left_prong",
        contact_tol=0.01,
        name="axle contacts left prong",
    )
    ctx.expect_contact(
        camera_body,
        side_fork,
        elem_a="axle_knobs",
        elem_b="right_prong",
        contact_tol=0.01,
        name="axle contacts right prong",
    )

    # Check pitch joint motion (camera tilts up/down around Y axis)
    # With axis=(0, -1, 0), positive q should pitch camera up
    with ctx.pose({pitch_joint: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(camera_body, elem="camera_cube")
        rest_max_z = rest_aabb[1][2] if rest_aabb else None

    with ctx.pose({pitch_joint: 0.5}):
        up_aabb = ctx.part_element_world_aabb(camera_body, elem="camera_cube")
        up_max_z = up_aabb[1][2] if up_aabb else None
        ctx.check(
            "pitch up increases camera cube max Z",
            up_max_z is not None and rest_max_z is not None and up_max_z > rest_max_z,
            details=f"rest max_z={rest_max_z}, up max_z={up_max_z}",
        )

    with ctx.pose({pitch_joint: -0.5}):
        down_aabb = ctx.part_element_world_aabb(camera_body, elem="camera_cube")
        down_max_z = down_aabb[1][2] if down_aabb else None
        ctx.check(
            "pitch down decreases camera cube max Z",
            down_max_z is not None and rest_max_z is not None and down_max_z < rest_max_z,
            details=f"rest max_z={rest_max_z}, down max_z={down_max_z}",
        )

    # Check vertical support is root part
    ctx.check(
        "vertical support is root part",
        vertical_support == object_model.root_parts()[0],
        details="Root part mismatch",
    )

    return ctx.report()


object_model = build_object_model()
