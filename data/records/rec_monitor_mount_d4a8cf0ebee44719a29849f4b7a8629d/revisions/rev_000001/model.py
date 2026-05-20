from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_cylinder_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 64,
) -> MeshGeometry:
    """A true hollow vertical bearing ring, centered on local Z."""
    geom = MeshGeometry()
    z0 = -height / 2.0
    z1 = height / 2.0

    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []
    for i in range(segments):
        angle = 2.0 * pi * i / segments
        c = cos(angle)
        s = sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z0))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z1))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z0))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z1))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner wall, wound inward.
        geom.add_face(inner_bottom[j], inner_bottom[i], inner_top[i])
        geom.add_face(inner_bottom[j], inner_top[i], inner_top[j])
        # Top annular face.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        # Bottom annular face.
        geom.add_face(outer_bottom[j], outer_bottom[i], inner_bottom[i])
        geom.add_face(outer_bottom[j], inner_bottom[i], inner_bottom[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_post_monitor_arm")

    model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    model.material("cast_dark", rgba=(0.08, 0.085, 0.09, 1.0))
    model.material("powder_coat", rgba=(0.13, 0.14, 0.145, 1.0))
    model.material("brushed_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    model.material("rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("trim_cap", rgba=(0.01, 0.011, 0.013, 1.0))

    # Grounded desk clamp and vertical post.
    clamp_post = model.part("clamp_post")
    clamp_post.visual(
        Box((0.18, 0.13, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="matte_black",
        name="upper_clamp_plate",
    )
    clamp_post.visual(
        Box((0.030, 0.120, 0.230)),
        origin=Origin(xyz=(-0.075, 0.0, 0.045)),
        material="matte_black",
        name="clamp_spine",
    )
    clamp_post.visual(
        Box((0.155, 0.110, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, -0.058)),
        material="matte_black",
        name="lower_clamp_jaw",
    )
    clamp_post.visual(
        Cylinder(radius=0.010, length=0.074),
        origin=Origin(xyz=(0.045, 0.0, -0.020)),
        material="brushed_steel",
        name="clamp_screw",
    )
    clamp_post.visual(
        Cylinder(radius=0.036, length=0.007),
        origin=Origin(xyz=(0.045, 0.0, 0.021)),
        material="rubber",
        name="pressure_pad",
    )
    clamp_post.visual(
        Cylinder(radius=0.052, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material="matte_black",
        name="post_flange",
    )
    clamp_post.visual(
        Cylinder(radius=0.026, length=0.610),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material="powder_coat",
        name="vertical_post",
    )
    clamp_post.visual(
        Cylinder(radius=0.056, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.557)),
        material="brushed_steel",
        name="thrust_washer",
    )
    clamp_post.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.651)),
        material="trim_cap",
        name="post_cap",
    )

    # Long primary arm: the collar ring is part of this moving casting.
    primary_link = model.part("primary_link")
    collar_ring_mesh = mesh_from_geometry(
        _annular_cylinder_mesh(outer_radius=0.056, inner_radius=0.033, height=0.070),
        "collar_bearing_ring",
    )
    primary_link.visual(
        collar_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material="cast_dark",
        name="collar_ring",
    )
    primary_link.visual(
        Box((0.120, 0.055, 0.052)),
        origin=Origin(xyz=(0.095, 0.0, -0.010)),
        material="cast_dark",
        name="collar_lug",
    )
    primary_link.visual(
        Cylinder(radius=0.055, length=0.052),
        origin=Origin(xyz=(0.095, 0.0, -0.010)),
        material="cast_dark",
        name="shoulder_boss",
    )
    primary_link.visual(
        Box((0.405, 0.070, 0.045)),
        origin=Origin(xyz=(0.335, 0.0, 0.000)),
        material="powder_coat",
        name="link_cover",
    )
    primary_link.visual(
        Box((0.345, 0.050, 0.012)),
        origin=Origin(xyz=(0.350, 0.0, -0.028)),
        material="trim_cap",
        name="cable_cover",
    )
    primary_link.visual(
        Cylinder(radius=0.062, length=0.042),
        origin=Origin(xyz=(0.585, 0.0, -0.024)),
        material="cast_dark",
        name="elbow_bearing",
    )
    primary_link.visual(
        Cylinder(radius=0.033, length=0.010),
        origin=Origin(xyz=(0.585, 0.0, 0.002)),
        material="brushed_steel",
        name="elbow_washer",
    )
    primary_link.visual(
        Box((0.070, 0.022, 0.012)),
        origin=Origin(xyz=(0.300, -0.046, -0.004)),
        material="trim_cap",
        name="primary_trim_0",
    )
    primary_link.visual(
        Box((0.070, 0.022, 0.012)),
        origin=Origin(xyz=(0.300, 0.046, -0.004)),
        material="trim_cap",
        name="primary_trim_1",
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        Cylinder(radius=0.052, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material="cast_dark",
        name="elbow_rotor",
    )
    secondary_link.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material="trim_cap",
        name="elbow_cap",
    )
    secondary_link.visual(
        Box((0.145, 0.078, 0.040)),
        origin=Origin(xyz=(0.070, 0.067, 0.044)),
        material="cast_dark",
        name="offset_bridge",
    )
    secondary_link.visual(
        Box((0.255, 0.055, 0.040)),
        origin=Origin(xyz=(0.235, 0.130, 0.045)),
        material="powder_coat",
        name="link_cover",
    )
    secondary_link.visual(
        Box((0.205, 0.038, 0.010)),
        origin=Origin(xyz=(0.245, 0.130, 0.018)),
        material="trim_cap",
        name="cable_cover",
    )
    secondary_link.visual(
        Cylinder(radius=0.052, length=0.075),
        origin=Origin(xyz=(0.385, 0.130, -0.003)),
        material="cast_dark",
        name="wrist_bearing",
    )
    secondary_link.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.385, 0.130, 0.041)),
        material="brushed_steel",
        name="wrist_washer",
    )

    head_pan = model.part("head_pan")
    head_pan.visual(
        Cylinder(radius=0.046, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material="cast_dark",
        name="pan_bearing",
    )
    head_pan.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material="brushed_steel",
        name="pan_spindle",
    )
    head_pan.visual(
        Cylinder(radius=0.028, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.127)),
        material="trim_cap",
        name="pan_cap",
    )
    head_pan.visual(
        Box((0.125, 0.060, 0.042)),
        origin=Origin(xyz=(0.074, 0.0, 0.105)),
        material="cast_dark",
        name="neck_casting",
    )
    head_pan.visual(
        Box((0.055, 0.116, 0.036)),
        origin=Origin(xyz=(0.125, 0.0, 0.105)),
        material="cast_dark",
        name="yoke_bridge",
    )
    head_pan.visual(
        Box((0.070, 0.010, 0.092)),
        origin=Origin(xyz=(0.170, -0.060, 0.105)),
        material="cast_dark",
        name="yoke_cheek_0",
    )
    head_pan.visual(
        Box((0.070, 0.010, 0.092)),
        origin=Origin(xyz=(0.170, 0.060, 0.105)),
        material="cast_dark",
        name="yoke_cheek_1",
    )

    vesa_head = model.part("vesa_head")
    vesa_head.visual(
        Cylinder(radius=0.022, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="tilt_trunnion",
    )
    vesa_head.visual(
        Box((0.040, 0.052, 0.036)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material="cast_dark",
        name="trunnion_block",
    )
    vesa_head.visual(
        Box((0.010, 0.126, 0.126)),
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        material="matte_black",
        name="frame_plate",
    )
    vesa_head.visual(
        Box((0.020, 0.180, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, 0.068)),
        material="powder_coat",
        name="frame_top",
    )
    vesa_head.visual(
        Box((0.020, 0.180, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, -0.068)),
        material="powder_coat",
        name="frame_bottom",
    )
    vesa_head.visual(
        Box((0.020, 0.018, 0.150)),
        origin=Origin(xyz=(0.055, -0.090, 0.0)),
        material="powder_coat",
        name="frame_side_0",
    )
    vesa_head.visual(
        Box((0.020, 0.018, 0.150)),
        origin=Origin(xyz=(0.055, 0.090, 0.0)),
        material="powder_coat",
        name="frame_side_1",
    )
    vesa_head.visual(
        Box((0.014, 0.160, 0.012)),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material="brushed_steel",
        name="vesa_crossbar",
    )
    vesa_head.visual(
        Box((0.014, 0.012, 0.116)),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material="brushed_steel",
        name="vesa_upright",
    )
    for idx, (y, z) in enumerate(
        ((-0.050, -0.050), (-0.050, 0.050), (0.050, -0.050), (0.050, 0.050))
    ):
        vesa_head.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(0.052, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material="brushed_steel",
            name=f"vesa_screw_{idx}",
        )

    collar_yaw = model.articulation(
        "collar_yaw",
        ArticulationType.REVOLUTE,
        parent=clamp_post,
        child=primary_link,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-1.55, upper=1.55),
    )
    elbow_yaw = model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(0.585, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=32.0, velocity=1.4, lower=0.0, upper=2.25),
    )
    pan_yaw = model.articulation(
        "pan_yaw",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=head_pan,
        origin=Origin(xyz=(0.385, 0.130, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.6, lower=-1.25, upper=1.25),
    )
    tilt_pitch = model.articulation(
        "tilt_pitch",
        ArticulationType.REVOLUTE,
        parent=head_pan,
        child=vesa_head,
        origin=Origin(xyz=(0.180, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.55, upper=0.65),
    )

    # Keep references named for static analyzers and for readability of the build.
    _ = (collar_yaw, elbow_yaw, pan_yaw, tilt_pitch)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("clamp_post")
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    pan = object_model.get_part("head_pan")
    head = object_model.get_part("vesa_head")
    collar = object_model.get_articulation("collar_yaw")
    elbow = object_model.get_articulation("elbow_yaw")
    pan_yaw = object_model.get_articulation("pan_yaw")
    tilt = object_model.get_articulation("tilt_pitch")

    ctx.expect_gap(
        primary,
        post,
        axis="z",
        positive_elem="collar_ring",
        negative_elem="thrust_washer",
        max_gap=0.001,
        max_penetration=0.000001,
        name="collar ring sits on thrust washer",
    )
    ctx.expect_gap(
        secondary,
        primary,
        axis="z",
        positive_elem="elbow_rotor",
        negative_elem="elbow_bearing",
        min_gap=0.012,
        max_gap=0.025,
        name="elbow bearing stack has vertical running clearance",
    )
    ctx.expect_gap(
        pan,
        secondary,
        axis="z",
        positive_elem="pan_spindle",
        negative_elem="wrist_washer",
        max_gap=0.001,
        max_penetration=0.000001,
        name="wrist pan spindle seats on its washer",
    )
    ctx.expect_gap(
        head,
        pan,
        axis="x",
        positive_elem="frame_plate",
        negative_elem="yoke_bridge",
        min_gap=0.018,
        name="VESA frame stands proud of pan yoke bridge",
    )

    with ctx.pose({elbow: 2.20}):
        ctx.expect_gap(
            secondary,
            primary,
            axis="z",
            positive_elem="link_cover",
            negative_elem="link_cover",
            min_gap=0.001,
            name="folded links remain vertically stacked apart",
        )

    rest_head_aabb = ctx.part_element_world_aabb(head, elem="frame_plate")
    with ctx.pose({tilt: 0.60}):
        tilted_head_aabb = ctx.part_element_world_aabb(head, elem="frame_plate")
        ctx.expect_gap(
            head,
            pan,
            axis="x",
            positive_elem="frame_plate",
            negative_elem="yoke_bridge",
            min_gap=0.004,
            name="tilted VESA frame clears pan yoke bridge",
        )

    ctx.check(
        "tilt visibly pitches the head frame",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][0] > rest_head_aabb[1][0] + 0.010,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )

    ctx.check(
        "commercial arm exposes four user-facing revolute axes",
        all(j is not None for j in (collar, elbow, pan_yaw, tilt)),
    )

    return ctx.report()


object_model = build_object_model()
