from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _leg_direction(yaw: float, spread_from_vertical: float) -> tuple[float, float, float]:
    return (
        math.sin(spread_from_vertical) * math.cos(yaw),
        math.sin(spread_from_vertical) * math.sin(yaw),
        -math.cos(spread_from_vertical),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plein_air_field_easel")

    matte_black = model.material("matte_black", rgba=(0.16, 0.16, 0.17, 1.0))
    anodized_aluminum = model.material(
        "anodized_aluminum",
        rgba=(0.69, 0.71, 0.74, 1.0),
    )
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    birch = model.material("birch", rgba=(0.72, 0.63, 0.50, 1.0))

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.065, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        material=matte_black,
        name="hub_casting",
    )
    hub.visual(
        Cylinder(radius=0.030, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=matte_black,
        name="center_post_sleeve",
    )
    hub.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=graphite,
        name="center_lock_collar",
    )
    hub.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(
            xyz=(0.046, 0.0, 0.820),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="center_lock_stem",
    )
    hub.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.064, 0.0, 0.820)),
        material=rubber,
        name="center_lock_knob",
    )

    leg_spread = 0.43
    sleeve_length = 0.130
    leg_mouth_radius = 0.110
    leg_mouth_z = 0.635
    leg_specs = (
        ("leg_front", math.pi / 2.0),
        ("leg_left", 7.0 * math.pi / 6.0),
        ("leg_right", 11.0 * math.pi / 6.0),
    )

    for index, (_, yaw) in enumerate(leg_specs):
        dx, dy, dz = _leg_direction(yaw, leg_spread)
        hub.visual(
            Cylinder(radius=0.019, length=sleeve_length),
            origin=Origin(
                xyz=(
                    leg_mouth_radius * math.cos(yaw) - 0.5 * sleeve_length * dx,
                    leg_mouth_radius * math.sin(yaw) - 0.5 * sleeve_length * dy,
                    leg_mouth_z - 0.5 * sleeve_length * dz,
                ),
                rpy=(0.0, math.pi - leg_spread, yaw),
            ),
            material=matte_black,
            name=f"leg_sleeve_{index}",
        )

    hub.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, 0.44)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
    )

    for name, yaw in leg_specs:
        leg = model.part(name)
        leg.visual(
            Cylinder(radius=0.014, length=0.600),
            origin=Origin(xyz=(0.0, 0.0, 0.300)),
            material=anodized_aluminum,
            name="lower_tube",
        )
        leg.visual(
            Box((0.040, 0.024, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, 0.075)),
            material=graphite,
            name="flip_lock",
        )
        leg.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.610)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.060, 0.060, 0.640)),
            mass=0.55,
            origin=Origin(xyz=(0.0, 0.0, 0.320)),
        )

        model.articulation(
            f"hub_to_{name}",
            ArticulationType.PRISMATIC,
            parent=hub,
            child=leg,
            origin=Origin(
                xyz=(
                    leg_mouth_radius * math.cos(yaw),
                    leg_mouth_radius * math.sin(yaw),
                    leg_mouth_z,
                ),
                rpy=(0.0, math.pi - leg_spread, yaw),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=0.12,
                lower=0.0,
                upper=0.060,
            ),
        )

    center_post = model.part("center_post")
    center_post.visual(
        Cylinder(radius=0.017, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=anodized_aluminum,
        name="mast_inner_tube",
    )
    center_post.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=graphite,
        name="mast_clamp_ring",
    )
    center_post.visual(
        Box((0.090, 0.040, 0.080)),
        origin=Origin(xyz=(0.0, -0.020, 0.555)),
        material=graphite,
        name="tilt_head",
    )
    center_post.visual(
        Box((0.012, 0.022, 0.070)),
        origin=Origin(xyz=(-0.038, -0.020, 0.600)),
        material=graphite,
        name="tilt_cheek_left",
    )
    center_post.visual(
        Box((0.012, 0.022, 0.070)),
        origin=Origin(xyz=(0.038, -0.020, 0.600)),
        material=graphite,
        name="tilt_cheek_right",
    )
    center_post.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(
            xyz=(0.058, -0.020, 0.555),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="tilt_lock_stem",
    )
    center_post.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.074, -0.020, 0.555)),
        material=rubber,
        name="tilt_lock_knob",
    )
    center_post.inertial = Inertial.from_geometry(
        Box((0.160, 0.080, 0.660)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.010, 0.330)),
    )

    model.articulation(
        "hub_to_center_post",
        ArticulationType.PRISMATIC,
        parent=hub,
        child=center_post,
        origin=Origin(xyz=(0.0, 0.0, 0.940)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.18,
            lower=0.0,
            upper=0.260,
        ),
    )

    canvas_platform = model.part("canvas_platform")
    canvas_platform.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    canvas_platform.visual(
        Box((0.056, 0.028, 0.160)),
        origin=Origin(xyz=(0.0, -0.014, 0.080)),
        material=graphite,
        name="receiver_spine",
    )
    canvas_platform.visual(
        Box((0.420, 0.012, 0.500)),
        origin=Origin(xyz=(0.0, -0.006, 0.315)),
        material=birch,
        name="platform_board",
    )
    canvas_platform.visual(
        Box((0.380, 0.055, 0.014)),
        origin=Origin(xyz=(0.0, 0.024, 0.042)),
        material=birch,
        name="canvas_shelf",
    )
    canvas_platform.visual(
        Box((0.380, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.046, 0.044)),
        material=graphite,
        name="shelf_lip",
    )
    canvas_platform.visual(
        Box((0.014, 0.026, 0.180)),
        origin=Origin(xyz=(-0.196, 0.010, 0.135)),
        material=graphite,
        name="side_guide_left",
    )
    canvas_platform.visual(
        Box((0.014, 0.026, 0.180)),
        origin=Origin(xyz=(0.196, 0.010, 0.135)),
        material=graphite,
        name="side_guide_right",
    )
    canvas_platform.visual(
        Box((0.100, 0.024, 0.016)),
        origin=Origin(xyz=(0.0, 0.004, 0.570)),
        material=graphite,
        name="top_clip_block",
    )
    canvas_platform.inertial = Inertial.from_geometry(
        Box((0.440, 0.090, 0.620)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.002, 0.310)),
    )

    model.articulation(
        "center_post_to_canvas_platform",
        ArticulationType.REVOLUTE,
        parent=center_post,
        child=canvas_platform,
        origin=Origin(xyz=(0.0, -0.020, 0.600)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-0.300,
            upper=0.600,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    hub = object_model.get_part("hub")
    center_post = object_model.get_part("center_post")
    canvas_platform = object_model.get_part("canvas_platform")
    post_joint = object_model.get_articulation("hub_to_center_post")
    platform_joint = object_model.get_articulation("center_post_to_canvas_platform")

    leg_parts = [
        object_model.get_part("leg_front"),
        object_model.get_part("leg_left"),
        object_model.get_part("leg_right"),
    ]
    leg_joints = [
        object_model.get_articulation("hub_to_leg_front"),
        object_model.get_articulation("hub_to_leg_left"),
        object_model.get_articulation("hub_to_leg_right"),
    ]

    for leg in leg_parts:
        ctx.expect_contact(
            leg,
            hub,
            contact_tol=0.002,
            name=f"{leg.name} remains seated in the hub sleeve",
        )

    ctx.expect_contact(
        center_post,
        hub,
        contact_tol=0.002,
        name="center post remains seated in the hub sleeve",
    )
    ctx.expect_overlap(
        canvas_platform,
        center_post,
        axes="x",
        elem_a="hinge_barrel",
        elem_b="tilt_head",
        min_overlap=0.050,
        name="platform hinge stays centered over the tilt head",
    )

    center_post_rest = ctx.part_world_position(center_post)
    with ctx.pose({post_joint: post_joint.motion_limits.upper}):
        center_post_extended = ctx.part_world_position(center_post)
    ctx.check(
        "center post extends upward",
        center_post_rest is not None
        and center_post_extended is not None
        and center_post_extended[2] > center_post_rest[2] + 0.20,
        details=f"rest={center_post_rest}, extended={center_post_extended}",
    )

    for leg, leg_joint in zip(leg_parts, leg_joints):
        rest_pos = ctx.part_world_position(leg)
        with ctx.pose({leg_joint: leg_joint.motion_limits.upper}):
            extended_pos = ctx.part_world_position(leg)
        rest_radius = None if rest_pos is None else math.hypot(rest_pos[0], rest_pos[1])
        extended_radius = None if extended_pos is None else math.hypot(extended_pos[0], extended_pos[1])
        ctx.check(
            f"{leg.name} extends outward from the hub",
            rest_pos is not None
            and extended_pos is not None
            and rest_radius is not None
            and extended_radius is not None
            and extended_radius > rest_radius + 0.015
            and extended_pos[2] < rest_pos[2] - 0.030,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    def _aabb_center_y(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    with ctx.pose({platform_joint: platform_joint.motion_limits.lower}):
        platform_forward = ctx.part_element_world_aabb(canvas_platform, elem="platform_board")
    with ctx.pose({platform_joint: platform_joint.motion_limits.upper}):
        platform_back = ctx.part_element_world_aabb(canvas_platform, elem="platform_board")

    forward_center_y = _aabb_center_y(platform_forward)
    back_center_y = _aabb_center_y(platform_back)
    ctx.check(
        "platform positive tilt leans the canvas rearward",
        forward_center_y is not None
        and back_center_y is not None
        and back_center_y < forward_center_y - 0.12,
        details=f"forward_center_y={forward_center_y}, back_center_y={back_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
