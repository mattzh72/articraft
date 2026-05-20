from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _box_on_ring(part, *, radius: float, angle: float, z: float, size, tangential_offset=0.0, material=None, name=None) -> None:
    """Place a yawed box with local +X radial and local +Y tangential."""
    ca = math.cos(angle)
    sa = math.sin(angle)
    x = radius * ca - tangential_offset * sa
    y = radius * sa + tangential_offset * ca
    part.visual(
        Box(size),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, angle)),
        material=material,
        name=name,
    )


def _cylinder_between(part, start, end, radius: float, *, material=None, name=None) -> None:
    """Cylinder between two points in a local XZ plane."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > 1e-9:
        raise ValueError("this helper is intentionally limited to local XZ-plane cylinders")
    pitch = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_photo_device")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.06, 0.065, 0.07, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.45, 0.47, 0.48, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    camera_body = model.material("camera_body", rgba=(0.025, 0.026, 0.030, 1.0))
    glass = model.material("dark_lens_glass", rgba=(0.02, 0.045, 0.070, 1.0))

    tripod_core = model.part("tripod_core")
    tripod_core.visual(
        Cylinder(radius=0.033, length=0.37),
        origin=Origin(xyz=(0.0, 0.0, 0.935)),
        material=dark_metal,
        name="center_column",
    )
    tripod_core.visual(
        Cylinder(radius=0.092, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=dark_metal,
        name="crown_disk",
    )
    tripod_core.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 1.105)),
        material=satin_metal,
        name="fixed_pan_bearing",
    )
    tripod_core.visual(
        Cylinder(radius=0.020, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=dark_metal,
        name="lower_column_stub",
    )

    leg_angles = (math.radians(90.0), math.radians(210.0), math.radians(330.0))
    for i, angle in enumerate(leg_angles):
        _box_on_ring(
            tripod_core,
            radius=0.098,
            angle=angle,
            z=0.820,
            size=(0.065, 0.012, 0.047),
            tangential_offset=0.031,
            material=dark_metal,
            name=f"leg_yoke_outer_{i}",
        )
        _box_on_ring(
            tripod_core,
            radius=0.098,
            angle=angle,
            z=0.820,
            size=(0.065, 0.012, 0.047),
            tangential_offset=-0.031,
            material=dark_metal,
            name=f"leg_yoke_inner_{i}",
        )

    for i, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.015, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name="hinge_knuckle",
        )
        _cylinder_between(
            leg,
            (0.010, 0.0, -0.010),
            (0.565, 0.0, -0.800),
            0.016,
            material=matte_black,
            name="leg_tube",
        )
        _cylinder_between(
            leg,
            (0.405, 0.0, -0.570),
            (0.600, 0.0, -0.850),
            0.012,
            material=dark_metal,
            name="lower_leg_sleeve",
        )
        leg.visual(
            Sphere(radius=0.035),
            origin=Origin(xyz=(0.610, 0.0, -0.855)),
            material=rubber,
            name="rubber_foot",
        )
        model.articulation(
            f"crown_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=tripod_core,
            child=leg,
            origin=Origin(xyz=(0.145 * math.cos(angle), 0.145 * math.sin(angle), 0.820), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-0.28, upper=0.78),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.064, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=satin_metal,
        name="rotating_pan_disk",
    )
    pan_head.visual(
        Cylinder(radius=0.031, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=dark_metal,
        name="pan_neck",
    )
    pan_head.visual(
        Box((0.165, 0.104, 0.074)),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=matte_black,
        name="head_body",
    )
    pan_head.visual(
        Cylinder(radius=0.031, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.132), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="tilt_boss",
    )

    model.articulation(
        "core_to_pan",
        ArticulationType.CONTINUOUS,
        parent=tripod_core,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.8),
    )

    side_handle = model.part("side_handle")
    side_handle.visual(
        Cylinder(radius=0.021, length=0.032),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="pivot_collar",
    )
    side_handle.visual(
        Cylinder(radius=0.0085, length=0.300),
        origin=Origin(xyz=(0.0, -0.180, -0.010), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handle_stem",
    )
    side_handle.visual(
        Cylinder(radius=0.018, length=0.118),
        origin=Origin(xyz=(0.0, -0.376, -0.018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rubber_grip",
    )
    handle_joint = model.articulation(
        "head_to_handle",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=side_handle,
        origin=Origin(xyz=(0.0, -0.052, 0.132)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.55, upper=0.75),
    )

    camera_mount = model.part("camera_mount")
    camera_mount.visual(
        Cylinder(radius=0.014, length=0.084),
        origin=Origin(xyz=(0.0, 0.036, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="tilt_axle",
    )
    camera_mount.visual(
        Box((0.036, 0.032, 0.090)),
        origin=Origin(xyz=(0.0, 0.082, 0.042)),
        material=satin_metal,
        name="side_trunnion",
    )
    camera_mount.visual(
        Box((0.205, 0.150, 0.025)),
        origin=Origin(xyz=(0.042, 0.0, 0.065)),
        material=satin_metal,
        name="quick_release_plate",
    )
    camera_mount.visual(
        Box((0.180, 0.076, 0.100)),
        origin=Origin(xyz=(0.060, 0.0, 0.125)),
        material=camera_body,
        name="camera_body",
    )
    camera_mount.visual(
        Cylinder(radius=0.032, length=0.092),
        origin=Origin(xyz=(0.190, 0.0, 0.132), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lens_barrel",
    )
    camera_mount.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.240, 0.0, 0.132), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    camera_mount.visual(
        Box((0.085, 0.043, 0.026)),
        origin=Origin(xyz=(0.025, 0.0, 0.188)),
        material=camera_body,
        name="viewfinder_hump",
    )
    camera_mount.visual(
        Box((0.040, 0.030, 0.006)),
        origin=Origin(xyz=(-0.035, 0.0, 0.181)),
        material=satin_metal,
        name="hot_shoe",
    )
    model.articulation(
        "head_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_mount,
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.8, lower=-0.55, upper=0.75),
        mimic=Mimic(joint=handle_joint.name, multiplier=1.0, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_core = object_model.get_part("tripod_core")
    pan_head = object_model.get_part("pan_head")
    camera_mount = object_model.get_part("camera_mount")
    side_handle = object_model.get_part("side_handle")

    pan_joint = object_model.get_articulation("core_to_pan")
    handle_joint = object_model.get_articulation("head_to_handle")
    camera_joint = object_model.get_articulation("head_to_camera")

    ctx.allow_overlap(
        camera_mount,
        pan_head,
        elem_a="tilt_axle",
        elem_b="tilt_boss",
        reason="The camera tilt axle is intentionally captured inside the pan-head tilt boss.",
    )
    ctx.allow_overlap(
        camera_mount,
        pan_head,
        elem_a="tilt_axle",
        elem_b="head_body",
        reason="The solid head body is a simplified bearing housing with a through-bore for the tilt axle.",
    )
    ctx.expect_within(
        camera_mount,
        pan_head,
        axes="xz",
        inner_elem="tilt_axle",
        outer_elem="tilt_boss",
        margin=0.001,
        name="camera tilt axle stays coaxial in the boss",
    )
    ctx.expect_overlap(
        camera_mount,
        pan_head,
        axes="y",
        elem_a="tilt_axle",
        elem_b="tilt_boss",
        min_overlap=0.040,
        name="camera tilt axle remains captured across the boss",
    )
    ctx.expect_within(
        camera_mount,
        pan_head,
        axes="xz",
        inner_elem="tilt_axle",
        outer_elem="head_body",
        margin=0.001,
        name="camera tilt axle passes through the bearing housing",
    )
    ctx.expect_overlap(
        camera_mount,
        pan_head,
        axes="y",
        elem_a="tilt_axle",
        elem_b="head_body",
        min_overlap=0.040,
        name="camera tilt axle is retained by the head body",
    )

    ctx.allow_overlap(
        pan_head,
        side_handle,
        elem_a="tilt_boss",
        elem_b="pivot_collar",
        reason="The side handle collar nests onto the same hinge boss that drives the tilt head.",
    )
    ctx.expect_within(
        side_handle,
        pan_head,
        axes="xz",
        inner_elem="pivot_collar",
        outer_elem="tilt_boss",
        margin=0.001,
        name="side handle collar is coaxial with the head pivot",
    )
    ctx.expect_overlap(
        side_handle,
        pan_head,
        axes="y",
        elem_a="pivot_collar",
        elem_b="tilt_boss",
        min_overlap=0.006,
        name="side handle collar seats on the pivot boss",
    )

    ctx.check(
        "tripod has three crown leg hinges",
        all(
            object_model.get_articulation(f"crown_to_leg_{i}").articulation_type == ArticulationType.REVOLUTE
            for i in range(3)
        ),
        details="Each folding leg should use a revolute crown hinge.",
    )
    ctx.check(
        "pan head is continuous about vertical axis",
        pan_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(pan_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={pan_joint.articulation_type}, axis={pan_joint.axis}",
    )
    ctx.check(
        "side handle drives camera tilt",
        camera_joint.mimic is not None and camera_joint.mimic.joint == handle_joint.name,
        details=f"camera mimic={camera_joint.mimic}",
    )

    def _elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    lens_rest = _elem_center(camera_mount, "lens_barrel")
    with ctx.pose({pan_joint: math.pi / 2.0}):
        lens_panned = _elem_center(camera_mount, "lens_barrel")
    ctx.check(
        "pan head rotates the camera around the center column",
        lens_rest is not None
        and lens_panned is not None
        and lens_panned[1] > lens_rest[1] + 0.12
        and abs(lens_panned[0]) < 0.08,
        details=f"rest={lens_rest}, panned={lens_panned}",
    )

    lens_level = _elem_center(camera_mount, "lens_barrel")
    with ctx.pose({handle_joint: -0.40}):
        lens_tilted = _elem_center(camera_mount, "lens_barrel")
    ctx.check(
        "handle rotation tilts the device upward",
        lens_level is not None and lens_tilted is not None and lens_tilted[2] > lens_level[2] + 0.045,
        details=f"level={lens_level}, tilted={lens_tilted}",
    )

    leg_0 = object_model.get_part("leg_0")
    leg_joint = object_model.get_articulation("crown_to_leg_0")
    foot_rest = _elem_center(leg_0, "rubber_foot")
    with ctx.pose({leg_joint: 0.70}):
        foot_folded = _elem_center(leg_0, "rubber_foot")
    rest_radius = math.hypot(foot_rest[0], foot_rest[1]) if foot_rest is not None else None
    folded_radius = math.hypot(foot_folded[0], foot_folded[1]) if foot_folded is not None else None
    ctx.check(
        "crown hinge folds a leg inward",
        rest_radius is not None and folded_radius is not None and folded_radius < rest_radius - 0.20,
        details=f"rest_radius={rest_radius}, folded_radius={folded_radius}",
    )

    return ctx.report()


object_model = build_object_model()
