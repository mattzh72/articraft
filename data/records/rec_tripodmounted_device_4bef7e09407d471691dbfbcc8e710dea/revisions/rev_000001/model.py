from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_segment(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_optical_device")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    olive = model.material("olive", rgba=(0.33, 0.36, 0.25, 1.0))
    glass = model.material("glass", rgba=(0.52, 0.68, 0.80, 0.45))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.044, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.522)),
        material=graphite,
        name="hub",
    )
    crown.visual(
        Box((0.108, 0.108, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.549)),
        material=dark_metal,
        name="crown_plate",
    )
    post_data = (
        ((0.020, 0.0, 0.640), (0.012, 0.028, 0.168), "guide_x_pos"),
        ((-0.020, 0.0, 0.640), (0.012, 0.028, 0.168), "guide_x_neg"),
        ((0.0, 0.020, 0.640), (0.028, 0.012, 0.168), "guide_y_pos"),
        ((0.0, -0.020, 0.640), (0.028, 0.012, 0.168), "guide_y_neg"),
    )
    for xyz, size, name in post_data:
        crown.visual(Box(size), origin=Origin(xyz=xyz), material=dark_metal, name=name)
    crown.visual(
        Cylinder(radius=0.0038, length=0.028),
        origin=Origin(xyz=(0.038, 0.0, 0.616), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="clamp_stem",
    )
    crown.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.056, 0.0, 0.616)),
        material=matte_black,
        name="clamp_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        crown.visual(
            Box((0.032, 0.018, 0.012)),
            origin=Origin(xyz=(0.050 * c, 0.050 * s, 0.533), rpy=(0.0, 0.0, angle)),
            material=graphite,
            name=f"hinge_pad_{index}",
        )

        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.0065, length=0.020),
            origin=Origin(
                xyz=(0.0, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, angle + math.pi / 2.0),
            ),
            material=dark_metal,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.020, 0.016, 0.030)),
            origin=Origin(xyz=(0.018 * c, 0.018 * s, -0.007), rpy=(0.0, 0.0, angle)),
            material=dark_metal,
            name="shoulder",
        )

        upper_a = (0.018 * c, 0.018 * s, -0.007)
        upper_b = (0.122 * c, 0.122 * s, -0.240)
        lower_b = (0.238 * c, 0.238 * s, -0.508)
        foot_center = (0.248 * c, 0.248 * s, -0.520)

        _add_segment(
            leg,
            upper_a,
            upper_b,
            radius=0.011,
            material=graphite,
            name="upper_tube",
        )
        leg.visual(
            Cylinder(radius=0.013, length=0.018),
            origin=Origin(
                xyz=upper_b,
                rpy=(math.pi / 2.0, 0.0, angle + math.pi / 2.0),
            ),
            material=dark_metal,
            name="knee_collar",
        )
        _add_segment(
            leg,
            upper_b,
            lower_b,
            radius=0.0085,
            material=dark_metal,
            name="lower_tube",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.026),
            origin=Origin(
                xyz=(
                    (lower_b[0] + foot_center[0]) * 0.5,
                    (lower_b[1] + foot_center[1]) * 0.5,
                    (lower_b[2] + foot_center[2]) * 0.5,
                ),
                rpy=_rpy_for_cylinder(lower_b, foot_center),
            ),
            material=rubber,
            name="foot",
        )
        leg.visual(
            Sphere(radius=0.010),
            origin=Origin(xyz=foot_center),
            material=rubber,
            name="toe",
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(0.058 * c, 0.058 * s, 0.520)),
            axis=(-s, c, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=1.6,
                lower=-0.18,
                upper=0.90,
            ),
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.014, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=graphite,
        name="shaft",
    )
    mast.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.329)),
        material=dark_metal,
        name="head_cap",
    )

    model.articulation(
        "crown_to_mast",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.562)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.16,
            lower=0.0,
            upper=0.120,
        ),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="pan_base",
    )
    pan_head.visual(
        Box((0.032, 0.040, 0.050)),
        origin=Origin(xyz=(-0.004, 0.0, 0.043)),
        material=dark_metal,
        name="column",
    )
    pan_head.visual(
        Box((0.014, 0.088, 0.040)),
        origin=Origin(xyz=(0.000, 0.0, 0.060)),
        material=graphite,
        name="bridge",
    )
    for sign, suffix in ((1.0, "pos"), (-1.0, "neg")):
        pan_head.visual(
            Box((0.080, 0.008, 0.040)),
            origin=Origin(xyz=(0.038, 0.040 * sign, 0.052)),
            material=graphite,
            name=f"arm_{suffix}",
        )

    model.articulation(
        "mast_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.338)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5),
    )

    device = model.part("device")
    device.visual(
        Box((0.120, 0.060, 0.070)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=olive,
        name="body",
    )
    device.visual(
        Box((0.050, 0.022, 0.018)),
        origin=Origin(xyz=(-0.004, 0.0, 0.034)),
        material=graphite,
        name="top_ridge",
    )
    device.visual(
        Box((0.050, 0.042, 0.010)),
        origin=Origin(xyz=(0.006, 0.0, -0.040)),
        material=dark_metal,
        name="mount_shoe",
    )
    device.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.096, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="objective",
    )
    device.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    device.visual(
        Cylinder(radius=0.023, length=0.002),
        origin=Origin(xyz=(0.113, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    device.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(-0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="eyepiece",
    )
    device.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="eyecup",
    )
    for sign, suffix in ((1.0, "pos"), (-1.0, "neg")):
        device.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(
                xyz=(0.000, 0.033 * sign, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"trunnion_{suffix}",
        )
        device.visual(
            Box((0.036, 0.010, 0.010)),
            origin=Origin(xyz=(0.094, 0.022 * sign, 0.036)),
            material=dark_metal,
            name=f"cover_block_{suffix}",
        )

    model.articulation(
        "pan_head_to_device",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=device,
        origin=Origin(xyz=(0.078, 0.0, 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.75,
            upper=0.95,
        ),
    )

    cover = model.part("cover")
    cover.visual(
        Cylinder(radius=0.0045, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_sleeve",
    )
    cover.visual(
        Box((0.004, 0.058, 0.064)),
        origin=Origin(xyz=(0.003, 0.0, -0.032)),
        material=matte_black,
        name="cover_plate",
    )
    cover.visual(
        Box((0.010, 0.022, 0.010)),
        origin=Origin(xyz=(0.006, 0.0, -0.004)),
        material=matte_black,
        name="pull_tab",
    )

    model.articulation(
        "device_to_cover",
        ArticulationType.REVOLUTE,
        parent=device,
        child=cover,
        origin=Origin(xyz=(0.115, 0.0, 0.035)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    device = object_model.get_part("device")
    cover = object_model.get_part("cover")
    leg_0 = object_model.get_part("leg_0")

    mast_slide = object_model.get_articulation("crown_to_mast")
    leg_hinge = object_model.get_articulation("crown_to_leg_0")
    pan_joint = object_model.get_articulation("mast_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_device")
    cover_joint = object_model.get_articulation("device_to_cover")

    ctx.expect_gap(
        cover,
        device,
        axis="x",
        positive_elem="cover_plate",
        negative_elem="front_glass",
        min_gap=0.0005,
        max_gap=0.008,
        name="cover closes just ahead of the front glass",
    )
    ctx.expect_overlap(
        cover,
        device,
        axes="yz",
        elem_a="cover_plate",
        elem_b="body",
        min_overlap=0.045,
        name="closed cover spans the device face",
    )

    rest_pan_pos = ctx.part_world_position(pan_head)
    with ctx.pose({mast_slide: 0.120}):
        extended_pan_pos = ctx.part_world_position(pan_head)
    ctx.check(
        "mast extends upward",
        rest_pan_pos is not None
        and extended_pan_pos is not None
        and extended_pan_pos[2] > rest_pan_pos[2] + 0.10,
        details=f"rest={rest_pan_pos}, extended={extended_pan_pos}",
    )

    rest_foot = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))
    with ctx.pose({leg_hinge: 0.40}):
        folded_foot = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))
    rest_radius = None if rest_foot is None else math.hypot(rest_foot[0], rest_foot[1])
    folded_radius = None if folded_foot is None else math.hypot(folded_foot[0], folded_foot[1])
    ctx.check(
        "tripod leg folds toward the mast",
        rest_foot is not None
        and folded_foot is not None
        and rest_radius is not None
        and folded_radius is not None
        and folded_radius < rest_radius - 0.18
        and folded_foot[0] < rest_foot[0] - 0.18,
        details=f"rest_foot={rest_foot}, folded_foot={folded_foot}",
    )

    rest_device_pos = ctx.part_world_position(device)
    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_device_pos = ctx.part_world_position(device)
    ctx.check(
        "pan head yaws the device around the mast",
        rest_device_pos is not None
        and panned_device_pos is not None
        and panned_device_pos[1] > rest_device_pos[0] - 0.01
        and panned_device_pos[0] < rest_device_pos[0] - 0.02,
        details=f"rest={rest_device_pos}, panned={panned_device_pos}",
    )

    rest_front = _aabb_center(ctx.part_element_world_aabb(device, elem="front_glass"))
    with ctx.pose({tilt_joint: 0.70}):
        tilted_front = _aabb_center(ctx.part_element_world_aabb(device, elem="front_glass"))
    ctx.check(
        "device tilts upward",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.05,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    rest_cover = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_plate"))
    with ctx.pose({cover_joint: 1.35}):
        open_cover = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_plate"))
    ctx.check(
        "front cover opens upward and outward",
        rest_cover is not None
        and open_cover is not None
        and open_cover[0] > rest_cover[0] + 0.020
        and open_cover[2] > rest_cover[2] + 0.020,
        details=f"rest_cover={rest_cover}, open_cover={open_cover}",
    )

    ctx.check(
        "crown remains the sole structural root",
        len(object_model.root_parts()) == 1 and object_model.root_parts()[0].name == crown.name,
        details=f"roots={[part.name for part in object_model.root_parts()]}",
    )

    return ctx.report()


object_model = build_object_model()
