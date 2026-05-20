from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube(height: float, outer_radius: float, inner_radius: float, center_z: float) -> cq.Workplane:
    """CadQuery tube centered on the Z axis."""
    outer = cq.Workplane("XY").cylinder(height, outer_radius)
    inner = cq.Workplane("XY").cylinder(height + 0.006, inner_radius)
    return outer.cut(inner).translate((0.0, 0.0, center_z))


def _rotated_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    yaw_deg: float,
) -> cq.Workplane:
    box = cq.Workplane("XY").box(*size).translate(center)
    return box.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), yaw_deg)


def _make_crown_geometry() -> cq.Workplane:
    """Tripod crown with a hollow mast sleeve and three fork-style leg clevises."""
    crown = _tube(0.230, 0.034, 0.022, 0.505)
    crown = crown.union(_tube(0.042, 0.086, 0.026, 0.430))

    hinge_radius = 0.108
    for index in range(3):
        yaw = index * 120.0
        for side in (-1.0, 1.0):
            crown = crown.union(
                _rotated_box(
                    (0.062, 0.012, 0.048),
                    (hinge_radius, side * 0.029, 0.430),
                    yaw,
                )
            )
    return crown


def _make_pan_head_geometry() -> cq.Workplane:
    """Compact pan yoke with a clearanced collar around the sliding mast."""
    head = cq.Workplane("XY").cylinder(0.024, 0.052).translate((0.0, 0.0, 0.012))

    for side in (-1.0, 1.0):
        head = head.union(
            cq.Workplane("XY")
            .box(0.030, 0.090, 0.028)
            .translate((0.0, side * 0.045, 0.038))
        )
        head = head.union(
            cq.Workplane("XY")
            .box(0.034, 0.014, 0.132)
            .translate((0.0, side * 0.097, 0.090))
        )
    return head


def _make_device_body_geometry() -> cq.Workplane:
    """Rounded rectangular optical body, authored as one manufactured shell."""
    body = cq.Workplane("XY").box(0.220, 0.160, 0.112).edges("|Z").fillet(0.012)
    body = body.edges(">X").fillet(0.008)
    return body.translate((0.020, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_tripod_optic")

    matte_black = Material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = Material("graphite_body", rgba=(0.08, 0.085, 0.09, 1.0))
    anodized = Material("dark_anodized_metal", rgba=(0.035, 0.038, 0.045, 1.0))
    rubber = Material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    glass = Material("blue_coated_glass", rgba=(0.08, 0.20, 0.34, 0.72))
    cover_mat = Material("protective_cover", rgba=(0.025, 0.026, 0.029, 1.0))

    model.materials.extend([matte_black, graphite, anodized, rubber, glass, cover_mat])

    crown = model.part("crown")
    crown.visual(
        mesh_from_cadquery(_make_crown_geometry(), "crown"),
        material=anodized,
        name="crown_shell",
    )
    # Three small clamp screws make the sleeve read as a real sliding mast crown.
    for index in range(3):
        yaw = index * 2.0 * math.pi / 3.0 + math.radians(35.0)
        crown.visual(
            Cylinder(radius=0.006, length=0.046),
            origin=Origin(
                xyz=(0.037 * math.cos(yaw), 0.037 * math.sin(yaw), 0.545),
                rpy=(0.0, math.pi / 2.0, yaw),
            ),
            material=matte_black,
            name=f"clamp_screw_{index}",
        )
    for index in range(3):
        yaw = index * 2.0 * math.pi / 3.0
        crown.visual(
            Cylinder(radius=0.005, length=0.078),
            origin=Origin(
                xyz=(0.108 * math.cos(yaw), 0.108 * math.sin(yaw), 0.430),
                rpy=(-math.pi / 2.0, 0.0, yaw),
            ),
            material=anodized,
            name=f"hinge_pin_{index}",
        )

    leg_vector = (0.320, 0.0, -0.410)
    leg_length = math.sqrt(leg_vector[0] ** 2 + leg_vector[2] ** 2)
    leg_pitch = math.atan2(leg_vector[0], leg_vector[2])
    leg_unit = (leg_vector[0] / leg_length, 0.0, leg_vector[2] / leg_length)
    strut_start = 0.016
    strut_length = leg_length - strut_start
    strut_center_distance = strut_start + strut_length / 2.0
    for index in range(3):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.016, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=anodized,
            name="hinge_knuckle",
        )
        leg.visual(
            Cylinder(radius=0.010, length=strut_length),
            origin=Origin(
                xyz=(leg_unit[0] * strut_center_distance, 0.0, leg_unit[2] * strut_center_distance),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=matte_black,
            name="carbon_strut",
        )
        leg.visual(
            Cylinder(radius=0.013, length=0.055),
            origin=Origin(
                xyz=(0.115, 0.0, -0.148),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=anodized,
            name="upper_collar",
        )
        leg.visual(
            Cylinder(radius=0.013, length=0.052),
            origin=Origin(
                xyz=(0.238, 0.0, -0.305),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=anodized,
            name="lower_collar",
        )
        leg.visual(
            Box((0.082, 0.040, 0.018)),
            origin=Origin(xyz=(0.340, 0.0, -0.421), rpy=(0.0, 0.0, 0.0)),
            material=rubber,
            name="rubber_foot",
        )

        yaw = index * 2.0 * math.pi / 3.0
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(0.108 * math.cos(yaw), 0.108 * math.sin(yaw), 0.430),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=-0.15, upper=0.65),
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.016, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=anodized,
        name="sliding_tube",
    )
    mast.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=matte_black,
        name="top_plug",
    )
    model.articulation(
        "crown_to_mast",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.160),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        mesh_from_cadquery(_make_pan_head_geometry(), "pan_head"),
        material=anodized,
        name="pan_yoke",
    )
    model.articulation(
        "mast_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0),
    )

    device = model.part("device")
    device.visual(
        mesh_from_cadquery(_make_device_body_geometry(), "device_body"),
        material=graphite,
        name="body_shell",
    )
    device.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.139, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="front_lens_ring",
    )
    device.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.147, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    device.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(-0.104, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="rear_eyepiece",
    )
    for side in (-1.0, 1.0):
        device.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(0.0, side * 0.081, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=anodized,
            name=f"tilt_pin_{0 if side < 0 else 1}",
        )
        device.visual(
            Box((0.030, 0.014, 0.016)),
            origin=Origin(xyz=(0.141, side * 0.055, 0.062)),
            material=anodized,
            name=f"cover_hinge_mount_{0 if side < 0 else 1}",
        )

    model.articulation(
        "pan_head_to_device",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=device,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.1, lower=-0.55, upper=0.85),
    )

    cover = model.part("front_cover")
    cover.visual(
        Box((0.008, 0.148, 0.106)),
        origin=Origin(xyz=(0.004, 0.0, -0.053)),
        material=cover_mat,
        name="cover_panel",
    )
    cover.visual(
        Cylinder(radius=0.007, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="hinge_barrel",
    )
    cover.visual(
        Box((0.012, 0.120, 0.012)),
        origin=Origin(xyz=(0.006, 0.0, -0.103)),
        material=rubber,
        name="soft_lip",
    )
    model.articulation(
        "device_to_front_cover",
        ArticulationType.REVOLUTE,
        parent=device,
        child=cover,
        origin=Origin(xyz=(0.156, 0.0, 0.063)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.8, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    mast = object_model.get_part("mast")
    pan_head = object_model.get_part("pan_head")
    device = object_model.get_part("device")
    cover = object_model.get_part("front_cover")

    mast_slide = object_model.get_articulation("crown_to_mast")
    pan = object_model.get_articulation("mast_to_pan_head")
    tilt = object_model.get_articulation("pan_head_to_device")
    cover_hinge = object_model.get_articulation("device_to_front_cover")

    for index in range(3):
        leg = object_model.get_part(f"leg_{index}")
        ctx.allow_overlap(
            crown,
            leg,
            elem_a=f"hinge_pin_{index}",
            elem_b="hinge_knuckle",
            reason="The visible crown hinge pin is intentionally captured through the leg knuckle.",
        )
        ctx.expect_overlap(
            crown,
            leg,
            axes="xyz",
            elem_a=f"hinge_pin_{index}",
            elem_b="hinge_knuckle",
            min_overlap=0.006,
            name=f"leg {index} knuckle is captured by its crown hinge pin",
        )

    for index in range(3):
        ctx.allow_overlap(
            crown,
            mast,
            elem_a=f"clamp_screw_{index}",
            elem_b="sliding_tube",
            reason="The clamp screw tip is modeled lightly biting the sliding mast to support the retained prismatic fit.",
        )
        ctx.expect_overlap(
            crown,
            mast,
            axes="xyz",
            elem_a=f"clamp_screw_{index}",
            elem_b="sliding_tube",
            min_overlap=0.002,
            name=f"clamp screw {index} reaches the sliding mast",
        )

    for index in range(2):
        ctx.allow_overlap(
            device,
            pan_head,
            elem_a=f"tilt_pin_{index}",
            elem_b="pan_yoke",
            reason="The device tilt pin is intentionally seated in the side yoke bearing.",
        )
        ctx.expect_overlap(
            device,
            pan_head,
            axes="xyz",
            elem_a=f"tilt_pin_{index}",
            elem_b="pan_yoke",
            min_overlap=0.004,
            name=f"tilt pin {index} is seated in the pan yoke",
        )

    ctx.expect_within(
        mast,
        crown,
        axes="xy",
        inner_elem="sliding_tube",
        outer_elem="crown_shell",
        margin=0.002,
        name="mast is centered in the crown sleeve",
    )
    ctx.expect_overlap(
        mast,
        crown,
        axes="z",
        elem_a="sliding_tube",
        elem_b="crown_shell",
        min_overlap=0.09,
        name="collapsed mast remains retained in sleeve",
    )
    with ctx.pose({mast_slide: 0.160}):
        ctx.expect_overlap(
            mast,
            crown,
            axes="z",
            elem_a="sliding_tube",
            elem_b="crown_shell",
            min_overlap=0.05,
            name="extended mast keeps retained insertion",
        )
        extended_mast = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.0}):
        rest_mast = ctx.part_world_position(mast)
    ctx.check(
        "mast translates upward on prismatic joint",
        rest_mast is not None and extended_mast is not None and extended_mast[2] > rest_mast[2] + 0.14,
        details=f"rest={rest_mast}, extended={extended_mast}",
    )

    with ctx.pose({pan: math.pi / 2.0}):
        panned_device = ctx.part_world_aabb(device)
    with ctx.pose({pan: 0.0}):
        forward_device = ctx.part_world_aabb(device)
    ctx.check(
        "head pans around the vertical mast axis",
        forward_device is not None
        and panned_device is not None
        and panned_device[1][1] > forward_device[1][1] + 0.045,
        details=f"forward={forward_device}, panned={panned_device}",
    )

    leg_hinge = object_model.get_articulation("crown_to_leg_0")
    leg_0 = object_model.get_part("leg_0")
    with ctx.pose({leg_hinge: 0.65}):
        folded_leg = ctx.part_world_aabb(leg_0)
    with ctx.pose({leg_hinge: 0.0}):
        splayed_leg = ctx.part_world_aabb(leg_0)
    ctx.check(
        "tripod leg folds inward on the crown hinge",
        splayed_leg is not None
        and folded_leg is not None
        and folded_leg[1][0] < splayed_leg[1][0] - 0.10,
        details=f"splayed={splayed_leg}, folded={folded_leg}",
    )

    with ctx.pose({tilt: 0.65}):
        tilted_device = ctx.part_world_aabb(device)
    with ctx.pose({tilt: 0.0}):
        level_device = ctx.part_world_aabb(device)
    ctx.check(
        "device tilt raises the optical face",
        level_device is not None
        and tilted_device is not None
        and tilted_device[1][2] > level_device[1][2] + 0.02,
        details=f"level={level_device}, tilted={tilted_device}",
    )

    ctx.expect_gap(
        cover,
        device,
        axis="x",
        min_gap=0.001,
        max_gap=0.020,
        positive_elem="cover_panel",
        negative_elem="front_glass",
        name="closed cover sits just proud of front glass",
    )
    with ctx.pose({cover_hinge: 1.45}):
        opened_cover = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 0.0}):
        closed_cover = ctx.part_world_aabb(cover)
    ctx.check(
        "front cover rotates upward on hinge",
        closed_cover is not None
        and opened_cover is not None
        and opened_cover[0][2] > closed_cover[0][2] + 0.03,
        details=f"closed={closed_cover}, opened={opened_cover}",
    )

    ctx.expect_contact(
        pan_head,
        device,
        elem_a="pan_yoke",
        elem_b="tilt_pin_1",
        contact_tol=0.002,
        name="side yoke captures the device tilt pin",
    )

    return ctx.report()


object_model = build_object_model()
