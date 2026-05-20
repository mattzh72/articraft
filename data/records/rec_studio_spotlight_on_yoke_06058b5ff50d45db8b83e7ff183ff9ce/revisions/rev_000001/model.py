from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_z(z_min: float, z_max: float, outer_r: float, inner_r: float) -> cq.Workplane:
    """A hollow vertical tube/ring in meters."""
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def _annular_x(x_min: float, x_max: float, outer_r: float, inner_r: float) -> cq.Workplane:
    """A hollow tube/ring whose axis is the local X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(x_max - x_min)
        .translate((x_min, 0.0, 0.0))
    )


def _rotated_box(
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    yaw_rad: float,
) -> cq.Workplane:
    """CadQuery box centered at a local point, then yawed around the global Z axis."""
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2])
        .translate(center)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(yaw_rad))
    )


def _tripod_sleeve_shape() -> cq.Workplane:
    """One connected hollow sleeve/crown mesh with three clevis-style leg brackets."""
    shape = _annular_z(0.66, 1.16, 0.050, 0.035)
    shape = shape.union(_annular_z(0.615, 0.735, 0.090, 0.035))
    shape = shape.union(_annular_z(1.10, 1.17, 0.063, 0.035))

    # Three crown hinge clevises.  Each clevis has two tangential leaves with a
    # narrow gap for the moving leg knuckle, rather than one solid block.
    for i in range(3):
        yaw = 2.0 * math.pi * i / 3.0
        for y in (-0.0215, 0.0215):
            # The strap attaches each leaf to the crown; the two outer pads leave
            # an actual pin hole around the leg knuckle instead of occupying it.
            shape = shape.union(_rotated_box((0.098, y, 0.680), (0.024, 0.017, 0.056), yaw))
            shape = shape.union(_rotated_box((0.123, y, 0.7015), (0.050, 0.017, 0.017), yaw))
            shape = shape.union(_rotated_box((0.123, y, 0.6585), (0.050, 0.017, 0.017), yaw))

    return shape


def _lamp_can_shape() -> cq.Workplane:
    """Black cylindrical can with open front bezel and rear socket bushing."""
    shell = _annular_x(-0.225, 0.225, 0.155, 0.136)
    shell = shell.union(_annular_x(0.212, 0.245, 0.167, 0.130))
    shell = shell.union(_annular_x(-0.238, -0.225, 0.148, 0.014))
    shell = shell.union(_annular_x(-0.255, -0.238, 0.028, 0.012))
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yoke_spotlight_tripod")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.012, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.033, 0.030, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.10, 0.105, 0.11, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.48, 0.50, 0.51, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    glass = model.material("warm_lens_glass", rgba=(1.0, 0.78, 0.28, 0.55))
    emitter = model.material("warm_led", rgba=(1.0, 0.92, 0.48, 1.0))
    white_mark = model.material("white_marking", rgba=(0.92, 0.90, 0.84, 1.0))

    tripod_sleeve = model.part("tripod_sleeve")
    tripod_sleeve.visual(
        mesh_from_cadquery(_tripod_sleeve_shape(), "tripod_sleeve_shell", tolerance=0.0015),
        material=dark_metal,
        name="sleeve_shell",
    )
    for i in range(3):
        yaw = 2.0 * math.pi * i / 3.0
        tripod_sleeve.visual(
            Box((0.012, 0.018, 0.030)),
            origin=Origin(
                xyz=(0.034 * math.cos(yaw), 0.034 * math.sin(yaw), 1.125),
                rpy=(0.0, 0.0, yaw),
            ),
            material=brushed,
            name=f"guide_pad_{i}",
        )

    leg_axis_angle = math.radians(150.0)
    leg_dir_x = math.sin(leg_axis_angle)
    leg_dir_z = math.cos(leg_axis_angle)
    leg_length = 0.720
    leg_start = 0.035
    leg_center_dist = leg_start + leg_length * 0.5
    leg_center = (leg_dir_x * leg_center_dist, 0.0, leg_dir_z * leg_center_dist)
    foot_dist = leg_start + leg_length
    foot_center = (leg_dir_x * foot_dist, 0.0, leg_dir_z * foot_dist + 0.006)

    for i in range(3):
        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.013, length=0.026),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name="hinge_knuckle",
        )
        leg.visual(
            Box((0.052, 0.010, 0.062)),
            origin=Origin(xyz=(0.026, 0.0, -0.031)),
            material=brushed,
            name="leg_web",
        )
        leg.visual(
            Cylinder(radius=0.016, length=leg_length),
            origin=Origin(xyz=leg_center, rpy=(0.0, leg_axis_angle, 0.0)),
            material=dark_metal,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.043, length=0.018),
            origin=Origin(xyz=foot_center),
            material=rubber,
            name="rubber_foot",
        )

        yaw = 2.0 * math.pi * i / 3.0
        hinge_radius = 0.130
        model.articulation(
            f"sleeve_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=tripod_sleeve,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(yaw), hinge_radius * math.sin(yaw), 0.680),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=-0.45, upper=0.65),
        )

    center_mast = model.part("center_mast")
    center_mast.visual(
        Cylinder(radius=0.028, length=1.120),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=brushed,
        name="mast_tube",
    )
    center_mast.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.405)),
        material=brushed,
        name="lower_bushing",
    )
    center_mast.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=brushed,
        name="upper_bushing",
    )
    center_mast.visual(
        Cylinder(radius=0.046, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.663)),
        material=dark_metal,
        name="head_cap",
    )
    model.articulation(
        "sleeve_to_mast",
        ArticulationType.PRISMATIC,
        parent=tripod_sleeve,
        child=center_mast,
        origin=Origin(xyz=(0.0, 0.0, 1.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.22, lower=0.0, upper=0.350),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.026, length=0.172),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_metal,
        name="pan_post",
    )
    yoke.visual(
        Box((0.060, 0.500, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        material=dark_metal,
        name="lower_bridge",
    )
    for side, y, arm_name, boss_name in (
        (0, -0.215, "side_arm_0", "tilt_boss_0"),
        (1, 0.215, "side_arm_1", "tilt_boss_1"),
    ):
        yoke.visual(
            Box((0.072, 0.026, 0.390)),
            origin=Origin(xyz=(0.0, y, 0.345)),
            material=dark_metal,
            name=arm_name,
        )
        yoke.visual(
            Cylinder(radius=0.047, length=0.026),
            origin=Origin(xyz=(0.0, y, 0.420), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=boss_name,
        )
    model.articulation(
        "mast_to_yoke",
        ArticulationType.REVOLUTE,
        parent=center_mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )

    lamp_can = model.part("lamp_can")
    lamp_can.visual(
        mesh_from_cadquery(_lamp_can_shape(), "lamp_can_shell", tolerance=0.001),
        material=matte_black,
        name="can_shell",
    )
    lamp_can.visual(
        Cylinder(radius=0.132, length=0.010),
        origin=Origin(xyz=(0.236, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    lamp_can.visual(
        Sphere(radius=0.030),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=emitter,
        name="led_emitter",
    )
    lamp_can.visual(
        Cylinder(radius=0.022, length=0.404),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="tilt_trunnion",
    )
    for side, y in enumerate((-0.184, 0.184)):
        lamp_can.visual(
            Cylinder(radius=0.035, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name=f"side_washer_{side}",
        )
    # Fixed rear mode markings around the bushing: small surface decals on the can body.
    for j, (yy, zz, sy, sz) in enumerate(
        (
            (0.000, 0.058, 0.010, 0.032),
            (0.000, -0.058, 0.010, 0.032),
            (0.058, 0.000, 0.032, 0.010),
            (-0.058, 0.000, 0.032, 0.010),
        )
    ):
        lamp_can.visual(
            Box((0.003, sy, sz)),
            origin=Origin(xyz=(-0.239, yy, zz)),
            material=white_mark,
            name=f"mode_mark_{j}",
        )
    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_can,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.65, upper=1.05),
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        Cylinder(radius=0.009, length=0.052),
        origin=Origin(xyz=(-0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="dial_shaft",
    )
    mode_dial.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(-0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="dial_shoulder",
    )
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.072,
            0.030,
            body_style="faceted",
            edge_radius=0.0008,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0014, width=0.0022),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        ),
        "rear_mode_dial",
    )
    mode_dial.visual(
        dial_mesh,
        origin=Origin(xyz=(-0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="dial_cap",
    )
    mode_dial.visual(
        Box((0.003, 0.008, 0.043)),
        origin=Origin(xyz=(-0.080, 0.0, 0.010)),
        material=white_mark,
        name="dial_pointer",
    )
    model.articulation(
        "can_to_dial",
        ArticulationType.CONTINUOUS,
        parent=lamp_can,
        child=mode_dial,
        origin=Origin(xyz=(-0.255, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod_sleeve = object_model.get_part("tripod_sleeve")
    center_mast = object_model.get_part("center_mast")
    yoke = object_model.get_part("yoke")
    lamp_can = object_model.get_part("lamp_can")
    mode_dial = object_model.get_part("mode_dial")

    mast_slide = object_model.get_articulation("sleeve_to_mast")
    pan_joint = object_model.get_articulation("mast_to_yoke")
    tilt_joint = object_model.get_articulation("yoke_to_can")
    dial_joint = object_model.get_articulation("can_to_dial")

    def coord(vec, idx: int) -> float:
        return vec[idx] if hasattr(vec, "__getitem__") else (vec.x, vec.y, vec.z)[idx]

    def aabb_center(aabb, idx: int) -> float:
        return 0.5 * (coord(aabb[0], idx) + coord(aabb[1], idx))

    ctx.check(
        "primary joints are authored",
        mast_slide.articulation_type == ArticulationType.PRISMATIC
        and pan_joint.articulation_type == ArticulationType.REVOLUTE
        and tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected prismatic mast, pan hinge, tilt hinge, and continuous rear dial.",
    )
    ctx.check(
        "tripod has three crown leg hinges",
        all(
            object_model.get_articulation(f"sleeve_to_leg_{i}").articulation_type
            == ArticulationType.REVOLUTE
            for i in range(3)
        ),
        details="Every tripod leg should rotate on a crown hinge.",
    )

    ctx.expect_within(
        center_mast,
        tripod_sleeve,
        axes="xy",
        inner_elem="mast_tube",
        outer_elem="sleeve_shell",
        margin=0.004,
        name="mast is centered in the tripod sleeve",
    )
    ctx.expect_overlap(
        center_mast,
        tripod_sleeve,
        axes="z",
        elem_a="mast_tube",
        elem_b="sleeve_shell",
        min_overlap=0.18,
        name="collapsed mast remains inserted in the sleeve",
    )

    rest_mast_pos = ctx.part_world_position(center_mast)
    with ctx.pose({mast_slide: 0.350}):
        ctx.expect_within(
            center_mast,
            tripod_sleeve,
            axes="xy",
            inner_elem="mast_tube",
            outer_elem="sleeve_shell",
            margin=0.004,
            name="extended mast stays centered in the sleeve",
        )
        ctx.expect_overlap(
            center_mast,
            tripod_sleeve,
            axes="z",
            elem_a="mast_tube",
            elem_b="sleeve_shell",
            min_overlap=0.10,
            name="extended mast retains insertion in the sleeve",
        )
        extended_mast_pos = ctx.part_world_position(center_mast)

    ctx.check(
        "mast slide extends upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.30,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        ctx.expect_contact(
            leg,
            tripod_sleeve,
            elem_a="hinge_knuckle",
            elem_b="sleeve_shell",
            contact_tol=0.004,
            name=f"leg {i} knuckle seats in crown hinge",
        )
        foot_aabb = ctx.part_element_world_aabb(leg, elem="rubber_foot")
        ctx.check(
            f"leg {i} reaches the floor",
            foot_aabb is not None and -0.010 <= coord(foot_aabb[0], 2) <= 0.030,
            details=f"foot_aabb={foot_aabb}",
        )

    ctx.expect_contact(
        lamp_can,
        yoke,
        elem_a="tilt_trunnion",
        elem_b="tilt_boss_0",
        contact_tol=0.004,
        name="can trunnion seats in one yoke boss",
    )
    ctx.expect_contact(
        lamp_can,
        yoke,
        elem_a="tilt_trunnion",
        elem_b="tilt_boss_1",
        contact_tol=0.004,
        name="can trunnion seats in opposite yoke boss",
    )

    front_rest = ctx.part_element_world_aabb(lamp_can, elem="front_lens")
    with ctx.pose({pan_joint: 0.75}):
        front_panned = ctx.part_element_world_aabb(lamp_can, elem="front_lens")
    ctx.check(
        "yoke pan swings the lamp around the vertical head joint",
        front_rest is not None
        and front_panned is not None
        and abs(aabb_center(front_panned, 1) - aabb_center(front_rest, 1)) > 0.12,
        details=f"rest={front_rest}, panned={front_panned}",
    )

    with ctx.pose({tilt_joint: 0.70}):
        front_tilted = ctx.part_element_world_aabb(lamp_can, elem="front_lens")
    ctx.check(
        "positive can tilt raises the front lens",
        front_rest is not None
        and front_tilted is not None
        and aabb_center(front_tilted, 2) > aabb_center(front_rest, 2) + 0.10,
        details=f"rest={front_rest}, tilted={front_tilted}",
    )

    ctx.expect_gap(
        lamp_can,
        mode_dial,
        axis="x",
        positive_elem="can_shell",
        negative_elem="dial_cap",
        min_gap=0.015,
        name="mode dial sits behind the rear can body",
    )
    pointer_rest = ctx.part_element_world_aabb(mode_dial, elem="dial_pointer")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        pointer_rotated = ctx.part_element_world_aabb(mode_dial, elem="dial_pointer")
    ctx.check(
        "rear mode dial rotates continuously about its short shaft",
        pointer_rest is not None
        and pointer_rotated is not None
        and abs(aabb_center(pointer_rotated, 1) - aabb_center(pointer_rest, 1)) > 0.006,
        details=f"rest={pointer_rest}, rotated={pointer_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
