from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    BoltPattern,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba=rgba)


def _can_shell_mesh():
    """Hollow cylindrical Fresnel-style lamp can, authored along local +X."""
    tube = (
        cq.Workplane("YZ")
        .circle(0.176)
        .circle(0.148)
        .extrude(0.540)
        .translate((-0.270, 0.0, 0.0))
    )
    front_lip = (
        cq.Workplane("YZ")
        .circle(0.194)
        .circle(0.150)
        .extrude(0.040)
        .translate((0.245, 0.0, 0.0))
    )
    rear_cap = (
        cq.Workplane("YZ")
        .circle(0.148)
        .extrude(0.026)
        .translate((-0.296, 0.0, 0.0))
    )
    rib_0 = (
        cq.Workplane("YZ")
        .circle(0.184)
        .circle(0.148)
        .extrude(0.018)
        .translate((-0.145, 0.0, 0.0))
    )
    rib_1 = (
        cq.Workplane("YZ")
        .circle(0.184)
        .circle(0.148)
        .extrude(0.018)
        .translate((0.050, 0.0, 0.0))
    )
    return tube.union(front_lip).union(rear_cap).union(rib_0).union(rib_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yoke_stand_spotlight")

    matte_black = model.material("matte_black", rgba=(0.010, 0.010, 0.009, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.036, 0.033, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.10, 0.10, 0.095, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.55, 0.54, 0.50, 1.0))
    warm_lens = model.material("warm_fresnel_glass", rgba=(1.0, 0.78, 0.32, 0.42))

    wheel_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.043,
            0.030,
            rim=WheelRim(inner_radius=0.028, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.017,
                width=0.024,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.022, hole_diameter=0.0025),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "caster_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.034,
            inner_radius=0.043,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.0035, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.0035, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.03),
            shoulder=TireShoulder(width=0.0035, radius=0.002),
        ),
        "caster_tire",
    )

    base = model.part("base")
    base.visual(Box((0.76, 0.052, 0.036)), origin=Origin(xyz=(0.0, 0.205, 0.130)), material=dark_metal, name="front_rail")
    base.visual(Box((0.76, 0.052, 0.036)), origin=Origin(xyz=(0.0, -0.205, 0.130)), material=dark_metal, name="rear_rail")
    base.visual(Box((0.052, 0.462, 0.036)), origin=Origin(xyz=(0.342, 0.0, 0.130)), material=dark_metal, name="side_rail_0")
    base.visual(Box((0.052, 0.462, 0.036)), origin=Origin(xyz=(-0.342, 0.0, 0.130)), material=dark_metal, name="side_rail_1")
    base.visual(Box((0.680, 0.046, 0.032)), origin=Origin(xyz=(0.0, 0.0, 0.132)), material=dark_metal, name="center_crossbar_x")
    base.visual(Box((0.046, 0.380, 0.032)), origin=Origin(xyz=(0.0, 0.0, 0.132)), material=dark_metal, name="center_crossbar_y")
    base.visual(Cylinder(radius=0.125, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.165)), material=dark_metal, name="pedestal_plate")
    base.visual(Cylinder(radius=0.036, length=0.635), origin=Origin(xyz=(0.0, 0.0, 0.4925)), material=dark_metal, name="short_post")
    base.visual(Cylinder(radius=0.052, length=0.038), origin=Origin(xyz=(0.0, 0.0, 0.815)), material=brushed, name="pan_bearing_pin")

    wheel_positions = (
        (0.320, 0.205, 0.057),
        (-0.320, 0.205, 0.057),
        (0.320, -0.205, 0.057),
        (-0.320, -0.205, 0.057),
    )
    for i, (x, y, z) in enumerate(wheel_positions):
        for side in (-1.0, 1.0):
            base.visual(
                Box((0.008, 0.060, 0.078)),
                origin=Origin(xyz=(x + side * 0.026, y, 0.094)),
                material=dark_metal,
                name=f"caster_fork_{i}_{int(side > 0)}",
            )
            base.visual(
                Cylinder(radius=0.008, length=0.012),
                origin=Origin(xyz=(x + side * 0.036, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=brushed,
                name=f"axle_cap_{i}_{int(side > 0)}",
            )

        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, material=rubber, name="rubber_tire")
        wheel.visual(wheel_rim_mesh, material=brushed, name="metal_rim")
        model.articulation(
            f"base_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    yoke = model.part("yoke")
    yoke.visual(Cylinder(radius=0.056, length=0.045), origin=Origin(xyz=(0.0, 0.0, 0.0225)), material=dark_metal, name="pan_collar")
    yoke.visual(Box((0.105, 0.500, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=dark_metal, name="lower_bridge")
    yoke.visual(Box((0.060, 0.044, 0.390)), origin=Origin(xyz=(0.0, 0.235, 0.245)), material=dark_metal, name="side_arm_0")
    yoke.visual(Box((0.060, 0.044, 0.390)), origin=Origin(xyz=(0.0, -0.235, 0.245)), material=dark_metal, name="side_arm_1")
    yoke.visual(
        Cylinder(radius=0.060, length=0.046),
        origin=Origin(xyz=(0.0, 0.214, 0.245), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="tilt_bearing_0",
    )
    yoke.visual(
        Cylinder(radius=0.060, length=0.046),
        origin=Origin(xyz=(0.0, -0.214, 0.245), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="tilt_bearing_1",
    )
    model.articulation(
        "base_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.834)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.0),
    )

    can = model.part("can")
    can.visual(mesh_from_cadquery(_can_shell_mesh(), "lamp_can"), material=matte_black, name="can_shell")
    can.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.335, 0.335),
                (0.445, 0.445),
                0.030,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.020,
                outer_corner_radius=0.030,
                face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
                center=False,
            ),
            "front_barndoor_frame",
        ),
        origin=Origin(xyz=(0.280, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="front_frame",
    )
    can.visual(Box((0.014, 0.565, 0.020)), origin=Origin(xyz=(0.304, 0.0, 0.226)), material=satin_black, name="top_hinge_rail")
    can.visual(Box((0.014, 0.565, 0.020)), origin=Origin(xyz=(0.304, 0.0, -0.226)), material=satin_black, name="bottom_hinge_rail")
    can.visual(Box((0.014, 0.020, 0.565)), origin=Origin(xyz=(0.304, -0.226, 0.0)), material=satin_black, name="side_hinge_rail_0")
    can.visual(Box((0.014, 0.020, 0.565)), origin=Origin(xyz=(0.304, 0.226, 0.0)), material=satin_black, name="side_hinge_rail_1")
    can.visual(
        Cylinder(radius=0.151, length=0.006),
        origin=Origin(xyz=(0.252, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_lens,
        name="fresnel_lens",
    )
    for ring_radius in (0.060, 0.095, 0.122):
        can.visual(
            Cylinder(radius=ring_radius, length=0.004),
            origin=Origin(xyz=(0.254, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed,
            name=f"lens_ring_{int(ring_radius * 1000)}",
        )
    for y in (-0.250, 0.250):
        can.visual(
            Box((0.022, 0.060, 0.034)),
            origin=Origin(xyz=(0.312, y, 0.238)),
            material=satin_black,
            name=f"top_hinge_mount_{int(y > 0)}",
        )
        can.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(0.326, y, 0.238), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=f"top_hinge_knuckle_{int(y > 0)}",
        )
        can.visual(
            Box((0.022, 0.060, 0.034)),
            origin=Origin(xyz=(0.312, y, -0.238)),
            material=satin_black,
            name=f"bottom_hinge_mount_{int(y > 0)}",
        )
        can.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(0.326, y, -0.238), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=f"bottom_hinge_knuckle_{int(y > 0)}",
        )
    for z in (-0.250, 0.250):
        can.visual(
            Box((0.022, 0.034, 0.060)),
            origin=Origin(xyz=(0.312, -0.238, z)),
            material=satin_black,
            name=f"side0_hinge_mount_{int(z > 0)}",
        )
        can.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(0.326, -0.238, z)),
            material=brushed,
            name=f"side0_hinge_knuckle_{int(z > 0)}",
        )
        can.visual(
            Box((0.022, 0.034, 0.060)),
            origin=Origin(xyz=(0.312, 0.238, z)),
            material=satin_black,
            name=f"side1_hinge_mount_{int(z > 0)}",
        )
        can.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(0.326, 0.238, z)),
            material=brushed,
            name=f"side1_hinge_knuckle_{int(z > 0)}",
        )
    can.visual(
        Cylinder(radius=0.025, length=0.090),
        origin=Origin(xyz=(0.0, 0.205, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="trunnion_pin_0",
    )
    can.visual(
        Cylinder(radius=0.025, length=0.090),
        origin=Origin(xyz=(0.0, -0.205, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="trunnion_pin_1",
    )
    can.visual(
        Cylinder(radius=0.014, length=0.065),
        origin=Origin(xyz=(-0.3275, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed,
        name="knob_shaft",
    )
    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.75, upper=0.85),
    )

    knob = model.part("rear_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.074,
                0.044,
                body_style="lobed",
                base_diameter=0.050,
                top_diameter=0.070,
                crown_radius=0.002,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0014, width=0.002),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                bore=KnobBore(style="round", diameter=0.014),
                center=False,
            ),
            "rear_control_knob",
        ),
        origin=Origin(rpy=(0.0, -pi / 2.0, 0.0)),
        material=satin_black,
        name="knob_cap",
    )
    model.articulation(
        "can_to_rear_knob",
        ArticulationType.CONTINUOUS,
        parent=can,
        child=knob,
        origin=Origin(xyz=(-0.360, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    leaf_specs = (
        ("top_leaf", (0.0, 0.0, 1.0), (0.326, 0.0, 0.238), (0.010, 0.420, 0.165), (0.012, 0.0, 0.082), (0.0, 1.0, 0.0), -1.20, 1.20),
        ("bottom_leaf", (0.0, 0.0, -1.0), (0.326, 0.0, -0.238), (0.010, 0.420, 0.165), (0.012, 0.0, -0.082), (0.0, 1.0, 0.0), -1.20, 1.20),
        ("side_leaf_0", (0.0, -1.0, 0.0), (0.326, -0.238, 0.0), (0.010, 0.165, 0.420), (0.012, -0.082, 0.0), (0.0, 0.0, 1.0), -1.20, 1.20),
        ("side_leaf_1", (0.0, 1.0, 0.0), (0.326, 0.238, 0.0), (0.010, 0.165, 0.420), (0.012, 0.082, 0.0), (0.0, 0.0, 1.0), -1.20, 1.20),
    )
    for name, direction, hinge_xyz, panel_size, panel_xyz, axis, lower, upper in leaf_specs:
        leaf = model.part(name)
        leaf.visual(Box(panel_size), origin=Origin(xyz=panel_xyz), material=matte_black, name="black_leaf")
        if axis == (0.0, 1.0, 0.0):
            leaf.visual(
                Cylinder(radius=0.012, length=0.420),
                origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
                material=brushed,
                name="hinge_barrel",
            )
        else:
            leaf.visual(Cylinder(radius=0.012, length=0.420), material=brushed, name="hinge_barrel")
        model.articulation(
            f"can_to_{name}",
            ArticulationType.REVOLUTE,
            parent=can,
            child=leaf,
            origin=Origin(xyz=hinge_xyz),
            axis=axis,
            motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=lower, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")

    ctx.allow_overlap(
        yoke,
        can,
        elem_a="tilt_bearing_0",
        elem_b="trunnion_pin_0",
        reason="The can trunnion pin is intentionally captured inside the yoke bearing boss.",
    )
    ctx.allow_overlap(
        yoke,
        can,
        elem_a="tilt_bearing_1",
        elem_b="trunnion_pin_1",
        reason="The can trunnion pin is intentionally captured inside the yoke bearing boss.",
    )
    ctx.allow_overlap(
        yoke,
        can,
        elem_a="side_arm_0",
        elem_b="trunnion_pin_0",
        reason="The trunnion pin passes through the bored yoke arm; the arm is a simplified solid proxy around that bore.",
    )
    ctx.allow_overlap(
        yoke,
        can,
        elem_a="side_arm_1",
        elem_b="trunnion_pin_1",
        reason="The trunnion pin passes through the bored yoke arm; the arm is a simplified solid proxy around that bore.",
    )
    ctx.expect_overlap(
        yoke,
        can,
        axes="y",
        elem_a="tilt_bearing_0",
        elem_b="trunnion_pin_0",
        min_overlap=0.020,
        name="upper side trunnion retained in bearing",
    )
    ctx.expect_overlap(
        yoke,
        can,
        axes="y",
        elem_a="tilt_bearing_1",
        elem_b="trunnion_pin_1",
        min_overlap=0.020,
        name="lower side trunnion retained in bearing",
    )
    ctx.expect_overlap(
        yoke,
        can,
        axes="y",
        elem_a="side_arm_0",
        elem_b="trunnion_pin_0",
        min_overlap=0.020,
        name="trunnion penetrates first bored arm",
    )
    ctx.expect_overlap(
        yoke,
        can,
        axes="y",
        elem_a="side_arm_1",
        elem_b="trunnion_pin_1",
        min_overlap=0.020,
        name="trunnion penetrates second bored arm",
    )

    ctx.expect_gap(yoke, base, axis="z", max_gap=0.005, max_penetration=0.0, positive_elem="pan_collar", negative_elem="pan_bearing_pin", name="yoke collar sits on post bearing")
    ctx.expect_overlap(can, yoke, axes="xy", elem_a="trunnion_pin_0", elem_b="tilt_bearing_0", min_overlap=0.015, name="tilt pin shares bearing footprint")

    tilt = object_model.get_articulation("yoke_to_can")
    top_hinge = object_model.get_articulation("can_to_top_leaf")
    side_hinge = object_model.get_articulation("can_to_side_leaf_1")

    rest_front = ctx.part_element_world_aabb(can, elem="front_frame")
    with ctx.pose({tilt: 0.55}):
        raised_front = ctx.part_element_world_aabb(can, elem="front_frame")
    ctx.check(
        "tilt joint lifts front frame",
        rest_front is not None
        and raised_front is not None
        and (raised_front[0][2] + raised_front[1][2]) > (rest_front[0][2] + rest_front[1][2]) + 0.08,
        details=f"rest={rest_front}, tilted={raised_front}",
    )

    top_leaf = object_model.get_part("top_leaf")
    side_leaf = object_model.get_part("side_leaf_1")
    top_rest = ctx.part_element_world_aabb(top_leaf, elem="black_leaf")
    with ctx.pose({top_hinge: 0.65}):
        top_moved = ctx.part_element_world_aabb(top_leaf, elem="black_leaf")
    ctx.check(
        "top barndoor swings forward",
        top_rest is not None
        and top_moved is not None
        and top_moved[1][0] > top_rest[1][0] + 0.035,
        details=f"rest={top_rest}, moved={top_moved}",
    )

    side_rest = ctx.part_element_world_aabb(side_leaf, elem="black_leaf")
    with ctx.pose({side_hinge: -0.65}):
        side_moved = ctx.part_element_world_aabb(side_leaf, elem="black_leaf")
    ctx.check(
        "side barndoor swings forward",
        side_rest is not None
        and side_moved is not None
        and side_moved[1][0] > side_rest[1][0] + 0.035,
        details=f"rest={side_rest}, moved={side_moved}",
    )

    return ctx.report()


object_model = build_object_model()
