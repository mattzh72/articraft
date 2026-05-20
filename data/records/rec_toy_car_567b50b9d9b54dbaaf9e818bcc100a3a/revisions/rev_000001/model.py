from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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


def _lower_body_shell() -> cq.Workplane:
    """Toy coupe body with open side wheel-arch cutouts."""

    length = 0.420
    half_width = 0.095
    bottom_z = 0.043
    arch_r = 0.052
    front_cx = 0.133
    rear_cx = -0.135

    side_profile = (
        cq.Workplane("XZ")
        .moveTo(-length / 2, bottom_z)
        .lineTo(rear_cx - arch_r, bottom_z)
        .threePointArc((rear_cx, bottom_z + arch_r), (rear_cx + arch_r, bottom_z))
        .lineTo(front_cx - arch_r, bottom_z)
        .threePointArc((front_cx, bottom_z + arch_r), (front_cx + arch_r, bottom_z))
        .lineTo(length / 2, bottom_z)
        .lineTo(0.205, 0.104)
        .lineTo(0.066, 0.113)
        .lineTo(0.025, 0.123)
        .lineTo(-0.105, 0.116)
        .lineTo(-0.205, 0.105)
        .close()
        .extrude(2.0 * half_width)
        .translate((0.0, half_width, 0.0))
    )

    floor = (
        cq.Workplane("XY")
        .box(0.415, 0.198, 0.020)
        .translate((0.0, 0.0, 0.044))
    )
    hood = (
        cq.Workplane("XY")
        .box(0.145, 0.146, 0.048)
        .translate((0.136, 0.0, 0.081))
    )
    trunk = (
        cq.Workplane("XY")
        .box(0.116, 0.146, 0.052)
        .translate((-0.154, 0.0, 0.083))
    )
    cowl = (
        cq.Workplane("XY")
        .box(0.110, 0.140, 0.034)
        .translate((0.010, 0.0, 0.088))
    )

    cabin = (
        cq.Workplane("XZ")
        .moveTo(-0.098, 0.107)
        .lineTo(0.064, 0.107)
        .lineTo(0.034, 0.151)
        .lineTo(-0.048, 0.158)
        .lineTo(-0.091, 0.122)
        .close()
        .extrude(0.126)
        .translate((0.0, 0.063, 0.0))
    )

    return side_profile.union(floor).union(hood).union(trunk).union(cowl).union(cabin)


def _door_window_mesh(name: str, side: float):
    """A shallow trapezoid coupe side-window cap for a door part."""
    window = (
        cq.Workplane("XZ")
        .moveTo(-0.098, 0.050)
        .lineTo(-0.020, 0.050)
        .lineTo(-0.006, 0.086)
        .lineTo(-0.085, 0.083)
        .close()
        .extrude(0.0035)
    )
    if side > 0:
        window = window.translate((0.0, 0.0050, 0.0))
    else:
        window = window.translate((0.0, -0.0020, 0.0))
    return mesh_from_cadquery(window, name, tolerance=0.0005, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_coupe_car")

    toy_red = model.material("painted_red_plastic", rgba=(0.82, 0.04, 0.025, 1.0))
    dark_red = model.material("dark_red_shadow", rgba=(0.40, 0.02, 0.02, 1.0))
    black = model.material("soft_black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    glass = model.material("smoked_plastic_glass", rgba=(0.02, 0.06, 0.09, 0.72))
    silver = model.material("satin_silver_plastic", rgba=(0.72, 0.70, 0.65, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_lower_body_shell(), "coupe_body_shell", tolerance=0.0007, angular_tolerance=0.08),
        material=toy_red,
        name="body_shell",
    )
    body.visual(
        Box((0.014, 0.120, 0.044)),
        origin=Origin(xyz=(0.047, 0.0, 0.130)),
        material=glass,
        name="windshield",
    )
    body.visual(
        Box((0.006, 0.108, 0.036)),
        origin=Origin(xyz=(-0.076, 0.0, 0.134), rpy=(0.0, 0.67, 0.0)),
        material=glass,
        name="rear_window",
    )
    body.visual(
        Box((0.380, 0.108, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_red,
        name="underside_shadow",
    )

    axle_radius = 0.00575
    axle_len = 0.238
    wheel_z = 0.043
    front_x = 0.133
    rear_x = -0.135
    for x, name in ((front_x, "front_axle"), (rear_x, "rear_axle")):
        body.visual(
            Cylinder(radius=axle_radius, length=axle_len),
            origin=Origin(xyz=(x, 0.0, wheel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name=name,
        )

    hinge_x = 0.058
    hinge_y = 0.112
    for side_name, side in (("left", 1.0), ("right", -1.0)):
        y = side * hinge_y
        body.visual(
            Box((0.010, 0.020, 0.070)),
            origin=Origin(xyz=(hinge_x - 0.006, side * 0.099, 0.104)),
            material=toy_red,
            name=f"{side_name}_hinge_pillar",
        )
        body.visual(
            Box((0.012, 0.026, 0.017)),
            origin=Origin(xyz=(hinge_x - 0.004, y - side * 0.013, 0.077)),
            material=silver,
            name=f"{side_name}_lower_hinge_leaf",
        )
        body.visual(
            Box((0.012, 0.026, 0.017)),
            origin=Origin(xyz=(hinge_x - 0.004, y - side * 0.013, 0.128)),
            material=silver,
            name=f"{side_name}_upper_hinge_leaf",
        )
        for z, barrel_name in ((0.077, "lower"), (0.128, "upper")):
            body.visual(
                Cylinder(radius=0.0042, length=0.018),
                origin=Origin(xyz=(hinge_x, y, z)),
                material=silver,
                name=f"{side_name}_{barrel_name}_hinge_knuckle",
            )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.039,
            0.032,
            inner_radius=0.029,
            carcass=TireCarcass(belt_width_ratio=0.62, sidewall_bulge=0.08),
            tread=TireTread(style="block", depth=0.0028, count=18, land_ratio=0.60),
            grooves=(TireGroove(center_offset=0.0, width=0.0035, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.003, radius=0.0015),
        ),
        "coupe_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.028,
            0.029,
            rim=WheelRim(inner_radius=0.019, flange_height=0.003, flange_thickness=0.0015, bead_seat_depth=0.001),
            hub=WheelHub(
                radius=0.010,
                width=0.021,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.014, hole_diameter=0.002),
            ),
            face=WheelFace(dish_depth=0.0025, front_inset=0.0015, rear_inset=0.001),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0016, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.0115),
        ),
        "coupe_wheel",
    )

    wheel_positions = {
        "left_front_wheel": (front_x, 0.117, wheel_z),
        "right_front_wheel": (front_x, -0.117, wheel_z),
        "left_rear_wheel": (rear_x, 0.117, wheel_z),
        "right_rear_wheel": (rear_x, -0.117, wheel_z),
    }
    for part_name, xyz in wheel_positions.items():
        wheel = model.part(part_name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=silver,
            name="rim",
        )
        model.articulation(
            f"{part_name}_joint",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=20.0),
        )

    door_len = 0.116
    door_h = 0.052
    door_z0 = 0.062
    hinge_z0 = door_z0
    for side_name, side, axis in (("left", 1.0, (0.0, 0.0, -1.0)), ("right", -1.0, (0.0, 0.0, 1.0))):
        door = model.part(f"{side_name}_door")
        door.visual(
            Box((door_len, 0.005, door_h)),
            origin=Origin(xyz=(-door_len / 2.0, side * 0.0030, door_h / 2.0)),
            material=toy_red,
            name="door_skin",
        )
        door.visual(
            _door_window_mesh(f"{side_name}_door_window", side),
            material=glass,
            name="side_window",
        )
        door.visual(
            Box((0.018, 0.003, 0.004)),
            origin=Origin(xyz=(-0.085, side * 0.0070, 0.031)),
            material=silver,
            name="door_handle",
        )
        door.visual(
            Cylinder(radius=0.0040, length=0.030),
            origin=Origin(xyz=(0.0, side * 0.0025, 0.041)),
            material=silver,
            name="hinge_knuckle",
        )
        door.visual(
            Box((0.010, 0.004, 0.024)),
            origin=Origin(xyz=(-0.004, side * 0.0025, 0.041)),
            material=silver,
            name="hinge_leaf",
        )
        model.articulation(
            f"{side_name}_door_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, side * hinge_y, hinge_z0)),
            axis=axis,
            motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wheel_specs = (
        ("left_front_wheel", "front_axle"),
        ("right_front_wheel", "front_axle"),
        ("left_rear_wheel", "rear_axle"),
        ("right_rear_wheel", "rear_axle"),
    )
    for wheel_name, axle_name in wheel_specs:
        ctx.allow_overlap(
            "body",
            wheel_name,
            elem_a=axle_name,
            elem_b="rim",
            reason="The fixed axle shaft is intentionally captured through the toy wheel hub so the wheel is visibly mounted while it rotates.",
        )
        ctx.expect_within(
            "body",
            wheel_name,
            axes="xz",
            inner_elem=axle_name,
            outer_elem="rim",
            margin=0.001,
            name=f"{wheel_name} axle is centered in hub",
        )
        ctx.expect_overlap(
            "body",
            wheel_name,
            axes="y",
            elem_a=axle_name,
            elem_b="rim",
            min_overlap=0.010,
            name=f"{wheel_name} hub remains on axle",
        )
        joint = object_model.get_articulation(f"{wheel_name}_joint")
        ctx.check(
            f"{wheel_name} uses continuous rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"joint_type={joint.articulation_type}",
        )

    left_front = object_model.get_part("left_front_wheel")
    right_front = object_model.get_part("right_front_wheel")
    left_rear = object_model.get_part("left_rear_wheel")
    right_rear = object_model.get_part("right_rear_wheel")
    ctx.expect_origin_gap(left_front, right_front, axis="y", min_gap=0.20, name="front wheels are on opposite sides")
    ctx.expect_origin_gap(left_rear, right_rear, axis="y", min_gap=0.20, name="rear wheels are on opposite sides")
    ctx.expect_origin_gap(left_front, left_rear, axis="x", min_gap=0.24, name="front axle sits ahead of rear axle")

    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")

    ctx.expect_gap(
        left_door,
        body,
        axis="y",
        positive_elem="door_skin",
        negative_elem="body_shell",
        min_gap=0.001,
        max_gap=0.020,
        name="left door sits just proud of side body",
    )
    ctx.expect_gap(
        body,
        right_door,
        axis="y",
        positive_elem="body_shell",
        negative_elem="door_skin",
        min_gap=0.001,
        max_gap=0.020,
        name="right door sits just proud of side body",
    )
    ctx.expect_overlap(left_door, body, axes="xz", elem_a="door_skin", elem_b="body_shell", min_overlap=0.045, name="left door covers side opening")
    ctx.expect_overlap(right_door, body, axes="xz", elem_a="door_skin", elem_b="body_shell", min_overlap=0.045, name="right door covers side opening")

    left_rest = ctx.part_world_aabb(left_door)
    right_rest = ctx.part_world_aabb(right_door)
    with ctx.pose({left_hinge: 0.85, right_hinge: 0.85}):
        left_open = ctx.part_world_aabb(left_door)
        right_open = ctx.part_world_aabb(right_door)
    ctx.check(
        "left door swings outward from A-pillar",
        left_rest is not None and left_open is not None and left_open[1][1] > left_rest[1][1] + 0.030,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right door swings outward from A-pillar",
        right_rest is not None and right_open is not None and right_open[0][1] < right_rest[0][1] - 0.030,
        details=f"rest={right_rest}, open={right_open}",
    )

    for hinge_name in ("left_door_hinge", "right_door_hinge"):
        hinge = object_model.get_articulation(hinge_name)
        limits = hinge.motion_limits
        ctx.check(
            f"{hinge_name} has realistic outward limits",
            hinge.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.9 <= limits.upper <= 1.2,
            details=f"type={hinge.articulation_type}, limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
