from __future__ import annotations

import math

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


LEG_SWEEP = math.radians(22.0)
LEG_HINGE_RADIUS = 0.100
LEG_HINGE_Z = 0.004
HEAD_PAN_Z = 0.0495
BODY_TILT_Z = 0.130
BODY_REAR_X = -0.024
BATTERY_DOOR_WIDTH = 0.080


def cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def cq_cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").cylinder(length, radius).translate(center)


def cq_cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").cylinder(length, radius).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0).translate(center)


def cq_cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").cylinder(length, radius).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0).translate(center)


def rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def radial_origin(local_xyz: tuple[float, float, float], yaw: float, *, rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Origin:
    x, y, z = local_xyz
    wx, wy = rotate_xy(x, y, yaw)
    roll, pitch, local_yaw = rpy
    return Origin(xyz=(wx, wy, z), rpy=(roll, pitch, yaw + local_yaw))


def leg_spike_geometry() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.0065)
        .workplane(offset=-0.085)
        .circle(0.0008)
        .loft(combine=True)
    )


def build_leg_structure() -> cq.Workplane:
    leg = cq_cyl_y(0.010, 0.028, (0.0, 0.0, 0.0))
    leg = leg.union(cq_box((0.062, 0.026, 0.024), (0.050, 0.0, -0.014)))

    upper_beam = (
        cq_box((0.046, 0.026, 0.850), (0.206, 0.0, -0.410))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -math.degrees(LEG_SWEEP))
    )
    lower_beam = (
        cq_box((0.036, 0.020, 0.250), (0.388, 0.0, -0.880))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -math.degrees(LEG_SWEEP))
    )
    foot_collar = (
        cq.Workplane("XY")
        .cylinder(0.048, 0.015)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -math.degrees(LEG_SWEEP))
        .translate((0.438, 0.0, -0.997))
    )
    foot_spike = (
        leg_spike_geometry()
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -math.degrees(LEG_SWEEP))
        .translate((0.448, 0.0, -1.026))
    )

    leg = leg.union(upper_beam).union(lower_beam).union(foot_collar).union(foot_spike)
    return leg


def build_center_hub() -> cq.Workplane:
    hub = cq_cyl_z(0.078, 0.035, (0.0, 0.0, -0.005))
    hub = hub.union(cq_cyl_z(0.062, 0.020, (0.0, 0.0, 0.023)))
    hub = hub.union(cq_cyl_z(0.038, 0.160, (0.0, 0.0, -0.105)))
    hub = hub.union(cq_cyl_z(0.050, 0.018, (0.0, 0.0, -0.188)))
    hub = hub.union(cq_cyl_z(0.024, 0.022, (0.0, 0.0, 0.042)))

    for i in range(3):
        yaw = i * (2.0 * math.pi / 3.0)
        bridge = cq_box((0.040, 0.050, 0.018), (0.082, 0.0, LEG_HINGE_Z))
        lug_a = cq_cyl_y(0.0085, 0.012, (LEG_HINGE_RADIUS, 0.018, LEG_HINGE_Z))
        lug_b = cq_cyl_y(0.0085, 0.012, (LEG_HINGE_RADIUS, -0.018, LEG_HINGE_Z))
        hinge_mount = bridge.union(lug_a).union(lug_b).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(yaw))
        hub = hub.union(hinge_mount)

    return hub


def build_head_structure() -> cq.Workplane:
    head = cq_cyl_z(0.050, 0.024, (0.0, 0.0, 0.014))
    head = head.union(cq_box((0.052, 0.168, 0.024), (-0.014, 0.0, 0.036)))
    head = head.union(cq_box((0.050, 0.016, 0.162), (-0.022, 0.088, 0.112)))
    head = head.union(cq_box((0.050, 0.016, 0.162), (-0.022, -0.088, 0.112)))
    head = head.union(cq_box((0.030, 0.176, 0.030), (-0.036, 0.0, 0.150)))
    head = head.union(cq_cyl_y(0.018, 0.018, (-0.010, 0.080, BODY_TILT_Z)))
    head = head.union(cq_cyl_y(0.018, 0.018, (-0.010, -0.080, BODY_TILT_Z)))
    return head


def build_body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(0.188, 0.120, 0.150)
        .edges("|Z")
        .fillet(0.016)
        .translate((0.072, 0.0, 0.0))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.115, 0.104, 0.028)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.050, 0.0, 0.060))
    )
    shell = shell.union(top_cap)

    battery_cavity = cq_box((0.038, 0.086, 0.112), (-0.006, 0.0, 0.0))
    shell = shell.cut(battery_cavity)

    return shell


def build_battery_door() -> cq.Workplane:
    door = cq.Workplane("XY").box(0.004, BATTERY_DOOR_WIDTH, 0.108).translate((-0.006, BATTERY_DOOR_WIDTH * 0.5, 0.0))
    pull_tab = cq_box((0.010, 0.014, 0.028), (-0.012, BATTERY_DOOR_WIDTH - 0.006, 0.0))
    hinge_knuckle = cq_cyl_z(0.0037, 0.044, (-0.005, 0.0, 0.0))
    return door.union(pull_tab).union(hinge_knuckle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_device_tripod")

    painted_metal = model.material("painted_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.15, 0.16, 0.18, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.07, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.16, 0.30, 0.42, 0.85))
    anodized = model.material("anodized", rgba=(0.52, 0.56, 0.60, 1.0))

    center_hub = model.part("center_hub")
    center_hub.visual(
        Cylinder(radius=0.078, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=painted_metal,
        name="crown_base",
    )
    center_hub.visual(
        Cylinder(radius=0.062, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=painted_metal,
        name="crown_cap",
    )
    center_hub.visual(
        Cylinder(radius=0.038, length=0.166),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=painted_metal,
        name="center_column",
    )
    center_hub.visual(
        Cylinder(radius=0.050, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.188)),
        material=painted_metal,
        name="ground_collar",
    )
    center_hub.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=painted_metal,
        name="pan_spindle",
    )
    for i in range(3):
        yaw = i * (2.0 * math.pi / 3.0)
        center_hub.visual(
            Box((0.040, 0.050, 0.018)),
            origin=radial_origin((0.082, 0.0, LEG_HINGE_Z), yaw),
            material=painted_metal,
            name=f"crown_bridge_{i}",
        )
        center_hub.visual(
            Cylinder(radius=0.0085, length=0.012),
            origin=radial_origin((LEG_HINGE_RADIUS, 0.018, LEG_HINGE_Z), yaw, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_metal,
            name=f"crown_lug_{i}_0",
        )
        center_hub.visual(
            Cylinder(radius=0.0085, length=0.012),
            origin=radial_origin((LEG_HINGE_RADIUS, -0.018, LEG_HINGE_Z), yaw, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_metal,
            name=f"crown_lug_{i}_1",
        )

    for i in range(3):
        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.062, 0.026, 0.024)),
            origin=Origin(xyz=(0.050, 0.0, -0.014)),
            material=anodized,
            name="upper_shoe",
        )
        leg.visual(
            Box((0.046, 0.026, 0.850)),
            origin=Origin(xyz=(0.206, 0.0, -0.410), rpy=(0.0, -LEG_SWEEP, 0.0)),
            material=anodized,
            name="upper_beam",
        )
        leg.visual(
            Box((0.036, 0.020, 0.250)),
            origin=Origin(xyz=(0.388, 0.0, -0.880), rpy=(0.0, -LEG_SWEEP, 0.0)),
            material=anodized,
            name="lower_beam",
        )
        leg.visual(
            Cylinder(radius=0.015, length=0.048),
            origin=Origin(xyz=(0.438, 0.0, -0.997), rpy=(0.0, -LEG_SWEEP, 0.0)),
            material=anodized,
            name="foot_collar",
        )
        leg.visual(
            mesh_from_cadquery(leg_spike_geometry(), f"leg_{i}_spike"),
            origin=Origin(xyz=(0.448, 0.0, -1.026), rpy=(0.0, -LEG_SWEEP, 0.0)),
            material=dark_metal,
            name="foot_spike",
        )
        model.articulation(
            f"hub_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=center_hub,
            child=leg,
            origin=Origin(
                xyz=(
                    math.cos(i * (2.0 * math.pi / 3.0)) * LEG_HINGE_RADIUS,
                    math.sin(i * (2.0 * math.pi / 3.0)) * LEG_HINGE_RADIUS,
                    LEG_HINGE_Z,
                ),
                rpy=(0.0, 0.0, i * (2.0 * math.pi / 3.0)),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=45.0,
                velocity=1.2,
                lower=-0.20,
                upper=1.20,
            ),
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.050, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=painted_metal,
        name="pan_collar",
    )
    head.visual(
        Box((0.026, 0.176, 0.024)),
        origin=Origin(xyz=(-0.045, 0.0, 0.034)),
        material=painted_metal,
        name="rear_bridge",
    )
    head.visual(
        Box((0.024, 0.016, 0.166)),
        origin=Origin(xyz=(-0.034, 0.088, 0.113)),
        material=painted_metal,
        name="cheek_0",
    )
    head.visual(
        Box((0.024, 0.016, 0.166)),
        origin=Origin(xyz=(-0.034, -0.088, 0.113)),
        material=painted_metal,
        name="cheek_1",
    )
    head.visual(
        Box((0.026, 0.176, 0.024)),
        origin=Origin(xyz=(-0.058, 0.0, 0.185)),
        material=painted_metal,
        name="upper_brace",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(-0.027, 0.082, BODY_TILT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_metal,
        name="trunnion_socket_0",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(-0.027, -0.082, BODY_TILT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_metal,
        name="trunnion_socket_1",
    )
    model.articulation(
        "hub_to_head",
        ArticulationType.CONTINUOUS,
        parent=center_hub,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, HEAD_PAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
    )

    body = model.part("body")
    body.visual(mesh_from_cadquery(build_body_shell(), "body_shell"), material=black_polymer, name="body_shell")
    body.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.068, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_0",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, -0.068, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_1",
    )
    body.visual(
        Cylinder(radius=0.0046, length=0.022),
        origin=Origin(xyz=(BODY_REAR_X - 0.005, -0.040, 0.037)),
        material=dark_metal,
        name="door_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.0046, length=0.022),
        origin=Origin(xyz=(BODY_REAR_X - 0.005, -0.040, -0.037)),
        material=dark_metal,
        name="door_knuckle_1",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="lens_hood",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.177, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )
    body.visual(
        Box((0.045, 0.070, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, 0.078)),
        material=dark_metal,
        name="top_panel",
    )
    model.articulation(
        "head_to_body",
        ArticulationType.REVOLUTE,
        parent=head,
        child=body,
        origin=Origin(xyz=(-0.001, 0.0, BODY_TILT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.35,
            upper=1.00,
        ),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(mesh_from_cadquery(build_battery_door(), "battery_door"), material=dark_metal, name="door_panel")
    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(BODY_REAR_X - 0.005, -BATTERY_DOOR_WIDTH * 0.5, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    center_hub = object_model.get_part("center_hub")
    body = object_model.get_part("body")
    battery_door = object_model.get_part("battery_door")
    leg_0 = object_model.get_part("leg_0")

    head_pan = object_model.get_articulation("hub_to_head")
    body_tilt = object_model.get_articulation("head_to_body")
    battery_hinge = object_model.get_articulation("body_to_battery_door")
    leg_hinge = object_model.get_articulation("hub_to_leg_0")
    leg_1 = object_model.get_part("leg_1")
    leg_2 = object_model.get_part("leg_2")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return tuple((mn[i] + mx[i]) * 0.5 for i in range(3))

    for index, leg in enumerate((leg_0, leg_1, leg_2)):
        for crown_elem in (f"crown_bridge_{index}", f"crown_lug_{index}_0", f"crown_lug_{index}_1"):
            ctx.allow_overlap(
                center_hub,
                leg,
                elem_a=crown_elem,
                elem_b="hinge_barrel",
                reason="The crown hinge is represented by interleaved simplified hinge solids around the leg barrel.",
            )

    ctx.expect_gap(
        body,
        battery_door,
        axis="x",
        positive_elem="body_shell",
        negative_elem="door_panel",
        max_gap=0.010,
        max_penetration=0.004,
        name="battery door seats against rear housing opening",
    )
    ctx.expect_overlap(
        body,
        battery_door,
        axes="yz",
        elem_a="body_shell",
        elem_b="door_panel",
        min_overlap=0.060,
        name="battery door covers the battery opening",
    )

    front_rest = aabb_center(ctx.part_element_world_aabb(body, elem="front_glass"))
    with ctx.pose({body_tilt: 0.75}):
        front_tilted = aabb_center(ctx.part_element_world_aabb(body, elem="front_glass"))
    ctx.check(
        "positive body tilt raises optics",
        front_rest is not None and front_tilted is not None and front_tilted[2] > front_rest[2] + 0.040,
        details=f"rest={front_rest}, tilted={front_tilted}",
    )

    with ctx.pose({head_pan: math.pi / 2.0}):
        front_panned = aabb_center(ctx.part_element_world_aabb(body, elem="front_glass"))
    ctx.check(
        "head pan swings optics around the vertical axis",
        front_rest is not None and front_panned is not None and abs(front_panned[1]) > 0.120 and abs(front_panned[0]) < abs(front_rest[0]) - 0.080,
        details=f"rest={front_rest}, panned={front_panned}",
    )

    door_closed = aabb_center(ctx.part_element_world_aabb(battery_door, elem="door_panel"))
    with ctx.pose({battery_hinge: 1.15}):
        door_open = aabb_center(ctx.part_element_world_aabb(battery_door, elem="door_panel"))
    ctx.check(
        "battery door opens outward from the rear housing",
        door_closed is not None and door_open is not None and door_open[0] < door_closed[0] - 0.020,
        details=f"closed={door_closed}, open={door_open}",
    )

    leg_rest = ctx.part_world_aabb(leg_0)
    with ctx.pose({leg_hinge: 1.10}):
        leg_folded = ctx.part_world_aabb(leg_0)
    ctx.check(
        "crown hinge folds a leg inward",
        leg_rest is not None
        and leg_folded is not None
        and aabb_center(leg_rest) is not None
        and aabb_center(leg_folded) is not None
        and aabb_center(leg_folded)[0] < aabb_center(leg_rest)[0] - 0.450,
        details=f"rest={leg_rest}, folded={leg_folded}",
    )
    ctx.check(
        "tripod hub remains above the feet",
        leg_rest is not None and ctx.part_world_aabb(center_hub) is not None and ctx.part_world_aabb(center_hub)[0][2] > leg_rest[0][2] + 0.850,
        details=f"hub={ctx.part_world_aabb(center_hub)}, leg={leg_rest}",
    )

    return ctx.report()


object_model = build_object_model()
