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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """A softly radiused rectangular body centered in X/Y and standing on z=0."""

    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(radius)
        .translate((0.0, 0.0, height / 2.0))
    )


def _hopper_shell() -> cq.Workplane:
    """Thin transparent globe shell with annular lips at top and bottom."""

    wall = 0.008
    center_z = 0.765
    radius = 0.240
    z_min = 0.585
    z_max = 0.965
    steps = 18

    outer = []
    for i in range(steps + 1):
        z = z_min + (z_max - z_min) * i / steps
        r = math.sqrt(max(radius * radius - (z - center_z) ** 2, 0.0))
        outer.append((r, z))

    inner = [(max(r - wall, 0.020), z) for r, z in reversed(outer)]
    profile = outer + inner

    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )


def _funnel_shell() -> cq.Workplane:
    """Small metal funnel under the candy globe leading toward the wheel."""

    profile = [
        (0.138, 0.552),
        (0.185, 0.602),
        (0.160, 0.618),
        (0.090, 0.575),
        (0.072, 0.552),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_capsule_candy_vendor")

    red = model.material("gloss_red", rgba=(0.82, 0.05, 0.04, 1.0))
    dark_red = model.material("dark_red", rgba=(0.42, 0.02, 0.02, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.018, 0.015, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.78, 0.74, 0.66, 1.0))
    dark = model.material("dark_shadow", rgba=(0.01, 0.01, 0.012, 1.0))
    clear = model.material("clear_polycarbonate", rgba=(0.72, 0.93, 1.0, 0.32))
    yellow = model.material("candy_yellow", rgba=(1.0, 0.80, 0.07, 1.0))
    blue = model.material("candy_blue", rgba=(0.05, 0.24, 0.95, 1.0))
    green = model.material("candy_green", rgba=(0.05, 0.70, 0.18, 1.0))
    pink = model.material("candy_pink", rgba=(1.0, 0.22, 0.58, 1.0))
    white = model.material("candy_white", rgba=(0.96, 0.94, 0.84, 1.0))

    cabinet = model.part("cabinet")

    cabinet.visual(
        mesh_from_cadquery(_rounded_box(0.44, 0.34, 0.55, 0.035), "rounded_lower_body"),
        material=red,
        name="lower_body",
    )
    cabinet.visual(
        Box((0.50, 0.38, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_red,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.46, 0.36, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.559)),
        material=chrome,
        name="top_plate",
    )
    cabinet.visual(
        Cylinder(radius=0.156, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.579)),
        material=red,
        name="hopper_collar",
    )
    cabinet.visual(
        mesh_from_cadquery(_funnel_shell(), "funnel_shell"),
        material=chrome,
        name="candy_funnel",
    )
    cabinet.visual(
        mesh_from_cadquery(_hopper_shell(), "clear_hopper_shell", tolerance=0.0008),
        material=clear,
        name="clear_hopper",
    )
    cabinet.visual(
        Cylinder(radius=0.118, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.976)),
        material=red,
        name="top_cap",
    )
    cabinet.visual(
        Sphere(radius=0.030),
        origin=Origin(xyz=(0.0, 0.0, 1.005)),
        material=chrome,
        name="cap_knob",
    )

    # A connected, overlapping candy/capsule pile resting visibly in the clear hopper.
    candy_specs = [
        ((0.000, 0.000, 0.618), 0.050, yellow, "candy_0"),
        ((0.075, 0.000, 0.622), 0.043, blue, "candy_1"),
        ((-0.071, 0.018, 0.623), 0.041, green, "candy_2"),
        ((0.030, 0.066, 0.626), 0.040, pink, "candy_3"),
        ((-0.032, -0.068, 0.626), 0.042, white, "candy_4"),
        ((0.090, -0.060, 0.635), 0.039, green, "candy_5"),
        ((-0.094, -0.040, 0.636), 0.039, blue, "candy_6"),
        ((0.012, -0.005, 0.686), 0.046, pink, "candy_7"),
        ((0.066, 0.048, 0.692), 0.039, white, "candy_8"),
        ((-0.060, 0.052, 0.690), 0.040, yellow, "candy_9"),
        ((0.045, -0.064, 0.694), 0.039, green, "candy_10"),
        ((-0.048, -0.056, 0.692), 0.041, blue, "candy_11"),
        ((0.000, 0.025, 0.746), 0.043, white, "candy_12"),
        ((0.055, -0.018, 0.748), 0.037, yellow, "candy_13"),
        ((-0.052, -0.018, 0.748), 0.037, pink, "candy_14"),
    ]
    for xyz, radius, material, name in candy_specs:
        cabinet.visual(Sphere(radius=radius), origin=Origin(xyz=xyz), material=material, name=name)

    # Fixed front face details around the rotary dispenser.
    cabinet.visual(
        Cylinder(radius=0.112, length=0.014),
        origin=Origin(xyz=(0.0, -0.176, 0.445), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="wheel_bezel",
    )
    cabinet.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, -0.187, 0.445), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="axle_socket",
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.085, length=0.030),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="dispensing_disk",
    )
    wheel.visual(
        Cylinder(radius=0.035, length=0.050),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="center_hub",
    )
    wheel.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.0, -0.043, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_red,
        name="finger_button",
    )
    for i, material in enumerate((yellow, blue, green, pink, white, green)):
        angle = 2.0 * math.pi * i / 6.0
        x = 0.055 * math.cos(angle)
        z = 0.055 * math.sin(angle)
        wheel.visual(
            Cylinder(radius=0.0115, length=0.006),
            origin=Origin(xyz=(x, -0.020, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"pocket_{i}",
        )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.320, 0.014, 0.240)),
        origin=Origin(xyz=(0.160, -0.007, 0.0)),
        material=chrome,
        name="panel_plate",
    )
    service_panel.visual(
        Cylinder(radius=0.009, length=0.205),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=dark,
        name="side_hinge_barrel",
    )
    service_panel.visual(
        Box((0.018, 0.012, 0.210)),
        origin=Origin(xyz=(0.006, -0.014, 0.0)),
        material=dark_red,
        name="hinge_leaf",
    )
    service_panel.visual(
        Box((0.180, 0.006, 0.070)),
        origin=Origin(xyz=(0.160, -0.0165, 0.040)),
        material=dark,
        name="chute_mouth",
    )
    service_panel.visual(
        Box((0.210, 0.018, 0.010)),
        origin=Origin(xyz=(0.160, -0.024, 0.080)),
        material=red,
        name="chute_top_lip",
    )
    service_panel.visual(
        Box((0.020, 0.018, 0.080)),
        origin=Origin(xyz=(0.045, -0.022, 0.040)),
        material=red,
        name="chute_side_0",
    )
    service_panel.visual(
        Box((0.020, 0.018, 0.080)),
        origin=Origin(xyz=(0.275, -0.022, 0.040)),
        material=red,
        name="chute_side_1",
    )
    for x, name in ((0.040, "flap_hinge_0"), (0.280, "flap_hinge_1")):
        service_panel.visual(
            Cylinder(radius=0.005, length=0.035),
            origin=Origin(xyz=(x, -0.018, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=name,
        )
    service_panel.visual(
        Cylinder(radius=0.0025, length=0.210),
        origin=Origin(xyz=(0.160, -0.0235, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="flap_hinge_pin",
    )
    service_panel.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.292, -0.024, -0.085)),
        material=dark_red,
        name="small_pull",
    )

    retrieval_flap = model.part("retrieval_flap")
    retrieval_flap.visual(
        Box((0.185, 0.008, 0.074)),
        origin=Origin(xyz=(0.0, -0.007, -0.037)),
        material=clear,
        name="flap_window",
    )
    retrieval_flap.visual(
        Box((0.205, 0.011, 0.010)),
        origin=Origin(xyz=(0.0, -0.008, -0.078)),
        material=chrome,
        name="lower_grip",
    )
    retrieval_flap.visual(
        Cylinder(radius=0.0048, length=0.100),
        origin=Origin(xyz=(0.0, -0.0065, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="flap_hinge_barrel",
    )

    model.articulation(
        "cabinet_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=wheel,
        origin=Origin(xyz=(0.0, -0.209, 0.445)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    model.articulation(
        "cabinet_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_panel,
        origin=Origin(xyz=(-0.172, -0.170, 0.200)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=4.0, velocity=1.6),
    )

    model.articulation(
        "service_panel_to_retrieval_flap",
        ArticulationType.REVOLUTE,
        parent=service_panel,
        child=retrieval_flap,
        origin=Origin(xyz=(0.160, -0.017, -0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=1.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    wheel = object_model.get_part("wheel")
    service_panel = object_model.get_part("service_panel")
    retrieval_flap = object_model.get_part("retrieval_flap")
    wheel_joint = object_model.get_articulation("cabinet_to_wheel")
    panel_joint = object_model.get_articulation("cabinet_to_service_panel")
    flap_joint = object_model.get_articulation("service_panel_to_retrieval_flap")

    ctx.allow_overlap(
        cabinet,
        wheel,
        elem_a="axle_socket",
        elem_b="center_hub",
        reason="The rotating wheel hub is intentionally captured inside the fixed front axle socket.",
    )
    ctx.allow_overlap(
        service_panel,
        retrieval_flap,
        elem_a="flap_hinge_pin",
        elem_b="flap_hinge_barrel",
        reason="The retrieval flap rotates around a small hinge pin passing through its hinge barrel.",
    )

    ctx.check(
        "wheel is continuous on horizontal axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.expect_gap(
        cabinet,
        wheel,
        axis="y",
        max_gap=0.016,
        max_penetration=0.0,
        positive_elem="wheel_bezel",
        negative_elem="dispensing_disk",
        name="wheel stands proud of fixed bezel",
    )
    ctx.expect_overlap(
        cabinet,
        wheel,
        axes="xz",
        min_overlap=0.018,
        elem_a="axle_socket",
        elem_b="center_hub",
        name="wheel hub is centered in axle socket",
    )
    ctx.expect_overlap(
        cabinet,
        wheel,
        axes="y",
        min_overlap=0.003,
        elem_a="axle_socket",
        elem_b="center_hub",
        name="wheel hub remains inserted in axle socket",
    )
    ctx.expect_gap(
        cabinet,
        service_panel,
        axis="y",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="lower_body",
        negative_elem="panel_plate",
        name="service panel is seated on front face",
    )
    ctx.expect_overlap(
        service_panel,
        cabinet,
        axes="xz",
        min_overlap=0.18,
        elem_a="panel_plate",
        elem_b="lower_body",
        name="service panel lies beneath wheel on the body",
    )
    ctx.expect_gap(
        service_panel,
        retrieval_flap,
        axis="y",
        max_gap=0.008,
        max_penetration=0.0,
        positive_elem="panel_plate",
        negative_elem="flap_window",
        name="retrieval flap sits proud of service panel",
    )
    ctx.expect_overlap(
        service_panel,
        retrieval_flap,
        axes="x",
        min_overlap=0.090,
        elem_a="flap_hinge_pin",
        elem_b="flap_hinge_barrel",
        name="retrieval flap hinge pin spans the barrel",
    )
    ctx.expect_overlap(
        service_panel,
        retrieval_flap,
        axes="yz",
        min_overlap=0.002,
        elem_a="flap_hinge_pin",
        elem_b="flap_hinge_barrel",
        name="retrieval flap barrel is captured on hinge pin",
    )

    closed_panel_aabb = ctx.part_world_aabb(service_panel)
    with ctx.pose({panel_joint: panel_joint.motion_limits.upper}):
        open_panel_aabb = ctx.part_world_aabb(service_panel)
    ctx.check(
        "service panel swings outward on side hinge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.10,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    closed_flap_aabb = ctx.part_world_aabb(retrieval_flap)
    with ctx.pose({flap_joint: flap_joint.motion_limits.upper}):
        open_flap_aabb = ctx.part_world_aabb(retrieval_flap)
    ctx.check(
        "retrieval flap swings outward below chute",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][1] < closed_flap_aabb[0][1] - 0.035,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
