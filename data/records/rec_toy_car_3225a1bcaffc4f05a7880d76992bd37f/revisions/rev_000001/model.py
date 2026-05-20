from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
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


def _rounded_box(size: tuple[float, float, float], radius: float = 0.006) -> cq.Workplane:
    """Small CadQuery helper for toy-like rounded plastic pieces."""
    shape = cq.Workplane("XY").box(*size)
    try:
        return shape.edges("|Z").fillet(radius)
    except Exception:
        return shape


def _body_shell() -> cq.Workplane:
    """A connected red roadster tub: low base, fenders, side sills, cowl and tail."""
    base = _rounded_box((0.540, 0.155, 0.052), 0.012).translate((0.000, 0.000, 0.064))
    side_sill_y = 0.082
    shell = base
    for y in (-side_sill_y, side_sill_y):
        shell = shell.union(_rounded_box((0.500, 0.030, 0.066), 0.007).translate((-0.005, y, 0.095)))
        # Long raised hood rails keep the separate hood visually supported.
        shell = shell.union(_rounded_box((0.235, 0.020, 0.038), 0.005).translate((0.145, y * 0.95, 0.116)))
        # Short rear deck rails frame the trunk opening.
        shell = shell.union(_rounded_box((0.150, 0.020, 0.038), 0.005).translate((-0.180, y * 0.95, 0.116)))
    # Cowl and rear cockpit bulkhead leave an open cabin between them.
    shell = shell.union(_rounded_box((0.030, 0.160, 0.070), 0.006).translate((0.015, 0.000, 0.124)))
    shell = shell.union(_rounded_box((0.026, 0.160, 0.062), 0.006).translate((-0.090, 0.000, 0.121)))
    shell = shell.union(_rounded_box((0.026, 0.155, 0.048), 0.006).translate((-0.265, 0.000, 0.104)))
    shell = shell.union(_rounded_box((0.026, 0.135, 0.040), 0.006).translate((0.262, 0.000, 0.100)))
    # Four rounded fender pods attach to the side sills while leaving the narrow wheels visible.
    for x in (-0.180, 0.185):
        for y in (-0.085, 0.085):
            shell = shell.union(_rounded_box((0.098, 0.038, 0.038), 0.010).translate((x, y, 0.105)))
    return shell


def _lid_panel(length: float, width: float, thickness: float, name: str) -> cq.Workplane:
    """A thin rounded lid with a small rolled hinge edge at local x=0."""
    panel = _rounded_box((length, width, thickness), 0.005).translate((length * 0.5, 0.0, thickness * 0.5))
    hinge_roll = (
        cq.Workplane("XY")
        .cylinder(width * 0.92, 0.0045)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, 0.0, thickness * 0.45))
    )
    lip = _rounded_box((0.012, width * 0.86, thickness * 0.70), 0.002).translate((0.004, 0.0, thickness * 0.42))
    return panel.union(hinge_roll).union(lip)


def _make_wheel(part, prefix: str, wheel_mat, tire_mat) -> None:
    wheel_origin = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.030,
                0.022,
                rim=WheelRim(inner_radius=0.020, flange_height=0.0035, flange_thickness=0.0018),
                hub=WheelHub(
                    radius=0.010,
                    width=0.018,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.014, hole_diameter=0.0018),
                ),
                face=WheelFace(dish_depth=0.0025, front_inset=0.0015, rear_inset=0.001),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.0018, window_radius=0.0035),
                bore=WheelBore(style="round", diameter=0.0045),
            ),
            f"{prefix}_rim",
        ),
        origin=wheel_origin,
        material=wheel_mat,
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.041,
                0.026,
                inner_radius=0.031,
                tread=TireTread(style="circumferential", depth=0.0025, count=3),
                grooves=(TireGroove(center_offset=0.0, width=0.0028, depth=0.0015),),
                sidewall=TireSidewall(style="rounded", bulge=0.05),
            ),
            f"{prefix}_tire",
        ),
        origin=wheel_origin,
        material=tire_mat,
        name="tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_roadster")

    red = model.material("painted_red_plastic", rgba=(0.78, 0.05, 0.035, 1.0))
    dark = model.material("black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.03, 0.03, 0.032, 1.0))
    silver = model.material("bright_toy_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    glass = model.material("clear_blue_windshield", rgba=(0.55, 0.80, 1.0, 0.42))
    tan = model.material("tan_molded_seats", rgba=(0.72, 0.50, 0.30, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shell(), "roadster_body_shell"), material=red, name="shell")
    body.visual(Box((0.122, 0.090, 0.012)), origin=Origin(xyz=(-0.030, 0.000, 0.098)), material=dark, name="cabin_floor")
    body.visual(Box((0.035, 0.055, 0.028)), origin=Origin(xyz=(-0.056, 0.000, 0.117)), material=tan, name="seat_cushion")
    body.visual(Box((0.014, 0.055, 0.046)), origin=Origin(xyz=(-0.081, 0.000, 0.141), rpy=(0.0, 0.22, 0.0)), material=tan, name="seat_back")
    body.visual(Cylinder(radius=0.016, length=0.0035), origin=Origin(xyz=(0.005, 0.000, 0.151), rpy=(pi / 2.0, 0.0, 0.0)), material=dark, name="steering_wheel")
    body.visual(Cylinder(radius=0.0022, length=0.038), origin=Origin(xyz=(0.012, 0.000, 0.130), rpy=(0.0, 0.55, 0.0)), material=silver, name="steering_column")
    body.visual(Box((0.008, 0.142, 0.052)), origin=Origin(xyz=(0.022, 0.000, 0.158), rpy=(0.0, -0.24, 0.0)), material=glass, name="windshield")
    body.visual(Cylinder(radius=0.005, length=0.230), origin=Origin(xyz=(0.185, 0.000, 0.051), rpy=(-pi / 2.0, 0.0, 0.0)), material=silver, name="front_axle")
    body.visual(Cylinder(radius=0.005, length=0.230), origin=Origin(xyz=(-0.180, 0.000, 0.051), rpy=(-pi / 2.0, 0.0, 0.0)), material=silver, name="rear_axle")
    body.visual(Box((0.016, 0.050, 0.012)), origin=Origin(xyz=(0.035, 0.000, 0.124)), material=silver, name="hood_hinge_bracket")
    body.visual(Cylinder(radius=0.003, length=0.106), origin=Origin(xyz=(0.040, 0.000, 0.131), rpy=(-pi / 2.0, 0.0, 0.0)), material=silver, name="hood_hinge_pin")
    body.visual(Box((0.014, 0.050, 0.012)), origin=Origin(xyz=(-0.255, 0.000, 0.125)), material=silver, name="trunk_hinge_bracket")
    body.visual(Cylinder(radius=0.003, length=0.100), origin=Origin(xyz=(-0.255, 0.000, 0.131), rpy=(-pi / 2.0, 0.0, 0.0)), material=silver, name="trunk_hinge_pin")

    hood = model.part("hood")
    hood.visual(mesh_from_cadquery(_lid_panel(0.210, 0.134, 0.010, "hood"), "roadster_hood_panel"), material=red, name="panel")
    hood.visual(Box((0.145, 0.010, 0.004)), origin=Origin(xyz=(0.115, 0.000, 0.012)), material=red, name="center_crease")

    trunk = model.part("trunk_lid")
    trunk.visual(mesh_from_cadquery(_lid_panel(0.140, 0.128, 0.010, "trunk"), "roadster_trunk_panel"), material=red, name="panel")
    trunk.visual(Box((0.060, 0.006, 0.004)), origin=Origin(xyz=(0.082, 0.000, 0.012)), material=silver, name="pull_handle")

    wheel_positions = {
        "front_left_wheel": (0.185, 0.121, 0.051),
        "front_right_wheel": (0.185, -0.121, 0.051),
        "rear_left_wheel": (-0.180, 0.121, 0.051),
        "rear_right_wheel": (-0.180, -0.121, 0.051),
    }
    wheel_parts = {}
    for wheel_name in wheel_positions:
        wheel = model.part(wheel_name)
        _make_wheel(wheel, wheel_name, silver, rubber)
        wheel_parts[wheel_name] = wheel

    model.articulation(
        "hood_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.040, 0.000, 0.131)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "trunk_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk,
        origin=Origin(xyz=(-0.255, 0.000, 0.132)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=1.8, lower=0.0, upper=1.05),
    )
    for wheel_name, xyz in wheel_positions.items():
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_parts[wheel_name],
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.20, velocity=30.0),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    hood = object_model.get_part("hood")
    trunk = object_model.get_part("trunk_lid")
    hood_hinge = object_model.get_articulation("hood_hinge")
    trunk_hinge = object_model.get_articulation("trunk_hinge")

    for joint_name in (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name}_continuous", joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS)

    for wheel_name, axle_name in (
        ("front_left_wheel", "front_axle"),
        ("front_right_wheel", "front_axle"),
        ("rear_left_wheel", "rear_axle"),
        ("rear_right_wheel", "rear_axle"),
    ):
        ctx.allow_overlap(
            body,
            wheel_name,
            elem_a=axle_name,
            elem_b="rim",
            reason="The toy axle is intentionally captured inside the wheel hub/rim so the wheel reads as mounted on a real shaft.",
        )
        ctx.expect_overlap(
            body,
            wheel_name,
            axes="y",
            min_overlap=0.003,
            elem_a=axle_name,
            elem_b="rim",
            name=f"{wheel_name} axle retained in hub",
        )

    ctx.allow_overlap(
        body,
        trunk,
        elem_a="trunk_hinge_pin",
        elem_b="panel",
        reason="The small tail hinge pin is intentionally captured inside the rolled rear edge of the trunk lid.",
    )
    ctx.expect_gap(
        trunk,
        body,
        axis="z",
        positive_elem="panel",
        negative_elem="trunk_hinge_pin",
        max_penetration=0.004,
        name="trunk hinge pin seated in rolled lid",
    )
    ctx.allow_overlap(
        body,
        hood,
        elem_a="hood_hinge_pin",
        elem_b="panel",
        reason="The rear hood hinge pin is intentionally captured inside the rolled rear edge of the hood.",
    )
    ctx.expect_gap(
        hood,
        body,
        axis="z",
        positive_elem="panel",
        negative_elem="hood_hinge_pin",
        max_penetration=0.004,
        name="hood hinge pin seated in rolled lid",
    )

    ctx.expect_overlap(hood, body, axes="xy", min_overlap=0.08, elem_a="panel", elem_b="shell", name="hood covers long front bay")
    ctx.expect_overlap(trunk, body, axes="xy", min_overlap=0.05, elem_a="panel", elem_b="shell", name="trunk covers short tail deck")
    closed_hood_aabb = ctx.part_world_aabb(hood)
    closed_trunk_aabb = ctx.part_world_aabb(trunk)
    with ctx.pose({hood_hinge: 0.9, trunk_hinge: 0.8}):
        open_hood_aabb = ctx.part_world_aabb(hood)
        open_trunk_aabb = ctx.part_world_aabb(trunk)
    ctx.check(
        "hood opens upward on rear hinge",
        closed_hood_aabb is not None
        and open_hood_aabb is not None
        and float(open_hood_aabb[1][2] - closed_hood_aabb[1][2]) > 0.060,
        details=f"closed={closed_hood_aabb}, open={open_hood_aabb}",
    )
    ctx.check(
        "trunk opens upward from tail hinge",
        closed_trunk_aabb is not None
        and open_trunk_aabb is not None
        and float(open_trunk_aabb[1][2] - closed_trunk_aabb[1][2]) > 0.035,
        details=f"closed={closed_trunk_aabb}, open={open_trunk_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
