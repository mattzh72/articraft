from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


GAS_RADIUS = 0.027
BASE_SLEEVE_INNER_RADIUS = 0.0265
SEAT_SOCKET_INNER_RADIUS = 0.0265
FOOT_CLAMP_INNER_RADIUS = 0.0265

GAS_LIFT_TRAVEL = 0.16
FOOT_RING_TRAVEL = 0.16


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float):
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _rounded_panel(width: float, thickness: float, height: float, radius: float):
    panel = cq.Workplane("XY").box(width, thickness, height)
    return panel.edges().fillet(radius)


def _seat_cushion_mesh():
    profile = [
        (0.000, 0.000),
        (0.185, 0.000),
        (0.212, 0.010),
        (0.224, 0.034),
        (0.213, 0.064),
        (0.182, 0.076),
        (0.000, 0.076),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=72), "rounded_seat_cushion")


def _radial_origin(radius: float, z: float, angle: float, local_yaw_offset: float = 0.0) -> Origin:
    return Origin(
        xyz=(radius * cos(angle), radius * sin(angle), z),
        rpy=(0.0, 0.0, angle + local_yaw_offset),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stool_height_office_chair")

    model.material("black_powdercoat", rgba=(0.02, 0.022, 0.026, 1.0))
    model.material("dark_plastic", rgba=(0.045, 0.045, 0.052, 1.0))
    model.material("rubber_black", rgba=(0.006, 0.006, 0.007, 1.0))
    model.material("brushed_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("seat_fabric", rgba=(0.10, 0.16, 0.23, 1.0))
    model.material("mechanism_black", rgba=(0.015, 0.015, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.085, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material="black_powdercoat",
        name="center_hub",
    )
    base.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.046, BASE_SLEEVE_INNER_RADIUS, 0.275),
            "base_column_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material="black_powdercoat",
        name="base_sleeve",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material="brushed_chrome",
        name="base_retaining_collar",
    )

    caster_radius = 0.365
    for i in range(5):
        angle = 2.0 * pi * i / 5.0
        base.visual(
            Box((0.295, 0.044, 0.034)),
            origin=_radial_origin(0.182, 0.120, angle),
            material="black_powdercoat",
            name=f"star_arm_{i}",
        )
        base.visual(
            Cylinder(radius=0.023, length=0.046),
            origin=_radial_origin(caster_radius, 0.112, angle),
            material="black_powdercoat",
            name=f"caster_socket_{i}",
        )
        base.visual(
            Box((0.055, 0.035, 0.026)),
            origin=_radial_origin(0.320, 0.112, angle),
            material="black_powdercoat",
            name=f"socket_bridge_{i}",
        )

    gas_column = model.part("gas_column")
    gas_column.visual(
        Cylinder(radius=GAS_RADIUS, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material="brushed_chrome",
        name="chrome_post",
    )
    gas_column.visual(
        Cylinder(radius=0.038, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material="brushed_chrome",
        name="upper_gas_shroud",
    )

    foot_ring = model.part("foot_ring")
    foot_ring.visual(
        mesh_from_geometry(TorusGeometry(radius=0.285, tube=0.012, radial_segments=18, tubular_segments=96), "foot_ring_tube"),
        material="brushed_chrome",
        name="ring_tube",
    )
    foot_ring.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.058, FOOT_CLAMP_INNER_RADIUS, 0.064),
            "foot_ring_clamp_collar",
        ),
        material="black_powdercoat",
        name="clamp_collar",
    )
    foot_ring.visual(
        Box((0.046, 0.030, 0.076)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material="black_powdercoat",
        name="clamp_block",
    )
    foot_ring.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.077, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_chrome",
        name="clamp_screw",
    )
    for i in range(3):
        angle = 2.0 * pi * i / 3.0 + pi / 6.0
        foot_ring.visual(
            Cylinder(radius=0.0085, length=0.235),
            origin=Origin(
                xyz=(0.166 * cos(angle), 0.166 * sin(angle), 0.0),
                rpy=(0.0, pi / 2.0, angle),
            ),
            material="brushed_chrome",
            name=f"ring_spoke_{i}",
        )

    seat = model.part("seat")
    seat.visual(_seat_cushion_mesh(), material="seat_fabric", name="seat_cushion")
    seat.visual(
        Cylinder(radius=0.115, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material="mechanism_black",
        name="underseat_plate",
    )
    seat.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.058, SEAT_SOCKET_INNER_RADIUS, 0.090),
            "seat_post_socket",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material="mechanism_black",
        name="seat_socket",
    )
    seat.visual(
        Box((0.170, 0.130, 0.030)),
        origin=Origin(xyz=(0.0, 0.105, 0.006)),
        material="mechanism_black",
        name="tilt_mechanism_box",
    )
    for x in (-0.115, 0.115):
        seat.visual(
            Box((0.020, 0.165, 0.020)),
            origin=Origin(xyz=(x, 0.150, 0.055)),
            material="black_powdercoat",
            name=f"rear_bracket_{0 if x < 0 else 1}",
        )
        seat.visual(
            Cylinder(radius=0.009, length=0.240),
            origin=Origin(xyz=(x, 0.230, 0.172)),
            material="black_powdercoat",
            name=f"back_post_{0 if x < 0 else 1}",
        )
    seat.visual(
        mesh_from_cadquery(_rounded_panel(0.340, 0.048, 0.215, 0.016), "small_backrest_pad"),
        origin=Origin(xyz=(0.0, 0.250, 0.278)),
        material="seat_fabric",
        name="backrest_pad",
    )
    seat.visual(
        Box((0.265, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.224, 0.193)),
        material="black_powdercoat",
        name="back_crossbar",
    )

    model.articulation(
        "gas_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gas_column,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=GAS_LIFT_TRAVEL, effort=240.0, velocity=0.22),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=gas_column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.5),
    )
    model.articulation(
        "foot_ring_slide",
        ArticulationType.PRISMATIC,
        parent=gas_column,
        child=foot_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.055, upper=FOOT_RING_TRAVEL, effort=80.0, velocity=0.18),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.038,
            0.030,
            inner_radius=0.026,
            tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.60),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )
    for i in range(5):
        angle = 2.0 * pi * i / 5.0
        fork = model.part(f"caster_fork_{i}")
        fork.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, 0.000)),
            material="brushed_chrome",
            name="kingpin",
        )
        fork.visual(
            Box((0.072, 0.030, 0.015)),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material="black_powdercoat",
            name="fork_crown",
        )
        fork.visual(
            Box((0.007, 0.030, 0.088)),
            origin=Origin(xyz=(-0.023, 0.0, -0.067)),
            material="black_powdercoat",
            name="fork_cheek_0",
        )
        fork.visual(
            Box((0.007, 0.030, 0.088)),
            origin=Origin(xyz=(0.023, 0.0, -0.067)),
            material="black_powdercoat",
            name="fork_cheek_1",
        )
        fork.visual(
            Cylinder(radius=0.006, length=0.062),
            origin=Origin(xyz=(0.0, 0.0, -0.076), rpy=(0.0, pi / 2.0, 0.0)),
            material="brushed_chrome",
            name="axle_pin",
        )
        fork.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(-0.031, 0.0, -0.076), rpy=(0.0, pi / 2.0, 0.0)),
            material="brushed_chrome",
            name="axle_cap_0",
        )
        fork.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.031, 0.0, -0.076), rpy=(0.0, pi / 2.0, 0.0)),
            material="brushed_chrome",
            name="axle_cap_1",
        )

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(tire_mesh, material="rubber_black", name="tire")
        wheel.visual(
            Cylinder(radius=0.0265, length=0.032),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material="dark_plastic",
            name="wheel_hub",
        )

        model.articulation(
            f"caster_swivel_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=_radial_origin(caster_radius, 0.112, angle),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=8.0),
        )
        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.076)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    gas = object_model.get_part("gas_column")
    seat = object_model.get_part("seat")
    foot_ring = object_model.get_part("foot_ring")
    gas_lift = object_model.get_articulation("gas_lift")
    foot_slide = object_model.get_articulation("foot_ring_slide")

    ctx.allow_overlap(
        base,
        gas,
        elem_a="base_sleeve",
        elem_b="chrome_post",
        reason="The chrome gas post is intentionally captured inside the tall base sleeve so the chair column remains clipped into the five-star base.",
    )
    ctx.allow_overlap(
        base,
        gas,
        elem_a="center_hub",
        elem_b="chrome_post",
        reason="The bottom of the gas post passes through the molded central hub as a retained column socket.",
    )
    ctx.allow_overlap(
        base,
        gas,
        elem_a="base_retaining_collar",
        elem_b="chrome_post",
        reason="The collar is a simplified retained trim ring around the post at the top of the base sleeve.",
    )
    ctx.expect_within(
        gas,
        base,
        axes="xy",
        inner_elem="chrome_post",
        outer_elem="base_sleeve",
        margin=0.0,
        name="gas post is centered in base sleeve",
    )
    ctx.expect_overlap(
        gas,
        base,
        axes="z",
        elem_a="chrome_post",
        elem_b="base_sleeve",
        min_overlap=0.08,
        name="gas post remains inserted in base sleeve",
    )
    ctx.expect_overlap(
        gas,
        base,
        axes="z",
        elem_a="chrome_post",
        elem_b="center_hub",
        min_overlap=0.02,
        name="gas post passes through central hub",
    )
    ctx.expect_overlap(
        gas,
        base,
        axes="z",
        elem_a="chrome_post",
        elem_b="base_retaining_collar",
        min_overlap=0.018,
        name="gas post passes through retaining collar",
    )
    with ctx.pose({gas_lift: GAS_LIFT_TRAVEL}):
        ctx.expect_overlap(
            gas,
            base,
            axes="z",
            elem_a="chrome_post",
            elem_b="base_sleeve",
            min_overlap=0.08,
            name="raised gas post remains inserted in base sleeve",
        )

    ctx.allow_overlap(
        gas,
        seat,
        elem_a="chrome_post",
        elem_b="seat_socket",
        reason="The upper end of the gas column is intentionally seated inside the under-seat socket for the swivel bearing.",
    )
    ctx.allow_overlap(
        gas,
        seat,
        elem_a="upper_gas_shroud",
        elem_b="seat_socket",
        reason="The upper gas-column shroud nests into the under-seat socket as the visible swivel-bearing housing.",
    )
    ctx.allow_overlap(
        gas,
        seat,
        elem_a="chrome_post",
        elem_b="underseat_plate",
        reason="The post passes through the center of the simplified under-seat bearing plate before entering the socket.",
    )
    ctx.expect_within(
        gas,
        seat,
        axes="xy",
        inner_elem="chrome_post",
        outer_elem="seat_socket",
        margin=0.0,
        name="gas post is centered in seat socket",
    )
    ctx.expect_overlap(
        gas,
        seat,
        axes="z",
        elem_a="chrome_post",
        elem_b="seat_socket",
        min_overlap=0.06,
        name="gas post plugs into seat socket",
    )
    ctx.expect_overlap(
        gas,
        seat,
        axes="z",
        elem_a="upper_gas_shroud",
        elem_b="seat_socket",
        min_overlap=0.05,
        name="upper shroud sits in seat socket",
    )
    ctx.expect_overlap(
        gas,
        seat,
        axes="z",
        elem_a="chrome_post",
        elem_b="underseat_plate",
        min_overlap=0.018,
        name="gas post passes through under-seat plate",
    )

    ctx.allow_overlap(
        gas,
        foot_ring,
        elem_a="chrome_post",
        elem_b="clamp_collar",
        reason="The adjustable foot-ring clamp encircles and lightly grips the gas post while sliding along it.",
    )
    ctx.expect_within(
        gas,
        foot_ring,
        axes="xy",
        inner_elem="chrome_post",
        outer_elem="clamp_collar",
        margin=0.0,
        name="foot-ring clamp is concentric with post",
    )
    ctx.expect_overlap(
        gas,
        foot_ring,
        axes="z",
        elem_a="chrome_post",
        elem_b="clamp_collar",
        min_overlap=0.05,
        name="foot-ring clamp overlaps post height",
    )
    with ctx.pose({foot_slide: FOOT_RING_TRAVEL}):
        ctx.expect_overlap(
            gas,
            foot_ring,
            axes="z",
            elem_a="chrome_post",
            elem_b="clamp_collar",
            min_overlap=0.05,
            name="raised foot-ring clamp stays on post",
        )

    for i in range(5):
        fork = object_model.get_part(f"caster_fork_{i}")
        wheel = object_model.get_part(f"caster_wheel_{i}")
        ctx.allow_overlap(
            base,
            fork,
            elem_a=f"caster_socket_{i}",
            elem_b="kingpin",
            reason="The caster kingpin is intentionally captured inside the vertical socket so the fork can swivel under the five-star base.",
        )
        ctx.expect_within(
            fork,
            base,
            axes="xy",
            inner_elem="kingpin",
            outer_elem=f"caster_socket_{i}",
            margin=0.001,
            name=f"caster {i} kingpin sits in socket",
        )
        ctx.expect_overlap(
            fork,
            base,
            axes="z",
            elem_a="kingpin",
            elem_b=f"caster_socket_{i}",
            min_overlap=0.04,
            name=f"caster {i} kingpin retained vertically",
        )
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a="axle_pin",
            elem_b="wheel_hub",
            reason="The caster wheel hub is intentionally pierced by the axle pin it spins on.",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="x",
            elem_a="axle_pin",
            elem_b="wheel_hub",
            min_overlap=0.025,
            name=f"caster {i} axle crosses wheel hub",
        )

    rest_seat_pos = ctx.part_world_position(seat)
    rest_ring_pos = ctx.part_world_position(foot_ring)
    with ctx.pose({gas_lift: GAS_LIFT_TRAVEL, foot_slide: FOOT_RING_TRAVEL}):
        raised_seat_pos = ctx.part_world_position(seat)
        raised_ring_pos = ctx.part_world_position(foot_ring)
    ctx.check(
        "gas lift raises seat",
        rest_seat_pos is not None
        and raised_seat_pos is not None
        and raised_seat_pos[2] > rest_seat_pos[2] + GAS_LIFT_TRAVEL * 0.90,
        details=f"rest={rest_seat_pos}, raised={raised_seat_pos}",
    )
    ctx.check(
        "foot ring slides upward on post",
        rest_ring_pos is not None
        and raised_ring_pos is not None
        and raised_ring_pos[2] > rest_ring_pos[2] + FOOT_RING_TRAVEL * 0.90,
        details=f"rest={rest_ring_pos}, raised={raised_ring_pos}",
    )

    return ctx.report()


object_model = build_object_model()
