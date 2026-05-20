from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


AXLE_Z = 8.15
RIM_RADIUS = 5.00
HANGER_RADIUS = 6.25
CABIN_COUNT = 8


STEEL = Material("painted_white_steel", rgba=(0.82, 0.86, 0.88, 1.0))
DARK_STEEL = Material("dark_axle_steel", rgba=(0.16, 0.18, 0.20, 1.0))
PLATFORM_MAT = Material("weathered_boarding_platform", rgba=(0.45, 0.39, 0.32, 1.0))
WATER = Material("harbor_water", rgba=(0.05, 0.22, 0.34, 1.0))
CABIN_FRAME = Material("cabin_white_frame", rgba=(0.92, 0.94, 0.94, 1.0))
GLASS = Material("slightly_blue_glass", rgba=(0.35, 0.70, 0.95, 0.45))
SAFETY_YELLOW = Material("safety_yellow", rgba=(1.0, 0.76, 0.10, 1.0))


def _torus_mesh(radius: float, tube: float, name: str):
    geom = TorusGeometry(radius, tube, radial_segments=24, tubular_segments=96)
    # TorusGeometry is built in the local XY plane.  Rotate it so the wheel
    # lies in the XZ plane and spins about the horizontal Y axle.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _add_sloped_cylinder(part, start, end, radius: float, material, name: str) -> None:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0:
        return
    # The support tubes only need XZ-plane sloping plus occasional Y-axis
    # crossbars, so a compact orientation rule is clearer than a full frame
    # solver here.
    if abs(dy) > 1e-9 and abs(dx) < 1e-9 and abs(dz) < 1e-9:
        rpy = (-math.pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, math.atan2(dx, dz), 0.0)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=rpy,
        ),
        material=material,
        name=name,
    )


def _add_wheel_spokes(wheel) -> None:
    for i in range(16):
        theta = 2.0 * math.pi * i / 16.0
        r_inner = 0.42
        r_outer = RIM_RADIUS - 0.03
        r_mid = (r_inner + r_outer) / 2.0
        length = r_outer - r_inner
        wheel.visual(
            Box((length, 0.105, 0.105)),
            origin=Origin(
                xyz=(r_mid * math.cos(theta), 0.0, r_mid * math.sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=STEEL,
            name=f"spoke_{i}",
        )


def _add_hanger_mounts(wheel) -> None:
    mount_length = HANGER_RADIUS - RIM_RADIUS + 0.25
    mount_radius = (HANGER_RADIUS + RIM_RADIUS) / 2.0
    for i in range(CABIN_COUNT):
        theta = -math.pi / 2.0 + 2.0 * math.pi * i / CABIN_COUNT
        wheel.visual(
            Box((mount_length, 0.18, 0.22)),
            origin=Origin(
                xyz=(mount_radius * math.cos(theta), 0.0, mount_radius * math.sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=DARK_STEEL,
            name=f"hanger_mount_{i}",
        )


def _add_cabin_visuals(cabin) -> None:
    depth = 1.05
    width = 1.20
    height = 0.95
    hanger_drop = 0.55
    body_y = -0.75
    center_z = -(hanger_drop + height / 2.0)

    # Pivot pin and two short hanger arms make the separate cabin visibly
    # supported from its rim pivot.
    cabin.visual(
        Cylinder(radius=0.060, length=2.30),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK_STEEL,
        name="pivot_pin",
    )
    for y in (-0.42, -1.08):
        cabin.visual(
            Box((0.090, 0.070, hanger_drop + 0.12)),
            origin=Origin(xyz=(0.0, y, -hanger_drop / 2.0)),
            material=DARK_STEEL,
            name=f"hanger_arm_{0 if y < 0 else 1}",
        )

    cabin.visual(
        Box((width, depth, height)),
        origin=Origin(xyz=(0.0, body_y, center_z)),
        material=CABIN_FRAME,
        name="body_shell",
    )
    # Blue glass panels are slightly proud but still seated into the cabin body.
    cabin.visual(
        Box((width * 0.74, 0.012, height * 0.48)),
        origin=Origin(xyz=(0.0, body_y - depth / 2.0 - 0.004, center_z + 0.025)),
        material=GLASS,
        name="front_window",
    )
    cabin.visual(
        Box((width * 0.74, 0.012, height * 0.48)),
        origin=Origin(xyz=(0.0, body_y + depth / 2.0 + 0.004, center_z + 0.025)),
        material=GLASS,
        name="rear_window",
    )
    for x, suffix in ((-width / 2.0 - 0.004, "0"), (width / 2.0 + 0.004, "1")):
        cabin.visual(
            Box((0.012, depth * 0.62, height * 0.46)),
            origin=Origin(xyz=(x, body_y, center_z + 0.025)),
            material=GLASS,
            name=f"side_window_{suffix}",
        )
    cabin.visual(
        Box((width * 0.56, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, body_y - depth / 2.0 - 0.028, center_z - 0.140)),
        material=SAFETY_YELLOW,
        name="door_sill",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="waterfront_observation_wheel")

    site = model.part("support_frame")
    site.visual(
        Box((14.0, 8.0, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=WATER,
        name="water_base",
    )
    site.visual(
        Box((4.8, 2.0, 0.55)),
        origin=Origin(xyz=(0.0, -2.35, 0.275)),
        material=PLATFORM_MAT,
        name="boarding_platform",
    )
    site.visual(
        Box((2.2, 1.20, 0.12)),
        origin=Origin(xyz=(0.0, -0.85, 0.300)),
        material=PLATFORM_MAT,
        name="boarding_bridge",
    )
    post_index = 0
    for x in (-2.10, 2.10):
        for y in (-3.20, -1.50):
            site.visual(
                Box((0.11, 0.11, 1.05)),
                origin=Origin(xyz=(x, y, 1.075)),
                material=STEEL,
                name=f"platform_post_{post_index}",
            )
            post_index += 1
    site.visual(
        Box((4.30, 0.095, 0.095)),
        origin=Origin(xyz=(0.0, -3.20, 1.60)),
        material=STEEL,
        name="front_platform_rail",
    )
    site.visual(
        Box((4.30, 0.095, 0.095)),
        origin=Origin(xyz=(0.0, -1.50, 1.60)),
        material=STEEL,
        name="rear_platform_rail",
    )

    # Two tall support legs cradle the rotating wheel, with a cross axle at the
    # top and foot pads tied into the platform/waterfront deck.
    for x, suffix in ((-3.55, "0"), (3.55, "1")):
        site.visual(
            Box((0.95, 2.25, 0.18)),
            origin=Origin(xyz=(x, 0.0, 0.09)),
            material=DARK_STEEL,
            name=f"foot_pad_{suffix}",
        )
        _add_sloped_cylinder(
            site,
            (x, -0.95, 0.18),
            (0.0, -0.72, AXLE_Z),
            0.165,
            STEEL,
            f"support_leg_{suffix}",
        )
        _add_sloped_cylinder(
            site,
            (x, 0.95, 0.18),
            (0.0, 0.72, AXLE_Z),
            0.130,
            STEEL,
            f"rear_brace_{suffix}",
        )
    _add_sloped_cylinder(
        site,
        (-3.55, 0.0, 0.22),
        (3.55, 0.0, 0.22),
        0.075,
        STEEL,
        "lower_cross_tie",
    )
    site.visual(
        Cylinder(radius=0.250, length=1.70),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK_STEEL,
        name="axle_pin",
    )
    for y, suffix in ((-0.78, "0"), (0.78, "1")):
        site.visual(
            Box((1.25, 0.20, 0.62)),
            origin=Origin(xyz=(0.0, y, AXLE_Z)),
            material=STEEL,
            name=f"axle_yoke_{suffix}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        _torus_mesh(RIM_RADIUS, 0.110, "outer_rim"),
        material=STEEL,
        name="outer_rim",
    )
    wheel.visual(
        _torus_mesh(4.25, 0.070, "inner_rim"),
        material=STEEL,
        name="inner_rim",
    )
    wheel.visual(
        Cylinder(radius=0.46, length=0.80),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK_STEEL,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.74, length=0.11),
        origin=Origin(xyz=(0.0, -0.455, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="front_hub_plate",
    )
    wheel.visual(
        Cylinder(radius=0.74, length=0.11),
        origin=Origin(xyz=(0.0, 0.455, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="rear_hub_plate",
    )
    _add_wheel_spokes(wheel)
    _add_hanger_mounts(wheel)
    axle_joint = model.articulation(
        "axle",
        ArticulationType.CONTINUOUS,
        parent=site,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.25),
    )

    for i in range(CABIN_COUNT):
        # Start with a cabin at the lowest boarding position, then distribute
        # the rest evenly around the rim.
        theta = -math.pi / 2.0 + 2.0 * math.pi * i / CABIN_COUNT
        pivot = (HANGER_RADIUS * math.cos(theta), 0.0, HANGER_RADIUS * math.sin(theta))
        cabin = model.part(f"cabin_{i}")
        _add_cabin_visuals(cabin)
        model.articulation(
            f"hanger_{i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=cabin,
            origin=Origin(xyz=pivot),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=60.0, velocity=0.8),
            mimic=Mimic(joint=axle_joint.name, multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    site = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    axle = object_model.get_articulation("axle")

    ctx.allow_overlap(
        site,
        wheel,
        elem_a="axle_pin",
        elem_b="hub_barrel",
        reason="The fixed axle pin is intentionally captured inside the rotating wheel hub.",
    )
    for hub_elem in ("front_hub_plate", "rear_hub_plate"):
        ctx.allow_overlap(
            site,
            wheel,
            elem_a="axle_pin",
            elem_b=hub_elem,
            reason="The axle pin intentionally passes through the solid hub-plate proxy.",
        )
    ctx.expect_within(
        site,
        wheel,
        axes="xz",
        inner_elem="axle_pin",
        outer_elem="hub_barrel",
        margin=0.010,
        name="axle pin is centered inside the hub barrel",
    )
    ctx.expect_overlap(
        site,
        wheel,
        axes="y",
        elem_a="axle_pin",
        elem_b="hub_barrel",
        min_overlap=0.30,
        name="axle pin spans through the wheel hub",
    )

    for i in range(CABIN_COUNT):
        cabin = object_model.get_part(f"cabin_{i}")
        ctx.allow_overlap(
            wheel,
            cabin,
            elem_a=f"hanger_mount_{i}",
            elem_b="pivot_pin",
            reason="The cabin pivot pin is intentionally captured by the wheel-side hanger mount.",
        )
        ctx.expect_overlap(
            wheel,
            cabin,
            axes="y",
            elem_a=f"hanger_mount_{i}",
            elem_b="pivot_pin",
            min_overlap=0.015,
            name=f"hanger mount {i} captures the cabin pivot pin",
        )
        ctx.expect_overlap(
            wheel,
            cabin,
            axes="xz",
            elem_a=f"hanger_mount_{i}",
            elem_b="pivot_pin",
            min_overlap=0.08,
            name=f"hanger mount {i} is aligned with the pivot pin",
        )

    ctx.check(
        "one continuous axle plus eight cabin pivots",
        len(object_model.articulations) == CABIN_COUNT + 1,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    bottom_cabin = object_model.get_part("cabin_0")
    top_cabin = object_model.get_part("cabin_4")
    ctx.expect_gap(
        bottom_cabin,
        site,
        axis="z",
        positive_elem="body_shell",
        negative_elem="boarding_bridge",
        min_gap=0.02,
        name="lowest cabin clears the boarding bridge",
    )
    ctx.expect_overlap(
        bottom_cabin,
        site,
        axes="xy",
        elem_a="body_shell",
        elem_b="boarding_bridge",
        min_overlap=0.18,
        name="lowest cabin aligns with the boarding bridge footprint",
    )

    rest_bottom = ctx.part_world_position(bottom_cabin)
    rest_top = ctx.part_world_position(top_cabin)
    with ctx.pose({axle: math.pi / 3.0}):
        turned_bottom = ctx.part_world_position(bottom_cabin)
        turned_top = ctx.part_world_position(top_cabin)
        ctx.expect_gap(
            bottom_cabin,
            site,
            axis="z",
            positive_elem="body_shell",
            negative_elem="water_base",
            min_gap=0.05,
            name="rotated cabin stays above the waterfront base",
        )

    ctx.check(
        "wheel rotation carries cabin pivots around the rim",
        rest_bottom is not None
        and turned_bottom is not None
        and abs(rest_bottom[0] - turned_bottom[0]) > 0.40
        and rest_top is not None
        and turned_top is not None
        and abs(rest_top[0] - turned_top[0]) > 0.40,
        details=f"bottom {rest_bottom}->{turned_bottom}, top {rest_top}->{turned_top}",
    )

    return ctx.report()


object_model = build_object_model()
