from __future__ import annotations

import math

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
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


AXLE_Z = 0.88
WHEEL_OUTER_RADIUS = 0.78
PADDLE_COUNT = 16


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_undershot_waterwheel")

    painted_steel = model.material("oxide_painted_steel", rgba=(0.10, 0.16, 0.18, 1.0))
    worn_edges = model.material("worn_bright_edges", rgba=(0.58, 0.62, 0.58, 1.0))
    dark_bearing = model.material("dark_bearing_housings", rgba=(0.035, 0.038, 0.04, 1.0))
    paddle_poly = model.material("molded_paddle_polymer", rgba=(0.93, 0.56, 0.12, 1.0))
    fastener_metal = model.material("zinc_fasteners", rgba=(0.72, 0.74, 0.70, 1.0))
    channel_blue = model.material("shallow_water_cue", rgba=(0.18, 0.40, 0.72, 0.45))

    frame = model.part("frame")

    def add_frame_box(name: str, size, xyz, *, rpy=(0.0, 0.0, 0.0), material=painted_steel):
        frame.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def add_yz_brace(name: str, x: float, y0: float, z0: float, y1: float, z1: float):
        dy = y1 - y0
        dz = z1 - z0
        length = math.sqrt(dy * dy + dz * dz)
        angle = math.atan2(-dy, dz)
        add_frame_box(
            name,
            (0.085, 0.075, length),
            (x, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
            rpy=(angle, 0.0, 0.0),
        )

    # Broad skid base and low flume channel: the wheel is intentionally set low
    # so the lower paddles skim a shallow flow path rather than needing a deep
    # head of water.
    for x, suffix in ((-0.58, "0"), (0.58, "1")):
        add_frame_box(f"base_skid_{suffix}", (0.13, 1.72, 0.10), (x, 0.0, 0.05))
        add_frame_box(f"side_post_{suffix}_0", (0.13, 0.045, 1.12), (x, -0.112, 0.56))
        add_frame_box(f"side_post_{suffix}_1", (0.13, 0.045, 1.12), (x, 0.112, 0.56))
        add_frame_box(f"top_rail_{suffix}", (0.13, 1.72, 0.075), (x, 0.0, 1.04))
        add_yz_brace(f"front_brace_{suffix}", x, -0.74, 0.11, -0.13, AXLE_Z - 0.14)
        add_yz_brace(f"rear_brace_{suffix}", x, 0.74, 0.11, 0.13, AXLE_Z - 0.14)

    for y, suffix in ((-0.78, "0"), (0.78, "1")):
        add_frame_box(f"floor_crossbeam_{suffix}", (1.32, 0.11, 0.10), (0.0, y, 0.10))
        add_frame_box(f"upper_cross_tie_{suffix}", (1.20, 0.075, 0.075), (0.0, y * 1.08, 1.04))

    add_frame_box("flow_pan", (0.78, 1.36, 0.035), (0.0, 0.0, 0.018), material=worn_edges)
    add_frame_box("water_strip", (0.66, 1.24, 0.018), (0.0, 0.0, 0.0445), material=channel_blue)
    add_frame_box("flow_lip_0", (0.055, 1.36, 0.13), (-0.395, 0.0, 0.085), material=worn_edges)
    add_frame_box("flow_lip_1", (0.055, 1.36, 0.13), (0.395, 0.0, 0.085), material=worn_edges)
    add_frame_box("pan_side_mount_0", (0.17, 1.36, 0.050), (-0.485, 0.0, 0.030), material=worn_edges)
    add_frame_box("pan_side_mount_1", (0.17, 1.36, 0.050), (0.485, 0.0, 0.030), material=worn_edges)

    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.070, tube=0.030, radial_segments=18, tubular_segments=48),
        "bearing_collar",
    )
    for x, suffix in ((-0.58, "0"), (0.58, "1")):
        add_frame_box(f"pillow_block_{suffix}", (0.15, 0.26, 0.15), (x, 0.0, AXLE_Z - 0.155), material=dark_bearing)
        add_frame_box(f"cap_cheek_{suffix}_0", (0.16, 0.040, 0.055), (x, -0.087, AXLE_Z - 0.035), material=dark_bearing)
        add_frame_box(f"cap_cheek_{suffix}_1", (0.16, 0.040, 0.055), (x, 0.087, AXLE_Z - 0.035), material=dark_bearing)
        add_frame_box(f"cap_bridge_{suffix}", (0.16, 0.19, 0.040), (x, 0.0, AXLE_Z + 0.075), material=dark_bearing)
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, 0.0, AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_bearing,
            name=f"bearing_collar_{suffix}",
        )
        for y in (-0.075, 0.075):
            frame.visual(
                Cylinder(radius=0.018, length=0.020),
                origin=Origin(xyz=(x, y, AXLE_Z - 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=fastener_metal,
                name=f"bearing_bolt_{suffix}_{'n' if y < 0 else 'p'}",
            )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.675,
                0.38,
                rim=WheelRim(
                    inner_radius=0.575,
                    flange_height=0.030,
                    flange_thickness=0.018,
                    bead_seat_depth=0.010,
                ),
                hub=WheelHub(
                    radius=0.110,
                    width=0.300,
                    cap_style="flat",
                    bolt_pattern=BoltPattern(count=8, circle_diameter=0.160, hole_diameter=0.018),
                ),
                face=WheelFace(dish_depth=0.015, front_inset=0.010, rear_inset=0.010),
                spokes=WheelSpokes(style="split_y", count=8, thickness=0.022, window_radius=0.055),
                bore=WheelBore(style="round", diameter=0.090),
            ),
            "painted_wheel_rim",
        ),
        material=painted_steel,
        name="rim_spoke_hub",
    )

    wheel.visual(
        Cylinder(radius=0.045, length=1.34),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edges,
        name="spindle",
    )

    for x, suffix in ((-0.235, "0"), (0.235, "1")):
        wheel.visual(
            Cylinder(radius=0.165, length=0.045),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=painted_steel,
            name=f"flange_plate_{suffix}",
        )
        for i in range(8):
            theta = 2.0 * math.pi * i / 8.0 + (math.pi / 8.0)
            y = 0.130 * math.sin(theta)
            z = 0.130 * math.cos(theta)
            wheel.visual(
                Cylinder(radius=0.012, length=0.020),
                origin=Origin(xyz=(x + (0.030 if x > 0 else -0.030), y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=fastener_metal,
                name=f"flange_bolt_{suffix}_{i}",
            )

    paddle_center_radius = 0.705
    for i in range(PADDLE_COUNT):
        theta = 2.0 * math.pi * i / PADDLE_COUNT
        y = paddle_center_radius * math.sin(theta)
        z = paddle_center_radius * math.cos(theta)
        wheel.visual(
            Box((0.46, 0.060, 0.205)),
            origin=Origin(xyz=(0.0, y, z), rpy=(-theta, 0.0, 0.0)),
            material=paddle_poly,
            name=f"paddle_{i}",
        )
        # Molded ribs and clamp bars make each paddle look replaceable and stiff
        # rather than like a thin decorative plank.
        wheel.visual(
            Box((0.49, 0.035, 0.038)),
            origin=Origin(xyz=(0.0, y * 0.985, z * 0.985), rpy=(-theta, 0.0, 0.0)),
            material=painted_steel,
            name=f"paddle_clamp_{i}",
        )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("frame_to_wheel")

    ctx.check(
        "wheel spins on horizontal axle",
        tuple(spin.axis) == (1.0, 0.0, 0.0) and spin.motion_limits is not None,
        details=f"axis={spin.axis}, limits={spin.motion_limits}",
    )

    for suffix in ("0", "1"):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=f"bearing_collar_{suffix}",
            elem_b="spindle",
            reason="The steel spindle is intentionally captured in a simplified bearing collar bore.",
        )
        ctx.expect_within(
            wheel,
            frame,
            axes="yz",
            inner_elem="spindle",
            outer_elem=f"bearing_collar_{suffix}",
            margin=0.0,
            name=f"spindle centered in bearing {suffix}",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="x",
            elem_a="spindle",
            elem_b=f"bearing_collar_{suffix}",
            min_overlap=0.020,
            name=f"spindle passes through bearing {suffix}",
        )

    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem="paddle_8",
        negative_elem="water_strip",
        min_gap=0.010,
        max_gap=0.090,
        name="lower paddle skims shallow flow",
    )

    return ctx.report()


object_model = build_object_model()
