from __future__ import annotations

import math

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
    TorusGeometry,
    mesh_from_geometry,
)


def _bar_between(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    depth_y: float,
    thickness: float,
    material: Material,
) -> None:
    """Add a rectangular timber/strap whose long local X axis joins two XZ points."""
    sx, sy, sz = start
    ex, ey, ez = end
    if abs(sy - ey) > 1e-9:
        raise ValueError("_bar_between is for bars lying in a constant-Y plane")
    dx = ex - sx
    dz = ez - sz
    length = math.hypot(dx, dz)
    angle = math.atan2(dz, dx)
    part.visual(
        Box((length, depth_y, thickness)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, sy, (sz + ez) * 0.5),
            rpy=(0.0, -angle, 0.0),
        ),
        material=material,
        name=name,
    )


def _xz_torus(name: str, radius: float, tube: float):
    geom = TorusGeometry(radius, tube, radial_segments=20, tubular_segments=64)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    weathered_wood = model.material("weathered_wood", rgba=(0.47, 0.30, 0.16, 1.0))
    dark_wood = model.material("dark_wet_wood", rgba=(0.30, 0.18, 0.09, 1.0))
    iron = model.material("blackened_iron", rgba=(0.05, 0.055, 0.055, 1.0))
    stone = model.material("rough_stone", rgba=(0.40, 0.39, 0.36, 1.0))
    water = model.material("channel_water", rgba=(0.10, 0.34, 0.55, 0.55))

    axle_z = 0.82
    frame_y = 0.42
    wheel_radius = 0.54
    paddle_radius = 0.575

    frame = model.part("frame")

    # Low masonry/wood sluice: the undershot wheel sits in this channel and is
    # struck at its lower paddles by water running under it.
    frame.visual(
        Box((1.60, 0.62, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=stone,
        name="channel_floor",
    )
    frame.visual(
        Box((1.48, 0.52, 0.018)),
        origin=Origin(xyz=(-0.02, 0.0, 0.058)),
        material=water,
        name="water_run",
    )
    for side, y in enumerate((-0.34, 0.34)):
        frame.visual(
            Box((1.62, 0.055, 0.20)),
            origin=Origin(xyz=(0.0, y, 0.13)),
            material=stone,
            name=f"channel_wall_{side}",
        )
    frame.visual(
        Box((0.10, 0.74, 0.10)),
        origin=Origin(xyz=(-0.74, 0.0, 0.23)),
        material=dark_wood,
        name="trough_lip",
    )

    # Foot sills tie the two side frames together so the whole fixed support is
    # one physical assembly.
    for i, x in enumerate((-0.58, 0.58)):
        frame.visual(
            Box((0.12, 0.92, 0.08)),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material=weathered_wood,
            name=f"foot_sill_{i}",
        )

    # Two A-frame side supports, one on each side of the wheel.
    for side_name, y in (("front", -frame_y), ("rear", frame_y)):
        _bar_between(
            frame,
            f"{side_name}_leg_0",
            (-0.58, y, 0.095),
            (-0.11, y, axle_z - 0.070),
            depth_y=0.065,
            thickness=0.065,
            material=weathered_wood,
        )
        _bar_between(
            frame,
            f"{side_name}_leg_1",
            (0.58, y, 0.095),
            (0.11, y, axle_z - 0.070),
            depth_y=0.065,
            thickness=0.065,
            material=weathered_wood,
        )
        _bar_between(
            frame,
            f"{side_name}_cross_tie",
            (-0.42, y, 0.35),
            (0.42, y, 0.35),
            depth_y=0.060,
            thickness=0.055,
            material=weathered_wood,
        )
        frame.visual(
            Box((0.24, 0.070, 0.050)),
            origin=Origin(xyz=(0.0, y, axle_z - 0.058)),
            material=weathered_wood,
            name=f"{side_name}_bearing_block",
        )
        frame.visual(
            _xz_torus(f"{side_name}_bearing_ring_mesh", 0.080, 0.024),
            origin=Origin(xyz=(0.0, y, axle_z)),
            material=iron,
            name=f"{side_name}_bearing_ring",
        )

    # Fixed guard hoop on the front side: a secondary non-rotating frame around
    # the moving wheel with a clear gap from the paddle tips.
    guard_y = -0.305
    frame.visual(
        _xz_torus("guard_hoop_mesh", 0.710, 0.022),
        origin=Origin(xyz=(0.0, guard_y, axle_z)),
        material=iron,
        name="guard_hoop",
    )
    frame.visual(
        _xz_torus("guard_hub_ring_mesh", 0.082, 0.020),
        origin=Origin(xyz=(0.0, guard_y, axle_z)),
        material=iron,
        name="guard_hub_ring",
    )
    # Standoffs join the guard hub ring to the front bearing without crossing
    # the rotating axle.
    for i, z in enumerate((axle_z - 0.105, axle_z + 0.105)):
        frame.visual(
            Box((0.040, frame_y - abs(guard_y) + 0.040, 0.035)),
            origin=Origin(xyz=(0.0, (guard_y - frame_y) * 0.5, z)),
            material=iron,
            name=f"guard_standoff_{i}",
        )
    for i, theta in enumerate((0.0, math.pi * 0.5, math.pi, math.pi * 1.5)):
        inner = 0.115
        outer = 0.690
        _bar_between(
            frame,
            f"guard_spoke_{i}",
            (math.cos(theta) * inner, guard_y, axle_z + math.sin(theta) * inner),
            (math.cos(theta) * outer, guard_y, axle_z + math.sin(theta) * outer),
            depth_y=0.034,
            thickness=0.034,
            material=iron,
        )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.033, length=1.04),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.105, length=0.56),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_wood,
        name="hub",
    )
    for side, y in enumerate((-0.215, 0.215)):
        wheel.visual(
            _xz_torus(f"rim_{side}_mesh", wheel_radius - 0.050, 0.025),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_wood,
            name=f"rim_{side}",
        )
        for i in range(8):
            theta = i * math.tau / 8.0
            radius_mid = 0.305
            _bar_between(
                wheel,
                f"spoke_{side}_{i}",
                (math.cos(theta) * 0.085, y, math.sin(theta) * 0.085),
                (math.cos(theta) * 0.485, y, math.sin(theta) * 0.485),
                depth_y=0.055,
                thickness=0.038,
                material=weathered_wood,
            )

    # Broad paddle boards cross the full wheel width and touch both rim rings.
    for i in range(12):
        theta = i * math.tau / 12.0
        wheel.visual(
            Box((0.070, 0.500, 0.130)),
            origin=Origin(
                xyz=(math.cos(theta) * paddle_radius, 0.0, math.sin(theta) * paddle_radius),
                rpy=(0.0, math.pi / 2.0 - theta, 0.0),
            ),
            material=weathered_wood,
            name=f"paddle_{i}",
        )

    model.articulation(
        "axle_joint",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    axle_joint = object_model.get_articulation("axle_joint")

    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        outer_elem="guard_hoop",
        margin=0.03,
        name="paddle wheel sits inside fixed guard hoop",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="front_bearing_ring",
        margin=0.0,
        name="axle is centered in front bearing",
    )

    rest_paddle = ctx.part_element_world_aabb(wheel, elem="paddle_0")
    with ctx.pose({axle_joint: math.pi / 2.0}):
        turned_paddle = ctx.part_element_world_aabb(wheel, elem="paddle_0")
        ctx.expect_within(
            wheel,
            frame,
            axes="xz",
            outer_elem="guard_hoop",
            margin=0.03,
            name="rotated wheel remains inside guard hoop",
        )
    ctx.check(
        "continuous joint moves paddle around axle",
        rest_paddle is not None
        and turned_paddle is not None
        and turned_paddle[0][2] < rest_paddle[0][2] - 0.35,
        details=f"rest={rest_paddle}, turned={turned_paddle}",
    )

    return ctx.report()


object_model = build_object_model()
