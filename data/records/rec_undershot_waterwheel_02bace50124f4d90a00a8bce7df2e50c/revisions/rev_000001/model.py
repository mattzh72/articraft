from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_undershot_waterwheel")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.58, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.02, 1.0))
    red = model.material("lockout_red", rgba=(0.82, 0.05, 0.03, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    water = model.material("water_blue", rgba=(0.08, 0.25, 0.42, 0.72))
    concrete = model.material("sealed_concrete", rgba=(0.34, 0.34, 0.32, 1.0))

    y_axis = (-math.pi / 2.0, 0.0, 0.0)
    x_axis = (0.0, math.pi / 2.0, 0.0)

    def radial_box_origin(radius: float, angle: float, y: float = 0.0) -> Origin:
        return Origin(
            xyz=(radius * math.cos(angle), y, radius * math.sin(angle)),
            rpy=(0.0, -angle, 0.0),
        )

    def tangent_box_origin(radius: float, angle: float, y: float = 0.0) -> Origin:
        return Origin(
            xyz=(radius * math.cos(angle), y, radius * math.sin(angle)),
            rpy=(0.0, math.pi / 2.0 - angle, 0.0),
        )

    def add_xz_bar(part, name: str, y: float, start: tuple[float, float], end: tuple[float, float], material) -> None:
        sx, sz = start
        ex, ez = end
        dx = ex - sx
        dz = ez - sz
        length = math.hypot(dx, dz)
        theta = math.atan2(-dz, dx)
        part.visual(
            Box((length, 0.075, 0.060)),
            origin=Origin(xyz=((sx + ex) / 2.0, y, (sz + ez) / 2.0), rpy=(0.0, theta, 0.0)),
            material=material,
            name=name,
        )

    frame = model.part("frame")

    # Connected fixed foundation, water race, side frames, guards, bearing housings,
    # stop brackets, and lockout guide.
    frame.visual(Box((2.20, 1.55, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.05)), material=concrete, name="base_slab")
    frame.visual(Box((1.95, 0.78, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.13)), material=water, name="water_race")
    frame.visual(Box((1.95, 0.060, 0.28)), origin=Origin(xyz=(0.0, 0.42, 0.24)), material=dark_steel, name="race_wall_0")
    frame.visual(Box((1.95, 0.060, 0.28)), origin=Origin(xyz=(0.0, -0.42, 0.24)), material=dark_steel, name="race_wall_1")

    for side_index, sy in enumerate((-0.55, 0.55)):
        frame.visual(Box((1.78, 0.11, 0.10)), origin=Origin(xyz=(0.0, sy, 0.15)), material=galvanized, name=f"side_sill_{side_index}")
        frame.visual(Box((1.78, 0.11, 0.10)), origin=Origin(xyz=(0.0, sy, 1.80)), material=galvanized, name=f"top_rail_{side_index}")
        for post_index, x in enumerate((-0.80, 0.80)):
            frame.visual(Box((0.10, 0.11, 1.70)), origin=Origin(xyz=(x, sy, 0.95)), material=galvanized, name=f"side_post_{side_index}_{post_index}")
            frame.visual(Box((0.30, 0.18, 0.055)), origin=Origin(xyz=(x, sy, 0.128)), material=dark_steel, name=f"foot_plate_{side_index}_{post_index}")

        frame.visual(Box((0.28, 0.16, 0.82)), origin=Origin(xyz=(0.0, sy, 0.51)), material=galvanized, name=f"bearing_pedestal_{side_index}")
        cap_y = sy - math.copysign(0.16, sy)
        cap_name = "bearing_cap_neg" if sy < 0.0 else "bearing_cap_pos"
        block_name = "bearing_block_neg" if sy < 0.0 else "bearing_block_pos"
        frame.visual(Cylinder(radius=0.18, length=0.12), origin=Origin(xyz=(0.0, cap_y, 1.00), rpy=y_axis), material=dark_steel, name=cap_name)
        frame.visual(Box((0.43, 0.20, 0.24)), origin=Origin(xyz=(0.0, sy, 1.00)), material=dark_steel, name=block_name)
        add_xz_bar(frame, f"diagonal_brace_{side_index}_0", sy, (-0.80, 0.20), (-0.16, 1.03), galvanized)
        add_xz_bar(frame, f"diagonal_brace_{side_index}_1", sy, (0.80, 0.20), (0.16, 1.03), galvanized)

        # Pillow-block cap bolts on the outer faces.
        bolt_y = sy + math.copysign(0.105, sy)
        for bolt_index, (bx, bz) in enumerate(((-0.14, 0.92), (0.14, 0.92), (-0.14, 1.08), (0.14, 1.08))):
            frame.visual(
                Cylinder(radius=0.022, length=0.014),
                origin=Origin(xyz=(bx, bolt_y, bz), rpy=y_axis),
                material=galvanized,
                name=f"bearing_bolt_{side_index}_{bolt_index}",
            )

    # Fixed guard hoops and cross ties.  They are close to the wheel but clear of
    # the rotating paddles, and their lower segment is tied into the base.
    guard_radius = 0.90
    guard_n = 16
    guard_len = 2.0 * math.pi * guard_radius / guard_n * 1.10
    for side_index, gy in enumerate((-0.34, 0.34)):
        for i in range(guard_n):
            a = 2.0 * math.pi * i / guard_n
            origin = tangent_box_origin(guard_radius, a, gy)
            origin = Origin(xyz=(origin.xyz[0], origin.xyz[1], origin.xyz[2] + 1.0), rpy=origin.rpy)
            frame.visual(
                Box((guard_len, 0.035, 0.045)),
                origin=origin,
                material=safety_yellow,
                name=f"guard_hoop_{side_index}_{i}",
            )
    frame.visual(Box((0.060, 0.72, 0.060)), origin=Origin(xyz=(0.90, 0.0, 1.00)), material=safety_yellow, name="guard_cross_side")
    frame.visual(Box((0.060, 0.72, 0.060)), origin=Origin(xyz=(-0.64, 0.0, 0.36)), material=safety_yellow, name="guard_lower_tie")
    frame.visual(Box((0.15, 0.72, 0.060)), origin=Origin(xyz=(0.0, 0.0, 1.90)), material=safety_yellow, name="guard_top_tie")

    # Positive over-travel bumpers: fixed brackets with rubber pads in the path
    # of rim lugs, mounted to the guard frame rather than floating near the wheel.
    for stop_index, sx in enumerate((-0.68, 0.68)):
        frame.visual(Box((0.20, 0.080, 0.055)), origin=Origin(xyz=(sx, -0.34, 1.66)), material=safety_yellow, name=f"stop_bracket_{stop_index}")
        frame.visual(Box((0.12, 0.090, 0.085)), origin=Origin(xyz=(sx, -0.34, 1.58)), material=rubber, name=f"stop_bumper_{stop_index}")

    # Lockout pin guide: a guarded red sliding pin sits in a fixed yoke carried
    # by a post tied into the base slab.
    frame.visual(Box((0.080, 0.080, 0.88)), origin=Origin(xyz=(-1.05, -0.37, 0.54)), material=safety_yellow, name="lockout_stand")
    frame.visual(Box((0.14, 0.14, 0.030)), origin=Origin(xyz=(-0.98, -0.37, 0.94)), material=safety_yellow, name="lockout_saddle")
    frame.visual(Box((0.22, 0.020, 0.11)), origin=Origin(xyz=(-1.05, -0.415, 1.00)), material=safety_yellow, name="lockout_cheek_0")
    frame.visual(Box((0.22, 0.020, 0.11)), origin=Origin(xyz=(-1.05, -0.325, 1.00)), material=safety_yellow, name="lockout_cheek_1")
    frame.visual(Box((0.12, 0.045, 0.045)), origin=Origin(xyz=(-0.86, -0.35, 1.00)), material=safety_yellow, name="receiver_arm")
    frame.visual(Box((0.075, 0.065, 0.13)), origin=Origin(xyz=(-0.78, -0.37, 1.00)), material=dark_steel, name="lockout_receiver")

    wheel = model.part("wheel")
    wheel.visual(Cylinder(radius=0.070, length=1.34), origin=Origin(rpy=y_axis), material=dark_steel, name="axle_shaft")
    wheel.visual(Cylinder(radius=0.160, length=0.58), origin=Origin(rpy=y_axis), material=dark_steel, name="hub_drum")
    for flange_index, y in enumerate((-0.305, 0.305)):
        wheel.visual(Cylinder(radius=0.205, length=0.035), origin=Origin(xyz=(0.0, y, 0.0), rpy=y_axis), material=galvanized, name=f"hub_flange_{flange_index}")

    # Two side rims, radial spoke plates, wide undershot paddles, and bolted rim
    # splice plates all rotate as one heavy-duty wheel on the central axle.
    rim_radius = 0.67
    rim_n = 16
    rim_len = 2.0 * math.pi * rim_radius / rim_n * 1.12
    for side_index, y in enumerate((-0.245, 0.245)):
        for i in range(rim_n):
            a = 2.0 * math.pi * i / rim_n
            wheel.visual(
                Box((rim_len, 0.060, 0.070)),
                origin=tangent_box_origin(rim_radius, a, y),
                material=galvanized,
                name=f"rim_{side_index}_{i}",
            )

        for i in range(8):
            a = 2.0 * math.pi * i / 8.0
            wheel.visual(
                Box((0.56, 0.050, 0.055)),
                origin=radial_box_origin(0.40, a, y),
                material=galvanized,
                name=f"spoke_{side_index}_{i}",
            )

        # Exposed fasteners on the rim side plates.
        bolt_y = y + math.copysign(0.034, y)
        for i in range(8):
            a = 2.0 * math.pi * (i + 0.5) / 8.0
            wheel.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(0.67 * math.cos(a), bolt_y, 0.67 * math.sin(a)), rpy=y_axis),
                material=dark_steel,
                name=f"rim_bolt_{side_index}_{i}",
            )

    for i in range(8):
        a = 2.0 * math.pi * (i + 0.5) / 8.0
        wheel.visual(
            Box((0.29, 0.56, 0.050)),
            origin=radial_box_origin(0.695, a, 0.0),
            material=dark_steel,
            name=f"paddle_{i}",
        )

    for lug_index, a in enumerate((math.radians(52.0), math.radians(128.0))):
        wheel.visual(
            Box((0.13, 0.070, 0.090)),
            origin=radial_box_origin(0.76, a, -0.29),
            material=galvanized,
            name=f"stop_lug_{lug_index}",
        )

    lockout_pin = model.part("lockout_pin")
    lockout_pin.visual(Cylinder(radius=0.022, length=0.25), origin=Origin(rpy=x_axis), material=red, name="pin_shaft")
    lockout_pin.visual(Cylinder(radius=0.018, length=0.18), origin=Origin(xyz=(-0.11, 0.0, 0.0)), material=red, name="tee_handle")
    lockout_pin.visual(Box((0.045, 0.012, 0.070)), origin=Origin(xyz=(-0.11, -0.022, -0.070)), material=safety_yellow, name="lockout_tag")

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=850.0, velocity=2.0),
    )

    model.articulation(
        "frame_to_lockout_pin",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lockout_pin,
        origin=Origin(xyz=(-1.05, -0.37, 1.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.15, lower=0.0, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    lockout_pin = object_model.get_part("lockout_pin")
    wheel_joint = object_model.get_articulation("frame_to_wheel")
    pin_joint = object_model.get_articulation("frame_to_lockout_pin")

    # The axle is intentionally captured by solid pillow-block proxies.  The
    # scoped allowances are paired with containment and insertion checks.
    for cap_name in ("bearing_cap_neg", "bearing_cap_pos", "bearing_block_neg", "bearing_block_pos"):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=cap_name,
            elem_b="axle_shaft",
            reason="The rotating axle is intentionally seated inside the side-bearing support proxy.",
        )
        ctx.expect_within(
            wheel,
            frame,
            axes="xz",
            inner_elem="axle_shaft",
            outer_elem=cap_name,
            margin=0.002,
            name=f"axle centered in {cap_name}",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="y",
            elem_a="axle_shaft",
            elem_b=cap_name,
            min_overlap=0.05,
            name=f"axle inserted through {cap_name}",
        )

    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: 1.0}):
        spun_wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spins about fixed centered axle",
        rest_wheel_pos is not None and spun_wheel_pos is not None and abs(rest_wheel_pos[2] - spun_wheel_pos[2]) < 1e-6,
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    rest_pin = ctx.part_world_position(lockout_pin)
    with ctx.pose({pin_joint: 0.10}):
        extended_pin = ctx.part_world_position(lockout_pin)
        ctx.expect_gap(
            frame,
            lockout_pin,
            axis="x",
            positive_elem="lockout_receiver",
            negative_elem="pin_shaft",
            min_gap=0.0,
            max_gap=0.015,
            name="lockout pin reaches receiver without collision",
        )
    ctx.check(
        "lockout pin slides toward receiver",
        rest_pin is not None and extended_pin is not None and extended_pin[0] > rest_pin[0] + 0.09,
        details=f"rest={rest_pin}, extended={extended_pin}",
    )

    return ctx.report()


object_model = build_object_model()
