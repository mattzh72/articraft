from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _ring_wire(radius: float, x: float, wire_radius: float, samples: int = 28):
    points = [
        (
            x,
            radius * math.cos((2.0 * math.pi * i) / samples),
            radius * math.sin((2.0 * math.pi * i) / samples),
        )
        for i in range(samples)
    ]
    return tube_from_spline_points(
        points,
        radius=wire_radius,
        samples_per_segment=4,
        closed_spline=True,
        radial_segments=14,
        cap_ends=False,
    )


def _fan_guard_geometry():
    guard = _ring_wire(0.156, 0.030, 0.0045)
    guard.merge(_ring_wire(0.160, 0.072, 0.0045))
    guard.merge(_ring_wire(0.162, 0.112, 0.0045))

    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        y = 0.158 * math.cos(angle)
        z = 0.158 * math.sin(angle)
        guard.merge(
            tube_from_spline_points(
                [(0.028, y, z), (0.072, y, z), (0.114, y, z)],
                radius=0.0025,
                samples_per_segment=6,
                radial_segments=10,
                cap_ends=True,
            )
        )

    for index in range(4):
        angle = (2.0 * math.pi * index) / 4.0
        c = math.cos(angle)
        s = math.sin(angle)
        guard.merge(
            tube_from_spline_points(
                [
                    (0.020, 0.046 * c, 0.046 * s),
                    (0.026, 0.100 * c, 0.100 * s),
                    (0.032, 0.156 * c, 0.156 * s),
                ],
                radius=0.0035,
                samples_per_segment=8,
                radial_segments=10,
                cap_ends=True,
            )
        )

    for index in range(6):
        angle = (2.0 * math.pi * index) / 6.0
        c = math.cos(angle)
        s = math.sin(angle)
        guard.merge(
            tube_from_spline_points(
                [
                    (0.104, 0.018 * c, 0.018 * s),
                    (0.108, 0.090 * c, 0.090 * s),
                    (0.112, 0.156 * c, 0.156 * s),
                ],
                radius=0.0032,
                samples_per_segment=8,
                radial_segments=10,
                cap_ends=True,
            )
        )

    guard.merge(CylinderGeometry(radius=0.023, height=0.010).rotate_y(math.pi / 2.0).translate(0.104, 0.0, 0.0))
    return guard


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    base_plastic = model.material("base_plastic", rgba=(0.16, 0.17, 0.19, 1.0))
    fan_gray = model.material("fan_gray", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    blade_blue = model.material("blade_blue", rgba=(0.50, 0.62, 0.74, 0.92))

    base = model.part("base")
    base_plate = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.23, 0.19, 0.04), 0.028),
        "base_plate",
    )
    base.visual(
        base_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=base_plastic,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.28),
        origin=Origin(xyz=(-0.04, 0.0, 0.168)),
        material=base_plastic,
        name="support_column",
    )
    base.visual(
        Box((0.045, 0.172, 0.04)),
        origin=Origin(xyz=(-0.040, 0.0, 0.326)),
        material=base_plastic,
        name="yoke_bridge",
    )
    base.visual(
        Box((0.11, 0.012, 0.145)),
        origin=Origin(xyz=(0.008, 0.076, 0.3475)),
        material=fan_gray,
        name="left_bracket",
    )
    base.visual(
        Box((0.11, 0.012, 0.145)),
        origin=Origin(xyz=(0.008, -0.076, 0.3475)),
        material=fan_gray,
        name="right_bracket",
    )
    base.visual(
        Box((0.026, 0.172, 0.070)),
        origin=Origin(xyz=(-0.072, 0.0, 0.342)),
        material=fan_gray,
        name="rear_knuckle",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.23, 0.19, 0.40)),
        mass=2.6,
        origin=Origin(xyz=(-0.01, 0.0, 0.20)),
    )

    head = model.part("head")
    guard_cage = mesh_from_geometry(_fan_guard_geometry(), "guard_cage")
    head.visual(
        Cylinder(radius=0.016, length=0.140),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="pivot_barrel",
    )
    head.visual(
        Cylinder(radius=0.060, length=0.095),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fan_gray,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.022),
        origin=Origin(xyz=(-0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="bearing_nose",
    )
    head.visual(
        guard_cage,
        material=dark_trim,
        name="guard_cage",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.25, 0.34, 0.34)),
        mass=1.4,
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fan_gray,
        name="axle_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.028, length=0.026),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="spinner_cap",
    )
    rotor.visual(
        Box((0.012, 0.124, 0.050)),
        origin=Origin(xyz=(0.026, 0.067, 0.0), rpy=(0.0, 0.20, 0.0)),
        material=blade_blue,
        name="blade_0",
    )
    rotor.visual(
        Box((0.012, 0.124, 0.050)),
        origin=Origin(
            xyz=(0.026, -0.0335, 0.058),
            rpy=(2.0 * math.pi / 3.0, 0.20, 0.0),
        ),
        material=blade_blue,
        name="blade_1",
    )
    rotor.visual(
        Box((0.012, 0.124, 0.050)),
        origin=Origin(
            xyz=(0.026, -0.0335, -0.058),
            rpy=(-2.0 * math.pi / 3.0, 0.20, 0.0),
        ),
        material=blade_blue,
        name="blade_2",
    )
    rotor.inertial = Inertial.from_geometry(Cylinder(radius=0.125, length=0.028), mass=0.18)

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.058, 0.0, 0.37)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=math.radians(-20.0),
            upper=math.radians(30.0),
        ),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=35.0),
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
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    ctx.expect_origin_gap(
        head,
        base,
        axis="z",
        min_gap=0.30,
        name="tilt axis sits high above the base",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        margin=0.02,
        inner_elem="hub",
        outer_elem="guard_cage",
        name="rotor hub stays centered within the guard envelope",
    )
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({tilt: tilt.motion_limits.lower}):
        low_pos = ctx.part_world_position(rotor)
    ctx.check(
        "positive tilt raises the fan head",
        rest_pos is not None and low_pos is not None and rest_pos[2] > low_pos[2] + 0.02,
        details=f"upper={rest_pos}, lower={low_pos}",
    )
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            rotor,
            head,
            axes="yz",
            max_dist=1e-6,
            name="rotor stays on the motor axis while spinning",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
