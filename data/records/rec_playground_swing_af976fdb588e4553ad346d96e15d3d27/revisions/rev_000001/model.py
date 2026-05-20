from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
)


def _origin_between(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an Origin and length for a cylinder whose local +Z spans a -> b."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    radial = sqrt(dx * dx + dy * dy)
    pitch = atan2(radial, dz)
    yaw = atan2(dy, dx)
    midpoint = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)
    return Origin(xyz=midpoint, rpy=(0.0, pitch, yaw)), length


def _add_tube(part, name: str, a, b, radius: float, material) -> None:
    origin, length = _origin_between(a, b)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tire_gantry_swing")

    blue = model.material("powder_coated_blue", rgba=(0.08, 0.27, 0.68, 1.0))
    dark_blue = model.material("dark_blue_end_caps", rgba=(0.03, 0.08, 0.18, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    black_rubber = model.material("weathered_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    yellow = model.material("yellow_safety_clamps", rgba=(0.95, 0.68, 0.08, 1.0))

    frame = model.part("frame")

    # One welded gantry frame: A-frame side supports, ground rails, and a round top beam.
    _add_tube(frame, "top_beam", (-1.45, 0.0, 2.35), (1.45, 0.0, 2.35), 0.055, blue)
    for x, stem in ((-1.25, "support_0"), (1.25, "support_1")):
        _add_tube(frame, f"{stem}_front_leg", (x, 0.68, 0.07), (x, 0.0, 2.34), 0.045, blue)
        _add_tube(frame, f"{stem}_rear_leg", (x, -0.68, 0.07), (x, 0.0, 2.34), 0.045, blue)
        _add_tube(frame, f"{stem}_foot", (x, -0.78, 0.07), (x, 0.78, 0.07), 0.040, blue)

    _add_tube(frame, "front_ground_rail", (-1.25, 0.68, 0.07), (1.25, 0.68, 0.07), 0.034, dark_blue)
    _add_tube(frame, "rear_ground_rail", (-1.25, -0.68, 0.07), (1.25, -0.68, 0.07), 0.034, dark_blue)
    _add_tube(frame, "upper_brace_front", (-1.25, 0.20, 1.74), (1.25, 0.20, 1.74), 0.030, dark_blue)
    _add_tube(frame, "upper_brace_rear", (-1.25, -0.20, 1.74), (1.25, -0.20, 1.74), 0.030, dark_blue)

    # Clevis plates hang from the beam and carry a fixed axle captured by the swing sleeve.
    for x in (-0.45, 0.45):
        frame.visual(
            Box((0.050, 0.120, 0.215)),
            origin=Origin(xyz=(x, 0.0, 2.245)),
            material=blue,
            name=f"clevis_plate_{0 if x < 0 else 1}",
        )
    _add_tube(frame, "pivot_pin", (-0.50, 0.0, 2.18), (0.50, 0.0, 2.18), 0.018, galvanized)

    swing = model.part("tire_assembly")
    # Child frame is the top pivot axis.  All hanger geometry is below it at q=0.
    _add_tube(swing, "pivot_sleeve", (-0.38, 0.0, 0.0), (0.38, 0.0, 0.0), 0.032, galvanized)

    tire = TireGeometry(
        0.390,
        0.165,
        inner_radius=0.235,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.10),
        tread=TireTread(style="block", depth=0.012, count=28, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.020, depth=0.005),),
        sidewall=TireSidewall(style="rounded", bulge=0.07),
        shoulder=TireShoulder(width=0.018, radius=0.006),
    )
    swing.visual(
        mesh_from_geometry(tire, "horizontal_playground_tire"),
        origin=Origin(xyz=(0.0, 0.0, -1.300), rpy=(0.0, -pi / 2.0, 0.0)),
        material=black_rubber,
        name="tire",
    )

    top_points = [(-0.28, 0.0, -0.055), (0.0, 0.0, -0.055), (0.28, 0.0, -0.055)]
    tire_points = [(-0.28, -0.18, -1.210), (0.0, 0.325, -1.210), (0.28, -0.18, -1.210)]
    for i, (top, bottom) in enumerate(zip(top_points, tire_points)):
        _add_tube(swing, f"sleeve_lug_{i}", (top[0], top[1], -0.031), top, 0.012, galvanized)
        _add_tube(swing, f"hanger_{i}", top, bottom, 0.017, galvanized)
        swing.visual(Sphere(0.040), origin=Origin(xyz=bottom), material=yellow, name=f"tire_clamp_{i}")

    model.articulation(
        "frame_to_tire",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing,
        origin=Origin(xyz=(0.0, 0.0, 2.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    swing = object_model.get_part("tire_assembly")
    hinge = object_model.get_articulation("frame_to_tire")

    ctx.allow_overlap(
        frame,
        swing,
        elem_a="pivot_pin",
        elem_b="pivot_sleeve",
        reason="The fixed axle is intentionally captured inside the rotating swing sleeve.",
    )
    ctx.expect_within(
        frame,
        swing,
        axes="yz",
        inner_elem="pivot_pin",
        outer_elem="pivot_sleeve",
        margin=0.001,
        name="pivot pin is concentric with sleeve",
    )
    ctx.expect_overlap(
        frame,
        swing,
        axes="x",
        elem_a="pivot_pin",
        elem_b="pivot_sleeve",
        min_overlap=0.70,
        name="pivot sleeve spans the captured axle",
    )

    ctx.expect_gap(
        swing,
        swing,
        axis="z",
        positive_elem="pivot_sleeve",
        negative_elem="tire",
        min_gap=1.05,
        name="hanger drop separates beam pivot from tire seat",
    )
    ctx.expect_within(
        swing,
        frame,
        axes="x",
        inner_elem="tire",
        outer_elem="top_beam",
        margin=0.02,
        name="tire hangs within top beam span",
    )

    rest_box = ctx.part_element_world_aabb(swing, elem="tire")
    rest_y = (rest_box[0][1] + rest_box[1][1]) * 0.5 if rest_box is not None else None
    with ctx.pose({hinge: 0.45}):
        swung_box = ctx.part_element_world_aabb(swing, elem="tire")
        swung_y = (swung_box[0][1] + swung_box[1][1]) * 0.5 if swung_box is not None else None
        ctx.expect_gap(
            swing,
            swing,
            axis="z",
            positive_elem="pivot_sleeve",
            negative_elem="tire",
            min_gap=0.90,
            name="swing arc keeps tire below pivot",
        )
    ctx.check(
        "positive hinge swings tire forward",
        rest_y is not None and swung_y is not None and swung_y > rest_y + 0.25,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()
