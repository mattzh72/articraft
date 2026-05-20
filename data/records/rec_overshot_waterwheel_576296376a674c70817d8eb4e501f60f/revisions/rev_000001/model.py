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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rural_overshot_waterwheel")

    weathered_oak = model.material("weathered_oak", color=(0.48, 0.31, 0.17, 1.0))
    dark_oak = model.material("dark_wet_oak", color=(0.30, 0.19, 0.10, 1.0))
    iron = model.material("blackened_iron", color=(0.05, 0.05, 0.045, 1.0))
    water = model.material("shallow_water", color=(0.22, 0.45, 0.70, 0.45))

    frame = model.part("timber_frame")

    def timber(name: str, size: tuple[float, float, float], xyz: tuple[float, float, float]) -> None:
        frame.visual(Box(size), origin=Origin(xyz=xyz), material=weathered_oak, name=name)

    # Two long sills and four upright posts make a simple rural timber trestle.
    for x in (-0.62, 0.62):
        timber(f"sill_{x:+.0f}", (0.16, 2.05, 0.12), (x, -0.15, 0.06))
        timber(f"axle_post_{x:+.0f}", (0.16, 0.18, 1.99), (x, 0.0, 1.055))
        timber(f"launder_post_{x:+.0f}", (0.14, 0.14, 2.02), (x, -0.95, 1.07))

    timber("top_tie", (1.40, 0.28, 0.12), (0.0, 0.20, 2.04))
    timber("rear_tie", (1.40, 0.16, 0.12), (0.0, -0.95, 2.06))
    timber("base_cross", (1.40, 0.14, 0.12), (0.0, -1.10, 0.12))

    # Open bearing saddles under the rotating axle; the axle rests on these blocks.
    frame.visual(
        Box((0.16, 0.22, 0.09)),
        origin=Origin(xyz=(-0.50, 0.0, 0.950)),
        material=weathered_oak,
        name="bearing_0",
    )
    frame.visual(
        Box((0.16, 0.22, 0.09)),
        origin=Origin(xyz=(0.50, 0.0, 0.950)),
        material=weathered_oak,
        name="bearing_1",
    )

    # A narrow open launder: bottom board, two side boards, and a closed upstream end.
    frame.visual(
        Box((0.34, 1.135, 0.05)),
        origin=Origin(xyz=(0.0, -0.6725, 2.13)),
        material=weathered_oak,
        name="launder_bottom",
    )
    frame.visual(
        Box((0.05, 1.135, 0.24)),
        origin=Origin(xyz=(-0.195, -0.6725, 2.245)),
        material=weathered_oak,
        name="launder_side_0",
    )
    frame.visual(
        Box((0.05, 1.135, 0.24)),
        origin=Origin(xyz=(0.195, -0.6725, 2.245)),
        material=weathered_oak,
        name="launder_side_1",
    )
    timber("launder_end", (0.34, 0.05, 0.20), (0.0, -1.24, 2.245))
    frame.visual(
        Box((0.24, 0.82, 0.020)),
        origin=Origin(xyz=(0.0, -0.72, 2.165)),
        material=water,
        name="water_in_launder",
    )

    wheel = model.part("wheel")
    for x, name in ((-0.23, "rim_0"), (0.23, "rim_1")):
        rim = TorusGeometry(radius=0.75, tube=0.035, radial_segments=20, tubular_segments=64)
        rim.rotate_y(math.pi / 2.0).translate(x, 0.0, 0.0)
        wheel.visual(mesh_from_geometry(rim, name), material=dark_oak, name=name)

    wheel.visual(
        Cylinder(radius=0.055, length=1.00),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.13, length=0.56),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_oak,
        name="hub",
    )

    # Radial spokes and bucket boards are authored in the wheel frame, whose X axis
    # is the axle.  The buckets span the wheel width and overlap both side rims.
    for i in range(8):
        theta = i * math.tau / 8.0
        roll = theta - math.pi / 2.0
        wheel.visual(
            Box((0.12, 0.045, 0.74)),
            origin=Origin(
                xyz=(0.0, 0.38 * math.cos(theta), 0.38 * math.sin(theta)),
                rpy=(roll, 0.0, 0.0),
            ),
            material=weathered_oak,
            name=f"spoke_{i}",
        )

    for i in range(16):
        theta = i * math.tau / 16.0
        roll = theta - math.pi / 2.0
        bucket_name = "bucket_4" if i == 4 else f"bucket_{i}"
        wheel.visual(
            Box((0.52, 0.045, 0.16)),
            origin=Origin(
                xyz=(0.0, 0.80 * math.cos(theta), 0.80 * math.sin(theta)),
                rpy=(roll, 0.0, 0.0),
            ),
            material=weathered_oak,
            name=bucket_name,
        )

    flap = model.part("shutoff_flap")
    flap.visual(
        Cylinder(radius=0.025, length=0.42),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.34, 0.030, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=weathered_oak,
        name="flap_board",
    )
    flap.visual(
        Box((0.30, 0.034, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.215)),
        material=dark_oak,
        name="lower_batten",
    )

    model.articulation(
        "axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, -0.080, 2.295)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("timber_frame")
    wheel = object_model.get_part("wheel")
    flap = object_model.get_part("shutoff_flap")
    axle = object_model.get_articulation("axle")
    flap_hinge = object_model.get_articulation("flap_hinge")

    ctx.check(
        "wheel uses a continuous axle joint",
        axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axle joint type was {axle.articulation_type!r}",
    )
    ctx.check(
        "shutoff flap has realistic swing limits",
        flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.lower == 0.0
        and 1.0 <= flap_hinge.motion_limits.upper <= 1.4,
        details=f"flap limits were {flap_hinge.motion_limits!r}",
    )

    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="bearing_0",
        contact_tol=0.002,
        name="left bearing saddle supports axle",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="bearing_1",
        contact_tol=0.002,
        name="right bearing saddle supports axle",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="launder_bottom",
        negative_elem="bucket_4",
        min_gap=0.15,
        name="launder clears the wheel crown",
    )
    ctx.expect_contact(
        flap,
        frame,
        elem_a="hinge_barrel",
        elem_b="launder_side_0",
        contact_tol=0.003,
        name="one launder cheek supports flap hinge",
    )
    ctx.expect_contact(
        flap,
        frame,
        elem_a="hinge_barrel",
        elem_b="launder_side_1",
        contact_tol=0.003,
        name="opposite launder cheek supports flap hinge",
    )

    closed = ctx.part_element_world_aabb(flap, elem="flap_board")
    with ctx.pose({flap_hinge: 1.1}):
        opened = ctx.part_element_world_aabb(flap, elem="flap_board")
    ctx.check(
        "flap swings downstream away from outlet",
        closed is not None
        and opened is not None
        and opened[1][1] > closed[1][1] + 0.08,
        details=f"closed={closed}, opened={opened}",
    )

    return ctx.report()


object_model = build_object_model()
