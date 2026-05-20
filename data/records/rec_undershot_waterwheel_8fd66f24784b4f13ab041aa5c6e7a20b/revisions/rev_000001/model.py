from __future__ import annotations

from math import atan2, cos, pi, sin, sqrt

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


AXLE_HEIGHT = 0.80


def _radial_origin(radius: float, theta: float) -> Origin:
    """Place a visual whose local +Z direction is radial in the wheel YZ plane."""
    return Origin(
        xyz=(0.0, -radius * sin(theta), radius * cos(theta)),
        rpy=(theta, 0.0, 0.0),
    )


def _side_frame_beam(part, x: float, y0: float, z0: float, y1: float, z1: float, material, name: str) -> None:
    """Add one rectangular brace in a side frame at fixed X."""
    length = sqrt((y1 - y0) ** 2 + (z1 - z0) ** 2)
    angle = atan2(-(y1 - y0), z1 - z0)
    part.visual(
        Box((0.075, 0.055, length)),
        origin=Origin(xyz=(x, (y0 + y1) * 0.5, (z0 + z1) * 0.5), rpy=(angle, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    weathered_wood = model.material("weathered_wood", rgba=(0.50, 0.32, 0.18, 1.0))
    end_grain = model.material("dark_end_grain", rgba=(0.34, 0.21, 0.12, 1.0))
    iron = model.material("dark_wrought_iron", rgba=(0.10, 0.11, 0.11, 1.0))
    wet_water = model.material("shallow_blue_water", rgba=(0.18, 0.42, 0.72, 0.58))

    support = model.part("support")

    # Ground-connected timber trestle: two side frames tied by cross sills.
    support.visual(Box((1.20, 0.08, 0.08)), origin=Origin(xyz=(0.0, -0.35, 0.04)), material=weathered_wood, name="front_sill")
    support.visual(Box((1.20, 0.08, 0.08)), origin=Origin(xyz=(0.0, 0.35, 0.04)), material=weathered_wood, name="rear_sill")
    for idx, x in enumerate((-0.50, 0.50)):
        side = "side_0" if idx == 0 else "side_1"
        support.visual(Box((0.085, 0.86, 0.09)), origin=Origin(xyz=(x, 0.0, 0.10)), material=weathered_wood, name=f"{side}_foot_rail")
        support.visual(Box((0.09, 0.09, 0.76)), origin=Origin(xyz=(x, -0.30, 0.43)), material=weathered_wood, name=f"{side}_front_post")
        support.visual(Box((0.09, 0.09, 0.76)), origin=Origin(xyz=(x, 0.30, 0.43)), material=weathered_wood, name=f"{side}_rear_post")
        support.visual(Box((0.09, 0.72, 0.075)), origin=Origin(xyz=(x, 0.0, 0.66)), material=weathered_wood, name=f"{side}_top_rail")
        support.visual(Box((0.12, 0.28, 0.055)), origin=Origin(xyz=(x, 0.0, 0.70)), material=end_grain, name=f"{side}_bearing_seat")
        support.visual(
            Cylinder(radius=0.075, length=0.125),
            origin=Origin(xyz=(x, 0.0, AXLE_HEIGHT), rpy=(0.0, pi / 2.0, 0.0)),
            material=iron,
            name=f"bearing_sleeve_{idx}",
        )
        _side_frame_beam(support, x, -0.34, 0.14, 0.20, 0.66, weathered_wood, f"{side}_diagonal_0")
        _side_frame_beam(support, x, 0.34, 0.14, -0.20, 0.66, weathered_wood, f"{side}_diagonal_1")

    # Low undershot race/trough edge: water leaves this lip toward the lower paddles.
    support.visual(Box((1.04, 0.08, 0.06)), origin=Origin(xyz=(0.0, -0.72, 0.12)), material=weathered_wood, name="trough_cross_tie")
    support.visual(Box((0.08, 0.70, 0.07)), origin=Origin(xyz=(-0.50, -0.70, 0.10)), material=weathered_wood, name="trough_runner_0")
    support.visual(Box((0.08, 0.70, 0.07)), origin=Origin(xyz=(0.50, -0.70, 0.10)), material=weathered_wood, name="trough_runner_1")
    support.visual(Box((0.12, 0.08, 0.22)), origin=Origin(xyz=(-0.47, -1.10, 0.19)), material=weathered_wood, name="trough_post_0")
    support.visual(Box((0.12, 0.08, 0.22)), origin=Origin(xyz=(0.47, -1.10, 0.19)), material=weathered_wood, name="trough_post_1")
    support.visual(Box((0.12, 0.08, 0.22)), origin=Origin(xyz=(-0.47, -0.82, 0.19)), material=weathered_wood, name="trough_post_2")
    support.visual(Box((0.12, 0.08, 0.22)), origin=Origin(xyz=(0.47, -0.82, 0.19)), material=weathered_wood, name="trough_post_3")
    support.visual(Box((0.86, 0.52, 0.04)), origin=Origin(xyz=(0.0, -0.96, 0.24)), material=weathered_wood, name="trough_bottom")
    support.visual(Box((0.05, 0.52, 0.18)), origin=Origin(xyz=(-0.43, -0.96, 0.33)), material=weathered_wood, name="trough_side_0")
    support.visual(Box((0.05, 0.52, 0.18)), origin=Origin(xyz=(0.43, -0.96, 0.33)), material=weathered_wood, name="trough_side_1")
    support.visual(Box((0.86, 0.05, 0.16)), origin=Origin(xyz=(0.0, -1.22, 0.32)), material=weathered_wood, name="trough_end")
    support.visual(Box((0.86, 0.50, 0.018)), origin=Origin(xyz=(0.0, -0.96, 0.34)), material=wet_water, name="water_surface")
    support.visual(Box((0.78, 0.016, 0.09)), origin=Origin(xyz=(0.0, -0.704, 0.296)), material=wet_water, name="falling_water_lip")

    wheel = model.part("wheel")

    # The entire rotating member is authored around the child frame at the axle.
    wheel.visual(
        Cylinder(radius=0.052, length=1.12),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.105, length=0.54),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=weathered_wood,
        name="hub",
    )

    rim_mesh = mesh_from_geometry(TorusGeometry(radius=0.505, tube=0.026, radial_segments=18, tubular_segments=72).rotate_y(pi / 2.0), "waterwheel_rim")
    for idx, x in enumerate((-0.23, 0.23)):
        wheel.visual(rim_mesh, origin=Origin(xyz=(x, 0.0, 0.0)), material=weathered_wood, name=f"rim_{idx}")

    for side_index, x in enumerate((-0.23, 0.23)):
        for spoke_index in range(8):
            theta = 2.0 * pi * spoke_index / 8.0
            origin = _radial_origin(0.29, theta)
            wheel.visual(
                Box((0.055, 0.045, 0.43)),
                origin=Origin(xyz=(x, origin.xyz[1], origin.xyz[2]), rpy=origin.rpy),
                material=weathered_wood,
                name=f"spoke_{side_index}_{spoke_index}",
            )

    for paddle_index in range(12):
        theta = 2.0 * pi * paddle_index / 12.0
        # Broad boards bridge both side rims and protrude past them to catch the
        # shallow undershot flow.
        wheel.visual(
            Box((0.56, 0.105, 0.19)),
            origin=_radial_origin(0.595, theta),
            material=weathered_wood,
            name=f"paddle_{paddle_index}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    for sleeve in ("bearing_sleeve_0", "bearing_sleeve_1"):
        ctx.allow_overlap(
            support,
            wheel,
            elem_a=sleeve,
            elem_b="axle",
            reason="The rotating axle is intentionally captured inside the visible bearing sleeve.",
        )
        ctx.expect_within(
            wheel,
            support,
            axes="yz",
            inner_elem="axle",
            outer_elem=sleeve,
            margin=0.0,
            name=f"axle centered in {sleeve}",
        )
        ctx.expect_overlap(
            support,
            wheel,
            axes="x",
            elem_a=sleeve,
            elem_b="axle",
            min_overlap=0.09,
            name=f"{sleeve} grips axle length",
        )

    ctx.check(
        "wheel has continuous spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({spin: pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
        ctx.expect_origin_gap(wheel, support, axis="z", min_gap=AXLE_HEIGHT - 0.001, max_gap=AXLE_HEIGHT + 0.001, name="wheel origin remains on axle height")

    ctx.check(
        "rotating member stays centered on support axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0]) < 0.001
        and abs(rest_pos[1]) < 0.001
        and abs(rest_pos[2] - AXLE_HEIGHT) < 0.001
        and max(abs(rest_pos[i] - turned_pos[i]) for i in range(3)) < 0.001,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
