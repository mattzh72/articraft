from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _annular_disk(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    y_offset: float = 0.0,
    segments: int = 72,
) -> MeshGeometry:
    """Flat annular disk in the XZ plane, with its bore along the Y axle."""
    outer = superellipse_profile(2.0 * outer_radius, 2.0 * outer_radius, exponent=2.0, segments=segments)
    inner = superellipse_profile(2.0 * inner_radius, 2.0 * inner_radius, exponent=2.0, segments=segments)
    return (
        ExtrudeWithHolesGeometry(outer, [inner], thickness, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y_offset, 0.0)
    )


def _radial_box(
    *,
    size: tuple[float, float, float],
    radius: float,
    angle: float,
) -> MeshGeometry:
    """Box whose local X length runs radially in the wheel's XZ plane."""
    return BoxGeometry(size).translate(radius, 0.0, 0.0).rotate_y(-angle)


def _build_wooden_wheel() -> MeshGeometry:
    wheel = MeshGeometry()

    # Two side rims, like the paired wooden rings on a traditional paddle wheel.
    wheel.merge(_annular_disk(outer_radius=0.305, inner_radius=0.258, thickness=0.036, y_offset=-0.165))
    wheel.merge(_annular_disk(outer_radius=0.305, inner_radius=0.258, thickness=0.036, y_offset=0.165))

    # Radial spokes overlap both the hub and the paired rims.
    for spoke_index in range(8):
        angle = spoke_index * math.tau / 8.0
        wheel.merge(
            _radial_box(
                size=(0.250, 0.310, 0.038),
                radius=0.166,
                angle=angle,
            )
        )

    # Broad cross-flow paddles extend through the wheel width and protrude beyond
    # the rims so the object reads as an undershot waterwheel rather than a tire.
    for paddle_index in range(12):
        angle = paddle_index * math.tau / 12.0
        wheel.merge(
            _radial_box(
                size=(0.094, 0.420, 0.034),
                radius=0.322,
                angle=angle,
            )
        )

    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    weathered_wood = model.material("weathered_wood", rgba=(0.50, 0.31, 0.15, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.08, 0.08, 0.075, 1.0))
    stone = model.material("stone", rgba=(0.42, 0.41, 0.38, 1.0))
    trough_wood = model.material("trough_wood", rgba=(0.40, 0.24, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.00, 1.02, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=stone,
        name="base_slab",
    )

    # Two low side frames stand outside the wheel cheeks and carry open bearings.
    for side_index, y in enumerate((-0.350, 0.350)):
        for x in (-0.290, 0.290):
            frame.visual(
                Box((0.060, 0.060, 0.390)),
                origin=Origin(xyz=(x, y, 0.235)),
                material=weathered_wood,
                name=f"side_post_{side_index}_{0 if x < 0 else 1}",
            )
        for x in (-0.185, 0.185):
            frame.visual(
                Box((0.230, 0.070, 0.075)),
                origin=Origin(xyz=(x, y, 0.430)),
                material=weathered_wood,
                name=f"split_cap_{side_index}_{0 if x < 0 else 1}",
            )
        # Split bearing blocks leave a real open hole for the rotating axle.
        frame.visual(
            Box((0.115, 0.050, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.495)),
            material=dark_iron,
            name=f"bearing_top_{side_index}",
        )
        frame.visual(
            Box((0.115, 0.050, 0.025)),
            # Top of the saddle is tangent to the axle underside
            # (0.430 - axle radius 0.026) so the wheel is supported, not floating.
            origin=Origin(xyz=(0.0, y, 0.3915)),
            material=dark_iron,
            name=f"bearing_bottom_{side_index}",
        )
        for x in (-0.070, 0.070):
            frame.visual(
                Box((0.025, 0.050, 0.115)),
                origin=Origin(xyz=(x, y, 0.430)),
                material=dark_iron,
                name=f"bearing_cheek_{side_index}_{0 if x < 0 else 1}",
            )

    # A shallow upstream channel/lip points water at the lower paddles without
    # crossing the swept wheel envelope.
    frame.visual(
        Box((0.305, 0.730, 0.035)),
        origin=Origin(xyz=(-0.585, 0.0, 0.190)),
        material=trough_wood,
        name="trough_floor",
    )
    frame.visual(
        Box((0.040, 0.730, 0.170)),
        origin=Origin(xyz=(-0.755, 0.0, 0.255)),
        material=trough_wood,
        name="outer_trough_wall",
    )
    frame.visual(
        Box((0.036, 0.730, 0.120)),
        origin=Origin(xyz=(-0.415, 0.0, 0.250)),
        material=trough_wood,
        name="inner_trough_lip",
    )
    for post_index, (x, y) in enumerate(((-0.735, -0.315), (-0.735, 0.315), (-0.435, -0.315), (-0.435, 0.315))):
        frame.visual(
            Box((0.050, 0.050, 0.145)),
            origin=Origin(xyz=(x, y, 0.112)),
            material=weathered_wood,
            name=f"trough_stand_{post_index}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(_build_wooden_wheel(), "wooden_wheel"),
        material=weathered_wood,
        name="wooden_wheel",
    )
    wheel.visual(
        Cylinder(radius=0.026, length=0.880),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.070, length=0.240),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="hub",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.check(
        "wheel spin is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.check(
        "axle axis is horizontal",
        tuple(round(v, 6) for v in spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        min_gap=0.015,
        max_gap=0.045,
        positive_elem="wooden_wheel",
        negative_elem="base_slab",
        name="paddles run just above the mounting slab",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="xy",
        inner_elem="wooden_wheel",
        outer_elem="base_slab",
        margin=0.0,
        name="wheel stays inside the low wide base footprint",
    )

    rest_position = ctx.part_world_position(wheel)
    with ctx.pose({spin: math.pi / 7.0}):
        turned_position = ctx.part_world_position(wheel)
        ctx.expect_gap(
            wheel,
            frame,
            axis="z",
            min_gap=0.010,
            positive_elem="wooden_wheel",
            negative_elem="base_slab",
            name="rotated paddles clear the base",
        )
    ctx.check(
        "rotation keeps axle center fixed",
        rest_position is not None and turned_position is not None and rest_position == turned_position,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
