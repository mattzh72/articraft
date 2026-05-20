from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    stone = model.material("stone", rgba=(0.63, 0.62, 0.58, 1.0))
    timber = model.material("timber", rgba=(0.49, 0.32, 0.18, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.36, 0.22, 0.12, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    iron = model.material("iron", rgba=(0.44, 0.46, 0.48, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _brace_origin(
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> tuple[float, Origin]:
        dx = end[0] - start[0]
        dz = end[2] - start[2]
        length = hypot(dx, dz)
        return (
            length,
            Origin(
                xyz=(
                    0.5 * (start[0] + end[0]),
                    start[1],
                    0.5 * (start[2] + end[2]),
                ),
                rpy=(0.0, atan2(dz, dx), 0.0),
            ),
        )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.80, 0.86, 1.28)),
        mass=820.0,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
    )
    frame.visual(
        Box((1.80, 0.86, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stone,
        name="base_plinth",
    )
    frame.visual(
        Box((0.26, 0.40, 0.78)),
        origin=Origin(xyz=(-0.22, 0.0, 0.45)),
        material=stone,
        name="channel_pier",
    )
    frame.visual(
        Box((0.38, 0.72, 0.06)),
        origin=Origin(xyz=(-0.12, 0.0, 0.87)),
        material=timber,
        name="trough_floor",
    )
    frame.visual(
        Box((0.10, 0.72, 0.34)),
        origin=Origin(xyz=(-0.26, 0.0, 1.04)),
        material=timber,
        name="trough_back_wall",
    )
    frame.visual(
        Box((0.08, 0.72, 0.06)),
        origin=Origin(xyz=(0.03, 0.0, 1.11)),
        material=timber,
        name="trough_edge",
    )
    frame.visual(
        Box((0.34, 0.04, 0.28)),
        origin=Origin(xyz=(-0.12, 0.38, 1.00)),
        material=timber,
        name="left_trough_cheek",
    )
    frame.visual(
        Box((0.34, 0.04, 0.28)),
        origin=Origin(xyz=(-0.12, -0.38, 1.00)),
        material=timber,
        name="right_trough_cheek",
    )
    frame.visual(
        Box((0.12, 0.10, 1.00)),
        origin=Origin(xyz=(0.56, 0.34, 0.56)),
        material=wet_timber,
        name="left_support_post",
    )
    frame.visual(
        Box((0.12, 0.10, 1.00)),
        origin=Origin(xyz=(0.56, -0.34, 0.56)),
        material=wet_timber,
        name="right_support_post",
    )
    frame.visual(
        Box((0.14, 0.12, 0.18)),
        origin=Origin(xyz=(0.56, 0.34, 0.62)),
        material=dark_iron,
        name="left_bearing",
    )
    frame.visual(
        Box((0.14, 0.12, 0.18)),
        origin=Origin(xyz=(0.56, -0.34, 0.62)),
        material=dark_iron,
        name="right_bearing",
    )

    left_brace_length, left_brace_origin = _brace_origin(
        (0.10, 0.34, 0.16),
        (0.52, 0.34, 0.72),
    )
    frame.visual(
        Box((left_brace_length, 0.08, 0.08)),
        origin=left_brace_origin,
        material=wet_timber,
        name="left_brace",
    )
    right_brace_length, right_brace_origin = _brace_origin(
        (0.10, -0.34, 0.16),
        (0.52, -0.34, 0.72),
    )
    frame.visual(
        Box((right_brace_length, 0.08, 0.08)),
        origin=right_brace_origin,
        material=wet_timber,
        name="right_brace",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.50, length=0.56),
        mass=140.0,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    rim_mesh = _mesh(
        "waterwheel_rim",
        TorusGeometry(radius=0.44, tube=0.028, radial_segments=18, tubular_segments=56),
    )
    wheel.visual(
        Cylinder(radius=0.045, length=0.56),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.12, length=0.38),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="hub_barrel",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, 0.13, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wet_timber,
        name="left_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -0.13, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wet_timber,
        name="right_rim",
    )

    for spoke_index in range(8):
        angle = 2.0 * pi * spoke_index / 8.0
        wheel.visual(
            Box((0.34, 0.26, 0.045)),
            origin=Origin(
                xyz=(0.27 * cos(angle), 0.0, 0.27 * sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=dark_iron,
            name=f"spoke_{spoke_index:02d}",
        )

    for paddle_index in range(12):
        angle = 2.0 * pi * paddle_index / 12.0
        wheel.visual(
            Box((0.065, 0.30, 0.16)),
            origin=Origin(
                xyz=(0.38 * cos(angle), 0.0, 0.38 * sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=timber,
            name=f"paddle_{paddle_index:02d}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.56, 0.0, 0.60)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=240.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.expect_origin_gap(
        wheel,
        frame,
        axis="x",
        min_gap=0.40,
        max_gap=0.70,
        name="wheel is offset forward from the base center",
    )
    ctx.expect_origin_distance(
        wheel,
        frame,
        axes="y",
        max_dist=0.001,
        name="wheel stays centered between the side frames",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="trough_edge",
        negative_elem="left_rim",
        min_gap=0.01,
        max_gap=0.06,
        name="trough edge sits just above the wheel rim",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="left_bearing",
        contact_tol=1e-6,
        name="left support block carries the axle end",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="right_bearing",
        contact_tol=1e-6,
        name="right support block carries the axle end",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({spin: pi / 2.0}):
        rotated_pos = ctx.part_world_position(wheel)
    ctx.check(
        "continuous wheel joint keeps the axle fixed in place",
        rest_pos is not None and rotated_pos is not None and all(
            abs(a - b) <= 1e-6 for a, b in zip(rest_pos, rotated_pos)
        ),
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
