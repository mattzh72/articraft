from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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

    weathered_timber = model.material("weathered_timber", rgba=(0.47, 0.34, 0.21, 1.0))
    damp_timber = model.material("damp_timber", rgba=(0.35, 0.25, 0.16, 1.0))
    iron = model.material("iron", rgba=(0.26, 0.28, 0.30, 1.0))
    aged_iron = model.material("aged_iron", rgba=(0.40, 0.42, 0.44, 1.0))

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.44, 0.78)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    for side_y, side_name in ((-0.18, "left"), (0.18, "right")):
        support_frame.visual(
            Box((0.54, 0.06, 0.08)),
            origin=Origin(xyz=(0.0, side_y, 0.04)),
            material=damp_timber,
            name=f"{side_name}_skid",
        )
        support_frame.visual(
            Box((0.06, 0.06, 0.72)),
            origin=Origin(xyz=(-0.18, side_y, 0.40)),
            material=weathered_timber,
            name=f"{side_name}_front_post",
        )
        support_frame.visual(
            Box((0.06, 0.06, 0.72)),
            origin=Origin(xyz=(0.18, side_y, 0.40)),
            material=weathered_timber,
            name=f"{side_name}_rear_post",
        )
        support_frame.visual(
            Box((0.42, 0.06, 0.08)),
            origin=Origin(xyz=(0.0, side_y, 0.50)),
            material=weathered_timber,
            name=f"{side_name}_axle_beam",
        )
        support_frame.visual(
            Box((0.42, 0.06, 0.06)),
            origin=Origin(xyz=(0.0, side_y, 0.73)),
            material=weathered_timber,
            name=f"{side_name}_top_rail",
        )
        support_frame.visual(
            Box((0.12, 0.04, 0.16)),
            origin=Origin(xyz=(0.0, side_y - (0.01 if side_y > 0.0 else -0.01), 0.43)),
            material=aged_iron,
            name=f"{side_name}_bearing_block",
        )
        support_frame.visual(
            Box((0.46, 0.05, 0.05)),
            origin=Origin(
                xyz=(-0.08, side_y, 0.29),
                rpy=(0.0, -0.86 if side_y < 0.0 else 0.86, 0.0),
            ),
            material=damp_timber,
            name=f"{side_name}_diagonal_brace",
        )

    support_frame.visual(
        Box((0.08, 0.30, 0.08)),
        origin=Origin(xyz=(-0.18, 0.0, 0.04)),
        material=damp_timber,
        name="front_tie",
    )
    support_frame.visual(
        Box((0.08, 0.30, 0.08)),
        origin=Origin(xyz=(0.18, 0.0, 0.04)),
        material=damp_timber,
        name="rear_tie",
    )
    support_frame.visual(
        Box((0.10, 0.36, 0.24)),
        origin=Origin(xyz=(-0.19, 0.0, 0.81)),
        material=damp_timber,
        name="trough_back",
    )
    support_frame.visual(
        Box((0.22, 0.36, 0.04)),
        origin=Origin(xyz=(-0.09, 0.0, 0.79)),
        material=weathered_timber,
        name="trough_floor",
    )
    support_frame.visual(
        Box((0.20, 0.36, 0.04)),
        origin=Origin(xyz=(-0.07, 0.0, 0.735)),
        material=weathered_timber,
        name="trough_lip",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.30, length=0.30),
        mass=55.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.27, tube=0.025, radial_segments=18, tubular_segments=72).rotate_x(pi / 2.0),
        "waterwheel_rim",
    )
    wheel.visual(
        Cylinder(radius=0.03, length=0.30),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.085, length=0.24),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=aged_iron,
        name="hub_barrel",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -0.11, 0.0)),
        material=iron,
        name="left_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, 0.11, 0.0)),
        material=iron,
        name="right_rim",
    )

    for spoke_index in range(6):
        angle = spoke_index * (pi / 3.0)
        for side_name, side_y in (("left", -0.11), ("right", 0.11)):
            wheel.visual(
                Box((0.24, 0.03, 0.03)),
                origin=Origin(xyz=(0.17, side_y, 0.0), rpy=(0.0, angle, 0.0)),
                material=iron,
                name=f"{side_name}_spoke_{spoke_index:02d}",
            )

    for paddle_index in range(10):
        angle = paddle_index * (2.0 * pi / 10.0)
        wheel.visual(
            Box((0.11, 0.26, 0.04)),
            origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, angle, 0.0)),
            material=weathered_timber,
            name=f"paddle_{paddle_index:02d}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="trough_lip",
        min_gap=0.02,
        max_gap=0.10,
        name="wheel stays just below the trough edge",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle_shaft",
        elem_b="left_bearing_block",
        name="left bearing block supports the axle",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle_shaft",
        elem_b="right_bearing_block",
        name="right bearing block supports the axle",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    rest_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem="paddle_00"))
    with ctx.pose({spin: pi / 2.0}):
        turned_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem="paddle_00"))

    moved = (
        rest_center is not None
        and turned_center is not None
        and abs(rest_center[0] - turned_center[0]) > 0.12
        and abs(rest_center[2] - turned_center[2]) > 0.12
    )
    ctx.check(
        "continuous joint visibly rotates a paddle around the axle",
        moved,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
