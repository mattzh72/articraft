from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    weathered_oak = model.material("weathered_oak", rgba=(0.52, 0.40, 0.27, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.35, 0.25, 0.17, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.28, 0.19, 0.12, 1.0))
    iron = model.material("iron", rgba=(0.31, 0.33, 0.35, 1.0))
    stone = model.material("stone", rgba=(0.56, 0.56, 0.54, 1.0))

    support = model.part("support")
    support.inertial = Inertial.from_geometry(
        Box((1.80, 1.15, 1.36)),
        mass=180.0,
        origin=Origin(xyz=(0.0, -0.08, 0.68)),
    )

    support.visual(
        Box((1.80, 0.34, 0.16)),
        origin=Origin(xyz=(0.0, 0.08, 0.08)),
        material=stone,
        name="front_foundation",
    )
    support.visual(
        Box((1.80, 0.24, 0.14)),
        origin=Origin(xyz=(0.0, -0.68, 0.07)),
        material=stone,
        name="rear_foundation",
    )
    support.visual(
        Box((1.58, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.24, 0.14)),
        material=dark_oak,
        name="front_sill",
    )
    support.visual(
        Box((1.58, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, -0.66, 0.14)),
        material=dark_oak,
        name="rear_sill",
    )

    for side_x, side_name in ((0.64, "left"), (-0.64, "right")):
        support.visual(
            Box((0.12, 0.16, 1.10)),
            origin=Origin(xyz=(side_x, 0.16, 0.55)),
            material=weathered_oak,
            name=f"{side_name}_front_post",
        )
        support.visual(
            Box((0.12, 0.16, 1.04)),
            origin=Origin(xyz=(side_x, -0.62, 0.52)),
            material=weathered_oak,
            name=f"{side_name}_rear_post",
        )
        support.visual(
            Box((0.09, 0.96, 0.09)),
            origin=Origin(xyz=(side_x, -0.12, 0.46), rpy=(0.92, 0.0, 0.0)),
            material=dark_oak,
            name=f"{side_name}_diagonal_brace",
        )
    support.visual(
        Box((0.40, 0.18, 0.14)),
        origin=Origin(xyz=(0.42, 0.10, 0.58)),
        material=wet_timber,
        name="left_bearing_beam",
    )
    support.visual(
        Box((0.40, 0.18, 0.14)),
        origin=Origin(xyz=(-0.42, 0.10, 0.58)),
        material=wet_timber,
        name="right_bearing_beam",
    )

    support.visual(
        Box((1.42, 0.90, 0.08)),
        origin=Origin(xyz=(0.0, -0.30, 1.08)),
        material=weathered_oak,
        name="trough_floor",
    )
    support.visual(
        Box((1.42, 0.12, 0.28)),
        origin=Origin(xyz=(0.0, 0.18, 1.16)),
        material=weathered_oak,
        name="trough_lip",
    )
    support.visual(
        Box((1.42, 0.12, 0.30)),
        origin=Origin(xyz=(0.0, -0.76, 1.15)),
        material=weathered_oak,
        name="rear_bulkhead",
    )
    support.visual(
        Box((0.14, 0.90, 0.34)),
        origin=Origin(xyz=(0.64, -0.30, 1.17)),
        material=weathered_oak,
        name="left_trough_wall",
    )
    support.visual(
        Box((0.14, 0.90, 0.34)),
        origin=Origin(xyz=(-0.64, -0.30, 1.17)),
        material=weathered_oak,
        name="right_trough_wall",
    )
    support.visual(
        Box((1.54, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, -0.34, 1.29)),
        material=dark_oak,
        name="trough_cap_tie",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.38, length=0.36),
        mass=28.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    wheel.visual(
        Cylinder(radius=0.045, length=0.38),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.09, length=0.26),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wet_timber,
        name="hub",
    )

    rim_radius = 0.33
    rim_side_x = 0.15
    for side_x, side_name in ((rim_side_x, "left"), (-rim_side_x, "right")):
        for segment_index in range(12):
            angle = segment_index * (2.0 * pi / 12.0)
            wheel.visual(
                Box((0.06, 0.19, 0.09)),
                origin=Origin(
                    xyz=(side_x, -sin(angle) * rim_radius, cos(angle) * rim_radius),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=dark_oak,
                name=f"{side_name}_rim_segment_{segment_index}",
            )
        for spoke_index in range(8):
            angle = spoke_index * (2.0 * pi / 8.0)
            wheel.visual(
                Box((0.06, 0.05, 0.36)),
                origin=Origin(
                    xyz=(side_x, -sin(angle) * 0.20, cos(angle) * 0.20),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=weathered_oak,
                name=f"{side_name}_spoke_{spoke_index}",
            )

    wheel.visual(
        Box((0.36, 0.045, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=wet_timber,
        name="paddle_0",
    )
    for paddle_index in range(1, 10):
        angle = paddle_index * (2.0 * pi / 10.0)
        wheel.visual(
            Box((0.36, 0.045, 0.22)),
            origin=Origin(
                xyz=(0.0, -sin(angle) * 0.31, cos(angle) * 0.31),
                rpy=(angle, 0.0, 0.0),
            ),
            material=wet_timber,
            name=f"paddle_{paddle_index}",
        )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.10, 0.58)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0),
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

    support = object_model.get_part("support")
    wheel = object_model.get_part("wheel")
    spin_joint = object_model.get_articulation("support_to_wheel")

    ctx.expect_gap(
        support,
        wheel,
        axis="z",
        positive_elem="trough_lip",
        min_gap=0.02,
        max_gap=0.12,
        name="wheel hangs just below the trough lip",
    )
    ctx.expect_overlap(
        wheel,
        support,
        axes="x",
        elem_b="trough_lip",
        min_overlap=0.30,
        name="wheel stays centered under the trough edge",
    )
    ctx.expect_gap(
        support,
        wheel,
        axis="x",
        positive_elem="left_bearing_beam",
        negative_elem="axle",
        min_gap=0.02,
        max_gap=0.07,
        name="axle clears the left support beam",
    )
    ctx.expect_gap(
        wheel,
        support,
        axis="x",
        positive_elem="axle",
        negative_elem="right_bearing_beam",
        min_gap=0.02,
        max_gap=0.07,
        name="axle clears the right support beam",
    )

    support_aabb = ctx.part_world_aabb(support)
    wheel_aabb = ctx.part_world_aabb(wheel)
    support_width = None
    support_depth = None
    wheel_width = None
    wheel_depth = None
    if support_aabb is not None:
        support_width = support_aabb[1][0] - support_aabb[0][0]
        support_depth = support_aabb[1][1] - support_aabb[0][1]
    if wheel_aabb is not None:
        wheel_width = wheel_aabb[1][0] - wheel_aabb[0][0]
        wheel_depth = wheel_aabb[1][1] - wheel_aabb[0][1]
    ctx.check(
        "fixed support is broader than the rotating wheel",
        support_width is not None
        and support_depth is not None
        and wheel_width is not None
        and wheel_depth is not None
        and support_width > wheel_width + 1.0
        and support_depth > wheel_depth + 0.25,
        details=(
            f"support_width={support_width}, wheel_width={wheel_width}, "
            f"support_depth={support_depth}, wheel_depth={wheel_depth}"
        ),
    )

    rest_origin = ctx.part_world_position(wheel)
    rest_paddle = ctx.part_element_world_aabb(wheel, elem="paddle_0")
    with ctx.pose({spin_joint: pi / 2.0}):
        turned_origin = ctx.part_world_position(wheel)
        turned_paddle = ctx.part_element_world_aabb(wheel, elem="paddle_0")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    rest_paddle_center = aabb_center(rest_paddle)
    turned_paddle_center = aabb_center(turned_paddle)
    ctx.check(
        "wheel rotates about a fixed axle",
        rest_origin is not None
        and turned_origin is not None
        and max(abs(turned_origin[index] - rest_origin[index]) for index in range(3)) < 1e-6
        and rest_paddle_center is not None
        and turned_paddle_center is not None
        and abs(turned_paddle_center[1] - rest_paddle_center[1]) > 0.20
        and abs(turned_paddle_center[2] - rest_paddle_center[2]) > 0.20,
        details=(
            f"rest_origin={rest_origin}, turned_origin={turned_origin}, "
            f"rest_paddle_center={rest_paddle_center}, turned_paddle_center={turned_paddle_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
