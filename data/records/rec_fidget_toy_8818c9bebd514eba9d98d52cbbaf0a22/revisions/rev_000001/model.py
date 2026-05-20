from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _cylinder_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt(dx * dx + dy * dy), dz)
    return (
        Origin(
            xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_strut(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    origin, length = _cylinder_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="newtons_cradle")

    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.84, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.70, 0.72, 0.76, 1.0))
    piano_black = model.material("piano_black", rgba=(0.09, 0.09, 0.10, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.16, 0.16, 0.17, 1.0))

    base_lower_size = (0.220, 0.120, 0.012)
    base_top_size = (0.200, 0.100, 0.008)
    base_top_z = 0.016
    support_x = 0.095
    support_y = 0.044
    support_base_z = 0.016
    rail_z = 0.158
    rail_y = 0.034
    support_radius = 0.0045
    rail_radius = 0.0032
    pendulum_drop = 0.123
    ball_radius = 0.012
    pivot_sleeve_radius = 0.0024
    pivot_sleeve_length = 0.008
    pivot_sleeve_drop = 0.0056
    rod_radius = 0.0016
    ball_x_positions = (-0.048, -0.024, 0.0, 0.024, 0.048)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.220, 0.120, 0.180)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )
    frame.visual(
        Box(base_lower_size),
        origin=Origin(xyz=(0.0, 0.0, base_lower_size[2] * 0.5)),
        material=piano_black,
        name="base_lower",
    )
    frame.visual(
        Box(base_top_size),
        origin=Origin(xyz=(0.0, 0.0, base_top_z)),
        material=piano_black,
        name="base_top",
    )
    frame.visual(
        Box((0.168, 0.072, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, base_top_z + 0.005)),
        material=dark_rubber,
        name="impact_pad",
    )

    left_support = wire_from_points(
        [
            (-support_x, rail_y + 0.010, support_base_z),
            (-support_x, rail_y, rail_z),
            (-support_x, -rail_y, rail_z),
            (-support_x, -rail_y - 0.010, support_base_z),
        ],
        radius=support_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.020,
        corner_segments=12,
    )
    right_support = wire_from_points(
        [
            (support_x, rail_y + 0.010, support_base_z),
            (support_x, rail_y, rail_z),
            (support_x, -rail_y, rail_z),
            (support_x, -rail_y - 0.010, support_base_z),
        ],
        radius=support_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.020,
        corner_segments=12,
    )
    frame.visual(mesh_from_geometry(left_support, "left_support"), material=chrome, name="support_0")
    frame.visual(mesh_from_geometry(right_support, "right_support"), material=chrome, name="support_1")
    frame.visual(
        Cylinder(radius=rail_radius, length=0.194),
        origin=Origin(xyz=(0.0, rail_y, rail_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="front_rail",
    )
    frame.visual(
        Cylinder(radius=rail_radius, length=0.194),
        origin=Origin(xyz=(0.0, -rail_y, rail_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="rear_rail",
    )

    for index, ball_x in enumerate(ball_x_positions):
        pendulum = model.part(f"pendulum_{index}")
        pendulum.inertial = Inertial.from_geometry(
            Box((0.028, 0.076, 0.132)),
            mass=0.11,
            origin=Origin(xyz=(0.0, 0.0, -0.066)),
        )

        front_sleeve_center = (0.0, rail_y, -pivot_sleeve_drop)
        rear_sleeve_center = (0.0, -rail_y, -pivot_sleeve_drop)
        ball_center = (0.0, 0.0, -pendulum_drop)
        front_anchor = (0.0, 0.0085, -0.1140)
        rear_anchor = (0.0, -0.0085, -0.1140)

        pendulum.visual(
            Cylinder(radius=pivot_sleeve_radius, length=pivot_sleeve_length),
            origin=Origin(xyz=front_sleeve_center, rpy=(0.0, pi * 0.5, 0.0)),
            material=chrome,
            name="front_pivot",
        )
        pendulum.visual(
            Cylinder(radius=pivot_sleeve_radius, length=pivot_sleeve_length),
            origin=Origin(xyz=rear_sleeve_center, rpy=(0.0, pi * 0.5, 0.0)),
            material=chrome,
            name="rear_pivot",
        )
        _add_strut(
            pendulum,
            front_sleeve_center,
            front_anchor,
            radius=rod_radius,
            material=chrome,
            name="front_rod",
        )
        _add_strut(
            pendulum,
            rear_sleeve_center,
            rear_anchor,
            radius=rod_radius,
            material=chrome,
            name="rear_rod",
        )
        pendulum.visual(
            Sphere(radius=ball_radius),
            origin=Origin(xyz=ball_center),
            material=polished_steel,
            name="ball",
        )
        pendulum.visual(
            Cylinder(radius=0.0020, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.112), rpy=(pi * 0.5, 0.0, 0.0)),
            material=chrome,
            name="ball_clamp",
        )

        model.articulation(
            f"pendulum_{index}_swing",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=pendulum,
            origin=Origin(xyz=(ball_x, 0.0, rail_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=2.5, lower=-0.50, upper=0.50),
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

    # For bounded REVOLUTE/PRISMATIC joints, add exact lower/upper motion-limit
    # checks for prompt-critical contacts, clearances, and motion direction. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.expect_contact(lid, body, elem_a="hinge_leaf", elem_b="body_leaf")

    frame = object_model.get_part("frame")
    center_pendulum = object_model.get_part("pendulum_2")

    for index in range(4):
        left = object_model.get_part(f"pendulum_{index}")
        right = object_model.get_part(f"pendulum_{index + 1}")
        ctx.expect_gap(
            right,
            left,
            axis="x",
            positive_elem="ball",
            negative_elem="ball",
            max_gap=0.001,
            max_penetration=0.0002,
            name=f"ball_pair_{index}_contact_spacing",
        )

    ctx.expect_gap(
        center_pendulum,
        frame,
        axis="z",
        positive_elem="ball",
        negative_elem="base_top",
        min_gap=0.0015,
        max_gap=0.0060,
        name="center_ball_hovers_just_above_base",
    )

    def _ball_center(part_name: str) -> tuple[float, float, float] | None:
        part = object_model.get_part(part_name)
        aabb = ctx.part_element_world_aabb(part, elem="ball")
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) * 0.5,
            (lo[1] + hi[1]) * 0.5,
            (lo[2] + hi[2]) * 0.5,
        )

    left_joint = object_model.get_articulation("pendulum_0_swing")
    right_joint = object_model.get_articulation("pendulum_4_swing")
    left_rest = _ball_center("pendulum_0")
    right_rest = _ball_center("pendulum_4")

    with ctx.pose({left_joint: -0.50, right_joint: 0.50}):
        left_swung = _ball_center("pendulum_0")
        right_swung = _ball_center("pendulum_4")

    ctx.check(
        "outer_balls_swing_outward",
        left_rest is not None
        and right_rest is not None
        and left_swung is not None
        and right_swung is not None
        and left_swung[0] < left_rest[0] - 0.045
        and right_swung[0] > right_rest[0] + 0.045,
        details=f"left_rest={left_rest}, left_swung={left_swung}, right_rest={right_rest}, right_swung={right_swung}",
    )
    ctx.check(
        "outer_balls_rise_when_swung",
        left_rest is not None
        and right_rest is not None
        and left_swung is not None
        and right_swung is not None
        and left_swung[2] > left_rest[2] + 0.010
        and right_swung[2] > right_rest[2] + 0.010,
        details=f"left_rest={left_rest}, left_swung={left_swung}, right_rest={right_rest}, right_swung={right_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
