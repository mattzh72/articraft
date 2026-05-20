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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


WHEEL_CENTER_Z = 2.55
WHEEL_RADIUS = 2.0
PIVOT_RADIUS = 1.66
CABIN_COUNT = 6


def _segment_origin_and_length(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("segment length must be positive")

    mid = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )

    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        rpy = (0.0, 0.0, 0.0 if dz >= 0.0 else math.pi)
    elif abs(dx) < 1e-9 and abs(dz) < 1e-9:
        rpy = ((-math.pi / 2.0) if dy >= 0.0 else (math.pi / 2.0), 0.0, 0.0)
    elif abs(dy) < 1e-9:
        rpy = (0.0, math.atan2(dx, dz), 0.0)
    elif abs(dx) < 1e-9:
        rpy = (-math.atan2(dy, dz), 0.0, 0.0)
    else:
        raise ValueError("unsupported segment orientation for this helper")

    return Origin(xyz=mid, rpy=rpy), length


def _add_segment_box(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    size_x: float,
    size_y: float,
    material,
    name: str | None = None,
) -> None:
    origin, length = _segment_origin_and_length(start, end)
    part.visual(
        Box((size_x, size_y, length)),
        origin=origin,
        material=material,
        name=name,
    )


def _add_segment_cylinder(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    origin, length = _segment_origin_and_length(start, end)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _wheel_point(radius: float, angle: float, y: float = 0.0) -> tuple[float, float, float]:
    return (radius * math.sin(angle), y, radius * math.cos(angle))


def _cabin_angle(index: int) -> float:
    return math.pi + index * (2.0 * math.pi / CABIN_COUNT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="urban_observation_wheel")

    steel_dark = model.material("steel_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.60, 0.62, 0.66, 1.0))
    painted_white = model.material("painted_white", rgba=(0.93, 0.94, 0.96, 1.0))
    accent_red = model.material("accent_red", rgba=(0.77, 0.28, 0.24, 1.0))
    cabin_body = model.material("cabin_body", rgba=(0.90, 0.90, 0.88, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.62, 0.78, 0.90, 0.55))

    base = model.part("base")
    base.visual(
        Box((1.68, 0.84, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=steel_dark,
        name="deck",
    )
    base.visual(
        Box((0.58, 0.32, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=painted_white,
        name="loading_plinth",
    )

    support_y = 0.34
    for side in (-1.0, 1.0):
        y = side * support_y
        _add_segment_box(
            base,
            (-0.45, y, 0.12),
            (0.0, y, WHEEL_CENTER_Z - 0.06),
            size_x=0.12,
            size_y=0.08,
            material=steel_mid,
            name=f"rear_leg_{'left' if side > 0.0 else 'right'}",
        )
        _add_segment_box(
            base,
            (0.45, y, 0.12),
            (0.0, y, WHEEL_CENTER_Z - 0.06),
            size_x=0.12,
            size_y=0.08,
            material=steel_mid,
            name=f"front_leg_{'left' if side > 0.0 else 'right'}",
        )
        base.visual(
            Box((0.74, 0.08, 0.08)),
            origin=Origin(xyz=(0.0, y, 0.82)),
            material=steel_mid,
            name=f"side_tie_{'left' if side > 0.0 else 'right'}",
        )
        base.visual(
            Box((0.22, 0.08, 0.20)),
            origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z - 0.02)),
            material=steel_dark,
            name=f"bearing_block_{'left' if side > 0.0 else 'right'}",
        )

    wheel = model.part("wheel")
    ring_offset = 0.16
    rim_tube = 0.07

    left_ring = TorusGeometry(WHEEL_RADIUS, rim_tube, radial_segments=22, tubular_segments=64)
    left_ring.rotate_x(math.pi / 2.0).translate(0.0, ring_offset, 0.0)
    wheel.visual(
        mesh_from_geometry(left_ring, "wheel_rim_left"),
        material=painted_white,
        name="rim_left",
    )

    right_ring = TorusGeometry(WHEEL_RADIUS, rim_tube, radial_segments=22, tubular_segments=64)
    right_ring.rotate_x(math.pi / 2.0).translate(0.0, -ring_offset, 0.0)
    wheel.visual(
        mesh_from_geometry(right_ring, "wheel_rim_right"),
        material=painted_white,
        name="rim_right",
    )

    wheel.visual(
        Cylinder(radius=0.07, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.18, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="hub_drum",
    )
    for side in (-1.0, 1.0):
        wheel.visual(
            Cylinder(radius=0.28, length=0.04),
            origin=Origin(
                xyz=(0.0, side * 0.12, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=painted_white,
            name=f"hub_plate_{'left' if side > 0.0 else 'right'}",
        )

    for spoke_index in range(CABIN_COUNT):
        spoke_angle = (spoke_index + 0.5) * (2.0 * math.pi / CABIN_COUNT)
        for side in (-1.0, 1.0):
            y = side * 0.12
            _add_segment_cylinder(
                wheel,
                (0.0, y, 0.0),
                _wheel_point(WHEEL_RADIUS - 0.04, spoke_angle, y),
                radius=0.022,
                material=steel_mid,
                name=None,
            )

    for cabin_index in range(CABIN_COUNT):
        angle = _cabin_angle(cabin_index)
        _add_segment_cylinder(
            wheel,
            _wheel_point(WHEEL_RADIUS - 0.01, angle, -ring_offset),
            _wheel_point(WHEEL_RADIUS - 0.01, angle, ring_offset),
            radius=0.024,
            material=steel_dark,
            name=f"rim_tie_{cabin_index}",
        )
        if cabin_index == 0:
            _add_segment_box(
                wheel,
                _wheel_point(PIVOT_RADIUS, angle, -0.12),
                _wheel_point(WHEEL_RADIUS - 0.01, angle, -0.12),
                size_x=0.05,
                size_y=0.04,
                material=accent_red,
                name="fork_0_right",
            )
            _add_segment_box(
                wheel,
                _wheel_point(PIVOT_RADIUS, angle, 0.12),
                _wheel_point(WHEEL_RADIUS - 0.01, angle, 0.12),
                size_x=0.05,
                size_y=0.04,
                material=accent_red,
                name="fork_0_left",
            )
        else:
            for side in (-1.0, 1.0):
                y = side * 0.12
                _add_segment_box(
                    wheel,
                    _wheel_point(PIVOT_RADIUS, angle, y),
                    _wheel_point(WHEEL_RADIUS - 0.01, angle, y),
                    size_x=0.05,
                    size_y=0.04,
                    material=accent_red,
                    name=f"fork_{cabin_index}_{'left' if side > 0.0 else 'right'}",
                )

    model.articulation(
        "base_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2),
    )

    for cabin_index in range(CABIN_COUNT):
        cabin = model.part(f"cabin_{cabin_index}")
        angle = _cabin_angle(cabin_index)

        cabin.visual(
            Cylinder(radius=0.02, length=0.20),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel_dark,
            name="hanger_rod",
        )
        cabin.visual(
            Box((0.34, 0.03, 0.03)),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=steel_dark,
            name="hanger_crosshead",
        )
        for side in (-1.0, 1.0):
            cabin.visual(
                Box((0.026, 0.024, 0.37)),
                origin=Origin(xyz=(side * 0.155, 0.0, -0.185)),
                material=steel_dark,
                name=f"hanger_strap_{'left' if side > 0.0 else 'right'}",
            )

        cabin.visual(
            Box((0.42, 0.18, 0.03)),
            origin=Origin(xyz=(0.0, 0.0, -0.378)),
            material=cabin_body,
            name="roof",
        )
        cabin.visual(
            Box((0.42, 0.18, 0.03)),
            origin=Origin(xyz=(0.0, 0.0, -0.688)),
            material=cabin_body,
            name="floor",
        )

        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                cabin.visual(
                    Box((0.03, 0.03, 0.28)),
                    origin=Origin(xyz=(sx * 0.195, sy * 0.075, -0.533)),
                    material=cabin_body,
                    name=None,
                )

        cabin.visual(
            Box((0.33, 0.006, 0.28)),
            origin=Origin(xyz=(0.0, 0.087, -0.533)),
            material=glass_blue,
            name="front_glass",
        )
        cabin.visual(
            Box((0.33, 0.006, 0.28)),
            origin=Origin(xyz=(0.0, -0.087, -0.533)),
            material=glass_blue,
            name="rear_glass",
        )
        cabin.visual(
            Box((0.006, 0.12, 0.28)),
            origin=Origin(xyz=(0.197, 0.0, -0.533)),
            material=glass_blue,
            name="right_glass",
        )
        cabin.visual(
            Box((0.006, 0.12, 0.28)),
            origin=Origin(xyz=(-0.197, 0.0, -0.533)),
            material=glass_blue,
            name="left_glass",
        )

        cabin.visual(
            Box((0.18, 0.028, 0.04)),
            origin=Origin(xyz=(0.0, 0.0, -0.393)),
            material=steel_dark,
            name="roof_brace",
        )

        model.articulation(
            f"wheel_to_cabin_{cabin_index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=cabin,
            origin=Origin(xyz=_wheel_point(PIVOT_RADIUS, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=2.0),
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
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("base_to_wheel")
    lowest_cabin = object_model.get_part("cabin_0")
    lowest_cabin_joint = object_model.get_articulation("wheel_to_cabin_0")

    ctx.check(
        "wheel articulation is continuous about horizontal axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )

    for cabin_index in range(CABIN_COUNT):
        cabin = object_model.get_part(f"cabin_{cabin_index}")
        joint = object_model.get_articulation(f"wheel_to_cabin_{cabin_index}")
        ctx.check(
            f"cabin {cabin_index} exists and hangs from a y-axis pivot",
            cabin is not None
            and joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"joint_type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_gap(
        lowest_cabin,
        base,
        axis="z",
        min_gap=0.02,
        max_gap=0.10,
        negative_elem="loading_plinth",
        name="lowest cabin clears the loading plinth",
    )
    ctx.expect_contact(
        lowest_cabin,
        wheel,
        elem_a="hanger_rod",
        elem_b="fork_0_left",
        name="lowest cabin hanger rod seats in the left fork bracket",
    )
    ctx.expect_contact(
        lowest_cabin,
        wheel,
        elem_a="hanger_rod",
        elem_b="fork_0_right",
        name="lowest cabin hanger rod seats in the right fork bracket",
    )

    rest_pos = ctx.part_world_position(lowest_cabin)
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(lowest_cabin)
    ctx.check(
        "wheel rotation carries the cabins around the axle",
        rest_pos is not None
        and turned_pos is not None
        and abs(turned_pos[0] - rest_pos[0]) > 1.4
        and abs(turned_pos[2] - rest_pos[2]) > 0.8,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple(0.5 * (lo + hi) for lo, hi in zip(aabb[0], aabb[1]))

    rest_side = _aabb_center(ctx.part_element_world_aabb(lowest_cabin, elem="right_glass"))
    with ctx.pose({lowest_cabin_joint: math.pi / 2.0}):
        rotated_side = _aabb_center(ctx.part_element_world_aabb(lowest_cabin, elem="right_glass"))
    ctx.check(
        "cabins can swing on their own hanger pivots",
        rest_side is not None
        and rotated_side is not None
        and abs(rotated_side[0] - rest_side[0]) > 0.10
        and abs(rotated_side[2] - rest_side[2]) > 0.10,
        details=f"rest={rest_side}, rotated={rotated_side}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
