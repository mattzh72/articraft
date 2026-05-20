from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi

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


def _segment_pose(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    run = hypot(dx, dy)
    length = hypot(run, dz)
    origin = Origin(
        xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
        rpy=(0.0, atan2(run, dz), atan2(dy, dx)),
    )
    return origin, length


def _add_segment_cylinder(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str | None = None,
) -> None:
    origin, length = _segment_pose(start, end)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _add_segment_box(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    thickness: float,
    width: float,
    material,
    name: str | None = None,
) -> None:
    origin, length = _segment_pose(start, end)
    part.visual(
        Box((thickness, width, length)),
        origin=origin,
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toddler_bucket_swing")

    steel = model.material("steel", rgba=(0.52, 0.56, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    seat_black = model.material("seat_black", rgba=(0.12, 0.12, 0.13, 1.0))
    strap_black = model.material("strap_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.35, 1.90, 2.15)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
    )

    top_beam_z = 2.06
    side_y = 0.74
    leg_spread_x = 0.60
    leg_radius = 0.045

    frame.visual(
        Cylinder(radius=0.055, length=1.76),
        origin=Origin(xyz=(0.0, 0.0, top_beam_z), rpy=(1.5708, 0.0, 0.0)),
        material=steel,
        name="top_beam",
    )

    for y_side, prefix in ((-side_y, "left"), (side_y, "right")):
        _add_segment_cylinder(
            frame,
            start=(leg_spread_x, y_side, 0.04),
            end=(0.0, y_side, top_beam_z),
            radius=leg_radius,
            material=steel,
            name=f"{prefix}_front_leg",
        )
        _add_segment_cylinder(
            frame,
            start=(-leg_spread_x, y_side, 0.04),
            end=(0.0, y_side, top_beam_z),
            radius=leg_radius,
            material=steel,
            name=f"{prefix}_rear_leg",
        )
        _add_segment_cylinder(
            frame,
            start=(0.47, y_side, 0.48),
            end=(-0.47, y_side, 0.48),
            radius=0.030,
            material=dark_steel,
            name=f"{prefix}_side_brace",
        )

    hanger_y = 0.24
    support_center_z = 1.981
    for y_pos, prefix in ((-hanger_y, "left"), (hanger_y, "right")):
        frame.visual(
            Box((0.085, 0.052, 0.050)),
            origin=Origin(xyz=(0.0, y_pos, support_center_z)),
            material=dark_steel,
            name=f"{prefix}_hanger_support",
        )
        frame.visual(
            Cylinder(radius=0.013, length=0.074),
            origin=Origin(
                xyz=(0.0, y_pos, support_center_z - 0.012),
                rpy=(1.5708, 0.0, 0.0),
            ),
            material=steel,
            name=f"{prefix}_hanger_joint",
        )

    bucket_swing = model.part("bucket_swing")
    bucket_swing.inertial = Inertial.from_geometry(
        Box((0.40, 0.38, 1.48)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -0.74)),
    )

    bucket_swing.visual(
        Box((0.040, 0.030, 0.022)),
        origin=Origin(xyz=(0.0, -hanger_y, -0.010)),
        material=strap_black,
        name="left_strap_cap",
    )
    bucket_swing.visual(
        Box((0.040, 0.030, 0.022)),
        origin=Origin(xyz=(0.0, hanger_y, -0.010)),
        material=strap_black,
        name="right_strap_cap",
    )

    _add_segment_box(
        bucket_swing,
        start=(0.0, -hanger_y, -0.020),
        end=(0.0, -0.165, -1.035),
        thickness=0.010,
        width=0.030,
        material=strap_black,
        name="left_support_strap",
    )
    _add_segment_box(
        bucket_swing,
        start=(0.0, hanger_y, -0.020),
        end=(0.0, 0.165, -1.035),
        thickness=0.010,
        width=0.030,
        material=strap_black,
        name="right_support_strap",
    )

    bucket_swing.visual(
        Box((0.050, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, -0.165, -1.055)),
        material=seat_black,
        name="left_seat_anchor",
    )
    bucket_swing.visual(
        Box((0.050, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, 0.165, -1.055)),
        material=seat_black,
        name="right_seat_anchor",
    )

    bucket_swing.visual(
        Box((0.240, 0.210, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -1.305)),
        material=seat_black,
        name="bucket_base",
    )
    bucket_swing.visual(
        Box((0.070, 0.340, 0.355)),
        origin=Origin(xyz=(-0.128, 0.0, -1.148), rpy=(0.0, -0.26, 0.0)),
        material=seat_black,
        name="bucket_back",
    )
    bucket_swing.visual(
        Box((0.245, 0.035, 0.315)),
        origin=Origin(xyz=(-0.005, -0.160, -1.154), rpy=(-0.12, 0.0, 0.0)),
        material=seat_black,
        name="left_bucket_side",
    )
    bucket_swing.visual(
        Box((0.245, 0.035, 0.315)),
        origin=Origin(xyz=(-0.005, 0.160, -1.154), rpy=(0.12, 0.0, 0.0)),
        material=seat_black,
        name="right_bucket_side",
    )
    bucket_swing.visual(
        Box((0.050, 0.108, 0.220)),
        origin=Origin(xyz=(0.104, -0.085, -1.170)),
        material=seat_black,
        name="left_front_guard",
    )
    bucket_swing.visual(
        Box((0.050, 0.108, 0.220)),
        origin=Origin(xyz=(0.104, 0.085, -1.170)),
        material=seat_black,
        name="right_front_guard",
    )
    bucket_swing.visual(
        Box((0.058, 0.060, 0.255)),
        origin=Origin(xyz=(0.086, 0.0, -1.185)),
        material=seat_black,
        name="center_crotch_post",
    )
    bucket_swing.visual(
        Cylinder(radius=0.028, length=0.275),
        origin=Origin(xyz=(-0.087, 0.0, -0.995), rpy=(pi / 2.0, 0.0, 0.0)),
        material=seat_black,
        name="rear_rim_pad",
    )
    bucket_swing.visual(
        Cylinder(radius=0.018, length=0.228),
        origin=Origin(xyz=(-0.003, -0.150, -0.998), rpy=(0.0, pi / 2.0, 0.0)),
        material=seat_black,
    )
    bucket_swing.visual(
        Cylinder(radius=0.018, length=0.228),
        origin=Origin(xyz=(-0.003, 0.150, -0.998), rpy=(0.0, pi / 2.0, 0.0)),
        material=seat_black,
    )

    model.articulation(
        "frame_to_bucket_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bucket_swing,
        origin=Origin(xyz=(0.0, 0.0, 1.955)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=1.5,
            lower=-0.75,
            upper=0.75,
        ),
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
    frame = object_model.get_part("frame")
    bucket_swing = object_model.get_part("bucket_swing")
    swing_joint = object_model.get_articulation("frame_to_bucket_swing")

    ctx.expect_overlap(
        bucket_swing,
        frame,
        axes="xy",
        elem_a="left_strap_cap",
        elem_b="left_hanger_support",
        min_overlap=0.025,
        name="left strap cap stays aligned under left hanger support",
    )
    ctx.expect_gap(
        frame,
        bucket_swing,
        axis="z",
        positive_elem="left_hanger_support",
        negative_elem="left_strap_cap",
        max_gap=0.003,
        max_penetration=0.0,
        name="left strap cap sits just below left hanger support",
    )
    ctx.expect_overlap(
        bucket_swing,
        frame,
        axes="xy",
        elem_a="right_strap_cap",
        elem_b="right_hanger_support",
        min_overlap=0.025,
        name="right strap cap stays aligned under right hanger support",
    )
    ctx.expect_gap(
        frame,
        bucket_swing,
        axis="z",
        positive_elem="right_hanger_support",
        negative_elem="right_strap_cap",
        max_gap=0.003,
        max_penetration=0.0,
        name="right strap cap sits just below right hanger support",
    )

    rest_aabb = ctx.part_element_world_aabb(bucket_swing, elem="bucket_base")
    with ctx.pose({swing_joint: 0.40}):
        swung_aabb = ctx.part_element_world_aabb(bucket_swing, elem="bucket_base")
    rest_center_x = None if rest_aabb is None else (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5
    swung_center_x = None if swung_aabb is None else (swung_aabb[0][0] + swung_aabb[1][0]) * 0.5
    ctx.check(
        "bucket seat swings forward around the top beam hangers",
        rest_center_x is not None
        and swung_center_x is not None
        and swung_center_x > rest_center_x + 0.18,
        details=f"rest_center_x={rest_center_x}, swung_center_x={swung_center_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
