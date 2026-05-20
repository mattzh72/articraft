from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _segment(a: tuple[float, float, float], b: tuple[float, float, float], radius: float) -> MeshGeometry:
    return tube_from_spline_points(
        [a, b],
        radius=radius,
        samples_per_segment=1,
        radial_segments=14,
        cap_ends=True,
    )


def _merge_segments(segments: list[tuple[tuple[float, float, float], tuple[float, float, float], float]]) -> MeshGeometry:
    merged = MeshGeometry()
    for start, end, radius in segments:
        merged.merge(_segment(start, end, radius))
    return merged


def _panel_segments(
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z: float,
    border_radius: float,
    rail_radius: float,
    rail_count: int,
) -> list[tuple[tuple[float, float, float], tuple[float, float, float], float]]:
    segments = [
        ((x_min, y_min, z), (x_max, y_min, z), border_radius),
        ((x_min, y_max, z), (x_max, y_max, z), border_radius),
        ((x_min, y_min, z), (x_min, y_max, z), border_radius),
        ((x_max, y_min, z), (x_max, y_max, z), border_radius),
    ]
    if rail_count > 0:
        for index in range(rail_count):
            t = (index + 1) / (rail_count + 1)
            y = y_min + (y_max - y_min) * t
            overlap = border_radius * 0.45
            segments.append(((x_min - overlap, y, z), (x_max + overlap, y, z), rail_radius))
    return segments


def _top_rack_geometry() -> MeshGeometry:
    return _merge_segments(
        _panel_segments(
            x_min=-0.36,
            x_max=0.36,
            y_min=-0.26,
            y_max=0.26,
            z=0.92,
            border_radius=0.010,
            rail_radius=0.007,
            rail_count=7,
        )
    )


def _stand_geometry() -> MeshGeometry:
    top_z = 0.92
    foot_z = 0.05
    side_top_z = 0.84
    side_x = 0.34
    front_y = 0.26
    back_y = -0.26
    border_radius = 0.010
    rail_radius = 0.007

    segments: list[tuple[tuple[float, float, float], tuple[float, float, float], float]] = []

    for sign in (-1.0, 1.0):
        x = sign * side_x
        segments.extend(
            [
                ((x, front_y - 0.02, foot_z), (x, front_y - 0.02, side_top_z), border_radius),
                ((x, back_y + 0.02, foot_z), (x, back_y + 0.02, side_top_z), border_radius),
                ((x, back_y, foot_z), (x, front_y, foot_z), border_radius),
                ((x, back_y + 0.01, 0.42), (x, front_y - 0.01, 0.42), rail_radius),
                ((x, front_y - 0.02, side_top_z), (sign * 0.36, front_y, top_z), border_radius),
                ((x, back_y + 0.02, side_top_z), (sign * 0.36, back_y, top_z), border_radius),
                ((x, front_y - 0.02, foot_z), (sign * 0.29, 0.12, 0.46), rail_radius),
                ((x, back_y + 0.02, foot_z), (sign * 0.29, -0.12, 0.46), rail_radius),
                ((x, back_y + 0.02, 0.42), (sign * 0.27, -0.238, 0.48), 0.006),
            ]
        )

    return _merge_segments(segments)


def _wing_geometry(direction: float) -> MeshGeometry:
    inner_x = direction * 0.02
    outer_x = direction * 0.30
    segments = _panel_segments(
        x_min=min(inner_x, outer_x),
        x_max=max(inner_x, outer_x),
        y_min=-0.26,
        y_max=0.26,
        z=0.0,
        border_radius=0.009,
        rail_radius=0.006,
        rail_count=4,
    )
    return _merge_segments(segments)


def _lower_support_geometry() -> MeshGeometry:
    rear_y = 0.05
    rear_z = -0.075
    front_y = 0.24
    front_z = -0.205
    hinge_half_width = 0.27
    rear_half_width = 0.20
    front_half_width = 0.24
    border_radius = 0.008
    rail_radius = 0.0055

    segments: list[tuple[tuple[float, float, float], tuple[float, float, float], float]] = [
        ((-rear_half_width, rear_y, rear_z), (-front_half_width, front_y, front_z), border_radius),
        ((rear_half_width, rear_y, rear_z), (front_half_width, front_y, front_z), border_radius),
        ((-rear_half_width, rear_y, rear_z), (rear_half_width, rear_y, rear_z), border_radius),
        ((-front_half_width, front_y, front_z), (front_half_width, front_y, front_z), border_radius),
        ((-hinge_half_width, -0.0375, 0.0), (-rear_half_width, rear_y, rear_z), rail_radius),
        ((hinge_half_width, -0.0375, 0.0), (rear_half_width, rear_y, rear_z), rail_radius),
        ((-hinge_half_width, -0.0375, 0.0), (hinge_half_width, -0.0375, 0.0), 0.0045),
    ]

    for index in range(4):
        t = (index + 1) / 5.0
        y = rear_y + (front_y - rear_y) * t
        z = rear_z + (front_z - rear_z) * t
        half_width = rear_half_width + (front_half_width - rear_half_width) * t
        segments.append(((-half_width, y, z), (half_width, y, z), rail_radius))

    return _merge_segments(segments)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    aluminum = model.material("aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    polymer = model.material("polymer", rgba=(0.16, 0.17, 0.18, 1.0))

    central_frame = model.part("central_frame")
    central_frame.visual(
        mesh_from_geometry(_top_rack_geometry(), "top_rack"),
        material=aluminum,
        name="top_rack",
    )
    central_frame.visual(
        mesh_from_geometry(_stand_geometry(), "stand_frame"),
        material=aluminum,
        name="stand_frame",
    )
    central_frame.visual(
        Box((0.02, 0.03, 0.016)),
        origin=Origin(xyz=(-0.362, 0.22, 0.92)),
        material=polymer,
        name="left_wing_hinge_cap_front",
    )
    central_frame.visual(
        Box((0.02, 0.03, 0.016)),
        origin=Origin(xyz=(-0.362, -0.22, 0.92)),
        material=polymer,
        name="left_wing_hinge_cap_rear",
    )
    central_frame.visual(
        Box((0.02, 0.03, 0.016)),
        origin=Origin(xyz=(0.362, 0.22, 0.92)),
        material=polymer,
        name="right_wing_hinge_cap_front",
    )
    central_frame.visual(
        Box((0.02, 0.03, 0.016)),
        origin=Origin(xyz=(0.362, -0.22, 0.92)),
        material=polymer,
        name="right_wing_hinge_cap_rear",
    )
    central_frame.visual(
        Box((0.052, 0.032, 0.02)),
        origin=Origin(xyz=(-0.27, -0.228, 0.48)),
        material=polymer,
        name="left_support_hinge_mount",
    )
    central_frame.visual(
        Box((0.052, 0.032, 0.02)),
        origin=Origin(xyz=(0.27, -0.228, 0.48)),
        material=polymer,
        name="right_support_hinge_mount",
    )
    central_frame.inertial = Inertial.from_geometry(
        Box((0.78, 0.58, 0.92)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    left_wing_frame = model.part("left_wing_frame")
    left_wing_frame.visual(
        mesh_from_geometry(_wing_geometry(-1.0), "left_wing_frame"),
        material=aluminum,
        name="wing_frame",
    )
    left_wing_frame.visual(
        Box((0.032, 0.026, 0.016)),
        origin=Origin(xyz=(-0.018, 0.22, 0.0)),
        material=polymer,
        name="hinge_cap_front",
    )
    left_wing_frame.visual(
        Box((0.032, 0.026, 0.016)),
        origin=Origin(xyz=(-0.018, -0.22, 0.0)),
        material=polymer,
        name="hinge_cap_rear",
    )
    left_wing_frame.inertial = Inertial.from_geometry(
        Box((0.32, 0.56, 0.06)),
        mass=1.2,
        origin=Origin(xyz=(-0.16, 0.0, 0.0)),
    )

    right_wing_frame = model.part("right_wing_frame")
    right_wing_frame.visual(
        mesh_from_geometry(_wing_geometry(1.0), "right_wing_frame"),
        material=aluminum,
        name="wing_frame",
    )
    right_wing_frame.visual(
        Box((0.032, 0.026, 0.016)),
        origin=Origin(xyz=(0.018, 0.22, 0.0)),
        material=polymer,
        name="hinge_cap_front",
    )
    right_wing_frame.visual(
        Box((0.032, 0.026, 0.016)),
        origin=Origin(xyz=(0.018, -0.22, 0.0)),
        material=polymer,
        name="hinge_cap_rear",
    )
    right_wing_frame.inertial = Inertial.from_geometry(
        Box((0.32, 0.56, 0.06)),
        mass=1.2,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )

    lower_support_frame = model.part("lower_support_frame")
    lower_support_frame.visual(
        mesh_from_geometry(_lower_support_geometry(), "lower_support_frame"),
        material=aluminum,
        name="support_frame",
    )
    lower_support_frame.inertial = Inertial.from_geometry(
        Box((0.56, 0.32, 0.24)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.10, -0.10)),
    )

    model.articulation(
        "left_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=left_wing_frame,
        origin=Origin(xyz=(-0.37, 0.0, 0.92)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "right_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=right_wing_frame,
        origin=Origin(xyz=(0.37, 0.0, 0.92)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "lower_support_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=lower_support_frame,
        origin=Origin(xyz=(0.0, -0.17, 0.48)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central_frame = object_model.get_part("central_frame")
    left_wing_frame = object_model.get_part("left_wing_frame")
    right_wing_frame = object_model.get_part("right_wing_frame")
    lower_support_frame = object_model.get_part("lower_support_frame")
    left_wing_hinge = object_model.get_articulation("left_wing_hinge")
    right_wing_hinge = object_model.get_articulation("right_wing_hinge")
    lower_support_hinge = object_model.get_articulation("lower_support_hinge")

    with ctx.pose({left_wing_hinge: 0.0, right_wing_hinge: 0.0, lower_support_hinge: 0.0}):
        ctx.expect_overlap(
            left_wing_frame,
            central_frame,
            axes="y",
            elem_a="wing_frame",
            elem_b="top_rack",
            min_overlap=0.44,
            name="left wing aligns with the center rack span",
        )
        ctx.expect_gap(
            central_frame,
            left_wing_frame,
            axis="x",
            positive_elem="top_rack",
            negative_elem="wing_frame",
            min_gap=0.002,
            max_gap=0.05,
            name="left wing parks beside the center rack without collision",
        )
        ctx.expect_overlap(
            right_wing_frame,
            central_frame,
            axes="y",
            elem_a="wing_frame",
            elem_b="top_rack",
            min_overlap=0.44,
            name="right wing aligns with the center rack span",
        )
        ctx.expect_gap(
            right_wing_frame,
            central_frame,
            axis="x",
            positive_elem="wing_frame",
            negative_elem="top_rack",
            min_gap=0.002,
            max_gap=0.05,
            name="right wing parks beside the center rack without collision",
        )
        ctx.expect_gap(
            central_frame,
            lower_support_frame,
            axis="z",
            positive_elem="top_rack",
            negative_elem="support_frame",
            min_gap=0.34,
            max_gap=0.52,
            name="lower support frame hangs clearly below the top rack",
        )
        ctx.expect_overlap(
            lower_support_frame,
            central_frame,
            axes="x",
            elem_a="support_frame",
            elem_b="stand_frame",
            min_overlap=0.50,
            name="lower support frame stays centered within the stand width",
        )

    left_rest_aabb = ctx.part_element_world_aabb(left_wing_frame, elem="wing_frame")
    right_rest_aabb = ctx.part_element_world_aabb(right_wing_frame, elem="wing_frame")
    support_rest_aabb = ctx.part_element_world_aabb(lower_support_frame, elem="support_frame")

    with ctx.pose(
        {
            left_wing_hinge: math.radians(60.0),
            right_wing_hinge: math.radians(60.0),
            lower_support_hinge: math.radians(62.0),
        }
    ):
        left_raised_aabb = ctx.part_element_world_aabb(left_wing_frame, elem="wing_frame")
        right_raised_aabb = ctx.part_element_world_aabb(right_wing_frame, elem="wing_frame")
        support_folded_aabb = ctx.part_element_world_aabb(lower_support_frame, elem="support_frame")

    ctx.check(
        "left wing lifts above the center rack",
        left_rest_aabb is not None
        and left_raised_aabb is not None
        and left_raised_aabb[1][2] > left_rest_aabb[1][2] + 0.16,
        details=f"rest={left_rest_aabb}, raised={left_raised_aabb}",
    )
    ctx.check(
        "right wing lifts above the center rack",
        right_rest_aabb is not None
        and right_raised_aabb is not None
        and right_raised_aabb[1][2] > right_rest_aabb[1][2] + 0.16,
        details=f"rest={right_rest_aabb}, raised={right_raised_aabb}",
    )
    ctx.check(
        "lower support frame folds upward",
        support_rest_aabb is not None
        and support_folded_aabb is not None
        and support_folded_aabb[1][2] > support_rest_aabb[1][2] + 0.16,
        details=f"rest={support_rest_aabb}, folded={support_folded_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
