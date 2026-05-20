from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

from sdk import ArticulatedObject, ArticulationType, Box, Inertial, MotionLimits, Origin, TestContext, TestReport


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _beam_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    mid = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    horizontal = sqrt(dx * dx + dy * dy)
    yaw = atan2(dy, dx) if horizontal > 1e-9 else 0.0
    pitch = atan2(horizontal, dz)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _add_beam(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    size_x: float,
    size_y: float,
    material,
) -> None:
    origin, length = _beam_origin(start, end)
    part.visual(Box((size_x, size_y, length)), origin=origin, material=material, name=name)


def _interpolate_point(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    t: float,
) -> tuple[float, float, float]:
    return (_lerp(start[0], end[0], t), _lerp(start[1], end[1], t), _lerp(start[2], end[2], t))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    cap_orange = model.material("cap_orange", rgba=(0.94, 0.43, 0.08, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.56, 0.24, 0.98)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.02, 0.49)),
    )

    front_left_top = (-0.185, 0.000, 0.905)
    front_right_top = (0.185, 0.000, 0.905)
    front_left_bottom = (-0.255, 0.095, 0.020)
    front_right_bottom = (0.255, 0.095, 0.020)

    _add_beam(front_frame, "left_front_leg", front_left_top, front_left_bottom, 0.046, 0.018, aluminum)
    _add_beam(front_frame, "right_front_leg", front_right_top, front_right_bottom, 0.046, 0.018, aluminum)

    front_frame.visual(
        Box((0.405, 0.110, 0.055)),
        origin=Origin(xyz=(0.0, 0.055, 0.9225)),
        material=cap_orange,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.330, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, -0.030, 0.895)),
        material=dark_gray,
        name="rear_cap_spine",
    )
    front_frame.visual(
        Box((0.042, 0.024, 0.042)),
        origin=Origin(xyz=(-0.185, -0.028, 0.920)),
        material=dark_gray,
        name="left_hinge_cheek",
    )
    front_frame.visual(
        Box((0.042, 0.024, 0.042)),
        origin=Origin(xyz=(0.185, -0.028, 0.920)),
        material=dark_gray,
        name="right_hinge_cheek",
    )

    tread_heights = (0.220, 0.420, 0.620, 0.790)
    for index, tread_z in enumerate(tread_heights, start=1):
        t = (tread_z - front_left_bottom[2]) / (front_left_top[2] - front_left_bottom[2])
        left_point = _interpolate_point(front_left_bottom, front_left_top, t)
        right_point = _interpolate_point(front_right_bottom, front_right_top, t)
        tread_width = abs(right_point[0] - left_point[0]) - 0.038
        tread_center_y = (left_point[1] + right_point[1]) * 0.5 + 0.028
        front_frame.visual(
            Box((tread_width, 0.108, 0.028)),
            origin=Origin(xyz=(0.0, tread_center_y, tread_z)),
            material=aluminum,
            name=f"tread_{index}",
        )

    front_frame.visual(
        Box((0.082, 0.045, 0.028)),
        origin=Origin(xyz=(-0.255, 0.095, 0.014)),
        material=dark_gray,
        name="left_front_foot",
    )
    front_frame.visual(
        Box((0.082, 0.045, 0.028)),
        origin=Origin(xyz=(0.255, 0.095, 0.014)),
        material=dark_gray,
        name="right_front_foot",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.64, 1.04)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.280, -0.500)),
    )

    rear_left_top = (-0.185, 0.000, -0.030)
    rear_right_top = (0.185, 0.000, -0.030)
    rear_left_bottom = (-0.220, -0.560, -0.940)
    rear_right_bottom = (0.220, -0.560, -0.940)

    _add_beam(rear_frame, "left_rear_leg", rear_left_top, rear_left_bottom, 0.040, 0.016, aluminum)
    _add_beam(rear_frame, "right_rear_leg", rear_right_top, rear_right_bottom, 0.040, 0.016, aluminum)

    rear_frame.visual(
        Box((0.394, 0.034, 0.060)),
        origin=Origin(xyz=(0.0, -0.038, -0.052)),
        material=dark_gray,
        name="rear_top_bridge",
    )
    rear_frame.visual(
        Box((0.050, 0.012, 0.040)),
        origin=Origin(xyz=(-0.185, 0.014, -0.009)),
        material=dark_gray,
        name="left_rear_hinge_tab",
    )
    rear_frame.visual(
        Box((0.050, 0.012, 0.040)),
        origin=Origin(xyz=(0.185, 0.014, -0.009)),
        material=dark_gray,
        name="right_rear_hinge_tab",
    )
    rear_frame.visual(
        Box((0.032, 0.036, 0.032)),
        origin=Origin(xyz=(-0.192, -0.006, -0.024)),
        material=dark_gray,
        name="left_rear_hinge_web",
    )
    rear_frame.visual(
        Box((0.032, 0.036, 0.032)),
        origin=Origin(xyz=(0.192, -0.006, -0.024)),
        material=dark_gray,
        name="right_rear_hinge_web",
    )

    for name, t in (("rear_brace_upper", 0.32), ("rear_brace_mid", 0.56), ("rear_brace_low", 0.80)):
        left_point = _interpolate_point(rear_left_top, rear_left_bottom, t)
        right_point = _interpolate_point(rear_right_top, rear_right_bottom, t)
        brace_width = abs(right_point[0] - left_point[0]) + 0.014
        rear_frame.visual(
            Box((brace_width, 0.030, 0.034)),
            origin=Origin(
                xyz=(
                    0.0,
                    (left_point[1] + right_point[1]) * 0.5,
                    (left_point[2] + right_point[2]) * 0.5,
                )
            ),
            material=aluminum,
            name=name,
        )

    rear_frame.visual(
        Box((0.088, 0.048, 0.034)),
        origin=Origin(xyz=(-0.220, -0.560, -0.947)),
        material=dark_gray,
        name="left_rear_foot",
    )
    rear_frame.visual(
        Box((0.088, 0.048, 0.034)),
        origin=Origin(xyz=(0.220, -0.560, -0.947)),
        material=dark_gray,
        name="right_rear_foot",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, -0.060, 0.940)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=0.0, upper=0.82),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    rear_hinge = object_model.get_articulation("rear_hinge")

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="y",
        positive_elem="left_front_foot",
        negative_elem="left_rear_foot",
        min_gap=0.58,
        max_gap=0.78,
        name="open stance keeps rear feet well behind the climbing feet",
    )

    tread_aabb = ctx.part_element_world_aabb(front_frame, elem="tread_2")
    tread_width = None if tread_aabb is None else tread_aabb[1][0] - tread_aabb[0][0]
    ctx.check(
        "main tread is wide enough for a step ladder",
        tread_width is not None and tread_width >= 0.37,
        details=f"tread_width={tread_width}",
    )
    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a="left_hinge_cheek",
        elem_b="left_rear_hinge_tab",
        name="rear support frame is physically pinned into the top hinge cheek",
    )

    rest_rear_foot = ctx.part_element_world_aabb(rear_frame, elem="left_rear_foot")
    with ctx.pose({rear_hinge: 0.65}):
        folded_rear_foot = ctx.part_element_world_aabb(rear_frame, elem="left_rear_foot")
        ctx.check(
            "rear frame folds forward from the open stance",
            rest_rear_foot is not None
            and folded_rear_foot is not None
            and folded_rear_foot[0][1] > rest_rear_foot[0][1] + 0.45,
            details=f"rest={rest_rear_foot}, folded={folded_rear_foot}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
