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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


RIB_COUNT = 8
RIB_LENGTH = 0.094
RIB_THICKNESS = 0.00135
RIB_STACK_PITCH = 0.00155
RIB_ROOT_RADIUS = 0.0068
RIB_HOLE_RADIUS = 0.00255
RIB_BASE_HALF_WIDTH = 0.0047
RIB_MID_HALF_WIDTH = 0.00335
RIB_TIP_HALF_WIDTH = 0.00195
OPEN_BASE_ANGLE = math.radians(8.0)
OPEN_SPAN = math.radians(116.0)
OPEN_STEP = OPEN_SPAN / (RIB_COUNT - 1)
RIVET_SHAFT_RADIUS = 0.0022
RIVET_SHAFT_LENGTH = 0.0145
RIVET_HEAD_RADIUS = 0.0054
RIVET_HEAD_THICKNESS = 0.0014


def _arc_points(
    center: tuple[float, float],
    radius: float,
    start_angle: float,
    end_angle: float,
    segments: int,
) -> list[tuple[float, float]]:
    return [
        (
            center[0] + radius * math.cos(start_angle + (end_angle - start_angle) * i / segments),
            center[1] + radius * math.sin(start_angle + (end_angle - start_angle) * i / segments),
        )
        for i in range(segments + 1)
    ]


def _rib_outer_profile() -> list[tuple[float, float]]:
    join_angle = math.asin(RIB_BASE_HALF_WIDTH / RIB_ROOT_RADIUS)
    root_lower = (
        RIB_ROOT_RADIUS * math.cos(join_angle),
        -RIB_ROOT_RADIUS * math.sin(join_angle),
    )
    root_upper = (
        RIB_ROOT_RADIUS * math.cos(join_angle),
        RIB_ROOT_RADIUS * math.sin(join_angle),
    )
    tip_center_x = RIB_LENGTH - RIB_TIP_HALF_WIDTH

    lower_path = [
        root_lower,
        (0.018, -0.00445),
        (0.050, -RIB_MID_HALF_WIDTH),
        (tip_center_x, -RIB_TIP_HALF_WIDTH),
    ]
    tip_arc = _arc_points(
        (tip_center_x, 0.0),
        RIB_TIP_HALF_WIDTH,
        -math.pi / 2.0,
        math.pi / 2.0,
        10,
    )
    upper_path = [
        (0.050, RIB_MID_HALF_WIDTH),
        (0.018, 0.00445),
        root_upper,
    ]
    root_arc = _arc_points(
        (0.0, 0.0),
        RIB_ROOT_RADIUS,
        join_angle,
        2.0 * math.pi - join_angle,
        18,
    )

    return lower_path + tip_arc[1:] + upper_path + root_arc[1:]


def _rib_hole_profile() -> list[tuple[float, float]]:
    return _arc_points((0.0, 0.0), RIB_HOLE_RADIUS, 0.0, 2.0 * math.pi, 20)[:-1]


def _stack_z(index: int) -> float:
    return (index - 0.5 * (RIB_COUNT - 1)) * RIB_STACK_PITCH


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_fan_fidget")

    rib_material = model.material("rib_material", rgba=(0.82, 0.72, 0.56, 1.0))
    rivet_material = model.material("rivet_material", rgba=(0.75, 0.68, 0.48, 1.0))

    rivet = model.part("rivet")
    rivet.visual(
        Cylinder(radius=RIVET_SHAFT_RADIUS, length=RIVET_SHAFT_LENGTH),
        material=rivet_material,
        name="shaft",
    )
    rivet.visual(
        Cylinder(radius=RIVET_HEAD_RADIUS, length=RIVET_HEAD_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * (RIVET_SHAFT_LENGTH + RIVET_HEAD_THICKNESS))),
        material=rivet_material,
        name="lower_head",
    )
    rivet.visual(
        Cylinder(radius=RIVET_HEAD_RADIUS, length=RIVET_HEAD_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (RIVET_SHAFT_LENGTH + RIVET_HEAD_THICKNESS))),
        material=rivet_material,
        name="upper_head",
    )
    rivet.inertial = Inertial.from_geometry(
        Cylinder(radius=RIVET_HEAD_RADIUS, length=RIVET_SHAFT_LENGTH + 2.0 * RIVET_HEAD_THICKNESS),
        mass=0.01,
    )

    rib_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rib_outer_profile(),
            [_rib_hole_profile()],
            RIB_THICKNESS,
            center=True,
        ),
        "rib_shell",
    )

    for index in range(RIB_COUNT):
        rib = model.part(f"rib_{index}")
        rib.visual(rib_mesh, material=rib_material, name="rib_shell")
        rib.inertial = Inertial.from_geometry(
            Box((RIB_LENGTH + RIB_ROOT_RADIUS, 2.0 * RIB_BASE_HALF_WIDTH, RIB_THICKNESS)),
            mass=0.004,
            origin=Origin(xyz=(0.5 * (RIB_LENGTH - RIB_ROOT_RADIUS), 0.0, 0.0)),
        )

        open_angle = OPEN_BASE_ANGLE + index * OPEN_STEP
        model.articulation(
            f"rivet_to_rib_{index}",
            ArticulationType.REVOLUTE,
            parent=rivet,
            child=rib,
            origin=Origin(xyz=(0.0, 0.0, _stack_z(index)), rpy=(0.0, 0.0, open_angle)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.08,
                velocity=6.0,
                lower=-open_angle,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rivet = object_model.get_part("rivet")
    ribs = [object_model.get_part(f"rib_{index}") for index in range(RIB_COUNT)]
    joints = [object_model.get_articulation(f"rivet_to_rib_{index}") for index in range(RIB_COUNT)]

    open_angles = []
    limits_ok = True
    for index, joint in enumerate(joints):
        limits = joint.motion_limits
        expected_open = OPEN_BASE_ANGLE + index * OPEN_STEP
        open_angles.append(expected_open)
        limits_ok = limits_ok and limits is not None
        limits_ok = limits_ok and limits.lower is not None and abs(limits.lower + expected_open) < 1e-9
        limits_ok = limits_ok and limits.upper is not None and abs(limits.upper) < 1e-9
    ctx.check(
        "fan has eight rib joints with staged opening limits",
        limits_ok,
        details=f"open_angles_deg={[round(math.degrees(a), 3) for a in open_angles]}",
    )

    closed_pose = {joint: joint.motion_limits.lower for joint in joints if joint.motion_limits is not None}
    with ctx.pose(closed_pose):
        for index in range(RIB_COUNT - 1):
            ctx.expect_gap(
                ribs[index + 1],
                ribs[index],
                axis="z",
                min_gap=0.00005,
                max_gap=0.00035,
                name=f"rib_stack_gap_{index}",
            )

        ctx.expect_gap(
            ribs[0],
            rivet,
            axis="z",
            positive_elem="rib_shell",
            negative_elem="lower_head",
            min_gap=0.0008,
            max_gap=0.0014,
            name="lower_head_sits_below_stack",
        )
        ctx.expect_gap(
            rivet,
            ribs[-1],
            axis="z",
            positive_elem="upper_head",
            negative_elem="rib_shell",
            min_gap=0.0008,
            max_gap=0.0014,
            name="upper_head_sits_above_stack",
        )

        closed_centers = []
        for rib in ribs:
            aabb = ctx.part_element_world_aabb(rib, elem="rib_shell")
            if aabb is not None:
                closed_centers.append(_aabb_center(aabb))
        closed_angles = [math.degrees(math.atan2(center[1], center[0])) for center in closed_centers if center[0] > 0.0]
        ctx.check(
            "closed ribs align into one bundle",
            len(closed_angles) == RIB_COUNT and max(abs(angle) for angle in closed_angles) < 1.0,
            details=f"closed_angles_deg={[round(angle, 3) for angle in closed_angles]}",
        )

    with ctx.pose({joint: 0.0 for joint in joints}):
        open_aabbs = [ctx.part_element_world_aabb(rib, elem="rib_shell") for rib in ribs]
        open_centers = [_aabb_center(aabb) for aabb in open_aabbs if aabb is not None]
        center_angles = [math.degrees(math.atan2(center[1], center[0])) for center in open_centers]
        angle_steps = [center_angles[index + 1] - center_angles[index] for index in range(len(center_angles) - 1)]
        fan_y_max = max(aabb[1][1] for aabb in open_aabbs if aabb is not None)
        fan_y_min = min(aabb[0][1] for aabb in open_aabbs if aabb is not None)

        ctx.check(
            "open ribs sweep in order",
            len(center_angles) == RIB_COUNT and all(step > 10.0 for step in angle_steps),
            details=f"center_angles_deg={[round(angle, 3) for angle in center_angles]} steps_deg={[round(step, 3) for step in angle_steps]}",
        )
        ctx.check(
            "open fan reaches a broad hand-fan arc",
            len(center_angles) == RIB_COUNT
            and (center_angles[-1] - center_angles[0]) > 95.0
            and (fan_y_max - fan_y_min) > 0.07,
            details=(
                f"span_deg={round(center_angles[-1] - center_angles[0], 3) if len(center_angles) == RIB_COUNT else None}, "
                f"y_span={round(fan_y_max - fan_y_min, 5)}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
