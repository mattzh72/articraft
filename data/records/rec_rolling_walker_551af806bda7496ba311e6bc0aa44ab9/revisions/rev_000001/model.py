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
    mesh_from_geometry,
    tube_from_spline_points,
)


FRAME_TUBE_RADIUS = 0.013
FRAME_HALF_WIDTH = 0.285
FRONT_Y = 0.20
REAR_Y = -0.18
HANDLE_Z = 0.88
FRONT_CASTER_MOUNT_Z = 0.285
REAR_TIP_LENGTH = 0.045
CASTER_WHEEL_RADIUS = 0.098
CASTER_WHEEL_WIDTH = 0.036
CASTER_AXLE_Z = -0.187


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float = FRAME_TUBE_RADIUS,
    samples_per_segment: int = 16,
    radial_segments: int = 18,
) -> object:
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _side_frame_points(side_x: float) -> list[tuple[float, float, float]]:
    return [
        (side_x, REAR_Y, REAR_TIP_LENGTH),
        (side_x, REAR_Y, 0.50),
        (side_x, -0.12, 0.80),
        (side_x, 0.00, HANDLE_Z),
        (side_x, 0.12, HANDLE_Z),
        (side_x, 0.18, 0.80),
        (side_x, FRONT_Y, 0.50),
        (side_x, FRONT_Y, FRONT_CASTER_MOUNT_Z),
    ]


def _build_caster_fork(part, *, material) -> None:
    part.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=material,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=material,
        name="swivel_collar",
    )
    part.visual(
        Box((0.060, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.081)),
        material=material,
        name="fork_crown",
    )
    part.visual(
        Box((0.008, 0.024, 0.146)),
        origin=Origin(xyz=(0.026, 0.0, -0.156)),
        material=material,
        name="left_blade",
    )
    part.visual(
        Box((0.008, 0.024, 0.146)),
        origin=Origin(xyz=(-0.026, 0.0, -0.156)),
        material=material,
        name="right_blade",
    )


def _build_caster_wheel(part, *, tire_material, hub_material) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=CASTER_WHEEL_RADIUS, length=CASTER_WHEEL_WIDTH),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.020),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.028),
        origin=spin_origin,
        material=hub_material,
        name="core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    frame_aluminum = model.material("frame_aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    grip_foam = model.material("grip_foam", rgba=(0.12, 0.12, 0.13, 1.0))
    ferrule_rubber = model.material("ferrule_rubber", rgba=(0.20, 0.20, 0.21, 1.0))
    fork_gray = model.material("fork_gray", rgba=(0.65, 0.67, 0.70, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.73, 0.74, 0.76, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.64, 0.54, 0.90)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.01, 0.45)),
    )

    left_side_points = _side_frame_points(FRAME_HALF_WIDTH)
    right_side_points = _mirror_x(left_side_points)
    frame_geom = tube_from_spline_points(
        left_side_points,
        radius=FRAME_TUBE_RADIUS,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=False,
    )
    frame_geom.merge(
        tube_from_spline_points(
            right_side_points,
            radius=FRAME_TUBE_RADIUS,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=False,
        )
    )
    for points in (
        [(-FRAME_HALF_WIDTH, FRONT_Y, 0.50), (FRAME_HALF_WIDTH, FRONT_Y, 0.50)],
        [(-FRAME_HALF_WIDTH, REAR_Y, 0.50), (FRAME_HALF_WIDTH, REAR_Y, 0.50)],
        [(-FRAME_HALF_WIDTH, 0.00, HANDLE_Z), (FRAME_HALF_WIDTH, 0.00, HANDLE_Z)],
    ):
        frame_geom.merge(
            tube_from_spline_points(
                points,
                radius=FRAME_TUBE_RADIUS * 0.92,
                samples_per_segment=2,
                radial_segments=16,
                cap_ends=True,
            )
        )
    frame.visual(
        mesh_from_geometry(frame_geom, "walker_frame"),
        material=frame_aluminum,
        name="frame_shell",
    )

    grip_length = 0.17
    frame.visual(
        Cylinder(radius=0.019, length=grip_length),
        origin=Origin(xyz=(FRAME_HALF_WIDTH, 0.035, HANDLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_foam,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.019, length=grip_length),
        origin=Origin(xyz=(-FRAME_HALF_WIDTH, 0.035, HANDLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_foam,
        name="right_grip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=REAR_TIP_LENGTH),
        origin=Origin(xyz=(FRAME_HALF_WIDTH, REAR_Y, REAR_TIP_LENGTH * 0.5)),
        material=ferrule_rubber,
        name="left_rear_tip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=REAR_TIP_LENGTH),
        origin=Origin(xyz=(-FRAME_HALF_WIDTH, REAR_Y, REAR_TIP_LENGTH * 0.5)),
        material=ferrule_rubber,
        name="right_rear_tip",
    )
    for side_x, name in ((FRAME_HALF_WIDTH, "left_caster_socket"), (-FRAME_HALF_WIDTH, "right_caster_socket")):
        frame.visual(
            Cylinder(radius=0.020, length=0.040),
            origin=Origin(xyz=(side_x, FRONT_Y, FRONT_CASTER_MOUNT_Z + 0.020)),
            material=fork_gray,
            name=name,
        )

    left_caster = model.part("left_caster")
    left_caster.inertial = Inertial.from_geometry(
        Box((0.08, 0.05, 0.24)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )
    _build_caster_fork(left_caster, material=fork_gray)

    right_caster = model.part("right_caster")
    right_caster.inertial = Inertial.from_geometry(
        Box((0.08, 0.05, 0.24)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )
    _build_caster_fork(right_caster, material=fork_gray)

    left_front_wheel = model.part("left_front_wheel")
    left_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=CASTER_WHEEL_RADIUS, length=CASTER_WHEEL_WIDTH),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_caster_wheel(left_front_wheel, tire_material=wheel_rubber, hub_material=hub_gray)

    right_front_wheel = model.part("right_front_wheel")
    right_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=CASTER_WHEEL_RADIUS, length=CASTER_WHEEL_WIDTH),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_caster_wheel(right_front_wheel, tire_material=wheel_rubber, hub_material=hub_gray)

    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_caster,
        origin=Origin(xyz=(FRAME_HALF_WIDTH, FRONT_Y, FRONT_CASTER_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_caster,
        origin=Origin(xyz=(-FRAME_HALF_WIDTH, FRONT_Y, FRONT_CASTER_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster,
        child=left_front_wheel,
        origin=Origin(xyz=(0.0, 0.0, CASTER_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster,
        child=right_front_wheel,
        origin=Origin(xyz=(0.0, 0.0, CASTER_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
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
    left_wheel = object_model.get_part("left_front_wheel")
    right_wheel = object_model.get_part("right_front_wheel")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")
    left_grip_aabb = ctx.part_element_world_aabb(frame, elem="left_grip")
    right_grip_aabb = ctx.part_element_world_aabb(frame, elem="right_grip")
    left_tip_aabb = ctx.part_element_world_aabb(frame, elem="left_rear_tip")
    right_tip_aabb = ctx.part_element_world_aabb(frame, elem="right_rear_tip")
    left_wheel_aabb = ctx.part_element_world_aabb(left_wheel, elem="tire")
    right_wheel_aabb = ctx.part_element_world_aabb(right_wheel, elem="tire")

    grip_height_ok = (
        left_grip_aabb is not None
        and right_grip_aabb is not None
        and left_grip_aabb[1][2] > 0.86
        and right_grip_aabb[1][2] > 0.86
    )
    ctx.check(
        "grips are at adult walker height",
        grip_height_ok,
        details=f"left_grip={left_grip_aabb}, right_grip={right_grip_aabb}",
    )

    grip_spacing_ok = (
        left_grip_aabb is not None
        and right_grip_aabb is not None
        and 0.50 <= (left_grip_aabb[0][0] - right_grip_aabb[1][0]) <= 0.62
    )
    ctx.check(
        "grips are realistically spaced",
        grip_spacing_ok,
        details=f"left_grip={left_grip_aabb}, right_grip={right_grip_aabb}",
    )

    rear_tips_grounded = (
        left_tip_aabb is not None
        and right_tip_aabb is not None
        and abs(left_tip_aabb[0][2]) <= 0.002
        and abs(right_tip_aabb[0][2]) <= 0.002
    )
    ctx.check(
        "rear support tips meet the floor",
        rear_tips_grounded,
        details=f"left_tip={left_tip_aabb}, right_tip={right_tip_aabb}",
    )

    front_wheels_grounded = (
        left_wheel_aabb is not None
        and right_wheel_aabb is not None
        and abs(left_wheel_aabb[0][2]) <= 0.002
        and abs(right_wheel_aabb[0][2]) <= 0.002
    )
    ctx.check(
        "front caster wheels meet the floor",
        front_wheels_grounded,
        details=f"left_wheel={left_wheel_aabb}, right_wheel={right_wheel_aabb}",
    )

    joint_config_ok = (
        left_swivel.axis == (0.0, 0.0, 1.0)
        and right_swivel.axis == (0.0, 0.0, 1.0)
        and left_spin.axis == (1.0, 0.0, 0.0)
        and right_spin.axis == (1.0, 0.0, 0.0)
        and left_spin.motion_limits is not None
        and right_spin.motion_limits is not None
        and left_spin.motion_limits.lower is None
        and left_spin.motion_limits.upper is None
        and right_spin.motion_limits.lower is None
        and right_spin.motion_limits.upper is None
    )
    ctx.check(
        "caster swivel and wheel spin joints use the intended axes",
        joint_config_ok,
        details=(
            f"left_swivel_axis={left_swivel.axis}, right_swivel_axis={right_swivel.axis}, "
            f"left_spin_axis={left_spin.axis}, right_spin_axis={right_spin.axis}"
        ),
    )

    def extent(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    left_tire_rest = ctx.part_element_world_aabb(left_wheel, elem="tire")
    right_tire_rest = ctx.part_element_world_aabb(right_wheel, elem="tire")
    with ctx.pose({left_swivel: 1.15, right_swivel: -1.15}):
        left_tire_swiveled = ctx.part_element_world_aabb(left_wheel, elem="tire")
        right_tire_swiveled = ctx.part_element_world_aabb(right_wheel, elem="tire")

    swivel_changes_orientation = (
        left_tire_rest is not None
        and right_tire_rest is not None
        and left_tire_swiveled is not None
        and right_tire_swiveled is not None
        and extent(left_tire_swiveled, 0) is not None
        and extent(left_tire_rest, 0) is not None
        and extent(right_tire_swiveled, 0) is not None
        and extent(right_tire_rest, 0) is not None
        and extent(left_tire_swiveled, 0) > extent(left_tire_rest, 0) + 0.08
        and extent(right_tire_swiveled, 1) < extent(right_tire_rest, 1) - 0.08
    )
    ctx.check(
        "caster swivels rotate the wheel orientation about the stem",
        swivel_changes_orientation,
        details=(
            f"left_rest={left_tire_rest}, left_swiveled={left_tire_swiveled}, "
            f"right_rest={right_tire_rest}, right_swiveled={right_tire_swiveled}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
