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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


TRAY_OUTER_RADIUS = 0.198
TRAY_OUTER_WALL_THICKNESS = 0.006
TRAY_CENTER_OPEN_RADIUS = 0.100
TRAY_INNER_WALL_THICKNESS = 0.006
TRAY_FLOOR_THICKNESS = 0.004
TRAY_WALL_HEIGHT = 0.036
TRAY_DIVIDER_HEIGHT = 0.032
TRAY_DIVIDER_THICKNESS = 0.005
TRAY_SEGMENTS = 6

HANDLE_PIVOT_RADIUS = 0.006
HANDLE_PIVOT_X = (
    TRAY_CENTER_OPEN_RADIUS + TRAY_INNER_WALL_THICKNESS + HANDLE_PIVOT_RADIUS
)
HANDLE_JOINT_Z = TRAY_FLOOR_THICKNESS + TRAY_WALL_HEIGHT
CUP_OUTER_RADIUS = 0.040


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    geometry = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        height,
        center=True,
    )
    geometry.translate(0.0, 0.0, height * 0.5)
    return mesh_from_geometry(geometry, name)


def _build_cup_shell(name: str):
    outer_profile = [
        (0.000, 0.000),
        (0.024, 0.003),
        (0.034, 0.010),
        (0.038, 0.030),
        (0.040, 0.047),
        (0.042, 0.050),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.020, 0.006),
        (0.030, 0.012),
        (0.035, 0.028),
        (0.0375, 0.046),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def _xy_radius_from_center(aabb) -> float | None:
    center = _aabb_center(aabb)
    if aabb is None or center is None:
        return None
    min_corner, max_corner = aabb
    return max(
        max_corner[0] - center[0],
        center[0] - min_corner[0],
        max_corner[1] - center[1],
        center[1] - min_corner[1],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="divided_snack_tray_lazy_susan")

    base_dark = model.material("base_dark", rgba=(0.17, 0.17, 0.18, 1.0))
    bearing_gray = model.material("bearing_gray", rgba=(0.43, 0.45, 0.48, 1.0))
    tray_cream = model.material("tray_cream", rgba=(0.86, 0.83, 0.77, 1.0))
    divider_tint = model.material("divider_tint", rgba=(0.80, 0.76, 0.70, 1.0))
    cup_cream = model.material("cup_cream", rgba=(0.96, 0.94, 0.89, 1.0))
    handle_taupe = model.material("handle_taupe", rgba=(0.60, 0.55, 0.49, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.170, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=base_dark,
        name="foot_disc",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=base_dark,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=bearing_gray,
        name="turntable_cap",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.052),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    tray = model.part("tray")
    tray.visual(
        _annulus_mesh(
            TRAY_OUTER_RADIUS - TRAY_OUTER_WALL_THICKNESS,
            TRAY_CENTER_OPEN_RADIUS + TRAY_INNER_WALL_THICKNESS,
            TRAY_FLOOR_THICKNESS,
            "tray_floor",
        ),
        material=tray_cream,
        name="tray_floor",
    )
    tray.visual(
        _annulus_mesh(
            TRAY_OUTER_RADIUS,
            TRAY_OUTER_RADIUS - TRAY_OUTER_WALL_THICKNESS,
            TRAY_WALL_HEIGHT,
            "outer_wall",
        ),
        origin=Origin(xyz=(0.0, 0.0, TRAY_FLOOR_THICKNESS)),
        material=tray_cream,
        name="outer_wall",
    )
    tray.visual(
        _annulus_mesh(
            TRAY_CENTER_OPEN_RADIUS + TRAY_INNER_WALL_THICKNESS,
            TRAY_CENTER_OPEN_RADIUS,
            TRAY_WALL_HEIGHT,
            "inner_wall",
        ),
        origin=Origin(xyz=(0.0, 0.0, TRAY_FLOOR_THICKNESS)),
        material=tray_cream,
        name="inner_wall",
    )
    tray.visual(
        Box((0.020, 0.018, 0.004)),
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, HANDLE_JOINT_Z - 0.002)),
        material=divider_tint,
        name="handle_pivot_pad",
    )
    divider_center_radius = (
        (
            TRAY_OUTER_RADIUS - TRAY_OUTER_WALL_THICKNESS
            + TRAY_CENTER_OPEN_RADIUS
            + TRAY_INNER_WALL_THICKNESS
        )
        * 0.5
    )
    divider_length = (
        TRAY_OUTER_RADIUS
        - TRAY_OUTER_WALL_THICKNESS
        - TRAY_CENTER_OPEN_RADIUS
        - TRAY_INNER_WALL_THICKNESS
    )
    for divider_index in range(TRAY_SEGMENTS):
        tray.visual(
            Box((divider_length, TRAY_DIVIDER_THICKNESS, TRAY_DIVIDER_HEIGHT)),
            origin=Origin(
                xyz=(
                    divider_center_radius * math.cos((2.0 * math.pi * divider_index) / TRAY_SEGMENTS),
                    divider_center_radius * math.sin((2.0 * math.pi * divider_index) / TRAY_SEGMENTS),
                    TRAY_FLOOR_THICKNESS + (TRAY_DIVIDER_HEIGHT * 0.5),
                ),
                rpy=(0.0, 0.0, (2.0 * math.pi * divider_index) / TRAY_SEGMENTS),
            ),
            material=divider_tint,
            name=f"divider_{divider_index}",
        )
    tray.inertial = Inertial.from_geometry(
        Cylinder(radius=TRAY_OUTER_RADIUS, length=TRAY_FLOOR_THICKNESS + TRAY_WALL_HEIGHT),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, (TRAY_FLOOR_THICKNESS + TRAY_WALL_HEIGHT) * 0.5)),
    )

    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0),
    )

    center_cup_handle = model.part("center_cup_handle")
    center_cup_handle.visual(
        Cylinder(radius=HANDLE_PIVOT_RADIUS, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=handle_taupe,
        name="pivot_barrel",
    )
    center_cup_handle.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=handle_taupe,
        name="pivot_collar",
    )
    center_cup_handle.visual(
        Cylinder(radius=0.006, length=0.072),
        origin=Origin(xyz=(-0.036, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_taupe,
        name="handle_arm",
    )
    center_cup_handle.visual(
        Cylinder(radius=0.0055, length=0.036),
        origin=Origin(xyz=(-0.072, 0.0, -0.002)),
        material=handle_taupe,
        name="hanger",
    )
    center_cup_handle.visual(
        Cylinder(radius=0.008, length=0.022),
        origin=Origin(xyz=(-0.083, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_taupe,
        name="cup_mount",
    )
    center_cup_handle.visual(
        _build_cup_shell("center_cup_shell"),
        origin=Origin(xyz=(-0.112, 0.0, -0.024)),
        material=cup_cream,
        name="center_cup_shell",
    )
    center_cup_handle.inertial = Inertial.from_geometry(
        Box((0.170, 0.090, 0.080)),
        mass=0.28,
        origin=Origin(xyz=(-0.085, 0.0, -0.004)),
    )

    model.articulation(
        "tray_to_center_cup_handle",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=center_cup_handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
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
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    center_cup_handle = object_model.get_part("center_cup_handle")
    tray_spin = object_model.get_articulation("base_to_tray")
    cup_pivot = object_model.get_articulation("tray_to_center_cup_handle")

    ctx.check(
        "tray spin joint is continuous around vertical axis",
        tray_spin.articulation_type == ArticulationType.CONTINUOUS
        and tray_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={tray_spin.articulation_type}, axis={tray_spin.axis}",
    )
    ctx.check(
        "center cup handle joint is revolute around local vertical axis",
        cup_pivot.articulation_type == ArticulationType.REVOLUTE
        and cup_pivot.axis == (0.0, 0.0, 1.0)
        and cup_pivot.motion_limits is not None
        and cup_pivot.motion_limits.lower is not None
        and cup_pivot.motion_limits.upper is not None
        and cup_pivot.motion_limits.lower < 0.0 < cup_pivot.motion_limits.upper,
        details=(
            f"type={cup_pivot.articulation_type}, axis={cup_pivot.axis}, "
            f"limits={cup_pivot.motion_limits}"
        ),
    )

    with ctx.pose({tray_spin: 0.0, cup_pivot: 0.0}):
        ctx.expect_contact(
            tray,
            base,
            elem_a="tray_floor",
            elem_b="turntable_cap",
            name="tray sits on turntable cap",
        )
        ctx.expect_overlap(
            tray,
            base,
            axes="xy",
            elem_a="tray_floor",
            elem_b="turntable_cap",
            min_overlap=0.18,
            name="tray floor overlaps the rotating base footprint",
        )
        ctx.expect_gap(
            center_cup_handle,
            tray,
            axis="z",
            positive_elem="center_cup_shell",
            negative_elem="tray_floor",
            min_gap=0.010,
            max_gap=0.030,
            name="center cup hangs above the tray floor",
        )

        cup_aabb = ctx.part_element_world_aabb(center_cup_handle, elem="center_cup_shell")
        cup_center = _aabb_center(cup_aabb)
        cup_radius = _xy_radius_from_center(cup_aabb)
        cup_centered = (
            cup_center is not None
            and cup_radius is not None
            and abs(cup_center[0]) + cup_radius <= TRAY_CENTER_OPEN_RADIUS
            and abs(cup_center[1]) + cup_radius <= TRAY_CENTER_OPEN_RADIUS
        )
        ctx.check(
            "center cup sits within the tray opening at rest",
            cup_centered,
            details=f"aabb={cup_aabb}, hole_radius={TRAY_CENTER_OPEN_RADIUS}",
        )

        divider_rest_aabb = ctx.part_element_world_aabb(tray, elem="divider_0")

    with ctx.pose({tray_spin: math.pi / 2.0, cup_pivot: 0.0}):
        divider_rotated_aabb = ctx.part_element_world_aabb(tray, elem="divider_0")

    divider_rest_center = _aabb_center(divider_rest_aabb)
    divider_rotated_center = _aabb_center(divider_rotated_aabb)
    tray_rotates = (
        divider_rest_center is not None
        and divider_rotated_center is not None
        and abs(divider_rotated_center[1]) > 0.10
        and abs(divider_rotated_center[0]) < 0.04
        and abs(
            math.hypot(divider_rest_center[0], divider_rest_center[1])
            - math.hypot(divider_rotated_center[0], divider_rotated_center[1])
        )
        <= 0.01
    )
    ctx.check(
        "tray rotation carries the snack dividers around the base axis",
        tray_rotates,
        details=f"rest={divider_rest_center}, rotated={divider_rotated_center}",
    )

    with ctx.pose({tray_spin: 0.0, cup_pivot: 0.45}):
        swung_cup_aabb = ctx.part_element_world_aabb(center_cup_handle, elem="center_cup_shell")
        swung_cup_center = _aabb_center(swung_cup_aabb)
        cup_swings = (
            swung_cup_center is not None
            and swung_cup_center[1] < -0.035
            and math.hypot(swung_cup_center[0], swung_cup_center[1]) < 0.070
        )
        ctx.check(
            "center cup handle swings the cup sideways while keeping it in the opening",
            cup_swings,
            details=f"aabb={swung_cup_aabb}, center={swung_cup_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
