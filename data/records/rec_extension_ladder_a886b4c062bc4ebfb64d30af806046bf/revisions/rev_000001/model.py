from __future__ import annotations

import math

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

BASE_LENGTH = 4.40
FLY_LENGTH = 4.00
BASE_WIDTH = 0.47
FLY_WIDTH = 0.41
BASE_RAIL_DEPTH = 0.028
BASE_RAIL_WIDTH = 0.067
FLY_RAIL_DEPTH = 0.024
FLY_RAIL_WIDTH = 0.058
RUNG_RADIUS = 0.015
RUNG_PITCH = 0.305
FLY_X_OFFSET = 0.056
FLY_REST_Z = 0.55
FLY_TRAVEL = 1.55


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_ladder_section(
    part,
    *,
    length: float,
    outer_width: float,
    rail_depth: float,
    rail_width: float,
    rung_radius: float,
    rung_pitch: float,
    rung_start: float,
    rung_end_margin: float,
    rail_material,
    rung_material,
    include_feet: bool = False,
    add_top_brackets: bool = False,
) -> None:
    rail_center_y = outer_width * 0.5 - rail_width * 0.5
    rung_length = outer_width - rail_width + 0.018

    for side_index, y_pos in enumerate((-rail_center_y, rail_center_y)):
        part.visual(
            Box((rail_depth, rail_width, length)),
            origin=Origin(xyz=(0.0, y_pos, length * 0.5)),
            material=rail_material,
            name=f"rail_{side_index}",
        )

    rung_count = max(1, int((length - rung_start - rung_end_margin) / rung_pitch) + 1)
    for rung_index in range(rung_count):
        z_pos = rung_start + rung_index * rung_pitch
        if z_pos > length - rung_end_margin:
            break
        part.visual(
            Cylinder(radius=rung_radius, length=rung_length),
            origin=Origin(xyz=(0.0, 0.0, z_pos), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=rung_material,
            name=f"rung_{rung_index}",
        )

    part.visual(
        Box((rail_depth + 0.004, outer_width - 0.030, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, length - 0.032)),
        material=rail_material,
        name="top_cap",
    )

    if include_feet:
        foot_z = 0.028
        for side_index, y_pos in enumerate((-rail_center_y, rail_center_y)):
            part.visual(
                Box((0.048, rail_width + 0.020, 0.056)),
                origin=Origin(xyz=(-0.004, y_pos, foot_z)),
                material="rubber_black",
                name=f"foot_{side_index}",
            )
        part.visual(
            Box((0.016, outer_width - 0.040, 0.020)),
            origin=Origin(xyz=(-0.010, 0.0, 0.060)),
            material=rail_material,
            name="foot_brace",
        )

    if add_top_brackets:
        bracket_y = rail_center_y - 0.010
        for side_index, y_pos in enumerate((-bracket_y, bracket_y)):
            part.visual(
                Box((0.020, 0.028, 0.090)),
                origin=Origin(xyz=(rail_depth * 0.5, y_pos, length + 0.015)),
                material=rail_material,
                name=f"hook_ear_{side_index}",
            )
        part.visual(
            Box((0.016, outer_width - 0.060, 0.020)),
            origin=Origin(xyz=(rail_depth * 0.45, 0.0, length - 0.100)),
            material=rail_material,
            name="hook_bridge",
        )
        for z_pos in (1.15, 2.45):
            for side_index, y_pos in enumerate((-bracket_y, bracket_y)):
                part.visual(
                    Box((0.060, 0.060, 0.090)),
                    origin=Origin(xyz=(-0.012, y_pos, z_pos)),
                    material=rail_material,
                    name=f"guide_pad_{side_index}_{int(z_pos * 100):03d}",
                )


def _add_roof_hook(part, *, arm_y: float, hook_material, roller_material) -> None:
    hinge_radius = 0.014
    arm_radius = 0.013
    brace_radius = 0.010

    part.visual(
        Cylinder(radius=hinge_radius, length=arm_y * 2.0 + 0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=hook_material,
        name="hinge_sleeve",
    )

    arch_points = [
        (0.0, 0.0, 0.0),
        (0.050, 0.0, 0.170),
        (0.150, 0.0, 0.290),
        (0.245, 0.0, 0.125),
        (0.170, 0.0, -0.045),
    ]
    arm_tip = arch_points[-1]

    for side_sign in (-1.0, 1.0):
        y_pos = side_sign * arm_y
        side_points = [(x, y_pos, z) for x, _, z in arch_points]
        for seg_index in range(len(side_points) - 1):
            _add_member(
                part,
                side_points[seg_index],
                side_points[seg_index + 1],
                arm_radius,
                hook_material,
                name=f"arm_{int(side_sign > 0)}_{seg_index}",
            )
        _add_member(
            part,
            (0.020, y_pos, 0.070),
            (0.108, y_pos, 0.078),
            brace_radius,
            hook_material,
        )

    _add_member(
        part,
        (0.050, -arm_y, 0.170),
        (0.050, arm_y, 0.170),
        brace_radius,
        hook_material,
        name="upper_spreader",
    )
    _add_member(
        part,
        (0.190, -arm_y, 0.020),
        (0.190, arm_y, 0.020),
        brace_radius,
        hook_material,
        name="lower_spreader",
    )

    roller_radius = 0.040
    roller_half_span = arm_y + 0.008
    roller_center = (arm_tip[0], 0.0, arm_tip[2] + 0.010)
    part.visual(
        Cylinder(radius=0.009, length=roller_half_span * 2.0),
        origin=Origin(xyz=roller_center, rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=hook_material,
        name="roller_axle",
    )
    for side_index, y_pos in enumerate((-arm_y - 0.012, arm_y + 0.012)):
        part.visual(
            Cylinder(radius=roller_radius, length=0.018),
            origin=Origin(
                xyz=(roller_center[0], y_pos, roller_center[2]),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=roller_material,
            name="hook_tip_roller" if side_index == 0 else f"roller_{side_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_hook_extension_ladder")

    model.material("aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("bright_aluminum", rgba=(0.86, 0.87, 0.89, 1.0))
    model.material("steel_dark", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("rubber_black", rgba=(0.10, 0.10, 0.10, 1.0))
    model.material("nylon_roller", rgba=(0.20, 0.23, 0.27, 1.0))

    base_section = model.part("base_section")
    _add_ladder_section(
        base_section,
        length=BASE_LENGTH,
        outer_width=BASE_WIDTH,
        rail_depth=BASE_RAIL_DEPTH,
        rail_width=BASE_RAIL_WIDTH,
        rung_radius=RUNG_RADIUS,
        rung_pitch=RUNG_PITCH,
        rung_start=0.370,
        rung_end_margin=0.320,
        rail_material="aluminum",
        rung_material="bright_aluminum",
        include_feet=True,
    )
    base_section.inertial = Inertial.from_geometry(
        Box((BASE_RAIL_DEPTH, BASE_WIDTH, BASE_LENGTH)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_LENGTH * 0.5)),
    )

    fly_section = model.part("fly_section")
    _add_ladder_section(
        fly_section,
        length=FLY_LENGTH,
        outer_width=FLY_WIDTH,
        rail_depth=FLY_RAIL_DEPTH,
        rail_width=FLY_RAIL_WIDTH,
        rung_radius=RUNG_RADIUS * 0.95,
        rung_pitch=RUNG_PITCH,
        rung_start=0.335,
        rung_end_margin=0.280,
        rail_material="bright_aluminum",
        rung_material="bright_aluminum",
        add_top_brackets=True,
    )
    fly_section.visual(
        Box((0.020, FLY_WIDTH - 0.045, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material="steel_dark",
        name="bottom_stop",
    )
    fly_section.inertial = Inertial.from_geometry(
        Box((FLY_RAIL_DEPTH, FLY_WIDTH, FLY_LENGTH)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, FLY_LENGTH * 0.5)),
    )

    roof_hook = model.part("roof_hook")
    _add_roof_hook(
        roof_hook,
        arm_y=0.155,
        hook_material="steel_dark",
        roller_material="nylon_roller",
    )
    roof_hook.inertial = Inertial.from_geometry(
        Box((0.32, 0.40, 0.40)),
        mass=3.5,
        origin=Origin(xyz=(0.13, 0.0, 0.10)),
    )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=fly_section,
        origin=Origin(xyz=(FLY_X_OFFSET, 0.0, FLY_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=0.0,
            upper=FLY_TRAVEL,
        ),
    )
    model.articulation(
        "hook_hinge",
        ArticulationType.REVOLUTE,
        parent=fly_section,
        child=roof_hook,
        origin=Origin(xyz=(0.036, 0.0, FLY_LENGTH + 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=-1.15,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_section = object_model.get_part("base_section")
    fly_section = object_model.get_part("fly_section")
    roof_hook = object_model.get_part("roof_hook")
    fly_slide = object_model.get_articulation("fly_slide")
    hook_hinge = object_model.get_articulation("hook_hinge")

    slide_upper = fly_slide.motion_limits.upper if fly_slide.motion_limits else None
    hook_lower = hook_hinge.motion_limits.lower if hook_hinge.motion_limits else None

    ctx.expect_overlap(
        base_section,
        fly_section,
        axes="y",
        min_overlap=0.34,
        name="fly section stays centered between ladder rails",
    )
    ctx.expect_overlap(
        base_section,
        fly_section,
        axes="z",
        min_overlap=3.70,
        name="collapsed ladder sections retain deep overlap",
    )

    rest_fly_aabb = ctx.part_world_aabb(fly_section)
    deployed_roller_aabb = ctx.part_element_world_aabb(roof_hook, elem="hook_tip_roller")

    if slide_upper is not None:
        with ctx.pose({fly_slide: slide_upper}):
            ctx.expect_overlap(
                base_section,
                fly_section,
                axes="y",
                min_overlap=0.34,
                name="extended fly remains laterally aligned with the base section",
            )
            ctx.expect_overlap(
                base_section,
                fly_section,
                axes="z",
                min_overlap=2.20,
                name="extended fly stays retained in the base section",
            )
            extended_fly_aabb = ctx.part_world_aabb(fly_section)
        ctx.check(
            "fly section extends upward",
            rest_fly_aabb is not None
            and extended_fly_aabb is not None
            and extended_fly_aabb[1][2] > rest_fly_aabb[1][2] + 1.40,
            details=f"rest={rest_fly_aabb}, extended={extended_fly_aabb}",
        )

    if hook_lower is not None:
        with ctx.pose({hook_hinge: hook_lower}):
            stowed_roller_aabb = ctx.part_element_world_aabb(roof_hook, elem="hook_tip_roller")
        ctx.check(
            "roof hook folds back toward the ladder",
            deployed_roller_aabb is not None
            and stowed_roller_aabb is not None
            and stowed_roller_aabb[1][0] < deployed_roller_aabb[1][0] - 0.045,
            details=f"deployed={deployed_roller_aabb}, stowed={stowed_roller_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
