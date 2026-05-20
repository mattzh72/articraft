from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="platform_wheelchair_lift")

    powder_coat = model.material("powder_coat", rgba=(0.27, 0.29, 0.31, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.74, 0.12, 1.0))

    column = model.part("column_assembly")
    column.visual(
        Box((0.66, 0.56, 0.025)),
        origin=Origin(xyz=(0.0, 0.02, 0.0125)),
        material=powder_coat,
        name="base_plate",
    )
    column.visual(
        Box((0.30, 0.26, 0.24)),
        origin=Origin(xyz=(0.0, -0.03, 0.12)),
        material=powder_coat,
        name="lower_machine_housing",
    )
    column.visual(
        Box((0.22, 0.14, 0.50)),
        origin=Origin(xyz=(0.0, -0.12, 0.45)),
        material=powder_coat,
        name="rear_service_box",
    )
    column.visual(
        Box((0.14, 0.14, 1.72)),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=powder_coat,
        name="mast",
    )
    column.visual(
        Box((0.20, 0.06, 1.06)),
        origin=Origin(xyz=(0.0, 0.10, 0.86)),
        material=powder_coat,
        name="track_cover",
    )
    column.visual(
        Box((0.22, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.01, 1.77)),
        material=powder_coat,
        name="top_head",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.66, 0.56, 1.82)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.02, 0.91)),
    )

    platform = model.part("platform")
    platform.visual(
        Box((0.26, 0.05, 0.56)),
        origin=Origin(xyz=(0.0, -0.025, 0.23)),
        material=powder_coat,
        name="carriage_plate",
    )
    platform.visual(
        Box((0.30, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.01, 0.50)),
        material=powder_coat,
        name="carriage_cap",
    )
    for side, x_pos in (("left", -0.10), ("right", 0.10)):
        platform.visual(
            Box((0.06, 0.09, 0.18)),
            origin=Origin(xyz=(x_pos, -0.015, 0.12)),
            material=powder_coat,
            name=f"{side}_lower_guide",
        )
        platform.visual(
            Box((0.06, 0.09, 0.18)),
            origin=Origin(xyz=(x_pos, -0.015, 0.38)),
            material=powder_coat,
            name=f"{side}_upper_guide",
        )
        platform.visual(
            Box((0.12, 0.40, 0.045)),
            origin=Origin(xyz=(0.18 if side == "right" else -0.18, 0.20, -0.0525)),
            material=powder_coat,
            name=f"{side}_support_arm",
        )
    platform.visual(
        Box((0.50, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.03, -0.045)),
        material=powder_coat,
        name="rear_crossmember",
    )
    platform.visual(
        Box((0.50, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.37, -0.055)),
        material=powder_coat,
        name="front_crossmember",
    )
    platform.visual(
        Box((0.82, 0.80, 0.03)),
        origin=Origin(xyz=(0.0, 0.40, -0.015)),
        material=aluminum,
        name="deck",
    )
    platform.visual(
        Box((0.82, 0.04, 0.09)),
        origin=Origin(xyz=(0.0, 0.02, 0.015)),
        material=powder_coat,
        name="rear_sill",
    )
    platform.visual(
        Box((0.04, 0.80, 0.09)),
        origin=Origin(xyz=(-0.39, 0.40, 0.015)),
        material=powder_coat,
        name="left_side_curb",
    )
    platform.visual(
        Box((0.04, 0.80, 0.09)),
        origin=Origin(xyz=(0.39, 0.40, 0.015)),
        material=powder_coat,
        name="right_side_curb",
    )
    for index, x_pos in enumerate((-0.27, 0.0, 0.27)):
        platform.visual(
            Box((0.12, 0.04, 0.04)),
            origin=Origin(xyz=(x_pos, 0.78, -0.005)),
            material=powder_coat,
            name=f"hinge_mount_{index}",
        )
    platform.inertial = Inertial.from_geometry(
        Box((0.82, 0.86, 0.62)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.39, 0.18)),
    )

    knee_guard = model.part("knee_guard")
    knee_guard.visual(
        Cylinder(radius=0.012, length=0.70),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=safety_yellow,
        name="hinge_tube",
    )
    knee_guard.visual(
        Box((0.76, 0.026, 0.29)),
        origin=Origin(xyz=(0.0, 0.022, 0.145)),
        material=safety_yellow,
        name="panel_skin",
    )
    knee_guard.visual(
        Box((0.76, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.02, 0.31)),
        material=safety_yellow,
        name="top_rail",
    )
    knee_guard.visual(
        Box((0.04, 0.03, 0.22)),
        origin=Origin(xyz=(-0.36, 0.018, 0.15)),
        material=safety_yellow,
        name="left_end_stile",
    )
    knee_guard.visual(
        Box((0.04, 0.03, 0.22)),
        origin=Origin(xyz=(0.36, 0.018, 0.15)),
        material=safety_yellow,
        name="right_end_stile",
    )
    knee_guard.inertial = Inertial.from_geometry(
        Box((0.76, 0.04, 0.33)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.02, 0.165)),
    )

    model.articulation(
        "column_to_platform",
        ArticulationType.PRISMATIC,
        parent=column,
        child=platform,
        origin=Origin(xyz=(0.0, 0.19, 0.11)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4500.0,
            velocity=0.08,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "platform_to_knee_guard",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=knee_guard,
        origin=Origin(xyz=(0.0, 0.80, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
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

    column = object_model.get_part("column_assembly")
    platform = object_model.get_part("platform")
    knee_guard = object_model.get_part("knee_guard")
    lift = object_model.get_articulation("column_to_platform")
    guard_hinge = object_model.get_articulation("platform_to_knee_guard")

    ctx.check(
        "wheelchair lift parts exist",
        all(part is not None for part in (column, platform, knee_guard)),
        details="Expected column assembly, moving platform, and knee guard parts.",
    )

    lift_upper = lift.motion_limits.upper if lift.motion_limits is not None else None
    guard_upper = guard_hinge.motion_limits.upper if guard_hinge.motion_limits is not None else None

    with ctx.pose({lift: 0.0, guard_hinge: 0.0}):
        ctx.expect_origin_distance(
            platform,
            column,
            axes="x",
            max_dist=0.001,
            name="platform stays centered on the column",
        )
        ctx.expect_gap(
            platform,
            column,
            axis="y",
            positive_elem="carriage_plate",
            negative_elem="track_cover",
            min_gap=0.01,
            max_gap=0.03,
            name="carriage plate clears the column track cover",
        )
        ctx.expect_overlap(
            knee_guard,
            platform,
            axes="x",
            elem_a="panel_skin",
            elem_b="deck",
            min_overlap=0.72,
            name="knee guard spans most of the platform width",
        )
        rest_platform_pos = ctx.part_world_position(platform)
        closed_panel_aabb = ctx.part_element_world_aabb(knee_guard, elem="panel_skin")

    with ctx.pose({lift: lift_upper if lift_upper is not None else 0.0, guard_hinge: 0.0}):
        raised_platform_pos = ctx.part_world_position(platform)
        ctx.expect_gap(
            platform,
            column,
            axis="y",
            positive_elem="carriage_plate",
            negative_elem="track_cover",
            min_gap=0.01,
            max_gap=0.03,
            name="carriage plate clears the track cover while raised",
        )

    ctx.check(
        "platform slides upward along the mast",
        rest_platform_pos is not None
        and raised_platform_pos is not None
        and raised_platform_pos[2] > rest_platform_pos[2] + 0.70,
        details=f"rest={rest_platform_pos}, raised={raised_platform_pos}",
    )

    with ctx.pose({guard_hinge: guard_upper if guard_upper is not None else 0.0}):
        open_panel_aabb = ctx.part_element_world_aabb(knee_guard, elem="panel_skin")

    ctx.check(
        "knee guard folds outward and downward from the platform edge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] < closed_panel_aabb[1][2] - 0.20
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.18,
        details=f"closed_panel={closed_panel_aabb}, open_panel={open_panel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
