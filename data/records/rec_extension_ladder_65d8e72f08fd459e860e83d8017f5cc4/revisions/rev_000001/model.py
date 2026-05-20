from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="work_platform_ladder")

    aluminum = model.material("aluminum", color=(0.76, 0.77, 0.79))
    platform_gray = model.material("platform_gray", color=(0.24, 0.25, 0.27))
    safety_yellow = model.material("safety_yellow", color=(0.94, 0.78, 0.12))

    header_size = (0.10, 0.58, 0.07)
    stile_size = (0.07, 0.05, 1.55)
    rung_size = (0.04, 0.48, 0.028)
    platform_size = (0.31, 0.44, 0.022)
    guard_post_size = (0.024, 0.024, 0.75)
    guard_top_size = (0.204, 0.024, 0.024)

    stile_y = 0.265
    rung_offsets = (-0.29, -0.59, -0.89, -1.19)

    header = model.part("top_header")
    header.visual(
        Box(header_size),
        material=aluminum,
        name="header_body",
    )

    left_stile = model.part("left_stile")
    left_stile.visual(
        Box(stile_size),
        origin=Origin(xyz=(0.0, 0.0, -stile_size[2] / 2.0)),
        material=aluminum,
        name="stile_body",
    )

    right_stile = model.part("right_stile")
    right_stile.visual(
        Box(stile_size),
        origin=Origin(xyz=(0.0, 0.0, -stile_size[2] / 2.0)),
        material=aluminum,
        name="stile_body",
    )

    model.articulation(
        "header_to_left_stile",
        ArticulationType.FIXED,
        parent=header,
        child=left_stile,
        origin=Origin(xyz=(0.0, -stile_y, -header_size[2] / 2.0)),
    )
    model.articulation(
        "header_to_right_stile",
        ArticulationType.FIXED,
        parent=header,
        child=right_stile,
        origin=Origin(xyz=(0.0, stile_y, -header_size[2] / 2.0)),
    )

    rung_parts = []
    for index, rung_z in enumerate(rung_offsets, start=1):
        rung = model.part(f"rung_{index}")
        rung.visual(
            Box(rung_size),
            origin=Origin(xyz=(0.0, rung_size[1] / 2.0, 0.0)),
            material=aluminum,
            name="rung_body",
        )
        model.articulation(
            f"left_stile_to_rung_{index}",
            ArticulationType.FIXED,
            parent=left_stile,
            child=rung,
            origin=Origin(xyz=(0.0, stile_size[1] / 2.0, rung_z)),
        )
        rung_parts.append(rung)

    platform = model.part("platform")
    platform.visual(
        Box(platform_size),
        origin=Origin(
            xyz=(-platform_size[0] / 2.0, 0.0, platform_size[2] / 2.0),
        ),
        material=platform_gray,
        name="platform_panel",
    )
    model.articulation(
        "header_to_platform",
        ArticulationType.REVOLUTE,
        parent=header,
        child=platform,
        origin=Origin(xyz=(0.03, 0.0, header_size[2] / 2.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )

    left_guard = model.part("left_guard_rail")
    left_guard.visual(
        Box(guard_post_size),
        origin=Origin(xyz=(-0.09, 0.0, 0.359)),
        material=safety_yellow,
        name="rear_post",
    )
    left_guard.visual(
        Box(guard_post_size),
        origin=Origin(xyz=(0.09, 0.0, 0.359)),
        material=safety_yellow,
        name="front_post",
    )
    left_guard.visual(
        Box(guard_top_size),
        origin=Origin(xyz=(0.0, 0.0, 0.746)),
        material=safety_yellow,
        name="top_bar",
    )
    model.articulation(
        "platform_to_left_guard",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=left_guard,
        origin=Origin(xyz=(-0.16, -0.232, 0.038)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )

    right_guard = model.part("right_guard_rail")
    right_guard.visual(
        Box(guard_post_size),
        origin=Origin(xyz=(-0.09, 0.0, 0.359)),
        material=safety_yellow,
        name="rear_post",
    )
    right_guard.visual(
        Box(guard_post_size),
        origin=Origin(xyz=(0.09, 0.0, 0.359)),
        material=safety_yellow,
        name="front_post",
    )
    right_guard.visual(
        Box(guard_top_size),
        origin=Origin(xyz=(0.0, 0.0, 0.746)),
        material=safety_yellow,
        name="top_bar",
    )
    model.articulation(
        "platform_to_right_guard",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=right_guard,
        origin=Origin(xyz=(-0.16, 0.232, 0.038)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
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

    header = object_model.get_part("top_header")
    left_stile = object_model.get_part("left_stile")
    right_stile = object_model.get_part("right_stile")
    platform = object_model.get_part("platform")
    left_guard = object_model.get_part("left_guard_rail")
    right_guard = object_model.get_part("right_guard_rail")
    middle_rung = object_model.get_part("rung_2")

    platform_hinge = object_model.get_articulation("header_to_platform")
    left_guard_hinge = object_model.get_articulation("platform_to_left_guard")
    right_guard_hinge = object_model.get_articulation("platform_to_right_guard")

    ctx.expect_contact(
        left_stile,
        header,
        name="left stile mounts to top header",
    )
    ctx.expect_contact(
        right_stile,
        header,
        name="right stile mounts to top header",
    )
    ctx.expect_contact(
        middle_rung,
        left_stile,
        name="mid rung bears on left stile",
    )
    ctx.expect_contact(
        middle_rung,
        right_stile,
        name="mid rung bears on right stile",
    )

    with ctx.pose({platform_hinge: 0.0}):
        ctx.expect_contact(
            platform,
            header,
            name="platform seats on top header when deployed",
        )
        ctx.expect_contact(
            left_guard,
            platform,
            name="left guard rail is mounted on platform",
        )
        ctx.expect_contact(
            right_guard,
            platform,
            name="right guard rail is mounted on platform",
        )

    platform_rest = ctx.part_element_world_aabb(platform, elem="platform_panel")
    with ctx.pose({platform_hinge: 1.35}):
        platform_folded = ctx.part_element_world_aabb(platform, elem="platform_panel")
    ctx.check(
        "platform folds upward from the ladder top",
        platform_rest is not None
        and platform_folded is not None
        and platform_folded[1][2] > platform_rest[1][2] + 0.22,
        details=f"rest={platform_rest}, folded={platform_folded}",
    )

    left_guard_rest = ctx.part_world_aabb(left_guard)
    right_guard_rest = ctx.part_world_aabb(right_guard)
    with ctx.pose({left_guard_hinge: 1.35, right_guard_hinge: 1.35}):
        left_guard_folded = ctx.part_world_aabb(left_guard)
        right_guard_folded = ctx.part_world_aabb(right_guard)
    ctx.check(
        "left guard rail folds down from its upright position",
        left_guard_rest is not None
        and left_guard_folded is not None
        and left_guard_folded[1][2] < left_guard_rest[1][2] - 0.45,
        details=f"rest={left_guard_rest}, folded={left_guard_folded}",
    )
    ctx.check(
        "right guard rail folds down from its upright position",
        right_guard_rest is not None
        and right_guard_folded is not None
        and right_guard_folded[1][2] < right_guard_rest[1][2] - 0.45,
        details=f"rest={right_guard_rest}, folded={right_guard_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
