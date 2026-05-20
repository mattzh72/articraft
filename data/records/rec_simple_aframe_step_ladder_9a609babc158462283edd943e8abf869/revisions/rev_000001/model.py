from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box_min_z(aabb):
    if aabb is None:
        return None
    return aabb[0][2]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    plastic = model.material("plastic", rgba=(0.18, 0.18, 0.19, 1.0))
    tread = model.material("tread", rgba=(0.68, 0.70, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.52, 0.34, 1.08)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.10, 0.54)),
    )

    front_rail_size = (0.040, 0.018, 1.06)
    for x_pos, visual_name in [(-0.205, "front_left_rail"), (0.205, "front_right_rail")]:
        front_frame.visual(
            Box(front_rail_size),
            origin=Origin(xyz=(x_pos, 0.105, 0.518), rpy=(0.20, 0.0, 0.0)),
            material=aluminum,
            name=visual_name,
        )

    front_frame.visual(
        Box((0.42, 0.19, 0.050)),
        origin=Origin(xyz=(0.0, 0.025, 0.995)),
        material=plastic,
        name="front_top_cap",
    )
    front_frame.visual(
        Box((0.30, 0.080, 0.018)),
        origin=Origin(xyz=(0.0, 0.030, 1.029)),
        material=plastic,
        name="top_handle_ridge",
    )

    step_positions = [
        (0.154, 0.180, "step_1"),
        (0.112, 0.390, "step_2"),
        (0.070, 0.605, "step_3"),
        (0.030, 0.812, "step_4"),
    ]
    for y_pos, z_pos, visual_name in step_positions:
        front_frame.visual(
            Box((0.378, 0.090, 0.028)),
            origin=Origin(xyz=(0.0, y_pos, z_pos)),
            material=tread,
            name=visual_name,
        )
        front_frame.visual(
            Box((0.340, 0.020, 0.024)),
            origin=Origin(xyz=(0.0, y_pos - 0.050, z_pos - 0.026)),
            material=aluminum,
            name=f"{visual_name}_brace",
        )

    front_frame.visual(
        Box((0.392, 0.024, 0.042)),
        origin=Origin(xyz=(0.0, 0.176, 0.092)),
        material=aluminum,
        name="lower_spreader",
    )

    for x_pos, visual_name in [(-0.205, "front_left_foot"), (0.205, "front_right_foot")]:
        front_frame.visual(
            Box((0.052, 0.050, 0.032)),
            origin=Origin(xyz=(x_pos, 0.208, 0.016)),
            material=rubber,
            name=visual_name,
        )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.42, 1.04)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.17, -0.51)),
    )

    rear_rail_size = (0.034, 0.016, 1.02)
    for x_pos, visual_name in [(-0.188, "rear_left_rail"), (0.188, "rear_right_rail")]:
        rear_frame.visual(
            Box(rear_rail_size),
            origin=Origin(xyz=(x_pos, -0.245, -0.515), rpy=(-0.33, 0.0, 0.0)),
            material=aluminum,
            name=visual_name,
        )

    rear_frame.visual(
        Box((0.388, 0.032, 0.080)),
        origin=Origin(xyz=(0.0, -0.085, -0.100)),
        material=aluminum,
        name="rear_top_yoke",
    )

    rear_frame.visual(
        Box((0.392, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.273, -0.602)),
        material=aluminum,
        name="rear_mid_brace",
    )
    rear_frame.visual(
        Box((0.364, 0.048, 0.040)),
        origin=Origin(xyz=(0.0, -0.358, -0.864)),
        material=aluminum,
        name="rear_lower_brace",
    )

    rear_frame.visual(
        Box((0.074, 0.018, 0.620)),
        origin=Origin(xyz=(0.151, -0.205, -0.360), rpy=(-0.23, 0.0, 0.0)),
        material=aluminum,
        name="offset_side_support",
    )

    for x_pos, visual_name in [(-0.188, "rear_left_foot"), (0.188, "rear_right_foot")]:
        rear_frame.visual(
            Box((0.052, 0.060, 0.040)),
            origin=Origin(xyz=(x_pos, -0.406, -1.010)),
            material=rubber,
            name=visual_name,
        )

    model.articulation(
        "rear_fold",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=-0.12,
            upper=0.50,
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

    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    rear_fold = object_model.get_articulation("rear_fold")

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="y",
        positive_elem="front_left_foot",
        negative_elem="rear_left_foot",
        min_gap=0.45,
        max_gap=0.62,
        name="open stance keeps rear foot well behind front foot",
    )
    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="z",
        positive_elem="front_top_cap",
        negative_elem="rear_top_yoke",
        max_gap=0.012,
        max_penetration=0.002,
        name="rear yoke sits just below the front top cap at the hinge",
    )

    front_left_foot = ctx.part_element_world_aabb(front_frame, elem="front_left_foot")
    front_right_foot = ctx.part_element_world_aabb(front_frame, elem="front_right_foot")
    rear_left_foot = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
    rear_right_foot = ctx.part_element_world_aabb(rear_frame, elem="rear_right_foot")
    ground_clearances = [
        _box_min_z(front_left_foot),
        _box_min_z(front_right_foot),
        _box_min_z(rear_left_foot),
        _box_min_z(rear_right_foot),
    ]
    ctx.check(
        "all four feet rest on the ground in the open pose",
        all(value is not None and abs(value) <= 0.015 for value in ground_clearances),
        details=f"foot min z values={ground_clearances}",
    )

    open_rear_foot = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
    with ctx.pose({rear_fold: 0.50}):
        folded_rear_foot = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
    ctx.check(
        "rear frame folds toward the front frame",
        open_rear_foot is not None
        and folded_rear_foot is not None
        and folded_rear_foot[0][1] > open_rear_foot[0][1] + 0.45,
        details=f"open={open_rear_foot}, folded={folded_rear_foot}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
