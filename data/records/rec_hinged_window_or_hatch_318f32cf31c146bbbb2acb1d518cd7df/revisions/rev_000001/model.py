from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

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


HINGE_ALONG_Y = Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_access_hatch")

    curb_gray = model.material("curb_gray", rgba=(0.70, 0.72, 0.74, 1.0))
    lid_gray = model.material("lid_gray", rgba=(0.83, 0.84, 0.85, 1.0))
    hardware = model.material("hardware", rgba=(0.32, 0.34, 0.36, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    curb_frame = model.part("curb_frame")
    curb_frame.visual(
        Box((0.06, 0.90, 0.22)),
        origin=Origin(xyz=(0.47, 0.0, 0.11)),
        material=curb_gray,
        name="front_wall",
    )
    curb_frame.visual(
        Box((0.06, 0.90, 0.22)),
        origin=Origin(xyz=(-0.47, 0.0, 0.11)),
        material=curb_gray,
        name="rear_wall",
    )
    curb_frame.visual(
        Box((1.00, 0.06, 0.22)),
        origin=Origin(xyz=(0.0, 0.42, 0.11)),
        material=curb_gray,
        name="left_wall",
    )
    curb_frame.visual(
        Box((1.00, 0.06, 0.22)),
        origin=Origin(xyz=(0.0, -0.42, 0.11)),
        material=curb_gray,
        name="right_wall",
    )
    curb_frame.visual(
        Box((0.10, 1.02, 0.008)),
        origin=Origin(xyz=(0.55, 0.0, 0.004)),
        material=curb_gray,
        name="front_flange",
    )
    curb_frame.visual(
        Box((0.10, 1.02, 0.008)),
        origin=Origin(xyz=(-0.55, 0.0, 0.004)),
        material=curb_gray,
        name="rear_flange",
    )
    curb_frame.visual(
        Box((1.00, 0.10, 0.008)),
        origin=Origin(xyz=(0.0, 0.50, 0.004)),
        material=curb_gray,
        name="left_flange",
    )
    curb_frame.visual(
        Box((1.00, 0.10, 0.008)),
        origin=Origin(xyz=(0.0, -0.50, 0.004)),
        material=curb_gray,
        name="right_flange",
    )
    for side, y in (("left", 0.25), ("right", -0.25)):
        curb_frame.visual(
            Cylinder(radius=0.016, length=0.18),
            origin=Origin(xyz=(-0.50, y, 0.25), rpy=HINGE_ALONG_Y.rpy),
            material=hardware,
            name=f"{side}_frame_hinge_barrel",
        )
        curb_frame.visual(
            Box((0.05, 0.18, 0.04)),
            origin=Origin(xyz=(-0.525, y, 0.22)),
            material=hardware,
            name=f"{side}_frame_hinge_leaf",
        )
    curb_frame.visual(
        Box((0.04, 0.012, 0.06)),
        origin=Origin(xyz=(-0.40, 0.400, 0.17)),
        material=hardware,
        name="left_stay_bracket",
    )
    curb_frame.visual(
        Box((0.04, 0.012, 0.06)),
        origin=Origin(xyz=(-0.40, -0.400, 0.17)),
        material=hardware,
        name="right_stay_bracket",
    )
    curb_frame.inertial = Inertial.from_geometry(
        Box((1.10, 1.02, 0.26)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((1.04, 0.94, 0.008)),
        origin=Origin(xyz=(0.52, 0.0, 0.026)),
        material=lid_gray,
        name="lid_top_skin",
    )
    lid.visual(
        Box((0.78, 0.66, 0.018)),
        origin=Origin(xyz=(0.58, 0.0, 0.015)),
        material=lid_gray,
        name="lid_stiffener",
    )
    lid.visual(
        Box((0.02, 0.94, 0.055)),
        origin=Origin(xyz=(0.04, 0.0, -0.0025)),
        material=lid_gray,
        name="lid_rear_skirt",
    )
    lid.visual(
        Box((0.03, 0.91, 0.055)),
        origin=Origin(xyz=(1.025, 0.0, -0.002)),
        material=lid_gray,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((1.04, 0.03, 0.055)),
        origin=Origin(xyz=(0.52, 0.455, -0.002)),
        material=lid_gray,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((1.04, 0.03, 0.055)),
        origin=Origin(xyz=(0.52, -0.455, -0.002)),
        material=lid_gray,
        name="lid_right_skirt",
    )
    lid.visual(
        Cylinder(radius=0.015, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=HINGE_ALONG_Y.rpy),
        material=hardware,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.03, 0.30, 0.04)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=hardware,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.08, 0.14, 0.018)),
        origin=Origin(xyz=(0.30, 0.385, 0.008)),
        material=hardware,
        name="left_stay_pad",
    )
    lid.visual(
        Box((0.08, 0.14, 0.018)),
        origin=Origin(xyz=(0.30, -0.385, 0.008)),
        material=hardware,
        name="right_stay_pad",
    )
    lid.visual(
        Box((0.018, 0.022, 0.014)),
        origin=Origin(xyz=(0.86, 0.070, 0.022)),
        material=hardware,
        name="left_handle_mount",
    )
    lid.visual(
        Box((0.018, 0.022, 0.014)),
        origin=Origin(xyz=(0.86, -0.070, 0.022)),
        material=hardware,
        name="right_handle_mount",
    )
    lid.inertial = Inertial.from_geometry(
        Box((1.04, 0.94, 0.065)),
        mass=34.0,
        origin=Origin(xyz=(0.52, 0.0, 0.003)),
    )

    pull_handle = model.part("pull_handle")
    handle_loop = tube_from_spline_points(
        [
            (0.0, -0.070, 0.0),
            (0.028, -0.070, 0.0),
            (0.062, -0.042, 0.0),
            (0.078, 0.0, 0.0),
            (0.062, 0.042, 0.0),
            (0.028, 0.070, 0.0),
            (0.0, 0.070, 0.0),
        ],
        radius=0.0055,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )
    pull_handle.visual(
        _save_mesh("roof_hatch_handle_loop", handle_loop),
        material=hardware,
        name="handle_loop",
    )
    pull_handle.visual(
        Cylinder(radius=0.0065, length=0.08),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=HINGE_ALONG_Y.rpy),
        material=grip_black,
        name="handle_grip",
    )
    pull_handle.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=HINGE_ALONG_Y.rpy),
        material=hardware,
        name="right_pivot_boss",
    )
    pull_handle.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, -0.070, 0.0), rpy=HINGE_ALONG_Y.rpy),
        material=hardware,
        name="left_pivot_boss",
    )
    pull_handle.inertial = Inertial.from_geometry(
        Box((0.09, 0.16, 0.03)),
        mass=1.1,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    stay_rod = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.09, 0.0, 0.018),
            (0.18, 0.0, 0.045),
            (0.21, 0.0, 0.062),
        ],
        radius=0.007,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    for stay_name in ("left_stay", "right_stay"):
        stay = model.part(stay_name)
        stay.visual(
            _save_mesh(f"roof_hatch_{stay_name}_rod", stay_rod.copy()),
            material=hardware,
            name="stay_rod",
        )
        stay.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(rpy=HINGE_ALONG_Y.rpy),
            material=hardware,
            name="stay_pivot",
        )
        stay.visual(
            Box((0.030, 0.018, 0.010)),
            origin=Origin(xyz=(0.215, 0.0, 0.066)),
            material=hardware,
            name="stay_tip",
        )
        stay.inertial = Inertial.from_geometry(
            Box((0.25, 0.03, 0.08)),
            mass=1.8,
            origin=Origin(xyz=(0.12, 0.0, 0.032)),
        )

    model.articulation(
        "curb_to_lid",
        ArticulationType.REVOLUTE,
        parent=curb_frame,
        child=lid,
        origin=Origin(xyz=(-0.50, 0.0, 0.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=0.0,
            upper=radians(72.0),
        ),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=pull_handle,
        origin=Origin(xyz=(0.86, 0.0, 0.039)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=radians(85.0),
        ),
    )
    model.articulation(
        "curb_to_left_stay",
        ArticulationType.REVOLUTE,
        parent=curb_frame,
        child="left_stay",
        origin=Origin(xyz=(-0.40, 0.385, 0.17)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=radians(65.0),
        ),
    )
    model.articulation(
        "curb_to_right_stay",
        ArticulationType.REVOLUTE,
        parent=curb_frame,
        child="right_stay",
        origin=Origin(xyz=(-0.40, -0.385, 0.17)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=radians(65.0),
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

    curb_frame = object_model.get_part("curb_frame")
    lid = object_model.get_part("lid")
    pull_handle = object_model.get_part("pull_handle")
    left_stay = object_model.get_part("left_stay")
    right_stay = object_model.get_part("right_stay")

    lid_hinge = object_model.get_articulation("curb_to_lid")
    handle_pivot = object_model.get_articulation("lid_to_handle")
    left_stay_joint = object_model.get_articulation("curb_to_left_stay")
    right_stay_joint = object_model.get_articulation("curb_to_right_stay")

    with ctx.pose(
        {
            lid_hinge: 0.0,
            handle_pivot: 0.0,
            left_stay_joint: 0.0,
            right_stay_joint: 0.0,
        }
    ):
        ctx.expect_overlap(
            lid,
            curb_frame,
            axes="xy",
            min_overlap=0.85,
            name="lid covers the curb in plan",
        )
        ctx.expect_gap(
            lid,
            curb_frame,
            axis="z",
            positive_elem="lid_front_skirt",
            negative_elem="front_wall",
            min_gap=0.0005,
            max_gap=0.010,
            name="closed lid front skirt seats just above the curb",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    with ctx.pose({lid_hinge: radians(68.0)}):
        open_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    ctx.check(
        "lid front edge lifts when opened",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.45,
        details=f"closed={closed_front}, open={open_front}",
    )

    closed_grip = ctx.part_element_world_aabb(pull_handle, elem="handle_grip")
    with ctx.pose({handle_pivot: radians(75.0)}):
        open_grip = ctx.part_element_world_aabb(pull_handle, elem="handle_grip")
    ctx.check(
        "pull handle rotates upward",
        closed_grip is not None
        and open_grip is not None
        and open_grip[1][2] > closed_grip[1][2] + 0.05,
        details=f"closed={closed_grip}, open={open_grip}",
    )

    closed_left_tip = ctx.part_element_world_aabb(left_stay, elem="stay_tip")
    closed_right_tip = ctx.part_element_world_aabb(right_stay, elem="stay_tip")
    with ctx.pose(
        {
            lid_hinge: radians(55.0),
            left_stay_joint: radians(45.0),
            right_stay_joint: radians(45.0),
        }
    ):
        open_left_tip = ctx.part_element_world_aabb(left_stay, elem="stay_tip")
        open_right_tip = ctx.part_element_world_aabb(right_stay, elem="stay_tip")
    ctx.check(
        "left stay swings upward on its support joint",
        closed_left_tip is not None
        and open_left_tip is not None
        and open_left_tip[1][2] > closed_left_tip[1][2] + 0.08,
        details=f"closed={closed_left_tip}, open={open_left_tip}",
    )
    ctx.check(
        "right stay swings upward on its support joint",
        closed_right_tip is not None
        and open_right_tip is not None
        and open_right_tip[1][2] > closed_right_tip[1][2] + 0.08,
        details=f"closed={closed_right_tip}, open={open_right_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
