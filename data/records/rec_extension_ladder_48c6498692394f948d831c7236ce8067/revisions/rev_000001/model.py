from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_xz_bar(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    width: float,
    thickness: float,
    material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    pitch = atan2(-dz, dx)
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(
            xyz=(
                0.5 * (start[0] + end[0]),
                0.5 * (start[1] + end[1]),
                0.5 * (start[2] + end[2]),
            ),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="step_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    plastic = model.material("dark_plastic", rgba=(0.16, 0.16, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    front_frame = model.part("front_frame")
    front_frame.visual(
        Box((0.024, 0.045, 1.40)),
        origin=Origin(xyz=(0.0, -0.22, 0.70)),
        material=aluminum,
        name="left_stile",
    )
    front_frame.visual(
        Box((0.024, 0.045, 1.40)),
        origin=Origin(xyz=(0.0, 0.22, 0.70)),
        material=aluminum,
        name="right_stile",
    )
    front_frame.visual(
        Box((0.070, 0.395, 0.035)),
        origin=Origin(xyz=(0.020, 0.0, 1.3225)),
        material=aluminum,
        name="top_bridge",
    )

    step_levels = (
        ("lower_step", 0.29),
        ("mid_lower_step", 0.56),
        ("mid_upper_step", 0.83),
        ("upper_step", 1.10),
    )
    for step_name, step_z in step_levels:
        front_frame.visual(
            Box((0.100, 0.395, 0.030)),
            origin=Origin(xyz=(0.030, 0.0, step_z)),
            material=aluminum,
            name=step_name,
        )
        front_frame.visual(
            Box((0.094, 0.330, 0.004)),
            origin=Origin(xyz=(0.030, 0.0, step_z + 0.017)),
            material=rubber,
            name=f"{step_name}_tread",
        )

    front_frame.visual(
        Box((0.050, 0.060, 0.025)),
        origin=Origin(xyz=(0.0, -0.22, 0.0125)),
        material=rubber,
        name="left_front_foot",
    )
    front_frame.visual(
        Box((0.050, 0.060, 0.025)),
        origin=Origin(xyz=(0.0, 0.22, 0.0125)),
        material=rubber,
        name="right_front_foot",
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        Box((0.260, 0.460, 0.045)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0225)),
        material=plastic,
        name="cap_body",
    )
    top_cap.visual(
        Box((0.070, 0.400, 0.060)),
        origin=Origin(xyz=(-0.110, 0.0, 0.000)),
        material=plastic,
        name="cap_front_lip",
    )

    rear_brace = model.part("rear_brace")
    rear_brace.visual(
        Box((0.080, 0.090, 0.022)),
        origin=Origin(xyz=(0.030, 0.0, -0.011)),
        material=steel,
        name="rear_hinge_plate",
    )
    rear_brace.visual(
        Box((0.030, 0.065, 1.540)),
        origin=Origin(xyz=(0.300, 0.0, -0.720), rpy=(0.0, -0.40, 0.0)),
        material=aluminum,
        name="rear_leg",
    )
    rear_brace.visual(
        Box((0.022, 0.205, 0.012)),
        origin=Origin(xyz=(0.331, 0.1025, -0.780)),
        material=steel,
        name="rear_spreader_arm",
    )
    rear_brace.visual(
        Box((0.032, 0.030, 0.012)),
        origin=Origin(xyz=(0.331, 0.205, -0.780)),
        material=steel,
        name="brace_spreader_pad",
    )
    rear_brace.visual(
        Box((0.090, 0.100, 0.025)),
        origin=Origin(xyz=(0.600, 0.0, -1.4075)),
        material=rubber,
        name="rear_foot",
    )

    spreader_inner = model.part("spreader_inner")
    spreader_inner.visual(
        Box((0.024, 0.024, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=steel,
        name="front_spreader_mount",
    )
    _add_xz_bar(
        spreader_inner,
        start=(0.024, 0.0, 0.0),
        end=(0.258, 0.0, -0.055),
        width=0.016,
        thickness=0.008,
        material=steel,
        name="inner_spreader_leaf",
    )
    spreader_outer = model.part("spreader_outer")
    _add_xz_bar(
        spreader_outer,
        start=(0.000, 0.0, 0.0),
        end=(0.145, 0.0, -0.045),
        width=0.016,
        thickness=0.008,
        material=steel,
        name="outer_spreader_leaf",
    )
    spreader_outer.visual(
        Box((0.026, 0.022, 0.010)),
        origin=Origin(xyz=(0.132, 0.0, -0.045)),
        material=steel,
        name="outer_end_lug",
    )

    model.articulation(
        "front_to_top_cap",
        ArticulationType.FIXED,
        parent=front_frame,
        child=top_cap,
        origin=Origin(xyz=(0.10, 0.0, 1.40)),
    )
    model.articulation(
        "apex_hinge",
        ArticulationType.REVOLUTE,
        parent=top_cap,
        child=rear_brace,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-0.10,
            upper=0.38,
        ),
    )
    model.articulation(
        "front_to_spreader_inner",
        ArticulationType.FIXED,
        parent=front_frame,
        child=spreader_inner,
        origin=Origin(xyz=(0.012, 0.205, 0.72)),
    )
    model.articulation(
        "spreader_lock_hinge",
        ArticulationType.REVOLUTE,
        parent=spreader_inner,
        child=spreader_outer,
        origin=Origin(xyz=(0.258, 0.0, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=-1.15,
            upper=0.10,
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
    top_cap = object_model.get_part("top_cap")
    rear_brace = object_model.get_part("rear_brace")
    spreader_outer = object_model.get_part("spreader_outer")

    apex_hinge = object_model.get_articulation("apex_hinge")
    spreader_lock_hinge = object_model.get_articulation("spreader_lock_hinge")

    with ctx.pose({apex_hinge: 0.0, spreader_lock_hinge: 0.0}):
        ctx.expect_gap(
            top_cap,
            front_frame,
            axis="z",
            positive_elem="cap_body",
            negative_elem="right_stile",
            max_gap=0.002,
            max_penetration=0.0,
            name="top cap sits directly on the front stile tops",
        )
        ctx.expect_gap(
            rear_brace,
            front_frame,
            axis="x",
            positive_elem="rear_foot",
            negative_elem="right_stile",
            min_gap=0.30,
            name="rear brace opens behind the front section",
        )
        ctx.expect_contact(
            spreader_outer,
            rear_brace,
            elem_a="outer_end_lug",
            elem_b="brace_spreader_pad",
            contact_tol=0.004,
            name="spreader bar reaches the rear brace in the locked-open pose",
        )
        ctx.expect_overlap(
            spreader_outer,
            rear_brace,
            axes="yz",
            elem_a="outer_end_lug",
            elem_b="brace_spreader_pad",
            min_overlap=0.010,
            name="spreader lug stays aligned with the rear brace pad",
        )

    with ctx.pose({apex_hinge: 0.0}):
        open_foot = ctx.part_element_world_aabb(rear_brace, elem="rear_foot")
    with ctx.pose({apex_hinge: 0.34}):
        folded_foot = ctx.part_element_world_aabb(rear_brace, elem="rear_foot")
    ctx.check(
        "rear brace folds forward around the apex hinge",
        open_foot is not None
        and folded_foot is not None
        and folded_foot[0][0] < open_foot[0][0] - 0.22,
        details=f"open_foot={open_foot}, folded_foot={folded_foot}",
    )

    with ctx.pose({spreader_lock_hinge: 0.0}):
        open_lug = ctx.part_element_world_aabb(spreader_outer, elem="outer_end_lug")
    with ctx.pose({spreader_lock_hinge: -1.0}):
        folded_lug = ctx.part_element_world_aabb(spreader_outer, elem="outer_end_lug")
    ctx.check(
        "spreader leaf folds upward from the lock position",
        open_lug is not None
        and folded_lug is not None
        and folded_lug[1][2] > open_lug[1][2] + 0.08
        and folded_lug[1][0] < open_lug[1][0] - 0.02,
        details=f"open_lug={open_lug}, folded_lug={folded_lug}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
