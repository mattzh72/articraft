from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="theater_recliner")

    steel = model.material("powder_coated_steel", rgba=(0.14, 0.14, 0.16, 1.0))
    shell = model.material("vinyl_shell", rgba=(0.24, 0.21, 0.19, 1.0))
    cushion = model.material("seat_cushion", rgba=(0.36, 0.30, 0.27, 1.0))

    floor_frame = model.part("floor_frame")
    floor_frame.visual(
        Box((0.70, 0.08, 0.04)),
        origin=Origin(xyz=(-0.02, -0.32, 0.02)),
        material=steel,
        name="left_runner",
    )
    floor_frame.visual(
        Box((0.70, 0.08, 0.04)),
        origin=Origin(xyz=(-0.02, 0.32, 0.02)),
        material=steel,
        name="right_runner",
    )
    floor_frame.visual(
        Box((0.08, 0.56, 0.04)),
        origin=Origin(xyz=(-0.33, 0.0, 0.02)),
        material=steel,
        name="rear_crossmember",
    )
    floor_frame.visual(
        Box((0.08, 0.56, 0.04)),
        origin=Origin(xyz=(0.29, 0.0, 0.02)),
        material=steel,
        name="front_crossmember",
    )
    floor_frame.visual(
        Box((0.18, 0.08, 0.22)),
        origin=Origin(xyz=(-0.05, -0.32, 0.15)),
        material=steel,
        name="left_pedestal",
    )
    floor_frame.visual(
        Box((0.18, 0.08, 0.22)),
        origin=Origin(xyz=(-0.05, 0.32, 0.15)),
        material=steel,
        name="right_pedestal",
    )
    floor_frame.visual(
        Box((0.28, 0.64, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=steel,
        name="top_saddle",
    )

    seat_base = model.part("seat_base")
    seat_base.visual(
        Box((0.76, 0.86, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=shell,
        name="seat_shell",
    )
    seat_base.visual(
        Box((0.62, 0.14, 0.14)),
        origin=Origin(xyz=(-0.01, -0.36, 0.23)),
        material=shell,
        name="left_armrest",
    )
    seat_base.visual(
        Box((0.62, 0.14, 0.14)),
        origin=Origin(xyz=(-0.01, 0.36, 0.23)),
        material=shell,
        name="right_armrest",
    )
    seat_base.visual(
        Box((0.52, 0.54, 0.09)),
        origin=Origin(xyz=(0.03, 0.0, 0.205)),
        material=cushion,
        name="seat_cushion",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.10, 0.68, 0.10)),
        origin=Origin(xyz=(-0.05, 0.0, 0.05)),
        material=shell,
        name="back_lower_frame",
    )
    backrest.visual(
        Cylinder(radius=0.03, length=0.06),
        origin=Origin(xyz=(-0.03, -0.30, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="left_back_hinge_barrel",
    )
    backrest.visual(
        Cylinder(radius=0.03, length=0.06),
        origin=Origin(xyz=(-0.03, 0.30, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="right_back_hinge_barrel",
    )
    backrest.visual(
        Box((0.12, 0.72, 0.58)),
        origin=Origin(xyz=(-0.06, 0.0, 0.39)),
        material=cushion,
        name="back_panel",
    )
    backrest.visual(
        Box((0.09, 0.66, 0.10)),
        origin=Origin(xyz=(-0.08, 0.0, 0.73)),
        material=cushion,
        name="head_pad",
    )

    footrest = model.part("footrest")
    footrest.visual(
        Box((0.07, 0.64, 0.06)),
        origin=Origin(xyz=(0.035, 0.0, -0.03)),
        material=shell,
        name="hinge_rail",
    )
    footrest.visual(
        Cylinder(radius=0.025, length=0.05),
        origin=Origin(xyz=(0.025, -0.30, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="left_foot_hinge_barrel",
    )
    footrest.visual(
        Cylinder(radius=0.025, length=0.05),
        origin=Origin(xyz=(0.025, 0.30, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="right_foot_hinge_barrel",
    )
    footrest.visual(
        Box((0.07, 0.60, 0.30)),
        origin=Origin(xyz=(0.035, 0.0, -0.21)),
        material=cushion,
        name="foot_panel",
    )
    footrest.visual(
        Box((0.09, 0.52, 0.08)),
        origin=Origin(xyz=(0.045, 0.0, -0.39)),
        material=cushion,
        name="calf_pad",
    )

    model.articulation(
        "floor_to_seat",
        ArticulationType.FIXED,
        parent=floor_frame,
        child=seat_base,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_base,
        child=backrest,
        origin=Origin(xyz=(-0.38, 0.0, 0.16)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=0.55,
        ),
    )
    model.articulation(
        "seat_to_footrest",
        ArticulationType.REVOLUTE,
        parent=seat_base,
        child=footrest,
        origin=Origin(xyz=(0.38, 0.0, 0.16)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.5,
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

    floor_frame = object_model.get_part("floor_frame")
    seat_base = object_model.get_part("seat_base")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    back_hinge = object_model.get_articulation("seat_to_backrest")
    foot_hinge = object_model.get_articulation("seat_to_footrest")

    ctx.check(
        "backrest hinge uses a transverse axis",
        tuple(back_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={back_hinge.axis}",
    )
    ctx.check(
        "footrest hinge uses a transverse axis",
        tuple(foot_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={foot_hinge.axis}",
    )

    with ctx.pose({back_hinge: 0.0, foot_hinge: 0.0}):
        ctx.expect_gap(
            seat_base,
            floor_frame,
            axis="z",
            positive_elem="seat_shell",
            negative_elem="top_saddle",
            max_gap=0.001,
            max_penetration=0.000001,
            name="seat base sits on the floor frame saddle",
        )
        ctx.expect_overlap(
            seat_base,
            floor_frame,
            axes="xy",
            elem_a="seat_shell",
            elem_b="top_saddle",
            min_overlap=0.20,
            name="seat base is centered over the floor frame",
        )
        ctx.expect_gap(
            seat_base,
            backrest,
            axis="x",
            positive_elem="seat_shell",
            negative_elem="back_panel",
            min_gap=0.0,
            max_gap=0.02,
            name="backrest hinge sits at the rear edge of the seat",
        )
        ctx.expect_gap(
            footrest,
            seat_base,
            axis="x",
            positive_elem="hinge_rail",
            negative_elem="seat_shell",
            min_gap=0.0,
            max_gap=0.02,
            name="footrest hinge sits at the front edge of the seat",
        )
        ctx.expect_overlap(
            backrest,
            seat_base,
            axes="y",
            elem_a="back_panel",
            elem_b="seat_shell",
            min_overlap=0.70,
            name="backrest spans nearly the full seat width",
        )
        ctx.expect_overlap(
            footrest,
            seat_base,
            axes="y",
            elem_a="foot_panel",
            elem_b="seat_shell",
            min_overlap=0.55,
            name="footrest board aligns with the seat width",
        )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    back_rest_aabb = ctx.part_element_world_aabb(backrest, elem="back_panel")
    with ctx.pose({back_hinge: back_hinge.motion_limits.upper}):
        back_reclined_aabb = ctx.part_element_world_aabb(backrest, elem="back_panel")

    back_rest_center = _aabb_center(back_rest_aabb)
    back_reclined_center = _aabb_center(back_reclined_aabb)
    ctx.check(
        "backrest reclines backward from the seat rear hinge",
        back_rest_center is not None
        and back_reclined_center is not None
        and back_reclined_center[0] < back_rest_center[0] - 0.08,
        details=f"upright={back_rest_center}, reclined={back_reclined_center}",
    )

    foot_rest_aabb = ctx.part_element_world_aabb(footrest, elem="foot_panel")
    with ctx.pose({foot_hinge: foot_hinge.motion_limits.upper}):
        foot_raised_aabb = ctx.part_element_world_aabb(footrest, elem="foot_panel")

    foot_rest_center = _aabb_center(foot_rest_aabb)
    foot_raised_center = _aabb_center(foot_raised_aabb)
    ctx.check(
        "footrest swings forward and upward from the front hinge",
        foot_rest_center is not None
        and foot_raised_center is not None
        and foot_raised_center[0] > foot_rest_center[0] + 0.10
        and foot_raised_center[2] > foot_rest_center[2] + 0.10,
        details=f"stowed={foot_rest_center}, raised={foot_raised_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
