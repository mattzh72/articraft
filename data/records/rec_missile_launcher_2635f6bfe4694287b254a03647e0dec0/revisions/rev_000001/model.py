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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artillery_launcher")

    olive = model.material("olive_drab", rgba=(0.39, 0.43, 0.29, 1.0))
    steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    rubber = model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((2.00, 1.40, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=steel,
        name="base_plinth",
    )
    pedestal.visual(
        Box((0.86, 0.86, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, 0.57)),
        material=olive,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.40, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=steel,
        name="pedestal_cap",
    )
    pedestal.visual(
        Box((1.48, 0.26, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=rubber,
        name="service_step",
    )

    support_frame = model.part("support_frame")
    support_frame.visual(
        Cylinder(radius=0.41, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=steel,
        name="yaw_ring",
    )
    support_frame.visual(
        Box((1.08, 1.02, 0.12)),
        origin=Origin(xyz=(0.06, 0.0, 0.12)),
        material=olive,
        name="frame_deck",
    )
    support_frame.visual(
        Box((0.70, 0.06, 0.82)),
        origin=Origin(xyz=(-0.05, 0.50, 0.53)),
        material=olive,
        name="left_bracket",
    )
    support_frame.visual(
        Box((0.70, 0.06, 0.82)),
        origin=Origin(xyz=(-0.05, -0.50, 0.53)),
        material=olive,
        name="right_bracket",
    )
    support_frame.visual(
        Box((0.18, 0.90, 0.08)),
        origin=Origin(xyz=(-0.28, 0.0, 0.32)),
        material=steel,
        name="rear_crossbeam",
    )
    support_frame.visual(
        Box((0.26, 0.94, 0.20)),
        origin=Origin(xyz=(0.34, 0.0, 0.20)),
        material=steel,
        name="front_support_beam",
    )
    support_frame.visual(
        Box((0.46, 0.10, 0.10)),
        origin=Origin(xyz=(-0.13, 0.47, 0.30)),
        material=steel,
        name="left_gusset",
    )
    support_frame.visual(
        Box((0.46, 0.10, 0.10)),
        origin=Origin(xyz=(-0.13, -0.47, 0.30)),
        material=steel,
        name="right_gusset",
    )
    support_frame.visual(
        Cylinder(radius=0.10, length=0.06),
        origin=Origin(
            xyz=(-0.18, 0.47, 0.62),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_bearing",
    )
    support_frame.visual(
        Cylinder(radius=0.10, length=0.06),
        origin=Origin(
            xyz=(-0.18, -0.47, 0.62),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_bearing",
    )

    launch_box = model.part("launch_box")
    launch_box.visual(
        Box((1.46, 0.82, 0.50)),
        origin=Origin(xyz=(0.53, 0.0, 0.03)),
        material=olive,
        name="launcher_body",
    )
    launch_box.visual(
        Box((0.70, 0.78, 0.08)),
        origin=Origin(xyz=(0.20, 0.0, 0.31)),
        material=steel,
        name="top_service_cover",
    )
    launch_box.visual(
        Box((0.12, 0.86, 0.54)),
        origin=Origin(xyz=(1.20, 0.0, 0.03)),
        material=steel,
        name="front_face_plate",
    )
    launch_box.visual(
        Box((0.26, 0.90, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, -0.17)),
        material=steel,
        name="rear_mount_block",
    )
    launch_box.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(
            xyz=(0.0, 0.41, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_trunnion",
    )
    launch_box.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(
            xyz=(0.0, -0.41, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_trunnion",
    )

    model.articulation(
        "pedestal_to_frame",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=support_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=-2.1,
            upper=2.1,
        ),
    )
    model.articulation(
        "frame_to_launch_box",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=launch_box,
        origin=Origin(xyz=(-0.18, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.7,
            lower=0.0,
            upper=1.10,
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

    pedestal = object_model.get_part("pedestal")
    support_frame = object_model.get_part("support_frame")
    launch_box = object_model.get_part("launch_box")
    yaw_joint = object_model.get_articulation("pedestal_to_frame")
    pitch_joint = object_model.get_articulation("frame_to_launch_box")

    ctx.expect_contact(
        support_frame,
        pedestal,
        elem_a="yaw_ring",
        elem_b="pedestal_cap",
        name="yaw ring seats on pedestal cap",
    )
    ctx.expect_gap(
        launch_box,
        support_frame,
        axis="z",
        positive_elem="launcher_body",
        negative_elem="frame_deck",
        min_gap=0.18,
        max_gap=0.30,
        name="launcher body clears the frame deck at rest",
    )
    ctx.expect_contact(
        launch_box,
        support_frame,
        elem_a="left_trunnion",
        elem_b="left_bearing",
        name="left trunnion meets left bearing",
    )
    ctx.expect_contact(
        launch_box,
        support_frame,
        elem_a="right_trunnion",
        elem_b="right_bearing",
        name="right trunnion meets right bearing",
    )
    ctx.expect_overlap(
        launch_box,
        support_frame,
        axes="y",
        elem_a="launcher_body",
        elem_b="rear_crossbeam",
        min_overlap=0.78,
        name="launch box remains centered between the side brackets",
    )

    rest_front = ctx.part_element_world_aabb(launch_box, elem="front_face_plate")
    with ctx.pose({pitch_joint: pitch_joint.motion_limits.upper}):
        raised_front = ctx.part_element_world_aabb(launch_box, elem="front_face_plate")
    rest_top = None if rest_front is None else rest_front[1][2]
    raised_top = None if raised_front is None else raised_front[1][2]
    ctx.check(
        "positive pitch raises the launcher muzzle end",
        rest_top is not None
        and raised_top is not None
        and raised_top > rest_top + 0.55,
        details=f"rest_top={rest_top}, raised_top={raised_top}",
    )

    def _aabb_center_xy(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return (
            0.5 * (min_corner[0] + max_corner[0]),
            0.5 * (min_corner[1] + max_corner[1]),
        )

    rest_beam = ctx.part_element_world_aabb(support_frame, elem="front_support_beam")
    with ctx.pose({yaw_joint: math.pi / 2.0}):
        turned_beam = ctx.part_element_world_aabb(support_frame, elem="front_support_beam")
    rest_center = _aabb_center_xy(rest_beam)
    turned_center = _aabb_center_xy(turned_beam)
    ctx.check(
        "positive yaw swings the upper frame around the vertical axis",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.20
        and abs(rest_center[1]) < 0.08
        and turned_center[1] > 0.20
        and abs(turned_center[0]) < 0.10,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
