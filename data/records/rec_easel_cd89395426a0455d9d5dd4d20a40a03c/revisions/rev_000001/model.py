from __future__ import annotations

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
    model = ArticulatedObject(name="presentation_easel")

    frame_mat = model.material("frame_mat", rgba=(0.73, 0.75, 0.78, 1.0))
    board_mat = model.material("board_mat", rgba=(0.96, 0.97, 0.98, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.34, 0.36, 0.39, 1.0))
    foot_mat = model.material("foot_mat", rgba=(0.18, 0.19, 0.20, 1.0))

    frame_width = 0.76
    rail_width = 0.036
    frame_depth = 0.028
    frame_height = 1.74
    rail_x = frame_width / 2.0 - rail_width / 2.0

    frame = model.part("frame")

    for idx, x_pos in enumerate((-rail_x, rail_x)):
        frame.visual(
            Box((rail_width, frame_depth, frame_height)),
            origin=Origin(xyz=(x_pos, 0.0, frame_height / 2.0)),
            material=frame_mat,
            name=f"front_rail_{idx}",
        )

    frame.visual(
        Box((frame_width - rail_width, frame_depth, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, frame_height - 0.024)),
        material=frame_mat,
        name="top_bar",
    )
    frame.visual(
        Box((frame_width - rail_width, frame_depth, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=frame_mat,
        name="lower_bar",
    )

    frame.visual(
        Box((0.688, 0.014, 1.170)),
        origin=Origin(xyz=(0.0, -0.002, 0.945)),
        material=trim_mat,
        name="back_panel",
    )
    frame.visual(
        Box((0.650, 0.004, 1.130)),
        origin=Origin(xyz=(0.0, 0.005, 0.945)),
        material=board_mat,
        name="writing_face",
    )

    frame.visual(
        Box((0.560, 0.075, 0.022)),
        origin=Origin(xyz=(0.0, 0.033, 0.405)),
        material=trim_mat,
        name="tray_shelf",
    )
    frame.visual(
        Box((0.560, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.066, 0.408)),
        material=trim_mat,
        name="tray_lip",
    )
    for idx, x_pos in enumerate((-0.220, 0.220)):
        frame.visual(
            Box((0.028, 0.048, 0.072)),
            origin=Origin(xyz=(x_pos, 0.016, 0.372)),
            material=trim_mat,
            name=f"tray_bracket_{idx}",
        )

    for idx, x_pos in enumerate((-rail_x, rail_x)):
        frame.visual(
            Box((0.050, 0.060, 0.020)),
            origin=Origin(xyz=(x_pos, 0.010, 0.010)),
            material=foot_mat,
            name=f"front_foot_{idx}",
        )

    frame.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(-0.120, -0.022, 1.688), rpy=(0.0, 1.57079632679, 0.0)),
        material=frame_mat,
        name="rear_hinge_0",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.120, -0.022, 1.688), rpy=(0.0, 1.57079632679, 0.0)),
        material=frame_mat,
        name="rear_hinge_1",
    )
    frame.visual(
        Box((0.070, 0.016, 0.028)),
        origin=Origin(xyz=(-0.120, -0.018, 1.700)),
        material=frame_mat,
        name="rear_hinge_mount_0",
    )
    frame.visual(
        Box((0.070, 0.016, 0.028)),
        origin=Origin(xyz=(0.120, -0.018, 1.700)),
        material=frame_mat,
        name="rear_hinge_mount_1",
    )

    for idx, x_pos in enumerate((-0.362, 0.362)):
        frame.visual(
            Box((0.018, 0.016, 0.090)),
            origin=Origin(xyz=(x_pos, 0.008, 1.280)),
            material=frame_mat,
            name=f"holder_mount_{idx}",
        )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.009, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=trim_mat,
        name="leg_barrel",
    )
    rear_leg.visual(
        Box((0.160, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.010, -0.028)),
        material=trim_mat,
        name="leg_bracket",
    )
    rear_leg.visual(
        Box((0.032, 0.022, 1.780)),
        origin=Origin(xyz=(0.0, -0.338, -0.860), rpy=(-0.380, 0.0, 0.0)),
        material=trim_mat,
        name="leg_shaft",
    )
    rear_leg.visual(
        Box((0.110, 0.042, 0.020)),
        origin=Origin(xyz=(0.0, -0.676, -1.686), rpy=(-0.380, 0.0, 0.0)),
        material=foot_mat,
        name="leg_foot",
    )

    model.articulation(
        "rear_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, -0.024, 1.688)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-0.08, upper=0.36),
    )

    def add_holder(name: str, hinge_name: str, x_pos: float, sign: float, axis_y: float) -> None:
        holder = model.part(name)
        holder.visual(
            Cylinder(radius=0.006, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
            material=frame_mat,
            name="hinge_barrel",
        )
        holder.visual(
            Box((0.028, 0.012, 0.022)),
            origin=Origin(xyz=(sign * 0.014, 0.0, 0.0)),
            material=frame_mat,
            name="holder_stub",
        )
        holder.visual(
            Box((0.230, 0.012, 0.016)),
            origin=Origin(xyz=(sign * 0.129, 0.0, 0.0)),
            material=frame_mat,
            name="holder_arm",
        )
        holder.visual(
            Box((0.020, 0.010, 0.060)),
            origin=Origin(xyz=(sign * 0.240, 0.0, -0.022)),
            material=frame_mat,
            name="holder_tip",
        )

        model.articulation(
            hinge_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=holder,
            origin=Origin(xyz=(x_pos, 0.030, 1.280)),
            axis=(0.0, axis_y, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.35),
        )

    add_holder("holder_0", "holder_0_hinge", -0.362, -1.0, -1.0)
    add_holder("holder_1", "holder_1_hinge", 0.362, 1.0, 1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    rear_leg = object_model.get_part("rear_leg")
    holder_0 = object_model.get_part("holder_0")
    holder_1 = object_model.get_part("holder_1")

    rear_leg_hinge = object_model.get_articulation("rear_leg_hinge")
    holder_0_hinge = object_model.get_articulation("holder_0_hinge")
    holder_1_hinge = object_model.get_articulation("holder_1_hinge")

    ctx.expect_gap(
        frame,
        rear_leg,
        axis="y",
        positive_elem="back_panel",
        negative_elem="leg_shaft",
        min_gap=0.010,
        name="rear leg stays behind the board",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    rear_leg_aabb = ctx.part_world_aabb(rear_leg)
    holder_0_aabb = ctx.part_world_aabb(holder_0)
    holder_1_aabb = ctx.part_world_aabb(holder_1)

    ctx.check(
        "page holders start spread wider than the board",
        frame_aabb is not None
        and holder_0_aabb is not None
        and holder_1_aabb is not None
        and holder_0_aabb[0][0] < frame_aabb[0][0] - 0.18
        and holder_1_aabb[1][0] > frame_aabb[1][0] + 0.18,
        details=f"frame={frame_aabb}, holder_0={holder_0_aabb}, holder_1={holder_1_aabb}",
    )

    with ctx.pose({holder_0_hinge: 1.30, holder_1_hinge: 1.30}):
        ctx.expect_gap(
            holder_0,
            frame,
            axis="y",
            positive_elem="holder_arm",
            negative_elem="writing_face",
            min_gap=0.008,
            name="holder 0 folds in front of the whiteboard",
        )
        ctx.expect_gap(
            holder_1,
            frame,
            axis="y",
            positive_elem="holder_arm",
            negative_elem="writing_face",
            min_gap=0.008,
            name="holder 1 folds in front of the whiteboard",
        )

        folded_holder_0_aabb = ctx.part_world_aabb(holder_0)
        folded_holder_1_aabb = ctx.part_world_aabb(holder_1)
        ctx.check(
            "page holders fold back toward the frame edges",
            frame_aabb is not None
            and folded_holder_0_aabb is not None
            and folded_holder_1_aabb is not None
            and folded_holder_0_aabb[0][0] > frame_aabb[0][0] - 0.08
            and folded_holder_1_aabb[1][0] < frame_aabb[1][0] + 0.08,
            details=(
                f"frame={frame_aabb}, "
                f"folded_holder_0={folded_holder_0_aabb}, folded_holder_1={folded_holder_1_aabb}"
            ),
        )

    rear_leg_shaft_aabb = ctx.part_element_world_aabb(rear_leg, elem="leg_shaft")
    with ctx.pose({rear_leg_hinge: 0.28}):
        folded_leg_aabb = ctx.part_world_aabb(rear_leg)
        folded_leg_shaft_aabb = ctx.part_element_world_aabb(rear_leg, elem="leg_shaft")
        ctx.expect_gap(
            frame,
            rear_leg,
            axis="y",
            positive_elem="back_panel",
            negative_elem="leg_shaft",
            min_gap=0.002,
            name="rear leg can fold closer without crossing the board",
        )
        ctx.check(
            "rear leg rotates forward toward the board",
            rear_leg_shaft_aabb is not None
            and folded_leg_shaft_aabb is not None
            and folded_leg_shaft_aabb[1][1] > rear_leg_shaft_aabb[1][1] + 0.20,
            details=(
                f"rest={rear_leg_aabb}, folded={folded_leg_aabb}, "
                f"rest_shaft={rear_leg_shaft_aabb}, folded_shaft={folded_leg_shaft_aabb}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
