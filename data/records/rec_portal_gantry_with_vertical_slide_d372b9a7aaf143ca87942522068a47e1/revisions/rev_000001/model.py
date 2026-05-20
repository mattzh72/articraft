from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_portal_module")

    frame_mat = model.material("powder_coated_steel", color=(0.18, 0.20, 0.22, 1.0))
    rail_mat = model.material("polished_linear_rail", color=(0.72, 0.74, 0.76, 1.0))
    carriage_mat = model.material("blue_anodized_carriage", color=(0.08, 0.22, 0.42, 1.0))
    stage_mat = model.material("dark_z_stage", color=(0.12, 0.13, 0.15, 1.0))
    face_mat = model.material("plain_tool_face", color=(0.78, 0.78, 0.74, 1.0))

    frame = model.part("portal_frame")

    def frame_visual(name: str, size: tuple[float, float, float], xyz: tuple[float, float, float]) -> None:
        frame.visual(Box(size), origin=Origin(xyz=xyz), material=frame_mat, name=name)

    # Two stiff side-wall cheek frames form the portal uprights.  Each wall is a
    # connected rectangular perimeter rather than a solid block so the open
    # service window reads clearly from the side.
    for x, prefix in ((-0.88, "wall_0"), (0.88, "wall_1")):
        frame_visual(f"{prefix}_front_upright", (0.10, 0.085, 1.18), (x, -0.18, 0.62))
        frame_visual(f"{prefix}_rear_upright", (0.10, 0.085, 1.18), (x, 0.18, 0.62))
        frame_visual(f"{prefix}_base_rail", (0.10, 0.43, 0.16), (x, 0.0, 0.08))
        frame_visual(f"{prefix}_top_rail", (0.10, 0.43, 0.16), (x, 0.0, 1.16))

    frame_visual("floor_tie", (1.86, 0.34, 0.08), (0.0, 0.0, 0.04))
    frame_visual("top_beam", (1.86, 0.22, 0.18), (0.0, 0.0, 1.17))
    frame_visual("rear_tie", (1.86, 0.07, 0.10), (0.0, 0.205, 0.52))

    # Horizontal rails on the front of the top beam carry the beam carriage.
    for z, name in ((1.105, "lower_rail"), (1.225, "upper_rail")):
        frame.visual(
            Cylinder(radius=0.025, length=1.46),
            origin=Origin(xyz=(0.0, -0.132, z), rpy=(0.0, 1.57079632679, 0.0)),
            material=rail_mat,
            name=name,
        )
        frame_visual(f"{name}_end_0", (0.06, 0.06, 0.08), (-0.76, -0.125, z))
        frame_visual(f"{name}_end_1", (0.06, 0.06, 0.08), (0.76, -0.125, z))

    carriage = model.part("beam_carriage")
    carriage.visual(
        Box((0.28, 0.08, 0.34)),
        origin=Origin(xyz=(0.0, -0.04, 0.0)),
        material=carriage_mat,
        name="saddle_plate",
    )
    for z, name in ((-0.060, "lower_bearing"), (0.060, "upper_bearing")):
        carriage.visual(
            Box((0.23, 0.035, 0.050)),
            origin=Origin(xyz=(0.0, -0.0155, z)),
            material=rail_mat,
            name=name,
        )
    for x, name in ((-0.072, "z_rail_0"), (0.072, "z_rail_1")):
        carriage.visual(
            Cylinder(radius=0.018, length=0.36),
            origin=Origin(xyz=(x, -0.095, 0.0)),
            material=rail_mat,
            name=name,
        )

    z_stage = model.part("z_stage")
    z_stage.visual(
        Box((0.16, 0.06, 0.62)),
        origin=Origin(xyz=(0.0, -0.03, -0.06)),
        material=stage_mat,
        name="vertical_slide",
    )
    z_stage.visual(
        Box((0.20, 0.035, 0.16)),
        origin=Origin(xyz=(0.0, -0.0775, -0.32)),
        material=face_mat,
        name="tool_face",
    )
    z_stage.visual(
        Box((0.11, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.0775, -0.2225)),
        material=stage_mat,
        name="tool_face_boss",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(-0.55, -0.159, 1.165)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.45, lower=0.0, upper=1.10),
    )
    model.articulation(
        "carriage_to_z_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=z_stage,
        origin=Origin(xyz=(0.0, -0.113, 0.08)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.28),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("portal_frame")
    carriage = object_model.get_part("beam_carriage")
    z_stage = object_model.get_part("z_stage")
    x_slide = object_model.get_articulation("frame_to_carriage")
    z_slide = object_model.get_articulation("carriage_to_z_stage")

    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="upper_rail",
        negative_elem="saddle_plate",
        name="carriage rides just in front of beam rail",
    )
    ctx.expect_within(
        carriage,
        frame,
        axes="x",
        inner_elem="saddle_plate",
        outer_elem="upper_rail",
        margin=0.0,
        name="carriage starts within rail span",
    )
    ctx.expect_gap(
        carriage,
        z_stage,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="z_rail_0",
        negative_elem="vertical_slide",
        name="z slide rides just in front of carriage rails",
    )
    ctx.expect_overlap(
        z_stage,
        carriage,
        axes="z",
        min_overlap=0.18,
        elem_a="vertical_slide",
        elem_b="z_rail_0",
        name="z slide starts retained on guide rail",
    )

    start_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({x_slide: 1.10}):
        ctx.expect_within(
            carriage,
            frame,
            axes="x",
            inner_elem="saddle_plate",
            outer_elem="upper_rail",
            margin=0.0,
            name="carriage remains within rail span",
        )
        end_carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "beam carriage translates along x",
        start_carriage_pos is not None
        and end_carriage_pos is not None
        and end_carriage_pos[0] > start_carriage_pos[0] + 1.0,
        details=f"start={start_carriage_pos}, end={end_carriage_pos}",
    )

    start_stage_pos = ctx.part_world_position(z_stage)
    with ctx.pose({z_slide: 0.28}):
        ctx.expect_overlap(
            z_stage,
            carriage,
            axes="z",
            min_overlap=0.10,
            elem_a="vertical_slide",
            elem_b="z_rail_0",
            name="z slide remains retained at full travel",
        )
        end_stage_pos = ctx.part_world_position(z_stage)
    ctx.check(
        "z stage translates vertically downward",
        start_stage_pos is not None
        and end_stage_pos is not None
        and end_stage_pos[2] < start_stage_pos[2] - 0.25,
        details=f"start={start_stage_pos}, end={end_stage_pos}",
    )

    return ctx.report()


object_model = build_object_model()
