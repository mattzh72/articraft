from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="workstation_motherboard")

    pcb = model.material("pcb", rgba=(0.07, 0.14, 0.10, 1.0))
    slot_black = model.material("slot_black", rgba=(0.12, 0.12, 0.13, 1.0))
    socket_black = model.material("socket_black", rgba=(0.20, 0.21, 0.22, 1.0))
    dark_cover = model.material("dark_cover", rgba=(0.10, 0.10, 0.11, 1.0))
    anodized = model.material("anodized", rgba=(0.48, 0.50, 0.53, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    connector = model.material("connector", rgba=(0.78, 0.80, 0.70, 1.0))

    board_length = 0.305
    board_width = 0.244
    board_thickness = 0.0024
    embed = 0.00025

    cpu_center = (-0.020, 0.030)
    socket_size = 0.068
    socket_height = 0.0055

    frame_outer_x = 0.082
    frame_outer_y = 0.076
    frame_bar = 0.006
    frame_thickness = 0.002

    chipset_center = (0.073, -0.040)
    sink_base_size = (0.070, 0.055, 0.004)
    sink_fin_size = (0.006, 0.048, 0.014)
    shroud_width = 0.086
    shroud_depth = 0.070
    shroud_top_thickness = 0.003

    board = model.part("motherboard")

    def mounted_z(height: float, z0: float | None = None) -> float:
        base = board_thickness - embed if z0 is None else z0
        return base + height / 2.0

    board.visual(
        Box((board_length, board_width, board_thickness)),
        origin=Origin(xyz=(0.0, 0.0, board_thickness / 2.0)),
        material=pcb,
        name="pcb",
    )

    board.visual(
        Box((0.018, 0.094, 0.035)),
        origin=Origin(
            xyz=(-board_length / 2.0 + 0.009, 0.064, mounted_z(0.035)),
        ),
        material=steel,
        name="rear_io_stack",
    )
    board.visual(
        Box((0.030, 0.100, 0.012)),
        origin=Origin(
            xyz=(-board_length / 2.0 + 0.023, 0.064, mounted_z(0.012)),
        ),
        material=dark_cover,
        name="io_shroud",
    )
    board.visual(
        Box((0.052, 0.014, 0.010)),
        origin=Origin(
            xyz=(board_length / 2.0 - 0.026, 0.076, mounted_z(0.010)),
        ),
        material=connector,
        name="eps_power_header",
    )
    board.visual(
        Box((0.013, 0.132, 0.010)),
        origin=Origin(
            xyz=(board_length / 2.0 - 0.0065, -0.010, mounted_z(0.010)),
        ),
        material=connector,
        name="atx_power_header",
    )

    board.visual(
        Box((socket_size, socket_size, socket_height)),
        origin=Origin(
            xyz=(cpu_center[0], cpu_center[1], mounted_z(socket_height)),
        ),
        material=socket_black,
        name="cpu_socket_base",
    )
    board.visual(
        Box((0.051, 0.051, 0.0014)),
        origin=Origin(
            xyz=(
                cpu_center[0],
                cpu_center[1],
                mounted_z(0.0014, board_thickness - embed + socket_height - 0.0002),
            ),
        ),
        material=connector,
        name="cpu_socket_contacts",
    )
    board.visual(
        Box((0.060, 0.012, 0.013)),
        origin=Origin(
            xyz=(
                cpu_center[0],
                cpu_center[1] + socket_size / 2.0 + 0.008,
                mounted_z(0.013),
            ),
        ),
        material=anodized,
        name="vrm_heatsink_top",
    )
    board.visual(
        Box((0.012, 0.058, 0.012)),
        origin=Origin(
            xyz=(
                cpu_center[0] - socket_size / 2.0 - 0.011,
                cpu_center[1],
                mounted_z(0.012),
            ),
        ),
        material=anodized,
        name="vrm_heatsink_left",
    )

    socket_top = board_thickness - embed + socket_height
    frame_hinge_x = cpu_center[0] - frame_outer_x / 2.0
    frame_joint_z = socket_top + 0.0010

    board.visual(
        Box((0.004, frame_outer_y + 0.004, 0.004)),
        origin=Origin(
            xyz=(frame_hinge_x - 0.002, cpu_center[1], mounted_z(0.004, socket_top - 0.0001)),
        ),
        material=anodized,
        name="socket_hinge_rail",
    )
    board.visual(
        Box((0.006, 0.018, 0.012)),
        origin=Origin(
            xyz=(
                cpu_center[0] + frame_outer_x / 2.0 + 0.004,
                cpu_center[1],
                mounted_z(0.012),
            ),
        ),
        material=steel,
        name="retention_latch_block",
    )

    ram_x = cpu_center[0] + 0.084
    for idx in range(6):
        ram_y = -0.055 + idx * 0.026
        board.visual(
            Box((0.007, 0.120, 0.010)),
            origin=Origin(xyz=(ram_x, ram_y, mounted_z(0.010))),
            material=slot_black,
            name=f"dimm_slot_{idx}",
        )

    pcie_x = -0.006
    for idx, y in enumerate((-0.095, -0.068, -0.041)):
        length = 0.128 if idx == 0 else 0.102
        board.visual(
            Box((length, 0.010, 0.011)),
            origin=Origin(xyz=(pcie_x, y, mounted_z(0.011))),
            material=slot_black,
            name=f"pcie_slot_{idx}",
        )

    board.visual(
        Box(sink_base_size),
        origin=Origin(
            xyz=(
                chipset_center[0],
                chipset_center[1],
                mounted_z(sink_base_size[2]),
            ),
        ),
        material=anodized,
        name="chipset_heatsink_base",
    )

    fin_bottom = board_thickness - embed + sink_base_size[2] - 0.00015
    fin_specs = (
        ("chipset_fin_0", -0.024),
        ("chipset_fin_1", -0.016),
        ("chipset_fin_2", -0.008),
        ("chipset_fin_3", 0.0),
        ("chipset_fin_4", 0.008),
        ("chipset_fin_5", 0.016),
        ("chipset_fin_6", 0.024),
    )
    for fin_name, dx in fin_specs:
        board.visual(
            Box(sink_fin_size),
            origin=Origin(
                xyz=(
                    chipset_center[0] + dx,
                    chipset_center[1],
                    mounted_z(sink_fin_size[2], fin_bottom),
                ),
            ),
            material=steel,
            name=fin_name,
        )

    shroud_hinge_y = chipset_center[1] - shroud_depth / 2.0
    board.visual(
        Box((shroud_width + 0.004, 0.005, 0.004)),
        origin=Origin(
            xyz=(
                chipset_center[0],
                shroud_hinge_y - 0.0025,
                mounted_z(0.004, fin_bottom + sink_fin_size[2] + 0.0006),
            ),
        ),
        material=dark_cover,
        name="chipset_hinge_block",
    )
    hinge_support_height = fin_bottom + sink_fin_size[2] + 0.0006 - (board_thickness - embed)
    for support_name, support_x in (
        ("chipset_hinge_support_left", chipset_center[0] - 0.028),
        ("chipset_hinge_support_right", chipset_center[0] + 0.028),
    ):
        board.visual(
            Box((0.008, 0.008, hinge_support_height)),
            origin=Origin(
                xyz=(
                    support_x,
                    shroud_hinge_y - 0.0025,
                    mounted_z(hinge_support_height),
                ),
            ),
            material=dark_cover,
            name=support_name,
        )

    retention_frame = model.part("retention_frame")
    retention_frame.visual(
        Box((frame_bar, frame_outer_y, frame_thickness)),
        origin=Origin(xyz=(frame_bar / 2.0, 0.0, frame_thickness / 2.0)),
        material=steel,
        name="frame_left_bar",
    )
    retention_frame.visual(
        Box((frame_outer_x, frame_bar, frame_thickness)),
        origin=Origin(
            xyz=(frame_outer_x / 2.0, (frame_outer_y - frame_bar) / 2.0, frame_thickness / 2.0),
        ),
        material=steel,
        name="frame_top_bar",
    )
    retention_frame.visual(
        Box((frame_outer_x, frame_bar, frame_thickness)),
        origin=Origin(
            xyz=(frame_outer_x / 2.0, -(frame_outer_y - frame_bar) / 2.0, frame_thickness / 2.0),
        ),
        material=steel,
        name="frame_bottom_bar",
    )
    retention_frame.visual(
        Box((frame_bar, frame_outer_y, frame_thickness)),
        origin=Origin(
            xyz=(frame_outer_x - frame_bar / 2.0, 0.0, frame_thickness / 2.0),
        ),
        material=steel,
        name="frame_right_bar",
    )
    retention_frame.visual(
        Box((0.010, 0.016, 0.002)),
        origin=Origin(
            xyz=(frame_outer_x + 0.004, 0.0, frame_thickness / 2.0),
        ),
        material=steel,
        name="frame_latch_tongue",
    )

    chipset_shroud = model.part("chipset_shroud")
    chipset_shroud.visual(
        Box((shroud_width, shroud_depth, shroud_top_thickness)),
        origin=Origin(xyz=(0.0, shroud_depth / 2.0, 0.0)),
        material=dark_cover,
        name="shroud_top",
    )
    chipset_shroud.visual(
        Box((0.004, shroud_depth - 0.010, 0.018)),
        origin=Origin(xyz=(-shroud_width / 2.0 + 0.002, shroud_depth / 2.0, -0.010)),
        material=dark_cover,
        name="shroud_left_skirt",
    )
    chipset_shroud.visual(
        Box((0.004, shroud_depth - 0.010, 0.018)),
        origin=Origin(xyz=(shroud_width / 2.0 - 0.002, shroud_depth / 2.0, -0.010)),
        material=dark_cover,
        name="shroud_right_skirt",
    )
    chipset_shroud.visual(
        Box((shroud_width - 0.006, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, shroud_depth - 0.002, -0.008)),
        material=dark_cover,
        name="shroud_front_lip",
    )
    chipset_shroud.visual(
        Cylinder(radius=0.003, length=shroud_width * 0.42),
        origin=Origin(
            xyz=(-0.016, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="shroud_hinge_left",
    )
    chipset_shroud.visual(
        Cylinder(radius=0.003, length=shroud_width * 0.42),
        origin=Origin(
            xyz=(0.016, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="shroud_hinge_right",
    )

    model.articulation(
        "socket_retention_hinge",
        ArticulationType.REVOLUTE,
        parent=board,
        child=retention_frame,
        origin=Origin(xyz=(frame_hinge_x, cpu_center[1], frame_joint_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    shroud_joint_z = fin_bottom + sink_fin_size[2] + 0.004
    model.articulation(
        "chipset_shroud_hinge",
        ArticulationType.REVOLUTE,
        parent=board,
        child=chipset_shroud,
        origin=Origin(xyz=(chipset_center[0], shroud_hinge_y, shroud_joint_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
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

    board = object_model.get_part("motherboard")
    retention_frame = object_model.get_part("retention_frame")
    chipset_shroud = object_model.get_part("chipset_shroud")
    socket_hinge = object_model.get_articulation("socket_retention_hinge")
    shroud_hinge = object_model.get_articulation("chipset_shroud_hinge")

    ctx.expect_overlap(
        retention_frame,
        board,
        axes="xy",
        elem_b="cpu_socket_base",
        min_overlap=0.050,
        name="retention frame spans the CPU socket footprint",
    )
    ctx.expect_gap(
        retention_frame,
        board,
        axis="z",
        negative_elem="cpu_socket_base",
        max_gap=0.004,
        max_penetration=0.0,
        name="retention frame rests just above the socket body",
    )

    ctx.expect_overlap(
        chipset_shroud,
        board,
        axes="xy",
        elem_b="chipset_heatsink_base",
        min_overlap=0.045,
        name="chipset shroud covers the heatsink footprint",
    )
    ctx.expect_gap(
        chipset_shroud,
        board,
        axis="z",
        positive_elem="shroud_top",
        negative_elem="chipset_fin_3",
        min_gap=0.002,
        max_gap=0.010,
        name="shroud top clears the heatsink fins",
    )

    ctx.check(
        "hinge axes match the intended opening directions",
        socket_hinge.axis == (0.0, -1.0, 0.0) and shroud_hinge.axis == (1.0, 0.0, 0.0),
        details=f"socket_axis={socket_hinge.axis}, shroud_axis={shroud_hinge.axis}",
    )

    closed_frame_aabb = ctx.part_world_aabb(retention_frame)
    closed_shroud_aabb = ctx.part_world_aabb(chipset_shroud)
    with ctx.pose(
        {
            socket_hinge: socket_hinge.motion_limits.upper,
            shroud_hinge: shroud_hinge.motion_limits.upper,
        }
    ):
        open_frame_aabb = ctx.part_world_aabb(retention_frame)
        open_shroud_aabb = ctx.part_world_aabb(chipset_shroud)

    ctx.check(
        "retention frame opens upward",
        closed_frame_aabb is not None
        and open_frame_aabb is not None
        and open_frame_aabb[1][2] > closed_frame_aabb[1][2] + 0.040,
        details=f"closed={closed_frame_aabb}, open={open_frame_aabb}",
    )
    ctx.check(
        "chipset shroud opens upward",
        closed_shroud_aabb is not None
        and open_shroud_aabb is not None
        and open_shroud_aabb[1][2] > closed_shroud_aabb[1][2] + 0.030,
        details=f"closed={closed_shroud_aabb}, open={open_shroud_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
