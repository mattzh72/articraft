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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_motherboard")

    pcb_x = 0.305
    pcb_y = 0.244
    pcb_t = 0.0016

    slot_height = 0.011
    slot_width = 0.010
    clip_axis_z = 0.0135
    clip_barrel_r = 0.0015

    m2_cover_length = 0.086
    m2_cover_width = 0.024
    m2_cover_thickness = 0.0032
    m2_center_x = 0.015
    m2_center_y = -0.010
    m2_hinge_x = m2_center_x - (m2_cover_length / 2.0)

    model.material("pcb_black", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("slot_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("metal_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("metal_silver", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("connector_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("accent_red", rgba=(0.62, 0.09, 0.12, 1.0))

    board = model.part("board")
    board.visual(
        Box((pcb_x, pcb_y, pcb_t)),
        origin=Origin(xyz=(0.0, 0.0, pcb_t / 2.0)),
        material="pcb_black",
        name="pcb",
    )
    board.visual(
        Box((0.064, 0.064, 0.0035)),
        origin=Origin(xyz=(-0.018, 0.040, pcb_t + 0.00175)),
        material="connector_gray",
        name="cpu_socket_frame",
    )
    board.visual(
        Box((0.048, 0.048, 0.0015)),
        origin=Origin(xyz=(-0.018, 0.040, pcb_t + 0.00425)),
        material="metal_dark",
        name="cpu_socket_cap",
    )
    board.visual(
        Box((0.118, 0.018, 0.020)),
        origin=Origin(xyz=(-0.006, 0.093, pcb_t + 0.010)),
        material="metal_dark",
        name="vrm_top_heatsink",
    )
    board.visual(
        Box((0.018, 0.086, 0.020)),
        origin=Origin(xyz=(-0.076, 0.048, pcb_t + 0.010)),
        material="metal_dark",
        name="vrm_side_heatsink",
    )
    board.visual(
        Box((0.050, 0.066, 0.022)),
        origin=Origin(xyz=(-0.123, 0.083, pcb_t + 0.011)),
        material="metal_dark",
        name="rear_io_shroud",
    )
    board.visual(
        Box((0.058, 0.010, 0.016)),
        origin=Origin(xyz=(0.111, 0.095, pcb_t + 0.008)),
        material="connector_gray",
        name="atx_power_header",
    )
    for index in range(4):
        board.visual(
            Box((0.006, 0.128, 0.010)),
            origin=Origin(xyz=(0.086 + 0.008 * index, 0.032, pcb_t + 0.005)),
            material="slot_black",
            name=f"ram_slot_{index + 1}",
        )
    board.visual(
        Box((0.088, 0.026, 0.0016)),
        origin=Origin(xyz=(m2_center_x, m2_center_y, pcb_t + 0.0008)),
        material="metal_dark",
        name="m2_base",
    )
    board.visual(
        Box((0.008, 0.008, 0.0035)),
        origin=Origin(xyz=(0.057, m2_center_y, pcb_t + 0.00175)),
        material="connector_gray",
        name="m2_standoff",
    )
    board.visual(
        Box((0.006, 0.026, 0.0014)),
        origin=Origin(xyz=(m2_hinge_x - 0.003, m2_center_y, pcb_t + 0.0007)),
        material="metal_dark",
        name="m2_hinge_support",
    )
    board.visual(
        Box((0.054, 0.050, 0.020)),
        origin=Origin(xyz=(0.088, -0.055, pcb_t + 0.010)),
        material="metal_dark",
        name="chipset_heatsink",
    )
    board.visual(
        Box((0.062, 0.006, 0.0008)),
        origin=Origin(xyz=(0.088, -0.055, pcb_t + 0.0204)),
        material="accent_red",
        name="chipset_accent_bar",
    )
    board.visual(
        Box((0.100, 0.006, 0.0008)),
        origin=Origin(xyz=(0.008, 0.094, pcb_t + 0.0204)),
        material="accent_red",
        name="vrm_accent_bar",
    )
    board.inertial = Inertial.from_geometry(
        Box((pcb_x, pcb_y, 0.025)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
    )

    def add_slot_with_clip(
        slot_name: str,
        clip_name: str,
        slot_joint_name: str,
        clip_joint_name: str,
        *,
        slot_length: float,
        slot_center: tuple[float, float],
    ) -> None:
        slot_part = model.part(slot_name)
        slot_part.visual(
            Box((slot_length, slot_width, slot_height)),
            origin=Origin(xyz=(0.0, 0.0, slot_height / 2.0)),
            material="slot_black",
            name="slot_body",
        )
        slot_part.visual(
            Box((slot_length - 0.010, slot_width * 0.35, 0.0035)),
            origin=Origin(xyz=(0.0, 0.0, slot_height - 0.00175)),
            material="connector_gray",
            name="contact_ridge",
        )
        slot_part.visual(
            Box((0.006, slot_width * 0.70, clip_axis_z - clip_barrel_r)),
            origin=Origin(
                xyz=(
                    (slot_length / 2.0) + 0.003,
                    0.0,
                    (clip_axis_z - clip_barrel_r) / 2.0,
                )
            ),
            material="slot_black",
            name="clip_pedestal",
        )
        slot_part.visual(
            Box((0.004, slot_width, slot_height)),
            origin=Origin(xyz=(-(slot_length / 2.0) + 0.002, 0.0, slot_height / 2.0)),
            material="slot_black",
            name="slot_tail_block",
        )
        slot_part.inertial = Inertial.from_geometry(
            Box((slot_length + 0.010, slot_width, clip_axis_z)),
            mass=0.09 if slot_length > 0.05 else 0.04,
            origin=Origin(xyz=(0.004, 0.0, clip_axis_z / 2.0)),
        )

        model.articulation(
            slot_joint_name,
            ArticulationType.FIXED,
            parent=board,
            child=slot_part,
            origin=Origin(xyz=(slot_center[0], slot_center[1], pcb_t)),
        )

        clip = model.part(clip_name)
        clip.visual(
            Cylinder(radius=clip_barrel_r, length=slot_width * 0.88),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="slot_black",
            name="clip_barrel",
        )
        clip.visual(
            Box((0.014, slot_width * 0.55, 0.0024)),
            origin=Origin(xyz=(-0.007, 0.0, -0.0014)),
            material="slot_black",
            name="clip_finger",
        )
        clip.visual(
            Box((0.004, slot_width * 0.75, 0.0050)),
            origin=Origin(xyz=(0.002, 0.0, 0.0015)),
            material="slot_black",
            name="clip_paddle",
        )
        clip.visual(
            Box((0.0025, slot_width * 0.40, 0.0040)),
            origin=Origin(xyz=(-0.0122, 0.0, 0.0003)),
            material="slot_black",
            name="clip_hook",
        )
        clip.inertial = Inertial.from_geometry(
            Box((0.018, slot_width, 0.010)),
            mass=0.01,
            origin=Origin(xyz=(-0.004, 0.0, 0.0005)),
        )

        model.articulation(
            clip_joint_name,
            ArticulationType.REVOLUTE,
            parent=slot_part,
            child=clip,
            origin=Origin(xyz=((slot_length / 2.0) + 0.004, 0.0, clip_axis_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=3.0,
                lower=0.0,
                upper=0.90,
            ),
        )

    add_slot_with_clip(
        "pcie_x16_top",
        "pcie_x16_top_clip",
        "board_to_pcie_x16_top",
        "pcie_x16_top_clip_hinge",
        slot_length=0.090,
        slot_center=(0.012, -0.050),
    )
    add_slot_with_clip(
        "pcie_x1_mid",
        "pcie_x1_mid_clip",
        "board_to_pcie_x1_mid",
        "pcie_x1_mid_clip_hinge",
        slot_length=0.034,
        slot_center=(0.012, -0.068),
    )
    add_slot_with_clip(
        "pcie_x16_lower",
        "pcie_x16_lower_clip",
        "board_to_pcie_x16_lower",
        "pcie_x16_lower_clip_hinge",
        slot_length=0.090,
        slot_center=(0.012, -0.086),
    )
    add_slot_with_clip(
        "pcie_x1_bottom",
        "pcie_x1_bottom_clip",
        "board_to_pcie_x1_bottom",
        "pcie_x1_bottom_clip_hinge",
        slot_length=0.034,
        slot_center=(0.012, -0.104),
    )

    m2_cover = model.part("m2_cover")
    m2_cover.visual(
        Cylinder(radius=0.0018, length=m2_cover_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="metal_dark",
        name="cover_hinge_barrel",
    )
    m2_cover.visual(
        Box((m2_cover_length, m2_cover_width, m2_cover_thickness)),
        origin=Origin(
            xyz=(
                m2_cover_length / 2.0,
                0.0,
                -0.0012,
            )
        ),
        material="metal_silver",
        name="cover_panel",
    )
    m2_cover.visual(
        Box((0.014, m2_cover_width * 0.45, 0.0012)),
        origin=Origin(
            xyz=(
                m2_cover_length - 0.007,
                0.0,
                0.0009,
            )
        ),
        material="accent_red",
        name="cover_tip",
    )
    m2_cover.inertial = Inertial.from_geometry(
        Box((m2_cover_length, m2_cover_width, 0.007)),
        mass=0.05,
        origin=Origin(xyz=(m2_cover_length / 2.0, 0.0, -0.0004)),
    )

    model.articulation(
        "board_to_m2_cover",
        ArticulationType.REVOLUTE,
        parent=board,
        child=m2_cover,
        origin=Origin(xyz=(m2_hinge_x, m2_center_y, pcb_t + 0.0048)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=0.0,
            upper=1.40,
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

    board = object_model.get_part("board")
    m2_cover = object_model.get_part("m2_cover")
    m2_hinge = object_model.get_articulation("board_to_m2_cover")
    top_slot = object_model.get_part("pcie_x16_top")
    top_clip = object_model.get_part("pcie_x16_top_clip")
    top_clip_hinge = object_model.get_articulation("pcie_x16_top_clip_hinge")
    lower_slot = object_model.get_part("pcie_x16_lower")

    ctx.expect_overlap(
        m2_cover,
        board,
        axes="xy",
        elem_a="cover_panel",
        elem_b="m2_base",
        min_overlap=0.020,
        name="M.2 cover spans the M.2 slot footprint",
    )
    ctx.expect_gap(
        m2_cover,
        board,
        axis="z",
        positive_elem="cover_tip",
        negative_elem="pcb",
        min_gap=0.0002,
        max_gap=0.0060,
        name="closed M.2 cover sits just above the motherboard surface",
    )
    with ctx.pose({m2_hinge: 1.15}):
        ctx.expect_gap(
            m2_cover,
            board,
            axis="z",
            positive_elem="cover_tip",
            negative_elem="pcb",
            min_gap=0.030,
            name="opened M.2 cover lifts clear of the board",
        )

    ctx.expect_contact(
        top_slot,
        board,
        elem_a="slot_body",
        elem_b="pcb",
        contact_tol=0.0005,
        name="top PCIe slot housing sits on the PCB",
    )
    ctx.expect_contact(
        lower_slot,
        board,
        elem_a="slot_body",
        elem_b="pcb",
        contact_tol=0.0005,
        name="lower PCIe slot housing sits on the PCB",
    )

    closed_clip_aabb = ctx.part_element_world_aabb(top_clip, elem="clip_finger")
    with ctx.pose({top_clip_hinge: 0.75}):
        ctx.expect_gap(
            top_clip,
            top_slot,
            axis="z",
            positive_elem="clip_hook",
            negative_elem="slot_body",
            min_gap=0.004,
            name="PCIe retention clip opens upward from the slot",
        )
        open_clip_aabb = ctx.part_element_world_aabb(top_clip, elem="clip_finger")

    ctx.check(
        "PCIe retention clip gains lift when opened",
        closed_clip_aabb is not None
        and open_clip_aabb is not None
        and open_clip_aabb[1][2] > closed_clip_aabb[1][2] + 0.006,
        details=f"closed={closed_clip_aabb}, open={open_clip_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
