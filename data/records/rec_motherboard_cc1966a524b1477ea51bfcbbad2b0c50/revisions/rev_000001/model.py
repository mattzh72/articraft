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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motherboard_with_gpu_release_and_m2_cover")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.32, 0.12, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    slot_black = model.material("slot_black", rgba=(0.07, 0.07, 0.08, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.35, 0.37, 0.40, 1.0))
    aluminum = model.material("aluminum", rgba=(0.71, 0.73, 0.76, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.62, 1.0))
    gold = model.material("gold", rgba=(0.80, 0.67, 0.25, 1.0))
    ssd_blue = model.material("ssd_blue", rgba=(0.14, 0.19, 0.34, 1.0))

    board_len = 0.305
    board_wid = 0.244
    board_thk = 0.0016
    board_top = board_thk

    pcie_slot_center = (0.018, -0.055)
    pcie_slot_len = 0.086
    pcie_slot_wid = 0.012
    pcie_slot_x = pcie_slot_center[0]
    pcie_slot_y = pcie_slot_center[1]
    pcie_slot_right = pcie_slot_x + pcie_slot_len / 2.0

    lever_pivot_x = pcie_slot_right + 0.007
    lever_pivot_y = pcie_slot_y - 0.002
    lever_pivot_z = board_top + 0.0028

    m2_hinge_x = -0.018
    m2_hinge_y = 0.022
    m2_hinge_z = board_top + 0.0031

    board = model.part("motherboard")
    board.visual(
        Box((board_len, board_wid, board_thk)),
        origin=Origin(xyz=(0.0, 0.0, board_thk / 2.0)),
        material=pcb_green,
        name="pcb",
    )

    # Rear I/O and processor area so the assembly reads as a motherboard.
    board.visual(
        Box((0.048, 0.028, 0.016)),
        origin=Origin(xyz=(-0.123, 0.096, board_top + 0.008)),
        material=dark_gray,
        name="rear_io_shroud",
    )
    board.visual(
        Box((0.062, 0.018, 0.014)),
        origin=Origin(xyz=(-0.072, 0.086, board_top + 0.007)),
        material=gunmetal,
        name="vrm_heatsink",
    )
    board.visual(
        Box((0.046, 0.046, 0.004)),
        origin=Origin(xyz=(-0.050, 0.041, board_top + 0.002)),
        material=steel,
        name="cpu_socket",
    )
    board.visual(
        Box((0.138, 0.007, 0.009)),
        origin=Origin(xyz=(0.070, 0.084, board_top + 0.0045)),
        material=matte_black,
        name="ram_slot_a",
    )
    board.visual(
        Box((0.138, 0.007, 0.009)),
        origin=Origin(xyz=(0.070, 0.071, board_top + 0.0045)),
        material=matte_black,
        name="ram_slot_b",
    )

    # Full-length PCIe slot with a visible channel.
    board.visual(
        Box((pcie_slot_len, pcie_slot_wid, 0.0022)),
        origin=Origin(xyz=(pcie_slot_x, pcie_slot_y, board_top + 0.0011)),
        material=slot_black,
        name="pcie_slot_base",
    )
    board.visual(
        Box((pcie_slot_len, 0.0018, 0.0048)),
        origin=Origin(xyz=(pcie_slot_x, pcie_slot_y + 0.0051, board_top + 0.0032)),
        material=slot_black,
        name="pcie_slot_wall_upper",
    )
    board.visual(
        Box((pcie_slot_len, 0.0018, 0.0048)),
        origin=Origin(xyz=(pcie_slot_x, pcie_slot_y - 0.0051, board_top + 0.0032)),
        material=slot_black,
        name="pcie_slot_wall_lower",
    )
    board.visual(
        Box((0.006, pcie_slot_wid, 0.0048)),
        origin=Origin(xyz=(pcie_slot_x - 0.040, pcie_slot_y, board_top + 0.0032)),
        material=slot_black,
        name="pcie_slot_fixed_end",
    )
    board.visual(
        Box((0.004, pcie_slot_wid, 0.0048)),
        origin=Origin(xyz=(pcie_slot_x + 0.039, pcie_slot_y, board_top + 0.0032)),
        material=slot_black,
        name="pcie_slot_latch_housing",
    )
    board.visual(
        Box((0.073, 0.0030, 0.0007)),
        origin=Origin(xyz=(pcie_slot_x - 0.002, pcie_slot_y, board_top + 0.00255)),
        material=gold,
        name="pcie_contacts",
    )
    board.visual(
        Cylinder(radius=0.0026, length=0.0028),
        origin=Origin(xyz=(lever_pivot_x, lever_pivot_y, board_top + 0.0014)),
        material=dark_gray,
        name="pcie_lever_pivot",
    )

    # M.2 socket region under the hinged cover.
    board.visual(
        Box((0.014, 0.006, 0.0030)),
        origin=Origin(xyz=(-0.020, m2_hinge_y, board_top + 0.0015)),
        material=matte_black,
        name="m2_socket_body",
    )
    board.visual(
        Box((0.076, 0.022, 0.0020)),
        origin=Origin(xyz=(0.024, m2_hinge_y, board_top + 0.0010)),
        material=ssd_blue,
        name="m2_drive",
    )
    board.visual(
        Cylinder(radius=0.0028, length=0.0020),
        origin=Origin(xyz=(0.064, m2_hinge_y, board_top + 0.0010)),
        material=steel,
        name="m2_standoff",
    )
    board.visual(
        Box((0.011, 0.012, 0.0014)),
        origin=Origin(xyz=(m2_hinge_x - 0.001, m2_hinge_y, board_top + 0.0006)),
        material=gunmetal,
        name="m2_hinge_shelf",
    )

    board.inertial = Inertial.from_geometry(
        Box((board_len, board_wid, 0.020)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    lever = model.part("gpu_release_lever")
    lever.visual(
        Cylinder(radius=0.0030, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0007)),
        material=dark_gray,
        name="lever_pivot_collar",
    )
    lever.visual(
        Box((0.0050, 0.0260, 0.0038)),
        origin=Origin(xyz=(0.0005, -0.0140, 0.0025)),
        material=matte_black,
        name="lever_arm",
    )
    lever.visual(
        Box((0.0090, 0.0080, 0.0054)),
        origin=Origin(xyz=(0.0020, -0.0280, 0.0033)),
        material=matte_black,
        name="lever_grip",
    )
    lever.visual(
        Box((0.0060, 0.0045, 0.0022)),
        origin=Origin(xyz=(-0.0030, -0.0020, 0.0018)),
        material=matte_black,
        name="lever_latch_toe",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.012, 0.032, 0.006)),
        mass=0.02,
        origin=Origin(xyz=(0.001, -0.014, 0.003)),
    )

    cover = model.part("m2_cover")
    cover.visual(
        Cylinder(radius=0.0018, length=0.026),
        origin=Origin(xyz=(0.0032, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="cover_barrel",
    )
    cover.visual(
        Box((0.090, 0.024, 0.0014)),
        origin=Origin(xyz=(0.047, 0.0, 0.0002)),
        material=aluminum,
        name="cover_plate",
    )
    cover.visual(
        Box((0.074, 0.020, 0.0005)),
        origin=Origin(xyz=(0.047, 0.0, -0.00075)),
        material=dark_gray,
        name="thermal_pad",
    )
    cover.visual(
        Box((0.010, 0.016, 0.0022)),
        origin=Origin(xyz=(0.089, 0.0, 0.0007)),
        material=aluminum,
        name="cover_tab",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.094, 0.026, 0.003)),
        mass=0.04,
        origin=Origin(xyz=(0.047, 0.0, 0.0008)),
    )

    model.articulation(
        "pcie_slot_to_gpu_release",
        ArticulationType.REVOLUTE,
        parent=board,
        child=lever,
        origin=Origin(xyz=(lever_pivot_x, lever_pivot_y, lever_pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.0,
            lower=0.0,
            upper=0.85,
        ),
    )
    model.articulation(
        "board_to_m2_cover",
        ArticulationType.REVOLUTE,
        parent=board,
        child=cover,
        origin=Origin(xyz=(m2_hinge_x, m2_hinge_y, m2_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=0.0,
            upper=1.55,
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
    lever = object_model.get_part("gpu_release_lever")
    cover = object_model.get_part("m2_cover")
    lever_joint = object_model.get_articulation("pcie_slot_to_gpu_release")
    cover_joint = object_model.get_articulation("board_to_m2_cover")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

    ctx.expect_gap(
        lever,
        board,
        axis="z",
        positive_elem="lever_pivot_collar",
        negative_elem="pcie_lever_pivot",
        max_gap=0.0002,
        max_penetration=0.0,
        name="GPU release lever sits on its pivot pedestal",
    )
    ctx.expect_gap(
        cover,
        board,
        axis="z",
        positive_elem="cover_barrel",
        negative_elem="m2_hinge_shelf",
        max_gap=0.0002,
        max_penetration=1e-6,
        name="M.2 cover barrel is seated on the hinge shelf",
    )
    ctx.expect_overlap(
        cover,
        board,
        axes="xy",
        elem_a="cover_plate",
        elem_b="m2_drive",
        min_overlap=0.018,
        name="M.2 cover spans over the storage device",
    )
    ctx.expect_gap(
        cover,
        board,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="m2_drive",
        min_gap=0.0004,
        max_gap=0.0015,
        name="Closed M.2 cover clears the drive with a slim gap",
    )

    lever_rest = aabb_center(ctx.part_element_world_aabb(lever, elem="lever_grip"))
    with ctx.pose({lever_joint: 0.75}):
        lever_open = aabb_center(ctx.part_element_world_aabb(lever, elem="lever_grip"))
    ctx.check(
        "GPU release lever swings outward from the PCIe slot end",
        lever_rest is not None
        and lever_open is not None
        and lever_open[0] > lever_rest[0] + 0.012,
        details=f"rest={lever_rest}, open={lever_open}",
    )

    cover_rest = aabb_center(ctx.part_element_world_aabb(cover, elem="cover_tab"))
    with ctx.pose({cover_joint: 1.20}):
        cover_open = aabb_center(ctx.part_element_world_aabb(cover, elem="cover_tab"))
    ctx.check(
        "M.2 cover lifts upward on its hinge",
        cover_rest is not None
        and cover_open is not None
        and cover_open[2] > cover_rest[2] + 0.040,
        details=f"rest={cover_rest}, open={cover_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
