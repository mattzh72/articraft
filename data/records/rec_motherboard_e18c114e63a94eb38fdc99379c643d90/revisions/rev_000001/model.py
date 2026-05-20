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
    model = ArticulatedObject(name="enthusiast_motherboard")

    board_w = 0.305
    board_d = 0.244
    board_t = 0.0024
    board_top = board_t

    pcb = model.material("pcb_black", rgba=(0.08, 0.10, 0.09, 1.0))
    slot_black = model.material("slot_black", rgba=(0.14, 0.14, 0.16, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.23, 0.24, 0.27, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    smoked_cover = model.material("smoked_cover", rgba=(0.12, 0.12, 0.13, 1.0))
    accent = model.material("accent", rgba=(0.78, 0.60, 0.18, 1.0))

    board = model.part("motherboard")
    board.visual(
        Box((board_w, board_d, board_t)),
        origin=Origin(xyz=(0.0, 0.0, board_t / 2.0)),
        material=pcb,
        name="pcb",
    )

    # Rear I/O armor and power-delivery heatsinks.
    board.visual(
        Box((0.052, 0.052, 0.030)),
        origin=Origin(xyz=(-0.122, 0.090, board_top + 0.015)),
        material=dark_metal,
        name="rear_io_shroud",
    )
    board.visual(
        Box((0.110, 0.032, 0.024)),
        origin=Origin(xyz=(-0.012, 0.094, board_top + 0.012)),
        material=dark_metal,
        name="vrm_heatsink_top",
    )
    board.visual(
        Box((0.030, 0.090, 0.022)),
        origin=Origin(xyz=(-0.073, 0.028, board_top + 0.011)),
        material=dark_metal,
        name="vrm_heatsink_side",
    )

    # CPU socket stack.
    board.visual(
        Box((0.078, 0.078, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, board_top + 0.003)),
        material=dark_metal,
        name="cpu_socket_base",
    )
    board.visual(
        Box((0.056, 0.056, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, board_top + 0.0068)),
        material=accent,
        name="cpu_socket_pad",
    )
    board.visual(
        Box((0.005, 0.074, 0.004)),
        origin=Origin(xyz=(-0.0365, 0.0, board_top + 0.008)),
        material=brushed_metal,
        name="cpu_socket_left_frame",
    )
    board.visual(
        Box((0.005, 0.074, 0.004)),
        origin=Origin(xyz=(0.0365, 0.0, board_top + 0.008)),
        material=brushed_metal,
        name="cpu_socket_right_frame",
    )
    board.visual(
        Box((0.068, 0.005, 0.004)),
        origin=Origin(xyz=(0.0, 0.0345, board_top + 0.008)),
        material=brushed_metal,
        name="cpu_socket_top_frame",
    )
    board.visual(
        Box((0.068, 0.005, 0.004)),
        origin=Origin(xyz=(0.0, -0.0345, board_top + 0.008)),
        material=brushed_metal,
        name="cpu_socket_bottom_frame",
    )
    board.visual(
        Box((0.014, 0.024, 0.012)),
        origin=Origin(xyz=(-0.0415, 0.0, board_top + 0.006)),
        material=brushed_metal,
        name="cpu_plate_hinge_block",
    )

    # DIMM banks.
    dimm_slot_xs = (0.060, 0.0695, 0.079, 0.0885)
    dimm_slot_y = 0.005
    dimm_slot_len = 0.135
    dimm_slot_w = 0.0068
    dimm_slot_h = 0.008
    for idx, x in enumerate(dimm_slot_xs, start=1):
        board.visual(
            Box((dimm_slot_w, dimm_slot_len, dimm_slot_h)),
            origin=Origin(xyz=(x, dimm_slot_y, board_top + dimm_slot_h / 2.0)),
            material=slot_black,
            name=f"dimm_slot_{idx}",
        )

    # PCIe area, storage covers, and edge connectors.
    board.visual(
        Box((0.118, 0.009, 0.013)),
        origin=Origin(xyz=(0.000, -0.038, board_top + 0.0065)),
        material=slot_black,
        name="pcie_slot_primary",
    )
    board.visual(
        Box((0.104, 0.009, 0.013)),
        origin=Origin(xyz=(0.004, -0.066, board_top + 0.0065)),
        material=slot_black,
        name="pcie_slot_secondary",
    )
    board.visual(
        Box((0.118, 0.009, 0.013)),
        origin=Origin(xyz=(0.000, -0.094, board_top + 0.0065)),
        material=slot_black,
        name="pcie_slot_tertiary",
    )
    board.visual(
        Box((0.086, 0.024, 0.006)),
        origin=Origin(xyz=(0.006, -0.013, board_top + 0.003)),
        material=dark_metal,
        name="m2_shield_upper",
    )
    board.visual(
        Box((0.082, 0.024, 0.006)),
        origin=Origin(xyz=(0.012, -0.118, board_top + 0.003)),
        material=dark_metal,
        name="m2_shield_lower",
    )
    board.visual(
        Box((0.060, 0.055, 0.020)),
        origin=Origin(xyz=(0.088, -0.084, board_top + 0.010)),
        material=dark_metal,
        name="chipset_heatsink",
    )
    board.visual(
        Box((0.008, 0.055, 0.016)),
        origin=Origin(xyz=(0.1485, 0.042, board_top + 0.008)),
        material=slot_black,
        name="atx_power_connector",
    )
    board.visual(
        Box((0.030, 0.010, 0.014)),
        origin=Origin(xyz=(-0.108, 0.117, board_top + 0.007)),
        material=slot_black,
        name="eps_power_connector",
    )
    board.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(-0.104, -0.072, board_top + 0.002)),
        material=brushed_metal,
        name="cmos_battery",
    )

    # Debug display nest near the upper edge.
    board.visual(
        Box((0.060, 0.030, 0.004)),
        origin=Origin(xyz=(0.092, 0.096, board_top + 0.002)),
        material=dark_metal,
        name="diagnostic_housing",
    )
    board.visual(
        Box((0.046, 0.016, 0.002)),
        origin=Origin(xyz=(0.092, 0.096, board_top + 0.003)),
        material=accent,
        name="diagnostic_display",
    )
    board.visual(
        Box((0.010, 0.010, 0.007)),
        origin=Origin(xyz=(0.068, 0.111, board_top + 0.0035)),
        material=dark_metal,
        name="diagnostic_hinge_left",
    )
    board.visual(
        Box((0.010, 0.010, 0.007)),
        origin=Origin(xyz=(0.116, 0.111, board_top + 0.0035)),
        material=dark_metal,
        name="diagnostic_hinge_right",
    )

    cpu_load_plate = model.part("cpu_load_plate")
    cpu_load_plate.visual(
        Box((0.005, 0.060, 0.0015)),
        origin=Origin(xyz=(0.0025, 0.0, 0.0)),
        material=brushed_metal,
        name="plate_left_rail",
    )
    cpu_load_plate.visual(
        Box((0.005, 0.060, 0.0015)),
        origin=Origin(xyz=(0.0675, 0.0, 0.0)),
        material=brushed_metal,
        name="plate_right_rail",
    )
    cpu_load_plate.visual(
        Box((0.070, 0.005, 0.0015)),
        origin=Origin(xyz=(0.035, 0.0275, 0.0)),
        material=brushed_metal,
        name="plate_top_rail",
    )
    cpu_load_plate.visual(
        Box((0.070, 0.005, 0.0015)),
        origin=Origin(xyz=(0.035, -0.0275, 0.0)),
        material=brushed_metal,
        name="plate_bottom_rail",
    )
    cpu_load_plate.visual(
        Cylinder(radius=0.002, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="plate_hinge_barrel",
    )
    cpu_load_plate.visual(
        Box((0.012, 0.010, 0.0015)),
        origin=Origin(xyz=(0.076, 0.0, 0.0)),
        material=brushed_metal,
        name="plate_finger_tab",
    )

    model.articulation(
        "cpu_load_plate_hinge",
        ArticulationType.REVOLUTE,
        parent=board,
        child=cpu_load_plate,
        origin=Origin(xyz=(-0.0365, 0.0, board_top + 0.0132)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    diagnostic_panel = model.part("diagnostic_panel")
    diagnostic_panel.visual(
        Box((0.058, 0.028, 0.002)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=smoked_cover,
        name="panel_cover",
    )
    diagnostic_panel.visual(
        Box((0.050, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.024, 0.001)),
        material=smoked_cover,
        name="panel_lip",
    )
    diagnostic_panel.visual(
        Cylinder(radius=0.0016, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="panel_hinge_barrel",
    )

    model.articulation(
        "diagnostic_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=board,
        child=diagnostic_panel,
        origin=Origin(xyz=(0.092, 0.111, board_top + 0.0083)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=0.0, upper=1.5),
    )

    latch_height = 0.018
    latch_w = 0.008
    latch_t = 0.003
    latch_z = board_top + dimm_slot_h

    for idx, x in enumerate(dimm_slot_xs, start=1):
        for side_name, direction, axis in (
            ("top", 1.0, (-1.0, 0.0, 0.0)),
            ("bottom", -1.0, (1.0, 0.0, 0.0)),
        ):
            latch = model.part(f"dimm{idx}_{side_name}_latch")
            latch.visual(
                Box((latch_w, latch_t, latch_height)),
                origin=Origin(xyz=(0.0, direction * latch_t / 2.0, latch_height / 2.0)),
                material=slot_black,
                name="latch_body",
            )
            latch.visual(
                Box((0.0035, 0.006, 0.004)),
                origin=Origin(xyz=(0.0, -direction * 0.001, 0.016)),
                material=slot_black,
                name="latch_hook",
            )

            model.articulation(
                f"dimm{idx}_{side_name}_latch_hinge",
                ArticulationType.REVOLUTE,
                parent=board,
                child=latch,
                origin=Origin(
                    xyz=(
                        x,
                        dimm_slot_y + direction * (dimm_slot_len / 2.0),
                        latch_z,
                    )
                ),
                axis=axis,
                motion_limits=MotionLimits(
                    effort=0.4,
                    velocity=4.0,
                    lower=0.0,
                    upper=1.05,
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
    cpu_load_plate = object_model.get_part("cpu_load_plate")
    diagnostic_panel = object_model.get_part("diagnostic_panel")
    cpu_hinge = object_model.get_articulation("cpu_load_plate_hinge")
    panel_hinge = object_model.get_articulation("diagnostic_panel_hinge")

    dimm_latches = []
    for idx in range(1, 5):
        for side in ("top", "bottom"):
            latch = object_model.get_part(f"dimm{idx}_{side}_latch")
            dimm_latches.append((idx, side, latch))

    with ctx.pose({cpu_hinge: 0.0, panel_hinge: 0.0}):
        ctx.expect_overlap(
            cpu_load_plate,
            board,
            axes="xy",
            elem_b="cpu_socket_pad",
            min_overlap=0.050,
            name="cpu load plate covers the socket opening",
        )
        ctx.expect_gap(
            cpu_load_plate,
            board,
            axis="z",
            negative_elem="cpu_socket_top_frame",
            max_gap=0.002,
            max_penetration=0.0,
            name="cpu load plate rests just above the socket frame",
        )
        ctx.expect_overlap(
            diagnostic_panel,
            board,
            axes="xy",
            elem_b="diagnostic_display",
            min_overlap=0.014,
            name="diagnostic panel covers the debug display",
        )
        ctx.expect_gap(
            diagnostic_panel,
            board,
            axis="z",
            negative_elem="diagnostic_housing",
            max_gap=0.003,
            max_penetration=0.0,
            name="diagnostic panel clears the display housing when closed",
        )

        for idx, side, latch in dimm_latches:
            ctx.expect_overlap(
                latch,
                board,
                axes="x",
                elem_b=f"dimm_slot_{idx}",
                min_overlap=0.005,
                name=f"dimm {idx} {side} latch stays centered on its slot",
            )
            ctx.expect_gap(
                latch,
                board,
                axis="z",
                negative_elem=f"dimm_slot_{idx}",
                max_gap=0.001,
                max_penetration=0.0,
                name=f"dimm {idx} {side} latch is mounted at the slot top",
            )

    load_plate_rest = ctx.part_world_aabb(cpu_load_plate)
    panel_rest = ctx.part_world_aabb(diagnostic_panel)
    top_latch_rest = ctx.part_world_aabb(object_model.get_part("dimm1_top_latch"))
    bottom_latch_rest = ctx.part_world_aabb(object_model.get_part("dimm1_bottom_latch"))

    with ctx.pose({cpu_hinge: 1.2}):
        load_plate_open = ctx.part_world_aabb(cpu_load_plate)
    ctx.check(
        "cpu load plate opens upward",
        load_plate_rest is not None
        and load_plate_open is not None
        and load_plate_open[1][2] > load_plate_rest[1][2] + 0.045,
        details=f"rest={load_plate_rest}, open={load_plate_open}",
    )

    with ctx.pose({panel_hinge: 1.15}):
        panel_open = ctx.part_world_aabb(diagnostic_panel)
    ctx.check(
        "diagnostic panel flips upward",
        panel_rest is not None
        and panel_open is not None
        and panel_open[1][2] > panel_rest[1][2] + 0.020,
        details=f"rest={panel_rest}, open={panel_open}",
    )

    with ctx.pose(dimm1_top_latch_hinge=0.85):
        top_latch_open = ctx.part_world_aabb(object_model.get_part("dimm1_top_latch"))
    ctx.check(
        "top dimm latch opens outward toward the board edge",
        top_latch_rest is not None
        and top_latch_open is not None
        and top_latch_open[1][1] > top_latch_rest[1][1] + 0.004,
        details=f"rest={top_latch_rest}, open={top_latch_open}",
    )

    with ctx.pose(dimm1_bottom_latch_hinge=0.85):
        bottom_latch_open = ctx.part_world_aabb(object_model.get_part("dimm1_bottom_latch"))
    ctx.check(
        "bottom dimm latch opens outward away from the slot bank",
        bottom_latch_rest is not None
        and bottom_latch_open is not None
        and bottom_latch_open[0][1] < bottom_latch_rest[0][1] - 0.004,
        details=f"rest={bottom_latch_rest}, open={bottom_latch_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
