from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="server_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.13, 0.36, 0.18, 1.0))
    solder_mask_dark = model.material("solder_mask_dark", rgba=(0.10, 0.14, 0.10, 1.0))
    slot_black = model.material("slot_black", rgba=(0.07, 0.07, 0.08, 1.0))
    dimm_green = model.material("dimm_green", rgba=(0.20, 0.48, 0.24, 1.0))
    chip_black = model.material("chip_black", rgba=(0.04, 0.04, 0.04, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    connector_beige = model.material("connector_beige", rgba=(0.82, 0.80, 0.70, 1.0))
    service_blue = model.material("service_blue", rgba=(0.25, 0.45, 0.82, 1.0))

    board_len = 0.56
    board_wid = 0.28
    board_thk = 0.003
    board_top = board_thk

    board = model.part("board")
    board.visual(
        Box((board_len, board_wid, board_thk)),
        origin=Origin(xyz=(0.0, 0.0, board_thk / 2.0)),
        material=pcb_green,
        name="pcb",
    )

    def add_block(
        part,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
        name: str | None = None,
    ) -> None:
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    # CPU socket and surrounding power-delivery details.
    socket_size = 0.082
    socket_height = 0.004
    cpu_hinge_x = -0.037
    board.visual(
        Box((socket_size, socket_size, socket_height)),
        origin=Origin(xyz=(0.0, 0.0, board_top + socket_height / 2.0)),
        material=slot_black,
        name="cpu_socket_frame",
    )
    add_block(
        board,
        (0.062, 0.062, 0.0015),
        (0.0, 0.0, board_top + socket_height + 0.00075),
        brushed_metal,
        name="cpu_socket_contacts",
    )
    add_block(
        board,
        (0.018, 0.110, 0.022),
        (-0.066, 0.0, board_top + 0.011),
        brushed_metal,
        name="left_vrm_sink",
    )
    add_block(
        board,
        (0.018, 0.110, 0.022),
        (0.066, 0.0, board_top + 0.011),
        brushed_metal,
        name="right_vrm_sink",
    )
    add_block(
        board,
        (0.040, 0.040, 0.018),
        (0.185, -0.052, board_top + 0.009),
        brushed_metal,
        name="chipset_sink",
    )
    for x, y in (
        (-0.028, 0.064),
        (-0.028, -0.064),
        (0.028, 0.064),
        (0.028, -0.064),
    ):
        add_block(
            board,
            (0.020, 0.026, 0.006),
            (x, y, board_top + 0.003),
            solder_mask_dark,
        )

    # CPU load-plate hinge support.
    for y in (-0.026, 0.026):
        add_block(
            board,
            (0.006, 0.014, 0.008),
            (cpu_hinge_x - 0.003, y, board_top + 0.004),
            brushed_metal,
        )
    add_block(
        board,
        (0.004, 0.062, 0.004),
        (cpu_hinge_x - 0.001, 0.0, board_top + 0.008),
        brushed_metal,
    )

    # Edge connectors and add-in slots for board silhouette.
    add_block(
        board,
        (0.026, 0.102, 0.034),
        (-0.252, 0.082, board_top + 0.017),
        brushed_metal,
        name="rear_io_stack",
    )
    add_block(
        board,
        (0.070, 0.014, 0.016),
        (0.216, 0.121, board_top + 0.008),
        connector_beige,
        name="power_connector",
    )
    for idx, x in enumerate((-0.170, -0.045, 0.080), start=1):
        add_block(
            board,
            (0.090, 0.010, 0.012),
            (x, -0.112, board_top + 0.006),
            slot_black,
            name=f"pcie_slot_{idx}",
        )

    # DIMM banks around the CPU.
    slot_width = 0.011
    slot_length = 0.178
    slot_height = 0.010
    module_thickness = 0.0035
    module_length = 0.146
    module_height = 0.102
    chip_thickness = 0.0012
    chip_length = 0.022
    chip_height = 0.018
    module_center_z = board_top + slot_height + module_height / 2.0

    def add_latch(slot_name: str, slot_x: float, end_sign: float) -> None:
        end_name = "top" if end_sign > 0.0 else "bottom"
        latch = model.part(f"{slot_name}_{end_name}_latch")
        inward_sign = -1.0 if end_sign > 0.0 else 1.0
        add_block(
            latch,
            (0.010, 0.003, 0.022),
            (0.0, inward_sign * 0.0005, 0.011),
            connector_beige,
            name="arm",
        )
        add_block(
            latch,
            (0.010, 0.004, 0.004),
            (0.0, inward_sign * 0.004, 0.022),
            connector_beige,
            name="hook",
        )

        pivot_y = end_sign * (slot_length / 2.0 + 0.002)
        axis_x = -1.0 if end_sign > 0.0 else 1.0
        model.articulation(
            f"board_to_{slot_name}_{end_name}_latch",
            ArticulationType.REVOLUTE,
            parent=board,
            child=latch,
            origin=Origin(xyz=(slot_x, pivot_y, board_top + slot_height)),
            axis=(axis_x, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=4.0,
                lower=0.0,
                upper=0.85,
            ),
        )

    def add_dimm_slot(slot_name: str, slot_x: float) -> None:
        add_block(
            board,
            (slot_width, slot_length, slot_height),
            (slot_x, 0.0, board_top + slot_height / 2.0),
            slot_black,
            name=f"{slot_name}_housing",
        )
        if slot_name == "left_slot_2":
            board.visual(
                Box((module_thickness, module_length, module_height)),
                origin=Origin(xyz=(slot_x, 0.0, module_center_z)),
                material=dimm_green,
                name="left_slot_2_module",
            )
        else:
            add_block(
                board,
                (module_thickness, module_length, module_height),
                (slot_x, 0.0, module_center_z),
                dimm_green,
                name=f"{slot_name}_module",
            )

        chip_y_positions = (-0.052, -0.026, 0.0, 0.026, 0.052)
        chip_center_z = board_top + slot_height + 0.048
        chip_x_offset = module_thickness / 2.0 + chip_thickness / 2.0
        for side_sign in (-1.0, 1.0):
            for chip_y in chip_y_positions:
                add_block(
                    board,
                    (chip_thickness, chip_length, chip_height),
                    (slot_x + side_sign * chip_x_offset, chip_y, chip_center_z),
                    chip_black,
                )

        add_latch(slot_name, slot_x, 1.0)
        add_latch(slot_name, slot_x, -1.0)

    left_bank_x = (-0.160, -0.140, -0.120, -0.100)
    right_bank_x = (0.100, 0.120, 0.140, 0.160)
    for index, slot_x in enumerate(left_bank_x, start=1):
        add_dimm_slot(f"left_slot_{index}", slot_x)
    for index, slot_x in enumerate(right_bank_x, start=1):
        add_dimm_slot(f"right_slot_{index}", slot_x)

    # Fold-out service tab support over the left DIMM group.
    for y in (-0.076, 0.076):
        add_block(
            board,
            (0.006, 0.014, 0.116),
            (-0.188, y, board_top + 0.058),
            brushed_metal,
        )
    board.visual(
        Box((0.004, 0.168, 0.006)),
        origin=Origin(xyz=(-0.188, 0.0, board_top + 0.113)),
        material=brushed_metal,
        name="service_support_rail",
    )

    cpu_load_plate = model.part("cpu_load_plate")
    cpu_load_plate.visual(
        Box((0.074, 0.074, 0.0015)),
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
        material=brushed_metal,
        name="load_plate_panel",
    )
    add_block(
        cpu_load_plate,
        (0.006, 0.074, 0.005),
        (0.071, 0.0, 0.0025),
        brushed_metal,
        name="load_plate_lip",
    )
    add_block(
        cpu_load_plate,
        (0.028, 0.006, 0.004),
        (0.087, 0.0, 0.0045),
        brushed_metal,
        name="load_plate_handle",
    )
    model.articulation(
        "board_to_cpu_load_plate",
        ArticulationType.REVOLUTE,
        parent=board,
        child=cpu_load_plate,
        origin=Origin(xyz=(cpu_hinge_x, 0.0, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    service_tab = model.part("service_tab")
    service_tab.visual(
        Box((0.088, 0.188, 0.003)),
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        material=service_blue,
        name="service_tab_panel",
    )
    add_block(
        service_tab,
        (0.010, 0.188, 0.008),
        (0.083, 0.0, 0.0055),
        service_blue,
        name="service_tab_grip",
    )
    model.articulation(
        "board_to_service_tab",
        ArticulationType.REVOLUTE,
        parent=board,
        child=service_tab,
        origin=Origin(xyz=(-0.188, 0.0, 0.1205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=3.0,
            lower=0.0,
            upper=1.30,
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
    cpu_load_plate = object_model.get_part("cpu_load_plate")
    service_tab = object_model.get_part("service_tab")
    cpu_hinge = object_model.get_articulation("board_to_cpu_load_plate")
    service_hinge = object_model.get_articulation("board_to_service_tab")

    left_top_latch = object_model.get_part("left_slot_1_top_latch")
    left_bottom_latch = object_model.get_part("left_slot_1_bottom_latch")
    left_top_joint = object_model.get_articulation("board_to_left_slot_1_top_latch")
    left_bottom_joint = object_model.get_articulation("board_to_left_slot_1_bottom_latch")

    ctx.expect_overlap(
        cpu_load_plate,
        board,
        axes="xy",
        elem_a="load_plate_panel",
        elem_b="cpu_socket_frame",
        min_overlap=0.070,
        name="load plate covers the CPU socket",
    )
    ctx.expect_overlap(
        service_tab,
        board,
        axes="y",
        elem_a="service_tab_panel",
        elem_b="left_slot_2_module",
        min_overlap=0.140,
        name="service tab sits over the left memory bank",
    )
    ctx.expect_contact(
        service_tab,
        board,
        elem_b="service_support_rail",
        contact_tol=0.0001,
        name="service tab rests on the support rail when closed",
    )
    ctx.expect_gap(
        service_tab,
        board,
        axis="z",
        positive_elem="service_tab_panel",
        negative_elem="left_slot_2_module",
        min_gap=0.003,
        max_gap=0.010,
        name="service tab clears the DIMMs when closed",
    )

    for side in ("left", "right"):
        for idx in range(1, 5):
            for end in ("top", "bottom"):
                latch = object_model.get_part(f"{side}_slot_{idx}_{end}_latch")
                ctx.expect_contact(
                    latch,
                    board,
                    contact_tol=0.0005,
                    name=f"{side} slot {idx} {end} latch is seated on its slot pivot",
                )

    closed_plate_aabb = ctx.part_world_aabb(cpu_load_plate)
    with ctx.pose({cpu_hinge: 1.20}):
        open_plate_aabb = ctx.part_world_aabb(cpu_load_plate)
    ctx.check(
        "CPU load plate rotates upward",
        closed_plate_aabb is not None
        and open_plate_aabb is not None
        and open_plate_aabb[1][2] > closed_plate_aabb[1][2] + 0.045,
        details=f"closed={closed_plate_aabb}, open={open_plate_aabb}",
    )

    closed_tab_aabb = ctx.part_world_aabb(service_tab)
    with ctx.pose({service_hinge: 1.10}):
        open_tab_aabb = ctx.part_world_aabb(service_tab)
    ctx.check(
        "service tab folds upward",
        closed_tab_aabb is not None
        and open_tab_aabb is not None
        and open_tab_aabb[1][2] > closed_tab_aabb[1][2] + 0.050,
        details=f"closed={closed_tab_aabb}, open={open_tab_aabb}",
    )

    closed_top_aabb = ctx.part_world_aabb(left_top_latch)
    with ctx.pose({left_top_joint: 0.75}):
        open_top_aabb = ctx.part_world_aabb(left_top_latch)
    ctx.check(
        "top DIMM latch opens outward from the slot",
        closed_top_aabb is not None
        and open_top_aabb is not None
        and open_top_aabb[1][1] > closed_top_aabb[1][1] + 0.004,
        details=f"closed={closed_top_aabb}, open={open_top_aabb}",
    )

    closed_bottom_aabb = ctx.part_world_aabb(left_bottom_latch)
    with ctx.pose({left_bottom_joint: 0.75}):
        open_bottom_aabb = ctx.part_world_aabb(left_bottom_latch)
    ctx.check(
        "bottom DIMM latch opens outward from the slot",
        closed_bottom_aabb is not None
        and open_bottom_aabb is not None
        and open_bottom_aabb[0][1] < closed_bottom_aabb[0][1] - 0.004,
        details=f"closed={closed_bottom_aabb}, open={open_bottom_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
