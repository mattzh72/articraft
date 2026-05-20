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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    hw = width * 0.5
    hd = depth * 0.5
    return [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]


def _circle_profile(
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 18,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workstation_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.30, 0.18, 1.0))
    pcb_dark = model.material("pcb_dark", rgba=(0.08, 0.18, 0.12, 1.0))
    socket_black = model.material("socket_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.10, 0.10, 0.11, 1.0))
    blue_slot = model.material("blue_slot", rgba=(0.14, 0.18, 0.32, 1.0))
    latch_gray = model.material("latch_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.74, 0.76, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.40, 0.42, 0.44, 1.0))
    connector_black = model.material("connector_black", rgba=(0.07, 0.07, 0.08, 1.0))
    port_metal = model.material("port_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    pad_gold = model.material("pad_gold", rgba=(0.77, 0.61, 0.18, 1.0))

    board_width = 0.272
    board_length = 0.305
    board_thickness = 0.0024

    board = model.part("board")
    board_outline = [
        (-board_width * 0.5, -board_length * 0.5),
        (board_width * 0.5, -board_length * 0.5),
        (board_width * 0.5, board_length * 0.5),
        (-0.106, board_length * 0.5),
        (-board_width * 0.5, 0.122),
    ]
    board_holes = [
        _circle_profile(0.0033, cx=-0.118, cy=0.136),
        _circle_profile(0.0033, cx=0.119, cy=0.132),
        _circle_profile(0.0033, cx=-0.114, cy=-0.136),
        _circle_profile(0.0033, cx=0.114, cy=-0.136),
        _circle_profile(0.0030, cx=-0.020, cy=0.095),
        _circle_profile(0.0030, cx=0.098, cy=0.078),
        _circle_profile(0.0030, cx=-0.026, cy=-0.044),
        _circle_profile(0.0030, cx=0.085, cy=-0.094),
    ]
    board_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            board_outline,
            board_holes,
            board_thickness,
            center=False,
        ),
        "workstation_pcb",
    )
    board.visual(board_mesh, material=pcb_green, name="pcb")
    board.visual(
        Box((0.160, 0.090, 0.0002)),
        origin=Origin(xyz=(-0.005, 0.070, board_thickness + 0.0001)),
        material=pcb_dark,
        name="cpu_zone_mask",
    )
    board.visual(
        Box((0.112, 0.020, 0.018)),
        origin=Origin(xyz=(-0.010, 0.130, board_thickness + 0.009)),
        material=heatsink_gray,
        name="top_vrm_heatsink",
    )
    board.visual(
        Box((0.018, 0.086, 0.018)),
        origin=Origin(xyz=(-0.070, 0.076, board_thickness + 0.009)),
        material=heatsink_gray,
        name="side_vrm_heatsink",
    )
    board.visual(
        Box((0.042, 0.040, 0.016)),
        origin=Origin(xyz=(0.015, -0.058, board_thickness + 0.008)),
        material=heatsink_gray,
        name="chipset_heatsink",
    )
    board.visual(
        Box((0.086, 0.022, 0.008)),
        origin=Origin(xyz=(-0.016, -0.010, board_thickness + 0.004)),
        material=heatsink_gray,
        name="upper_m2_heatsink",
    )
    board.visual(
        Box((0.070, 0.020, 0.007)),
        origin=Origin(xyz=(-0.022, -0.084, board_thickness + 0.0035)),
        material=heatsink_gray,
        name="lower_m2_heatsink",
    )
    board.visual(
        Box((0.016, 0.060, 0.014)),
        origin=Origin(xyz=(0.122, -0.088, board_thickness + 0.007)),
        material=connector_black,
        name="atx_power_connector",
    )
    board.visual(
        Box((0.024, 0.016, 0.014)),
        origin=Origin(xyz=(0.084, 0.139, board_thickness + 0.007)),
        material=connector_black,
        name="eps_power_connector",
    )
    board.visual(
        Box((0.036, 0.020, 0.022)),
        origin=Origin(xyz=(-0.111, 0.136, board_thickness + 0.011)),
        material=port_metal,
        name="rear_io_block_upper",
    )
    board.visual(
        Box((0.030, 0.018, 0.018)),
        origin=Origin(xyz=(-0.112, 0.112, board_thickness + 0.009)),
        material=port_metal,
        name="rear_io_block_lower",
    )
    board.visual(
        Box((0.025, 0.018, 0.008)),
        origin=Origin(xyz=(-0.107, 0.086, board_thickness + 0.004)),
        material=connector_black,
        name="audio_stack",
    )

    dimm_slot_width = 0.0065
    dimm_slot_length = 0.135
    dimm_slot_height = 0.008
    dimm_x_positions = [0.056 + 0.0080 * index for index in range(8)]
    dimm_slot_center_y = 0.030
    for index, center_x in enumerate(dimm_x_positions):
        slot_material = blue_slot if index % 2 == 0 else dark_slot
        board.visual(
            Box((dimm_slot_width, dimm_slot_length, dimm_slot_height)),
            origin=Origin(
                xyz=(
                    center_x,
                    dimm_slot_center_y,
                    board_thickness + dimm_slot_height * 0.5,
                )
            ),
            material=slot_material,
            name=f"dimm_slot_{index}",
        )
        board.visual(
            Box((dimm_slot_width * 0.70, dimm_slot_length * 0.78, 0.0014)),
            origin=Origin(
                xyz=(
                    center_x,
                    dimm_slot_center_y,
                    board_thickness + 0.0007,
                )
            ),
            material=pad_gold,
            name=f"dimm_contacts_{index}",
        )

    pcie_specs = [
        ("pcie_slot_0", 0.112, -0.020),
        ("pcie_slot_1", 0.090, -0.054),
        ("pcie_slot_2", 0.112, -0.092),
        ("pcie_slot_3", 0.090, -0.126),
    ]
    pcie_slot_height = 0.012
    pcie_slot_width = 0.012
    pcie_center_x = -0.018
    for index, (slot_name, slot_length, slot_y) in enumerate(pcie_specs):
        board.visual(
            Box((slot_length, pcie_slot_width, pcie_slot_height)),
            origin=Origin(
                xyz=(
                    pcie_center_x,
                    slot_y,
                    board_thickness + pcie_slot_height * 0.5,
                )
            ),
            material=dark_slot,
            name=slot_name,
        )
        board.visual(
            Box((slot_length * 0.82, pcie_slot_width * 0.42, 0.0014)),
            origin=Origin(
                xyz=(
                    pcie_center_x - 0.004,
                    slot_y,
                    board_thickness + 0.0007,
                )
            ),
            material=pad_gold,
            name=f"pcie_contacts_{index}",
        )

    socket_body = model.part("socket_body")
    socket_body.visual(
        Box((0.078, 0.082, 0.0066)),
        origin=Origin(xyz=(0.0, 0.0, 0.0033)),
        material=socket_black,
        name="socket_housing",
    )
    socket_body.visual(
        Box((0.070, 0.074, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0, 0.0077)),
        material=dark_slot,
        name="socket_top_frame",
    )
    socket_body.visual(
        Box((0.058, 0.062, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0069)),
        material=pad_gold,
        name="socket_contact_field",
    )
    socket_body.visual(
        Box((0.052, 0.006, 0.0036)),
        origin=Origin(xyz=(0.0, 0.040, 0.0090)),
        material=steel,
        name="hinge_bridge",
    )
    socket_body.visual(
        Cylinder(radius=0.0028, length=0.022),
        origin=Origin(xyz=(0.0, 0.041, 0.0118), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    socket_body.visual(
        Box((0.010, 0.010, 0.004)),
        origin=Origin(xyz=(0.044, -0.028, 0.0045)),
        material=steel,
        name="lever_support",
    )
    socket_body.visual(
        Box((0.006, 0.030, 0.010)),
        origin=Origin(xyz=(0.039, -0.006, 0.005)),
        material=steel,
        name="lever_guard",
    )
    model.articulation(
        "board_to_socket_body",
        ArticulationType.FIXED,
        parent=board,
        child=socket_body,
        origin=Origin(xyz=(-0.014, 0.070, board_thickness)),
    )

    load_plate = model.part("cpu_load_plate")
    load_plate_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(0.072, 0.074),
            [_rect_profile(0.056, 0.058)],
            0.0012,
            center=False,
        ),
        "cpu_load_plate_frame",
    )
    load_plate.visual(
        load_plate_frame_mesh,
        origin=Origin(xyz=(0.0, -0.037, -0.0022)),
        material=steel,
        name="load_plate_frame",
    )
    load_plate.visual(
        Cylinder(radius=0.0030, length=0.010),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="plate_knuckle_left",
    )
    load_plate.visual(
        Cylinder(radius=0.0030, length=0.010),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="plate_knuckle_right",
    )
    load_plate.visual(
        Box((0.010, 0.006, 0.005)),
        origin=Origin(xyz=(-0.018, -0.003, -0.0015)),
        material=steel,
        name="plate_ear_left",
    )
    load_plate.visual(
        Box((0.010, 0.006, 0.005)),
        origin=Origin(xyz=(0.018, -0.003, -0.0015)),
        material=steel,
        name="plate_ear_right",
    )
    model.articulation(
        "socket_to_load_plate",
        ArticulationType.REVOLUTE,
        parent=socket_body,
        child=load_plate,
        origin=Origin(xyz=(0.0, 0.041, 0.0118)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    lever = model.part("socket_lever")
    lever_mesh = mesh_from_geometry(
        wire_from_points(
            [
                (0.0, 0.0, 0.0),
                (0.0, 0.010, 0.001),
                (0.0, 0.050, 0.001),
                (0.0, 0.062, 0.007),
                (0.0, 0.070, 0.013),
            ],
            radius=0.00115,
            radial_segments=12,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.004,
            corner_segments=6,
        ),
        "cpu_socket_lever_wire",
    )
    lever.visual(
        Cylinder(radius=0.0020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lever_pivot_barrel",
    )
    lever.visual(lever_mesh, material=steel, name="lever_wire")
    lever.visual(
        Cylinder(radius=0.0018, length=0.012),
        origin=Origin(xyz=(0.0, 0.067, 0.011), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_gray,
        name="lever_handle",
    )
    model.articulation(
        "socket_to_lever",
        ArticulationType.REVOLUTE,
        parent=socket_body,
        child=lever,
        origin=Origin(xyz=(0.044, -0.028, 0.0085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    dimm_latch_barrel_radius = 0.0012
    dimm_latch_length_y = 0.010
    for index, center_x in enumerate(dimm_x_positions):
        latch = model.part(f"dimm_latch_{index}")
        latch.visual(
            Cylinder(radius=dimm_latch_barrel_radius, length=0.0060),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=latch_gray,
            name="dimm_latch_barrel",
        )
        latch.visual(
            Box((0.0056, 0.0120, 0.0084)),
            origin=Origin(xyz=(0.0, 0.0060, 0.0042)),
            material=latch_gray,
            name="dimm_latch_body",
        )
        latch.visual(
            Box((0.0042, 0.0060, 0.0030)),
            origin=Origin(xyz=(0.0, 0.0090, 0.0014)),
            material=latch_gray,
            name="dimm_latch_toe",
        )
        latch.visual(
            Box((0.0048, 0.0024, 0.0012)),
            origin=Origin(xyz=(0.0, 0.0, -0.0006)),
            material=latch_gray,
            name="dimm_mount_lug",
        )
        model.articulation(
            f"board_to_dimm_latch_{index}",
            ArticulationType.REVOLUTE,
            parent=board,
            child=latch,
            origin=Origin(
                xyz=(
                    center_x,
                    dimm_slot_center_y + dimm_slot_length * 0.5 + 0.0012,
                    board_thickness + dimm_slot_height + dimm_latch_barrel_radius,
                )
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=4.0,
                lower=0.0,
                upper=math.radians(70.0),
            ),
        )

    pcie_latch_barrel_radius = 0.0014
    for index, (_, slot_length, slot_y) in enumerate(pcie_specs):
        latch = model.part(f"pcie_latch_{index}")
        latch.visual(
            Cylinder(radius=pcie_latch_barrel_radius, length=0.0090),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=latch_gray,
            name="pcie_latch_barrel",
        )
        latch.visual(
            Box((0.0105, 0.0048, 0.012)),
            origin=Origin(xyz=(-0.0052, 0.0, 0.0060)),
            material=latch_gray,
            name="pcie_latch_body",
        )
        latch.visual(
            Box((0.0040, 0.0048, 0.0040)),
            origin=Origin(xyz=(-0.0090, 0.0, 0.0020)),
            material=latch_gray,
            name="pcie_latch_hook",
        )
        latch.visual(
            Box((0.0050, 0.0048, 0.0014)),
            origin=Origin(xyz=(-0.0025, 0.0, -0.0007)),
            material=latch_gray,
            name="pcie_mount_tab",
        )
        model.articulation(
            f"board_to_pcie_latch_{index}",
            ArticulationType.REVOLUTE,
            parent=board,
            child=latch,
            origin=Origin(
                xyz=(
                    pcie_center_x + slot_length * 0.5 + 0.0050,
                    slot_y,
                    board_thickness + pcie_slot_height + pcie_latch_barrel_radius,
                )
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=4.0,
                lower=0.0,
                upper=math.radians(38.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    board = object_model.get_part("board")
    socket_body = object_model.get_part("socket_body")
    load_plate = object_model.get_part("cpu_load_plate")
    lever = object_model.get_part("socket_lever")

    dimm_latches = [object_model.get_part(f"dimm_latch_{index}") for index in range(8)]
    pcie_latches = [object_model.get_part(f"pcie_latch_{index}") for index in range(4)]

    plate_joint = object_model.get_articulation("socket_to_load_plate")
    lever_joint = object_model.get_articulation("socket_to_lever")
    dimm_joints = [object_model.get_articulation(f"board_to_dimm_latch_{index}") for index in range(8)]
    pcie_joints = [object_model.get_articulation(f"board_to_pcie_latch_{index}") for index in range(4)]

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "expected_part_population",
        len(object_model.parts) == 16,
        details=f"expected 16 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "load_plate_axis",
        tuple(load_plate is not None for _ in [0]) == (True,)
        and tuple(plate_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"unexpected load plate axis {plate_joint.axis}",
    )
    ctx.check(
        "lever_axis",
        tuple(lever_joint.axis) == (1.0, 0.0, 0.0),
        details=f"unexpected lever axis {lever_joint.axis}",
    )
    ctx.check(
        "dimm_latch_axes",
        all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in dimm_joints),
        details="all DIMM latches should rotate about local x-axis hinge lines",
    )
    ctx.check(
        "pcie_latch_axes",
        all(tuple(joint.axis) == (0.0, 1.0, 0.0) for joint in pcie_joints),
        details="all PCIe latches should rotate about local y-axis end pivots",
    )

    ctx.expect_contact(socket_body, board, name="socket_body_contacts_board")
    ctx.expect_within(socket_body, board, axes="xy", margin=0.0, name="socket_body_within_board")
    ctx.expect_overlap(
        load_plate,
        socket_body,
        axes="xy",
        min_overlap=0.050,
        elem_a="load_plate_frame",
        elem_b="socket_housing",
        name="load_plate_covers_socket",
    )
    ctx.expect_gap(
        load_plate,
        socket_body,
        axis="z",
        positive_elem="load_plate_frame",
        negative_elem="socket_top_frame",
        min_gap=0.0001,
        max_gap=0.004,
        name="load_plate_closed_clearance",
    )
    ctx.expect_contact(lever, socket_body, name="lever_contacts_socket_support")

    for index, latch in enumerate(dimm_latches):
        ctx.expect_contact(latch, board, name=f"dimm_latch_{index}_mounted")
        ctx.expect_within(
            latch,
            board,
            axes="xy",
            margin=0.012,
            name=f"dimm_latch_{index}_within_board",
        )

    for index, latch in enumerate(pcie_latches):
        ctx.expect_contact(latch, board, name=f"pcie_latch_{index}_mounted")
        ctx.expect_within(
            latch,
            board,
            axes="xy",
            margin=0.010,
            name=f"pcie_latch_{index}_within_board",
        )

    plate_closed = ctx.part_element_world_aabb(load_plate, elem="load_plate_frame")
    assert plate_closed is not None
    with ctx.pose({plate_joint: math.radians(85.0)}):
        plate_open = ctx.part_element_world_aabb(load_plate, elem="load_plate_frame")
        assert plate_open is not None
        ctx.check(
            "load_plate_opens_upward",
            plate_open[1][2] > plate_closed[1][2] + 0.040,
            details=f"closed max z {plate_closed[1][2]:.4f}, open max z {plate_open[1][2]:.4f}",
        )

    lever_closed = ctx.part_element_world_aabb(lever, elem="lever_wire")
    assert lever_closed is not None
    with ctx.pose({lever_joint: math.radians(95.0)}):
        lever_open = ctx.part_element_world_aabb(lever, elem="lever_wire")
        assert lever_open is not None
        ctx.check(
            "lever_lifts_clear",
            lever_open[1][2] > lever_closed[1][2] + 0.025,
            details=f"closed max z {lever_closed[1][2]:.4f}, open max z {lever_open[1][2]:.4f}",
        )

    rep_dimm = dimm_latches[0]
    rep_dimm_joint = dimm_joints[0]
    dimm_closed = ctx.part_element_world_aabb(rep_dimm, elem="dimm_latch_body")
    assert dimm_closed is not None
    with ctx.pose({rep_dimm_joint: math.radians(60.0)}):
        dimm_open = ctx.part_element_world_aabb(rep_dimm, elem="dimm_latch_body")
        assert dimm_open is not None
        ctx.check(
            "dimm_latch_rotates_open",
            dimm_open[1][2] > dimm_closed[1][2] + 0.004,
            details=f"closed max z {dimm_closed[1][2]:.4f}, open max z {dimm_open[1][2]:.4f}",
        )

    rep_pcie = pcie_latches[0]
    rep_pcie_joint = pcie_joints[0]
    pcie_closed = ctx.part_element_world_aabb(rep_pcie, elem="pcie_latch_body")
    assert pcie_closed is not None
    with ctx.pose({rep_pcie_joint: math.radians(30.0)}):
        pcie_open = ctx.part_element_world_aabb(rep_pcie, elem="pcie_latch_body")
        assert pcie_open is not None
        ctx.check(
            "pcie_latch_rotates_up",
            pcie_open[1][2] > pcie_closed[1][2] + 0.0025
            and pcie_open[1][0] > pcie_closed[1][0] + 0.004,
            details=(
                f"closed max {pcie_closed[1][0]:.4f}, {pcie_closed[1][2]:.4f}; "
                f"open max {pcie_open[1][0]:.4f}, {pcie_open[1][2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
