from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BOARD_SIZE = 0.170
BOARD_THICKNESS = 0.0016
BOARD_TOP = BOARD_THICKNESS


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _finned_heatsink_mesh(
    name: str,
    *,
    size_x: float,
    size_y: float,
    base_thickness: float,
    fin_height: float,
    fin_count: int,
) -> object:
    geom = BoxGeometry((size_x, size_y, base_thickness)).translate(0.0, 0.0, base_thickness * 0.5)
    fin_thickness = size_x / (fin_count * 2.0 + 1.0)
    first_x = -0.5 * size_x + fin_thickness
    step = (size_x - 2.0 * fin_thickness) / max(fin_count - 1, 1)
    for index in range(fin_count):
        x = first_x + index * step
        geom.merge(
            BoxGeometry((fin_thickness, size_y * 0.88, fin_height)).translate(
                x,
                0.0,
                base_thickness + fin_height * 0.5,
            )
        )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_motherboard")

    pcb_black = model.material("pcb_black", rgba=(0.12, 0.14, 0.13, 1.0))
    pcb_accent = model.material("pcb_accent", rgba=(0.19, 0.23, 0.20, 1.0))
    socket_gray = model.material("socket_gray", rgba=(0.65, 0.67, 0.69, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.09, 0.10, 0.11, 1.0))
    latch_gray = model.material("latch_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.32, 0.35, 0.38, 1.0))
    port_black = model.material("port_black", rgba=(0.08, 0.08, 0.09, 1.0))
    port_metal = model.material("port_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    connector_white = model.material("connector_white", rgba=(0.85, 0.86, 0.83, 1.0))
    card_green = model.material("card_green", rgba=(0.16, 0.34, 0.22, 1.0))
    chipset_label = model.material("chipset_label", rgba=(0.62, 0.72, 0.66, 1.0))

    vrm_heatsink_mesh = _finned_heatsink_mesh(
        "vrm_heatsink",
        size_x=0.040,
        size_y=0.013,
        base_thickness=0.0028,
        fin_height=0.0082,
        fin_count=5,
    )
    chipset_heatsink_mesh = _finned_heatsink_mesh(
        "chipset_heatsink",
        size_x=0.024,
        size_y=0.024,
        base_thickness=0.0030,
        fin_height=0.0105,
        fin_count=6,
    )

    board = model.part("motherboard")
    board.visual(
        Box((BOARD_SIZE, BOARD_SIZE, BOARD_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_THICKNESS * 0.5)),
        material=pcb_black,
        name="pcb",
    )
    board.visual(
        Box((0.142, 0.013, 0.030)),
        origin=Origin(xyz=(-0.004, 0.0785, BOARD_TOP + 0.015)),
        material=steel,
        name="rear_io_shroud",
    )
    board.visual(
        Box((0.018, 0.012, 0.018)),
        origin=Origin(xyz=(-0.063, 0.0850, BOARD_TOP + 0.009)),
        material=port_metal,
        name="rear_usb_stack",
    )
    board.visual(
        Box((0.016, 0.012, 0.016)),
        origin=Origin(xyz=(-0.043, 0.0850, BOARD_TOP + 0.008)),
        material=port_metal,
        name="rear_lan_port",
    )
    board.visual(
        Box((0.019, 0.012, 0.014)),
        origin=Origin(xyz=(-0.020, 0.0850, BOARD_TOP + 0.007)),
        material=port_metal,
        name="rear_display_ports",
    )
    board.visual(
        Box((0.016, 0.012, 0.015)),
        origin=Origin(xyz=(0.006, 0.0850, BOARD_TOP + 0.0075)),
        material=port_black,
        name="rear_audio_block",
    )
    board.visual(
        Box((0.018, 0.016, 0.014)),
        origin=Origin(xyz=(-0.059, 0.066, BOARD_TOP + 0.007)),
        material=connector_white,
        name="eps_power_header",
    )
    board.visual(
        vrm_heatsink_mesh,
        origin=Origin(xyz=(-0.036, 0.048, BOARD_TOP)),
        material=heatsink_gray,
        name="vrm_heatsink",
    )
    board.visual(
        Box((0.045, 0.045, 0.0042)),
        origin=Origin(xyz=(-0.014, 0.008, BOARD_TOP + 0.0021)),
        material=socket_gray,
        name="cpu_socket_base",
    )
    board.visual(
        Box((0.037, 0.037, 0.0012)),
        origin=Origin(xyz=(-0.014, 0.008, BOARD_TOP + 0.0050)),
        material=steel,
        name="cpu_retention_plate",
    )
    board.visual(
        Box((0.003, 0.026, 0.0020)),
        origin=Origin(xyz=(0.006, 0.008, BOARD_TOP + 0.0054)),
        material=steel,
        name="cpu_retention_arm",
    )
    board.visual(
        Box((0.014, 0.040, 0.016)),
        origin=Origin(xyz=(0.075, 0.014, BOARD_TOP + 0.008)),
        material=connector_white,
        name="atx_power_header",
    )

    dimm_slot_centers = (0.044, 0.055)
    dimm_slot_center_y = 0.007
    dimm_slot_length = 0.066
    dimm_slot_width = 0.0076
    dimm_slot_body_height = 0.0044
    dimm_rail_height = 0.0080
    rail_offset = 0.0031
    for slot_index, slot_x in enumerate(dimm_slot_centers, start=1):
        prefix = f"dimm_slot_{slot_index}"
        board.visual(
            Box((dimm_slot_width, dimm_slot_length, dimm_slot_body_height)),
            origin=Origin(
                xyz=(
                    slot_x,
                    dimm_slot_center_y,
                    BOARD_TOP + dimm_slot_body_height * 0.5,
                )
            ),
            material=dark_slot,
            name=f"{prefix}_body",
        )
        board.visual(
            Box((0.0014, dimm_slot_length, dimm_rail_height)),
            origin=Origin(
                xyz=(
                    slot_x - rail_offset,
                    dimm_slot_center_y,
                    BOARD_TOP + dimm_rail_height * 0.5,
                )
            ),
            material=dark_slot,
            name=f"{prefix}_left_rail",
        )
        board.visual(
            Box((0.0014, dimm_slot_length, dimm_rail_height)),
            origin=Origin(
                xyz=(
                    slot_x + rail_offset,
                    dimm_slot_center_y,
                    BOARD_TOP + dimm_rail_height * 0.5,
                )
            ),
            material=dark_slot,
            name=f"{prefix}_right_rail",
        )
        board.visual(
            Box((0.0088, 0.006, 0.010)),
            origin=Origin(
                xyz=(
                    slot_x,
                    -0.029,
                    BOARD_TOP + 0.005,
                )
            ),
            material=latch_gray,
            name=f"{prefix}_fixed_tab",
        )

    board.visual(
        Box((0.082, 0.0115, 0.0088)),
        origin=Origin(xyz=(-0.001, -0.051, BOARD_TOP + 0.0044)),
        material=dark_slot,
        name="pcie_slot_body",
    )
    board.visual(
        Box((0.076, 0.0030, 0.0020)),
        origin=Origin(xyz=(-0.003, -0.051, BOARD_TOP + 0.0098)),
        material=port_black,
        name="pcie_slot_opening",
    )
    board.visual(
        Box((0.0050, 0.0115, 0.0100)),
        origin=Origin(xyz=(0.0415, -0.051, BOARD_TOP + 0.0050)),
        material=latch_gray,
        name="pcie_slot_end_block",
    )

    board.visual(
        chipset_heatsink_mesh,
        origin=Origin(xyz=(-0.028, -0.031, BOARD_TOP)),
        material=heatsink_gray,
        name="chipset_heatsink",
    )
    board.visual(
        Box((0.015, 0.010, 0.0010)),
        origin=Origin(xyz=(-0.028, -0.031, BOARD_TOP + 0.0135)),
        material=chipset_label,
        name="chipset_badge",
    )

    board.visual(
        Box((0.0075, 0.025, 0.0040)),
        origin=Origin(xyz=(-0.020, -0.006, BOARD_TOP + 0.0020)),
        material=connector_white,
        name="m2_socket",
    )
    board.visual(
        Cylinder(radius=0.0018, length=0.0026),
        origin=Origin(xyz=(0.0545, -0.006, BOARD_TOP + 0.0013)),
        material=steel,
        name="m2_standoff",
    )
    board.visual(
        Cylinder(radius=0.00075, length=0.0010),
        origin=Origin(xyz=(0.0630, -0.006, BOARD_TOP + 0.0011)),
        material=steel,
        name="m2_pivot_cap",
    )
    board.visual(
        Cylinder(radius=0.00045, length=0.0012),
        origin=Origin(xyz=(0.0630, -0.006, BOARD_TOP + 0.0006)),
        material=steel,
        name="m2_pivot_stem",
    )
    board.visual(
        Box((0.075, 0.022, 0.0012)),
        origin=Origin(xyz=(0.0180, -0.006, BOARD_TOP + 0.0032)),
        material=card_green,
        name="m2_card",
    )
    board.visual(
        Box((0.028, 0.006, 0.0003)),
        origin=Origin(xyz=(0.015, -0.006, BOARD_TOP + 0.00395)),
        material=pcb_accent,
        name="m2_label_strip",
    )

    for cap_index, x_pos in enumerate((-0.061, -0.051, -0.041, -0.031), start=1):
        board.visual(
            Cylinder(radius=0.0032, length=0.0090),
            origin=Origin(xyz=(x_pos, -0.007, BOARD_TOP + 0.0045)),
            material=port_black,
            name=f"audio_cap_{cap_index}",
        )

    for phase_index, (x_pos, y_pos) in enumerate(
        (
            (-0.050, 0.026),
            (-0.036, 0.026),
            (-0.022, 0.026),
            (0.000, 0.034),
            (0.014, 0.034),
            (0.028, 0.034),
        ),
        start=1,
    ):
        board.visual(
            Box((0.009, 0.009, 0.004)),
            origin=Origin(xyz=(x_pos, y_pos, BOARD_TOP + 0.002)),
            material=heatsink_gray,
            name=f"vrm_choke_{phase_index}",
        )

    board.inertial = Inertial.from_geometry(
        Box((0.190, 0.190, 0.040)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    dimm_barrel_radius = 0.00125
    dimm_latch_dims = Box((0.010, 0.007, 0.015))
    for latch_name in ("a", "b"):
        latch = model.part(f"dimm_latch_{latch_name}")
        latch.visual(
            Cylinder(radius=dimm_barrel_radius, length=0.010),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=latch_gray,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.0086, 0.0020, 0.0070)),
            origin=Origin(xyz=(0.0, -0.0008, 0.0047)),
            material=latch_gray,
            name="latch_post",
        )
        latch.visual(
            Box((0.0086, 0.0028, 0.0050)),
            origin=Origin(xyz=(0.0, -0.0028, 0.0105)),
            material=latch_gray,
            name="latch_body",
        )
        latch.visual(
            Box((0.0086, 0.0014, 0.0030)),
            origin=Origin(xyz=(0.0, -0.0047, 0.0120)),
            material=latch_gray,
            name="latch_hook",
        )
        latch.inertial = Inertial.from_geometry(
            dimm_latch_dims,
            mass=0.004,
            origin=Origin(xyz=(0.0, -0.0020, 0.0075)),
        )

    pcie_latch = model.part("pcie_latch")
    pcie_barrel_radius = 0.00110
    pcie_latch.visual(
        Cylinder(radius=pcie_barrel_radius, length=0.006),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=latch_gray,
        name="pivot_barrel",
    )
    pcie_latch.visual(
        Box((0.0018, 0.0048, 0.0076)),
        origin=Origin(xyz=(-0.0010, 0.0, 0.0048)),
        material=latch_gray,
        name="latch_post",
    )
    pcie_latch.visual(
        Box((0.0050, 0.0048, 0.0040)),
        origin=Origin(xyz=(-0.0042, 0.0, 0.0103)),
        material=latch_gray,
        name="latch_body",
    )
    pcie_latch.visual(
        Box((0.0022, 0.0048, 0.0030)),
        origin=Origin(xyz=(-0.0077, 0.0, 0.0116)),
        material=latch_gray,
        name="latch_hook",
    )
    pcie_latch.inertial = Inertial.from_geometry(
        Box((0.010, 0.006, 0.015)),
        mass=0.003,
        origin=Origin(xyz=(-0.0038, 0.0, 0.0075)),
    )

    m2_lever = model.part("m2_retention_lever")
    m2_lever.visual(
        Cylinder(radius=0.0012, length=0.0006),
        origin=Origin(xyz=(0.0, 0.0, 0.0003)),
        material=latch_gray,
        name="pivot_collar",
    )
    m2_lever.visual(
        Box((0.0074, 0.0016, 0.0008)),
        origin=Origin(xyz=(-0.0046, 0.0, 0.0007)),
        material=latch_gray,
        name="lever_arm",
    )
    m2_lever.visual(
        Box((0.0020, 0.0042, 0.0008)),
        origin=Origin(xyz=(-0.0085, 0.0, 0.0007)),
        material=latch_gray,
        name="lever_head",
    )
    m2_lever.inertial = Inertial.from_geometry(
        Box((0.011, 0.005, 0.0016)),
        mass=0.001,
        origin=Origin(xyz=(-0.0048, 0.0, 0.0007)),
    )

    model.articulation(
        "dimm_slot_a_latch_joint",
        ArticulationType.REVOLUTE,
        parent=board,
        child="dimm_latch_a",
        origin=Origin(xyz=(dimm_slot_centers[0], 0.0460, BOARD_TOP + dimm_barrel_radius)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=3.0,
            lower=0.0,
            upper=1.00,
        ),
    )
    model.articulation(
        "dimm_slot_b_latch_joint",
        ArticulationType.REVOLUTE,
        parent=board,
        child="dimm_latch_b",
        origin=Origin(xyz=(dimm_slot_centers[1], 0.0460, BOARD_TOP + dimm_barrel_radius)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=3.0,
            lower=0.0,
            upper=1.00,
        ),
    )
    model.articulation(
        "pcie_slot_latch_joint",
        ArticulationType.REVOLUTE,
        parent=board,
        child=pcie_latch,
        origin=Origin(xyz=(0.0500, -0.0510, BOARD_TOP + pcie_barrel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=3.0,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "m2_retention_joint",
        ArticulationType.REVOLUTE,
        parent=board,
        child=m2_lever,
        origin=Origin(xyz=(0.0630, -0.0060, BOARD_TOP)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.05,
            velocity=6.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("motherboard")
    dimm_a = object_model.get_part("dimm_latch_a")
    dimm_b = object_model.get_part("dimm_latch_b")
    pcie_latch = object_model.get_part("pcie_latch")
    m2_lever = object_model.get_part("m2_retention_lever")

    dimm_a_joint = object_model.get_articulation("dimm_slot_a_latch_joint")
    dimm_b_joint = object_model.get_articulation("dimm_slot_b_latch_joint")
    pcie_joint = object_model.get_articulation("pcie_slot_latch_joint")
    m2_joint = object_model.get_articulation("m2_retention_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "dimm_latch_joint_axes",
        dimm_a_joint.axis == (-1.0, 0.0, 0.0) and dimm_b_joint.axis == (-1.0, 0.0, 0.0),
        f"Unexpected DIMM latch axes: {dimm_a_joint.axis}, {dimm_b_joint.axis}",
    )
    ctx.check(
        "pcie_latch_joint_axis",
        pcie_joint.axis == (0.0, 1.0, 0.0),
        f"Unexpected PCIe latch axis: {pcie_joint.axis}",
    )
    ctx.check(
        "m2_retention_joint_axis",
        m2_joint.axis == (0.0, 0.0, -1.0),
        f"Unexpected M.2 retention axis: {m2_joint.axis}",
    )

    ctx.expect_contact(
        dimm_a,
        board,
        elem_a="pivot_barrel",
        elem_b="pcb",
        name="dimm_latch_a_seated_on_pivot",
    )
    ctx.expect_contact(
        dimm_b,
        board,
        elem_a="pivot_barrel",
        elem_b="pcb",
        name="dimm_latch_b_seated_on_pivot",
    )
    ctx.expect_contact(
        pcie_latch,
        board,
        elem_a="pivot_barrel",
        elem_b="pcb",
        name="pcie_latch_seated_on_pivot",
    )
    ctx.expect_gap(
        m2_lever,
        board,
        axis="z",
        positive_elem="pivot_collar",
        negative_elem="pcb",
        max_gap=0.0,
        max_penetration=0.0,
        name="m2_lever_base_seated_on_board",
    )
    ctx.expect_contact(
        m2_lever,
        board,
        elem_a="pivot_collar",
        elem_b="m2_pivot_cap",
        name="m2_lever_clipped_to_mounting_pivot",
    )

    dimm_a_rest = ctx.part_world_aabb(dimm_a)
    dimm_b_rest = ctx.part_world_aabb(dimm_b)
    pcie_rest = ctx.part_world_aabb(pcie_latch)
    m2_rest = ctx.part_world_aabb(m2_lever)

    with ctx.pose(
        {
            dimm_a_joint: 0.92,
            dimm_b_joint: 0.92,
            pcie_joint: 0.82,
            m2_joint: 1.35,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="opened_mechanisms_clear")
        ctx.expect_contact(
            dimm_a,
            board,
            elem_a="pivot_barrel",
            elem_b="pcb",
            name="dimm_latch_a_keeps_pivot_contact_open",
        )
        ctx.expect_contact(
            dimm_b,
            board,
            elem_a="pivot_barrel",
            elem_b="pcb",
            name="dimm_latch_b_keeps_pivot_contact_open",
        )
        ctx.expect_contact(
            pcie_latch,
            board,
            elem_a="pivot_barrel",
            elem_b="pcb",
            name="pcie_latch_keeps_pivot_contact_open",
        )
        ctx.expect_contact(
            m2_lever,
            board,
            elem_a="pivot_collar",
            elem_b="m2_pivot_cap",
            name="m2_lever_stays_captured_when_open",
        )

        dimm_a_open = ctx.part_world_aabb(dimm_a)
        dimm_b_open = ctx.part_world_aabb(dimm_b)
        pcie_open = ctx.part_world_aabb(pcie_latch)
        m2_open = ctx.part_world_aabb(m2_lever)

        if dimm_a_rest is None or dimm_a_open is None:
            ctx.fail("dimm_latch_a_aabb_available", "Missing DIMM latch A AABB for motion check.")
        else:
            ctx.check(
                "dimm_latch_a_swings_outward",
                dimm_a_open[1][1] > dimm_a_rest[1][1] + 0.003,
                f"Expected DIMM latch A to move toward +Y; rest={dimm_a_rest}, open={dimm_a_open}",
            )

        if dimm_b_rest is None or dimm_b_open is None:
            ctx.fail("dimm_latch_b_aabb_available", "Missing DIMM latch B AABB for motion check.")
        else:
            ctx.check(
                "dimm_latch_b_swings_outward",
                dimm_b_open[1][1] > dimm_b_rest[1][1] + 0.003,
                f"Expected DIMM latch B to move toward +Y; rest={dimm_b_rest}, open={dimm_b_open}",
            )

        if pcie_rest is None or pcie_open is None:
            ctx.fail("pcie_latch_aabb_available", "Missing PCIe latch AABB for motion check.")
        else:
            ctx.check(
                "pcie_latch_rotates_away_from_slot",
                pcie_open[0][0] > pcie_rest[0][0] + 0.003,
                f"Expected PCIe latch to open toward +X; rest={pcie_rest}, open={pcie_open}",
            )

        if m2_rest is None or m2_open is None:
            ctx.fail("m2_lever_aabb_available", "Missing M.2 lever AABB for motion check.")
        else:
            ctx.check(
                "m2_lever_swivels_clear_of_card_end",
                m2_open[1][1] > m2_rest[1][1] + 0.003,
                f"Expected M.2 lever to swing toward +Y; rest={m2_rest}, open={m2_open}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
