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


BOARD_X = 0.244
BOARD_Y = 0.244
BOARD_T = 0.0018

CPU_SOCKET_POS = (-0.002, 0.054, BOARD_T)
DIMM_SLOT_XS = (0.055, 0.064, 0.073, 0.082)
DIMM_SLOT_Y = 0.022
PCIE_X16_POS = (-0.035, -0.028, BOARD_T)


def add_box(part, name, size, center, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def add_cylinder(part, name, radius, length, center, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="micro_atx_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.12, 0.34, 0.20, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    slot_black = model.material("slot_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    aluminum = model.material("aluminum", rgba=(0.58, 0.60, 0.62, 1.0))
    off_white = model.material("off_white", rgba=(0.86, 0.86, 0.82, 1.0))
    gold = model.material("gold", rgba=(0.83, 0.68, 0.24, 1.0))

    motherboard = model.part("motherboard")
    add_box(
        motherboard,
        "pcb",
        (BOARD_X, BOARD_Y, BOARD_T),
        (0.0, 0.0, BOARD_T / 2.0),
        pcb_green,
    )
    add_box(
        motherboard,
        "rear_io_shroud",
        (0.028, 0.050, 0.032),
        (-0.108, 0.090, BOARD_T + 0.016),
        dark_gray,
    )
    add_box(
        motherboard,
        "rear_io_ports_upper",
        (0.010, 0.026, 0.018),
        (-0.127, 0.100, BOARD_T + 0.009),
        steel,
    )
    add_box(
        motherboard,
        "rear_io_ports_lower",
        (0.010, 0.030, 0.018),
        (-0.127, 0.067, BOARD_T + 0.009),
        steel,
    )
    add_box(
        motherboard,
        "vrm_sink_top",
        (0.065, 0.020, 0.018),
        (-0.034, 0.102, BOARD_T + 0.009),
        aluminum,
    )
    add_box(
        motherboard,
        "vrm_sink_left",
        (0.018, 0.062, 0.020),
        (-0.054, 0.061, BOARD_T + 0.010),
        aluminum,
    )
    add_box(
        motherboard,
        "chipset_sink",
        (0.032, 0.032, 0.014),
        (0.084, -0.064, BOARD_T + 0.007),
        aluminum,
    )
    add_box(
        motherboard,
        "m2_heatsink",
        (0.074, 0.020, 0.006),
        (0.020, -0.076, BOARD_T + 0.003),
        dark_gray,
    )
    add_box(
        motherboard,
        "pcie_x1_slot_upper",
        (0.064, 0.008, 0.010),
        (-0.055, -0.064, BOARD_T + 0.005),
        slot_black,
    )
    add_box(
        motherboard,
        "pcie_x1_slot_lower",
        (0.064, 0.008, 0.010),
        (-0.050, -0.094, BOARD_T + 0.005),
        slot_black,
    )
    add_box(
        motherboard,
        "power_24pin",
        (0.008, 0.052, 0.014),
        (0.118, 0.015, BOARD_T + 0.007),
        off_white,
    )
    add_box(
        motherboard,
        "eps_8pin",
        (0.016, 0.016, 0.013),
        (0.092, 0.114, BOARD_T + 0.0065),
        off_white,
    )
    add_box(
        motherboard,
        "sata_stack",
        (0.012, 0.030, 0.014),
        (0.116, -0.090, BOARD_T + 0.007),
        off_white,
    )
    add_box(
        motherboard,
        "front_header",
        (0.020, 0.006, 0.007),
        (0.090, -0.114, BOARD_T + 0.0035),
        matte_black,
    )
    add_box(
        motherboard,
        "usb_header",
        (0.022, 0.010, 0.010),
        (0.020, -0.109, BOARD_T + 0.005),
        matte_black,
    )
    add_cylinder(
        motherboard,
        "coin_cell",
        radius=0.011,
        length=0.0032,
        center=(0.084, -0.058, BOARD_T + 0.0016),
        material=steel,
    )
    add_box(
        motherboard,
        "coin_cell_holder",
        (0.026, 0.022, 0.0018),
        (0.084, -0.058, BOARD_T + 0.0009),
        matte_black,
    )

    cpu_socket = model.part("cpu_socket")
    add_box(cpu_socket, "socket_body", (0.062, 0.058, 0.0075), (0.0, 0.0, 0.00375), matte_black)
    add_box(cpu_socket, "retention_frame", (0.074, 0.070, 0.0012), (0.0, 0.0, 0.0081), steel)
    add_box(cpu_socket, "contact_pad", (0.046, 0.042, 0.0008), (0.0, 0.0, 0.0079), gold)
    add_box(cpu_socket, "lever_ear_front", (0.006, 0.002, 0.006), (0.034, 0.004, 0.004), steel)
    add_box(cpu_socket, "lever_ear_back", (0.006, 0.002, 0.006), (0.034, -0.004, 0.004), steel)
    add_box(cpu_socket, "lever_capture", (0.002, 0.010, 0.0018), (0.035, 0.0, 0.0068), steel)

    model.articulation(
        "motherboard_to_cpu_socket",
        ArticulationType.FIXED,
        parent=motherboard,
        child=cpu_socket,
        origin=Origin(xyz=CPU_SOCKET_POS),
    )

    cpu_lever = model.part("cpu_socket_lever")
    add_cylinder(
        cpu_lever,
        "pivot_barrel",
        radius=0.0014,
        length=0.006,
        center=(0.0, 0.0, 0.0),
        material=steel,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    add_cylinder(
        cpu_lever,
        "lever_shaft",
        radius=0.0011,
        length=0.028,
        center=(-0.014, 0.0, 0.0018),
        material=steel,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    add_box(cpu_lever, "lever_bend", (0.003, 0.003, 0.0045), (-0.027, 0.0, 0.0046), steel)
    add_cylinder(
        cpu_lever,
        "lever_handle",
        radius=0.0013,
        length=0.012,
        center=(-0.028, 0.0, 0.0085),
        material=steel,
    )

    model.articulation(
        "cpu_socket_to_lever",
        ArticulationType.REVOLUTE,
        parent=cpu_socket,
        child=cpu_lever,
        origin=Origin(xyz=(0.034, 0.0, 0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    for index, slot_x in enumerate(DIMM_SLOT_XS, start=1):
        slot = model.part(f"dimm_slot_{index}")
        add_box(slot, "slot_body", (0.0074, 0.134, 0.0088), (0.0, 0.0, 0.0044), slot_black)
        add_box(slot, "contact_ridge", (0.0030, 0.122, 0.0016), (0.0, 0.0, 0.0096), gold)
        add_box(slot, "lower_end_stop", (0.0092, 0.010, 0.011), (0.0, -0.062, 0.0055), matte_black)
        add_box(slot, "upper_hinge_shelf", (0.0092, 0.006, 0.006), (0.0, 0.065, 0.003), matte_black)
        add_box(slot, "pivot_ear_left", (0.002, 0.003, 0.004), (-0.004, 0.067, 0.0078), matte_black)
        add_box(slot, "pivot_ear_right", (0.002, 0.003, 0.004), (0.004, 0.067, 0.0078), matte_black)

        model.articulation(
            f"motherboard_to_dimm_slot_{index}",
            ArticulationType.FIXED,
            parent=motherboard,
            child=slot,
            origin=Origin(xyz=(slot_x, DIMM_SLOT_Y, BOARD_T)),
        )

        latch = model.part(f"dimm_latch_{index}")
        add_cylinder(
            latch,
            "pivot_barrel",
            radius=0.0008,
            length=0.006,
            center=(0.0, 0.0, 0.0),
            material=off_white,
            rpy=(0.0, math.pi / 2.0, 0.0),
        )
        add_box(latch, "latch_blade", (0.0090, 0.0030, 0.0090), (0.0, -0.0015, 0.0045), off_white)
        add_box(latch, "finger_tab", (0.010, 0.0020, 0.0030), (0.0, -0.0010, 0.0082), off_white)

        model.articulation(
            f"dimm_slot_{index}_to_latch",
            ArticulationType.REVOLUTE,
            parent=slot,
            child=latch,
            origin=Origin(xyz=(0.0, 0.067, 0.0078)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=4.0,
                lower=math.radians(-60.0),
                upper=math.radians(10.0),
            ),
        )

    pcie_slot = model.part("pcie_x16_slot")
    add_box(pcie_slot, "slot_body", (0.142, 0.0108, 0.0110), (0.0, 0.0, 0.0055), slot_black)
    add_box(pcie_slot, "contact_ridge", (0.128, 0.0025, 0.0016), (-0.004, 0.0, 0.0118), gold)
    add_box(pcie_slot, "shield_front", (0.130, 0.0018, 0.0070), (-0.004, 0.0045, 0.0075), steel)
    add_box(pcie_slot, "shield_back", (0.130, 0.0018, 0.0070), (-0.004, -0.0045, 0.0075), steel)
    add_box(pcie_slot, "shield_cap", (0.130, 0.0090, 0.0012), (-0.004, 0.0, 0.0116), steel)
    add_box(pcie_slot, "slot_end_cap", (0.010, 0.012, 0.014), (0.076, 0.0, 0.007), slot_black)
    add_box(pcie_slot, "pivot_ear_front", (0.004, 0.002, 0.006), (0.080, 0.0035, 0.0100), slot_black)
    add_box(pcie_slot, "pivot_ear_back", (0.004, 0.002, 0.006), (0.080, -0.0035, 0.0100), slot_black)
    add_box(pcie_slot, "pivot_keeper", (0.002, 0.006, 0.002), (0.081, 0.0, 0.0130), slot_black)

    model.articulation(
        "motherboard_to_pcie_x16_slot",
        ArticulationType.FIXED,
        parent=motherboard,
        child=pcie_slot,
        origin=Origin(xyz=PCIE_X16_POS),
    )

    pcie_latch = model.part("pcie_x16_latch")
    add_cylinder(
        pcie_latch,
        "pivot_barrel",
        radius=0.0012,
        length=0.005,
        center=(0.0, 0.0, 0.0),
        material=off_white,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    add_box(pcie_latch, "latch_arm", (0.014, 0.004, 0.009), (-0.006, 0.0, 0.0055), off_white)
    add_box(pcie_latch, "retention_tooth", (0.003, 0.006, 0.004), (-0.012, 0.0, 0.0040), off_white)
    add_box(pcie_latch, "press_tab", (0.005, 0.005, 0.003), (0.003, 0.0, 0.0090), off_white)

    model.articulation(
        "pcie_x16_slot_to_latch",
        ArticulationType.REVOLUTE,
        parent=pcie_slot,
        child=pcie_latch,
        origin=Origin(xyz=(0.080, 0.0, 0.0100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    motherboard = object_model.get_part("motherboard")
    cpu_socket = object_model.get_part("cpu_socket")
    cpu_lever = object_model.get_part("cpu_socket_lever")
    pcie_slot = object_model.get_part("pcie_x16_slot")
    pcie_latch = object_model.get_part("pcie_x16_latch")

    dimm_slots = [object_model.get_part(f"dimm_slot_{index}") for index in range(1, 5)]
    dimm_latches = [object_model.get_part(f"dimm_latch_{index}") for index in range(1, 5)]

    cpu_lever_joint = object_model.get_articulation("cpu_socket_to_lever")
    pcie_latch_joint = object_model.get_articulation("pcie_x16_slot_to_latch")
    dimm_latch_joints = [
        object_model.get_articulation(f"dimm_slot_{index}_to_latch")
        for index in range(1, 5)
    ]

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for fixed_part in [cpu_socket, pcie_slot, *dimm_slots]:
        ctx.expect_contact(fixed_part, motherboard)

    ctx.expect_contact(cpu_lever, cpu_socket)
    ctx.expect_contact(pcie_latch, pcie_slot)
    for latch, slot in zip(dimm_latches, dimm_slots):
        ctx.expect_contact(latch, slot)

    ctx.expect_origin_distance(cpu_socket, motherboard, axes="x", max_dist=0.02)
    ctx.expect_origin_gap(cpu_socket, motherboard, axis="y", min_gap=0.045, max_gap=0.070)
    ctx.expect_origin_gap(dimm_slots[0], cpu_socket, axis="x", min_gap=0.045, max_gap=0.065)
    ctx.expect_origin_gap(cpu_socket, pcie_slot, axis="y", min_gap=0.070, max_gap=0.095)

    slot_positions = [ctx.part_world_position(slot) for slot in dimm_slots]
    if all(pos is not None for pos in slot_positions):
        slot_xs = [pos[0] for pos in slot_positions if pos is not None]
        slot_gaps = [slot_xs[i + 1] - slot_xs[i] for i in range(len(slot_xs) - 1)]
        ctx.check(
            "memory_slots_evenly_spaced",
            all(0.0080 <= gap <= 0.0100 for gap in slot_gaps),
            details=f"slot_gaps={slot_gaps}",
        )
    else:
        ctx.fail("memory_slots_evenly_spaced", "could not resolve all DIMM slot positions")

    ctx.check(
        "cpu_lever_axis_is_local_y",
        tuple(cpu_lever_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={cpu_lever_joint.axis}",
    )
    ctx.check(
        "graphics_latch_axis_is_local_y",
        tuple(pcie_latch_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={pcie_latch_joint.axis}",
    )
    for index, joint in enumerate(dimm_latch_joints, start=1):
        ctx.check(
            f"dimm_latch_{index}_axis_is_local_x",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    cpu_lever_rest = ctx.part_world_aabb(cpu_lever)
    assert cpu_lever_rest is not None
    with ctx.pose({cpu_lever_joint: math.radians(95.0)}):
        cpu_lever_open = ctx.part_world_aabb(cpu_lever)
        assert cpu_lever_open is not None
        ctx.check(
            "cpu_lever_rotates_upward",
            cpu_lever_open[1][2] > cpu_lever_rest[1][2] + 0.012,
            details=f"rest_max_z={cpu_lever_rest[1][2]}, open_max_z={cpu_lever_open[1][2]}",
        )
        ctx.expect_contact(cpu_lever, cpu_socket)

    dimm_latch_rest = ctx.part_world_aabb(dimm_latches[0])
    assert dimm_latch_rest is not None
    with ctx.pose({dimm_latch_joints[0]: math.radians(-50.0)}):
        dimm_latch_open = ctx.part_world_aabb(dimm_latches[0])
        assert dimm_latch_open is not None
        ctx.check(
            "dimm_latch_rotates_clear_of_slot",
            dimm_latch_open[1][1] > dimm_latch_rest[1][1] + 0.004,
            details=f"rest_max_y={dimm_latch_rest[1][1]}, open_max_y={dimm_latch_open[1][1]}",
        )
        ctx.expect_contact(dimm_latches[0], dimm_slots[0])

    for latch, slot, joint in zip(dimm_latches, dimm_slots, dimm_latch_joints):
        with ctx.pose({joint: math.radians(-50.0)}):
            ctx.expect_contact(latch, slot)

    pcie_latch_rest = ctx.part_world_aabb(pcie_latch)
    assert pcie_latch_rest is not None
    with ctx.pose({pcie_latch_joint: math.radians(45.0)}):
        pcie_latch_open = ctx.part_world_aabb(pcie_latch)
        assert pcie_latch_open is not None
        ctx.check(
            "graphics_latch_releases_upward",
            pcie_latch_open[1][2] > pcie_latch_rest[1][2] + 0.004,
            details=f"rest_max_z={pcie_latch_rest[1][2]}, open_max_z={pcie_latch_open[1][2]}",
        )
        ctx.expect_contact(pcie_latch, pcie_slot)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
