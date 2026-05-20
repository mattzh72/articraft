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


BOARD_SIZE = 0.170
BOARD_THICKNESS = 0.0016
BOARD_TOP = BOARD_THICKNESS


def add_box(part, size, center, *, material, name):
    return part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def add_cylinder(part, radius, length, center, *, material, name, rpy=(0.0, 0.0, 0.0)):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def element_center_from_aabb(aabb):
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def element_size_from_aabb(aabb):
    return tuple(hi - lo for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_motherboard")

    board_green = model.material("board_green", rgba=(0.08, 0.23, 0.12, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    connector_black = model.material("connector_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.24, 0.26, 1.0))
    sink_gray = model.material("sink_gray", rgba=(0.38, 0.40, 0.44, 1.0))
    silver = model.material("silver", rgba=(0.72, 0.74, 0.76, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.69, 1.0))
    gold = model.material("gold", rgba=(0.79, 0.67, 0.24, 1.0))
    socket_white = model.material("socket_white", rgba=(0.83, 0.84, 0.86, 1.0))
    capacitor_black = model.material("capacitor_black", rgba=(0.07, 0.07, 0.08, 1.0))
    battery_silver = model.material("battery_silver", rgba=(0.76, 0.77, 0.78, 1.0))

    board = model.part("board")
    add_box(
        board,
        (BOARD_SIZE, BOARD_SIZE, BOARD_THICKNESS),
        (0.0, 0.0, BOARD_THICKNESS * 0.5),
        material=board_green,
        name="pcb",
    )

    # Rear I/O cluster
    add_box(
        board,
        (0.014, 0.056, 0.021),
        (-0.078, 0.044, BOARD_TOP + 0.0105),
        material=dark_gray,
        name="rear_io_stack",
    )
    add_box(
        board,
        (0.006, 0.020, 0.012),
        (-0.074, 0.058, BOARD_TOP + 0.006),
        material=connector_black,
        name="usb_block",
    )
    add_box(
        board,
        (0.006, 0.014, 0.015),
        (-0.074, 0.034, BOARD_TOP + 0.0075),
        material=connector_black,
        name="network_port",
    )

    # CPU socket and surrounding silicon / cooling
    socket_x = -0.006
    socket_y = 0.018
    socket_outer = 0.052
    socket_inner = 0.042
    socket_frame_height = 0.004
    frame_wall = (socket_outer - socket_inner) * 0.5
    frame_center_offset = socket_inner * 0.5 + frame_wall * 0.5
    socket_frame_z = BOARD_TOP + socket_frame_height * 0.5

    add_box(
        board,
        (socket_outer, frame_wall, socket_frame_height),
        (socket_x, socket_y + frame_center_offset, socket_frame_z),
        material=socket_white,
        name="socket_frame_top",
    )
    add_box(
        board,
        (socket_outer, frame_wall, socket_frame_height),
        (socket_x, socket_y - frame_center_offset, socket_frame_z),
        material=socket_white,
        name="socket_frame_bottom",
    )
    add_box(
        board,
        (frame_wall, socket_inner, socket_frame_height),
        (socket_x - frame_center_offset, socket_y, socket_frame_z),
        material=socket_white,
        name="socket_frame_left",
    )
    add_box(
        board,
        (frame_wall, socket_inner, socket_frame_height),
        (socket_x + frame_center_offset, socket_y, socket_frame_z),
        material=socket_white,
        name="socket_frame_right",
    )
    add_box(
        board,
        (0.038, 0.038, 0.0024),
        (socket_x, socket_y, BOARD_TOP + 0.0012),
        material=silver,
        name="cpu_package",
    )
    add_box(
        board,
        (0.009, 0.011, 0.0032),
        (socket_x + 0.029, socket_y - 0.018, BOARD_TOP + 0.0016),
        material=socket_white,
        name="socket_arm_boss",
    )
    add_box(
        board,
        (0.018, 0.040, 0.012),
        (-0.036, 0.048, BOARD_TOP + 0.006),
        material=sink_gray,
        name="vrm_heatsink",
    )
    add_box(
        board,
        (0.028, 0.028, 0.010),
        (-0.010, -0.016, BOARD_TOP + 0.005),
        material=sink_gray,
        name="chipset_heatsink",
    )
    add_box(
        board,
        (0.013, 0.074, 0.003),
        (-0.047, -0.010, BOARD_TOP + 0.0015),
        material=matte_black,
        name="m2_slot_cover",
    )

    # Memory slots
    dimm_x_positions = (0.046, 0.058)
    dimm_center_y = 0.010
    dimm_length = 0.133
    dimm_width = 0.006
    dimm_height = 0.009
    for index, x in enumerate(dimm_x_positions, start=1):
        add_box(
            board,
            (dimm_width, dimm_length, dimm_height),
            (x, dimm_center_y, BOARD_TOP + dimm_height * 0.5),
            material=connector_black,
            name=f"dimm_slot_{index}_body",
        )
        add_box(
            board,
            (0.003, dimm_length - 0.010, 0.0015),
            (x, dimm_center_y, BOARD_TOP + 0.0074),
            material=gold,
            name=f"dimm_slot_{index}_contacts",
        )

    # Main expansion slot
    pcie_length = 0.089
    pcie_width = 0.009
    pcie_height = 0.011
    pcie_x = -0.004
    pcie_y = -0.050
    add_box(
        board,
        (pcie_length, pcie_width, pcie_height),
        (pcie_x, pcie_y, BOARD_TOP + pcie_height * 0.5),
        material=connector_black,
        name="pcie_slot_body",
    )
    add_box(
        board,
        (pcie_length - 0.012, 0.003, 0.0014),
        (pcie_x - 0.001, pcie_y, BOARD_TOP + 0.0082),
        material=gold,
        name="pcie_slot_contacts",
    )

    # Board-edge power connector and small auxiliary details
    add_box(
        board,
        (0.012, 0.050, 0.010),
        (0.079, -0.006, BOARD_TOP + 0.005),
        material=matte_black,
        name="edge_power_connector",
    )
    add_box(
        board,
        (0.022, 0.016, 0.004),
        (0.050, -0.050, BOARD_TOP + 0.002),
        material=matte_black,
        name="sata_ports",
    )
    add_cylinder(
        board,
        0.010,
        0.003,
        (-0.054, -0.046, BOARD_TOP + 0.0015),
        material=battery_silver,
        name="cmos_battery",
    )

    cap_positions = [
        (-0.056, 0.014),
        (-0.046, 0.014),
        (-0.036, 0.014),
        (-0.052, 0.004),
        (-0.042, 0.004),
        (0.034, 0.056),
        (0.026, 0.056),
    ]
    for index, (x, y) in enumerate(cap_positions, start=1):
        add_cylinder(
            board,
            0.0032,
            0.010,
            (x, y, BOARD_TOP + 0.005),
            material=capacitor_black,
            name=f"capacitor_{index}",
        )

    add_box(
        board,
        (0.018, 0.018, 0.002),
        (-0.040, -0.060, BOARD_TOP + 0.001),
        material=matte_black,
        name="audio_section",
    )
    add_box(
        board,
        (0.020, 0.010, 0.006),
        (0.010, 0.060, BOARD_TOP + 0.003),
        material=dark_gray,
        name="eps_power_header",
    )

    # Socket retention arm
    socket_arm = model.part("socket_arm")
    add_cylinder(
        socket_arm,
        0.0015,
        0.0032,
        (0.0, 0.0, 0.0016),
        material=steel,
        name="hinge_barrel",
    )
    add_cylinder(
        socket_arm,
        0.0011,
        0.038,
        (-0.001, -0.019, 0.0018),
        material=steel,
        name="lever_shaft",
        rpy=(pi * 0.5, 0.0, 0.0),
    )
    add_cylinder(
        socket_arm,
        0.0011,
        0.014,
        (0.005, -0.036, 0.0018),
        material=steel,
        name="handle_tip",
        rpy=(0.0, pi * 0.5, 0.0),
    )
    add_box(
        socket_arm,
        (0.004, 0.006, 0.003),
        (0.010, -0.0375, 0.0018),
        material=steel,
        name="lock_hook",
    )
    model.articulation(
        "socket_arm_joint",
        ArticulationType.REVOLUTE,
        parent=board,
        child=socket_arm,
        origin=Origin(xyz=(socket_x + 0.029, socket_y - 0.018, BOARD_TOP + 0.0032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    # One small latch per DIMM slot
    for index, x in enumerate(dimm_x_positions, start=1):
        latch = model.part(f"dimm_latch_{index}")
        add_cylinder(
            latch,
            0.0012,
            0.004,
            (0.0, 0.0, 0.0),
            material=dark_gray,
            name="hinge_barrel",
            rpy=(0.0, pi * 0.5, 0.0),
        )
        add_box(
            latch,
            (0.008, 0.010, 0.012),
            (0.0, 0.0061, 0.0050),
            material=dark_gray,
            name="latch_tab",
        )
        add_box(
            latch,
            (0.004, 0.010, 0.004),
            (0.0, 0.0090, 0.0012),
            material=dark_gray,
            name="latch_foot",
        )
        model.articulation(
            f"dimm_latch_{index}_joint",
            ArticulationType.REVOLUTE,
            parent=board,
            child=latch,
            origin=Origin(
                xyz=(
                    x + dimm_width * 0.5 + 0.002,
                    dimm_center_y + dimm_length * 0.5 - 0.0025,
                    BOARD_TOP + 0.0050,
                )
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=5.0,
                lower=0.0,
                upper=0.75,
            ),
        )

    # PCIe slot end release tab
    pcie_clip = model.part("pcie_slot_clip")
    add_cylinder(
        pcie_clip,
        0.0015,
        0.009,
        (0.0, 0.0, 0.0),
        material=dark_gray,
        name="hinge_barrel",
        rpy=(pi * 0.5, 0.0, 0.0),
    )
    add_box(
        pcie_clip,
        (0.010, 0.004, 0.012),
        (-0.006, 0.004, 0.0065),
        material=dark_gray,
        name="clip_tab",
    )
    add_box(
        pcie_clip,
        (0.005, 0.007, 0.004),
        (-0.0025, 0.0035, 0.0016),
        material=dark_gray,
        name="clip_foot",
    )
    model.articulation(
        "pcie_slot_clip_joint",
        ArticulationType.REVOLUTE,
        parent=board,
        child=pcie_clip,
        origin=Origin(
            xyz=(
                pcie_x + pcie_length * 0.5,
                pcie_y,
                BOARD_TOP + pcie_height + 0.0015,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=5.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    # A small release clip on the board-edge power connector
    edge_clip = model.part("edge_connector_clip")
    add_cylinder(
        edge_clip,
        0.0012,
        0.040,
        (0.0, 0.0, 0.0),
        material=dark_gray,
        name="hinge_barrel",
        rpy=(pi * 0.5, 0.0, 0.0),
    )
    add_box(
        edge_clip,
        (0.006, 0.034, 0.007),
        (0.0030, 0.0, 0.004),
        material=dark_gray,
        name="clip_body",
    )
    add_box(
        edge_clip,
        (0.0025, 0.020, 0.003),
        (0.0058, 0.0, 0.0015),
        material=dark_gray,
        name="release_nib",
    )
    model.articulation(
        "edge_connector_clip_joint",
        ArticulationType.REVOLUTE,
        parent=board,
        child=edge_clip,
        origin=Origin(xyz=(0.0740, -0.006, BOARD_TOP + 0.0112)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=5.0,
            lower=0.0,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    socket_arm = object_model.get_part("socket_arm")
    dimm_latch_1 = object_model.get_part("dimm_latch_1")
    dimm_latch_2 = object_model.get_part("dimm_latch_2")
    pcie_slot_clip = object_model.get_part("pcie_slot_clip")
    edge_connector_clip = object_model.get_part("edge_connector_clip")

    socket_arm_joint = object_model.get_articulation("socket_arm_joint")
    dimm_latch_1_joint = object_model.get_articulation("dimm_latch_1_joint")
    dimm_latch_2_joint = object_model.get_articulation("dimm_latch_2_joint")
    pcie_slot_clip_joint = object_model.get_articulation("pcie_slot_clip_joint")
    edge_connector_clip_joint = object_model.get_articulation("edge_connector_clip_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    pcb_aabb = ctx.part_element_world_aabb(board, elem="pcb")
    pcb_size = element_size_from_aabb(pcb_aabb)
    ctx.check(
        "mini_itx_pcb_size",
        abs(pcb_size[0] - BOARD_SIZE) < 1e-6
        and abs(pcb_size[1] - BOARD_SIZE) < 1e-6
        and abs(pcb_size[2] - BOARD_THICKNESS) < 1e-6,
        f"pcb size was {pcb_size}, expected {(BOARD_SIZE, BOARD_SIZE, BOARD_THICKNESS)}",
    )

    for visual_name in (
        "socket_frame_top",
        "socket_frame_bottom",
        "socket_frame_left",
        "socket_frame_right",
        "dimm_slot_1_body",
        "dimm_slot_2_body",
        "pcie_slot_body",
        "edge_power_connector",
    ):
        ctx.check(
            f"{visual_name}_present",
            any(visual.name == visual_name for visual in board.visuals),
            f"missing board visual {visual_name}",
        )

    def check_joint_axis(name, joint, expected_axis):
        actual_axis = tuple(float(value) for value in joint.axis)
        ok = all(abs(a - b) < 1e-9 for a, b in zip(actual_axis, expected_axis))
        ctx.check(
            name,
            ok,
            f"expected joint axis {expected_axis}, got {actual_axis}",
        )

    check_joint_axis("socket_arm_axis", socket_arm_joint, (0.0, 0.0, 1.0))
    check_joint_axis("dimm_latch_1_axis", dimm_latch_1_joint, (1.0, 0.0, 0.0))
    check_joint_axis("dimm_latch_2_axis", dimm_latch_2_joint, (1.0, 0.0, 0.0))
    check_joint_axis("pcie_slot_clip_axis", pcie_slot_clip_joint, (0.0, 1.0, 0.0))
    check_joint_axis("edge_connector_clip_axis", edge_connector_clip_joint, (0.0, 1.0, 0.0))

    with ctx.pose({socket_arm_joint: 0.0}):
        ctx.expect_contact(
            socket_arm,
            board,
            elem_a="hinge_barrel",
            elem_b="socket_arm_boss",
            name="socket_arm_hinge_contact_closed",
        )
    with ctx.pose({socket_arm_joint: socket_arm_joint.motion_limits.upper}):
        ctx.expect_contact(
            socket_arm,
            board,
            elem_a="hinge_barrel",
            elem_b="socket_arm_boss",
            name="socket_arm_hinge_contact_open",
        )

    for latch, joint, slot_name in (
        (dimm_latch_1, dimm_latch_1_joint, "dimm_slot_1_body"),
        (dimm_latch_2, dimm_latch_2_joint, "dimm_slot_2_body"),
    ):
        with ctx.pose({joint: 0.0}):
            ctx.expect_contact(
                latch,
                board,
                elem_a="hinge_barrel",
                elem_b=slot_name,
                name=f"{latch.name}_hinge_contact_closed",
            )
        with ctx.pose({joint: joint.motion_limits.upper}):
            ctx.expect_contact(
                latch,
                board,
                elem_a="hinge_barrel",
                elem_b=slot_name,
                name=f"{latch.name}_hinge_contact_open",
            )

    with ctx.pose({pcie_slot_clip_joint: 0.0}):
        ctx.expect_contact(
            pcie_slot_clip,
            board,
            elem_a="hinge_barrel",
            elem_b="pcie_slot_body",
            name="pcie_clip_contact_closed",
        )
    with ctx.pose({pcie_slot_clip_joint: pcie_slot_clip_joint.motion_limits.upper}):
        ctx.expect_contact(
            pcie_slot_clip,
            board,
            elem_a="hinge_barrel",
            elem_b="pcie_slot_body",
            name="pcie_clip_contact_open",
        )

    with ctx.pose({edge_connector_clip_joint: 0.0}):
        ctx.expect_contact(
            edge_connector_clip,
            board,
            elem_a="hinge_barrel",
            elem_b="edge_power_connector",
            name="edge_clip_contact_closed",
        )
    with ctx.pose({edge_connector_clip_joint: edge_connector_clip_joint.motion_limits.upper}):
        ctx.expect_contact(
            edge_connector_clip,
            board,
            elem_a="hinge_barrel",
            elem_b="edge_power_connector",
            name="edge_clip_contact_open",
        )

    def check_motion(name, part, elem_name, joint, *, min_xy_delta=0.0, min_z_delta=0.0, max_z_delta=None):
        with ctx.pose({joint: joint.motion_limits.lower}):
            closed_aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        with ctx.pose({joint: joint.motion_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(part, elem=elem_name)

        closed_center = element_center_from_aabb(closed_aabb)
        open_center = element_center_from_aabb(open_aabb)
        delta_xy = ((open_center[0] - closed_center[0]) ** 2 + (open_center[1] - closed_center[1]) ** 2) ** 0.5
        delta_z = abs(open_center[2] - closed_center[2])
        ok = delta_xy >= min_xy_delta and delta_z >= min_z_delta
        if max_z_delta is not None:
            ok = ok and delta_z <= max_z_delta
        ctx.check(
            name,
            ok,
            f"center moved by xy={delta_xy:.4f} z={delta_z:.4f} for {part.name}:{elem_name}",
        )

    check_motion(
        "socket_arm_sweeps_in_board_plane",
        socket_arm,
        "handle_tip",
        socket_arm_joint,
        min_xy_delta=0.015,
        min_z_delta=0.0,
        max_z_delta=0.004,
    )
    check_motion(
        "dimm_latch_1_rotates",
        dimm_latch_1,
        "latch_tab",
        dimm_latch_1_joint,
        min_xy_delta=0.002,
        min_z_delta=0.0015,
    )
    check_motion(
        "dimm_latch_2_rotates",
        dimm_latch_2,
        "latch_tab",
        dimm_latch_2_joint,
        min_xy_delta=0.002,
        min_z_delta=0.0015,
    )
    check_motion(
        "pcie_slot_clip_rotates",
        pcie_slot_clip,
        "clip_tab",
        pcie_slot_clip_joint,
        min_xy_delta=0.003,
        min_z_delta=0.001,
    )
    check_motion(
        "edge_connector_clip_rotates",
        edge_connector_clip,
        "clip_body",
        edge_connector_clip_joint,
        min_xy_delta=0.0015,
        min_z_delta=0.002,
    )

    for joint in (
        socket_arm_joint,
        dimm_latch_1_joint,
        dimm_latch_2_joint,
        pcie_slot_clip_joint,
        edge_connector_clip_joint,
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
