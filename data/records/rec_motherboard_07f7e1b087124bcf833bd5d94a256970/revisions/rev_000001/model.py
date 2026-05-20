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
    model = ArticulatedObject(name="industrial_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.14, 0.31, 0.17, 1.0))
    solder_mask = model.material("solder_mask", rgba=(0.10, 0.23, 0.13, 1.0))
    socket_black = model.material("socket_black", rgba=(0.14, 0.14, 0.15, 1.0))
    socket_cream = model.material("socket_cream", rgba=(0.82, 0.80, 0.72, 1.0))
    dimm_black = model.material("dimm_black", rgba=(0.18, 0.19, 0.20, 1.0))
    latch_gray = model.material("latch_gray", rgba=(0.82, 0.84, 0.86, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.55, 0.58, 0.60, 1.0))
    port_dark = model.material("port_dark", rgba=(0.22, 0.24, 0.26, 1.0))
    gold = model.material("gold", rgba=(0.78, 0.67, 0.22, 1.0))

    board_x = 0.190
    board_y = 0.170
    board_t = 0.0016

    socket_x = -0.036
    socket_y = 0.008
    socket_w = 0.054
    socket_l = 0.057
    socket_base_h = 0.0024
    socket_frame_h = 0.0042
    socket_wall = 0.0040

    slot_center_y = 0.002
    slot_pitch_x = 0.012
    slot_x0 = 0.031
    slot_w = 0.0086
    slot_l = 0.136
    slot_base_h = 0.0040
    slot_rail_h = 0.0046
    slot_end_support_len = 0.0040

    board = model.part("board")
    board.visual(
        Box((board_x, board_y, board_t)),
        origin=Origin(xyz=(0.0, 0.0, board_t / 2.0)),
        material=pcb_green,
        name="pcb",
    )
    board.visual(
        Box((0.024, 0.050, 0.016)),
        origin=Origin(xyz=(-board_x / 2.0 + 0.012, 0.046, board_t + 0.008)),
        material=port_dark,
        name="rear_io_stack",
    )
    board.visual(
        Box((0.032, 0.030, 0.018)),
        origin=Origin(xyz=(-0.020, -0.052, board_t + 0.009)),
        material=heatsink_gray,
        name="chipset_heatsink",
    )
    board.visual(
        Box((0.038, 0.012, 0.006)),
        origin=Origin(xyz=(-0.057, 0.048, board_t + 0.003)),
        material=socket_black,
        name="vrm_block",
    )
    board.visual(
        Box((0.009, 0.028, 0.012)),
        origin=Origin(xyz=(0.074, 0.058, board_t + 0.006)),
        material=port_dark,
        name="power_header",
    )
    board.visual(
        Box((0.055, 0.0025, 0.0004)),
        origin=Origin(xyz=(0.070, -0.064, board_t + 0.0002)),
        material=gold,
        name="edge_fingers",
    )

    socket = model.part("processor_socket")
    socket.visual(
        Box((socket_w, socket_l, socket_base_h)),
        origin=Origin(xyz=(0.0, 0.0, socket_base_h / 2.0)),
        material=socket_cream,
        name="socket_base",
    )
    socket.visual(
        Box((socket_w, socket_wall, socket_frame_h)),
        origin=Origin(
            xyz=(0.0, socket_l / 2.0 - socket_wall / 2.0, socket_base_h + socket_frame_h / 2.0)
        ),
        material=socket_black,
        name="socket_frame_top",
    )
    socket.visual(
        Box((socket_w, socket_wall, socket_frame_h)),
        origin=Origin(
            xyz=(0.0, -socket_l / 2.0 + socket_wall / 2.0, socket_base_h + socket_frame_h / 2.0)
        ),
        material=socket_black,
        name="socket_frame_bottom",
    )
    socket.visual(
        Box((socket_wall, socket_l - 2.0 * socket_wall, socket_frame_h)),
        origin=Origin(
            xyz=(-socket_w / 2.0 + socket_wall / 2.0, 0.0, socket_base_h + socket_frame_h / 2.0)
        ),
        material=socket_black,
        name="socket_frame_left",
    )
    socket.visual(
        Box((socket_wall, socket_l - 2.0 * socket_wall, socket_frame_h)),
        origin=Origin(
            xyz=(socket_w / 2.0 - socket_wall / 2.0, 0.0, socket_base_h + socket_frame_h / 2.0)
        ),
        material=socket_black,
        name="socket_frame_right",
    )
    socket.visual(
        Box((0.038, 0.041, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, socket_base_h + 0.0004)),
        material=gold,
        name="contact_field",
    )
    socket.visual(
        Box((0.006, 0.034, 0.0082)),
        origin=Origin(xyz=(socket_w / 2.0 + 0.003, -0.005, 0.0041)),
        material=steel,
        name="lever_pivot_bracket",
    )
    socket.visual(
        Cylinder(radius=0.0018, length=0.040),
        origin=Origin(
            xyz=(socket_w / 2.0 + 0.0046, -0.005, 0.0082),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="lever_pivot_barrel",
    )
    socket.visual(
        Box((0.004, 0.010, 0.004)),
        origin=Origin(xyz=(socket_w / 2.0 - 0.001, -socket_l / 2.0 + 0.008, 0.006)),
        material=steel,
        name="lever_latch_catch",
    )

    model.articulation(
        "board_to_socket",
        ArticulationType.FIXED,
        parent=board,
        child=socket,
        origin=Origin(xyz=(socket_x, socket_y, board_t)),
    )

    lever = model.part("socket_lever")
    lever.visual(
        Box((0.0032, 0.047, 0.0018)),
        origin=Origin(xyz=(0.0072, 0.0, 0.0009)),
        material=steel,
        name="lever_bar",
    )
    lever.visual(
        Box((0.0046, 0.010, 0.010)),
        origin=Origin(xyz=(0.0102, 0.021, 0.0058)),
        material=steel,
        name="lever_handle",
    )
    lever.visual(
        Box((0.0050, 0.008, 0.0040)),
        origin=Origin(xyz=(0.0043, -0.020, 0.0020)),
        material=steel,
        name="lever_cam_foot",
    )
    model.articulation(
        "socket_to_lever",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=lever,
        origin=Origin(xyz=(socket_w / 2.0 + 0.0046, -0.005, 0.0082)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.22),
    )

    def add_dimm_slot(slot_name: str, x_pos: float) -> tuple[object, object, object]:
        slot = model.part(slot_name)
        slot.visual(
            Box((slot_w, slot_l, slot_base_h)),
            origin=Origin(xyz=(0.0, 0.0, slot_base_h / 2.0)),
            material=dimm_black,
            name="slot_base",
        )
        rail_t = 0.0015
        slot.visual(
            Box((rail_t, slot_l, slot_rail_h)),
            origin=Origin(
                xyz=(-slot_w / 2.0 + rail_t / 2.0, 0.0, slot_base_h + slot_rail_h / 2.0)
            ),
            material=dimm_black,
            name="slot_left_rail",
        )
        slot.visual(
            Box((rail_t, slot_l, slot_rail_h)),
            origin=Origin(
                xyz=(slot_w / 2.0 - rail_t / 2.0, 0.0, slot_base_h + slot_rail_h / 2.0)
            ),
            material=dimm_black,
            name="slot_right_rail",
        )
        slot.visual(
            Box((slot_w - 0.0034, 0.012, 0.0026)),
            origin=Origin(xyz=(0.0, -0.015, slot_base_h + 0.0013)),
            material=socket_black,
            name="slot_key",
        )
        for end_name, end_sign in (("top", 1.0), ("bottom", -1.0)):
            slot.visual(
                Box((slot_w + 0.0030, slot_end_support_len, 0.0102)),
                origin=Origin(
                    xyz=(
                        0.0,
                        end_sign * (slot_l / 2.0 + slot_end_support_len / 2.0),
                        0.0051,
                    )
                ),
                material=dimm_black,
                name=f"{end_name}_support",
            )

        model.articulation(
            f"board_to_{slot_name}",
            ArticulationType.FIXED,
            parent=board,
            child=slot,
            origin=Origin(xyz=(x_pos, slot_center_y, board_t)),
        )

        latches: list[object] = []
        for end_name, end_sign in (("top", 1.0), ("bottom", -1.0)):
            latch = model.part(f"{slot_name}_{end_name}_latch")
            latch.visual(
                Box((slot_w + 0.0025, 0.0028, 0.0072)),
                origin=Origin(xyz=(0.0, end_sign * 0.0014, 0.0138)),
                material=latch_gray,
                name="latch_blade",
            )
            latch.visual(
                Box((slot_w + 0.0015, 0.0026, 0.0030)),
                origin=Origin(xyz=(0.0, -end_sign * 0.0004, 0.0131)),
                material=latch_gray,
                name="latch_hook",
            )
            latch.visual(
                Box((slot_w + 0.0030, 0.0035, 0.0035)),
                origin=Origin(xyz=(0.0, end_sign * 0.0042, 0.0087)),
                material=latch_gray,
                name="latch_tab",
            )
            model.articulation(
                f"{slot_name}_to_{end_name}_latch",
                ArticulationType.REVOLUTE,
                parent=slot,
                child=latch,
                origin=Origin(
                    xyz=(0.0, end_sign * (slot_l / 2.0 + slot_end_support_len), 0.0)
                ),
                axis=(-end_sign, 0.0, 0.0),
                motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.05),
            )
            latches.append(latch)

        return slot, latches[0], latches[1]

    slot_a, slot_a_top, slot_a_bottom = add_dimm_slot("dimm_slot_a", slot_x0)
    slot_b, slot_b_top, slot_b_bottom = add_dimm_slot("dimm_slot_b", slot_x0 + slot_pitch_x)

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
    socket = object_model.get_part("processor_socket")
    lever = object_model.get_part("socket_lever")
    slot_a = object_model.get_part("dimm_slot_a")
    slot_b = object_model.get_part("dimm_slot_b")

    articulated_parts = (
        "socket_lever",
        "dimm_slot_a_top_latch",
        "dimm_slot_a_bottom_latch",
        "dimm_slot_b_top_latch",
        "dimm_slot_b_bottom_latch",
    )
    for part_name in (
        "board",
        "processor_socket",
        "dimm_slot_a",
        "dimm_slot_b",
        *articulated_parts,
    ):
        ctx.check(
            f"{part_name} present",
            object_model.get_part(part_name) is not None,
            details=f"missing part {part_name}",
        )

    ctx.expect_gap(
        socket,
        board,
        axis="z",
        max_gap=0.0002,
        max_penetration=1e-6,
        negative_elem="pcb",
        name="processor socket sits on the board",
    )
    ctx.expect_gap(
        slot_a,
        board,
        axis="z",
        max_gap=0.0002,
        max_penetration=1e-6,
        negative_elem="pcb",
        name="first dimm slot sits on the board",
    )
    ctx.expect_gap(
        slot_b,
        board,
        axis="z",
        max_gap=0.0002,
        max_penetration=1e-6,
        negative_elem="pcb",
        name="second dimm slot sits on the board",
    )
    ctx.expect_overlap(
        slot_a,
        slot_b,
        axes="y",
        min_overlap=0.120,
        name="paired dimm slots run in parallel",
    )
    ctx.expect_origin_gap(
        slot_b,
        slot_a,
        axis="x",
        min_gap=0.010,
        max_gap=0.014,
        name="dimm slots are closely spaced",
    )
    ctx.expect_origin_gap(
        slot_a,
        socket,
        axis="x",
        min_gap=0.050,
        max_gap=0.075,
        name="memory slots sit beside the processor socket",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    lever_joint = object_model.get_articulation("socket_to_lever")
    lever_limits = lever_joint.motion_limits
    lever_open = 1.0 if lever_limits is None or lever_limits.upper is None else lever_limits.upper
    lever_rest_center = aabb_center(ctx.part_element_world_aabb(lever, elem="lever_handle"))
    with ctx.pose({lever_joint: lever_open}):
        lever_open_center = aabb_center(ctx.part_element_world_aabb(lever, elem="lever_handle"))
    ctx.check(
        "socket lever rotates upward from its side pivot",
        lever_rest_center is not None
        and lever_open_center is not None
        and lever_open_center[2] > lever_rest_center[2] + 0.005
        and abs(lever_open_center[0] - lever_rest_center[0]) > 0.010,
        details=f"rest={lever_rest_center}, open={lever_open_center}",
    )

    for latch_name, joint_name, outward_sign in (
        ("dimm_slot_a_top_latch", "dimm_slot_a_to_top_latch", 1.0),
        ("dimm_slot_a_bottom_latch", "dimm_slot_a_to_bottom_latch", -1.0),
        ("dimm_slot_b_top_latch", "dimm_slot_b_to_top_latch", 1.0),
        ("dimm_slot_b_bottom_latch", "dimm_slot_b_to_bottom_latch", -1.0),
    ):
        latch = object_model.get_part(latch_name)
        latch_joint = object_model.get_articulation(joint_name)
        latch_open = 1.0
        if latch_joint.motion_limits is not None and latch_joint.motion_limits.upper is not None:
            latch_open = latch_joint.motion_limits.upper
        rest_center = aabb_center(ctx.part_element_world_aabb(latch, elem="latch_blade"))
        with ctx.pose({latch_joint: latch_open}):
            open_center = aabb_center(ctx.part_element_world_aabb(latch, elem="latch_blade"))
        ctx.check(
            f"{latch_name} swings outward at the slot end",
            rest_center is not None
            and open_center is not None
            and outward_sign * (open_center[1] - rest_center[1]) > 0.004
            and open_center[2] < rest_center[2] - 0.001,
            details=f"rest={rest_center}, open={open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
