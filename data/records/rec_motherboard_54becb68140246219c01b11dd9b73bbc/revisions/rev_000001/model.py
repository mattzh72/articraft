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


BOARD_SIZE = (0.305, 0.244, 0.0016)
BOARD_HALF_X = BOARD_SIZE[0] / 2.0
BOARD_HALF_Y = BOARD_SIZE[1] / 2.0

CPU_SOCKET_POS = (0.035, 0.038)
CPU_SOCKET_OUTER = 0.054

MEMORY_SLOT_XS = (0.091, 0.102, 0.113, 0.124)
MEMORY_SLOT_Y = 0.010
MEMORY_SLOT_LENGTH = 0.133
MEMORY_SLOT_WIDTH = 0.0086
MEMORY_SLOT_BASE_HEIGHT = 0.0065
MEMORY_SLOT_WALL_HEIGHT = 0.0050
MEMORY_SLOT_PIVOT_Y = MEMORY_SLOT_LENGTH / 2.0 + 0.0015
MEMORY_SLOT_PIVOT_Z = MEMORY_SLOT_BASE_HEIGHT + MEMORY_SLOT_WALL_HEIGHT - 0.0010

PRIMARY_PCIE_POS = (0.006, -0.068)
SECONDARY_PCIE_POS = (-0.010, -0.096)
PCIE_LENGTH = 0.092
PCIE_WIDTH = 0.010
PCIE_BASE_HEIGHT = 0.0070
PCIE_WALL_HEIGHT = 0.0060
PCIE_PIVOT_X = PCIE_LENGTH / 2.0 - 0.0040
PCIE_PIVOT_Z = PCIE_BASE_HEIGHT + PCIE_WALL_HEIGHT - 0.0020


def _board_mount(_board, x: float, y: float) -> Origin:
    return Origin(xyz=(x, y, BOARD_SIZE[2]))


def _add_heatsink(
    part,
    *,
    size: tuple[float, float, float],
    fin_count: int,
    fin_axis: str,
    material,
) -> None:
    sx, sy, sz = size
    base_height = sz * 0.38
    fin_height = sz - base_height
    part.visual(
        Box((sx, sy, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=material,
        name="base_block",
    )
    if fin_axis == "x":
        pitch = sx / (fin_count + 1)
        fin_width = min(0.003, pitch * 0.45)
        for index in range(fin_count):
            x = -sx / 2.0 + pitch * (index + 1)
            part.visual(
                Box((fin_width, sy, fin_height)),
                origin=Origin(xyz=(x, 0.0, base_height + fin_height / 2.0)),
                material=material,
                name=f"fin_{index}",
            )
    else:
        pitch = sy / (fin_count + 1)
        fin_width = min(0.003, pitch * 0.45)
        for index in range(fin_count):
            y = -sy / 2.0 + pitch * (index + 1)
            part.visual(
                Box((sx, fin_width, fin_height)),
                origin=Origin(xyz=(0.0, y, base_height + fin_height / 2.0)),
                material=material,
                name=f"fin_{index}",
            )


def _add_memory_slot_geometry(part, *, material, add_supports: bool = True) -> None:
    wall_thickness = 0.0013
    side_wall_length = MEMORY_SLOT_LENGTH - 0.004
    full_height = MEMORY_SLOT_BASE_HEIGHT + MEMORY_SLOT_WALL_HEIGHT

    part.visual(
        Box((MEMORY_SLOT_WIDTH, MEMORY_SLOT_LENGTH, MEMORY_SLOT_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MEMORY_SLOT_BASE_HEIGHT / 2.0)),
        material=material,
        name="slot_base",
    )
    part.visual(
        Box((wall_thickness, side_wall_length, full_height)),
        origin=Origin(
            xyz=(
                -MEMORY_SLOT_WIDTH / 2.0 + wall_thickness / 2.0,
                0.0,
                full_height / 2.0,
            )
        ),
        material=material,
        name="left_wall",
    )
    part.visual(
        Box((wall_thickness, side_wall_length, full_height)),
        origin=Origin(
            xyz=(
                MEMORY_SLOT_WIDTH / 2.0 - wall_thickness / 2.0,
                0.0,
                full_height / 2.0,
            )
        ),
        material=material,
        name="right_wall",
    )
    part.visual(
        Box((0.0030, 0.0060, 0.0035)),
        origin=Origin(xyz=(0.0, -0.010, MEMORY_SLOT_BASE_HEIGHT + 0.00175)),
        material=material,
        name="key_ridge",
    )

    if add_supports:
        for end_sign, end_name in ((1.0, "top"), (-1.0, "bottom")):
            part.visual(
                Box((MEMORY_SLOT_WIDTH, 0.0045, full_height)),
                origin=Origin(
                    xyz=(
                        0.0,
                        end_sign * (MEMORY_SLOT_LENGTH / 2.0 - 0.00225),
                        full_height / 2.0,
                    )
                ),
                material=material,
                name=f"{end_name}_end_block",
            )


def _add_pcie_slot_geometry(part, *, material, add_clip_supports: bool) -> None:
    wall_thickness = 0.0014
    side_wall_length = PCIE_LENGTH - 0.006
    full_height = PCIE_BASE_HEIGHT + PCIE_WALL_HEIGHT

    part.visual(
        Box((PCIE_LENGTH, PCIE_WIDTH, PCIE_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PCIE_BASE_HEIGHT / 2.0)),
        material=material,
        name="slot_base",
    )
    part.visual(
        Box((side_wall_length, wall_thickness, full_height)),
        origin=Origin(
            xyz=(
                0.0,
                -PCIE_WIDTH / 2.0 + wall_thickness / 2.0,
                full_height / 2.0,
            )
        ),
        material=material,
        name="lower_wall",
    )
    part.visual(
        Box((side_wall_length, wall_thickness, full_height)),
        origin=Origin(
            xyz=(
                0.0,
                PCIE_WIDTH / 2.0 - wall_thickness / 2.0,
                full_height / 2.0,
            )
        ),
        material=material,
        name="upper_wall",
    )
    part.visual(
        Box((0.0070, 0.0030, 0.0030)),
        origin=Origin(xyz=(-0.016, 0.0, PCIE_BASE_HEIGHT + 0.0015)),
        material=material,
        name="key_ridge",
    )

    if add_clip_supports:
        for side_index, y in enumerate((-0.0026, 0.0026)):
            part.visual(
                Box((0.0100, 0.0020, 0.0060)),
                origin=Origin(xyz=(PCIE_PIVOT_X, y, PCIE_PIVOT_Z)),
                material=material,
                name=f"clip_end_block_{side_index}",
            )


def _add_memory_latch_geometry(part, *, material, direction: float = 1.0) -> None:
    part.visual(
        Cylinder(radius=0.0015, length=0.0040),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="pivot_sleeve",
    )
    part.visual(
        Box((0.0026, 0.0024, 0.0018)),
        origin=Origin(xyz=(0.0, direction * 0.0012, 0.0009)),
        material=material,
        name="pivot_bridge",
    )
    part.visual(
        Box((0.0040, 0.0090, 0.0018)),
        origin=Origin(xyz=(0.0, direction * 0.0056, 0.0009)),
        material=material,
        name="latch_leaf",
    )
    part.visual(
        Box((0.0030, 0.0030, 0.0040)),
        origin=Origin(xyz=(0.0, direction * 0.0095, 0.0020)),
        material=material,
        name="hook_block",
    )


def _add_cpu_lever_geometry(part, *, metal_material) -> None:
    part.visual(
        Cylinder(radius=0.0015, length=0.0030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="pivot_sleeve",
    )
    part.visual(
        Box((0.0270, 0.0024, 0.0018)),
        origin=Origin(xyz=(0.0150, 0.0, 0.0)),
        material=metal_material,
        name="lever_arm",
    )
    part.visual(
        Box((0.0060, 0.0042, 0.0018)),
        origin=Origin(xyz=(0.0310, 0.0, 0.0)),
        material=metal_material,
        name="lever_tip",
    )


def _add_pcie_clip_geometry(part, *, material) -> None:
    part.visual(
        Cylinder(radius=0.0020, length=0.0060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="pivot_sleeve",
    )
    part.visual(
        Box((0.0032, 0.0030, 0.0030)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0015)),
        material=material,
        name="pivot_bridge",
    )
    part.visual(
        Box((0.0060, 0.0022, 0.0100)),
        origin=Origin(xyz=(0.0, 0.0026, 0.0070)),
        material=material,
        name="clip_blade",
    )
    part.visual(
        Box((0.0040, 0.0040, 0.0030)),
        origin=Origin(xyz=(0.0, 0.0048, 0.0115)),
        material=material,
        name="clip_finger",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.11, 0.43, 0.20, 1.0))
    solder_mask = model.material("solder_mask", rgba=(0.08, 0.33, 0.15, 1.0))
    slot_black = model.material("slot_black", rgba=(0.14, 0.15, 0.16, 1.0))
    latch_beige = model.material("latch_beige", rgba=(0.86, 0.84, 0.76, 1.0))
    socket_gray = model.material("socket_gray", rgba=(0.72, 0.73, 0.75, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.52, 0.54, 0.58, 1.0))
    connector_dark = model.material("connector_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.81, 1.0))
    port_silver = model.material("port_silver", rgba=(0.68, 0.69, 0.71, 1.0))

    pcb = model.part("pcb")
    pcb.visual(
        Box(BOARD_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BOARD_SIZE[2] / 2.0)),
        material=pcb_green,
        name="pcb_panel",
    )
    pcb.visual(
        Box((0.115, 0.070, 0.0002)),
        origin=Origin(xyz=(0.030, 0.036, BOARD_SIZE[2] + 0.0001)),
        material=solder_mask,
        name="socket_region",
    )
    pcb.inertial = Inertial.from_geometry(
        Box(BOARD_SIZE),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, BOARD_SIZE[2] / 2.0)),
    )

    cpu_socket = model.part("cpu_socket")
    socket_wall = 0.0040
    socket_height = 0.0080
    socket_inner = CPU_SOCKET_OUTER - 2.0 * socket_wall
    cpu_socket.visual(
        Box((socket_inner, socket_inner, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0006)),
        material=socket_gray,
        name="contact_field",
    )
    cpu_socket.visual(
        Box((socket_wall, CPU_SOCKET_OUTER, socket_height)),
        origin=Origin(xyz=(-CPU_SOCKET_OUTER / 2.0 + socket_wall / 2.0, 0.0, socket_height / 2.0)),
        material=socket_gray,
        name="left_frame",
    )
    cpu_socket.visual(
        Box((socket_wall, CPU_SOCKET_OUTER, socket_height)),
        origin=Origin(xyz=(CPU_SOCKET_OUTER / 2.0 - socket_wall / 2.0, 0.0, socket_height / 2.0)),
        material=socket_gray,
        name="right_frame",
    )
    cpu_socket.visual(
        Box((socket_inner, socket_wall, socket_height)),
        origin=Origin(xyz=(0.0, -CPU_SOCKET_OUTER / 2.0 + socket_wall / 2.0, socket_height / 2.0)),
        material=socket_gray,
        name="lower_frame",
    )
    cpu_socket.visual(
        Box((socket_inner, socket_wall, socket_height)),
        origin=Origin(xyz=(0.0, CPU_SOCKET_OUTER / 2.0 - socket_wall / 2.0, socket_height / 2.0)),
        material=socket_gray,
        name="upper_frame",
    )
    lever_pivot_x = CPU_SOCKET_OUTER / 2.0 + 0.0010
    for side_index, y in enumerate((-0.00225, 0.00225)):
        cpu_socket.visual(
            Box((0.0020, 0.0015, 0.0060)),
            origin=Origin(xyz=(lever_pivot_x, y, 0.0038)),
            material=socket_gray,
            name=f"lever_support_{side_index}",
        )
    cpu_socket.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.010)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )
    model.articulation(
        "pcb_to_cpu_socket",
        ArticulationType.FIXED,
        parent=pcb,
        child=cpu_socket,
        origin=_board_mount(pcb, *CPU_SOCKET_POS),
    )

    cpu_socket_lever = model.part("cpu_socket_lever")
    _add_cpu_lever_geometry(cpu_socket_lever, metal_material=steel)
    cpu_socket_lever.inertial = Inertial.from_geometry(
        Box((0.038, 0.006, 0.004)),
        mass=0.01,
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
    )
    model.articulation(
        "cpu_socket_to_lever",
        ArticulationType.REVOLUTE,
        parent=cpu_socket,
        child=cpu_socket_lever,
        origin=Origin(xyz=(lever_pivot_x, 0.0, 0.0038)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    for index, slot_x in enumerate(MEMORY_SLOT_XS):
        slot = model.part(f"memory_slot_{index}")
        _add_memory_slot_geometry(slot, material=slot_black, add_supports=True)
        slot.inertial = Inertial.from_geometry(
            Box((MEMORY_SLOT_WIDTH, MEMORY_SLOT_LENGTH + 0.006, 0.014)),
            mass=0.018,
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
        )
        model.articulation(
            f"pcb_to_memory_slot_{index}",
            ArticulationType.FIXED,
            parent=pcb,
            child=slot,
            origin=_board_mount(pcb, slot_x, MEMORY_SLOT_Y),
        )

        top_latch = model.part(f"memory_slot_{index}_top_latch")
        _add_memory_latch_geometry(top_latch, material=latch_beige, direction=1.0)
        top_latch.inertial = Inertial.from_geometry(
            Box((0.004, 0.013, 0.005)),
            mass=0.0025,
            origin=Origin(xyz=(0.0, 0.0065, 0.0025)),
        )
        model.articulation(
            f"memory_slot_{index}_top_latch_hinge",
            ArticulationType.REVOLUTE,
            parent=slot,
            child=top_latch,
            origin=Origin(xyz=(0.0, MEMORY_SLOT_PIVOT_Y, MEMORY_SLOT_PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.2,
                velocity=5.0,
                lower=0.0,
                upper=math.radians(68.0),
            ),
        )

        bottom_latch = model.part(f"memory_slot_{index}_bottom_latch")
        _add_memory_latch_geometry(bottom_latch, material=latch_beige, direction=-1.0)
        bottom_latch.inertial = Inertial.from_geometry(
            Box((0.004, 0.013, 0.005)),
            mass=0.0025,
            origin=Origin(xyz=(0.0, -0.0065, 0.0025)),
        )
        model.articulation(
            f"memory_slot_{index}_bottom_latch_hinge",
            ArticulationType.REVOLUTE,
            parent=slot,
            child=bottom_latch,
            origin=Origin(xyz=(0.0, -MEMORY_SLOT_PIVOT_Y, MEMORY_SLOT_PIVOT_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.2,
                velocity=5.0,
                lower=0.0,
                upper=math.radians(68.0),
            ),
        )

    pcie_slot_primary = model.part("pcie_slot_primary")
    _add_pcie_slot_geometry(pcie_slot_primary, material=slot_black, add_clip_supports=True)
    pcie_slot_primary.inertial = Inertial.from_geometry(
        Box((PCIE_LENGTH + 0.004, PCIE_WIDTH, 0.015)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
    )
    model.articulation(
        "pcb_to_pcie_slot_primary",
        ArticulationType.FIXED,
        parent=pcb,
        child=pcie_slot_primary,
        origin=_board_mount(pcb, *PRIMARY_PCIE_POS),
    )

    pcie_retention_clip = model.part("pcie_retention_clip")
    _add_pcie_clip_geometry(pcie_retention_clip, material=latch_beige)
    pcie_retention_clip.inertial = Inertial.from_geometry(
        Box((0.006, 0.006, 0.015)),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.003, 0.0075)),
    )
    model.articulation(
        "pcie_primary_to_retention_clip",
        ArticulationType.REVOLUTE,
        parent=pcie_slot_primary,
        child=pcie_retention_clip,
        origin=Origin(xyz=(PCIE_PIVOT_X, 0.0, PCIE_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )

    pcie_slot_secondary = model.part("pcie_slot_secondary")
    _add_pcie_slot_geometry(pcie_slot_secondary, material=slot_black, add_clip_supports=False)
    pcie_slot_secondary.inertial = Inertial.from_geometry(
        Box((PCIE_LENGTH, PCIE_WIDTH, 0.014)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )
    model.articulation(
        "pcb_to_pcie_slot_secondary",
        ArticulationType.FIXED,
        parent=pcb,
        child=pcie_slot_secondary,
        origin=_board_mount(pcb, *SECONDARY_PCIE_POS),
    )

    vrm_heatsink = model.part("vrm_heatsink")
    _add_heatsink(
        vrm_heatsink,
        size=(0.078, 0.018, 0.018),
        fin_count=7,
        fin_axis="x",
        material=heatsink_gray,
    )
    vrm_heatsink.inertial = Inertial.from_geometry(
        Box((0.078, 0.018, 0.018)),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )
    model.articulation(
        "pcb_to_vrm_heatsink",
        ArticulationType.FIXED,
        parent=pcb,
        child=vrm_heatsink,
        origin=_board_mount(pcb, -0.034, 0.092),
    )

    chipset_heatsink = model.part("chipset_heatsink")
    _add_heatsink(
        chipset_heatsink,
        size=(0.040, 0.040, 0.017),
        fin_count=6,
        fin_axis="y",
        material=heatsink_gray,
    )
    chipset_heatsink.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.017)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
    )
    model.articulation(
        "pcb_to_chipset_heatsink",
        ArticulationType.FIXED,
        parent=pcb,
        child=chipset_heatsink,
        origin=_board_mount(pcb, 0.020, -0.012),
    )

    io_connector_wall = model.part("io_connector_wall")
    io_connector_wall.visual(
        Box((0.014, 0.052, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=connector_dark,
        name="io_housing",
    )
    for index, y in enumerate((-0.016, 0.0, 0.016)):
        io_connector_wall.visual(
            Box((0.008, 0.011, 0.010)),
            origin=Origin(xyz=(0.003, y, 0.009)),
            material=port_silver,
            name=f"port_{index}",
        )
    io_connector_wall.inertial = Inertial.from_geometry(
        Box((0.014, 0.052, 0.022)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )
    model.articulation(
        "pcb_to_io_connector_wall",
        ArticulationType.FIXED,
        parent=pcb,
        child=io_connector_wall,
        origin=_board_mount(pcb, -BOARD_HALF_X + 0.007, 0.082),
    )

    atx_power_connector = model.part("atx_power_connector")
    atx_power_connector.visual(
        Box((0.012, 0.050, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=connector_dark,
        name="power_wall",
    )
    for index, y in enumerate((-0.015, 0.0, 0.015)):
        atx_power_connector.visual(
            Box((0.004, 0.010, 0.008)),
            origin=Origin(xyz=(-0.002, y, 0.006)),
            material=socket_gray,
            name=f"power_port_{index}",
        )
    atx_power_connector.inertial = Inertial.from_geometry(
        Box((0.012, 0.050, 0.016)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "pcb_to_atx_power_connector",
        ArticulationType.FIXED,
        parent=pcb,
        child=atx_power_connector,
        origin=_board_mount(pcb, BOARD_HALF_X - 0.007, 0.090),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pcb = object_model.get_part("pcb")
    cpu_socket = object_model.get_part("cpu_socket")
    cpu_socket_lever = object_model.get_part("cpu_socket_lever")
    pcie_slot_primary = object_model.get_part("pcie_slot_primary")
    pcie_slot_secondary = object_model.get_part("pcie_slot_secondary")
    pcie_retention_clip = object_model.get_part("pcie_retention_clip")
    vrm_heatsink = object_model.get_part("vrm_heatsink")
    chipset_heatsink = object_model.get_part("chipset_heatsink")
    io_connector_wall = object_model.get_part("io_connector_wall")
    atx_power_connector = object_model.get_part("atx_power_connector")

    lever_joint = object_model.get_articulation("cpu_socket_to_lever")
    clip_joint = object_model.get_articulation("pcie_primary_to_retention_clip")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    mounted_parts = (
        ("cpu_socket", cpu_socket, 0.040),
        ("pcie_slot_primary", pcie_slot_primary, 0.008),
        ("pcie_slot_secondary", pcie_slot_secondary, 0.008),
        ("vrm_heatsink", vrm_heatsink, 0.015),
        ("chipset_heatsink", chipset_heatsink, 0.020),
        ("io_connector_wall", io_connector_wall, 0.012),
        ("atx_power_connector", atx_power_connector, 0.010),
    )
    for name, part, min_overlap in mounted_parts:
        ctx.expect_contact(part, pcb, name=f"{name}_contacts_pcb")
        ctx.expect_overlap(part, pcb, axes="xy", min_overlap=min_overlap, name=f"{name}_overlaps_pcb_footprint")

    ctx.expect_within(cpu_socket, pcb, axes="xy", margin=0.0, name="cpu_socket_within_board")
    ctx.expect_within(vrm_heatsink, pcb, axes="xy", margin=0.0, name="vrm_heatsink_within_board")
    ctx.expect_within(chipset_heatsink, pcb, axes="xy", margin=0.0, name="chipset_heatsink_within_board")
    ctx.expect_within(pcie_slot_primary, pcb, axes="xy", margin=0.0, name="pcie_primary_within_board")
    ctx.expect_within(pcie_slot_secondary, pcb, axes="xy", margin=0.0, name="pcie_secondary_within_board")

    ctx.expect_contact(cpu_socket_lever, cpu_socket, name="cpu_socket_lever_contacts_socket")
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        ctx.expect_contact(cpu_socket_lever, cpu_socket, name="cpu_socket_lever_contacts_socket_open")
    lever_rest = ctx.part_world_aabb(cpu_socket_lever)
    assert lever_rest is not None
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        lever_open = ctx.part_world_aabb(cpu_socket_lever)
        assert lever_open is not None
        ctx.check(
            "cpu_socket_lever_lifts_clear",
            lever_open[1][2] > lever_rest[1][2] + 0.016,
            f"rest_max_z={lever_rest[1][2]:.4f}, open_max_z={lever_open[1][2]:.4f}",
        )

    for index in range(len(MEMORY_SLOT_XS)):
        slot = object_model.get_part(f"memory_slot_{index}")
        top_latch = object_model.get_part(f"memory_slot_{index}_top_latch")
        bottom_latch = object_model.get_part(f"memory_slot_{index}_bottom_latch")
        top_joint = object_model.get_articulation(f"memory_slot_{index}_top_latch_hinge")
        bottom_joint = object_model.get_articulation(f"memory_slot_{index}_bottom_latch_hinge")

        ctx.expect_contact(slot, pcb, name=f"memory_slot_{index}_contacts_pcb")
        ctx.expect_overlap(
            slot,
            pcb,
            axes="xy",
            min_overlap=0.008,
            name=f"memory_slot_{index}_overlaps_pcb_footprint",
        )
        ctx.expect_contact(top_latch, slot, name=f"memory_slot_{index}_top_latch_contacts_slot")
        ctx.expect_contact(bottom_latch, slot, name=f"memory_slot_{index}_bottom_latch_contacts_slot")

        top_rest = ctx.part_world_aabb(top_latch)
        bottom_rest = ctx.part_world_aabb(bottom_latch)
        assert top_rest is not None
        assert bottom_rest is not None

        with ctx.pose({top_joint: top_joint.motion_limits.upper}):
            top_open = ctx.part_world_aabb(top_latch)
            assert top_open is not None
            ctx.expect_contact(top_latch, slot, name=f"memory_slot_{index}_top_latch_contacts_slot_open")
            ctx.check(
                f"memory_slot_{index}_top_latch_opens",
                top_open[1][2] > top_rest[1][2] + 0.003,
                f"rest_max_z={top_rest[1][2]:.4f}, open_max_z={top_open[1][2]:.4f}",
            )

        with ctx.pose({bottom_joint: bottom_joint.motion_limits.upper}):
            bottom_open = ctx.part_world_aabb(bottom_latch)
            assert bottom_open is not None
            ctx.expect_contact(bottom_latch, slot, name=f"memory_slot_{index}_bottom_latch_contacts_slot_open")
            ctx.check(
                f"memory_slot_{index}_bottom_latch_opens",
                bottom_open[1][2] > bottom_rest[1][2] + 0.003,
                f"rest_max_z={bottom_rest[1][2]:.4f}, open_max_z={bottom_open[1][2]:.4f}",
            )

    ctx.expect_contact(pcie_retention_clip, pcie_slot_primary, name="pcie_clip_contacts_primary_slot")
    clip_rest = ctx.part_element_world_aabb(pcie_retention_clip, elem="clip_finger")
    assert clip_rest is not None
    with ctx.pose({clip_joint: clip_joint.motion_limits.upper}):
        clip_open = ctx.part_element_world_aabb(pcie_retention_clip, elem="clip_finger")
        assert clip_open is not None
        ctx.expect_contact(pcie_retention_clip, pcie_slot_primary, name="pcie_clip_contacts_primary_slot_open")
        ctx.check(
            "pcie_clip_rotates_outward",
            clip_open[1][1] > clip_rest[1][1] + 0.003,
            f"rest_max_y={clip_rest[1][1]:.4f}, open_max_y={clip_open[1][1]:.4f}",
        )

    articulated_joint_names = ["cpu_socket_to_lever", "pcie_primary_to_retention_clip"]
    articulated_joint_names.extend(
        f"memory_slot_{index}_{end}_latch_hinge"
        for index in range(len(MEMORY_SLOT_XS))
        for end in ("top", "bottom")
    )
    for joint_name in articulated_joint_names:
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint_name}_lower_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint_name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
