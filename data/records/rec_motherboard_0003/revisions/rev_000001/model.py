from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)

BOARD_X = 0.305
BOARD_Y = 0.244
BOARD_T = 0.0022
BOARD_TOP = BOARD_T

CPU_SOCKET_X = -0.046
CPU_SOCKET_Y = 0.030
CPU_SOCKET_SIZE = (0.053, 0.053, 0.0046)
CPU_FRAME_HEIGHT = 0.0018
CPU_FRAME_OUTER = 0.058
CPU_FRAME_WIDTH = 0.0040

RAM_SLOT_SIZE = (0.0088, 0.134, 0.0105)
RAM_SLOT_Y = 0.010
RAM_SLOT_XS = (0.070, 0.0815, 0.093, 0.1045)

PCIE_X16_SIZE = (0.176, 0.0088, 0.0120)
PCIE_X16_POS = (0.003, -0.083)


def _top_origin(
    x: float,
    y: float,
    size: tuple[float, float, float],
    *,
    z0: float = BOARD_TOP,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    return Origin(xyz=(x, y, z0 + size[2] * 0.5), rpy=rpy)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 20,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * i) / segments),
            cy + radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return (
        aabb[1][0] - aabb[0][0],
        aabb[1][1] - aabb[0][1],
        aabb[1][2] - aabb[0][2],
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return (
        (aabb[0][0] + aabb[1][0]) * 0.5,
        (aabb[0][1] + aabb[1][1]) * 0.5,
        (aabb[0][2] + aabb[1][2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_motherboard", assets=ASSETS)

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.34, 0.16, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    slot_black = model.material("slot_black", rgba=(0.12, 0.12, 0.13, 1.0))
    latch_grey = model.material("latch_grey", rgba=(0.72, 0.74, 0.76, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.32, 0.34, 0.36, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    io_black = model.material("io_black", rgba=(0.06, 0.07, 0.08, 1.0))
    heatsink_blue = model.material("heatsink_blue", rgba=(0.19, 0.30, 0.46, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.55, 0.57, 0.60, 1.0))
    gold = model.material("gold", rgba=(0.78, 0.64, 0.21, 1.0))

    motherboard = model.part("motherboard")
    pcb_outline = [
        (-BOARD_X * 0.5, -BOARD_Y * 0.5),
        (BOARD_X * 0.5, -BOARD_Y * 0.5),
        (BOARD_X * 0.5, BOARD_Y * 0.5),
        (-BOARD_X * 0.5, BOARD_Y * 0.5),
    ]
    pcb_hole_centers = [
        (-0.141, 0.108),
        (0.000, 0.108),
        (0.141, 0.108),
        (-0.141, -0.108),
        (0.000, -0.108),
        (0.141, -0.108),
        (-0.070, -0.006),
        (0.076, -0.006),
    ]
    pcb_geom = ExtrudeWithHolesGeometry(
        pcb_outline,
        [_circle_profile(x, y, 0.0035) for x, y in pcb_hole_centers],
        height=BOARD_T,
        center=False,
    )
    motherboard.visual(
        _save_mesh("motherboard_pcb.obj", pcb_geom),
        origin=Origin(),
        material=pcb_green,
        name="pcb",
    )

    motherboard.visual(
        Box(CPU_SOCKET_SIZE),
        origin=_top_origin(CPU_SOCKET_X, CPU_SOCKET_Y, CPU_SOCKET_SIZE),
        material=matte_black,
        name="cpu_socket_body",
    )
    frame_z0 = BOARD_TOP + CPU_SOCKET_SIZE[2]
    motherboard.visual(
        Box((CPU_FRAME_OUTER, CPU_FRAME_WIDTH, CPU_FRAME_HEIGHT)),
        origin=_top_origin(CPU_SOCKET_X, CPU_SOCKET_Y + (CPU_FRAME_OUTER - CPU_FRAME_WIDTH) * 0.5, (CPU_FRAME_OUTER, CPU_FRAME_WIDTH, CPU_FRAME_HEIGHT), z0=frame_z0),
        material=aluminum,
        name="cpu_frame_north",
    )
    motherboard.visual(
        Box((CPU_FRAME_OUTER, CPU_FRAME_WIDTH, CPU_FRAME_HEIGHT)),
        origin=_top_origin(CPU_SOCKET_X, CPU_SOCKET_Y - (CPU_FRAME_OUTER - CPU_FRAME_WIDTH) * 0.5, (CPU_FRAME_OUTER, CPU_FRAME_WIDTH, CPU_FRAME_HEIGHT), z0=frame_z0),
        material=aluminum,
        name="cpu_frame_south",
    )
    motherboard.visual(
        Box((CPU_FRAME_WIDTH, CPU_FRAME_OUTER - 2.0 * CPU_FRAME_WIDTH, CPU_FRAME_HEIGHT)),
        origin=_top_origin(CPU_SOCKET_X - (CPU_FRAME_OUTER - CPU_FRAME_WIDTH) * 0.5, CPU_SOCKET_Y, (CPU_FRAME_WIDTH, CPU_FRAME_OUTER - 2.0 * CPU_FRAME_WIDTH, CPU_FRAME_HEIGHT), z0=frame_z0),
        material=aluminum,
        name="cpu_frame_west",
    )
    motherboard.visual(
        Box((CPU_FRAME_WIDTH, CPU_FRAME_OUTER - 2.0 * CPU_FRAME_WIDTH, CPU_FRAME_HEIGHT)),
        origin=_top_origin(CPU_SOCKET_X + (CPU_FRAME_OUTER - CPU_FRAME_WIDTH) * 0.5, CPU_SOCKET_Y, (CPU_FRAME_WIDTH, CPU_FRAME_OUTER - 2.0 * CPU_FRAME_WIDTH, CPU_FRAME_HEIGHT), z0=frame_z0),
        material=aluminum,
        name="cpu_frame_east",
    )
    motherboard.visual(
        Box((0.040, 0.040, 0.0010)),
        origin=_top_origin(CPU_SOCKET_X, CPU_SOCKET_Y, (0.040, 0.040, 0.0010), z0=BOARD_TOP + 0.0014),
        material=accent_grey,
        name="cpu_contact_field",
    )

    cpu_pivot_radius = 0.00145
    cpu_pivot_xyz = (
        CPU_SOCKET_X + 0.0335,
        CPU_SOCKET_Y - 0.0280,
        BOARD_TOP + CPU_SOCKET_SIZE[2] + CPU_FRAME_HEIGHT + cpu_pivot_radius + 0.00015,
    )
    cpu_pedestal_height = cpu_pivot_xyz[2] - cpu_pivot_radius - BOARD_TOP
    motherboard.visual(
        Box((0.0060, 0.0120, cpu_pedestal_height)),
        origin=Origin(
            xyz=(
                cpu_pivot_xyz[0],
                cpu_pivot_xyz[1],
                BOARD_TOP + cpu_pedestal_height * 0.5,
            )
        ),
        material=gunmetal,
        name="cpu_arm_pedestal",
    )

    for index, slot_x in enumerate(RAM_SLOT_XS, start=1):
        slot_name = f"ram_slot_{index}"
        slot_material = slot_black if index % 2 else accent_grey
        motherboard.visual(
            Box(RAM_SLOT_SIZE),
            origin=_top_origin(slot_x, RAM_SLOT_Y, RAM_SLOT_SIZE),
            material=slot_material,
            name=slot_name,
        )

    ram_top_pivot_radius = 0.0012
    ram_top_pivot_xyz = (
        RAM_SLOT_XS[-1],
        RAM_SLOT_Y + RAM_SLOT_SIZE[1] * 0.5 - 0.0040,
        BOARD_TOP + RAM_SLOT_SIZE[2] + ram_top_pivot_radius,
    )
    ram_bottom_pivot_xyz = (
        RAM_SLOT_XS[-1],
        RAM_SLOT_Y - RAM_SLOT_SIZE[1] * 0.5 + 0.0040,
        BOARD_TOP + RAM_SLOT_SIZE[2] + ram_top_pivot_radius,
    )
    ram_latch_pedestal_size = (0.0115, 0.0055, RAM_SLOT_SIZE[2])
    motherboard.visual(
        Box(ram_latch_pedestal_size),
        origin=_top_origin(
            ram_top_pivot_xyz[0],
            ram_top_pivot_xyz[1],
            ram_latch_pedestal_size,
        ),
        material=slot_black,
        name="ram_top_latch_pedestal",
    )
    motherboard.visual(
        Box(ram_latch_pedestal_size),
        origin=_top_origin(
            ram_bottom_pivot_xyz[0],
            ram_bottom_pivot_xyz[1],
            ram_latch_pedestal_size,
        ),
        material=slot_black,
        name="ram_bottom_latch_pedestal",
    )

    motherboard.visual(
        Box(PCIE_X16_SIZE),
        origin=_top_origin(PCIE_X16_POS[0], PCIE_X16_POS[1], PCIE_X16_SIZE),
        material=slot_black,
        name="pcie_x16_slot",
    )
    motherboard.visual(
        Box((0.058, 0.0086, 0.0120)),
        origin=_top_origin(-0.065, -0.061, (0.058, 0.0086, 0.0120)),
        material=accent_grey,
        name="pcie_x1_slot_1",
    )
    motherboard.visual(
        Box((0.058, 0.0086, 0.0120)),
        origin=_top_origin(-0.049, -0.104, (0.058, 0.0086, 0.0120)),
        material=slot_black,
        name="pcie_x1_slot_2",
    )
    motherboard.visual(
        Box((0.045, 0.010, 0.004)),
        origin=_top_origin(0.065, -0.012, (0.045, 0.010, 0.004)),
        material=gold,
        name="m2_edge_connector",
    )

    pcie_pivot_radius = 0.0012
    pcie_pivot_xyz = (
        PCIE_X16_POS[0] + PCIE_X16_SIZE[0] * 0.5 - 0.0025,
        PCIE_X16_POS[1],
        BOARD_TOP + PCIE_X16_SIZE[2] + pcie_pivot_radius,
    )
    motherboard.visual(
        Box((0.0100, 0.0055, PCIE_X16_SIZE[2])),
        origin=_top_origin(
            pcie_pivot_xyz[0],
            pcie_pivot_xyz[1],
            (0.0100, 0.0055, PCIE_X16_SIZE[2]),
        ),
        material=slot_black,
        name="pcie_latch_pedestal",
    )

    motherboard.visual(
        Box((0.072, 0.028, 0.0045)),
        origin=_top_origin(-0.010, 0.091, (0.072, 0.028, 0.0045)),
        material=heatsink_blue,
        name="vrm_sink_base",
    )
    vrm_fin_count = 7
    vrm_fin_pitch = 0.009
    for fin_index in range(vrm_fin_count):
        fin_x = -0.010 - (vrm_fin_pitch * 0.5 * (vrm_fin_count - 1)) + fin_index * vrm_fin_pitch
        motherboard.visual(
            Box((0.0022, 0.024, 0.024)),
            origin=_top_origin(fin_x, 0.091, (0.0022, 0.024, 0.024), z0=BOARD_TOP + 0.0045),
            material=heatsink_blue,
            name=f"vrm_fin_{fin_index + 1}",
        )

    motherboard.visual(
        Box((0.041, 0.038, 0.0040)),
        origin=_top_origin(0.032, -0.041, (0.041, 0.038, 0.0040)),
        material=heatsink_blue,
        name="chipset_sink_base",
    )
    chipset_fin_count = 6
    chipset_fin_pitch = 0.006
    for fin_index in range(chipset_fin_count):
        fin_x = 0.032 - (chipset_fin_pitch * 0.5 * (chipset_fin_count - 1)) + fin_index * chipset_fin_pitch
        motherboard.visual(
            Box((0.0018, 0.032, 0.020)),
            origin=_top_origin(fin_x, -0.041, (0.0018, 0.032, 0.020), z0=BOARD_TOP + 0.0040),
            material=heatsink_blue,
            name=f"chipset_fin_{fin_index + 1}",
        )

    motherboard.visual(
        Box((0.011, 0.042, 0.012)),
        origin=_top_origin(0.141, 0.040, (0.011, 0.042, 0.012)),
        material=accent_grey,
        name="atx_power_socket",
    )
    motherboard.visual(
        Box((0.028, 0.011, 0.012)),
        origin=_top_origin(0.091, 0.107, (0.028, 0.011, 0.012)),
        material=accent_grey,
        name="eps_power_socket",
    )
    motherboard.visual(
        Box((0.046, 0.012, 0.010)),
        origin=_top_origin(-0.118, -0.100, (0.046, 0.012, 0.010)),
        material=accent_grey,
        name="sata_ports",
    )

    rear_io_y = BOARD_Y * 0.5
    motherboard.visual(
        Box((0.032, 0.024, 0.028)),
        origin=_top_origin(-0.134, rear_io_y, (0.032, 0.024, 0.028)),
        material=gunmetal,
        name="rear_io_shroud",
    )
    motherboard.visual(
        Box((0.018, 0.016, 0.016)),
        origin=_top_origin(-0.112, rear_io_y, (0.018, 0.016, 0.016)),
        material=aluminum,
        name="rear_usb_stack",
    )
    motherboard.visual(
        Box((0.015, 0.004, 0.010)),
        origin=_top_origin(-0.112, rear_io_y - 0.002, (0.015, 0.004, 0.010), z0=BOARD_TOP + 0.003),
        material=io_black,
        name="rear_usb_insert",
    )
    motherboard.visual(
        Box((0.018, 0.020, 0.020)),
        origin=_top_origin(-0.089, rear_io_y, (0.018, 0.020, 0.020)),
        material=aluminum,
        name="rear_ethernet",
    )
    motherboard.visual(
        Box((0.018, 0.016, 0.018)),
        origin=_top_origin(-0.064, rear_io_y, (0.018, 0.016, 0.018)),
        material=aluminum,
        name="rear_video",
    )
    motherboard.visual(
        Box((0.026, 0.016, 0.016)),
        origin=_top_origin(-0.035, rear_io_y, (0.026, 0.016, 0.016)),
        material=aluminum,
        name="rear_audio_block",
    )
    for jack_index, jack_x in enumerate((-0.043, -0.035, -0.027), start=1):
        motherboard.visual(
            Cylinder(radius=0.0045, length=0.006),
            origin=Origin(
                xyz=(jack_x, rear_io_y + 0.003, BOARD_TOP + 0.008),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=io_black,
            name=f"rear_audio_jack_{jack_index}",
        )
    motherboard.visual(
        Box((0.014, 0.003, 0.008)),
        origin=_top_origin(-0.064, rear_io_y + 0.0025, (0.014, 0.003, 0.008), z0=BOARD_TOP + 0.004),
        material=io_black,
        name="rear_video_insert",
    )
    motherboard.visual(
        Box((0.012, 0.003, 0.010)),
        origin=_top_origin(-0.089, rear_io_y + 0.0025, (0.012, 0.003, 0.010), z0=BOARD_TOP + 0.004),
        material=io_black,
        name="rear_ethernet_insert",
    )

    motherboard.inertial = Inertial.from_geometry(
        Box((BOARD_X, BOARD_Y, 0.050)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    cpu_arm = model.part("cpu_retention_arm")
    cpu_arm.visual(
        Cylinder(radius=cpu_pivot_radius, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=aluminum,
        name="pivot_barrel",
    )
    cpu_arm_wire = wire_from_points(
        [
            (0.0, 0.0, 0.0),
            (0.012, 0.0, 0.0002),
            (0.026, 0.0, -0.0001),
            (0.035, 0.002, -0.0018),
        ],
        radius=0.0011,
        radial_segments=12,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.0025,
        corner_segments=8,
    )
    cpu_arm.visual(
        _save_mesh("cpu_retention_arm.obj", cpu_arm_wire),
        material=aluminum,
        name="arm_wire",
    )
    cpu_arm.visual(
        Box((0.006, 0.0032, 0.0022)),
        origin=Origin(xyz=(0.036, 0.0022, -0.0016)),
        material=aluminum,
        name="arm_hook",
    )
    cpu_arm.inertial = Inertial.from_geometry(
        Box((0.042, 0.012, 0.008)),
        mass=0.010,
        origin=Origin(xyz=(0.021, 0.0, -0.001)),
    )

    ram_top_latch = model.part("ram_latch_top")
    ram_top_latch.visual(
        Cylinder(radius=ram_top_pivot_radius, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=latch_grey,
        name="pivot_barrel",
    )
    ram_top_latch.visual(
        Box((0.010, 0.018, 0.0028)),
        origin=Origin(xyz=(0.0, 0.0082, 0.0005)),
        material=latch_grey,
        name="latch_body",
    )
    ram_top_latch.visual(
        Box((0.010, 0.005, 0.007)),
        origin=Origin(xyz=(0.0, 0.0152, 0.0036)),
        material=latch_grey,
        name="latch_tab",
    )
    ram_top_latch.inertial = Inertial.from_geometry(
        Box((0.012, 0.022, 0.011)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.010, 0.003)),
    )

    ram_bottom_latch = model.part("ram_latch_bottom")
    ram_bottom_latch.visual(
        Cylinder(radius=ram_top_pivot_radius, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=latch_grey,
        name="pivot_barrel",
    )
    ram_bottom_latch.visual(
        Box((0.010, 0.018, 0.0028)),
        origin=Origin(xyz=(0.0, -0.0082, 0.0005)),
        material=latch_grey,
        name="latch_body",
    )
    ram_bottom_latch.visual(
        Box((0.010, 0.005, 0.007)),
        origin=Origin(xyz=(0.0, -0.0152, 0.0036)),
        material=latch_grey,
        name="latch_tab",
    )
    ram_bottom_latch.inertial = Inertial.from_geometry(
        Box((0.012, 0.022, 0.011)),
        mass=0.004,
        origin=Origin(xyz=(0.0, -0.010, 0.003)),
    )

    pcie_latch = model.part("pcie_slot_latch")
    pcie_latch.visual(
        Cylinder(radius=pcie_pivot_radius, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=latch_grey,
        name="pivot_barrel",
    )
    pcie_latch.visual(
        Box((0.011, 0.016, 0.0032)),
        origin=Origin(xyz=(0.0, 0.0072, 0.0006)),
        material=latch_grey,
        name="latch_body",
    )
    pcie_latch.visual(
        Box((0.011, 0.005, 0.009)),
        origin=Origin(xyz=(0.0, 0.0134, 0.0040)),
        material=latch_grey,
        name="latch_tab",
    )
    pcie_latch.inertial = Inertial.from_geometry(
        Box((0.013, 0.018, 0.012)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.009, 0.003)),
    )

    model.articulation(
        "cpu_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=cpu_arm,
        origin=Origin(xyz=cpu_pivot_xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(86.0),
        ),
    )
    model.articulation(
        "ram_top_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=ram_top_latch,
        origin=Origin(xyz=ram_top_pivot_xyz),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.1,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "ram_bottom_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=ram_bottom_latch,
        origin=Origin(xyz=ram_bottom_pivot_xyz),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.1,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "pcie_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=pcie_latch,
        origin=Origin(xyz=pcie_pivot_xyz),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.1,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    motherboard = object_model.get_part("motherboard")
    cpu_arm = object_model.get_part("cpu_retention_arm")
    ram_top_latch = object_model.get_part("ram_latch_top")
    ram_bottom_latch = object_model.get_part("ram_latch_bottom")
    pcie_latch = object_model.get_part("pcie_slot_latch")

    cpu_arm_hinge = object_model.get_articulation("cpu_arm_hinge")
    ram_top_hinge = object_model.get_articulation("ram_top_latch_hinge")
    ram_bottom_hinge = object_model.get_articulation("ram_bottom_latch_hinge")
    pcie_hinge = object_model.get_articulation("pcie_latch_hinge")

    pcb = motherboard.get_visual("pcb")
    cpu_socket = motherboard.get_visual("cpu_socket_body")
    ram_slot_outer = motherboard.get_visual("ram_slot_4")
    pcie_x16 = motherboard.get_visual("pcie_x16_slot")
    vrm_sink = motherboard.get_visual("vrm_sink_base")
    chipset_sink = motherboard.get_visual("chipset_sink_base")
    vrm_fin_mid = motherboard.get_visual("vrm_fin_4")
    chipset_fin_mid = motherboard.get_visual("chipset_fin_3")
    rear_usb = motherboard.get_visual("rear_usb_stack")
    rear_audio = motherboard.get_visual("rear_audio_block")
    cpu_pedestal = motherboard.get_visual("cpu_arm_pedestal")
    ram_top_pedestal = motherboard.get_visual("ram_top_latch_pedestal")
    ram_bottom_pedestal = motherboard.get_visual("ram_bottom_latch_pedestal")
    pcie_pedestal = motherboard.get_visual("pcie_latch_pedestal")

    cpu_pivot = cpu_arm.get_visual("pivot_barrel")
    cpu_wire = cpu_arm.get_visual("arm_wire")
    ram_top_pivot = ram_top_latch.get_visual("pivot_barrel")
    ram_top_body = ram_top_latch.get_visual("latch_body")
    ram_bottom_pivot = ram_bottom_latch.get_visual("pivot_barrel")
    ram_bottom_body = ram_bottom_latch.get_visual("latch_body")
    pcie_pivot = pcie_latch.get_visual("pivot_barrel")
    pcie_body = pcie_latch.get_visual("latch_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    pcb_aabb = ctx.part_element_world_aabb(motherboard, elem=pcb)
    cpu_socket_aabb = ctx.part_element_world_aabb(motherboard, elem=cpu_socket)
    ram_slot_aabb = ctx.part_element_world_aabb(motherboard, elem=ram_slot_outer)
    pcie_aabb = ctx.part_element_world_aabb(motherboard, elem=pcie_x16)
    vrm_aabb = ctx.part_element_world_aabb(motherboard, elem=vrm_sink)
    chipset_aabb = ctx.part_element_world_aabb(motherboard, elem=chipset_sink)
    vrm_fin_aabb = ctx.part_element_world_aabb(motherboard, elem=vrm_fin_mid)
    chipset_fin_aabb = ctx.part_element_world_aabb(motherboard, elem=chipset_fin_mid)
    rear_usb_aabb = ctx.part_element_world_aabb(motherboard, elem=rear_usb)
    rear_audio_aabb = ctx.part_element_world_aabb(motherboard, elem=rear_audio)

    assert pcb_aabb is not None
    assert cpu_socket_aabb is not None
    assert ram_slot_aabb is not None
    assert pcie_aabb is not None
    assert vrm_aabb is not None
    assert chipset_aabb is not None
    assert vrm_fin_aabb is not None
    assert chipset_fin_aabb is not None
    assert rear_usb_aabb is not None
    assert rear_audio_aabb is not None

    pcb_size = _aabb_size(pcb_aabb)
    cpu_center = _aabb_center(cpu_socket_aabb)
    ram_center = _aabb_center(ram_slot_aabb)
    pcie_size = _aabb_size(pcie_aabb)

    ctx.check(
        "pcb_atx_scale",
        0.300 <= pcb_size[0] <= 0.310 and 0.239 <= pcb_size[1] <= 0.247,
        f"Expected ATX-like PCB footprint, got {pcb_size}",
    )
    ctx.check(
        "cpu_socket_in_upper_left_quadrant",
        cpu_center[0] < -0.015 and cpu_center[1] > 0.0,
        f"CPU socket center should sit upper-left of board center, got {cpu_center}",
    )
    ctx.check(
        "ram_bank_on_right_side",
        ram_center[0] > 0.085 and ram_slot_aabb[1][1] > 0.070 and ram_slot_aabb[0][1] < -0.050,
        f"Expected outer RAM slot bank along the right edge, got center {ram_center} and aabb {ram_slot_aabb}",
    )
    ctx.check(
        "pcie_slot_reads_long_expansion_slot",
        pcie_size[0] > 0.165 and _aabb_center(pcie_aabb)[1] < -0.040,
        f"Expected a long lower expansion slot, got {pcie_size} at {_aabb_center(pcie_aabb)}",
    )
    ctx.check(
        "rear_io_straddles_board_edge",
        rear_usb_aabb[0][1] < pcb_aabb[1][1] and rear_usb_aabb[1][1] > pcb_aabb[1][1],
        f"Rear USB stack should project from rear edge, got usb {rear_usb_aabb} vs pcb {pcb_aabb}",
    )
    ctx.check(
        "rear_audio_cluster_on_rear_edge",
        rear_audio_aabb[0][1] < pcb_aabb[1][1] and rear_audio_aabb[1][1] > pcb_aabb[1][1],
        f"Rear audio block should sit on rear edge, got {rear_audio_aabb} vs pcb {pcb_aabb}",
    )
    ctx.check(
        "heatsinks_stand_above_connectors",
        vrm_fin_aabb[1][2] > ram_slot_aabb[1][2] + 0.010 and chipset_fin_aabb[1][2] > pcie_aabb[1][2] + 0.008,
        f"Heatsinks should rise above nearby slots, got vrm base {vrm_aabb}, vrm fin {vrm_fin_aabb}, chipset base {chipset_aabb}, chipset fin {chipset_fin_aabb}, ram {ram_slot_aabb}, pcie {pcie_aabb}",
    )

    ctx.expect_contact(
        cpu_arm,
        motherboard,
        elem_a=cpu_pivot,
        elem_b=cpu_pedestal,
        name="cpu_arm_pivot_contacts_socket_mount",
    )
    ctx.expect_contact(
        ram_top_latch,
        motherboard,
        elem_a=ram_top_pivot,
        elem_b=ram_top_pedestal,
        name="ram_top_latch_pivot_contacts_slot_end",
    )
    ctx.expect_contact(
        ram_bottom_latch,
        motherboard,
        elem_a=ram_bottom_pivot,
        elem_b=ram_bottom_pedestal,
        name="ram_bottom_latch_pivot_contacts_slot_end",
    )
    ctx.expect_contact(
        pcie_latch,
        motherboard,
        elem_a=pcie_pivot,
        elem_b=pcie_pedestal,
        name="pcie_latch_pivot_contacts_slot_end",
    )

    ctx.expect_within(
        cpu_arm,
        motherboard,
        axes="xy",
        inner_elem=cpu_wire,
        outer_elem=pcb,
        margin=0.003,
        name="cpu_arm_stays_over_board_footprint",
    )
    ctx.expect_gap(
        cpu_arm,
        motherboard,
        axis="z",
        positive_elem=cpu_wire,
        negative_elem=cpu_socket,
        min_gap=0.0002,
        max_gap=0.0030,
        name="cpu_arm_clears_socket_frame_when_closed",
    )
    ctx.expect_gap(
        ram_top_latch,
        motherboard,
        axis="z",
        positive_elem=ram_top_body,
        negative_elem=ram_slot_outer,
        min_gap=0.0002,
        max_gap=0.0040,
        name="ram_top_latch_sits_just_above_slot",
    )
    ctx.expect_gap(
        ram_bottom_latch,
        motherboard,
        axis="z",
        positive_elem=ram_bottom_body,
        negative_elem=ram_slot_outer,
        min_gap=0.0002,
        max_gap=0.0040,
        name="ram_bottom_latch_sits_just_above_slot",
    )
    ctx.expect_gap(
        pcie_latch,
        motherboard,
        axis="z",
        positive_elem=pcie_body,
        negative_elem=pcie_x16,
        min_gap=0.0,
        max_gap=0.0040,
        name="pcie_latch_sits_just_above_slot",
    )

    cpu_closed_aabb = ctx.part_world_aabb(cpu_arm)
    ram_top_closed_aabb = ctx.part_world_aabb(ram_top_latch)
    ram_bottom_closed_aabb = ctx.part_world_aabb(ram_bottom_latch)
    pcie_closed_aabb = ctx.part_world_aabb(pcie_latch)
    assert cpu_closed_aabb is not None
    assert ram_top_closed_aabb is not None
    assert ram_bottom_closed_aabb is not None
    assert pcie_closed_aabb is not None

    with ctx.pose({cpu_arm_hinge: math.radians(84.0)}):
        cpu_open_aabb = ctx.part_world_aabb(cpu_arm)
        assert cpu_open_aabb is not None
        ctx.expect_contact(cpu_arm, motherboard, elem_a=cpu_pivot, elem_b=cpu_pedestal)
        ctx.check(
            "cpu_arm_lifts_upward",
            cpu_open_aabb[1][2] > cpu_closed_aabb[1][2] + 0.020,
            f"CPU arm should swing upright, got closed {cpu_closed_aabb} open {cpu_open_aabb}",
        )

    with ctx.pose({ram_top_hinge: math.radians(88.0)}):
        ram_top_open_aabb = ctx.part_world_aabb(ram_top_latch)
        assert ram_top_open_aabb is not None
        ctx.expect_contact(ram_top_latch, motherboard, elem_a=ram_top_pivot, elem_b=ram_top_pedestal)
        ctx.check(
            "ram_top_latch_opens_upward",
            ram_top_open_aabb[1][2] > ram_top_closed_aabb[1][2] + 0.009,
            f"Top RAM latch should rotate up, got closed {ram_top_closed_aabb} open {ram_top_open_aabb}",
        )

    with ctx.pose({ram_bottom_hinge: math.radians(88.0)}):
        ram_bottom_open_aabb = ctx.part_world_aabb(ram_bottom_latch)
        assert ram_bottom_open_aabb is not None
        ctx.expect_contact(ram_bottom_latch, motherboard, elem_a=ram_bottom_pivot, elem_b=ram_bottom_pedestal)
        ctx.check(
            "ram_bottom_latch_opens_upward",
            ram_bottom_open_aabb[1][2] > ram_bottom_closed_aabb[1][2] + 0.009,
            f"Bottom RAM latch should rotate up, got closed {ram_bottom_closed_aabb} open {ram_bottom_open_aabb}",
        )

    with ctx.pose({pcie_hinge: math.radians(76.0)}):
        pcie_open_aabb = ctx.part_world_aabb(pcie_latch)
        assert pcie_open_aabb is not None
        ctx.expect_contact(pcie_latch, motherboard, elem_a=pcie_pivot, elem_b=pcie_pedestal)
        ctx.check(
            "pcie_latch_opens_upward",
            pcie_open_aabb[1][2] > pcie_closed_aabb[1][2] + 0.008,
            f"PCIe latch should tip up, got closed {pcie_closed_aabb} open {pcie_open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
