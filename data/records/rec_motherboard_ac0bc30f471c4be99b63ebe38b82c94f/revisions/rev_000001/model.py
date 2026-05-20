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
    mesh_from_geometry,
    wire_from_points,
)


BOARD_X = 0.305
BOARD_Y = 0.244
BOARD_T = 0.0018
BOARD_TOP_Z = BOARD_T * 0.5

SOCKET_X = -0.040
SOCKET_Y = 0.055
SOCKET_W = 0.078
SOCKET_H = 0.006

DIMM_SLOT_W = 0.008
DIMM_SLOT_L = 0.136
DIMM_SLOT_H = 0.012
DIMM_SLOT_Y = 0.018
DIMM_SLOT_XS = (0.056, 0.0675, 0.079, 0.0905)

PCIe_SLOT_X = -0.006
PCIe_SLOT_Y = -0.094
PCIe_SLOT_L = 0.172
PCIe_SLOT_W = 0.014
PCIe_SLOT_H = 0.012

M2_BAY_X = 0.032
M2_BAY_Y = -0.022
M2_BAY_L = 0.084
M2_BAY_W = 0.024


def _box(part, name, size, center, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_motherboard")

    pcb = model.material("pcb", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.11, 0.11, 0.12, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.23, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.76, 1.0))
    connector = model.material("connector", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.18, 0.19, 1.0))
    accent = model.material("accent", rgba=(0.42, 0.05, 0.06, 1.0))

    motherboard = model.part("motherboard")

    _box(motherboard, "pcb", (BOARD_X, BOARD_Y, BOARD_T), (0.0, 0.0, 0.0), pcb)
    _box(
        motherboard,
        "io_shroud",
        (0.036, 0.100, 0.038),
        (-BOARD_X * 0.5 + 0.018, 0.056, BOARD_TOP_Z + 0.019),
        gunmetal,
    )
    _box(
        motherboard,
        "vrm_sink_top",
        (0.094, 0.028, 0.024),
        (-0.070, 0.103, BOARD_TOP_Z + 0.012),
        gunmetal,
    )
    _box(
        motherboard,
        "vrm_sink_left",
        (0.032, 0.080, 0.024),
        (-0.100, 0.058, BOARD_TOP_Z + 0.012),
        gunmetal,
    )
    _box(
        motherboard,
        "cpu_socket_frame",
        (SOCKET_W, SOCKET_W, SOCKET_H),
        (SOCKET_X, SOCKET_Y, BOARD_TOP_Z + SOCKET_H * 0.5),
        steel,
    )
    _box(
        motherboard,
        "cpu_socket_inner",
        (0.060, 0.060, 0.003),
        (SOCKET_X, SOCKET_Y, BOARD_TOP_Z + 0.0015),
        dark_gray,
    )
    _box(
        motherboard,
        "chipset_sink",
        (0.056, 0.050, 0.020),
        (-0.050, -0.010, BOARD_TOP_Z + 0.010),
        gunmetal,
    )
    _box(
        motherboard,
        "audio_shroud",
        (0.084, 0.026, 0.008),
        (-0.090, -0.105, BOARD_TOP_Z + 0.004),
        dark_gray,
    )
    _box(
        motherboard,
        "atx_power",
        (0.008, 0.056, 0.014),
        (BOARD_X * 0.5 - 0.004, 0.038, BOARD_TOP_Z + 0.007),
        connector,
    )
    _box(
        motherboard,
        "eps_power",
        (0.044, 0.012, 0.012),
        (-0.116, BOARD_Y * 0.5 - 0.006, BOARD_TOP_Z + 0.006),
        connector,
    )
    _box(
        motherboard,
        "sata_stack",
        (0.020, 0.046, 0.016),
        (BOARD_X * 0.5 - 0.010, -0.080, BOARD_TOP_Z + 0.008),
        connector,
    )
    _box(
        motherboard,
        "pcie_primary",
        (PCIe_SLOT_L, PCIe_SLOT_W, PCIe_SLOT_H),
        (PCIe_SLOT_X, PCIe_SLOT_Y, BOARD_TOP_Z + PCIe_SLOT_H * 0.5),
        dark_slot,
    )
    _box(
        motherboard,
        "pcie_primary_shield",
        (0.146, 0.004, 0.002),
        (PCIe_SLOT_X - 0.004, PCIe_SLOT_Y, BOARD_TOP_Z + PCIe_SLOT_H + 0.001),
        steel,
    )
    _box(
        motherboard,
        "pcie_x1_top",
        (0.038, 0.013, 0.011),
        (-0.040, -0.064, BOARD_TOP_Z + 0.0055),
        dark_slot,
    )
    _box(
        motherboard,
        "pcie_x1_mid",
        (0.038, 0.013, 0.011),
        (-0.040, -0.041, BOARD_TOP_Z + 0.0055),
        dark_slot,
    )
    _box(
        motherboard,
        "m2_bay",
        (M2_BAY_L, M2_BAY_W, 0.003),
        (M2_BAY_X, M2_BAY_Y, BOARD_TOP_Z + 0.0015),
        dark_gray,
    )
    motherboard.visual(
        Cylinder(radius=0.003, length=0.004),
        origin=Origin(xyz=(M2_BAY_X + 0.037, M2_BAY_Y, BOARD_TOP_Z + 0.002)),
        material=steel,
        name="m2_standoff",
    )

    cpu_arm_joint_xyz = (0.001, SOCKET_Y + SOCKET_W * 0.5 - 0.005, BOARD_TOP_Z + SOCKET_H)
    _box(
        motherboard,
        "cpu_arm_post",
        (0.006, 0.006, 0.004),
        (cpu_arm_joint_xyz[0], cpu_arm_joint_xyz[1], cpu_arm_joint_xyz[2] - 0.002),
        steel,
    )

    dimm_joint_y = DIMM_SLOT_Y + DIMM_SLOT_L * 0.5
    dimm_joint_z = BOARD_TOP_Z + DIMM_SLOT_H
    for index, slot_x in enumerate(DIMM_SLOT_XS, start=1):
        _box(
            motherboard,
            f"dimm_slot_{index}",
            (DIMM_SLOT_W, DIMM_SLOT_L, DIMM_SLOT_H),
            (slot_x, DIMM_SLOT_Y, BOARD_TOP_Z + DIMM_SLOT_H * 0.5),
            dark_slot,
        )
        _box(
            motherboard,
            f"dimm_bottom_stop_{index}",
            (0.010, 0.006, 0.010),
            (slot_x, DIMM_SLOT_Y - DIMM_SLOT_L * 0.5, BOARD_TOP_Z + 0.005),
            dark_slot,
        )
        _box(
            motherboard,
            f"dimm{index}_pad",
            (DIMM_SLOT_W, 0.004, 0.004),
            (slot_x, dimm_joint_y + 0.002, dimm_joint_z - 0.002),
            dark_slot,
        )

    pcie_clip_joint_xyz = (
        PCIe_SLOT_X + PCIe_SLOT_L * 0.5,
        PCIe_SLOT_Y,
        BOARD_TOP_Z + PCIe_SLOT_H,
    )
    _box(
        motherboard,
        "pcie_clip_pad",
        (0.004, 0.012, 0.004),
        (pcie_clip_joint_xyz[0] + 0.002, pcie_clip_joint_xyz[1], pcie_clip_joint_xyz[2] - 0.002),
        dark_slot,
    )

    m2_cover_joint_xyz = (M2_BAY_X - M2_BAY_L * 0.5, M2_BAY_Y, BOARD_TOP_Z + 0.004)
    _box(
        motherboard,
        "m2_hinge_pad",
        (0.004, M2_BAY_W, 0.004),
        (m2_cover_joint_xyz[0] + 0.002, m2_cover_joint_xyz[1], m2_cover_joint_xyz[2] - 0.002),
        dark_gray,
    )

    for index, cap_xy in enumerate(
        (
            (-0.118, -0.091),
            (-0.108, -0.091),
            (-0.098, -0.091),
            (-0.088, -0.091),
        ),
        start=1,
    ):
        motherboard.visual(
            Cylinder(radius=0.004, length=0.010),
            origin=Origin(xyz=(cap_xy[0], cap_xy[1], BOARD_TOP_Z + 0.005)),
            material=dark_gray,
            name=f"audio_cap_{index}",
        )

    cpu_retention_arm = model.part("cpu_retention_arm")
    _box(cpu_retention_arm, "pivot_pad", (0.006, 0.006, 0.002), (0.0, 0.0, 0.001), steel)
    cpu_arm_wire = wire_from_points(
        [
            (0.0, 0.0, 0.0024),
            (0.0, -0.058, 0.0024),
            (0.016, -0.076, 0.0024),
        ],
        radius=0.0014,
        radial_segments=14,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.007,
        corner_segments=8,
    )
    cpu_retention_arm.visual(
        mesh_from_geometry(cpu_arm_wire, "cpu_retention_arm_wire"),
        material=steel,
        name="arm_wire",
    )

    model.articulation(
        "cpu_retention_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=cpu_retention_arm,
        origin=Origin(xyz=cpu_arm_joint_xyz),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    for index, slot_x in enumerate(DIMM_SLOT_XS, start=1):
        latch = model.part(f"memory_latch_{index}")
        _box(latch, "pivot_foot", (0.008, 0.004, 0.002), (0.0, 0.002, 0.001), dark_slot)
        _box(latch, "latch_blade", (0.008, 0.003, 0.020), (0.0, 0.0015, 0.012), dark_slot)
        _box(latch, "finger_tab", (0.010, 0.006, 0.004), (0.0, 0.004, 0.020), accent)
        model.articulation(
            f"memory_latch_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=motherboard,
            child=latch,
            origin=Origin(xyz=(slot_x, dimm_joint_y, dimm_joint_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=3.0,
                lower=0.0,
                upper=math.radians(55.0),
            ),
        )

    pcie_retention_clip = model.part("pcie_retention_clip")
    _box(pcie_retention_clip, "pivot_foot", (0.004, 0.012, 0.002), (0.002, 0.0, 0.001), dark_slot)
    _box(pcie_retention_clip, "clip_body", (0.010, 0.012, 0.016), (0.005, 0.0, 0.010), dark_slot)
    _box(pcie_retention_clip, "clip_hook", (0.004, 0.012, 0.004), (0.010, 0.0, 0.004), accent)
    model.articulation(
        "pcie_retention_clip_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=pcie_retention_clip,
        origin=Origin(xyz=pcie_clip_joint_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )

    storage_cover = model.part("storage_cover")
    _box(storage_cover, "hinge_foot", (0.004, M2_BAY_W, 0.002), (0.002, 0.0, 0.001), gunmetal)
    storage_cover.visual(
        Cylinder(radius=0.0018, length=M2_BAY_W),
        origin=Origin(xyz=(0.002, 0.0, 0.0032), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    _box(
        storage_cover,
        "cover_panel",
        (M2_BAY_L, M2_BAY_W, 0.003),
        (0.044, 0.0, 0.0035),
        gunmetal,
    )
    _box(
        storage_cover,
        "cover_accent",
        (0.050, 0.005, 0.0012),
        (0.044, 0.0, 0.0056),
        steel,
    )
    model.articulation(
        "storage_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=storage_cover,
        origin=Origin(xyz=m2_cover_joint_xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    motherboard = object_model.get_part("motherboard")
    cpu_retention_arm = object_model.get_part("cpu_retention_arm")
    storage_cover = object_model.get_part("storage_cover")
    pcie_retention_clip = object_model.get_part("pcie_retention_clip")
    memory_latches = [object_model.get_part(f"memory_latch_{index}") for index in range(1, 5)]

    cpu_retention_arm_hinge = object_model.get_articulation("cpu_retention_arm_hinge")
    storage_cover_hinge = object_model.get_articulation("storage_cover_hinge")
    pcie_retention_clip_hinge = object_model.get_articulation("pcie_retention_clip_hinge")
    memory_latch_hinges = [
        object_model.get_articulation(f"memory_latch_{index}_hinge") for index in range(1, 5)
    ]

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
        "cpu_retention_arm_axis",
        tuple(cpu_retention_arm_hinge.axis) == (0.0, 0.0, 1.0),
        f"axis={cpu_retention_arm_hinge.axis}",
    )
    ctx.check(
        "pcie_retention_clip_axis",
        tuple(pcie_retention_clip_hinge.axis) == (0.0, 1.0, 0.0),
        f"axis={pcie_retention_clip_hinge.axis}",
    )
    ctx.check(
        "storage_cover_axis",
        tuple(storage_cover_hinge.axis) == (0.0, -1.0, 0.0),
        f"axis={storage_cover_hinge.axis}",
    )
    for index, hinge in enumerate(memory_latch_hinges, start=1):
        ctx.check(
            f"memory_latch_{index}_axis",
            tuple(hinge.axis) == (1.0, 0.0, 0.0),
            f"axis={hinge.axis}",
        )

    ctx.expect_contact(
        cpu_retention_arm,
        motherboard,
        elem_a="pivot_pad",
        elem_b="cpu_arm_post",
        name="cpu_retention_arm_mount_contact",
    )
    ctx.expect_overlap(
        cpu_retention_arm,
        motherboard,
        elem_a="pivot_pad",
        elem_b="cpu_arm_post",
        axes="xy",
        min_overlap=0.005,
        name="cpu_retention_arm_mount_alignment",
    )

    for index, latch in enumerate(memory_latches, start=1):
        ctx.expect_contact(
            latch,
            motherboard,
            elem_a="pivot_foot",
            elem_b=f"dimm{index}_pad",
            name=f"memory_latch_{index}_mount_contact",
        )
        ctx.expect_overlap(
            latch,
            motherboard,
            elem_a="pivot_foot",
            elem_b=f"dimm{index}_pad",
            axes="xy",
            min_overlap=0.0035,
            name=f"memory_latch_{index}_mount_alignment",
        )
        ctx.expect_overlap(
            latch,
            motherboard,
            elem_a="latch_blade",
            elem_b=f"dimm_slot_{index}",
            axes="x",
            min_overlap=0.006,
            name=f"memory_latch_{index}_slot_alignment",
        )

    ctx.expect_contact(
        pcie_retention_clip,
        motherboard,
        elem_a="pivot_foot",
        elem_b="pcie_clip_pad",
        name="pcie_retention_clip_mount_contact",
    )
    ctx.expect_overlap(
        pcie_retention_clip,
        motherboard,
        elem_a="pivot_foot",
        elem_b="pcie_clip_pad",
        axes="xy",
        min_overlap=0.0035,
        name="pcie_retention_clip_mount_alignment",
    )
    ctx.expect_overlap(
        pcie_retention_clip,
        motherboard,
        elem_a="clip_body",
        elem_b="pcie_primary",
        axes="y",
        min_overlap=0.010,
        name="pcie_clip_at_slot_end",
    )

    ctx.expect_contact(
        storage_cover,
        motherboard,
        elem_a="hinge_foot",
        elem_b="m2_hinge_pad",
        name="storage_cover_mount_contact",
    )
    ctx.expect_overlap(
        storage_cover,
        motherboard,
        elem_a="hinge_foot",
        elem_b="m2_hinge_pad",
        axes="xy",
        min_overlap=0.0035,
        name="storage_cover_mount_alignment",
    )
    ctx.expect_overlap(
        storage_cover,
        motherboard,
        elem_a="cover_panel",
        elem_b="m2_bay",
        axes="xy",
        min_overlap=0.020,
        name="storage_cover_over_bay",
    )
    ctx.expect_gap(
        storage_cover,
        motherboard,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="m2_bay",
        min_gap=0.002,
        max_gap=0.0045,
        name="storage_cover_closed_clearance",
    )

    cover_closed_aabb = ctx.part_element_world_aabb(storage_cover, elem="cover_panel")
    ctx.check(
        "storage_cover_panel_aabb_available_closed",
        cover_closed_aabb is not None,
        "cover_panel AABB unavailable in closed pose",
    )
    with ctx.pose({storage_cover_hinge: math.radians(80.0)}):
        cover_open_aabb = ctx.part_element_world_aabb(storage_cover, elem="cover_panel")
        ctx.check(
            "storage_cover_panel_aabb_available_open",
            cover_open_aabb is not None,
            "cover_panel AABB unavailable in open pose",
        )
        if cover_closed_aabb is not None and cover_open_aabb is not None:
            ctx.check(
                "storage_cover_open_lift",
                cover_open_aabb[1][2] > cover_closed_aabb[1][2] + 0.060,
                (
                    f"closed_max_z={cover_closed_aabb[1][2]:.6f} "
                    f"open_max_z={cover_open_aabb[1][2]:.6f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
