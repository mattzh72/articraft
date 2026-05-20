from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BOARD_L = 0.305
BOARD_W = 0.244
PCB_T = 0.0018

CPU_SOCKET_ORIGIN = (-0.030, 0.045, PCB_T)
MEMORY_BANK_ORIGIN = (0.096, 0.044, PCB_T)
REAR_IO_ORIGIN = (-0.138, 0.066, PCB_T)
EXPANSION_ORIGIN = (-0.010, -0.058, PCB_T)
VRM_ORIGIN = (-0.060, 0.094, PCB_T)
CONNECTOR_ORIGIN = (0.129, 0.022, PCB_T)

CPU_ARM_LOCAL_ORIGIN = (0.030, -0.020, 0.004)
PCIE_LATCH_LOCAL_ORIGIN = (0.042, 0.016, 0.004)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder(radius: float, height: float, base: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate(base)


def _ring_plate(outer: float, inner: float, thickness: float, z_center: float) -> cq.Workplane:
    ring = _cq_box((outer, outer, thickness), (0.0, 0.0, z_center))
    cut = _cq_box((inner, inner, thickness + 0.004), (0.0, 0.0, z_center))
    return ring.cut(cut)


def _finned_block(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    fin_axis: str,
    fin_count: int,
) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    block = _cq_box(size, center)
    groove_height = sz * 0.72
    groove_center_z = cz + (sz / 2.0) - (groove_height / 2.0)
    if fin_axis == "x":
        pitch = sx / (fin_count + 1)
        groove_width = pitch * 0.48
        for idx in range(fin_count):
            x = cx - (sx / 2.0) + pitch * (idx + 1)
            groove = _cq_box((groove_width, sy + 0.002, groove_height), (x, cy, groove_center_z))
            block = block.cut(groove)
    else:
        pitch = sy / (fin_count + 1)
        groove_width = pitch * 0.48
        for idx in range(fin_count):
            y = cy - (sy / 2.0) + pitch * (idx + 1)
            groove = _cq_box((sx + 0.002, groove_width, groove_height), (cx, y, groove_center_z))
            block = block.cut(groove)
    return block


def _board_shape() -> cq.Workplane:
    board = _cq_box((BOARD_L, BOARD_W, PCB_T), (0.0, 0.0, PCB_T / 2.0))
    corner_cut = (
        _cq_box(
            (0.028, 0.028, 0.010),
            (BOARD_L / 2.0 - 0.010, -BOARD_W / 2.0 + 0.010, PCB_T / 2.0),
        ).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 45.0)
    )
    board = board.cut(corner_cut)
    for x, y in [
        (-0.139, 0.104),
        (-0.139, -0.103),
        (-0.015, 0.104),
        (0.108, 0.104),
        (0.136, 0.010),
        (0.136, -0.104),
        (-0.016, -0.104),
        (-0.073, -0.014),
        (0.055, -0.014),
    ]:
        board = board.cut(_cq_cylinder(0.0019, 0.012, (x, y, -0.004)))
    return board


def _cpu_socket_body_shape() -> cq.Workplane:
    body = _cq_box((0.068, 0.068, 0.0042), (0.0, 0.0, 0.0018))
    body = body.cut(_cq_box((0.048, 0.048, 0.0030), (0.0, 0.0, 0.0027)))
    body = body.cut(_cq_box((0.020, 0.008, 0.0022), (0.020, -0.026, 0.0014)))
    return body


def _cpu_socket_frame_shape() -> cq.Workplane:
    frame = _ring_plate(0.061, 0.048, 0.0013, 0.0049)
    frame = frame.union(_cq_cylinder(0.0017, 0.008, (CPU_ARM_LOCAL_ORIGIN[0], CPU_ARM_LOCAL_ORIGIN[1], 0.0)))
    frame = frame.union(_cq_box((0.0042, 0.010, 0.0040), (-0.031, 0.022, 0.0020)))
    frame = frame.union(_cq_box((0.010, 0.0040, 0.0012), (0.010, 0.030, 0.0054)))
    return frame


def _rear_io_housing_shape() -> cq.Workplane:
    housing = _cq_box((0.030, 0.098, 0.034), (0.0, 0.0, 0.017))
    bevel_cut = _cq_box((0.040, 0.120, 0.020), (0.012, 0.0, 0.031)).rotate(
        (0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        18.0,
    )
    housing = housing.cut(bevel_cut)
    housing = housing.union(_cq_box((0.024, 0.088, 0.006), (0.002, 0.0, 0.031)))
    housing = housing.union(_cq_box((0.006, 0.098, 0.018), (0.011, 0.0, 0.009)))
    return housing


def _chipset_sink_shape() -> cq.Workplane:
    sink = _finned_block((0.046, 0.036, 0.012), (0.054, 0.022, 0.006), fin_axis="x", fin_count=6)
    sink = sink.union(_cq_box((0.012, 0.020, 0.004), (0.054, 0.022, 0.010)))
    return sink


def _vrm_sink_shape() -> cq.Workplane:
    top = _finned_block((0.060, 0.018, 0.016), (0.006, 0.013, 0.008), fin_axis="x", fin_count=5)
    side = _finned_block((0.018, 0.040, 0.015), (-0.024, -0.004, 0.0075), fin_axis="y", fin_count=4)
    bridge = _cq_box((0.012, 0.014, 0.005), (-0.010, 0.003, 0.0025))
    return top.union(side).union(bridge)


def _cpu_retention_arm_shape() -> cq.Workplane:
    path = cq.Workplane("XY").polyline(
        [
            (0.0015, 0.000),
            (0.0045, 0.004),
            (0.0050, 0.042),
            (0.000, 0.048),
            (-0.006, 0.042),
        ]
    )
    tube = cq.Workplane("YZ").circle(0.00115).sweep(path, transition="round")
    sleeve = cq.Workplane("XY").circle(0.0022).extrude(0.004).translate((0.0, 0.0, -0.002))
    latch_foot = _cq_box((0.0040, 0.0028, 0.0016), (-0.006, 0.042, 0.0008))
    cam = _cq_box((0.0055, 0.0018, 0.0012), (0.005, 0.008, 0.0006))
    return sleeve.union(tube).union(latch_foot).union(cam)


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    name: str | None = None,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_cylinder_visual(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    name: str | None = None,
) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="enthusiast_motherboard", assets=ASSETS)

    solder_mask = model.material("solder_mask", rgba=(0.06, 0.11, 0.08, 1.0))
    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    slot_gray = model.material("slot_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.34, 0.36, 0.39, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.71, 1.0))
    gold = model.material("gold_contacts", rgba=(0.82, 0.70, 0.26, 1.0))
    port_metal = model.material("port_metal", rgba=(0.78, 0.80, 0.82, 1.0))

    pcb = model.part("pcb")
    pcb.visual(mesh_from_cadquery(_board_shape(), "motherboard_pcb.obj", assets=ASSETS), material=solder_mask)
    pcb.inertial = Inertial.from_geometry(
        Box((BOARD_L, BOARD_W, PCB_T)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, PCB_T / 2.0)),
    )

    cpu_socket = model.part("cpu_socket")
    cpu_socket.visual(
        mesh_from_cadquery(_cpu_socket_body_shape(), "cpu_socket_body.obj", assets=ASSETS),
        material=graphite,
    )
    cpu_socket.visual(
        mesh_from_cadquery(_cpu_socket_frame_shape(), "cpu_socket_frame.obj", assets=ASSETS),
        material=steel,
    )
    _add_box_visual(cpu_socket, (0.040, 0.040, 0.0006), (0.0, 0.0, 0.0007), gold)
    cpu_socket.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.008)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    memory_slots = model.part("memory_slots")
    _add_box_visual(memory_slots, (0.056, 0.140, 0.0008), (0.0, 0.0, -0.0002), matte_black)
    slot_xs = (-0.019, -0.006, 0.007, 0.020)
    for x in (slot_xs[0], slot_xs[2]):
        _add_box_visual(memory_slots, (0.0068, 0.132, 0.0080), (x, 0.0, 0.0036), matte_black)
    for x in (slot_xs[1], slot_xs[3]):
        _add_box_visual(memory_slots, (0.0068, 0.132, 0.0080), (x, 0.0, 0.0036), slot_gray)
    for x in slot_xs:
        _add_box_visual(memory_slots, (0.0022, 0.112, 0.0008), (x, 0.0, 0.0005), gold)
    memory_slots.inertial = Inertial.from_geometry(
        Box((0.056, 0.140, 0.010)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    rear_io = model.part("rear_io")
    _add_box_visual(rear_io, (0.034, 0.105, 0.0008), (0.0, 0.0, -0.0002), matte_black)
    rear_io.visual(
        mesh_from_cadquery(_rear_io_housing_shape(), "rear_io_shroud.obj", assets=ASSETS),
        material=gunmetal,
    )
    _add_box_visual(rear_io, (0.018, 0.013, 0.012), (-0.016, 0.042, 0.011), port_metal)
    _add_box_visual(rear_io, (0.018, 0.013, 0.012), (-0.016, 0.022, 0.011), port_metal)
    _add_box_visual(rear_io, (0.021, 0.015, 0.014), (-0.015, -0.004, 0.013), port_metal)
    _add_box_visual(rear_io, (0.018, 0.026, 0.006), (-0.016, -0.035, 0.013), port_metal)
    for idx in range(5):
        _add_cylinder_visual(
            rear_io,
            0.0032,
            0.010,
            (-0.016, -0.043 + idx * 0.009, 0.009),
            port_metal,
            rpy=(0.0, math.pi / 2.0, 0.0),
        )
    rear_io.inertial = Inertial.from_geometry(
        Box((0.034, 0.105, 0.036)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    expansion_zone = model.part("expansion_zone")
    _add_box_visual(expansion_zone, (0.184, 0.072, 0.0008), (0.0, 0.0, -0.0002), matte_black)
    _add_box_visual(expansion_zone, (0.108, 0.010, 0.010), (-0.015, 0.016, 0.0048), matte_black)
    _add_box_visual(expansion_zone, (0.106, 0.010, 0.010), (-0.015, -0.018, 0.0048), matte_black)
    _add_box_visual(expansion_zone, (0.038, 0.008, 0.008), (-0.062, 0.034, 0.0042), matte_black)
    _add_box_visual(expansion_zone, (0.036, 0.008, 0.008), (-0.062, -0.001, 0.0042), matte_black)
    _add_box_visual(expansion_zone, (0.108, 0.003, 0.004), (-0.015, 0.016, 0.0094), steel)
    _add_box_visual(expansion_zone, (0.106, 0.003, 0.004), (-0.015, -0.018, 0.0094), steel)
    expansion_zone.visual(
        mesh_from_cadquery(_chipset_sink_shape(), "chipset_sink.obj", assets=ASSETS),
        material=gunmetal,
    )
    _add_box_visual(expansion_zone, (0.064, 0.017, 0.005), (0.054, -0.017, 0.0026), graphite)
    _add_box_visual(
        expansion_zone,
        (0.012, 0.006, 0.005),
        (PCIE_LATCH_LOCAL_ORIGIN[0], PCIE_LATCH_LOCAL_ORIGIN[1], 0.0025),
        matte_black,
    )
    expansion_zone.inertial = Inertial.from_geometry(
        Box((0.184, 0.072, 0.016)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    vrm_cooling = model.part("vrm_cooling")
    _add_box_visual(vrm_cooling, (0.070, 0.052, 0.0008), (0.0, 0.0, -0.0002), matte_black)
    vrm_cooling.visual(
        mesh_from_cadquery(_vrm_sink_shape(), "vrm_heatsinks.obj", assets=ASSETS),
        material=gunmetal,
    )
    for x, y in [(-0.010, -0.004), (0.004, -0.004), (0.018, -0.004), (-0.010, 0.008)]:
        _add_box_visual(vrm_cooling, (0.010, 0.010, 0.007), (x, y, 0.0035), graphite)
    for x, y in [(-0.031, 0.012), (-0.031, 0.000), (-0.031, -0.012), (0.024, 0.010)]:
        _add_cylinder_visual(vrm_cooling, 0.0030, 0.010, (x, y, 0.005), matte_black)
    vrm_cooling.inertial = Inertial.from_geometry(
        Box((0.070, 0.052, 0.018)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    connectors = model.part("connectors")
    _add_box_visual(connectors, (0.022, 0.150, 0.0008), (0.0, 0.0, -0.0002), matte_black)
    _add_box_visual(connectors, (0.014, 0.056, 0.015), (0.0, 0.025, 0.0075), slot_gray)
    _add_box_visual(connectors, (0.014, 0.024, 0.011), (0.0, 0.082, 0.0055), slot_gray)
    _add_box_visual(connectors, (0.008, 0.020, 0.004), (0.0, 0.060, 0.002), matte_black)
    _add_box_visual(connectors, (0.012, 0.012, 0.006), (0.0, 0.108, 0.003), matte_black)
    _add_box_visual(connectors, (0.006, 0.018, 0.004), (0.0, 0.099, 0.002), matte_black)
    for y in (-0.055, -0.040, -0.025):
        _add_box_visual(connectors, (0.018, 0.012, 0.010), (-0.002, y, 0.005), matte_black)
    _add_box_visual(connectors, (0.008, 0.044, 0.004), (-0.002, -0.040, 0.002), matte_black)
    _add_box_visual(connectors, (0.006, 0.130, 0.0016), (-0.003, 0.012, 0.0008), matte_black)
    connectors.inertial = Inertial.from_geometry(
        Box((0.022, 0.150, 0.016)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    cpu_retention_arm = model.part("cpu_retention_arm")
    cpu_retention_arm.visual(
        mesh_from_cadquery(_cpu_retention_arm_shape(), "cpu_retention_arm.obj", assets=ASSETS),
        material=steel,
    )
    cpu_retention_arm.inertial = Inertial.from_geometry(
        Box((0.018, 0.055, 0.006)),
        mass=0.01,
        origin=Origin(xyz=(0.003, 0.027, 0.0)),
    )

    pcie_latch = model.part("pcie_latch")
    _add_cylinder_visual(
        pcie_latch,
        0.0015,
        0.012,
        (0.0, 0.0, 0.0),
        matte_black,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    _add_box_visual(pcie_latch, (0.012, 0.005, 0.012), (0.0, -0.0012, 0.0060), matte_black)
    _add_box_visual(pcie_latch, (0.012, 0.0038, 0.0030), (0.0, -0.0025, 0.0110), slot_gray)
    pcie_latch.inertial = Inertial.from_geometry(
        Box((0.012, 0.008, 0.013)),
        mass=0.003,
        origin=Origin(xyz=(0.0, -0.001, 0.006)),
    )

    model.articulation(
        "pcb_to_cpu_socket",
        ArticulationType.FIXED,
        parent="pcb",
        child="cpu_socket",
        origin=Origin(xyz=CPU_SOCKET_ORIGIN),
    )
    model.articulation(
        "pcb_to_memory_slots",
        ArticulationType.FIXED,
        parent="pcb",
        child="memory_slots",
        origin=Origin(xyz=MEMORY_BANK_ORIGIN),
    )
    model.articulation(
        "pcb_to_rear_io",
        ArticulationType.FIXED,
        parent="pcb",
        child="rear_io",
        origin=Origin(xyz=REAR_IO_ORIGIN),
    )
    model.articulation(
        "pcb_to_expansion_zone",
        ArticulationType.FIXED,
        parent="pcb",
        child="expansion_zone",
        origin=Origin(xyz=EXPANSION_ORIGIN),
    )
    model.articulation(
        "pcb_to_vrm_cooling",
        ArticulationType.FIXED,
        parent="pcb",
        child="vrm_cooling",
        origin=Origin(xyz=VRM_ORIGIN),
    )
    model.articulation(
        "pcb_to_connectors",
        ArticulationType.FIXED,
        parent="pcb",
        child="connectors",
        origin=Origin(xyz=CONNECTOR_ORIGIN),
    )
    model.articulation(
        "cpu_arm_joint",
        ArticulationType.REVOLUTE,
        parent="cpu_socket",
        child="cpu_retention_arm",
        origin=Origin(xyz=CPU_ARM_LOCAL_ORIGIN),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.12, velocity=3.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "pcie_latch_joint",
        ArticulationType.REVOLUTE,
        parent="expansion_zone",
        child="pcie_latch",
        origin=Origin(xyz=PCIE_LATCH_LOCAL_ORIGIN),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=3.5, lower=0.0, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap("cpu_retention_arm", "cpu_socket", reason="sleeved lever wraps a retention peg")
    ctx.allow_overlap("pcie_latch", "expansion_zone", reason="hinge barrel nests into the slot-end pivot boss")
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    for part_name, min_overlap in [
        ("cpu_socket", 0.030),
        ("memory_slots", 0.020),
        ("rear_io", 0.020),
        ("expansion_zone", 0.040),
        ("vrm_cooling", 0.028),
        ("connectors", 0.018),
    ]:
        ctx.expect_aabb_overlap(part_name, "pcb", axes="xy", min_overlap=min_overlap)
        ctx.expect_aabb_gap(part_name, "pcb", axis="z", max_gap=0.0015, max_penetration=0.0030)

    ctx.expect_aabb_overlap("memory_slots", "cpu_socket", axes="y", min_overlap=0.055)
    ctx.expect_aabb_overlap("rear_io", "cpu_socket", axes="y", min_overlap=0.040)
    ctx.expect_aabb_overlap("expansion_zone", "cpu_socket", axes="x", min_overlap=0.050)
    ctx.expect_aabb_overlap("vrm_cooling", "cpu_socket", axes="x", min_overlap=0.040)
    ctx.expect_aabb_overlap("vrm_cooling", "cpu_socket", axes="y", min_overlap=0.010)
    ctx.expect_aabb_overlap("memory_slots", "connectors", axes="y", min_overlap=0.045)

    ctx.expect_joint_motion_axis(
        "cpu_arm_joint",
        "cpu_retention_arm",
        world_axis="x",
        direction="positive",
        min_delta=0.008,
    )
    ctx.expect_joint_motion_axis(
        "pcie_latch_joint",
        "pcie_latch",
        world_axis="y",
        direction="positive",
        min_delta=0.002,
    )

    with ctx.pose(cpu_arm_joint=0.0):
        ctx.expect_aabb_overlap("cpu_retention_arm", "cpu_socket", axes="y", min_overlap=0.040)
        ctx.expect_aabb_overlap("cpu_retention_arm", "cpu_socket", axes="x", min_overlap=0.008)

    with ctx.pose(cpu_arm_joint=1.15):
        ctx.expect_aabb_overlap("cpu_retention_arm", "pcb", axes="xy", min_overlap=0.003)
        ctx.expect_aabb_overlap("cpu_retention_arm", "cpu_socket", axes="x", min_overlap=0.004)

    with ctx.pose(pcie_latch_joint=0.0):
        ctx.expect_aabb_overlap("pcie_latch", "expansion_zone", axes="x", min_overlap=0.008)

    with ctx.pose(pcie_latch_joint=0.75):
        ctx.expect_aabb_overlap("pcie_latch", "expansion_zone", axes="x", min_overlap=0.008)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
