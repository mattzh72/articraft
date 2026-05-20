from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

BOARD_W = 0.305
BOARD_D = 0.244
PCB_T = 0.0018
BOARD_TOP = PCB_T
BOARD_MIN_X = -BOARD_W / 2.0
BOARD_MAX_X = BOARD_W / 2.0
BOARD_MIN_Y = -BOARD_D / 2.0
BOARD_MAX_Y = BOARD_D / 2.0

CPU_SOCKET_CENTER = (-0.008, 0.032)
CPU_PIVOT = (0.020, 0.032, BOARD_TOP + 0.0062)

DIMM_SLOT_CENTERS_X = (0.060, 0.070, 0.080, 0.090)
DIMM_SLOT_CENTER_Y = 0.018
DIMM_SLOT_LENGTH = 0.126
DIMM_LATCH_ORIGIN = (
    DIMM_SLOT_CENTERS_X[0],
    DIMM_SLOT_CENTER_Y + DIMM_SLOT_LENGTH / 2.0 + 0.003,
    BOARD_TOP + 0.0062,
)

MOUNT_HOLES = (
    (-0.139, 0.108),
    (-0.058, 0.108),
    (0.024, 0.108),
    (0.138, 0.105),
    (-0.139, 0.010),
    (0.008, 0.006),
    (0.138, 0.006),
    (-0.139, -0.107),
    (0.138, -0.107),
)


def _add_box(part, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material)


def _add_cylinder(part, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
    )


def _pcb_shape():
    outline = [
        (BOARD_MIN_X + 0.004, BOARD_MIN_Y),
        (BOARD_MAX_X - 0.020, BOARD_MIN_Y),
        (BOARD_MAX_X, BOARD_MIN_Y + 0.020),
        (BOARD_MAX_X, BOARD_MAX_Y - 0.004),
        (BOARD_MAX_X - 0.004, BOARD_MAX_Y),
        (BOARD_MIN_X, BOARD_MAX_Y),
        (BOARD_MIN_X, BOARD_MIN_Y + 0.010),
    ]
    pcb = cq.Workplane("XY").moveTo(*outline[0]).polyline(outline[1:]).close().extrude(PCB_T)
    hole_cutter = (
        cq.Workplane("XY")
        .pushPoints(MOUNT_HOLES)
        .circle(0.0019)
        .extrude(PCB_T + 0.003)
        .translate((0.0, 0.0, -0.0015))
    )
    return pcb.cut(hole_cutter)


def _io_shroud_shape():
    base = cq.Workplane("XY").box(0.032, 0.074, 0.020, centered=(True, True, False))
    step = (
        cq.Workplane("XY")
        .box(0.024, 0.070, 0.008, centered=(True, True, False))
        .translate((0.003, 0.0, 0.020))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.008, 0.074, 0.004, centered=(True, True, False))
        .translate((0.015, 0.0, 0.028))
    )
    return base.union(step).union(nose)


def _cpu_arm_shape():
    pivot = cq.Workplane("XZ").circle(0.0014).extrude(0.010, both=True)
    arm_path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(-0.018, 0.0)
        .lineTo(-0.028, 0.002)
        .lineTo(-0.040, 0.010)
        .lineTo(-0.052, 0.010)
    )
    rod = cq.Workplane("YZ").circle(0.00085).sweep(arm_path, transition="round")
    hook = cq.Workplane("XY").circle(0.00085).extrude(0.006).translate((-0.046, 0.0, 0.002))
    grip = (
        cq.Workplane("XY")
        .box(0.010, 0.0032, 0.0016, centered=(True, True, False))
        .translate((-0.052, 0.0, 0.0092))
    )
    return pivot.union(rod).union(hook).union(grip)


def _dimm_latch_shape():
    body = (
        cq.Workplane("YZ")
        .moveTo(-0.0015, 0.0002)
        .lineTo(0.0048, 0.0002)
        .lineTo(0.0066, 0.0045)
        .lineTo(0.0032, 0.0122)
        .lineTo(-0.0020, 0.0103)
        .close()
        .extrude(0.006, both=True)
    )
    knuckle = cq.Workplane("YZ").circle(0.0014).extrude(0.006, both=True)
    finger = (
        cq.Workplane("XY")
        .box(0.006, 0.0018, 0.0042, centered=(True, True, False))
        .translate((0.0, 0.0050, 0.0038))
    )
    return body.union(knuckle).union(finger)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="enthusiast_motherboard", assets=ASSETS)

    model.material("pcb", rgba=(0.08, 0.22, 0.11, 1.0))
    model.material("matte_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("dark_gray", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("graphite", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("slot_gray", rgba=(0.43, 0.45, 0.47, 1.0))
    model.material("latch_gray", rgba=(0.63, 0.64, 0.67, 1.0))
    model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("aluminum", rgba=(0.60, 0.62, 0.66, 1.0))
    model.material("gold", rgba=(0.84, 0.70, 0.23, 1.0))
    model.material("copper", rgba=(0.66, 0.39, 0.22, 1.0))
    model.material("accent", rgba=(0.88, 0.90, 0.92, 1.0))

    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_pcb_shape(), "motherboard_pcb.obj", assets=ASSETS),
        material="pcb",
    )
    board.inertial = Inertial.from_geometry(
        Box((BOARD_W, BOARD_D, PCB_T)),
        mass=0.92,
        origin=Origin(xyz=(0.0, 0.0, PCB_T / 2.0)),
    )

    board.visual(
        mesh_from_cadquery(_io_shroud_shape(), "rear_io_shroud.obj", assets=ASSETS),
        origin=Origin(xyz=(BOARD_MIN_X + 0.016, 0.070, BOARD_TOP)),
        material="graphite",
    )

    for y in (0.038, 0.057, 0.076, 0.095):
        _add_box(
            board,
            (0.018, 0.014, 0.014),
            (BOARD_MIN_X + 0.009, y, BOARD_TOP + 0.007 - 0.0001),
            "steel",
        )
    _add_box(
        board,
        (0.020, 0.016, 0.019),
        (BOARD_MIN_X + 0.010, 0.017, BOARD_TOP + 0.0095 - 0.0001),
        "steel",
    )
    _add_box(
        board,
        (0.016, 0.014, 0.014),
        (BOARD_MIN_X + 0.008, -0.003, BOARD_TOP + 0.007 - 0.0001),
        "steel",
    )
    _add_box(
        board,
        (0.024, 0.018, 0.012),
        (-0.092, 0.108, BOARD_TOP + 0.006 - 0.0001),
        "matte_black",
    )
    _add_box(
        board,
        (0.024, 0.018, 0.012),
        (-0.064, 0.108, BOARD_TOP + 0.006 - 0.0001),
        "matte_black",
    )
    _add_box(
        board,
        (0.018, 0.012, 0.004),
        (-0.092, 0.108, BOARD_TOP + 0.002 - 0.0001),
        "gold",
    )
    _add_box(
        board,
        (0.018, 0.012, 0.004),
        (-0.064, 0.108, BOARD_TOP + 0.002 - 0.0001),
        "gold",
    )

    _add_box(
        board,
        (0.062, 0.024, 0.009),
        (-0.018, 0.108, BOARD_TOP + 0.0045 - 0.0001),
        "graphite",
    )
    for i in range(7):
        _add_box(
            board,
            (0.0042, 0.022, 0.010),
            (-0.044 + i * 0.009, 0.108, BOARD_TOP + 0.014 - 0.0001),
            "aluminum",
        )
    _add_box(
        board,
        (0.024, 0.060, 0.009),
        (-0.058, 0.060, BOARD_TOP + 0.0045 - 0.0001),
        "graphite",
    )
    for i in range(6):
        _add_box(
            board,
            (0.022, 0.0040, 0.010),
            (-0.058, 0.035 + i * 0.010, BOARD_TOP + 0.014 - 0.0001),
            "aluminum",
        )

    _add_box(
        board,
        (0.048, 0.050, 0.0032),
        (CPU_SOCKET_CENTER[0], CPU_SOCKET_CENTER[1], BOARD_TOP + 0.0016 - 0.0001),
        "dark_gray",
    )
    _add_box(
        board,
        (0.052, 0.004, 0.0011),
        (CPU_SOCKET_CENTER[0], CPU_SOCKET_CENTER[1] + 0.025, BOARD_TOP + 0.0037 - 0.0001),
        "steel",
    )
    _add_box(
        board,
        (0.052, 0.004, 0.0011),
        (CPU_SOCKET_CENTER[0], CPU_SOCKET_CENTER[1] - 0.025, BOARD_TOP + 0.0037 - 0.0001),
        "steel",
    )
    _add_box(
        board,
        (0.004, 0.050, 0.0011),
        (CPU_SOCKET_CENTER[0] - 0.024, CPU_SOCKET_CENTER[1], BOARD_TOP + 0.0037 - 0.0001),
        "steel",
    )
    _add_box(
        board,
        (0.004, 0.050, 0.0011),
        (CPU_SOCKET_CENTER[0] + 0.024, CPU_SOCKET_CENTER[1], BOARD_TOP + 0.0037 - 0.0001),
        "steel",
    )
    _add_box(
        board,
        (0.038, 0.038, 0.0008),
        (CPU_SOCKET_CENTER[0], CPU_SOCKET_CENTER[1], BOARD_TOP + 0.0047 - 0.0001),
        "accent",
    )
    _add_box(board, (0.005, 0.012, 0.006), CPU_PIVOT, "steel")
    _add_box(
        board,
        (0.007, 0.006, 0.005),
        (CPU_PIVOT[0] - 0.048, CPU_PIVOT[1], BOARD_TOP + 0.0045),
        "steel",
    )

    for x in (-0.046, -0.034, -0.022, -0.010, 0.002, 0.014):
        _add_box(
            board,
            (0.0085, 0.0085, 0.0065),
            (x, 0.084, BOARD_TOP + 0.00325 - 0.0001),
            "matte_black",
        )
        _add_box(
            board,
            (0.0085, 0.0085, 0.0065),
            (x, 0.071, BOARD_TOP + 0.00325 - 0.0001),
            "matte_black",
        )
    for y in (0.022, 0.034, 0.046, 0.058):
        _add_box(
            board,
            (0.0085, 0.0085, 0.0065),
            (-0.078, y, BOARD_TOP + 0.00325 - 0.0001),
            "matte_black",
        )

    for x in (-0.070, -0.082, -0.094):
        for y in (0.080, 0.066, 0.052):
            _add_cylinder(
                board,
                0.0035,
                0.009,
                (x, y, BOARD_TOP + 0.0045 - 0.0001),
                "dark_gray",
            )
            _add_cylinder(
                board,
                0.0024,
                0.001,
                (x, y, BOARD_TOP + 0.0094 - 0.0001),
                "steel",
            )

    for idx, x in enumerate(DIMM_SLOT_CENTERS_X):
        material = "matte_black" if idx % 2 == 0 else "slot_gray"
        _add_box(
            board,
            (0.0074, 0.126, 0.0102),
            (x, DIMM_SLOT_CENTER_Y, BOARD_TOP + 0.0051 - 0.0001),
            material,
        )
        _add_box(
            board,
            (0.0020, 0.112, 0.0011),
            (x, DIMM_SLOT_CENTER_Y, BOARD_TOP + 0.00055),
            "gold",
        )
        _add_box(
            board,
            (0.0090, 0.008, 0.0108),
            (x, DIMM_SLOT_CENTER_Y - 0.066, BOARD_TOP + 0.0054 - 0.0001),
            "slot_gray",
        )
        if idx > 0:
            _add_box(
                board,
                (0.0090, 0.008, 0.0108),
                (x, DIMM_SLOT_CENTER_Y + 0.066, BOARD_TOP + 0.0054 - 0.0001),
                "slot_gray",
            )
    _add_box(
        board,
        (0.0085, 0.0060, 0.0060),
        DIMM_LATCH_ORIGIN,
        "slot_gray",
    )

    _add_box(
        board,
        (0.013, 0.055, 0.012),
        (BOARD_MAX_X - 0.007, 0.015, BOARD_TOP + 0.006 - 0.0001),
        "matte_black",
    )
    _add_box(
        board,
        (0.009, 0.046, 0.004),
        (BOARD_MAX_X - 0.009, 0.015, BOARD_TOP + 0.002 - 0.0001),
        "gold",
    )

    for y in (-0.070, -0.050):
        _add_box(
            board,
            (0.015, 0.020, 0.013),
            (BOARD_MAX_X - 0.010, y, BOARD_TOP + 0.0065 - 0.0001),
            "matte_black",
        )
        _add_box(
            board,
            (0.010, 0.015, 0.004),
            (BOARD_MAX_X - 0.012, y, BOARD_TOP + 0.002 - 0.0001),
            "gold",
        )

    _add_box(
        board,
        (0.070, 0.017, 0.005),
        (0.030, -0.010, BOARD_TOP + 0.0025 - 0.0001),
        "graphite",
    )
    for i in range(8):
        _add_box(
            board,
            (0.004, 0.015, 0.006),
            (0.000 + i * 0.0085, -0.010, BOARD_TOP + 0.0080 - 0.0001),
            "aluminum",
        )
    _add_box(
        board,
        (0.070, 0.0025, 0.0014),
        (0.030, -0.003, BOARD_TOP + 0.0058 - 0.0001),
        "accent",
    )

    _add_box(
        board,
        (0.040, 0.040, 0.006),
        (0.083, -0.048, BOARD_TOP + 0.003 - 0.0001),
        "graphite",
    )
    for i in range(5):
        _add_box(
            board,
            (0.034, 0.004, 0.007),
            (0.083, -0.060 + i * 0.006, BOARD_TOP + 0.0095 - 0.0001),
            "aluminum",
        )
    _add_cylinder(
        board,
        0.010,
        0.0032,
        (0.112, -0.004, BOARD_TOP + 0.0016 - 0.0001),
        "steel",
    )
    _add_cylinder(
        board,
        0.009,
        0.0006,
        (0.112, -0.004, BOARD_TOP + 0.0034 - 0.0001),
        "accent",
    )

    pcie_rows = (
        (-0.020, 0.044, "matte_black"),
        (-0.040, 0.122, "slot_gray"),
        (-0.064, 0.122, "matte_black"),
        (-0.088, 0.044, "slot_gray"),
    )
    for y, length, material in pcie_rows:
        _add_box(
            board,
            (length, 0.008, 0.011),
            (-0.010, y, BOARD_TOP + 0.0055 - 0.0001),
            material,
        )
        _add_box(
            board,
            (length - 0.010, 0.0022, 0.0010),
            (-0.006, y, BOARD_TOP + 0.0005),
            "gold",
        )
    for y in (-0.040, -0.064):
        _add_box(
            board,
            (0.116, 0.0024, 0.0012),
            (-0.004, y + 0.0014, BOARD_TOP + 0.0106 - 0.0001),
            "steel",
        )

    _add_box(
        board,
        (0.020, 0.012, 0.008),
        (0.114, 0.071, BOARD_TOP + 0.004 - 0.0001),
        "dark_gray",
    )
    _add_box(
        board,
        (0.014, 0.008, 0.0015),
        (0.114, 0.071, BOARD_TOP + 0.0085 - 0.0001),
        "accent",
    )
    _add_box(
        board,
        (0.014, 0.022, 0.012),
        (0.122, 0.098, BOARD_TOP + 0.006 - 0.0001),
        "matte_black",
    )
    _add_box(
        board,
        (0.010, 0.016, 0.004),
        (0.122, 0.098, BOARD_TOP + 0.002 - 0.0001),
        "gold",
    )

    for y in (-0.074, -0.088, -0.102):
        _add_cylinder(
            board,
            0.004,
            0.010,
            (-0.113, y, BOARD_TOP + 0.005 - 0.0001),
            "dark_gray",
        )
        _add_cylinder(
            board,
            0.0026,
            0.0010,
            (-0.113, y, BOARD_TOP + 0.0104 - 0.0001),
            "steel",
        )
    _add_box(
        board,
        (0.030, 0.010, 0.006),
        (-0.118, -0.108, BOARD_TOP + 0.003 - 0.0001),
        "matte_black",
    )
    _add_box(
        board,
        (0.020, 0.014, 0.010),
        (0.050, -0.110, BOARD_TOP + 0.005 - 0.0001),
        "matte_black",
    )
    _add_box(
        board,
        (0.024, 0.010, 0.008),
        (0.085, -0.109, BOARD_TOP + 0.004 - 0.0001),
        "matte_black",
    )
    for x in (-0.040, -0.020, 0.000, 0.020):
        _add_box(
            board,
            (0.010, 0.006, 0.004),
            (x, -0.110, BOARD_TOP + 0.002 - 0.0001),
            "gold",
        )

    cpu_arm = model.part("cpu_arm")
    cpu_arm.visual(
        mesh_from_cadquery(_cpu_arm_shape(), "cpu_retention_arm.obj", assets=ASSETS),
        material="steel",
    )
    cpu_arm.inertial = Inertial.from_geometry(
        Box((0.054, 0.010, 0.012)),
        mass=0.012,
        origin=Origin(xyz=(-0.026, 0.0, 0.006)),
    )

    dimm_latch = model.part("dimm_latch")
    dimm_latch.visual(
        mesh_from_cadquery(_dimm_latch_shape(), "dimm_slot_latch.obj", assets=ASSETS),
        material="latch_gray",
    )
    dimm_latch.inertial = Inertial.from_geometry(
        Box((0.012, 0.010, 0.013)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.002, 0.006)),
    )

    model.articulation(
        "cpu_retention_arm",
        ArticulationType.REVOLUTE,
        parent=board,
        child=cpu_arm,
        origin=Origin(xyz=CPU_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.15,
            effort=2.0,
            velocity=2.5,
        ),
    )
    model.articulation(
        "dimm_a1_latch",
        ArticulationType.REVOLUTE,
        parent=board,
        child=dimm_latch,
        origin=Origin(xyz=DIMM_LATCH_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.9,
            effort=1.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "board",
        "cpu_arm",
        reason="retention arm pivot nests inside the fixed steel hinge bracket",
    )
    ctx.allow_overlap(
        "board",
        "dimm_latch",
        reason="dimm latch hinge knuckle sits inside the slot-end support boss",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_within("cpu_arm", "board", axes="xy")
    ctx.expect_aabb_within("dimm_latch", "board", axes="xy")
    ctx.expect_aabb_overlap("cpu_arm", "board", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_overlap("dimm_latch", "board", axes="xy", min_overlap=0.002)
    ctx.expect_aabb_contact("board", "cpu_arm")
    ctx.expect_aabb_contact("board", "dimm_latch")
    ctx.expect_joint_motion_axis(
        "cpu_retention_arm",
        "cpu_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.010,
    )
    ctx.expect_joint_motion_axis(
        "dimm_a1_latch",
        "dimm_latch",
        world_axis="y",
        direction="negative",
        min_delta=0.004,
    )

    with ctx.pose(cpu_retention_arm=1.15):
        ctx.expect_aabb_within("cpu_arm", "board", axes="xy")
        ctx.expect_aabb_contact("board", "cpu_arm")
        ctx.expect_aabb_overlap("cpu_arm", "board", axes="xy", min_overlap=0.006)

    with ctx.pose(dimm_a1_latch=0.9):
        ctx.expect_aabb_within("dimm_latch", "board", axes="xy")
        ctx.expect_aabb_contact("board", "dimm_latch")
        ctx.expect_aabb_overlap("dimm_latch", "board", axes="xy", min_overlap=0.0015)

    with ctx.pose(cpu_retention_arm=1.15, dimm_a1_latch=0.9):
        ctx.expect_aabb_within("cpu_arm", "board", axes="xy")
        ctx.expect_aabb_within("dimm_latch", "board", axes="xy")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
