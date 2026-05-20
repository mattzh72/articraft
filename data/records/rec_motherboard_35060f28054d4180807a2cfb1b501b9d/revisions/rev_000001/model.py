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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="consumer_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.34, 0.18, 1.0))
    socket_black = model.material("socket_black", rgba=(0.12, 0.12, 0.13, 1.0))
    slot_black = model.material("slot_black", rgba=(0.09, 0.09, 0.10, 1.0))
    metal_silver = model.material("metal_silver", rgba=(0.72, 0.73, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    gold = model.material("gold", rgba=(0.78, 0.64, 0.22, 1.0))
    io_black = model.material("io_black", rgba=(0.16, 0.17, 0.19, 1.0))

    board_x = 0.305
    board_y = 0.244
    board_t = 0.0018

    motherboard = model.part("motherboard")
    motherboard.visual(
        Box((board_x, board_y, board_t)),
        origin=Origin(xyz=(0.0, 0.0, board_t / 2.0)),
        material=pcb_green,
        name="pcb",
    )

    socket_x = 0.012
    socket_y = 0.004
    socket_frame_x = 0.052
    socket_frame_y = 0.052
    socket_frame_h = 0.0052
    socket_frame_top = board_t + socket_frame_h

    motherboard.visual(
        Box((0.060, 0.060, 0.0012)),
        origin=Origin(xyz=(socket_x, socket_y, board_t + 0.0006)),
        material=dark_metal,
        name="socket_land_grid",
    )
    motherboard.visual(
        Box((socket_frame_x, socket_frame_y, socket_frame_h)),
        origin=Origin(xyz=(socket_x, socket_y, board_t + socket_frame_h / 2.0)),
        material=socket_black,
        name="socket_frame",
    )
    motherboard.visual(
        Box((0.036, 0.036, 0.0005)),
        origin=Origin(xyz=(socket_x, socket_y, socket_frame_top + 0.00025)),
        material=gold,
        name="cpu_contact_plate",
    )

    # Memory slots and expansion slots to help the board read clearly as a consumer motherboard.
    dimm_x = 0.088
    for idx, dimm_y in enumerate((0.020, 0.035)):
        motherboard.visual(
            Box((0.135, 0.008, 0.010)),
            origin=Origin(xyz=(dimm_x, dimm_y, board_t + 0.005)),
            material=slot_black,
            name=f"dimm_slot_{idx + 1}",
        )

    for idx, slot_y in enumerate((-0.060, -0.083)):
        motherboard.visual(
            Box((0.110, 0.010, 0.012)),
            origin=Origin(xyz=(0.008, slot_y, board_t + 0.006)),
            material=slot_black,
            name=f"pcie_slot_{idx + 1}",
        )

    motherboard.visual(
        Box((0.036, 0.036, 0.018)),
        origin=Origin(xyz=(-0.040, -0.038, board_t + 0.009)),
        material=heatsink_gray,
        name="chipset_heatsink",
    )
    motherboard.visual(
        Box((0.048, 0.028, 0.020)),
        origin=Origin(xyz=(0.006, 0.057, board_t + 0.010)),
        material=heatsink_gray,
        name="vrm_heatsink",
    )

    shield_x = -0.078
    shield_len = 0.158
    shield_t = 0.0012
    shield_h = 0.041
    shield_y = board_y / 2.0 - shield_t / 2.0 + 0.0002

    motherboard.visual(
        Box((shield_len, shield_t, shield_h)),
        origin=Origin(xyz=(shield_x, shield_y, board_t + shield_h / 2.0)),
        material=metal_silver,
        name="rear_shield",
    )

    io_blocks = (
        ("ethernet_port", -0.090, 0.017, 0.016, 0.016),
        ("usb_stack_left", -0.116, 0.014, 0.016, 0.017),
        ("usb_stack_right", -0.064, 0.014, 0.016, 0.017),
        ("video_port", -0.039, 0.020, 0.014, 0.016),
    )
    for name, x_pos, sx, sy, sz in io_blocks:
        motherboard.visual(
            Box((sx, sy, sz)),
            origin=Origin(
                xyz=(
                    x_pos,
                    shield_y - (sy / 2.0) + 0.0004,
                    board_t + sz / 2.0,
                )
            ),
            material=io_black,
            name=name,
        )

    arm = model.part("cpu_retention_arm")
    arm_wire = tube_from_spline_points(
        [
            (0.000, 0.000, 0.000),
            (0.000, -0.012, 0.000),
            (0.000, -0.050, 0.000),
            (0.004, -0.074, 0.000),
            (0.012, -0.084, 0.000),
        ],
        radius=0.00135,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    arm.visual(
        mesh_from_geometry(arm_wire, "cpu_retention_arm_wire"),
        material=metal_silver,
        name="arm_main",
    )
    arm.visual(
        Cylinder(radius=0.0025, length=0.005),
        origin=Origin(),
        material=metal_silver,
        name="arm_pivot",
    )
    arm.visual(
        Box((0.014, 0.0032, 0.006)),
        origin=Origin(xyz=(0.012, -0.084, 0.0)),
        material=metal_silver,
        name="arm_tab",
    )

    arm_pivot_x = socket_x + socket_frame_x / 2.0 + 0.006
    arm_pivot_y = socket_y + 0.021
    arm_pivot_z = socket_frame_top + 0.0016
    model.articulation(
        "socket_to_retention_arm",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=arm,
        origin=Origin(xyz=(arm_pivot_x, arm_pivot_y, arm_pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=0.0,
            upper=1.9,
        ),
    )

    flap = model.part("io_shield_flap")
    flap.visual(
        Box((0.146, 0.0008, 0.028)),
        origin=Origin(xyz=(0.0, 0.0004, -0.014)),
        material=metal_silver,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.0015, length=0.146),
        origin=Origin(xyz=(0.0, 0.0010, -0.0006), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_silver,
        name="flap_barrel",
    )
    flap.visual(
        Box((0.018, 0.0036, 0.006)),
        origin=Origin(xyz=(0.0, 0.0018, -0.027)),
        material=metal_silver,
        name="flap_pull_tab",
    )

    model.articulation(
        "rear_shield_to_flap",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=flap,
        origin=Origin(
            xyz=(
                shield_x,
                shield_y + shield_t / 2.0,
                board_t + shield_h,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )

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

    motherboard = object_model.get_part("motherboard")
    arm = object_model.get_part("cpu_retention_arm")
    flap = object_model.get_part("io_shield_flap")

    arm_joint = object_model.get_articulation("socket_to_retention_arm")
    flap_joint = object_model.get_articulation("rear_shield_to_flap")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_pt, max_pt) = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(min_pt, max_pt))

    ctx.check(
        "retention arm uses a vertical side pivot",
        tuple(round(v, 6) for v in arm_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={arm_joint.axis}",
    )
    ctx.check(
        "rear flap uses a rear-edge hinge axis",
        tuple(round(v, 6) for v in flap_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={flap_joint.axis}",
    )

    with ctx.pose({arm_joint: 0.0, flap_joint: 0.0}):
        ctx.expect_gap(
            arm,
            motherboard,
            axis="z",
            positive_elem="arm_main",
            negative_elem="socket_frame",
            min_gap=0.0001,
            max_gap=0.0030,
            name="retention arm clears the socket top in the locked pose",
        )
        ctx.expect_overlap(
            arm,
            motherboard,
            axes="y",
            elem_a="arm_main",
            elem_b="socket_frame",
            min_overlap=0.018,
            name="retention arm runs alongside the socket edge",
        )
        ctx.expect_gap(
            flap,
            motherboard,
            axis="y",
            positive_elem="flap_panel",
            negative_elem="rear_shield",
            min_gap=0.0,
            max_gap=0.0002,
            name="rear I/O flap sits flush against the shield face",
        )
        ctx.expect_overlap(
            flap,
            motherboard,
            axes="xz",
            elem_a="flap_panel",
            elem_b="rear_shield",
            min_overlap=0.024,
            name="rear I/O flap aligns with the shield opening band",
        )

    closed_arm_center = None
    open_arm_center = None
    with ctx.pose({arm_joint: 0.0}):
        closed_arm_center = _aabb_center(ctx.part_element_world_aabb(arm, elem="arm_tab"))
    with ctx.pose({arm_joint: arm_joint.motion_limits.upper}):
        open_arm_center = _aabb_center(ctx.part_element_world_aabb(arm, elem="arm_tab"))
    ctx.check(
        "retention arm swings outboard when opened",
        closed_arm_center is not None
        and open_arm_center is not None
        and open_arm_center[0] > closed_arm_center[0] + 0.018,
        details=f"closed_center={closed_arm_center}, open_center={open_arm_center}",
    )

    closed_flap_center = None
    open_flap_center = None
    with ctx.pose({flap_joint: 0.0}):
        closed_flap_center = _aabb_center(
            ctx.part_element_world_aabb(flap, elem="flap_panel")
        )
    with ctx.pose({flap_joint: flap_joint.motion_limits.upper}):
        open_flap_center = _aabb_center(
            ctx.part_element_world_aabb(flap, elem="flap_panel")
        )
    ctx.check(
        "rear I/O flap swings inward over the board when opened",
        closed_flap_center is not None
        and open_flap_center is not None
        and open_flap_center[1] < closed_flap_center[1] - 0.010
        and open_flap_center[2] > closed_flap_center[2] + 0.006,
        details=f"closed_center={closed_flap_center}, open_center={open_flap_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
