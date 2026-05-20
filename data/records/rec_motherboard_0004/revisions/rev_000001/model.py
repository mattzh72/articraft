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
    rounded_rect_profile,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)
BOARD_SIZE = 0.170
BOARD_HALF = BOARD_SIZE * 0.5
BOARD_THICKNESS = 0.0016
BOARD_TOP = BOARD_THICKNESS


def _mesh(name: str, geometry):
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
            cx + radius * math.cos(2.0 * math.pi * idx / segments),
            cy + radius * math.sin(2.0 * math.pi * idx / segments),
        )
        for idx in range(segments)
    ]


def _mounted_center(height: float, *, base_z: float = BOARD_TOP) -> float:
    return base_z + 0.5 * height


def _top_box(
    part,
    name: str,
    size: tuple[float, float, float],
    center_xy: tuple[float, float],
    material,
    *,
    base_z: float = BOARD_TOP,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=(center_xy[0], center_xy[1], _mounted_center(size[2], base_z=base_z))),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_motherboard", assets=ASSETS)

    pcb_green = model.material("pcb_green", rgba=(0.08, 0.33, 0.16, 1.0))
    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    slot_black = model.material("slot_black", rgba=(0.15, 0.15, 0.16, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.25, 0.26, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    aluminum = model.material("aluminum", rgba=(0.63, 0.65, 0.68, 1.0))
    socket_tan = model.material("socket_tan", rgba=(0.82, 0.77, 0.66, 1.0))
    port_metal = model.material("port_metal", rgba=(0.77, 0.79, 0.82, 1.0))
    gold = model.material("gold", rgba=(0.82, 0.69, 0.22, 1.0))

    motherboard = model.part("motherboard")
    motherboard.inertial = Inertial.from_geometry(
        Box((BOARD_SIZE, BOARD_SIZE, 0.020)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    board_profile = rounded_rect_profile(BOARD_SIZE, BOARD_SIZE, 0.0055, corner_segments=8)
    hole_profiles = [
        _circle_profile(-0.072, -0.069, 0.0018),
        _circle_profile(-0.072, 0.069, 0.0018),
        _circle_profile(0.071, -0.069, 0.0018),
        _circle_profile(0.071, 0.069, 0.0018),
    ]
    pcb_geom = ExtrudeWithHolesGeometry(
        board_profile,
        hole_profiles,
        height=BOARD_THICKNESS,
        center=False,
    )
    motherboard.visual(_mesh("motherboard_pcb.obj", pcb_geom), material=pcb_green, name="pcb")

    socket_base_height = 0.0030
    socket_frame_height = 0.0024
    socket_base_xy = (-0.004, 0.004)
    _top_box(
        motherboard,
        "cpu_socket_base",
        (0.041, 0.041, socket_base_height),
        socket_base_xy,
        socket_tan,
    )
    _top_box(
        motherboard,
        "cpu_socket_north_wall",
        (0.039, 0.0032, socket_frame_height),
        (socket_base_xy[0], socket_base_xy[1] + 0.0185),
        steel,
        base_z=BOARD_TOP + socket_base_height,
    )
    _top_box(
        motherboard,
        "cpu_socket_south_wall",
        (0.039, 0.0032, socket_frame_height),
        (socket_base_xy[0], socket_base_xy[1] - 0.0185),
        steel,
        base_z=BOARD_TOP + socket_base_height,
    )
    _top_box(
        motherboard,
        "cpu_socket_west_wall",
        (0.0032, 0.0325, socket_frame_height),
        (socket_base_xy[0] - 0.0187, socket_base_xy[1]),
        steel,
        base_z=BOARD_TOP + socket_base_height,
    )
    _top_box(
        motherboard,
        "cpu_socket_east_wall",
        (0.0032, 0.0325, socket_frame_height),
        (socket_base_xy[0] + 0.0187, socket_base_xy[1]),
        steel,
        base_z=BOARD_TOP + socket_base_height,
    )
    _top_box(
        motherboard,
        "socket_contact_field",
        (0.026, 0.026, 0.0008),
        socket_base_xy,
        gold,
        base_z=BOARD_TOP + 0.0002,
    )
    _top_box(
        motherboard,
        "socket_hinge_pad",
        (0.0034, 0.0070, 0.0016),
        (0.0180, -0.0140),
        steel,
        base_z=BOARD_TOP + socket_base_height + 0.0010,
    )

    vrm_base_z = BOARD_TOP
    _top_box(motherboard, "vrm_sink_base", (0.024, 0.020, 0.0075), (-0.039, 0.038), aluminum, base_z=vrm_base_z)
    for idx, x_shift in enumerate((-0.006, 0.0, 0.006)):
        _top_box(
            motherboard,
            f"vrm_sink_fin_{idx}",
            (0.0030, 0.020, 0.0030),
            (-0.039 + x_shift, 0.038),
            dark_gray,
            base_z=BOARD_TOP + 0.0075,
        )

    _top_box(motherboard, "chipset_sink_base", (0.020, 0.020, 0.0065), (-0.021, -0.040), aluminum)
    for idx, y_shift in enumerate((-0.006, 0.0, 0.006)):
        _top_box(
            motherboard,
            f"chipset_sink_fin_{idx}",
            (0.020, 0.0030, 0.0028),
            (-0.021, -0.040 + y_shift),
            dark_gray,
            base_z=BOARD_TOP + 0.0065,
        )

    _top_box(motherboard, "m2_heatsink", (0.054, 0.011, 0.0032), (-0.003, -0.014), dark_gray)

    _top_box(motherboard, "pcie_slot_body", (0.092, 0.010, 0.012), (0.002, -0.056), slot_black)
    _top_box(motherboard, "pcie_slot_retainer", (0.008, 0.013, 0.014), (0.050, -0.056), matte_black)
    _top_box(
        motherboard,
        "pcie_gold_fingers",
        (0.060, 0.0040, 0.0007),
        (0.002, -0.0465),
        gold,
        base_z=BOARD_TOP,
    )

    dimm_base_z = BOARD_TOP
    _top_box(motherboard, "dimm_slot_inner", (0.0070, 0.067, 0.010), (0.043, 0.000), slot_black, base_z=dimm_base_z)
    _top_box(motherboard, "dimm_slot_outer", (0.0070, 0.067, 0.010), (0.053, 0.000), slot_black, base_z=dimm_base_z)
    _top_box(motherboard, "dimm_upper_pivot_pad", (0.0080, 0.0040, 0.0016), (0.053, 0.0355), matte_black, base_z=BOARD_TOP + 0.010)
    _top_box(motherboard, "dimm_lower_pivot_pad", (0.0080, 0.0040, 0.0016), (0.053, -0.0355), matte_black, base_z=BOARD_TOP + 0.010)

    _top_box(motherboard, "rear_io_shroud", (0.062, 0.012, 0.014), (-0.026, 0.078), matte_black)
    _top_box(motherboard, "rear_io_ports", (0.052, 0.008, 0.010), (-0.026, 0.078), port_metal)
    _top_box(motherboard, "eps_power", (0.014, 0.014, 0.011), (-0.067, 0.066), matte_black)
    _top_box(motherboard, "atx_24pin", (0.008, 0.026, 0.012), (0.079, 0.021), matte_black)
    _top_box(motherboard, "sata_stack", (0.015, 0.022, 0.011), (0.067, -0.069), matte_black)
    _top_box(motherboard, "front_usb_header", (0.014, 0.008, 0.009), (0.053, -0.024), matte_black)
    _top_box(motherboard, "audio_codec", (0.010, 0.010, 0.0025), (-0.058, -0.062), dark_gray)

    for idx, cap_xy in enumerate(
        [
            (-0.054, 0.055),
            (-0.046, 0.055),
            (-0.038, 0.055),
            (0.020, 0.051),
            (0.030, 0.051),
            (0.040, 0.051),
            (0.061, -0.018),
            (0.061, -0.008),
        ]
    ):
        motherboard.visual(
            Cylinder(radius=0.0024, length=0.0060),
            origin=Origin(xyz=(cap_xy[0], cap_xy[1], BOARD_TOP + 0.0030)),
            material=dark_gray,
            name=f"capacitor_{idx}",
        )

    for idx, chip_xy in enumerate(
        [
            (-0.050, 0.010),
            (-0.041, 0.010),
            (-0.032, 0.010),
            (-0.061, -0.007),
            (-0.052, -0.007),
            (0.028, -0.024),
            (0.028, -0.032),
            (0.028, -0.040),
        ]
    ):
        _top_box(motherboard, f"controller_{idx}", (0.006, 0.006, 0.0016), chip_xy, matte_black)

    cpu_lever = model.part("cpu_lever")
    cpu_lever.inertial = Inertial.from_geometry(
        Box((0.027, 0.008, 0.010)),
        mass=0.010,
        origin=Origin(xyz=(0.013, 0.0, 0.004)),
    )
    cpu_lever.visual(
        Box((0.0034, 0.0060, 0.0016)),
        origin=Origin(),
        material=steel,
        name="pivot_block",
    )
    cpu_lever.visual(
        Box((0.0050, 0.0014, 0.0014)),
        origin=Origin(xyz=(0.0030, 0.0, 0.0015)),
        material=steel,
        name="lever_root",
    )
    lever_wire = wire_from_points(
        [
            (0.0015, 0.0, 0.0022),
            (0.0180, 0.0, 0.0022),
            (0.0240, 0.0, 0.0050),
        ],
        radius=0.00065,
        radial_segments=12,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.0018,
        corner_segments=6,
    )
    cpu_lever.visual(_mesh("cpu_socket_lever.obj", lever_wire), material=steel, name="lever_wire")
    cpu_lever.visual(
        Box((0.0040, 0.0022, 0.0018)),
        origin=Origin(xyz=(0.0248, 0.0, 0.0051)),
        material=steel,
        name="finger_tab",
    )

    dimm_latch_upper = model.part("dimm_latch_upper")
    dimm_latch_upper.inertial = Inertial.from_geometry(
        Box((0.006, 0.006, 0.012)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )
    dimm_latch_upper.visual(
        Box((0.006, 0.0030, 0.0016)),
        origin=Origin(),
        material=slot_black,
        name="pivot_block",
    )
    dimm_latch_upper.visual(
        Box((0.006, 0.0026, 0.0092)),
        origin=Origin(xyz=(0.0, -0.0013, 0.0054)),
        material=slot_black,
        name="latch_body",
    )
    dimm_latch_upper.visual(
        Box((0.006, 0.0048, 0.0022)),
        origin=Origin(xyz=(0.0, -0.0032, 0.0097)),
        material=slot_black,
        name="retention_hook",
    )
    dimm_latch_upper.visual(
        Box((0.006, 0.0020, 0.0030)),
        origin=Origin(xyz=(0.0, 0.0010, 0.0068)),
        material=slot_black,
        name="finger_grip",
    )

    dimm_latch_lower = model.part("dimm_latch_lower")
    dimm_latch_lower.inertial = Inertial.from_geometry(
        Box((0.006, 0.006, 0.012)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )
    dimm_latch_lower.visual(
        Box((0.006, 0.0030, 0.0016)),
        origin=Origin(),
        material=slot_black,
        name="pivot_block",
    )
    dimm_latch_lower.visual(
        Box((0.006, 0.0026, 0.0092)),
        origin=Origin(xyz=(0.0, 0.0013, 0.0054)),
        material=slot_black,
        name="latch_body",
    )
    dimm_latch_lower.visual(
        Box((0.006, 0.0048, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0032, 0.0097)),
        material=slot_black,
        name="retention_hook",
    )
    dimm_latch_lower.visual(
        Box((0.006, 0.0020, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0010, 0.0068)),
        material=slot_black,
        name="finger_grip",
    )

    model.articulation(
        "socket_lever_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=cpu_lever,
        origin=Origin(xyz=(0.0180, -0.0140, 0.0072)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "upper_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=dimm_latch_upper,
        origin=Origin(xyz=(0.053, 0.0355, 0.0132)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=3.0,
            lower=0.0,
            upper=0.55,
        ),
    )
    model.articulation(
        "lower_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=dimm_latch_lower,
        origin=Origin(xyz=(0.053, -0.0355, 0.0132)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=3.0,
            lower=0.0,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    motherboard = object_model.get_part("motherboard")
    cpu_lever = object_model.get_part("cpu_lever")
    dimm_latch_upper = object_model.get_part("dimm_latch_upper")
    dimm_latch_lower = object_model.get_part("dimm_latch_lower")

    socket_lever_hinge = object_model.get_articulation("socket_lever_hinge")
    upper_latch_hinge = object_model.get_articulation("upper_latch_hinge")
    lower_latch_hinge = object_model.get_articulation("lower_latch_hinge")

    pcb = motherboard.get_visual("pcb")
    cpu_socket_base = motherboard.get_visual("cpu_socket_base")
    socket_hinge_pad = motherboard.get_visual("socket_hinge_pad")
    pcie_slot_body = motherboard.get_visual("pcie_slot_body")
    dimm_slot_inner = motherboard.get_visual("dimm_slot_inner")
    dimm_slot_outer = motherboard.get_visual("dimm_slot_outer")
    dimm_upper_pivot_pad = motherboard.get_visual("dimm_upper_pivot_pad")
    dimm_lower_pivot_pad = motherboard.get_visual("dimm_lower_pivot_pad")

    lever_pivot_block = cpu_lever.get_visual("pivot_block")
    lever_wire = cpu_lever.get_visual("lever_wire")
    upper_pivot_block = dimm_latch_upper.get_visual("pivot_block")
    upper_latch_body = dimm_latch_upper.get_visual("latch_body")
    lower_pivot_block = dimm_latch_lower.get_visual("pivot_block")
    lower_latch_body = dimm_latch_lower.get_visual("latch_body")

    def _dims(aabb):
        if aabb is None:
            return None
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        )

    def _center(aabb):
        if aabb is None:
            return None
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

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

    ctx.expect_contact(
        motherboard,
        cpu_lever,
        elem_a=socket_hinge_pad,
        elem_b=lever_pivot_block,
        name="cpu lever pivot block contacts socket hinge pad",
    )
    ctx.expect_contact(
        motherboard,
        dimm_latch_upper,
        elem_a=dimm_upper_pivot_pad,
        elem_b=upper_pivot_block,
        name="upper dimm latch pivot rests on slot pad",
    )
    ctx.expect_contact(
        motherboard,
        dimm_latch_lower,
        elem_a=dimm_lower_pivot_pad,
        elem_b=lower_pivot_block,
        name="lower dimm latch pivot rests on slot pad",
    )
    ctx.expect_gap(
        cpu_lever,
        motherboard,
        axis="z",
        min_gap=0.0012,
        max_gap=0.0080,
        positive_elem=lever_wire,
        negative_elem=cpu_socket_base,
        name="cpu lever clears socket base",
    )
    ctx.expect_gap(
        dimm_latch_upper,
        motherboard,
        axis="z",
        min_gap=0.0020,
        max_gap=0.0200,
        positive_elem=upper_latch_body,
        negative_elem=pcb,
        name="upper latch stands above board",
    )
    ctx.expect_gap(
        dimm_latch_lower,
        motherboard,
        axis="z",
        min_gap=0.0020,
        max_gap=0.0200,
        positive_elem=lower_latch_body,
        negative_elem=pcb,
        name="lower latch stands above board",
    )
    ctx.expect_within(cpu_lever, motherboard, axes="xy", margin=0.004, name="cpu lever stays within board footprint")
    ctx.expect_within(
        dimm_latch_upper,
        motherboard,
        axes="xy",
        margin=0.002,
        name="upper latch stays within board footprint",
    )
    ctx.expect_within(
        dimm_latch_lower,
        motherboard,
        axes="xy",
        margin=0.002,
        name="lower latch stays within board footprint",
    )
    ctx.expect_origin_gap(
        dimm_latch_upper,
        dimm_latch_lower,
        axis="y",
        min_gap=0.065,
        max_gap=0.078,
        name="dimm latch pair spans slot length",
    )

    pcb_aabb = ctx.part_element_world_aabb(motherboard, elem=pcb)
    socket_aabb = ctx.part_element_world_aabb(motherboard, elem=cpu_socket_base)
    pcie_aabb = ctx.part_element_world_aabb(motherboard, elem=pcie_slot_body)
    dimm_inner_aabb = ctx.part_element_world_aabb(motherboard, elem=dimm_slot_inner)
    dimm_outer_aabb = ctx.part_element_world_aabb(motherboard, elem=dimm_slot_outer)

    pcb_dims = _dims(pcb_aabb)
    socket_center = _center(socket_aabb)
    pcb_center = _center(pcb_aabb)
    pcie_dims = _dims(pcie_aabb)
    pcie_center = _center(pcie_aabb)
    dimm_inner_center = _center(dimm_inner_aabb)
    dimm_outer_center = _center(dimm_outer_aabb)
    dimm_outer_dims = _dims(dimm_outer_aabb)

    ctx.check(
        "mini itx board footprint",
        pcb_dims is not None
        and abs(pcb_dims[0] - BOARD_SIZE) < 0.004
        and abs(pcb_dims[1] - BOARD_SIZE) < 0.004
        and pcb_dims[2] < 0.003,
        details=f"pcb_dims={pcb_dims}",
    )
    ctx.check(
        "cpu socket sits near board center",
        socket_center is not None
        and pcb_center is not None
        and abs(socket_center[0] - pcb_center[0]) < 0.020
        and abs(socket_center[1] - pcb_center[1]) < 0.020,
        details=f"socket_center={socket_center}, pcb_center={pcb_center}",
    )
    ctx.check(
        "pcie slot occupies lower board zone",
        pcie_dims is not None
        and pcie_center is not None
        and pcie_dims[0] > 0.085
        and pcie_dims[1] < 0.014
        and pcie_center[1] < -0.040,
        details=f"pcie_dims={pcie_dims}, pcie_center={pcie_center}",
    )
    ctx.check(
        "paired dimm slots stand along right edge",
        dimm_inner_center is not None
        and dimm_outer_center is not None
        and dimm_outer_dims is not None
        and dimm_outer_center[0] > 0.045
        and abs(dimm_outer_center[1] - dimm_inner_center[1]) < 0.002
        and abs(dimm_outer_center[0] - dimm_inner_center[0]) > 0.008
        and dimm_outer_dims[1] > 0.060,
        details=(
            f"dimm_inner_center={dimm_inner_center}, "
            f"dimm_outer_center={dimm_outer_center}, "
            f"dimm_outer_dims={dimm_outer_dims}"
        ),
    )

    lever_rest = ctx.part_element_world_aabb(cpu_lever, elem=lever_wire)
    upper_rest = ctx.part_element_world_aabb(dimm_latch_upper, elem=upper_latch_body)
    lower_rest = ctx.part_element_world_aabb(dimm_latch_lower, elem=lower_latch_body)
    lever_rest_center = _center(lever_rest)
    upper_rest_center = _center(upper_rest)
    lower_rest_center = _center(lower_rest)

    with ctx.pose({socket_lever_hinge: 0.90}):
        lever_open = ctx.part_element_world_aabb(cpu_lever, elem=lever_wire)
        lever_open_center = _center(lever_open)
        ctx.check(
            "cpu lever opens upward from socket edge",
            lever_rest_center is not None
            and lever_open_center is not None
            and lever_open_center[2] > lever_rest_center[2] + 0.0085
            and lever_open_center[0] < lever_rest_center[0] - 0.004,
            details=f"lever_rest_center={lever_rest_center}, lever_open_center={lever_open_center}",
        )

    with ctx.pose({upper_latch_hinge: 0.45, lower_latch_hinge: 0.45}):
        upper_open = ctx.part_element_world_aabb(dimm_latch_upper, elem=upper_latch_body)
        lower_open = ctx.part_element_world_aabb(dimm_latch_lower, elem=lower_latch_body)
        upper_open_center = _center(upper_open)
        lower_open_center = _center(lower_open)
        ctx.check(
            "upper dimm latch swings outward",
            upper_rest_center is not None
            and upper_open_center is not None
            and upper_open_center[1] > upper_rest_center[1] + 0.0015,
            details=f"upper_rest_center={upper_rest_center}, upper_open_center={upper_open_center}",
        )
        ctx.check(
            "lower dimm latch swings outward",
            lower_rest_center is not None
            and lower_open_center is not None
            and lower_open_center[1] < lower_rest_center[1] - 0.0015,
            details=f"lower_rest_center={lower_rest_center}, lower_open_center={lower_open_center}",
        )
        ctx.check(
            "opened latches remain above pcb",
            upper_open is not None
            and lower_open is not None
            and upper_open[0][2] > BOARD_TOP + 0.002
            and lower_open[0][2] > BOARD_TOP + 0.002,
            details=f"upper_open={upper_open}, lower_open={lower_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
