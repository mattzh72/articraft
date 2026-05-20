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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.08, 0.34, 0.16, 1.0))
    solder_mask = model.material("solder_mask", rgba=(0.10, 0.40, 0.19, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    socket_black = model.material("socket_black", rgba=(0.15, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.36, 0.39, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.67, 0.69, 0.72, 1.0))
    slot_black = model.material("slot_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gold = model.material("gold", rgba=(0.76, 0.63, 0.24, 1.0))

    board_size = 0.170
    board_thickness = 0.0016
    half_board = board_size * 0.5

    motherboard = model.part("motherboard")
    motherboard.inertial = Inertial.from_geometry(
        Box((board_size, board_size, 0.020)),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    motherboard.visual(
        Box((board_size, board_size, board_thickness)),
        origin=Origin(xyz=(0.0, 0.0, board_thickness * 0.5)),
        material=pcb_green,
        name="pcb",
    )
    motherboard.visual(
        Box((0.146, 0.138, 0.0002)),
        origin=Origin(xyz=(-0.002, -0.004, board_thickness + 0.0001)),
        material=solder_mask,
        name="board_silkscreen",
    )

    socket_center = (-0.012, -0.008)
    socket_top_z = 0.0052
    motherboard.visual(
        Box((0.040, 0.037, 0.0036)),
        origin=Origin(xyz=(socket_center[0], socket_center[1], board_thickness + 0.0018)),
        material=socket_black,
        name="cpu_socket_housing",
    )
    motherboard.visual(
        Box((0.036, 0.033, 0.0008)),
        origin=Origin(xyz=(socket_center[0], socket_center[1], board_thickness + 0.0004)),
        material=gold,
        name="cpu_contact_pad",
    )
    motherboard.visual(
        Box((0.036, 0.004, 0.0025)),
        origin=Origin(xyz=(socket_center[0] + 0.002, socket_center[1] + 0.020, board_thickness + 0.00125)),
        material=dark_steel,
        name="upper_socket_retainer",
    )
    motherboard.visual(
        Box((0.004, 0.032, 0.002)),
        origin=Origin(xyz=(socket_center[0] - 0.022, socket_center[1], board_thickness + 0.001)),
        material=dark_steel,
        name="load_plate_hinge_base",
    )
    motherboard.visual(
        Box((0.004, 0.010, 0.004)),
        origin=Origin(xyz=(socket_center[0] - 0.022, socket_center[1] - 0.014, board_thickness + 0.002)),
        material=dark_steel,
        name="load_plate_hinge_pedestal_lower",
    )
    motherboard.visual(
        Box((0.004, 0.010, 0.004)),
        origin=Origin(xyz=(socket_center[0] - 0.022, socket_center[1] + 0.010, board_thickness + 0.002)),
        material=dark_steel,
        name="load_plate_hinge_pedestal_upper",
    )
    motherboard.visual(
        Box((0.005, 0.013, 0.0048)),
        origin=Origin(xyz=(socket_center[0] + 0.024, socket_center[1], board_thickness + 0.0024)),
        material=dark_steel,
        name="socket_latch_block",
    )

    motherboard.visual(
        Box((0.024, 0.056, 0.010)),
        origin=Origin(xyz=(-0.045, 0.030, board_thickness + 0.005)),
        material=brushed_aluminum,
        name="vrm_heatsink_left",
    )
    motherboard.visual(
        Box((0.052, 0.012, 0.010)),
        origin=Origin(xyz=(-0.012, 0.050, board_thickness + 0.005)),
        material=brushed_aluminum,
        name="vrm_heatsink_top",
    )
    motherboard.visual(
        Box((0.006, 0.128, 0.008)),
        origin=Origin(xyz=(0.050, -0.004, board_thickness + 0.004)),
        material=slot_black,
        name="dimm_slot_inner",
    )
    motherboard.visual(
        Box((0.006, 0.128, 0.008)),
        origin=Origin(xyz=(0.060, -0.004, board_thickness + 0.004)),
        material=slot_black,
        name="dimm_slot_outer",
    )
    motherboard.visual(
        Box((0.010, 0.040, 0.012)),
        origin=Origin(xyz=(0.078, -0.032, board_thickness + 0.006)),
        material=matte_black,
        name="atx_power_header",
    )
    motherboard.visual(
        Box((0.088, 0.008, 0.011)),
        origin=Origin(xyz=(0.006, -0.060, board_thickness + 0.0055)),
        material=slot_black,
        name="pcie_slot",
    )
    motherboard.visual(
        Box((0.028, 0.028, 0.010)),
        origin=Origin(xyz=(0.020, 0.024, board_thickness + 0.005)),
        material=brushed_aluminum,
        name="chipset_heatsink",
    )
    motherboard.visual(
        Box((0.022, 0.080, 0.002)),
        origin=Origin(xyz=(0.020, -0.020, board_thickness + 0.001)),
        material=dark_steel,
        name="m2_shield",
    )

    motherboard.visual(
        Box((0.148, 0.010, 0.016)),
        origin=Origin(xyz=(-0.004, half_board + 0.004, 0.008)),
        material=steel,
        name="rear_io_housing",
    )
    motherboard.visual(
        Box((0.014, 0.006, 0.013)),
        origin=Origin(xyz=(-0.058, half_board + 0.003, 0.0073)),
        material=matte_black,
        name="ethernet_stack",
    )
    motherboard.visual(
        Box((0.024, 0.006, 0.013)),
        origin=Origin(xyz=(-0.036, half_board + 0.003, 0.0073)),
        material=matte_black,
        name="usb_stack_left",
    )
    motherboard.visual(
        Box((0.024, 0.006, 0.013)),
        origin=Origin(xyz=(-0.010, half_board + 0.003, 0.0073)),
        material=matte_black,
        name="usb_stack_right",
    )
    motherboard.visual(
        Box((0.018, 0.006, 0.013)),
        origin=Origin(xyz=(0.016, half_board + 0.003, 0.0073)),
        material=matte_black,
        name="display_port_cluster",
    )
    motherboard.visual(
        Box((0.034, 0.008, 0.012)),
        origin=Origin(xyz=(0.054, half_board + 0.004, 0.010)),
        material=matte_black,
        name="wireless_bay",
    )
    motherboard.visual(
        Cylinder(radius=0.0036, length=0.0066),
        origin=Origin(xyz=(0.046, half_board + 0.0047, 0.0105), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="antenna_connector_left",
    )
    motherboard.visual(
        Cylinder(radius=0.0036, length=0.0066),
        origin=Origin(xyz=(0.061, half_board + 0.0047, 0.0105), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="antenna_connector_right",
    )
    motherboard.visual(
        Box((0.042, 0.004, 0.004)),
        origin=Origin(xyz=(0.054, half_board + 0.0015, 0.016)),
        material=dark_steel,
        name="rear_cover_bracket",
    )
    motherboard.visual(
        Box((0.004, 0.004, 0.006)),
        origin=Origin(xyz=(0.034, half_board + 0.0015, 0.017)),
        material=dark_steel,
        name="rear_cover_bracket_left",
    )
    motherboard.visual(
        Box((0.004, 0.004, 0.006)),
        origin=Origin(xyz=(0.074, half_board + 0.0015, 0.017)),
        material=dark_steel,
        name="rear_cover_bracket_right",
    )

    load_plate = model.part("cpu_load_plate")
    load_plate.inertial = Inertial.from_geometry(
        Box((0.042, 0.042, 0.004)),
        mass=0.025,
        origin=Origin(xyz=(0.021, 0.0, -0.001)),
    )
    load_frame_mesh = _save_mesh(
        "cpu_load_plate_frame",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.041, 0.042, 0.0025, corner_segments=6),
            [rounded_rect_profile(0.031, 0.029, 0.0018, corner_segments=6)],
            height=0.0010,
            center=True,
        ),
    )
    load_plate.visual(
        load_frame_mesh,
        origin=Origin(xyz=(0.0205, 0.0, -0.0012)),
        material=steel,
        name="load_plate_frame",
    )
    load_plate.visual(
        Cylinder(radius=0.0014, length=0.034),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="load_plate_hinge_barrel",
    )
    load_plate.visual(
        Box((0.004, 0.008, 0.0012)),
        origin=Origin(xyz=(0.0020, -0.012, -0.0012)),
        material=steel,
        name="load_plate_ear_lower",
    )
    load_plate.visual(
        Box((0.004, 0.008, 0.0012)),
        origin=Origin(xyz=(0.0020, 0.012, -0.0012)),
        material=steel,
        name="load_plate_ear_upper",
    )
    load_plate.visual(
        Box((0.006, 0.010, 0.0012)),
        origin=Origin(xyz=(0.039, 0.0, -0.0012)),
        material=steel,
        name="load_plate_latch_tab",
    )

    rear_cover = model.part("rear_cover_panel")
    rear_cover.inertial = Inertial.from_geometry(
        Box((0.040, 0.004, 0.022)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.001, -0.010)),
    )
    rear_cover.visual(
        Box((0.040, 0.0018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0009, -0.010)),
        material=matte_black,
        name="rear_cover_panel",
    )
    rear_cover.visual(
        Box((0.038, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, -0.0002, -0.0015)),
        material=matte_black,
        name="rear_cover_leaf",
    )
    rear_cover.visual(
        Cylinder(radius=0.0015, length=0.038),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_cover_hinge_sleeve",
    )

    load_plate_hinge_x = socket_center[0] - 0.0205
    load_plate_hinge_y = socket_center[1]
    load_plate_hinge_z = 0.0070
    model.articulation(
        "cpu_load_plate_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=load_plate,
        origin=Origin(xyz=(load_plate_hinge_x, load_plate_hinge_y, load_plate_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.45),
    )

    rear_cover_hinge_x = 0.054
    rear_cover_hinge_y = half_board + 0.0084
    rear_cover_hinge_z = 0.0180
    model.articulation(
        "rear_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=rear_cover,
        origin=Origin(xyz=(rear_cover_hinge_x, rear_cover_hinge_y, rear_cover_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.55),
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
    load_plate = object_model.get_part("cpu_load_plate")
    rear_cover = object_model.get_part("rear_cover_panel")
    load_hinge = object_model.get_articulation("cpu_load_plate_hinge")
    cover_hinge = object_model.get_articulation("rear_cover_hinge")

    socket_body = motherboard.get_visual("cpu_socket_housing")
    wireless_bay = motherboard.get_visual("wireless_bay")
    load_frame = load_plate.get_visual("load_plate_frame")
    cover_panel = rear_cover.get_visual("rear_cover_panel")

    ctx.check(
        "cpu load plate hinge uses upward-opening axis",
        load_hinge.axis == (0.0, -1.0, 0.0)
        and load_hinge.motion_limits is not None
        and load_hinge.motion_limits.lower == 0.0
        and load_hinge.motion_limits.upper is not None
        and load_hinge.motion_limits.upper >= 1.3,
        details=f"axis={load_hinge.axis}, limits={load_hinge.motion_limits}",
    )
    ctx.check(
        "rear cover hinge uses rear-edge axis",
        cover_hinge.axis == (1.0, 0.0, 0.0)
        and cover_hinge.motion_limits is not None
        and cover_hinge.motion_limits.lower == 0.0
        and cover_hinge.motion_limits.upper is not None
        and cover_hinge.motion_limits.upper >= 1.4,
        details=f"axis={cover_hinge.axis}, limits={cover_hinge.motion_limits}",
    )

    with ctx.pose({load_hinge: 0.0}):
        ctx.expect_gap(
            load_plate,
            motherboard,
            axis="z",
            positive_elem=load_frame,
            negative_elem=socket_body,
            max_gap=0.0012,
            max_penetration=0.0,
            name="closed load plate sits just above the socket",
        )
        ctx.expect_overlap(
            load_plate,
            motherboard,
            axes="xy",
            elem_a=load_frame,
            elem_b=socket_body,
            min_overlap=0.030,
            name="closed load plate spans the socket footprint",
        )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_gap(
            rear_cover,
            motherboard,
            axis="y",
            positive_elem=cover_panel,
            negative_elem=wireless_bay,
            min_gap=0.0002,
            max_gap=0.0030,
            name="rear cover sits just outside the wireless bay",
        )
        ctx.expect_overlap(
            rear_cover,
            motherboard,
            axes="xz",
            elem_a=cover_panel,
            elem_b=wireless_bay,
            min_overlap=0.008,
            name="rear cover shields the wireless connector area",
        )

    closed_load_aabb = ctx.part_element_world_aabb(load_plate, elem="load_plate_frame")
    with ctx.pose({load_hinge: 1.20}):
        open_load_aabb = ctx.part_element_world_aabb(load_plate, elem="load_plate_frame")
    ctx.check(
        "cpu load plate opens upward",
        closed_load_aabb is not None
        and open_load_aabb is not None
        and open_load_aabb[1][2] > closed_load_aabb[1][2] + 0.015,
        details=f"closed={closed_load_aabb}, open={open_load_aabb}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(rear_cover, elem="rear_cover_panel")
    with ctx.pose({cover_hinge: 1.35}):
        open_cover_aabb = ctx.part_element_world_aabb(rear_cover, elem="rear_cover_panel")
    ctx.check(
        "rear cover flips up away from the io edge",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.006
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] - 0.001,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
