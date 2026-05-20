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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 18) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((math.tau * index) / segments),
            radius * math.sin((math.tau * index) / segments),
        )
        for index in range(segments)
    ]


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="server_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.39, 0.20, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.11, 0.11, 0.12, 1.0))
    slot_black = model.material("slot_black", rgba=(0.14, 0.14, 0.16, 1.0))
    slot_gray = model.material("slot_gray", rgba=(0.77, 0.77, 0.74, 1.0))
    socket_gray = model.material("socket_gray", rgba=(0.66, 0.67, 0.69, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.36, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    heatsink = model.material("heatsink", rgba=(0.67, 0.70, 0.74, 1.0))
    connector_beige = model.material("connector_beige", rgba=(0.78, 0.74, 0.63, 1.0))

    board_x = 0.305
    board_y = 0.232
    board_t = 0.0032

    socket_cx = 0.015
    socket_cy = 0.020
    socket_w = 0.074
    socket_l = 0.074
    socket_h = 0.0050

    dimm_len = 0.142
    dimm_w = 0.0068
    dimm_h = 0.0082
    dimm_center_y = socket_cy
    dimm_barrel_r = 0.00115
    dimm_x_positions = [-0.088, -0.078, -0.068, -0.058, 0.074, 0.084, 0.094, 0.104]

    motherboard = model.part("motherboard")

    mounting_holes = [
        (-0.140, 0.098),
        (-0.020, 0.102),
        (0.110, 0.102),
        (0.140, 0.060),
        (-0.140, -0.095),
        (-0.015, -0.099),
        (0.110, -0.100),
        (0.140, -0.055),
        (0.015, -0.016),
    ]
    pcb_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(board_x, board_y, 0.008, corner_segments=6),
        [_shift_profile(_circle_profile(0.0020), x, y) for x, y in mounting_holes],
        board_t,
        center=False,
    )
    motherboard.visual(
        mesh_from_geometry(pcb_geom, "server_motherboard_pcb"),
        material=pcb_green,
        name="pcb",
    )

    motherboard.visual(
        Box((0.074, 0.016, 0.016)),
        origin=Origin(xyz=(socket_cx, socket_cy + 0.057, board_t + 0.008)),
        material=heatsink,
        name="vrm_heatsink_top",
    )
    motherboard.visual(
        Box((0.016, 0.086, 0.014)),
        origin=Origin(xyz=(socket_cx - 0.059, socket_cy, board_t + 0.007)),
        material=heatsink,
        name="vrm_heatsink_left",
    )
    motherboard.visual(
        Box((0.016, 0.066, 0.012)),
        origin=Origin(xyz=(socket_cx + 0.060, socket_cy + 0.006, board_t + 0.006)),
        material=heatsink,
        name="vrm_heatsink_right",
    )
    motherboard.visual(
        Box((0.014, 0.090, 0.017)),
        origin=Origin(xyz=(-board_x / 2.0 + 0.007, 0.012, board_t + 0.0085)),
        material=steel,
        name="rear_io_stack",
    )
    motherboard.visual(
        Box((0.012, 0.050, 0.014)),
        origin=Origin(xyz=(board_x / 2.0 - 0.006, 0.060, board_t + 0.007)),
        material=connector_beige,
        name="main_power_connector",
    )
    motherboard.visual(
        Box((0.044, 0.012, 0.012)),
        origin=Origin(xyz=(0.122, -0.094, board_t + 0.006)),
        material=connector_beige,
        name="sata_bank",
    )
    motherboard.visual(
        Box((0.090, 0.008, 0.011)),
        origin=Origin(xyz=(0.005, -0.063, board_t + 0.0055)),
        material=dark_plastic,
        name="pcie_slot_1",
    )
    motherboard.visual(
        Box((0.090, 0.008, 0.011)),
        origin=Origin(xyz=(0.020, -0.088, board_t + 0.0055)),
        material=dark_plastic,
        name="pcie_slot_2",
    )
    motherboard.visual(
        Box((0.038, 0.038, 0.0032)),
        origin=Origin(xyz=(0.082, -0.076, board_t + 0.0016)),
        material=dark_steel,
        name="blower_pad",
    )
    motherboard.visual(
        Box((0.020, 0.018, 0.003)),
        origin=Origin(xyz=(0.044, -0.077, board_t + 0.0015)),
        material=dark_plastic,
        name="bmc_chip",
    )
    motherboard.visual(
        Box((0.022, 0.010, 0.010)),
        origin=Origin(xyz=(0.112, -0.028, board_t + 0.005)),
        material=connector_beige,
        name="front_panel_header",
    )
    motherboard.visual(
        Box((0.074, 0.074, socket_h)),
        origin=Origin(xyz=(socket_cx, socket_cy, board_t + (socket_h / 2.0))),
        material=socket_gray,
        name="cpu_socket_body",
    )
    motherboard.visual(
        Box((0.057, 0.057, 0.0014)),
        origin=Origin(xyz=(socket_cx + 0.002, socket_cy, board_t + socket_h + 0.0007)),
        material=dark_steel,
        name="cpu_land_area",
    )

    dimm_slot_materials = [
        slot_gray,
        slot_black,
        slot_gray,
        slot_black,
        slot_gray,
        slot_black,
        slot_gray,
        slot_black,
    ]
    for index, (slot_x, slot_material) in enumerate(
        zip(dimm_x_positions, dimm_slot_materials),
        start=1,
    ):
        motherboard.visual(
            Box((dimm_w, dimm_len, dimm_h)),
            origin=Origin(xyz=(slot_x, dimm_center_y, board_t + (dimm_h / 2.0))),
            material=slot_material,
            name=f"dimm_slot_{index}",
        )

    motherboard.inertial = Inertial.from_geometry(
        Box((board_x, board_y, 0.030)),
        mass=1.45,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    cpu_retention_frame = model.part("cpu_retention_frame")
    frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.076, 0.080, 0.0025, corner_segments=4),
            [rounded_rect_profile(0.060, 0.064, 0.0018, corner_segments=4)],
            0.0022,
            center=True,
        ),
        "cpu_retention_frame_plate",
    )
    cpu_retention_frame.visual(
        frame_mesh,
        origin=Origin(xyz=(0.038, 0.0, 0.0020)),
        material=steel,
        name="retention_plate",
    )
    cpu_retention_frame.visual(
        Cylinder(radius=0.0014, length=0.010),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="frame_hinge_clip_front",
    )
    cpu_retention_frame.visual(
        Cylinder(radius=0.0014, length=0.010),
        origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="frame_hinge_clip_rear",
    )
    cpu_retention_frame.visual(
        Box((0.008, 0.010, 0.003)),
        origin=Origin(xyz=(0.004, 0.023, 0.0013)),
        material=steel,
        name="frame_hinge_web_front",
    )
    cpu_retention_frame.visual(
        Box((0.008, 0.010, 0.003)),
        origin=Origin(xyz=(0.004, -0.023, 0.0013)),
        material=steel,
        name="frame_hinge_web_rear",
    )
    cpu_retention_frame.inertial = Inertial.from_geometry(
        Box((0.078, 0.082, 0.007)),
        mass=0.08,
        origin=Origin(xyz=(0.039, 0.0, 0.002)),
    )

    cpu_retention_lever = model.part("cpu_retention_lever")
    cpu_retention_lever.visual(
        Cylinder(radius=0.0012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lever_pivot",
    )
    cpu_retention_lever.visual(
        Cylinder(radius=0.0011, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="lever_arm",
    )
    cpu_retention_lever.visual(
        Cylinder(radius=0.0018, length=0.014),
        origin=Origin(xyz=(0.031, 0.0, 0.007), rpy=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="lever_handle",
    )
    cpu_retention_lever.inertial = Inertial.from_geometry(
        Box((0.036, 0.014, 0.016)),
        mass=0.02,
        origin=Origin(xyz=(0.018, 0.0, 0.007)),
    )

    chipset_blower_housing = model.part("chipset_blower_housing")
    chipset_blower_housing.visual(
        Box((0.036, 0.036, 0.0032)),
        origin=Origin(xyz=(0.0, 0.0, 0.0016)),
        material=satin_black,
        name="blower_mount",
    )
    chipset_blower_housing.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0080)),
        material=dark_plastic,
        name="blower_shroud",
    )
    chipset_blower_housing.visual(
        Cylinder(radius=0.0145, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0131)),
        material=steel,
        name="blower_grille",
    )
    chipset_blower_housing.visual(
        Box((0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.016, 0.0, 0.005)),
        material=dark_plastic,
        name="blower_outlet",
    )
    chipset_blower_housing.inertial = Inertial.from_geometry(
        Box((0.038, 0.036, 0.015)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
    )

    chipset_blower_rotor = model.part("chipset_blower_rotor")
    chipset_blower_rotor.visual(
        Cylinder(radius=0.0042, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="rotor_hub",
    )
    for blade_index in range(5):
        blade_angle = blade_index * math.tau / 5.0
        chipset_blower_rotor.visual(
            Box((0.0045, 0.013, 0.0016)),
            origin=Origin(
                xyz=(0.0082 * math.cos(blade_angle), 0.0082 * math.sin(blade_angle), 0.0),
                rpy=(0.0, 0.0, blade_angle + (math.pi / 2.0)),
            ),
            material=dark_steel,
            name=f"rotor_blade_{blade_index + 1}",
        )
    chipset_blower_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.004),
        mass=0.015,
    )

    model.articulation(
        "motherboard_to_cpu_retention_frame",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=cpu_retention_frame,
        origin=Origin(
            xyz=(
                socket_cx - (socket_w / 2.0),
                socket_cy,
                board_t + socket_h + 0.0014,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "motherboard_to_cpu_retention_lever",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=cpu_retention_lever,
        origin=Origin(
            xyz=(
                socket_cx + 0.034,
                socket_cy - 0.026,
                board_t + socket_h + 0.0012,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "motherboard_to_chipset_blower_housing",
        ArticulationType.FIXED,
        parent=motherboard,
        child=chipset_blower_housing,
        origin=Origin(xyz=(0.082, -0.076, board_t)),
    )
    model.articulation(
        "chipset_blower_housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=chipset_blower_housing,
        child=chipset_blower_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=40.0),
    )

    for slot_index, slot_x in enumerate(dimm_x_positions, start=1):
        for suffix, sign, axis in (
            ("upper", 1.0, (-1.0, 0.0, 0.0)),
            ("lower", -1.0, (1.0, 0.0, 0.0)),
        ):
            latch = model.part(f"dimm_{slot_index}_{suffix}_latch")
            latch.visual(
                Cylinder(radius=dimm_barrel_r, length=0.014),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=slot_gray,
                name="latch_barrel",
            )
            latch.visual(
                Box((0.013, 0.0032, 0.010)),
                origin=Origin(xyz=(0.0, sign * 0.0021, 0.0055)),
                material=slot_gray,
                name="latch_body",
            )
            latch.visual(
                Box((0.007, 0.0042, 0.003)),
                origin=Origin(xyz=(0.0, sign * 0.0051, 0.0108)),
                material=slot_gray,
                name="latch_hook",
            )
            latch.inertial = Inertial.from_geometry(
                Box((0.013, 0.008, 0.014)),
                mass=0.003,
                origin=Origin(xyz=(0.0, sign * 0.003, 0.007)),
            )
            model.articulation(
                f"motherboard_to_dimm_{slot_index}_{suffix}_latch",
                ArticulationType.REVOLUTE,
                parent=motherboard,
                child=latch,
                origin=Origin(
                    xyz=(
                        slot_x,
                        dimm_center_y + sign * ((dimm_len / 2.0) - 0.004),
                        board_t + dimm_h + dimm_barrel_r,
                    )
                ),
                axis=axis,
                motion_limits=MotionLimits(
                    effort=0.2,
                    velocity=4.0,
                    lower=0.0,
                    upper=math.radians(48.0),
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motherboard = object_model.get_part("motherboard")
    cpu_retention_frame = object_model.get_part("cpu_retention_frame")
    cpu_retention_lever = object_model.get_part("cpu_retention_lever")
    chipset_blower_housing = object_model.get_part("chipset_blower_housing")
    chipset_blower_rotor = object_model.get_part("chipset_blower_rotor")

    frame_hinge = object_model.get_articulation("motherboard_to_cpu_retention_frame")
    lever_joint = object_model.get_articulation("motherboard_to_cpu_retention_lever")
    blower_joint = object_model.get_articulation("chipset_blower_housing_to_rotor")

    slot_visuals = [motherboard.get_visual(f"dimm_slot_{index}") for index in range(1, 9)]
    latch_parts = [
        object_model.get_part(f"dimm_{index}_{suffix}_latch")
        for index in range(1, 9)
        for suffix in ("upper", "lower")
    ]
    latch_joints = [
        object_model.get_articulation(f"motherboard_to_dimm_{index}_{suffix}_latch")
        for index in range(1, 9)
        for suffix in ("upper", "lower")
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    pcb_aabb = ctx.part_element_world_aabb(motherboard, elem="pcb")
    if pcb_aabb is not None:
        pcb_dx = pcb_aabb[1][0] - pcb_aabb[0][0]
        pcb_dy = pcb_aabb[1][1] - pcb_aabb[0][1]
        ctx.check(
            "board_has_narrow_server_proportions",
            pcb_dx > pcb_dy and (pcb_dx / pcb_dy) > 1.25,
            f"pcb footprint {pcb_dx:.3f} x {pcb_dy:.3f}",
        )

    ctx.check("eight_memory_slots_present", len(slot_visuals) == 8, "Expected 8 DIMM slot visuals.")
    ctx.check(
        "cpu_frame_axis_is_hinge_like",
        tuple(frame_hinge.axis) == (0.0, -1.0, 0.0),
        f"axis={frame_hinge.axis}",
    )
    ctx.check(
        "cpu_lever_axis_is_local_pivot",
        tuple(lever_joint.axis) == (0.0, -1.0, 0.0),
        f"axis={lever_joint.axis}",
    )
    ctx.check(
        "blower_rotor_axis_is_vertical",
        tuple(blower_joint.axis) == (0.0, 0.0, 1.0),
        f"axis={blower_joint.axis}",
    )
    ctx.check(
        "blower_rotor_is_continuous",
        blower_joint.motion_limits is not None
        and blower_joint.motion_limits.lower is None
        and blower_joint.motion_limits.upper is None,
        f"limits={blower_joint.motion_limits}",
    )

    ctx.expect_contact(
        cpu_retention_frame,
        motherboard,
        elem_a="frame_hinge_clip_front",
        elem_b="cpu_socket_body",
        name="cpu_frame_front_clip_contacts_socket",
    )
    ctx.expect_contact(
        cpu_retention_frame,
        motherboard,
        elem_a="frame_hinge_clip_rear",
        elem_b="cpu_socket_body",
        name="cpu_frame_rear_clip_contacts_socket",
    )
    ctx.expect_contact(
        cpu_retention_lever,
        motherboard,
        elem_a="lever_pivot",
        elem_b="cpu_socket_body",
        name="cpu_lever_pivot_contacts_socket",
    )
    ctx.expect_contact(
        chipset_blower_housing,
        motherboard,
        elem_a="blower_mount",
        elem_b="blower_pad",
        name="blower_housing_mounted_to_board",
    )
    ctx.expect_within(
        chipset_blower_rotor,
        chipset_blower_housing,
        axes="xy",
        margin=0.001,
        name="blower_rotor_within_housing",
    )

    for index, slot_visual in enumerate(slot_visuals, start=1):
        ctx.expect_overlap(
            motherboard,
            motherboard,
            elem_a=slot_visual.name,
            elem_b="cpu_socket_body",
            axes="y",
            min_overlap=0.050,
            name=f"dimm_slot_{index}_runs_along_socket_bank",
        )

    for joint in latch_joints:
        expected_axis = (-1.0, 0.0, 0.0) if "_upper_" in joint.name else (1.0, 0.0, 0.0)
        ctx.check(
            f"{joint.name}_axis",
            tuple(joint.axis) == expected_axis,
            f"axis={joint.axis}",
        )

    for index in range(1, 9):
        for suffix in ("upper", "lower"):
            latch = object_model.get_part(f"dimm_{index}_{suffix}_latch")
            ctx.expect_contact(
                latch,
                motherboard,
                elem_a="latch_barrel",
                elem_b=f"dimm_slot_{index}",
                name=f"dimm_{index}_{suffix}_latch_contacts_slot",
            )

    frame_rest_aabb = ctx.part_world_aabb(cpu_retention_frame)
    lever_rest_aabb = ctx.part_world_aabb(cpu_retention_lever)
    upper_latch_rest_aabb = ctx.part_world_aabb(latch_parts[0])
    lower_latch_rest_aabb = ctx.part_world_aabb(latch_parts[1])

    with ctx.pose({frame_hinge: math.radians(104.0)}):
        ctx.expect_contact(
            cpu_retention_frame,
            motherboard,
            elem_a="frame_hinge_clip_front",
            elem_b="cpu_socket_body",
            name="cpu_frame_front_clip_stays_attached_open",
        )
        ctx.expect_contact(
            cpu_retention_frame,
            motherboard,
            elem_a="frame_hinge_clip_rear",
            elem_b="cpu_socket_body",
            name="cpu_frame_rear_clip_stays_attached_open",
        )
        frame_open_aabb = ctx.part_world_aabb(cpu_retention_frame)
        if frame_rest_aabb is not None and frame_open_aabb is not None:
            ctx.check(
                "cpu_retention_frame_opens_upward",
                frame_open_aabb[1][2] > frame_rest_aabb[1][2] + 0.040,
                f"rest_zmax={frame_rest_aabb[1][2]:.4f}, open_zmax={frame_open_aabb[1][2]:.4f}",
            )

    with ctx.pose({lever_joint: math.radians(96.0)}):
        ctx.expect_contact(
            cpu_retention_lever,
            motherboard,
            elem_a="lever_pivot",
            elem_b="cpu_socket_body",
            name="cpu_lever_pivot_stays_attached_open",
        )
        lever_open_aabb = ctx.part_world_aabb(cpu_retention_lever)
        if lever_rest_aabb is not None and lever_open_aabb is not None:
            ctx.check(
                "cpu_retention_lever_lifts",
                lever_open_aabb[1][2] > lever_rest_aabb[1][2] + 0.015,
                f"rest_zmax={lever_rest_aabb[1][2]:.4f}, open_zmax={lever_open_aabb[1][2]:.4f}",
            )

    upper_latch_joint = object_model.get_articulation("motherboard_to_dimm_1_upper_latch")
    lower_latch_joint = object_model.get_articulation("motherboard_to_dimm_1_lower_latch")
    with ctx.pose({upper_latch_joint: math.radians(42.0), lower_latch_joint: math.radians(42.0)}):
        ctx.expect_contact(
            object_model.get_part("dimm_1_upper_latch"),
            motherboard,
            elem_a="latch_barrel",
            elem_b="dimm_slot_1",
            name="upper_dimm_latch_hinge_stays_attached_open",
        )
        ctx.expect_contact(
            object_model.get_part("dimm_1_lower_latch"),
            motherboard,
            elem_a="latch_barrel",
            elem_b="dimm_slot_1",
            name="lower_dimm_latch_hinge_stays_attached_open",
        )
        upper_latch_open_aabb = ctx.part_world_aabb(object_model.get_part("dimm_1_upper_latch"))
        lower_latch_open_aabb = ctx.part_world_aabb(object_model.get_part("dimm_1_lower_latch"))
        if upper_latch_rest_aabb is not None and upper_latch_open_aabb is not None:
            ctx.check(
                "upper_dimm_latch_opens",
                upper_latch_open_aabb[1][1] > upper_latch_rest_aabb[1][1] + 0.004,
                f"rest_ymax={upper_latch_rest_aabb[1][1]:.4f}, open_ymax={upper_latch_open_aabb[1][1]:.4f}",
            )
        if lower_latch_rest_aabb is not None and lower_latch_open_aabb is not None:
            ctx.check(
                "lower_dimm_latch_opens",
                lower_latch_open_aabb[0][1] < lower_latch_rest_aabb[0][1] - 0.004,
                f"rest_ymin={lower_latch_rest_aabb[0][1]:.4f}, open_ymin={lower_latch_open_aabb[0][1]:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
