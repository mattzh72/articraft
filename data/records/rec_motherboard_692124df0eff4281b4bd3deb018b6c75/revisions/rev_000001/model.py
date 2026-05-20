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


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 18,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _load_plate_mesh() -> object:
    outer = rounded_rect_profile(0.072, 0.066, 0.002, corner_segments=5)
    inner = rounded_rect_profile(0.054, 0.050, 0.0015, corner_segments=4)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            height=0.0012,
            center=True,
        ).translate(0.036, 0.0, 0.0),
        "socket_load_plate",
    )


def _socket_frame_mesh() -> object:
    outer = rounded_rect_profile(0.078, 0.078, 0.003, corner_segments=6)
    inner = rounded_rect_profile(0.056, 0.056, 0.002, corner_segments=5)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            height=0.006,
            center=True,
        ),
        "cpu_socket_frame",
    )


def _board_mesh(board_x: float, board_y: float, board_t: float) -> object:
    hole_centers = [
        (-0.137, 0.148),
        (0.137, 0.148),
        (-0.137, -0.148),
        (0.137, -0.148),
        (-0.137, 0.030),
        (0.137, 0.030),
        (-0.015, -0.122),
        (0.117, -0.122),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(board_x, board_y, 0.006, corner_segments=7),
            [_circle_profile(0.0033, center=center, segments=16) for center in hole_centers],
            height=board_t,
            center=True,
        ),
        "server_motherboard_pcb",
    )


def _add_slot_assembly(
    model: ArticulatedObject,
    board,
    *,
    slot_index: int,
    slot_x: float,
    slot_y: float,
    board_t: float,
    slot_material,
    latch_material,
    steel_material,
) -> tuple[str, str, str]:
    slot_name = f"dimm_slot_{slot_index}"
    slot = model.part(slot_name)
    slot.visual(
        Box((0.009, 0.128, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=slot_material,
        name="slot_body",
    )
    slot.visual(
        Box((0.013, 0.009, 0.017)),
        origin=Origin(xyz=(0.0, 0.0675, 0.0085)),
        material=slot_material,
        name="top_cheek",
    )
    slot.visual(
        Box((0.013, 0.009, 0.017)),
        origin=Origin(xyz=(0.0, -0.0675, 0.0085)),
        material=slot_material,
        name="bottom_cheek",
    )
    slot.visual(
        Box((0.004, 0.116, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material="contact_gold",
        name="contact_rail",
    )
    slot.visual(
        Box((0.013, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.072, 0.015)),
        material=latch_material,
        name="top_latch_seat",
    )
    slot.visual(
        Box((0.013, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.072, 0.015)),
        material=latch_material,
        name="bottom_latch_seat",
    )
    slot.inertial = Inertial.from_geometry(
        Box((0.013, 0.148, 0.015)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
    )
    model.articulation(
        f"motherboard_to_{slot_name}",
        ArticulationType.FIXED,
        parent=board,
        child=slot,
        origin=Origin(xyz=(slot_x, slot_y, board_t)),
    )

    top_latch_name = f"{slot_name}_top_latch"
    top_latch = model.part(top_latch_name)
    top_latch.visual(
        Cylinder(radius=0.0015, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_material,
        name="pivot_pin",
    )
    top_latch.visual(
        Box((0.004, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -0.004, 0.0045)),
        material=latch_material,
        name="latch_arm",
    )
    top_latch.visual(
        Box((0.007, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, -0.009, 0.008)),
        material=latch_material,
        name="latch_hook",
    )
    top_latch.visual(
        Box((0.010, 0.003, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=latch_material,
        name="latch_base",
    )
    top_latch.inertial = Inertial.from_geometry(
        Box((0.010, 0.014, 0.012)),
        mass=0.008,
        origin=Origin(xyz=(0.0, -0.004, 0.006)),
    )
    model.articulation(
        f"{slot_name}_top_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=slot,
        child=top_latch,
        origin=Origin(xyz=(0.0, 0.072, 0.015)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.5,
            lower=0.0,
            upper=0.62,
        ),
    )

    bottom_latch_name = f"{slot_name}_bottom_latch"
    bottom_latch = model.part(bottom_latch_name)
    bottom_latch.visual(
        Cylinder(radius=0.0015, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_material,
        name="pivot_pin",
    )
    bottom_latch.visual(
        Box((0.004, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.004, 0.0045)),
        material=latch_material,
        name="latch_arm",
    )
    bottom_latch.visual(
        Box((0.007, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, 0.009, 0.008)),
        material=latch_material,
        name="latch_hook",
    )
    bottom_latch.visual(
        Box((0.010, 0.003, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=latch_material,
        name="latch_base",
    )
    bottom_latch.inertial = Inertial.from_geometry(
        Box((0.010, 0.014, 0.012)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.004, 0.006)),
    )
    model.articulation(
        f"{slot_name}_bottom_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=slot,
        child=bottom_latch,
        origin=Origin(xyz=(0.0, -0.072, 0.015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.5,
            lower=0.0,
            upper=0.62,
        ),
    )

    return slot_name, top_latch_name, bottom_latch_name


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="server_motherboard")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.31, 0.18, 1.0))
    solder_mask_dark = model.material("solder_mask_dark", rgba=(0.06, 0.16, 0.10, 1.0))
    socket_black = model.material("socket_black", rgba=(0.10, 0.10, 0.11, 1.0))
    slot_black = model.material("slot_black", rgba=(0.11, 0.11, 0.12, 1.0))
    slot_blue = model.material("slot_blue", rgba=(0.20, 0.24, 0.39, 1.0))
    latch_black = model.material("latch_black", rgba=(0.16, 0.16, 0.18, 1.0))
    heat_sink = model.material("heat_sink", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_connector = model.material("dark_connector", rgba=(0.20, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.80, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    contact_gold = model.material("contact_gold", rgba=(0.82, 0.69, 0.32, 1.0))
    io_silver = model.material("io_silver", rgba=(0.74, 0.75, 0.77, 1.0))
    handle_beige = model.material("handle_beige", rgba=(0.73, 0.67, 0.56, 1.0))

    board_x = 0.305
    board_y = 0.330
    board_t = 0.0024

    motherboard = model.part("motherboard")
    motherboard.visual(
        _board_mesh(board_x, board_y, board_t),
        origin=Origin(xyz=(0.0, 0.0, board_t / 2.0)),
        material=pcb_green,
        name="pcb_main",
    )
    motherboard.visual(
        Box((0.022, 0.100, 0.030)),
        origin=Origin(xyz=(-0.1415, 0.112, board_t + 0.015)),
        material=io_silver,
        name="rear_io_stack",
    )
    motherboard.visual(
        Box((0.094, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.110, board_t + 0.012)),
        material=heat_sink,
        name="vrm_sink_top",
    )
    motherboard.visual(
        Box((0.020, 0.088, 0.024)),
        origin=Origin(xyz=(-0.050, 0.055, board_t + 0.012)),
        material=heat_sink,
        name="vrm_sink_left",
    )
    motherboard.visual(
        Box((0.020, 0.088, 0.024)),
        origin=Origin(xyz=(0.050, 0.055, board_t + 0.012)),
        material=heat_sink,
        name="vrm_sink_right",
    )
    motherboard.visual(
        Box((0.042, 0.042, 0.018)),
        origin=Origin(xyz=(0.0, -0.040, board_t + 0.009)),
        material=heat_sink,
        name="chipset_sink",
    )
    motherboard.visual(
        Box((0.110, 0.008, 0.012)),
        origin=Origin(xyz=(-0.010, -0.096, board_t + 0.006)),
        material=dark_connector,
        name="pcie_slot_upper",
    )
    motherboard.visual(
        Box((0.110, 0.008, 0.012)),
        origin=Origin(xyz=(-0.010, -0.116, board_t + 0.006)),
        material=dark_connector,
        name="pcie_slot_lower",
    )
    motherboard.visual(
        Box((0.010, 0.062, 0.014)),
        origin=Origin(xyz=(0.142, 0.090, board_t + 0.007)),
        material=dark_connector,
        name="atx_power_connector",
    )
    motherboard.visual(
        Box((0.016, 0.018, 0.013)),
        origin=Origin(xyz=(0.128, 0.145, board_t + 0.0065)),
        material=dark_connector,
        name="eps_power_upper",
    )
    motherboard.visual(
        Box((0.016, 0.018, 0.013)),
        origin=Origin(xyz=(0.108, 0.145, board_t + 0.0065)),
        material=dark_connector,
        name="eps_power_lower",
    )
    motherboard.visual(
        Box((0.056, 0.020, 0.006)),
        origin=Origin(xyz=(0.058, -0.060, board_t + 0.003)),
        material=solder_mask_dark,
        name="m2_shield",
    )
    for index, x_pos in enumerate((-0.070, -0.056, -0.042, 0.042, 0.056, 0.070)):
        motherboard.visual(
            Cylinder(radius=0.0032, length=0.010),
            origin=Origin(xyz=(x_pos, 0.122, board_t + 0.005)),
            material=dark_connector if index % 2 == 0 else io_silver,
            name=f"vrm_cap_{index}",
        )
    motherboard.inertial = Inertial.from_geometry(
        Box((board_x, board_y, 0.035)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
    )

    socket_frame = model.part("cpu_socket_frame")
    socket_frame.visual(
        _socket_frame_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=socket_black,
        name="socket_body",
    )
    socket_frame.visual(
        Box((0.058, 0.058, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0008)),
        material=socket_black,
        name="socket_contact_substrate",
    )
    socket_frame.visual(
        Box((0.054, 0.054, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.00205)),
        material=contact_gold,
        name="socket_contacts",
    )
    socket_frame.visual(
        Box((0.010, 0.018, 0.002)),
        origin=Origin(xyz=(0.0405, -0.027, 0.004)),
        material=socket_black,
        name="lever_pivot_seat",
    )
    socket_frame.visual(
        Box((0.006, 0.024, 0.006)),
        origin=Origin(xyz=(-0.040, 0.020, 0.003)),
        material=socket_black,
        name="hinge_upper_lug",
    )
    socket_frame.visual(
        Box((0.006, 0.024, 0.006)),
        origin=Origin(xyz=(-0.040, -0.020, 0.003)),
        material=socket_black,
        name="hinge_lower_lug",
    )
    socket_frame.inertial = Inertial.from_geometry(
        Box((0.078, 0.078, 0.008)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "motherboard_to_cpu_socket_frame",
        ArticulationType.FIXED,
        parent=motherboard,
        child=socket_frame,
        origin=Origin(xyz=(0.0, 0.055, board_t)),
    )

    load_plate = model.part("socket_load_plate")
    load_plate.visual(
        _load_plate_mesh(),
        material=brushed_steel,
        name="plate_frame",
    )
    load_plate.visual(
        Box((0.006, 0.018, 0.0015)),
        origin=Origin(xyz=(0.069, 0.0, 0.0)),
        material=brushed_steel,
        name="plate_finger",
    )
    load_plate.inertial = Inertial.from_geometry(
        Box((0.074, 0.068, 0.002)),
        mass=0.035,
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
    )
    model.articulation(
        "cpu_socket_frame_to_load_plate",
        ArticulationType.REVOLUTE,
        parent=socket_frame,
        child=load_plate,
        origin=Origin(xyz=(-0.036, 0.0, 0.00675)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    socket_lever = model.part("socket_lever")
    socket_lever.visual(
        Cylinder(radius=0.0015, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lever_pivot_pin",
    )
    socket_lever.visual(
        Cylinder(radius=0.0017, length=0.078),
        origin=Origin(xyz=(0.039, 0.0, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lever_main_rod",
    )
    socket_lever.visual(
        Cylinder(radius=0.0017, length=0.024),
        origin=Origin(xyz=(0.083, 0.0, 0.0055), rpy=(0.0, 0.55, 0.0)),
        material=steel,
        name="lever_raise_rod",
    )
    socket_lever.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.096, 0.0, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_beige,
        name="lever_handle_tip",
    )
    socket_lever.visual(
        Box((0.007, 0.004, 0.002)),
        origin=Origin(xyz=(0.006, 0.0, -0.0015)),
        material=steel,
        name="lever_cam",
    )
    socket_lever.inertial = Inertial.from_geometry(
        Box((0.102, 0.018, 0.020)),
        mass=0.02,
        origin=Origin(xyz=(0.050, 0.0, 0.005)),
    )
    model.articulation(
        "cpu_socket_frame_to_lever",
        ArticulationType.REVOLUTE,
        parent=socket_frame,
        child=socket_lever,
        origin=Origin(xyz=(0.0405, -0.027, 0.0065)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    dimm_slots: list[tuple[str, str, str]] = []
    slot_positions = (-0.105, -0.093, -0.081, -0.069, 0.069, 0.081, 0.093, 0.105)
    for slot_index, slot_x in enumerate(slot_positions):
        dimm_slots.append(
            _add_slot_assembly(
                model,
                motherboard,
                slot_index=slot_index,
                slot_x=slot_x,
                slot_y=0.048,
                board_t=board_t,
                slot_material=slot_blue if slot_index % 2 else slot_black,
                latch_material=latch_black,
                steel_material=steel,
            )
        )

    model.meta["slot_assemblies"] = dimm_slots
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    motherboard = object_model.get_part("motherboard")
    socket_frame = object_model.get_part("cpu_socket_frame")
    load_plate = object_model.get_part("socket_load_plate")
    socket_lever = object_model.get_part("socket_lever")
    load_plate_hinge = object_model.get_articulation("cpu_socket_frame_to_load_plate")
    lever_hinge = object_model.get_articulation("cpu_socket_frame_to_lever")

    ctx.check(
        "load plate hinge axis is socket-edge aligned",
        tuple(load_plate_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={load_plate_hinge.axis}",
    )
    ctx.check(
        "socket lever hinge axis is socket-edge aligned",
        tuple(lever_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={lever_hinge.axis}",
    )

    ctx.expect_contact(socket_frame, motherboard, name="socket frame mounted to motherboard")
    ctx.expect_overlap(socket_frame, motherboard, axes="xy", min_overlap=0.070, name="socket sits within board footprint")
    ctx.expect_gap(
        load_plate,
        socket_frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.0012,
        positive_elem="plate_frame",
        negative_elem="socket_body",
        name="load plate rests on socket frame without penetration",
    )
    ctx.expect_overlap(
        load_plate,
        socket_frame,
        axes="xy",
        min_overlap=0.050,
        name="load plate covers socket opening",
    )
    ctx.expect_contact(socket_lever, socket_frame, name="lever pivot touches socket frame")

    slot_assemblies = tuple(object_model.meta.get("slot_assemblies", ()))
    for slot_name, top_latch_name, bottom_latch_name in slot_assemblies:
        slot = object_model.get_part(slot_name)
        top_latch = object_model.get_part(top_latch_name)
        bottom_latch = object_model.get_part(bottom_latch_name)
        top_joint = object_model.get_articulation(f"{slot_name}_top_latch_hinge")
        bottom_joint = object_model.get_articulation(f"{slot_name}_bottom_latch_hinge")

        ctx.check(
            f"{slot_name} top latch axis points outward",
            tuple(top_joint.axis) == (-1.0, 0.0, 0.0),
            details=f"axis={top_joint.axis}",
        )
        ctx.check(
            f"{slot_name} bottom latch axis points outward",
            tuple(bottom_joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={bottom_joint.axis}",
        )
        ctx.expect_contact(slot, motherboard, name=f"{slot_name} seated on motherboard")
        ctx.expect_gap(
            slot,
            motherboard,
            axis="z",
            max_penetration=1e-6,
            max_gap=0.0002,
            negative_elem="pcb_main",
            name=f"{slot_name} has no stand-off gap",
        )
        ctx.expect_overlap(
            slot,
            motherboard,
            axes="xy",
            min_overlap=0.009,
            name=f"{slot_name} lies within board footprint",
        )
        ctx.expect_contact(top_latch, slot, name=f"{top_latch_name} mounted on slot")
        ctx.expect_contact(bottom_latch, slot, name=f"{bottom_latch_name} mounted on slot")

    def elem_aabb(part, elem: str, label: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            ctx.fail(label, f"missing world AABB for {part.name}:{elem}")
        return aabb

    plate_rest = elem_aabb(load_plate, "plate_finger", "load plate finger measurable at rest")
    lever_rest = elem_aabb(socket_lever, "lever_handle_tip", "socket lever tip measurable at rest")
    if plate_rest is not None:
        with ctx.pose({load_plate_hinge: math.radians(72.0)}):
            plate_open = elem_aabb(load_plate, "plate_finger", "load plate finger measurable when open")
            if plate_open is not None:
                ctx.check(
                    "load plate swings upward",
                    plate_open[1][2] > plate_rest[1][2] + 0.040,
                    details=f"rest_z={plate_rest[1][2]:.4f}, open_z={plate_open[1][2]:.4f}",
                )
                ctx.expect_gap(
                    load_plate,
                    socket_frame,
                    axis="z",
                    min_gap=0.020,
                    positive_elem="plate_finger",
                    negative_elem="socket_body",
                    name="load plate finger clears above socket when open",
                )

    if lever_rest is not None:
        with ctx.pose({lever_hinge: math.radians(78.0)}):
            lever_open = elem_aabb(socket_lever, "lever_handle_tip", "socket lever tip measurable when open")
            if lever_open is not None:
                ctx.check(
                    "socket lever rotates upward",
                    lever_open[1][2] > lever_rest[1][2] + 0.020,
                    details=f"rest_z={lever_rest[1][2]:.4f}, open_z={lever_open[1][2]:.4f}",
                )
                ctx.expect_gap(
                    socket_lever,
                    socket_frame,
                    axis="z",
                    min_gap=0.010,
                    positive_elem="lever_handle_tip",
                    negative_elem="socket_body",
                    name="socket lever handle rises above the socket frame",
                )

    first_slot = object_model.get_part("dimm_slot_0")
    first_top_latch = object_model.get_part("dimm_slot_0_top_latch")
    first_bottom_latch = object_model.get_part("dimm_slot_0_bottom_latch")
    first_top_joint = object_model.get_articulation("dimm_slot_0_top_latch_hinge")
    first_bottom_joint = object_model.get_articulation("dimm_slot_0_bottom_latch_hinge")
    top_rest = elem_aabb(first_top_latch, "latch_hook", "top latch hook measurable at rest")
    bottom_rest = elem_aabb(first_bottom_latch, "latch_hook", "bottom latch hook measurable at rest")

    if top_rest is not None:
        with ctx.pose({first_top_joint: 0.55}):
            top_open = elem_aabb(first_top_latch, "latch_hook", "top latch hook measurable when open")
            if top_open is not None:
                ctx.check(
                    "top dimm latch swings away from slot center",
                    top_open[1][1] > top_rest[1][1] + 0.004,
                    details=f"rest_y={top_rest[1][1]:.4f}, open_y={top_open[1][1]:.4f}",
                )
                ctx.expect_contact(first_top_latch, first_slot, name="top dimm latch stays pinned to slot when open")

    if bottom_rest is not None:
        with ctx.pose({first_bottom_joint: 0.55}):
            bottom_open = elem_aabb(first_bottom_latch, "latch_hook", "bottom latch hook measurable when open")
            if bottom_open is not None:
                ctx.check(
                    "bottom dimm latch swings away from slot center",
                    bottom_open[0][1] < bottom_rest[0][1] - 0.004,
                    details=f"rest_y={bottom_rest[0][1]:.4f}, open_y={bottom_open[0][1]:.4f}",
                )
                ctx.expect_contact(first_bottom_latch, first_slot, name="bottom dimm latch stays pinned to slot when open")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
