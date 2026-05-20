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
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _tube_shell_mesh(name: str, outer_radius: float, inner_radius: float, length: float):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (outer_radius, -0.5 * length),
                (outer_radius, 0.5 * length),
            ],
            [
                (inner_radius, -0.5 * length),
                (inner_radius, 0.5 * length),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _axis_matches(
    axis: tuple[float, float, float] | None,
    target: tuple[float, float, float],
    tol: float = 1e-6,
) -> bool:
    if axis is None:
        return False
    return all(abs(a - b) <= tol for a, b in zip(axis, target))


def _visual_names(part) -> set[str]:
    return {visual.name for visual in part.visuals if visual.name}


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer")

    width = 0.60
    depth = 0.64
    height = 0.86
    shell_t = 0.018
    front_t = 0.022
    service_panel_t = 0.012

    door_center_z = 0.42
    door_hinge_x = 0.370
    door_outer_radius = 0.215

    drum_center = (0.03, 0.0, 0.425)
    drum_radius = 0.225
    drum_length = 0.34

    service_panel_width = 0.476
    service_panel_depth = 0.362
    service_hinge_x = (-0.5 * depth) + 0.014
    service_panel_center_x = service_hinge_x + (0.5 * service_panel_depth)
    service_side_rail_w = 0.5 * (width - service_panel_width)

    white_enamel = model.material("white_enamel", rgba=(0.94, 0.95, 0.96, 1.0))
    warm_white = model.material("warm_white", rgba=(0.90, 0.91, 0.92, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.09, 0.10, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.60, 0.78, 0.92, 0.35))
    steel = model.material("steel", rgba=(0.64, 0.68, 0.72, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.12, 0.45, 0.23, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.61, 0.28, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((depth, shell_t, height)),
        origin=Origin(xyz=(0.0, -(0.5 * width) + (0.5 * shell_t), 0.5 * height)),
        material=white_enamel,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((depth, shell_t, height)),
        origin=Origin(xyz=(0.0, (0.5 * width) - (0.5 * shell_t), 0.5 * height)),
        material=white_enamel,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((depth - (2.0 * shell_t), width - (2.0 * shell_t), shell_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * shell_t)),
        material=warm_white,
        name="cabinet_floor",
    )
    cabinet.visual(
        Box((shell_t, width - (2.0 * shell_t), 0.80)),
        origin=Origin(xyz=(-(0.5 * depth) + (0.5 * shell_t), 0.0, 0.40)),
        material=white_enamel,
        name="rear_panel",
    )
    cabinet.visual(
        Box((0.15, width - (2.0 * shell_t), 0.18)),
        origin=Origin(xyz=((0.5 * depth) - 0.075, 0.0, 0.09)),
        material=white_enamel,
        name="lower_kick_panel",
    )
    cabinet.visual(
        Box((front_t, width - (2.0 * shell_t), 0.22)),
        origin=Origin(xyz=((0.5 * depth) - (0.5 * front_t), 0.0, 0.75)),
        material=white_enamel,
        name="front_upper_frame",
    )
    cabinet.visual(
        Box((front_t, 0.082, 0.46)),
        origin=Origin(
            xyz=((0.5 * depth) - (0.5 * front_t), -0.252, door_center_z),
        ),
        material=white_enamel,
        name="front_left_stile",
    )
    cabinet.visual(
        Box((front_t, 0.082, 0.46)),
        origin=Origin(
            xyz=((0.5 * depth) - (0.5 * front_t), 0.252, door_center_z),
        ),
        material=white_enamel,
        name="front_right_stile",
    )
    cabinet.visual(
        Box((0.262, width, shell_t)),
        origin=Origin(xyz=(0.189, 0.0, height - (0.5 * shell_t))),
        material=white_enamel,
        name="top_front_strip",
    )
    cabinet.visual(
        Box((service_panel_depth, service_side_rail_w, shell_t)),
        origin=Origin(
            xyz=(
                service_panel_center_x,
                -(0.5 * width) + (0.5 * service_side_rail_w),
                height - (0.5 * shell_t),
            ),
        ),
        material=white_enamel,
        name="top_left_rail",
    )
    cabinet.visual(
        Box((service_panel_depth, service_side_rail_w, shell_t)),
        origin=Origin(
            xyz=(
                service_panel_center_x,
                (0.5 * width) - (0.5 * service_side_rail_w),
                height - (0.5 * shell_t),
            ),
        ),
        material=white_enamel,
        name="top_right_rail",
    )
    cabinet.visual(
        Box((0.05, width - 0.04, 0.095)),
        origin=Origin(xyz=((0.5 * depth) - 0.005, 0.0, height - 0.075)),
        material=warm_white,
        name="control_console",
    )
    cabinet.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.202,
                tube=0.030,
                radial_segments=18,
                tubular_segments=64,
            ),
            "cabinet_porthole_bezel",
        ),
        origin=Origin(
            xyz=((0.5 * depth) - 0.001, 0.0, door_center_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=white_enamel,
        name="front_porthole_bezel",
    )
    cabinet.visual(
        _tube_shell_mesh("cabinet_porthole_liner", 0.184, 0.158, 0.10),
        origin=Origin(
            xyz=((0.5 * depth) - 0.058, 0.0, door_center_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_gray,
        name="front_porthole_liner",
    )
    cabinet.visual(
        Box((0.022, 0.050, 0.048)),
        origin=Origin(xyz=(-0.331, -0.202, 0.822)),
        material=white_enamel,
        name="service_left_pedestal",
    )
    cabinet.visual(
        Box((0.022, 0.050, 0.048)),
        origin=Origin(xyz=(-0.331, 0.202, 0.822)),
        material=white_enamel,
        name="service_right_pedestal",
    )
    cabinet.visual(
        Box((0.044, 0.050, 0.012)),
        origin=Origin(xyz=(-0.322, -0.202, 0.806)),
        material=white_enamel,
        name="service_left_bracket",
    )
    cabinet.visual(
        Box((0.044, 0.050, 0.012)),
        origin=Origin(xyz=(-0.322, 0.202, 0.806)),
        material=white_enamel,
        name="service_right_bracket",
    )
    cabinet.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(
            xyz=(-0.324, -0.202, 0.850),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="service_left_pin",
    )
    cabinet.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(
            xyz=(-0.324, 0.202, 0.850),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="service_right_pin",
    )
    cabinet.visual(
        Box((0.032, 0.032, 0.052)),
        origin=Origin(xyz=(0.334, -door_outer_radius - 0.008, door_center_z + 0.105)),
        material=white_enamel,
        name="door_upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.032, 0.032, 0.052)),
        origin=Origin(xyz=(0.334, -door_outer_radius - 0.008, door_center_z - 0.105)),
        material=white_enamel,
        name="door_lower_hinge_mount",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(
            xyz=(door_hinge_x - 0.021, -door_outer_radius, door_center_z + 0.105),
        ),
        material=steel,
        name="door_upper_pin",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(
            xyz=(door_hinge_x - 0.021, -door_outer_radius, door_center_z - 0.105),
        ),
        material=steel,
        name="door_lower_pin",
    )
    cabinet.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=Origin(
            xyz=(drum_center[0] - 0.34, drum_center[1], drum_center[2]),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_gray,
        name="rear_bearing_housing",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((depth, width, height)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * height)),
    )

    drum = model.part("drum")
    drum.visual(
        _tube_shell_mesh("drum_shell", drum_radius, drum_radius - 0.016, drum_length),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.204,
                tube=0.018,
                radial_segments=16,
                tubular_segments=48,
            ),
            "drum_front_lip",
        ),
        origin=Origin(
            xyz=((0.5 * drum_length) - 0.015, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="drum_front_lip",
    )
    for index, angle in enumerate((0.0, (2.0 * math.pi) / 3.0, (4.0 * math.pi) / 3.0)):
        drum.visual(
            Box((0.27, 0.036, 0.020)),
            origin=Origin(
                xyz=(0.0, 0.191 * math.cos(angle), 0.191 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=steel,
            name=f"baffle_{index + 1}",
        )
    drum.visual(
        Cylinder(radius=0.215, length=0.012),
        origin=Origin(
            xyz=(-0.155, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="rear_web",
    )
    drum.visual(
        Cylinder(radius=0.074, length=0.090),
        origin=Origin(
            xyz=(-0.185, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_gray,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.030, length=0.140),
        origin=Origin(
            xyz=(-0.240, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_gray,
        name="rear_axle",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_radius, length=drum_length),
        mass=14.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.184,
                tube=0.025,
                radial_segments=16,
                tubular_segments=56,
            ),
            "door_outer_bezel",
        ),
        origin=Origin(
            xyz=(0.012, door_outer_radius, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=white_enamel,
        name="door_outer_bezel",
    )
    door.visual(
        _tube_shell_mesh("door_inner_ring", 0.176, 0.150, 0.032),
        origin=Origin(
            xyz=(0.004, door_outer_radius, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber_black,
        name="door_inner_ring",
    )
    door.visual(
        mesh_from_geometry(
            DomeGeometry(radius=0.160, radial_segments=36, height_segments=16, closed=True),
            "door_glass_dome",
        ),
        origin=Origin(
            xyz=(0.006, door_outer_radius, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Box((0.028, 0.070, 0.032)),
        origin=Origin(xyz=(0.018, 0.036, door_center_z * 0.0 + 0.105)),
        material=warm_white,
        name="door_upper_leaf",
    )
    door.visual(
        Box((0.028, 0.070, 0.032)),
        origin=Origin(xyz=(0.018, 0.036, -0.105)),
        material=warm_white,
        name="door_lower_leaf",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=steel,
        name="door_upper_knuckle",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=steel,
        name="door_lower_knuckle",
    )
    door.visual(
        Box((0.030, 0.054, 0.028)),
        origin=Origin(xyz=(0.036, 0.385, -0.045)),
        material=warm_white,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.060, 0.44, 0.44)),
        mass=4.0,
        origin=Origin(xyz=(0.018, door_outer_radius, 0.0)),
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((service_panel_depth, service_panel_width, service_panel_t)),
        origin=Origin(xyz=(0.5 * service_panel_depth, 0.0, 0.0)),
        material=white_enamel,
        name="service_panel_shell",
    )
    service_panel.visual(
        Box((0.050, 0.160, 0.008)),
        origin=Origin(
            xyz=(service_panel_depth - 0.030, 0.0, -0.002),
        ),
        material=charcoal,
        name="service_pull_recess",
    )
    service_panel.visual(
        Box((0.040, 0.034, 0.016)),
        origin=Origin(xyz=(0.020, -0.202, -0.002)),
        material=warm_white,
        name="service_left_bridge",
    )
    service_panel.visual(
        Box((0.040, 0.034, 0.016)),
        origin=Origin(xyz=(0.020, 0.202, -0.002)),
        material=warm_white,
        name="service_right_bridge",
    )
    service_panel.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(
            xyz=(0.0, -0.202, -0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="service_left_knuckle",
    )
    service_panel.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(
            xyz=(0.0, 0.202, -0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="service_right_knuckle",
    )
    service_panel.inertial = Inertial.from_geometry(
        Box((service_panel_depth, service_panel_width, service_panel_t)),
        mass=1.8,
        origin=Origin(xyz=(0.5 * service_panel_depth, 0.0, 0.0)),
    )

    control_board = model.part("control_board")
    control_board.visual(
        Box((0.160, 0.280, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pcb_green,
        name="pcb",
    )
    control_board.visual(
        Box((0.052, 0.040, 0.018)),
        origin=Origin(xyz=(-0.106, -0.102, -0.005)),
        material=charcoal,
        name="left_standoff_rail",
    )
    control_board.visual(
        Box((0.052, 0.040, 0.018)),
        origin=Origin(xyz=(-0.106, 0.102, -0.005)),
        material=charcoal,
        name="right_standoff_rail",
    )
    control_board.visual(
        Box((0.058, 0.060, 0.028)),
        origin=Origin(xyz=(0.050, 0.0, 0.010)),
        material=dark_gray,
        name="connector_pack",
    )
    control_board.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(
            xyz=(-0.050, -0.082, 0.004),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="left_header",
    )
    control_board.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(
            xyz=(-0.050, 0.082, 0.004),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="right_header",
    )
    control_board.inertial = Inertial.from_geometry(
        Box((0.170, 0.290, 0.040)),
        mass=0.8,
        origin=Origin(),
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=drum_center),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=16.0),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(door_hinge_x, -door_outer_radius, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "cabinet_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_panel,
        origin=Origin(xyz=(service_hinge_x, 0.0, height - (0.5 * service_panel_t))),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "cabinet_to_control_board",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_board,
        origin=Origin(xyz=(-0.170, 0.0, 0.750)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    service_panel = object_model.get_part("service_panel")
    control_board = object_model.get_part("control_board")

    drum_spin = object_model.get_articulation("cabinet_to_drum")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    panel_hinge = object_model.get_articulation("cabinet_to_service_panel")

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

    ctx.check(
        "door carries outer bezel, glass, and two hinge knuckles",
        {
            "door_outer_bezel",
            "door_glass",
            "door_upper_knuckle",
            "door_lower_knuckle",
        }.issubset(_visual_names(door)),
        details=str(sorted(_visual_names(door))),
    )
    ctx.check(
        "service panel carries two hinge knuckles",
        {
            "service_panel_shell",
            "service_left_knuckle",
            "service_right_knuckle",
        }.issubset(_visual_names(service_panel)),
        details=str(sorted(_visual_names(service_panel))),
    )
    ctx.check(
        "drum is continuous on a front to back axle",
        drum_spin.articulation_type == ArticulationType.CONTINUOUS
        and _axis_matches(drum_spin.axis, (1.0, 0.0, 0.0)),
        details=f"type={drum_spin.articulation_type}, axis={drum_spin.axis}",
    )
    ctx.check(
        "door hinge is a left edge revolute axis that opens outward",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and _axis_matches(door_hinge.axis, (0.0, 0.0, -1.0)),
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}",
    )
    ctx.check(
        "service panel is rear hinged",
        panel_hinge.articulation_type == ArticulationType.REVOLUTE
        and _axis_matches(panel_hinge.axis, (0.0, -1.0, 0.0)),
        details=f"type={panel_hinge.articulation_type}, axis={panel_hinge.axis}",
    )

    ctx.expect_contact(
        cabinet,
        door,
        elem_a="door_upper_pin",
        elem_b="door_upper_knuckle",
        name="upper door knuckle sits on its cabinet pin",
    )
    ctx.expect_contact(
        cabinet,
        door,
        elem_a="door_lower_pin",
        elem_b="door_lower_knuckle",
        name="lower door knuckle sits on its cabinet pin",
    )
    ctx.expect_contact(
        cabinet,
        service_panel,
        elem_a="service_left_pin",
        elem_b="service_left_knuckle",
        name="left service hinge pin supports the panel",
    )
    ctx.expect_contact(
        cabinet,
        service_panel,
        elem_a="service_right_pin",
        elem_b="service_right_knuckle",
        name="right service hinge pin supports the panel",
    )
    ctx.expect_contact(
        cabinet,
        drum,
        elem_a="rear_bearing_housing",
        elem_b="rear_axle",
        name="drum axle lands in the rear bearing housing",
    )
    ctx.expect_origin_gap(
        cabinet,
        door,
        axis="y",
        min_gap=0.18,
        max_gap=0.24,
        name="door hinge line sits on the cabinet left edge",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="x",
        min_gap=0.005,
        max_gap=0.030,
        positive_elem="door_outer_bezel",
        negative_elem="front_porthole_bezel",
        name="closed door sits just proud of the cabinet front",
    )
    ctx.expect_within(
        control_board,
        service_panel,
        axes="xy",
        margin=0.030,
        name="control board footprint stays beneath the service panel opening",
    )
    ctx.expect_origin_gap(
        service_panel,
        control_board,
        axis="z",
        min_gap=0.08,
        max_gap=0.14,
        name="service panel is mounted above the control board",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: math.radians(100.0)}):
        opened_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward into front access space",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][0] > closed_door_aabb[1][0] + 0.10,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    closed_panel_aabb = ctx.part_world_aabb(service_panel)
    with ctx.pose({panel_hinge: math.radians(70.0)}):
        opened_panel_aabb = ctx.part_world_aabb(service_panel)
    ctx.check(
        "service panel lifts up on its rear pins",
        closed_panel_aabb is not None
        and opened_panel_aabb is not None
        and opened_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.18,
        details=f"closed={closed_panel_aabb}, opened={opened_panel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
