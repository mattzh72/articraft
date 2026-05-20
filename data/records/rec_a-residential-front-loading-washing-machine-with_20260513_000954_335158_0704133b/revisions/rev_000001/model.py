from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_loading_washing_machine")

    body_white = model.material("powder_coated_white", rgba=(0.94, 0.95, 0.95, 1.0))
    enamel_shadow = model.material("shadowed_panel_gaps", rgba=(0.055, 0.060, 0.067, 1.0))
    rubber_black = model.material("black_rubber", rgba=(0.020, 0.022, 0.025, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.73, 1.0))
    dark_glass = model.material("smoked_glass", rgba=(0.35, 0.50, 0.62, 0.36))
    satin_chrome = model.material("satin_chrome", rgba=(0.78, 0.79, 0.77, 1.0))
    blue_led = model.material("blue_led_lens", rgba=(0.10, 0.34, 0.82, 1.0))

    body_w = 0.60
    body_d = 0.64
    body_h = 0.86
    panel_t = 0.018
    front_y = body_d / 2.0
    opening_z = 0.405
    opening_radius = 0.215
    door_axis_x = -0.252
    door_axis_y = front_y + 0.004
    drawer_center_x = -0.185
    drawer_center_z = 0.735
    drawer_w = 0.165
    drawer_h = 0.070
    control_z = 0.735

    def circle_profile(radius: float, segments: int = 80) -> list[tuple[float, float]]:
        return [
            (
                radius * math.cos(2.0 * math.pi * index / segments),
                radius * math.sin(2.0 * math.pi * index / segments),
            )
            for index in range(segments)
        ]

    def rect_profile(width: float, height: float) -> list[tuple[float, float]]:
        return [
            (-width / 2.0, -height / 2.0),
            (width / 2.0, -height / 2.0),
            (width / 2.0, height / 2.0),
            (-width / 2.0, height / 2.0),
        ]

    def offset_profile(
        profile: list[tuple[float, float]], dx: float, dz: float
    ) -> list[tuple[float, float]]:
        return [(x + dx, z + dz) for x, z in profile]

    def front_panel_mesh() -> object:
        outer = rect_profile(body_w - 2.0 * panel_t + 0.004, body_h - 2.0 * panel_t + 0.004)
        door_hole = offset_profile(circle_profile(opening_radius), 0.0, opening_z - body_h / 2.0)
        drawer_hole = offset_profile(
            rounded_rect_profile(drawer_w, drawer_h, 0.010, corner_segments=8),
            drawer_center_x,
            drawer_center_z - body_h / 2.0,
        )
        geom = ExtrudeWithHolesGeometry(
            outer,
            [door_hole, drawer_hole],
            panel_t,
            cap=True,
            center=True,
            closed=True,
        )
        geom.rotate_x(math.pi / 2.0)
        return geom

    def gasket_mesh() -> object:
        geom = LatheGeometry.from_shell_profiles(
            [
                (0.174, -0.018),
                (0.206, -0.017),
                (0.230, -0.004),
                (0.239, 0.015),
            ],
            [
                (0.147, -0.010),
                (0.175, -0.010),
                (0.195, -0.001),
                (0.198, 0.010),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        )
        geom.rotate_x(math.pi / 2.0)
        return geom

    def door_bezel_mesh() -> object:
        geom = LatheGeometry.from_shell_profiles(
            [
                (0.168, -0.022),
                (0.222, -0.022),
                (0.259, -0.006),
                (0.266, 0.017),
                (0.244, 0.031),
            ],
            [
                (0.144, -0.012),
                (0.180, -0.012),
                (0.206, 0.001),
                (0.201, 0.015),
                (0.174, 0.021),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        )
        geom.rotate_x(math.pi / 2.0)
        return geom

    def inner_lip_mesh() -> object:
        geom = LatheGeometry.from_shell_profiles(
            [
                (0.132, -0.009),
                (0.165, -0.009),
                (0.185, 0.000),
                (0.179, 0.014),
            ],
            [
                (0.109, -0.004),
                (0.139, -0.004),
                (0.158, 0.002),
                (0.153, 0.010),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        )
        geom.rotate_x(math.pi / 2.0)
        return geom

    body = model.part("body")
    body.visual(
        Box((panel_t, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + panel_t / 2.0, 0.0, body_h / 2.0)),
        material=body_white,
        name="left_side_panel",
    )
    body.visual(
        Box((panel_t, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - panel_t / 2.0, 0.0, body_h / 2.0)),
        material=body_white,
        name="right_side_panel",
    )
    body.visual(
        Box((body_w - 2.0 * panel_t, body_d, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_t / 2.0)),
        material=body_white,
        name="bottom_panel",
    )
    body.visual(
        Box((body_w - 2.0 * panel_t, body_d, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - panel_t / 2.0)),
        material=body_white,
        name="top_panel",
    )
    body.visual(
        Box((body_w - 2.0 * panel_t, panel_t, body_h - 2.0 * panel_t)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + panel_t / 2.0, body_h / 2.0)),
        material=body_white,
        name="back_panel",
    )
    body.visual(
        mesh_from_geometry(front_panel_mesh(), "front_panel_with_openings"),
        origin=Origin(xyz=(0.0, front_y - panel_t / 2.0, body_h / 2.0)),
        material=body_white,
        name="front_panel",
    )
    body.visual(
        mesh_from_geometry(gasket_mesh(), "rubber_door_gasket"),
        origin=Origin(xyz=(0.0, front_y - 0.018, opening_z)),
        material=rubber_black,
        name="door_gasket",
    )
    body.visual(
        Box((0.360, 0.016, 0.092)),
        origin=Origin(xyz=(0.095, front_y + 0.002, control_z)),
        material=enamel_shadow,
        name="control_panel_insert",
    )
    body.visual(
        Box((0.150, 0.125, 0.005)),
        origin=Origin(xyz=(drawer_center_x, front_y - 0.067, drawer_center_z - 0.035)),
        material=enamel_shadow,
        name="drawer_cavity_floor",
    )
    body.visual(
        Box((0.150, 0.125, 0.005)),
        origin=Origin(xyz=(drawer_center_x, front_y - 0.067, drawer_center_z + 0.035)),
        material=enamel_shadow,
        name="drawer_cavity_ceiling",
    )
    body.visual(
        Box((0.006, 0.125, 0.074)),
        origin=Origin(xyz=(drawer_center_x - 0.085, front_y - 0.067, drawer_center_z)),
        material=enamel_shadow,
        name="drawer_cavity_wall_0",
    )
    body.visual(
        Box((0.006, 0.125, 0.074)),
        origin=Origin(xyz=(drawer_center_x + 0.085, front_y - 0.067, drawer_center_z)),
        material=enamel_shadow,
        name="drawer_cavity_wall_1",
    )
    body.visual(
        Box((0.030, 0.032, 0.080)),
        origin=Origin(xyz=(door_axis_x - 0.028, front_y - 0.002, opening_z + 0.150)),
        material=body_white,
        name="upper_hinge_mount",
    )
    body.visual(
        Box((0.030, 0.032, 0.080)),
        origin=Origin(xyz=(door_axis_x - 0.028, front_y - 0.002, opening_z - 0.150)),
        material=body_white,
        name="lower_hinge_mount",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.066),
        origin=Origin(xyz=(door_axis_x, front_y + 0.001, opening_z + 0.150)),
        material=satin_chrome,
        name="upper_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.066),
        origin=Origin(xyz=(door_axis_x, front_y + 0.001, opening_z - 0.150)),
        material=satin_chrome,
        name="lower_hinge_barrel",
    )
    body.visual(
        Box((0.235, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + 0.035, opening_z)),
        material=enamel_shadow,
        name="rear_bearing_crossmember",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(
            xyz=(0.0, -body_d / 2.0 + 0.050, opening_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=enamel_shadow,
        name="rear_bearing_housing",
    )
    for index, x in enumerate((-0.225, 0.225)):
        body.visual(
            Box((0.066, 0.060, 0.024)),
            origin=Origin(xyz=(x, -body_d / 2.0 + 0.070, 0.012)),
            material=enamel_shadow,
            name=f"rear_leveling_foot_{index}",
        )
        body.visual(
            Box((0.066, 0.060, 0.024)),
            origin=Origin(xyz=(x, front_y - 0.070, 0.012)),
            material=enamel_shadow,
            name=f"front_leveling_foot_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=0.210, length=0.300),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="perforated_drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.223, length=0.026),
        origin=Origin(xyz=(0.0, 0.150, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_chrome,
        name="front_drum_rim",
    )
    drum.visual(
        Cylinder(radius=0.085, length=0.034),
        origin=Origin(xyz=(0.0, -0.150, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_chrome,
        name="rear_drum_hub",
    )
    drum.visual(
        Cylinder(radius=0.015, length=0.115),
        origin=Origin(xyz=(0.0, -0.215, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_chrome,
        name="drive_axle",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.030, 0.230, 0.018)),
            origin=Origin(
                xyz=(0.172 * math.cos(angle), 0.0, 0.172 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=satin_chrome,
            name=f"inner_lifter_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.225, length=0.360),
        mass=7.5,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(door_bezel_mesh(), "chrome_door_bezel"),
        origin=Origin(xyz=(0.252, 0.026, 0.0)),
        material=satin_chrome,
        name="door_bezel",
    )
    door.visual(
        mesh_from_geometry(inner_lip_mesh(), "dark_inner_door_lip"),
        origin=Origin(xyz=(0.252, 0.016, 0.0)),
        material=rubber_black,
        name="inner_lip",
    )
    door.visual(
        Cylinder(radius=0.152, length=0.009),
        origin=Origin(xyz=(0.252, 0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_glass,
        name="convex_window",
    )
    door.visual(
        Box((0.210, 0.016, 0.020)),
        origin=Origin(xyz=(0.116, 0.006, 0.150)),
        material=satin_chrome,
        name="upper_hinge_arm",
    )
    door.visual(
        Box((0.210, 0.016, 0.020)),
        origin=Origin(xyz=(0.116, 0.006, -0.150)),
        material=satin_chrome,
        name="lower_hinge_arm",
    )
    door.visual(
        Box((0.036, 0.038, 0.120)),
        origin=Origin(xyz=(0.468, 0.052, 0.0)),
        material=body_white,
        name="vertical_pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.53, 0.090, 0.53)),
        mass=4.4,
        origin=Origin(xyz=(0.260, 0.026, 0.0)),
    )

    soap_drawer = model.part("soap_drawer")
    soap_drawer.visual(
        Box((drawer_w + 0.014, 0.022, drawer_h + 0.006)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=body_white,
        name="drawer_front",
    )
    soap_drawer.visual(
        Box((drawer_w - 0.018, 0.140, 0.005)),
        origin=Origin(xyz=(0.0, -0.074, -0.026)),
        material=body_white,
        name="tray_floor",
    )
    soap_drawer.visual(
        Box((0.005, 0.110, drawer_h - 0.034)),
        origin=Origin(xyz=(-drawer_w / 2.0 + 0.040, -0.100, -0.008)),
        material=body_white,
        name="tray_wall_0",
    )
    soap_drawer.visual(
        Box((0.005, 0.110, drawer_h - 0.034)),
        origin=Origin(xyz=(drawer_w / 2.0 - 0.040, -0.100, -0.008)),
        material=body_white,
        name="tray_wall_1",
    )
    soap_drawer.visual(
        Box((0.060, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.002, -0.020)),
        material=body_white,
        name="front_tray_bridge",
    )
    soap_drawer.visual(
        Box((drawer_w - 0.024, 0.005, drawer_h - 0.018)),
        origin=Origin(xyz=(0.0, -0.144, -0.001)),
        material=body_white,
        name="tray_back_wall",
    )
    soap_drawer.visual(
        Box((0.006, 0.110, drawer_h - 0.026)),
        origin=Origin(xyz=(-0.028, -0.082, -0.005)),
        material=body_white,
        name="detergent_divider_0",
    )
    soap_drawer.visual(
        Box((0.006, 0.110, drawer_h - 0.026)),
        origin=Origin(xyz=(0.030, -0.082, -0.005)),
        material=body_white,
        name="detergent_divider_1",
    )
    soap_drawer.visual(
        Box((0.078, 0.009, 0.014)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=enamel_shadow,
        name="recessed_pull",
    )

    dial = model.part("program_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.072,
                0.028,
                body_style="skirted",
                top_diameter=0.057,
                grip=KnobGrip(style="ribbed", count=26, depth=0.0012, width=0.0016),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "ribbed_program_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_chrome,
        name="dial_cap",
    )

    buttons: list[object] = []
    button_xs = (0.105, 0.155, 0.205)
    for index, x in enumerate(button_xs):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.034, 0.012, 0.020)),
            origin=Origin(),
            material=body_white,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.003, 0.010)),
            origin=Origin(xyz=(0.0, 0.0072, 0.0)),
            material=blue_led if index == 0 else enamel_shadow,
            name="status_lens",
        )
        buttons.append(button)

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, opening_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=18.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_axis_x, door_axis_y, opening_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "body_to_soap_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=soap_drawer,
        origin=Origin(xyz=(drawer_center_x, front_y + 0.006, drawer_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.125),
    )
    model.articulation(
        "body_to_program_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.000, front_y + 0.024, control_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.4, velocity=3.0),
    )
    for index, button in enumerate(buttons):
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_xs[index], front_y + 0.016, control_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.007),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    soap_drawer = object_model.get_part("soap_drawer")
    dial = object_model.get_part("program_dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")

    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_soap_drawer")
    dial_joint = object_model.get_articulation("body_to_program_dial")
    button_joint_0 = object_model.get_articulation("body_to_button_0")
    button_joint_1 = object_model.get_articulation("body_to_button_1")
    button_joint_2 = object_model.get_articulation("body_to_button_2")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        body,
        drum,
        elem_a="rear_bearing_housing",
        elem_b="drive_axle",
        reason="The drive axle is intentionally captured inside the rear bearing housing.",
    )

    ctx.check(
        "drum has continuous spin",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={drum_joint.articulation_type}",
    )
    ctx.check(
        "drum spins about front-back axis",
        tuple(drum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drum_joint.axis}",
    )
    ctx.check(
        "door hinge is vertical",
        tuple(door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "program dial rotates about the panel normal",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )
    for index, button_joint in enumerate((button_joint_0, button_joint_1, button_joint_2)):
        ctx.check(
            f"button {index} presses into the control panel",
            button_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(button_joint.axis) == (0.0, -1.0, 0.0)
            and button_joint.motion_limits.upper > 0.004,
            details=f"type={button_joint.articulation_type}, axis={button_joint.axis}",
        )

    with ctx.pose({door_joint: 0.0, drawer_joint: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_bezel",
            elem_b="front_panel",
            min_overlap=0.40,
            name="porthole door covers the front opening",
        )
        ctx.expect_overlap(
            drum,
            door,
            axes="xz",
            elem_a="perforated_drum_shell",
            elem_b="convex_window",
            min_overlap=0.26,
            name="drum is visible through the porthole window",
        )
        ctx.expect_gap(
            soap_drawer,
            body,
            axis="y",
            positive_elem="drawer_front",
            negative_elem="front_panel",
            max_gap=0.040,
            max_penetration=0.0,
            name="detergent drawer front is seated at the fascia",
        )

    with ctx.pose({door_joint: 0.0}):
        closed_handle = ctx.part_element_world_aabb(door, elem="vertical_pull_handle")
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        open_handle = ctx.part_element_world_aabb(door, elem="vertical_pull_handle")
    ctx.check(
        "door handle swings forward when opened",
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][1] > closed_handle[0][1] + 0.12,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    with ctx.pose({drawer_joint: 0.0}):
        drawer_closed = ctx.part_world_position(soap_drawer)
    with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
        drawer_open = ctx.part_world_position(soap_drawer)
        ctx.expect_gap(
            soap_drawer,
            body,
            axis="y",
            positive_elem="drawer_front",
            negative_elem="front_panel",
            min_gap=0.095,
            name="detergent drawer extends out of the washer",
        )
    ctx.check(
        "detergent drawer translates forward",
        drawer_closed is not None and drawer_open is not None and drawer_open[1] > drawer_closed[1] + 0.11,
        details=f"closed={drawer_closed}, open={drawer_open}",
    )

    with ctx.pose({button_joint_0: 0.0, button_joint_1: 0.0, button_joint_2: 0.0}):
        button_rest_positions = [
            ctx.part_world_position(button_0),
            ctx.part_world_position(button_1),
            ctx.part_world_position(button_2),
        ]
    with ctx.pose(
        {
            button_joint_0: button_joint_0.motion_limits.upper,
            button_joint_1: button_joint_1.motion_limits.upper,
            button_joint_2: button_joint_2.motion_limits.upper,
        }
    ):
        button_pressed_positions = [
            ctx.part_world_position(button_0),
            ctx.part_world_position(button_1),
            ctx.part_world_position(button_2),
        ]
    ctx.check(
        "all three buttons travel inward",
        all(
            rest is not None and pressed is not None and pressed[1] < rest[1] - 0.005
            for rest, pressed in zip(button_rest_positions, button_pressed_positions, strict=True)
        ),
        details=f"rest={button_rest_positions}, pressed={button_pressed_positions}",
    )

    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_a="dial_cap",
        elem_b="control_panel_insert",
        min_overlap=0.004,
        name="rotary dial is mounted on the control panel",
    )

    return ctx.report()


object_model = build_object_model()
