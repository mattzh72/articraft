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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_stacked_washer")

    white_enamel = model.material("white_enamel", rgba=(0.95, 0.96, 0.97, 1.0))
    shadow_gray = model.material("shadow_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.22, 0.28, 0.33, 0.38))
    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.77, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.62, 0.65, 0.69, 1.0))

    body_width = 0.60
    body_depth = 0.67
    body_height = 1.78
    shell_t = 0.018
    front_y = body_depth * 0.5
    back_y = -front_y
    door_center_z = 0.80
    door_axis_x = -0.255

    body = model.part("body")
    side_height = body_height - 0.04
    body.visual(
        Box((shell_t, body_depth, side_height)),
        origin=Origin(xyz=(-body_width * 0.5 + shell_t * 0.5, 0.0, 0.04 + side_height * 0.5)),
        material=white_enamel,
        name="left_side",
    )
    body.visual(
        Box((shell_t, body_depth, side_height)),
        origin=Origin(xyz=(body_width * 0.5 - shell_t * 0.5, 0.0, 0.04 + side_height * 0.5)),
        material=white_enamel,
        name="right_side",
    )
    body.visual(
        Box((body_width - (2.0 * shell_t), body_depth, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=white_enamel,
        name="base_pan",
    )
    body.visual(
        Box((body_width, body_depth, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, body_height - 0.011)),
        material=white_enamel,
        name="top_cap",
    )
    body.visual(
        Box((body_width - (2.0 * shell_t), shell_t, body_height - 0.04)),
        origin=Origin(
            xyz=(0.0, back_y + shell_t * 0.5, 0.04 + (body_height - 0.04) * 0.5)
        ),
        material=white_enamel,
        name="rear_panel",
    )
    body.visual(
        Box((0.564, 0.024, 0.58)),
        origin=Origin(xyz=(0.0, 0.323, 0.29)),
        material=white_enamel,
        name="lower_front_plinth",
    )
    body.visual(
        Box((0.564, 0.024, 0.13)),
        origin=Origin(xyz=(0.0, 0.323, 1.145)),
        material=white_enamel,
        name="upper_front_rail",
    )
    body.visual(
        Box((0.564, 0.024, 0.244)),
        origin=Origin(xyz=(0.0, 0.323, 1.332)),
        material=white_enamel,
        name="mid_fascia",
    )
    body.visual(
        Box((0.118, 0.024, 0.62)),
        origin=Origin(xyz=(-0.241, 0.323, door_center_z)),
        material=white_enamel,
        name="left_door_stile",
    )
    body.visual(
        Box((0.118, 0.024, 0.62)),
        origin=Origin(xyz=(0.241, 0.323, door_center_z)),
        material=white_enamel,
        name="right_door_stile",
    )
    body.visual(
        Box((0.564, 0.024, 0.10)),
        origin=Origin(xyz=(0.0, 0.323, 1.69)),
        material=white_enamel,
        name="top_lintel",
    )
    body.visual(
        Box((0.286, 0.024, 0.15)),
        origin=Origin(xyz=(0.139, 0.323, 1.565)),
        material=white_enamel,
        name="right_console_panel",
    )
    body.visual(
        Box((0.020, 0.024, 0.15)),
        origin=Origin(xyz=(-0.008, 0.323, 1.565)),
        material=white_enamel,
        name="drawer_divider",
    )
    body.visual(
        Box((0.284, 0.024, 0.036)),
        origin=Origin(xyz=(-0.149, 0.323, 1.472)),
        material=white_enamel,
        name="drawer_bottom_lip",
    )
    body.visual(
        Box((0.281, 0.322, 0.012)),
        origin=Origin(xyz=(-0.155, 0.156, 1.484)),
        material=shadow_gray,
        name="drawer_shelf",
    )
    body.visual(
        _mesh(
            "washer_front_gasket",
            TorusGeometry(
                radius=0.183,
                tube=0.024,
                radial_segments=18,
                tubular_segments=56,
            ).rotate_x(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, 0.305, door_center_z)),
        material=graphite,
        name="front_gasket",
    )
    body.visual(
        Box((0.046, 0.042, 0.092)),
        origin=Origin(xyz=(door_axis_x - 0.023, 0.330, door_center_z + 0.125)),
        material=satin_silver,
        name="upper_hinge_pad",
    )
    body.visual(
        Box((0.046, 0.042, 0.092)),
        origin=Origin(xyz=(door_axis_x - 0.023, 0.330, door_center_z - 0.125)),
        material=satin_silver,
        name="lower_hinge_pad",
    )
    body.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, -0.312, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow_gray,
        name="rear_bearing_pad",
    )
    body.visual(
        Box((0.110, 0.006, 0.040)),
        origin=Origin(xyz=(0.075, 0.338, 1.600)),
        material=graphite,
        name="display_window",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.205, 0.346, 1.565), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="selector_knob",
    )
    for index, x_pos in enumerate((0.045, 0.070, 0.095)):
        body.visual(
            Box((0.014, 0.008, 0.008)),
            origin=Origin(xyz=(x_pos, 0.339, 1.540)),
            material=shadow_gray,
            name=f"status_button_{index}",
        )
    for index, x_pos in enumerate((-0.22, 0.22)):
        for y_pos in (-0.24, 0.24):
            body.visual(
                Box((0.050, 0.050, 0.020)),
                origin=Origin(xyz=(x_pos, y_pos, 0.01)),
                material=shadow_gray,
                name=f"foot_{index}_{'front' if y_pos > 0.0 else 'rear'}",
            )
    body.visual(
        Box((0.540, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, 0.329, 0.045)),
        material=graphite,
        name="toe_kick",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=0.18, length=0.30),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.185, length=0.025),
        origin=Origin(xyz=(0.0, 0.162, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="front_rim",
    )
    drum.visual(
        Cylinder(radius=0.080, length=0.11),
        origin=Origin(xyz=(0.0, -0.202, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow_gray,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.030, length=0.064),
        origin=Origin(xyz=(0.0, -0.254, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow_gray,
        name="axle_shaft",
    )
    drum.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, -0.294, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow_gray,
        name="rear_axle_tip",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.075, 0.240, 0.018)),
            origin=Origin(
                xyz=(0.140 * math.sin(angle), 0.0, 0.140 * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=satin_silver,
            name=f"drum_baffle_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Box((0.38, 0.34, 0.38)),
        mass=14.0,
        origin=Origin(),
    )

    door = model.part("door")
    door.visual(
        _mesh(
            "washer_door_frame",
            TorusGeometry(
                radius=0.205,
                tube=0.035,
                radial_segments=18,
                tubular_segments=56,
            ).rotate_x(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        material=satin_silver,
        name="door_frame",
    )
    door.visual(
        Cylinder(radius=0.176, length=0.020),
        origin=Origin(xyz=(0.255, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoked_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.090, 0.030, 0.320)),
        origin=Origin(xyz=(0.045, 0.010, 0.0)),
        material=white_enamel,
        name="hinge_arm",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=satin_silver,
        name="upper_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=satin_silver,
        name="lower_hinge_barrel",
    )
    door.visual(
        Box((0.034, 0.028, 0.104)),
        origin=Origin(xyz=(0.435, 0.022, 0.0)),
        material=satin_silver,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.50, 0.07, 0.50)),
        mass=4.5,
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
    )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        Box((0.236, 0.322, 0.088)),
        origin=Origin(xyz=(0.0, 0.161, 0.044)),
        material=white_enamel,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.252, 0.018, 0.116)),
        origin=Origin(xyz=(0.0, 0.331, 0.058)),
        material=white_enamel,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.092, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.342, 0.058)),
        material=satin_silver,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.26, 0.34, 0.12)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.17, 0.06)),
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=18.0,
            lower=-6.0,
            upper=6.0,
        ),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_axis_x, 0.365, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "body_to_detergent_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-0.155, -0.005, 1.49)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.25,
            lower=0.0,
            upper=0.145,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("detergent_drawer")

    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_detergent_drawer")

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

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
        "door articulation is left-edge revolute",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_joint.axis) == (0.0, 0.0, 1.0),
        f"type={door_joint.articulation_type}, axis={door_joint.axis}",
    )
    ctx.check(
        "drum articulation spins on front-back axle",
        drum_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(drum_joint.axis) == (0.0, 1.0, 0.0),
        f"type={drum_joint.articulation_type}, axis={drum_joint.axis}",
    )
    ctx.check(
        "drawer articulation is forward prismatic",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(drawer_joint.axis) == (0.0, 1.0, 0.0),
        f"type={drawer_joint.articulation_type}, axis={drawer_joint.axis}",
    )

    ctx.expect_contact(
        drum,
        body,
        elem_a="rear_axle_tip",
        elem_b="rear_bearing_pad",
        name="drum axle seats on rear bearing pad",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="upper_hinge_barrel",
        elem_b="upper_hinge_pad",
        name="upper hinge barrel is mounted to body pad",
    )
    ctx.expect_contact(
        drawer,
        body,
        elem_a="drawer_tray",
        elem_b="drawer_shelf",
        name="drawer tray rides on body shelf",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(door, elem="door_handle")
    rest_drawer_pos = ctx.part_world_position(drawer)
    rest_baffle_aabb = ctx.part_element_world_aabb(drum, elem="drum_baffle_0")
    assert rest_handle_aabb is not None
    assert rest_drawer_pos is not None
    assert rest_baffle_aabb is not None
    rest_handle_center = _aabb_center(rest_handle_aabb)
    rest_baffle_center = _aabb_center(rest_baffle_aabb)

    with ctx.pose({door_joint: math.radians(75.0)}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem="door_handle")
        assert open_handle_aabb is not None
        open_handle_center = _aabb_center(open_handle_aabb)
        ctx.check(
            "door swings outward from left hinge",
            open_handle_center[1] > rest_handle_center[1] + 0.25
            and open_handle_center[0] < rest_handle_center[0] - 0.20,
            f"rest={rest_handle_center}, open={open_handle_center}",
        )
        ctx.expect_contact(
            door,
            body,
            elem_a="upper_hinge_barrel",
            elem_b="upper_hinge_pad",
            name="upper hinge stays seated when door opens",
        )

    with ctx.pose({drawer_joint: 0.145}):
        open_drawer_pos = ctx.part_world_position(drawer)
        assert open_drawer_pos is not None
        ctx.check(
            "drawer slides forward",
            open_drawer_pos[1] > rest_drawer_pos[1] + 0.14,
            f"rest={rest_drawer_pos}, open={open_drawer_pos}",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="drawer_tray",
            elem_b="drawer_shelf",
            name="drawer remains guided when extended",
        )

    with ctx.pose({drum_joint: math.pi / 3.0}):
        spun_baffle_aabb = ctx.part_element_world_aabb(drum, elem="drum_baffle_0")
        assert spun_baffle_aabb is not None
        spun_baffle_center = _aabb_center(spun_baffle_aabb)
        ctx.check(
            "drum baffle moves with axle rotation",
            spun_baffle_center[0] > rest_baffle_center[0] + 0.08
            and spun_baffle_center[2] < rest_baffle_center[2] - 0.05,
            f"rest={rest_baffle_center}, spun={spun_baffle_center}",
        )
        ctx.expect_contact(
            drum,
            body,
            elem_a="rear_axle_tip",
            elem_b="rear_bearing_pad",
            name="drum axle stays seated while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
