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


BODY_WIDTH = 0.490
BODY_DEPTH = 0.375
BODY_HEIGHT = 0.312
FOOT_HEIGHT = 0.012

OPENING_WIDTH = 0.296
OPENING_HEIGHT = 0.194
OPENING_OFFSET_X = -0.040
OPENING_OFFSET_Z = -0.004

DOOR_WIDTH = 0.308
DOOR_HEIGHT = 0.214
DOOR_THICKNESS = 0.022
DOOR_BOTTOM_Z = 0.061
DOOR_X = OPENING_OFFSET_X
HINGE_AXIS_OFFSET_Z = 0.008
HINGE_X_OFFSET = DOOR_WIDTH * 0.5 - 0.014
DOOR_STILE_WIDTH = 0.032
DOOR_STILE_CENTER_X = DOOR_WIDTH * 0.5 - DOOR_STILE_WIDTH * 0.5
HANDLE_MOUNT_X = DOOR_WIDTH * 0.5 - 0.028

CONTROL_PANEL_WIDTH = 0.082
CONTROL_PANEL_HEIGHT = 0.228
CONTROL_PANEL_THICKNESS = 0.010
CONTROL_PANEL_X = 0.196
CONTROL_PANEL_Z = 0.185

KNOB_RADIUS = 0.020
KNOB_FACE_Y = -0.034
KNOB_PANEL_Y = -(BODY_DEPTH * 0.5 + CONTROL_PANEL_THICKNESS)


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _add_knob_geometry(part, knob_material, cap_material) -> None:
    spin_axis = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=spin_axis.rpy),
        material=cap_material,
        name="shaft",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=spin_axis.rpy),
        material=knob_material,
        name="rear_skirt",
    )
    part.visual(
        Cylinder(radius=KNOB_RADIUS, length=0.016),
        origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=spin_axis.rpy),
        material=knob_material,
        name="knob_body",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=spin_axis.rpy),
        material=cap_material,
        name="face_cap",
    )
    part.visual(
        Box((0.0024, 0.0040, 0.010)),
        origin=Origin(xyz=(0.0, -0.036, 0.007)),
        material=cap_material,
        name="indicator",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.045, 0.040, 0.045)),
        mass=0.060,
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_toaster_oven")

    shell_paint = model.material("shell_paint", rgba=(0.93, 0.93, 0.90, 1.0))
    polymer_black = model.material("polymer_black", rgba=(0.12, 0.13, 0.14, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.16, 0.18, 0.20, 0.45))
    satin_metal = model.material("satin_metal", rgba=(0.73, 0.75, 0.77, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.46, 0.48, 0.50, 1.0))
    elastomer = model.material("elastomer", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")

    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + BODY_HEIGHT - 0.009)),
        material=shell_paint,
        name="top_shell",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + 0.010)),
        material=shell_paint,
        name="bottom_shell",
    )
    body.visual(
        Box((0.016, BODY_DEPTH, BODY_HEIGHT - 0.038)),
        origin=Origin(
            xyz=(-BODY_WIDTH * 0.5 + 0.008, 0.0, FOOT_HEIGHT + BODY_HEIGHT * 0.5)
        ),
        material=shell_paint,
        name="left_wall",
    )
    body.visual(
        Box((0.016, BODY_DEPTH, BODY_HEIGHT - 0.038)),
        origin=Origin(
            xyz=(BODY_WIDTH * 0.5 - 0.008, 0.0, FOOT_HEIGHT + BODY_HEIGHT * 0.5)
        ),
        material=shell_paint,
        name="right_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 0.032, 0.010, BODY_HEIGHT - 0.038)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 - 0.005,
                FOOT_HEIGHT + BODY_HEIGHT * 0.5,
            )
        ),
        material=shell_paint,
        name="rear_panel",
    )
    body.visual(
        Box((BODY_WIDTH - 0.032, 0.018, DOOR_BOTTOM_Z - FOOT_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_DEPTH * 0.5 - 0.009),
                (FOOT_HEIGHT + DOOR_BOTTOM_Z) * 0.5,
            )
        ),
        material=shell_paint,
        name="front_lower_rail",
    )
    body.visual(
        Box((BODY_WIDTH - 0.032, 0.018, 0.063)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH * 0.5 - 0.009), 0.2925)),
        material=shell_paint,
        name="front_upper_rail",
    )
    body.visual(
        Box((0.042, 0.018, 0.200)),
        origin=Origin(xyz=(-0.224, -(BODY_DEPTH * 0.5 - 0.009), 0.161)),
        material=shell_paint,
        name="front_left_upright",
    )
    body.visual(
        Box((CONTROL_PANEL_WIDTH, 0.018, 0.248)),
        origin=Origin(xyz=(CONTROL_PANEL_X, -(BODY_DEPTH * 0.5 - 0.009), 0.185)),
        material=shell_paint,
        name="control_panel_support",
    )
    body.visual(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_PANEL_X,
                -(BODY_DEPTH * 0.5 + CONTROL_PANEL_THICKNESS * 0.5),
                CONTROL_PANEL_Z,
            )
        ),
        material=polymer_black,
        name="control_panel",
    )
    body.visual(
        Box((0.024, 0.018, 0.024)),
        origin=Origin(
            xyz=(
                DOOR_X - 0.146,
                -(BODY_DEPTH * 0.5 - 0.009),
                DOOR_BOTTOM_Z + HINGE_AXIS_OFFSET_Z,
            )
        ),
        material=hinge_metal,
        name="left_hinge_support",
    )
    body.visual(
        Box((0.024, 0.018, 0.024)),
        origin=Origin(
            xyz=(
                DOOR_X + 0.146,
                -(BODY_DEPTH * 0.5 - 0.009),
                DOOR_BOTTOM_Z + HINGE_AXIS_OFFSET_Z,
            )
        ),
        material=hinge_metal,
        name="right_hinge_support",
    )
    for foot_name, foot_x, foot_y in (
        ("front_left_foot", -0.175, -0.128),
        ("front_right_foot", 0.175, -0.128),
        ("rear_left_foot", -0.175, 0.128),
        ("rear_right_foot", 0.175, 0.128),
    ):
        body.visual(
            Box((0.050, 0.022, FOOT_HEIGHT)),
            origin=Origin(xyz=(foot_x, foot_y, FOOT_HEIGHT * 0.5)),
            material=elastomer,
            name=foot_name,
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT + FOOT_HEIGHT)),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT + FOOT_HEIGHT) * 0.5)),
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.192)),
        material=shell_paint,
        name="door_top_rail",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.020, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=shell_paint,
        name="door_bottom_rail",
    )
    door.visual(
        Box((0.034, DOOR_THICKNESS, 0.194)),
        origin=Origin(xyz=(-0.158, 0.0, 0.109)),
        material=shell_paint,
        name="left_door_stile",
    )
    door.visual(
        Box((0.034, DOOR_THICKNESS, 0.194)),
        origin=Origin(xyz=(0.158, 0.0, 0.109)),
        material=shell_paint,
        name="right_door_stile",
    )
    door.visual(
        Box((0.316, 0.008, 0.138)),
        origin=Origin(xyz=(0.0, 0.002, 0.108)),
        material=smoked_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.020, 0.018, 0.024)),
        origin=Origin(xyz=(-0.140, -0.017, 0.151)),
        material=polymer_black,
        name="left_handle_mount",
    )
    door.visual(
        Box((0.020, 0.018, 0.024)),
        origin=Origin(xyz=(0.140, -0.017, 0.151)),
        material=polymer_black,
        name="right_handle_mount",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.280),
        origin=Origin(
            xyz=(0.0, -0.034, 0.151),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_metal,
        name="handle_bar",
    )
    door.visual(
        Box((0.018, 0.018, 0.016)),
        origin=Origin(xyz=(-0.146, 0.0, 0.008)),
        material=hinge_metal,
        name="left_hinge_arm",
    )
    door.visual(
        Box((0.018, 0.018, 0.016)),
        origin=Origin(xyz=(0.146, 0.0, 0.008)),
        material=hinge_metal,
        name="right_hinge_arm",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(
            xyz=(-0.146, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_metal,
        name="left_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(
            xyz=(0.146, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_metal,
        name="right_hinge_barrel",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS + 0.030, DOOR_HEIGHT)),
        mass=1.7,
        origin=Origin(xyz=(0.0, -0.010, 0.107)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(
                DOOR_X,
                -(BODY_DEPTH * 0.5 + DOOR_THICKNESS * 0.5),
                DOOR_BOTTOM_Z + HINGE_AXIS_OFFSET_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    knob_specs = (
        ("temperature_knob", 0.257),
        ("function_knob", 0.183),
        ("timer_knob", 0.109),
    )
    for knob_name, knob_z in knob_specs:
        knob = model.part(knob_name)
        _add_knob_geometry(knob, polymer_black, satin_metal)
        model.articulation(
            f"body_to_{knob_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(CONTROL_PANEL_X, KNOB_PANEL_Y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.25,
                velocity=4.0,
                lower=-2.45,
                upper=2.45,
            ),
        )

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

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    temperature_knob = object_model.get_part("temperature_knob")
    function_knob = object_model.get_part("function_knob")
    timer_knob = object_model.get_part("timer_knob")
    door_hinge = object_model.get_articulation("body_to_door")
    temperature_joint = object_model.get_articulation("body_to_temperature_knob")
    function_joint = object_model.get_articulation("body_to_function_knob")
    timer_joint = object_model.get_articulation("body_to_timer_knob")
    front_upper_rail = body.get_visual("front_upper_rail")
    control_panel = body.get_visual("control_panel")
    handle_bar = door.get_visual("handle_bar")

    ctx.expect_contact(door, body, name="door_is_physically_supported")
    ctx.expect_contact(
        temperature_knob,
        body,
        elem_b=control_panel,
        name="temperature_knob_is_shaft_mounted",
    )
    ctx.expect_contact(
        function_knob,
        body,
        elem_b=control_panel,
        name="function_knob_is_shaft_mounted",
    )
    ctx.expect_contact(
        timer_knob,
        body,
        elem_b=control_panel,
        name="timer_knob_is_shaft_mounted",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem=front_upper_rail,
            max_gap=0.001,
            max_penetration=0.0,
            name="door_sits_flush_when_closed",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="xz",
            min_overlap=0.180,
            name="door_covers_the_oven_opening",
        )

    closed_handle_aabb = ctx.part_element_world_aabb(door, elem=handle_bar)
    if closed_handle_aabb is None:
        ctx.fail("door_handle_aabb_available", "Could not evaluate closed door handle bounds.")
    else:
        closed_handle_center = tuple(
            (closed_handle_aabb[0][index] + closed_handle_aabb[1][index]) * 0.5
            for index in range(3)
        )
        with ctx.pose({door_hinge: 1.15}):
            open_handle_aabb = ctx.part_element_world_aabb(door, elem=handle_bar)
            if open_handle_aabb is None:
                ctx.fail("door_open_handle_aabb_available", "Could not evaluate open door handle bounds.")
            else:
                open_handle_center = tuple(
                    (open_handle_aabb[0][index] + open_handle_aabb[1][index]) * 0.5
                    for index in range(3)
                )
                ctx.check(
                    "door_opens_downward_and_outward",
                    open_handle_center[1] < closed_handle_center[1] - 0.060
                    and open_handle_center[2] < closed_handle_center[2] - 0.080,
                    details=(
                        f"closed={closed_handle_center}, "
                        f"open={open_handle_center}"
                    ),
                )
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_pose_stays_clear")

    for name, joint in (
        ("temperature_knob_axis_is_front_to_back", temperature_joint),
        ("function_knob_axis_is_front_to_back", function_joint),
        ("timer_knob_axis_is_front_to_back", timer_joint),
    ):
        ctx.check(name, joint.axis == (0.0, 1.0, 0.0), details=f"axis={joint.axis}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
