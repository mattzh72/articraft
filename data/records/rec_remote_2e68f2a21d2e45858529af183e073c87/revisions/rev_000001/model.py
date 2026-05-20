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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satellite_remote")

    housing = model.material("housing", rgba=(0.18, 0.19, 0.21, 1.0))
    trim = model.material("trim", rgba=(0.28, 0.30, 0.33, 1.0))
    keypad = model.material("keypad", rgba=(0.13, 0.14, 0.15, 1.0))
    accent = model.material("accent", rgba=(0.18, 0.47, 0.78, 1.0))
    power_red = model.material("power_red", rgba=(0.77, 0.14, 0.13, 1.0))
    display_glass = model.material("display_glass", rgba=(0.18, 0.36, 0.44, 0.55))
    guard_smoke = model.material("guard_smoke", rgba=(0.20, 0.25, 0.31, 0.45))
    door_plastic = model.material("door_plastic", rgba=(0.24, 0.25, 0.27, 1.0))

    body = model.part("body")
    body_shell = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.082, 0.212, 0.012),
            0.026,
            cap=True,
            center=True,
            closed=True,
        ),
        "remote_body_shell",
    )
    body.visual(body_shell, material=housing, name="body_shell")
    body.visual(
        Box((0.068, 0.184, 0.0012)),
        origin=Origin(xyz=(0.0, 0.004, 0.0136)),
        material=trim,
        name="front_bezel",
    )
    body.visual(
        Box((0.044, 0.022, 0.0014)),
        origin=Origin(xyz=(0.0, 0.069, 0.0137)),
        material=display_glass,
        name="screen",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.0018),
        origin=Origin(xyz=(0.0, 0.095, 0.0139)),
        material=power_red,
        name="power_button",
    )

    for name, xyz, size in (
        ("menu_button", (-0.019, 0.036, 0.0138), (0.014, 0.009, 0.0016)),
        ("guide_button", (0.019, 0.036, 0.0138), (0.014, 0.009, 0.0016)),
        ("info_button", (-0.019, 0.018, 0.0138), (0.014, 0.009, 0.0016)),
        ("exit_button", (0.019, 0.018, 0.0138), (0.014, 0.009, 0.0016)),
    ):
        body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=keypad,
            name=name,
        )

    dish_button_z = 0.01385
    body.visual(
        Cylinder(radius=0.009, length=0.0017),
        origin=Origin(xyz=(0.0, -0.019, dish_button_z)),
        material=accent,
        name="dish_ok_button",
    )
    for name, xyz in (
        ("dish_up_button", (0.0, -0.003, dish_button_z)),
        ("dish_down_button", (0.0, -0.035, dish_button_z)),
        ("dish_left_button", (-0.015, -0.019, dish_button_z)),
        ("dish_right_button", (0.015, -0.019, dish_button_z)),
    ):
        body.visual(
            Cylinder(radius=0.0065, length=0.0017),
            origin=Origin(xyz=xyz),
            material=keypad,
            name=name,
        )

    for name, xyz in (
        ("left_rear_track", (-0.0315, -0.008, -0.0136)),
        ("right_rear_track", (0.0315, -0.008, -0.0136)),
    ):
        body.visual(
            Box((0.003, 0.114, 0.0012)),
            origin=Origin(xyz=xyz),
            material=trim,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((0.082, 0.212, 0.026)),
        mass=0.24,
    )

    guard = model.part("guard")
    guard.visual(
        Box((0.056, 0.068, 0.0014)),
        origin=Origin(xyz=(0.0, 0.034, 0.0020)),
        material=guard_smoke,
        name="guard_cover",
    )
    guard.visual(
        Box((0.0024, 0.068, 0.0034)),
        origin=Origin(xyz=(-0.0268, 0.034, -0.0004)),
        material=guard_smoke,
        name="guard_left_wall",
    )
    guard.visual(
        Box((0.0024, 0.068, 0.0034)),
        origin=Origin(xyz=(0.0268, 0.034, -0.0004)),
        material=guard_smoke,
        name="guard_right_wall",
    )
    guard.visual(
        Box((0.056, 0.0022, 0.0034)),
        origin=Origin(xyz=(0.0, 0.0669, -0.0004)),
        material=guard_smoke,
        name="guard_top_wall",
    )
    guard.visual(
        Cylinder(radius=0.0022, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="guard_hinge_barrel",
    )
    guard.inertial = Inertial.from_geometry(
        Box((0.056, 0.068, 0.0048)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.034, 0.0005)),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.058, 0.106, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=door_plastic,
        name="battery_door_panel",
    )
    for name, xyz in (
        ("battery_door_grip_left", (-0.013, -0.034, -0.0016)),
        ("battery_door_grip_right", (0.013, -0.034, -0.0016)),
    ):
        battery_door.visual(
            Box((0.010, 0.020, 0.0012)),
            origin=Origin(xyz=xyz),
            material=trim,
            name=name,
        )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.058, 0.106, 0.0032)),
        mass=0.04,
    )

    model.articulation(
        "body_to_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=guard,
        origin=Origin(xyz=(0.0, -0.055, 0.0152)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=2.2,
        ),
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, -0.008, -0.0140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.12,
            lower=0.0,
            upper=0.028,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    guard = object_model.get_part("guard")
    battery_door = object_model.get_part("battery_door")
    guard_hinge = object_model.get_articulation("body_to_guard")
    door_slide = object_model.get_articulation("body_to_battery_door")

    def elem_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    with ctx.pose({guard_hinge: 0.0, door_slide: 0.0}):
        ctx.expect_gap(
            guard,
            body,
            axis="z",
            positive_elem="guard_cover",
            negative_elem="body_shell",
            max_gap=0.006,
            max_penetration=0.0,
            name="closed guard sits just above the front controls",
        )
        ctx.expect_overlap(
            guard,
            body,
            axes="xy",
            elem_a="guard_cover",
            min_overlap=0.05,
            name="closed guard covers a central control cluster",
        )
        ctx.expect_gap(
            body,
            battery_door,
            axis="z",
            positive_elem="body_shell",
            negative_elem="battery_door_panel",
            max_gap=0.0015,
            max_penetration=0.0,
            name="battery door sits flush on the rear shell",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="xy",
            elem_a="battery_door_panel",
            min_overlap=0.05,
            name="battery door spans the rear battery bay",
        )
        closed_guard_center = elem_center(ctx.part_element_world_aabb(guard, elem="guard_cover"))
        closed_door_pos = ctx.part_world_position(battery_door)

    with ctx.pose({guard_hinge: 2.1}):
        ctx.expect_gap(
            guard,
            body,
            axis="z",
            positive_elem="guard_cover",
            negative_elem="body_shell",
            min_gap=0.0005,
            name="opened guard cover no longer clips into the front shell",
        )
        open_guard_center = elem_center(ctx.part_element_world_aabb(guard, elem="guard_cover"))

    ctx.check(
        "guard flips down from its bottom hinge",
        closed_guard_center is not None
        and open_guard_center is not None
        and open_guard_center[2] > closed_guard_center[2] + 0.015
        and open_guard_center[1] < closed_guard_center[1] - 0.030,
        details=f"closed_guard_center={closed_guard_center}, open_guard_center={open_guard_center}",
    )

    with ctx.pose({door_slide: 0.028}):
        ctx.expect_gap(
            body,
            battery_door,
            axis="z",
            positive_elem="body_shell",
            negative_elem="battery_door_panel",
            max_gap=0.0015,
            max_penetration=0.0,
            name="open battery door stays on the rear guide plane",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="y",
            elem_a="battery_door_panel",
            min_overlap=0.10,
            name="battery door remains captured by the rear guide span",
        )
        open_door_pos = ctx.part_world_position(battery_door)

    ctx.check(
        "battery door slides along the short body axis",
        closed_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[0] > closed_door_pos[0] + 0.020
        and abs(open_door_pos[1] - closed_door_pos[1]) < 0.001
        and abs(open_door_pos[2] - closed_door_pos[2]) < 0.001,
        details=f"closed_door_pos={closed_door_pos}, open_door_pos={open_door_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
