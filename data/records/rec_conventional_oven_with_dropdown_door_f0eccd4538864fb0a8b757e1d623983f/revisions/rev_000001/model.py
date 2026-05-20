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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_convection_oven")

    body_metal = model.material("body_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    door_trim = model.material("door_trim", rgba=(0.72, 0.74, 0.77, 1.0))
    glass = model.material("oven_glass", rgba=(0.14, 0.20, 0.24, 0.34))
    dark_plastic = model.material("dark_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.70, 0.71, 0.73, 1.0))
    accent = model.material("accent", rgba=(0.42, 0.45, 0.48, 1.0))

    body_width = 0.54
    body_depth = 0.39
    body_height = 0.32
    shell_thickness = 0.014
    front_skin = 0.012

    door_width = 0.390
    door_height = 0.215
    door_center_y = -0.059
    door_hinge_z = 0.058

    tray_width = 0.388
    tray_length = 0.290

    housing = model.part("housing")
    housing.visual(
        Box((0.360, body_width, shell_thickness)),
        origin=Origin(xyz=(-0.015, 0.0, shell_thickness / 2.0)),
        material=body_metal,
        name="bottom_shell",
    )
    housing.visual(
        Box((0.030, 0.114, shell_thickness)),
        origin=Origin(xyz=(0.180, 0.213, shell_thickness / 2.0)),
        material=body_metal,
        name="control_floor",
    )
    housing.visual(
        Box((body_depth, body_width, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - shell_thickness / 2.0)),
        material=body_metal,
        name="top_shell",
    )
    housing.visual(
        Box((body_depth, shell_thickness, body_height)),
        origin=Origin(
            xyz=(0.0, -body_width / 2.0 + shell_thickness / 2.0, body_height / 2.0)
        ),
        material=body_metal,
        name="left_wall",
    )
    housing.visual(
        Box((body_depth, shell_thickness, body_height)),
        origin=Origin(
            xyz=(0.0, body_width / 2.0 - shell_thickness / 2.0, body_height / 2.0)
        ),
        material=body_metal,
        name="right_wall",
    )
    housing.visual(
        Box((shell_thickness, body_width, body_height)),
        origin=Origin(
            xyz=(-body_depth / 2.0 + shell_thickness / 2.0, 0.0, body_height / 2.0)
        ),
        material=body_metal,
        name="rear_wall",
    )
    housing.visual(
        Box((0.310, body_width - 2.0 * shell_thickness, 0.006)),
        origin=Origin(xyz=(-0.006, 0.0, 0.045)),
        material=body_metal,
        name="chamber_floor",
    )
    housing.visual(
        Box((0.310, 0.006, 0.034)),
        origin=Origin(xyz=(-0.005, -0.251, 0.025)),
        material=accent,
        name="left_tray_guide",
    )
    housing.visual(
        Box((0.310, 0.006, 0.034)),
        origin=Origin(xyz=(-0.005, 0.133, 0.025)),
        material=accent,
        name="right_tray_guide",
    )
    housing.visual(
        Box((front_skin, 0.394, 0.018)),
        origin=Origin(xyz=(0.189, door_center_y, 0.049)),
        material=body_metal,
        name="tray_slot_lintel",
    )
    housing.visual(
        Box((front_skin, 0.394, 0.047)),
        origin=Origin(xyz=(0.189, door_center_y, 0.2965)),
        material=body_metal,
        name="door_header",
    )
    housing.visual(
        Box((front_skin, 0.018, 0.272)),
        origin=Origin(xyz=(0.189, 0.147, 0.180)),
        material=body_metal,
        name="door_mullion",
    )
    housing.visual(
        Box((front_skin, 0.114, 0.044)),
        origin=Origin(xyz=(0.189, 0.213, 0.022)),
        material=body_metal,
        name="control_base",
    )
    housing.visual(
        Box((front_skin, 0.114, 0.272)),
        origin=Origin(xyz=(0.189, 0.213, 0.180)),
        material=body_metal,
        name="control_panel",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(0.204, 0.213, 0.238), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="temperature_knob",
    )
    housing.visual(
        Cylinder(radius=0.023, length=0.022),
        origin=Origin(xyz=(0.204, 0.213, 0.162), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="function_knob",
    )
    housing.visual(
        Box((0.016, 0.032, 0.010)),
        origin=Origin(xyz=(0.201, 0.213, 0.091)),
        material=dark_plastic,
        name="timer_button",
    )
    housing.visual(
        Box((0.016, 0.032, 0.010)),
        origin=Origin(xyz=(0.201, 0.213, 0.066)),
        material=dark_plastic,
        name="light_button",
    )
    housing.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, body_height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    door = model.part("door")
    door.visual(
        Box((0.018, door_width, door_height)),
        origin=Origin(xyz=(0.009, 0.0, door_height / 2.0)),
        material=body_metal,
        name="door_panel",
    )
    door.visual(
        Box((0.008, 0.320, 0.145)),
        origin=Origin(xyz=(0.014, 0.0, 0.117)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Box((0.004, 0.338, 0.014)),
        origin=Origin(xyz=(0.018, 0.0, 0.191)),
        material=door_trim,
        name="glass_trim_top",
    )
    door.visual(
        Box((0.004, 0.338, 0.014)),
        origin=Origin(xyz=(0.018, 0.0, 0.043)),
        material=door_trim,
        name="glass_trim_bottom",
    )
    door.visual(
        Box((0.004, 0.014, 0.162)),
        origin=Origin(xyz=(0.018, -0.169, 0.117)),
        material=door_trim,
        name="glass_trim_left",
    )
    door.visual(
        Box((0.004, 0.014, 0.162)),
        origin=Origin(xyz=(0.018, 0.169, 0.117)),
        material=door_trim,
        name="glass_trim_right",
    )
    door.visual(
        Box((0.014, 0.022, 0.014)),
        origin=Origin(xyz=(0.017, -0.108, 0.156)),
        material=door_trim,
        name="handle_mount_left",
    )
    door.visual(
        Box((0.014, 0.022, 0.014)),
        origin=Origin(xyz=(0.017, 0.108, 0.156)),
        material=door_trim,
        name="handle_mount_right",
    )
    door.visual(
        Box((0.012, 0.270, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, 0.156)),
        material=door_trim,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.040, door_width, door_height)),
        mass=1.8,
        origin=Origin(xyz=(0.020, 0.0, door_height / 2.0)),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((tray_length, tray_width, 0.003)),
        origin=Origin(xyz=(-0.140, 0.0, 0.0015)),
        material=tray_metal,
        name="tray_pan",
    )
    crumb_tray.visual(
        Box((tray_length, 0.004, 0.012)),
        origin=Origin(xyz=(-0.140, -0.192, 0.006)),
        material=tray_metal,
        name="tray_side_left",
    )
    crumb_tray.visual(
        Box((tray_length, 0.004, 0.012)),
        origin=Origin(xyz=(-0.140, 0.192, 0.006)),
        material=tray_metal,
        name="tray_side_right",
    )
    crumb_tray.visual(
        Box((0.004, tray_width, 0.012)),
        origin=Origin(xyz=(-0.286, 0.0, 0.006)),
        material=tray_metal,
        name="tray_back",
    )
    crumb_tray.visual(
        Box((0.012, tray_width, 0.016)),
        origin=Origin(xyz=(0.006, 0.0, 0.008)),
        material=tray_metal,
        name="tray_front_face",
    )
    crumb_tray.visual(
        Box((0.010, 0.120, 0.012)),
        origin=Origin(xyz=(0.016, 0.0, 0.012)),
        material=dark_plastic,
        name="tray_pull",
    )
    crumb_tray.inertial = Inertial.from_geometry(
        Box((0.306, tray_width, 0.018)),
        mass=0.45,
        origin=Origin(xyz=(-0.135, 0.0, 0.009)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.195, door_center_y, door_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )
    model.articulation(
        "crumb_tray_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crumb_tray,
        origin=Origin(xyz=(0.183, door_center_y, 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.160,
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

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    crumb_tray = object_model.get_part("crumb_tray")
    door_hinge = object_model.get_articulation("door_hinge")
    crumb_tray_slide = object_model.get_articulation("crumb_tray_slide")

    door_open = (
        door_hinge.motion_limits.upper
        if door_hinge.motion_limits is not None and door_hinge.motion_limits.upper is not None
        else math.radians(90.0)
    )
    tray_open = (
        crumb_tray_slide.motion_limits.upper
        if crumb_tray_slide.motion_limits is not None
        and crumb_tray_slide.motion_limits.upper is not None
        else 0.16
    )

    closed_door_aabb = None
    closed_tray_pos = None
    with ctx.pose({door_hinge: 0.0, crumb_tray_slide: 0.0}):
        ctx.expect_gap(
            door,
            housing,
            axis="x",
            positive_elem="door_panel",
            negative_elem="door_header",
            max_gap=0.010,
            max_penetration=0.0,
            name="closed door sits flush to the housing front",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="yz",
            min_overlap=0.18,
            name="door covers the front oven opening footprint",
        )
        ctx.expect_within(
            crumb_tray,
            housing,
            axes="yz",
            margin=0.012,
            name="crumb tray stays nested under the front opening",
        )
        ctx.expect_overlap(
            crumb_tray,
            housing,
            axes="x",
            min_overlap=0.14,
            name="crumb tray remains inserted when closed",
        )
        closed_door_aabb = ctx.part_world_aabb(door)
        closed_tray_pos = ctx.part_world_position(crumb_tray)

    with ctx.pose({door_hinge: door_open}):
        open_door_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door drops forward and downward on its bottom hinge",
            closed_door_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.12
            and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10,
            details=f"closed={closed_door_aabb}, open={open_door_aabb}",
        )

    with ctx.pose({crumb_tray_slide: tray_open}):
        extended_tray_pos = ctx.part_world_position(crumb_tray)
        ctx.expect_within(
            crumb_tray,
            housing,
            axes="yz",
            margin=0.012,
            name="extended crumb tray stays aligned in its slot",
        )
        ctx.expect_overlap(
            crumb_tray,
            housing,
            axes="x",
            min_overlap=0.12,
            name="extended crumb tray keeps retained insertion",
        )
        ctx.check(
            "crumb tray slides outward from the oven front",
            closed_tray_pos is not None
            and extended_tray_pos is not None
            and extended_tray_pos[0] > closed_tray_pos[0] + 0.12,
            details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
        )

    with ctx.pose({door_hinge: door_open, crumb_tray_slide: tray_open}):
        ctx.expect_gap(
            door,
            crumb_tray,
            axis="z",
            min_gap=0.008,
            name="open door clears the extended crumb tray",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
