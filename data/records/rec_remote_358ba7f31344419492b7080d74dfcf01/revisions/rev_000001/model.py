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


BODY_WIDTH = 0.052
BODY_DEPTH = 0.019
BODY_HEIGHT = 0.170
BOTTOM_BLOCK_HEIGHT = 0.0265
FRONT_SHELL_THICKNESS = 0.0035
REAR_SHELL_THICKNESS = 0.0030
SIDE_WALL_THICKNESS = 0.0035
TOP_COLLAR_HEIGHT = 0.016

EXTENSION_WIDTH = 0.035
EXTENSION_DEPTH = 0.0105
EXTENSION_LENGTH = 0.170
EXTENSION_TRAVEL = 0.075
EXTENSION_JOINT_Z = BODY_HEIGHT - TOP_COLLAR_HEIGHT

BATTERY_DOOR_WIDTH = 0.044
BATTERY_DOOR_HEIGHT = 0.092
BATTERY_DOOR_THICKNESS = 0.0024
BATTERY_DOOR_CENTER_Z = 0.074
BATTERY_DOOR_TRAVEL = 0.006


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_telescoping_remote")

    shell_black = model.material("shell_black", rgba=(0.15, 0.15, 0.16, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.22, 0.24, 1.0))
    soft_button = model.material("soft_button", rgba=(0.32, 0.32, 0.34, 1.0))
    glass = model.material("glass", rgba=(0.08, 0.09, 0.11, 1.0))
    metal = model.material("metal", rgba=(0.62, 0.64, 0.68, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BOTTOM_BLOCK_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_BLOCK_HEIGHT / 2.0)),
        material=shell_black,
        name="bottom_block",
    )
    body.visual(
        Box((BODY_WIDTH, FRONT_SHELL_THICKNESS, BODY_HEIGHT - BOTTOM_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH / 2.0 - FRONT_SHELL_THICKNESS / 2.0,
                BOTTOM_BLOCK_HEIGHT + (BODY_HEIGHT - BOTTOM_BLOCK_HEIGHT) / 2.0,
            )
        ),
        material=shell_black,
        name="front_shell",
    )
    for side_name, x_sign in (("left_wall", -1.0), ("right_wall", 1.0)):
        body.visual(
            Box((SIDE_WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT - BOTTOM_BLOCK_HEIGHT)),
            origin=Origin(
                xyz=(
                    x_sign * (BODY_WIDTH / 2.0 - SIDE_WALL_THICKNESS / 2.0),
                    0.0,
                    BOTTOM_BLOCK_HEIGHT + (BODY_HEIGHT - BOTTOM_BLOCK_HEIGHT) / 2.0,
                )
            ),
            material=shell_black,
            name=side_name,
        )

    body.visual(
        Box((BODY_WIDTH, REAR_SHELL_THICKNESS, 0.026)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_DEPTH / 2.0 - REAR_SHELL_THICKNESS / 2.0),
                0.013,
            )
        ),
        material=charcoal,
        name="rear_bottom_panel",
    )
    body.visual(
        Box((BODY_WIDTH, REAR_SHELL_THICKNESS, 0.038)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_DEPTH / 2.0 - REAR_SHELL_THICKNESS / 2.0),
                0.151,
            )
        ),
        material=charcoal,
        name="rear_top_panel",
    )
    for strip_name, x_sign in (("rear_left_strip", -1.0), ("rear_right_strip", 1.0)):
        body.visual(
            Box((0.004, REAR_SHELL_THICKNESS, 0.106)),
            origin=Origin(
                xyz=(
                    x_sign * (BODY_WIDTH / 2.0 - 0.002),
                    -(BODY_DEPTH / 2.0 - REAR_SHELL_THICKNESS / 2.0),
                    0.079,
                )
            ),
            material=charcoal,
            name=strip_name,
        )

    for rail_name, x_sign in (("top_left_rail", -1.0), ("top_right_rail", 1.0)):
        body.visual(
            Box((0.0085, BODY_DEPTH, TOP_COLLAR_HEIGHT)),
            origin=Origin(
                xyz=(
                    x_sign * (BODY_WIDTH / 2.0 - 0.0085 / 2.0),
                    0.0,
                    BODY_HEIGHT - TOP_COLLAR_HEIGHT / 2.0,
                )
            ),
            material=shell_black,
            name=rail_name,
        )
    body.visual(
        Box((0.035, 0.003, TOP_COLLAR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.008, BODY_HEIGHT - TOP_COLLAR_HEIGHT / 2.0)),
        material=shell_black,
        name="top_front_lip",
    )
    body.visual(
        Box((0.035, 0.003, TOP_COLLAR_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.008, BODY_HEIGHT - TOP_COLLAR_HEIGHT / 2.0)),
        material=shell_black,
        name="top_rear_lip",
    )

    body.visual(
        Box((0.027, 0.0016, 0.022)),
        origin=Origin(xyz=(0.0, 0.0087, 0.145)),
        material=glass,
        name="display_window",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0086, 0.129), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_button,
        name="power_button",
    )
    body.visual(
        Box((0.024, 0.0018, 0.018)),
        origin=Origin(xyz=(0.0, 0.0088, 0.095)),
        material=soft_button,
        name="nav_pad",
    )
    body.visual(
        Box((0.010, 0.0018, 0.034)),
        origin=Origin(xyz=(0.0, 0.0088, 0.095)),
        material=soft_button,
        name="nav_pad_vertical",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.0020),
        origin=Origin(xyz=(-0.010, 0.0087, 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_button,
        name="left_menu_button",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.0020),
        origin=Origin(xyz=(0.010, 0.0087, 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_button,
        name="right_menu_button",
    )
    body.visual(
        Box((0.020, 0.0016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0088, 0.041)),
        material=soft_button,
        name="confirm_bar",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    extension = model.part("extension")
    extension.visual(
        Box((EXTENSION_WIDTH, EXTENSION_DEPTH, EXTENSION_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=metal,
        name="inner_member",
    )
    extension.visual(
        Box((0.038, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=charcoal,
        name="extension_cap",
    )
    extension.visual(
        Box((0.020, 0.0016, 0.012)),
        origin=Origin(xyz=(0.0, 0.0062, 0.066)),
        material=glass,
        name="tip_window",
    )
    extension.inertial = Inertial.from_geometry(
        Box((0.038, 0.014, 0.188)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((BATTERY_DOOR_WIDTH, BATTERY_DOOR_THICKNESS, BATTERY_DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, BATTERY_DOOR_THICKNESS / 2.0, 0.0)),
        material=charcoal,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.014, 0.0016, 0.008)),
        origin=Origin(xyz=(0.0, 0.0020, 0.039)),
        material=shell_black,
        name="upper_latch_rib",
    )
    battery_door.visual(
        Box((0.024, 0.0014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0019, -0.036)),
        material=shell_black,
        name="lower_grip_step",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((BATTERY_DOOR_WIDTH, BATTERY_DOOR_THICKNESS, BATTERY_DOOR_HEIGHT)),
        mass=0.012,
        origin=Origin(xyz=(0.0, BATTERY_DOOR_THICKNESS / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_extension",
        ArticulationType.PRISMATIC,
        parent=body,
        child=extension,
        origin=Origin(xyz=(0.0, 0.0, EXTENSION_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.12,
            lower=0.0,
            upper=EXTENSION_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0, BATTERY_DOOR_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=BATTERY_DOOR_TRAVEL,
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

    body = object_model.get_part("body")
    extension = object_model.get_part("extension")
    battery_door = object_model.get_part("battery_door")
    extension_joint = object_model.get_articulation("body_to_extension")
    battery_joint = object_model.get_articulation("body_to_battery_door")

    ctx.expect_within(
        extension,
        body,
        axes="xy",
        margin=0.001,
        name="extension stays centered in the body cavity",
    )
    ctx.expect_overlap(
        extension,
        body,
        axes="z",
        min_overlap=0.055,
        name="extension keeps retained insertion at full travel envelope",
    )
    ctx.expect_within(
        battery_door,
        body,
        axes="xz",
        margin=0.006,
        name="battery door remains aligned to the rear opening footprint",
    )

    body_aabb = ctx.part_world_aabb(body)
    door_aabb = ctx.part_world_aabb(battery_door)
    ctx.check(
        "battery door sits flush with the rear shell at rest",
        body_aabb is not None
        and door_aabb is not None
        and abs(door_aabb[0][1] - body_aabb[0][1]) <= 0.0006,
        details=f"body_aabb={body_aabb}, door_aabb={door_aabb}",
    )

    extension_rest = ctx.part_world_position(extension)
    with ctx.pose({extension_joint: EXTENSION_TRAVEL}):
        ctx.expect_within(
            extension,
            body,
            axes="xy",
            margin=0.001,
            name="extended section stays centered in the body opening",
        )
        ctx.expect_overlap(
            extension,
            body,
            axes="z",
            min_overlap=0.055,
            name="extended section remains telescopically inserted",
        )
        extension_extended = ctx.part_world_position(extension)

    ctx.check(
        "extension slides upward from the top of the remote",
        extension_rest is not None
        and extension_extended is not None
        and extension_extended[2] > extension_rest[2] + 0.05,
        details=f"rest={extension_rest}, extended={extension_extended}",
    )

    door_rest = ctx.part_world_position(battery_door)
    with ctx.pose({battery_joint: BATTERY_DOOR_TRAVEL}):
        ctx.expect_within(
            battery_door,
            body,
            axes="xz",
            margin=0.006,
            name="opened battery door still tracks the rear opening in xz",
        )
        door_open = ctx.part_world_position(battery_door)

    ctx.check(
        "battery door presses outward from the rear face",
        door_rest is not None
        and door_open is not None
        and door_open[1] < door_rest[1] - 0.004,
        details=f"rest={door_rest}, open={door_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
