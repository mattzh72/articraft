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


CABINET_WIDTH = 0.44
CABINET_DEPTH = 0.34
CABINET_BODY_HEIGHT = 0.52
PLINTH_HEIGHT = 0.06
SIDE_THICKNESS = 0.018
BOTTOM_THICKNESS = 0.018
TOP_THICKNESS = 0.022
BACK_THICKNESS = 0.008
CARCASS_DEPTH = CABINET_DEPTH - BACK_THICKNESS
INNER_WIDTH = CABINET_WIDTH - (2.0 * SIDE_THICKNESS)
OPENING_HEIGHT = CABINET_BODY_HEIGHT - BOTTOM_THICKNESS

TOP_WIDTH = 0.46
TOP_DEPTH = 0.36

DOOR_THICKNESS = 0.020
DOOR_WIDTH = 0.400
DOOR_HEIGHT = OPENING_HEIGHT - 0.004
DOOR_FRAME_WIDTH = 0.055
DOOR_HINGE_AXIS_OFFSET = 0.006
DOOR_LEAF_CENTER_Y = 0.005
DOOR_HINGE_X = -0.208
DOOR_HINGE_Y = (CARCASS_DEPTH / 2.0) + (BACK_THICKNESS / 2.0) + (DOOR_THICKNESS / 2.0) + 0.001
DOOR_HINGE_Z = PLINTH_HEIGHT + BOTTOM_THICKNESS + (OPENING_HEIGHT / 2.0)
DOOR_OPEN_ANGLE = math.radians(105.0)

TRAY_WIDTH = 0.32
TRAY_DEPTH = 0.22
TRAY_FLOOR_THICKNESS = 0.008
TRAY_WALL_THICKNESS = 0.012
TRAY_SIDE_RISE = 0.042
TRAY_FRONT_RISE = 0.026
TRAY_REAR_RISE = 0.034
TRAY_HINGE_Y = (CARCASS_DEPTH / 2.0) + (BACK_THICKNESS / 2.0) - 0.020
TRAY_HINGE_Z = PLINTH_HEIGHT + BOTTOM_THICKNESS + 0.026
TRAY_OPEN_ANGLE = math.radians(78.0)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_cabinet_with_flip_tray")

    warm_oak = model.material("warm_oak", rgba=(0.71, 0.56, 0.40, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.39, 0.27, 0.18, 1.0))
    interior_oak = model.material("interior_oak", rgba=(0.77, 0.66, 0.50, 1.0))
    hardware_black = model.material("hardware_black", rgba=(0.14, 0.14, 0.14, 1.0))

    cabinet = model.part("cabinet_case")
    cabinet.visual(
        Box((0.39, 0.30, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT / 2.0)),
        material=dark_walnut,
        name="plinth_base",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, CARCASS_DEPTH, CABINET_BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH / 2.0) + (SIDE_THICKNESS / 2.0),
                BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + (CABINET_BODY_HEIGHT / 2.0),
            )
        ),
        material=warm_oak,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, CARCASS_DEPTH, CABINET_BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                (CABINET_WIDTH / 2.0) - (SIDE_THICKNESS / 2.0),
                BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + (CABINET_BODY_HEIGHT / 2.0),
            )
        ),
        material=warm_oak,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((INNER_WIDTH, CARCASS_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + (BOTTOM_THICKNESS / 2.0),
            )
        ),
        material=warm_oak,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((INNER_WIDTH, BACK_THICKNESS, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH / 2.0) + (BACK_THICKNESS / 2.0),
                PLINTH_HEIGHT + BOTTOM_THICKNESS + (OPENING_HEIGHT / 2.0),
            )
        ),
        material=warm_oak,
        name="back_panel",
    )
    cabinet.visual(
        Box((TOP_WIDTH, TOP_DEPTH, TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                PLINTH_HEIGHT + CABINET_BODY_HEIGHT + (TOP_THICKNESS / 2.0),
            )
        ),
        material=dark_walnut,
        name="top_slab",
    )
    cabinet.visual(
        Box((0.34, 0.028, 0.026)),
        origin=Origin(
            xyz=(
                0.0,
                TRAY_HINGE_Y + 0.006,
                PLINTH_HEIGHT + BOTTOM_THICKNESS + 0.013,
            )
        ),
        material=warm_oak,
        name="tray_mount_rail",
    )
    cabinet.visual(
        Box((0.010, 0.006, 0.150)),
        origin=Origin(
            xyz=(
                DOOR_HINGE_X,
                DOOR_HINGE_Y - 0.008,
                DOOR_HINGE_Z + 0.155,
            )
        ),
        material=hardware_black,
        name="hinge_leaf_upper",
    )
    cabinet.visual(
        Box((0.010, 0.006, 0.150)),
        origin=Origin(
            xyz=(
                DOOR_HINGE_X,
                DOOR_HINGE_Y - 0.008,
                DOOR_HINGE_Z - 0.155,
            )
        ),
        material=hardware_black,
        name="hinge_leaf_lower",
    )
    cabinet.visual(
        Cylinder(radius=0.005, length=0.150),
        origin=Origin(
            xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z + 0.155),
        ),
        material=hardware_black,
        name="hinge_knuckle_upper",
    )
    cabinet.visual(
        Cylinder(radius=0.005, length=0.150),
        origin=Origin(
            xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z - 0.155),
        ),
        material=hardware_black,
        name="hinge_knuckle_lower",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((TOP_WIDTH, TOP_DEPTH, PLINTH_HEIGHT + CABINET_BODY_HEIGHT + TOP_THICKNESS)),
        mass=13.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (PLINTH_HEIGHT + CABINET_BODY_HEIGHT + TOP_THICKNESS) / 2.0,
            )
        ),
    )

    door = model.part("main_door")
    opening_panel_width = DOOR_WIDTH - (2.0 * DOOR_FRAME_WIDTH)
    opening_panel_height = DOOR_HEIGHT - (2.0 * DOOR_FRAME_WIDTH)
    door.visual(
        Box((DOOR_FRAME_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                DOOR_HINGE_AXIS_OFFSET + (DOOR_FRAME_WIDTH / 2.0),
                DOOR_LEAF_CENTER_Y,
                0.0,
            )
        ),
        material=dark_walnut,
        name="left_stile",
    )
    door.visual(
        Box((DOOR_FRAME_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                DOOR_HINGE_AXIS_OFFSET + DOOR_WIDTH - (DOOR_FRAME_WIDTH / 2.0),
                DOOR_LEAF_CENTER_Y,
                0.0,
            )
        ),
        material=dark_walnut,
        name="right_stile",
    )
    door.visual(
        Box((opening_panel_width, DOOR_THICKNESS, DOOR_FRAME_WIDTH)),
        origin=Origin(
            xyz=(
                DOOR_HINGE_AXIS_OFFSET + DOOR_FRAME_WIDTH + (opening_panel_width / 2.0),
                DOOR_LEAF_CENTER_Y,
                (DOOR_HEIGHT / 2.0) - (DOOR_FRAME_WIDTH / 2.0),
            )
        ),
        material=dark_walnut,
        name="top_rail",
    )
    door.visual(
        Box((opening_panel_width, DOOR_THICKNESS, DOOR_FRAME_WIDTH)),
        origin=Origin(
            xyz=(
                DOOR_HINGE_AXIS_OFFSET + DOOR_FRAME_WIDTH + (opening_panel_width / 2.0),
                DOOR_LEAF_CENTER_Y,
                -(DOOR_HEIGHT / 2.0) + (DOOR_FRAME_WIDTH / 2.0),
            )
        ),
        material=dark_walnut,
        name="bottom_rail",
    )
    door.visual(
        Box((opening_panel_width, 0.010, opening_panel_height)),
        origin=Origin(
            xyz=(
                DOOR_HINGE_AXIS_OFFSET + DOOR_FRAME_WIDTH + (opening_panel_width / 2.0),
                0.001,
                0.0,
            )
        ),
        material=interior_oak,
        name="center_panel",
    )
    door.visual(
        Box((0.010, 0.010, 0.160)),
        origin=Origin(
            xyz=(
                0.005,
                0.0,
                0.0,
            )
        ),
        material=hardware_black,
        name="hinge_leaf_center",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.160),
        origin=Origin(),
        material=hardware_black,
        name="hinge_knuckle_center",
    )
    handle_x = DOOR_HINGE_AXIS_OFFSET + DOOR_WIDTH - 0.034
    door.visual(
        Cylinder(radius=0.0028, length=0.014),
        origin=Origin(
            xyz=(handle_x, 0.022, -0.034),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_black,
        name="handle_post_lower",
    )
    door.visual(
        Cylinder(radius=0.0028, length=0.014),
        origin=Origin(
            xyz=(handle_x, 0.022, 0.034),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_black,
        name="handle_post_upper",
    )
    door.visual(
        Cylinder(radius=0.0035, length=0.090),
        origin=Origin(xyz=(handle_x, 0.0325, 0.0)),
        material=hardware_black,
        name="handle_grip",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH + DOOR_HINGE_AXIS_OFFSET, 0.032, DOOR_HEIGHT)),
        mass=2.7,
        origin=Origin(
            xyz=(
                DOOR_HINGE_AXIS_OFFSET + (DOOR_WIDTH / 2.0),
                0.0,
                0.0,
            )
        ),
    )

    tray = model.part("inner_tray")
    tray.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, TRAY_FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                TRAY_DEPTH / 2.0,
                TRAY_FLOOR_THICKNESS / 2.0,
            )
        ),
        material=interior_oak,
        name="tray_floor",
    )
    tray.visual(
        Box((TRAY_WALL_THICKNESS, TRAY_DEPTH, TRAY_SIDE_RISE)),
        origin=Origin(
            xyz=(
                -(TRAY_WIDTH / 2.0) + (TRAY_WALL_THICKNESS / 2.0),
                TRAY_DEPTH / 2.0,
                TRAY_FLOOR_THICKNESS + (TRAY_SIDE_RISE / 2.0),
            )
        ),
        material=interior_oak,
        name="tray_side_left",
    )
    tray.visual(
        Box((TRAY_WALL_THICKNESS, TRAY_DEPTH, TRAY_SIDE_RISE)),
        origin=Origin(
            xyz=(
                (TRAY_WIDTH / 2.0) - (TRAY_WALL_THICKNESS / 2.0),
                TRAY_DEPTH / 2.0,
                TRAY_FLOOR_THICKNESS + (TRAY_SIDE_RISE / 2.0),
            )
        ),
        material=interior_oak,
        name="tray_side_right",
    )
    tray.visual(
        Box((TRAY_WIDTH, TRAY_WALL_THICKNESS, TRAY_REAR_RISE)),
        origin=Origin(
            xyz=(
                0.0,
                TRAY_WALL_THICKNESS / 2.0,
                TRAY_FLOOR_THICKNESS + (TRAY_REAR_RISE / 2.0),
            )
        ),
        material=dark_walnut,
        name="tray_rear_wall",
    )
    tray.visual(
        Box((TRAY_WIDTH, TRAY_WALL_THICKNESS, TRAY_FRONT_RISE)),
        origin=Origin(
            xyz=(
                0.0,
                TRAY_DEPTH - (TRAY_WALL_THICKNESS / 2.0),
                TRAY_FLOOR_THICKNESS + (TRAY_FRONT_RISE / 2.0),
            )
        ),
        material=dark_walnut,
        name="tray_front_lip",
    )
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_WIDTH, TRAY_DEPTH, TRAY_FLOOR_THICKNESS + TRAY_SIDE_RISE)),
        mass=0.9,
        origin=Origin(
            xyz=(
                0.0,
                TRAY_DEPTH / 2.0,
                (TRAY_FLOOR_THICKNESS + TRAY_SIDE_RISE) / 2.0,
            )
        ),
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.2,
            lower=0.0,
            upper=DOOR_OPEN_ANGLE,
        ),
    )
    model.articulation(
        "cabinet_to_tray",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=tray,
        origin=Origin(
            xyz=(0.0, TRAY_HINGE_Y, TRAY_HINGE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=TRAY_OPEN_ANGLE,
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

    cabinet = object_model.get_part("cabinet_case")
    door = object_model.get_part("main_door")
    tray = object_model.get_part("inner_tray")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    tray_hinge = object_model.get_articulation("cabinet_to_tray")
    handle = door.get_visual("handle_grip")
    tray_lip = tray.get_visual("tray_front_lip")

    ctx.check(
        "door hinge is vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "tray hinge is lower horizontal",
        tuple(tray_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={tray_hinge.axis}",
    )

    with ctx.pose({door_hinge: 0.0, tray_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.35,
            name="closed door spans the cabinet opening",
        )
        ctx.expect_within(
            tray,
            cabinet,
            axes="xz",
            margin=0.02,
            name="closed tray stays inside the cabinet footprint",
        )
        closed_handle_aabb = ctx.part_element_world_aabb(door, elem=handle)
        closed_lip_aabb = ctx.part_element_world_aabb(tray, elem=tray_lip)

    with ctx.pose({door_hinge: DOOR_OPEN_ANGLE}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem=handle)

    with ctx.pose({door_hinge: DOOR_OPEN_ANGLE, tray_hinge: TRAY_OPEN_ANGLE}):
        open_lip_aabb = ctx.part_element_world_aabb(tray, elem=tray_lip)
        ctx.expect_within(
            tray,
            cabinet,
            axes="x",
            margin=0.03,
            name="opened tray remains between the cabinet sides",
        )

    closed_handle_center = _aabb_center(closed_handle_aabb)
    open_handle_center = _aabb_center(open_handle_aabb)
    closed_lip_center = _aabb_center(closed_lip_aabb)
    open_lip_center = _aabb_center(open_lip_aabb)

    ctx.check(
        "door swings outward from the case",
        closed_handle_center is not None
        and open_handle_center is not None
        and open_handle_center[1] > closed_handle_center[1] + 0.10,
        details=f"closed={closed_handle_center}, open={open_handle_center}",
    )
    ctx.check(
        "tray flips forward and downward when opened",
        closed_lip_center is not None
        and open_lip_center is not None
        and open_lip_center[1] > closed_lip_center[1] + 0.10
        and open_lip_center[2] < closed_lip_center[2] - 0.05,
        details=f"closed={closed_lip_center}, open={open_lip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
