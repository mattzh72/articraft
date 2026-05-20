from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

HERE = os.path.dirname(__file__) or "/"
os.chdir(HERE)

PLATE_SIZE = 0.400
PLATE_THICKNESS = 0.008
SHADE_WIDTH = 0.340
SHADE_DEPTH = 0.340
SHADE_HEIGHT = 0.088
WALL_THICKNESS = 0.016
BOTTOM_PANEL_THICKNESS = 0.006

FRAME_OUTER_WIDTH = 0.252
FRAME_OUTER_DEPTH = 0.182
FRAME_OPENING_WIDTH = 0.216
FRAME_OPENING_DEPTH = 0.146
FRAME_RAIL = 0.018

INNER_SHADE_WIDTH = SHADE_WIDTH - (2.0 * WALL_THICKNESS)
INNER_SHADE_DEPTH = SHADE_DEPTH - (2.0 * WALL_THICKNESS)
BOTTOM_FRAME_CENTER_Z = PLATE_THICKNESS + SHADE_HEIGHT - (BOTTOM_PANEL_THICKNESS * 0.5)
BOTTOM_APRON_DEPTH = (INNER_SHADE_DEPTH - FRAME_OUTER_DEPTH) * 0.5
BOTTOM_APRON_WIDTH = (INNER_SHADE_WIDTH - FRAME_OUTER_WIDTH) * 0.5

DOOR_WIDTH = 0.210
DOOR_DEPTH = 0.140
DOOR_THICKNESS = 0.006
DOOR_RAIL = 0.018
DOOR_GLASS_THICKNESS = 0.004
DOOR_PULL_WIDTH = 0.032
DOOR_PULL_DEPTH = 0.010
DOOR_PULL_HEIGHT = 0.010

HINGE_RADIUS = 0.004
HINGE_AXIS_Y = -(FRAME_OPENING_DEPTH * 0.5)
HINGE_AXIS_Z = BOTTOM_FRAME_CENTER_Z
DOOR_PANEL_START_Y = HINGE_RADIUS
DOOR_PANEL_Z = 0.0

HINGE_CENTER_LENGTH = 0.086
HINGE_SIDE_LENGTH = 0.052
HINGE_LEFT_X = -((HINGE_CENTER_LENGTH * 0.5) + (HINGE_SIDE_LENGTH * 0.5))
HINGE_RIGHT_X = (HINGE_CENTER_LENGTH * 0.5) + (HINGE_SIDE_LENGTH * 0.5)

REAR_FRAME_GAP = 0.108
REAR_FRAME_SEGMENT = (FRAME_OUTER_WIDTH - REAR_FRAME_GAP) * 0.5
REAR_FRAME_SEGMENT_X = (REAR_FRAME_GAP * 0.5) + (REAR_FRAME_SEGMENT * 0.5)

DOOR_OPEN_ANGLE = 1.25


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_ceiling_light")

    painted_steel = model.material("painted_steel", rgba=(0.94, 0.94, 0.92, 1.0))
    warm_white = model.material("warm_white", rgba=(0.95, 0.94, 0.90, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.71, 0.72, 0.74, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.92, 0.94, 0.95, 0.72))
    dark_trim = model.material("dark_trim", rgba=(0.36, 0.37, 0.39, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((PLATE_SIZE, PLATE_SIZE, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS * 0.5)),
        material=painted_steel,
        name="base_plate",
    )
    housing.visual(
        Box((SHADE_WIDTH, WALL_THICKNESS, SHADE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, (SHADE_DEPTH * 0.5) - (WALL_THICKNESS * 0.5), PLATE_THICKNESS + (SHADE_HEIGHT * 0.5))
        ),
        material=warm_white,
        name="front_wall",
    )
    housing.visual(
        Box((SHADE_WIDTH, WALL_THICKNESS, SHADE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -((SHADE_DEPTH * 0.5) - (WALL_THICKNESS * 0.5)), PLATE_THICKNESS + (SHADE_HEIGHT * 0.5))
        ),
        material=warm_white,
        name="rear_wall",
    )
    housing.visual(
        Box((WALL_THICKNESS, INNER_SHADE_DEPTH, SHADE_HEIGHT)),
        origin=Origin(
            xyz=((SHADE_WIDTH * 0.5) - (WALL_THICKNESS * 0.5), 0.0, PLATE_THICKNESS + (SHADE_HEIGHT * 0.5))
        ),
        material=warm_white,
        name="right_wall",
    )
    housing.visual(
        Box((WALL_THICKNESS, INNER_SHADE_DEPTH, SHADE_HEIGHT)),
        origin=Origin(
            xyz=(-((SHADE_WIDTH * 0.5) - (WALL_THICKNESS * 0.5)), 0.0, PLATE_THICKNESS + (SHADE_HEIGHT * 0.5))
        ),
        material=warm_white,
        name="left_wall",
    )
    housing.visual(
        Box((INNER_SHADE_WIDTH, BOTTOM_APRON_DEPTH, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, ((INNER_SHADE_DEPTH * 0.5) + (FRAME_OUTER_DEPTH * 0.5)) * 0.5, BOTTOM_FRAME_CENTER_Z)
        ),
        material=warm_white,
        name="front_apron",
    )
    housing.visual(
        Box((INNER_SHADE_WIDTH, BOTTOM_APRON_DEPTH, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, -(((INNER_SHADE_DEPTH * 0.5) + (FRAME_OUTER_DEPTH * 0.5)) * 0.5), BOTTOM_FRAME_CENTER_Z)
        ),
        material=warm_white,
        name="rear_apron",
    )
    housing.visual(
        Box((BOTTOM_APRON_WIDTH, FRAME_OUTER_DEPTH, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(((INNER_SHADE_WIDTH * 0.5) + (FRAME_OUTER_WIDTH * 0.5)) * 0.5, 0.0, BOTTOM_FRAME_CENTER_Z)
        ),
        material=warm_white,
        name="right_apron",
    )
    housing.visual(
        Box((BOTTOM_APRON_WIDTH, FRAME_OUTER_DEPTH, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(-(((INNER_SHADE_WIDTH * 0.5) + (FRAME_OUTER_WIDTH * 0.5)) * 0.5), 0.0, BOTTOM_FRAME_CENTER_Z)
        ),
        material=warm_white,
        name="left_apron",
    )
    housing.visual(
        Box((FRAME_OUTER_WIDTH, FRAME_RAIL, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, (FRAME_OUTER_DEPTH * 0.5) - (FRAME_RAIL * 0.5), BOTTOM_FRAME_CENTER_Z)
        ),
        material=painted_steel,
        name="front_frame",
    )
    housing.visual(
        Box((FRAME_RAIL, FRAME_OPENING_DEPTH, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=((FRAME_OUTER_WIDTH * 0.5) - (FRAME_RAIL * 0.5), 0.0, BOTTOM_FRAME_CENTER_Z)
        ),
        material=painted_steel,
        name="right_frame",
    )
    housing.visual(
        Box((FRAME_RAIL, FRAME_OPENING_DEPTH, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(-((FRAME_OUTER_WIDTH * 0.5) - (FRAME_RAIL * 0.5)), 0.0, BOTTOM_FRAME_CENTER_Z)
        ),
        material=painted_steel,
        name="left_frame",
    )
    housing.visual(
        Box((REAR_FRAME_SEGMENT, FRAME_RAIL, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(-REAR_FRAME_SEGMENT_X, -((FRAME_OUTER_DEPTH * 0.5) - (FRAME_RAIL * 0.5)), BOTTOM_FRAME_CENTER_Z)
        ),
        material=painted_steel,
        name="rear_frame_left",
    )
    housing.visual(
        Box((REAR_FRAME_SEGMENT, FRAME_RAIL, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(REAR_FRAME_SEGMENT_X, -((FRAME_OUTER_DEPTH * 0.5) - (FRAME_RAIL * 0.5)), BOTTOM_FRAME_CENTER_Z)
        ),
        material=painted_steel,
        name="rear_frame_right",
    )
    housing.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_SIDE_LENGTH),
        origin=Origin(
            xyz=(HINGE_LEFT_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_metal,
        name="left_hinge_barrel",
    )
    housing.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_SIDE_LENGTH),
        origin=Origin(
            xyz=(HINGE_RIGHT_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_metal,
        name="right_hinge_barrel",
    )
    housing.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=satin_metal,
        name="lamp_socket",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=satin_metal,
        name="bulb_neck",
    )
    housing.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=frosted_glass,
        name="bulb_globe",
    )
    housing.inertial = Inertial.from_geometry(
        Box((PLATE_SIZE, PLATE_SIZE, PLATE_THICKNESS + SHADE_HEIGHT)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, (PLATE_THICKNESS + SHADE_HEIGHT) * 0.5)),
    )

    access_door = model.part("access_door")
    access_door.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_CENTER_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_metal,
        name="door_hinge_barrel",
    )
    access_door.visual(
        Box((DOOR_WIDTH, DOOR_RAIL, DOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, DOOR_PANEL_START_Y + (DOOR_RAIL * 0.5), DOOR_PANEL_Z)),
        material=dark_trim,
        name="rear_rail",
    )
    access_door.visual(
        Box((DOOR_WIDTH, DOOR_RAIL, DOOR_THICKNESS)),
        origin=Origin(
            xyz=(0.0, DOOR_PANEL_START_Y + DOOR_DEPTH - (DOOR_RAIL * 0.5), DOOR_PANEL_Z)
        ),
        material=dark_trim,
        name="front_rail",
    )
    access_door.visual(
        Box((DOOR_RAIL, DOOR_DEPTH - (2.0 * DOOR_RAIL), DOOR_THICKNESS)),
        origin=Origin(
            xyz=((DOOR_WIDTH * 0.5) - (DOOR_RAIL * 0.5), DOOR_PANEL_START_Y + (DOOR_DEPTH * 0.5), DOOR_PANEL_Z)
        ),
        material=dark_trim,
        name="right_rail",
    )
    access_door.visual(
        Box((DOOR_RAIL, DOOR_DEPTH - (2.0 * DOOR_RAIL), DOOR_THICKNESS)),
        origin=Origin(
            xyz=(-((DOOR_WIDTH * 0.5) - (DOOR_RAIL * 0.5)), DOOR_PANEL_START_Y + (DOOR_DEPTH * 0.5), DOOR_PANEL_Z)
        ),
        material=dark_trim,
        name="left_rail",
    )
    access_door.visual(
        Box((DOOR_WIDTH - (2.0 * DOOR_RAIL), DOOR_DEPTH - (2.0 * DOOR_RAIL), DOOR_GLASS_THICKNESS)),
        origin=Origin(
            xyz=(0.0, DOOR_PANEL_START_Y + (DOOR_DEPTH * 0.5), DOOR_PANEL_Z)
        ),
        material=frosted_glass,
        name="door_glass",
    )
    access_door.visual(
        Box((DOOR_PULL_WIDTH, DOOR_PULL_DEPTH, DOOR_PULL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, DOOR_PANEL_START_Y + DOOR_DEPTH - 0.015, DOOR_PANEL_Z + 0.007)
        ),
        material=satin_metal,
        name="door_pull",
    )
    access_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_PANEL_START_Y + DOOR_DEPTH, 0.016)),
        mass=0.55,
        origin=Origin(xyz=(0.0, (DOOR_PANEL_START_Y + DOOR_DEPTH) * 0.5, 0.0)),
    )

    model.articulation(
        "shade_access_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=access_door,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=DOOR_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    os.chdir(HERE)
    ctx = TestContext(object_model, asset_root=HERE)
    housing = object_model.get_part("housing")
    access_door = object_model.get_part("access_door")
    door_hinge = object_model.get_articulation("shade_access_hinge")

    base_plate = housing.get_visual("base_plate")
    front_frame = housing.get_visual("front_frame")
    left_frame = housing.get_visual("left_frame")
    right_frame = housing.get_visual("right_frame")
    left_hinge_barrel = housing.get_visual("left_hinge_barrel")
    right_hinge_barrel = housing.get_visual("right_hinge_barrel")
    bulb_globe = housing.get_visual("bulb_globe")

    door_hinge_barrel = access_door.get_visual("door_hinge_barrel")
    door_glass = access_door.get_visual("door_glass")
    door_pull = access_door.get_visual("door_pull")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    limits = door_hinge.motion_limits
    closed_angle = limits.lower if limits is not None and limits.lower is not None else 0.0
    open_angle = limits.upper if limits is not None and limits.upper is not None else DOOR_OPEN_ANGLE

    with ctx.pose({door_hinge: closed_angle}):
        ctx.expect_overlap(
            access_door,
            housing,
            axes="xy",
            min_overlap=0.100,
            elem_a=door_glass,
            elem_b=base_plate,
            name="closed_door_reads_centered_under_plate",
        )
        ctx.expect_contact(
            access_door,
            housing,
            elem_a=door_hinge_barrel,
            elem_b=left_hinge_barrel,
            name="closed_hinge_contacts_left_barrel",
        )
        ctx.expect_contact(
            access_door,
            housing,
            elem_a=door_hinge_barrel,
            elem_b=right_hinge_barrel,
            name="closed_hinge_contacts_right_barrel",
        )
        ctx.expect_gap(
            access_door,
            housing,
            axis="z",
            min_gap=0.008,
            max_gap=0.016,
            positive_elem=door_glass,
            negative_elem=bulb_globe,
            name="closed_door_clears_bulb",
        )
        ctx.expect_gap(
            access_door,
            housing,
            axis="x",
            min_gap=0.018,
            max_gap=0.024,
            positive_elem=door_glass,
            negative_elem=left_frame,
            name="closed_left_margin",
        )
        ctx.expect_gap(
            housing,
            access_door,
            axis="x",
            min_gap=0.018,
            max_gap=0.024,
            positive_elem=right_frame,
            negative_elem=door_glass,
            name="closed_right_margin",
        )
        ctx.expect_gap(
            housing,
            access_door,
            axis="y",
            min_gap=0.018,
            max_gap=0.024,
            positive_elem=front_frame,
            negative_elem=door_glass,
            name="closed_front_margin",
        )

    with ctx.pose({door_hinge: open_angle}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="open_pose_no_floating")
        ctx.expect_contact(
            access_door,
            housing,
            elem_a=door_hinge_barrel,
            elem_b=left_hinge_barrel,
            name="open_hinge_contacts_left_barrel",
        )
        ctx.expect_contact(
            access_door,
            housing,
            elem_a=door_hinge_barrel,
            elem_b=right_hinge_barrel,
            name="open_hinge_contacts_right_barrel",
        )
        ctx.expect_gap(
            access_door,
            housing,
            axis="z",
            min_gap=0.110,
            positive_elem=door_pull,
            negative_elem=bulb_globe,
            name="open_door_allows_bulb_access",
        )
        ctx.expect_gap(
            access_door,
            housing,
            axis="z",
            min_gap=0.110,
            positive_elem=door_pull,
            negative_elem=front_frame,
            name="open_door_hangs_below_shade",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
