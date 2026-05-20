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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


POST_HEIGHT = 1.28
POST_SIZE = 0.11
BOARD_LENGTH = 0.34
BOARD_WIDTH = 0.18
BOARD_THICKNESS = 0.035

BODY_LENGTH = 0.48
BODY_WIDTH = 0.22
BODY_EAVE_Z = 0.105
FLOOR_THICKNESS = 0.010
WALL_THICKNESS = 0.008
ROOF_THICKNESS = 0.008
REAR_CAP_THICKNESS = 0.012

DOOR_WIDTH = 0.214
DOOR_THICKNESS = 0.014
DOOR_FRONT_GAP = 0.0012
HINGE_OFFSET_Y = 0.003

HASP_THICKNESS = 0.004
HASP_LENGTH = 0.042
HASP_HEIGHT = 0.012
HASP_PIVOT_Z = 0.077


def _d_loop(
    x_pos: float,
    *,
    width: float,
    eave_z: float,
    bottom_z: float,
    y_center: float = 0.0,
    segments: int = 24,
) -> list[tuple[float, float, float]]:
    radius = width * 0.5
    loop = [
        (x_pos, y_center - radius, bottom_z),
        (x_pos, y_center + radius, bottom_z),
        (x_pos, y_center + radius, eave_z),
    ]
    for idx in range(1, segments):
        theta = (idx / segments) * math.pi
        loop.append(
            (
                x_pos,
                y_center + radius * math.cos(theta),
                eave_z + radius * math.sin(theta),
            )
        )
    loop.append((x_pos, y_center - radius, eave_z))
    return loop


def _roof_band_loop(
    x_pos: float,
    *,
    width: float,
    eave_z: float,
    thickness: float,
    y_center: float = 0.0,
    segments: int = 24,
) -> list[tuple[float, float, float]]:
    outer_radius = width * 0.5
    inner_radius = outer_radius - thickness
    loop = [(x_pos, y_center + outer_radius, eave_z)]
    for idx in range(1, segments + 1):
        theta = (idx / segments) * math.pi
        loop.append(
            (
                x_pos,
                y_center + outer_radius * math.cos(theta),
                eave_z + outer_radius * math.sin(theta),
            )
        )
    loop.append((x_pos, y_center - inner_radius, eave_z))
    for idx in range(segments - 1, 0, -1):
        theta = (idx / segments) * math.pi
        loop.append(
            (
                x_pos,
                y_center + inner_radius * math.cos(theta),
                eave_z + inner_radius * math.sin(theta),
            )
        )
    loop.append((x_pos, y_center + inner_radius, eave_z))
    return loop


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_post_mounted_mailbox")

    painted_metal = model.material("painted_metal", rgba=(0.18, 0.20, 0.26, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.27, 0.29, 0.31, 1.0))
    galvanized = model.material("galvanized", rgba=(0.68, 0.70, 0.74, 1.0))
    cedar = model.material("cedar", rgba=(0.50, 0.33, 0.20, 1.0))

    post = model.part("post")
    post.visual(
        Box((POST_SIZE, POST_SIZE, POST_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT * 0.5)),
        material=cedar,
        name="elem_post_shaft",
    )
    post.visual(
        Box((BOARD_LENGTH, BOARD_WIDTH, BOARD_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT + BOARD_THICKNESS * 0.5)),
        material=cedar,
        name="elem_mount_board",
    )

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(BODY_LENGTH * 0.5, 0.0, FLOOR_THICKNESS * 0.5)),
        material=painted_metal,
        name="elem_floor",
    )
    body.visual(
        Box((BODY_LENGTH - REAR_CAP_THICKNESS, WALL_THICKNESS, BODY_EAVE_Z - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                (BODY_LENGTH - REAR_CAP_THICKNESS) * 0.5,
                -BODY_WIDTH * 0.5 + WALL_THICKNESS * 0.5,
                FLOOR_THICKNESS + (BODY_EAVE_Z - FLOOR_THICKNESS) * 0.5,
            )
        ),
        material=painted_metal,
        name="elem_left_wall",
    )
    body.visual(
        Box((BODY_LENGTH - REAR_CAP_THICKNESS, WALL_THICKNESS, BODY_EAVE_Z - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                (BODY_LENGTH - REAR_CAP_THICKNESS) * 0.5,
                BODY_WIDTH * 0.5 - WALL_THICKNESS * 0.5,
                FLOOR_THICKNESS + (BODY_EAVE_Z - FLOOR_THICKNESS) * 0.5,
            )
        ),
        material=painted_metal,
        name="elem_right_wall",
    )

    roof_shell = section_loft(
        [
            _roof_band_loop(
                0.0,
                width=BODY_WIDTH,
                eave_z=BODY_EAVE_Z,
                thickness=ROOF_THICKNESS,
            ),
            _roof_band_loop(
                BODY_LENGTH - REAR_CAP_THICKNESS,
                width=BODY_WIDTH,
                eave_z=BODY_EAVE_Z,
                thickness=ROOF_THICKNESS,
            ),
        ]
    )
    body.visual(
        mesh_from_geometry(roof_shell, "mailbox_roof_shell"),
        material=painted_metal,
        name="elem_roof_shell",
    )

    rear_cap = section_loft(
        [
            _d_loop(
                BODY_LENGTH - REAR_CAP_THICKNESS,
                width=BODY_WIDTH,
                eave_z=BODY_EAVE_Z,
                bottom_z=FLOOR_THICKNESS,
            ),
            _d_loop(
                BODY_LENGTH,
                width=BODY_WIDTH,
                eave_z=BODY_EAVE_Z,
                bottom_z=FLOOR_THICKNESS,
            ),
        ]
    )
    body.visual(
        mesh_from_geometry(rear_cap, "mailbox_rear_cap"),
        material=painted_metal,
        name="elem_rear_cap",
    )
    body.visual(
        Box((0.018, 0.018, 0.018)),
        origin=Origin(xyz=(0.009, 0.093, HASP_PIVOT_Z)),
        material=dark_hardware,
        name="elem_keeper",
    )

    door = model.part("door")
    door_panel = section_loft(
        [
            _d_loop(
                -(DOOR_FRONT_GAP + DOOR_THICKNESS),
                width=DOOR_WIDTH,
                eave_z=BODY_EAVE_Z,
                bottom_z=0.0,
                y_center=DOOR_WIDTH * 0.5,
            ),
            _d_loop(
                -DOOR_FRONT_GAP,
                width=DOOR_WIDTH,
                eave_z=BODY_EAVE_Z,
                bottom_z=0.0,
                y_center=DOOR_WIDTH * 0.5,
            ),
        ]
    )
    door.visual(
        mesh_from_geometry(door_panel, "mailbox_door_panel"),
        material=painted_metal,
        name="elem_door_panel",
    )
    door.visual(
        Cylinder(radius=0.0055, length=0.182),
        origin=Origin(xyz=(-0.0053, 0.0, 0.091)),
        material=galvanized,
        name="elem_hinge_knuckle",
    )
    door.visual(
        Box((0.008, 0.024, 0.018)),
        origin=Origin(
            xyz=(
                -0.009,
                DOOR_WIDTH - 0.010,
                HASP_PIVOT_Z,
            )
        ),
        material=dark_hardware,
        name="elem_latch_mount",
    )

    hasp = model.part("hasp")
    hasp.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(),
        material=dark_hardware,
        name="elem_hasp_pivot",
    )
    hasp.visual(
        Box((HASP_THICKNESS, HASP_LENGTH, HASP_HEIGHT)),
        origin=Origin(xyz=(0.0, -HASP_LENGTH * 0.5, 0.0)),
        material=galvanized,
        name="elem_hasp_tab",
    )

    model.articulation(
        "post_to_body",
        ArticulationType.FIXED,
        parent=post,
        child=body,
        origin=Origin(xyz=(-BODY_LENGTH * 0.5, 0.0, POST_HEIGHT + BOARD_THICKNESS)),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -BODY_WIDTH * 0.5 + HINGE_OFFSET_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(130.0),
        ),
    )
    model.articulation(
        "door_to_hasp",
        ArticulationType.REVOLUTE,
        parent=door,
        child=hasp,
        origin=Origin(
            xyz=(
                -(DOOR_FRONT_GAP + DOOR_THICKNESS + HASP_THICKNESS * 0.5 + 0.001),
                DOOR_WIDTH - 0.016,
                HASP_PIVOT_Z,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(120.0),
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

    post = object_model.get_part("post")
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hasp = object_model.get_part("hasp")
    door_hinge = object_model.get_articulation("body_to_door")
    latch_pivot = object_model.get_articulation("door_to_hasp")

    ctx.expect_contact(
        body,
        post,
        elem_a="elem_floor",
        elem_b="elem_mount_board",
        name="mailbox floor contacts mounting board",
    )
    ctx.expect_overlap(
        body,
        post,
        axes="xy",
        elem_a="elem_floor",
        elem_b="elem_mount_board",
        min_overlap=0.12,
        name="mounting board supports the mailbox floor footprint",
    )

    with ctx.pose({door_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="x",
            min_gap=0.0008,
            max_gap=0.0018,
            positive_elem="elem_floor",
            negative_elem="elem_door_panel",
            name="closed door sits just ahead of the body opening",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a="elem_door_panel",
            elem_b="elem_roof_shell",
            min_overlap=0.10,
            name="door projects over the mailbox face",
        )
        ctx.expect_overlap(
            hasp,
            body,
            axes="yz",
            elem_a="elem_hasp_tab",
            elem_b="elem_keeper",
            min_overlap=0.006,
            name="closed hasp lines up with the keeper",
        )

    closed_door = ctx.part_element_world_aabb(door, elem="elem_door_panel")
    with ctx.pose({door_hinge: math.radians(105.0), latch_pivot: 0.0}):
        open_door = ctx.part_element_world_aabb(door, elem="elem_door_panel")
    ctx.check(
        "door opens outward from the front face",
        closed_door is not None
        and open_door is not None
        and open_door[0][0] < closed_door[0][0] - 0.12,
        details=f"closed={closed_door}, open={open_door}",
    )

    closed_hasp = None
    swung_hasp = None
    with ctx.pose({door_hinge: 0.0, latch_pivot: 0.0}):
        closed_hasp = ctx.part_element_world_aabb(hasp, elem="elem_hasp_tab")
    with ctx.pose({door_hinge: 0.0, latch_pivot: math.radians(90.0)}):
        swung_hasp = ctx.part_element_world_aabb(hasp, elem="elem_hasp_tab")
    ctx.check(
        "hasp swings away from the keeper",
        closed_hasp is not None
        and swung_hasp is not None
        and swung_hasp[0][0] < closed_hasp[0][0] - 0.02,
        details=f"closed={closed_hasp}, swung={swung_hasp}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
