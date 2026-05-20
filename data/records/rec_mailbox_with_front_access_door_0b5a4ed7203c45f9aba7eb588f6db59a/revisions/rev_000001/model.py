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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_DEPTH = 0.53
BODY_WIDTH = 0.22
BODY_BOTTOM_Z = -0.115
BODY_SPRING_Z = 0.005
BODY_WALL = 0.006
BODY_TOTAL_HEIGHT = 0.23
DOOR_THICKNESS = 0.012
DOOR_WIDTH = BODY_WIDTH - 0.008
FLAG_PIVOT_X = 0.095
FLAG_PIVOT_Y = 0.125
FLAG_PIVOT_Z = 0.030


def _mailbox_profile(width: float, bottom_z: float, spring_z: float, *, arc_segments: int = 20):
    half_width = width * 0.5
    radius = half_width
    points = [(-half_width, bottom_z), (half_width, bottom_z), (half_width, spring_z)]
    for index in range(1, arc_segments):
        angle = math.pi * index / arc_segments
        points.append((radius * math.cos(angle), spring_z + radius * math.sin(angle)))
    points.append((-half_width, spring_z))
    return points


def _extruded_face_mesh(
    profile: list[tuple[float, float]],
    thickness: float,
    *,
    name: str,
):
    geom = ExtrudeGeometry(profile, thickness, cap=True, center=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _build_body_shell_mesh():
    outer = _mailbox_profile(BODY_WIDTH, BODY_BOTTOM_Z, BODY_SPRING_Z, arc_segments=28)
    inner = _mailbox_profile(
        BODY_WIDTH - (2.0 * BODY_WALL),
        BODY_BOTTOM_Z + BODY_WALL,
        BODY_SPRING_Z,
        arc_segments=28,
    )
    shell = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        BODY_DEPTH,
        cap=True,
        center=True,
        closed=True,
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, "mailbox_body_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curbside_mailbox")

    painted_metal = model.material("painted_metal", rgba=(0.20, 0.22, 0.25, 1.0))
    red_flag = model.material("red_flag", rgba=(0.80, 0.08, 0.08, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.55, 0.42, 0.28, 1.0))
    latch_black = model.material("latch_black", rgba=(0.10, 0.10, 0.11, 1.0))

    support_post = model.part("support_post")
    support_post.visual(
        Box((0.09, 0.09, 1.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=weathered_wood,
        name="post_column",
    )
    support_post.visual(
        Box((0.42, 0.09, 0.08)),
        origin=Origin(xyz=(0.165, 0.0, 0.94)),
        material=weathered_wood,
        name="cantilever_arm",
    )
    support_post.visual(
        Box((0.30, 0.18, 0.02)),
        origin=Origin(xyz=(0.23, 0.0, 0.99)),
        material=weathered_wood,
        name="mount_platform",
    )
    support_post.visual(
        Box((0.30, 0.05, 0.05)),
        origin=Origin(xyz=(0.12, 0.0, 0.81), rpy=(0.0, math.radians(40.0), 0.0)),
        material=weathered_wood,
        name="diagonal_brace",
    )
    support_post.inertial = Inertial.from_geometry(
        Box((0.42, 0.18, 1.04)),
        mass=12.0,
        origin=Origin(xyz=(0.165, 0.0, 0.52)),
    )

    body = model.part("body")
    body.visual(
        _build_body_shell_mesh(),
        origin=Origin(xyz=(BODY_DEPTH * 0.5, 0.0, 0.0)),
        material=painted_metal,
        name="body_shell",
    )
    rear_face_profile = _mailbox_profile(BODY_WIDTH, BODY_BOTTOM_Z, BODY_SPRING_Z, arc_segments=28)
    body.visual(
        _extruded_face_mesh(rear_face_profile, BODY_WALL, name="mailbox_rear_face"),
        origin=Origin(xyz=(BODY_DEPTH - (BODY_WALL * 0.5), 0.0, 0.0)),
        material=painted_metal,
        name="rear_panel",
    )
    body.visual(
        Box((0.032, 0.014, 0.032)),
        origin=Origin(xyz=(FLAG_PIVOT_X, 0.110, FLAG_PIVOT_Z)),
        material=painted_metal,
        name="flag_bracket",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH, BODY_WIDTH, BODY_TOTAL_HEIGHT)),
        mass=4.0,
        origin=Origin(xyz=(BODY_DEPTH * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "post_to_body",
        ArticulationType.FIXED,
        parent=support_post,
        child=body,
        origin=Origin(xyz=(0.08, 0.0, 1.110)),
    )

    door = model.part("front_door")
    door.visual(
        _extruded_face_mesh(
            _mailbox_profile(DOOR_WIDTH, -0.112, 0.004, arc_segments=28),
            DOOR_THICKNESS,
            name="mailbox_front_door",
        ),
        origin=Origin(xyz=(-(DOOR_THICKNESS * 0.5), DOOR_WIDTH * 0.5, 0.0)),
        material=painted_metal,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.188),
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
        material=painted_metal,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.006, 0.010, 0.024)),
        origin=Origin(xyz=(-0.012, DOOR_WIDTH - 0.026, -0.010)),
        material=latch_black,
        name="door_latch_stem",
    )
    door.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(
            xyz=(-0.016, DOOR_WIDTH - 0.026, -0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=latch_black,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.028, DOOR_WIDTH, 0.225)),
        mass=0.8,
        origin=Origin(xyz=(-0.007, DOOR_WIDTH * 0.5, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -(DOOR_WIDTH * 0.5), 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=2.10,
        ),
    )

    flag = model.part("signal_flag")
    flag.visual(
        Cylinder(radius=0.0075, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_flag,
        name="flag_hub",
    )
    flag.visual(
        Box((0.012, 0.006, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=red_flag,
        name="flag_stem",
    )
    flag.visual(
        Box((0.028, 0.004, 0.115)),
        origin=Origin(xyz=(0.014, 0.0, 0.072)),
        material=red_flag,
        name="flag_plate",
    )
    flag.inertial = Inertial.from_geometry(
        Box((0.04, 0.02, 0.13)),
        mass=0.1,
        origin=Origin(xyz=(0.006, 0.0, 0.060)),
    )

    model.articulation(
        "body_to_flag",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flag,
        origin=Origin(xyz=(FLAG_PIVOT_X, FLAG_PIVOT_Y, FLAG_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=-1.30,
            upper=0.0,
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
    support_post = object_model.get_part("support_post")
    body = object_model.get_part("body")
    door = object_model.get_part("front_door")
    flag = object_model.get_part("signal_flag")
    door_joint = object_model.get_articulation("body_to_door")
    flag_joint = object_model.get_articulation("body_to_flag")

    ctx.expect_gap(
        body,
        support_post,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="body_shell",
        negative_elem="mount_platform",
        name="mailbox body sits on the support platform",
    )
    ctx.expect_overlap(
        body,
        support_post,
        axes="xy",
        min_overlap=0.15,
        elem_a="body_shell",
        elem_b="mount_platform",
        name="body footprint overlaps the support mount",
    )
    ctx.expect_gap(
        body,
        door,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="body_shell",
        negative_elem="door_panel",
        name="closed door sits just in front of the mailbox opening",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="yz",
        min_overlap=0.18,
        elem_a="body_shell",
        elem_b="door_panel",
        name="closed door covers the mailbox opening",
    )

    closed_door = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.75}):
        opened_door = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "front door swings forward on a vertical side hinge",
        closed_door is not None
        and opened_door is not None
        and opened_door[0][0] < closed_door[0][0] - 0.08,
        details=f"closed={closed_door}, opened={opened_door}",
    )

    raised_flag = ctx.part_element_world_aabb(flag, elem="flag_plate")
    body_shell = ctx.part_element_world_aabb(body, elem="body_shell")
    with ctx.pose({flag_joint: -1.20}):
        lowered_flag = ctx.part_element_world_aabb(flag, elem="flag_plate")
    ctx.check(
        "raised flag stands above the mailbox roof",
        raised_flag is not None
        and body_shell is not None
        and raised_flag[1][2] > body_shell[1][2] + 0.015,
        details=f"raised={raised_flag}, body={body_shell}",
    )
    ctx.check(
        "signal flag can pivot down from the raised position",
        raised_flag is not None
        and lowered_flag is not None
        and ((lowered_flag[0][2] + lowered_flag[1][2]) * 0.5)
        < (((raised_flag[0][2] + raised_flag[1][2]) * 0.5) - 0.03)
        and lowered_flag[0][0] < raised_flag[0][0] - 0.03,
        details=f"raised={raised_flag}, lowered={lowered_flag}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
