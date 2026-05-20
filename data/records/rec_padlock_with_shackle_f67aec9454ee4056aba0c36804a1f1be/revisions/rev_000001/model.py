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
    tube_from_spline_points,
)


BODY_W = 0.048
BODY_T = 0.018
BODY_H = 0.052
SHACKLE_RADIUS = 0.0036
SHACKLE_SPAN = 0.024
SHACKLE_RISE = 0.033
KEYWAY_Z = 0.0175


def _rounded_body_mesh():
    profile = rounded_rect_profile(BODY_W, BODY_H, radius=0.0055, corner_segments=8)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, BODY_T, center=True).rotate_x(math.pi / 2.0),
        "padlock_body_shell",
    )


def _shackle_mesh():
    shackle_path = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.024),
        (0.005, 0.0, 0.030),
        (0.012, 0.0, SHACKLE_RISE),
        (0.019, 0.0, 0.030),
        (SHACKLE_SPAN, 0.0, 0.024),
        (SHACKLE_SPAN, 0.0, 0.0),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            shackle_path,
            radius=SHACKLE_RADIUS,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        "padlock_shackle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_padlock")

    brass = model.material("brass", rgba=(0.77, 0.63, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.79, 0.82, 0.86, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.44, 0.46, 0.50, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_body_mesh(),
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
        material=brass,
        name="body_shell",
    )
    body.visual(
        Box((0.043, 0.0010, 0.046)),
        origin=Origin(xyz=(0.0, BODY_T * 0.5 + 0.0005, BODY_H * 0.46)),
        material=steel_dark,
        name="front_face_plate",
    )
    body.visual(
        Box((0.043, 0.0010, 0.046)),
        origin=Origin(xyz=(0.0, -BODY_T * 0.5 - 0.0005, BODY_H * 0.46)),
        material=steel_dark,
        name="rear_face_plate",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.0016),
        origin=Origin(xyz=(-0.012, 0.0, BODY_H - 0.0008)),
        material=graphite,
        name="retained_socket_trim",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.0016),
        origin=Origin(xyz=(0.012, 0.0, BODY_H - 0.0008)),
        material=graphite,
        name="free_socket_trim",
    )
    body.visual(
        Cylinder(radius=0.0090, length=0.0040),
        origin=Origin(
            xyz=(0.0, BODY_T * 0.5 + 0.0020, KEYWAY_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="cylinder_face",
    )
    body.visual(
        Cylinder(radius=0.0058, length=0.0044),
        origin=Origin(
            xyz=(0.0, BODY_T * 0.5 + 0.0022, KEYWAY_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="cylinder_core",
    )
    body.visual(
        Box((0.038, 0.0012, 0.0125)),
        origin=Origin(xyz=(0.0, BODY_T * 0.5 + 0.0019, KEYWAY_Z)),
        material=steel_dark,
        name="dust_track",
    )
    body.visual(
        Box((0.0022, 0.0018, 0.0125)),
        origin=Origin(xyz=(-0.0172, BODY_T * 0.5 + 0.0022, KEYWAY_Z)),
        material=steel_dark,
        name="dust_track_left_stop",
    )
    body.visual(
        Box((0.0022, 0.0018, 0.0125)),
        origin=Origin(xyz=(0.0172, BODY_T * 0.5 + 0.0022, KEYWAY_Z)),
        material=steel_dark,
        name="dust_track_right_stop",
    )
    body.visual(
        Box((0.0046, 0.0004, 0.0088)),
        origin=Origin(xyz=(0.0, BODY_T * 0.5 + 0.0016, KEYWAY_Z - 0.0001)),
        material=graphite,
        name="keyway_slot",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_T, BODY_H)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    shackle = model.part("shackle")
    shackle.visual(_shackle_mesh(), material=steel, name="shackle_bar")
    shackle.inertial = Inertial.from_geometry(
        Box((0.032, 0.008, 0.040)),
        mass=0.12,
        origin=Origin(xyz=(0.012, 0.0, 0.018)),
    )

    dust_door = model.part("dust_door")
    dust_door.visual(
        Box((0.0145, 0.0010, 0.0105)),
        material=brass,
        name="dust_cover",
    )
    dust_door.visual(
        Box((0.0030, 0.0016, 0.0105)),
        origin=Origin(xyz=(0.0056, 0.0, 0.0)),
        material=brass,
        name="dust_grip",
    )
    dust_door.inertial = Inertial.from_geometry(
        Box((0.017, 0.003, 0.012)),
        mass=0.01,
        origin=Origin(xyz=(0.0015, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(-0.012, 0.0, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "body_to_dust_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=dust_door,
        origin=Origin(xyz=(0.0, BODY_T * 0.5 + 0.0032, KEYWAY_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.06,
            lower=0.0,
            upper=0.0100,
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
    shackle = object_model.get_part("shackle")
    dust_door = object_model.get_part("dust_door")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    dust_joint = object_model.get_articulation("body_to_dust_door")
    shackle_open = (
        shackle_joint.motion_limits.upper
        if shackle_joint.motion_limits is not None and shackle_joint.motion_limits.upper is not None
        else math.radians(85.0)
    )
    dust_open = (
        dust_joint.motion_limits.upper
        if dust_joint.motion_limits is not None and dust_joint.motion_limits.upper is not None
        else 0.0100
    )

    def _interval_overlap(a_min: float, a_max: float, b_min: float, b_max: float) -> float:
        return max(0.0, min(a_max, b_max) - max(a_min, b_min))

    ctx.check(
        "padlock parts exist",
        all(part is not None for part in (body, shackle, dust_door)),
        details="Expected body, shackle, and dust door parts.",
    )

    with ctx.pose({shackle_joint: 0.0, dust_joint: 0.0}):
        ctx.expect_gap(
            shackle,
            body,
            axis="z",
            min_gap=-0.0001,
            max_gap=0.001,
            name="closed shackle seats on body top",
        )
        ctx.expect_within(
            dust_door,
            body,
            axes="xz",
            inner_elem="dust_cover",
            outer_elem="dust_track",
            margin=0.0008,
            name="closed dust door stays within the front track",
        )
        ctx.expect_overlap(
            dust_door,
            body,
            axes="xz",
            elem_a="dust_cover",
            elem_b="keyway_slot",
            min_overlap=0.004,
            name="closed dust door covers the keyway",
        )

    closed_shackle_aabb = ctx.part_world_aabb(shackle)
    with ctx.pose({shackle_joint: shackle_open}):
        open_shackle_aabb = ctx.part_world_aabb(shackle)

    ctx.check(
        "shackle swings forward on retained leg",
        closed_shackle_aabb is not None
        and open_shackle_aabb is not None
        and open_shackle_aabb[1][1] > closed_shackle_aabb[1][1] + 0.015,
        details=f"closed={closed_shackle_aabb}, open={open_shackle_aabb}",
    )

    closed_door_pos = ctx.part_world_position(dust_door)
    with ctx.pose({dust_joint: dust_open}):
        open_door_pos = ctx.part_world_position(dust_door)
        ctx.expect_within(
            dust_door,
            body,
            axes="xz",
            inner_elem="dust_cover",
            outer_elem="dust_track",
            margin=0.0008,
            name="open dust door remains captured by the track",
        )
        open_door_aabb = ctx.part_element_world_aabb(dust_door, elem="dust_cover")
        slot_aabb = ctx.part_element_world_aabb(body, elem="keyway_slot")

    uncovered = False
    if open_door_aabb is not None and slot_aabb is not None:
        overlap_x = _interval_overlap(
            open_door_aabb[0][0],
            open_door_aabb[1][0],
            slot_aabb[0][0],
            slot_aabb[1][0],
        )
        overlap_z = _interval_overlap(
            open_door_aabb[0][2],
            open_door_aabb[1][2],
            slot_aabb[0][2],
            slot_aabb[1][2],
        )
        uncovered = overlap_x < 0.001 and overlap_z > 0.006

    ctx.check(
        "dust door slides aside to uncover the keyway",
        closed_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[0] > closed_door_pos[0] + 0.006
        and uncovered,
        details=(
            f"closed_pos={closed_door_pos}, open_pos={open_door_pos}, "
            f"open_cover={open_door_aabb}, slot={slot_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
