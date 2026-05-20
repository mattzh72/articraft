from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    superellipse_profile,
    tube_from_spline_points,
)


FRAME_WIDTH = 0.142
LENS_SEPARATION = 0.068
FRAME_DEPTH = 0.0022
HINGE_Z = 0.018
NOSE_PIVOT_Z = 0.010


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _signed_area(profile: list[tuple[float, float]]) -> float:
    area = 0.0
    for index, (x0, y0) in enumerate(profile):
        x1, y1 = profile[(index + 1) % len(profile)]
        area += x0 * y1 - x1 * y0
    return 0.5 * area


def _ensure_ccw(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return profile if _signed_area(profile) >= 0.0 else list(reversed(profile))


def _ensure_cw(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return profile if _signed_area(profile) <= 0.0 else list(reversed(profile))


def _scale_profile(profile: list[tuple[float, float]], scale: float) -> list[tuple[float, float]]:
    return [(x * scale, y * scale) for x, y in profile]


def _mirror_profile_x(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    mirrored = [(-x, y) for x, y in profile]
    return _ensure_ccw(mirrored)


def _aviator_right_profile() -> list[tuple[float, float]]:
    control_points = [
        (-0.0215, 0.0165),
        (-0.0110, 0.0265),
        (0.0125, 0.0275),
        (0.0260, 0.0190),
        (0.0290, 0.0010),
        (0.0240, -0.0210),
        (0.0100, -0.0330),
        (-0.0060, -0.0320),
        (-0.0185, -0.0150),
        (-0.0235, 0.0020),
    ]
    return _ensure_ccw(
        sample_catmull_rom_spline_2d(
            control_points,
            samples_per_segment=10,
            closed=True,
        )
    )


def _rim_mesh(profile: list[tuple[float, float]], *, name: str):
    inner = _ensure_cw(_scale_profile(profile, 0.935))
    geom = ExtrudeWithHolesGeometry(
        _ensure_ccw(profile),
        [inner],
        FRAME_DEPTH,
        center=True,
    ).rotate_x(pi / 2.0)
    return _mesh(name, geom)


def _lens_mesh(profile: list[tuple[float, float]], *, name: str):
    geom = ExtrudeGeometry(
        _scale_profile(profile, 0.903),
        0.0013,
        center=True,
    ).rotate_x(pi / 2.0)
    return _mesh(name, geom)


def _nose_pad_mesh(name: str):
    geom = ExtrudeGeometry(
        superellipse_profile(0.013, 0.0078, exponent=2.2, segments=32),
        0.0024,
        center=True,
    ).rotate_x(pi / 2.0)
    return _mesh(name, geom)


def _bridge_bar_mesh(name: str, points: list[tuple[float, float, float]], radius: float):
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        ),
    )


def _temple_arm_mesh(name: str, side_sign: float):
    points = [
        (0.0, -0.0024, 0.0),
        (side_sign * 0.0008, -0.034, -0.0004),
        (side_sign * 0.0030, -0.080, -0.0016),
        (side_sign * 0.0060, -0.128, -0.0038),
        (side_sign * 0.0095, -0.150, -0.0160),
    ]
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=0.00115,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )


def _nose_arm_mesh(name: str, side_sign: float):
    points = [
        (0.0, -0.0030, 0.0),
        (side_sign * 0.0030, -0.0030, -0.0040),
        (side_sign * 0.0062, -0.0030, -0.0105),
    ]
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=0.00072,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        ),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aviator_glasses")

    metal = model.material("gunmetal", rgba=(0.62, 0.64, 0.68, 1.0))
    lens_tint = model.material("lens_tint", rgba=(0.58, 0.68, 0.74, 0.34))
    nose_pad = model.material("nose_pad", rgba=(0.88, 0.90, 0.92, 0.70))

    right_profile = _aviator_right_profile()
    left_profile = _mirror_profile_x(right_profile)

    lens_offset = LENS_SEPARATION * 0.5
    hinge_x = 0.0651

    front_frame = model.part("front_frame")
    front_frame.visual(
        _rim_mesh(left_profile, name="left_rim_mesh"),
        origin=Origin(xyz=(-lens_offset, 0.0, -0.001)),
        material=metal,
        name="left_rim",
    )
    front_frame.visual(
        _rim_mesh(right_profile, name="right_rim_mesh"),
        origin=Origin(xyz=(lens_offset, 0.0, -0.001)),
        material=metal,
        name="right_rim",
    )
    front_frame.visual(
        _bridge_bar_mesh(
            "lower_bridge_mesh",
            [
                (-0.0117, 0.0, 0.0136),
                (0.0, 0.0, 0.0110),
                (0.0117, 0.0, 0.0136),
            ],
            radius=0.00115,
        ),
        material=metal,
        name="lower_bridge",
    )
    front_frame.visual(
        _bridge_bar_mesh(
            "upper_bridge_mesh",
            [
                (-0.0220, 0.0, 0.0240),
                (-0.0080, 0.0, 0.0280),
                (0.0080, 0.0, 0.0280),
                (0.0220, 0.0, 0.0240),
            ],
            radius=0.00095,
        ),
        material=metal,
        name="upper_bridge",
    )
    front_frame.visual(
        Box((0.0042, 0.0026, 0.0085)),
        origin=Origin(xyz=(-hinge_x, 0.0, HINGE_Z)),
        material=metal,
        name="left_hinge_block",
    )
    front_frame.visual(
        Box((0.0042, 0.0026, 0.0085)),
        origin=Origin(xyz=(hinge_x, 0.0, HINGE_Z)),
        material=metal,
        name="right_hinge_block",
    )
    front_frame.visual(
        Box((0.0038, 0.0022, 0.0044)),
        origin=Origin(xyz=(-0.0623, 0.0, 0.0176)),
        material=metal,
        name="left_endpiece",
    )
    front_frame.visual(
        Box((0.0038, 0.0022, 0.0044)),
        origin=Origin(xyz=(0.0623, 0.0, 0.0176)),
        material=metal,
        name="right_endpiece",
    )
    front_frame.visual(
        Cylinder(radius=0.00115, length=0.0036),
        origin=Origin(xyz=(-0.0105, 0.0, NOSE_PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_pad_socket",
    )
    front_frame.visual(
        Cylinder(radius=0.00115, length=0.0036),
        origin=Origin(xyz=(0.0105, 0.0, NOSE_PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_pad_socket",
    )

    left_lens = model.part("left_lens")
    left_lens.visual(
        _lens_mesh(left_profile, name="left_lens_mesh"),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=lens_tint,
        name="left_lens_body",
    )

    right_lens = model.part("right_lens")
    right_lens.visual(
        _lens_mesh(right_profile, name="right_lens_mesh"),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=lens_tint,
        name="right_lens_body",
    )

    left_temple = model.part("left_temple")
    left_temple.visual(
        Box((0.0032, 0.0026, 0.0085)),
        origin=Origin(xyz=(0.0, -0.0026, 0.0)),
        material=metal,
        name="left_temple_hinge_leaf",
    )
    left_temple.visual(
        _temple_arm_mesh("left_temple_arm_mesh", side_sign=1.0),
        material=metal,
        name="left_temple_arm",
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        Box((0.0032, 0.0026, 0.0085)),
        origin=Origin(xyz=(0.0, -0.0026, 0.0)),
        material=metal,
        name="right_temple_hinge_leaf",
    )
    right_temple.visual(
        _temple_arm_mesh("right_temple_arm_mesh", side_sign=-1.0),
        material=metal,
        name="right_temple_arm",
    )

    left_nose_pad = model.part("left_nose_pad")
    left_nose_pad.visual(
        Cylinder(radius=0.00105, length=0.0032),
        origin=Origin(xyz=(0.0, -0.0034, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_pad_pivot",
    )
    left_nose_pad.visual(
        _nose_arm_mesh("left_nose_arm_mesh", side_sign=-1.0),
        material=metal,
        name="left_pad_arm",
    )
    left_nose_pad.visual(
        _nose_pad_mesh("left_nose_pad_mesh"),
        origin=Origin(xyz=(-0.0080, -0.0030, -0.0125), rpy=(0.16, 0.0, -0.36)),
        material=nose_pad,
        name="left_pad_cushion",
    )

    right_nose_pad = model.part("right_nose_pad")
    right_nose_pad.visual(
        Cylinder(radius=0.00105, length=0.0032),
        origin=Origin(xyz=(0.0, -0.0034, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_pad_pivot",
    )
    right_nose_pad.visual(
        _nose_arm_mesh("right_nose_arm_mesh", side_sign=1.0),
        material=metal,
        name="right_pad_arm",
    )
    right_nose_pad.visual(
        _nose_pad_mesh("right_nose_pad_mesh"),
        origin=Origin(xyz=(0.0080, -0.0030, -0.0125), rpy=(0.16, 0.0, 0.36)),
        material=nose_pad,
        name="right_pad_cushion",
    )

    model.articulation(
        "front_to_left_lens",
        ArticulationType.FIXED,
        parent=front_frame,
        child=left_lens,
        origin=Origin(xyz=(-lens_offset, 0.0, 0.0)),
    )
    model.articulation(
        "front_to_right_lens",
        ArticulationType.FIXED,
        parent=front_frame,
        child=right_lens,
        origin=Origin(xyz=(lens_offset, 0.0, 0.0)),
    )
    model.articulation(
        "front_to_left_temple",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(-hinge_x, 0.0, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "front_to_right_temple",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(hinge_x, 0.0, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "front_to_left_nose_pad",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_nose_pad,
        origin=Origin(xyz=(-0.0105, 0.0, NOSE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=2.5, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "front_to_right_nose_pad",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_nose_pad,
        origin=Origin(xyz=(0.0105, 0.0, NOSE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=2.5, lower=-0.35, upper=0.35),
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
    front_frame = object_model.get_part("front_frame")
    left_lens = object_model.get_part("left_lens")
    right_lens = object_model.get_part("right_lens")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_nose_pad = object_model.get_part("left_nose_pad")
    right_nose_pad = object_model.get_part("right_nose_pad")

    left_temple_hinge = object_model.get_articulation("front_to_left_temple")
    right_temple_hinge = object_model.get_articulation("front_to_right_temple")
    left_pad_joint = object_model.get_articulation("front_to_left_nose_pad")
    right_pad_joint = object_model.get_articulation("front_to_right_nose_pad")

    with ctx.pose(
        {
            left_temple_hinge: 0.0,
            right_temple_hinge: 0.0,
            left_pad_joint: 0.0,
            right_pad_joint: 0.0,
        }
    ):
        ctx.expect_contact(
            front_frame,
            left_temple,
            elem_a="left_hinge_block",
            elem_b="left_temple_hinge_leaf",
            contact_tol=0.00025,
            name="left temple leaf seats on hinge block",
        )
        ctx.expect_contact(
            front_frame,
            right_temple,
            elem_a="right_hinge_block",
            elem_b="right_temple_hinge_leaf",
            contact_tol=0.00025,
            name="right temple leaf seats on hinge block",
        )
        ctx.expect_contact(
            front_frame,
            left_nose_pad,
            elem_a="left_pad_socket",
            elem_b="left_pad_pivot",
            contact_tol=0.00025,
            name="left nose pad pivot meets bridge socket",
        )
        ctx.expect_contact(
            front_frame,
            right_nose_pad,
            elem_a="right_pad_socket",
            elem_b="right_pad_pivot",
            contact_tol=0.00025,
            name="right nose pad pivot meets bridge socket",
        )
        ctx.expect_within(
            left_lens,
            front_frame,
            axes="xz",
            inner_elem="left_lens_body",
            outer_elem="left_rim",
            margin=0.003,
            name="left lens stays within left rim footprint",
        )
        ctx.expect_within(
            right_lens,
            front_frame,
            axes="xz",
            inner_elem="right_lens_body",
            outer_elem="right_rim",
            margin=0.003,
            name="right lens stays within right rim footprint",
        )

        frame_center = _aabb_center(ctx.part_world_aabb(front_frame))
        left_open_center = _aabb_center(ctx.part_world_aabb(left_temple))
        right_open_center = _aabb_center(ctx.part_world_aabb(right_temple))
        ctx.check(
            "left temple rests behind front frame",
            frame_center is not None
            and left_open_center is not None
            and left_open_center[1] < frame_center[1] - 0.045,
            details=f"frame_center={frame_center}, left_open_center={left_open_center}",
        )
        ctx.check(
            "right temple rests behind front frame",
            frame_center is not None
            and right_open_center is not None
            and right_open_center[1] < frame_center[1] - 0.045,
            details=f"frame_center={frame_center}, right_open_center={right_open_center}",
        )

        left_pad_rest = _aabb_center(ctx.part_element_world_aabb(left_nose_pad, elem="left_pad_cushion"))
        right_pad_rest = _aabb_center(ctx.part_element_world_aabb(right_nose_pad, elem="right_pad_cushion"))

    with ctx.pose({left_temple_hinge: left_temple_hinge.motion_limits.upper}):
        left_folded_center = _aabb_center(ctx.part_world_aabb(left_temple))
    with ctx.pose({right_temple_hinge: right_temple_hinge.motion_limits.upper}):
        right_folded_center = _aabb_center(ctx.part_world_aabb(right_temple))

    ctx.check(
        "left temple folds inward toward the lenses",
        left_open_center is not None
        and left_folded_center is not None
        and left_folded_center[0] > left_open_center[0] + 0.030,
        details=f"open={left_open_center}, folded={left_folded_center}",
    )
    ctx.check(
        "right temple folds inward toward the lenses",
        right_open_center is not None
        and right_folded_center is not None
        and right_folded_center[0] < right_open_center[0] - 0.030,
        details=f"open={right_open_center}, folded={right_folded_center}",
    )

    with ctx.pose({left_pad_joint: left_pad_joint.motion_limits.upper}):
        left_pad_rotated = _aabb_center(ctx.part_element_world_aabb(left_nose_pad, elem="left_pad_cushion"))
    with ctx.pose({right_pad_joint: right_pad_joint.motion_limits.upper}):
        right_pad_rotated = _aabb_center(ctx.part_element_world_aabb(right_nose_pad, elem="right_pad_cushion"))

    ctx.check(
        "left nose pad can splay outward on its pivot",
        left_pad_rest is not None
        and left_pad_rotated is not None
        and left_pad_rotated[0] < left_pad_rest[0] - 0.0015,
        details=f"rest={left_pad_rest}, rotated={left_pad_rotated}",
    )
    ctx.check(
        "right nose pad can splay outward on its pivot",
        right_pad_rest is not None
        and right_pad_rotated is not None
        and right_pad_rotated[0] > right_pad_rest[0] + 0.0015,
        details=f"rest={right_pad_rest}, rotated={right_pad_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
