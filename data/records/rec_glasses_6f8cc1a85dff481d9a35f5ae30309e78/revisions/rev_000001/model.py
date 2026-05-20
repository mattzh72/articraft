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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _mirror_points_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _build_temple_mesh(name: str, *, side_sign: float):
    arm_profile = rounded_rect_profile(0.0054, 0.0024, radius=0.0008, corner_segments=5)
    left_path = [
        (0.0004, -0.0030, 0.0000),
        (0.0012, -0.0300, -0.0004),
        (0.0038, -0.0860, -0.0038),
        (0.0090, -0.1420, -0.0180),
    ]
    path = left_path if side_sign > 0.0 else _mirror_points_x(left_path)
    return mesh_from_geometry(
        sweep_profile_along_spline(
            path,
            profile=arm_profile,
            samples_per_segment=18,
            cap_profile=True,
        ),
        name,
    )


def _add_spring_segment(part, *, material, metal) -> None:
    part.visual(
        Box((0.0058, 0.0180, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0100, 0.0)),
        material=material,
        name="spring_body",
    )
    part.visual(
        Cylinder(radius=0.0017, length=0.0064),
        origin=Origin(xyz=(0.0, -0.0012, 0.0)),
        material=metal,
        name="front_knuckle",
    )
    part.visual(
        Cylinder(radius=0.0017, length=0.0064),
        origin=Origin(xyz=(0.0, -0.0180, 0.0)),
        material=metal,
        name="rear_knuckle",
    )


def _add_temple_arm(part, *, arm_mesh, material, metal) -> None:
    part.visual(
        Cylinder(radius=0.0017, length=0.0064),
        origin=Origin(xyz=(0.0, -0.0004, 0.0)),
        material=metal,
        name="hinge_knuckle",
    )
    part.visual(
        Box((0.0062, 0.0124, 0.0034)),
        origin=Origin(xyz=(0.0, -0.0080, 0.0)),
        material=material,
        name="hinge_root",
    )
    part.visual(
        arm_mesh,
        material=material,
        name="arm_body",
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_office_glasses")

    acetate = model.material("acetate", rgba=(0.10, 0.10, 0.11, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.35, 0.37, 1.0))

    front_outer = [
        (-0.066, 0.021),
        (-0.066, -0.004),
        (-0.061, -0.017),
        (-0.048, -0.021),
        (-0.020, -0.021),
        (-0.013, -0.017),
        (-0.008, -0.012),
        (0.008, -0.012),
        (0.013, -0.017),
        (0.020, -0.021),
        (0.048, -0.021),
        (0.061, -0.017),
        (0.066, -0.004),
        (0.066, 0.021),
        (0.011, 0.021),
        (0.006, 0.0175),
        (-0.006, 0.0175),
        (-0.011, 0.021),
    ]
    lens_opening = rounded_rect_profile(0.050, 0.031, 0.0048, corner_segments=8)
    front_shell_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            front_outer,
            [
                _shift_profile(lens_opening, dx=0.033, dy=-0.001),
                _shift_profile(lens_opening, dx=-0.033, dy=-0.001),
            ],
            height=0.0042,
            center=True,
        ).rotate_x(-math.pi / 2.0),
        "front_frame_shell",
    )

    left_temple_mesh = _build_temple_mesh("left_temple_arm", side_sign=1.0)
    right_temple_mesh = _build_temple_mesh("right_temple_arm", side_sign=-1.0)

    front_frame = model.part("front_frame")
    front_frame.visual(
        front_shell_mesh,
        material=acetate,
        name="front_shell",
    )
    front_frame.visual(
        Box((0.018, 0.0046, 0.0060)),
        origin=Origin(xyz=(0.0, -0.0008, -0.0020)),
        material=acetate,
        name="bridge_saddle",
    )
    front_frame.visual(
        Box((0.0075, 0.0080, 0.0100)),
        origin=Origin(xyz=(0.0685, -0.0038, 0.0105)),
        material=graphite,
        name="left_hinge_block",
    )
    front_frame.visual(
        Box((0.0075, 0.0080, 0.0100)),
        origin=Origin(xyz=(-0.0685, -0.0038, 0.0105)),
        material=graphite,
        name="right_hinge_block",
    )
    front_frame.visual(
        Cylinder(radius=0.0019, length=0.0070),
        origin=Origin(xyz=(0.0690, -0.0078, 0.0105)),
        material=dark_metal,
        name="left_hinge_barrel",
    )
    front_frame.visual(
        Cylinder(radius=0.0019, length=0.0070),
        origin=Origin(xyz=(-0.0690, -0.0078, 0.0105)),
        material=dark_metal,
        name="right_hinge_barrel",
    )

    left_spring = model.part("left_spring_segment")
    _add_spring_segment(left_spring, material=graphite, metal=dark_metal)

    right_spring = model.part("right_spring_segment")
    _add_spring_segment(right_spring, material=graphite, metal=dark_metal)

    left_temple = model.part("left_temple")
    _add_temple_arm(left_temple, arm_mesh=left_temple_mesh, material=acetate, metal=dark_metal)

    right_temple = model.part("right_temple")
    _add_temple_arm(right_temple, arm_mesh=right_temple_mesh, material=acetate, metal=dark_metal)

    model.articulation(
        "front_to_left_spring",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_spring,
        origin=Origin(xyz=(0.0690, -0.0102, 0.0105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=0.0,
            upper=0.32,
        ),
    )
    model.articulation(
        "left_spring_to_temple",
        ArticulationType.REVOLUTE,
        parent=left_spring,
        child=left_temple,
        origin=Origin(xyz=(0.0, -0.0210, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "front_to_right_spring",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_spring,
        origin=Origin(xyz=(-0.0690, -0.0102, 0.0105)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=0.0,
            upper=0.32,
        ),
    )
    model.articulation(
        "right_spring_to_temple",
        ArticulationType.REVOLUTE,
        parent=right_spring,
        child=right_temple,
        origin=Origin(xyz=(0.0, -0.0210, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=0.0,
            upper=1.65,
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
    front_frame = object_model.get_part("front_frame")
    left_spring = object_model.get_part("left_spring_segment")
    right_spring = object_model.get_part("right_spring_segment")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")

    left_spring_joint = object_model.get_articulation("front_to_left_spring")
    right_spring_joint = object_model.get_articulation("front_to_right_spring")
    left_temple_joint = object_model.get_articulation("left_spring_to_temple")
    right_temple_joint = object_model.get_articulation("right_spring_to_temple")

    front_aabb = ctx.part_world_aabb(front_frame)
    if front_aabb is not None:
        width = front_aabb[1][0] - front_aabb[0][0]
        height = front_aabb[1][2] - front_aabb[0][2]
        ctx.check(
            "front frame has realistic office-glasses proportions",
            0.138 <= width <= 0.147 and 0.040 <= height <= 0.047,
            details=f"width={width:.4f}, height={height:.4f}",
        )
    else:
        ctx.fail("front frame AABB exists", "front_frame returned no world AABB")

    ctx.check(
        "left spring hinge axis is vertical",
        left_spring_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={left_spring_joint.axis}",
    )
    ctx.check(
        "right spring hinge axis is vertical",
        right_spring_joint.axis == (0.0, 0.0, -1.0),
        details=f"axis={right_spring_joint.axis}",
    )
    ctx.check(
        "left temple hinge axis is vertical",
        left_temple_joint.axis == (0.0, 0.0, -1.0),
        details=f"axis={left_temple_joint.axis}",
    )
    ctx.check(
        "right temple hinge axis is vertical",
        right_temple_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={right_temple_joint.axis}",
    )

    ctx.expect_contact(
        front_frame,
        left_spring,
        contact_tol=0.00005,
        name="left spring segment sits just behind the frame hinge block",
    )
    ctx.expect_contact(
        front_frame,
        right_spring,
        contact_tol=0.00005,
        name="right spring segment sits just behind the frame hinge block",
    )
    ctx.expect_contact(
        left_spring,
        left_temple,
        contact_tol=0.00005,
        name="left temple arm starts just behind its spring segment",
    )
    ctx.expect_contact(
        right_spring,
        right_temple,
        contact_tol=0.00005,
        name="right temple arm starts just behind its spring segment",
    )

    rest_left_center = _aabb_center(ctx.part_element_world_aabb(left_temple, elem="arm_body"))
    rest_right_center = _aabb_center(ctx.part_element_world_aabb(right_temple, elem="arm_body"))

    with ctx.pose({left_spring_joint: 0.24, right_spring_joint: 0.24}):
        sprung_left_center = _aabb_center(ctx.part_element_world_aabb(left_temple, elem="arm_body"))
        sprung_right_center = _aabb_center(ctx.part_element_world_aabb(right_temple, elem="arm_body"))
        ctx.check(
            "spring hinges splay the temples outward",
            rest_left_center is not None
            and rest_right_center is not None
            and sprung_left_center is not None
            and sprung_right_center is not None
            and sprung_left_center[0] > rest_left_center[0] + 0.004
            and sprung_right_center[0] < rest_right_center[0] - 0.004,
            details=(
                f"rest_left={rest_left_center}, sprung_left={sprung_left_center}, "
                f"rest_right={rest_right_center}, sprung_right={sprung_right_center}"
            ),
        )

    with ctx.pose({left_temple_joint: 1.40, right_temple_joint: 1.40}):
        folded_left_center = _aabb_center(ctx.part_element_world_aabb(left_temple, elem="arm_body"))
        folded_right_center = _aabb_center(ctx.part_element_world_aabb(right_temple, elem="arm_body"))
        ctx.check(
            "main temple hinges fold both arms inward toward the lenses",
            rest_left_center is not None
            and rest_right_center is not None
            and folded_left_center is not None
            and folded_right_center is not None
            and folded_left_center[0] < rest_left_center[0] - 0.035
            and folded_right_center[0] > rest_right_center[0] + 0.035,
            details=(
                f"rest_left={rest_left_center}, folded_left={folded_left_center}, "
                f"rest_right={rest_right_center}, folded_right={folded_right_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
