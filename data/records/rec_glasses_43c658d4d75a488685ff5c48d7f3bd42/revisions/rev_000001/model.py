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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _lens_mesh(name: str, *, width: float, height: float, thickness: float, corner: float):
    return _mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(width, height, corner, corner_segments=10),
            thickness,
            center=True,
        ).rotate_x(pi / 2.0),
    )


def _wire_mesh(name: str, points: list[tuple[float, float, float]], *, radius: float, closed: bool = False):
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            closed_spline=closed,
            radial_segments=18,
            cap_ends=not closed,
        ),
    )


def _rounded_loop_points(
    *,
    width: float,
    height: float,
    corner: float,
) -> list[tuple[float, float, float]]:
    return [(x, 0.0, z) for x, z in rounded_rect_profile(width, height, corner, corner_segments=10)]


def _temple_points(side_sign: float) -> list[tuple[float, float, float]]:
    return [
        (0.0, 0.0030, 0.0),
        (side_sign * 0.010, -0.034, -0.001),
        (side_sign * 0.009, -0.086, -0.003),
        (side_sign * 0.005, -0.130, -0.016),
        (0.0, -0.146, -0.033),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="half_rim_reading_glasses_with_sun_clip")

    gunmetal = model.material("gunmetal", rgba=(0.26, 0.28, 0.31, 1.0))
    black_coating = model.material("black_coating", rgba=(0.10, 0.10, 0.11, 1.0))
    clear_lens = model.material("clear_lens", rgba=(0.78, 0.86, 0.94, 0.20))
    sun_tint = model.material("sun_tint", rgba=(0.18, 0.20, 0.18, 0.58))
    nose_pad_clear = model.material("nose_pad_clear", rgba=(0.92, 0.95, 0.98, 0.45))

    lens_width = 0.048
    lens_height = 0.028
    lens_corner = 0.007
    lens_center_x = 0.0325
    main_lens_z = -0.001
    outer_hinge_x = 0.061
    temple_hinge_z = 0.0115

    main_lens_mesh = _lens_mesh(
        "main_reading_lens",
        width=lens_width,
        height=lens_height,
        thickness=0.0018,
        corner=lens_corner,
    )
    sun_lens_mesh = _lens_mesh(
        "sun_clip_lens",
        width=0.0514,
        height=0.0314,
        thickness=0.0014,
        corner=0.0075,
    )
    sun_loop_mesh = _wire_mesh(
        "sun_clip_lens_loop",
        _rounded_loop_points(width=0.052, height=0.032, corner=0.0078),
        radius=0.00095,
        closed=True,
    )

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.145, 0.020, 0.048)),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    left_brow_points = [
        (-0.011, 0.0, 0.009),
        (-0.020, 0.0, 0.014),
        (-0.033, 0.0, 0.016),
        (-0.050, 0.0, 0.014),
        (-outer_hinge_x, 0.0, temple_hinge_z),
    ]
    right_brow_points = [(-x, y, z) for x, y, z in left_brow_points]
    brow_bridge_points = [
        (-0.011, 0.0, 0.009),
        (-0.006, 0.0, 0.011),
        (0.0, 0.0, 0.012),
        (0.006, 0.0, 0.011),
        (0.011, 0.0, 0.009),
    ]
    nose_bridge_points = [
        (-0.011, 0.0, 0.009),
        (-0.006, 0.0, 0.001),
        (0.0, 0.0, -0.0045),
        (0.006, 0.0, 0.001),
        (0.011, 0.0, 0.009),
    ]

    front_frame.visual(
        main_lens_mesh,
        origin=Origin(xyz=(-lens_center_x, 0.0, main_lens_z)),
        material=clear_lens,
        name="left_main_lens",
    )
    front_frame.visual(
        main_lens_mesh,
        origin=Origin(xyz=(lens_center_x, 0.0, main_lens_z)),
        material=clear_lens,
        name="right_main_lens",
    )
    front_frame.visual(
        _wire_mesh("left_brow_wire", left_brow_points, radius=0.00155),
        material=gunmetal,
        name="left_brow_wire",
    )
    front_frame.visual(
        _wire_mesh("right_brow_wire", right_brow_points, radius=0.00155),
        material=gunmetal,
        name="right_brow_wire",
    )
    front_frame.visual(
        _wire_mesh("front_brow_bridge", brow_bridge_points, radius=0.00145),
        material=gunmetal,
        name="brow_bridge",
    )
    front_frame.visual(
        _wire_mesh("front_nose_bridge", nose_bridge_points, radius=0.00125),
        material=gunmetal,
        name="nose_bridge",
    )
    front_frame.visual(
        Box((0.0045, 0.0014, 0.0080)),
        origin=Origin(xyz=(-outer_hinge_x, 0.0007, temple_hinge_z)),
        material=gunmetal,
        name="left_hinge_plate",
    )
    front_frame.visual(
        Box((0.0045, 0.0014, 0.0080)),
        origin=Origin(xyz=(outer_hinge_x, 0.0007, temple_hinge_z)),
        material=gunmetal,
        name="right_hinge_plate",
    )

    for side_sign, prefix in ((-1.0, "left"), (1.0, "right")):
        clip_x = side_sign * 0.024
        arm_points = [
            (clip_x, 0.0, 0.0140),
            (clip_x, 0.0003, 0.0162),
            (clip_x, 0.0004, 0.0180),
        ]
        front_frame.visual(
            _wire_mesh(f"{prefix}_clip_post", arm_points, radius=0.00085),
            material=gunmetal,
            name=f"{prefix}_clip_post",
        )
        front_frame.visual(
            Cylinder(radius=0.0010, length=0.0042),
            origin=Origin(xyz=(clip_x, 0.0004, 0.0180), rpy=(0.0, pi / 2.0, 0.0)),
            material=gunmetal,
            name=f"{prefix}_clip_pivot",
        )

    left_pad_arm_points = [
        (-0.0055, 0.0000, -0.0010),
        (-0.0070, -0.0028, -0.0060),
        (-0.0092, -0.0052, -0.0102),
    ]
    right_pad_arm_points = [(-x, y, z) for x, y, z in left_pad_arm_points]
    front_frame.visual(
        _wire_mesh("left_pad_arm", left_pad_arm_points, radius=0.00065),
        material=gunmetal,
        name="left_pad_arm",
    )
    front_frame.visual(
        _wire_mesh("right_pad_arm", right_pad_arm_points, radius=0.00065),
        material=gunmetal,
        name="right_pad_arm",
    )
    front_frame.visual(
        Cylinder(radius=0.0035, length=0.0018),
        origin=Origin(xyz=(-0.0100, -0.0058, -0.0110), rpy=(pi / 2.0, 0.0, 0.0)),
        material=nose_pad_clear,
        name="left_nose_pad",
    )
    front_frame.visual(
        Cylinder(radius=0.0035, length=0.0018),
        origin=Origin(xyz=(0.0100, -0.0058, -0.0110), rpy=(pi / 2.0, 0.0, 0.0)),
        material=nose_pad_clear,
        name="right_nose_pad",
    )

    left_temple = model.part("left_temple")
    left_temple.inertial = Inertial.from_geometry(
        Box((0.024, 0.160, 0.040)),
        mass=0.006,
        origin=Origin(xyz=(0.0, -0.068, -0.012)),
    )
    left_temple_points = _temple_points(-1.0)
    left_temple.visual(
        Cylinder(radius=0.0017, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0030, 0.0)),
        material=gunmetal,
        name="temple_barrel",
    )
    left_temple.visual(
        _wire_mesh("left_temple_wire", left_temple_points, radius=0.00115),
        material=gunmetal,
        name="temple_wire",
    )
    left_temple.visual(
        _wire_mesh("left_temple_sleeve", left_temple_points[2:], radius=0.00175),
        material=black_coating,
        name="temple_sleeve",
    )
    left_temple.visual(
        Sphere(radius=0.0022),
        origin=Origin(xyz=left_temple_points[-1]),
        material=black_coating,
        name="temple_tip",
    )

    right_temple = model.part("right_temple")
    right_temple.inertial = Inertial.from_geometry(
        Box((0.024, 0.160, 0.040)),
        mass=0.006,
        origin=Origin(xyz=(0.0, -0.068, -0.012)),
    )
    right_temple_points = _temple_points(1.0)
    right_temple.visual(
        Cylinder(radius=0.0017, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0030, 0.0)),
        material=gunmetal,
        name="temple_barrel",
    )
    right_temple.visual(
        _wire_mesh("right_temple_wire", right_temple_points, radius=0.00115),
        material=gunmetal,
        name="temple_wire",
    )
    right_temple.visual(
        _wire_mesh("right_temple_sleeve", right_temple_points[2:], radius=0.00175),
        material=black_coating,
        name="temple_sleeve",
    )
    right_temple.visual(
        Sphere(radius=0.0022),
        origin=Origin(xyz=right_temple_points[-1]),
        material=black_coating,
        name="temple_tip",
    )

    sun_clip = model.part("sun_clip")
    sun_clip.inertial = Inertial.from_geometry(
        Box((0.138, 0.018, 0.055)),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.006, -0.010)),
    )
    sun_lens_y = 0.0042
    sun_lens_z = -0.0180
    sun_clip.visual(
        sun_lens_mesh,
        origin=Origin(xyz=(-lens_center_x, sun_lens_y, sun_lens_z)),
        material=sun_tint,
        name="left_sun_lens",
    )
    sun_clip.visual(
        sun_lens_mesh,
        origin=Origin(xyz=(lens_center_x, sun_lens_y, sun_lens_z)),
        material=sun_tint,
        name="right_sun_lens",
    )
    sun_clip.visual(
        sun_loop_mesh,
        origin=Origin(xyz=(-lens_center_x, sun_lens_y, sun_lens_z)),
        material=black_coating,
        name="left_sun_rim",
    )
    sun_clip.visual(
        sun_loop_mesh,
        origin=Origin(xyz=(lens_center_x, sun_lens_y, sun_lens_z)),
        material=black_coating,
        name="right_sun_rim",
    )
    sun_clip.visual(
        Cylinder(radius=0.0010, length=0.0220),
        origin=Origin(xyz=(0.0, sun_lens_y, -0.0115), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_coating,
        name="sun_bridge",
    )
    for side_sign, prefix in ((-1.0, "left"), (1.0, "right")):
        arm_points = [
            (side_sign * 0.024, sun_lens_y, -0.0030),
            (side_sign * 0.024, 0.0034, -0.0014),
            (side_sign * 0.024, 0.0018, 0.0),
        ]
        sun_clip.visual(
            _wire_mesh(f"{prefix}_sun_pivot_arm", arm_points, radius=0.00082),
            material=black_coating,
            name=f"{prefix}_pivot_arm",
        )
        sun_clip.visual(
            Cylinder(radius=0.00110, length=0.0048),
            origin=Origin(xyz=(side_sign * 0.024, 0.0018, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=black_coating,
            name=f"{prefix}_pivot_barrel",
        )

    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(-outer_hinge_x, 0.0, temple_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=1.70,
        ),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(outer_hinge_x, 0.0, temple_hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=1.70,
        ),
    )
    model.articulation(
        "sun_clip_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=sun_clip,
        origin=Origin(xyz=(0.0, 0.0, 0.0180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=0.0,
            upper=2.35,
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
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    sun_clip = object_model.get_part("sun_clip")

    left_temple_hinge = object_model.get_articulation("left_temple_hinge")
    right_temple_hinge = object_model.get_articulation("right_temple_hinge")
    sun_clip_hinge = object_model.get_articulation("sun_clip_hinge")

    def _center_of_aabb(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))

    with ctx.pose({sun_clip_hinge: 0.0}):
        ctx.expect_gap(
            sun_clip,
            front_frame,
            axis="y",
            min_gap=0.0015,
            max_gap=0.0060,
            positive_elem="left_sun_lens",
            negative_elem="left_main_lens",
            name="left sun lens sits ahead of reading lens",
        )
        ctx.expect_gap(
            sun_clip,
            front_frame,
            axis="y",
            min_gap=0.0015,
            max_gap=0.0060,
            positive_elem="right_sun_lens",
            negative_elem="right_main_lens",
            name="right sun lens sits ahead of reading lens",
        )
        ctx.expect_overlap(
            sun_clip,
            front_frame,
            axes="xz",
            min_overlap=0.022,
            elem_a="left_sun_lens",
            elem_b="left_main_lens",
            name="left sun lens covers main lens footprint",
        )
        ctx.expect_overlap(
            sun_clip,
            front_frame,
            axes="xz",
            min_overlap=0.022,
            elem_a="right_sun_lens",
            elem_b="right_main_lens",
            name="right sun lens covers main lens footprint",
        )

    left_tip_open = _center_of_aabb(ctx.part_element_world_aabb(left_temple, elem="temple_tip"))
    right_tip_open = _center_of_aabb(ctx.part_element_world_aabb(right_temple, elem="temple_tip"))
    with ctx.pose({left_temple_hinge: 1.55, right_temple_hinge: 1.55}):
        left_tip_folded = _center_of_aabb(ctx.part_element_world_aabb(left_temple, elem="temple_tip"))
        right_tip_folded = _center_of_aabb(ctx.part_element_world_aabb(right_temple, elem="temple_tip"))

    ctx.check(
        "left temple folds inward toward the bridge",
        left_tip_open is not None
        and left_tip_folded is not None
        and left_tip_folded[0] > left_tip_open[0] + 0.055
        and left_tip_folded[1] > left_tip_open[1] + 0.070,
        details=f"open={left_tip_open}, folded={left_tip_folded}",
    )
    ctx.check(
        "right temple folds inward toward the bridge",
        right_tip_open is not None
        and right_tip_folded is not None
        and right_tip_folded[0] < right_tip_open[0] - 0.055
        and right_tip_folded[1] > right_tip_open[1] + 0.070,
        details=f"open={right_tip_open}, folded={right_tip_folded}",
    )

    sun_lens_rest = _center_of_aabb(ctx.part_element_world_aabb(sun_clip, elem="left_sun_lens"))
    with ctx.pose({sun_clip_hinge: 2.25}):
        sun_lens_up = _center_of_aabb(ctx.part_element_world_aabb(sun_clip, elem="left_sun_lens"))
        ctx.expect_gap(
            sun_clip,
            front_frame,
            axis="z",
            min_gap=0.005,
            positive_elem="left_sun_lens",
            negative_elem="left_main_lens",
            name="flipped left sun lens clears above main lens",
        )

    ctx.check(
        "sun clip flips upward above the brow line",
        sun_lens_rest is not None
        and sun_lens_up is not None
        and sun_lens_up[2] > sun_lens_rest[2] + 0.025,
        details=f"rest={sun_lens_rest}, raised={sun_lens_up}",
    )

    ctx.check(
        "left temple hinge axis is vertical",
        tuple(left_temple_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={left_temple_hinge.axis}",
    )
    ctx.check(
        "sun clip hinge runs transversely across the front",
        tuple(sun_clip_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={sun_clip_hinge.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
