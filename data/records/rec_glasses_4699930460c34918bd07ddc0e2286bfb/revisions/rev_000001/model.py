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
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    superellipse_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _scale_profile(
    profile: list[tuple[float, float]],
    *,
    sx: float = 1.0,
    sy: float = 1.0,
) -> list[tuple[float, float]]:
    return [(x * sx, y * sy) for x, y in profile]


def _mirror_profile_x(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-x, y) for x, y in reversed(profile)]


def _mirror_points_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _build_front_outline() -> list[tuple[float, float]]:
    control_points = [
        (0.000, 0.018),
        (0.018, 0.021),
        (0.043, 0.029),
        (0.063, 0.036),
        (0.074, 0.030),
        (0.076, 0.013),
        (0.073, -0.003),
        (0.063, -0.018),
        (0.041, -0.027),
        (0.015, -0.023),
        (0.004, -0.011),
        (-0.004, -0.011),
        (-0.015, -0.023),
        (-0.041, -0.027),
        (-0.063, -0.018),
        (-0.073, -0.003),
        (-0.076, 0.013),
        (-0.074, 0.030),
        (-0.063, 0.036),
        (-0.043, 0.029),
        (-0.018, 0.021),
    ]
    return sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=6,
        closed=True,
    )


def _build_right_lens_profile() -> list[tuple[float, float]]:
    control_points = [
        (-0.020, 0.009),
        (-0.010, 0.017),
        (0.006, 0.019),
        (0.019, 0.016),
        (0.024, 0.008),
        (0.025, -0.004),
        (0.021, -0.014),
        (0.010, -0.020),
        (-0.006, -0.020),
        (-0.018, -0.016),
        (-0.024, -0.008),
        (-0.024, 0.003),
    ]
    return sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=6,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cat_eye_eyeglasses")

    frame_acetate = model.material("frame_acetate", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.55, 0.56, 0.58, 1.0))
    clear_lens = model.material("clear_lens", rgba=(0.76, 0.84, 0.90, 0.28))
    nose_pad_silicone = model.material("nose_pad_silicone", rgba=(0.92, 0.89, 0.82, 0.72))

    frame_depth = 0.0045
    lens_thickness = 0.0022
    lens_center_x = 0.0335
    hinge_x = 0.0735
    hinge_y = -0.0055
    hinge_z = 0.0180
    nose_pivot_x = 0.0120
    nose_pivot_y = -0.0105
    nose_pivot_z = -0.0120

    front_outline = _build_front_outline()
    right_lens_profile_local = _build_right_lens_profile()
    left_lens_profile_local = _mirror_profile_x(right_lens_profile_local)

    front_frame_geom = (
        ExtrudeWithHolesGeometry(
            front_outline,
            [
                _translate_profile(left_lens_profile_local, dx=-lens_center_x),
                _translate_profile(right_lens_profile_local, dx=lens_center_x),
            ],
            height=frame_depth,
            center=True,
        ).rotate_x(math.pi / 2.0)
    )
    front_frame_mesh = _save_mesh("front_frame_rim", front_frame_geom)

    right_lens_geom = (
        ExtrudeGeometry(
            _scale_profile(right_lens_profile_local, sx=1.0, sy=1.0),
            lens_thickness,
            center=True,
        ).rotate_x(math.pi / 2.0)
    )
    left_lens_geom = (
        ExtrudeGeometry(
            _scale_profile(left_lens_profile_local, sx=1.0, sy=1.0),
            lens_thickness,
            center=True,
        ).rotate_x(math.pi / 2.0)
    )
    right_lens_mesh = _save_mesh("right_lens", right_lens_geom)
    left_lens_mesh = _save_mesh("left_lens", left_lens_geom)

    temple_profile = rounded_rect_profile(0.0062, 0.0026, radius=0.0008, corner_segments=5)
    temple_geom = sweep_profile_along_spline(
        [
            (0.0, -0.0015, 0.0000),
            (0.0, -0.0320, 0.0008),
            (0.0, -0.0720, -0.0015),
            (0.0, -0.1140, -0.0090),
            (0.0, -0.1380, -0.0180),
        ],
        profile=temple_profile,
        samples_per_segment=16,
        cap_profile=True,
    )
    temple_mesh = _save_mesh("slim_temple", temple_geom)

    right_nose_arm_geom = tube_from_spline_points(
        [
            (0.0060, -0.0015, -0.0010),
            (0.0075, -0.0050, -0.0045),
            (0.0105, -0.0085, -0.0090),
            (0.0120, -0.0105, -0.0120),
        ],
        radius=0.0011,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    left_nose_arm_geom = tube_from_spline_points(
        _mirror_points_x(
            [
                (0.0060, -0.0015, -0.0010),
                (0.0075, -0.0050, -0.0045),
                (0.0105, -0.0085, -0.0090),
                (0.0120, -0.0105, -0.0120),
            ]
        ),
        radius=0.0011,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    right_nose_arm_mesh = _save_mesh("right_nose_arm", right_nose_arm_geom)
    left_nose_arm_mesh = _save_mesh("left_nose_arm", left_nose_arm_geom)

    pad_face_geom = (
        ExtrudeGeometry(
            superellipse_profile(0.0100, 0.0150, exponent=2.0, segments=36),
            0.0018,
            center=True,
        ).rotate_x(math.pi / 2.0)
    )
    pad_face_mesh = _save_mesh("nose_pad_face", pad_face_geom)

    pad_bracket_geom = tube_from_spline_points(
        [
            (0.0000, -0.00245, 0.0000),
            (0.0000, -0.00320, -0.0024),
            (0.0000, -0.0042, -0.0048),
        ],
        radius=0.00075,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    pad_bracket_mesh = _save_mesh("nose_pad_bracket", pad_bracket_geom)

    front_frame = model.part("front_frame")
    front_frame.visual(front_frame_mesh, material=frame_acetate, name="front_rim")
    front_frame.visual(
        Box((0.0080, 0.0060, 0.0100)),
        origin=Origin(xyz=(-hinge_x, -0.0025, hinge_z)),
        material=frame_acetate,
        name="left_hinge_block",
    )
    front_frame.visual(
        Box((0.0080, 0.0060, 0.0100)),
        origin=Origin(xyz=(hinge_x, -0.0025, hinge_z)),
        material=frame_acetate,
        name="right_hinge_block",
    )
    front_frame.visual(
        Cylinder(radius=0.0025, length=0.0100),
        origin=Origin(xyz=(-hinge_x, -0.0025, hinge_z)),
        material=dark_metal,
        name="left_hinge_barrel",
    )
    front_frame.visual(
        Cylinder(radius=0.0025, length=0.0100),
        origin=Origin(xyz=(hinge_x, -0.0025, hinge_z)),
        material=dark_metal,
        name="right_hinge_barrel",
    )
    front_frame.visual(left_nose_arm_mesh, material=dark_metal, name="left_nose_arm")
    front_frame.visual(right_nose_arm_mesh, material=dark_metal, name="right_nose_arm")
    front_frame.visual(
        Cylinder(radius=0.0017, length=0.0040),
        origin=Origin(
            xyz=(-nose_pivot_x, nose_pivot_y, nose_pivot_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_metal,
        name="left_nose_pivot",
    )
    front_frame.visual(
        Cylinder(radius=0.0017, length=0.0040),
        origin=Origin(
            xyz=(nose_pivot_x, nose_pivot_y, nose_pivot_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_metal,
        name="right_nose_pivot",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.156, 0.022, 0.070)),
        mass=0.030,
        origin=Origin(xyz=(0.0, -0.003, 0.004)),
    )

    left_lens = model.part("left_lens")
    left_lens.visual(left_lens_mesh, material=clear_lens, name="lens")
    left_lens.inertial = Inertial.from_geometry(
        Box((0.048, lens_thickness, 0.040)),
        mass=0.004,
    )

    right_lens = model.part("right_lens")
    right_lens.visual(right_lens_mesh, material=clear_lens, name="lens")
    right_lens.inertial = Inertial.from_geometry(
        Box((0.048, lens_thickness, 0.040)),
        mass=0.004,
    )

    left_temple = model.part("left_temple")
    left_temple.visual(temple_mesh, material=frame_acetate, name="temple_arm")
    left_temple.visual(
        Box((0.0060, 0.0060, 0.0070)),
        origin=Origin(xyz=(0.0, -0.0030, 0.0)),
        material=frame_acetate,
        name="hinge_lug",
    )
    left_temple.visual(
        Cylinder(radius=0.0017, length=0.0070),
        origin=Origin(xyz=(0.0, -0.0035, 0.0)),
        material=dark_metal,
        name="hinge_pin_cover",
    )
    left_temple.inertial = Inertial.from_geometry(
        Box((0.010, 0.140, 0.024)),
        mass=0.010,
        origin=Origin(xyz=(0.0, -0.070, -0.009)),
    )

    right_temple = model.part("right_temple")
    right_temple.visual(temple_mesh, material=frame_acetate, name="temple_arm")
    right_temple.visual(
        Box((0.0060, 0.0060, 0.0070)),
        origin=Origin(xyz=(0.0, -0.0030, 0.0)),
        material=frame_acetate,
        name="hinge_lug",
    )
    right_temple.visual(
        Cylinder(radius=0.0017, length=0.0070),
        origin=Origin(xyz=(0.0, -0.0035, 0.0)),
        material=dark_metal,
        name="hinge_pin_cover",
    )
    right_temple.inertial = Inertial.from_geometry(
        Box((0.010, 0.140, 0.024)),
        mass=0.010,
        origin=Origin(xyz=(0.0, -0.070, -0.009)),
    )

    left_nose_pad = model.part("left_nose_pad")
    left_nose_pad.visual(
        Cylinder(radius=0.00075, length=0.0020),
        origin=Origin(xyz=(0.0, -0.00245, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="pivot_stem",
    )
    left_nose_pad.visual(pad_bracket_mesh, material=dark_metal, name="pad_bracket")
    left_nose_pad.visual(
        pad_face_mesh,
        origin=Origin(xyz=(0.0, -0.0048, -0.0072), rpy=(0.30, 0.0, 0.0)),
        material=nose_pad_silicone,
        name="pad_face",
    )
    left_nose_pad.inertial = Inertial.from_geometry(
        Box((0.010, 0.008, 0.016)),
        mass=0.0012,
        origin=Origin(xyz=(0.0, -0.0048, -0.0072)),
    )

    right_nose_pad = model.part("right_nose_pad")
    right_nose_pad.visual(
        Cylinder(radius=0.00075, length=0.0020),
        origin=Origin(xyz=(0.0, -0.00245, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="pivot_stem",
    )
    right_nose_pad.visual(pad_bracket_mesh, material=dark_metal, name="pad_bracket")
    right_nose_pad.visual(
        pad_face_mesh,
        origin=Origin(xyz=(0.0, -0.0048, -0.0072), rpy=(0.30, 0.0, 0.0)),
        material=nose_pad_silicone,
        name="pad_face",
    )
    right_nose_pad.inertial = Inertial.from_geometry(
        Box((0.010, 0.008, 0.016)),
        mass=0.0012,
        origin=Origin(xyz=(0.0, -0.0048, -0.0072)),
    )

    model.articulation(
        "front_to_left_lens",
        ArticulationType.FIXED,
        parent=front_frame,
        child=left_lens,
        origin=Origin(xyz=(-lens_center_x, -0.0002, 0.0)),
    )
    model.articulation(
        "front_to_right_lens",
        ArticulationType.FIXED,
        parent=front_frame,
        child=right_lens,
        origin=Origin(xyz=(lens_center_x, -0.0002, 0.0)),
    )
    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(-hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "left_nose_pad_pivot",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_nose_pad,
        origin=Origin(xyz=(-nose_pivot_x, nose_pivot_y, nose_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.02,
            velocity=2.0,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "right_nose_pad_pivot",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_nose_pad,
        origin=Origin(xyz=(nose_pivot_x, nose_pivot_y, nose_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.02,
            velocity=2.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_frame = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_nose_pad = object_model.get_part("left_nose_pad")
    right_nose_pad = object_model.get_part("right_nose_pad")

    left_temple_hinge = object_model.get_articulation("left_temple_hinge")
    right_temple_hinge = object_model.get_articulation("right_temple_hinge")
    left_nose_pad_pivot = object_model.get_articulation("left_nose_pad_pivot")
    right_nose_pad_pivot = object_model.get_articulation("right_nose_pad_pivot")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    rim_aabb = ctx.part_element_world_aabb(front_frame, elem="front_rim")
    rim_width = None
    rim_height = None
    if rim_aabb is not None:
        rim_width = rim_aabb[1][0] - rim_aabb[0][0]
        rim_height = rim_aabb[1][2] - rim_aabb[0][2]
    ctx.check(
        "front frame has realistic cat-eye scale",
        rim_width is not None
        and rim_height is not None
        and 0.140 <= rim_width <= 0.160
        and 0.055 <= rim_height <= 0.075,
        details=f"width={rim_width}, height={rim_height}",
    )

    with ctx.pose({left_temple_hinge: 0.0, right_temple_hinge: 0.0}):
        ctx.expect_contact(
            front_frame,
            left_temple,
            elem_a="left_hinge_block",
            elem_b="hinge_lug",
            name="left temple lug seats on hinge block",
        )
        ctx.expect_contact(
            front_frame,
            right_temple,
            elem_a="right_hinge_block",
            elem_b="hinge_lug",
            name="right temple lug seats on hinge block",
        )
        ctx.expect_gap(
            front_frame,
            left_temple,
            axis="y",
            positive_elem="front_rim",
            negative_elem="temple_arm",
            min_gap=0.002,
            name="left temple stays behind the front rim when open",
        )
        ctx.expect_gap(
            front_frame,
            right_temple,
            axis="y",
            positive_elem="front_rim",
            negative_elem="temple_arm",
            min_gap=0.002,
            name="right temple stays behind the front rim when open",
        )
        ctx.expect_gap(
            front_frame,
            left_nose_pad,
            axis="y",
            positive_elem="front_rim",
            negative_elem="pad_face",
            min_gap=0.003,
            name="left nose pad sits behind the frame front",
        )
        ctx.expect_gap(
            front_frame,
            right_nose_pad,
            axis="y",
            positive_elem="front_rim",
            negative_elem="pad_face",
            min_gap=0.003,
            name="right nose pad sits behind the frame front",
        )
        left_temple_rest = _aabb_center(ctx.part_element_world_aabb(left_temple, elem="temple_arm"))
        right_temple_rest = _aabb_center(ctx.part_element_world_aabb(right_temple, elem="temple_arm"))

    with ctx.pose({left_temple_hinge: 1.18, right_temple_hinge: 1.18}):
        left_temple_folded = _aabb_center(ctx.part_element_world_aabb(left_temple, elem="temple_arm"))
        right_temple_folded = _aabb_center(ctx.part_element_world_aabb(right_temple, elem="temple_arm"))

    ctx.check(
        "left temple folds inward toward the front",
        left_temple_rest is not None
        and left_temple_folded is not None
        and left_temple_folded[0] > left_temple_rest[0] + 0.020
        and left_temple_folded[1] > left_temple_rest[1] + 0.030,
        details=f"rest={left_temple_rest}, folded={left_temple_folded}",
    )
    ctx.check(
        "right temple folds inward toward the front",
        right_temple_rest is not None
        and right_temple_folded is not None
        and right_temple_folded[0] < right_temple_rest[0] - 0.020
        and right_temple_folded[1] > right_temple_rest[1] + 0.030,
        details=f"rest={right_temple_rest}, folded={right_temple_folded}",
    )

    with ctx.pose({left_nose_pad_pivot: -0.25}):
        left_pad_low = _aabb_center(ctx.part_element_world_aabb(left_nose_pad, elem="pad_face"))
    with ctx.pose({left_nose_pad_pivot: 0.25}):
        left_pad_high = _aabb_center(ctx.part_element_world_aabb(left_nose_pad, elem="pad_face"))
    ctx.check(
        "left nose pad pivots on its bracket",
        left_pad_low is not None
        and left_pad_high is not None
        and abs(left_pad_high[2] - left_pad_low[2]) > 0.0012,
        details=f"low={left_pad_low}, high={left_pad_high}",
    )

    with ctx.pose({right_nose_pad_pivot: -0.25}):
        right_pad_low = _aabb_center(ctx.part_element_world_aabb(right_nose_pad, elem="pad_face"))
    with ctx.pose({right_nose_pad_pivot: 0.25}):
        right_pad_high = _aabb_center(ctx.part_element_world_aabb(right_nose_pad, elem="pad_face"))
    ctx.check(
        "right nose pad pivots on its bracket",
        right_pad_low is not None
        and right_pad_high is not None
        and abs(right_pad_high[2] - right_pad_low[2]) > 0.0012,
        details=f"low={right_pad_low}, high={right_pad_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
