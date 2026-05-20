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
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


FRAME_WIDTH = 0.142
LENS_CENTER_X = 0.031
LENS_CENTER_Z = 0.0
HINGE_X = 0.068
HINGE_Z = 0.012
NOSE_PIVOT_X = 0.0125
NOSE_PIVOT_Y = -0.0115
NOSE_PIVOT_Z = -0.011


def _closed_profile_path(
    profile_2d: list[tuple[float, float]],
    *,
    center_x: float,
    center_z: float,
    face_wrap: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_w = max(abs(px) for px, _ in profile_2d) or 1.0
    path: list[tuple[float, float, float]] = []
    for px, pz in profile_2d:
        normalized = abs(px) / half_w
        y = -face_wrap * normalized * normalized
        path.append((center_x + px, y, center_z + pz))
    return path


def _brow_piece_mesh(
    *,
    length: float,
    height: float,
    thickness: float,
    center: tuple[float, float, float],
) -> object:
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(length, height, radius=min(height * 0.45, 0.0045)),
        thickness,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(*center)
    return mesh_from_geometry(geom, f"brow_piece_{center[0]:+.3f}".replace(".", "_"))


def _lens_mesh(*, center_x: float, center_z: float, thickness: float, name: str) -> object:
    geom = ExtrudeGeometry.centered(
        superellipse_profile(0.0475, 0.0395, exponent=2.4, segments=40),
        thickness,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(center_x, -0.0001, center_z)
    return mesh_from_geometry(geom, name)


def _lower_rim_mesh(*, center_x: float, center_z: float, name: str) -> object:
    loop = _closed_profile_path(
        superellipse_profile(0.049, 0.041, exponent=2.2, segments=32),
        center_x=center_x,
        center_z=center_z,
        face_wrap=0.0035,
    )
    geom = tube_from_spline_points(
        loop,
        radius=0.00105,
        samples_per_segment=3,
        closed_spline=True,
        radial_segments=16,
        cap_ends=False,
    )
    return mesh_from_geometry(geom, name)


def _temple_body_mesh(*, sign: float, name: str) -> object:
    path = [
        (0.0025 * sign, -0.007, 0.0005),
        (0.0040 * sign, -0.030, -0.0005),
        (0.0085 * sign, -0.088, -0.006),
        (0.0120 * sign, -0.128, -0.022),
        (0.0145 * sign, -0.148, -0.051),
    ]
    geom = sweep_profile_along_spline(
        path,
        profile=rounded_rect_profile(0.0052, 0.0023, radius=0.0008),
        samples_per_segment=16,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(geom, name)


def _nose_pad_mesh(*, sign: float, name: str) -> object:
    geom = ExtrudeGeometry.centered(
        superellipse_profile(0.0115, 0.0165, exponent=2.8, segments=28),
        0.0014,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.rotate_y(0.30)
    geom.rotate_z(0.55 * sign)
    geom.translate(0.0030 * sign, -0.0043, -0.0082)
    return mesh_from_geometry(geom, name)


def _nose_pad_arm_mesh(*, sign: float, name: str) -> object:
    geom = tube_from_spline_points(
        [
            (0.0, -0.00165, 0.0),
            (0.0014 * sign, -0.0028, -0.0033),
            (0.0025 * sign, -0.0039, -0.0061),
        ],
        radius=0.00055,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="browline_glasses")

    acetate = model.material("acetate_black", rgba=(0.10, 0.08, 0.07, 1.0))
    metal = model.material("gunmetal", rgba=(0.62, 0.63, 0.67, 1.0))
    lens_clear = model.material("lens_clear", rgba=(0.82, 0.89, 0.95, 0.22))
    pad_soft = model.material("nose_pad_soft", rgba=(0.92, 0.94, 0.97, 0.72))

    front = model.part("front")
    front.visual(
        _brow_piece_mesh(
            length=0.058,
            height=0.013,
            thickness=0.0046,
            center=(-LENS_CENTER_X, 0.0, 0.014),
        ),
        material=acetate,
        name="left_brow",
    )
    front.visual(
        _brow_piece_mesh(
            length=0.058,
            height=0.013,
            thickness=0.0046,
            center=(LENS_CENTER_X, 0.0, 0.014),
        ),
        material=acetate,
        name="right_brow",
    )
    front.visual(
        Box((0.020, 0.0048, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=acetate,
        name="bridge_bar",
    )
    front.visual(
        Cylinder(radius=0.0022, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="bridge_wire",
    )
    front.visual(
        _lower_rim_mesh(center_x=-LENS_CENTER_X, center_z=LENS_CENTER_Z, name="left_lower_rim_mesh"),
        material=metal,
        name="left_lower_rim",
    )
    front.visual(
        _lower_rim_mesh(center_x=LENS_CENTER_X, center_z=LENS_CENTER_Z, name="right_lower_rim_mesh"),
        material=metal,
        name="right_lower_rim",
    )
    front.visual(
        _lens_mesh(center_x=-LENS_CENTER_X, center_z=LENS_CENTER_Z, thickness=0.0012, name="left_lens_mesh"),
        material=lens_clear,
        name="left_lens",
    )
    front.visual(
        _lens_mesh(center_x=LENS_CENTER_X, center_z=LENS_CENTER_Z, thickness=0.0012, name="right_lens_mesh"),
        material=lens_clear,
        name="right_lens",
    )
    front.visual(
        Box((0.010, 0.0048, 0.012)),
        origin=Origin(xyz=(-0.0640, 0.0, HINGE_Z)),
        material=acetate,
        name="left_hinge_block",
    )
    front.visual(
        Box((0.010, 0.0048, 0.012)),
        origin=Origin(xyz=(0.0640, 0.0, HINGE_Z)),
        material=acetate,
        name="right_hinge_block",
    )
    front.visual(
        Cylinder(radius=0.0012, length=0.010),
        origin=Origin(xyz=(-HINGE_X, 0.0, HINGE_Z), rpy=(0.0, 0.0, 0.0)),
        material=metal,
        name="left_hinge_barrel",
    )
    front.visual(
        Cylinder(radius=0.0012, length=0.010),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(0.0, 0.0, 0.0)),
        material=metal,
        name="right_hinge_barrel",
    )

    left_bracket = tube_from_spline_points(
        [
            (-0.007, -0.0005, 0.006),
            (-0.009, -0.0035, 0.0005),
            (-0.011, -0.008, -0.006),
            (-NOSE_PIVOT_X, NOSE_PIVOT_Y, NOSE_PIVOT_Z),
        ],
        radius=0.00075,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    right_bracket = tube_from_spline_points(
        [
            (0.007, -0.0005, 0.006),
            (0.009, -0.0035, 0.0005),
            (0.011, -0.008, -0.006),
            (NOSE_PIVOT_X, NOSE_PIVOT_Y, NOSE_PIVOT_Z),
        ],
        radius=0.00075,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    front.visual(mesh_from_geometry(left_bracket, "left_nose_bracket_mesh"), material=metal, name="left_nose_bracket")
    front.visual(mesh_from_geometry(right_bracket, "right_nose_bracket_mesh"), material=metal, name="right_nose_bracket")
    front.visual(
        Cylinder(radius=0.0010, length=0.006),
        origin=Origin(
            xyz=(-NOSE_PIVOT_X, NOSE_PIVOT_Y, NOSE_PIVOT_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="left_pad_bracket_pivot",
    )
    front.visual(
        Cylinder(radius=0.0010, length=0.006),
        origin=Origin(
            xyz=(NOSE_PIVOT_X, NOSE_PIVOT_Y, NOSE_PIVOT_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="right_pad_bracket_pivot",
    )

    left_temple = model.part("left_temple")
    left_temple.visual(
        Box((0.0035, 0.008, 0.010)),
        origin=Origin(xyz=(-0.00175, -0.004, 0.0)),
        material=metal,
        name="left_hinge_plate",
    )
    left_temple.visual(
        _temple_body_mesh(sign=-1.0, name="left_temple_body_mesh"),
        material=acetate,
        name="left_temple_body",
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        Box((0.0035, 0.008, 0.010)),
        origin=Origin(xyz=(0.00175, -0.004, 0.0)),
        material=metal,
        name="right_hinge_plate",
    )
    right_temple.visual(
        _temple_body_mesh(sign=1.0, name="right_temple_body_mesh"),
        material=acetate,
        name="right_temple_body",
    )

    left_pad = model.part("left_nose_pad")
    left_pad.visual(
        Cylinder(radius=0.00065, length=0.004),
        origin=Origin(xyz=(0.0, -0.00165, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="left_pad_tab",
    )
    left_pad.visual(
        _nose_pad_arm_mesh(sign=-1.0, name="left_nose_pad_arm_mesh"),
        material=metal,
        name="left_pad_arm",
    )
    left_pad.visual(
        _nose_pad_mesh(sign=-1.0, name="left_nose_pad_mesh"),
        material=pad_soft,
        name="left_pad_surface",
    )

    right_pad = model.part("right_nose_pad")
    right_pad.visual(
        Cylinder(radius=0.00065, length=0.004),
        origin=Origin(xyz=(0.0, -0.00165, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="right_pad_tab",
    )
    right_pad.visual(
        _nose_pad_arm_mesh(sign=1.0, name="right_nose_pad_arm_mesh"),
        material=metal,
        name="right_pad_arm",
    )
    right_pad.visual(
        _nose_pad_mesh(sign=1.0, name="right_nose_pad_mesh"),
        material=pad_soft,
        name="right_pad_surface",
    )

    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=left_temple,
        origin=Origin(xyz=(-HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=2.0, velocity=2.5),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=right_temple,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=2.0, velocity=2.5),
    )
    model.articulation(
        "left_pad_pivot",
        ArticulationType.REVOLUTE,
        parent=front,
        child=left_pad,
        origin=Origin(xyz=(-NOSE_PIVOT_X, NOSE_PIVOT_Y, NOSE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.35, effort=0.4, velocity=1.0),
    )
    model.articulation(
        "right_pad_pivot",
        ArticulationType.REVOLUTE,
        parent=front,
        child=right_pad,
        origin=Origin(xyz=(NOSE_PIVOT_X, NOSE_PIVOT_Y, NOSE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.35, effort=0.4, velocity=1.0),
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

    front = object_model.get_part("front")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_pad = object_model.get_part("left_nose_pad")
    right_pad = object_model.get_part("right_nose_pad")

    left_hinge = object_model.get_articulation("left_temple_hinge")
    right_hinge = object_model.get_articulation("right_temple_hinge")
    left_pad_joint = object_model.get_articulation("left_pad_pivot")
    right_pad_joint = object_model.get_articulation("right_pad_pivot")

    ctx.check(
        "all glasses parts exist",
        all(part is not None for part in (front, left_temple, right_temple, left_pad, right_pad)),
        details="Missing one or more required glasses parts.",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            front,
            left_temple,
            elem_a="left_hinge_block",
            elem_b="left_hinge_plate",
            contact_tol=0.0005,
            name="left temple plate seats on brow hinge block",
        )
        ctx.expect_contact(
            front,
            right_temple,
            elem_a="right_hinge_block",
            elem_b="right_hinge_plate",
            contact_tol=0.0005,
            name="right temple plate seats on brow hinge block",
        )

    left_open = ctx.part_element_world_aabb(left_temple, elem="left_temple_body")
    right_open = ctx.part_element_world_aabb(right_temple, elem="right_temple_body")
    with ctx.pose({left_hinge: 1.35, right_hinge: 1.35}):
        left_folded = ctx.part_element_world_aabb(left_temple, elem="left_temple_body")
        right_folded = ctx.part_element_world_aabb(right_temple, elem="right_temple_body")
    ctx.check(
        "left temple folds inward over the front",
        left_open is not None
        and left_folded is not None
        and left_folded[1][0] > left_open[1][0] + 0.060,
        details=f"open={left_open}, folded={left_folded}",
    )
    ctx.check(
        "right temple folds inward over the front",
        right_open is not None
        and right_folded is not None
        and right_folded[0][0] < right_open[0][0] - 0.060,
        details=f"open={right_open}, folded={right_folded}",
    )

    left_pad_rest = ctx.part_element_world_aabb(left_pad, elem="left_pad_surface")
    right_pad_rest = ctx.part_element_world_aabb(right_pad, elem="right_pad_surface")
    with ctx.pose({left_pad_joint: 0.28, right_pad_joint: 0.28}):
        left_pad_turned = ctx.part_element_world_aabb(left_pad, elem="left_pad_surface")
        right_pad_turned = ctx.part_element_world_aabb(right_pad, elem="right_pad_surface")

    def _center_y(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    left_rest_y = _center_y(left_pad_rest)
    left_turn_y = _center_y(left_pad_turned)
    right_rest_y = _center_y(right_pad_rest)
    right_turn_y = _center_y(right_pad_turned)

    ctx.check(
        "left nose pad pivots on its bracket",
        left_rest_y is not None and left_turn_y is not None and left_turn_y > left_rest_y + 0.0015,
        details=f"rest_y={left_rest_y}, turned_y={left_turn_y}",
    )
    ctx.check(
        "right nose pad pivots on its bracket",
        right_rest_y is not None and right_turn_y is not None and right_turn_y > right_rest_y + 0.0015,
        details=f"rest_y={right_rest_y}, turned_y={right_turn_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
