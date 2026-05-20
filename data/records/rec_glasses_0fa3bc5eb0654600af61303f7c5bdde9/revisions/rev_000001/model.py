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
    superellipse_profile,
    tube_from_spline_points,
)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    samples: int = 16,
    radial_segments: int = 18,
    cap_ends: bool = True,
):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=radial_segments,
            cap_ends=cap_ends,
        ),
        name,
    )


def _lens_mesh(name: str, width: float, height: float, thickness: float):
    profile = rounded_rect_profile(
        width,
        height,
        radius=min(width, height) * 0.24,
        corner_segments=10,
    )
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


def _nose_pad_mesh(name: str, width: float, height: float, thickness: float):
    profile = superellipse_profile(width, height, exponent=2.5, segments=44)
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="half_rim_reading_glasses")

    metal = model.material("metal", rgba=(0.46, 0.48, 0.52, 1.0))
    dark_tip = model.material("dark_tip", rgba=(0.14, 0.14, 0.15, 1.0))
    lens_clear = model.material("lens_clear", rgba=(0.85, 0.93, 0.98, 0.28))
    nose_pad_soft = model.material("nose_pad_soft", rgba=(0.93, 0.93, 0.89, 0.92))

    lens_width = 0.048
    lens_height = 0.031
    lens_thickness = 0.0016
    lens_center_z = -0.003
    lens_center_x = 0.0315
    hinge_x = 0.063
    hinge_axis_z = 0.009
    nose_pivot_x = 0.0105
    nose_pivot_y = -0.0009
    nose_pivot_z = 0.004

    frame_front = model.part("frame_front")
    frame_front.inertial = Inertial.from_geometry(
        Box((0.136, 0.018, 0.042)),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    brow_bar_points = [
        (-0.064, 0.0, 0.0145),
        (-0.051, 0.0, 0.0150),
        (-0.034, 0.0, 0.0138),
        (-0.012, 0.0, 0.0124),
        (0.000, 0.0, 0.0110),
        (0.012, 0.0, 0.0124),
        (0.034, 0.0, 0.0138),
        (0.051, 0.0, 0.0150),
        (0.064, 0.0, 0.0145),
    ]
    frame_front.visual(
        _tube_mesh(
            "brow_bar",
            brow_bar_points,
            radius=0.0012,
            samples=18,
            radial_segments=20,
        ),
        material=metal,
        name="brow_bar",
    )

    bridge_saddle_points = [
        (-0.0085, -0.0005, 0.0102),
        (-0.0045, -0.0014, 0.0072),
        (0.0045, -0.0014, 0.0072),
        (0.0085, -0.0005, 0.0102),
    ]
    frame_front.visual(
        _tube_mesh(
            "bridge_saddle",
            bridge_saddle_points,
            radius=0.0008,
            samples=16,
            radial_segments=16,
        ),
        material=metal,
        name="bridge_saddle",
    )

    left_support_points = [
        (-0.0558, 0.0, 0.0102),
        (-0.0560, 0.0, 0.0015),
        (-0.0500, 0.0, -0.0115),
        (-0.0410, 0.0, -0.0186),
        (-0.0315, 0.0, -0.0198),
        (-0.0210, 0.0, -0.0184),
        (-0.0130, 0.0, -0.0098),
        (-0.0088, 0.0, 0.0088),
    ]
    frame_front.visual(
        _tube_mesh(
            "left_lower_rim",
            left_support_points,
            radius=0.0008,
            samples=18,
            radial_segments=18,
        ),
        material=metal,
        name="left_lower_rim",
    )
    frame_front.visual(
        _tube_mesh(
            "right_lower_rim",
            _mirror_x(left_support_points),
            radius=0.0008,
            samples=18,
            radial_segments=18,
        ),
        material=metal,
        name="right_lower_rim",
    )

    left_hinge_drop_points = [
        (-0.0565, 0.0, 0.0100),
        (-0.0605, 0.0, 0.0080),
        (-0.0630, 0.0, 0.0050),
    ]
    frame_front.visual(
        _tube_mesh(
            "left_hinge_drop",
            left_hinge_drop_points,
            radius=0.00075,
            samples=10,
            radial_segments=16,
        ),
        material=metal,
        name="left_hinge_drop",
    )
    frame_front.visual(
        _tube_mesh(
            "right_hinge_drop",
            _mirror_x(left_hinge_drop_points),
            radius=0.00075,
            samples=10,
            radial_segments=16,
        ),
        material=metal,
        name="right_hinge_drop",
    )

    left_bracket_points = [
        (-0.0065, 0.0, 0.0104),
        (-0.0088, -0.0004, 0.0080),
        (-0.0105, -0.0006, 0.0062),
        (-0.0105, -0.0002, 0.0040),
    ]
    frame_front.visual(
        _tube_mesh(
            "left_pad_bracket",
            left_bracket_points,
            radius=0.00065,
            samples=10,
            radial_segments=14,
        ),
        material=metal,
        name="left_pad_bracket",
    )
    frame_front.visual(
        _tube_mesh(
            "right_pad_bracket",
            _mirror_x(left_bracket_points),
            radius=0.00065,
            samples=10,
            radial_segments=14,
        ),
        material=metal,
        name="right_pad_bracket",
    )

    frame_front.visual(
        _lens_mesh("left_lens_mesh", lens_width, lens_height, lens_thickness),
        origin=Origin(
            xyz=(-lens_center_x, 0.0, lens_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=lens_clear,
        name="left_lens",
    )
    frame_front.visual(
        _lens_mesh("right_lens_mesh", lens_width, lens_height, lens_thickness),
        origin=Origin(
            xyz=(lens_center_x, 0.0, lens_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=lens_clear,
        name="right_lens",
    )

    for side, sign in (("left", -1.0), ("right", 1.0)):
        frame_front.visual(
            Cylinder(radius=0.00165, length=0.004),
            origin=Origin(xyz=(sign * hinge_x, 0.0, 0.013)),
            material=metal,
            name=f"{side}_upper_hinge_barrel",
        )
        frame_front.visual(
            Cylinder(radius=0.00165, length=0.004),
            origin=Origin(xyz=(sign * hinge_x, 0.0, 0.005)),
            material=metal,
            name=f"{side}_lower_hinge_barrel",
        )
        frame_front.visual(
            Cylinder(radius=0.0011, length=0.0012),
            origin=Origin(
                xyz=(sign * nose_pivot_x, -0.0002, nose_pivot_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name=f"{side}_pad_boss",
        )

    left_temple = model.part("left_temple")
    left_temple.inertial = Inertial.from_geometry(
        Box((0.018, 0.150, 0.028)),
        mass=0.004,
        origin=Origin(xyz=(-0.006, -0.073, -0.008)),
    )
    left_temple.visual(
        Cylinder(radius=0.0016, length=0.004),
        material=metal,
        name="left_hinge_knuckle",
    )
    left_temple_points = [
        (0.0000, 0.0000, 0.0000),
        (-0.0007, -0.0012, 0.0000),
        (-0.0010, -0.0030, 0.0000),
        (-0.0055, -0.0240, 0.0018),
        (-0.0100, -0.0620, 0.0025),
        (-0.0090, -0.1040, -0.0025),
        (-0.0060, -0.1350, -0.0135),
        (-0.0035, -0.1470, -0.0210),
    ]
    left_temple.visual(
        _tube_mesh(
            "left_temple_wire",
            left_temple_points,
            radius=0.0010,
            samples=20,
            radial_segments=18,
        ),
        material=metal,
        name="left_temple_wire",
    )
    left_temple.visual(
        _tube_mesh(
            "left_tip_sleeve",
            left_temple_points[3:],
            radius=0.00175,
            samples=18,
            radial_segments=18,
        ),
        material=dark_tip,
        name="left_tip_sleeve",
    )

    right_temple = model.part("right_temple")
    right_temple.inertial = Inertial.from_geometry(
        Box((0.018, 0.150, 0.028)),
        mass=0.004,
        origin=Origin(xyz=(0.006, -0.073, -0.008)),
    )
    right_temple.visual(
        Cylinder(radius=0.0016, length=0.004),
        material=metal,
        name="right_hinge_knuckle",
    )
    right_temple_points = _mirror_x(left_temple_points)
    right_temple.visual(
        _tube_mesh(
            "right_temple_wire",
            right_temple_points,
            radius=0.0010,
            samples=20,
            radial_segments=18,
        ),
        material=metal,
        name="right_temple_wire",
    )
    right_temple.visual(
        _tube_mesh(
            "right_tip_sleeve",
            right_temple_points[3:],
            radius=0.00175,
            samples=18,
            radial_segments=18,
        ),
        material=dark_tip,
        name="right_tip_sleeve",
    )

    left_nose_pad = model.part("left_nose_pad")
    left_nose_pad.inertial = Inertial.from_geometry(
        Box((0.012, 0.013, 0.016)),
        mass=0.001,
        origin=Origin(xyz=(0.003, -0.006, -0.006)),
    )
    left_nose_pad.visual(
        Cylinder(radius=0.0011, length=0.0008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_pad_pivot_collar",
    )
    left_nose_pad_points = [
        (0.0000, -0.0004, 0.0000),
        (0.0016, -0.0032, -0.0020),
        (0.0034, -0.0065, -0.0058),
        (0.0048, -0.0087, -0.0090),
    ]
    left_nose_pad.visual(
        _tube_mesh(
            "left_pad_arm",
            left_nose_pad_points,
            radius=0.00055,
            samples=14,
            radial_segments=14,
        ),
        material=metal,
        name="left_pad_arm",
    )
    left_nose_pad.visual(
        _nose_pad_mesh("left_pad_mesh", 0.0078, 0.0135, 0.0018),
        origin=Origin(
            xyz=(0.0048, -0.0100, -0.0094),
            rpy=(-math.pi / 2.0, 0.0, 0.16),
        ),
        material=nose_pad_soft,
        name="left_pad_cushion",
    )

    right_nose_pad = model.part("right_nose_pad")
    right_nose_pad.inertial = Inertial.from_geometry(
        Box((0.012, 0.013, 0.016)),
        mass=0.001,
        origin=Origin(xyz=(-0.003, -0.006, -0.006)),
    )
    right_nose_pad.visual(
        Cylinder(radius=0.0011, length=0.0008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_pad_pivot_collar",
    )
    right_nose_pad_points = _mirror_x(left_nose_pad_points)
    right_nose_pad.visual(
        _tube_mesh(
            "right_pad_arm",
            right_nose_pad_points,
            radius=0.00055,
            samples=14,
            radial_segments=14,
        ),
        material=metal,
        name="right_pad_arm",
    )
    right_nose_pad.visual(
        _nose_pad_mesh("right_pad_mesh", 0.0078, 0.0135, 0.0018),
        origin=Origin(
            xyz=(-0.0048, -0.0100, -0.0094),
            rpy=(-math.pi / 2.0, 0.0, -0.16),
        ),
        material=nose_pad_soft,
        name="right_pad_cushion",
    )

    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=frame_front,
        child=left_temple,
        origin=Origin(xyz=(-hinge_x, 0.0, hinge_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=frame_front,
        child=right_temple,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-1.65,
            upper=0.0,
        ),
    )
    model.articulation(
        "left_nose_pad_pivot",
        ArticulationType.REVOLUTE,
        parent=frame_front,
        child=left_nose_pad,
        origin=Origin(xyz=(-nose_pivot_x, nose_pivot_y, nose_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.1,
            velocity=3.0,
            lower=-0.45,
            upper=0.35,
        ),
    )
    model.articulation(
        "right_nose_pad_pivot",
        ArticulationType.REVOLUTE,
        parent=frame_front,
        child=right_nose_pad,
        origin=Origin(xyz=(nose_pivot_x, nose_pivot_y, nose_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.1,
            velocity=3.0,
            lower=-0.35,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame_front = object_model.get_part("frame_front")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_nose_pad = object_model.get_part("left_nose_pad")
    right_nose_pad = object_model.get_part("right_nose_pad")

    left_temple_hinge = object_model.get_articulation("left_temple_hinge")
    right_temple_hinge = object_model.get_articulation("right_temple_hinge")
    left_nose_pad_pivot = object_model.get_articulation("left_nose_pad_pivot")
    right_nose_pad_pivot = object_model.get_articulation("right_nose_pad_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_temple, frame_front, name="left_temple_attached_to_frame")
    ctx.expect_contact(right_temple, frame_front, name="right_temple_attached_to_frame")
    ctx.expect_contact(left_nose_pad, frame_front, name="left_nose_pad_attached_to_frame")
    ctx.expect_contact(right_nose_pad, frame_front, name="right_nose_pad_attached_to_frame")

    def _aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    with ctx.pose({left_temple_hinge: 0.0, right_temple_hinge: 0.0}):
        left_tip_open = ctx.part_element_world_aabb(left_temple, elem="left_tip_sleeve")
        right_tip_open = ctx.part_element_world_aabb(right_temple, elem="right_tip_sleeve")
        assert left_tip_open is not None
        assert right_tip_open is not None
        left_tip_open_center = _aabb_center(left_tip_open)
        right_tip_open_center = _aabb_center(right_tip_open)
        ctx.check(
            "left_temple_open_runs_back",
            left_tip_open_center[1] < -0.070,
            f"left temple tip should sit well behind frame in open pose, got y={left_tip_open_center[1]:.4f}",
        )
        ctx.check(
            "right_temple_open_runs_back",
            right_tip_open_center[1] < -0.070,
            f"right temple tip should sit well behind frame in open pose, got y={right_tip_open_center[1]:.4f}",
        )
        ctx.expect_gap(
            frame_front,
            left_nose_pad,
            axis="y",
            negative_elem="left_pad_cushion",
            min_gap=0.006,
            max_gap=0.016,
            name="left_nose_pad_sits_behind_bridge",
        )
        ctx.expect_gap(
            frame_front,
            right_nose_pad,
            axis="y",
            negative_elem="right_pad_cushion",
            min_gap=0.006,
            max_gap=0.016,
            name="right_nose_pad_sits_behind_bridge",
        )

    with ctx.pose({left_temple_hinge: 1.55, right_temple_hinge: -1.55}):
        left_tip_aabb = ctx.part_element_world_aabb(left_temple, elem="left_tip_sleeve")
        right_tip_aabb = ctx.part_element_world_aabb(right_temple, elem="right_tip_sleeve")
        assert left_tip_aabb is not None
        assert right_tip_aabb is not None
        left_tip_center = _aabb_center(left_tip_aabb)
        right_tip_center = _aabb_center(right_tip_aabb)
        ctx.check(
            "left_temple_folds_inboard",
            left_tip_center[0] > 0.0,
            f"folded left temple tip should cross toward centerline, got x={left_tip_center[0]:.4f}",
        )
        ctx.check(
            "right_temple_folds_inboard",
            right_tip_center[0] < 0.0,
            f"folded right temple tip should cross toward centerline, got x={right_tip_center[0]:.4f}",
        )
        ctx.expect_contact(left_temple, frame_front, name="left_temple_keeps_hinge_contact_folded")
        ctx.expect_contact(right_temple, frame_front, name="right_temple_keeps_hinge_contact_folded")

    left_pad_rest = ctx.part_element_world_aabb(left_nose_pad, elem="left_pad_cushion")
    right_pad_rest = ctx.part_element_world_aabb(right_nose_pad, elem="right_pad_cushion")
    assert left_pad_rest is not None
    assert right_pad_rest is not None
    left_pad_rest_center = _aabb_center(left_pad_rest)
    right_pad_rest_center = _aabb_center(right_pad_rest)

    with ctx.pose({left_nose_pad_pivot: -0.30, right_nose_pad_pivot: 0.30}):
        left_pad_turned = ctx.part_element_world_aabb(left_nose_pad, elem="left_pad_cushion")
        right_pad_turned = ctx.part_element_world_aabb(right_nose_pad, elem="right_pad_cushion")
        assert left_pad_turned is not None
        assert right_pad_turned is not None
        left_pad_turned_center = _aabb_center(left_pad_turned)
        right_pad_turned_center = _aabb_center(right_pad_turned)
        ctx.check(
            "left_nose_pad_rotates_inward",
            abs(left_pad_turned_center[0]) < abs(left_pad_rest_center[0]) - 0.001,
            (
                "left pad should move inward toward bridge when pivoted; "
                f"rest x={left_pad_rest_center[0]:.4f}, turned x={left_pad_turned_center[0]:.4f}"
            ),
        )
        ctx.check(
            "right_nose_pad_rotates_inward",
            abs(right_pad_turned_center[0]) < abs(right_pad_rest_center[0]) - 0.001,
            (
                "right pad should move inward toward bridge when pivoted; "
                f"rest x={right_pad_rest_center[0]:.4f}, turned x={right_pad_turned_center[0]:.4f}"
            ),
        )
        ctx.expect_contact(left_nose_pad, frame_front, name="left_pad_keeps_pivot_contact")
        ctx.expect_contact(right_nose_pad, frame_front, name="right_pad_keeps_pivot_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
