from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


FRAME_WIRE_RADIUS = 0.00115
TEMPLE_WIRE_RADIUS = 0.00125
HINGE_BARREL_RADIUS = 0.0019
HINGE_BARREL_LENGTH = 0.005
NOSE_PIVOT_RADIUS = 0.0013
NOSE_PIVOT_LENGTH = 0.003


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _teardrop_profile() -> list[tuple[float, float]]:
    control_points = [
        (-0.022, 0.020),
        (-0.028, 0.013),
        (-0.031, 0.002),
        (-0.029, -0.011),
        (-0.022, -0.023),
        (-0.010, -0.030),
        (0.000, -0.032),
        (0.010, -0.030),
        (0.022, -0.023),
        (0.029, -0.011),
        (0.031, 0.002),
        (0.028, 0.013),
        (0.022, 0.020),
        (0.000, 0.024),
    ]
    return sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=10,
        closed=True,
    )


def _profile_points_3d(
    profile: list[tuple[float, float]],
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
    z_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + x_shift, y_shift, z + z_shift) for x, z in profile]


def _build_front_metal() -> MeshGeometry:
    profile = _teardrop_profile()
    left_center_x = 0.036
    right_center_x = -0.036
    lens_center_z = -0.004

    front = MeshGeometry()
    front.merge(
        tube_from_spline_points(
            _profile_points_3d(profile, x_shift=left_center_x, z_shift=lens_center_z),
            radius=FRAME_WIRE_RADIUS,
            samples_per_segment=8,
            closed_spline=True,
            cap_ends=False,
            radial_segments=16,
        )
    )
    front.merge(
        tube_from_spline_points(
            _profile_points_3d(profile, x_shift=right_center_x, z_shift=lens_center_z),
            radius=FRAME_WIRE_RADIUS,
            samples_per_segment=8,
            closed_spline=True,
            cap_ends=False,
            radial_segments=16,
        )
    )

    upper_bridge = [
        (-0.014, 0.0, 0.016),
        (-0.006, 0.0, 0.022),
        (0.000, 0.0, 0.024),
        (0.006, 0.0, 0.022),
        (0.014, 0.0, 0.016),
    ]
    lower_bridge = [
        (-0.008, -0.0005, 0.004),
        (-0.003, -0.0008, 0.009),
        (0.000, -0.0010, 0.010),
        (0.003, -0.0008, 0.009),
        (0.008, -0.0005, 0.004),
    ]
    front.merge(
        tube_from_spline_points(
            upper_bridge,
            radius=0.00095,
            samples_per_segment=16,
            radial_segments=14,
        )
    )
    front.merge(
        tube_from_spline_points(
            lower_bridge,
            radius=0.00110,
            samples_per_segment=16,
            radial_segments=14,
        )
    )

    for side in (1.0, -1.0):
        hinge_x = side * 0.069
        hinge_y = -0.0023
        hinge_z = 0.018

        front.merge(
            CylinderGeometry(
                radius=HINGE_BARREL_RADIUS,
                height=0.004,
                radial_segments=18,
            ).translate(hinge_x, hinge_y, hinge_z + 0.0045)
        )
        front.merge(
            CylinderGeometry(
                radius=HINGE_BARREL_RADIUS,
                height=0.004,
                radial_segments=18,
            ).translate(hinge_x, hinge_y, hinge_z - 0.0045)
        )
        front.merge(
            BoxGeometry((0.0045, 0.0022, 0.013)).translate(
                side * 0.0645,
                -0.0018,
                hinge_z,
            )
        )
        front.merge(
            tube_from_spline_points(
                [
                    (side * 0.058, -0.0005, 0.015),
                    (side * 0.063, -0.0015, 0.017),
                    (hinge_x, hinge_y, hinge_z),
                ],
                radius=0.0010,
                samples_per_segment=10,
                radial_segments=12,
            )
        )

        front.merge(
            SphereGeometry(0.0016).translate(side * 0.0105, -0.0015, 0.0048)
        )

    return front


def _build_lens_geometry() -> MeshGeometry:
    return ExtrudeGeometry(
        _teardrop_profile(),
        0.0014,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _build_temple_bar_geometry(side: float) -> MeshGeometry:
    temple = MeshGeometry()
    temple.merge(
        CylinderGeometry(
            radius=HINGE_BARREL_RADIUS,
            height=HINGE_BARREL_LENGTH,
            radial_segments=18,
        )
    )
    temple.merge(
        BoxGeometry((0.0036, 0.0022, 0.008)).translate(side * 0.0022, -0.0016, 0.0)
    )
    temple.merge(
        tube_from_spline_points(
            [
                (side * 0.0014, -0.0012, 0.0000),
                (side * 0.0045, -0.0250, 0.0010),
                (side * 0.0085, -0.0700, 0.0020),
                (side * 0.0095, -0.1120, -0.0030),
                (side * 0.0072, -0.1370, -0.0170),
                (side * 0.0045, -0.1310, -0.0310),
            ],
            radius=TEMPLE_WIRE_RADIUS,
            samples_per_segment=16,
            radial_segments=14,
        )
    )
    return temple


def _nose_arm_pivot_center(side: float) -> tuple[float, float, float]:
    return (side * 0.0070, -0.0072, -0.0115)


def _build_nose_pad_arm_geometry(side: float) -> MeshGeometry:
    arm = MeshGeometry()
    pivot_x, pivot_y, pivot_z = _nose_arm_pivot_center(side)

    arm.merge(SphereGeometry(0.0016))
    arm.merge(
        tube_from_spline_points(
            [
                (0.0000, -0.0003, 0.0000),
                (side * 0.0015, -0.0018, -0.0018),
                (side * 0.0038, -0.0042, -0.0058),
                (side * 0.0058, -0.0060, -0.0090),
                (pivot_x, pivot_y, pivot_z),
            ],
            radius=0.0008,
            samples_per_segment=12,
            radial_segments=12,
        )
    )
    arm.merge(
        CylinderGeometry(
            radius=NOSE_PIVOT_RADIUS,
            height=NOSE_PIVOT_LENGTH,
            radial_segments=16,
        ).translate(pivot_x, pivot_y, pivot_z + 0.0030)
    )
    arm.merge(
        CylinderGeometry(
            radius=NOSE_PIVOT_RADIUS,
            height=NOSE_PIVOT_LENGTH,
            radial_segments=16,
        ).translate(pivot_x, pivot_y, pivot_z - 0.0030)
    )
    return arm


def _build_nose_pad_geometry(side: float) -> MeshGeometry:
    pad = MeshGeometry()
    pad.merge(
        CylinderGeometry(
            radius=NOSE_PIVOT_RADIUS,
            height=NOSE_PIVOT_LENGTH,
            radial_segments=16,
        )
    )
    pad.merge(
        CylinderGeometry(
            radius=0.0009,
            height=0.0050,
            radial_segments=14,
        )
        .rotate_x(math.pi / 2.0)
        .translate(side * 0.0018, -0.0006, -0.0032)
    )
    pad.merge(
        ExtrudeGeometry(
            rounded_rect_profile(0.0088, 0.0136, 0.0038, corner_segments=6),
            0.0022,
            center=True,
        )
        .rotate_x(math.pi / 2.0)
        .rotate_z(side * 0.30)
        .translate(side * 0.0038, -0.0011, -0.0070)
    )
    return pad


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aviator_glasses")

    metal = model.material("metal", rgba=(0.83, 0.72, 0.46, 1.0))
    lens_tint = model.material("lens_tint", rgba=(0.30, 0.36, 0.30, 0.55))
    temple_tip = model.material("temple_tip", rgba=(0.20, 0.16, 0.12, 1.0))
    nose_pad = model.material("nose_pad", rgba=(0.90, 0.92, 0.93, 0.78))

    front = model.part("front")
    front.inertial = Inertial.from_geometry(
        Box((0.145, 0.024, 0.060)),
        mass=0.080,
        origin=Origin(xyz=(0.0, -0.001, -0.005)),
    )
    front.visual(
        _save_mesh("front_metal", _build_front_metal()),
        material=metal,
        name="front_metal",
    )
    left_lens_geom = _build_lens_geometry().translate(0.036, -0.0008, -0.004)
    right_lens_geom = _build_lens_geometry().translate(-0.036, -0.0008, -0.004)
    front.visual(
        _save_mesh("left_lens", left_lens_geom),
        material=lens_tint,
        name="left_lens",
    )
    front.visual(
        _save_mesh("right_lens", right_lens_geom),
        material=lens_tint,
        name="right_lens",
    )

    left_temple = model.part("left_temple")
    left_temple.inertial = Inertial.from_geometry(
        Box((0.020, 0.145, 0.036)),
        mass=0.014,
        origin=Origin(xyz=(0.006, -0.070, -0.010)),
    )
    left_temple.visual(
        _save_mesh("left_temple_bar", _build_temple_bar_geometry(1.0)),
        material=metal,
        name="bar",
    )
    left_temple.visual(
        Cylinder(radius=0.0022, length=0.026),
        origin=Origin(
            xyz=(0.0062, -0.132, -0.024),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=temple_tip,
        name="tip_sleeve",
    )

    right_temple = model.part("right_temple")
    right_temple.inertial = Inertial.from_geometry(
        Box((0.020, 0.145, 0.036)),
        mass=0.014,
        origin=Origin(xyz=(-0.006, -0.070, -0.010)),
    )
    right_temple.visual(
        _save_mesh("right_temple_bar", _build_temple_bar_geometry(-1.0)),
        material=metal,
        name="bar",
    )
    right_temple.visual(
        Cylinder(radius=0.0022, length=0.026),
        origin=Origin(
            xyz=(-0.0062, -0.132, -0.024),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=temple_tip,
        name="tip_sleeve",
    )

    left_nose_pad_arm = model.part("left_nose_pad_arm")
    left_nose_pad_arm.inertial = Inertial.from_geometry(
        Box((0.016, 0.012, 0.016)),
        mass=0.002,
        origin=Origin(xyz=(0.0035, -0.0035, -0.0055)),
    )
    left_nose_pad_arm.visual(
        _save_mesh("left_nose_pad_arm", _build_nose_pad_arm_geometry(1.0)),
        material=metal,
        name="arm",
    )

    right_nose_pad_arm = model.part("right_nose_pad_arm")
    right_nose_pad_arm.inertial = Inertial.from_geometry(
        Box((0.016, 0.012, 0.016)),
        mass=0.002,
        origin=Origin(xyz=(-0.0035, -0.0035, -0.0055)),
    )
    right_nose_pad_arm.visual(
        _save_mesh("right_nose_pad_arm", _build_nose_pad_arm_geometry(-1.0)),
        material=metal,
        name="arm",
    )

    left_nose_pad = model.part("left_nose_pad")
    left_nose_pad.inertial = Inertial.from_geometry(
        Box((0.010, 0.004, 0.016)),
        mass=0.001,
        origin=Origin(xyz=(0.003, -0.001, -0.006)),
    )
    left_nose_pad.visual(
        _save_mesh("left_nose_pad", _build_nose_pad_geometry(1.0)),
        material=nose_pad,
        name="pad_face",
    )

    right_nose_pad = model.part("right_nose_pad")
    right_nose_pad.inertial = Inertial.from_geometry(
        Box((0.010, 0.004, 0.016)),
        mass=0.001,
        origin=Origin(xyz=(-0.003, -0.001, -0.006)),
    )
    right_nose_pad.visual(
        _save_mesh("right_nose_pad", _build_nose_pad_geometry(-1.0)),
        material=nose_pad,
        name="pad_face",
    )

    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=left_temple,
        origin=Origin(xyz=(0.069, -0.0023, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-1.30,
            upper=0.10,
        ),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=right_temple,
        origin=Origin(xyz=(-0.069, -0.0023, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-0.10,
            upper=1.30,
        ),
    )
    model.articulation(
        "front_to_left_nose_pad_arm",
        ArticulationType.FIXED,
        parent=front,
        child=left_nose_pad_arm,
        origin=Origin(xyz=(0.0105, -0.0047, 0.0048)),
    )
    model.articulation(
        "front_to_right_nose_pad_arm",
        ArticulationType.FIXED,
        parent=front,
        child=right_nose_pad_arm,
        origin=Origin(xyz=(-0.0105, -0.0047, 0.0048)),
    )
    model.articulation(
        "left_nose_pad_swivel",
        ArticulationType.REVOLUTE,
        parent=left_nose_pad_arm,
        child=left_nose_pad,
        origin=Origin(xyz=_nose_arm_pivot_center(1.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.05,
            velocity=4.0,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "right_nose_pad_swivel",
        ArticulationType.REVOLUTE,
        parent=right_nose_pad_arm,
        child=right_nose_pad,
        origin=Origin(xyz=_nose_arm_pivot_center(-1.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.05,
            velocity=4.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_nose_pad_arm = object_model.get_part("left_nose_pad_arm")
    right_nose_pad_arm = object_model.get_part("right_nose_pad_arm")
    left_nose_pad = object_model.get_part("left_nose_pad")
    right_nose_pad = object_model.get_part("right_nose_pad")
    left_temple_hinge = object_model.get_articulation("left_temple_hinge")
    right_temple_hinge = object_model.get_articulation("right_temple_hinge")
    left_nose_pad_swivel = object_model.get_articulation("left_nose_pad_swivel")
    right_nose_pad_swivel = object_model.get_articulation("right_nose_pad_swivel")

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

    ctx.expect_contact(left_temple, front, name="left temple clipped into hinge")
    ctx.expect_contact(right_temple, front, name="right temple clipped into hinge")
    ctx.expect_contact(left_nose_pad_arm, front, name="left nose arm mounted to front")
    ctx.expect_contact(right_nose_pad_arm, front, name="right nose arm mounted to front")
    ctx.expect_contact(left_nose_pad, left_nose_pad_arm, name="left nose pad on pivot")
    ctx.expect_contact(right_nose_pad, right_nose_pad_arm, name="right nose pad on pivot")

    def _aabb_center(aabb):
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

    left_tip_rest = ctx.part_element_world_aabb(left_temple, elem="tip_sleeve")
    right_tip_rest = ctx.part_element_world_aabb(right_temple, elem="tip_sleeve")
    assert left_tip_rest is not None
    assert right_tip_rest is not None

    with ctx.pose({left_temple_hinge: -1.15, right_temple_hinge: 1.15}):
        left_tip_folded = ctx.part_element_world_aabb(left_temple, elem="tip_sleeve")
        right_tip_folded = ctx.part_element_world_aabb(right_temple, elem="tip_sleeve")
        assert left_tip_folded is not None
        assert right_tip_folded is not None
        left_tip_rest_center = _aabb_center(left_tip_rest)
        right_tip_rest_center = _aabb_center(right_tip_rest)
        left_tip_folded_center = _aabb_center(left_tip_folded)
        right_tip_folded_center = _aabb_center(right_tip_folded)
        ctx.check(
            "left temple folds inward",
            left_tip_folded_center[0] < left_tip_rest_center[0] - 0.07,
            details=(
                f"expected left tip x to move inward from {left_tip_rest_center[0]:.4f} "
                f"to below {left_tip_rest_center[0] - 0.07:.4f}, got {left_tip_folded_center[0]:.4f}"
            ),
        )
        ctx.check(
            "right temple folds inward",
            right_tip_folded_center[0] > right_tip_rest_center[0] + 0.07,
            details=(
                f"expected right tip x to move inward from {right_tip_rest_center[0]:.4f} "
                f"to above {right_tip_rest_center[0] + 0.07:.4f}, got {right_tip_folded_center[0]:.4f}"
            ),
        )
        ctx.expect_contact(left_temple, front, name="left temple stays retained when folded")
        ctx.expect_contact(right_temple, front, name="right temple stays retained when folded")

    left_pad_rest = ctx.part_element_world_aabb(left_nose_pad, elem="pad_face")
    right_pad_rest = ctx.part_element_world_aabb(right_nose_pad, elem="pad_face")
    assert left_pad_rest is not None
    assert right_pad_rest is not None

    with ctx.pose({left_nose_pad_swivel: 0.25, right_nose_pad_swivel: -0.25}):
        left_pad_swiveled = ctx.part_element_world_aabb(left_nose_pad, elem="pad_face")
        right_pad_swiveled = ctx.part_element_world_aabb(right_nose_pad, elem="pad_face")
        assert left_pad_swiveled is not None
        assert right_pad_swiveled is not None
        left_pad_rest_center = _aabb_center(left_pad_rest)
        right_pad_rest_center = _aabb_center(right_pad_rest)
        left_pad_swiveled_center = _aabb_center(left_pad_swiveled)
        right_pad_swiveled_center = _aabb_center(right_pad_swiveled)
        ctx.check(
            "left nose pad swivels",
            abs(left_pad_swiveled_center[1] - left_pad_rest_center[1]) > 0.0007,
            details=(
                f"left pad center y changed by "
                f"{abs(left_pad_swiveled_center[1] - left_pad_rest_center[1]):.5f} m"
            ),
        )
        ctx.check(
            "right nose pad swivels",
            abs(right_pad_swiveled_center[1] - right_pad_rest_center[1]) > 0.0007,
            details=(
                f"right pad center y changed by "
                f"{abs(right_pad_swiveled_center[1] - right_pad_rest_center[1]):.5f} m"
            ),
        )
        ctx.expect_contact(left_nose_pad, left_nose_pad_arm, name="left nose pad stays on its pivot")
        ctx.expect_contact(right_nose_pad, right_nose_pad_arm, name="right nose pad stays on its pivot")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
