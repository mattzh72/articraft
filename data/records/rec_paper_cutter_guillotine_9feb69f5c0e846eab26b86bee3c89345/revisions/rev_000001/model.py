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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BED_X = 0.38
BED_Y = 0.29
BED_Z = 0.014

CUT_YAW = math.radians(-38.0)
CUT_DIR = (math.cos(CUT_YAW), math.sin(CUT_YAW), 0.0)
CUT_NORMAL = (-math.sin(CUT_YAW), math.cos(CUT_YAW), 0.0)

PIVOT_ORIGIN = (-0.173, 0.118, 0.040)
CUT_LINE_CENTER = (-0.010, -0.006, 0.0)


def _diag_xyz(along: float = 0.0, normal: float = 0.0, z: float = 0.0) -> tuple[float, float, float]:
    return (
        CUT_DIR[0] * along + CUT_NORMAL[0] * normal,
        CUT_DIR[1] * along + CUT_NORMAL[1] * normal,
        z,
    )


def _shift(origin: tuple[float, float, float], *, along: float = 0.0, normal: float = 0.0, z: float = 0.0) -> tuple[float, float, float]:
    dx, dy, dz = _diag_xyz(along, normal, z)
    return (origin[0] + dx, origin[1] + dy, origin[2] + dz)


CUT_STRIP_ORIGIN = _shift(CUT_LINE_CENTER, normal=-0.004, z=0.01425)
GUARD_HINGE_ORIGIN = _shift(CUT_LINE_CENTER, along=0.045, normal=0.021, z=0.025)


def _circle_profile(radius: float, *, segments: int = 20) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _eyelet_mesh(name: str, *, outer_w: float, outer_h: float, hole_radius: float, depth: float):
    outer = rounded_rect_profile(
        outer_w,
        outer_h,
        radius=min(outer_w, outer_h) * 0.16,
        corner_segments=6,
    )
    geom = ExtrudeWithHolesGeometry(
        outer,
        [_circle_profile(hole_radius, segments=24)],
        height=depth,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def _axis_rpy(yaw: float) -> tuple[float, float, float]:
    return (0.0, math.pi / 2.0, yaw)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="craft_guillotine_cutter")

    board = model.material("board", rgba=(0.90, 0.91, 0.88, 1.0))
    board_trim = model.material("board_trim", rgba=(0.76, 0.77, 0.74, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.81, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.84, 0.85, 0.87, 1.0))
    clear_guard = model.material("clear_guard", rgba=(0.80, 0.92, 0.98, 0.33))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        Box((BED_X, BED_Y, BED_Z)),
        origin=Origin(xyz=(0.0, 0.0, BED_Z * 0.5)),
        material=board,
        name="bed",
    )
    base.visual(
        Box((BED_X - 0.020, BED_Y - 0.022, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, BED_Z + 0.001)),
        material=board_trim,
        name="bed_skin",
    )
    base.visual(
        Box((0.314, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, BED_Y * 0.5 - 0.006, BED_Z + 0.012)),
        material=dark_plastic,
        name="rear_fence",
    )
    base.visual(
        Box((0.012, 0.210, 0.024)),
        origin=Origin(xyz=(-BED_X * 0.5 + 0.006, 0.024, BED_Z + 0.012)),
        material=dark_plastic,
        name="side_fence",
    )
    base.visual(
        Box((0.324, 0.008, 0.0015)),
        origin=Origin(xyz=CUT_STRIP_ORIGIN, rpy=(0.0, 0.0, CUT_YAW)),
        material=board_trim,
        name="cutting_strip",
    )
    base.visual(
        Box((0.032, 0.044, 0.018)),
        origin=Origin(
            xyz=(PIVOT_ORIGIN[0], PIVOT_ORIGIN[1], PIVOT_ORIGIN[2] - 0.025),
        ),
        material=dark_plastic,
        name="pivot_pedestal",
    )
    base.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(xyz=PIVOT_ORIGIN, rpy=_axis_rpy(0.0)),
        material=steel,
        name="pivot_rod",
    )
    for name, x_offset in (("pivot_left_collar", -0.010), ("pivot_right_collar", 0.010)):
        base.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(
                xyz=(PIVOT_ORIGIN[0] + x_offset, PIVOT_ORIGIN[1], PIVOT_ORIGIN[2]),
                rpy=_axis_rpy(0.0),
            ),
            material=steel,
            name=name,
        )
    base.visual(
        Box((0.190, 0.014, 0.005)),
        origin=Origin(
            xyz=_shift(GUARD_HINGE_ORIGIN, normal=-0.010, z=-0.010),
            rpy=(0.0, 0.0, CUT_YAW),
        ),
        material=board_trim,
        name="guard_rail",
    )
    base.visual(
        Cylinder(radius=0.0035, length=0.212),
        origin=Origin(xyz=GUARD_HINGE_ORIGIN, rpy=_axis_rpy(CUT_YAW)),
        material=steel,
        name="guard_rod",
    )
    for name, along_pos in (("guard_post_a", -0.102), ("guard_post_b", 0.102)):
        base.visual(
            Box((0.014, 0.014, 0.018)),
            origin=Origin(
                xyz=_shift(GUARD_HINGE_ORIGIN, along=along_pos, normal=-0.010, z=-0.007),
                rpy=(0.0, 0.0, CUT_YAW),
            ),
            material=dark_plastic,
            name=name,
        )
    guard_collar_positions = {
        "rear_clip_outer_collar": -0.076,
        "rear_clip_inner_collar": -0.060,
        "front_clip_inner_collar": 0.060,
        "front_clip_outer_collar": 0.076,
    }
    for name, along_pos in guard_collar_positions.items():
        base.visual(
            Cylinder(radius=0.0062, length=0.004),
            origin=Origin(
                xyz=_shift(GUARD_HINGE_ORIGIN, along=along_pos),
                rpy=_axis_rpy(CUT_YAW),
            ),
            material=steel,
            name=name,
        )
    for x_sign, y_sign, name in (
        (-1.0, -1.0, "foot_back_left"),
        (1.0, -1.0, "foot_back_right"),
        (-1.0, 1.0, "foot_front_left"),
        (1.0, 1.0, "foot_front_right"),
    ):
        base.visual(
            Box((0.020, 0.016, 0.004)),
            origin=Origin(
                xyz=(x_sign * 0.150, y_sign * 0.105, -0.002),
            ),
            material=rubber,
            name=name,
        )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.31, 0.05)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    blade_arm = model.part("blade_arm")
    blade_eyelet = _eyelet_mesh(
        "blade_arm_eyelet",
        outer_w=0.030,
        outer_h=0.030,
        hole_radius=0.006,
        depth=0.016,
    )
    blade_arm.visual(
        blade_eyelet,
        origin=Origin(rpy=_axis_rpy(0.0)),
        material=dark_plastic,
        name="pivot_eyelet",
    )
    blade_arm.visual(
        Box((0.330, 0.028, 0.012)),
        origin=Origin(xyz=_diag_xyz(0.205, -0.002, -0.004), rpy=(0.0, 0.0, CUT_YAW)),
        material=dark_plastic,
        name="arm_spine",
    )
    blade_arm.visual(
        Box((0.074, 0.028, 0.008)),
        origin=Origin(xyz=_diag_xyz(0.048, -0.002, -0.014), rpy=(0.0, 0.0, CUT_YAW)),
        material=dark_plastic,
        name="pivot_reinforcement",
    )
    blade_arm.visual(
        Box((0.330, 0.006, 0.015)),
        origin=Origin(xyz=_diag_xyz(0.186, -0.010, -0.014), rpy=(0.0, 0.0, CUT_YAW)),
        material=steel,
        name="blade_backer",
    )
    blade_arm.visual(
        Box((0.330, 0.004, 0.003)),
        origin=Origin(xyz=_diag_xyz(0.186, -0.015, -0.023), rpy=(0.0, 0.0, CUT_YAW)),
        material=blade_steel,
        name="blade_edge",
    )
    blade_arm.visual(
        Box((0.094, 0.028, 0.022)),
        origin=Origin(xyz=_diag_xyz(0.302, -0.001, 0.006), rpy=(0.0, 0.0, CUT_YAW)),
        material=dark_plastic,
        name="handle_pad",
    )
    blade_arm.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(
            xyz=_diag_xyz(0.302, -0.001, 0.017),
            rpy=(math.pi / 2.0, 0.0, CUT_YAW),
        ),
        material=rubber,
        name="handle_grip",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.42, 0.06, 0.05)),
        mass=0.55,
        origin=Origin(xyz=_diag_xyz(0.205, -0.002, -0.008)),
    )

    finger_guard = model.part("finger_guard")
    guard_clip = _eyelet_mesh(
        "finger_guard_clip",
        outer_w=0.018,
        outer_h=0.018,
        hole_radius=0.0068,
        depth=0.012,
    )
    for name, along_pos in (("rear_clip", -0.068), ("front_clip", 0.068)):
        finger_guard.visual(
            guard_clip,
            origin=Origin(xyz=_diag_xyz(along_pos, 0.0, 0.0), rpy=_axis_rpy(CUT_YAW)),
            material=dark_plastic,
            name=name,
        )
    finger_guard.visual(
        Box((0.146, 0.014, 0.006)),
        origin=Origin(xyz=_diag_xyz(0.0, 0.015, -0.004), rpy=(0.0, 0.0, CUT_YAW)),
        material=dark_plastic,
        name="guard_frame_top",
    )
    finger_guard.visual(
        Box((0.156, 0.038, 0.0018)),
        origin=Origin(xyz=_diag_xyz(0.0, 0.032, -0.004), rpy=(0.0, 0.0, CUT_YAW)),
        material=clear_guard,
        name="guard_panel",
    )
    finger_guard.visual(
        Box((0.156, 0.004, 0.004)),
        origin=Origin(xyz=_diag_xyz(0.0, 0.051, -0.005), rpy=(0.0, 0.0, CUT_YAW)),
        material=dark_plastic,
        name="guard_frame_bottom",
    )
    for name, along_pos in (("guard_end_cap_a", -0.078), ("guard_end_cap_b", 0.078)):
        finger_guard.visual(
            Box((0.006, 0.043, 0.006)),
            origin=Origin(xyz=_diag_xyz(along_pos, 0.032, -0.004), rpy=(0.0, 0.0, CUT_YAW)),
            material=dark_plastic,
            name=name,
        )
    finger_guard.inertial = Inertial.from_geometry(
        Box((0.30, 0.06, 0.02)),
        mass=0.16,
        origin=Origin(xyz=_diag_xyz(0.0, 0.020, -0.004)),
    )

    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=PIVOT_ORIGIN),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "base_to_finger_guard",
        ArticulationType.REVOLUTE,
        parent=base,
        child=finger_guard,
        origin=Origin(xyz=GUARD_HINGE_ORIGIN),
        axis=CUT_DIR,
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    finger_guard = object_model.get_part("finger_guard")
    blade_hinge = object_model.get_articulation("base_to_blade_arm")
    guard_hinge = object_model.get_articulation("base_to_finger_guard")

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

    ctx.check(
        "blade_hinge_axis_orientation",
        blade_hinge.axis == (-1.0, 0.0, 0.0),
        f"Expected blade arm hinge axis (-1, 0, 0), got {blade_hinge.axis}.",
    )
    ctx.check(
        "guard_hinge_axis_alignment",
        all(abs(a - b) < 1e-6 for a, b in zip(guard_hinge.axis, CUT_DIR)),
        f"Expected guard hinge axis {CUT_DIR}, got {guard_hinge.axis}.",
    )

    ctx.expect_contact(blade_arm, base, elem_a="pivot_eyelet", elem_b="pivot_left_collar")
    ctx.expect_contact(blade_arm, base, elem_a="pivot_eyelet", elem_b="pivot_right_collar")
    ctx.expect_contact(
        finger_guard,
        base,
        elem_a="rear_clip",
        elem_b="rear_clip_inner_collar",
        contact_tol=0.001,
    )
    ctx.expect_contact(
        finger_guard,
        base,
        elem_a="front_clip",
        elem_b="front_clip_outer_collar",
        contact_tol=0.001,
    )

    ctx.expect_overlap(
        blade_arm,
        base,
        axes="xy",
        min_overlap=0.18,
        elem_a="blade_edge",
        elem_b="cutting_strip",
    )
    ctx.expect_gap(
        blade_arm,
        base,
        axis="z",
        min_gap=0.0002,
        max_gap=0.0012,
        positive_elem="blade_edge",
        negative_elem="cutting_strip",
    )
    ctx.expect_gap(
        finger_guard,
        base,
        axis="z",
        min_gap=0.0035,
        max_gap=0.0080,
        positive_elem="guard_panel",
        negative_elem="bed",
    )

    rest_handle = ctx.part_element_world_aabb(blade_arm, elem="handle_pad")
    rest_guard = ctx.part_element_world_aabb(finger_guard, elem="guard_panel")
    assert rest_handle is not None
    assert rest_guard is not None

    with ctx.pose({blade_hinge: math.radians(65.0)}):
        raised_handle = ctx.part_element_world_aabb(blade_arm, elem="handle_pad")
        assert raised_handle is not None
        ctx.check(
            "blade_arm_raises_above_bed",
            raised_handle[1][2] > rest_handle[1][2] + 0.14,
            f"Raised handle z-max {raised_handle[1][2]:.4f} did not rise enough above rest {rest_handle[1][2]:.4f}.",
        )
        ctx.expect_contact(blade_arm, base, elem_a="pivot_eyelet", elem_b="pivot_left_collar")

    with ctx.pose({guard_hinge: math.radians(95.0)}):
        raised_guard = ctx.part_element_world_aabb(finger_guard, elem="guard_panel")
        assert raised_guard is not None
        ctx.check(
            "finger_guard_flips_up",
            raised_guard[1][2] > rest_guard[1][2] + 0.05,
            f"Raised guard z-max {raised_guard[1][2]:.4f} did not rise enough above rest {rest_guard[1][2]:.4f}.",
        )
        ctx.expect_contact(
            finger_guard,
            base,
            elem_a="front_clip",
            elem_b="front_clip_outer_collar",
            contact_tol=0.001,
        )
        ctx.expect_contact(
            finger_guard,
            base,
            elem_a="rear_clip",
            elem_b="rear_clip_inner_collar",
            contact_tol=0.001,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
