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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_base_shell_mesh():
    return LatheGeometry(
        [
            (0.000, 0.000),
            (0.078, 0.000),
            (0.094, 0.0025),
            (0.106, 0.0070),
            (0.108, 0.0140),
            (0.097, 0.0190),
            (0.070, 0.0220),
            (0.000, 0.0220),
        ],
        segments=72,
    )


def _build_head_shell_mesh():
    outer_profile = [
        (0.038, -0.072),
        (0.050, -0.064),
        (0.057, -0.020),
        (0.060, 0.028),
        (0.059, 0.064),
        (0.054, 0.082),
        (0.049, 0.090),
    ]
    inner_profile = [
        (0.031, -0.064),
        (0.043, -0.056),
        (0.050, -0.018),
        (0.053, 0.030),
        (0.051, 0.062),
        (0.046, 0.082),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_yoke_frame_mesh():
    return tube_from_spline_points(
        [
            (-0.098, 0.000, 0.246),
            (-0.094, 0.000, 0.206),
            (-0.074, 0.000, 0.170),
            (-0.048, 0.000, 0.132),
            (-0.024, 0.000, 0.118),
            (0.024, 0.000, 0.118),
            (0.048, 0.000, 0.132),
            (0.074, 0.000, 0.170),
            (0.094, 0.000, 0.206),
            (0.098, 0.000, 0.246),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_studio_spotlight")

    stand_paint = model.material("stand_paint", rgba=(0.26, 0.28, 0.31, 1.0))
    head_paint = model.material("head_paint", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.11, 0.12, 0.13, 1.0))
    elastomer = model.material("elastomer", rgba=(0.17, 0.17, 0.18, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.35, 0.37, 0.40, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.84, 0.90, 0.96, 0.35))

    base_shell_mesh = _save_mesh("studio_spot_base_shell", _build_base_shell_mesh())
    yoke_frame_mesh = _save_mesh("studio_spot_yoke_frame", _build_yoke_frame_mesh())
    lock_grip_mesh = _save_mesh(
        "studio_spot_lock_grip",
        TorusGeometry(radius=0.014, tube=0.003, radial_segments=14, tubular_segments=36),
    )
    head_shell_mesh = _save_mesh("studio_spot_head_shell", _build_head_shell_mesh())
    front_bezel_mesh = _save_mesh(
        "studio_spot_front_bezel",
        TorusGeometry(radius=0.051, tube=0.003, radial_segments=14, tubular_segments=40),
    )

    base_yoke = model.part("base_yoke")
    base_yoke.visual(base_shell_mesh, material=stand_paint, name="base_shell")
    base_yoke.visual(
        Cylinder(radius=0.086, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=elastomer,
        name="foot_pad",
    )
    base_yoke.visual(
        Cylinder(radius=0.036, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=stand_paint,
        name="stem_collar",
    )
    base_yoke.visual(
        Cylinder(radius=0.021, length=0.091),
        origin=Origin(xyz=(0.0, 0.0, 0.0735)),
        material=stand_paint,
        name="stem",
    )
    base_yoke.visual(yoke_frame_mesh, material=stand_paint, name="yoke_frame")
    base_yoke.visual(
        Box((0.014, 0.028, 0.046)),
        origin=Origin(xyz=(-0.097, 0.0, 0.255)),
        material=stand_paint,
        name="left_yoke_cheek",
    )
    base_yoke.visual(
        Box((0.014, 0.028, 0.046)),
        origin=Origin(xyz=(0.097, 0.0, 0.255)),
        material=stand_paint,
        name="right_yoke_cheek",
    )
    base_yoke.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(-0.085, 0.0, 0.272), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_paint,
        name="left_yoke_pivot",
    )
    base_yoke.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.085, 0.0, 0.272), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_paint,
        name="right_yoke_pivot",
    )
    base_yoke.visual(
        Cylinder(radius=0.019, length=0.020),
        origin=Origin(xyz=(-0.101, 0.0, 0.272), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_polymer,
        name="left_lock_core",
    )
    base_yoke.visual(
        Cylinder(radius=0.019, length=0.020),
        origin=Origin(xyz=(0.101, 0.0, 0.272), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_polymer,
        name="right_lock_core",
    )
    base_yoke.visual(
        lock_grip_mesh,
        origin=Origin(xyz=(-0.101, 0.0, 0.272), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=elastomer,
        name="left_lock_grip",
    )
    base_yoke.visual(
        lock_grip_mesh,
        origin=Origin(xyz=(0.101, 0.0, 0.272), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=elastomer,
        name="right_lock_grip",
    )
    base_yoke.inertial = Inertial.from_geometry(
        Box((0.220, 0.120, 0.290)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    head = model.part("head")
    head.visual(
        head_shell_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=head_paint,
        name="head_shell",
    )
    head.visual(
        front_bezel_mesh,
        origin=Origin(xyz=(0.0, 0.086, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.050, length=0.062),
        origin=Origin(xyz=(0.0, 0.0505, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_polymer,
        name="light_baffle",
    )
    head.visual(
        Cylinder(radius=0.049, length=0.003),
        origin=Origin(xyz=(0.0, 0.081575, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(0.0, -0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_polymer,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(-0.068, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.068, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.160, 0.190, 0.120)),
        mass=1.3,
        origin=Origin(),
    )

    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=base_yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.272)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.4,
            lower=math.radians(-55.0),
            upper=math.radians(80.0),
        ),
    )

    return model


def _aabb_center(aabb):
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_yoke = object_model.get_part("base_yoke")
    head = object_model.get_part("head")
    tilt = object_model.get_articulation("yoke_tilt")
    left_yoke_pivot = base_yoke.get_visual("left_yoke_pivot")
    right_yoke_pivot = base_yoke.get_visual("right_yoke_pivot")
    base_shell = base_yoke.get_visual("base_shell")
    left_trunnion = head.get_visual("left_trunnion")
    right_trunnion = head.get_visual("right_trunnion")
    front_lens = head.get_visual("front_lens")

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

    ctx.expect_contact(
        head,
        base_yoke,
        elem_a=left_trunnion,
        elem_b=left_yoke_pivot,
        name="left trunnion is clamped by left yoke pivot",
    )
    ctx.expect_contact(
        head,
        base_yoke,
        elem_a=right_trunnion,
        elem_b=right_yoke_pivot,
        name="right trunnion is clamped by right yoke pivot",
    )
    ctx.expect_gap(
        head,
        base_yoke,
        axis="z",
        positive_elem=front_lens,
        negative_elem=base_shell,
        min_gap=0.19,
        name="front lens clears weighted base",
    )

    closed_lens_aabb = ctx.part_element_world_aabb(head, elem=front_lens)
    with ctx.pose({tilt: math.radians(40.0)}):
        ctx.expect_contact(
            head,
            base_yoke,
            elem_a=left_trunnion,
            elem_b=left_yoke_pivot,
            name="left trunnion stays seated while tilting",
        )
        ctx.expect_contact(
            head,
            base_yoke,
            elem_a=right_trunnion,
            elem_b=right_yoke_pivot,
            name="right trunnion stays seated while tilting",
        )
        opened_lens_aabb = ctx.part_element_world_aabb(head, elem=front_lens)

    if closed_lens_aabb is None or opened_lens_aabb is None:
        ctx.fail("front lens AABB available for tilt check", "front lens visual could not be measured")
    else:
        closed_center = _aabb_center(closed_lens_aabb)
        opened_center = _aabb_center(opened_lens_aabb)
        ctx.check(
            "positive tilt raises beam aim",
            opened_center[2] > closed_center[2] + 0.03,
            details=(
                f"expected opened lens z > closed lens z by 0.03 m, "
                f"got closed={closed_center[2]:.4f}, opened={opened_center[2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
