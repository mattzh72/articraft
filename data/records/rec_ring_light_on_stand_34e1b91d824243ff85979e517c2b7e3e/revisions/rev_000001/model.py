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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _annulus_mesh(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    name: str,
):
    outer = _circle_profile(outer_radius)
    inner = list(reversed(_circle_profile(inner_radius)))
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_ring_light")

    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.97, 0.97, 0.95, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    mast_receiver_mesh = _annulus_mesh(0.046, 0.020, 0.160, "mast_receiver")
    upper_stop_collar_mesh = _annulus_mesh(0.020, 0.016, 0.020, "upper_stop_collar")
    lower_mast_shell = _annulus_mesh(0.020, 0.017, 0.795, "lower_mast_shell")
    height_lock_collar_mesh = _annulus_mesh(0.028, 0.018, 0.060, "height_lock_collar")
    ring_housing_mesh = _annulus_mesh(0.230, 0.162, 0.036, "ring_housing")
    ring_diffuser_mesh = _annulus_mesh(0.230, 0.168, 0.004, "ring_diffuser")

    stand_base = model.part("stand_base")
    stand_base.visual(
        Cylinder(radius=0.072, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=black,
        name="hub_base",
    )
    stand_base.visual(
        mast_receiver_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=dark_gray,
        name="mast_receiver",
    )
    stand_base.visual(
        lower_mast_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.6325)),
        material=satin_metal,
        name="lower_mast_shell",
    )
    stand_base.visual(
        height_lock_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        material=dark_gray,
        name="height_lock_collar",
    )
    stand_base.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.032, 0.0, 0.995), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="height_lock_knob",
    )

    leg_radius = 0.010
    foot_radius = 0.030
    hub_anchor = (0.0, 0.0, 0.060)
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        foot = (
            0.340 * math.cos(angle),
            0.340 * math.sin(angle),
            0.026,
        )
        brace = (
            0.140 * math.cos(angle),
            0.140 * math.sin(angle),
            0.112,
        )
        _add_member(
            stand_base,
            hub_anchor,
            foot,
            leg_radius,
            dark_gray,
            name=f"leg_{index}",
        )
        _add_member(
            stand_base,
            brace,
            foot,
            0.0065,
            dark_gray,
            name=f"leg_brace_{index}",
        )
        stand_base.visual(
            Cylinder(radius=foot_radius, length=0.026),
            origin=Origin(xyz=(foot[0], foot[1], 0.013)),
            material=rubber,
            name=f"foot_{index}",
        )
    stand_base.inertial = Inertial.from_geometry(
        Box((0.74, 0.74, 1.06)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        upper_stop_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.920)),
        material=satin_metal,
        name="upper_stop_collar",
    )
    upper_mast.visual(
        Cylinder(radius=0.0155, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 1.110)),
        material=satin_metal,
        name="upper_tube",
    )
    upper_mast.visual(
        Cylinder(radius=0.020, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 1.300)),
        material=dark_gray,
        name="upper_transition_collar",
    )
    upper_mast.visual(
        Box((0.018, 0.028, 0.300)),
        origin=Origin(xyz=(0.006, 0.0, 1.185)),
        material=dark_gray,
        name="yoke_spine",
    )
    upper_mast.visual(
        Box((0.018, 0.436, 0.020)),
        origin=Origin(xyz=(0.006, 0.0, 1.290)),
        material=dark_gray,
        name="yoke_bridge",
    )
    upper_mast.visual(
        Box((0.012, 0.014, 0.250)),
        origin=Origin(xyz=(0.015, 0.225, 1.170)),
        material=dark_gray,
        name="yoke_left_arm",
    )
    upper_mast.visual(
        Box((0.012, 0.014, 0.250)),
        origin=Origin(xyz=(0.015, -0.225, 1.170)),
        material=dark_gray,
        name="yoke_right_arm",
    )
    upper_mast.visual(
        Box((0.010, 0.036, 0.034)),
        origin=Origin(xyz=(0.020, 0.0, 0.940)),
        material=dark_gray,
        name="phone_hinge_block",
    )
    upper_mast.visual(
        Box((0.006, 0.006, 0.034)),
        origin=Origin(xyz=(0.025, 0.016, 0.940)),
        material=black,
        name="phone_hinge_left_cheek",
    )
    upper_mast.visual(
        Box((0.006, 0.006, 0.034)),
        origin=Origin(xyz=(0.025, -0.016, 0.940)),
        material=black,
        name="phone_hinge_right_cheek",
    )
    upper_mast.inertial = Inertial.from_geometry(
        Box((0.10, 0.48, 1.22)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        ring_housing_mesh,
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="ring_housing",
    )
    ring_head.visual(
        ring_diffuser_mesh,
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=diffuser_white,
        name="ring_diffuser",
    )
    ring_head.visual(
        Box((0.016, 0.436, 0.018)),
        origin=Origin(xyz=(0.029, 0.0, 0.0)),
        material=dark_gray,
        name="rear_mount_bar",
    )
    ring_head.visual(
        Box((0.022, 0.018, 0.018)),
        origin=Origin(xyz=(0.011, 0.224, 0.0)),
        material=dark_gray,
        name="left_trunnion_boss",
    )
    ring_head.visual(
        Box((0.022, 0.018, 0.018)),
        origin=Origin(xyz=(0.011, -0.224, 0.0)),
        material=dark_gray,
        name="right_trunnion_boss",
    )
    ring_head.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.224, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="left_trunnion",
    )
    ring_head.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, -0.224, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="right_trunnion",
    )
    ring_head.visual(
        Box((0.030, 0.110, 0.030)),
        origin=Origin(xyz=(0.034, 0.0, -0.175)),
        material=dark_gray,
        name="driver_housing",
    )
    ring_head.inertial = Inertial.from_geometry(
        Box((0.050, 0.470, 0.470)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    phone_arm = model.part("phone_arm")
    phone_arm.visual(
        Box((0.012, 0.026, 0.026)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=black,
        name="hinge_knuckle",
    )
    phone_arm.visual(
        Box((0.022, 0.014, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, -0.006)),
        material=dark_gray,
        name="offset_link",
    )
    phone_arm.visual(
        Box((0.012, 0.018, 0.120)),
        origin=Origin(xyz=(0.037, 0.0, -0.060)),
        material=dark_gray,
        name="drop_strut",
    )
    phone_arm.visual(
        Box((0.132, 0.018, 0.012)),
        origin=Origin(xyz=(0.103, 0.0, -0.114)),
        material=dark_gray,
        name="arm_bar",
    )
    phone_arm.visual(
        Box((0.024, 0.038, 0.040)),
        origin=Origin(xyz=(0.176, 0.0, -0.114)),
        material=black,
        name="cradle_mount",
    )
    phone_arm.visual(
        Box((0.012, 0.086, 0.152)),
        origin=Origin(xyz=(0.188, 0.0, -0.114)),
        material=black,
        name="phone_cradle",
    )
    phone_arm.visual(
        Box((0.020, 0.090, 0.010)),
        origin=Origin(xyz=(0.194, 0.0, -0.043)),
        material=rubber,
        name="top_jaw",
    )
    phone_arm.visual(
        Box((0.020, 0.090, 0.010)),
        origin=Origin(xyz=(0.194, 0.0, -0.185)),
        material=rubber,
        name="bottom_jaw",
    )
    phone_arm.inertial = Inertial.from_geometry(
        Box((0.210, 0.100, 0.170)),
        mass=0.35,
        origin=Origin(xyz=(0.110, 0.0, -0.090)),
    )

    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=stand_base,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.35,
            lower=0.0,
            upper=0.45,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_mast,
        child=ring_head,
        origin=Origin(xyz=(0.036, 0.0, 1.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=math.radians(-40.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "phone_arm_fold",
        ArticulationType.REVOLUTE,
        parent=upper_mast,
        child=phone_arm,
        origin=Origin(xyz=(0.025, 0.0, 0.940)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=math.radians(-8.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand_base = object_model.get_part("stand_base")
    upper_mast = object_model.get_part("upper_mast")
    ring_head = object_model.get_part("ring_head")
    phone_arm = object_model.get_part("phone_arm")

    mast_extension = object_model.get_articulation("mast_extension")
    head_tilt = object_model.get_articulation("head_tilt")
    phone_arm_fold = object_model.get_articulation("phone_arm_fold")

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

    ctx.expect_contact(upper_mast, stand_base, name="upper mast remains seated in lower mast")
    ctx.expect_contact(ring_head, upper_mast, name="ring head trunnions seat in the yoke")
    ctx.expect_contact(phone_arm, upper_mast, name="phone arm stays clipped into its hinge")
    ctx.expect_within(
        phone_arm,
        ring_head,
        axes="y",
        inner_elem="phone_cradle",
        outer_elem="ring_housing",
        margin=0.005,
        name="phone cradle stays centered within ring opening width",
    )
    ctx.expect_origin_gap(
        ring_head,
        phone_arm,
        axis="z",
        min_gap=0.20,
        max_gap=0.35,
        name="phone arm hinge sits below the ring head",
    )

    rest_upper_pos = ctx.part_world_position(upper_mast)
    if rest_upper_pos is None:
        ctx.fail("upper mast world position available", "Upper mast world position is None.")
    else:
        with ctx.pose({mast_extension: 0.45}):
            extended_upper_pos = ctx.part_world_position(upper_mast)
            if extended_upper_pos is None:
                ctx.fail(
                    "extended upper mast world position available",
                    "Extended upper mast world position is None.",
                )
            else:
                ctx.check(
                    "mast extends upward",
                    extended_upper_pos[2] > rest_upper_pos[2] + 0.44,
                    (
                        f"Expected mast extension to raise the upper mast by > 0.44 m, "
                        f"got {extended_upper_pos[2] - rest_upper_pos[2]:.4f} m."
                    ),
                )

    ring_rest = ctx.part_world_aabb(ring_head)
    if ring_rest is None:
        ctx.fail("ring head rest aabb available", "Ring head AABB is None at rest pose.")
    else:
        ring_rest_x = ring_rest[1][0] - ring_rest[0][0]
        with ctx.pose({head_tilt: math.radians(32.0)}):
            ring_tilted = ctx.part_world_aabb(ring_head)
            if ring_tilted is None:
                ctx.fail(
                    "ring head tilted aabb available",
                    "Ring head AABB is None in tilted pose.",
                )
            else:
                ring_tilted_x = ring_tilted[1][0] - ring_tilted[0][0]
                ctx.check(
                    "ring head tilts on horizontal axis",
                    ring_tilted_x > ring_rest_x + 0.12,
                    (
                        f"Expected ring head x-extent to grow by > 0.12 m when tilted, "
                        f"got {ring_tilted_x - ring_rest_x:.4f} m."
                    ),
                )
            ctx.expect_contact(
                ring_head,
                upper_mast,
                name="ring head stays captured in yoke while tilted",
            )

    cradle_rest = ctx.part_element_world_aabb(phone_arm, elem="phone_cradle")
    if cradle_rest is None:
        ctx.fail("phone cradle rest aabb available", "Phone cradle AABB is None at rest pose.")
    else:
        cradle_rest_center_x = (cradle_rest[0][0] + cradle_rest[1][0]) * 0.5
        cradle_rest_center_z = (cradle_rest[0][2] + cradle_rest[1][2]) * 0.5
        with ctx.pose({phone_arm_fold: math.radians(72.0)}):
            cradle_folded = ctx.part_element_world_aabb(phone_arm, elem="phone_cradle")
            if cradle_folded is None:
                ctx.fail(
                    "phone cradle folded aabb available",
                    "Phone cradle AABB is None in folded pose.",
                )
            else:
                cradle_folded_center_x = (cradle_folded[0][0] + cradle_folded[1][0]) * 0.5
                cradle_folded_center_z = (cradle_folded[0][2] + cradle_folded[1][2]) * 0.5
                ctx.check(
                    "phone arm folds out from its stowed position",
                    (
                        cradle_folded_center_z < cradle_rest_center_z - 0.08
                        and abs(cradle_folded_center_x - cradle_rest_center_x) > 0.10
                    ),
                    (
                        "Expected folded cradle to swing outward and lower noticeably; "
                        f"delta_x={cradle_folded_center_x - cradle_rest_center_x:.4f} m, "
                        f"delta_z={cradle_folded_center_z - cradle_rest_center_z:.4f} m."
                    ),
                )
            ctx.expect_contact(
                phone_arm,
                upper_mast,
                name="phone arm stays attached at folded pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
