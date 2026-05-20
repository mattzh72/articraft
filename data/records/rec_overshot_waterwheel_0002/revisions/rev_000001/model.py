from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _wheel_point(theta: float, radial: float, tangential: float = 0.0) -> tuple[float, float]:
    return (
        radial * math.sin(theta) + tangential * math.cos(theta),
        radial * math.cos(theta) - tangential * math.sin(theta),
    )


def _add_side_beam(
    part,
    *,
    name: str,
    x: float,
    y0: float,
    z0: float,
    y1: float,
    z1: float,
    width: float,
    depth: float,
    material,
) -> None:
    dy = y1 - y0
    dz = z1 - z0
    length = math.hypot(dy, dz)
    angle = math.atan2(dz, dy)
    part.visual(
        Box((width, length, depth)),
        origin=Origin(xyz=(x, (y0 + y1) * 0.5, (z0 + z1) * 0.5), rpy=(angle, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel", assets=ASSETS)

    timber = model.material("timber", rgba=(0.47, 0.33, 0.20, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.39, 0.28, 0.18, 1.0))
    iron = model.material("iron", rgba=(0.29, 0.31, 0.34, 1.0))

    axle_height = 0.70
    support_x = 0.31
    wheel_outer_radius = 0.494
    wheel_width = 0.15

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.76, 0.66, 1.30)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )
    frame.visual(
        Box((0.10, 0.64, 0.08)),
        origin=Origin(xyz=(-support_x, 0.0, 0.04)),
        material=timber,
        name="left_skid",
    )
    frame.visual(
        Box((0.10, 0.64, 0.08)),
        origin=Origin(xyz=(support_x, 0.0, 0.04)),
        material=timber,
        name="right_skid",
    )
    frame.visual(
        Box((0.74, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.26, 0.06)),
        material=timber,
        name="front_tie",
    )
    frame.visual(
        Box((0.74, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.26, 0.06)),
        material=timber,
        name="rear_tie",
    )
    frame.visual(
        Box((0.54, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=timber,
        name="center_stretcher",
    )

    _add_side_beam(
        frame,
        name="left_front_post",
        x=-support_x,
        y0=-0.23,
        z0=0.10,
        y1=-0.06,
        z1=axle_height + 0.04,
        width=0.08,
        depth=0.08,
        material=timber,
    )
    _add_side_beam(
        frame,
        name="left_rear_post",
        x=-support_x,
        y0=0.23,
        z0=0.10,
        y1=0.06,
        z1=axle_height + 0.04,
        width=0.08,
        depth=0.08,
        material=timber,
    )
    _add_side_beam(
        frame,
        name="right_front_post",
        x=support_x,
        y0=-0.23,
        z0=0.10,
        y1=-0.06,
        z1=axle_height + 0.04,
        width=0.08,
        depth=0.08,
        material=timber,
    )
    _add_side_beam(
        frame,
        name="right_rear_post",
        x=support_x,
        y0=0.23,
        z0=0.10,
        y1=0.06,
        z1=axle_height + 0.04,
        width=0.08,
        depth=0.08,
        material=timber,
    )

    frame.visual(
        Box((0.08, 0.20, 0.08)),
        origin=Origin(xyz=(-support_x, 0.0, axle_height + 0.08)),
        material=timber,
        name="left_top_cap",
    )
    frame.visual(
        Box((0.08, 0.20, 0.08)),
        origin=Origin(xyz=(support_x, 0.0, axle_height + 0.08)),
        material=timber,
        name="right_top_cap",
    )
    frame.visual(
        Box((0.08, 0.10, 0.06)),
        origin=Origin(xyz=(-support_x, 0.0, axle_height + 0.11)),
        material=wet_timber,
        name="left_bearing_block",
    )
    frame.visual(
        Box((0.08, 0.10, 0.06)),
        origin=Origin(xyz=(support_x, 0.0, axle_height + 0.11)),
        material=wet_timber,
        name="right_bearing_block",
    )
    frame.visual(
        Box((0.008, 0.07, 0.012)),
        origin=Origin(xyz=(-0.27, 0.0, axle_height - 0.03)),
        material=iron,
        name="left_bearing_face",
    )
    frame.visual(
        Box((0.008, 0.07, 0.012)),
        origin=Origin(xyz=(0.27, 0.0, axle_height - 0.03)),
        material=iron,
        name="right_bearing_face",
    )
    frame.visual(
        Box((0.04, 0.07, 0.15)),
        origin=Origin(xyz=(-0.29, 0.0, axle_height + 0.039)),
        material=wet_timber,
        name="left_bearing_web",
    )
    frame.visual(
        Box((0.04, 0.07, 0.15)),
        origin=Origin(xyz=(0.29, 0.0, axle_height + 0.039)),
        material=wet_timber,
        name="right_bearing_web",
    )

    chute = model.part("chute")
    chute.inertial = Inertial.from_geometry(
        Box((0.78, 0.30, 1.32)),
        mass=16.0,
        origin=Origin(xyz=(0.0, -0.22, 0.74)),
    )
    chute.visual(
        Box((0.05, 0.05, 1.24)),
        origin=Origin(xyz=(-0.20, -0.275, 0.72)),
        material=timber,
        name="left_support_post",
    )
    chute.visual(
        Box((0.05, 0.05, 1.24)),
        origin=Origin(xyz=(0.20, -0.275, 0.72)),
        material=timber,
        name="right_support_post",
    )
    chute.visual(
        Box((0.45, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, -0.275, 1.37)),
        material=timber,
        name="top_crossbeam",
    )
    chute.visual(
        Box((0.21, 0.05, 0.05)),
        origin=Origin(xyz=(-0.149, -0.275, 1.345)),
        material=timber,
        name="left_support_arm",
    )
    chute.visual(
        Box((0.21, 0.05, 0.05)),
        origin=Origin(xyz=(0.149, -0.275, 1.345)),
        material=timber,
        name="right_support_arm",
    )
    chute.visual(
        Box((0.026, 0.05, 0.11)),
        origin=Origin(xyz=(-0.098, -0.28, 1.315)),
        material=timber,
        name="left_hanger",
    )
    chute.visual(
        Box((0.026, 0.05, 0.11)),
        origin=Origin(xyz=(0.098, -0.28, 1.315)),
        material=timber,
        name="right_hanger",
    )
    chute.visual(
        Box((0.22, 0.24, 0.03)),
        origin=Origin(xyz=(0.0, -0.18, 1.225)),
        material=wet_timber,
        name="chute_floor",
    )
    chute.visual(
        Box((0.022, 0.24, 0.08)),
        origin=Origin(xyz=(-0.099, -0.18, 1.25)),
        material=wet_timber,
        name="chute_left_wall",
    )
    chute.visual(
        Box((0.022, 0.24, 0.08)),
        origin=Origin(xyz=(0.099, -0.18, 1.25)),
        material=wet_timber,
        name="chute_right_wall",
    )
    chute.visual(
        Box((0.22, 0.022, 0.09)),
        origin=Origin(xyz=(0.0, -0.289, 1.255)),
        material=wet_timber,
        name="chute_back_board",
    )
    chute.visual(
        Box((0.16, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, -0.07, 1.215)),
        material=wet_timber,
        name="chute_lip",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.50, length=0.54),
        mass=34.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    wheel.visual(
        Cylinder(radius=0.024, length=0.54),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.090, length=0.18),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=timber,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.11, length=0.022),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wet_timber,
        name="hub_flange_left",
    )
    wheel.visual(
        Cylinder(radius=0.11, length=0.022),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wet_timber,
        name="hub_flange_right",
    )

    rim_mesh = _save_mesh(
        "waterwheel_rim_ring.obj",
        TorusGeometry(radius=0.48, tube=0.014, radial_segments=18, tubular_segments=72).rotate_y(math.pi / 2.0),
    )
    wheel.visual(rim_mesh, origin=Origin(xyz=(-0.078, 0.0, 0.0)), material=iron, name="rim_left")
    wheel.visual(rim_mesh, origin=Origin(xyz=(0.078, 0.0, 0.0)), material=iron, name="rim_right")

    for index in range(12):
        theta = (2.0 * math.pi * index) / 12.0
        spoke_y, spoke_z = _wheel_point(theta, 0.27)
        wheel.visual(
            Box((wheel_width, 0.40, 0.022)),
            origin=Origin(
                xyz=(0.0, spoke_y, spoke_z),
                rpy=(math.pi / 2.0 - theta, 0.0, 0.0),
            ),
            material=timber,
            name=f"spoke_{index:02d}",
        )

    floor_radius = 0.460
    floor_size = (wheel_width, 0.078, 0.016)
    back_size = (wheel_width, 0.014, 0.075)
    back_tangential = 0.032
    back_radial = floor_radius - (floor_size[2] * 0.5 + back_size[2] * 0.5)
    for index in range(16):
        theta = (2.0 * math.pi * index) / 16.0
        floor_y, floor_z = _wheel_point(theta, floor_radius)
        back_y, back_z = _wheel_point(theta, back_radial, back_tangential)
        wheel.visual(
            Box(floor_size),
            origin=Origin(xyz=(0.0, floor_y, floor_z), rpy=(-theta, 0.0, 0.0)),
            material=wet_timber,
            name=f"bucket_floor_{index:02d}",
        )
        wheel.visual(
            Box(back_size),
            origin=Origin(xyz=(0.0, back_y, back_z), rpy=(-theta, 0.0, 0.0)),
            material=wet_timber,
            name=f"bucket_back_{index:02d}",
        )

    model.articulation(
        "chute_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=chute,
        origin=Origin(),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    axle_height = 0.70
    frame = object_model.get_part("frame")
    chute = object_model.get_part("chute")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")
    axle = wheel.get_visual("axle")
    rim_left = wheel.get_visual("rim_left")
    top_bucket = wheel.get_visual("bucket_floor_00")
    bottom_bucket = wheel.get_visual("bucket_floor_08")
    chute_floor = chute.get_visual("chute_floor")
    left_support_post = chute.get_visual("left_support_post")
    right_support_post = chute.get_visual("right_support_post")
    center_stretcher = frame.get_visual("center_stretcher")
    front_tie = frame.get_visual("front_tie")
    left_bearing_face = frame.get_visual("left_bearing_face")
    right_bearing_face = frame.get_visual("right_bearing_face")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem=axle,
        negative_elem=left_bearing_face,
        max_gap=0.001,
        max_penetration=0.0,
        name="left_journal_rests_on_left_bearing",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem=axle,
        negative_elem=right_bearing_face,
        max_gap=0.001,
        max_penetration=0.0,
        name="right_journal_rests_on_right_bearing",
    )
    ctx.expect_origin_distance(
        wheel,
        frame,
        axes="xy",
        max_dist=0.001,
        name="wheel_centered_between_side_supports",
    )
    ctx.expect_overlap(
        chute,
        wheel,
        axes="x",
        elem_a=chute_floor,
        elem_b=top_bucket,
        min_overlap=0.12,
        name="chute_aligned_over_bucket_path",
    )
    ctx.expect_gap(
        chute,
        wheel,
        axis="z",
        positive_elem=chute_floor,
        negative_elem=top_bucket,
        min_gap=0.015,
        max_gap=0.08,
        name="chute_sits_just_above_top_bucket",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem=bottom_bucket,
        negative_elem=center_stretcher,
        min_gap=0.06,
        name="wheel_clears_center_stretcher_below",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem=rim_left,
        negative_elem=center_stretcher,
        min_gap=0.05,
        name="rim_bottom_clears_frame",
    )
    ctx.expect_contact(
        chute,
        frame,
        elem_a=left_support_post,
        elem_b=front_tie,
        name="left_chute_post_seats_on_front_tie",
    )
    ctx.expect_contact(
        chute,
        frame,
        elem_a=right_support_post,
        elem_b=front_tie,
        name="right_chute_post_seats_on_front_tie",
    )

    def _center_from_aabb(aabb) -> tuple[float, float, float]:
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    rest_bucket_aabb = ctx.part_element_world_aabb(wheel, elem=top_bucket)
    assert rest_bucket_aabb is not None
    rest_center = _center_from_aabb(rest_bucket_aabb)
    assert rest_center[2] > axle_height + 0.42

    with ctx.pose({wheel_spin: math.pi / 2.0}):
        quarter_bucket_aabb = ctx.part_element_world_aabb(wheel, elem=top_bucket)
        assert quarter_bucket_aabb is not None
        quarter_center = _center_from_aabb(quarter_bucket_aabb)
        assert abs(quarter_center[1]) > 0.38
        assert abs(quarter_center[2] - axle_height) < 0.09
        ctx.expect_gap(
            wheel,
            frame,
            axis="z",
            positive_elem=axle,
            negative_elem=left_bearing_face,
            max_gap=0.001,
            max_penetration=0.0,
            name="left_journal_stays_supported_at_quarter_turn",
        )
        ctx.expect_gap(
            chute,
            wheel,
            axis="z",
            positive_elem=chute_floor,
            negative_elem=top_bucket,
            min_gap=0.30,
            name="rotated_bucket_moves_well_below_chute",
        )

    with ctx.pose({wheel_spin: math.pi}):
        inverted_bucket_aabb = ctx.part_element_world_aabb(wheel, elem=top_bucket)
        assert inverted_bucket_aabb is not None
        inverted_center = _center_from_aabb(inverted_bucket_aabb)
        assert inverted_center[2] < axle_height - 0.38
        ctx.expect_gap(
            wheel,
            frame,
            axis="z",
            positive_elem=axle,
            negative_elem=right_bearing_face,
            max_gap=0.001,
            max_penetration=0.0,
            name="right_journal_stays_supported_at_half_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
