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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        (lower[0] + upper[0]) * 0.5,
        (lower[1] + upper[1]) * 0.5,
        (lower[2] + upper[2]) * 0.5,
    )


def _build_lock_arm_geometry():
    arm_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.004, 0.0, 0.040),
            (0.020, 0.0, 0.090),
            (0.060, 0.0, 0.135),
            (0.112, 0.0, 0.162),
            (0.142, 0.0, 0.146),
        ],
        radius=0.0075,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    pivot_boss = CylinderGeometry(radius=0.014, height=0.030, radial_segments=24)
    pivot_boss.rotate_y(math.pi * 0.5)
    arm_geom.merge(pivot_boss)
    return arm_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="centrifugal_juicer")

    body_black = model.material("body_black", rgba=(0.15, 0.15, 0.16, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    basket_steel = model.material("basket_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.72, 0.77, 0.80, 0.26))
    clear_lid = model.material("clear_lid", rgba=(0.82, 0.90, 0.96, 0.22))
    pusher_dark = model.material("pusher_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    arm_steel = model.material("arm_steel", rgba=(0.68, 0.70, 0.73, 1.0))

    arm_frame_mesh = mesh_from_geometry(_build_lock_arm_geometry(), "lock_arm_frame")
    collar_ring = LatheGeometry.from_shell_profiles(
        [
            (0.084, 0.000),
            (0.107, 0.002),
            (0.107, 0.018),
            (0.084, 0.018),
        ],
        [
            (0.080, 0.003),
            (0.102, 0.004),
            (0.102, 0.016),
            (0.080, 0.015),
        ],
        segments=48,
    )

    body = model.part("body")

    body_shell = ExtrudeGeometry.from_z0(rounded_rect_profile(0.40, 0.28, 0.036), 0.095)
    body.visual(mesh_from_geometry(body_shell, "body_shell"), material=body_black, name="body_shell")
    body.visual(
        mesh_from_geometry(collar_ring, "motor_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=trim_silver,
        name="motor_collar",
    )

    bowl_wall = LatheGeometry.from_shell_profiles(
        [
            (0.108, 0.000),
            (0.136, 0.004),
            (0.141, 0.050),
            (0.128, 0.066),
        ],
        [
            (0.103, 0.004),
            (0.131, 0.008),
            (0.134, 0.048),
            (0.120, 0.060),
        ],
        segments=56,
    )
    body.visual(
        mesh_from_geometry(bowl_wall, "bowl_wall"),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=smoke_clear,
        name="bowl_wall",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=trim_silver,
        name="drive_spindle",
    )
    body.visual(
        Box((0.058, 0.060, 0.030)),
        origin=Origin(xyz=(0.162, 0.0, 0.121)),
        material=smoke_clear,
        name="spout_body",
    )
    body.visual(
        Box((0.040, 0.048, 0.008)),
        origin=Origin(xyz=(0.194, 0.0, 0.137)),
        material=smoke_clear,
        name="spout_lip",
    )
    for sign, name in ((1.0, "left_shoulder"), (-1.0, "right_shoulder")):
        body.visual(
            Box((0.090, 0.028, 0.050)),
            origin=Origin(xyz=(-0.010, sign * 0.136, 0.110)),
            material=body_black,
            name=name,
        )
    body.visual(
        Box((0.060, 0.160, 0.028)),
        origin=Origin(xyz=(-0.152, 0.0, 0.122)),
        material=body_black,
        name="rear_bridge",
    )
    for sign, name in ((1.0, "hinge_cheek_0"), (-1.0, "hinge_cheek_1")):
        body.visual(
            Box((0.018, 0.020, 0.060)),
            origin=Origin(xyz=(-0.156, sign * 0.055, 0.160)),
            material=body_black,
            name=name,
        )
    body.visual(
        Box((0.014, 0.126, 0.010)),
        origin=Origin(xyz=(-0.158, 0.0, 0.185)),
        material=body_black,
        name="hinge_tie",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.40, 0.28, 0.18)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    basket = model.part("basket")
    basket_shell = LatheGeometry.from_shell_profiles(
        [
            (0.020, 0.000),
            (0.070, 0.006),
            (0.079, 0.045),
            (0.067, 0.056),
        ],
        [
            (0.000, 0.003),
            (0.066, 0.009),
            (0.073, 0.043),
            (0.061, 0.052),
        ],
        segments=56,
    )
    basket.visual(mesh_from_geometry(basket_shell, "basket_shell"), material=basket_steel, name="basket_shell")
    basket.visual(
        Cylinder(radius=0.068, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=basket_steel,
        name="basket_disc",
    )
    basket.visual(
        Cylinder(radius=0.017, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=trim_silver,
        name="basket_hub",
    )
    basket.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=body_black,
        name="drive_stub",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.079, length=0.070),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=40.0),
    )

    lid = model.part("lid")
    lid_shell = LatheGeometry.from_shell_profiles(
        [
            (0.116, 0.000),
            (0.139, 0.002),
            (0.145, 0.038),
            (0.132, 0.092),
            (0.058, 0.118),
        ],
        [
            (0.111, 0.003),
            (0.134, 0.006),
            (0.139, 0.036),
            (0.125, 0.087),
            (0.049, 0.112),
        ],
        segments=56,
    )
    lid.visual(
        mesh_from_geometry(lid_shell, "lid_shell"),
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material=clear_lid,
        name="lid_shell",
    )
    chute_shell = LatheGeometry.from_shell_profiles(
        [
            (0.054, 0.000),
            (0.058, 0.012),
            (0.060, 0.118),
        ],
        [
            (0.047, 0.004),
            (0.050, 0.014),
            (0.052, 0.114),
        ],
        segments=48,
    )
    lid.visual(
        mesh_from_geometry(chute_shell, "chute_shell"),
        origin=Origin(xyz=(0.135, 0.0, 0.112)),
        material=clear_lid,
        name="chute_shell",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.108),
        origin=Origin(xyz=(0.006, 0.0, 0.018), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=body_black,
        name="hinge_sleeve",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.29, 0.29, 0.24)),
        mass=0.7,
        origin=Origin(xyz=(0.135, 0.0, 0.120)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.135, 0.0, 0.158)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.045, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, -0.083)),
        material=pusher_dark,
        name="pusher_body",
    )
    pusher.visual(
        Cylinder(radius=0.058, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=pusher_dark,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=trim_silver,
        name="pusher_knob",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.057, length=0.220),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.135, 0.0, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.16,
            lower=-0.050,
            upper=0.0,
        ),
    )

    closed_arm_roll = math.radians(5.0)
    for sign, part_name, joint_name, axis in (
        (1.0, "left_arm", "body_to_left_arm", (-1.0, 0.0, 0.0)),
        (-1.0, "right_arm", "body_to_right_arm", (1.0, 0.0, 0.0)),
    ):
        arm = model.part(part_name)
        arm.visual(
            arm_frame_mesh,
            origin=Origin(rpy=(sign * closed_arm_roll, 0.0, 0.0)),
            material=arm_steel,
            name="arm_frame",
        )
        arm.visual(
            Box((0.034, 0.024, 0.016)),
            origin=Origin(xyz=(0.136, 0.0, 0.144), rpy=(sign * closed_arm_roll, 0.0, 0.0)),
            material=body_black,
            name="clamp_pad",
        )
        arm.visual(
            Box((0.024, 0.020, 0.020)),
            origin=Origin(xyz=(0.0, -sign * 0.010, 0.0)),
            material=arm_steel,
            name="pivot_mount",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.18, 0.04, 0.18)),
            mass=0.18,
            origin=Origin(xyz=(0.072, 0.0, 0.090)),
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=(-0.010, sign * 0.170, 0.108)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.6,
                lower=0.0,
                upper=math.radians(48.0),
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

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    basket_spin = object_model.get_articulation("body_to_basket")
    left_arm_joint = object_model.get_articulation("body_to_left_arm")
    right_arm_joint = object_model.get_articulation("body_to_right_arm")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="lid_shell",
        negative_elem="bowl_wall",
        name="lid seats just above the bowl rim",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        margin=0.0015,
        inner_elem="pusher_body",
        outer_elem="chute_shell",
        name="pusher stays centered in the feed chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        min_overlap=0.090,
        elem_a="pusher_body",
        elem_b="chute_shell",
        name="pusher remains substantially inserted at rest",
    )
    ctx.expect_within(
        basket,
        body,
        axes="xy",
        margin=0.015,
        inner_elem="basket_shell",
        outer_elem="bowl_wall",
        name="basket stays within the collector bowl footprint",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    closed_left_pad = _aabb_center(ctx.part_element_world_aabb(left_arm, elem="clamp_pad"))
    closed_right_pad = _aabb_center(ctx.part_element_world_aabb(right_arm, elem="clamp_pad"))
    rest_pusher = ctx.part_world_position(pusher)

    with ctx.pose({lid_hinge: math.radians(72.0)}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    with ctx.pose({pusher_slide: -0.050}):
        inserted_pusher = ctx.part_world_position(pusher)
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            min_overlap=0.070,
            elem_a="pusher_body",
            elem_b="chute_shell",
            name="pusher keeps retained insertion when pressed down",
        )
        ctx.check(
            "pusher moves downward along the chute axis",
            rest_pusher is not None
            and inserted_pusher is not None
            and inserted_pusher[2] < rest_pusher[2] - 0.040,
            details=f"rest={rest_pusher}, inserted={inserted_pusher}",
        )

    with ctx.pose({left_arm_joint: math.radians(42.0), right_arm_joint: math.radians(42.0)}):
        open_left_pad = _aabb_center(ctx.part_element_world_aabb(left_arm, elem="clamp_pad"))
        open_right_pad = _aabb_center(ctx.part_element_world_aabb(right_arm, elem="clamp_pad"))
        ctx.check(
            "locking arms swing outward to release the lid",
            closed_left_pad is not None
            and closed_right_pad is not None
            and open_left_pad is not None
            and open_right_pad is not None
            and open_left_pad[1] > closed_left_pad[1] + 0.050
            and open_right_pad[1] < closed_right_pad[1] - 0.050,
            details=(
                f"closed_left={closed_left_pad}, open_left={open_left_pad}, "
                f"closed_right={closed_right_pad}, open_right={open_right_pad}"
            ),
        )

    ctx.check(
        "basket uses a vertical continuous drive joint",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 4) for v in basket_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={basket_spin.articulation_type}, axis={basket_spin.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
