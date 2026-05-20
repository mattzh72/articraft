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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_double_pet_flap")

    frame_plastic = model.material("frame_plastic", rgba=(0.90, 0.91, 0.93, 1.0))
    trim_plastic = model.material("trim_plastic", rgba=(0.82, 0.84, 0.86, 1.0))
    smoked_flap = model.material("smoked_flap", rgba=(0.22, 0.28, 0.30, 0.55))
    gasket_dark = model.material("gasket_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    bar_dark = model.material("bar_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    bar_grip = model.material("bar_grip", rgba=(0.26, 0.27, 0.29, 1.0))

    outer_w = 0.310
    outer_h = 0.380
    tunnel_w = 0.270
    tunnel_h = 0.340
    opening_w = 0.210
    opening_h = 0.250
    depth = 0.030
    bezel_t = 0.004
    side_wall = (tunnel_w - opening_w) * 0.5
    top_wall = (tunnel_h - opening_h) * 0.5

    flap_w = 0.186
    flap_h = 0.236
    flap_t = 0.004
    hinge_z = opening_h * 0.5 - 0.006
    front_flap_y = 0.0085
    rear_flap_y = -0.0085

    bar_x = 0.101
    bar_w = 0.006
    bar_d = 0.010
    bar_len = 0.172
    bar_joint_z = hinge_z - 0.002

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((outer_w, depth + 2.0 * bezel_t, outer_h)),
        mass=1.9,
    )

    # Main tunnel body.
    frame.visual(
        Box((side_wall, depth, tunnel_h)),
        origin=Origin(xyz=(-(opening_w * 0.5 + side_wall * 0.5), 0.0, 0.0)),
        material=frame_plastic,
        name="left_tunnel_wall",
    )
    frame.visual(
        Box((side_wall, depth, tunnel_h)),
        origin=Origin(xyz=(opening_w * 0.5 + side_wall * 0.5, 0.0, 0.0)),
        material=frame_plastic,
        name="right_tunnel_wall",
    )
    frame.visual(
        Box((tunnel_w, depth, top_wall)),
        origin=Origin(xyz=(0.0, 0.0, opening_h * 0.5 + top_wall * 0.5)),
        material=frame_plastic,
        name="top_tunnel_wall",
    )
    frame.visual(
        Box((tunnel_w, depth, top_wall)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_h * 0.5 + top_wall * 0.5))),
        material=frame_plastic,
        name="bottom_tunnel_wall",
    )

    # Low-profile front and rear trim rings with slight overlap into the tunnel walls.
    flange_y = depth * 0.5 + bezel_t * 0.5 - 0.0005
    side_trim_w = (outer_w - opening_w) * 0.5
    top_trim_h = (outer_h - opening_h) * 0.5
    for side_sign, y_sign, prefix in (
        (-1.0, 1.0, "front"),
        (1.0, 1.0, "front"),
        (-1.0, -1.0, "rear"),
        (1.0, -1.0, "rear"),
    ):
        frame.visual(
            Box((side_trim_w, bezel_t, outer_h)),
            origin=Origin(
                xyz=(
                    side_sign * (opening_w * 0.5 + side_trim_w * 0.5),
                    y_sign * flange_y,
                    0.0,
                )
            ),
            material=trim_plastic,
            name=f"{prefix}_{'left' if side_sign < 0.0 else 'right'}_trim",
        )
    for z_sign, y_sign, prefix in (
        (1.0, 1.0, "front"),
        (-1.0, 1.0, "front"),
        (1.0, -1.0, "rear"),
        (-1.0, -1.0, "rear"),
    ):
        frame.visual(
            Box((outer_w, bezel_t, top_trim_h)),
            origin=Origin(
                xyz=(
                    0.0,
                    y_sign * flange_y,
                    z_sign * (opening_h * 0.5 + top_trim_h * 0.5),
                )
            ),
            material=trim_plastic,
            name=f"{prefix}_{'top' if z_sign > 0.0 else 'bottom'}_trim",
        )

    # Small hinge covers tucked under the top of the frame to support the two separate flaps.
    frame.visual(
        Box((flap_w + 0.012, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.012, hinge_z + 0.005)),
        material=frame_plastic,
        name="front_hinge_cover",
    )
    frame.visual(
        Box((flap_w + 0.012, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.012, hinge_z + 0.005)),
        material=frame_plastic,
        name="rear_hinge_cover",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=flap_w + 0.016),
        origin=Origin(xyz=(0.0, 0.012, hinge_z + 0.003), rpy=(0.0, pi * 0.5, 0.0)),
        material=trim_plastic,
        name="front_hinge_socket",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=flap_w + 0.016),
        origin=Origin(xyz=(0.0, -0.012, hinge_z + 0.003), rpy=(0.0, pi * 0.5, 0.0)),
        material=trim_plastic,
        name="rear_hinge_socket",
    )

    # Slim right-side guide track for the vertical lock bar.
    frame.visual(
        Box((0.016, 0.012, 0.228)),
        origin=Origin(xyz=(0.121, 0.0, -0.005)),
        material=trim_plastic,
        name="guide_spine",
    )
    frame.visual(
        Box((0.020, 0.012, 0.036)),
        origin=Origin(xyz=(0.119, 0.0, 0.093)),
        material=frame_plastic,
        name="upper_guide_outer",
    )
    frame.visual(
        Box((0.012, 0.003, 0.036)),
        origin=Origin(xyz=(0.108, 0.0075, 0.093)),
        material=frame_plastic,
        name="upper_guide_front_cheek",
    )
    frame.visual(
        Box((0.012, 0.003, 0.036)),
        origin=Origin(xyz=(0.108, -0.0075, 0.093)),
        material=frame_plastic,
        name="upper_guide_rear_cheek",
    )
    frame.visual(
        Box((0.020, 0.012, 0.040)),
        origin=Origin(xyz=(0.119, 0.0, -0.104)),
        material=frame_plastic,
        name="lower_guide_outer",
    )
    frame.visual(
        Box((0.012, 0.003, 0.040)),
        origin=Origin(xyz=(0.108, 0.0075, -0.104)),
        material=frame_plastic,
        name="lower_guide_front_cheek",
    )
    frame.visual(
        Box((0.012, 0.003, 0.040)),
        origin=Origin(xyz=(0.108, -0.0075, -0.104)),
        material=frame_plastic,
        name="lower_guide_rear_cheek",
    )

    front_flap = model.part("front_flap")
    front_flap.inertial = Inertial.from_geometry(
        Box((flap_w, flap_t + 0.002, flap_h)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -flap_h * 0.5)),
    )
    front_flap.visual(
        Box((flap_w, flap_t, flap_h)),
        origin=Origin(xyz=(0.0, 0.0, -flap_h * 0.5)),
        material=smoked_flap,
        name="front_panel",
    )
    front_flap.visual(
        Box((flap_w * 0.96, flap_t + 0.002, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=gasket_dark,
        name="front_top_rail",
    )
    front_flap.visual(
        Box((flap_w * 0.98, flap_t + 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -(flap_h - 0.007))),
        material=gasket_dark,
        name="front_bottom_weight",
    )

    rear_flap = model.part("rear_flap")
    rear_flap.inertial = Inertial.from_geometry(
        Box((flap_w, flap_t + 0.002, flap_h)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -flap_h * 0.5)),
    )
    rear_flap.visual(
        Box((flap_w, flap_t, flap_h)),
        origin=Origin(xyz=(0.0, 0.0, -flap_h * 0.5)),
        material=smoked_flap,
        name="rear_panel",
    )
    rear_flap.visual(
        Box((flap_w * 0.96, flap_t + 0.002, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=gasket_dark,
        name="rear_top_rail",
    )
    rear_flap.visual(
        Box((flap_w * 0.98, flap_t + 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -(flap_h - 0.007))),
        material=gasket_dark,
        name="rear_bottom_weight",
    )

    lock_bar = model.part("lock_bar")
    lock_bar.inertial = Inertial.from_geometry(
        Box((0.014, 0.012, bar_len + 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -(bar_len * 0.5 - 0.004))),
    )
    lock_bar.visual(
        Box((bar_w, bar_d, bar_len)),
        origin=Origin(xyz=(0.0, 0.0, -bar_len * 0.5)),
        material=bar_dark,
        name="bar_shaft",
    )
    lock_bar.visual(
        Box((0.014, 0.012, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=bar_grip,
        name="bar_knob",
    )
    lock_bar.visual(
        Box((0.010, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -(bar_len - 0.004))),
        material=bar_dark,
        name="bar_foot",
    )

    model.articulation(
        "front_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_flap,
        origin=Origin(xyz=(0.0, front_flap_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "rear_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_flap,
        origin=Origin(xyz=(0.0, rear_flap_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "lock_bar_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lock_bar,
        origin=Origin(xyz=(bar_x, 0.0, bar_joint_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.25, lower=0.0, upper=0.102),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_flap = object_model.get_part("front_flap")
    rear_flap = object_model.get_part("rear_flap")
    lock_bar = object_model.get_part("lock_bar")
    front_hinge = object_model.get_articulation("front_flap_hinge")
    rear_hinge = object_model.get_articulation("rear_flap_hinge")
    bar_slide = object_model.get_articulation("lock_bar_slide")

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_overlap(
        front_flap,
        frame,
        axes="xz",
        elem_a="front_panel",
        min_overlap=0.18,
        name="front flap spans the frame opening",
    )
    ctx.expect_overlap(
        rear_flap,
        frame,
        axes="xz",
        elem_a="rear_panel",
        min_overlap=0.18,
        name="rear flap spans the frame opening",
    )
    ctx.expect_gap(
        front_flap,
        rear_flap,
        axis="y",
        positive_elem="front_panel",
        negative_elem="rear_panel",
        min_gap=0.012,
        max_gap=0.018,
        name="front and rear flaps sit on separate depth planes",
    )

    front_rest = aabb_center(ctx.part_element_world_aabb(front_flap, elem="front_panel"))
    with ctx.pose({front_hinge: 0.75}):
        front_open = aabb_center(ctx.part_element_world_aabb(front_flap, elem="front_panel"))
    ctx.check(
        "front flap swings toward the outer side",
        front_rest is not None
        and front_open is not None
        and front_open[1] > front_rest[1] + 0.06
        and front_open[2] > front_rest[2] + 0.025,
        details=f"rest={front_rest}, opened={front_open}",
    )

    rear_rest = aabb_center(ctx.part_element_world_aabb(rear_flap, elem="rear_panel"))
    with ctx.pose({rear_hinge: -0.75}):
        rear_open = aabb_center(ctx.part_element_world_aabb(rear_flap, elem="rear_panel"))
    ctx.check(
        "rear flap swings toward the inner side",
        rear_rest is not None
        and rear_open is not None
        and rear_open[1] < rear_rest[1] - 0.06
        and rear_open[2] > rear_rest[2] + 0.025,
        details=f"rest={rear_rest}, opened={rear_open}",
    )

    bar_rest = aabb_center(ctx.part_element_world_aabb(lock_bar, elem="bar_shaft"))
    bar_upper = 0.102
    if bar_slide.motion_limits is not None and bar_slide.motion_limits.upper is not None:
        bar_upper = bar_slide.motion_limits.upper
    with ctx.pose({bar_slide: bar_upper}):
        bar_lowered = aabb_center(ctx.part_element_world_aabb(lock_bar, elem="bar_shaft"))
        ctx.expect_gap(
            lock_bar,
            front_flap,
            axis="x",
            positive_elem="bar_shaft",
            negative_elem="front_panel",
            min_gap=0.004,
            max_gap=0.010,
            name="lowered lock bar clears the front flap edge",
        )
        ctx.expect_gap(
            lock_bar,
            rear_flap,
            axis="x",
            positive_elem="bar_shaft",
            negative_elem="rear_panel",
            min_gap=0.004,
            max_gap=0.010,
            name="lowered lock bar clears the rear flap edge",
        )
    ctx.check(
        "lock bar drops straight down through the side guides",
        bar_rest is not None
        and bar_lowered is not None
        and bar_lowered[2] < bar_rest[2] - 0.09
        and abs(bar_lowered[0] - bar_rest[0]) < 1e-6
        and abs(bar_lowered[1] - bar_rest[1]) < 1e-6,
        details=f"raised={bar_rest}, lowered={bar_lowered}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
