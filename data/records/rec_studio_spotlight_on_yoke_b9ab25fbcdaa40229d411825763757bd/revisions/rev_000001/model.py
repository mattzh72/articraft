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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_yoke_stand")

    stand_black = model.material("stand_black", rgba=(0.12, 0.12, 0.13, 1.0))
    housing_black = model.material("housing_black", rgba=(0.18, 0.18, 0.19, 1.0))
    bracket_gray = model.material("bracket_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    metal_silver = model.material("metal_silver", rgba=(0.67, 0.69, 0.72, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.83, 0.93, 0.55))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.27, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=stand_black,
        name="floor_base",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=stand_black,
        name="base_riser",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.960),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=stand_black,
        name="mast_tube",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 1.125)),
        material=metal_silver,
        name="upper_riser",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.235)),
        material=stand_black,
        name="top_spigot",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.54, 0.54, 1.255)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.6275)),
    )

    bearing_module = model.part("bearing_module")
    bearing_module.visual(
        Cylinder(radius=0.066, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=bracket_gray,
        name="lower_housing",
    )
    bearing_module.visual(
        Cylinder(radius=0.082, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.1075)),
        material=metal_silver,
        name="upper_race",
    )
    bearing_module.visual(
        Box((0.072, 0.110, 0.080)),
        origin=Origin(xyz=(-0.082, 0.0, 0.060)),
        material=housing_black,
        name="driver_box",
    )
    bearing_module.inertial = Inertial.from_geometry(
        Box((0.236, 0.164, 0.125)),
        mass=2.2,
        origin=Origin(xyz=(-0.018, 0.0, 0.0625)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.072, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=bracket_gray,
        name="pan_hub",
    )
    yoke.visual(
        Box((0.046, 0.340, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=bracket_gray,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.028, 0.024, 0.380)),
        origin=Origin(xyz=(0.0, 0.144, 0.300)),
        material=bracket_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.028, 0.024, 0.380)),
        origin=Origin(xyz=(0.0, -0.144, 0.300)),
        material=bracket_gray,
        name="right_arm",
    )
    yoke.visual(
        Cylinder(radius=0.042, length=0.016),
        origin=Origin(
            xyz=(0.0, 0.152, 0.310),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_silver,
        name="left_tilt_washer",
    )
    yoke.visual(
        Cylinder(radius=0.042, length=0.016),
        origin=Origin(
            xyz=(0.0, -0.152, 0.310),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_silver,
        name="right_tilt_washer",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.144, 0.340, 0.490)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
    )

    head_module = model.part("head_module")
    can_outer_profile = [
        (0.028, -0.160),
        (0.078, -0.148),
        (0.095, -0.050),
        (0.102, 0.070),
        (0.122, 0.180),
        (0.145, 0.235),
    ]
    can_inner_profile = [
        (0.000, -0.160),
        (0.066, -0.145),
        (0.080, -0.050),
        (0.087, 0.072),
        (0.105, 0.182),
        (0.122, 0.235),
    ]
    can_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            can_outer_profile,
            can_inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "spotlight_can_shell",
    )
    head_module.visual(
        can_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_black,
        name="can_shell",
    )
    head_module.visual(
        Cylinder(radius=0.118, length=0.006),
        origin=Origin(
            xyz=(0.214, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_glass,
        name="lens_glass",
    )
    head_module.visual(
        Box((0.100, 0.108, 0.046)),
        origin=Origin(xyz=(-0.055, 0.0, 0.108)),
        material=bracket_gray,
        name="ballast_box",
    )
    head_module.visual(
        Cylinder(radius=0.050, length=0.032),
        origin=Origin(
            xyz=(0.0, 0.116, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_silver,
        name="left_trunnion",
    )
    head_module.visual(
        Cylinder(radius=0.050, length=0.032),
        origin=Origin(
            xyz=(0.0, -0.116, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_silver,
        name="right_trunnion",
    )
    head_module.inertial = Inertial.from_geometry(
        Box((0.395, 0.264, 0.300)),
        mass=5.8,
        origin=Origin(xyz=(0.0375, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=bearing_module,
        origin=Origin(xyz=(0.0, 0.0, 1.255)),
    )
    model.articulation(
        "bearing_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=bearing_module,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "yoke_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head_module,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=-math.radians(28.0),
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bearing = object_model.get_part("bearing_module")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head_module")
    pan = object_model.get_articulation("bearing_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_head_tilt")

    def aabb_center(aabb):
        if aabb is None:
            return None
        min_pt, max_pt = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))

    ctx.expect_gap(
        bearing,
        base,
        axis="z",
        positive_elem="lower_housing",
        negative_elem="top_spigot",
        max_gap=0.001,
        max_penetration=1e-5,
        name="bearing module seats on the stand spigot",
    )
    ctx.expect_gap(
        yoke,
        bearing,
        axis="z",
        positive_elem="pan_hub",
        negative_elem="upper_race",
        max_gap=0.001,
        max_penetration=1e-5,
        name="pan hub sits on the bearing race",
    )
    ctx.expect_overlap(
        yoke,
        bearing,
        axes="xy",
        elem_a="pan_hub",
        elem_b="upper_race",
        min_overlap=0.140,
        name="pan hub remains centered over the bearing race",
    )
    ctx.expect_contact(
        head,
        yoke,
        elem_a="left_trunnion",
        elem_b="left_arm",
        name="left trunnion is carried by the left yoke arm",
    )
    ctx.expect_contact(
        head,
        yoke,
        elem_a="right_trunnion",
        elem_b="right_arm",
        name="right trunnion is carried by the right yoke arm",
    )
    ctx.expect_gap(
        head,
        yoke,
        axis="z",
        positive_elem="can_shell",
        negative_elem="lower_bridge",
        min_gap=0.020,
        name="lamp can clears the lower yoke bridge at rest",
    )

    rest_lens = aabb_center(ctx.part_element_world_aabb(head, elem="lens_glass"))

    with ctx.pose({tilt: tilt.motion_limits.upper}):
        ctx.expect_gap(
            head,
            yoke,
            axis="z",
            positive_elem="can_shell",
            negative_elem="lower_bridge",
            min_gap=0.010,
            name="lamp can still clears the bridge at full upward tilt",
        )
        tilted_up_lens = aabb_center(ctx.part_element_world_aabb(head, elem="lens_glass"))

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        tilted_down_lens = aabb_center(ctx.part_element_world_aabb(head, elem="lens_glass"))

    with ctx.pose({pan: 1.2}):
        panned_lens = aabb_center(ctx.part_element_world_aabb(head, elem="lens_glass"))

    ctx.check(
        "tilt joint raises the beam at the upper stop",
        rest_lens is not None
        and tilted_up_lens is not None
        and tilted_up_lens[2] > rest_lens[2] + 0.18,
        details=f"rest={rest_lens}, upper={tilted_up_lens}",
    )
    ctx.check(
        "tilt joint lowers the beam at the lower stop",
        rest_lens is not None
        and tilted_down_lens is not None
        and tilted_down_lens[2] < rest_lens[2] - 0.08,
        details=f"rest={rest_lens}, lower={tilted_down_lens}",
    )
    ctx.check(
        "pan joint sweeps the lens sideways while keeping height",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[1] > rest_lens[1] + 0.16
        and abs(panned_lens[2] - rest_lens[2]) < 0.01,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
