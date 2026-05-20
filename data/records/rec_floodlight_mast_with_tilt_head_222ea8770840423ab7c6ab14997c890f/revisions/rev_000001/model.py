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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="event_floodlight_tripod")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.90, 0.78, 0.16, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.80, 0.90, 0.98, 0.45))

    stand = model.part("stand")
    hub_center_z = 0.755
    hinge_z = 0.742
    post_top_z = 1.705

    stand.visual(
        Cylinder(radius=0.050, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, hub_center_z)),
        material=matte_black,
        name="hub_shell",
    )
    stand.visual(
        Cylinder(radius=0.080, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, hub_center_z - 0.060)),
        material=steel_gray,
        name="hub_collar",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 1.250)),
        material=matte_black,
        name="center_post",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.675)),
        material=steel_gray,
        name="top_socket",
    )
    stand.visual(
        Box((0.035, 0.085, 0.075)),
        origin=Origin(xyz=(-0.005, 0.0, 1.735)),
        material=steel_gray,
        name="tilt_knuckle",
    )
    stand.visual(
        Box((0.028, 0.075, 0.050)),
        origin=Origin(xyz=(0.018, 0.0, 1.780)),
        material=safety_yellow,
        name="yoke_neck",
    )
    stand.visual(
        Box((0.024, 0.308, 0.022)),
        origin=Origin(xyz=(0.028, 0.0, 1.804)),
        material=safety_yellow,
        name="yoke_crossbar",
    )
    stand.visual(
        Box((0.016, 0.016, 0.194)),
        origin=Origin(xyz=(0.032, 0.148, 1.701)),
        material=safety_yellow,
        name="yoke_arm_left",
    )
    stand.visual(
        Box((0.016, 0.016, 0.194)),
        origin=Origin(xyz=(0.032, -0.148, 1.701)),
        material=safety_yellow,
        name="yoke_arm_right",
    )

    hinge_radius = 0.078
    hinge_mount_radius = 0.057
    hinge_ear_offset = 0.016

    leg_angles = (
        ("front_leg", 0.0),
        ("rear_left_leg", 2.0 * math.pi / 3.0),
        ("rear_right_leg", 4.0 * math.pi / 3.0),
    )

    stand.visual(
        Box((0.052, 0.042, 0.016)),
        origin=Origin(xyz=(hinge_mount_radius, 0.0, hinge_z - 0.023), rpy=(0.0, 0.0, 0.0)),
        material=steel_gray,
        name="leg_hinge_mount_0",
    )
    stand.visual(
        Box((0.032, 0.006, 0.034)),
        origin=Origin(
            xyz=(hinge_radius, -hinge_ear_offset, hinge_z),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=steel_gray,
        name="leg_hinge_ear_0_in",
    )
    stand.visual(
        Box((0.032, 0.006, 0.034)),
        origin=Origin(
            xyz=(hinge_radius, hinge_ear_offset, hinge_z),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=steel_gray,
        name="leg_hinge_ear_0_out",
    )
    stand.visual(
        Box((0.052, 0.042, 0.016)),
        origin=Origin(
            xyz=(
                hinge_mount_radius * math.cos(2.0 * math.pi / 3.0),
                hinge_mount_radius * math.sin(2.0 * math.pi / 3.0),
                hinge_z - 0.023,
            ),
            rpy=(0.0, 0.0, 2.0 * math.pi / 3.0),
        ),
        material=steel_gray,
        name="leg_hinge_mount_1",
    )
    stand.visual(
        Box((0.032, 0.006, 0.034)),
        origin=Origin(
            xyz=(
                hinge_radius * math.cos(2.0 * math.pi / 3.0)
                + hinge_ear_offset * math.cos(math.pi / 6.0),
                hinge_radius * math.sin(2.0 * math.pi / 3.0)
                + hinge_ear_offset * math.sin(math.pi / 6.0),
                hinge_z,
            ),
            rpy=(0.0, 0.0, 2.0 * math.pi / 3.0),
        ),
        material=steel_gray,
        name="leg_hinge_ear_1_in",
    )
    stand.visual(
        Box((0.032, 0.006, 0.034)),
        origin=Origin(
            xyz=(
                hinge_radius * math.cos(2.0 * math.pi / 3.0)
                - hinge_ear_offset * math.cos(math.pi / 6.0),
                hinge_radius * math.sin(2.0 * math.pi / 3.0)
                - hinge_ear_offset * math.sin(math.pi / 6.0),
                hinge_z,
            ),
            rpy=(0.0, 0.0, 2.0 * math.pi / 3.0),
        ),
        material=steel_gray,
        name="leg_hinge_ear_1_out",
    )
    stand.visual(
        Box((0.052, 0.042, 0.016)),
        origin=Origin(
            xyz=(
                hinge_mount_radius * math.cos(4.0 * math.pi / 3.0),
                hinge_mount_radius * math.sin(4.0 * math.pi / 3.0),
                hinge_z - 0.023,
            ),
            rpy=(0.0, 0.0, 4.0 * math.pi / 3.0),
        ),
        material=steel_gray,
        name="leg_hinge_mount_2",
    )
    stand.visual(
        Box((0.032, 0.006, 0.034)),
        origin=Origin(
            xyz=(
                hinge_radius * math.cos(4.0 * math.pi / 3.0)
                - hinge_ear_offset * math.cos(math.pi / 6.0),
                hinge_radius * math.sin(4.0 * math.pi / 3.0)
                + hinge_ear_offset * math.sin(math.pi / 6.0),
                hinge_z,
            ),
            rpy=(0.0, 0.0, 4.0 * math.pi / 3.0),
        ),
        material=steel_gray,
        name="leg_hinge_ear_2_in",
    )
    stand.visual(
        Box((0.032, 0.006, 0.034)),
        origin=Origin(
            xyz=(
                hinge_radius * math.cos(4.0 * math.pi / 3.0)
                + hinge_ear_offset * math.cos(math.pi / 6.0),
                hinge_radius * math.sin(4.0 * math.pi / 3.0)
                - hinge_ear_offset * math.sin(math.pi / 6.0),
                hinge_z,
            ),
            rpy=(0.0, 0.0, 4.0 * math.pi / 3.0),
        ),
        material=steel_gray,
        name="leg_hinge_ear_2_out",
    )

    stand.inertial = Inertial.from_geometry(
        Box((1.320, 1.320, 1.860)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.0, 0.930)),
    )

    for leg_name, angle in leg_angles:
        leg = model.part(leg_name)

        leg.visual(
            Cylinder(radius=0.0115, length=0.026),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_gray,
            name="hinge_sleeve",
        )

        leg_tube = tube_from_spline_points(
            [
                (0.006, 0.0, 0.0),
                (0.110, 0.0, -0.075),
                (0.330, 0.0, -0.385),
                (0.570, 0.0, -0.717),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        leg.visual(
            mesh_from_geometry(leg_tube, f"{leg_name}_tube"),
            material=matte_black,
            name="main_tube",
        )
        leg.visual(
            Box((0.065, 0.016, 0.040)),
            origin=Origin(xyz=(0.060, 0.0, -0.045), rpy=(0.0, 0.55, 0.0)),
            material=steel_gray,
            name="brace_block",
        )
        leg.visual(
            Cylinder(radius=0.019, length=0.030),
            origin=Origin(xyz=(0.570, 0.0, -0.732)),
            material=rubber_black,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.620, 0.055, 0.760)),
            mass=0.55,
            origin=Origin(xyz=(0.285, 0.0, -0.360)),
        )

        model.articulation(
            f"{leg_name}_hinge",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), hinge_z),
                rpy=(0.0, 0.0, angle),
            ),
            # At q=0 the leg is fully splayed. Negative motion folds it back
            # toward the post; increasing q toward 0 opens it outward.
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=1.2,
                lower=-0.55,
                upper=0.0,
            ),
        )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Box((0.092, 0.262, 0.202)),
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
        material=matte_black,
        name="housing",
    )
    lamp_head.visual(
        Box((0.014, 0.274, 0.214)),
        origin=Origin(xyz=(0.097, 0.0, 0.0)),
        material=safety_yellow,
        name="front_bezel",
    )
    lamp_head.visual(
        Box((0.004, 0.242, 0.184)),
        origin=Origin(xyz=(0.104, 0.0, 0.0)),
        material=glass_tint,
        name="front_glass",
    )
    lamp_head.visual(
        Box((0.032, 0.012, 0.138)),
        origin=Origin(xyz=(0.0, 0.134, 0.0)),
        material=safety_yellow,
        name="side_mount_left",
    )
    lamp_head.visual(
        Box((0.032, 0.012, 0.138)),
        origin=Origin(xyz=(0.0, -0.134, 0.0)),
        material=safety_yellow,
        name="side_mount_right",
    )

    for index, y in enumerate((-0.086, -0.043, 0.0, 0.043, 0.086)):
        lamp_head.visual(
            Box((0.010, 0.014, 0.166)),
            origin=Origin(xyz=(0.005, y, 0.0)),
            material=steel_gray,
            name=f"heat_sink_fin_{index}",
        )

    lamp_head.visual(
        Box((0.012, 0.170, 0.010)),
        origin=Origin(xyz=(0.056, 0.0, 0.123)),
        material=steel_gray,
        name="carry_handle_bar",
    )
    lamp_head.visual(
        Box((0.012, 0.010, 0.054)),
        origin=Origin(xyz=(0.056, 0.070, 0.096)),
        material=steel_gray,
        name="carry_handle_post_left",
    )
    lamp_head.visual(
        Box((0.012, 0.010, 0.054)),
        origin=Origin(xyz=(0.056, -0.070, 0.096)),
        material=steel_gray,
        name="carry_handle_post_right",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.120, 0.290, 0.250)),
        mass=1.8,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_lamp_head",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lamp_head,
        origin=Origin(xyz=(0.040, 0.0, post_top_z)),
        # The floodlight body extends forward along local +X from the hinge line.
        # -Y makes positive tilt raise the front of the lamp.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=-0.70,
            upper=0.55,
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

    stand = object_model.get_part("stand")
    lamp_head = object_model.get_part("lamp_head")
    head_tilt = object_model.get_articulation("stand_to_lamp_head")
    leg_joints = [
        object_model.get_articulation("front_leg_hinge"),
        object_model.get_articulation("rear_left_leg_hinge"),
        object_model.get_articulation("rear_right_leg_hinge"),
    ]
    leg_parts = [
        object_model.get_part("front_leg"),
        object_model.get_part("rear_left_leg"),
        object_model.get_part("rear_right_leg"),
    ]

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        part = object_model.get_part(part_name)
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    with ctx.pose({head_tilt: 0.0}):
        glass_rest = elem_center("lamp_head", "front_glass")
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        glass_up = elem_center("lamp_head", "front_glass")
    with ctx.pose({head_tilt: head_tilt.motion_limits.lower}):
        glass_down = elem_center("lamp_head", "front_glass")

    ctx.check(
        "lamp head tilts upward at positive angles",
        glass_rest is not None
        and glass_up is not None
        and glass_up[2] > glass_rest[2] + 0.04,
        details=f"rest={glass_rest}, up={glass_up}",
    )
    ctx.check(
        "lamp head tilts downward at negative angles",
        glass_rest is not None
        and glass_down is not None
        and glass_down[2] < glass_rest[2] - 0.04,
        details=f"rest={glass_rest}, down={glass_down}",
    )

    for leg_part, leg_joint in zip(leg_parts, leg_joints):
        with ctx.pose({leg_joint: 0.0}):
            foot_rest = elem_center(leg_part.name, "foot_pad")
        with ctx.pose({leg_joint: leg_joint.motion_limits.lower}):
            foot_folded = elem_center(leg_part.name, "foot_pad")

        rest_radius = None if foot_rest is None else math.hypot(foot_rest[0], foot_rest[1])
        folded_radius = (
            None if foot_folded is None else math.hypot(foot_folded[0], foot_folded[1])
        )

        ctx.check(
            f"{leg_part.name} is visibly splayed in the default pose",
            rest_radius is not None and rest_radius > 0.52,
            details=f"foot_center={foot_rest}, radial_distance={rest_radius}",
        )
        ctx.check(
            f"{leg_part.name} folds back toward the post",
            rest_radius is not None
            and folded_radius is not None
            and folded_radius < rest_radius - 0.20,
            details=f"rest={foot_rest}, folded={foot_folded}",
        )

    ctx.expect_contact(
        "front_leg",
        stand,
        elem_a="hinge_sleeve",
        elem_b="leg_hinge_ear_0_in",
        name="front leg hinge sleeve seats against its hub ear",
    )
    ctx.expect_contact(
        "rear_left_leg",
        stand,
        elem_a="hinge_sleeve",
        elem_b="leg_hinge_ear_1_in",
        name="rear left leg hinge sleeve seats against its hub ear",
    )
    ctx.expect_contact(
        "rear_right_leg",
        stand,
        elem_a="hinge_sleeve",
        elem_b="leg_hinge_ear_2_in",
        name="rear right leg hinge sleeve seats against its hub ear",
    )

    ctx.expect_contact(
        lamp_head,
        stand,
        elem_a="side_mount_left",
        elem_b="yoke_arm_left",
        name="lamp head left side bracket rides on the yoke arm",
    )
    ctx.expect_contact(
        lamp_head,
        stand,
        elem_a="side_mount_right",
        elem_b="yoke_arm_right",
        name="lamp head right side bracket rides on the yoke arm",
    )

    ctx.expect_gap(
        lamp_head,
        stand,
        axis="x",
        min_gap=-0.010,
        max_gap=0.090,
        positive_elem="housing",
        negative_elem="tilt_knuckle",
        name="lamp head stays mounted just forward of the post-top knuckle",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
