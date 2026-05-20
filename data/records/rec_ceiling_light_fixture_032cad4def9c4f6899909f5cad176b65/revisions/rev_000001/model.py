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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oyster_ceiling_light")

    housing_white = model.material("housing_white", rgba=(0.95, 0.95, 0.96, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.74, 0.75, 0.78, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.93, 0.95, 0.98, 0.42))
    socket_gray = model.material("socket_gray", rgba=(0.43, 0.43, 0.45, 1.0))
    warm_bulb = model.material("warm_bulb", rgba=(0.99, 0.94, 0.76, 0.30))

    housing_radius = 0.162
    shell_radius = 0.164
    hinge_x = 0.126
    hinge_y = -0.110
    hinge_z = 0.052

    housing = model.part("housing")

    housing.visual(
        Cylinder(radius=housing_radius, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=housing_white,
        name="top_plate",
    )
    housing_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (shell_radius, 0.004),
            (shell_radius, 0.012),
            (0.160, 0.022),
            (0.154, 0.028),
        ],
        inner_profile=[
            (0.156, 0.004),
            (0.156, 0.012),
            (0.153, 0.020),
            (0.149, 0.026),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    housing.visual(
        mesh_from_geometry(housing_shell, "oyster_housing_shell_v5"),
        material=housing_white,
        name="housing_shell",
    )
    housing.visual(
        Cylinder(radius=0.031, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=socket_gray,
        name="socket_body",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=socket_gray,
        name="bulb_neck",
    )
    housing.visual(
        Sphere(radius=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=warm_bulb,
        name="bulb_globe",
    )
    housing.inertial = Inertial.from_geometry(
        Cylinder(radius=shell_radius, length=0.035),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    diffuser = model.part("diffuser")

    diffuser_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.149, 0.000),
            (0.145, 0.012),
            (0.132, 0.042),
            (0.102, 0.085),
            (0.056, 0.118),
            (0.010, 0.132),
        ],
        inner_profile=[
            (0.145, 0.001),
            (0.141, 0.011),
            (0.128, 0.039),
            (0.099, 0.081),
            (0.055, 0.113),
            (0.007, 0.126),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    diffuser.visual(
        mesh_from_geometry(diffuser_shell, "oyster_glass_bowl_v5"),
        origin=Origin(xyz=(0.0, 0.120, 0.006)),
        material=frosted_glass,
        name="glass_bowl",
    )
    for side_name, x_pos in (("left", -hinge_x), ("right", hinge_x)):
        diffuser.visual(
            Box((0.028, 0.022, 0.010)),
            origin=Origin(
                xyz=(x_pos - math.copysign(0.008, x_pos), 0.010, 0.007),
            ),
            material=trim_metal,
            name=f"{side_name}_hinge_tab",
        )
        diffuser.visual(
            Cylinder(radius=0.0055, length=0.018),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim_metal,
            name=f"{side_name}_hinge_sleeve",
        )
    diffuser.inertial = Inertial.from_geometry(
        Cylinder(radius=0.149, length=0.132),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.120, 0.066)),
    )

    model.articulation(
        "housing_to_diffuser",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=diffuser,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        pin_part = model.part(f"{side_name}_hinge_pin_part")
        pin_part.visual(
            Cylinder(radius=0.0035, length=0.024),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_metal,
            name="pin",
        )
        pin_part.visual(
            Box((0.008, 0.016, 0.030)),
            origin=Origin(
                xyz=(-side_sign * 0.015, 0.008, -0.014),
            ),
            material=trim_metal,
            name="anchor",
        )
        pin_part.inertial = Inertial.from_geometry(
            Box((0.028, 0.016, 0.030)),
            mass=0.05,
            origin=Origin(xyz=(-side_sign * 0.008, 0.006, -0.010)),
        )
        model.articulation(
            f"housing_to_{side_name}_hinge_pin",
            ArticulationType.FIXED,
            parent=housing,
            child=pin_part,
            origin=Origin(xyz=(side_sign * hinge_x, hinge_y, hinge_z)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    diffuser = object_model.get_part("diffuser")
    left_pin = object_model.get_part("left_hinge_pin_part")
    right_pin = object_model.get_part("right_hinge_pin_part")
    hinge = object_model.get_articulation("housing_to_diffuser")

    housing.get_visual("top_plate")
    housing.get_visual("housing_shell")
    housing.get_visual("bulb_globe")
    diffuser.get_visual("glass_bowl")
    diffuser.get_visual("left_hinge_tab")
    diffuser.get_visual("left_hinge_sleeve")
    diffuser.get_visual("right_hinge_tab")
    diffuser.get_visual("right_hinge_sleeve")
    left_pin.get_visual("pin")
    left_pin.get_visual("anchor")
    right_pin.get_visual("pin")
    right_pin.get_visual("anchor")

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
    ctx.allow_overlap(
        left_pin,
        diffuser,
        elem_a="pin",
        elem_b="left_hinge_sleeve",
        reason="Left hinge pin sits inside the captured diffuser hinge sleeve.",
    )
    ctx.allow_overlap(
        right_pin,
        diffuser,
        elem_a="pin",
        elem_b="right_hinge_sleeve",
        reason="Right hinge pin sits inside the captured diffuser hinge sleeve.",
    )
    ctx.allow_overlap(
        housing,
        left_pin,
        reason="Left hinge pin anchor is press-fit into the housing rim.",
    )
    ctx.allow_overlap(
        housing,
        right_pin,
        reason="Right hinge pin anchor is press-fit into the housing rim.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            left_pin,
            diffuser,
            elem_a="pin",
            elem_b="left_hinge_sleeve",
            name="left_hinge_closed_contact",
        )
        ctx.expect_contact(
            right_pin,
            diffuser,
            elem_a="pin",
            elem_b="right_hinge_sleeve",
            name="right_hinge_closed_contact",
        )
        ctx.expect_contact(housing, left_pin, name="left_pin_mounted_to_housing")
        ctx.expect_contact(housing, right_pin, name="right_pin_mounted_to_housing")
        ctx.expect_overlap(
            diffuser,
            housing,
            axes="xy",
            min_overlap=0.26,
            name="diffuser_closed_covers_housing",
        )
        ctx.expect_gap(
            diffuser,
            housing,
            axis="z",
            positive_elem="glass_bowl",
            negative_elem="housing_shell",
            min_gap=0.02,
            max_gap=0.04,
            name="diffuser_closed_below_housing_rim",
        )
        ctx.expect_within(
            diffuser,
            housing,
            axes="xy",
            margin=0.02,
            name="diffuser_closed_within_housing_footprint",
        )

    limits = hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="hinge_lower_no_floating")

        closed_bowl = ctx.part_element_world_aabb(diffuser, elem="glass_bowl")
        with ctx.pose({hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="hinge_upper_no_floating")
            ctx.expect_contact(
                left_pin,
                diffuser,
                elem_a="pin",
                elem_b="left_hinge_sleeve",
                name="left_hinge_open_contact",
            )
            ctx.expect_contact(
                right_pin,
                diffuser,
                elem_a="pin",
                elem_b="right_hinge_sleeve",
                name="right_hinge_open_contact",
            )
            open_bowl = ctx.part_element_world_aabb(diffuser, elem="glass_bowl")

        if closed_bowl is not None and open_bowl is not None:
            closed_min_z = closed_bowl[0][2]
            closed_max_z = closed_bowl[1][2]
            open_min_z = open_bowl[0][2]
            open_max_z = open_bowl[1][2]
            ctx.check(
                "diffuser_swings_downward",
                open_max_z > closed_max_z + 0.08,
                details=(
                    f"Expected opened diffuser bowl to hang noticeably lower; "
                    f"closed_max_z={closed_max_z:.4f}, open_max_z={open_max_z:.4f}"
                ),
            )
            ctx.check(
                "diffuser_open_pose_drops_bottom_edge",
                open_min_z < closed_min_z - 0.02,
                details=(
                    f"Expected diffuser bottom edge to move meaningfully downward in the open pose; "
                    f"closed_min_z={closed_min_z:.4f}, open_min_z={open_min_z:.4f}"
                ),
            )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
