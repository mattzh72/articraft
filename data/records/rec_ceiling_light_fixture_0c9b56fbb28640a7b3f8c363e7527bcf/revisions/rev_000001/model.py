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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_gooseneck_spotlight")

    housing_white = model.material("housing_white", rgba=(0.93, 0.94, 0.95, 1.0))
    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.30, 0.34, 0.38, 0.85))

    housing = model.part("housing")
    housing.visual(
        Box((0.29, 0.17, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=housing_white,
        name="ceiling_flange",
    )
    housing.visual(
        Box((0.26, 0.14, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=housing_white,
        name="housing_body",
    )
    housing.visual(
        Box((0.224, 0.104, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=matte_black,
        name="driver_cover",
    )
    housing.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=matte_black,
        name="swivel_mount_plate",
    )

    arm_assembly = model.part("arm_assembly")
    arm_assembly.visual(
        Cylinder(radius=0.027, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=satin_black,
        name="base_flange",
    )
    arm_assembly.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=matte_black,
        name="swivel_collar",
    )

    gooseneck_geom = tube_from_spline_points(
        [
            (0.0, 0.0, -0.026),
            (0.010, 0.0, -0.055),
            (0.028, 0.0, -0.105),
            (0.044, 0.0, -0.168),
            (0.052, 0.0, -0.216),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    arm_assembly.visual(
        mesh_from_geometry(gooseneck_geom, "gooseneck_arm"),
        material=matte_black,
        name="gooseneck",
    )
    arm_assembly.visual(
        Box((0.022, 0.056, 0.012)),
        origin=Origin(xyz=(0.052, 0.0, -0.214)),
        material=satin_black,
        name="fork_crown",
    )
    arm_assembly.visual(
        Box((0.018, 0.006, 0.028)),
        origin=Origin(xyz=(0.052, -0.028, -0.218)),
        material=satin_black,
        name="fork_left",
    )
    arm_assembly.visual(
        Box((0.018, 0.006, 0.028)),
        origin=Origin(xyz=(0.052, 0.028, -0.218)),
        material=satin_black,
        name="fork_right",
    )

    spot_head = model.part("spot_head")
    spot_head.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="head_trunnion",
    )
    spot_head.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=satin_black,
        name="rear_cap",
    )
    spot_head.visual(
        Cylinder(radius=0.034, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=matte_black,
        name="head_barrel",
    )
    spot_head.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=satin_black,
        name="front_bezel",
    )
    spot_head.visual(
        Cylinder(radius=0.029, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=matte_black,
        name="reflector_core",
    )
    spot_head.visual(
        Cylinder(radius=0.027, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=dark_glass,
        name="lens",
    )

    model.articulation(
        "housing_to_arm",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm_assembly,
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-math.radians(110.0),
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.REVOLUTE,
        parent=arm_assembly,
        child=spot_head,
        origin=Origin(xyz=(0.052, 0.0, -0.232)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-math.radians(70.0),
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    arm_assembly = object_model.get_part("arm_assembly")
    spot_head = object_model.get_part("spot_head")
    housing_to_arm = object_model.get_articulation("housing_to_arm")
    arm_to_head = object_model.get_articulation("arm_to_head")
    head_barrel = spot_head.get_visual("head_barrel")

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

    ctx.check(
        "base_joint_axis_is_vertical",
        housing_to_arm.axis == (0.0, 0.0, 1.0),
        details=f"axis={housing_to_arm.axis}",
    )
    ctx.check(
        "head_joint_axis_is_horizontal",
        arm_to_head.axis == (0.0, 1.0, 0.0),
        details=f"axis={arm_to_head.axis}",
    )

    ctx.expect_contact(arm_assembly, housing, name="arm_base_contacts_housing")
    ctx.expect_contact(spot_head, arm_assembly, name="head_trunnion_contacts_fork")
    ctx.expect_gap(
        housing,
        spot_head,
        axis="z",
        min_gap=0.14,
        name="head_hangs_below_housing",
    )

    rest_head_origin = ctx.part_world_position(spot_head)
    assert rest_head_origin is not None
    rest_barrel_aabb = ctx.part_element_world_aabb(spot_head, elem=head_barrel)
    assert rest_barrel_aabb is not None

    with ctx.pose({housing_to_arm: math.radians(50.0)}):
        turned_head_origin = ctx.part_world_position(spot_head)
        assert turned_head_origin is not None
        ctx.check(
            "base_swivel_rotates_arm_assembly",
            turned_head_origin[1] > rest_head_origin[1] + 0.03,
            details=f"rest={rest_head_origin}, turned={turned_head_origin}",
        )
        ctx.expect_contact(arm_assembly, housing, name="arm_keeps_mount_contact_when_swiveled")

    with ctx.pose({arm_to_head: math.radians(30.0)}):
        tilted_barrel_aabb = ctx.part_element_world_aabb(spot_head, elem=head_barrel)
        assert tilted_barrel_aabb is not None
        ctx.check(
            "head_tilt_swings_barrel_forward",
            tilted_barrel_aabb[0][0] < rest_barrel_aabb[0][0] - 0.025,
            details=f"rest={rest_barrel_aabb}, tilted={tilted_barrel_aabb}",
        )
        ctx.expect_contact(spot_head, arm_assembly, name="head_keeps_pivot_contact_when_tilted")
        ctx.expect_gap(
            housing,
            spot_head,
            axis="z",
            min_gap=0.12,
            name="tilted_head_clears_housing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
