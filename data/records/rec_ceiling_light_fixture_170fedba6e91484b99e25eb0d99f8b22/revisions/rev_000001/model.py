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
)


TRACK_LENGTH = 1.40
TRACK_WIDTH = 0.038
TRACK_BODY_HEIGHT = 0.022
TRACK_LIP_WIDTH = 0.008
TRACK_LIP_HEIGHT = 0.006
TRACK_BOTTOM_Z = 0.0
COLLAR_ORIGIN_Z = -0.012
SLIDE_TRAVEL = 0.09


def _add_collar(part, material) -> None:
    part.visual(
        Box((0.066, 0.032, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=material,
        name="slider_body",
    )
    part.visual(
        Box((0.012, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.007, 0.000)),
        material=material,
        name="left_cheek",
    )
    part.visual(
        Box((0.012, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.007, 0.000)),
        material=material,
        name="right_cheek",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.066, 0.032, 0.024)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )


def _add_head(part, housing_material, trim_material, lens_material) -> None:
    part.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="hinge_barrel",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=trim_material,
        name="neck_tube",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=housing_material,
        name="lamp_body",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.083)),
        material=trim_material,
        name="bezel",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
        material=lens_material,
        name="lens",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.064, 0.056, 0.088)),
        mass=0.52,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_lighting_system")

    ceiling_white = model.material("ceiling_white", rgba=(0.95, 0.95, 0.94, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.64, 0.65, 0.67, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.76, 0.83, 0.88, 0.65))

    rail = model.part("track_rail")
    rail.visual(
        Box((TRACK_LENGTH + 0.04, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=ceiling_white,
        name="ceiling_plate",
    )
    rail.visual(
        Box((TRACK_LENGTH, TRACK_WIDTH, TRACK_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=satin_black,
        name="rail_body",
    )
    rail.visual(
        Box((TRACK_LENGTH, TRACK_LIP_WIDTH, TRACK_LIP_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.015, 0.003)),
        material=satin_black,
        name="left_lip",
    )
    rail.visual(
        Box((TRACK_LENGTH, TRACK_LIP_WIDTH, TRACK_LIP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.015, 0.003)),
        material=satin_black,
        name="right_lip",
    )
    rail.inertial = Inertial.from_geometry(
        Box((TRACK_LENGTH + 0.04, 0.050, 0.034)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    head_specs = (
        ("left", -0.48),
        ("center", 0.0),
        ("right", 0.48),
    )
    for label, x_pos in head_specs:
        collar = model.part(f"{label}_collar")
        _add_collar(collar, warm_grey)

        head = model.part(f"{label}_head")
        _add_head(head, matte_black, warm_grey, lens_glass)

        model.articulation(
            f"rail_to_{label}_collar",
            ArticulationType.PRISMATIC,
            parent=rail,
            child=collar,
            origin=Origin(xyz=(x_pos, 0.0, COLLAR_ORIGIN_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.20,
                lower=-SLIDE_TRAVEL,
                upper=SLIDE_TRAVEL,
            ),
        )
        model.articulation(
            f"{label}_collar_to_{label}_head",
            ArticulationType.REVOLUTE,
            parent=collar,
            child=head,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.5,
                lower=math.radians(-55.0),
                upper=math.radians(20.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    rail = object_model.get_part("track_rail")
    left_collar = object_model.get_part("left_collar")
    center_collar = object_model.get_part("center_collar")
    right_collar = object_model.get_part("right_collar")
    left_head = object_model.get_part("left_head")
    center_head = object_model.get_part("center_head")
    right_head = object_model.get_part("right_head")

    slide_joints = [
        object_model.get_articulation("rail_to_left_collar"),
        object_model.get_articulation("rail_to_center_collar"),
        object_model.get_articulation("rail_to_right_collar"),
    ]
    tilt_joints = [
        object_model.get_articulation("left_collar_to_left_head"),
        object_model.get_articulation("center_collar_to_center_head"),
        object_model.get_articulation("right_collar_to_right_head"),
    ]
    assemblies = [
        ("left", left_collar, left_head, slide_joints[0], tilt_joints[0]),
        ("center", center_collar, center_head, slide_joints[1], tilt_joints[1]),
        ("right", right_collar, right_head, slide_joints[2], tilt_joints[2]),
    ]

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0005)
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

    rail_aabb = ctx.part_world_aabb(rail)
    rail_bottom = TRACK_BOTTOM_Z
    if rail_aabb is not None:
        rail_length = rail_aabb[1][0] - rail_aabb[0][0]
        rail_height = rail_aabb[1][2] - rail_aabb[0][2]
        rail_bottom = rail_aabb[0][2]
        ctx.check(
            "rail_length_realistic",
            rail_length >= 1.35,
            f"Track rail should read as a long ceiling rail, got {rail_length:.3f} m.",
        )
        ctx.check(
            "rail_profile_shallow",
            rail_height <= 0.040,
            f"Track rail should remain shallow, got {rail_height:.3f} m tall.",
        )
    else:
        ctx.fail("rail_has_aabb", "Could not measure the track rail.")

    def _aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    ctx.expect_origin_distance(left_collar, right_collar, axes="x", min_dist=0.75, name="outer_collars_spread")
    ctx.expect_origin_distance(left_head, right_head, axes="x", min_dist=0.75, name="outer_heads_spread")

    for label, collar, head, slide_joint, tilt_joint in assemblies:
        ctx.expect_contact(collar, rail, contact_tol=0.0005, name=f"{label}_collar_contacts_track")
        ctx.expect_gap(
            rail,
            collar,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"{label}_collar_seats_under_track",
        )
        ctx.expect_overlap(
            collar,
            rail,
            axes="xy",
            min_overlap=0.020,
            name=f"{label}_collar_overlaps_track_footprint",
        )
        ctx.expect_within(
            collar,
            rail,
            axes="x",
            margin=0.020,
            name=f"{label}_collar_within_track_length",
        )
        ctx.expect_contact(head, collar, contact_tol=0.0005, name=f"{label}_head_contacts_collar")

        lens_rest_aabb = ctx.part_element_world_aabb(head, elem="lens")
        lens_rest = _aabb_center(lens_rest_aabb) if lens_rest_aabb is not None else None
        lens_rest_below_ok = lens_rest_aabb is not None and lens_rest_aabb[1][2] < rail_bottom - 0.010
        ctx.check(
            f"{label}_lens_below_track_at_rest",
            lens_rest_below_ok,
            f"{label} lens should sit clearly below the track at rest.",
        )

        slide_limits = slide_joint.motion_limits
        if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
            rest_pos = ctx.part_world_position(collar)
            lower_pos = None
            upper_pos = None
            with ctx.pose({slide_joint: slide_limits.lower}):
                lower_pos = ctx.part_world_position(collar)
                ctx.expect_contact(collar, rail, contact_tol=0.0005, name=f"{label}_slide_lower_contact")
                ctx.expect_within(collar, rail, axes="x", margin=0.020, name=f"{label}_slide_lower_within")
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_slide_lower_no_overlap")
                ctx.fail_if_isolated_parts(contact_tol=0.0005, name=f"{label}_slide_lower_no_floating")
            with ctx.pose({slide_joint: slide_limits.upper}):
                upper_pos = ctx.part_world_position(collar)
                ctx.expect_contact(collar, rail, contact_tol=0.0005, name=f"{label}_slide_upper_contact")
                ctx.expect_within(collar, rail, axes="x", margin=0.020, name=f"{label}_slide_upper_within")
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_slide_upper_no_overlap")
                ctx.fail_if_isolated_parts(contact_tol=0.0005, name=f"{label}_slide_upper_no_floating")

            slide_motion_ok = False
            if rest_pos is not None and lower_pos is not None and upper_pos is not None:
                slide_motion_ok = (
                    lower_pos[0] < rest_pos[0] - 0.08
                    and upper_pos[0] > rest_pos[0] + 0.08
                    and abs(lower_pos[1] - rest_pos[1]) <= 1e-6
                    and abs(upper_pos[1] - rest_pos[1]) <= 1e-6
                    and abs(lower_pos[2] - rest_pos[2]) <= 1e-6
                    and abs(upper_pos[2] - rest_pos[2]) <= 1e-6
                )
            ctx.check(
                f"{label}_slider_moves_along_track_axis",
                slide_motion_ok,
                f"{label} collar should translate along x only with clear travel to both sides.",
            )

        tilt_limits = tilt_joint.motion_limits
        if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
            lens_lower_aabb = None
            lens_upper_aabb = None
            with ctx.pose({tilt_joint: tilt_limits.lower}):
                lens_lower_aabb = ctx.part_element_world_aabb(head, elem="lens")
                ctx.expect_contact(head, collar, contact_tol=0.0005, name=f"{label}_tilt_lower_hinge_contact")
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_tilt_lower_no_overlap")
                ctx.fail_if_isolated_parts(contact_tol=0.0005, name=f"{label}_tilt_lower_no_floating")
            with ctx.pose({tilt_joint: tilt_limits.upper}):
                lens_upper_aabb = ctx.part_element_world_aabb(head, elem="lens")
                ctx.expect_contact(head, collar, contact_tol=0.0005, name=f"{label}_tilt_upper_hinge_contact")
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_tilt_upper_no_overlap")
                ctx.fail_if_isolated_parts(contact_tol=0.0005, name=f"{label}_tilt_upper_no_floating")

            tilt_motion_ok = False
            tilt_clear_ok = False
            if lens_rest is not None and lens_lower_aabb is not None and lens_upper_aabb is not None:
                lens_rest = _aabb_center(lens_rest_aabb)
                lens_lower = _aabb_center(lens_lower_aabb)
                lens_upper = _aabb_center(lens_upper_aabb)
                tilt_motion_ok = abs(lens_lower[0] - lens_upper[0]) >= 0.050
                tilt_clear_ok = lens_lower_aabb[1][2] < rail_bottom - 0.001 and lens_upper_aabb[1][2] < rail_bottom - 0.001
            ctx.check(
                f"{label}_tilt_changes_aim",
                tilt_motion_ok,
                f"{label} spotlight lens should sweep fore-aft across the tilt range.",
            )
            ctx.check(
                f"{label}_tilt_keeps_head_below_track",
                tilt_clear_ok,
                f"{label} spotlight should remain below the rail through the tilt range.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
