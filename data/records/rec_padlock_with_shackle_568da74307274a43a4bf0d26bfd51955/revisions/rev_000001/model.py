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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    body_width = 0.048
    body_thickness = 0.026
    body_height = 0.058
    body_corner_radius = 0.008
    shackle_radius = 0.0055
    shackle_spacing = 0.026
    shackle_leg_length = 0.070
    shackle_well_radius = 0.0064
    shackle_well_depth = 0.028
    shackle_axis_z = body_height - shackle_well_depth
    keyway_z = 0.018
    cap_hinge_x = 0.0105
    cap_disc_radius = 0.0102
    cap_disc_thickness = 0.0032
    cap_barrel_radius = 0.0016
    well_tunnel_y = shackle_radius * 2.0
    face_slab_depth = (body_thickness - well_tunnel_y) * 0.5
    bore_side_clearance = 0.0007
    left_outer_width = body_width * 0.5 - (shackle_spacing * 0.5 + shackle_radius + bore_side_clearance)
    center_bridge_width = shackle_spacing - 2.0 * (shackle_radius + bore_side_clearance)
    captive_insert_depth = 0.018
    free_leg_bottom = 0.026
    base_core_height = 0.006

    def build_face_slab():
        profile = [
            (x, -z)
            for x, z in rounded_rect_profile(
                body_width,
                body_height,
                body_corner_radius,
                corner_segments=8,
            )
        ]
        slab = ExtrudeGeometry(profile, face_slab_depth, center=True)
        slab.rotate_x(-math.pi / 2.0)
        slab.translate(0.0, 0.0, body_height * 0.5)
        return slab

    def build_shackle_crown():
        return tube_from_spline_points(
            [
                (0.0, 0.0, shackle_leg_length - 0.004),
                (0.0, 0.0, shackle_leg_length + 0.006),
                (shackle_spacing * 0.5, 0.0, shackle_leg_length + 0.019),
                (shackle_spacing, 0.0, shackle_leg_length + 0.006),
                (shackle_spacing, 0.0, shackle_leg_length - 0.004),
            ],
            radius=shackle_radius,
            samples_per_segment=20,
            radial_segments=22,
            cap_ends=True,
        )

    model = ArticulatedObject(name="weatherproof_padlock")

    body_rubber = model.material("body_rubber", rgba=(0.10, 0.19, 0.34, 1.0))
    bumper_black = model.material("bumper_black", rgba=(0.08, 0.08, 0.09, 1.0))
    shackle_chrome = model.material("shackle_chrome", rgba=(0.86, 0.88, 0.91, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.60, 0.25, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(build_face_slab(), "padlock_front_shell"),
        material=body_rubber,
        origin=Origin(xyz=(0.0, -(body_thickness - face_slab_depth) * 0.5, 0.0)),
        name="front_shell",
    )
    body.visual(
        mesh_from_geometry(build_face_slab(), "padlock_rear_shell"),
        material=body_rubber,
        origin=Origin(xyz=(0.0, (body_thickness - face_slab_depth) * 0.5, 0.0)),
        name="rear_shell",
    )
    body.visual(
        Box((left_outer_width, well_tunnel_y, body_height)),
        origin=Origin(xyz=(-body_width * 0.5 + left_outer_width * 0.5, 0.0, body_height * 0.5)),
        material=body_rubber,
        name="left_center_web",
    )
    body.visual(
        Box((center_bridge_width, well_tunnel_y, body_height)),
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
        material=body_rubber,
        name="center_bridge",
    )
    body.visual(
        Box((left_outer_width, well_tunnel_y, body_height)),
        origin=Origin(xyz=(body_width * 0.5 - left_outer_width * 0.5, 0.0, body_height * 0.5)),
        material=body_rubber,
        name="right_center_web",
    )
    body.visual(
        Box((2.0 * shackle_radius, well_tunnel_y, shackle_axis_z)),
        origin=Origin(xyz=(shackle_spacing * 0.5, 0.0, shackle_axis_z * 0.5)),
        material=dark_metal,
        name="free_leg_latch_block",
    )
    body.visual(
        Box((0.038, 0.017, base_core_height)),
        origin=Origin(xyz=(0.0, 0.0, base_core_height * 0.5)),
        material=dark_metal,
        name="base_core",
    )
    body.visual(
        Box((0.005, 0.0026, 0.034)),
        origin=Origin(xyz=(-0.0175, body_thickness * 0.5 - 0.0013, 0.021)),
        material=bumper_black,
        name="left_bumper_strip",
    )
    body.visual(
        Box((0.005, 0.0026, 0.034)),
        origin=Origin(xyz=(0.0175, body_thickness * 0.5 - 0.0013, 0.021)),
        material=bumper_black,
        name="right_bumper_strip",
    )
    body.visual(
        Cylinder(radius=0.0094, length=0.0014),
        origin=Origin(
            xyz=(0.0, body_thickness * 0.5 + 0.0007, keyway_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="key_bezel",
    )
    body.visual(
        Cylinder(radius=0.0072, length=0.0008),
        origin=Origin(
            xyz=(0.0, body_thickness * 0.5 + 0.0004, keyway_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="key_plug",
    )
    body.visual(
        Box((0.0026, 0.0005, 0.0050)),
        origin=Origin(xyz=(0.0, body_thickness * 0.5 + 0.00025, keyway_z - 0.0010)),
        material=dark_metal,
        name="key_slot",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_thickness, body_height)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    shackle = model.part("shackle")
    shackle.visual(
        Cylinder(radius=shackle_radius, length=shackle_leg_length + captive_insert_depth),
        origin=Origin(xyz=(0.0, 0.0, (shackle_leg_length - captive_insert_depth) * 0.5)),
        material=shackle_chrome,
        name="captive_leg",
    )
    shackle.visual(
        Cylinder(radius=shackle_radius, length=shackle_leg_length - free_leg_bottom),
        origin=Origin(xyz=(shackle_spacing, 0.0, (shackle_leg_length + free_leg_bottom) * 0.5)),
        material=shackle_chrome,
        name="free_leg",
    )
    shackle.visual(
        mesh_from_geometry(build_shackle_crown(), "padlock_shackle_crown"),
        material=shackle_chrome,
        name="shackle_crown",
    )
    shackle.visual(
        Cylinder(radius=shackle_radius + 0.0006, length=0.0036),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=shackle_chrome,
        name="captive_leg_band",
    )
    shackle.visual(
        Cylinder(radius=shackle_radius + 0.0006, length=0.0036),
        origin=Origin(xyz=(shackle_spacing, 0.0, 0.040)),
        material=shackle_chrome,
        name="free_leg_band",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((shackle_spacing + 2.0 * shackle_radius, 2.0 * shackle_radius, 0.090)),
        mass=0.22,
        origin=Origin(xyz=(shackle_spacing * 0.5, 0.0, 0.045)),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(-shackle_spacing * 0.5, 0.0, shackle_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    key_cap = model.part("key_cap")
    key_cap.visual(
        Cylinder(radius=cap_disc_radius, length=cap_disc_thickness),
        origin=Origin(xyz=(-cap_hinge_x, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bumper_black,
        name="cover_disc",
    )
    key_cap.visual(
        Box((0.0105, cap_disc_thickness, 0.010)),
        origin=Origin(xyz=(-0.00525, 0.0, 0.0)),
        material=bumper_black,
        name="hinge_arm",
    )
    key_cap.visual(
        Cylinder(radius=cap_barrel_radius, length=0.010),
        material=bumper_black,
        name="hinge_barrel",
    )
    key_cap.visual(
        Box((0.0048, 0.0036, 0.0060)),
        origin=Origin(xyz=(-0.0185, 0.0002, -0.0045)),
        material=bumper_black,
        name="grip_tab",
    )
    key_cap.inertial = Inertial.from_geometry(
        Box((0.026, 0.006, 0.018)),
        mass=0.03,
        origin=Origin(xyz=(-0.011, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_key_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=key_cap,
        origin=Origin(xyz=(cap_hinge_x, body_thickness * 0.5 + cap_barrel_radius, keyway_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    key_cap = object_model.get_part("key_cap")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    key_cap_joint = object_model.get_articulation("body_to_key_cap")

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

    ctx.expect_contact(shackle, body, name="shackle_captive_contact_closed")
    ctx.expect_contact(key_cap, body, name="key_cap_seats_on_front_face")

    captive_closed_aabb = ctx.part_element_world_aabb(shackle, elem="captive_leg")
    free_closed_aabb = ctx.part_element_world_aabb(shackle, elem="free_leg")
    cap_disc_closed_aabb = ctx.part_element_world_aabb(key_cap, elem="cover_disc")
    cap_barrel_closed_aabb = ctx.part_element_world_aabb(key_cap, elem="hinge_barrel")

    assert captive_closed_aabb is not None
    assert free_closed_aabb is not None
    assert cap_disc_closed_aabb is not None
    assert cap_barrel_closed_aabb is not None

    captive_closed = aabb_center(captive_closed_aabb)
    free_closed = aabb_center(free_closed_aabb)
    cap_disc_closed = aabb_center(cap_disc_closed_aabb)
    cap_barrel_closed = aabb_center(cap_barrel_closed_aabb)

    with ctx.pose({shackle_joint: 1.20}):
        ctx.expect_contact(shackle, body, name="shackle_remains_captured_open")
        captive_open_aabb = ctx.part_element_world_aabb(shackle, elem="captive_leg")
        free_open_aabb = ctx.part_element_world_aabb(shackle, elem="free_leg")
        assert captive_open_aabb is not None
        assert free_open_aabb is not None
        captive_open = aabb_center(captive_open_aabb)
        free_open = aabb_center(free_open_aabb)

        ctx.check(
            "shackle_pivots_about_captive_leg",
            abs(captive_open[0] - captive_closed[0]) < 0.001
            and abs(captive_open[1] - captive_closed[1]) < 0.0015,
            details=(
                f"captive leg drifted from {captive_closed[:2]} to {captive_open[:2]} instead of staying on the pivot axis"
            ),
        )
        ctx.check(
            "shackle_swings_free_leg_outboard",
            free_open[1] > free_closed[1] + 0.020 and free_open[0] < free_closed[0] - 0.006,
            details=f"free leg did not swing aside enough: closed={free_closed}, open={free_open}",
        )

    with ctx.pose({key_cap_joint: 1.45}):
        ctx.expect_contact(key_cap, body, name="key_cap_hinge_stays_mounted_open")
        cap_disc_open_aabb = ctx.part_element_world_aabb(key_cap, elem="cover_disc")
        cap_barrel_open_aabb = ctx.part_element_world_aabb(key_cap, elem="hinge_barrel")
        assert cap_disc_open_aabb is not None
        assert cap_barrel_open_aabb is not None
        cap_disc_open = aabb_center(cap_disc_open_aabb)
        cap_barrel_open = aabb_center(cap_barrel_open_aabb)

        ctx.check(
            "key_cap_rotates_about_its_face_hinge",
            abs(cap_barrel_open[0] - cap_barrel_closed[0]) < 0.001
            and abs(cap_barrel_open[1] - cap_barrel_closed[1]) < 0.001,
            details=f"hinge barrel shifted from {cap_barrel_closed} to {cap_barrel_open}",
        )
        ctx.check(
            "key_cap_swings_clear_of_keyway",
            cap_disc_open[0] > cap_disc_closed[0] + 0.007
            and cap_disc_open[1] > cap_disc_closed[1] + 0.007,
            details=f"cap disc did not rotate aside enough: closed={cap_disc_closed}, open={cap_disc_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
