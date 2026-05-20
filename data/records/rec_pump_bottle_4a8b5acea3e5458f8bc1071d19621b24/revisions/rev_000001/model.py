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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lotion_pump_bottle")

    bottle_ivory = model.material("bottle_ivory", rgba=(0.94, 0.93, 0.88, 1.0))
    pump_charcoal = model.material("pump_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    stem_white = model.material("stem_white", rgba=(0.96, 0.96, 0.95, 1.0))

    body_width = 0.074
    body_depth = 0.054
    wall_thickness = 0.003
    lower_body_height = 0.125
    shoulder_height = 0.038
    collar_height = 0.019
    base_thickness = 0.004
    neck_outer_radius = 0.015
    neck_inner_radius = 0.0086
    guide_radius = 0.0074
    total_body_height = lower_body_height + shoulder_height + collar_height

    def mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def section_from_profile(profile, z: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for x, y in profile]

    def yz_section(x: float, width: float, height: float, radius: float, z_center: float) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + z_center)
            for z, y in rounded_rect_profile(height, width, radius, corner_segments=6)
        ]

    lower_outer = superellipse_profile(body_width, body_depth, exponent=5.2, segments=56)
    lower_inner = superellipse_profile(
        body_width - 2.0 * wall_thickness,
        body_depth - 2.0 * wall_thickness,
        exponent=5.2,
        segments=56,
    )
    shoulder_mid_outer = superellipse_profile(0.058, 0.042, exponent=4.1, segments=56)
    neck_outer = superellipse_profile(2.0 * neck_outer_radius, 2.0 * neck_outer_radius, exponent=2.0, segments=56)
    neck_inner = superellipse_profile(2.0 * neck_inner_radius, 2.0 * neck_inner_radius, exponent=2.0, segments=56)

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        mesh(
            "bottle_lower_shell_v2",
            ExtrudeWithHolesGeometry(
                lower_outer,
                [lower_inner],
                lower_body_height,
                cap=True,
                center=False,
            ),
        ),
        material=bottle_ivory,
        name="body_shell",
    )
    bottle_body.visual(
        mesh(
            "bottle_base_insert_v2",
            ExtrudeGeometry.from_z0(lower_inner, base_thickness),
        ),
        material=bottle_ivory,
        name="base_insert",
    )
    bottle_body.visual(
        mesh(
            "bottle_outer_shoulder_v2",
            section_loft(
                [
                    section_from_profile(lower_outer, 0.0),
                    section_from_profile(shoulder_mid_outer, shoulder_height * 0.52),
                    section_from_profile(neck_outer, shoulder_height),
                ],
                cap=False,
                solid=False,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, lower_body_height)),
        material=bottle_ivory,
        name="outer_shoulder",
    )
    bottle_body.visual(
        mesh(
            "bottle_neck_collar_v2",
            LatheGeometry.from_shell_profiles(
                [
                    (neck_outer_radius, 0.0),
                    (0.016, 0.004),
                    (0.016, collar_height - 0.003),
                    (neck_outer_radius, collar_height),
                ],
                [
                    (neck_inner_radius, 0.0),
                    (neck_inner_radius, collar_height),
                ],
                segments=56,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, lower_body_height + shoulder_height)),
        material=pump_charcoal,
        name="neck_collar",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, total_body_height)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, total_body_height * 0.5)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=guide_radius, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=stem_white,
        name="guide_piston",
    )
    plunger.visual(
        Cylinder(radius=0.0055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=stem_white,
        name="stem_post",
    )
    plunger.visual(
        Cylinder(radius=0.0095, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=pump_charcoal,
        name="head_bearing",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.050)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    nozzle_head = model.part("nozzle_head")
    nozzle_head.visual(
        Cylinder(radius=0.0105, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=pump_charcoal,
        name="hub_socket",
    )
    nozzle_head.visual(
        mesh(
            "pump_actuator_body_v3",
            section_loft(
                [
                    yz_section(-0.018, 0.020, 0.014, 0.004, 0.017),
                    yz_section(0.006, 0.024, 0.020, 0.005, 0.018),
                    yz_section(0.024, 0.016, 0.013, 0.0035, 0.018),
                ]
            ),
        ),
        material=pump_charcoal,
        name="actuator_body",
    )
    nozzle_head.visual(
        mesh(
            "pump_thumb_pad_v3",
            ExtrudeGeometry(
                rounded_rect_profile(0.030, 0.024, 0.0045, corner_segments=6),
                0.005,
                center=True,
            ),
        ),
        origin=Origin(xyz=(-0.008, 0.0, 0.0285)),
        material=pump_charcoal,
        name="thumb_pad",
    )
    nozzle_head.visual(
        Cylinder(radius=0.0042, length=0.020),
        origin=Origin(xyz=(0.028, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_charcoal,
        name="nozzle_spout",
    )
    nozzle_head.visual(
        Cylinder(radius=0.0027, length=0.010),
        origin=Origin(xyz=(0.043, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_charcoal,
        name="outlet_tip",
    )
    nozzle_head.visual(
        mesh(
            "pump_lock_collar",
            LatheGeometry.from_shell_profiles(
                [
                    (0.014, 0.0),
                    (0.0155, 0.002),
                    (0.0155, 0.007),
                    (0.014, 0.009),
                ],
                [
                    (0.011, 0.0),
                    (0.011, 0.009),
                ],
                segments=40,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=pump_charcoal,
        name="lock_collar",
    )
    nozzle_head.visual(
        Box((0.006, 0.0025, 0.004)),
        origin=Origin(xyz=(0.009, 0.009, 0.004), rpy=(0.0, 0.0, math.radians(45))),
        material=pump_charcoal,
        name="lock_lug_a",
    )
    nozzle_head.visual(
        Box((0.006, 0.0025, 0.004)),
        origin=Origin(xyz=(-0.009, -0.009, 0.004), rpy=(0.0, 0.0, math.radians(45))),
        material=pump_charcoal,
        name="lock_lug_b",
    )
    nozzle_head.inertial = Inertial.from_geometry(
        Box((0.060, 0.028, 0.034)),
        mass=0.055,
        origin=Origin(xyz=(0.010, 0.0, 0.017)),
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=bottle_body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, total_body_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.10,
            lower=-0.012,
            upper=0.0,
        ),
    )
    model.articulation(
        "plunger_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=nozzle_head,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(90.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    bottle_body = object_model.get_part("bottle_body")
    plunger = object_model.get_part("plunger")
    nozzle_head = object_model.get_part("nozzle_head")
    body_to_plunger = object_model.get_articulation("body_to_plunger")
    plunger_to_nozzle = object_model.get_articulation("plunger_to_nozzle")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0015)
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

    body_aabb = ctx.part_world_aabb(bottle_body)
    if body_aabb is not None:
        body_dx = body_aabb[1][0] - body_aabb[0][0]
        body_dy = body_aabb[1][1] - body_aabb[0][1]
        body_dz = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "body_square_shouldered_proportions",
            0.070 <= body_dx <= 0.078 and 0.050 <= body_dy <= 0.058 and 0.178 <= body_dz <= 0.186 and body_dx > body_dy,
            f"Unexpected bottle dimensions dx={body_dx:.4f}, dy={body_dy:.4f}, dz={body_dz:.4f}",
        )
    else:
        ctx.fail("body_square_shouldered_proportions", "Bottle body AABB unavailable")

    ctx.expect_origin_distance(plunger, bottle_body, axes="xy", max_dist=1e-6)
    ctx.expect_within(
        plunger,
        bottle_body,
        axes="xy",
        inner_elem="guide_piston",
        outer_elem="neck_collar",
        margin=0.0005,
    )
    ctx.expect_contact(nozzle_head, plunger, elem_a="hub_socket", elem_b="head_bearing")
    ctx.expect_gap(nozzle_head, bottle_body, axis="z", min_gap=0.020)

    plunger_rest = ctx.part_world_position(plunger)
    nozzle_rest_aabb = ctx.part_world_aabb(nozzle_head)

    plunger_limits = body_to_plunger.motion_limits
    if plunger_limits is not None and plunger_limits.lower is not None and plunger_limits.upper is not None:
        with ctx.pose({body_to_plunger: plunger_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="plunger_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="plunger_lower_no_floating", contact_tol=0.0015)
            ctx.expect_within(
                plunger,
                bottle_body,
                axes="xy",
                inner_elem="guide_piston",
                outer_elem="neck_collar",
                margin=0.0005,
                name="plunger_lower_guided_within_collar",
            )
            ctx.expect_gap(nozzle_head, bottle_body, axis="z", min_gap=0.008, name="pressed_head_stays_above_bottle")

            plunger_pressed = ctx.part_world_position(plunger)
            if plunger_rest is not None and plunger_pressed is not None:
                ctx.check(
                    "plunger_translates_axially",
                    abs(plunger_pressed[0] - plunger_rest[0]) <= 1e-6
                    and abs(plunger_pressed[1] - plunger_rest[1]) <= 1e-6
                    and plunger_pressed[2] < plunger_rest[2] - 0.009,
                    f"Rest={plunger_rest}, pressed={plunger_pressed}",
                )
            else:
                ctx.fail("plunger_translates_axially", "Unable to measure plunger positions")

        with ctx.pose({body_to_plunger: plunger_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="plunger_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="plunger_upper_no_floating", contact_tol=0.0015)

    nozzle_limits = plunger_to_nozzle.motion_limits
    if nozzle_limits is not None and nozzle_limits.lower is not None and nozzle_limits.upper is not None:
        with ctx.pose({plunger_to_nozzle: nozzle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="nozzle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="nozzle_upper_no_floating", contact_tol=0.0015)
            ctx.expect_contact(nozzle_head, plunger, elem_a="hub_socket", elem_b="head_bearing")

            nozzle_twisted_aabb = ctx.part_world_aabb(nozzle_head)
            if nozzle_rest_aabb is not None and nozzle_twisted_aabb is not None:
                rest_x = nozzle_rest_aabb[1][0] - nozzle_rest_aabb[0][0]
                rest_y = nozzle_rest_aabb[1][1] - nozzle_rest_aabb[0][1]
                turned_x = nozzle_twisted_aabb[1][0] - nozzle_twisted_aabb[0][0]
                turned_y = nozzle_twisted_aabb[1][1] - nozzle_twisted_aabb[0][1]
                ctx.check(
                    "nozzle_twist_reorients_spout",
                    rest_x > rest_y + 0.010 and turned_y > turned_x + 0.010,
                    (
                        f"rest spans=({rest_x:.4f}, {rest_y:.4f}), "
                        f"twisted spans=({turned_x:.4f}, {turned_y:.4f})"
                    ),
                )
            else:
                ctx.fail("nozzle_twist_reorients_spout", "Unable to measure nozzle AABBs")

        with ctx.pose({body_to_plunger: plunger_limits.lower, plunger_to_nozzle: nozzle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="combined_pose_no_floating", contact_tol=0.0015)
            ctx.expect_contact(nozzle_head, plunger, elem_a="hub_socket", elem_b="head_bearing")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
