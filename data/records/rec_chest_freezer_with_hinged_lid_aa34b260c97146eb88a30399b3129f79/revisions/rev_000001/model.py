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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def _element_center(ctx: TestContext, part, elem: str):
    return _aabb_center(ctx.part_element_world_aabb(part, elem=elem))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_boat_chest_cooler")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.92, 1.0))
    liner_white = model.material("liner_white", rgba=(0.88, 0.90, 0.89, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.26, 0.28, 0.29, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.63, 0.65, 0.68, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.78, 1.0))
    valve_black = model.material("valve_black", rgba=(0.13, 0.14, 0.15, 1.0))
    lever_red = model.material("lever_red", rgba=(0.71, 0.11, 0.10, 1.0))

    outer_len = 0.96
    outer_w = 0.44
    body_h = 0.405
    wall_t = 0.028
    floor_t = 0.026
    runner_h = 0.018
    outer_r = 0.034
    inner_len = outer_len - 2.0 * wall_t
    inner_w = outer_w - 2.0 * wall_t
    inner_r = 0.018

    lid_outer_len = 0.99
    lid_outer_w = 0.46
    lid_outer_r = 0.038
    lid_inner_len = 0.90
    lid_inner_w = 0.38
    lid_inner_r = 0.020
    lid_rear_overhang = 0.022
    lid_center_x = (lid_outer_len * 0.5) - lid_rear_overhang
    lid_top_center_x = lid_center_x + 0.035
    lid_top_t = 0.032
    lid_skirt_top = 0.009
    lid_skirt_bottom = -0.047
    lid_skirt_center_x = lid_center_x + 0.024

    hinge_axis_x = -outer_len * 0.5 + 0.018
    hinge_axis_z = body_h + 0.048
    hinge_radius = 0.011
    hinge_centers_y = (-0.150, 0.150)
    body_knuckle_len = 0.022
    lid_knuckle_len = 0.032
    knuckle_split = 0.029

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((outer_len, outer_w, body_h)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    outer_profile = rounded_rect_profile(outer_len, outer_w, outer_r, corner_segments=8)
    inner_profile = rounded_rect_profile(inner_len, inner_w, inner_r, corner_segments=8)
    body_floor = ExtrudeGeometry(outer_profile, runner_h + floor_t, center=False)
    body_walls = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        body_h - (runner_h + floor_t),
        center=False,
    ).translate(0.0, 0.0, runner_h + floor_t)

    body.visual(
        _mesh("cooler_body_floor", body_floor),
        material=shell_white,
        name="body_floor",
    )
    body.visual(
        _mesh("cooler_body_walls", body_walls),
        material=liner_white,
        name="body_walls",
    )
    body.visual(
        Box((outer_len * 0.76, 0.045, runner_h)),
        origin=Origin(xyz=(0.0, outer_w * 0.5 - 0.052, runner_h * 0.5)),
        material=gasket_gray,
        name="starboard_runner",
    )
    body.visual(
        Box((outer_len * 0.76, 0.045, runner_h)),
        origin=Origin(xyz=(0.0, -outer_w * 0.5 + 0.052, runner_h * 0.5)),
        material=gasket_gray,
        name="port_runner",
    )
    body.visual(
        Box((0.026, 0.032, 0.054)),
        origin=Origin(xyz=(-0.030, -0.228, 0.364)),
        material=gasket_gray,
        name="stay_pad_support",
    )
    body.visual(
        Box((0.014, 0.018, 0.048)),
        origin=Origin(xyz=(-0.030, -0.244, 0.373)),
        material=gasket_gray,
        name="stay_pad",
    )
    body.visual(
        Box((0.040, 0.070, 0.070)),
        origin=Origin(xyz=(outer_len * 0.5 + 0.010, 0.0, runner_h + 0.058)),
        material=valve_black,
        name="drain_boss",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(
            xyz=(outer_len * 0.5 + 0.009, 0.0, runner_h + 0.058),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=valve_black,
        name="drain_flange",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(
            xyz=(outer_len * 0.5 + 0.034, 0.0, runner_h + 0.058),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=valve_black,
        name="drain_spigot",
    )

    for hinge_index, y_center in enumerate(hinge_centers_y):
        for segment_sign in (-1.0, 1.0):
            segment_center = y_center + segment_sign * knuckle_split
            suffix = "aft" if segment_sign < 0.0 else "forward"
            body.visual(
                Cylinder(radius=hinge_radius, length=body_knuckle_len),
                origin=Origin(
                    xyz=(hinge_axis_x, segment_center, hinge_axis_z),
                    rpy=(math.pi * 0.5, 0.0, 0.0),
                ),
                material=hinge_gray,
                name=f"body_hinge_{hinge_index}_{suffix}_barrel",
            )
            body.visual(
                Box((0.024, body_knuckle_len + 0.004, 0.044)),
                origin=Origin(
                    xyz=(hinge_axis_x - 0.018, segment_center, body_h + 0.019),
                ),
                material=hinge_gray,
                name=f"body_hinge_{hinge_index}_{suffix}_strap",
            )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_outer_len, lid_outer_w, 0.085)),
        mass=4.0,
        origin=Origin(xyz=(lid_center_x, 0.0, -0.006)),
    )

    lid_outer_profile = rounded_rect_profile(lid_outer_len, lid_outer_w, lid_outer_r, corner_segments=8)
    lid_top = ExtrudeGeometry(lid_outer_profile, lid_top_t, center=False).translate(lid_top_center_x, 0.0, 0.0)

    lid.visual(
        _mesh("cooler_lid_top", lid_top),
        material=shell_white,
        name="lid_top_shell",
    )
    lid.visual(
        Box((0.080, 0.404, lid_skirt_top - lid_skirt_bottom)),
        origin=Origin(xyz=(0.910, 0.0, (lid_skirt_top + lid_skirt_bottom) * 0.5)),
        material=liner_white,
        name="lid_skirt",
    )
    lid.visual(
        Box((0.780, 0.028, lid_skirt_top - lid_skirt_bottom)),
        origin=Origin(xyz=(0.490, 0.216, (lid_skirt_top + lid_skirt_bottom) * 0.5)),
        material=liner_white,
        name="lid_starboard_skirt",
    )
    lid.visual(
        Box((0.780, 0.028, lid_skirt_top - lid_skirt_bottom)),
        origin=Origin(xyz=(0.490, -0.216, (lid_skirt_top + lid_skirt_bottom) * 0.5)),
        material=liner_white,
        name="lid_port_skirt",
    )
    lid.visual(
        Box((0.040, 0.320, 0.032)),
        origin=Origin(xyz=(0.952, 0.0, -0.008)),
        material=liner_white,
        name="lid_front_lip",
    )
    lid.visual(
        Box((0.040, 0.040, 0.016)),
        origin=Origin(xyz=(0.088, -0.248, -0.024)),
        material=hinge_gray,
        name="stay_bracket_base",
    )
    lid.visual(
        Box((0.020, 0.008, 0.024)),
        origin=Origin(xyz=(0.102, -0.258, -0.010)),
        material=hinge_gray,
        name="stay_bracket_outer_ear",
    )
    lid.visual(
        Box((0.020, 0.008, 0.024)),
        origin=Origin(xyz=(0.102, -0.238, -0.010)),
        material=hinge_gray,
        name="stay_bracket_inner_ear",
    )

    for hinge_index, y_center in enumerate(hinge_centers_y):
        lid.visual(
            Cylinder(radius=hinge_radius, length=lid_knuckle_len),
            origin=Origin(
                xyz=(0.0, y_center, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=hinge_gray,
            name=f"lid_hinge_{hinge_index}_barrel",
        )
        lid.visual(
            Box((0.030, lid_knuckle_len + 0.002, 0.020)),
            origin=Origin(xyz=(0.014, y_center, -0.003)),
            material=hinge_gray,
            name=f"lid_hinge_{hinge_index}_strap",
        )

    stay_rod = model.part("stay_rod")
    stay_len = 0.39
    stay_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.005, length=stay_len),
        mass=0.20,
        origin=Origin(xyz=(0.195, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
    )
    stay_rod.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=hinge_gray,
        name="stay_hub",
    )
    stay_rod.visual(
        Cylinder(radius=0.0045, length=0.374),
        origin=Origin(xyz=(0.187, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="stay_bar",
    )
    stay_rod.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(xyz=(0.380, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=gasket_gray,
        name="stay_tip",
    )

    drain_lever = model.part("drain_lever")
    drain_lever.inertial = Inertial.from_geometry(
        Box((0.036, 0.080, 0.090)),
        mass=0.06,
        origin=Origin(xyz=(0.020, 0.0, 0.028)),
    )
    drain_lever.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_gray,
        name="lever_hub",
    )
    drain_lever.visual(
        Box((0.012, 0.020, 0.070)),
        origin=Origin(xyz=(0.018, 0.0, 0.035)),
        material=lever_red,
        name="lever_handle",
    )
    drain_lever.visual(
        Box((0.018, 0.036, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.006)),
        material=lever_red,
        name="lever_paddle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )
    model.articulation(
        "lid_to_stay_rod",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=stay_rod,
        origin=Origin(xyz=(0.110, -0.248, -0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "body_to_drain_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drain_lever,
        origin=Origin(xyz=(outer_len * 0.5 + 0.051, 0.0, runner_h + 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=1.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    stay_rod = object_model.get_part("stay_rod")
    drain_lever = object_model.get_part("drain_lever")
    lid_hinge = object_model.get_articulation("body_to_lid")
    stay_joint = object_model.get_articulation("lid_to_stay_rod")
    drain_joint = object_model.get_articulation("body_to_drain_lever")

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
        "mechanism axes match cooler hardware",
        lid_hinge.axis == (0.0, -1.0, 0.0)
        and stay_joint.axis == (0.0, 1.0, 0.0)
        and drain_joint.axis == (1.0, 0.0, 0.0),
        details=f"lid={lid_hinge.axis}, stay={stay_joint.axis}, drain={drain_joint.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, stay_joint: 0.0, drain_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_skirt",
            negative_elem="body_walls",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid sits tightly above cooler rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_top_shell",
            elem_b="body_walls",
            min_overlap=0.36,
            name="closed lid covers the cooler opening",
        )
        ctx.expect_gap(
            stay_rod,
            body,
            axis="z",
            positive_elem="stay_tip",
            negative_elem="stay_pad",
            min_gap=0.040,
            name="stowed stay rod clears the body support pad",
        )

    lid_open = math.radians(63.0)
    stay_deployed = 1.48
    drain_open = 1.25

    with ctx.pose({lid_hinge: 0.0}):
        closed_front_center = _element_center(ctx, lid, "lid_front_lip")
    with ctx.pose({lid_hinge: lid_open}):
        open_front_center = _element_center(ctx, lid, "lid_front_lip")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_front_lip",
            negative_elem="body_walls",
            min_gap=0.18,
            name="opened lid front edge lifts well clear of the cooler body",
        )
    ctx.check(
        "lid front swings upward",
        closed_front_center is not None
        and open_front_center is not None
        and open_front_center[2] > closed_front_center[2] + 0.20,
        details=f"closed={closed_front_center}, open={open_front_center}",
    )

    with ctx.pose({lid_hinge: lid_open, stay_joint: stay_deployed}):
        ctx.expect_contact(
            stay_rod,
            body,
            elem_a="stay_tip",
            elem_b="stay_pad",
            contact_tol=0.012,
            name="deployed stay rod props against the body support pad",
        )

    with ctx.pose({drain_joint: 0.0}):
        drain_closed_center = _element_center(ctx, drain_lever, "lever_handle")
    with ctx.pose({drain_joint: drain_open}):
        drain_open_center = _element_center(ctx, drain_lever, "lever_handle")
    ctx.check(
        "drain lever rotates from vertical toward the side",
        drain_closed_center is not None
        and drain_open_center is not None
        and drain_closed_center[2] > drain_open_center[2] + 0.018
        and abs(drain_open_center[1] - drain_closed_center[1]) > 0.028,
        details=f"closed={drain_closed_center}, open={drain_open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
