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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _translate_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_chest_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.92, 0.94, 0.95, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.98, 0.99, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    seal_black = model.material("seal_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.72, 0.86, 0.94, 0.38))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    outer_w = 1.52
    outer_d = 0.78
    inner_w = 1.34
    inner_d = 0.60
    body_z0 = 0.05
    wall_h = 0.73
    collar_h = 0.045
    floor_t = 0.028
    cabinet_top_z = body_z0 + wall_h + collar_h

    lid_w = 1.50
    lid_d = 0.765
    lid_t = 0.055
    glass_t = 0.008

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, cabinet_top_z)),
        mass=128.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_top_z * 0.5)),
    )

    cabinet_shell = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_w, outer_d, 0.055),
        [rounded_rect_profile(inner_w, inner_d, 0.035)],
        height=wall_h,
        center=True,
    )
    cabinet.visual(
        _mesh("cabinet_shell", cabinet_shell),
        origin=Origin(xyz=(0.0, 0.0, body_z0 + wall_h * 0.5)),
        material=cabinet_white,
        name="cabinet_shell",
    )

    top_collar = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_w + 0.02, outer_d + 0.02, 0.060),
        [rounded_rect_profile(inner_w + 0.02, inner_d + 0.02, 0.040)],
        height=collar_h,
        center=True,
    )
    cabinet.visual(
        _mesh("top_collar", top_collar),
        origin=Origin(xyz=(0.0, 0.0, body_z0 + wall_h + collar_h * 0.5)),
        material=trim_gray,
        name="top_collar",
    )

    cabinet.visual(
        Box((inner_w, inner_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, body_z0 + floor_t * 0.5)),
        material=liner_white,
        name="interior_floor",
    )
    cabinet.visual(
        Box((outer_w - 0.08, 0.032, 0.11)),
        origin=Origin(xyz=(0.0, outer_d * 0.5 - 0.016, body_z0 + 0.13)),
        material=dark_trim,
        name="front_kick_plate",
    )
    cabinet.visual(
        Box((outer_w - 0.10, 0.014, 0.036)),
        origin=Origin(xyz=(0.0, -outer_d * 0.5 - 0.005, cabinet_top_z - 0.022)),
        material=steel,
        name="rear_hinge_leaf",
    )
    cabinet.visual(
        Box((0.096, 0.014, 0.042)),
        origin=Origin(xyz=(0.0, outer_d * 0.5 + 0.009, cabinet_top_z + 0.001)),
        material=steel,
        name="padlock_backer",
    )

    loop_geom = wire_from_points(
        [
            (-0.024, 0.0, -0.010),
            (-0.024, 0.0, 0.038),
            (0.024, 0.0, 0.038),
            (0.024, 0.0, -0.010),
        ],
        radius=0.004,
        radial_segments=14,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.010,
        corner_segments=8,
    )
    cabinet.visual(
        _mesh("padlock_loop", loop_geom),
        origin=Origin(xyz=(0.0, outer_d * 0.5 + 0.011, cabinet_top_z - 0.005)),
        material=steel,
        name="padlock_loop",
    )

    for idx, (fx, fy) in enumerate(
        [
            (0.62, 0.29),
            (-0.62, 0.29),
            (0.62, -0.29),
            (-0.62, -0.29),
        ],
        start=1,
    ):
        cabinet.visual(
            Box((0.085, 0.085, body_z0)),
            origin=Origin(xyz=(fx, fy, body_z0 * 0.5)),
            material=dark_trim,
            name=f"foot_{idx}",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_t)),
        mass=29.0,
        origin=Origin(xyz=(0.0, lid_d * 0.5, lid_t * 0.5)),
    )

    lid_frame = ExtrudeWithHolesGeometry(
        rounded_rect_profile(lid_w, lid_d, 0.045),
        [rounded_rect_profile(1.32, 0.57, 0.030)],
        height=lid_t,
        center=True,
    )
    lid.visual(
        _mesh("lid_frame", lid_frame),
        origin=Origin(xyz=(0.0, lid_d * 0.5, lid_t * 0.5)),
        material=trim_gray,
        name="lid_frame",
    )

    lid_gasket = ExtrudeWithHolesGeometry(
        rounded_rect_profile(1.45, 0.70, 0.035),
        [rounded_rect_profile(1.31, 0.58, 0.028)],
        height=0.006,
        center=True,
    )
    lid.visual(
        _mesh("lid_gasket", lid_gasket),
        origin=Origin(xyz=(0.0, lid_d * 0.5, 0.003)),
        material=seal_black,
        name="lid_gasket",
    )

    lid.visual(
        Box((1.36, 0.61, glass_t)),
        origin=Origin(xyz=(0.0, lid_d * 0.5, 0.030)),
        material=glass_tint,
        name="glass_panel",
    )
    lid.visual(
        Box((lid_w - 0.08, 0.025, 0.030)),
        origin=Origin(xyz=(0.0, lid_d - 0.0125, 0.032)),
        material=trim_gray,
        name="front_rail",
    )
    lid.visual(
        Box((lid_w - 0.10, 0.016, 0.032)),
        origin=Origin(xyz=(0.0, 0.008, 0.038)),
        material=steel,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.012, 0.035, 0.018)),
        origin=Origin(xyz=(-0.048, lid_d + 0.005, 0.041)),
        material=steel,
        name="left_hasp_knuckle",
    )
    lid.visual(
        Box((0.012, 0.035, 0.018)),
        origin=Origin(xyz=(0.048, lid_d + 0.005, 0.041)),
        material=steel,
        name="right_hasp_knuckle",
    )

    hasp_arm = model.part("hasp_arm")
    hasp_arm.inertial = Inertial.from_geometry(
        Box((0.10, 0.010, 0.070)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.003, -0.034)),
    )

    hasp_outer = _translate_profile(rounded_rect_profile(0.096, 0.068, 0.012), dy=-0.034)
    hasp_hole = _translate_profile(rounded_rect_profile(0.072, 0.032, 0.008), dy=-0.039)
    hasp_geom = ExtrudeWithHolesGeometry(
        hasp_outer,
        [hasp_hole],
        height=0.006,
        center=True,
    ).rotate_x(pi * 0.5)
    hasp_arm.visual(
        _mesh("hasp_strap", hasp_geom),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=steel,
        name="hasp_strap",
    )
    hasp_arm.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(-0.031, 0.005, -0.003)),
        material=steel,
        name="left_hasp_web",
    )
    hasp_arm.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(0.031, 0.005, -0.003)),
        material=steel,
        name="right_hasp_web",
    )
    hasp_arm.visual(
        Cylinder(radius=0.006, length=0.086),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=steel,
        name="hasp_hinge_barrel",
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, -lid_d * 0.5, cabinet_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.22),
    )
    model.articulation(
        "lid_to_hasp",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp_arm,
        origin=Origin(xyz=(0.0, lid_d + 0.015, 0.041)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    hasp_arm = object_model.get_part("hasp_arm")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    hasp_hinge = object_model.get_articulation("lid_to_hasp")

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
        hasp_arm,
        lid,
        elem_a="hasp_hinge_barrel",
        elem_b="left_hasp_knuckle",
        reason="The security-hasp hinge is simplified as an engaged barrel between rolled mounting knuckles without modeling the separate hinge pin bore clearance.",
    )
    ctx.allow_overlap(
        hasp_arm,
        lid,
        elem_a="hasp_hinge_barrel",
        elem_b="right_hasp_knuckle",
        reason="The security-hasp hinge is simplified as an engaged barrel between rolled mounting knuckles without modeling the separate hinge pin bore clearance.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="lid_gasket",
        negative_elem="top_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="lid gasket seats on the cabinet collar",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="xy",
        elem_a="lid_frame",
        elem_b="top_collar",
        min_overlap=0.55,
        name="lid footprint covers the freezer opening",
    )
    ctx.expect_overlap(
        hasp_arm,
        cabinet,
        axes="xz",
        elem_a="hasp_strap",
        elem_b="padlock_loop",
        min_overlap=0.025,
        name="hasp lines up with the padlock loop",
    )
    ctx.expect_gap(
        hasp_arm,
        cabinet,
        axis="y",
        positive_elem="hasp_strap",
        negative_elem="padlock_loop",
        min_gap=0.0,
        max_gap=0.020,
        name="closed hasp hangs just ahead of the padlock loop",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    with ctx.pose({lid_hinge: 1.10}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_front is not None
        and opened_front is not None
        and opened_front[1][2] > closed_front[1][2] + 0.18,
        details=f"closed_front={closed_front}, opened_front={opened_front}",
    )

    closed_hasp = ctx.part_element_world_aabb(hasp_arm, elem="hasp_strap")
    with ctx.pose({hasp_hinge: 1.20}):
        swung_hasp = ctx.part_element_world_aabb(hasp_arm, elem="hasp_strap")
    ctx.check(
        "hasp swings forward to clear the loop",
        closed_hasp is not None
        and swung_hasp is not None
        and swung_hasp[1][1] > closed_hasp[1][1] + 0.035,
        details=f"closed_hasp={closed_hasp}, swung_hasp={swung_hasp}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
