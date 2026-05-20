from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    cut_opening_on_face,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _laminated_slice_mesh(
    name: str,
    *,
    width: float,
    thickness: float,
    height: float,
    shackle_channel_width: float,
    shackle_channel_depth: float,
):
    geom = BoxGeometry((width, thickness, height))
    opening = rounded_rect_profile(
        shackle_channel_width,
        thickness * 0.94,
        radius=min(thickness * 0.45, shackle_channel_width * 0.10),
        corner_segments=8,
    )
    geom = cut_opening_on_face(
        geom,
        face="+z",
        opening_profile=opening,
        depth=shackle_channel_depth,
        offset=(0.0, 0.0),
    )
    return _save_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_shackle_padlock")

    laminated_steel = model.material("laminated_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.80, 0.82, 0.85, 1.0))
    strap_steel = model.material("strap_steel", rgba=(0.50, 0.52, 0.56, 1.0))
    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.59, 0.22, 1.0))
    dark_insert = model.material("dark_insert", rgba=(0.10, 0.10, 0.11, 1.0))

    body_width = 0.055
    body_thickness = 0.023
    body_height = 0.050

    shackle_radius = 0.0042
    shackle_span = 0.028
    shackle_rise = 0.089
    shackle_socket_depth = 0.012
    shackle_socket_floor_z = body_height - shackle_socket_depth
    shackle_pivot_x = -shackle_span * 0.5
    shackle_channel_width = shackle_span + shackle_radius * 3.6

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_thickness, body_height)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    slice_count = 7
    slice_overlap = 0.00014
    slice_thickness = (body_thickness + slice_overlap * (slice_count - 1)) / slice_count
    slice_step = slice_thickness - slice_overlap
    slice_start_y = -body_thickness * 0.5 + slice_thickness * 0.5
    for index in range(slice_count):
        y_pos = slice_start_y + index * slice_step
        body.visual(
            _laminated_slice_mesh(
                f"body_lamination_{index}",
                width=body_width,
                thickness=slice_thickness,
                height=body_height,
                shackle_channel_width=shackle_channel_width,
                shackle_channel_depth=shackle_socket_depth,
            ),
            origin=Origin(xyz=(0.0, y_pos, body_height * 0.5)),
            material=bright_steel if index in (0, slice_count - 1) else laminated_steel,
            name=f"lamination_{index}",
        )

    strap_width = 0.0065
    strap_height = body_height * 0.90
    strap_z = strap_height * 0.5
    strap_offset_x = body_width * 0.5 - strap_width * 0.5 - 0.001
    body.visual(
        Box((strap_width, body_thickness + 0.0008, strap_height)),
        origin=Origin(xyz=(-strap_offset_x, 0.0, strap_z)),
        material=strap_steel,
        name="left_side_strap",
    )
    body.visual(
        Box((strap_width, body_thickness + 0.0008, strap_height)),
        origin=Origin(xyz=(strap_offset_x, 0.0, strap_z)),
        material=strap_steel,
        name="right_side_strap",
    )

    key_center_z = 0.0155
    escutcheon_radius = 0.0118
    escutcheon_length = 0.0018
    key_cylinder_radius = 0.0098
    key_cylinder_length = 0.0038
    key_cylinder_front_y = body_thickness * 0.5 + escutcheon_length + key_cylinder_length
    body.visual(
        Cylinder(radius=escutcheon_radius, length=escutcheon_length),
        origin=Origin(
            xyz=(0.0, body_thickness * 0.5 + escutcheon_length * 0.5, key_center_z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="key_escutcheon",
    )
    body.visual(
        Cylinder(radius=key_cylinder_radius, length=key_cylinder_length),
        origin=Origin(
            xyz=(0.0, body_thickness * 0.5 + escutcheon_length + key_cylinder_length * 0.5, key_center_z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="key_cylinder_face",
    )
    body.visual(
        Box((0.0032, 0.0010, 0.0068)),
        origin=Origin(xyz=(0.0, key_cylinder_front_y + 0.0005, key_center_z + 0.0007)),
        material=dark_insert,
        name="key_slot",
    )
    body.visual(
        Cylinder(radius=0.0015, length=0.0010),
        origin=Origin(
            xyz=(0.0, key_cylinder_front_y + 0.0005, key_center_z - 0.0030),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=dark_insert,
        name="key_pip",
    )

    cover_width = 0.020
    cover_height = 0.016
    cover_thickness = 0.0015
    cover_hinge_radius = 0.0016
    cover_hinge_z = 0.0085
    body.visual(
        Cylinder(radius=0.0010, length=cover_width * 0.82),
        origin=Origin(
            xyz=(0.0, key_cylinder_front_y, cover_hinge_z),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=strap_steel,
        name="dust_hinge_pin",
    )

    shackle = model.part("shackle")
    shackle.inertial = Inertial.from_geometry(
        Box((shackle_span + shackle_radius * 2.0, shackle_radius * 2.4, shackle_rise + shackle_radius * 2.0)),
        mass=0.14,
        origin=Origin(xyz=(shackle_span * 0.5, 0.0, shackle_rise * 0.5)),
    )
    shackle.visual(
        _save_mesh(
            "shackle_wire",
            wire_from_points(
                [
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, shackle_rise),
                    (shackle_span, 0.0, shackle_rise),
                    (shackle_span, 0.0, 0.0),
                ],
                radius=shackle_radius,
                radial_segments=20,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.012,
                corner_segments=10,
            ),
        ),
        material=chrome,
        name="shackle_wire",
    )
    pivot_collar_length = 0.0028
    shackle.visual(
        Cylinder(radius=0.0065, length=pivot_collar_length),
        origin=Origin(xyz=(0.0, 0.0, shackle_socket_depth + pivot_collar_length * 0.5)),
        material=chrome,
        name="pivot_collar",
    )
    shackle.visual(
        Cylinder(radius=shackle_radius * 0.96, length=0.0012),
        origin=Origin(xyz=(shackle_span, 0.0, 0.0006)),
        material=chrome,
        name="free_leg_tip",
    )

    dust_cover = model.part("dust_cover")
    dust_cover.inertial = Inertial.from_geometry(
        Box((cover_width, 0.004, cover_height + 0.003)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0012, cover_height * 0.5)),
    )
    dust_cover.visual(
        Cylinder(radius=cover_hinge_radius, length=cover_width * 0.90),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=brass,
        name="hinge_barrel",
    )
    dust_cover.visual(
        _save_mesh(
            "dust_cover_flap",
            ExtrudeGeometry(
                rounded_rect_profile(cover_width, cover_height, 0.0022, corner_segments=8),
                cover_thickness,
                center=True,
            )
            .rotate_x(pi * 0.5)
            .translate(0.0, cover_thickness * 0.5, cover_height * 0.5),
        ),
        material=brass,
        name="cover_flap",
    )
    dust_cover.visual(
        Box((cover_width * 0.46, cover_thickness * 1.35, 0.0018)),
        origin=Origin(xyz=(0.0, cover_thickness * 0.8, cover_height - 0.0004)),
        material=brass,
        name="cover_lip",
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(shackle_pivot_x, 0.0, shackle_socket_floor_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_dust_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dust_cover,
        origin=Origin(xyz=(0.0, key_cylinder_front_y, cover_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=-1.35, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    dust_cover = object_model.get_part("dust_cover")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    cover_joint = object_model.get_articulation("body_to_dust_cover")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        body,
        dust_cover,
        elem_a="dust_hinge_pin",
        elem_b="hinge_barrel",
        reason="The dust-cover knuckle rotates around a fixed hinge pin represented as a solid stand-in.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(shackle, body, name="shackle is captive in the body")
    ctx.expect_contact(dust_cover, body, name="dust cover is mounted to the body")
    ctx.expect_overlap(
        dust_cover,
        body,
        axes="xz",
        elem_a="cover_flap",
        elem_b="key_cylinder_face",
        min_overlap=0.012,
        name="dust cover sits over the keyway",
    )
    ctx.expect_gap(
        dust_cover,
        body,
        axis="y",
        positive_elem="cover_flap",
        negative_elem="key_cylinder_face",
        max_gap=0.0006,
        max_penetration=0.0,
        name="dust cover closes flush over key cylinder face",
    )

    closed_free_tip = ctx.part_element_world_aabb(shackle, elem="free_leg_tip")
    closed_cover = ctx.part_element_world_aabb(dust_cover, elem="cover_flap")
    assert closed_free_tip is not None
    assert closed_cover is not None

    with ctx.pose({shackle_joint: 1.20}):
        open_free_tip = ctx.part_element_world_aabb(shackle, elem="free_leg_tip")
        assert open_free_tip is not None
        ctx.expect_contact(shackle, body, name="open shackle stays attached at captive leg")
        ctx.check(
            "shackle free leg swings away from the receiver",
            open_free_tip[0][1] > closed_free_tip[1][1] + 0.014,
            details=f"closed_tip_max_y={closed_free_tip[1][1]:.5f}, open_tip_min_y={open_free_tip[0][1]:.5f}",
        )

    with ctx.pose({cover_joint: -1.10}):
        open_cover = ctx.part_element_world_aabb(dust_cover, elem="cover_flap")
        assert open_cover is not None
        ctx.expect_contact(dust_cover, body, name="open dust cover stays hinged")
        ctx.expect_gap(
            dust_cover,
            body,
            axis="y",
            positive_elem="cover_lip",
            negative_elem="key_cylinder_face",
            min_gap=0.004,
            name="open dust cover lip clears the keyway",
        )
        ctx.check(
            "dust cover flips down from the keyway",
            open_cover[1][2] < closed_cover[1][2] - 0.004,
            details=f"closed_cover_top_z={closed_cover[1][2]:.5f}, open_cover_top_z={open_cover[1][2]:.5f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
