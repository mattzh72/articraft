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
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_plate_mesh(
    name: str,
    *,
    depth: float,
    width: float,
    radius: float,
    height: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    center_z: float = 0.0,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(depth, width, radius, corner_segments=10),
        height,
        center=True,
    )
    geom.translate(center_x, center_y, center_z)
    return _mesh(name, geom)


def _rounded_ring_mesh(
    name: str,
    *,
    outer_depth: float,
    outer_width: float,
    outer_radius: float,
    inner_depth: float,
    inner_width: float,
    inner_radius: float,
    height: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    center_z: float = 0.0,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_depth, outer_width, outer_radius, corner_segments=10),
        [rounded_rect_profile(inner_depth, inner_width, inner_radius, corner_segments=10)],
        height,
        center=True,
    )
    geom.translate(center_x, center_y, center_z)
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sewing_box")

    painted_metal = model.material("painted_metal", rgba=(0.33, 0.43, 0.47, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.60, 0.62, 0.65, 1.0))
    polymer = model.material("polymer", rgba=(0.68, 0.70, 0.72, 1.0))
    elastomer = model.material("elastomer", rgba=(0.11, 0.12, 0.13, 1.0))

    body_depth = 0.240
    body_width = 0.350
    body_corner_radius = 0.026
    wall_thickness = 0.003
    base_thickness = 0.004
    wall_height = 0.120
    body_height = base_thickness + wall_height

    lid_outer_depth = 0.254
    lid_outer_width = 0.358
    lid_outer_radius = 0.028
    lid_skirt_inner_depth = 0.249
    lid_skirt_inner_width = 0.353
    lid_skirt_inner_radius = 0.025
    lid_skirt_height = 0.023
    lid_top_thickness = 0.005
    lid_rear_offset = 0.004
    lid_center_x = lid_rear_offset + (lid_outer_depth * 0.5)
    hinge_axis_x = -(body_depth * 0.5) - 0.0115
    hinge_axis_z = 0.123

    knuckle_radius = 0.0068
    body_knuckle_length = 0.068
    lid_knuckle_length = 0.198

    body = model.part("body")
    body.visual(
        _rounded_plate_mesh(
            "body_bottom_panel",
            depth=body_depth,
            width=body_width,
            radius=body_corner_radius,
            height=base_thickness,
            center_z=base_thickness * 0.5,
        ),
        material=painted_metal,
        name="body_bottom_panel",
    )
    body.visual(
        _rounded_ring_mesh(
            "body_wall_shell",
            outer_depth=body_depth,
            outer_width=body_width,
            outer_radius=body_corner_radius,
            inner_depth=body_depth - (2.0 * wall_thickness),
            inner_width=body_width - (2.0 * wall_thickness),
            inner_radius=body_corner_radius - wall_thickness,
            height=wall_height,
            center_z=base_thickness + (wall_height * 0.5),
        ),
        material=painted_metal,
        name="body_wall_shell",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        (
            (0.078, 0.122),
            (0.078, -0.122),
            (-0.078, 0.122),
            (-0.078, -0.122),
        )
    ):
        body.visual(
            Cylinder(radius=0.013, length=0.008),
            origin=Origin(xyz=(foot_x, foot_y, -0.001)),
            material=elastomer,
            name=f"foot_{foot_index}",
        )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((0.018, 0.072, 0.015)),
            origin=Origin(xyz=(hinge_axis_x + 0.0045, side_sign * 0.141, hinge_axis_z)),
            material=painted_metal,
            name=f"{side_name}_hinge_ear",
        )
        body.visual(
            Cylinder(radius=knuckle_radius, length=body_knuckle_length),
            origin=Origin(
                xyz=(hinge_axis_x, side_sign * 0.141, hinge_axis_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"{side_name}_hinge_barrel",
        )

    body.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, body_height)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        _rounded_plate_mesh(
            "lid_top_panel",
            depth=lid_outer_depth,
            width=lid_outer_width,
            radius=lid_outer_radius,
            height=lid_top_thickness,
            center_x=lid_center_x,
            center_z=0.0055,
        ),
        material=painted_metal,
        name="lid_top_panel",
    )
    lid.visual(
        Box((0.005, 0.352, lid_skirt_height)),
        origin=Origin(xyz=(0.2555, 0.0, -0.0085)),
        material=painted_metal,
        name="front_skirt",
    )
    lid.visual(
        Box((0.225, 0.003, lid_skirt_height)),
        origin=Origin(xyz=(0.1405, 0.1775, -0.0085)),
        material=painted_metal,
        name="left_side_skirt",
    )
    lid.visual(
        Box((0.225, 0.003, lid_skirt_height)),
        origin=Origin(xyz=(0.1405, -0.1775, -0.0085)),
        material=painted_metal,
        name="right_side_skirt",
    )
    lid.visual(
        _rounded_plate_mesh(
            "lid_inner_liner",
            depth=0.232,
            width=0.338,
            radius=0.021,
            height=0.0025,
            center_x=lid_center_x,
            center_z=0.0025,
        ),
        material=polymer,
        name="lid_inner_liner",
    )
    lid.visual(
        Box((0.008, 0.204, 0.012)),
        origin=Origin(xyz=(0.003, 0.0, 0.001)),
        material=painted_metal,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=knuckle_radius, length=lid_knuckle_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.012, 0.072, 0.012)),
        origin=Origin(xyz=(0.261, 0.0, -0.003)),
        material=polymer,
        name="front_pull",
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        lid.visual(
            Box((0.018, 0.022, 0.008)),
            origin=Origin(xyz=(0.132, side_sign * 0.072, 0.011)),
            material=polymer,
            name=f"{side_name}_handle_pedestal",
        )
    lid.visual(
        _mesh(
            "carry_handle",
            tube_from_spline_points(
                [
                    (0.132, -0.072, 0.015),
                    (0.130, -0.045, 0.033),
                    (0.128, 0.000, 0.045),
                    (0.130, 0.045, 0.033),
                    (0.132, 0.072, 0.015),
                ],
                radius=0.006,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=polymer,
        name="carry_handle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_outer_depth, lid_outer_width, 0.060)),
        mass=0.75,
        origin=Origin(xyz=(lid_center_x, 0.0, 0.004)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=1.92,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

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
        "lid_hinge_limits_are_reasonable",
        hinge.motion_limits is not None
        and math.isclose(hinge.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and (hinge.motion_limits.upper or 0.0) > 1.5,
        "lid should open from closed to a broad access angle.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_within(
            body,
            lid,
            axes="xy",
            inner_elem="body_wall_shell",
            outer_elem="lid_top_panel",
            margin=0.002,
            name="body_within_lid_footprint",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_inner_liner",
            negative_elem="body_wall_shell",
            min_gap=0.0,
            max_gap=0.003,
            name="closed_lid_has_tight_seam_gap",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_top_panel",
            elem_b="body_wall_shell",
            min_overlap=0.20,
            name="closed_lid_covers_body_plan",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem="front_skirt",
            negative_elem="body_wall_shell",
            min_gap=0.0005,
            max_gap=0.004,
            name="front_seam_is_tight_but_clear",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem="left_side_skirt",
            negative_elem="body_wall_shell",
            min_gap=0.0005,
            max_gap=0.004,
            name="left_side_seam_is_tight_but_clear",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="y",
            positive_elem="left_hinge_barrel",
            negative_elem="lid_hinge_barrel",
            min_gap=0.006,
            max_gap=0.014,
            name="hinge_barrels_are_interleaved_with_clearance",
        )

    with ctx.pose({hinge: 1.30}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_pull",
            negative_elem="body_wall_shell",
            min_gap=0.10,
            name="opened_lid_front_edge_clears_body",
        )

    with ctx.pose({hinge: 0.0}):
        closed_front_pull = ctx.part_element_world_aabb(lid, elem="front_pull")
    with ctx.pose({hinge: 1.30}):
        opened_front_pull = ctx.part_element_world_aabb(lid, elem="front_pull")
    if closed_front_pull is not None and opened_front_pull is not None:
        ctx.check(
            "lid_opens_upward",
            opened_front_pull[0][2] > closed_front_pull[1][2] + 0.08,
            "front pull should rise significantly when the lid opens.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
