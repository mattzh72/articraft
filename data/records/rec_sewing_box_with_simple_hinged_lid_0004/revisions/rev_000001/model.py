from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.260
BODY_DEPTH = 0.180
BODY_HEIGHT = 0.084
BODY_CORNER_RADIUS = 0.022
OPENING_WIDTH = 0.226
OPENING_DEPTH = 0.146
OPENING_RADIUS = 0.014

HINGE_AXIS_Y = -0.094
HINGE_AXIS_Z = 0.090


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_rect_solid(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
    radius: float,
):
    return _mesh(
        name,
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, depth, radius, corner_segments=10),
            height,
        ),
    )


def _rounded_rect_ring(
    name: str,
    *,
    outer_width: float,
    outer_depth: float,
    inner_width: float,
    inner_depth: float,
    height: float,
    outer_radius: float,
    inner_radius: float,
):
    return _mesh(
        name,
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_width, outer_depth, outer_radius, corner_segments=10),
            [rounded_rect_profile(inner_width, inner_depth, inner_radius, corner_segments=10)],
            height,
            center=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sewing_box", assets=ASSETS)

    shell_matte = model.material("shell_matte", rgba=(0.25, 0.19, 0.15, 1.0))
    inset_matte = model.material("inset_matte", rgba=(0.34, 0.27, 0.23, 1.0))
    satin_trim = model.material("satin_trim", rgba=(0.75, 0.69, 0.57, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.24, 0.26, 0.28, 1.0))
    lining_fabric = model.material("lining_fabric", rgba=(0.72, 0.64, 0.55, 1.0))
    lining_divider = model.material("lining_divider", rgba=(0.58, 0.51, 0.43, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    body_plinth_mesh = _rounded_rect_solid(
        "body_plinth.obj",
        width=0.248,
        depth=0.168,
        height=0.010,
        radius=0.018,
    )
    body_shell_mesh = _rounded_rect_ring(
        "body_shell.obj",
        outer_width=BODY_WIDTH,
        outer_depth=BODY_DEPTH,
        inner_width=OPENING_WIDTH,
        inner_depth=OPENING_DEPTH,
        height=0.072,
        outer_radius=BODY_CORNER_RADIUS,
        inner_radius=OPENING_RADIUS,
    )
    body_floor_mesh = _rounded_rect_solid(
        "body_floor.obj",
        width=0.244,
        depth=0.164,
        height=0.010,
        radius=0.017,
    )
    body_seat_ring_mesh = _rounded_rect_ring(
        "body_seat_ring.obj",
        outer_width=BODY_WIDTH,
        outer_depth=BODY_DEPTH,
        inner_width=OPENING_WIDTH,
        inner_depth=OPENING_DEPTH,
        height=0.006,
        outer_radius=BODY_CORNER_RADIUS,
        inner_radius=OPENING_RADIUS,
    )
    body_liner_mesh = _rounded_rect_solid(
        "body_liner.obj",
        width=0.212,
        depth=0.132,
        height=0.003,
        radius=0.012,
    )

    lid_top_mesh = _rounded_rect_solid(
        "lid_top.obj",
        width=0.262,
        depth=0.170,
        height=0.004,
        radius=0.019,
    )
    lid_frame_mesh = _rounded_rect_ring(
        "lid_frame.obj",
        outer_width=0.262,
        outer_depth=0.170,
        inner_width=0.212,
        inner_depth=0.120,
        height=0.010,
        outer_radius=0.019,
        inner_radius=0.011,
    )
    lid_panel_mesh = _rounded_rect_solid(
        "lid_panel.obj",
        width=0.214,
        depth=0.122,
        height=0.004,
        radius=0.010,
    )
    lid_landing_ring_mesh = _rounded_rect_ring(
        "lid_landing_ring.obj",
        outer_width=0.258,
        outer_depth=0.166,
        inner_width=0.230,
        inner_depth=0.138,
        height=0.006,
        outer_radius=0.018,
        inner_radius=0.012,
    )
    lid_lip_ring_mesh = _rounded_rect_ring(
        "lid_lip_ring.obj",
        outer_width=0.224,
        outer_depth=0.144,
        inner_width=0.202,
        inner_depth=0.122,
        height=0.014,
        outer_radius=0.013,
        inner_radius=0.008,
    )
    hinge_support_mesh = _rounded_rect_solid(
        "hinge_support.obj",
        width=0.030,
        depth=0.014,
        height=0.018,
        radius=0.004,
    )
    hinge_rail_mesh = _rounded_rect_solid(
        "hinge_rail.obj",
        width=0.140,
        depth=0.010,
        height=0.010,
        radius=0.003,
    )
    front_pull_mesh = _mesh(
        "front_pull.obj",
        sweep_profile_along_spline(
            [
                (-0.024, 0.0, 0.0),
                (-0.012, 0.0, 0.0014),
                (0.0, 0.0, 0.0020),
                (0.012, 0.0, 0.0014),
                (0.024, 0.0, 0.0),
            ],
            profile=rounded_rect_profile(0.004, 0.0032, radius=0.0011, corner_segments=6),
            samples_per_segment=12,
            cap_profile=True,
        ),
    )

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )
    body.visual(body_plinth_mesh, origin=Origin(xyz=(0.0, 0.0, 0.005)), material=shell_matte, name="base_plinth")
    body.visual(body_floor_mesh, origin=Origin(xyz=(0.0, 0.0, 0.013)), material=shell_matte, name="body_floor")
    body.visual(body_shell_mesh, origin=Origin(xyz=(0.0, 0.0, 0.046)), material=shell_matte, name="outer_shell")
    body.visual(
        body_seat_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=satin_trim,
        name="seat_ring",
    )
    body.visual(
        body_liner_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=lining_fabric,
        name="lining_pad",
    )
    body.visual(
        Box((0.006, 0.132, 0.028)),
        origin=Origin(xyz=(0.028, 0.0, 0.031)),
        material=lining_divider,
        name="center_divider",
    )
    body.visual(
        Box((0.084, 0.006, 0.028)),
        origin=Origin(xyz=(-0.036, 0.026, 0.031)),
        material=lining_divider,
        name="cross_divider",
    )
    body.visual(
        Box((0.056, 0.006, 0.022)),
        origin=Origin(xyz=(-0.070, -0.028, 0.028)),
        material=lining_divider,
        name="notion_divider",
    )
    body.visual(
        Box((0.174, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.086, 0.076)),
        material=satin_trim,
        name="rear_bridge",
    )
    body.visual(
        hinge_support_mesh,
        origin=Origin(xyz=(0.088, -0.092, 0.081)),
        material=shell_matte,
        name="left_hinge_support",
    )
    body.visual(
        hinge_support_mesh,
        origin=Origin(xyz=(-0.088, -0.092, 0.081)),
        material=shell_matte,
        name="right_hinge_support",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.088, HINGE_AXIS_Y, HINGE_AXIS_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(-0.088, HINGE_AXIS_Y, HINGE_AXIS_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="right_hinge_barrel",
    )
    for foot_name, foot_x, foot_y in (
        ("front_left_foot", 0.082, 0.054),
        ("front_right_foot", -0.082, 0.054),
        ("rear_left_foot", 0.082, -0.054),
        ("rear_right_foot", -0.082, -0.054),
    ):
        body.visual(
            Cylinder(radius=0.009, length=0.005),
            origin=Origin(xyz=(foot_x, foot_y, 0.0025)),
            material=foot_rubber,
            name=foot_name,
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.262, 0.184, 0.030)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.094, -0.001)),
    )
    lid.visual(lid_top_mesh, origin=Origin(xyz=(0.0, 0.094, 0.002)), material=shell_matte, name="top_slab")
    lid.visual(
        lid_frame_mesh,
        origin=Origin(xyz=(0.0, 0.094, 0.007)),
        material=satin_trim,
        name="frame_ring",
    )
    lid.visual(
        lid_panel_mesh,
        origin=Origin(xyz=(0.0, 0.094, 0.003)),
        material=inset_matte,
        name="inset_panel",
    )
    lid.visual(
        lid_landing_ring_mesh,
        origin=Origin(xyz=(0.0, 0.094, -0.0030)),
        material=inset_matte,
        name="landing_ring",
    )
    lid.visual(
        lid_lip_ring_mesh,
        origin=Origin(xyz=(0.0, 0.094, -0.006)),
        material=lining_fabric,
        name="lip_ring",
    )
    lid.visual(
        hinge_rail_mesh,
        origin=Origin(xyz=(0.0, 0.006, 0.004)),
        material=shell_matte,
        name="rear_hinge_rail",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="center_hinge_barrel",
    )
    lid.visual(
        front_pull_mesh,
        origin=Origin(xyz=(0.0, 0.1745, 0.0040)),
        material=satin_trim,
        name="front_pull",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    body_seat_ring = body.get_visual("seat_ring")
    body_left_barrel = body.get_visual("left_hinge_barrel")
    body_right_barrel = body.get_visual("right_hinge_barrel")
    lid_landing_ring = lid.get_visual("landing_ring")
    lid_center_barrel = lid.get_visual("center_hinge_barrel")
    lid_front_pull = lid.get_visual("front_pull")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    hinge_axis = tuple(round(value, 3) for value in lid_hinge.axis)
    ctx.check(
        "lid_hinge_uses_simple_crosswise_axis",
        hinge_axis == (1.0, 0.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "lid_hinge_has_realistic_open_range",
        lid_hinge.motion_limits is not None and 1.10 <= lid_hinge.motion_limits.upper <= 1.40,
        details=f"limits={lid_hinge.motion_limits}",
    )

    body_bounds = ctx.part_world_aabb(body)
    if body_bounds is not None:
        body_dims = tuple(body_bounds[1][axis] - body_bounds[0][axis] for axis in range(3))
        ctx.check(
            "body_reads_as_compact_storage_box",
            0.24 <= body_dims[0] <= 0.28 and 0.16 <= body_dims[1] <= 0.20 and 0.08 <= body_dims[2] <= 0.10,
            details=f"body_dims={body_dims}",
        )

    body_seat_aabb = ctx.part_element_world_aabb(body, elem=body_seat_ring)
    lid_landing_aabb = ctx.part_element_world_aabb(lid, elem=lid_landing_ring)
    if body_seat_aabb is not None and lid_landing_aabb is not None:
        body_center_xy = (
            0.5 * (body_seat_aabb[0][0] + body_seat_aabb[1][0]),
            0.5 * (body_seat_aabb[0][1] + body_seat_aabb[1][1]),
        )
        lid_center_xy = (
            0.5 * (lid_landing_aabb[0][0] + lid_landing_aabb[1][0]),
            0.5 * (lid_landing_aabb[0][1] + lid_landing_aabb[1][1]),
        )
        center_delta = (
            lid_center_xy[0] - body_center_xy[0],
            lid_center_xy[1] - body_center_xy[1],
        )
        ctx.check(
            "lid_seam_stays_centered_on_body",
            abs(center_delta[0]) <= 0.002 and abs(center_delta[1]) <= 0.002,
            details=f"center_delta_xy={center_delta}",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a=lid_landing_ring,
        elem_b=body_seat_ring,
        min_overlap=0.16,
        name="lid_frame_overlaps_seat_ring",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem=lid_landing_ring,
        negative_elem=body_seat_ring,
        max_gap=0.0002,
        max_penetration=1e-6,
        name="lid_lands_tightly_without_sinking",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a=lid_landing_ring,
        elem_b=body_seat_ring,
        contact_tol=0.0002,
        name="lid_landing_ring_contacts_body_seat_ring",
    )
    ctx.expect_gap(
        body,
        lid,
        axis="x",
        positive_elem=body_left_barrel,
        negative_elem=lid_center_barrel,
        min_gap=0.010,
        name="left_hinge_knuckle_break_is_clear",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        positive_elem=lid_center_barrel,
        negative_elem=body_right_barrel,
        min_gap=0.010,
        name="right_hinge_knuckle_break_is_clear",
    )

    with ctx.pose({lid_hinge: 0.70}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid_front_pull,
            negative_elem=body_seat_ring,
            min_gap=0.055,
            name="front_pull_rises_in_mid_open_pose",
        )

    with ctx.pose({lid_hinge: 1.15}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid_front_pull,
            negative_elem=body_seat_ring,
            min_gap=0.120,
            name="front_pull_clears_body_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
