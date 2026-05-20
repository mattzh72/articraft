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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _xz_section(
    width: float,
    height: float,
    radius: float,
    *,
    y_pos: float,
    z_center: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y_pos, z + z_center)
        for x, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    *,
    x_pos: float,
    y_center: float,
    z_center: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y + y_center, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    *,
    x_center: float,
    y_center: float,
    z_pos: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_center, y + y_center, z_pos)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate", assets=ASSETS)

    bed_paint = model.material("bed_paint", rgba=(0.25, 0.27, 0.30, 1.0))
    tailgate_paint = model.material("tailgate_paint", rgba=(0.72, 0.74, 0.77, 1.0))
    inner_trim = model.material("inner_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    cap_trim = model.material("cap_trim", rgba=(0.11, 0.12, 0.13, 1.0))
    latch_metal = model.material("latch_metal", rgba=(0.78, 0.80, 0.83, 1.0))

    tailgate_width = 1.46
    tailgate_height = 0.56

    bed_supports = model.part("bed_supports")
    bed_supports.visual(
        Box((0.10, 0.18, 0.72)),
        origin=Origin(xyz=(-0.80, 0.06, 0.36)),
        material=bed_paint,
        name="left_bedside",
    )
    bed_supports.visual(
        Box((0.10, 0.18, 0.72)),
        origin=Origin(xyz=(0.80, 0.06, 0.36)),
        material=bed_paint,
        name="right_bedside",
    )
    bed_supports.visual(
        Box((1.70, 0.12, 0.09)),
        origin=Origin(xyz=(0.0, 0.08, 0.045)),
        material=bed_paint,
        name="lower_sill",
    )
    bed_supports.visual(
        Box((0.022, 0.055, 0.12)),
        origin=Origin(xyz=(-0.761, 0.03, 0.52)),
        material=inner_trim,
        name="left_striker",
    )
    bed_supports.visual(
        Box((0.022, 0.055, 0.12)),
        origin=Origin(xyz=(0.761, 0.03, 0.52)),
        material=inner_trim,
        name="right_striker",
    )
    bed_supports.visual(
        Box((0.11, 0.10, 0.10)),
        origin=Origin(xyz=(-0.74, 0.09, 0.08)),
        material=inner_trim,
        name="left_hinge_pad",
    )
    bed_supports.visual(
        Box((0.11, 0.10, 0.10)),
        origin=Origin(xyz=(0.74, 0.09, 0.08)),
        material=inner_trim,
        name="right_hinge_pad",
    )
    bed_supports.inertial = Inertial.from_geometry(
        Box((1.70, 0.18, 0.72)),
        mass=80.0,
        origin=Origin(xyz=(0.0, 0.06, 0.36)),
    )

    tailgate = model.part("tailgate")
    outer_panel_geom = section_loft(
        [
            _xz_section(1.40, 0.48, 0.07, y_pos=-0.028, z_center=0.28),
            _xz_section(1.44, 0.52, 0.07, y_pos=-0.016, z_center=0.28),
            _xz_section(1.46, 0.54, 0.055, y_pos=0.012, z_center=0.28),
        ]
    )
    tailgate.visual(
        _save_mesh("tailgate_outer_panel.obj", outer_panel_geom),
        material=tailgate_paint,
        name="outer_panel",
    )

    stamp_geom = section_loft(
        [
            _xz_section(0.94, 0.22, 0.04, y_pos=-0.034, z_center=0.29, corner_segments=6),
            _xz_section(1.02, 0.26, 0.05, y_pos=-0.026, z_center=0.29, corner_segments=6),
        ]
    )
    tailgate.visual(
        _save_mesh("tailgate_outer_stamp.obj", stamp_geom),
        material=tailgate_paint,
        name="outer_stamp",
    )

    inner_outer = _shift_profile(rounded_rect_profile(1.34, 0.48, 0.045, corner_segments=8), dy=0.27)
    upper_hole = _shift_profile(rounded_rect_profile(1.06, 0.14, 0.026, corner_segments=6), dy=0.41)
    lower_hole = _shift_profile(rounded_rect_profile(1.12, 0.18, 0.03, corner_segments=6), dy=0.19)
    inner_frame_geom = ExtrudeWithHolesGeometry(
        inner_outer,
        [upper_hole, lower_hole],
        height=0.022,
        center=True,
    ).rotate_x(math.pi / 2.0)
    tailgate.visual(
        _save_mesh("tailgate_inner_frame.obj", inner_frame_geom),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=inner_trim,
        name="inner_frame",
    )

    top_cap_geom = section_loft(
        [
            _yz_section(0.052, 0.034, 0.012, x_pos=-0.73, y_center=0.006, z_center=tailgate_height - 0.018),
            _yz_section(0.060, 0.040, 0.014, x_pos=0.0, y_center=0.006, z_center=tailgate_height - 0.018),
            _yz_section(0.052, 0.034, 0.012, x_pos=0.73, y_center=0.006, z_center=tailgate_height - 0.018),
        ]
    )
    tailgate.visual(
        _save_mesh("tailgate_top_cap.obj", top_cap_geom),
        material=cap_trim,
        name="top_cap",
    )
    tailgate.visual(
        Box((tailgate_width - 0.02, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, 0.004, 0.019)),
        material=tailgate_paint,
        name="bottom_cap",
    )
    tailgate.visual(
        Box((tailgate_width - 0.04, 0.042, 0.004)),
        origin=Origin(xyz=(0.0, 0.006, 0.002)),
        material=inner_trim,
        name="bottom_seal",
    )
    left_side_geom = section_loft(
        [
            _xy_section(0.038, 0.042, 0.010, x_center=-0.71, y_center=0.006, z_pos=0.024),
            _xy_section(0.042, 0.050, 0.012, x_center=-0.71, y_center=0.006, z_pos=0.28),
            _xy_section(0.036, 0.044, 0.010, x_center=-0.71, y_center=0.006, z_pos=0.536),
        ]
    )
    right_side_geom = section_loft(
        [
            _xy_section(0.038, 0.042, 0.010, x_center=0.71, y_center=0.006, z_pos=0.024),
            _xy_section(0.042, 0.050, 0.012, x_center=0.71, y_center=0.006, z_pos=0.28),
            _xy_section(0.036, 0.044, 0.010, x_center=0.71, y_center=0.006, z_pos=0.536),
        ]
    )
    tailgate.visual(
        _save_mesh("tailgate_left_side_edge.obj", left_side_geom),
        material=tailgate_paint,
        name="left_side_edge",
    )
    tailgate.visual(
        _save_mesh("tailgate_right_side_edge.obj", right_side_geom),
        material=tailgate_paint,
        name="right_side_edge",
    )
    tailgate.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(-0.739, 0.022, 0.43), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=latch_metal,
        name="left_latch_point",
    )
    tailgate.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(0.739, 0.022, 0.43), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=latch_metal,
        name="right_latch_point",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((1.48, 0.07, 0.56)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed_supports,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bed_supports = object_model.get_part("bed_supports")
    tailgate = object_model.get_part("tailgate")
    tailgate_hinge = object_model.get_articulation("tailgate_hinge")

    lower_sill = bed_supports.get_visual("lower_sill")
    left_striker = bed_supports.get_visual("left_striker")
    right_striker = bed_supports.get_visual("right_striker")
    top_cap = tailgate.get_visual("top_cap")
    outer_panel = tailgate.get_visual("outer_panel")
    inner_frame = tailgate.get_visual("inner_frame")
    bottom_seal = tailgate.get_visual("bottom_seal")
    left_latch_point = tailgate.get_visual("left_latch_point")
    right_latch_point = tailgate.get_visual("right_latch_point")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(tailgate, bed_supports, axes="x", max_dist=1e-6)
    ctx.expect_contact(tailgate, bed_supports, elem_a=left_latch_point, elem_b=left_striker)
    ctx.expect_contact(tailgate, bed_supports, elem_a=right_latch_point, elem_b=right_striker)
    ctx.expect_gap(
        tailgate,
        bed_supports,
        axis="z",
        positive_elem=bottom_seal,
        negative_elem=lower_sill,
        min_gap=0.0,
        max_gap=0.001,
    )
    ctx.expect_overlap(tailgate, bed_supports, axes="z", min_overlap=0.52, elem_a=outer_panel)
    ctx.expect_gap(
        bed_supports,
        tailgate,
        axis="x",
        positive_elem=right_striker,
        negative_elem=right_latch_point,
        min_gap=-0.001,
        max_gap=0.001,
    )
    ctx.expect_gap(
        tailgate,
        bed_supports,
        axis="x",
        positive_elem=left_latch_point,
        negative_elem=left_striker,
        min_gap=-0.001,
        max_gap=0.001,
    )

    inner_frame_aabb = ctx.part_element_world_aabb(tailgate, elem=inner_frame)
    outer_panel_aabb = ctx.part_element_world_aabb(tailgate, elem=outer_panel)
    assert inner_frame_aabb is not None
    assert outer_panel_aabb is not None
    inner_width = inner_frame_aabb[1][0] - inner_frame_aabb[0][0]
    inner_height = inner_frame_aabb[1][2] - inner_frame_aabb[0][2]
    outer_width = outer_panel_aabb[1][0] - outer_panel_aabb[0][0]
    outer_height = outer_panel_aabb[1][2] - outer_panel_aabb[0][2]
    ctx.check(
        "inner frame reads as inset structure",
        inner_width < outer_width and inner_height < outer_height,
        details=(
            f"inner frame dims=({inner_width:.3f}, {inner_height:.3f}) "
            f"outer panel dims=({outer_width:.3f}, {outer_height:.3f})"
        ),
    )

    top_closed = ctx.part_element_world_aabb(tailgate, elem=top_cap)
    assert top_closed is not None
    closed_top_center_y = 0.5 * (top_closed[0][1] + top_closed[1][1])
    closed_top_center_z = 0.5 * (top_closed[0][2] + top_closed[1][2])

    with ctx.pose({tailgate_hinge: math.radians(90.0)}):
        top_open = ctx.part_element_world_aabb(tailgate, elem=top_cap)
        assert top_open is not None
        open_top_center_y = 0.5 * (top_open[0][1] + top_open[1][1])
        open_top_center_z = 0.5 * (top_open[0][2] + top_open[1][2])
        ctx.check(
            "tailgate opens downward",
            open_top_center_y < closed_top_center_y - 0.45,
            details=f"closed_y={closed_top_center_y:.3f}, open_y={open_top_center_y:.3f}",
        )
        ctx.check(
            "tailgate top drops near hinge height when open",
            open_top_center_z < closed_top_center_z - 0.40 and open_top_center_z < 0.18,
            details=f"closed_z={closed_top_center_z:.3f}, open_z={open_top_center_z:.3f}",
        )
        ctx.expect_gap(
            bed_supports,
            tailgate,
            axis="y",
            positive_elem=lower_sill,
            negative_elem=top_cap,
            min_gap=0.35,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
