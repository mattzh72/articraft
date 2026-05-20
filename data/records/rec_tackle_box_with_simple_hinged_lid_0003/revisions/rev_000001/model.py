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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tackle_box", assets=ASSETS)

    matte_body = model.material("matte_body", rgba=(0.20, 0.23, 0.25, 1.0))
    satin_panel = model.material("satin_panel", rgba=(0.36, 0.39, 0.42, 1.0))
    satin_hardware = model.material("satin_hardware", rgba=(0.67, 0.70, 0.73, 1.0))
    interior_finish = model.material("interior_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    latch_finish = model.material("latch_finish", rgba=(0.30, 0.32, 0.35, 1.0))

    body_width = 0.330
    body_depth = 0.210
    body_radius = 0.018
    wall_thickness = 0.0045
    floor_thickness = 0.005
    body_height = 0.125

    lid_width = 0.340
    lid_depth = 0.214
    lid_radius = 0.019
    lid_wall = 0.0042
    lid_height = 0.032
    lid_top_thickness = 0.004

    hinge_axis_y = -(body_depth * 0.5 + 0.006)
    hinge_axis_z = body_height + 0.004
    hinge_knuckle_radius = 0.0055
    hinge_outer_knuckle_len = 0.030
    hinge_center_knuckle_len = 0.150
    hinge_knuckle_offset_x = 0.090

    latch_knuckle_radius = 0.0042
    latch_outer_knuckle_len = 0.012
    latch_center_knuckle_len = 0.028
    latch_pivot_y = 0.226
    latch_pivot_z = 0.015

    def rr_profile(width: float, depth: float, radius: float):
        return rounded_rect_profile(width, depth, radius, corner_segments=8)

    def mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def ring_mesh(
        name: str,
        *,
        outer_w: float,
        outer_d: float,
        outer_r: float,
        inner_w: float,
        inner_d: float,
        inner_r: float,
        height: float,
    ):
        return mesh(
            name,
            ExtrudeWithHolesGeometry(
                rr_profile(outer_w, outer_d, outer_r),
                [rr_profile(inner_w, inner_d, inner_r)],
                height=height,
                center=False,
            ),
        )

    def panel_mesh(name: str, *, width: float, depth: float, radius: float, height: float):
        return mesh(name, ExtrudeGeometry.from_z0(rr_profile(width, depth, radius), height))

    def centered_panel_mesh(name: str, *, width: float, depth: float, radius: float, height: float):
        return mesh(name, ExtrudeGeometry(rr_profile(width, depth, radius), height))

    body_wall_mesh = ring_mesh(
        "tackle_body_wall.obj",
        outer_w=body_width,
        outer_d=body_depth,
        outer_r=body_radius,
        inner_w=body_width - 2.0 * wall_thickness,
        inner_d=body_depth - 2.0 * wall_thickness,
        inner_r=max(body_radius - wall_thickness, 0.002),
        height=body_height,
    )
    body_floor_mesh = panel_mesh(
        "tackle_body_floor.obj",
        width=body_width - 2.0 * wall_thickness + 0.0006,
        depth=body_depth - 2.0 * wall_thickness + 0.0006,
        radius=max(body_radius - wall_thickness + 0.0003, 0.002),
        height=floor_thickness,
    )
    lid_wall_mesh = ring_mesh(
        "tackle_lid_wall.obj",
        outer_w=lid_width,
        outer_d=lid_depth,
        outer_r=lid_radius,
        inner_w=lid_width - 2.0 * lid_wall,
        inner_d=lid_depth - 2.0 * lid_wall,
        inner_r=max(lid_radius - lid_wall, 0.002),
        height=lid_height,
    )
    lid_top_mesh = panel_mesh(
        "tackle_lid_top.obj",
        width=lid_width,
        depth=lid_depth,
        radius=lid_radius,
        height=lid_top_thickness,
    )
    lid_inset_mesh = panel_mesh(
        "tackle_lid_inset.obj",
        width=lid_width - 0.040,
        depth=lid_depth - 0.050,
        radius=lid_radius - 0.006,
        height=0.0022,
    )
    hinge_support_mesh = centered_panel_mesh(
        "tackle_hinge_support.obj",
        width=0.024,
        depth=0.012,
        radius=0.0025,
        height=0.012,
    )
    hinge_leaf_mesh = centered_panel_mesh(
        "tackle_hinge_leaf.obj",
        width=0.040,
        depth=0.014,
        radius=0.0035,
        height=0.012,
    )
    seam_trim_mesh = centered_panel_mesh(
        "tackle_front_seam_trim.obj",
        width=body_width - 0.070,
        depth=0.004,
        radius=0.0012,
        height=0.004,
    )
    striker_mesh = centered_panel_mesh(
        "tackle_front_striker.obj",
        width=0.062,
        depth=0.010,
        radius=0.003,
        height=0.018,
    )
    latch_mount_mesh = centered_panel_mesh(
        "tackle_latch_mount.obj",
        width=0.014,
        depth=0.018,
        radius=0.002,
        height=0.012,
    )
    latch_neck_mesh = centered_panel_mesh(
        "tackle_latch_neck.obj",
        width=0.014,
        depth=0.006,
        radius=0.0018,
        height=0.016,
    )
    latch_plate_mesh = centered_panel_mesh(
        "tackle_latch_plate.obj",
        width=0.052,
        depth=0.0055,
        radius=0.002,
        height=0.030,
    )
    latch_pull_mesh = centered_panel_mesh(
        "tackle_latch_pull.obj",
        width=0.040,
        depth=0.007,
        radius=0.0025,
        height=0.009,
    )

    base = model.part("base_shell")
    base.visual(body_wall_mesh, material=matte_body, name="body_wall")
    base.visual(body_floor_mesh, material=interior_finish, name="body_floor")
    base.visual(
        Cylinder(radius=hinge_knuckle_radius, length=hinge_outer_knuckle_len),
        origin=Origin(
            xyz=(-hinge_knuckle_offset_x, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_hardware,
        name="hinge_left_knuckle",
    )
    base.visual(
        Cylinder(radius=hinge_knuckle_radius, length=hinge_outer_knuckle_len),
        origin=Origin(
            xyz=(hinge_knuckle_offset_x, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_hardware,
        name="hinge_right_knuckle",
    )
    base.visual(
        hinge_support_mesh,
        origin=Origin(xyz=(-hinge_knuckle_offset_x, hinge_axis_y + 0.004, hinge_axis_z - 0.003)),
        material=satin_panel,
        name="hinge_support_left",
    )
    base.visual(
        hinge_support_mesh,
        origin=Origin(xyz=(hinge_knuckle_offset_x, hinge_axis_y + 0.004, hinge_axis_z - 0.003)),
        material=satin_panel,
        name="hinge_support_right",
    )
    base.visual(
        seam_trim_mesh,
        origin=Origin(xyz=(0.0, body_depth * 0.5 - 0.002, body_height - 0.002)),
        material=satin_panel,
        name="front_seam_trim",
    )
    base.visual(
        striker_mesh,
        origin=Origin(xyz=(0.0, body_depth * 0.5 + 0.001, body_height - 0.010)),
        material=latch_finish,
        name="front_striker",
    )
    base.visual(
        Cylinder(radius=0.0027, length=0.046),
        origin=Origin(
            xyz=(0.0, body_depth * 0.5 + 0.0045, body_height - 0.004),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_hardware,
        name="front_striker_bar",
    )
    base.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid_panel")
    lid.visual(
        lid_wall_mesh,
        origin=Origin(xyz=(0.0, 0.114, -0.004)),
        material=matte_body,
        name="lid_outer_wall",
    )
    lid.visual(
        lid_top_mesh,
        origin=Origin(xyz=(0.0, 0.114, lid_height - lid_top_thickness - 0.004)),
        material=matte_body,
        name="lid_top_panel",
    )
    lid.visual(
        lid_inset_mesh,
        origin=Origin(xyz=(0.0, 0.114, lid_height - 0.0062)),
        material=satin_panel,
        name="lid_inset_panel",
    )
    lid.visual(
        Cylinder(radius=hinge_knuckle_radius, length=hinge_center_knuckle_len),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_hardware,
        name="hinge_center_knuckle",
    )
    lid.visual(
        hinge_leaf_mesh,
        origin=Origin(xyz=(-0.048, 0.012, -0.002)),
        material=satin_panel,
        name="hinge_leaf_left",
    )
    lid.visual(
        hinge_leaf_mesh,
        origin=Origin(xyz=(0.048, 0.012, -0.002)),
        material=satin_panel,
        name="hinge_leaf_right",
    )
    lid.visual(
        latch_mount_mesh,
        origin=Origin(xyz=(-0.029, 0.214, 0.015)),
        material=satin_panel,
        name="latch_mount_left",
    )
    lid.visual(
        latch_mount_mesh,
        origin=Origin(xyz=(0.029, 0.214, 0.015)),
        material=satin_panel,
        name="latch_mount_right",
    )
    lid.visual(
        Cylinder(radius=latch_knuckle_radius, length=latch_outer_knuckle_len),
        origin=Origin(
            xyz=(-0.020, latch_pivot_y, latch_pivot_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_hardware,
        name="latch_left_knuckle",
    )
    lid.visual(
        Cylinder(radius=latch_knuckle_radius, length=latch_outer_knuckle_len),
        origin=Origin(
            xyz=(0.020, latch_pivot_y, latch_pivot_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_hardware,
        name="latch_right_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_height + 0.020)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.110, 0.008)),
    )

    latch = model.part("latch_clasp")
    latch.visual(
        Cylinder(radius=latch_knuckle_radius, length=latch_center_knuckle_len),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_hardware,
        name="latch_center_knuckle",
    )
    latch.visual(
        latch_neck_mesh,
        origin=Origin(xyz=(0.0, 0.002, -0.009)),
        material=latch_finish,
        name="latch_neck",
    )
    latch.visual(
        latch_plate_mesh,
        origin=Origin(xyz=(0.0, 0.002, -0.024)),
        material=latch_finish,
        name="latch_plate",
    )
    latch.visual(
        latch_pull_mesh,
        origin=Origin(xyz=(0.0, 0.0045, -0.040)),
        material=latch_finish,
        name="latch_pull",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.056, 0.012, 0.046)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.002, -0.023)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.0, latch_pivot_y, latch_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_shell")
    lid = object_model.get_part("lid_panel")
    latch = object_model.get_part("latch_clasp")
    lid_hinge = object_model.get_articulation("lid_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.check("lid_hinge_axis", lid_hinge.axis == (1.0, 0.0, 0.0), f"axis={lid_hinge.axis}")
    ctx.check(
        "lid_hinge_limits",
        lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and 1.8 < lid_hinge.motion_limits.upper < 2.0,
        f"limits={lid_hinge.motion_limits}",
    )
    ctx.check("latch_axis", latch_pivot.axis == (1.0, 0.0, 0.0), f"axis={latch_pivot.axis}")
    ctx.check(
        "latch_limits",
        latch_pivot.motion_limits is not None
        and latch_pivot.motion_limits.lower == 0.0
        and latch_pivot.motion_limits.upper is not None
        and 1.2 < latch_pivot.motion_limits.upper < 1.5,
        f"limits={latch_pivot.motion_limits}",
    )

    base.get_visual("body_wall")
    base.get_visual("front_seam_trim")
    base.get_visual("hinge_left_knuckle")
    base.get_visual("hinge_right_knuckle")
    lid.get_visual("lid_top_panel")
    lid.get_visual("hinge_center_knuckle")
    lid.get_visual("latch_left_knuckle")
    lid.get_visual("latch_right_knuckle")
    latch.get_visual("latch_center_knuckle")
    latch.get_visual("latch_plate")

    with ctx.pose({lid_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_contact(
            lid,
            base,
            elem_a="hinge_center_knuckle",
            elem_b="hinge_left_knuckle",
            contact_tol=0.0005,
            name="closed_hinge_contact",
        )
        ctx.expect_contact(
            latch,
            lid,
            elem_a="latch_center_knuckle",
            elem_b="latch_left_knuckle",
            contact_tol=0.0005,
            name="closed_latch_pivot_contact",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.190,
            elem_a="lid_top_panel",
            elem_b="body_wall",
            name="closed_lid_plan_overlap",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=-0.0005,
            max_gap=0.0015,
            positive_elem="lid_outer_wall",
            negative_elem="body_wall",
            name="closed_perimeter_seam",
        )
        ctx.expect_overlap(
            latch,
            base,
            axes="xz",
            min_overlap=0.016,
            elem_a="latch_plate",
            elem_b="front_striker",
            name="closed_latch_aligned_to_striker",
        )
        ctx.expect_gap(
            latch,
            base,
            axis="y",
            min_gap=0.001,
            max_gap=0.006,
            positive_elem="latch_plate",
            negative_elem="front_striker",
            name="closed_latch_front_gap",
        )

    latch_rest_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_pivot: math.radians(78.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="latch_open_no_overlap")
        ctx.fail_if_isolated_parts(name="latch_open_no_floating")
        ctx.expect_contact(
            latch,
            lid,
            elem_a="latch_center_knuckle",
            elem_b="latch_left_knuckle",
            contact_tol=0.0005,
            name="open_latch_pivot_contact",
        )
        latch_open_aabb = ctx.part_world_aabb(latch)
        ctx.check(
            "latch_rotates_forward",
            latch_rest_aabb is not None
            and latch_open_aabb is not None
            and latch_open_aabb[1][1] > latch_rest_aabb[1][1] + 0.012
            and latch_open_aabb[0][2] > latch_rest_aabb[0][2] + 0.010,
            f"rest={latch_rest_aabb}, open={latch_open_aabb}",
        )

    with ctx.pose({lid_hinge: math.radians(112.0), latch_pivot: math.radians(78.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_open_no_floating")
        ctx.expect_contact(
            lid,
            base,
            elem_a="hinge_center_knuckle",
            elem_b="hinge_left_knuckle",
            contact_tol=0.0005,
            name="open_hinge_contact",
        )
        lid_open_aabb = ctx.part_world_aabb(lid)
        base_aabb = ctx.part_world_aabb(base)
        ctx.check(
            "lid_swings_clear",
            lid_open_aabb is not None
            and base_aabb is not None
            and lid_open_aabb[1][2] > base_aabb[1][2] + 0.120
            and lid_open_aabb[0][1] < base_aabb[0][1] + 0.010,
            f"lid_open={lid_open_aabb}, base={base_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
