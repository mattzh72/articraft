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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_travel_blender")

    cup_clear = model.material("cup_clear", rgba=(0.84, 0.90, 0.96, 0.38))
    housing_black = model.material("housing_black", rgba=(0.13, 0.13, 0.14, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.73, 0.75, 0.79, 1.0))

    cup_body = model.part("cup_body")
    cup_outer = [
        (0.018, 0.000),
        (0.039, 0.006),
        (0.041, 0.030),
        (0.042, 0.125),
        (0.042, 0.178),
        (0.041, 0.185),
    ]
    cup_inner = [
        (0.000, 0.014),
        (0.031, 0.018),
        (0.0365, 0.038),
        (0.0380, 0.172),
    ]
    cup_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(cup_outer, cup_inner, segments=72),
        "cup_shell",
    )
    cup_body.visual(cup_shell_mesh, material=cup_clear, name="cup_shell")
    cup_body.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.011, 0.000), (0.016, 0.0015), (0.018, 0.0060)],
                [(0.0057, 0.0006), (0.0065, 0.0060)],
                segments=48,
            ),
            "drive_socket_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=housing_black,
        name="drive_socket",
    )

    base_drive = model.part("base_drive")
    base_drive.visual(
        Cylinder(radius=0.048, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=housing_black,
        name="motor_body",
    )
    base_drive.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        material=rubber_dark,
        name="base_ring",
    )
    base_drive.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=housing_black,
        name="top_mount",
    )
    base_drive.visual(
        Box((0.016, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.045, -0.034)),
        material=steel,
        name="power_button",
    )

    lid_ring = model.part("lid_ring")
    lid_ring.visual(
        Cylinder(radius=0.045, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=housing_black,
        name="lid_plate",
    )
    lid_ring.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=rubber_dark,
        name="seal_plug",
    )
    lid_ring.visual(
        Box((0.040, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.022, 0.012)),
        material=housing_black,
        name="spout_block",
    )
    lid_ring.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(-0.014, 0.006, 0.013)),
        material=steel,
        name="left_hinge_ear",
    )
    lid_ring.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(0.014, 0.006, 0.013)),
        material=steel,
        name="right_hinge_ear",
    )

    spout_cover = model.part("spout_cover")
    cover_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.040, 0.030, 0.008, corner_segments=8),
            0.004,
            center=True,
        ),
        "spout_cover_shell",
    )
    spout_cover.visual(
        cover_mesh,
        origin=Origin(xyz=(0.0, 0.015, 0.002)),
        material=housing_black,
        name="cover_shell",
    )
    spout_cover.visual(
        Box((0.018, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.032, 0.004)),
        material=housing_black,
        name="cover_tab",
    )
    spout_cover.visual(
        Cylinder(radius=0.003, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="blade_stem",
    )
    blade_assembly.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="blade_hub",
    )
    blade_assembly.visual(
        Box((0.028, 0.007, 0.0025)),
        origin=Origin(xyz=(0.010, 0.0, 0.008), rpy=(0.0, 0.22, 0.18)),
        material=blade_steel,
        name="blade_long_a",
    )
    blade_assembly.visual(
        Box((0.028, 0.007, 0.0025)),
        origin=Origin(xyz=(-0.010, 0.0, 0.008), rpy=(0.0, -0.22, 0.18)),
        material=blade_steel,
        name="blade_long_b",
    )
    blade_assembly.visual(
        Box((0.022, 0.006, 0.0025)),
        origin=Origin(xyz=(0.0, 0.009, 0.010), rpy=(0.0, -0.30, math.pi / 2.0 - 0.08)),
        material=blade_steel,
        name="blade_short_a",
    )
    blade_assembly.visual(
        Box((0.020, 0.006, 0.0025)),
        origin=Origin(xyz=(0.0, -0.008, 0.009), rpy=(0.0, 0.28, math.pi / 2.0 + 0.06)),
        material=blade_steel,
        name="blade_short_b",
    )

    model.articulation(
        "cup_to_base",
        ArticulationType.FIXED,
        parent=cup_body,
        child=base_drive,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )
    model.articulation(
        "cup_to_lid_ring",
        ArticulationType.FIXED,
        parent=cup_body,
        child=lid_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
    )
    model.articulation(
        "blade_axle",
        ArticulationType.REVOLUTE,
        parent=cup_body,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=25.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "spout_hinge",
        ArticulationType.REVOLUTE,
        parent=lid_ring,
        child=spout_cover,
        origin=Origin(xyz=(0.0, 0.007, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=3.0,
            lower=0.0,
            upper=1.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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
        "blade_assembly",
        "cup_body",
        elem_a="blade_stem",
        elem_b="cup_shell",
        reason="The blade spindle intentionally passes through the cup-bottom bearing opening, which is simplified inside the lathed cup shell.",
    )
    ctx.allow_overlap(
        "blade_assembly",
        "cup_body",
        elem_a="blade_stem",
        elem_b="drive_socket",
        reason="The blade spindle intentionally mates into the cup-bottom drive socket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    cup_body = object_model.get_part("cup_body")
    base_drive = object_model.get_part("base_drive")
    lid_ring = object_model.get_part("lid_ring")
    spout_cover = object_model.get_part("spout_cover")
    blade_assembly = object_model.get_part("blade_assembly")
    blade_axle = object_model.get_articulation("blade_axle")
    spout_hinge = object_model.get_articulation("spout_hinge")

    ctx.check(
        "prompt parts are present",
        {part.name for part in object_model.parts}
        >= {"cup_body", "base_drive", "lid_ring", "spout_cover", "blade_assembly"},
        details=str([part.name for part in object_model.parts]),
    )
    ctx.check(
        "blade axle is vertical",
        blade_axle.axis == (0.0, 0.0, 1.0),
        details=f"axis={blade_axle.axis}",
    )
    ctx.check(
        "spout hinge runs across the lid",
        spout_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={spout_hinge.axis}",
    )

    ctx.expect_gap(
        cup_body,
        base_drive,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="base drive seats directly under the cup",
    )
    ctx.expect_overlap(
        cup_body,
        base_drive,
        axes="xy",
        min_overlap=0.070,
        name="base drive stays centered below the cup",
    )
    ctx.expect_contact(
        lid_ring,
        cup_body,
        elem_a="lid_plate",
        elem_b="cup_shell",
        name="lid plate contacts the cup rim",
    )
    ctx.expect_within(
        lid_ring,
        cup_body,
        axes="xy",
        inner_elem="seal_plug",
        outer_elem="cup_shell",
        margin=0.001,
        name="lid seal plug fits within the cup opening",
    )
    ctx.expect_overlap(
        lid_ring,
        cup_body,
        axes="xy",
        min_overlap=0.072,
        name="lid ring covers the cup opening",
    )
    ctx.expect_within(
        blade_assembly,
        cup_body,
        axes="xy",
        margin=0.002,
        name="blade assembly stays within the cup wall",
    )

    ctx.expect_contact(
        spout_cover,
        lid_ring,
        elem_a="cover_shell",
        elem_b="spout_block",
        name="closed spout cover closes flush onto the spout block",
    )
    ctx.expect_overlap(
        spout_cover,
        lid_ring,
        axes="xy",
        min_overlap=0.030,
        name="spout cover aligns over the drinking opening",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    blade_closed = _aabb_center(ctx.part_element_world_aabb(blade_assembly, elem="blade_long_a"))
    with ctx.pose({blade_axle: math.pi / 2.0}):
        blade_turned = _aabb_center(ctx.part_element_world_aabb(blade_assembly, elem="blade_long_a"))
    ctx.check(
        "blade articulation rotates the cutter",
        blade_closed is not None
        and blade_turned is not None
        and blade_closed[0] > 0.010
        and blade_turned[1] > 0.010
        and abs(blade_turned[0]) < blade_closed[0],
        details=f"closed={blade_closed}, turned={blade_turned}",
    )

    spout_closed = _aabb_center(ctx.part_element_world_aabb(spout_cover, elem="cover_tab"))
    with ctx.pose({spout_hinge: 1.45}):
        spout_open = _aabb_center(ctx.part_element_world_aabb(spout_cover, elem="cover_tab"))
    ctx.check(
        "spout lid flips upward for drinking",
        spout_closed is not None
        and spout_open is not None
        and spout_open[2] > spout_closed[2] + 0.020
        and spout_open[1] < spout_closed[1],
        details=f"closed={spout_closed}, open={spout_open}",
    )

    cup_aabb = ctx.part_world_aabb(cup_body)
    base_aabb = ctx.part_world_aabb(base_drive)
    lid_aabb = ctx.part_world_aabb(lid_ring)
    spout_aabb = ctx.part_world_aabb(spout_cover)
    if cup_aabb and base_aabb and lid_aabb and spout_aabb:
        total_z_min = min(cup_aabb[0][2], base_aabb[0][2], lid_aabb[0][2], spout_aabb[0][2])
        total_z_max = max(cup_aabb[1][2], base_aabb[1][2], lid_aabb[1][2], spout_aabb[1][2])
        total_x_min = min(cup_aabb[0][0], base_aabb[0][0], lid_aabb[0][0], spout_aabb[0][0])
        total_x_max = max(cup_aabb[1][0], base_aabb[1][0], lid_aabb[1][0], spout_aabb[1][0])
        total_y_min = min(cup_aabb[0][1], base_aabb[0][1], lid_aabb[0][1], spout_aabb[0][1])
        total_y_max = max(cup_aabb[1][1], base_aabb[1][1], lid_aabb[1][1], spout_aabb[1][1])
        total_height = total_z_max - total_z_min
        max_diameter = max(total_x_max - total_x_min, total_y_max - total_y_min)
        ctx.check(
            "portable blender proportions are realistic",
            0.23 <= total_height <= 0.31 and 0.080 <= max_diameter <= 0.110,
            details=f"height={total_height:.4f}, diameter={max_diameter:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
