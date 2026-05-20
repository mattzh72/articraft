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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


BASE_TOP_Z = 0.24
PAD_HEIGHT = 0.014
PAD_TOP_Z = BASE_TOP_Z + PAD_HEIGHT
JAR_CENTER_X = 0.17

JAR_OUTER_W = 0.228
JAR_OUTER_D = 0.175
JAR_INNER_W = 0.214
JAR_INNER_D = 0.161
JAR_FLOOR_T = 0.012
JAR_WALL_H = 0.265
JAR_HEIGHT = JAR_FLOOR_T + JAR_WALL_H


def _rounded_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)]


def _build_base_body_geometry():
    return section_loft(
        [
            _rounded_section(0.66, 0.32, 0.075, 0.0),
            _rounded_section(0.62, 0.30, 0.068, 0.07),
            _rounded_section(0.56, 0.27, 0.058, 0.16),
            _rounded_section(0.50, 0.22, 0.048, BASE_TOP_Z),
        ]
    )


def _build_jar_shell_geometry():
    outer = rounded_rect_profile(JAR_OUTER_W, JAR_OUTER_D, 0.028, corner_segments=10)
    inner = rounded_rect_profile(JAR_INNER_W, JAR_INNER_D, 0.022, corner_segments=10)
    lip_outer = rounded_rect_profile(JAR_OUTER_W + 0.008, JAR_OUTER_D + 0.006, 0.031, corner_segments=10)
    lip_inner = rounded_rect_profile(JAR_INNER_W, JAR_INNER_D, 0.022, corner_segments=10)

    floor = ExtrudeGeometry.from_z0(outer, JAR_FLOOR_T)
    walls = ExtrudeWithHolesGeometry(outer, [inner], JAR_WALL_H, cap=True, center=False)
    walls.translate(0.0, 0.0, JAR_FLOOR_T)

    top_lip = ExtrudeWithHolesGeometry(lip_outer, [lip_inner], 0.008, cap=True, center=False)
    top_lip.translate(0.0, 0.0, JAR_HEIGHT - 0.008)

    return floor.merge(walls).merge(top_lip)


def _build_handle_geometry(side: float):
    attach_x = side * (JAR_OUTER_W * 0.5 - 0.004)
    outer_x = side * (JAR_OUTER_W * 0.5 + 0.040)
    return tube_from_spline_points(
        [
            (attach_x, 0.0, 0.055),
            (side * (JAR_OUTER_W * 0.5 + 0.030), 0.0, 0.095),
            (outer_x, 0.0, 0.155),
            (side * (JAR_OUTER_W * 0.5 + 0.030), 0.0, 0.220),
            (attach_x, 0.0, 0.248),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )


def _add_jar_subassembly(model: ArticulatedObject, base, prefix: str, x_center: float, handle_side: float) -> None:
    jar = model.part(f"{prefix}_jar")
    jar.visual(
        mesh_from_geometry(_build_jar_shell_geometry(), f"{prefix}_jar_shell"),
        material="jar_clear",
        name="jar_shell",
    )
    jar.visual(
        mesh_from_geometry(_build_handle_geometry(handle_side), f"{prefix}_handle"),
        material="handle_dark",
        name="handle",
    )
    jar.visual(
        Cylinder(radius=0.059, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material="trim_dark",
        name="coupling_collar",
    )

    model.articulation(
        f"base_to_{prefix}_jar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jar,
        origin=Origin(xyz=(x_center, 0.0, PAD_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=0.40),
    )

    blades = model.part(f"{prefix}_blade_assembly")
    blades.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="blade_metal",
        name="hub",
    )
    blades.visual(
        Cylinder(radius=0.005, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material="blade_metal",
        name="shaft",
    )
    blades.visual(
        Box((0.055, 0.012, 0.0025)),
        origin=Origin(xyz=(0.030, 0.0, 0.017), rpy=(0.0, 0.30, 0.0)),
        material="blade_metal",
        name="blade_a",
    )
    blades.visual(
        Box((0.055, 0.012, 0.0025)),
        origin=Origin(xyz=(-0.030, 0.0, 0.019), rpy=(0.0, -0.28, math.pi)),
        material="blade_metal",
        name="blade_b",
    )
    blades.visual(
        Box((0.055, 0.012, 0.0025)),
        origin=Origin(xyz=(0.0, 0.030, 0.023), rpy=(-0.30, 0.0, math.pi / 2.0)),
        material="blade_metal",
        name="blade_c",
    )
    blades.visual(
        Box((0.055, 0.012, 0.0025)),
        origin=Origin(xyz=(0.0, -0.030, 0.021), rpy=(0.30, 0.0, -math.pi / 2.0)),
        material="blade_metal",
        name="blade_d",
    )

    model.articulation(
        f"{prefix}_jar_to_blades",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blades,
        origin=Origin(xyz=(0.0, 0.0, JAR_FLOOR_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=28.0),
    )

    lid = model.part(f"{prefix}_lid")
    lid.visual(
        Box((JAR_OUTER_W - 0.006, JAR_OUTER_D - 0.007, 0.008)),
        origin=Origin(xyz=(0.0, (JAR_OUTER_D - 0.007) * 0.5, 0.004)),
        material="lid_dark",
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.006, length=JAR_OUTER_W - 0.030),
        origin=Origin(xyz=(0.0, -0.006, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="lid_dark",
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.050, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, JAR_OUTER_D - 0.009, 0.006)),
        material="lid_dark",
        name="front_tab",
    )
    lid.visual(
        Box((0.074, 0.056, 0.010)),
        origin=Origin(xyz=(0.0, 0.083, 0.013)),
        material="trim_mid",
        name="fill_cap",
    )

    model.articulation(
        f"{prefix}_jar_to_lid",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, -JAR_OUTER_D * 0.5, JAR_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.65),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((a + b) * 0.5 for a, b in zip(lo, hi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_jar_wide_body_blender")

    model.material("base_white", rgba=(0.93, 0.93, 0.95, 1.0))
    model.material("panel_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("trim_dark", rgba=(0.20, 0.20, 0.22, 1.0))
    model.material("trim_mid", rgba=(0.34, 0.35, 0.38, 1.0))
    model.material("blade_metal", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("jar_clear", rgba=(0.84, 0.92, 0.99, 0.34))
    model.material("lid_dark", rgba=(0.10, 0.11, 0.13, 1.0))
    model.material("handle_dark", rgba=(0.18, 0.18, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_build_base_body_geometry(), "base_body"),
        material="base_white",
        name="base_body",
    )
    base.visual(
        Cylinder(radius=0.055, length=PAD_HEIGHT),
        origin=Origin(xyz=(-JAR_CENTER_X, 0.0, BASE_TOP_Z + PAD_HEIGHT * 0.5)),
        material="trim_dark",
        name="left_pad",
    )
    base.visual(
        Cylinder(radius=0.055, length=PAD_HEIGHT),
        origin=Origin(xyz=(JAR_CENTER_X, 0.0, BASE_TOP_Z + PAD_HEIGHT * 0.5)),
        material="trim_dark",
        name="right_pad",
    )
    base.visual(
        Box((0.196, 0.012, 0.072)),
        origin=Origin(xyz=(0.0, 0.146, 0.108)),
        material="panel_dark",
        name="control_panel",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(-0.056, 0.155, 0.094), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="trim_mid",
        name="left_knob",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.056, 0.155, 0.094), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="trim_mid",
        name="right_knob",
    )

    _add_jar_subassembly(model, base, "left", -JAR_CENTER_X, -1.0)
    _add_jar_subassembly(model, base, "right", JAR_CENTER_X, 1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    left_jar = object_model.get_part("left_jar")
    right_jar = object_model.get_part("right_jar")
    left_blades = object_model.get_part("left_blade_assembly")
    right_blades = object_model.get_part("right_blade_assembly")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")

    left_bayonet = object_model.get_articulation("base_to_left_jar")
    left_blade_spin = object_model.get_articulation("left_jar_to_blades")
    left_lid_hinge = object_model.get_articulation("left_jar_to_lid")

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

    ctx.expect_contact(
        left_jar,
        base,
        elem_a="coupling_collar",
        elem_b="left_pad",
        name="left jar seats on its bayonet pad",
    )
    ctx.expect_contact(
        right_jar,
        base,
        elem_a="coupling_collar",
        elem_b="right_pad",
        name="right jar seats on its bayonet pad",
    )
    ctx.expect_gap(
        right_jar,
        left_jar,
        axis="x",
        min_gap=0.08,
        name="twin jars keep a clear center gap",
    )
    ctx.expect_contact(
        left_blades,
        left_jar,
        name="left blade assembly is mounted to the jar floor",
    )
    ctx.expect_contact(
        right_blades,
        right_jar,
        name="right blade assembly is mounted to the jar floor",
    )
    ctx.expect_within(
        left_blades,
        left_jar,
        axes="xy",
        outer_elem="jar_shell",
        margin=0.012,
        name="left blades stay within the jar body footprint",
    )
    ctx.expect_within(
        right_blades,
        right_jar,
        axes="xy",
        outer_elem="jar_shell",
        margin=0.012,
        name="right blades stay within the jar body footprint",
    )
    ctx.expect_contact(
        left_lid,
        left_jar,
        elem_a="lid_panel",
        elem_b="jar_shell",
        name="left lid rests on the left jar rim",
    )
    ctx.expect_contact(
        right_lid,
        right_jar,
        elem_a="lid_panel",
        elem_b="jar_shell",
        name="right lid rests on the right jar rim",
    )
    ctx.expect_overlap(
        left_lid,
        left_jar,
        axes="xy",
        elem_a="lid_panel",
        elem_b="jar_shell",
        min_overlap=0.16,
        name="left lid covers the jar opening",
    )
    ctx.expect_overlap(
        right_lid,
        right_jar,
        axes="xy",
        elem_a="lid_panel",
        elem_b="jar_shell",
        min_overlap=0.16,
        name="right lid covers the jar opening",
    )

    left_handle_rest = _aabb_center(ctx.part_element_world_aabb(left_jar, elem="handle"))
    with ctx.pose({left_bayonet: 0.35}):
        left_handle_twisted = _aabb_center(ctx.part_element_world_aabb(left_jar, elem="handle"))
    ctx.check(
        "left bayonet joint twists the jar about vertical axis",
        left_handle_rest is not None
        and left_handle_twisted is not None
        and abs(left_handle_twisted[1] - left_handle_rest[1]) > 0.03,
        details=f"rest={left_handle_rest}, twisted={left_handle_twisted}",
    )

    blade_a_rest = _aabb_center(ctx.part_element_world_aabb(left_blades, elem="blade_a"))
    with ctx.pose({left_blade_spin: 1.0}):
        blade_a_spun = _aabb_center(ctx.part_element_world_aabb(left_blades, elem="blade_a"))
    ctx.check(
        "left blade assembly spins around the jar centerline",
        blade_a_rest is not None
        and blade_a_spun is not None
        and abs(blade_a_spun[1] - blade_a_rest[1]) > 0.015
        and abs(blade_a_spun[2] - blade_a_rest[2]) < 0.01,
        details=f"rest={blade_a_rest}, spun={blade_a_spun}",
    )

    left_tab_rest = _aabb_center(ctx.part_element_world_aabb(left_lid, elem="front_tab"))
    with ctx.pose({left_lid_hinge: 1.40}):
        left_tab_open = _aabb_center(ctx.part_element_world_aabb(left_lid, elem="front_tab"))
    ctx.check(
        "left lid flips upward from the rear hinge",
        left_tab_rest is not None
        and left_tab_open is not None
        and left_tab_open[2] > left_tab_rest[2] + 0.10
        and left_tab_open[1] < left_tab_rest[1] - 0.04,
        details=f"closed={left_tab_rest}, open={left_tab_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
