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
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_loop(
    width: float,
    height: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    cx, cy = center
    points = [
        (cx - width * 0.5, cy - height * 0.5),
        (cx + width * 0.5, cy - height * 0.5),
        (cx + width * 0.5, cy + height * 0.5),
        (cx - width * 0.5, cy + height * 0.5),
    ]
    return list(reversed(points)) if clockwise else points


def _ring_shell(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    segments: int = 56,
):
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_canister_missile_launcher")

    support_gray = model.material("support_gray", rgba=(0.32, 0.34, 0.36, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    launcher_olive = model.material("launcher_olive", rgba=(0.34, 0.39, 0.24, 1.0))
    bolt_gray = model.material("bolt_gray", rgba=(0.46, 0.48, 0.50, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.06, 0.07, 0.08, 1.0))

    canister_outer = 0.86
    canister_length = 1.50
    tube_clear = 0.35
    divider = 0.06
    cell_offset = (tube_clear * 0.5) + (divider * 0.5)

    pack_body_mesh = _mesh(
        "pack_body",
        ExtrudeWithHolesGeometry(
            _rect_loop(canister_outer, canister_outer),
            [
                _rect_loop(tube_clear, tube_clear, center=(cell_offset, cell_offset), clockwise=True),
                _rect_loop(tube_clear, tube_clear, center=(cell_offset, -cell_offset), clockwise=True),
                _rect_loop(tube_clear, tube_clear, center=(-cell_offset, cell_offset), clockwise=True),
                _rect_loop(tube_clear, tube_clear, center=(-cell_offset, -cell_offset), clockwise=True),
            ],
            canister_length,
            cap=True,
            center=True,
            closed=True,
        ),
    )
    base_ring_mesh = _mesh(
        "azimuth_ring",
        _ring_shell(inner_radius=0.24, outer_radius=0.34, length=0.10),
    )
    bearing_shell_mesh = _mesh(
        "trunnion_bearing",
        _ring_shell(inner_radius=0.075, outer_radius=0.11, length=0.10),
    )

    support = model.part("support_column")
    support.visual(
        Box((0.78, 0.78, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=support_gray,
        name="foundation_block",
    )
    support.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=steel_dark,
        name="column_foot",
    )
    support.visual(
        Cylinder(radius=0.16, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=support_gray,
        name="main_column",
    )
    support.visual(
        Cylinder(radius=0.22, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
        material=steel_dark,
        name="bearing_collar",
    )
    support.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        material=bolt_gray,
        name="top_bearing_plate",
    )

    turret = model.part("turret_yoke")
    turret.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=bolt_gray,
        name="azimuth_hub",
    )
    turret.visual(
        base_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=steel_dark,
        name="azimuth_ring",
    )
    turret.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=steel_dark,
        name="turntable_deck",
    )
    turret.visual(
        Box((0.44, 0.56, 0.06)),
        origin=Origin(xyz=(-0.10, 0.0, 0.16)),
        material=support_gray,
        name="mount_plate",
    )
    turret.visual(
        Cylinder(radius=0.14, length=0.40),
        origin=Origin(xyz=(-0.18, 0.0, 0.38)),
        material=support_gray,
        name="pedestal_stem",
    )
    turret.visual(
        Box((0.30, 0.96, 0.16)),
        origin=Origin(xyz=(-0.24, 0.0, 0.30)),
        material=support_gray,
        name="lower_cradle_beam",
    )
    turret.visual(
        Box((0.24, 0.64, 0.22)),
        origin=Origin(xyz=(-0.18, 0.0, 0.54)),
        material=support_gray,
        name="center_saddle_block",
    )
    turret.visual(
        Box((0.34, 0.10, 0.72)),
        origin=Origin(xyz=(-0.24, 0.51, 0.56)),
        material=support_gray,
        name="left_yoke_arm",
    )
    turret.visual(
        Box((0.34, 0.10, 0.72)),
        origin=Origin(xyz=(-0.24, -0.51, 0.56)),
        material=support_gray,
        name="right_yoke_arm",
    )
    turret.visual(
        Box((0.22, 0.94, 0.10)),
        origin=Origin(xyz=(-0.20, 0.0, 0.87)),
        material=support_gray,
        name="top_yoke_tie",
    )
    turret.visual(
        bearing_shell_mesh,
        origin=Origin(xyz=(0.0, 0.51, 0.82), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="left_bearing",
    )
    turret.visual(
        bearing_shell_mesh,
        origin=Origin(xyz=(0.0, -0.51, 0.82), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="right_bearing",
    )
    for index, y_pos in enumerate((-0.22, 0.22)):
        turret.visual(
            Cylinder(radius=0.022, length=0.030),
            origin=Origin(xyz=(-0.02, y_pos, 0.16)),
            material=bolt_gray,
            name=f"mount_bolt_front_{index}",
        )
        turret.visual(
            Cylinder(radius=0.022, length=0.030),
            origin=Origin(xyz=(-0.18, y_pos, 0.16)),
            material=bolt_gray,
            name=f"mount_bolt_rear_{index}",
        )

    pack = model.part("canister_pack")
    pack.visual(
        pack_body_mesh,
        origin=Origin(xyz=(0.95, 0.0, 0.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=launcher_olive,
        name="pack_body",
    )
    pack.visual(
        Box((0.04, 0.86, 0.86)),
        origin=Origin(xyz=(0.18, 0.0, 0.20)),
        material=launcher_olive,
        name="rear_bulkhead",
    )
    pack.visual(
        Box((0.08, 0.92, 0.92)),
        origin=Origin(xyz=(1.74, 0.0, 0.20)),
        material=launcher_olive,
        name="front_bezel",
    )
    pack.visual(
        Cylinder(radius=0.055, length=0.18),
        origin=Origin(xyz=(0.0, 0.51, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="left_trunnion",
    )
    pack.visual(
        Cylinder(radius=0.055, length=0.18),
        origin=Origin(xyz=(0.0, -0.51, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="right_trunnion",
    )
    pack.visual(
        Cylinder(radius=0.095, length=0.04),
        origin=Origin(xyz=(0.0, 0.58, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="left_retainer_collar",
    )
    pack.visual(
        Cylinder(radius=0.095, length=0.04),
        origin=Origin(xyz=(0.0, -0.58, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="right_retainer_collar",
    )
    pack.visual(
        Box((0.24, 0.14, 0.26)),
        origin=Origin(xyz=(0.06, 0.39, 0.0)),
        material=launcher_olive,
        name="left_trunnion_pad",
    )
    pack.visual(
        Box((0.24, 0.14, 0.26)),
        origin=Origin(xyz=(0.06, -0.39, 0.0)),
        material=launcher_olive,
        name="right_trunnion_pad",
    )
    pack.visual(
        Box((0.26, 0.08, 0.28)),
        origin=Origin(xyz=(0.31, 0.39, 0.0)),
        material=launcher_olive,
        name="left_side_brace",
    )
    pack.visual(
        Box((0.26, 0.08, 0.28)),
        origin=Origin(xyz=(0.31, -0.39, 0.0)),
        material=launcher_olive,
        name="right_side_brace",
    )
    for row, z_pos in enumerate((0.405, -0.005)):
        for col, y_pos in enumerate((-0.205, 0.205)):
            pack.visual(
                Box((0.06, 0.31, 0.31)),
                origin=Origin(xyz=(1.70, y_pos, z_pos)),
                material=shadow_black,
                name=f"tube_cap_{row}_{col}",
            )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.8),
    )
    model.articulation(
        "elevation_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=pack,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.6,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_column")
    turret = object_model.get_part("turret_yoke")
    pack = object_model.get_part("canister_pack")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_pitch")

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

    ctx.check("support_column_present", support.name == "support_column")
    ctx.check("turret_yoke_present", turret.name == "turret_yoke")
    ctx.check("canister_pack_present", pack.name == "canister_pack")
    ctx.check(
        "azimuth_axis_vertical",
        tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical azimuth axis, got {azimuth.axis!r}",
    )
    ctx.check(
        "elevation_axis_horizontal",
        tuple(elevation.axis) == (0.0, 1.0, 0.0),
        details=f"expected horizontal elevation axis, got {elevation.axis!r}",
    )

    ctx.expect_origin_distance(
        turret,
        support,
        axes="xy",
        max_dist=0.001,
        name="turret_stays_centered_on_column",
    )
    ctx.expect_contact(
        turret,
        support,
        elem_a="azimuth_hub",
        elem_b="top_bearing_plate",
        name="azimuth_hub_seated_on_support_plate",
    )

    with ctx.pose({elevation: 0.0}):
        ctx.expect_contact(
            pack,
            turret,
            elem_a="left_retainer_collar",
            elem_b="left_bearing",
            name="left_trunnion_supported_in_level_pose",
        )
        ctx.expect_contact(
            pack,
            turret,
            elem_a="right_retainer_collar",
            elem_b="right_bearing",
            name="right_trunnion_supported_in_level_pose",
        )
        ctx.expect_gap(
            pack,
            turret,
            axis="z",
            positive_elem="pack_body",
            negative_elem="lower_cradle_beam",
            min_gap=0.14,
            name="pack_clears_lower_cradle_beam",
        )
        ctx.expect_overlap(
            pack,
            turret,
            axes="y",
            elem_a="pack_body",
            elem_b="top_yoke_tie",
            min_overlap=0.75,
            name="pack_tracks_between_yoke_arms",
        )

    with ctx.pose({elevation: 0.85}):
        ctx.expect_contact(
            pack,
            turret,
            elem_a="left_retainer_collar",
            elem_b="left_bearing",
            name="left_trunnion_retained_when_elevated",
        )
        ctx.expect_contact(
            pack,
            turret,
            elem_a="right_retainer_collar",
            elem_b="right_bearing",
            name="right_trunnion_retained_when_elevated",
        )
        ctx.expect_gap(
            pack,
            turret,
            axis="x",
            positive_elem="pack_body",
            negative_elem="center_saddle_block",
            min_gap=0.018,
            name="elevated_pack_stays_forward_of_center_saddle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
