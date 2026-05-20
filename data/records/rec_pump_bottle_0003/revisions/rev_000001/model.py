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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _xy_section(width: float, depth: float, z: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _yz_section(
    *,
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for z, y in rounded_rect_profile(height_z, width_y, radius, corner_segments=8)
    ]


def _build_body_shell_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _xy_section(0.092, 0.092, 0.000, 0.012),
                _xy_section(0.096, 0.096, 0.010, 0.013),
                _xy_section(0.096, 0.096, 0.108, 0.013),
                _xy_section(0.086, 0.086, 0.128, 0.016),
                _xy_section(0.060, 0.060, 0.150, 0.014),
            ]
        )
    )


def _build_neck_finish_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0145, 0.000),
            (0.0152, 0.002),
            (0.0152, 0.006),
        ],
        [
            (0.0080, 0.000),
            (0.0080, 0.006),
        ],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def _build_collar_ring_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0290, 0.000),
            (0.0290, 0.006),
            (0.0275, 0.018),
        ],
        [
            (0.0080, 0.000),
            (0.0080, 0.018),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_nozzle_shell_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _yz_section(x=-0.018, width_y=0.038, height_z=0.011, radius=0.0038, z_center=0.013),
                _yz_section(x=0.006, width_y=0.056, height_z=0.014, radius=0.0048, z_center=0.014),
                _yz_section(x=0.028, width_y=0.048, height_z=0.012, radius=0.0042, z_center=0.013),
            ]
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lotion_pump_bottle", assets=ASSETS)

    bottle_plastic = model.material("bottle_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    collar_metal = model.material("collar_metal", rgba=(0.82, 0.83, 0.84, 1.0))
    pump_white = model.material("pump_white", rgba=(0.97, 0.97, 0.96, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        _save_mesh(_build_body_shell_mesh(), "body_shell.obj"),
        material=bottle_plastic,
        name="body_shell",
    )
    bottle_body.visual(
        _save_mesh(_build_neck_finish_mesh(), "neck_finish.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=bottle_plastic,
        name="neck_finish",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Box((0.096, 0.096, 0.156)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
    )

    collar = model.part("collar")
    collar.visual(
        _save_mesh(_build_collar_ring_mesh(), "collar_ring.obj"),
        material=collar_metal,
        name="collar_ring",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.029, length=0.018),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=pump_white,
        name="guide_sleeve",
    )
    plunger.visual(
        Cylinder(radius=0.0048, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=pump_white,
        name="actuator_stem",
    )
    plunger.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=pump_white,
        name="stem_flange",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.022, 0.022, 0.050)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    nozzle_head = model.part("nozzle_head")
    nozzle_head.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=pump_white,
        name="nozzle_hub",
    )
    nozzle_head.visual(
        _save_mesh(_build_nozzle_shell_mesh(), "nozzle_shell.obj"),
        material=pump_white,
        name="nozzle_shell",
    )
    nozzle_head.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.035, 0.0, 0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_white,
        name="spout",
    )
    nozzle_head.visual(
        Cylinder(radius=0.0022, length=0.006),
        origin=Origin(xyz=(0.046, 0.0, 0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_white,
        name="outlet_tip",
    )
    nozzle_head.inertial = Inertial.from_geometry(
        Box((0.070, 0.058, 0.022)),
        mass=0.07,
        origin=Origin(xyz=(0.016, 0.0, 0.011)),
    )

    model.articulation(
        "body_to_collar",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
    )
    model.articulation(
        "collar_to_plunger",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=0.0,
            upper=0.008,
        ),
    )
    model.articulation(
        "plunger_to_nozzle",
        ArticulationType.CONTINUOUS,
        parent=plunger,
        child=nozzle_head,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottle_body = object_model.get_part("bottle_body")
    collar = object_model.get_part("collar")
    plunger = object_model.get_part("plunger")
    nozzle_head = object_model.get_part("nozzle_head")

    neck_finish = bottle_body.get_visual("neck_finish")
    body_shell = bottle_body.get_visual("body_shell")
    collar_ring = collar.get_visual("collar_ring")
    guide_sleeve = plunger.get_visual("guide_sleeve")
    actuator_stem = plunger.get_visual("actuator_stem")
    stem_flange = plunger.get_visual("stem_flange")
    nozzle_hub = nozzle_head.get_visual("nozzle_hub")

    collar_to_plunger = object_model.get_articulation("collar_to_plunger")
    plunger_to_nozzle = object_model.get_articulation("plunger_to_nozzle")

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
    ctx.allow_overlap(
        collar,
        plunger,
        elem_a=collar_ring,
        elem_b=guide_sleeve,
        reason="The plunger's lower guide sleeve intentionally nests inside the collar as a telescoping pump guide.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        collar,
        bottle_body,
        elem_a=collar_ring,
        elem_b=neck_finish,
        name="collar_seats_on_bottle_neck",
    )
    ctx.expect_contact(
        plunger,
        collar,
        elem_a=guide_sleeve,
        elem_b=collar_ring,
        name="plunger_is_guided_by_collar_bore",
    )
    ctx.expect_contact(
        nozzle_head,
        plunger,
        elem_a=nozzle_hub,
        elem_b=stem_flange,
        name="nozzle_hub_sits_on_plunger_flange",
    )
    ctx.expect_overlap(
        collar,
        bottle_body,
        axes="xy",
        min_overlap=0.028,
        name="pump_assembly_is_centered_on_square_bottle",
    )
    ctx.expect_gap(
        nozzle_head,
        collar,
        axis="z",
        min_gap=0.025,
        max_gap=0.040,
        positive_elem=nozzle_hub,
        negative_elem=collar_ring,
        name="nozzle_head_sits_above_collar",
    )
    ctx.expect_overlap(
        nozzle_head,
        plunger,
        axes="xy",
        min_overlap=0.020,
        elem_a=nozzle_hub,
        elem_b=stem_flange,
        name="nozzle_rotation_axis_matches_plunger_axis",
    )

    bottle_aabb = ctx.part_element_world_aabb(bottle_body, elem=body_shell)
    collar_aabb = ctx.part_world_aabb(collar)
    plunger_aabb = ctx.part_element_world_aabb(plunger, elem=actuator_stem)
    nozzle_aabb = ctx.part_world_aabb(nozzle_head)
    bottle_pos = ctx.part_world_position(bottle_body)
    plunger_rest_pos = ctx.part_world_position(plunger)
    nozzle_rest_pos = ctx.part_world_position(nozzle_head)

    if bottle_aabb is None or collar_aabb is None or plunger_aabb is None or nozzle_aabb is None:
        ctx.fail("world_aabbs_exist", "Expected all primary parts to provide measurable world-space AABBs.")
        return ctx.report()
    if bottle_pos is None or plunger_rest_pos is None or nozzle_rest_pos is None:
        ctx.fail("world_positions_exist", "Expected all moving parts to provide measurable world positions.")
        return ctx.report()

    bottle_dx = bottle_aabb[1][0] - bottle_aabb[0][0]
    bottle_dy = bottle_aabb[1][1] - bottle_aabb[0][1]
    bottle_dz = bottle_aabb[1][2] - bottle_aabb[0][2]
    nozzle_dx = nozzle_aabb[1][0] - nozzle_aabb[0][0]
    nozzle_dy = nozzle_aabb[1][1] - nozzle_aabb[0][1]
    plunger_dx = plunger_aabb[1][0] - plunger_aabb[0][0]
    collar_dz = collar_aabb[1][2] - collar_aabb[0][2]

    ctx.check(
        "bottle_reads_as_square_shouldered_container",
        0.092 <= bottle_dx <= 0.100
        and 0.092 <= bottle_dy <= 0.100
        and abs(bottle_dx - bottle_dy) <= 0.004
        and 0.165 <= bottle_dz <= 0.176,
        details=(
            f"Expected near-square bottle footprint and shoulder height; "
            f"got dx={bottle_dx:.4f}, dy={bottle_dy:.4f}, dz={bottle_dz:.4f}."
        ),
    )
    ctx.check(
        "pump_head_is_broad_relative_to_stem",
        nozzle_dy >= 3.5 * plunger_dx,
        details=(
            f"Expected a broad head over a slim plunger; "
            f"got nozzle_y={nozzle_dy:.4f}, plunger_x={plunger_dx:.4f}."
        ),
    )
    ctx.check(
        "collar_is_a_short_neck_band",
        0.016 <= collar_dz <= 0.020,
        details=f"Expected a short pump collar height, got {collar_dz:.4f}.",
    )
    ctx.check(
        "bottle_body_stays_rooted_at_world_origin",
        abs(bottle_pos[0]) <= 1e-6 and abs(bottle_pos[1]) <= 1e-6 and abs(bottle_pos[2]) <= 1e-6,
        details=f"Expected fixed bottle body at origin, got position {bottle_pos}.",
    )

    with ctx.pose({collar_to_plunger: 0.008}):
        plunger_pressed_pos = ctx.part_world_position(plunger)
        nozzle_pressed_pos = ctx.part_world_position(nozzle_head)
        if plunger_pressed_pos is None or nozzle_pressed_pos is None:
            ctx.fail("pressed_pose_positions_exist", "Expected pressed-pose moving-part positions.")
        else:
            ctx.check(
                "plunger_translates_axially_into_collar",
                plunger_pressed_pos[2] < plunger_rest_pos[2] - 0.007,
                details=(
                    f"Expected clear downward plunger travel; "
                    f"rest={plunger_rest_pos[2]:.4f}, pressed={plunger_pressed_pos[2]:.4f}."
                ),
            )
            ctx.check(
                "nozzle_tracks_plunger_press",
                nozzle_pressed_pos[2] < nozzle_rest_pos[2] - 0.007,
                details=(
                    f"Expected nozzle to ride down with plunger; "
                    f"rest={nozzle_rest_pos[2]:.4f}, pressed={nozzle_pressed_pos[2]:.4f}."
                ),
            )
        ctx.expect_contact(
            plunger,
            collar,
            elem_a=guide_sleeve,
            elem_b=collar_ring,
            name="plunger_remains_guided_when_pressed",
        )
        ctx.expect_gap(
            nozzle_head,
            collar,
            axis="z",
            min_gap=0.017,
            max_gap=0.032,
            positive_elem=nozzle_hub,
            negative_elem=collar_ring,
            name="pressed_nozzle_remains_above_collar",
        )

    rest_nozzle_aabb = nozzle_aabb
    with ctx.pose({plunger_to_nozzle: math.pi / 2.0}):
        turned_nozzle_aabb = ctx.part_world_aabb(nozzle_head)
        turned_nozzle_pos = ctx.part_world_position(nozzle_head)
        if turned_nozzle_aabb is None or turned_nozzle_pos is None:
            ctx.fail("turned_pose_measures_exist", "Expected measurable nozzle bounds in rotated pose.")
        else:
            rest_x_span = rest_nozzle_aabb[1][0] - rest_nozzle_aabb[0][0]
            rest_y_span = rest_nozzle_aabb[1][1] - rest_nozzle_aabb[0][1]
            turned_x_span = turned_nozzle_aabb[1][0] - turned_nozzle_aabb[0][0]
            turned_y_span = turned_nozzle_aabb[1][1] - turned_nozzle_aabb[0][1]
            ctx.check(
                "nozzle_rotation_preserves_stem_center",
                abs(turned_nozzle_pos[0] - nozzle_rest_pos[0]) <= 1e-6
                and abs(turned_nozzle_pos[1] - nozzle_rest_pos[1]) <= 1e-6
                and abs(turned_nozzle_pos[2] - nozzle_rest_pos[2]) <= 1e-6,
                details=(
                    f"Expected nozzle origin to stay on the same stem axis; "
                    f"rest={nozzle_rest_pos}, turned={turned_nozzle_pos}."
                ),
            )
            ctx.check(
                "nozzle_spout_swings_around_vertical_axis",
                rest_x_span > rest_y_span and turned_y_span > turned_x_span,
                details=(
                    f"Expected spout-dominant axis to swap after quarter-turn; "
                    f"rest spans=({rest_x_span:.4f}, {rest_y_span:.4f}), "
                    f"turned spans=({turned_x_span:.4f}, {turned_y_span:.4f})."
                ),
            )
        ctx.expect_contact(
            nozzle_head,
            plunger,
            elem_a=nozzle_hub,
            elem_b=stem_flange,
            name="nozzle_stays_seated_while_rotating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
