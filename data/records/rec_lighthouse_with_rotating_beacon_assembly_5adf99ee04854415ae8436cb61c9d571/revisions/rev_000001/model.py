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
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_masonry_lighthouse")

    stone = model.material("stone", rgba=(0.93, 0.92, 0.88, 1.0))
    trim = model.material("trim", rgba=(0.72, 0.70, 0.66, 1.0))
    iron = model.material("iron", rgba=(0.16, 0.18, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.74, 0.86, 0.92, 0.32))
    beacon_glass = model.material("beacon_glass", rgba=(0.95, 0.82, 0.48, 0.35))
    brass = model.material("brass", rgba=(0.72, 0.59, 0.25, 1.0))
    copper = model.material("copper", rgba=(0.32, 0.50, 0.45, 1.0))
    door_paint = model.material("door_paint", rgba=(0.33, 0.11, 0.08, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _face_origin(
        angle: float,
        apothem: float,
        *,
        radial: float = 0.0,
        tangential: float = 0.0,
        z: float = 0.0,
    ) -> Origin:
        ca = math.cos(angle)
        sa = math.sin(angle)
        x = ca * (apothem + radial) - sa * tangential
        y = sa * (apothem + radial) + ca * tangential
        return Origin(xyz=(x, y, z), rpy=(0.0, 0.0, angle))

    tower_height = 21.95
    gallery_deck_bottom = tower_height
    gallery_deck_thickness = 0.22
    gallery_deck_top = gallery_deck_bottom + gallery_deck_thickness
    lantern_base_z = gallery_deck_top
    lantern_wall_z = lantern_base_z + 0.10
    lantern_wall_height = 2.90
    lantern_top_z = lantern_wall_z + lantern_wall_height
    roof_apron_height = 0.16
    roof_apron_z = lantern_top_z + roof_apron_height * 0.5
    dome_base_z = lantern_top_z + roof_apron_height

    tower_outer_profile = [
        (3.85, 0.00),
        (3.78, 0.55),
        (3.52, 3.20),
        (3.16, 8.80),
        (2.76, 14.80),
        (2.28, 20.20),
        (2.10, tower_height),
    ]
    tower_inner_profile = [
        (2.82, 0.00),
        (2.75, 0.55),
        (2.52, 3.20),
        (2.22, 8.80),
        (1.90, 14.80),
        (1.60, 20.20),
        (1.46, tower_height),
    ]
    tower_shell_mesh = _mesh(
        "lighthouse_tower_shell",
        LatheGeometry.from_shell_profiles(
            tower_outer_profile,
            tower_inner_profile,
            segments=96,
        ),
    )
    gallery_top_rail_mesh = _mesh(
        "lighthouse_gallery_top_rail",
        TorusGeometry(
            radius=2.64,
            tube=0.032,
            radial_segments=16,
            tubular_segments=96,
        ),
    )
    gallery_mid_rail_mesh = _mesh(
        "lighthouse_gallery_mid_rail",
        TorusGeometry(
            radius=2.64,
            tube=0.022,
            radial_segments=14,
            tubular_segments=96,
        ),
    )
    dome_mesh = _mesh(
        "lighthouse_dome",
        DomeGeometry(
            radius=1.50,
            radial_segments=64,
            height_segments=22,
            closed=True,
        ),
    )
    beacon_shell_mesh = _mesh(
        "lighthouse_beacon_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.32, 0.00),
                (0.34, 0.08),
                (0.34, 0.22),
                (0.60, 0.30),
                (0.60, 1.38),
                (0.52, 1.48),
                (0.24, 1.52),
            ],
            [
                (0.18, 0.00),
                (0.18, 0.22),
                (0.46, 0.30),
                (0.46, 1.38),
                (0.20, 1.48),
                (0.18, 1.52),
            ],
            segments=72,
        ),
    )
    beacon_collar_mesh = _mesh(
        "lighthouse_beacon_collar",
        LatheGeometry.from_shell_profiles(
            [
                (0.24, 0.00),
                (0.24, 0.20),
            ],
            [
                (0.18, 0.00),
                (0.18, 0.20),
            ],
            segments=48,
        ),
    )

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=4.10, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=trim,
        name="base_plinth",
    )
    tower.visual(tower_shell_mesh, material=stone, name="tower_shell")
    tower.visual(
        Cylinder(radius=2.35, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 21.83)),
        material=trim,
        name="gallery_cornice",
    )
    tower.visual(
        Cylinder(radius=2.78, length=gallery_deck_thickness),
        origin=Origin(xyz=(0.0, 0.0, gallery_deck_bottom + gallery_deck_thickness * 0.5)),
        material=iron,
        name="gallery_deck",
    )

    post_count = 16
    railing_radius = 2.64
    post_radius = 0.028
    post_top_z = 23.18
    for index in range(post_count):
        angle = (2.0 * math.pi * index) / post_count
        tower.visual(
            Cylinder(radius=post_radius, length=post_top_z - gallery_deck_top),
            origin=Origin(
                xyz=(
                    railing_radius * math.cos(angle),
                    railing_radius * math.sin(angle),
                    (post_top_z + gallery_deck_top) * 0.5,
                )
            ),
            material=iron,
            name=f"gallery_post_{index:02d}",
        )
    tower.visual(
        gallery_top_rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, post_top_z)),
        material=iron,
        name="gallery_top_rail",
    )
    tower.visual(
        gallery_mid_rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, 22.72)),
        material=iron,
        name="gallery_mid_rail",
    )

    tower.visual(
        Cylinder(radius=1.56, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, lantern_base_z + 0.05)),
        material=iron,
        name="lantern_lower_ring",
    )

    lantern_apothem = 1.46
    side_count = 8
    side_length = 2.0 * lantern_apothem * math.tan(math.pi / side_count)
    frame_depth = 0.08
    jamb_width = 0.11
    sill_height = 0.42
    header_height = 0.30
    glass_height = lantern_wall_height - sill_height - header_height

    for face_index in range(side_count):
        angle = (2.0 * math.pi * face_index) / side_count
        if face_index == 0:
            opening_width = 0.86
            door_jamb_width = (side_length - opening_width) * 0.5
            tower.visual(
                Box((frame_depth, door_jamb_width, lantern_wall_height)),
                origin=_face_origin(
                    angle,
                    lantern_apothem,
                    tangential=-(opening_width + door_jamb_width) * 0.5,
                    z=lantern_wall_z + lantern_wall_height * 0.5,
                ),
                material=iron,
                name="door_frame_left",
            )
            tower.visual(
                Box((frame_depth, door_jamb_width, lantern_wall_height)),
                origin=_face_origin(
                    angle,
                    lantern_apothem,
                    tangential=(opening_width + door_jamb_width) * 0.5,
                    z=lantern_wall_z + lantern_wall_height * 0.5,
                ),
                material=iron,
                name="door_frame_right",
            )
            tower.visual(
                Box((frame_depth, opening_width, 0.025)),
                origin=_face_origin(
                    angle,
                    lantern_apothem,
                    z=lantern_wall_z + 0.0125,
                ),
                material=iron,
                name="door_threshold",
            )
            tower.visual(
                Box((frame_depth, opening_width, 0.10)),
                origin=_face_origin(
                    angle,
                    lantern_apothem,
                    z=lantern_wall_z + 2.10,
                ),
                material=iron,
                name="door_transom_bar",
            )
            tower.visual(
                Box((frame_depth, opening_width, 0.22)),
                origin=_face_origin(
                    angle,
                    lantern_apothem,
                    z=lantern_wall_z + lantern_wall_height - 0.11,
                ),
                material=iron,
                name="door_header",
            )
            tower.visual(
                Box((0.018, opening_width - 0.06, 0.65)),
                origin=_face_origin(
                    angle,
                    lantern_apothem,
                    radial=-0.012,
                    z=lantern_wall_z + 2.52,
                ),
                material=glass,
                name="door_transom_glass",
            )
            continue

        tower.visual(
            Box((frame_depth, jamb_width, lantern_wall_height)),
            origin=_face_origin(
                angle,
                lantern_apothem,
                tangential=-(side_length - jamb_width) * 0.5,
                z=lantern_wall_z + lantern_wall_height * 0.5,
            ),
            material=iron,
            name=f"lantern_post_left_{face_index}",
        )
        tower.visual(
            Box((frame_depth, jamb_width, lantern_wall_height)),
            origin=_face_origin(
                angle,
                lantern_apothem,
                tangential=(side_length - jamb_width) * 0.5,
                z=lantern_wall_z + lantern_wall_height * 0.5,
            ),
            material=iron,
            name=f"lantern_post_right_{face_index}",
        )
        tower.visual(
            Box((frame_depth, side_length - 2.0 * jamb_width, sill_height)),
            origin=_face_origin(
                angle,
                lantern_apothem,
                z=lantern_wall_z + sill_height * 0.5,
            ),
            material=iron,
            name=f"lantern_sill_{face_index}",
        )
        tower.visual(
            Box((frame_depth, side_length - 2.0 * jamb_width, header_height)),
            origin=_face_origin(
                angle,
                lantern_apothem,
                z=lantern_wall_z + lantern_wall_height - header_height * 0.5,
            ),
            material=iron,
            name=f"lantern_header_{face_index}",
        )
        tower.visual(
            Box((0.018, side_length - 2.0 * jamb_width - 0.03, glass_height)),
            origin=_face_origin(
                angle,
                lantern_apothem,
                radial=-0.012,
                z=lantern_wall_z + sill_height + glass_height * 0.5,
            ),
            material=glass,
            name=f"lantern_glass_{face_index}",
        )

    tower.visual(
        Cylinder(radius=1.58, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, lantern_top_z + 0.08)),
        material=iron,
        name="lantern_upper_ring",
    )
    tower.visual(
        Cylinder(radius=1.64, length=roof_apron_height),
        origin=Origin(xyz=(0.0, 0.0, roof_apron_z)),
        material=iron,
        name="roof_apron",
    )
    tower.visual(
        dome_mesh,
        origin=Origin(xyz=(0.0, 0.0, dome_base_z)),
        material=copper,
        name="roof_dome",
    )
    tower.visual(
        Cylinder(radius=0.07, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, dome_base_z + 1.67)),
        material=iron,
        name="roof_finial_stem",
    )
    tower.visual(
        Cylinder(radius=0.13, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, dome_base_z + 1.92)),
        material=iron,
        name="roof_vent_cap",
    )

    tower.visual(
        Cylinder(radius=0.50, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, 22.55)),
        material=iron,
        name="pedestal_base",
    )
    tower.visual(
        Cylinder(radius=0.36, length=0.21),
        origin=Origin(xyz=(0.0, 0.0, 23.035)),
        material=iron,
        name="pedestal_column",
    )
    tower.visual(
        Cylinder(radius=0.34, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 23.18)),
        material=brass,
        name="pedestal_shoulder",
    )
    tower.visual(
        Cylinder(radius=0.16, length=1.64),
        origin=Origin(xyz=(0.0, 0.0, 24.00)),
        material=brass,
        name="pedestal_spindle",
    )
    tower.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 24.80)),
        material=brass,
        name="pedestal_clip",
    )
    tower.inertial = Inertial.from_geometry(
        Box((8.2, 8.2, 27.7)),
        mass=98000.0,
        origin=Origin(xyz=(0.0, 0.0, 13.85)),
    )

    service_door = model.part("service_door")
    door_width = 0.78
    door_height = 2.02
    door_thickness = 0.045
    stile_width = 0.06
    rail_height = 0.10
    mid_rail_height = 0.10
    lower_panel_height = 0.82
    service_door.visual(
        Box((door_thickness, stile_width, door_height)),
        origin=Origin(xyz=(0.0, stile_width * 0.5, door_height * 0.5)),
        material=door_paint,
        name="hinge_stile",
    )
    service_door.visual(
        Box((door_thickness, stile_width, door_height)),
        origin=Origin(xyz=(0.0, door_width - stile_width * 0.5, door_height * 0.5)),
        material=door_paint,
        name="latch_stile",
    )
    service_door.visual(
        Box((door_thickness, door_width, rail_height)),
        origin=Origin(xyz=(0.0, door_width * 0.5, rail_height * 0.5)),
        material=door_paint,
        name="bottom_rail",
    )
    service_door.visual(
        Box((door_thickness, door_width, rail_height)),
        origin=Origin(xyz=(0.0, door_width * 0.5, door_height - rail_height * 0.5)),
        material=door_paint,
        name="top_rail",
    )
    service_door.visual(
        Box((door_thickness, door_width - 2.0 * stile_width, mid_rail_height)),
        origin=Origin(xyz=(0.0, door_width * 0.5, 0.92)),
        material=door_paint,
        name="mid_rail",
    )
    service_door.visual(
        Box((door_thickness * 0.78, door_width - 2.0 * stile_width - 0.04, lower_panel_height)),
        origin=Origin(xyz=(0.0, door_width * 0.5, rail_height + lower_panel_height * 0.5)),
        material=door_paint,
        name="lower_panel",
    )
    service_door.visual(
        Box((0.020, door_width - 2.0 * stile_width + 0.002, 0.952)),
        origin=Origin(xyz=(0.0, door_width * 0.5, 1.446)),
        material=glass,
        name="upper_glass",
    )
    service_door.visual(
        Cylinder(radius=0.022, length=0.03),
        origin=Origin(
            xyz=(door_thickness * 0.5 + 0.007, door_width - 0.07, 1.02),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="door_handle",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((door_thickness, door_width, door_height)),
        mass=42.0,
        origin=Origin(xyz=(0.0, door_width * 0.5, door_height * 0.5)),
    )

    beacon_drum = model.part("beacon_drum")
    beacon_drum.visual(
        beacon_shell_mesh,
        material=beacon_glass,
        name="beacon_drum_shell",
    )
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        beacon_drum.visual(
            Box((0.08, 0.10, 1.05)),
            origin=Origin(
                xyz=(0.0, 0.53, 0.88),
                rpy=(0.0, 0.0, angle),
            ),
            material=brass,
            name=f"beacon_frame_{index}",
        )
        beacon_drum.visual(
            Box((0.08, 0.36, 0.07)),
            origin=Origin(
                xyz=(0.0, 0.47, 0.34),
                rpy=(0.0, 0.0, angle),
            ),
            material=brass,
            name=f"beacon_lower_band_{index}",
        )
        beacon_drum.visual(
            Box((0.08, 0.30, 0.06)),
            origin=Origin(
                xyz=(0.0, 0.46, 1.43),
                rpy=(0.0, 0.0, angle),
            ),
            material=brass,
            name=f"beacon_upper_band_{index}",
        )
    beacon_drum.visual(
        Box((0.12, 0.22, 0.16)),
        origin=Origin(xyz=(0.49, 0.0, 0.54)),
        material=iron,
        name="drive_box",
    )
    beacon_drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.60, length=1.52),
        mass=360.0,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=service_door,
        origin=_face_origin(
            0.0,
            lantern_apothem,
            tangential=-door_width * 0.5,
            z=lantern_wall_z + 0.02,
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.3,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "beacon_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon_drum,
        origin=Origin(xyz=(0.0, 0.0, 23.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    service_door = object_model.get_part("service_door")
    beacon_drum = object_model.get_part("beacon_drum")
    door_hinge = object_model.get_articulation("door_hinge")
    beacon_spin = object_model.get_articulation("beacon_spin")

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
        "door_hinge_vertical_axis",
        abs(door_hinge.axis[0]) < 1e-9
        and abs(door_hinge.axis[1]) < 1e-9
        and abs(abs(door_hinge.axis[2]) - 1.0) < 1e-9,
        details=f"door hinge axis should be vertical, got {door_hinge.axis}",
    )
    ctx.check(
        "beacon_joint_is_continuous_vertical",
        beacon_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(beacon_spin.axis[0]) < 1e-9
        and abs(beacon_spin.axis[1]) < 1e-9
        and abs(beacon_spin.axis[2] - 1.0) < 1e-9,
        details=(
            "beacon should spin continuously about +Z, "
            f"got type={beacon_spin.articulation_type} axis={beacon_spin.axis}"
        ),
    )

    ctx.expect_contact(
        beacon_drum,
        tower,
        elem_a="beacon_drum_shell",
        elem_b="pedestal_shoulder",
        name="beacon_bears_on_pedestal_shoulder",
    )
    ctx.expect_gap(
        tower,
        beacon_drum,
        axis="z",
        positive_elem="pedestal_clip",
        negative_elem="beacon_drum_shell",
        min_gap=0.01,
        max_gap=0.05,
        name="beacon_top_clip_capture_gap",
    )

    door_closed_aabb = ctx.part_world_aabb(service_door)
    if door_closed_aabb is None:
        ctx.fail("door_closed_aabb_available", "service door AABB is unavailable in rest pose")
    else:
        with ctx.pose({door_hinge: math.radians(78.0), beacon_spin: 1.7}):
            ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_open_door_pose")
            ctx.expect_contact(
                beacon_drum,
                tower,
                elem_a="beacon_drum_shell",
                elem_b="pedestal_shoulder",
                name="beacon_stays_seated_while_rotating",
            )
            door_open_aabb = ctx.part_world_aabb(service_door)
            if door_open_aabb is None:
                ctx.fail("door_open_aabb_available", "service door AABB is unavailable in open pose")
            else:
                ctx.check(
                    "door_swings_outward_from_lantern",
                    door_open_aabb[1][0] > door_closed_aabb[1][0] + 0.18,
                    details=(
                        "door did not swing outward enough: "
                        f"closed max x={door_closed_aabb[1][0]:.3f}, "
                        f"open max x={door_open_aabb[1][0]:.3f}"
                    ),
                )
                ctx.check(
                    "door_rotates_about_side_hinge",
                    door_open_aabb[1][1] < door_closed_aabb[1][1] - 0.15,
                    details=(
                        "door did not pivot around its side hinge as expected: "
                        f"closed max y={door_closed_aabb[1][1]:.3f}, "
                        f"open max y={door_open_aabb[1][1]:.3f}"
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
