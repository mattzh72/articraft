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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _polar(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _rotated_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x - s * y, s * x + c * y)


def _tangent_yaw(angle: float) -> float:
    return angle + (math.pi * 0.5)


def _build_push_rail_mesh():
    rail_path = [
        (0.32, 0.055, 0.94),
        (0.52, 0.082, 0.94),
        (0.76, 0.095, 0.94),
        (1.00, 0.082, 0.94),
        (1.18, 0.055, 0.94),
    ]
    rail_geom = tube_from_spline_points(
        rail_path,
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(rail_geom, "revolving_door_push_rail")


def _add_arc_glazing(
    part,
    *,
    prefix: str,
    radius: float,
    start_angle: float,
    end_angle: float,
    pane_count: int,
    pane_width: float,
    pane_thickness: float,
    pane_height: float,
    bottom_z: float,
    glass_material,
) -> None:
    if pane_count == 1:
        angles = [(start_angle + end_angle) * 0.5]
    else:
        angles = [
            start_angle + (end_angle - start_angle) * i / (pane_count - 1)
            for i in range(pane_count)
        ]

    for i, angle in enumerate(angles):
        x, y = _polar(radius, angle)
        part.visual(
            Box((pane_width, pane_thickness, pane_height)),
            origin=Origin(
                xyz=(x, y, bottom_z + (pane_height * 0.5)),
                rpy=(0.0, 0.0, _tangent_yaw(angle)),
            ),
            material=glass_material,
            name=f"{prefix}_pane_{i}",
        )


def _add_radial_wing(
    part,
    *,
    index: int,
    angle: float,
    panel_length: float,
    panel_thickness: float,
    panel_height: float,
    panel_bottom_z: float,
    post_radius: float,
    glass_material,
    frame_material,
    rail_material,
    rail_mesh,
) -> None:
    panel_center_x = post_radius + (panel_length * 0.5) - 0.01
    bottom_rail_height = 0.09
    top_rail_height = 0.08
    stile_width = 0.055
    panel_center_y = 0.0
    panel_x, panel_y = _rotated_xy(panel_center_x, panel_center_y, angle)
    stile_x, stile_y = _rotated_xy(
        post_radius + panel_length - (stile_width * 0.5), 0.0, angle
    )

    part.visual(
        Box((panel_length, panel_thickness, panel_height)),
        origin=Origin(
            xyz=(panel_x, panel_y, panel_bottom_z + (panel_height * 0.5)),
            rpy=(0.0, 0.0, angle),
        ),
        material=glass_material,
        name=f"panel_{index}",
    )
    part.visual(
        Box((panel_length, 0.055, bottom_rail_height)),
        origin=Origin(
            xyz=(panel_x, panel_y, panel_bottom_z + (bottom_rail_height * 0.5)),
            rpy=(0.0, 0.0, angle),
        ),
        material=frame_material,
        name=f"panel_{index}_bottom_rail",
    )
    part.visual(
        Box((panel_length, 0.055, top_rail_height)),
        origin=Origin(
            xyz=(
                panel_x,
                panel_y,
                panel_bottom_z + panel_height - (top_rail_height * 0.5),
            ),
            rpy=(0.0, 0.0, angle),
        ),
        material=frame_material,
        name=f"panel_{index}_top_rail",
    )
    part.visual(
        Box((stile_width, 0.060, panel_height)),
        origin=Origin(
            xyz=(stile_x, stile_y, panel_bottom_z + (panel_height * 0.5)),
            rpy=(0.0, 0.0, angle),
        ),
        material=frame_material,
        name=f"panel_{index}_outer_stile",
    )

    part.visual(
        rail_mesh,
        origin=Origin(rpy=(0.0, 0.0, angle)),
        material=rail_material,
        name=f"rail_{index}",
    )

    standoff_xs = (0.44, 0.77, 1.02)
    for standoff_i, x_local in enumerate(standoff_xs):
        x_world, y_world = _rotated_xy(x_local, 0.050, angle)
        part.visual(
            Cylinder(radius=0.011, length=0.100),
            origin=Origin(
                xyz=(x_world, y_world, 0.94),
                rpy=(math.pi * 0.5, 0.0, angle),
            ),
            material=frame_material,
            name=f"panel_{index}_standoff_{standoff_i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wing_revolving_door")

    bronze = model.material("bronze", rgba=(0.30, 0.26, 0.20, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    glass = model.material("glass", rgba=(0.68, 0.84, 0.92, 0.28))
    dark_floor = model.material("dark_floor", rgba=(0.24, 0.24, 0.25, 1.0))
    canopy_dark = model.material("canopy_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    drum_radius = 1.44
    canopy_radius = 1.62
    floor_radius = 1.58
    floor_thickness = 0.05
    canopy_thickness = 0.16
    canopy_bottom_z = 2.19
    glazing_bottom_z = floor_thickness
    glazing_height = canopy_bottom_z - glazing_bottom_z
    wing_panel_length = 1.30
    wing_panel_thickness = 0.038
    wing_panel_height = 2.02
    wing_panel_bottom_z = 0.07
    post_radius = 0.10

    rail_mesh = _build_push_rail_mesh()

    drum_frame = model.part("drum_frame")
    drum_frame.visual(
        Cylinder(radius=floor_radius, length=floor_thickness),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=dark_floor,
        name="floor_plate",
    )
    drum_frame.visual(
        Cylinder(radius=canopy_radius, length=canopy_thickness),
        origin=Origin(xyz=(0.0, 0.0, canopy_bottom_z + (canopy_thickness * 0.5))),
        material=canopy_dark,
        name="canopy",
    )
    arc_half_span = math.radians(48.0)
    _add_arc_glazing(
        drum_frame,
        prefix="east",
        radius=drum_radius,
        start_angle=-arc_half_span,
        end_angle=arc_half_span,
        pane_count=7,
        pane_width=0.34,
        pane_thickness=0.028,
        pane_height=glazing_height,
        bottom_z=glazing_bottom_z,
        glass_material=glass,
    )
    _add_arc_glazing(
        drum_frame,
        prefix="west",
        radius=drum_radius,
        start_angle=math.pi - arc_half_span,
        end_angle=math.pi + arc_half_span,
        pane_count=7,
        pane_width=0.34,
        pane_thickness=0.028,
        pane_height=glazing_height,
        bottom_z=glazing_bottom_z,
        glass_material=glass,
    )

    opening_edge_angles = (
        -arc_half_span,
        arc_half_span,
        math.pi - arc_half_span,
        math.pi + arc_half_span,
    )
    for i, angle in enumerate(opening_edge_angles):
        x, y = _polar(drum_radius, angle)
        drum_frame.visual(
            Box((0.09, 0.12, glazing_height)),
            origin=Origin(
                xyz=(x, y, glazing_bottom_z + (glazing_height * 0.5)),
                rpy=(0.0, 0.0, _tangent_yaw(angle)),
            ),
            material=bronze,
            name=f"portal_post_{i}",
        )

    drum_frame.inertial = Inertial.from_geometry(
        Box((canopy_radius * 2.0, canopy_radius * 2.0, canopy_bottom_z + canopy_thickness)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_bottom_z + canopy_thickness) * 0.5)),
    )

    door_rotor = model.part("door_rotor")
    door_rotor.visual(
        Cylinder(radius=post_radius, length=2.08),
        origin=Origin(xyz=(0.0, 0.0, 1.11)),
        material=bronze,
        name="post_core",
    )
    door_rotor.visual(
        Cylinder(radius=0.15, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=stainless,
        name="bottom_hub",
    )
    door_rotor.visual(
        Cylinder(radius=0.15, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.11)),
        material=stainless,
        name="top_hub",
    )

    base_angle = math.pi * 0.5
    for i in range(3):
        _add_radial_wing(
            door_rotor,
            index=i,
            angle=base_angle + (i * (2.0 * math.pi / 3.0)),
            panel_length=wing_panel_length,
            panel_thickness=wing_panel_thickness,
            panel_height=wing_panel_height,
            panel_bottom_z=wing_panel_bottom_z,
            post_radius=post_radius,
            glass_material=glass,
            frame_material=bronze,
            rail_material=stainless,
            rail_mesh=rail_mesh,
        )

    door_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.40, length=2.12),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )

    model.articulation(
        "door_rotation",
        ArticulationType.CONTINUOUS,
        parent=drum_frame,
        child=door_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum_frame = object_model.get_part("drum_frame")
    door_rotor = object_model.get_part("door_rotor")
    door_rotation = object_model.get_articulation("door_rotation")

    def _visuals_present(part, names: tuple[str, ...]) -> tuple[bool, list[str]]:
        missing: list[str] = []
        for visual_name in names:
            try:
                part.get_visual(visual_name)
            except Exception:
                missing.append(visual_name)
        return (len(missing) == 0, missing)

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

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

    panel_ok, missing_panels = _visuals_present(
        door_rotor, ("panel_0", "panel_1", "panel_2")
    )
    rail_ok, missing_rails = _visuals_present(door_rotor, ("rail_0", "rail_1", "rail_2"))
    ctx.check(
        "three wing panels are authored",
        panel_ok,
        details=f"missing={missing_panels}",
    )
    ctx.check(
        "three push rails are authored",
        rail_ok,
        details=f"missing={missing_rails}",
    )
    ctx.check(
        "door rotates continuously about vertical post axis",
        door_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(float(v) for v in door_rotation.axis) == (0.0, 0.0, 1.0)
        and door_rotation.motion_limits is not None
        and door_rotation.motion_limits.lower is None
        and door_rotation.motion_limits.upper is None,
        details=(
            f"type={door_rotation.articulation_type}, axis={door_rotation.axis}, "
            f"limits={door_rotation.motion_limits}"
        ),
    )

    panel_centers = [
        _aabb_center(ctx.part_element_world_aabb(door_rotor, elem=f"panel_{i}"))
        for i in range(3)
    ]
    panel_angles = []
    if all(center is not None for center in panel_centers):
        panel_angles = sorted(
            math.atan2(center[1], center[0]) % (2.0 * math.pi) for center in panel_centers
        )
    panel_spacings = []
    if len(panel_angles) == 3:
        panel_spacings = [
            (panel_angles[(i + 1) % 3] - panel_angles[i]) % (2.0 * math.pi)
            for i in range(3)
        ]
    ctx.check(
        "three wings are equally spaced around the center post",
        len(panel_spacings) == 3
        and all(abs(spacing - (2.0 * math.pi / 3.0)) < 0.08 for spacing in panel_spacings),
        details=f"panel_angles={panel_angles}, spacings={panel_spacings}",
    )

    ctx.expect_gap(
        door_rotor,
        drum_frame,
        axis="z",
        positive_elem="panel_0",
        negative_elem="floor_plate",
        min_gap=0.01,
        max_gap=0.04,
        name="wing panel clears the floor plate by a small running gap",
    )
    ctx.expect_gap(
        drum_frame,
        door_rotor,
        axis="z",
        positive_elem="canopy",
        negative_elem="panel_0",
        min_gap=0.05,
        max_gap=0.14,
        name="wing panel stays below the canopy",
    )

    rail_rest_center = _aabb_center(ctx.part_element_world_aabb(door_rotor, elem="rail_0"))
    with ctx.pose({door_rotation: math.pi / 3.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at sixty degrees of rotation")
        rail_turned_center = _aabb_center(
            ctx.part_element_world_aabb(door_rotor, elem="rail_0")
        )

    moved_distance = None
    if rail_rest_center is not None and rail_turned_center is not None:
        moved_distance = math.dist(rail_rest_center[:2], rail_turned_center[:2])
    ctx.check(
        "push rail travels around the drum when the door rotates",
        moved_distance is not None and moved_distance > 0.45,
        details=(
            f"rest_center={rail_rest_center}, turned_center={rail_turned_center}, "
            f"moved_distance={moved_distance}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
