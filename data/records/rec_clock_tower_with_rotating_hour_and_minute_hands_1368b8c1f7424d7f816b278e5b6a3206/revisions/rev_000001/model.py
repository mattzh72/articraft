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
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _octagon_profile(flat_half: float, chamfer: float) -> list[tuple[float, float]]:
    return [
        (flat_half - chamfer, flat_half),
        (-flat_half + chamfer, flat_half),
        (-flat_half, flat_half - chamfer),
        (-flat_half, -flat_half + chamfer),
        (-flat_half + chamfer, -flat_half),
        (flat_half - chamfer, -flat_half),
        (flat_half, -flat_half + chamfer),
        (flat_half, flat_half - chamfer),
    ]


def _hand_mesh(
    name: str,
    *,
    length: float,
    back_length: float,
    base_half_width: float,
    tip_half_width: float,
    counter_half_width: float,
    thickness: float,
    hub_radius: float,
) :
    profile = [
        (-back_length, 0.0),
        (-back_length * 0.55, counter_half_width),
        (-0.10, base_half_width),
        (length * 0.22, base_half_width * 0.86),
        (length * 0.82, tip_half_width * 1.12),
        (length, 0.0),
        (length * 0.82, -tip_half_width * 1.12),
        (length * 0.22, -base_half_width * 0.86),
        (-0.10, -base_half_width),
        (-back_length * 0.55, -counter_half_width),
    ]
    hand_geom = ExtrudeGeometry.from_z0(profile, thickness)
    hand_geom.merge(
        CylinderGeometry(hub_radius, thickness, radial_segments=36).translate(0.0, 0.0, thickness * 0.5)
    )
    hand_geom.rotate_y(math.pi / 2.0)
    return _mesh(name, hand_geom)


def _face_pose(face: str) -> tuple[str, float, tuple[float, float, float], tuple[float, float, float]]:
    if face == "east":
        return "x", 1.0, (1.0, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)
    if face == "west":
        return "x", -1.0, (-1.0, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)
    if face == "north":
        return "y", 1.0, (0.0, 1.0, 0.0), (math.pi / 2.0, 0.0, 0.0)
    if face == "south":
        return "y", -1.0, (0.0, -1.0, 0.0), (math.pi / 2.0, 0.0, 0.0)
    raise ValueError(f"Unsupported face {face}")


def _clock_origin(face: str, plane: float, z: float) -> tuple[float, float, float]:
    axis_name, sign, _, _ = _face_pose(face)
    if axis_name == "x":
        return (sign * plane, 0.0, z)
    return (0.0, sign * plane, z)


def _clock_marker_position(face: str, plane: float, center_z: float, radius: float, angle: float) -> tuple[float, float, float]:
    axis_name, sign, _, _ = _face_pose(face)
    lateral = radius * math.sin(angle)
    vertical = radius * math.cos(angle)
    if axis_name == "x":
        return (sign * plane, lateral, center_z + vertical)
    return (lateral, sign * plane, center_z + vertical)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="baroque_church_tower")

    limestone = model.material("limestone", rgba=(0.84, 0.80, 0.72, 1.0))
    limestone_trim = model.material("limestone_trim", rgba=(0.90, 0.87, 0.79, 1.0))
    patinated_copper = model.material("patinated_copper", rgba=(0.34, 0.58, 0.53, 1.0))
    dark_slate = model.material("dark_slate", rgba=(0.18, 0.21, 0.23, 1.0))
    clock_enamel = model.material("clock_enamel", rgba=(0.96, 0.95, 0.90, 1.0))
    clock_iron = model.material("clock_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    gilt = model.material("gilt", rgba=(0.79, 0.67, 0.28, 1.0))

    shaft_width = 8.4
    shaft_half = shaft_width * 0.5
    plinth_width = 10.6
    plinth_height = 1.3
    shaft_height = 26.0
    shaft_base_z = plinth_height
    shaft_top_z = shaft_base_z + shaft_height
    clock_center_z = 23.6

    dial_radius = 1.58
    dial_thickness = 0.08
    bezel_thickness = 0.05
    bezel_radius = 1.72
    bezel_inner_radius = 1.44
    hour_boss_length = 0.03
    hour_hand_thickness = 0.035
    minute_hand_thickness = 0.028
    inter_hand_gap = 0.005
    hour_joint_plane = shaft_half + dial_thickness + hour_boss_length
    minute_joint_plane = hour_joint_plane + hour_hand_thickness + inter_hand_gap

    drum_height = 5.7
    drum_z = shaft_top_z
    drum_flat_half = 3.62
    drum_chamfer = 1.05
    drum_mesh = _mesh(
        "octagonal_drum",
        ExtrudeGeometry.from_z0(_octagon_profile(drum_flat_half, drum_chamfer), drum_height),
    )
    drum_cornice_mesh = _mesh(
        "drum_cornice",
        ExtrudeGeometry.from_z0(_octagon_profile(3.95, 1.10), 0.42),
    )

    dome_profile = [
        (3.54, 0.0),
        (3.86, 0.35),
        (4.18, 1.45),
        (4.28, 2.55),
        (3.88, 3.80),
        (2.70, 5.05),
        (1.48, 6.08),
        (0.72, 6.72),
        (0.34, 7.18),
        (0.0, 7.55),
    ]
    dome_mesh = _mesh("onion_dome", LatheGeometry(dome_profile, segments=84))
    bezel_mesh = _mesh(
        "clock_bezel_ring",
        LatheGeometry.from_shell_profiles(
            [
                (bezel_radius, -bezel_thickness * 0.5),
                (bezel_radius, bezel_thickness * 0.5),
            ],
            [
                (bezel_inner_radius, -bezel_thickness * 0.5),
                (bezel_inner_radius, bezel_thickness * 0.5),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
    )

    minute_hand_mesh = _hand_mesh(
        "minute_hand",
        length=1.36,
        back_length=0.30,
        base_half_width=0.080,
        tip_half_width=0.030,
        counter_half_width=0.094,
        thickness=minute_hand_thickness,
        hub_radius=0.115,
    )
    hour_hand_mesh = _hand_mesh(
        "hour_hand",
        length=0.98,
        back_length=0.24,
        base_half_width=0.092,
        tip_half_width=0.040,
        counter_half_width=0.100,
        thickness=hour_hand_thickness,
        hub_radius=0.175,
    )

    tower_body = model.part("tower_body")
    tower_body.visual(
        Box((plinth_width, plinth_width, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=limestone_trim,
        name="foundation_plinth",
    )
    tower_body.visual(
        Box((9.4, 9.4, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height + 0.275)),
        material=limestone_trim,
        name="water_table",
    )
    tower_body.visual(
        Box((shaft_width, shaft_width, shaft_height)),
        origin=Origin(xyz=(0.0, 0.0, shaft_base_z + shaft_height * 0.5)),
        material=limestone,
        name="main_shaft",
    )
    tower_body.visual(
        Box((9.2, 9.2, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 21.2)),
        material=limestone_trim,
        name="lower_clock_cornice",
    )
    tower_body.visual(
        Box((9.5, 9.5, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, shaft_top_z - 0.36)),
        material=limestone_trim,
        name="upper_cornice",
    )
    for corner_x in (-1.0, 1.0):
        for corner_y in (-1.0, 1.0):
            tower_body.visual(
                Box((0.58, 0.58, shaft_height)),
                origin=Origin(
                    xyz=(
                        corner_x * (shaft_half - 0.29),
                        corner_y * (shaft_half - 0.29),
                        shaft_base_z + shaft_height * 0.5,
                    )
                ),
                material=limestone_trim,
                name=f"quoin_{'p' if corner_x > 0 else 'n'}x_{'p' if corner_y > 0 else 'n'}y",
            )

    for side_name, offset in (
        ("east", (shaft_half + 0.12, 0.0, 17.7)),
        ("west", (-shaft_half - 0.12, 0.0, 17.7)),
        ("north", (0.0, shaft_half + 0.12, 17.7)),
        ("south", (0.0, -shaft_half - 0.12, 17.7)),
    ):
        tower_body.visual(
            Box((0.24, 2.05, 8.0)) if side_name in ("east", "west") else Box((2.05, 0.24, 8.0)),
            origin=Origin(xyz=offset),
            material=limestone_trim,
            name=f"{side_name}_pilaster_strip",
        )

    tower_body.visual(
        drum_cornice_mesh,
        origin=Origin(xyz=(0.0, 0.0, drum_z - 0.10)),
        material=limestone_trim,
        name="drum_base_cornice",
    )
    tower_body.visual(
        drum_mesh,
        origin=Origin(xyz=(0.0, 0.0, drum_z)),
        material=limestone_trim,
        name="octagonal_drum",
    )
    tower_body.visual(
        drum_cornice_mesh,
        origin=Origin(xyz=(0.0, 0.0, drum_z + drum_height - 0.08)),
        material=limestone_trim,
        name="drum_top_cornice",
    )
    tower_body.visual(
        dome_mesh,
        origin=Origin(xyz=(0.0, 0.0, drum_z + drum_height)),
        material=patinated_copper,
        name="onion_dome",
    )
    finial_base_z = drum_z + drum_height + dome_profile[-1][1]
    tower_body.visual(
        Cylinder(radius=0.16, length=1.10),
        origin=Origin(xyz=(0.0, 0.0, finial_base_z + 0.55)),
        material=dark_slate,
        name="finial_stem",
    )
    tower_body.visual(
        Sphere(radius=0.22),
        origin=Origin(xyz=(0.0, 0.0, finial_base_z + 1.26)),
        material=gilt,
        name="finial_knob",
    )
    tower_body.visual(
        Box((0.72, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, finial_base_z + 1.62)),
        material=gilt,
        name="cross_arm",
    )
    tower_body.visual(
        Box((0.08, 0.08, 0.82)),
        origin=Origin(xyz=(0.0, 0.0, finial_base_z + 1.70)),
        material=gilt,
        name="cross_post",
    )

    face_part_rpy = {
        "east": (0.0, 0.0, 0.0),
        "west": (0.0, 0.0, math.pi),
        "north": (0.0, 0.0, math.pi / 2.0),
        "south": (0.0, 0.0, -math.pi / 2.0),
    }
    dial_plate_thickness = 0.03
    bezel_center_x = dial_plate_thickness + bezel_thickness * 0.5 - 0.004
    marker_thickness = 0.012
    marker_center_x = dial_plate_thickness - 0.006

    for face in ("east", "west", "north", "south"):
        face_part = model.part(f"{face}_clock_face")
        face_part.visual(
            Cylinder(radius=dial_radius, length=dial_plate_thickness),
            origin=Origin(
                xyz=(dial_plate_thickness * 0.5, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=clock_enamel,
            name="dial_plate",
        )
        face_part.visual(
            bezel_mesh,
            origin=Origin(xyz=(bezel_center_x, 0.0, 0.0)),
            material=dark_slate,
            name="bezel_ring",
        )
        for marker_index in range(12):
            angle = 2.0 * math.pi * marker_index / 12.0
            lateral = dial_radius * 0.77 * math.sin(angle)
            vertical = dial_radius * 0.77 * math.cos(angle)
            if marker_index % 3 == 0:
                marker_long = 0.50
                marker_wide = 0.18
            else:
                marker_long = 0.28
                marker_wide = 0.12
            face_part.visual(
                Box((marker_thickness, marker_wide, marker_long)),
                origin=Origin(xyz=(marker_center_x, lateral, vertical)),
                material=clock_iron,
                name=f"marker_{marker_index:02d}",
            )

        model.articulation(
            f"tower_to_{face}_clock_face",
            ArticulationType.FIXED,
            parent=tower_body,
            child=face_part,
            origin=Origin(
                xyz=_clock_origin(face, shaft_half, clock_center_z),
                rpy=face_part_rpy[face],
            ),
        )

        hour_part = model.part(f"{face}_hour_hand")
        hour_part.visual(
            hour_hand_mesh,
            material=clock_iron,
            name="hour_hand_plate",
        )

        minute_part = model.part(f"{face}_minute_hand")
        minute_part.visual(
            minute_hand_mesh,
            material=clock_iron,
            name="minute_hand_plate",
        )
        minute_part.visual(
            Cylinder(radius=0.052, length=hour_hand_thickness + inter_hand_gap),
            origin=Origin(
                xyz=(-(hour_hand_thickness + inter_hand_gap) * 0.5, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=gilt,
            name="minute_sleeve",
        )

        model.articulation(
            f"{face}_hour_joint",
            ArticulationType.REVOLUTE,
            parent=face_part,
            child=hour_part,
            origin=Origin(xyz=(dial_plate_thickness, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.8,
                lower=-2.0 * math.pi,
                upper=2.0 * math.pi,
            ),
        )
        model.articulation(
            f"{face}_minute_joint",
            ArticulationType.REVOLUTE,
            parent=face_part,
            child=minute_part,
            origin=Origin(xyz=(dial_plate_thickness + hour_hand_thickness + inter_hand_gap, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.2,
                lower=-2.0 * math.pi,
                upper=2.0 * math.pi,
            ),
        )

    return model
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_body = object_model.get_part("tower_body")
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
    for face in ("east", "west", "north", "south"):
        ctx.allow_overlap(
            object_model.get_part(f"{face}_hour_hand"),
            object_model.get_part(f"{face}_minute_hand"),
            elem_a="hour_hand_plate",
            elem_b="minute_sleeve",
            reason="The minute-hand cannon sleeve passes through the hour-hand hub on a shared clock arbor.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    faces = ("east", "west", "north", "south")

    for face in faces:
        face_part = object_model.get_part(f"{face}_clock_face")
        hour_part = object_model.get_part(f"{face}_hour_hand")
        minute_part = object_model.get_part(f"{face}_minute_hand")
        face_mount = object_model.get_articulation(f"tower_to_{face}_clock_face")
        hour_joint = object_model.get_articulation(f"{face}_hour_joint")
        minute_joint = object_model.get_articulation(f"{face}_minute_joint")
        face_visual_names = {visual.name for visual in face_part.visuals if visual.name is not None}

        ctx.check(
            f"{face}_clock_face_present",
            face_part is not None,
            f"Missing clock-face part on {face} side.",
        )
        ctx.check(
            f"{face}_dial_present",
            "dial_plate" in face_visual_names and "bezel_ring" in face_visual_names,
            f"Missing dial plate or bezel ring on {face} clock face.",
        )
        ctx.expect_contact(face_part, tower_body, name=f"{face}_clock_face_mounted")
        ctx.expect_contact(hour_part, face_part, name=f"{face}_hour_hand_mounted")
        ctx.expect_contact(minute_part, face_part, name=f"{face}_minute_hand_mounted")
        ctx.expect_overlap(hour_part, face_part, axes="z", min_overlap=0.20, name=f"{face}_hour_clock_level")
        ctx.expect_overlap(minute_part, face_part, axes="z", min_overlap=0.20, name=f"{face}_minute_clock_level")
        ctx.check(
            f"{face}_concentric_joint_axes",
            tuple(hour_joint.axis) == tuple(minute_joint.axis),
            f"{face} hour/minute joints should share one axis, got {hour_joint.axis} vs {minute_joint.axis}",
        )
        ctx.check(
            f"{face}_face_mount_fixed",
            face_mount.articulation_type == ArticulationType.FIXED,
            f"{face} clock face should be rigidly mounted to tower body.",
        )

        hour_xyz = hour_joint.origin.xyz
        minute_xyz = minute_joint.origin.xyz
        same_hub = abs(hour_xyz[1] - minute_xyz[1]) < 1e-9 and abs(hour_xyz[2] - minute_xyz[2]) < 1e-9
        ctx.check(
            f"{face}_concentric_joint_line",
            same_hub,
            f"{face} hour/minute joint origins should align on one hub line, got {hour_xyz} and {minute_xyz}.",
        )
        ctx.check(
            f"{face}_minute_joint_ahead_of_hour",
            minute_xyz[0] > hour_xyz[0],
            f"{face} minute hand should sit proud of the hour hand on the shared hub.",
        )

    east_hour = object_model.get_part("east_hour_hand")
    east_minute = object_model.get_part("east_minute_hand")
    north_hour = object_model.get_part("north_hour_hand")
    north_minute = object_model.get_part("north_minute_hand")
    east_minute_joint = object_model.get_articulation("east_minute_joint")
    north_minute_joint = object_model.get_articulation("north_minute_joint")

    ctx.expect_gap(
        east_minute,
        east_hour,
        axis="x",
        min_gap=0.0,
        max_gap=0.008,
        positive_elem="minute_hand_plate",
        negative_elem="hour_hand_plate",
        name="east_hand_depth_layering",
    )
    ctx.expect_gap(
        north_minute,
        north_hour,
        axis="y",
        min_gap=0.0,
        max_gap=0.008,
        positive_elem="minute_hand_plate",
        negative_elem="hour_hand_plate",
        name="north_hand_depth_layering",
    )

    def _span(aabb, axis_index: int) -> float:
        return aabb[1][axis_index] - aabb[0][axis_index]

    east_rest = ctx.part_world_aabb(east_minute)
    north_rest = ctx.part_world_aabb(north_minute)
    assert east_rest is not None
    assert north_rest is not None
    ctx.check(
        "east_minute_rest_vertical",
        _span(east_rest, 2) > _span(east_rest, 1) + 0.80,
        "East minute hand should read primarily vertical in the rest pose.",
    )
    ctx.check(
        "north_minute_rest_vertical",
        _span(north_rest, 2) > _span(north_rest, 0) + 0.80,
        "North minute hand should read primarily vertical in the rest pose.",
    )

    with ctx.pose({east_minute_joint: math.pi / 2.0}):
        east_quarter = ctx.part_world_aabb(east_minute)
        assert east_quarter is not None
        ctx.expect_contact(east_minute, object_model.get_part("east_clock_face"), name="east_minute_contact_in_pose")
        ctx.check(
            "east_minute_rotates_in_dial_plane",
            _span(east_quarter, 1) > _span(east_quarter, 2) + 0.80,
            "East minute hand should swing across the dial plane about the face normal.",
        )

    with ctx.pose({north_minute_joint: math.pi / 2.0}):
        north_quarter = ctx.part_world_aabb(north_minute)
        assert north_quarter is not None
        ctx.expect_contact(north_minute, object_model.get_part("north_clock_face"), name="north_minute_contact_in_pose")
        ctx.check(
            "north_minute_rotates_in_dial_plane",
            _span(north_quarter, 0) > _span(north_quarter, 2) + 0.80,
            "North minute hand should swing across the dial plane about the face normal.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
