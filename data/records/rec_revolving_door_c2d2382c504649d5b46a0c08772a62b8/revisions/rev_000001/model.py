from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_slab(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 96,
) -> MeshGeometry:
    """Closed flat ring used for the circular threshold track."""

    geom = MeshGeometry()
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        ca = math.cos(angle)
        sa = math.sin(angle)
        geom.add_vertex(outer_radius * ca, outer_radius * sa, z_min)
        geom.add_vertex(inner_radius * ca, inner_radius * sa, z_min)
        geom.add_vertex(outer_radius * ca, outer_radius * sa, z_max)
        geom.add_vertex(inner_radius * ca, inner_radius * sa, z_max)

    for i in range(segments):
        j = (i + 1) % segments
        o0b, i0b, o0t, i0t = 4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3
        o1b, i1b, o1t, i1t = 4 * j, 4 * j + 1, 4 * j + 2, 4 * j + 3
        # Outer and inner cylindrical walls.
        geom.add_face(o0b, o1b, o1t)
        geom.add_face(o0b, o1t, o0t)
        geom.add_face(i1b, i0b, i0t)
        geom.add_face(i1b, i0t, i1t)
        # Top and bottom annular faces.
        geom.add_face(o0t, o1t, i1t)
        geom.add_face(o0t, i1t, i0t)
        geom.add_face(o1b, o0b, i0b)
        geom.add_face(o1b, i0b, i1b)
    return geom


def _curved_wall_segment(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    segments: int = 36,
) -> MeshGeometry:
    """Thin closed cylindrical glass segment with real thickness."""

    geom = MeshGeometry()
    for i in range(segments + 1):
        t = i / segments
        angle = start_angle + (end_angle - start_angle) * t
        ca = math.cos(angle)
        sa = math.sin(angle)
        geom.add_vertex(outer_radius * ca, outer_radius * sa, z_min)
        geom.add_vertex(inner_radius * ca, inner_radius * sa, z_min)
        geom.add_vertex(outer_radius * ca, outer_radius * sa, z_max)
        geom.add_vertex(inner_radius * ca, inner_radius * sa, z_max)

    for i in range(segments):
        o0b, i0b, o0t, i0t = 4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3
        o1b, i1b, o1t, i1t = 4 * (i + 1), 4 * (i + 1) + 1, 4 * (i + 1) + 2, 4 * (i + 1) + 3
        geom.add_face(o0b, o1b, o1t)
        geom.add_face(o0b, o1t, o0t)
        geom.add_face(i1b, i0b, i0t)
        geom.add_face(i1b, i0t, i1t)
        geom.add_face(o0t, o1t, i1t)
        geom.add_face(o0t, i1t, i0t)
        geom.add_face(o1b, o0b, i0b)
        geom.add_face(o1b, i0b, i1b)

    # End caps.
    for i in (0, segments):
        o_b, i_b, o_t, i_t = 4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3
        geom.add_face(o_b, i_b, i_t)
        geom.add_face(o_b, i_t, o_t)
    return geom


def _rotated_origin(radius: float, angle: float, z: float) -> Origin:
    return Origin(xyz=(radius * math.cos(angle), radius * math.sin(angle), z), rpy=(0.0, 0.0, angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hotel_lobby_revolving_door")

    bronze = model.material("dark_bronze_metal", rgba=(0.22, 0.17, 0.11, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.58, 0.56, 0.52, 1.0))
    dark = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    glass = model.material("pale_blue_glass", rgba=(0.55, 0.78, 0.95, 0.34))
    wing_glass = model.material("clear_wing_glass", rgba=(0.72, 0.90, 1.0, 0.42))

    # Fixed architectural drum: threshold ring, curved side glazing, and canopy.
    drum = model.part("drum")
    drum.visual(
        mesh_from_geometry(
            _annular_slab(inner_radius=1.40, outer_radius=1.72, z_min=0.0, z_max=0.065),
            "threshold_ring",
        ),
        material=bronze,
        name="threshold_ring",
    )
    drum.visual(
        Cylinder(radius=1.72, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 2.83)),
        material=bronze,
        name="canopy",
    )
    drum.visual(
        Cylinder(radius=1.52, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 2.715)),
        material=brushed,
        name="canopy_trim",
    )
    drum.visual(
        Cylinder(radius=0.18, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 2.725)),
        material=brushed,
        name="upper_bearing_hub",
    )
    drum.visual(
        Cylinder(radius=0.18, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=brushed,
        name="floor_bearing_hub",
    )
    for idx in range(4):
        angle = idx * math.pi / 2.0
        drum.visual(
            Box((1.30, 0.080, 0.035)),
            origin=_rotated_origin(0.80, angle, 0.032),
            material=brushed,
            name=f"floor_spoke_{idx}",
        )

    # Two fixed glass arcs form the drum sides and leave broad opposite lobby openings.
    side_arc_centers = (math.pi / 2.0, 3.0 * math.pi / 2.0)
    arc_span = math.radians(118.0)
    for idx, center_angle in enumerate(side_arc_centers):
        drum.visual(
            mesh_from_geometry(
                _curved_wall_segment(
                    inner_radius=1.535,
                    outer_radius=1.575,
                    z_min=0.045,
                    z_max=2.745,
                    start_angle=center_angle - arc_span / 2.0,
                    end_angle=center_angle + arc_span / 2.0,
                ),
                f"curved_glass_{idx}",
            ),
            material=glass,
            name=f"curved_glass_{idx}",
        )

    mullion_angles = [
        side_arc_centers[0] - arc_span / 2.0,
        side_arc_centers[0] + arc_span / 2.0,
        side_arc_centers[1] - arc_span / 2.0,
        side_arc_centers[1] + arc_span / 2.0,
    ]
    for idx, angle in enumerate(mullion_angles):
        drum.visual(
            Cylinder(radius=0.035, length=2.74),
            origin=Origin(xyz=(1.555 * math.cos(angle), 1.555 * math.sin(angle), 1.39)),
            material=bronze,
            name=f"mullion_{idx}",
        )

    # Rotating four-wing assembly.  The child frame is exactly on the center axis.
    wings = model.part("wing_assembly")
    wings.visual(
        Cylinder(radius=0.070, length=2.57),
        origin=Origin(xyz=(0.0, 0.0, 1.365)),
        material=brushed,
        name="central_post",
    )
    wings.visual(
        Cylinder(radius=0.135, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
        material=brushed,
        name="lower_rotor_collar",
    )
    wings.visual(
        Cylinder(radius=0.125, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 2.6375)),
        material=brushed,
        name="upper_rotor_collar",
    )

    wing_length = 1.285
    glass_start = 0.17
    glass_length = 1.10
    glass_center = glass_start + glass_length / 2.0
    panel_mid_z = 1.365
    panel_height = 2.42
    for idx in range(4):
        angle = idx * math.pi / 2.0
        wings.visual(
            Box((glass_length, 0.030, panel_height)),
            origin=_rotated_origin(glass_center, angle, panel_mid_z),
            material=wing_glass,
            name=f"glass_{idx}",
        )
        wings.visual(
            Box((wing_length, 0.070, 0.070)),
            origin=_rotated_origin(wing_length / 2.0, angle, 2.61),
            material=bronze,
            name=f"top_rail_{idx}",
        )
        wings.visual(
            Box((wing_length, 0.070, 0.070)),
            origin=_rotated_origin(wing_length / 2.0, angle, 0.12),
            material=bronze,
            name=f"bottom_rail_{idx}",
        )
        wings.visual(
            Box((0.070, 0.080, panel_height + 0.12)),
            origin=_rotated_origin(wing_length, angle, panel_mid_z),
            material=bronze,
            name=f"outer_stile_{idx}",
        )
        wings.visual(
            Box((0.105, 0.090, panel_height + 0.12)),
            origin=_rotated_origin(0.095, angle, panel_mid_z),
            material=bronze,
            name=f"post_clamp_{idx}",
        )
        wings.visual(
            Box((0.055, 0.060, panel_height)),
            origin=_rotated_origin(glass_start + glass_length * 0.52, angle, panel_mid_z),
            material=bronze,
            name=f"center_rail_{idx}",
        )

    model.articulation(
        "drum_to_wings",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=wings,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.9),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    wings = object_model.get_part("wing_assembly")
    spin = object_model.get_articulation("drum_to_wings")

    ctx.check(
        "wing assembly uses a continuous vertical spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        wings,
        drum,
        axes="xy",
        margin=0.02,
        name="four wings stay inside the cylindrical drum footprint",
    )
    ctx.expect_gap(
        drum,
        wings,
        axis="z",
        positive_elem="canopy",
        negative_elem="upper_rotor_collar",
        min_gap=0.02,
        name="fixed canopy clears the rotating upper collar",
    )

    rest_aabb = ctx.part_element_world_aabb(wings, elem="glass_0")
    with ctx.pose({spin: math.pi / 2.0}):
        quarter_turn_aabb = ctx.part_element_world_aabb(wings, elem="glass_0")

    if rest_aabb is not None and quarter_turn_aabb is not None:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5
        rest_center_y = (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
        turned_center_x = (quarter_turn_aabb[0][0] + quarter_turn_aabb[1][0]) * 0.5
        turned_center_y = (quarter_turn_aabb[0][1] + quarter_turn_aabb[1][1]) * 0.5
        ctx.check(
            "a glazed wing sweeps around the central post",
            rest_center_x > 0.60
            and abs(rest_center_y) < 0.05
            and abs(turned_center_x) < 0.05
            and turned_center_y > 0.60,
            details=(
                f"rest=({rest_center_x:.3f}, {rest_center_y:.3f}), "
                f"quarter=({turned_center_x:.3f}, {turned_center_y:.3f})"
            ),
        )
    else:
        ctx.fail("a glazed wing sweeps around the central post", "missing glass_0 world AABB")

    return ctx.report()


object_model = build_object_model()
