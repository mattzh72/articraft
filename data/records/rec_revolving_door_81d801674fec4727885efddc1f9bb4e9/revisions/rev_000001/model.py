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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float, *, y0: float = 0.0) -> list[tuple[float, float]]:
    half_w = width * 0.5
    return [
        (-half_w, y0),
        (half_w, y0),
        (half_w, y0 + height),
        (-half_w, y0 + height),
    ]


def _circle_profile(radius: float, *, segments: int = 72, angle_offset: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(angle_offset + (2.0 * math.pi * i) / segments),
            radius * math.sin(angle_offset + (2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _annular_sector_profile(
    inner_radius: float,
    outer_radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 32,
) -> list[tuple[float, float]]:
    outer = [
        (
            outer_radius * math.cos(start_angle + (end_angle - start_angle) * i / segments),
            outer_radius * math.sin(start_angle + (end_angle - start_angle) * i / segments),
        )
        for i in range(segments + 1)
    ]
    inner = [
        (
            inner_radius * math.cos(end_angle - (end_angle - start_angle) * i / segments),
            inner_radius * math.sin(end_angle - (end_angle - start_angle) * i / segments),
        )
        for i in range(segments + 1)
    ]
    return outer + inner


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="campus_center_revolving_door")

    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.63, 0.65, 0.68, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.28, 0.30, 0.32, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.83, 0.90, 0.28))
    facade_stone = model.material("facade_stone", rgba=(0.70, 0.70, 0.68, 1.0))
    threshold_metal = model.material("threshold_metal", rgba=(0.46, 0.47, 0.49, 1.0))

    wall_width = 7.0
    wall_height = 4.2
    wall_thickness = 0.24
    opening_width = 1.56
    opening_bottom = 0.03
    opening_top = 2.52

    drum_outer_radius = 1.35
    guide_inner_radius = 1.26
    ring_inner_radius = 1.14
    rotor_radius = 1.16
    drum_center_y = -drum_outer_radius

    bottom_ring_bottom = 0.03
    bottom_ring_height = 0.05
    guide_bottom = 0.075
    guide_top = 2.405
    top_ring_bottom = 2.40
    top_ring_height = 0.12
    collar_outer_radius = 0.18
    collar_inner_radius = 0.082
    collar_thickness = 0.015
    support_inner_reach = 0.14
    support_outer_reach = 1.20
    support_beam_width = 0.10
    support_beam_thickness = 0.02

    facade = model.part("facade")
    side_width = (wall_width - opening_width) * 0.5
    header_height = wall_height - opening_top
    frame_overlap = 0.02
    facade.visual(
        Box((side_width, wall_thickness, wall_height)),
        origin=Origin(xyz=(-(opening_width * 0.5 + side_width * 0.5), 0.0, wall_height * 0.5)),
        material=facade_stone,
        name="left_jamb",
    )
    facade.visual(
        Box((side_width, wall_thickness, wall_height)),
        origin=Origin(xyz=(opening_width * 0.5 + side_width * 0.5, 0.0, wall_height * 0.5)),
        material=facade_stone,
        name="right_jamb",
    )
    facade.visual(
        Box((opening_width + frame_overlap * 2.0, wall_thickness, header_height)),
        origin=Origin(
            xyz=(0.0, 0.0, opening_top + header_height * 0.5),
        ),
        material=facade_stone,
        name="header",
    )
    facade.visual(
        Box((opening_width + frame_overlap * 2.0, wall_thickness, opening_bottom)),
        origin=Origin(xyz=(0.0, 0.0, opening_bottom * 0.5)),
        material=facade_stone,
        name="threshold",
    )
    facade.inertial = Inertial.from_geometry(
        Box((wall_width, wall_thickness, wall_height)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, wall_height * 0.5)),
    )

    stationary_drum = model.part("stationary_drum")
    top_ring_geom = ExtrudeWithHolesGeometry(
        _circle_profile(drum_outer_radius, segments=88),
        [_circle_profile(ring_inner_radius, segments=88, angle_offset=0.03)],
        top_ring_height,
        center=False,
    )
    bottom_ring_geom = ExtrudeWithHolesGeometry(
        _circle_profile(drum_outer_radius, segments=88),
        [_circle_profile(ring_inner_radius, segments=88, angle_offset=0.03)],
        bottom_ring_height,
        center=False,
    )
    edge_angle = math.radians(55.0)
    right_guide_geom = ExtrudeWithHolesGeometry(
        _annular_sector_profile(
            guide_inner_radius,
            drum_outer_radius,
            -edge_angle,
            edge_angle,
            segments=44,
        ),
        [],
        guide_top - guide_bottom,
        center=False,
    )
    left_guide_geom = ExtrudeWithHolesGeometry(
        _annular_sector_profile(
            guide_inner_radius,
            drum_outer_radius,
            math.pi - edge_angle,
            math.pi + edge_angle,
            segments=44,
        ),
        [],
        guide_top - guide_bottom,
        center=False,
    )
    top_bearing_geom = ExtrudeWithHolesGeometry(
        _circle_profile(collar_outer_radius, segments=64),
        [_circle_profile(collar_inner_radius, segments=64, angle_offset=0.04)],
        collar_thickness,
        center=False,
    )
    bottom_bearing_geom = ExtrudeWithHolesGeometry(
        _circle_profile(collar_outer_radius, segments=64),
        [_circle_profile(collar_inner_radius, segments=64, angle_offset=0.04)],
        collar_thickness,
        center=False,
    )
    stationary_drum.visual(
        mesh_from_geometry(top_ring_geom, "drum_top_ring"),
        origin=Origin(xyz=(0.0, 0.0, top_ring_bottom)),
        material=anodized_aluminum,
        name="top_ring",
    )
    stationary_drum.visual(
        mesh_from_geometry(bottom_ring_geom, "drum_bottom_ring"),
        origin=Origin(xyz=(0.0, 0.0, bottom_ring_bottom)),
        material=threshold_metal,
        name="bottom_ring",
    )
    stationary_drum.visual(
        mesh_from_geometry(left_guide_geom, "drum_left_guide"),
        origin=Origin(xyz=(0.0, 0.0, guide_bottom)),
        material=glass,
        name="left_guide_wall",
    )
    stationary_drum.visual(
        mesh_from_geometry(right_guide_geom, "drum_right_guide"),
        origin=Origin(xyz=(0.0, 0.0, guide_bottom)),
        material=glass,
        name="right_guide_wall",
    )
    stationary_drum.visual(
        mesh_from_geometry(top_bearing_geom, "top_bearing_plate"),
        origin=Origin(xyz=(0.0, 0.0, top_ring_bottom - collar_thickness)),
        material=dark_aluminum,
        name="top_bearing_plate",
    )
    stationary_drum.visual(
        mesh_from_geometry(bottom_bearing_geom, "bottom_bearing_plate"),
        origin=Origin(
            xyz=(0.0, 0.0, bottom_ring_bottom + bottom_ring_height - collar_thickness),
        ),
        material=dark_aluminum,
        name="bottom_bearing_plate",
    )
    support_beam_length = support_outer_reach - support_inner_reach
    support_beam_center = (support_outer_reach + support_inner_reach) * 0.5
    top_support_z = top_ring_bottom - support_beam_thickness * 0.5
    bottom_support_z = bottom_ring_bottom + bottom_ring_height - support_beam_thickness * 0.5
    for prefix, z_center in (("top", top_support_z), ("bottom", bottom_support_z)):
        stationary_drum.visual(
            Box((support_beam_length, support_beam_width, support_beam_thickness)),
            origin=Origin(xyz=(support_beam_center, 0.0, z_center)),
            material=dark_aluminum,
            name=f"{prefix}_support_east",
        )
        stationary_drum.visual(
            Box((support_beam_length, support_beam_width, support_beam_thickness)),
            origin=Origin(xyz=(-support_beam_center, 0.0, z_center)),
            material=dark_aluminum,
            name=f"{prefix}_support_west",
        )
        stationary_drum.visual(
            Box((support_beam_width, support_beam_length, support_beam_thickness)),
            origin=Origin(xyz=(0.0, support_beam_center, z_center)),
            material=dark_aluminum,
            name=f"{prefix}_support_north",
        )
        stationary_drum.visual(
            Box((support_beam_width, support_beam_length, support_beam_thickness)),
            origin=Origin(xyz=(0.0, -support_beam_center, z_center)),
            material=dark_aluminum,
            name=f"{prefix}_support_south",
        )
    stationary_drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_outer_radius, length=opening_top - opening_bottom),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, (opening_top + opening_bottom) * 0.5)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.08, length=2.42),
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
        material=dark_aluminum,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 2.41)),
        material=anodized_aluminum,
        name="top_hub",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=anodized_aluminum,
        name="bottom_hub",
    )

    panel_height = 2.28
    panel_bottom = 0.10
    panel_thickness = 0.045
    hub_overlap = 0.03
    wing_span = rotor_radius
    panel_center = wing_span * 0.5 - hub_overlap

    rotor.visual(
        Box((panel_thickness, wing_span, panel_height)),
        origin=Origin(xyz=(0.0, panel_center, panel_bottom + panel_height * 0.5)),
        material=glass,
        name="wing_front",
    )
    rotor.visual(
        Box((panel_thickness, wing_span, panel_height)),
        origin=Origin(xyz=(0.0, -panel_center, panel_bottom + panel_height * 0.5)),
        material=glass,
        name="wing_back",
    )
    rotor.visual(
        Box((panel_thickness, wing_span, panel_height)),
        origin=Origin(
            xyz=(panel_center, 0.0, panel_bottom + panel_height * 0.5),
            rpy=(0.0, 0.0, math.pi / 2.0),
        ),
        material=glass,
        name="wing_right",
    )
    rotor.visual(
        Box((panel_thickness, wing_span, panel_height)),
        origin=Origin(
            xyz=(-panel_center, 0.0, panel_bottom + panel_height * 0.5),
            rpy=(0.0, 0.0, math.pi / 2.0),
        ),
        material=glass,
        name="wing_left",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=rotor_radius, length=2.42),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
    )

    model.articulation(
        "facade_to_stationary_drum",
        ArticulationType.FIXED,
        parent=facade,
        child=stationary_drum,
        origin=Origin(xyz=(0.0, drum_center_y, 0.0)),
    )
    model.articulation(
        "drum_rotation",
        ArticulationType.CONTINUOUS,
        parent=stationary_drum,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    facade = object_model.get_part("facade")
    stationary_drum = object_model.get_part("stationary_drum")
    rotor = object_model.get_part("rotor")
    drum_rotation = object_model.get_articulation("drum_rotation")
    drum_center = ctx.part_world_position(stationary_drum)
    drum_center_y = None if drum_center is None else drum_center[1]

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    limits = drum_rotation.motion_limits
    ctx.check(
        "revolving drum uses vertical continuous rotation",
        drum_rotation.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in drum_rotation.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={drum_rotation.joint_type}, axis={drum_rotation.axis}, "
            f"limits=({None if limits is None else limits.lower}, "
            f"{None if limits is None else limits.upper})"
        ),
    )

    ctx.expect_gap(
        facade,
        stationary_drum,
        axis="z",
        positive_elem="header",
        negative_elem="top_ring",
        max_gap=0.002,
        max_penetration=0.0,
        name="top canopy ring meets facade header flush",
    )
    ctx.expect_gap(
        stationary_drum,
        facade,
        axis="z",
        positive_elem="bottom_ring",
        negative_elem="threshold",
        max_gap=0.002,
        max_penetration=0.0,
        name="bottom ring meets threshold flush",
    )

    with ctx.pose({drum_rotation: 0.0}):
        ctx.expect_within(
            rotor,
            stationary_drum,
            axes="xy",
            margin=0.20,
            name="rotor stays inside drum envelope at rest",
        )
        front_center = _aabb_center(ctx.part_element_world_aabb(rotor, elem="wing_front"))
        ctx.check(
            "front wing starts aligned with entry opening",
            front_center is not None and abs(front_center[0]) < 0.05 and front_center[1] > -0.90,
            details=f"front wing center at rest={front_center}",
        )

    with ctx.pose({drum_rotation: math.pi / 2.0}):
        ctx.expect_within(
            rotor,
            stationary_drum,
            axes="xy",
            margin=0.20,
            name="rotor stays inside drum envelope quarter turn",
        )
        rotated_center = _aabb_center(ctx.part_element_world_aabb(rotor, elem="wing_front"))
        ctx.check(
            "front wing quarter turn sweeps sideways around the post",
            rotated_center is not None
            and drum_center_y is not None
            and rotated_center[0] < -0.45
            and abs(rotated_center[1] - drum_center_y) < 0.08,
            details=f"front wing center at quarter turn={rotated_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
