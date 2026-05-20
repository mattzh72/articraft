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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _build_sector_shell(
    *,
    radius: float,
    thickness: float,
    height: float,
    start_angle: float,
    end_angle: float,
    segments: int = 36,
    z0: float = 0.0,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_r = radius + thickness * 0.5
    inner_r = radius - thickness * 0.5

    bottom_outer: list[int] = []
    top_outer: list[int] = []
    bottom_inner: list[int] = []
    top_inner: list[int] = []

    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        c = math.cos(angle)
        s = math.sin(angle)
        bottom_outer.append(geom.add_vertex(outer_r * c, outer_r * s, z0))
        top_outer.append(geom.add_vertex(outer_r * c, outer_r * s, z0 + height))
        bottom_inner.append(geom.add_vertex(inner_r * c, inner_r * s, z0))
        top_inner.append(geom.add_vertex(inner_r * c, inner_r * s, z0 + height))

    for index in range(segments):
        bo0 = bottom_outer[index]
        bo1 = bottom_outer[index + 1]
        to0 = top_outer[index]
        to1 = top_outer[index + 1]
        bi0 = bottom_inner[index]
        bi1 = bottom_inner[index + 1]
        ti0 = top_inner[index]
        ti1 = top_inner[index + 1]

        _add_quad(geom, bo0, bo1, to1, to0)
        _add_quad(geom, bi1, bi0, ti0, ti1)
        _add_quad(geom, bi0, bi1, bo1, bo0)
        _add_quad(geom, ti0, to0, to1, ti1)

    full_circle = math.isclose(abs(end_angle - start_angle), 2.0 * math.pi, rel_tol=1e-6, abs_tol=1e-6)
    if not full_circle:
        _add_quad(geom, bottom_inner[0], bottom_outer[0], top_outer[0], top_inner[0])
        _add_quad(
            geom,
            bottom_outer[-1],
            bottom_inner[-1],
            top_inner[-1],
            top_outer[-1],
        )

    return geom


def _radial_origin(
    *,
    radius: float,
    angle: float,
    z: float,
) -> Origin:
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
        rpy=(0.0, 0.0, angle),
    )


def _add_revolving_wing(
    part,
    *,
    index: int,
    angle: float,
    metal,
    glass,
) -> None:
    inner_offset = 0.14
    glass_len = 0.80
    stile_thickness = 0.045
    rail_thickness = 0.050

    part.visual(
        Box((0.24, 0.060, 0.050)),
        origin=_radial_origin(radius=0.12, angle=angle, z=2.18),
        material=metal,
        name=f"wing_{index}_upper_arm",
    )
    part.visual(
        Box((0.24, 0.060, 0.050)),
        origin=_radial_origin(radius=0.12, angle=angle, z=0.10),
        material=metal,
        name=f"wing_{index}_lower_arm",
    )
    part.visual(
        Box((glass_len + stile_thickness, 0.040, 0.050)),
        origin=_radial_origin(
            radius=inner_offset + (glass_len + stile_thickness) * 0.5,
            angle=angle,
            z=2.18,
        ),
        material=metal,
        name=f"wing_{index}_top_rail",
    )
    part.visual(
        Box((glass_len + stile_thickness, 0.040, 0.050)),
        origin=_radial_origin(
            radius=inner_offset + (glass_len + stile_thickness) * 0.5,
            angle=angle,
            z=0.10,
        ),
        material=metal,
        name=f"wing_{index}_bottom_rail",
    )
    part.visual(
        Box((glass_len, 0.012, 2.04)),
        origin=_radial_origin(
            radius=inner_offset + glass_len * 0.5,
            angle=angle,
            z=1.14,
        ),
        material=glass,
        name=f"wing_{index}_glass",
    )
    part.visual(
        Box((stile_thickness, 0.055, 2.18)),
        origin=_radial_origin(
            radius=inner_offset + glass_len + stile_thickness * 0.5,
            angle=angle,
            z=1.14,
        ),
        material=metal,
        name=f"wing_{index}_stile",
    )


def _build_bypass_panel(model: ArticulatedObject, name: str, *, metal, glass):
    panel = model.part(name)

    panel_width = 0.82
    stile_width = 0.050

    panel.visual(
        Box((panel_width, 0.055, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=metal,
        name="bottom_rail",
    )
    panel.visual(
        Box((panel_width, 0.055, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 2.15)),
        material=metal,
        name="top_rail",
    )
    panel.visual(
        Box((stile_width, 0.055, 2.16)),
        origin=Origin(xyz=(-(panel_width * 0.5) + stile_width * 0.5, 0.0, 1.10)),
        material=metal,
        name="left_stile",
    )
    panel.visual(
        Box((stile_width, 0.055, 2.16)),
        origin=Origin(xyz=((panel_width * 0.5) - stile_width * 0.5, 0.0, 1.10)),
        material=metal,
        name="right_stile",
    )
    panel.visual(
        Box((0.72, 0.012, 2.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        material=glass,
        name="panel_glass",
    )
    panel.inertial = Inertial.from_geometry(
        Box((0.82, 0.060, 2.20)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )
    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_revolving_door")

    bronze = model.material("bronze", rgba=(0.33, 0.28, 0.22, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.22, 0.18, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    glass = model.material("glass", rgba=(0.66, 0.82, 0.90, 0.30))

    frame_width = 6.20
    frame_depth = 2.55
    threshold_t = 0.06
    header_t = 0.18
    header_bottom = 2.40
    shell_radius = 1.15
    shell_thickness = 0.026
    track_length = 1.72
    track_center_x = 2.145
    closed_panel_x = 1.695

    frame = model.part("entrance_frame")
    frame.visual(
        Box((frame_width, frame_depth, threshold_t)),
        origin=Origin(xyz=(0.0, 0.0, threshold_t * 0.5)),
        material=dark_bronze,
        name="threshold_base",
    )
    frame.visual(
        Box((frame_width, frame_depth, header_t)),
        origin=Origin(xyz=(0.0, 0.0, header_bottom + header_t * 0.5)),
        material=bronze,
        name="header_canopy",
    )
    frame.visual(
        Box((frame_width, 0.100, 0.240)),
        origin=Origin(xyz=(0.0, frame_depth * 0.5 - 0.050, header_bottom + 0.120)),
        material=bronze,
        name="front_header_fascia",
    )
    frame.visual(
        Box((frame_width, 0.100, 0.240)),
        origin=Origin(xyz=(0.0, -(frame_depth * 0.5 - 0.050), header_bottom + 0.120)),
        material=bronze,
        name="rear_header_fascia",
    )

    floor_ring_mesh = mesh_from_geometry(
        _build_sector_shell(
            radius=1.14,
            thickness=0.16,
            height=0.050,
            start_angle=0.0,
            end_angle=2.0 * math.pi,
            segments=72,
            z0=0.055,
        ),
        "revolving_floor_ring",
    )
    frame.visual(
        floor_ring_mesh,
        material=bronze,
        name="floor_guide_ring",
    )

    crown_ring_mesh = mesh_from_geometry(
        _build_sector_shell(
            radius=1.15,
            thickness=0.18,
            height=0.100,
            start_angle=0.0,
            end_angle=2.0 * math.pi,
            segments=72,
            z0=2.30,
        ),
        "revolving_crown_ring",
    )
    frame.visual(
        crown_ring_mesh,
        material=bronze,
        name="top_crown_ring",
    )

    right_shell_mesh = mesh_from_geometry(
        _build_sector_shell(
            radius=shell_radius,
            thickness=shell_thickness,
            height=2.37,
            start_angle=-math.radians(60.0),
            end_angle=math.radians(60.0),
            segments=32,
            z0=0.05,
        ),
        "drum_shell_right",
    )
    frame.visual(
        right_shell_mesh,
        material=glass,
        name="drum_shell_right",
    )
    left_shell_mesh = mesh_from_geometry(
        _build_sector_shell(
            radius=shell_radius,
            thickness=shell_thickness,
            height=2.37,
            start_angle=math.radians(120.0),
            end_angle=math.radians(240.0),
            segments=32,
            z0=0.05,
        ),
        "drum_shell_left",
    )
    frame.visual(
        left_shell_mesh,
        material=glass,
        name="drum_shell_left",
    )

    mullion_radius = shell_radius
    for idx, angle in enumerate(
        (
            math.radians(60.0),
            math.radians(120.0),
            math.radians(240.0),
            math.radians(300.0),
        )
    ):
        frame.visual(
            Box((0.090, 0.055, 2.38)),
            origin=_radial_origin(radius=mullion_radius, angle=angle, z=1.235),
            material=bronze,
            name=f"opening_mullion_{idx}",
        )

    for side, x in (("left", -1.230), ("right", 1.230)):
        frame.visual(
            Box((0.100, 0.180, 2.50)),
            origin=Origin(xyz=(x, 0.0, 1.280)),
            material=bronze,
            name=f"{side}_inner_post",
        )
    for side, x in (("left", -3.045), ("right", 3.045)):
        frame.visual(
            Box((0.110, 0.220, 2.50)),
            origin=Origin(xyz=(x, 0.0, 1.280)),
            material=bronze,
            name=f"{side}_outer_jamb",
        )

    frame.visual(
        Box((track_length, 0.100, 0.020)),
        origin=Origin(xyz=(-track_center_x, 0.0, 0.070)),
        material=steel,
        name="left_floor_track",
    )
    frame.visual(
        Box((track_length, 0.100, 0.020)),
        origin=Origin(xyz=(track_center_x, 0.0, 0.070)),
        material=steel,
        name="right_floor_track",
    )
    frame.visual(
        Box((track_length, 0.120, 0.100)),
        origin=Origin(xyz=(-track_center_x, 0.0, 2.340)),
        material=bronze,
        name="left_overhead_track",
    )
    frame.visual(
        Box((track_length, 0.120, 0.100)),
        origin=Origin(xyz=(track_center_x, 0.0, 2.340)),
        material=bronze,
        name="right_overhead_track",
    )

    frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, 2.58)),
        mass=900.0,
        origin=Origin(xyz=(0.0, 0.0, 1.29)),
    )

    drum = model.part("revolving_drum")
    drum.visual(
        Cylinder(radius=0.070, length=2.32),
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        material=dark_bronze,
        name="center_shaft",
    )
    drum.visual(
        Cylinder(radius=0.180, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 2.25)),
        material=bronze,
        name="top_hub",
    )
    drum.visual(
        Cylinder(radius=0.160, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=bronze,
        name="bottom_hub",
    )

    for index, angle in enumerate(
        (
            math.pi * 0.5,
            math.pi * 0.5 + 2.0 * math.pi / 3.0,
            math.pi * 0.5 + 4.0 * math.pi / 3.0,
        )
    ):
        _add_revolving_wing(drum, index=index, angle=angle, metal=bronze, glass=glass)

    drum.inertial = Inertial.from_geometry(
        Box((2.00, 2.00, 2.32)),
        mass=250.0,
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
    )

    left_panel = _build_bypass_panel(model, "left_bypass_panel", metal=bronze, glass=glass)
    right_panel = _build_bypass_panel(model, "right_bypass_panel", metal=bronze, glass=glass)

    model.articulation(
        "frame_to_drum",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, threshold_t)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2),
    )
    model.articulation(
        "frame_to_left_bypass",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_panel,
        origin=Origin(xyz=(-closed_panel_x, 0.0, threshold_t)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.7,
            lower=0.0,
            upper=0.88,
        ),
    )
    model.articulation(
        "frame_to_right_bypass",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_panel,
        origin=Origin(xyz=(closed_panel_x, 0.0, threshold_t)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.7,
            lower=0.0,
            upper=0.88,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("entrance_frame")
    drum = object_model.get_part("revolving_drum")
    left_panel = object_model.get_part("left_bypass_panel")
    right_panel = object_model.get_part("right_bypass_panel")

    drum_joint = object_model.get_articulation("frame_to_drum")
    left_slide = object_model.get_articulation("frame_to_left_bypass")
    right_slide = object_model.get_articulation("frame_to_right_bypass")

    ctx.check(
        "drum articulation is vertical continuous rotation",
        drum_joint.axis == (0.0, 0.0, 1.0) and drum_joint.motion_limits is not None,
        details=f"axis={drum_joint.axis}, limits={drum_joint.motion_limits}",
    )
    ctx.check(
        "left bypass panel slides outward toward -X",
        left_slide.axis == (-1.0, 0.0, 0.0),
        details=f"axis={left_slide.axis}",
    )
    ctx.check(
        "right bypass panel slides outward toward +X",
        right_slide.axis == (1.0, 0.0, 0.0),
        details=f"axis={right_slide.axis}",
    )

    ctx.expect_origin_distance(
        drum,
        frame,
        axes="xy",
        max_dist=0.001,
        name="revolving drum stays centered in the entrance frame",
    )

    ctx.expect_gap(
        left_panel,
        frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="left_floor_track",
        min_gap=0.0,
        max_gap=0.02,
        name="left bypass panel rides just above the floor track",
    )
    ctx.expect_gap(
        frame,
        left_panel,
        axis="z",
        positive_elem="left_overhead_track",
        negative_elem="top_rail",
        min_gap=0.03,
        max_gap=0.10,
        name="left bypass panel stays beneath the overhead track",
    )
    ctx.expect_gap(
        right_panel,
        frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="right_floor_track",
        min_gap=0.0,
        max_gap=0.02,
        name="right bypass panel rides just above the floor track",
    )
    ctx.expect_gap(
        frame,
        right_panel,
        axis="z",
        positive_elem="right_overhead_track",
        negative_elem="top_rail",
        min_gap=0.03,
        max_gap=0.10,
        name="right bypass panel stays beneath the overhead track",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    wing_rest = _aabb_center(ctx.part_element_world_aabb(drum, elem="wing_0_stile"))
    with ctx.pose({drum_joint: math.radians(50.0)}):
        wing_rotated = _aabb_center(ctx.part_element_world_aabb(drum, elem="wing_0_stile"))
    ctx.check(
        "revolving drum visibly turns its wings",
        wing_rest is not None
        and wing_rotated is not None
        and math.hypot(wing_rotated[0] - wing_rest[0], wing_rotated[1] - wing_rest[1]) > 0.35,
        details=f"rest={wing_rest}, rotated={wing_rotated}",
    )

    left_rest = ctx.part_world_position(left_panel)
    with ctx.pose({left_slide: 0.88}):
        left_open = ctx.part_world_position(left_panel)
    ctx.check(
        "left bypass panel opens away from the drum",
        left_rest is not None and left_open is not None and left_open[0] < left_rest[0] - 0.5,
        details=f"rest={left_rest}, open={left_open}",
    )

    right_rest = ctx.part_world_position(right_panel)
    with ctx.pose({right_slide: 0.88}):
        right_open = ctx.part_world_position(right_panel)
    ctx.check(
        "right bypass panel opens away from the drum",
        right_rest is not None and right_open is not None and right_open[0] > right_rest[0] + 0.5,
        details=f"rest={right_rest}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
