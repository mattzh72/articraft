from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hip_roof_mesh(width: float, depth: float, z_base: float, z_ridge: float, ridge: float) -> MeshGeometry:
    """Closed, solid-looking rectangular hip roof with a short ridge."""
    roof = MeshGeometry()
    x0, x1 = -width / 2.0, width / 2.0
    y0, y1 = -depth / 2.0, depth / 2.0
    r0, r1 = -ridge / 2.0, ridge / 2.0

    p0 = roof.add_vertex(x0, y0, z_base)
    p1 = roof.add_vertex(x1, y0, z_base)
    p2 = roof.add_vertex(x1, y1, z_base)
    p3 = roof.add_vertex(x0, y1, z_base)
    w = roof.add_vertex(r0, 0.0, z_ridge)
    e = roof.add_vertex(r1, 0.0, z_ridge)

    # Long trapezoidal roof planes.
    roof.add_face(p0, p1, e)
    roof.add_face(p0, e, w)
    roof.add_face(p3, w, e)
    roof.add_face(p3, e, p2)
    # Triangular hipped ends.
    roof.add_face(p1, p2, e)
    roof.add_face(p0, w, p3)
    # Flat underside so the roof reads as a physical cap rather than a paper plane.
    roof.add_face(p0, p3, p2)
    roof.add_face(p0, p2, p1)
    return roof


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_frame_clock_tower")

    timber = model.material("dark_oak", rgba=(0.23, 0.12, 0.055, 1.0))
    plaster = model.material("lime_plaster", rgba=(0.78, 0.70, 0.56, 1.0))
    stone = model.material("weathered_stone", rgba=(0.44, 0.43, 0.40, 1.0))
    clock_white = model.material("painted_clock_face", rgba=(0.93, 0.90, 0.80, 1.0))
    clock_black = model.material("black_enamel", rgba=(0.015, 0.012, 0.010, 1.0))
    roof_mat = model.material("slate_roof", rgba=(0.10, 0.13, 0.15, 1.0))
    hand_mat = model.material("blued_steel_hands", rgba=(0.02, 0.025, 0.035, 1.0))

    tower = model.part("tower")

    body_x = 1.30
    body_y = 1.05
    shaft_height = 4.95
    half_x = body_x / 2.0
    half_y = body_y / 2.0

    tower.visual(
        Box((1.44, 1.20, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=stone,
        name="stone_plinth",
    )
    tower.visual(
        Box((body_x, body_y, shaft_height)),
        origin=Origin(xyz=(0.0, 0.0, shaft_height / 2.0)),
        material=plaster,
        name="plaster_infill",
    )

    # Post-and-beam timber frame: four corner posts, story beams, top plate,
    # and diagonal braces that are slightly embedded into the infill panels.
    post_size = 0.14
    for ix, x in enumerate((-half_x + post_size / 2.0, half_x - post_size / 2.0)):
        for iy, y in enumerate((-half_y + post_size / 2.0, half_y - post_size / 2.0)):
            tower.visual(
                Box((post_size, post_size, shaft_height + 0.05)),
                origin=Origin(xyz=(x, y, shaft_height / 2.0)),
                material=timber,
                name=f"corner_post_{ix}_{iy}",
            )

    band_zs = (0.42, 2.35, 3.55, 4.85)
    for zi, z in enumerate(band_zs):
        tower.visual(
            Box((body_x + 0.10, 0.12, 0.14)),
            origin=Origin(xyz=(0.0, half_y + 0.025, z)),
            material=timber,
            name=f"front_beam_{zi}",
        )
        tower.visual(
            Box((body_x + 0.10, 0.12, 0.14)),
            origin=Origin(xyz=(0.0, -half_y - 0.025, z)),
            material=timber,
            name=f"rear_beam_{zi}",
        )
        tower.visual(
            Box((0.12, body_y + 0.10, 0.14)),
            origin=Origin(xyz=(half_x + 0.025, 0.0, z)),
            material=timber,
            name=f"side_beam_{zi}_0",
        )
        tower.visual(
            Box((0.12, body_y + 0.10, 0.14)),
            origin=Origin(xyz=(-half_x - 0.025, 0.0, z)),
            material=timber,
            name=f"side_beam_{zi}_1",
        )

    for side, y in (("front", half_y + 0.028), ("rear", -half_y - 0.028)):
        tower.visual(
            Box((0.12, 0.11, 3.08)),
            origin=Origin(xyz=(0.0, y, 1.90)),
            material=timber,
            name=f"{side}_center_post_lower",
        )
        tower.visual(
            Box((0.12, 0.11, 0.22)),
            origin=Origin(xyz=(0.0, y, 4.72)),
            material=timber,
            name=f"{side}_center_post_upper",
        )
        tower.visual(
            Box((1.92, 0.105, 0.105)),
            origin=Origin(xyz=(0.0, y, 1.42), rpy=(0.0, -1.08, 0.0)),
            material=timber,
            name=f"{side}_diagonal",
        )

    for side, x in (("east", half_x + 0.028), ("west", -half_x - 0.028)):
        tower.visual(
            Box((0.11, 0.12, 3.08)),
            origin=Origin(xyz=(x, 0.0, 1.90)),
            material=timber,
            name=f"{side}_center_post_lower",
        )
        tower.visual(
            Box((0.11, 0.12, 0.22)),
            origin=Origin(xyz=(x, 0.0, 4.72)),
            material=timber,
            name=f"{side}_center_post_upper",
        )
        tower.visual(
            Box((0.105, 1.72, 0.105)),
            origin=Origin(xyz=(x, 0.0, 1.42), rpy=(1.03, 0.0, 0.0)),
            material=timber,
            name=f"{side}_diagonal",
        )

    tower.visual(
        mesh_from_geometry(_hip_roof_mesh(1.68, 1.42, shaft_height - 0.01, 5.62, 0.56), "hip_roof"),
        origin=Origin(),
        material=roof_mat,
        name="hip_roof",
    )

    face_z = 4.18
    face_size = 0.62
    face_thickness = 0.030
    face_embed = 0.002
    bushing_len = 0.018
    hub_radius = 0.045
    hand_thickness = 0.008
    minute_angle = math.radians(58.0)
    hour_angle = math.radians(-52.0)

    face_specs = [
        {"i": 0, "name": "front", "normal": (0.0, 1.0, 0.0), "half": half_y, "axis": "y", "sign": 1.0},
        {"i": 1, "name": "rear", "normal": (0.0, -1.0, 0.0), "half": half_y, "axis": "y", "sign": -1.0},
        {"i": 2, "name": "east", "normal": (1.0, 0.0, 0.0), "half": half_x, "axis": "x", "sign": 1.0},
        {"i": 3, "name": "west", "normal": (-1.0, 0.0, 0.0), "half": half_x, "axis": "x", "sign": -1.0},
    ]

    def cylinder_rpy(axis_name: str, sign: float) -> tuple[float, float, float]:
        if axis_name == "y":
            return (-math.pi / 2.0 if sign > 0.0 else math.pi / 2.0, 0.0, 0.0)
        return (0.0, math.pi / 2.0 if sign > 0.0 else -math.pi / 2.0, 0.0)

    def add_face_details(spec: dict[str, object]) -> tuple[tuple[float, float, float], tuple[float, float, float], str]:
        idx = int(spec["i"])
        normal = spec["normal"]  # type: ignore[assignment]
        axis_name = str(spec["axis"])
        sign = float(spec["sign"])
        half = float(spec["half"])
        panel_center = sign * (half + face_thickness / 2.0 - face_embed)
        face_outer = half + face_thickness - face_embed
        hub_offset = face_outer + bushing_len

        if axis_name == "y":
            panel_size = (face_size, face_thickness, face_size)
            panel_xyz = (0.0, panel_center, face_z)
            bushing_xyz = (0.0, sign * (face_outer + bushing_len / 2.0), face_z)
            hub_xyz = (0.0, sign * hub_offset, face_z)
            border_specs = [
                ((face_size, 0.006, 0.026), (0.0, sign * (face_outer + 0.0025), face_z + face_size / 2.0 - 0.013), (0.0, 0.0, 0.0)),
                ((face_size, 0.006, 0.026), (0.0, sign * (face_outer + 0.0025), face_z - face_size / 2.0 + 0.013), (0.0, 0.0, 0.0)),
                ((0.026, 0.006, face_size), (face_size / 2.0 - 0.013, sign * (face_outer + 0.0025), face_z), (0.0, 0.0, 0.0)),
                ((0.026, 0.006, face_size), (-face_size / 2.0 + 0.013, sign * (face_outer + 0.0025), face_z), (0.0, 0.0, 0.0)),
            ]
        else:
            panel_size = (face_thickness, face_size, face_size)
            panel_xyz = (panel_center, 0.0, face_z)
            bushing_xyz = (sign * (face_outer + bushing_len / 2.0), 0.0, face_z)
            hub_xyz = (sign * hub_offset, 0.0, face_z)
            border_specs = [
                ((0.006, face_size, 0.026), (sign * (face_outer + 0.0025), 0.0, face_z + face_size / 2.0 - 0.013), (0.0, 0.0, 0.0)),
                ((0.006, face_size, 0.026), (sign * (face_outer + 0.0025), 0.0, face_z - face_size / 2.0 + 0.013), (0.0, 0.0, 0.0)),
                ((0.006, 0.026, face_size), (sign * (face_outer + 0.0025), face_size / 2.0 - 0.013, face_z), (0.0, 0.0, 0.0)),
                ((0.006, 0.026, face_size), (sign * (face_outer + 0.0025), -face_size / 2.0 + 0.013, face_z), (0.0, 0.0, 0.0)),
            ]

        tower.visual(Box(panel_size), origin=Origin(xyz=panel_xyz), material=clock_white, name=f"clock_face_{idx}")
        for bi, (size, xyz, rpy) in enumerate(border_specs):
            tower.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=clock_black, name=f"face_border_{idx}_{bi}")

        tick_r = face_size * 0.39
        for tick in range(12):
            a = 2.0 * math.pi * tick / 12.0
            major = tick % 3 == 0
            tick_len = 0.052 if major else 0.035
            if axis_name == "y":
                tx = tick_r * math.sin(a)
                tz = face_z + tick_r * math.cos(a)
                size = (tick_len, 0.007, tick_len)
                xyz = (tx, sign * (face_outer + 0.0025), tz)
            else:
                ty = tick_r * math.sin(a)
                tz = face_z + tick_r * math.cos(a)
                size = (0.007, tick_len, tick_len)
                xyz = (sign * (face_outer + 0.0025), ty, tz)
            tower.visual(Box(size), origin=Origin(xyz=xyz), material=clock_black, name=f"hour_mark_{idx}_{tick}")

        tower.visual(
            Cylinder(radius=hub_radius * 0.72, length=bushing_len),
            origin=Origin(xyz=bushing_xyz, rpy=cylinder_rpy(axis_name, sign)),
            material=clock_black,
            name=f"face_bushing_{idx}",
        )
        return hub_xyz, normal, axis_name

    def add_hand(
        face_idx: int,
        kind: str,
        hub_xyz: tuple[float, float, float],
        normal: tuple[float, float, float],
        axis_name: str,
        sign: float,
        angle: float,
        length: float,
        width: float,
        layer: int,
    ) -> None:
        part_name = f"{kind}_hand_{face_idx}"
        hand = model.part(part_name)
        normal_offset = hand_thickness * (layer + 0.5)
        if axis_name == "y":
            dx = math.sin(angle) * length / 2.0
            dz = math.cos(angle) * length / 2.0
            bar_origin = Origin(xyz=(dx, sign * normal_offset, dz), rpy=(0.0, angle, 0.0))
            bar_size = (width, hand_thickness, length)
            hub_origin = Origin(xyz=(0.0, sign * normal_offset, 0.0), rpy=cylinder_rpy(axis_name, sign))
        else:
            dy = math.sin(angle) * length / 2.0
            dz = math.cos(angle) * length / 2.0
            bar_origin = Origin(xyz=(sign * normal_offset, dy, dz), rpy=(-angle, 0.0, 0.0))
            bar_size = (hand_thickness, width, length)
            hub_origin = Origin(xyz=(sign * normal_offset, 0.0, 0.0), rpy=cylinder_rpy(axis_name, sign))

        hand.visual(Box(bar_size), origin=bar_origin, material=hand_mat, name="pointer")
        hand.visual(Cylinder(radius=hub_radius, length=hand_thickness), origin=hub_origin, material=hand_mat, name="hub")
        model.articulation(
            f"tower_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=tower,
            child=hand,
            origin=Origin(xyz=hub_xyz),
            axis=normal,
            motion_limits=MotionLimits(effort=0.1, velocity=1.5, lower=0.0, upper=2.0 * math.pi),
        )

    for spec in face_specs:
        hub_xyz, normal, axis_name = add_face_details(spec)
        idx = int(spec["i"])
        sign = float(spec["sign"])
        add_hand(idx, "hour", hub_xyz, normal, axis_name, sign, hour_angle, 0.185, 0.034, 0)
        add_hand(idx, "minute", hub_xyz, normal, axis_name, sign, minute_angle, 0.270, 0.022, 1)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    ctx.check(
        "four square clock faces",
        all(tower.get_visual(f"clock_face_{i}") is not None for i in range(4)),
        details="The upper story should have one clock face on each side.",
    )

    for i in range(4):
        hour_joint = object_model.get_articulation(f"tower_to_hour_hand_{i}")
        minute_joint = object_model.get_articulation(f"tower_to_minute_hand_{i}")
        hour_part = object_model.get_part(f"hour_hand_{i}")
        minute_part = object_model.get_part(f"minute_hand_{i}")

        ctx.check(
            f"face {i} hands are concentric",
            hour_joint.origin.xyz == minute_joint.origin.xyz and hour_joint.axis == minute_joint.axis,
            details=f"hour origin={hour_joint.origin.xyz}, minute origin={minute_joint.origin.xyz}",
        )
        ctx.check(
            f"face {i} hand joints are full revolutes",
            hour_joint.articulation_type == ArticulationType.REVOLUTE
            and minute_joint.articulation_type == ArticulationType.REVOLUTE
            and hour_joint.motion_limits is not None
            and minute_joint.motion_limits is not None
            and hour_joint.motion_limits.upper >= 2.0 * math.pi - 1e-6
            and minute_joint.motion_limits.upper >= 2.0 * math.pi - 1e-6,
            details="Both hour and minute hands should have a full sweep around the hub.",
        )
        ctx.expect_contact(
            hour_part,
            tower,
            elem_a="hub",
            elem_b=f"face_bushing_{i}",
            contact_tol=0.001,
            name=f"face {i} hour hand is seated on the bushing",
        )
        ctx.expect_contact(
            hour_part,
            minute_part,
            elem_a="hub",
            elem_b="hub",
            contact_tol=0.001,
            name=f"face {i} hand hubs are stacked concentrically",
        )

    return ctx.report()


object_model = build_object_model()
