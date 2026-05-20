from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TAU = 2.0 * math.pi


def _annular_disc_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str):
    """A thin circular steel rim, modeled as a real annular plate."""
    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )
    return mesh_from_cadquery(ring, name, tolerance=0.0015, angular_tolerance=0.08)


def _origin_on_normal(normal: tuple[float, float, float], distance: float, z: float) -> tuple[float, float, float]:
    return (normal[0] * distance, normal[1] * distance, z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_steel_civic_clock_tower")

    concrete = model.material("pale_cast_concrete", rgba=(0.62, 0.60, 0.55, 1.0))
    steel = model.material("brushed_dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.52, 0.80, 0.95, 0.34))
    clock_face = model.material("frosted_clock_glass", rgba=(0.93, 0.95, 0.93, 0.72))
    hand_material = model.material("blackened_steel_hands", rgba=(0.01, 0.01, 0.012, 1.0))

    tower = model.part("tower")

    # A low, civic-scale concrete plinth with a smaller top pad receiving the shaft.
    tower.visual(
        Box((1.70, 1.70, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=concrete,
        name="lower_plinth",
    )
    tower.visual(
        Box((1.24, 1.24, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=concrete,
        name="upper_plinth",
    )

    # Four panes plus steel posts/bands make a genuinely hollow square glass shaft.
    shaft_half = 0.425
    post_size = 0.070
    shaft_bottom = 0.50
    shaft_height = 5.05
    shaft_center_z = shaft_bottom + shaft_height / 2.0

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Box((post_size, post_size, shaft_height)),
                origin=Origin(xyz=(sx * shaft_half, sy * shaft_half, shaft_center_z)),
                material=steel,
                name=f"corner_post_{int(sx > 0)}_{int(sy > 0)}",
            )

    pane_height = 4.86
    pane_center_z = shaft_bottom + 0.09 + pane_height / 2.0
    pane_thickness = 0.035
    pane_span = 0.82
    for sx in (-1.0, 1.0):
        tower.visual(
            Box((pane_thickness, pane_span, pane_height)),
            origin=Origin(xyz=(sx * shaft_half, 0.0, pane_center_z)),
            material=glass,
            name=f"glass_pane_x_{int(sx > 0)}",
        )
    for sy in (-1.0, 1.0):
        tower.visual(
            Box((pane_span, pane_thickness, pane_height)),
            origin=Origin(xyz=(0.0, sy * shaft_half, pane_center_z)),
            material=glass,
            name=f"glass_pane_y_{int(sy > 0)}",
        )

    for z, band_name in ((0.56, "lower_band"), (5.50, "upper_band")):
        tower.visual(
            Box((0.96, 0.080, 0.080)),
            origin=Origin(xyz=(0.0, shaft_half, z)),
            material=steel,
            name=f"{band_name}_y_pos",
        )
        tower.visual(
            Box((0.96, 0.080, 0.080)),
            origin=Origin(xyz=(0.0, -shaft_half, z)),
            material=steel,
            name=f"{band_name}_y_neg",
        )
        tower.visual(
            Box((0.080, 0.96, 0.080)),
            origin=Origin(xyz=(shaft_half, 0.0, z)),
            material=steel,
            name=f"{band_name}_x_pos",
        )
        tower.visual(
            Box((0.080, 0.96, 0.080)),
            origin=Origin(xyz=(-shaft_half, 0.0, z)),
            material=steel,
            name=f"{band_name}_x_neg",
        )

    # Four identical cantilevered clock faces around the upper shaft.
    rim_mesh = _annular_disc_mesh(0.435, 0.390, 0.070, "steel_clock_rim")
    face_radius = 0.405
    face_offset = 0.700
    clock_z = 4.70
    rim_thickness = 0.070
    face_front_offset = face_offset + rim_thickness / 2.0
    joint_offset = face_front_offset + 0.030
    support_length = 0.270
    disc_rear_offset = face_offset - rim_thickness / 2.0
    support_center = (shaft_half + disc_rear_offset) / 2.0

    faces = (
        {"i": 0, "normal": (1.0, 0.0, 0.0), "rpy": (0.0, math.pi / 2.0, 0.0), "axis": "x", "sign": 1.0},
        {"i": 1, "normal": (0.0, 1.0, 0.0), "rpy": (-math.pi / 2.0, 0.0, 0.0), "axis": "y", "sign": 1.0},
        {"i": 2, "normal": (-1.0, 0.0, 0.0), "rpy": (0.0, -math.pi / 2.0, 0.0), "axis": "x", "sign": -1.0},
        {"i": 3, "normal": (0.0, -1.0, 0.0), "rpy": (math.pi / 2.0, 0.0, 0.0), "axis": "y", "sign": -1.0},
    )

    for face in faces:
        i = face["i"]
        normal = face["normal"]
        rpy = face["rpy"]
        center = _origin_on_normal(normal, face_offset, clock_z)

        tower.visual(
            Cylinder(radius=face_radius, length=rim_thickness),
            origin=Origin(xyz=center, rpy=rpy),
            material=clock_face,
            name=f"face_{i}_disc",
        )
        tower.visual(
            rim_mesh,
            origin=Origin(xyz=center, rpy=rpy),
            material=steel,
            name=f"face_{i}_rim",
        )

        for dz, label, radius in ((0.0, "center", 0.034), (0.22, "upper", 0.018), (-0.22, "lower", 0.018)):
            tower.visual(
                Cylinder(radius=radius, length=support_length),
                origin=Origin(
                    xyz=_origin_on_normal(normal, support_center, clock_z + dz),
                    rpy=rpy,
                ),
                material=steel,
                name=f"face_{i}_{label}_standoff",
            )

        # Raised minimalist hour ticks, mounted on the front of each disc.
        marker_thick = 0.008
        marker_embed = 0.003
        marker_normal_distance = face_front_offset + marker_thick / 2.0 - marker_embed
        if face["axis"] == "x":
            tick_center_base = (
                normal[0] * marker_normal_distance,
                0.0,
                clock_z,
            )
            tick_specs = (
                ("tick_12", (marker_thick, 0.038, 0.100), (tick_center_base[0], 0.0, clock_z + 0.315)),
                ("tick_6", (marker_thick, 0.038, 0.100), (tick_center_base[0], 0.0, clock_z - 0.315)),
                ("tick_3", (marker_thick, 0.100, 0.038), (tick_center_base[0], 0.315, clock_z)),
                ("tick_9", (marker_thick, 0.100, 0.038), (tick_center_base[0], -0.315, clock_z)),
            )
        else:
            tick_center_base = (
                0.0,
                normal[1] * marker_normal_distance,
                clock_z,
            )
            tick_specs = (
                ("tick_12", (0.038, marker_thick, 0.100), (0.0, tick_center_base[1], clock_z + 0.315)),
                ("tick_6", (0.038, marker_thick, 0.100), (0.0, tick_center_base[1], clock_z - 0.315)),
                ("tick_3", (0.100, marker_thick, 0.038), (0.315, tick_center_base[1], clock_z)),
                ("tick_9", (0.100, marker_thick, 0.038), (-0.315, tick_center_base[1], clock_z)),
            )

        for label, size, xyz in tick_specs:
            tower.visual(Box(size), origin=Origin(xyz=xyz), material=steel, name=f"face_{i}_{label}")

        tower.visual(
            Cylinder(radius=0.052, length=0.020),
            origin=Origin(
                xyz=_origin_on_normal(normal, face_front_offset + 0.007, clock_z),
                rpy=rpy,
            ),
            material=steel,
            name=f"face_{i}_pivot_boss",
        )

        # Each face receives separate concentric hour and minute revolute joints.
        joint_xyz = _origin_on_normal(normal, joint_offset, clock_z)
        axis = normal
        axis_key = face["axis"]

        for hand_kind, hand_len, hand_width, layer_offset, cap_radius, speed in (
            ("hour", 0.245, 0.050, -0.008, 0.038, 0.50),
            ("minute", 0.335, 0.030, 0.010, 0.030, 1.00),
        ):
            hand = model.part(f"{hand_kind}_hand_{i}")
            normal_thickness = 0.010
            local_offset = (
                normal[0] * layer_offset,
                normal[1] * layer_offset,
                hand_len / 2.0,
            )
            if axis_key == "x":
                hand_size = (normal_thickness, hand_width, hand_len)
            else:
                hand_size = (hand_width, normal_thickness, hand_len)
            hand.visual(
                Box(hand_size),
                origin=Origin(xyz=local_offset),
                material=hand_material,
                name="blade",
            )
            hand.visual(
                Cylinder(radius=cap_radius, length=normal_thickness),
                origin=Origin(
                    xyz=(normal[0] * layer_offset, normal[1] * layer_offset, 0.0),
                    rpy=rpy,
                ),
                material=hand_material,
                name="hub",
            )
            if hand_kind == "minute":
                hand.visual(
                    Cylinder(radius=0.012, length=0.008),
                    origin=Origin(
                        xyz=(normal[0] * 0.001, normal[1] * 0.001, 0.0),
                        rpy=rpy,
                    ),
                    material=hand_material,
                    name="spacer",
                )
            model.articulation(
                f"face_{i}_{hand_kind}_joint",
                ArticulationType.REVOLUTE,
                parent=tower,
                child=hand,
                origin=Origin(xyz=joint_xyz),
                axis=axis,
                motion_limits=MotionLimits(effort=0.8, velocity=speed, lower=0.0, upper=TAU),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")

    expected_axes = {
        0: (1.0, 0.0, 0.0),
        1: (0.0, 1.0, 0.0),
        2: (-1.0, 0.0, 0.0),
        3: (0.0, -1.0, 0.0),
    }

    for i, normal in expected_axes.items():
        hour = object_model.get_part(f"hour_hand_{i}")
        minute = object_model.get_part(f"minute_hand_{i}")
        hour_joint = object_model.get_articulation(f"face_{i}_hour_joint")
        minute_joint = object_model.get_articulation(f"face_{i}_minute_joint")

        ctx.check(
            f"face {i} has concentric revolute joints",
            hour_joint.articulation_type == ArticulationType.REVOLUTE
            and minute_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in hour_joint.axis) == normal
            and tuple(round(v, 6) for v in minute_joint.axis) == normal,
            details=f"hour={hour_joint.axis}, minute={minute_joint.axis}",
        )
        ctx.expect_origin_distance(
            hour,
            minute,
            axes="xyz",
            max_dist=0.0005,
            name=f"face {i} hand origins are concentric",
        )

        ctx.expect_contact(
            hour,
            tower,
            elem_a="hub",
            elem_b=f"face_{i}_pivot_boss",
            contact_tol=0.001,
            name=f"face {i} hour hub seats on pivot boss",
        )
        ctx.expect_contact(
            minute,
            hour,
            elem_a="spacer",
            elem_b="hub",
            contact_tol=0.001,
            name=f"face {i} minute spacer seats on hour hub",
        )

    minute_0 = object_model.get_part("minute_hand_0")
    joint_0 = object_model.get_articulation("face_0_minute_joint")
    before = ctx.part_world_aabb(minute_0)
    with ctx.pose({joint_0: math.pi / 2.0}):
        after = ctx.part_world_aabb(minute_0)
    ctx.check(
        "minute hand rotates in clock face plane",
        before is not None
        and after is not None
        and (before[1][2] - before[0][2]) > 0.30
        and (after[1][2] - after[0][2]) < 0.08
        and (after[1][1] - after[0][1]) > 0.30,
        details=f"before={before}, after={after}",
    )

    return ctx.report()


object_model = build_object_model()
