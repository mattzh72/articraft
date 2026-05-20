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


STEEL = Material("mat_dark_powder_coated_steel", rgba=(0.035, 0.042, 0.048, 1.0))
BLACK = Material("mat_black_hand_finish", rgba=(0.005, 0.005, 0.004, 1.0))
CONCRETE = Material("mat_pale_cast_concrete", rgba=(0.58, 0.56, 0.52, 1.0))
DIAL = Material("mat_warm_white_clock_dial", rgba=(0.92, 0.90, 0.84, 1.0))
MARKER = Material("mat_charcoal_hour_markers", rgba=(0.02, 0.02, 0.018, 1.0))


def _hollow_square_tube(outer: float, wall: float, height: float) -> cq.Workplane:
    """Open-ended square HSS tube, authored in meters."""
    inner = outer - 2.0 * wall
    return cq.Workplane("XY").rect(outer, outer).rect(inner, inner).extrude(height)


def _dial_rpy(normal: tuple[float, float, float]) -> tuple[float, float, float]:
    if normal[1] > 0.5:
        return (-math.pi / 2.0, 0.0, 0.0)
    if normal[1] < -0.5:
        return (math.pi / 2.0, 0.0, 0.0)
    if normal[0] > 0.5:
        return (0.0, math.pi / 2.0, 0.0)
    return (0.0, -math.pi / 2.0, 0.0)


def _face_axes(yaw: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return the face-local +X tangent and +Y outward normal in world axes."""
    return (
        (math.cos(yaw), math.sin(yaw), 0.0),
        (-math.sin(yaw), math.cos(yaw), 0.0),
    )


def _add_hand_visuals(
    hand,
    *,
    length: float,
    width: float,
    tail: float,
    thickness: float,
    y_offset: float,
    angle: float,
    hub_radius: float,
    hub_length: float,
) -> None:
    # In every hand's local frame, +Y is the axis protruding out of the clock
    # face and +Z is twelve-o'clock.  The visual angle lets the rest pose read
    # as a familiar 10:10 display while the revolute joint still rotates about
    # the face-center shaft.
    hand.visual(
        Box((width, thickness, length)),
        origin=Origin(
            xyz=(math.sin(angle) * length / 2.0, y_offset, math.cos(angle) * length / 2.0),
            rpy=(0.0, angle, 0.0),
        ),
        material=BLACK,
        name="long_blade",
    )
    hand.visual(
        Box((width * 0.60, thickness, tail)),
        origin=Origin(
            xyz=(-math.sin(angle) * tail / 2.0, y_offset, -math.cos(angle) * tail / 2.0),
            rpy=(0.0, angle, 0.0),
        ),
        material=BLACK,
        name="counter_tail",
    )
    hand.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(xyz=(0.0, y_offset, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=BLACK,
        name="center_hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contemporary_civic_clock_tower")

    # Overall dimensions are civic-street scale, in meters.
    plinth_lower_h = 0.28
    plinth_upper_h = 0.25
    plate_h = 0.04
    shaft_outer = 0.62
    shaft_wall = 0.08
    shaft_h = 4.20
    shaft_z0 = plinth_lower_h + plinth_upper_h + plate_h

    face_w = 0.56
    face_h = 0.72
    face_depth = 0.28
    face_z = 4.12
    dial_radius = 0.23
    dial_len = 0.024
    marker_len = 0.012
    hand_axis_clearance = 0.022

    tower = model.part("tower")

    tower.visual(
        Box((1.40, 1.40, plinth_lower_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_lower_h / 2.0)),
        material=CONCRETE,
        name="lower_plinth",
    )
    tower.visual(
        Box((0.90, 0.90, plinth_upper_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_lower_h + plinth_upper_h / 2.0)),
        material=CONCRETE,
        name="upper_plinth",
    )
    tower.visual(
        Box((0.78, 0.78, plate_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_lower_h + plinth_upper_h + plate_h / 2.0)),
        material=STEEL,
        name="base_plate",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Cylinder(radius=0.026, length=0.055),
                origin=Origin(xyz=(sx * 0.29, sy * 0.29, shaft_z0 + 0.0275)),
                material=STEEL,
                name=f"anchor_bolt_{int((sx + 1) / 2)}_{int((sy + 1) / 2)}",
            )

    tower.visual(
        mesh_from_cadquery(
            _hollow_square_tube(shaft_outer, shaft_wall, shaft_h),
            "square_hollow_steel_tube",
            tolerance=0.0015,
            angular_tolerance=0.12,
        ),
        origin=Origin(xyz=(0.0, 0.0, shaft_z0)),
        material=STEEL,
        name="hollow_tube_shaft",
    )

    face_specs = (
        (0, 0.0),
        (1, -math.pi / 2.0),
        (2, math.pi),
        (3, math.pi / 2.0),
    )

    shaft_half = shaft_outer / 2.0
    box_front = shaft_half + face_depth
    dial_center_depth = box_front + dial_len / 2.0 - 0.002
    marker_center_depth = box_front + dial_len + marker_len / 2.0 - 0.004
    hand_origin_depth = box_front + dial_len + marker_len + hand_axis_clearance
    hour_hub_length = 0.012
    bearing_outer_depth = hand_origin_depth - hour_hub_length / 2.0
    bearing_inner_depth = box_front + dial_len - 0.004
    bearing_length = bearing_outer_depth - bearing_inner_depth

    for face_index, yaw in face_specs:
        x_axis, normal = _face_axes(yaw)
        nx, ny, _ = normal

        box_center = (nx * (shaft_half + face_depth / 2.0), ny * (shaft_half + face_depth / 2.0), face_z)
        box_size = (face_depth, face_w, face_h) if abs(nx) > 0.5 else (face_w, face_depth, face_h)
        tower.visual(
            Box(box_size),
            origin=Origin(xyz=box_center),
            material=STEEL,
            name=f"face_box_{face_index}",
        )

        dial_center = (nx * dial_center_depth, ny * dial_center_depth, face_z)
        tower.visual(
            Cylinder(radius=dial_radius, length=dial_len),
            origin=Origin(xyz=dial_center, rpy=_dial_rpy(normal)),
            material=DIAL,
            name=f"dial_{face_index}",
        )

        for mark in range(12):
            theta = mark * math.tau / 12.0
            r = dial_radius * 0.82
            radius = 0.014 if mark % 3 == 0 else 0.009
            lx = math.sin(theta) * r
            lz = math.cos(theta) * r
            mark_center = (
                normal[0] * marker_center_depth + x_axis[0] * lx,
                normal[1] * marker_center_depth + x_axis[1] * lx,
                face_z + lz,
            )
            tower.visual(
                Cylinder(radius=radius, length=marker_len),
                origin=Origin(xyz=mark_center, rpy=_dial_rpy(normal)),
                material=MARKER,
                name=f"hour_marker_{face_index}_{mark}",
            )

        bearing_center_depth = bearing_inner_depth + bearing_length / 2.0
        tower.visual(
            Cylinder(radius=0.021, length=bearing_length),
            origin=Origin(
                xyz=(normal[0] * bearing_center_depth, normal[1] * bearing_center_depth, face_z),
                rpy=_dial_rpy(normal),
            ),
            material=MARKER,
            name=f"center_bearing_{face_index}",
        )

        joint_xyz = (normal[0] * hand_origin_depth, normal[1] * hand_origin_depth, face_z)

        hour_hand = model.part(f"face_{face_index}_hour_hand")
        _add_hand_visuals(
            hour_hand,
            length=0.135,
            width=0.030,
            tail=0.040,
            thickness=0.010,
            y_offset=0.000,
            angle=math.radians(-50.0),
            hub_radius=0.034,
            hub_length=hour_hub_length,
        )
        minute_hand = model.part(f"face_{face_index}_minute_hand")
        _add_hand_visuals(
            minute_hand,
            length=0.198,
            width=0.018,
            tail=0.052,
            thickness=0.008,
            y_offset=hour_hub_length,
            angle=math.radians(50.0),
            hub_radius=0.027,
            hub_length=0.012,
        )

        for label, child, velocity in (
            ("hour", hour_hand, 0.15),
            ("minute", minute_hand, 1.80),
        ):
            model.articulation(
                f"face_{face_index}_{label}_joint",
                ArticulationType.REVOLUTE,
                parent=tower,
                child=child,
                origin=Origin(xyz=joint_xyz, rpy=(0.0, 0.0, yaw)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=0.20,
                    velocity=velocity,
                    lower=0.0,
                    upper=math.tau,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.check(
        "four clock faces",
        all(object_model.get_part("tower").get_visual(f"dial_{i}") is not None for i in range(4)),
        details="The tower should carry one dial on each of the four upper sides.",
    )
    ctx.check(
        "eight concentric hand joints",
        len(object_model.articulations) == 8,
        details=f"Expected 8 hand joints, found {len(object_model.articulations)}.",
    )

    tower = object_model.get_part("tower")
    for i in range(4):
        hour = object_model.get_part(f"face_{i}_hour_hand")
        minute = object_model.get_part(f"face_{i}_minute_hand")
        ctx.expect_origin_distance(
            hour,
            minute,
            axes="xyz",
            max_dist=0.0001,
            name=f"face {i} hands share center",
        )

        ctx.expect_contact(
            tower,
            hour,
            elem_a=f"center_bearing_{i}",
            elem_b="center_hub",
            contact_tol=0.0005,
            name=f"face {i} hour hand rides on bearing",
        )
        ctx.expect_contact(
            hour,
            minute,
            elem_a="center_hub",
            elem_b="center_hub",
            contact_tol=0.0005,
            name=f"face {i} minute hub stacks on hour hub",
        )

    return ctx.report()


object_model = build_object_model()
