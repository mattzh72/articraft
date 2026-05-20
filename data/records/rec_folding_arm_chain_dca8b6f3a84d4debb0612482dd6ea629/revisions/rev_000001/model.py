from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _signed_area(profile: list[tuple[float, float]]) -> float:
    area = 0.0
    for i, (x0, y0) in enumerate(profile):
        x1, y1 = profile[(i + 1) % len(profile)]
        area += x0 * y1 - x1 * y0
    return 0.5 * area


def _ccw(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return profile if _signed_area(profile) > 0.0 else list(reversed(profile))


def _cw(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return profile if _signed_area(profile) < 0.0 else list(reversed(profile))


def _circle_profile(
    center: tuple[float, float],
    radius: float,
    *,
    segments: int = 36,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    cx, cy = center
    pts = [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    return _cw(pts) if clockwise else _ccw(pts)


def _capsule_profile(
    length: float,
    radius: float,
    *,
    segments_per_end: int = 20,
) -> list[tuple[float, float]]:
    """Rounded dog-bone outline with hinge centers at x=0 and x=length."""
    pts: list[tuple[float, float]] = []
    for i in range(segments_per_end + 1):
        theta = math.pi / 2.0 - math.pi * i / segments_per_end
        pts.append((length + radius * math.cos(theta), radius * math.sin(theta)))
    for i in range(segments_per_end + 1):
        theta = -math.pi / 2.0 - math.pi * i / segments_per_end
        pts.append((radius * math.cos(theta), radius * math.sin(theta)))
    return _ccw(pts)


def _translated(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_folding_arm_chain")

    dark_anodized = model.material("dark_anodized", rgba=(0.05, 0.06, 0.065, 1.0))
    link_metal = model.material("brushed_link_metal", rgba=(0.55, 0.57, 0.56, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.82, 0.80, 0.74, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    plate_length = 0.300
    plate_width = 0.170
    plate_thickness = 0.012
    plate_center_x = -0.135
    screw_points = [(-0.075, -0.052), (-0.075, 0.052), (0.070, -0.052), (0.070, 0.052)]
    plate_profile = _ccw(rounded_rect_profile(plate_length, plate_width, 0.025, corner_segments=8))
    plate_holes = [_circle_profile(point, 0.006, segments=28) for point in screw_points]
    plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(plate_profile, plate_holes, plate_thickness, center=True),
        "broad_mounting_plate",
    )

    root_plate = model.part("root_plate")
    root_plate.visual(
        plate_mesh,
        origin=Origin(xyz=(plate_center_x, 0.0, plate_thickness / 2.0)),
        material=dark_anodized,
        name="mounting_plate",
    )
    for idx, (sx, sy) in enumerate(screw_points):
        root_plate.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(plate_center_x + sx, sy, plate_thickness + 0.0013)),
            material=pin_steel,
            name=f"screw_head_{idx}",
        )

    # A fixed root post sits just beyond the plate edge; the first moving link
    # has a clearance hole, so the post reads as a captured pivot rather than a
    # solid overlap.
    root_plate.visual(
        Cylinder(radius=0.022, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness + 0.0030)),
        material=pin_steel,
        name="root_boss",
    )
    root_plate.visual(
        Cylinder(radius=0.0075, length=0.033),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness + 0.0165)),
        material=pin_steel,
        name="root_pin",
    )

    link_radius = 0.034
    link_hole_radius = 0.013
    link_thickness = 0.008
    upper_z = 0.026
    lower_z = 0.016
    link_0_length = 0.205
    link_1_length = 0.175

    def link_plate_mesh(name: str, length: float):
        return mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _capsule_profile(length, link_radius),
                [
                    _circle_profile((0.0, 0.0), link_hole_radius, segments=36),
                    _circle_profile((length, 0.0), link_hole_radius, segments=36),
                ],
                link_thickness,
                center=True,
            ),
            name,
        )

    link_0 = model.part("link_0")
    link_0.visual(
        link_plate_mesh("link_0_flat_plate", link_0_length),
        origin=Origin(xyz=(0.0, 0.0, upper_z)),
        material=link_metal,
        name="link_plate",
    )
    link_0.visual(
        Cylinder(radius=0.0075, length=0.026),
        origin=Origin(xyz=(link_0_length, 0.0, 0.0210)),
        material=pin_steel,
        name="middle_pin",
    )
    link_0.visual(
        Cylinder(radius=0.018, length=0.0045),
        origin=Origin(xyz=(link_0_length, 0.0, 0.03175)),
        material=pin_steel,
        name="middle_cap",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        link_plate_mesh("link_1_flat_plate", link_1_length),
        origin=Origin(xyz=(0.0, 0.0, lower_z)),
        material=link_metal,
        name="link_plate",
    )
    link_1.visual(
        Cylinder(radius=0.0075, length=0.026),
        origin=Origin(xyz=(link_1_length, 0.0, 0.0210)),
        material=pin_steel,
        name="pad_pin",
    )
    link_1.visual(
        Cylinder(radius=0.018, length=0.0045),
        origin=Origin(xyz=(link_1_length, 0.0, 0.01025)),
        material=pin_steel,
        name="pad_cap",
    )

    pad_length = 0.105
    pad_carrier_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _capsule_profile(pad_length, 0.033, segments_per_end=20),
            [_circle_profile((0.0, 0.0), link_hole_radius, segments=36)],
            link_thickness,
            center=True,
        ),
        "compact_pad_carrier",
    )
    pad_rubber_profile = _ccw(rounded_rect_profile(0.072, 0.058, 0.014, corner_segments=8))
    pad_rubber_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(pad_rubber_profile, [], 0.006, center=True),
        "compact_rubber_pad",
    )

    end_pad = model.part("end_pad")
    end_pad.visual(
        pad_carrier_mesh,
        origin=Origin(xyz=(0.0, 0.0, upper_z)),
        material=link_metal,
        name="carrier_plate",
    )
    end_pad.visual(
        pad_rubber_mesh,
        origin=Origin(xyz=(0.080, 0.0, upper_z + 0.0070)),
        material=rubber,
        name="rubber_pad",
    )

    joint_limits = MotionLimits(effort=16.0, velocity=2.5, lower=-2.20, upper=2.20)
    model.articulation(
        "root_joint",
        ArticulationType.REVOLUTE,
        parent=root_plate,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "middle_joint",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link_0_length, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "pad_joint",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_pad,
        origin=Origin(xyz=(link_1_length, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_joint = object_model.get_articulation("root_joint")
    middle_joint = object_model.get_articulation("middle_joint")
    pad_joint = object_model.get_articulation("pad_joint")

    joints = (root_joint, middle_joint, pad_joint)
    ctx.allow_overlap(
        "link_0",
        "root_plate",
        elem_a="link_plate",
        elem_b="root_pin",
        reason="The root pin is intentionally captured through the first link's clearance eye.",
    )
    ctx.allow_overlap(
        "link_0",
        "link_1",
        elem_a="middle_pin",
        elem_b="link_plate",
        reason="The middle pivot pin is intentionally captured through the lower link's clearance eye.",
    )
    ctx.allow_overlap(
        "end_pad",
        "link_1",
        elem_a="carrier_plate",
        elem_b="pad_pin",
        reason="The end-pad pivot pin is intentionally captured through the pad carrier's clearance eye.",
    )

    ctx.check(
        "three revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "all joints bend in horizontal plane",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.expect_gap(
        "link_0",
        "root_plate",
        axis="z",
        min_gap=0.002,
        max_gap=0.006,
        positive_elem="link_plate",
        negative_elem="root_boss",
        name="root knuckle has low vertical clearance",
    )
    ctx.expect_gap(
        "link_0",
        "link_1",
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="link_plate",
        negative_elem="link_plate",
        name="middle knuckles are vertically offset",
    )
    ctx.expect_gap(
        "end_pad",
        "link_1",
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="carrier_plate",
        negative_elem="link_plate",
        name="pad knuckle is vertically offset",
    )
    ctx.expect_overlap(
        "root_plate",
        "link_0",
        axes="xy",
        min_overlap=0.010,
        elem_a="root_pin",
        elem_b="link_plate",
        name="root pin remains inside first hinge eye",
    )
    ctx.expect_overlap(
        "link_0",
        "link_1",
        axes="xy",
        min_overlap=0.010,
        elem_a="middle_pin",
        elem_b="link_plate",
        name="middle pin remains inside second hinge eye",
    )
    ctx.expect_overlap(
        "link_1",
        "end_pad",
        axes="xy",
        min_overlap=0.010,
        elem_a="pad_pin",
        elem_b="carrier_plate",
        name="pad pin remains inside end-pad hinge eye",
    )

    rest_pad = ctx.part_world_position("end_pad")
    with ctx.pose({root_joint: 0.75, middle_joint: -0.90, pad_joint: 0.65}):
        bent_pad = ctx.part_world_position("end_pad")
    ctx.check(
        "folded pose stays planar and moves sideways",
        rest_pad is not None
        and bent_pad is not None
        and abs(bent_pad[2] - rest_pad[2]) < 1e-6
        and abs(bent_pad[1] - rest_pad[1]) > 0.06,
        details=f"rest={rest_pad}, bent={bent_pad}",
    )

    return ctx.report()


object_model = build_object_model()
