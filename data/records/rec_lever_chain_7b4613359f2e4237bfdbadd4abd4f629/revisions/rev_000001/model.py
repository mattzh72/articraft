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


def _arc_points(
    center_x: float,
    center_z: float,
    radius: float,
    start_deg: float,
    end_deg: float,
    steps: int,
) -> list[tuple[float, float]]:
    return [
        (
            center_x + radius * math.cos(math.radians(start_deg + (end_deg - start_deg) * i / steps)),
            center_z + radius * math.sin(math.radians(start_deg + (end_deg - start_deg) * i / steps)),
        )
        for i in range(steps + 1)
    ]


def _left_round_taper_profile(end_x: float, root_height: float, end_height: float) -> list[tuple[float, float]]:
    """Side outline for a flat link body: rounded proximal eye, tapered sides, square distal shoulder."""
    root_radius = root_height / 2.0
    return (
        _arc_points(0.0, 0.0, root_radius, -90.0, -270.0, 12)
        + [(end_x, end_height / 2.0), (end_x, -end_height / 2.0)]
    )


def _round_taper_tab_profile(
    start_x: float,
    end_x: float,
    start_height: float,
    end_height: float,
) -> list[tuple[float, float]]:
    """Side outline with rounded eyes at both ends and a tapered waist."""
    start_radius = start_height / 2.0
    end_radius = end_height / 2.0
    return (
        _arc_points(start_x, 0.0, start_radius, -90.0, -270.0, 12)
        + [(end_x, end_radius)]
        + _arc_points(end_x, 0.0, end_radius, 90.0, -90.0, 12)[1:]
    )


def _fork_cheek_profile(back_x: float, pivot_x: float, height: float) -> list[tuple[float, float]]:
    radius = height / 2.0
    return (
        [(back_x, -radius), (back_x, radius), (pivot_x, radius)]
        + _arc_points(pivot_x, 0.0, radius, 90.0, -90.0, 16)[1:]
    )


def _plate_from_xz_profile(
    profile: list[tuple[float, float]],
    thickness_y: float,
    *,
    holes: list[tuple[float, float, float]] | None = None,
) -> cq.Workplane:
    """Extrude a flat plate along local Y from a side profile drawn in XZ."""
    body = cq.Workplane("XZ").polyline(profile).close().extrude(thickness_y / 2.0, both=True)
    for hole_x, hole_z, hole_radius in holes or []:
        cutter = (
            cq.Workplane("XZ")
            .center(hole_x, hole_z)
            .circle(hole_radius)
            .extrude(thickness_y * 1.8, both=True)
        )
        body = body.cut(cutter)
    return body


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _link_body_shape(
    *,
    length: float,
    root_height: float,
    shoulder_height: float,
    body_thickness: float,
    body_hole_radius: float,
) -> cq.Workplane:
    body_end_x = length - 0.078
    return _plate_from_xz_profile(
        _left_round_taper_profile(body_end_x, root_height, shoulder_height),
        body_thickness,
        holes=[(0.0, 0.0, body_hole_radius)],
    )


def _fork_cheek_shape(
    *,
    length: float,
    fork_height: float,
    fork_hole_radius: float,
    cheek_thickness: float,
) -> cq.Workplane:
    return _plate_from_xz_profile(
        _fork_cheek_profile(length - 0.096, length, fork_height),
        cheek_thickness,
        holes=[(length, 0.0, fork_hole_radius)],
    )


def _end_tab_shape() -> cq.Workplane:
    length = 0.165
    return _plate_from_xz_profile(
        _round_taper_tab_profile(0.0, length, 0.034, 0.026),
        0.014,
        holes=[(0.0, 0.0, 0.010), (length, 0.0, 0.007)],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_lever_chain")

    bridge_paint = Material("bridge_blue", rgba=(0.10, 0.18, 0.28, 1.0))
    link_paint = Material("warm_link_paint", rgba=(0.86, 0.52, 0.12, 1.0))
    tab_paint = Material("small_tab_paint", rgba=(0.88, 0.63, 0.22, 1.0))
    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    support = model.part("bridge_support")
    support.visual(
        Box((0.34, 0.25, 0.040)),
        origin=Origin(xyz=(-0.105, 0.0, -0.580)),
        material=bridge_paint,
        name="base_plate",
    )
    for idx, y in enumerate((-0.085, 0.085)):
        support.visual(
            Box((0.052, 0.046, 0.525)),
            origin=Origin(xyz=(-0.110, y, -0.305)),
            material=bridge_paint,
            name=f"bridge_leg_{idx}",
        )
    support.visual(
        Box((0.104, 0.240, 0.058)),
        origin=Origin(xyz=(-0.105, 0.0, -0.045)),
        material=bridge_paint,
        name="top_bridge",
    )
    support.visual(
        Box((0.050, 0.090, 0.074)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        material=bridge_paint,
        name="clevis_mount",
    )
    for idx, y in enumerate((-0.017, 0.017)):
        support.visual(
            Box((0.116, 0.014, 0.096)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=bridge_paint,
            name=f"root_cheek_{idx}",
        )
        cap_y = y + (-0.011 if y < 0.0 else 0.011)
        support.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(0.0, cap_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"root_pin_cap_{idx}",
        )

    drive_length = 0.420
    drive_link = model.part("drive_link")
    drive_link.visual(
        mesh_from_cadquery(
            _link_body_shape(
                length=drive_length,
                root_height=0.070,
                shoulder_height=0.050,
                body_thickness=0.020,
                body_hole_radius=0.014,
            ),
            "drive_body_plate",
            tolerance=0.0007,
        ),
        material=link_paint,
        name="body_plate",
    )
    for idx, y in enumerate((-0.014, 0.014)):
        drive_link.visual(
            mesh_from_cadquery(
                _fork_cheek_shape(
                    length=drive_length,
                    fork_height=0.058,
                    fork_hole_radius=0.013,
                    cheek_thickness=0.010,
                ),
                f"drive_fork_cheek_{idx}",
                tolerance=0.0007,
            ),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=link_paint,
            name=f"fork_cheek_{idx}",
        )
    drive_link.visual(
        Box((0.030, 0.042, 0.034)),
        origin=Origin(xyz=(drive_length - 0.085, 0.0, 0.0)),
        material=link_paint,
        name="fork_bridge",
    )
    for idx, y in enumerate((-0.023, 0.023)):
        drive_link.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(drive_length, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"distal_pin_cap_{idx}",
        )

    coupler_length = 0.320
    coupler_link = model.part("coupler_link")
    coupler_link.visual(
        mesh_from_cadquery(
            _link_body_shape(
                length=coupler_length,
                root_height=0.052,
                shoulder_height=0.038,
                body_thickness=0.018,
                body_hole_radius=0.011,
            ),
            "coupler_body_plate",
            tolerance=0.0007,
        ),
        material=link_paint,
        name="body_plate",
    )
    for idx, y in enumerate((-0.011, 0.011)):
        coupler_link.visual(
            mesh_from_cadquery(
                _fork_cheek_shape(
                    length=coupler_length,
                    fork_height=0.044,
                    fork_hole_radius=0.010,
                    cheek_thickness=0.008,
                ),
                f"coupler_fork_cheek_{idx}",
                tolerance=0.0007,
            ),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=link_paint,
            name=f"fork_cheek_{idx}",
        )
    coupler_link.visual(
        Box((0.026, 0.034, 0.028)),
        origin=Origin(xyz=(coupler_length - 0.085, 0.0, 0.0)),
        material=link_paint,
        name="fork_bridge",
    )
    for idx, y in enumerate((-0.018, 0.018)):
        coupler_link.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(coupler_length, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"distal_pin_cap_{idx}",
        )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_end_tab_shape(), "end_tab_plate", tolerance=0.0007),
        material=tab_paint,
        name="tab_plate",
    )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=support,
        child=drive_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.5, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "link_pivot",
        ArticulationType.REVOLUTE,
        parent=drive_link,
        child=coupler_link,
        origin=Origin(xyz=(drive_length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "tab_pivot",
        ArticulationType.REVOLUTE,
        parent=coupler_link,
        child=end_tab,
        origin=Origin(xyz=(coupler_length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.5, lower=-1.55, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("root_pivot"),
        object_model.get_articulation("link_pivot"),
        object_model.get_articulation("tab_pivot"),
    ]
    ctx.check(
        "three revolute joints in series",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and [(j.parent, j.child) for j in joints]
        == [
            ("bridge_support", "drive_link"),
            ("drive_link", "coupler_link"),
            ("coupler_link", "end_tab"),
        ],
    )

    drive_box = ctx.part_element_world_aabb("drive_link", elem="body_plate")
    coupler_box = ctx.part_element_world_aabb("coupler_link", elem="body_plate")
    tab_box = ctx.part_element_world_aabb("end_tab", elem="tab_plate")
    if drive_box and coupler_box and tab_box:
        drive_height = drive_box[1][2] - drive_box[0][2]
        coupler_height = coupler_box[1][2] - coupler_box[0][2]
        tab_height = tab_box[1][2] - tab_box[0][2]
        drive_thickness = drive_box[1][1] - drive_box[0][1]
        coupler_thickness = coupler_box[1][1] - coupler_box[0][1]
        tab_thickness = tab_box[1][1] - tab_box[0][1]
        ctx.check(
            "sections taper toward end tab",
            drive_height > coupler_height > tab_height
            and drive_thickness > coupler_thickness > tab_thickness,
            details=(
                f"heights={drive_height:.3f},{coupler_height:.3f},{tab_height:.3f}; "
                f"thicknesses={drive_thickness:.3f},{coupler_thickness:.3f},{tab_thickness:.3f}"
            ),
        )
    else:
        ctx.fail("sections taper toward end tab", "missing element AABB for one or more link plates")

    rest_tip = ctx.part_world_position("end_tab")
    with ctx.pose({"root_pivot": 0.45, "link_pivot": -0.55, "tab_pivot": 0.70}):
        posed_tip = ctx.part_world_position("end_tab")
    ctx.check(
        "distal tab responds to serial pivots",
        rest_tip is not None
        and posed_tip is not None
        and math.dist(rest_tip, posed_tip) > 0.08,
        details=f"rest={rest_tip}, posed={posed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
