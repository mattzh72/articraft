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


def _frustum(bottom_radius: float, top_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(bottom_radius)
        .workplane(offset=height)
        .circle(top_radius)
        .loft(combine=True)
    )


def _add_octagonal_side_box(
    part,
    *,
    radius: float,
    theta: float,
    z: float,
    tangential_length: float,
    radial_depth: float,
    height: float,
    material: Material,
    name: str,
) -> None:
    part.visual(
        Box((tangential_length, radial_depth, height)),
        origin=Origin(
            xyz=(radius * math.cos(theta), radius * math.sin(theta), z),
            rpy=(0.0, 0.0, theta + math.pi / 2.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="masonry_lighthouse_lantern")

    stone = model.material("warm_gray_masonry", rgba=(0.55, 0.52, 0.46, 1.0))
    mortar = model.material("dark_mortar", rgba=(0.24, 0.23, 0.21, 1.0))
    black_metal = model.material("blackened_lantern_iron", rgba=(0.02, 0.025, 0.03, 1.0))
    weathered_metal = model.material("weathered_bronze", rgba=(0.34, 0.24, 0.13, 1.0))
    brass = model.material("polished_brass", rgba=(0.90, 0.68, 0.26, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.45, 0.75, 0.95, 0.35))
    amber_glass = model.material("amber_beacon_glass", rgba=(1.0, 0.72, 0.18, 0.62))
    door_paint = model.material("aged_green_door", rgba=(0.08, 0.20, 0.16, 1.0))

    tower = model.part("tower")

    # Masonry lantern base: a tapered stone drum, plinth, coping, and visible
    # mortar courses make the assembly read as masonry rather than a plain tube.
    tower.visual(
        mesh_from_cadquery(_frustum(0.82, 0.64, 1.05), "tapered_masonry_tower"),
        material=stone,
        name="tapered_tower",
    )
    tower.visual(
        Cylinder(radius=0.86, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stone,
        name="lower_plinth",
    )
    tower.visual(
        Cylinder(radius=0.72, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.11)),
        material=stone,
        name="masonry_gallery_slab",
    )
    tower.visual(
        Cylinder(radius=0.66, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=stone,
        name="lantern_sill",
    )
    tower.visual(
        Cylinder(radius=0.24, length=1.30),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        material=stone,
        name="masonry_core",
    )
    for course_i, z in enumerate((0.22, 0.39, 0.56, 0.73, 0.90)):
        tower.visual(
            Cylinder(radius=0.78 - 0.025 * course_i, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=mortar,
            name=f"mortar_course_{course_i}",
        )

    # Octagonal lantern-room frame with glazed sides and an unglazed service-door
    # bay on the +X side.
    frame_radius = 0.60
    side_len = 2.0 * frame_radius * math.tan(math.pi / 8.0)
    bottom_rail_z = 1.2875
    top_rail_z = 2.14
    rail_height = 0.075
    wall_height = 0.92
    for i in range(8):
        theta = i * math.pi / 4.0
        _add_octagonal_side_box(
            tower,
            radius=frame_radius,
            theta=theta,
            z=bottom_rail_z,
            tangential_length=side_len,
            radial_depth=0.050,
            height=rail_height,
            material=black_metal,
            name=f"lower_rail_{i}",
        )
        _add_octagonal_side_box(
            tower,
            radius=frame_radius,
            theta=theta,
            z=top_rail_z,
            tangential_length=side_len,
            radial_depth=0.050,
            height=rail_height,
            material=black_metal,
            name=f"upper_rail_{i}",
        )
        if i != 0:
            _add_octagonal_side_box(
                tower,
                radius=frame_radius - 0.010,
                theta=theta,
                z=1.72,
                tangential_length=side_len * 0.72,
                radial_depth=0.012,
                height=0.77,
                material=glass,
                name=f"glass_pane_{i}",
            )

    post_radius = frame_radius / math.cos(math.pi / 8.0)
    for i in range(8):
        theta = math.pi / 8.0 + i * math.pi / 4.0
        tower.visual(
            Box((0.065, 0.065, wall_height)),
            origin=Origin(
                xyz=(post_radius * math.cos(theta), post_radius * math.sin(theta), 1.72),
                rpy=(0.0, 0.0, theta),
            ),
            material=black_metal,
            name=f"corner_post_{i}",
        )

    # Door bay trim and fixed hinge leaf on the outer wall frame.  The moving
    # door closes just outside this frame and is carried by the vertical hinge.
    door_frame_x = 0.615
    tower.visual(
        Box((0.045, 0.060, 0.78)),
        origin=Origin(xyz=(door_frame_x, -0.155, 1.70)),
        material=black_metal,
        name="hinge_jamb",
    )
    tower.visual(
        Box((0.045, 0.026, 0.78)),
        origin=Origin(xyz=(door_frame_x, 0.125, 1.70)),
        material=black_metal,
        name="latch_jamb",
    )
    tower.visual(
        Box((0.045, 0.326, 0.052)),
        origin=Origin(xyz=(door_frame_x, -0.025, 2.095)),
        material=black_metal,
        name="door_lintel",
    )
    tower.visual(
        Box((0.045, 0.326, 0.052)),
        origin=Origin(xyz=(door_frame_x, -0.025, 1.305)),
        material=black_metal,
        name="door_sill",
    )
    tower.visual(
        Box((0.011, 0.012, 0.68)),
        origin=Origin(xyz=(0.6375, -0.140, 1.70)),
        material=black_metal,
        name="fixed_hinge_leaf",
    )

    # Roof and ventilator are fixed to the frame and connected through the upper
    # rail ring.
    tower.visual(
        mesh_from_cadquery(_frustum(0.69, 0.20, 0.34), "tapered_lantern_roof"),
        origin=Origin(xyz=(0.0, 0.0, 2.18)),
        material=weathered_metal,
        name="tapered_roof",
    )
    tower.visual(
        Cylinder(radius=0.20, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.60)),
        material=weathered_metal,
        name="ventilator_cowl",
    )
    tower.visual(
        Cylinder(radius=0.035, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 2.81)),
        material=black_metal,
        name="roof_finial",
    )

    # Fixed central vertical shaft inside the lantern room.  The rotating
    # carriage rests on the top bearing surface without interpenetrating it.
    shaft_top = 1.55
    shaft_bottom = 1.21
    tower.visual(
        Cylinder(radius=0.070, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, shaft_bottom + 0.040)),
        material=black_metal,
        name="shaft_base_bearing",
    )
    tower.visual(
        Cylinder(radius=0.035, length=shaft_top - shaft_bottom),
        origin=Origin(xyz=(0.0, 0.0, (shaft_top + shaft_bottom) / 2.0)),
        material=black_metal,
        name="central_shaft",
    )

    beacon = model.part("beacon_carriage")
    beacon.visual(
        Cylinder(radius=0.16, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=brass,
        name="turntable",
    )
    beacon.visual(
        Cylinder(radius=0.052, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=black_metal,
        name="rotating_mast",
    )
    beacon.visual(
        Box((0.58, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=brass,
        name="lamp_crossarm",
    )
    for side, x in (("front", 0.36), ("rear", -0.36)):
        beacon.visual(
            Cylinder(radius=0.085, length=0.16),
            origin=Origin(xyz=(x, 0.0, 0.275), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"{side}_lamp_body",
        )
        lens_x = x + (0.092 if x > 0.0 else -0.092)
        beacon.visual(
            Cylinder(radius=0.078, length=0.024),
            origin=Origin(xyz=(lens_x, 0.0, 0.275), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=amber_glass,
            name=f"{side}_lens",
        )
    beacon.visual(
        Box((0.16, 0.035, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=brass,
        name="upper_yoke",
    )

    model.articulation(
        "shaft_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, shaft_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )

    door = model.part("service_door")
    door.visual(
        Box((0.035, 0.22, 0.70)),
        origin=Origin(xyz=(0.0, 0.130, 0.350)),
        material=door_paint,
        name="door_panel",
    )
    door.visual(
        Box((0.041, 0.020, 0.68)),
        origin=Origin(xyz=(0.0, 0.022, 0.350)),
        material=black_metal,
        name="hinge_stile",
    )
    door.visual(
        Box((0.041, 0.018, 0.66)),
        origin=Origin(xyz=(0.0, 0.225, 0.350)),
        material=black_metal,
        name="latch_stile",
    )
    for z, name in ((0.055, "bottom_rail"), (0.350, "middle_rail"), (0.645, "top_rail")):
        door.visual(
            Box((0.041, 0.22, 0.030)),
            origin=Origin(xyz=(0.0, 0.130, z)),
            material=black_metal,
            name=f"door_{name}",
        )
    door.visual(
        Cylinder(radius=0.012, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=brass,
        name="hinge_barrel",
    )
    for z, name in ((0.17, "lower_hinge_strap"), (0.54, "upper_hinge_strap")):
        door.visual(
            Box((0.014, 0.065, 0.040)),
            origin=Origin(xyz=(0.021, 0.025, z)),
            material=brass,
            name=name,
        )
    door.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.023, 0.210, 0.355), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="round_pull",
    )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=door,
        origin=Origin(xyz=(0.655, -0.140, 1.35)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon_carriage")
    door = object_model.get_part("service_door")
    beacon_joint = object_model.get_articulation("shaft_to_beacon")
    door_joint = object_model.get_articulation("frame_to_door")

    ctx.check(
        "beacon uses continuous vertical rotation",
        beacon_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(beacon_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={beacon_joint.articulation_type}, axis={beacon_joint.axis}",
    )
    ctx.check(
        "service door has realistic hinge limits",
        door_joint.motion_limits is not None
        and door_joint.motion_limits.lower == 0.0
        and door_joint.motion_limits.upper is not None
        and 1.2 <= door_joint.motion_limits.upper <= 1.6,
        details=f"limits={door_joint.motion_limits}",
    )

    ctx.expect_contact(
        beacon,
        tower,
        elem_a="turntable",
        elem_b="central_shaft",
        contact_tol=0.001,
        name="turntable rests on central shaft",
    )
    ctx.expect_overlap(
        door,
        tower,
        axes="z",
        elem_a="hinge_barrel",
        elem_b="fixed_hinge_leaf",
        min_overlap=0.55,
        name="service door hinge spans the wall leaf",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.0}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "service door opens outward from lantern wall",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.06,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    with ctx.pose({beacon_joint: math.pi / 2.0}):
        rotated_aabb = ctx.part_world_aabb(beacon)
    ctx.check(
        "rotated beacon remains inside lantern footprint",
        rotated_aabb is not None
        and max(
            abs(rotated_aabb[0][0]),
            abs(rotated_aabb[1][0]),
            abs(rotated_aabb[0][1]),
            abs(rotated_aabb[1][1]),
        )
        < 0.56,
        details=f"rotated_aabb={rotated_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
