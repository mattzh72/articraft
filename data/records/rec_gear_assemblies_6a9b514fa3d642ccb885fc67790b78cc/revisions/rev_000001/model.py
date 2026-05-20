from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHAFT_AXIS_Z = 0.18

BACK_PLATE_Y = -0.055
FRONT_PLATE_Y = 0.055
PLATE_THICKNESS = 0.010
BEARING_BOSS_IN = 0.012
BEARING_BOSS_OUT = 0.006

BACK_SUPPORT_OUTER = BACK_PLATE_Y - PLATE_THICKNESS / 2.0 - BEARING_BOSS_OUT
BACK_SUPPORT_INNER = BACK_PLATE_Y + PLATE_THICKNESS / 2.0 + BEARING_BOSS_IN
FRONT_SUPPORT_INNER = FRONT_PLATE_Y - PLATE_THICKNESS / 2.0 - BEARING_BOSS_IN
FRONT_SUPPORT_OUTER = FRONT_PLATE_Y + PLATE_THICKNESS / 2.0 + BEARING_BOSS_OUT

BORE_RADIUS = 0.0105
JOURNAL_RADIUS = 0.0084
CENTER_SHAFT_RADIUS = 0.0108
EXPOSED_STUB_RADIUS = 0.0078
REAR_SHOULDER_Y = -0.028
FRONT_SHOULDER_Y = 0.028

INPUT_SHAFT_X = -0.138
IDLER_SHAFT_X = 0.0
OUTPUT_SHAFT_X = 0.122

STAGE_1_Y = -0.014
STAGE_2_Y = 0.014

STANDOFF_POSITIONS = [
    (-0.215, SHAFT_AXIS_Z - 0.105),
    (-0.215, SHAFT_AXIS_Z + 0.105),
    (0.215, SHAFT_AXIS_Z - 0.105),
    (0.215, SHAFT_AXIS_Z + 0.105),
]


def fuse_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def cylinder_y(radius: float, length: float, *, center_y: float = 0.0, x: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, center_y - length / 2.0, 0.0))
    )


def box_xyz(size_x: float, size_y: float, size_z: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def radial_box_y(
    radial_len: float,
    y_len: float,
    tangential_len: float,
    *,
    radius_center: float,
    angle_deg: float,
    y_center: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(radial_len, y_len, tangential_len)
        .translate((radius_center, y_center, 0.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
    )


def ring_plate(
    *,
    width: float,
    height: float,
    thickness: float,
    center_y: float,
    center_z: float,
    window_width: float,
    window_height: float,
    corner_radius: float,
) -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .center(0.0, center_z)
        .rect(width, height)
        .extrude(thickness)
        .translate((0.0, center_y - thickness / 2.0, 0.0))
        .edges("|Y")
        .fillet(corner_radius)
    )
    inner = (
        cq.Workplane("XZ")
        .center(0.0, center_z)
        .rect(window_width, window_height)
        .extrude(thickness + 0.004)
        .translate((0.0, center_y - thickness / 2.0 - 0.002, 0.0))
        .edges("|Y")
        .fillet(corner_radius * 0.7)
    )
    return outer.cut(inner)


def bearing_cover_pad(center_x: float, center_y: float, thickness: float) -> cq.Workplane:
    pad = (
        cq.Workplane("XZ")
        .center(center_x, SHAFT_AXIS_Z)
        .rect(0.062, 0.050)
        .extrude(thickness)
        .translate((0.0, center_y - thickness / 2.0, 0.0))
        .edges("|Y")
        .fillet(0.007)
    )
    return pad


def make_standoff_posts() -> cq.Workplane:
    posts = []
    for x_pos, z_pos in STANDOFF_POSITIONS:
        posts.append(cylinder_y(0.009, 0.091, center_y=-0.0045, x=x_pos, z=z_pos))
        posts.append(cylinder_y(0.013, 0.010, center_y=-0.045, x=x_pos, z=z_pos))
    return fuse_all(posts)


def make_front_mount_lugs() -> cq.Workplane:
    lugs = []
    for x_pos, z_pos in STANDOFF_POSITIONS:
        lug = cylinder_y(0.013, 0.009, center_y=0.0455, x=x_pos, z=z_pos)
        hole = cylinder_y(0.0035, 0.011, center_y=0.0455, x=x_pos, z=z_pos)
        lugs.append(lug.cut(hole))
    return fuse_all(lugs)


def make_back_frame() -> cq.Workplane:
    frame = ring_plate(
        width=0.52,
        height=0.30,
        thickness=PLATE_THICKNESS,
        center_y=BACK_PLATE_Y,
        center_z=SHAFT_AXIS_Z,
        window_width=0.44,
        window_height=0.19,
        corner_radius=0.018,
    )

    shaft_xs = (INPUT_SHAFT_X, IDLER_SHAFT_X, OUTPUT_SHAFT_X)

    rear_housings = [
        cylinder_y(
            0.028,
            0.022,
            center_y=-0.061,
            x=x_pos,
            z=SHAFT_AXIS_Z,
        )
        for x_pos in shaft_xs
    ]
    inboard_bosses = [
        cylinder_y(0.022, 0.012, center_y=-0.044, x=x_pos, z=SHAFT_AXIS_Z)
        for x_pos in shaft_xs
    ]

    feet = [
        box_xyz(0.080, 0.090, 0.060, center=(-0.215, -0.010, 0.030)),
        box_xyz(0.080, 0.090, 0.060, center=(0.215, -0.010, 0.030)),
    ]

    frame = fuse_all([frame, *rear_housings, *inboard_bosses, *feet])

    bores = [
        cylinder_y(BORE_RADIUS, 0.038, center_y=-0.055, x=x_pos, z=SHAFT_AXIS_Z)
        for x_pos in shaft_xs
    ]
    return frame.cut(fuse_all(bores))


def make_front_plate() -> cq.Workplane:
    front = ring_plate(
        width=0.50,
        height=0.28,
        thickness=PLATE_THICKNESS,
        center_y=FRONT_PLATE_Y,
        center_z=SHAFT_AXIS_Z,
        window_width=0.42,
        window_height=0.18,
        corner_radius=0.016,
    )

    shaft_xs = (INPUT_SHAFT_X, IDLER_SHAFT_X, OUTPUT_SHAFT_X)

    inward_bosses = [cylinder_y(0.022, 0.012, center_y=0.044, x=x_pos, z=SHAFT_AXIS_Z) for x_pos in shaft_xs]
    outward_cover_rings = [cylinder_y(0.028, 0.022, center_y=0.061, x=x_pos, z=SHAFT_AXIS_Z) for x_pos in shaft_xs]

    front = fuse_all([front, *inward_bosses, *outward_cover_rings])

    bores = [
        cylinder_y(BORE_RADIUS, 0.038, center_y=0.055, x=x_pos, z=SHAFT_AXIS_Z)
        for x_pos in shaft_xs
    ]
    return front.cut(fuse_all(bores))


def faux_spur_gear(
    *,
    outer_radius: float,
    width: float,
    teeth: int,
    tooth_depth: float,
    hub_radius: float,
    hub_length: float,
    pocket_radius: float,
    pocket_hole_radius: float,
    pocket_count: int,
) -> cq.Workplane:
    rim_inner_radius = max(hub_radius + 0.010, outer_radius * 0.62)
    rim = cylinder_y(outer_radius, width).cut(cylinder_y(rim_inner_radius, width + 0.004))
    hub = cylinder_y(hub_radius, hub_length)

    spoke_count = max(3, pocket_count)
    spoke_center = (hub_radius + rim_inner_radius) / 2.0
    spoke_len = rim_inner_radius - hub_radius + 0.006
    spoke_thickness = max(0.008, min(0.014, pocket_hole_radius * 1.6))
    spokes = [
        radial_box_y(
            spoke_len,
            width * 0.88,
            spoke_thickness,
            radius_center=spoke_center,
            angle_deg=360.0 * index / spoke_count,
        )
        for index in range(spoke_count)
    ]

    key_bar = box_xyz(
        hub_radius - CENTER_SHAFT_RADIUS + 0.006,
        width * 0.82,
        0.005,
        center=((hub_radius + CENTER_SHAFT_RADIUS) / 2.0, 0.0, 0.0),
    )

    gear = fuse_all([rim, hub, *spokes, key_bar])

    notch_center = outer_radius - tooth_depth / 2.0
    notch_width = max(0.005, min(0.014, 2.0 * math.pi * outer_radius / teeth * 0.42))
    notches = [
        radial_box_y(
            tooth_depth * 1.12,
            width + 0.006,
            notch_width,
            radius_center=notch_center,
            angle_deg=360.0 * index / teeth,
        )
        for index in range(teeth)
    ]
    gear = gear.cut(fuse_all(notches))
    return gear


def make_handwheel() -> cq.Workplane:
    outer_radius = 0.074
    inner_radius = 0.061
    width = 0.014
    hub_radius = 0.016
    hub_length = 0.022

    rim = cylinder_y(outer_radius, width).cut(cylinder_y(inner_radius, width + 0.004))
    hub = cylinder_y(hub_radius, hub_length)

    spoke_mid = (hub_radius + inner_radius) / 2.0
    spoke_len = inner_radius - hub_radius + 0.004
    spokes = [
        radial_box_y(spoke_len, width, 0.010, radius_center=spoke_mid, angle_deg=angle)
        for angle in (0.0, 90.0, 180.0, 270.0)
    ]

    return fuse_all([rim, hub, *spokes])


def make_output_flange() -> cq.Workplane:
    flange = cylinder_y(0.030, 0.012).union(cylinder_y(0.016, 0.020))
    bolt_points = [
        (0.019 * math.cos(angle), 0.019 * math.sin(angle))
        for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)
    ]
    bolt_holes = (
        cq.Workplane("XZ")
        .pushPoints(bolt_points)
        .circle(0.0032)
        .extrude(0.018)
        .translate((0.0, -0.009, 0.0))
    )
    return flange.cut(bolt_holes)


def make_stepped_shaft(
    *,
    front_end: float,
    rear_end: float,
    add_center_spacer: bool = False,
    d_flat_stub: bool = False,
    key_positions: tuple[float, ...] = (),
) -> cq.Workplane:
    pieces = [
        cylinder_y(EXPOSED_STUB_RADIUS, abs(rear_end + 0.074), center_y=(rear_end - 0.074) / 2.0),
        cylinder_y(JOURNAL_RADIUS, 0.036, center_y=-0.056),
        cylinder_y(CENTER_SHAFT_RADIUS, 0.064, center_y=0.0),
        cylinder_y(JOURNAL_RADIUS, 0.036, center_y=0.056),
        cylinder_y(EXPOSED_STUB_RADIUS, abs(front_end - 0.074), center_y=(front_end + 0.074) / 2.0),
        cylinder_y(0.013, 0.006, center_y=REAR_SHOULDER_Y),
        cylinder_y(0.013, 0.006, center_y=FRONT_SHOULDER_Y),
    ]
    if add_center_spacer:
        pieces.append(cylinder_y(0.014, 0.010, center_y=0.0))
    for key_y in key_positions:
        pieces.append(box_xyz(0.007, 0.014, 0.0045, center=(0.0135, key_y, 0.0)))

    shaft = fuse_all(pieces)

    if d_flat_stub:
        flat = box_xyz(
            0.020,
            max(front_end - 0.074, 0.020) + 0.004,
            0.008,
            center=(0.0, (front_end + 0.074) / 2.0, 0.010),
        )
        shaft = shaft.cut(flat)

    return shaft


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exposed_spur_reducer")

    model.material("frame_blue", color=(0.18, 0.24, 0.31, 1.0))
    model.material("cover_gray", color=(0.34, 0.37, 0.41, 1.0))
    model.material("steel", color=(0.70, 0.72, 0.75, 1.0))
    model.material("gear_steel", color=(0.58, 0.60, 0.63, 1.0))
    model.material("wheel_dark", color=(0.15, 0.16, 0.18, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_back_frame(), "frame_back"),
        name="back_frame",
        material="frame_blue",
    )
    frame.visual(
        mesh_from_cadquery(make_standoff_posts(), "frame_standoff_posts"),
        name="standoff_posts",
        material="steel",
    )

    front_cover = model.part("front_cover")
    front_cover.visual(
        mesh_from_cadquery(make_front_plate(), "front_cover_plate"),
        name="front_cover_plate",
        material="cover_gray",
    )
    front_cover.visual(
        mesh_from_cadquery(make_front_mount_lugs(), "front_mount_lugs"),
        name="front_mount_lugs",
        material="steel",
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        mesh_from_cadquery(
            make_stepped_shaft(front_end=0.108, rear_end=-0.082, d_flat_stub=True),
            "input_shaft_body",
        ),
        name="input_shaft_body",
        material="steel",
    )
    input_shaft.visual(
        mesh_from_cadquery(
            faux_spur_gear(
                outer_radius=0.050,
                width=0.014,
                teeth=16,
                tooth_depth=0.006,
                hub_radius=0.018,
                hub_length=0.022,
                pocket_radius=0.026,
                pocket_hole_radius=0.0085,
                pocket_count=3,
            ),
            "input_gear",
        ),
        origin=Origin(xyz=(0.0, STAGE_1_Y, 0.0)),
        name="input_gear",
        material="gear_steel",
    )
    input_shaft.visual(
        mesh_from_cadquery(make_handwheel(), "handwheel"),
        origin=Origin(xyz=(0.0, 0.096, 0.0)),
        name="handwheel",
        material="wheel_dark",
    )

    idler_shaft = model.part("idler_shaft")
    idler_shaft.visual(
        mesh_from_cadquery(
            make_stepped_shaft(front_end=0.078, rear_end=-0.078, add_center_spacer=True),
            "idler_shaft_body",
        ),
        name="idler_shaft_body",
        material="steel",
    )
    idler_shaft.visual(
        mesh_from_cadquery(
            faux_spur_gear(
                outer_radius=0.086,
                width=0.014,
                teeth=28,
                tooth_depth=0.007,
                hub_radius=0.020,
                hub_length=0.024,
                pocket_radius=0.042,
                pocket_hole_radius=0.011,
                pocket_count=4,
            ),
            "idler_large_gear",
        ),
        origin=Origin(xyz=(0.0, STAGE_1_Y, 0.0)),
        name="idler_large_gear",
        material="gear_steel",
    )
    idler_shaft.visual(
        mesh_from_cadquery(
            faux_spur_gear(
                outer_radius=0.042,
                width=0.014,
                teeth=14,
                tooth_depth=0.005,
                hub_radius=0.016,
                hub_length=0.020,
                pocket_radius=0.020,
                pocket_hole_radius=0.0055,
                pocket_count=3,
            ),
            "idler_pinion",
        ),
        origin=Origin(xyz=(0.0, STAGE_2_Y, 0.0)),
        name="idler_pinion",
        material="gear_steel",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        mesh_from_cadquery(
            make_stepped_shaft(front_end=0.096, rear_end=-0.080, d_flat_stub=True),
            "output_shaft_body",
        ),
        name="output_shaft_body",
        material="steel",
    )
    output_shaft.visual(
        mesh_from_cadquery(
            faux_spur_gear(
                outer_radius=0.078,
                width=0.014,
                teeth=26,
                tooth_depth=0.007,
                hub_radius=0.019,
                hub_length=0.024,
                pocket_radius=0.037,
                pocket_hole_radius=0.010,
                pocket_count=4,
            ),
            "output_gear",
        ),
        origin=Origin(xyz=(0.0, STAGE_2_Y, 0.0)),
        name="output_gear",
        material="gear_steel",
    )
    output_shaft.visual(
        mesh_from_cadquery(make_output_flange(), "output_flange"),
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
        name="output_flange",
        material="steel",
    )

    model.articulation(
        "frame_to_front_cover",
        ArticulationType.FIXED,
        parent=frame,
        child=front_cover,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_input_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=input_shaft,
        origin=Origin(xyz=(INPUT_SHAFT_X, 0.0, SHAFT_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    model.articulation(
        "frame_to_idler_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=idler_shaft,
        origin=Origin(xyz=(IDLER_SHAFT_X, 0.0, SHAFT_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    model.articulation(
        "frame_to_output_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(OUTPUT_SHAFT_X, 0.0, SHAFT_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_cover = object_model.get_part("front_cover")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")

    input_joint = object_model.get_articulation("frame_to_input_shaft")
    idler_joint = object_model.get_articulation("frame_to_idler_shaft")
    output_joint = object_model.get_articulation("frame_to_output_shaft")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        frame,
        input_shaft,
        elem_a="back_frame",
        elem_b="input_shaft_body",
        reason="The input shaft journals run inside the frame's bearing bores; tessellated bearing fits can register as overlap.",
    )
    ctx.allow_overlap(
        frame,
        idler_shaft,
        elem_a="back_frame",
        elem_b="idler_shaft_body",
        reason="The idler shaft journals run inside the frame's bearing bores; tessellated bearing fits can register as overlap.",
    )
    ctx.allow_overlap(
        frame,
        output_shaft,
        elem_a="back_frame",
        elem_b="output_shaft_body",
        reason="The output shaft journals run inside the frame's bearing bores; tessellated bearing fits can register as overlap.",
    )
    ctx.allow_overlap(
        front_cover,
        input_shaft,
        elem_a="front_cover_plate",
        elem_b="input_shaft_body",
        reason="The input shaft journals run inside the front bearing bores; tessellated bearing fits can register as overlap.",
    )
    ctx.allow_overlap(
        front_cover,
        idler_shaft,
        elem_a="front_cover_plate",
        elem_b="idler_shaft_body",
        reason="The idler shaft journals run inside the front bearing bores; tessellated bearing fits can register as overlap.",
    )
    ctx.allow_overlap(
        front_cover,
        output_shaft,
        elem_a="front_cover_plate",
        elem_b="output_shaft_body",
        reason="The output shaft journals run inside the front bearing bores; tessellated bearing fits can register as overlap.",
    )

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "shaft_axes_parallel",
        input_joint.axis == (0.0, 1.0, 0.0)
        and idler_joint.axis == (0.0, 1.0, 0.0)
        and output_joint.axis == (0.0, 1.0, 0.0),
        details="Input, idler, and output shafts should all rotate about the same parallel Y axis.",
    )

    ctx.expect_origin_distance(
        frame,
        front_cover,
        axes="y",
        max_dist=0.001,
        name="front_cover_clamped_to_frame_origin",
    )
    ctx.expect_within(
        front_cover,
        frame,
        axes="xz",
        margin=0.020,
        name="front_cover_clamped_to_frame_footprint",
    )

    ctx.expect_contact(frame, input_shaft, name="input_shaft_contacts_rear_bearing")
    ctx.expect_contact(front_cover, input_shaft, name="input_shaft_contacts_front_bearing")
    ctx.expect_contact(frame, idler_shaft, name="idler_shaft_contacts_rear_bearing")
    ctx.expect_contact(front_cover, idler_shaft, name="idler_shaft_contacts_front_bearing")
    ctx.expect_contact(frame, output_shaft, name="output_shaft_contacts_rear_bearing")
    ctx.expect_contact(front_cover, output_shaft, name="output_shaft_contacts_front_bearing")

    ctx.expect_gap(
        front_cover,
        output_shaft,
        axis="y",
        positive_elem="front_cover_plate",
        negative_elem="output_gear",
        min_gap=0.004,
        name="output_gear_clear_of_front_cover",
    )
    ctx.expect_gap(
        input_shaft,
        front_cover,
        axis="y",
        positive_elem="handwheel",
        negative_elem="front_cover_plate",
        min_gap=0.004,
        name="handwheel_clear_of_front_cover",
    )
    ctx.expect_gap(
        output_shaft,
        front_cover,
        axis="y",
        positive_elem="output_flange",
        negative_elem="front_cover_plate",
        min_gap=0.004,
        name="output_flange_clear_of_front_cover",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
