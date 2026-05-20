from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _compound(*objs):
    shapes = []
    for obj in objs:
        if obj is None:
            continue
        if isinstance(obj, cq.Workplane):
            shapes.append(obj.val())
        else:
            shapes.append(obj)
    return cq.Compound.makeCompound(shapes)


def _box(size, center):
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(radius, length, center):
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length)
        .translate((0.0, center[1] + length / 2.0, 0.0))
    )


def _side_plate(
    *,
    body_end,
    rear_height,
    tip_height,
    waist_height,
    plate_thickness,
    rear_hole_radius,
    window_length,
    window_height,
):
    rear_disc = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(rear_height / 2.0)
        .extrude(plate_thickness)
        .translate((0.0, plate_thickness / 2.0, 0.0))
    )
    front_disc = (
        cq.Workplane("XZ")
        .center(body_end, 0.0)
        .circle(tip_height / 2.0)
        .extrude(plate_thickness)
        .translate((0.0, plate_thickness / 2.0, 0.0))
    )
    web = (
        cq.Workplane("XZ")
        .center(body_end / 2.0, 0.0)
        .rect(body_end, waist_height)
        .extrude(plate_thickness)
        .translate((0.0, plate_thickness / 2.0, 0.0))
    )

    plate = rear_disc.union(front_disc).union(web)

    rear_hole = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(rear_hole_radius)
        .extrude(plate_thickness * 2.5)
        .translate((0.0, plate_thickness * 1.25, 0.0))
    )
    plate = plate.cut(rear_hole)

    if window_length > 0.0 and window_height > 0.0:
        window = (
            cq.Workplane("XZ")
            .center(body_end * 0.50, 0.0)
            .rect(window_length, window_height)
            .extrude(plate_thickness * 2.5)
            .translate((0.0, plate_thickness * 1.25, 0.0))
        )
        plate = plate.cut(window)

    return plate


def _bracket_link(
    *,
    pitch,
    body_end,
    rear_height,
    tip_height,
    waist_height,
    plate_thickness,
    inner_gap,
    rear_hole_radius,
    front_lug_width,
    front_lug_height,
    front_lug_back,
    front_lug_forward,
    front_axle_radius,
    front_axle_length,
    spacer_radius,
    spacer_positions,
):
    plate_center_y = inner_gap / 2.0 + plate_thickness / 2.0
    span = inner_gap + 2.0 * plate_thickness
    center_body_width = min(inner_gap - 0.006, max(front_lug_width * 0.82, 0.012))
    rear_ear_length = min(max(rear_height * 0.58, 0.038), body_end * 0.55)
    rear_blade_height = rear_height * 0.68
    beam_start = rear_ear_length + 0.014
    beam_end = max(body_end, beam_start + 0.020)

    rear_disc = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(rear_height / 2.0)
        .extrude(plate_thickness)
        .translate((0.0, plate_thickness / 2.0, 0.0))
    )
    rear_blade = (
        cq.Workplane("XZ")
        .center(rear_ear_length * 0.46, 0.0)
        .rect(rear_ear_length * 0.92, rear_blade_height)
        .extrude(plate_thickness)
        .translate((0.0, plate_thickness / 2.0, 0.0))
    )
    rear_hole = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(rear_hole_radius)
        .extrude(plate_thickness * 2.5)
        .translate((0.0, plate_thickness * 1.25, 0.0))
    )
    rear_ear = rear_disc.union(rear_blade).cut(rear_hole)
    left_ear = rear_ear.translate((0.0, plate_center_y, 0.0))
    right_ear = rear_ear.translate((0.0, -plate_center_y, 0.0))

    main_beam = _box(
        (
            beam_end - beam_start,
            center_body_width,
            waist_height * 0.78,
        ),
        (
            (beam_start + beam_end) / 2.0,
            0.0,
            0.0,
        ),
    )
    top_rib = _box(
        (
            max((beam_end - beam_start) * 0.84, 0.018),
            center_body_width * 0.72,
            waist_height * 0.18,
        ),
        (
            beam_start + (beam_end - beam_start) * 0.47,
            0.0,
            waist_height * 0.28,
        ),
    )
    rear_spacer = _y_cylinder(
        spacer_radius,
        span,
        (max(beam_start - 0.010, rear_ear_length * 0.86), 0.0, 0.0),
    )
    mid_spacer = _y_cylinder(
        spacer_radius * 0.92,
        span,
        (max(body_end - 0.012, beam_start + 0.016), 0.0, 0.0),
    )

    neck_start = max(beam_end - 0.014, beam_start + 0.016)
    neck_end = pitch - front_lug_back
    neck = None
    if neck_end > neck_start:
        neck = _box(
            (
                neck_end - neck_start,
                center_body_width * 0.88,
                waist_height * 0.56,
            ),
            (
                (neck_start + neck_end) / 2.0,
                0.0,
                0.0,
            ),
        )

    lug = _box(
        (
            front_lug_back + front_lug_forward,
            front_lug_width,
            front_lug_height,
        ),
        (
            pitch + (front_lug_forward - front_lug_back) / 2.0,
            0.0,
            0.0,
        ),
    )
    axle = _y_cylinder(front_axle_radius, front_axle_length, (pitch, 0.0, 0.0))

    return _compound(
        left_ear,
        right_ear,
        main_beam,
        top_rib,
        rear_spacer,
        mid_spacer,
        neck,
        lug,
        axle,
    )


def _grounded_foot(
    *,
    joint_origin,
    lug_width,
    axle_radius,
    axle_length,
):
    base_plate = _box((0.280, 0.170, 0.020), (-0.065, 0.0, 0.010))
    heel_block = _box((0.105, 0.120, 0.035), (-0.145, 0.0, 0.0275))
    pedestal = _box((0.086, 0.030, 0.092), (-0.018, 0.0, 0.066))
    front_boss = _box((0.052, 0.032, 0.046), (joint_origin[0] - 0.006, 0.0, joint_origin[2]))

    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.070, 0.020),
                (0.000, 0.020),
                (joint_origin[0] + 0.008, joint_origin[2] - 0.030),
                (-0.022, joint_origin[2] - 0.030),
            ]
        )
        .close()
        .extrude(0.028)
        .translate((0.0, -0.014, 0.0))
    )

    lug = _box((0.040, min(lug_width, 0.030), 0.042), (joint_origin[0], 0.0, joint_origin[2]))
    axle = _y_cylinder(axle_radius, axle_length, joint_origin)
    toe_rib = _box((0.060, 0.070, 0.020), (0.028, 0.0, 0.020))

    return _compound(base_plate, heel_block, pedestal, front_boss, gusset, lug, axle, toe_rib)


def _tray_end_bracket(
    *,
    tray_length,
    tray_width,
    floor_thickness,
    wall_height,
    wall_thickness,
    ear_thickness,
    ear_inner_gap,
    rear_hole_radius,
):
    plate_center_y = ear_inner_gap / 2.0 + ear_thickness / 2.0
    tray_start = 0.046
    floor_center = (tray_start + tray_length / 2.0, 0.0, -0.018)

    floor = _box((tray_length, tray_width, floor_thickness), floor_center)
    front_lip = _box(
        (wall_thickness, tray_width, wall_height * 0.72),
        (tray_start + tray_length + wall_thickness / 2.0, 0.0, -0.008),
    )
    rear_wall = _box(
        (wall_thickness, tray_width, wall_height * 0.82),
        (tray_start + wall_thickness / 2.0, 0.0, -0.008),
    )
    side_wall_y = tray_width / 2.0 - wall_thickness / 2.0
    left_wall = _box(
        (tray_length, wall_thickness, wall_height),
        (tray_start + tray_length / 2.0, side_wall_y, -0.007),
    )
    right_wall = _box(
        (tray_length, wall_thickness, wall_height),
        (tray_start + tray_length / 2.0, -side_wall_y, -0.007),
    )
    left_strut = _box((tray_start, ear_thickness, 0.015), (tray_start / 2.0, plate_center_y, -0.011))
    right_strut = _box((tray_start, ear_thickness, 0.015), (tray_start / 2.0, -plate_center_y, -0.011))

    ear_blank = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .rect(0.032, 0.032)
        .extrude(ear_thickness)
        .translate((0.0, ear_thickness / 2.0, 0.0))
    )
    hole = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(rear_hole_radius)
        .extrude(ear_thickness * 2.5)
        .translate((0.0, ear_thickness * 1.25, 0.0))
    )
    ear = ear_blank.cut(hole)
    left_ear = ear.translate((0.0, plate_center_y, 0.0))
    right_ear = ear.translate((0.0, -plate_center_y, 0.0))

    under_keel = _box((0.020, 0.010, 0.012), (tray_start * 0.55, 0.0, -0.016))

    return _compound(
        floor,
        front_lip,
        rear_wall,
        left_wall,
        right_wall,
        left_strut,
        right_strut,
        left_ear,
        right_ear,
        under_keel,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_bracket_arm")

    dark_base = model.material("dark_base", rgba=(0.18, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.66, 0.70, 1.0))
    light_steel = model.material("light_steel", rgba=(0.74, 0.77, 0.80, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.48, 0.52, 0.56, 1.0))

    link1_span = 0.038 + 2.0 * 0.007
    link2_span = 0.032 + 2.0 * 0.006
    link3_span = 0.026 + 2.0 * 0.005
    tray_span = 0.022 + 2.0 * 0.0045

    foot_joint = (0.030, 0.0, 0.115)

    foot = model.part("foot")
    foot.visual(
        mesh_from_cadquery(
            _grounded_foot(
                joint_origin=foot_joint,
                lug_width=0.038 - 0.002,
                axle_radius=0.010,
                axle_length=link1_span,
            ),
            "foot",
        ),
        material=dark_base,
        name="foot_shell",
    )

    link_long_base = model.part("link_long_base")
    link_long_base.visual(
        mesh_from_cadquery(
            _bracket_link(
                pitch=0.240,
                body_end=0.195,
                rear_height=0.086,
                tip_height=0.060,
                waist_height=0.050,
                plate_thickness=0.007,
                inner_gap=0.038,
                rear_hole_radius=0.010,
                front_lug_width=0.032 - 0.002,
                front_lug_height=0.042,
                front_lug_back=0.018,
                front_lug_forward=0.014,
                front_axle_radius=0.0085,
                front_axle_length=link2_span,
                spacer_radius=0.0075,
                spacer_positions=(0.048, 0.110, 0.165),
            ),
            "link_long_base",
        ),
        material=steel,
        name="link_long_base_shell",
    )

    link_short_mid = model.part("link_short_mid")
    link_short_mid.visual(
        mesh_from_cadquery(
            _bracket_link(
                pitch=0.145,
                body_end=0.113,
                rear_height=0.070,
                tip_height=0.050,
                waist_height=0.040,
                plate_thickness=0.006,
                inner_gap=0.032,
                rear_hole_radius=0.0085,
                front_lug_width=0.026 - 0.002,
                front_lug_height=0.034,
                front_lug_back=0.016,
                front_lug_forward=0.012,
                front_axle_radius=0.0072,
                front_axle_length=link3_span,
                spacer_radius=0.0060,
                spacer_positions=(0.040, 0.078, 0.100),
            ),
            "link_short_mid",
        ),
        material=steel,
        name="link_short_mid_shell",
    )

    link_long_tip = model.part("link_long_tip")
    link_long_tip.visual(
        mesh_from_cadquery(
            _bracket_link(
                pitch=0.195,
                body_end=0.156,
                rear_height=0.056,
                tip_height=0.042,
                waist_height=0.032,
                plate_thickness=0.005,
                inner_gap=0.026,
                rear_hole_radius=0.0072,
                front_lug_width=0.022 - 0.002,
                front_lug_height=0.028,
                front_lug_back=0.014,
                front_lug_forward=0.010,
                front_axle_radius=0.0062,
                front_axle_length=tray_span,
                spacer_radius=0.0045,
                spacer_positions=(0.036, 0.098, 0.140),
            ),
            "link_long_tip",
        ),
        material=light_steel,
        name="link_long_tip_shell",
    )

    tray_bracket = model.part("tray_bracket")
    tray_bracket.visual(
        mesh_from_cadquery(
            _tray_end_bracket(
                tray_length=0.118,
                tray_width=0.082,
                floor_thickness=0.0045,
                wall_height=0.026,
                wall_thickness=0.0045,
                ear_thickness=0.0045,
                ear_inner_gap=0.022,
                rear_hole_radius=0.0062,
            ),
            "tray_bracket",
        ),
        material=tray_finish,
        name="tray_bracket_shell",
    )

    shoulder = model.articulation(
        "foot_to_link_long_base",
        ArticulationType.REVOLUTE,
        parent=foot,
        child=link_long_base,
        origin=Origin(xyz=foot_joint),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=-1.15,
            upper=1.10,
        ),
    )
    elbow_a = model.articulation(
        "link_long_base_to_link_short_mid",
        ArticulationType.REVOLUTE,
        parent=link_long_base,
        child=link_short_mid,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.8,
            lower=-1.35,
            upper=1.30,
        ),
    )
    elbow_b = model.articulation(
        "link_short_mid_to_link_long_tip",
        ArticulationType.REVOLUTE,
        parent=link_short_mid,
        child=link_long_tip,
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=11.0,
            velocity=2.0,
            lower=-1.45,
            upper=1.40,
        ),
    )
    wrist = model.articulation(
        "link_long_tip_to_tray_bracket",
        ArticulationType.REVOLUTE,
        parent=link_long_tip,
        child=tray_bracket,
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=7.0,
            velocity=2.2,
            lower=-1.05,
            upper=0.90,
        ),
    )

    model.meta["joint_names"] = (
        shoulder.name,
        elbow_a.name,
        elbow_b.name,
        wrist.name,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foot = object_model.get_part("foot")
    link_long_base = object_model.get_part("link_long_base")
    link_short_mid = object_model.get_part("link_short_mid")
    link_long_tip = object_model.get_part("link_long_tip")
    tray_bracket = object_model.get_part("tray_bracket")

    shoulder = object_model.get_articulation("foot_to_link_long_base")
    elbow_a = object_model.get_articulation("link_long_base_to_link_short_mid")
    elbow_b = object_model.get_articulation("link_short_mid_to_link_long_tip")
    wrist = object_model.get_articulation("link_long_tip_to_tray_bracket")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
    ctx.allow_overlap(
        foot,
        link_long_base,
        reason="Grounded shoulder is represented as an interleaved clevis-and-pivot bracket with shared hinge envelope.",
    )
    ctx.allow_overlap(
        link_long_base,
        link_short_mid,
        reason="Adjacent long-to-short link pair intentionally nests at the revolute clevis joint.",
    )
    ctx.allow_overlap(
        link_short_mid,
        link_long_tip,
        reason="Adjacent short-to-long link pair intentionally nests at the revolute clevis joint.",
    )
    ctx.allow_overlap(
        link_long_tip,
        tray_bracket,
        reason="Tray wrist uses a nested ear-and-lug bracket around the final hinge axis.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for part in (foot, link_long_base, link_short_mid, link_long_tip, tray_bracket):
        ctx.check(f"{part.name}_present", part is not None)

    for joint in (shoulder, elbow_a, elbow_b, wrist):
        ctx.check(
            f"{joint.name}_parallel_y_axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis was {joint.axis}",
        )

    ctx.expect_contact(link_long_base, foot, name="foot_to_base_link_contact")
    ctx.expect_contact(link_short_mid, link_long_base, name="base_to_mid_link_contact")
    ctx.expect_contact(link_long_tip, link_short_mid, name="mid_to_tip_link_contact")
    ctx.expect_contact(tray_bracket, link_long_tip, name="tip_to_tray_contact")

    foot_aabb = ctx.part_world_aabb(foot)
    if foot_aabb is not None:
        ctx.check(
            "foot_reaches_ground_plane",
            abs(foot_aabb[0][2]) <= 0.002,
            details=f"foot min z was {foot_aabb[0][2]:.4f}",
        )

    link1_aabb = ctx.part_world_aabb(link_long_base)
    link2_aabb = ctx.part_world_aabb(link_short_mid)
    link3_aabb = ctx.part_world_aabb(link_long_tip)
    if link1_aabb is not None and link2_aabb is not None and link3_aabb is not None:
        link1_x = link1_aabb[1][0] - link1_aabb[0][0]
        link2_x = link2_aabb[1][0] - link2_aabb[0][0]
        link3_x = link3_aabb[1][0] - link3_aabb[0][0]
        ctx.check(
            "alternating_long_short_link_lengths",
            link1_x > link2_x and link3_x > link2_x,
            details=f"link x spans were {link1_x:.3f}, {link2_x:.3f}, {link3_x:.3f}",
        )

    if foot_aabb is not None and link1_aabb is not None and link3_aabb is not None:
        foot_y = foot_aabb[1][1] - foot_aabb[0][1]
        link1_y = link1_aabb[1][1] - link1_aabb[0][1]
        link3_y = link3_aabb[1][1] - link3_aabb[0][1]
        ctx.check(
            "base_is_visually_denser_than_tip",
            foot_y > link1_y > link3_y,
            details=f"y spans were {foot_y:.3f}, {link1_y:.3f}, {link3_y:.3f}",
        )

    with ctx.pose(
        {
            shoulder: 0.55,
            elbow_a: -0.85,
            elbow_b: 0.70,
            wrist: 0.20,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="posed_clearance_check")
        ctx.expect_gap(
            link_short_mid,
            foot,
            axis="x",
            min_gap=0.015,
            name="posed_mid_link_clears_foot",
        )
        ctx.expect_gap(
            tray_bracket,
            link_long_base,
            axis="x",
            min_gap=0.015,
            name="posed_tray_clears_base_link",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
