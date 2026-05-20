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


def box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def x_cylinder(
    x0: float,
    x1: float,
    radius: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("YZ").center(y, z).circle(radius).extrude(x1 - x0).translate((x0, 0.0, 0.0))


def z_cylinder(
    x: float,
    y: float,
    z0: float,
    z1: float,
    radius: float,
) -> cq.Workplane:
    return cq.Workplane("XY").center(x, y).circle(radius).extrude(z1 - z0).translate((0.0, 0.0, z0))


def y_cylinder(
    y0: float,
    y1: float,
    radius: float,
    *,
    x: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    y_min = min(y0, y1)
    y_max = max(y0, y1)
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(y_max - y_min).translate((0.0, y_min, 0.0))


def union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def bolt_circle_points(radius: float, count: int, phase: float = 0.0) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for idx in range(count):
        angle = phase + (2.0 * math.pi * idx / count)
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def build_frame_base() -> cq.Workplane:
    rail_len = 0.38
    rail_w = 0.028
    rail_h = 0.018
    rail_y = 0.062
    rail_x = 0.03

    cross_t = 0.022
    cross_w = 0.152
    cross_h = 0.018
    cross_xs = (-0.12, 0.0, 0.14)

    foot_l = 0.05
    foot_w = 0.042
    foot_h = 0.01
    foot_xs = (-0.14, 0.20)
    foot_ys = (-rail_y, rail_y)

    pedestal = box_at((0.128, 0.11, 0.024), (0.0, 0.0, 0.028))

    solids = [
        box_at((rail_len, rail_w, rail_h), (rail_x, -rail_y, rail_h * 0.5)),
        box_at((rail_len, rail_w, rail_h), (rail_x, rail_y, rail_h * 0.5)),
        pedestal,
    ]
    solids.extend(box_at((cross_t, cross_w, cross_h), (x, 0.0, cross_h * 0.5)) for x in cross_xs)
    solids.extend(
        box_at((foot_l, foot_w, foot_h), (x, y, foot_h * 0.5))
        for x in foot_xs
        for y in foot_ys
    )

    frame_base = union_all(*solids)

    hole_cutters = [
        z_cylinder(x, y, -0.001, foot_h + 0.001, 0.0055)
        for x in foot_xs
        for y in foot_ys
    ]
    return frame_base.cut(union_all(*hole_cutters))


def build_support_pedestal() -> cq.Workplane:
    return box_at((0.088, 0.07, 0.068), (0.0, 0.0, 0.074))


def build_support_mounts() -> cq.Workplane:
    left_mount = box_at((0.024, 0.056, 0.068), (-0.076, 0.0, 0.074))
    right_mount = box_at((0.024, 0.056, 0.068), (0.076, 0.0, 0.074))
    return union_all(left_mount, right_mount)


def build_housing_cheek(side: str) -> cq.Workplane:
    sign = -1.0 if side == "left" else 1.0
    cheek = box_at((0.092, 0.022, 0.082), (0.0, sign * 0.047, 0.149))
    rib = box_at((0.05, 0.022, 0.032), (-0.008, sign * 0.047, 0.121))
    outer_cover = y_cylinder(sign * 0.058, sign * 0.072, 0.018, z=0.145)
    bushing = y_cylinder(sign * 0.036, sign * 0.041, 0.024, z=0.145).cut(
        y_cylinder(sign * 0.035, sign * 0.042, 0.017, z=0.145)
    )
    return union_all(cheek, rib, outer_cover, bushing)


def build_top_cap() -> cq.Workplane:
    cap = box_at((0.096, 0.124, 0.016), (0.0, 0.0, 0.198))
    pad_left = box_at((0.032, 0.026, 0.012), (-0.026, 0.0, 0.184))
    pad_right = box_at((0.032, 0.026, 0.012), (0.026, 0.0, 0.184))
    return union_all(cap, pad_left, pad_right)


def build_cap_screw(x: float, y: float) -> cq.Workplane:
    shank = z_cylinder(x, y, 0.206, 0.221, 0.0035)
    head = z_cylinder(x, y, 0.221, 0.226, 0.0065)
    return union_all(shank, head)


def build_housing() -> cq.Workplane:
    axis_z = 0.145
    left_upright = box_at((0.082, 0.018, 0.066), (0.0, -0.052, 0.143))
    right_upright = box_at((0.082, 0.018, 0.066), (0.0, 0.052, 0.143))
    left_rib = box_at((0.044, 0.018, 0.028), (-0.012, -0.052, 0.121))
    right_rib = box_at((0.044, 0.018, 0.028), (-0.012, 0.052, 0.121))
    cap = box_at((0.096, 0.122, 0.014), (0.0, 0.0, 0.198))
    cap_leg_left = box_at((0.022, 0.022, 0.02), (-0.028, -0.032, 0.181))
    cap_leg_right = box_at((0.022, 0.022, 0.02), (-0.028, 0.032, 0.181))
    left_thrust_plate = x_cylinder(-0.048, -0.044, 0.03, z=axis_z).cut(
        x_cylinder(-0.049, -0.043, 0.0162, z=axis_z)
    )
    right_thrust_plate = x_cylinder(0.044, 0.048, 0.03, z=axis_z).cut(
        x_cylinder(0.043, 0.049, 0.0162, z=axis_z)
    )
    cap_screws = [build_cap_screw(x, y) for x in (-0.028, 0.028) for y in (-0.044, 0.044)]

    return union_all(
        build_support_pedestal(),
        left_upright,
        right_upright,
        left_rib,
        right_rib,
        cap,
        cap_leg_left,
        cap_leg_right,
        left_thrust_plate,
        right_thrust_plate,
        *cap_screws,
    )


def build_side_support(side: str) -> cq.Workplane:
    sign = -1.0 if side == "left" else 1.0
    y_center = sign * 0.018
    main_block = box_at((0.082, 0.036, 0.094), (0.0, y_center, 0.155))
    lower_foot = box_at((0.07, 0.032, 0.018), (0.0, y_center, 0.117))
    gusset = box_at((0.052, 0.036, 0.024), (-0.008, y_center, 0.129))
    cap_pad = box_at((0.072, 0.032, 0.008), (0.0, y_center, 0.198))
    screw_boss_front = z_cylinder(0.024, y_center, 0.184, 0.202, 0.005)
    screw_boss_rear = z_cylinder(-0.024, y_center, 0.184, 0.202, 0.005)
    support = union_all(
        main_block,
        lower_foot,
        gusset,
        cap_pad,
        screw_boss_front,
        screw_boss_rear,
    )
    journal_bore = x_cylinder(-0.041, 0.041, 0.0155, z=0.145)
    return support.cut(journal_bore)


def build_cap_assembly() -> cq.Workplane:
    cap_plate = box_at((0.096, 0.074, 0.014), (0.0, 0.0, 0.209))
    screws = [build_cap_screw(x, y) for x in (-0.024, 0.024) for y in (-0.018, 0.018)]
    return union_all(cap_plate, *screws)


def build_shaft_journal() -> cq.Workplane:
    main_journal = x_cylinder(-0.11, 0.09, 0.0155)
    left_collar = x_cylinder(-0.058, -0.048, 0.024)
    right_collar = x_cylinder(0.048, 0.058, 0.024)
    left_retainer = x_cylinder(-0.124, -0.11, 0.024)
    return union_all(main_journal, left_collar, right_collar, left_retainer)


def build_shaft_hub() -> cq.Workplane:
    hub_transition = (
        cq.Workplane("YZ")
        .circle(0.0155)
        .workplane(offset=0.032)
        .circle(0.03)
        .loft()
        .translate((0.058, 0.0, 0.0))
    )
    drum_seat = x_cylinder(0.09, 0.112, 0.03)
    outer_shoulder = x_cylinder(0.104, 0.12, 0.036)
    return union_all(hub_transition, drum_seat, outer_shoulder)


def build_shaft_stage() -> cq.Workplane:
    return union_all(build_shaft_journal(), build_shaft_hub())


def build_drum() -> cq.Workplane:
    shell = x_cylinder(0.098, 0.318, 0.09)
    left_cover_boss = x_cylinder(0.086, 0.104, 0.04)
    right_cover_boss = x_cylinder(0.312, 0.33, 0.04)
    cavity = x_cylinder(0.106, 0.31, 0.078)

    drum = union_all(shell, left_cover_boss, right_cover_boss).cut(cavity)

    screw_solids: list[cq.Workplane] = []
    for y, z in bolt_circle_points(0.064, 6, phase=math.pi / 6.0):
        screw_solids.append(x_cylinder(0.088, 0.098, 0.0045, y=y, z=z))
        screw_solids.append(x_cylinder(0.318, 0.328, 0.0045, y=y, z=z))

    return drum.union(union_all(*screw_solids))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_drum_module")

    frame_color = model.material("frame_color", color=(0.20, 0.22, 0.25, 1.0))
    housing_color = model.material("housing_color", color=(0.64, 0.66, 0.68, 1.0))
    shaft_color = model.material("shaft_color", color=(0.55, 0.57, 0.60, 1.0))
    drum_color = model.material("drum_color", color=(0.34, 0.36, 0.39, 1.0))

    axis_z = 0.145
    frame_base_shape = build_frame_base()
    pedestal_shape = build_support_pedestal()
    support_mounts_shape = build_support_mounts()
    left_support_shape = build_side_support("left")
    right_support_shape = build_side_support("right")
    cap_shape = build_cap_assembly()
    shaft_stage = build_shaft_stage()
    drum = build_drum()

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(frame_base_shape, "frame_base"),
        material=frame_color,
        name="frame_base",
    )
    frame.visual(
        mesh_from_cadquery(pedestal_shape, "center_pedestal"),
        material=frame_color,
        name="center_pedestal",
    )
    frame.visual(
        mesh_from_cadquery(support_mounts_shape, "support_mounts"),
        material=frame_color,
        name="support_mounts",
    )

    left_support = model.part("left_support")
    left_support.visual(
        mesh_from_cadquery(left_support_shape, "left_support"),
        material=housing_color,
        name="left_support",
    )

    right_support = model.part("right_support")
    right_support.visual(
        mesh_from_cadquery(right_support_shape, "right_support"),
        material=housing_color,
        name="right_support",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(cap_shape, "top_cap"),
        material=housing_color,
        name="top_cap",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_cadquery(shaft_stage, "shaft_stage"),
        material=shaft_color,
        name="shaft_stage",
    )
    rotor.visual(
        mesh_from_cadquery(drum, "drum_body"),
        material=drum_color,
        name="drum",
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=12.0),
    )
    model.articulation(
        "frame_to_left_support",
        ArticulationType.FIXED,
        parent=frame,
        child=left_support,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_right_support",
        ArticulationType.FIXED,
        parent=frame,
        child=right_support,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_cap",
        ArticulationType.FIXED,
        parent=frame,
        child=cap,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    stage = object_model.get_articulation("frame_to_rotor")
    frame_base = frame.get_visual("frame_base")
    pedestal = frame.get_visual("center_pedestal")
    support_mounts = frame.get_visual("support_mounts")
    left_support_visual = left_support.get_visual("left_support")
    right_support_visual = right_support.get_visual("right_support")
    cap_visual = cap.get_visual("top_cap")
    shaft_stage = rotor.get_visual("shaft_stage")
    drum = rotor.get_visual("drum")

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
        left_support,
        rotor,
        elem_a=left_support_visual,
        elem_b=shaft_stage,
        reason="split bearing support is modeled as a close running journal capture around the shaft stage",
    )
    ctx.allow_overlap(
        right_support,
        rotor,
        elem_a=right_support_visual,
        elem_b=shaft_stage,
        reason="split bearing support is modeled as a close running journal capture around the shaft stage",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    axis = stage.axis
    ctx.check(
        "articulation axis follows shaft centerline",
        abs(axis[0] - 1.0) < 1e-9 and abs(axis[1]) < 1e-9 and abs(axis[2]) < 1e-9,
        f"expected +X revolute axis, got {axis}",
    )
    ctx.check(
        "rotor stage is continuous",
        stage.articulation_type == ArticulationType.CONTINUOUS and stage.motion_limits is not None,
        "rotor should spin as one continuous stage about the supported shaft axis",
    )

    ctx.expect_contact(
        frame,
        left_support,
        elem_a=pedestal,
        elem_b=left_support_visual,
        contact_tol=5e-5,
        name="left support is mounted to the frame core",
    )
    ctx.expect_contact(
        frame,
        right_support,
        elem_a=pedestal,
        elem_b=right_support_visual,
        contact_tol=5e-5,
        name="right support is mounted to the frame core",
    )
    ctx.expect_contact(
        cap,
        left_support,
        elem_a=cap_visual,
        elem_b=left_support_visual,
        contact_tol=5e-5,
        name="cap clamps onto the left support",
    )
    ctx.expect_contact(
        cap,
        right_support,
        elem_a=cap_visual,
        elem_b=right_support_visual,
        contact_tol=5e-5,
        name="cap clamps onto the right support",
    )
    ctx.expect_contact(
        left_support,
        rotor,
        elem_a=left_support_visual,
        elem_b=shaft_stage,
        contact_tol=5e-5,
        name="left support captures the shaft collar",
    )
    ctx.expect_contact(
        right_support,
        rotor,
        elem_a=right_support_visual,
        elem_b=shaft_stage,
        contact_tol=5e-5,
        name="right support captures the shaft collar",
    )
    ctx.expect_gap(
        rotor,
        left_support,
        axis="x",
        positive_elem=drum,
        negative_elem=left_support_visual,
        min_gap=0.03,
        name="drum clears the left support axially",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem=drum,
        negative_elem=frame_base,
        min_gap=0.012,
        name="drum clears the low base frame",
    )

    with ctx.pose({stage: math.pi / 2.0}):
        ctx.expect_contact(
            left_support,
            rotor,
            elem_a=left_support_visual,
            elem_b=shaft_stage,
            contact_tol=5e-5,
            name="left support contact remains at quarter turn",
        )
        ctx.expect_contact(
            right_support,
            rotor,
            elem_a=right_support_visual,
            elem_b=shaft_stage,
            contact_tol=5e-5,
            name="right support contact remains at quarter turn",
        )
        ctx.expect_gap(
            rotor,
            left_support,
            axis="x",
            positive_elem=drum,
            negative_elem=left_support_visual,
            min_gap=0.03,
            name="drum still clears the left support at quarter turn",
        )
        ctx.expect_gap(
            rotor,
            frame,
            axis="z",
            positive_elem=drum,
            negative_elem=frame_base,
            min_gap=0.012,
            name="drum still clears the frame at quarter turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
