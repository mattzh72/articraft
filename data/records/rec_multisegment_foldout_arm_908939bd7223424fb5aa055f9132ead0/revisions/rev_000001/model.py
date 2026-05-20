from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


JOINT_AXIS = (0.0, 1.0, 0.0)

CLEVIS_GAP = 0.010
SIDE_PLATE_T = 0.0032
CLEVIS_OUTER_W = CLEVIS_GAP + (2.0 * SIDE_PLATE_T)
EYE_W = CLEVIS_GAP
PIN_R = 0.0038
PIN_HEAD_R = 0.0065
PIN_HEAD_T = 0.0024
BASE_HINGE_X = 0.030
BASE_HINGE_Z = 0.040

BASE_PLATE_L = 0.150
BASE_PLATE_W = 0.090
BASE_PLATE_T = 0.008

LINK_LENGTHS = (0.115, 0.105, 0.095, 0.085)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _cut_side_pockets(
    body: cq.Workplane,
    *,
    center_x: float,
    pocket_len: float,
    body_w: float,
    pocket_h: float,
    pocket_depth: float,
) -> cq.Workplane:
    cutters = []
    for sign in (-1.0, 1.0):
        cutters.append(
            cq.Workplane("XY")
            .box(pocket_len, pocket_depth + 0.0012, pocket_h)
            .translate(
                (
                    center_x,
                    sign * ((body_w / 2.0) - (pocket_depth / 2.0) + 0.0006),
                    0.0,
                )
            )
        )
    for cutter in cutters:
        body = body.cut(cutter)
    return body


def _make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.146, BASE_PLATE_W, BASE_PLATE_T)
        .edges("|Z")
        .fillet(0.008)
        .translate((-0.038, 0.0, -0.036))
    )

    for x_pos in (-0.070, 0.018):
        for y_pos in (-0.028, 0.028):
            hole = (
                cq.Workplane("XY")
                .circle(0.0045)
                .extrude(BASE_PLATE_T + 0.004, both=True)
                .translate((x_pos, y_pos, -0.036))
            )
            plate = plate.cut(hole)

    pedestal = cq.Workplane("XY").box(0.076, 0.052, 0.014).translate((-0.016, 0.0, -0.025))
    tower = cq.Workplane("XY").box(0.022, 0.020, 0.038).translate((0.006, 0.0, -0.001))
    front_spine = cq.Workplane("XY").box(0.014, 0.016, 0.014).translate((0.016, 0.0, 0.024))
    lower_web = cq.Workplane("XY").box(0.018, 0.018, 0.008).translate((0.016, 0.0, 0.013))
    rear_gusset = cq.Workplane("XY").box(0.018, 0.028, 0.010).translate((-0.006, 0.0, 0.007))
    saddle = cq.Workplane("XY").box(0.012, 0.022, 0.008).translate((0.022, 0.0, 0.030))

    shape = plate.union(pedestal).union(tower).union(front_spine).union(lower_web).union(rear_gusset).union(saddle)

    cheek_x = BASE_HINGE_X - 0.012
    cheek_h = 0.024
    cheek_len = 0.012
    cheek_center_y = (CLEVIS_GAP / 2.0) + (SIDE_PLATE_T / 2.0)
    for sign in (-1.0, 1.0):
        cheek = cq.Workplane("XY").box(cheek_len, SIDE_PLATE_T, cheek_h).translate(
            (cheek_x, sign * cheek_center_y, BASE_HINGE_Z)
        )
        shape = shape.union(cheek)
        shape = shape.union(
            _y_cylinder(
                0.0062,
                PIN_HEAD_T,
                (
                    BASE_HINGE_X - 0.010,
                    sign * ((CLEVIS_GAP / 2.0) + SIDE_PLATE_T + (PIN_HEAD_T / 2.0)),
                    BASE_HINGE_Z,
                ),
            )
        )

    hinge_hole = _y_cylinder(PIN_R, CLEVIS_OUTER_W + 0.004, (BASE_HINGE_X, 0.0, BASE_HINGE_Z))
    stop_tab = cq.Workplane("XY").box(0.010, 0.0035, 0.008).translate(
        (0.010, (CLEVIS_OUTER_W / 2.0) + 0.00175, BASE_HINGE_Z + 0.004)
    )

    shape = shape.cut(hinge_hole).union(stop_tab)
    return shape


def _make_pin_shape() -> cq.Workplane:
    shaft = _y_cylinder(PIN_R, CLEVIS_OUTER_W, (0.0, 0.0, 0.0))
    head_center_y = (CLEVIS_OUTER_W / 2.0) + (PIN_HEAD_T / 2.0)
    head = _y_cylinder(PIN_HEAD_R, PIN_HEAD_T, (0.0, head_center_y, 0.0))
    head = head.union(_y_cylinder(PIN_HEAD_R, PIN_HEAD_T, (0.0, -head_center_y, 0.0)))
    hex_drive = cq.Workplane("XY").box(0.0035, PIN_HEAD_T, 0.0035).translate((0.0, head_center_y, 0.0))
    return shaft.union(head).union(hex_drive)


def _make_link_shape(
    *,
    length: float,
    mid_w: float,
    mid_h: float,
    station_h: float,
    boss_r: float,
    stop_sign: float,
    terminal: bool = False,
) -> cq.Workplane:
    knuckle_r = boss_r
    neck_h = station_h - 0.008
    mid_start = 0.030
    mid_end = length - (0.020 if not terminal else 0.006)
    mid_len = mid_end - mid_start
    mid_x = (mid_start + mid_end) / 2.0

    eye = _y_cylinder(knuckle_r, EYE_W, (0.0, 0.0, 0.0))
    tongue = cq.Workplane("XY").box(0.018, EYE_W, neck_h).translate((0.009, 0.0, 0.0))
    shoulder = cq.Workplane("XY").box(0.014, max(EYE_W + 0.004, mid_w - 0.008), neck_h - 0.002).translate(
        (0.022, 0.0, 0.0)
    )
    mid = cq.Workplane("XY").box(mid_len, mid_w, mid_h).translate((mid_x, 0.0, 0.0))
    cover = (
        cq.Workplane("XY")
        .box(mid_len * 0.62, max(CLEVIS_GAP + 0.004, mid_w * 0.72), mid_h + 0.003)
        .translate((mid_x + 0.001, 0.0, 0.002))
    )
    shape = eye.union(tongue).union(shoulder).union(mid).union(cover)
    shape = _cut_side_pockets(
        shape,
        center_x=mid_x,
        pocket_len=max(0.028, mid_len * 0.54),
        body_w=mid_w,
        pocket_h=mid_h * 0.56,
        pocket_depth=0.0026,
    )

    shape = shape.cut(_y_cylinder(PIN_R, EYE_W + 0.004, (0.0, 0.0, 0.0)))

    if terminal:
        pad_arm = cq.Workplane("XY").box(0.026, max(0.018, mid_w - 0.002), station_h - 0.006).translate(
            (length + 0.012, 0.0, 0.0)
        )
        terminal_pad = cq.Workplane("XY").box(0.028, 0.036, 0.010).translate((length + 0.036, 0.0, 0.0))
        fork_bridge = cq.Workplane("XY").box(0.016, 0.028, 0.010).translate((length + 0.002, 0.0, 0.0))
        fork_slot = cq.Workplane("XY").box(0.022, 0.014, station_h + 0.002).translate((length + 0.012, 0.0, 0.0))
        nose = _y_cylinder(0.006, 0.036, (length + 0.048, 0.0, 0.0))
        stop_tab = cq.Workplane("XY").box(0.009, 0.0032, 0.007).translate(
            (length - 0.006, stop_sign * ((mid_w / 2.0) + 0.0016), 0.007)
        )
        shape = shape.union(pad_arm).union(terminal_pad).union(fork_bridge).union(nose).cut(fork_slot).union(stop_tab)
        return shape

    lower_web = cq.Workplane("XY").box(0.014, EYE_W + 0.002, 0.009).translate(
        (length - 0.018, 0.0, -(station_h * 0.26))
    )
    rear_bridge = cq.Workplane("XY").box(0.012, max(EYE_W + 0.002, mid_w - 0.012), 0.008).translate(
        (length - 0.030, 0.0, -(station_h * 0.10))
    )
    stop_tab = cq.Workplane("XY").box(0.010, 0.0035, 0.008).translate(
        (length - 0.014, stop_sign * ((CLEVIS_OUTER_W / 2.0) + 0.00175), (station_h / 2.0) - 0.004)
    )
    shape = shape.union(lower_web).union(rear_bridge).union(stop_tab)
    cheek_len = 0.016
    cheek_center_y = (CLEVIS_GAP / 2.0) + (SIDE_PLATE_T / 2.0)
    for sign in (-1.0, 1.0):
        cheek = cq.Workplane("XY").box(cheek_len, SIDE_PLATE_T, station_h).translate(
            (length - 0.014, sign * cheek_center_y, 0.0)
        )
        shape = shape.union(cheek)
        shape = shape.union(
            _y_cylinder(
                0.006,
                PIN_HEAD_T,
                (
                    length - 0.012,
                    sign * ((CLEVIS_GAP / 2.0) + SIDE_PLATE_T + (PIN_HEAD_T / 2.0)),
                    0.0,
                ),
            )
        )
    shape = shape.cut(_y_cylinder(PIN_R, CLEVIS_OUTER_W + 0.004, (length, 0.0, 0.0)))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_foldout_boom")

    model.material("powder_base", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("link_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("link_dark", rgba=(0.55, 0.58, 0.62, 1.0))

    base = model.part("base_plate")
    base.visual(mesh_from_cadquery(_make_base_shape(), "base_plate"), material="powder_base", name="base_shell")
    base.inertial = Inertial.from_geometry(
        Box((0.150, 0.090, 0.040)),
        mass=1.35,
        origin=Origin(xyz=(-0.026, 0.0, -0.022)),
    )

    link_specs = (
        ("link1", LINK_LENGTHS[0], 0.026, 0.020, 0.028, 0.0105, 1.0, False, "link_silver", 0.24),
        ("link2", LINK_LENGTHS[1], 0.024, 0.019, 0.026, 0.0098, -1.0, False, "link_dark", 0.20),
        ("link3", LINK_LENGTHS[2], 0.022, 0.018, 0.024, 0.0091, 1.0, False, "link_silver", 0.17),
        ("link4", LINK_LENGTHS[3], 0.020, 0.017, 0.022, 0.0085, -1.0, True, "link_dark", 0.15),
    )

    parts_by_name: dict[str, object] = {"base_plate": base}
    for name, length, mid_w, mid_h, station_h, boss_r, stop_sign, terminal, material, mass in link_specs:
        part = model.part(name)
        part.visual(
            mesh_from_cadquery(
                _make_link_shape(
                    length=length,
                    mid_w=mid_w,
                    mid_h=mid_h,
                    station_h=station_h,
                    boss_r=boss_r,
                    stop_sign=stop_sign,
                    terminal=terminal,
                ),
                f"{name}_shell",
            ),
            material=material,
            name=f"{name}_shell",
        )
        box_len = length + (0.040 if terminal else 0.028)
        box_center_x = (box_len / 2.0) - 0.004
        part.inertial = Inertial.from_geometry(
            Box((box_len, max(mid_w, 0.024), station_h)),
            mass=mass,
            origin=Origin(xyz=(box_center_x, 0.0, 0.0)),
        )
        parts_by_name[name] = part

    model.articulation(
        "base_to_link1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=parts_by_name["link1"],
        origin=Origin(xyz=(BASE_HINGE_X, 0.0, BASE_HINGE_Z)),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=0.0, upper=0.82, effort=18.0, velocity=1.8),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=parts_by_name["link1"],
        child=parts_by_name["link2"],
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=-2.05, upper=0.18, effort=15.0, velocity=2.2),
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=parts_by_name["link2"],
        child=parts_by_name["link3"],
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=-0.18, upper=2.05, effort=13.0, velocity=2.2),
    )
    model.articulation(
        "link3_to_link4",
        ArticulationType.REVOLUTE,
        parent=parts_by_name["link3"],
        child=parts_by_name["link4"],
        origin=Origin(xyz=(LINK_LENGTHS[2], 0.0, 0.0)),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=-1.65, upper=0.22, effort=10.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")
    link4 = object_model.get_part("link4")

    j1 = object_model.get_articulation("base_to_link1")
    j2 = object_model.get_articulation("link1_to_link2")
    j3 = object_model.get_articulation("link2_to_link3")
    j4 = object_model.get_articulation("link3_to_link4")

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
        base,
        link1,
        reason="Link1's central hinge eye intentionally nests inside the base bracket around the shared pin axis.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "four_serial_revolute_joints",
        len((j1, j2, j3, j4)) == 4
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in (j1, j2, j3, j4)),
        "Expected a four-joint serial revolute chain.",
    )
    for joint in (j1, j2, j3, j4):
        ctx.check(
            f"{joint.name}_axis_is_in_plane",
            tuple(joint.axis) == JOINT_AXIS,
            f"Expected axis {JOINT_AXIS}, got {joint.axis}.",
        )

    ctx.expect_contact(base, link1, contact_tol=0.0008, name="base_joint_supported")
    ctx.expect_contact(link1, link2, contact_tol=0.0008, name="joint_1_supported")
    ctx.expect_contact(link2, link3, contact_tol=0.0008, name="joint_2_supported")
    ctx.expect_contact(link3, link4, contact_tol=0.0008, name="joint_3_supported")

    ctx.expect_origin_distance(base, link4, axes="x", min_dist=0.30, name="open_pose_long_reach")

    transition_pose = {j1: 0.22, j2: -0.84, j3: 0.92, j4: -0.74}
    stowed_pose = {j1: 0.34, j2: -1.56, j3: 1.66, j4: -1.34}

    with ctx.pose(transition_pose):
        ctx.expect_origin_distance(base, link4, axes="x", min_dist=0.31, name="transition_pose_reach")

    with ctx.pose(stowed_pose):
        ctx.expect_origin_distance(base, link4, axes="x", max_dist=0.28, name="stowed_pose_compact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
