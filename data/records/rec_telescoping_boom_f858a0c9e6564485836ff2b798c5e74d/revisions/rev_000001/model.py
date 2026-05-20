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
)


OUTER_LEN = 1.55
OUTER_W = 0.32
OUTER_H = 0.24
OUTER_WALL = 0.012
OUTER_REAR_CAP = 0.02

MID_LEN = 1.32
MID_W = 0.26
MID_H = 0.18
MID_WALL = 0.01

INNER_TUBE_LEN = 0.86
INNER_TRANSITION_LEN = 0.08
INNER_LUG_LEN = 0.18
INNER_LEN = INNER_TUBE_LEN + INNER_TRANSITION_LEN + INNER_LUG_LEN
INNER_W = 0.20
INNER_H = 0.13
INNER_WALL = 0.008

BRACKET_T = 0.045
BRACKET_W = 0.52
BRACKET_H = 0.62
BRACKET_SLEEVE_LEN = 0.19
BRACKET_CLEAR = 0.0

MID_RETRACTED_X = 0.38
INNER_RETRACTED_X = 0.40

MID_EXTENSION = 0.62
INNER_EXTENSION = 0.50


def make_box_tube(
    length: float,
    width: float,
    height: float,
    wall: float,
    *,
    rear_cap: float = 0.0,
    front_cap: float = 0.0,
) -> cq.Workplane:
    body = cq.Workplane("XY").box(length, width, height).translate((length / 2.0, 0.0, 0.0))

    inner_w = width - 2.0 * wall
    inner_h = height - 2.0 * wall
    x0 = rear_cap
    x1 = length - front_cap
    if rear_cap <= 1e-6:
        x0 -= 0.001
    if front_cap <= 1e-6:
        x1 += 0.001
    inner_len = x1 - x0

    if inner_len > 0.0 and inner_w > 0.0 and inner_h > 0.0:
        inner = (
            cq.Workplane("XY")
            .box(inner_len, inner_w, inner_h)
            .translate(((x0 + x1) / 2.0, 0.0, 0.0))
        )
        body = body.cut(inner)
    return body


def add_slider_pads(
    body: cq.Workplane,
    *,
    width: float,
    height: float,
    length: float,
    pad_thickness: float,
    pad_length: float,
    pad_width: float,
    rear_offset: float,
    front_offset: float,
) -> cq.Workplane:
    pad_centers = (
        rear_offset + pad_length / 2.0,
        length - front_offset - pad_length / 2.0,
    )
    for x_center in pad_centers:
        for z_sign in (-1.0, 1.0):
            pad = (
                cq.Workplane("XY")
                .box(pad_length, pad_width, pad_thickness)
                .translate((x_center, 0.0, z_sign * (height / 2.0 + pad_thickness / 2.0)))
            )
            body = body.union(pad)
    return body


def x_cylinder(length: float, radius: float, center_x: float, center_y: float, center_z: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .center(center_y, center_z)
        .circle(radius)
        .extrude(length)
        .translate((center_x - length / 2.0, 0.0, 0.0))
    )


def y_cylinder(length: float, radius: float, center_x: float, center_y: float, center_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, center_y - length / 2.0, 0.0))
    )


def build_root_bracket() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BRACKET_T, BRACKET_W, BRACKET_H)
        .translate((-BRACKET_T / 2.0, 0.0, 0.0))
    )

    for y in (-0.17, 0.17):
        for z in (-0.22, 0.22):
            plate = plate.cut(
                x_cylinder(BRACKET_T + 0.01, 0.021, -BRACKET_T / 2.0, y, z)
            )

    sleeve_x_center = BRACKET_SLEEVE_LEN / 2.0 - 0.002
    cheek_t = 0.03
    cap_t = 0.05
    cheek_h = OUTER_H + 0.12
    cap_w = OUTER_W + 0.10

    left_cheek = (
        cq.Workplane("XY")
        .box(BRACKET_SLEEVE_LEN, cheek_t, cheek_h)
        .translate((sleeve_x_center, OUTER_W / 2.0 + BRACKET_CLEAR + cheek_t / 2.0, 0.0))
    )
    right_cheek = left_cheek.mirror("YZ")

    top_cap = (
        cq.Workplane("XY")
        .box(BRACKET_SLEEVE_LEN, cap_w, cap_t)
        .translate((sleeve_x_center, 0.0, OUTER_H / 2.0 + BRACKET_CLEAR + cap_t / 2.0))
    )
    bottom_cap = top_cap.mirror("XY")

    body = plate.union(left_cheek).union(right_cheek).union(top_cap).union(bottom_cap)
    return body


def build_outer_beam() -> cq.Workplane:
    return make_box_tube(
        OUTER_LEN,
        OUTER_W,
        OUTER_H,
        OUTER_WALL,
        rear_cap=OUTER_REAR_CAP,
        front_cap=0.0,
    )


def build_mid_beam() -> cq.Workplane:
    pad_t = (OUTER_H - 2.0 * OUTER_WALL - MID_H) / 2.0
    body = make_box_tube(MID_LEN, MID_W, MID_H, MID_WALL, rear_cap=0.0, front_cap=0.0)
    body = add_slider_pads(
        body,
        width=MID_W,
        height=MID_H,
        length=MID_LEN,
        pad_thickness=pad_t,
        pad_length=0.18,
        pad_width=MID_W - 0.06,
        rear_offset=0.09,
        front_offset=0.09,
    )
    return body


def build_inner_beam() -> cq.Workplane:
    pad_t = (MID_H - 2.0 * MID_WALL - INNER_H) / 2.0
    body = make_box_tube(
        INNER_TUBE_LEN,
        INNER_W,
        INNER_H,
        INNER_WALL,
        rear_cap=0.0,
        front_cap=0.012,
    )
    body = add_slider_pads(
        body,
        width=INNER_W,
        height=INNER_H,
        length=INNER_TUBE_LEN,
        pad_thickness=pad_t,
        pad_length=0.16,
        pad_width=INNER_W - 0.05,
        rear_offset=0.08,
        front_offset=0.08,
    )

    transition = (
        cq.Workplane("XY")
        .box(INNER_TRANSITION_LEN + 0.002, 0.17, 0.10)
        .translate((INNER_TUBE_LEN + INNER_TRANSITION_LEN / 2.0 - 0.001, 0.0, 0.0))
    )

    lug_width = 0.04
    lug_gap = 0.09
    lug_height = 0.09
    lug_x = INNER_TUBE_LEN + INNER_TRANSITION_LEN + INNER_LUG_LEN / 2.0 - 0.001
    lug_y = lug_gap / 2.0 + lug_width / 2.0

    left_lug = (
        cq.Workplane("XY")
        .box(INNER_LUG_LEN + 0.002, lug_width, lug_height)
        .translate((lug_x, lug_y, 0.0))
    )
    right_lug = left_lug.mirror("YZ")

    tip = body.union(transition).union(left_lug).union(right_lug)

    pin_hole = y_cylinder(
        2.0 * lug_width + lug_gap + 0.02,
        0.018,
        INNER_TUBE_LEN + INNER_TRANSITION_LEN + 0.11,
        0.0,
        0.0,
    )
    return tip.cut(pin_hole)


def add_box_visual(
    part,
    name: str,
    size: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center_xyz),
        material=material,
        name=name,
    )


def add_tube_visuals(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    material,
    rear_cap: float = 0.0,
    front_cap: float = 0.0,
) -> None:
    span_len = length - rear_cap - front_cap
    span_center_x = rear_cap + span_len / 2.0

    if span_len > 0.0:
        add_box_visual(
            part,
            f"{prefix}_top",
            (span_len, width, wall),
            (span_center_x, 0.0, height / 2.0 - wall / 2.0),
            material,
        )
        add_box_visual(
            part,
            f"{prefix}_bottom",
            (span_len, width, wall),
            (span_center_x, 0.0, -height / 2.0 + wall / 2.0),
            material,
        )
        add_box_visual(
            part,
            f"{prefix}_left",
            (span_len, wall, height - 2.0 * wall),
            (span_center_x, width / 2.0 - wall / 2.0, 0.0),
            material,
        )
        add_box_visual(
            part,
            f"{prefix}_right",
            (span_len, wall, height - 2.0 * wall),
            (span_center_x, -width / 2.0 + wall / 2.0, 0.0),
            material,
        )

    if rear_cap > 0.0:
        add_box_visual(
            part,
            f"{prefix}_rear_cap",
            (rear_cap, width, height),
            (rear_cap / 2.0, 0.0, 0.0),
            material,
        )

    if front_cap > 0.0:
        add_box_visual(
            part,
            f"{prefix}_front_cap",
            (front_cap, width, height),
            (length - front_cap / 2.0, 0.0, 0.0),
            material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_boom")

    bracket_color = model.material("bracket_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    boom_outer_color = model.material("boom_yellow_outer", rgba=(0.86, 0.68, 0.10, 1.0))
    boom_mid_color = model.material("boom_yellow_mid", rgba=(0.90, 0.73, 0.16, 1.0))
    boom_inner_color = model.material("boom_yellow_inner", rgba=(0.94, 0.78, 0.18, 1.0))

    root_bracket = model.part("root_bracket")
    cheek_t = 0.03
    cap_t = 0.05
    cheek_h = OUTER_H + 0.12
    cap_w = OUTER_W + 0.10
    sleeve_center_x = BRACKET_SLEEVE_LEN / 2.0
    add_box_visual(
        root_bracket,
        "bracket_plate",
        (BRACKET_T, BRACKET_W, BRACKET_H),
        (-BRACKET_T / 2.0, 0.0, 0.0),
        bracket_color,
    )
    add_box_visual(
        root_bracket,
        "left_cheek",
        (BRACKET_SLEEVE_LEN, cheek_t, cheek_h),
        (sleeve_center_x, OUTER_W / 2.0 + BRACKET_CLEAR + cheek_t / 2.0, 0.0),
        bracket_color,
    )
    add_box_visual(
        root_bracket,
        "right_cheek",
        (BRACKET_SLEEVE_LEN, cheek_t, cheek_h),
        (sleeve_center_x, -OUTER_W / 2.0 - BRACKET_CLEAR - cheek_t / 2.0, 0.0),
        bracket_color,
    )
    add_box_visual(
        root_bracket,
        "top_cap",
        (BRACKET_SLEEVE_LEN, cap_w, cap_t),
        (sleeve_center_x, 0.0, OUTER_H / 2.0 + BRACKET_CLEAR + cap_t / 2.0),
        bracket_color,
    )
    add_box_visual(
        root_bracket,
        "bottom_cap",
        (BRACKET_SLEEVE_LEN, cap_w, cap_t),
        (sleeve_center_x, 0.0, -OUTER_H / 2.0 - BRACKET_CLEAR - cap_t / 2.0),
        bracket_color,
    )
    root_bracket.inertial = Inertial.from_geometry(
        Box((0.24, BRACKET_W, BRACKET_H)),
        mass=78.0,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    outer_beam = model.part("outer_beam")
    add_tube_visuals(
        outer_beam,
        prefix="outer",
        length=OUTER_LEN,
        width=OUTER_W,
        height=OUTER_H,
        wall=OUTER_WALL,
        rear_cap=OUTER_REAR_CAP,
        front_cap=0.0,
        material=boom_outer_color,
    )
    outer_beam.inertial = Inertial.from_geometry(
        Box((OUTER_LEN, OUTER_W, OUTER_H)),
        mass=92.0,
        origin=Origin(xyz=(OUTER_LEN / 2.0, 0.0, 0.0)),
    )

    mid_beam = model.part("mid_beam")
    add_tube_visuals(
        mid_beam,
        prefix="mid",
        length=MID_LEN,
        width=MID_W,
        height=MID_H,
        wall=MID_WALL,
        material=boom_mid_color,
    )
    mid_pad_t = (OUTER_H - 2.0 * OUTER_WALL - MID_H) / 2.0
    for idx, pad_center_x in enumerate((0.18, MID_LEN - 0.18), start=1):
        add_box_visual(
            mid_beam,
            f"mid_pad_top_{idx}",
            (0.18, MID_W - 0.06, mid_pad_t),
            (pad_center_x, 0.0, MID_H / 2.0 + mid_pad_t / 2.0),
            bracket_color,
        )
        add_box_visual(
            mid_beam,
            f"mid_pad_bottom_{idx}",
            (0.18, MID_W - 0.06, mid_pad_t),
            (pad_center_x, 0.0, -MID_H / 2.0 - mid_pad_t / 2.0),
            bracket_color,
        )
    mid_beam.inertial = Inertial.from_geometry(
        Box((MID_LEN, MID_W, MID_H)),
        mass=58.0,
        origin=Origin(xyz=(MID_LEN / 2.0, 0.0, 0.0)),
    )

    inner_beam = model.part("inner_beam")
    add_tube_visuals(
        inner_beam,
        prefix="inner",
        length=INNER_TUBE_LEN,
        width=INNER_W,
        height=INNER_H,
        wall=INNER_WALL,
        front_cap=0.012,
        material=boom_inner_color,
    )
    inner_pad_t = (MID_H - 2.0 * MID_WALL - INNER_H) / 2.0
    for idx, pad_center_x in enumerate((0.16, INNER_TUBE_LEN - 0.16), start=1):
        add_box_visual(
            inner_beam,
            f"inner_pad_top_{idx}",
            (0.16, INNER_W - 0.05, inner_pad_t),
            (pad_center_x, 0.0, INNER_H / 2.0 + inner_pad_t / 2.0),
            bracket_color,
        )
        add_box_visual(
            inner_beam,
            f"inner_pad_bottom_{idx}",
            (0.16, INNER_W - 0.05, inner_pad_t),
            (pad_center_x, 0.0, -INNER_H / 2.0 - inner_pad_t / 2.0),
            bracket_color,
        )
    add_box_visual(
        inner_beam,
        "fork_transition",
        (INNER_TRANSITION_LEN, 0.17, 0.10),
        (INNER_TUBE_LEN + INNER_TRANSITION_LEN / 2.0, 0.0, 0.0),
        boom_inner_color,
    )
    lug_width = 0.04
    lug_gap = 0.09
    lug_height = 0.09
    lug_center_x = INNER_TUBE_LEN + INNER_TRANSITION_LEN + INNER_LUG_LEN / 2.0
    lug_center_y = lug_gap / 2.0 + lug_width / 2.0
    add_box_visual(
        inner_beam,
        "fork_lug_left",
        (INNER_LUG_LEN, lug_width, lug_height),
        (lug_center_x, lug_center_y, 0.0),
        boom_inner_color,
    )
    add_box_visual(
        inner_beam,
        "fork_lug_right",
        (INNER_LUG_LEN, lug_width, lug_height),
        (lug_center_x, -lug_center_y, 0.0),
        boom_inner_color,
    )
    inner_beam.inertial = Inertial.from_geometry(
        Box((INNER_LEN, INNER_W, INNER_H)),
        mass=37.0,
        origin=Origin(xyz=(INNER_LEN / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "bracket_to_outer",
        ArticulationType.FIXED,
        parent=root_bracket,
        child=outer_beam,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_mid",
        ArticulationType.PRISMATIC,
        parent=outer_beam,
        child=mid_beam,
        origin=Origin(xyz=(MID_RETRACTED_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.55,
            lower=0.0,
            upper=MID_EXTENSION,
        ),
    )
    model.articulation(
        "mid_to_inner",
        ArticulationType.PRISMATIC,
        parent=mid_beam,
        child=inner_beam,
        origin=Origin(xyz=(INNER_RETRACTED_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.65,
            lower=0.0,
            upper=INNER_EXTENSION,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    outer_beam = object_model.get_part("outer_beam")
    mid_beam = object_model.get_part("mid_beam")
    inner_beam = object_model.get_part("inner_beam")

    outer_to_mid = object_model.get_articulation("outer_to_mid")
    mid_to_inner = object_model.get_articulation("mid_to_inner")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        root_bracket,
        outer_beam,
        contact_tol=0.001,
        name="root bracket seats the outer beam",
    )
    ctx.expect_contact(
        outer_beam,
        mid_beam,
        contact_tol=0.0015,
        name="middle stage rides on the outer stage pads",
    )
    ctx.expect_contact(
        mid_beam,
        inner_beam,
        contact_tol=0.0015,
        name="inner stage rides on the middle stage pads",
    )

    ctx.expect_within(
        mid_beam,
        outer_beam,
        axes="yz",
        margin=0.0,
        name="middle stage stays inside the outer stage section",
    )
    ctx.expect_within(
        inner_beam,
        mid_beam,
        axes="yz",
        margin=0.0,
        name="inner stage stays inside the middle stage section",
    )

    ctx.check(
        "boom joints use the shared boom axis",
        tuple(outer_to_mid.axis) == (1.0, 0.0, 0.0) and tuple(mid_to_inner.axis) == (1.0, 0.0, 0.0),
        details=f"outer_to_mid={outer_to_mid.axis}, mid_to_inner={mid_to_inner.axis}",
    )
    ctx.check(
        "boom joints are prismatic",
        outer_to_mid.joint_type == ArticulationType.PRISMATIC
        and mid_to_inner.joint_type == ArticulationType.PRISMATIC,
        details=(
            f"outer_to_mid={outer_to_mid.joint_type}, "
            f"mid_to_inner={mid_to_inner.joint_type}"
        ),
    )

    outer_aabb = ctx.part_world_aabb(outer_beam)
    mid_aabb = ctx.part_world_aabb(mid_beam)
    inner_aabb = ctx.part_world_aabb(inner_beam)
    if outer_aabb and mid_aabb and inner_aabb:
        outer_y = outer_aabb[1][1] - outer_aabb[0][1]
        mid_y = mid_aabb[1][1] - mid_aabb[0][1]
        inner_y = inner_aabb[1][1] - inner_aabb[0][1]
        outer_z = outer_aabb[1][2] - outer_aabb[0][2]
        mid_z = mid_aabb[1][2] - mid_aabb[0][2]
        inner_z = inner_aabb[1][2] - inner_aabb[0][2]
        ctx.check(
            "stage widths step down toward the tip",
            outer_y > mid_y > inner_y,
            details=f"widths={outer_y:.3f}, {mid_y:.3f}, {inner_y:.3f}",
        )
        ctx.check(
            "stage heights step down toward the tip",
            outer_z > mid_z > inner_z,
            details=f"heights={outer_z:.3f}, {mid_z:.3f}, {inner_z:.3f}",
        )
    else:
        ctx.fail("stage section sizing could be measured", "one or more stage AABBs were unavailable")

    with ctx.pose({outer_to_mid: 0.52, mid_to_inner: 0.42}):
        ctx.expect_within(
            mid_beam,
            outer_beam,
            axes="yz",
            margin=0.0,
            name="middle stage stays aligned when extended",
        )
        ctx.expect_within(
            inner_beam,
            mid_beam,
            axes="yz",
            margin=0.0,
            name="inner stage stays aligned when extended",
        )
        ctx.expect_origin_gap(
            mid_beam,
            outer_beam,
            axis="x",
            min_gap=MID_RETRACTED_X + 0.50,
            max_gap=MID_RETRACTED_X + 0.54,
            name="middle stage slides outward along the boom axis",
        )
        ctx.expect_origin_gap(
            inner_beam,
            mid_beam,
            axis="x",
            min_gap=INNER_RETRACTED_X + 0.40,
            max_gap=INNER_RETRACTED_X + 0.44,
            name="inner stage slides outward along the boom axis",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
