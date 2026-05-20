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


LINK_THICKNESS = 0.0036
RIB_HEIGHT = 0.00045
FUSE_EPS = 0.00015
PLANE_STEP = 0.0064
ROOT_HOLE_RADIUS = 0.00315
SHOULDER_RADIUS = 0.00245
WASHER_RADIUS = 0.00490
HEAD_RADIUS = 0.00560
HEAD_THICKNESS = 0.00100
WASHER_THICKNESS = 0.00070
BOSS_THICKNESS = 0.00080

INNER_LINK_LENGTH = 0.220
MID_LINK_LENGTH = 0.165
OUTER_LINK_LENGTH = 0.128


def _cylinder_on_xz(
    center_x: float,
    center_z: float,
    radius: float,
    length_y: float,
    start_y: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .pushPoints([(center_x, center_z)])
        .circle(radius)
        .extrude(length_y)
        .translate((0.0, start_y, 0.0))
    )


def _poly_prism_xz(
    points: list[tuple[float, float]],
    thickness: float,
    center_y: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, center_y - thickness / 2.0, 0.0))
    )


def _rect_prism(
    *,
    center_x: float,
    center_y: float,
    center_z: float,
    size_x: float,
    size_y: float,
    size_z: float,
) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate((center_x, center_y, center_z))


def _root_hole_cutter(extra: float = 0.0012) -> cq.Workplane:
    return _cylinder_on_xz(
        center_x=0.0,
        center_z=0.0,
        radius=ROOT_HOLE_RADIUS,
        length_y=LINK_THICKNESS + 2.0 * extra,
        start_y=-(LINK_THICKNESS / 2.0 + extra),
    )


def _link_profile_points(length: float, root_width: float, waist_width: float, tip_width: float) -> list[tuple[float, float]]:
    return [
        (0.0, root_width / 2.0),
        (0.018, root_width / 2.0),
        (0.054, waist_width / 2.0),
        (length - 0.026, waist_width / 2.0),
        (length - 0.008, tip_width / 2.0),
        (length + 0.006, 0.0),
        (length - 0.008, -tip_width / 2.0),
        (length - 0.026, -waist_width / 2.0),
        (0.054, -waist_width / 2.0),
        (0.018, -root_width / 2.0),
        (0.0, -root_width / 2.0),
    ]


def make_link_body(*, length: float, root_width: float, waist_width: float, tip_width: float) -> cq.Workplane:
    body = _poly_prism_xz(_link_profile_points(length, root_width, waist_width, tip_width), LINK_THICKNESS)
    body = body.cut(_root_hole_cutter())
    top_left_press = _rect_prism(
        center_x=length * 0.57,
        center_y=LINK_THICKNESS / 2.0 - RIB_HEIGHT / 2.0,
        center_z=waist_width * 0.22,
        size_x=length * 0.40,
        size_y=RIB_HEIGHT + FUSE_EPS,
        size_z=max(0.0018, waist_width * 0.18),
    )
    top_right_press = _rect_prism(
        center_x=length * 0.57,
        center_y=LINK_THICKNESS / 2.0 - RIB_HEIGHT / 2.0,
        center_z=-waist_width * 0.22,
        size_x=length * 0.40,
        size_y=RIB_HEIGHT + FUSE_EPS,
        size_z=max(0.0018, waist_width * 0.18),
    )
    lower_press = _rect_prism(
        center_x=length * 0.39,
        center_y=-(LINK_THICKNESS / 2.0 - RIB_HEIGHT / 2.0),
        center_z=0.0,
        size_x=length * 0.28,
        size_y=RIB_HEIGHT + FUSE_EPS,
        size_z=max(0.0022, waist_width * 0.30),
    )
    return body.cut(top_left_press).cut(top_right_press).cut(lower_press)


def make_outgoing_stud(*, x_at_tip: float, child_center_y: float) -> cq.Workplane:
    child_near = child_center_y - LINK_THICKNESS / 2.0
    child_far = child_center_y + LINK_THICKNESS / 2.0
    shank = _cylinder_on_xz(
        x_at_tip,
        0.0,
        SHOULDER_RADIUS,
        child_far + WASHER_THICKNESS + HEAD_THICKNESS - LINK_THICKNESS / 2.0,
        LINK_THICKNESS / 2.0,
    )
    inner_washer = _cylinder_on_xz(
        x_at_tip,
        0.0,
        WASHER_RADIUS,
        WASHER_THICKNESS + FUSE_EPS,
        child_near - (WASHER_THICKNESS + FUSE_EPS),
    )
    outer_washer = _cylinder_on_xz(
        x_at_tip,
        0.0,
        WASHER_RADIUS,
        WASHER_THICKNESS + FUSE_EPS,
        child_far,
    )
    head = _cylinder_on_xz(
        x_at_tip,
        0.0,
        HEAD_RADIUS,
        HEAD_THICKNESS + FUSE_EPS,
        child_far + WASHER_THICKNESS,
    )
    return shank.union(inner_washer).union(outer_washer).union(head)


def make_outer_link_with_shoe() -> cq.Workplane:
    body = make_link_body(
        length=OUTER_LINK_LENGTH,
        root_width=0.019,
        waist_width=0.0115,
        tip_width=0.016,
    )
    shoe_plate = _poly_prism_xz(
        [
            (OUTER_LINK_LENGTH - 0.004, 0.0105),
            (OUTER_LINK_LENGTH + 0.040, 0.0030),
            (OUTER_LINK_LENGTH + 0.029, -0.0210),
            (OUTER_LINK_LENGTH - 0.003, -0.0130),
        ],
        LINK_THICKNESS,
    )
    shoe_pad = _poly_prism_xz(
        [
            (OUTER_LINK_LENGTH + 0.001, 0.0068),
            (OUTER_LINK_LENGTH + 0.024, 0.0014),
            (OUTER_LINK_LENGTH + 0.018, -0.0128),
            (OUTER_LINK_LENGTH + 0.001, -0.0092),
        ],
        0.0028,
        center_y=-(LINK_THICKNESS / 2.0 + 0.0014 - FUSE_EPS),
    )
    shoe_rib = _rect_prism(
        center_x=OUTER_LINK_LENGTH + 0.018,
        center_y=LINK_THICKNESS / 2.0 + 0.00045 - FUSE_EPS,
        center_z=-0.0030,
        size_x=0.019,
        size_y=0.0009,
        size_z=0.0034,
    )
    return body.union(shoe_plate).union(shoe_pad).union(shoe_rib)


def make_root_bracket_body() -> cq.Workplane:
    foot = _rect_prism(center_x=-0.045, center_y=0.0, center_z=-0.037, size_x=0.080, size_y=0.042, size_z=0.0075)
    upright = _rect_prism(center_x=-0.017, center_y=0.0, center_z=-0.0215, size_x=0.016, size_y=0.010, size_z=0.042)
    gusset = (
        cq.Workplane("XZ")
        .polyline([(-0.008, -0.003), (-0.060, -0.041), (-0.008, -0.041)])
        .close()
        .extrude(0.010)
        .translate((0.0, -0.005, 0.0))
    )
    ear_center_y = 0.0042
    left_ear = _rect_prism(center_x=-0.003, center_y=-ear_center_y, center_z=0.000, size_x=0.007, size_y=0.0024, size_z=0.015)
    right_ear = _rect_prism(center_x=-0.003, center_y=ear_center_y, center_z=0.000, size_x=0.007, size_y=0.0024, size_z=0.015)
    foot_rib = _rect_prism(center_x=-0.049, center_y=0.0, center_z=-0.032, size_x=0.040, size_y=0.006, size_z=0.004)
    pivot_pin = _cylinder_on_xz(0.0, 0.0, SHOULDER_RADIUS, 0.0088, -0.0044)
    left_retainer = _cylinder_on_xz(0.0, 0.0, WASHER_RADIUS, 0.00085, -0.00265)
    right_retainer = _cylinder_on_xz(0.0, 0.0, WASHER_RADIUS, 0.00085, 0.0018)
    left_head = _cylinder_on_xz(0.0, 0.0, HEAD_RADIUS, 0.00105, -0.00370)
    right_head = _cylinder_on_xz(0.0, 0.0, HEAD_RADIUS, 0.00105, 0.00265)
    foot_holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.022, 0.012), (-0.054, -0.012)])
        .circle(0.0029)
        .extrude(0.012)
        .translate((0.0, 0.0, -0.043))
    )
    return (
        foot.union(upright)
        .union(gusset)
        .union(left_ear)
        .union(right_ear)
        .union(foot_rib)
        .union(pivot_pin)
        .union(left_retainer)
        .union(right_retainer)
        .union(left_head)
        .union(right_head)
        .cut(foot_holes)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_support_stay")

    steel = model.material("zinc_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.30, 0.31, 0.33, 1.0))
    shoe_dark = model.material("shoe_dark", rgba=(0.24, 0.25, 0.27, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(make_root_bracket_body(), "root_bracket_body"),
        origin=Origin(),
        material=dark_oxide,
        name="bracket_body",
    )

    inner_link = model.part("inner_link")
    inner_link.visual(
        mesh_from_cadquery(
            make_link_body(
                length=INNER_LINK_LENGTH,
                root_width=0.023,
                waist_width=0.0145,
                tip_width=0.019,
            ),
            "inner_link_body",
        ),
        origin=Origin(),
        material=steel,
        name="inner_body",
    )
    inner_link.visual(
        mesh_from_cadquery(
            make_outgoing_stud(x_at_tip=INNER_LINK_LENGTH, child_center_y=PLANE_STEP),
            "inner_link_stud",
        ),
        origin=Origin(),
        material=dark_oxide,
        name="inner_stud",
    )

    mid_link = model.part("mid_link")
    mid_link.visual(
        mesh_from_cadquery(
            make_link_body(
                length=MID_LINK_LENGTH,
                root_width=0.020,
                waist_width=0.0130,
                tip_width=0.017,
            ),
            "mid_link_body",
        ),
        origin=Origin(),
        material=steel,
        name="mid_body",
    )
    mid_link.visual(
        mesh_from_cadquery(
            make_outgoing_stud(x_at_tip=MID_LINK_LENGTH, child_center_y=PLANE_STEP),
            "mid_link_stud",
        ),
        origin=Origin(),
        material=dark_oxide,
        name="mid_stud",
    )

    outer_link = model.part("outer_link")
    outer_link.visual(
        mesh_from_cadquery(make_outer_link_with_shoe(), "outer_link_body"),
        origin=Origin(),
        material=shoe_dark,
        name="outer_body",
    )

    model.articulation(
        "root_to_inner",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=inner_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.05, upper=2.65),
    )
    model.articulation(
        "inner_to_mid",
        ArticulationType.REVOLUTE,
        parent=inner_link,
        child=mid_link,
        origin=Origin(xyz=(INNER_LINK_LENGTH, PLANE_STEP, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5, lower=-2.40, upper=0.10),
    )
    model.articulation(
        "mid_to_outer",
        ArticulationType.REVOLUTE,
        parent=mid_link,
        child=outer_link,
        origin=Origin(xyz=(MID_LINK_LENGTH, PLANE_STEP, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-2.30, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    inner_link = object_model.get_part("inner_link")
    mid_link = object_model.get_part("mid_link")
    outer_link = object_model.get_part("outer_link")

    root_to_inner = object_model.get_articulation("root_to_inner")
    inner_to_mid = object_model.get_articulation("inner_to_mid")
    mid_to_outer = object_model.get_articulation("mid_to_outer")

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
        mid_link,
        outer_link,
        elem_a="mid_stud",
        elem_b="outer_body",
        reason="the modeled shoulder-bolt shank passes through the pierced outer-link pivot and triangulated meshes can report a tiny residual overlap at that pin-hole interface",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for joint_name, joint_obj, min_span in (
        ("root_to_inner", root_to_inner, 2.5),
        ("inner_to_mid", inner_to_mid, 2.0),
        ("mid_to_outer", mid_to_outer, 1.8),
    ):
        axis_ok = tuple(round(v, 6) for v in joint_obj.axis) == (0.0, 1.0, 0.0)
        ctx.check(
            f"{joint_name}_axis",
            axis_ok,
            details=f"{joint_name} axis should be (0, 1, 0), got {joint_obj.axis}",
        )
        limits = joint_obj.motion_limits
        range_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
            and (limits.upper - limits.lower) >= min_span
        )
        ctx.check(
            f"{joint_name}_range",
            range_ok,
            details=f"{joint_name} should include folded and deployed travel over {min_span} rad",
        )

    ctx.expect_contact(
        inner_link,
        root_bracket,
        elem_a="inner_body",
        elem_b="bracket_body",
        name="inner_link_supported_on_root_pivot",
    )
    ctx.expect_contact(
        mid_link,
        inner_link,
        elem_a="mid_body",
        elem_b="inner_stud",
        name="mid_link_supported_by_inner_stud",
    )
    ctx.expect_contact(
        outer_link,
        mid_link,
        elem_a="outer_body",
        elem_b="mid_stud",
        name="outer_link_supported_on_mid_stud",
    )
    ctx.expect_origin_gap(
        mid_link,
        inner_link,
        axis="y",
        min_gap=PLANE_STEP - 0.0002,
        max_gap=PLANE_STEP + 0.0002,
        name="mid_link_runs_in_second_plane",
    )
    ctx.expect_origin_gap(
        outer_link,
        mid_link,
        axis="y",
        min_gap=PLANE_STEP - 0.0002,
        max_gap=PLANE_STEP + 0.0002,
        name="outer_link_runs_in_third_plane",
    )

    with ctx.pose(root_to_inner=1.00, inner_to_mid=-0.92, mid_to_outer=-0.72):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_deployed_pose")
        ctx.expect_origin_gap(
            outer_link,
            root_bracket,
            axis="x",
            min_gap=0.24,
            name="outer_shoe_projects_outward",
        )
        ctx.expect_contact(
            mid_link,
            inner_link,
            elem_a="mid_body",
            elem_b="inner_stud",
            name="mid_link_stays_captured_when_deployed",
        )
        ctx.expect_contact(
            outer_link,
            mid_link,
            elem_a="outer_body",
            elem_b="mid_stud",
            name="outer_link_stays_captured_when_deployed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
