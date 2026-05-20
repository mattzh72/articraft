from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 2.45
OUTER_WIDTH = 0.22
OUTER_HEIGHT = 0.24
OUTER_WALL = 0.012

MIDDLE_LENGTH = 2.15
MIDDLE_WIDTH = 0.17
MIDDLE_HEIGHT = 0.216
MIDDLE_WALL = 0.010

INNER_A_LENGTH = 1.75
INNER_A_WIDTH = 0.13
INNER_A_HEIGHT = 0.196
INNER_A_WALL = 0.009

INNER_B_LENGTH = 1.32
INNER_B_WIDTH = 0.096
INNER_B_HEIGHT = 0.178
INNER_B_WALL = 0.008

OUTER_TO_MIDDLE_RETRACT = 0.52
MIDDLE_TO_INNER_A_RETRACT = 0.47
INNER_A_TO_INNER_B_RETRACT = 0.42

OUTER_TO_MIDDLE_EXTEND = 1.15
MIDDLE_TO_INNER_A_EXTEND = 0.95
INNER_A_TO_INNER_B_EXTEND = 0.72

BOOM_CENTER_Z = 0.98
def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_rectangular_tube(
    part,
    *,
    prefix: str,
    material,
    length: float,
    width: float,
    height: float,
    wall: float,
):
    inner_width = width - 2.0 * wall
    inner_height = height - 2.0 * wall
    half_width = width / 2.0
    half_height = height / 2.0

    _add_box_visual(
        part,
        name=f"{prefix}_top_wall",
        size=(length, width, wall),
        xyz=(length / 2.0, 0.0, half_height - wall / 2.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_bottom_wall",
        size=(length, width, wall),
        xyz=(length / 2.0, 0.0, -half_height + wall / 2.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_left_wall",
        size=(length, wall, inner_height),
        xyz=(length / 2.0, half_width - wall / 2.0, 0.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_right_wall",
        size=(length, wall, inner_height),
        xyz=(length / 2.0, -half_width + wall / 2.0, 0.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_rear_cap",
        size=(wall, inner_width, inner_height),
        xyz=(wall / 2.0, 0.0, 0.0),
        material=material,
    )


def _add_root_support(part, *, material):
    _add_box_visual(
        part,
        name="base_plate",
        size=(1.20, 0.82, 0.12),
        xyz=(0.0, 0.0, 0.06),
        material=material,
    )
    _add_box_visual(
        part,
        name="lower_pedestal",
        size=(0.48, 0.34, 0.36),
        xyz=(-0.02, 0.0, 0.30),
        material=material,
    )
    _add_box_visual(
        part,
        name="upper_pedestal",
        size=(0.30, 0.22, 0.24),
        xyz=(0.02, 0.0, 0.60),
        material=material,
    )
    _add_box_visual(
        part,
        name="boom_saddle_web",
        size=(0.22, 0.18, 0.12),
        xyz=(0.08, 0.0, 0.76),
        material=material,
    )
    _add_box_visual(
        part,
        name="boom_saddle",
        size=(0.56, 0.18, 0.04),
        xyz=(0.18, 0.0, 0.84),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_rectangular_boom")

    support_gray = model.material("support_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    outer_yellow = model.material("outer_yellow", rgba=(0.83, 0.67, 0.17, 1.0))
    middle_yellow = model.material("middle_yellow", rgba=(0.90, 0.74, 0.20, 1.0))
    inner_yellow = model.material("inner_yellow", rgba=(0.95, 0.80, 0.24, 1.0))

    root_support = model.part("root_support")
    _add_root_support(
        root_support,
        material=support_gray,
    )

    outer_section = model.part("outer_section")
    _add_rectangular_tube(
        outer_section,
        prefix="outer",
        material=outer_yellow,
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
    )

    middle_section = model.part("middle_section")
    _add_rectangular_tube(
        middle_section,
        prefix="middle",
        material=middle_yellow,
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
    )

    inner_a_section = model.part("inner_a_section")
    _add_rectangular_tube(
        inner_a_section,
        prefix="inner_a",
        material=inner_yellow,
        length=INNER_A_LENGTH,
        width=INNER_A_WIDTH,
        height=INNER_A_HEIGHT,
        wall=INNER_A_WALL,
    )

    inner_b_section = model.part("inner_b_section")
    _add_rectangular_tube(
        inner_b_section,
        prefix="inner_b",
        material=inner_yellow,
        length=INNER_B_LENGTH,
        width=INNER_B_WIDTH,
        height=INNER_B_HEIGHT,
        wall=INNER_B_WALL,
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=root_support,
        child=outer_section,
        origin=Origin(xyz=(0.08, 0.0, BOOM_CENTER_Z)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_section,
        child=middle_section,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_RETRACT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25000.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_EXTEND,
        ),
    )
    model.articulation(
        "middle_to_inner_a",
        ArticulationType.PRISMATIC,
        parent=middle_section,
        child=inner_a_section,
        origin=Origin(xyz=(MIDDLE_TO_INNER_A_RETRACT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.40,
            lower=0.0,
            upper=MIDDLE_TO_INNER_A_EXTEND,
        ),
    )
    model.articulation(
        "inner_a_to_inner_b",
        ArticulationType.PRISMATIC,
        parent=inner_a_section,
        child=inner_b_section,
        origin=Origin(xyz=(INNER_A_TO_INNER_B_RETRACT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.45,
            lower=0.0,
            upper=INNER_A_TO_INNER_B_EXTEND,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_support = object_model.get_part("root_support")
    outer_section = object_model.get_part("outer_section")
    middle_section = object_model.get_part("middle_section")
    inner_a_section = object_model.get_part("inner_a_section")
    inner_b_section = object_model.get_part("inner_b_section")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner_a = object_model.get_articulation("middle_to_inner_a")
    inner_a_to_inner_b = object_model.get_articulation("inner_a_to_inner_b")

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

    ctx.check(
        "all sliding joints are +x prismatic",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (1.0, 0.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 0.5
            for joint in (outer_to_middle, middle_to_inner_a, inner_a_to_inner_b)
        ),
        details="Expected three aligned +X prismatic stages with positive extension travel.",
    )

    ctx.expect_contact(
        root_support,
        outer_section,
        contact_tol=0.001,
        name="root support carries the outer boom section",
    )
    ctx.expect_contact(
        outer_section,
        middle_section,
        contact_tol=0.001,
        name="middle section rides on the outer section wear pads",
    )
    ctx.expect_contact(
        middle_section,
        inner_a_section,
        contact_tol=0.001,
        name="inner A section rides on the middle section wear pads",
    )
    ctx.expect_contact(
        inner_a_section,
        inner_b_section,
        contact_tol=0.001,
        name="inner B section rides on the inner A section wear pads",
    )

    ctx.expect_origin_distance(
        middle_section,
        outer_section,
        axes="yz",
        max_dist=0.001,
        name="middle and outer share a common boom axis",
    )
    ctx.expect_origin_distance(
        inner_a_section,
        middle_section,
        axes="yz",
        max_dist=0.001,
        name="inner A and middle share a common boom axis",
    )
    ctx.expect_origin_distance(
        inner_b_section,
        inner_a_section,
        axes="yz",
        max_dist=0.001,
        name="inner B and inner A share a common boom axis",
    )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_EXTEND,
            middle_to_inner_a: MIDDLE_TO_INNER_A_EXTEND,
            inner_a_to_inner_b: INNER_A_TO_INNER_B_EXTEND,
        }
    ):
        ctx.expect_contact(
            outer_section,
            middle_section,
            contact_tol=0.001,
            name="middle remains supported by outer at full extension",
        )
        ctx.expect_contact(
            middle_section,
            inner_a_section,
            contact_tol=0.001,
            name="inner A remains supported by middle at full extension",
        )
        ctx.expect_contact(
            inner_a_section,
            inner_b_section,
            contact_tol=0.001,
            name="inner B remains supported by inner A at full extension",
        )

        ctx.expect_origin_gap(
            middle_section,
            outer_section,
            axis="x",
            min_gap=OUTER_TO_MIDDLE_RETRACT + OUTER_TO_MIDDLE_EXTEND - 0.01,
            max_gap=OUTER_TO_MIDDLE_RETRACT + OUTER_TO_MIDDLE_EXTEND + 0.01,
            name="middle extends forward along the boom axis",
        )
        ctx.expect_origin_gap(
            inner_a_section,
            middle_section,
            axis="x",
            min_gap=MIDDLE_TO_INNER_A_RETRACT + MIDDLE_TO_INNER_A_EXTEND - 0.01,
            max_gap=MIDDLE_TO_INNER_A_RETRACT + MIDDLE_TO_INNER_A_EXTEND + 0.01,
            name="inner A extends forward along the boom axis",
        )
        ctx.expect_origin_gap(
            inner_b_section,
            inner_a_section,
            axis="x",
            min_gap=INNER_A_TO_INNER_B_RETRACT + INNER_A_TO_INNER_B_EXTEND - 0.01,
            max_gap=INNER_A_TO_INNER_B_RETRACT + INNER_A_TO_INNER_B_EXTEND + 0.01,
            name="inner B extends forward along the boom axis",
        )

        ctx.expect_overlap(
            outer_section,
            middle_section,
            axes="x",
            min_overlap=0.75,
            name="middle retains meaningful overlap inside the outer section",
        )
        ctx.expect_overlap(
            middle_section,
            inner_a_section,
            axes="x",
            min_overlap=0.72,
            name="inner A retains meaningful overlap inside the middle section",
        )
        ctx.expect_overlap(
            inner_a_section,
            inner_b_section,
            axes="x",
            min_overlap=0.60,
            name="inner B retains meaningful overlap inside the inner A section",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
