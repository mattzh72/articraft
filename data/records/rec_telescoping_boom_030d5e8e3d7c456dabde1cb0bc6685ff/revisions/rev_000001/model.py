from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import ArticulatedObject, ArticulationType, Box, Inertial, MotionLimits, Origin, TestContext, TestReport


OUTER_LENGTH = 1.52
OUTER_WIDTH = 0.22
OUTER_HEIGHT = 0.266
OUTER_WALL = 0.014
OUTER_SLOT_WIDTH = 0.128

MIDDLE_LENGTH = 1.28
MIDDLE_WIDTH = 0.154
MIDDLE_HEIGHT = 0.202
MIDDLE_WALL = 0.012
MIDDLE_SLOT_WIDTH = 0.082

INNER_BODY_LENGTH = 0.98
INNER_TOTAL_LENGTH = 1.16
INNER_WIDTH = 0.112
INNER_HEIGHT = 0.154
INNER_WALL = 0.010
INNER_SLOT_WIDTH = 0.058

OUTER_TO_MIDDLE_HOME = 0.24
OUTER_TO_MIDDLE_MAX = 0.60
MIDDLE_TO_INNER_HOME = 0.20
MIDDLE_TO_INNER_MAX = 0.36


def _add_box(part, *, name: str, size: tuple[float, float, float], center: tuple[float, float, float], material: str):
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _make_root_bracket(model: ArticulatedObject):
    part = model.part("root_bracket")
    steel = "root_charcoal"

    _add_box(part, name="base_plate", size=(0.50, 0.40, 0.024), center=(0.04, 0.0, -0.162), material=steel)
    _add_box(part, name="rear_plate", size=(0.024, 0.40, 0.30), center=(-0.030, 0.0, -0.01), material=steel)
    _add_box(part, name="left_cheek", size=(0.38, 0.014, 0.38), center=(0.08, 0.131, 0.01), material=steel)
    _add_box(part, name="right_cheek", size=(0.38, 0.014, 0.38), center=(0.08, -0.131, 0.01), material=steel)
    _add_box(part, name="top_tie", size=(0.22, 0.276, 0.016), center=(0.08, 0.0, 0.151), material=steel)
    _add_box(part, name="saddle_left", size=(0.18, 0.05, 0.008), center=(0.04, 0.055, -0.137), material=steel)
    _add_box(part, name="saddle_right", size=(0.18, 0.05, 0.008), center=(0.04, -0.055, -0.137), material=steel)
    _add_box(part, name="rear_stop", size=(0.008, 0.14, 0.18), center=(-0.022, 0.0, -0.01), material=steel)
    _add_box(part, name="left_rib", size=(0.16, 0.012, 0.12), center=(0.03, 0.132, -0.085), material=steel)
    _add_box(part, name="right_rib", size=(0.16, 0.012, 0.12), center=(0.03, -0.132, -0.085), material=steel)

    part.inertial = Inertial.from_geometry(Box((0.50, 0.40, 0.40)), mass=62.0, origin=Origin(xyz=(0.04, 0.0, 0.0)))
    return part


def _make_outer_beam(model: ArticulatedObject):
    part = model.part("outer_beam")
    paint = "outer_paint"
    roof_strip = (OUTER_WIDTH - OUTER_SLOT_WIDTH) / 2.0
    y_roof = OUTER_SLOT_WIDTH / 2.0 + roof_strip / 2.0
    y_web = OUTER_WIDTH / 2.0 - OUTER_WALL / 2.0
    z_bottom = -OUTER_HEIGHT / 2.0 + OUTER_WALL / 2.0
    z_top = OUTER_HEIGHT / 2.0 - OUTER_WALL / 2.0

    _add_box(part, name="bottom_plate", size=(OUTER_LENGTH, OUTER_WIDTH, OUTER_WALL), center=(OUTER_LENGTH / 2.0, 0.0, z_bottom), material=paint)
    _add_box(part, name="left_web", size=(OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT - 2.0 * OUTER_WALL), center=(OUTER_LENGTH / 2.0, y_web, 0.0), material=paint)
    _add_box(part, name="right_web", size=(OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT - 2.0 * OUTER_WALL), center=(OUTER_LENGTH / 2.0, -y_web, 0.0), material=paint)
    _add_box(part, name="top_left_strip", size=(OUTER_LENGTH, roof_strip, OUTER_WALL), center=(OUTER_LENGTH / 2.0, y_roof, z_top), material=paint)
    _add_box(part, name="top_right_strip", size=(OUTER_LENGTH, roof_strip, OUTER_WALL), center=(OUTER_LENGTH / 2.0, -y_roof, z_top), material=paint)
    _add_box(part, name="rear_cap", size=(0.018, OUTER_WIDTH, OUTER_HEIGHT), center=(-0.009, 0.0, 0.0), material=paint)
    _add_box(part, name="left_root_doubler", size=(0.32, 0.010, 0.18), center=(0.16, 0.115, 0.0), material=paint)
    _add_box(part, name="right_root_doubler", size=(0.32, 0.010, 0.18), center=(0.16, -0.115, 0.0), material=paint)
    _add_box(part, name="front_cap_left", size=(0.05, roof_strip, 0.012), center=(OUTER_LENGTH - 0.025, y_roof, z_top + 0.012), material=paint)
    _add_box(part, name="front_cap_right", size=(0.05, roof_strip, 0.012), center=(OUTER_LENGTH - 0.025, -y_roof, z_top + 0.012), material=paint)

    part.inertial = Inertial.from_geometry(Box((1.54, 0.24, 0.27)), mass=118.0, origin=Origin(xyz=(0.75, 0.0, 0.0)))
    return part


def _make_middle_beam(model: ArticulatedObject):
    part = model.part("middle_beam")
    paint = "middle_paint"
    roof_strip = (MIDDLE_WIDTH - MIDDLE_SLOT_WIDTH) / 2.0
    y_roof = MIDDLE_SLOT_WIDTH / 2.0 + roof_strip / 2.0
    y_web = MIDDLE_WIDTH / 2.0 - MIDDLE_WALL / 2.0
    z_bottom = -MIDDLE_HEIGHT / 2.0 + MIDDLE_WALL / 2.0
    z_top = MIDDLE_HEIGHT / 2.0 - MIDDLE_WALL / 2.0

    _add_box(part, name="bottom_plate", size=(MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_WALL), center=(MIDDLE_LENGTH / 2.0, 0.0, z_bottom), material=paint)
    _add_box(part, name="left_web", size=(MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT - 2.0 * MIDDLE_WALL), center=(MIDDLE_LENGTH / 2.0, y_web, 0.0), material=paint)
    _add_box(part, name="right_web", size=(MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT - 2.0 * MIDDLE_WALL), center=(MIDDLE_LENGTH / 2.0, -y_web, 0.0), material=paint)
    _add_box(part, name="top_left_strip", size=(MIDDLE_LENGTH, roof_strip, MIDDLE_WALL), center=(MIDDLE_LENGTH / 2.0, y_roof, z_top), material=paint)
    _add_box(part, name="top_right_strip", size=(MIDDLE_LENGTH, roof_strip, MIDDLE_WALL), center=(MIDDLE_LENGTH / 2.0, -y_roof, z_top), material=paint)
    _add_box(part, name="rear_cap", size=(0.014, MIDDLE_WIDTH, MIDDLE_HEIGHT), center=(-0.007, 0.0, 0.0), material=paint)
    _add_box(part, name="pad_rear_left", size=(0.16, 0.03, 0.018), center=(0.24, 0.044, -0.110), material="pad_black")
    _add_box(part, name="pad_rear_right", size=(0.16, 0.03, 0.018), center=(0.24, -0.044, -0.110), material="pad_black")
    _add_box(part, name="pad_front_left", size=(0.16, 0.03, 0.018), center=(1.08, 0.044, -0.110), material="pad_black")
    _add_box(part, name="pad_front_right", size=(0.16, 0.03, 0.018), center=(1.08, -0.044, -0.110), material="pad_black")
    _add_box(part, name="front_wear_cap_left", size=(0.045, roof_strip, 0.012), center=(MIDDLE_LENGTH - 0.0225, y_roof, z_top + 0.012), material=paint)
    _add_box(part, name="front_wear_cap_right", size=(0.045, roof_strip, 0.012), center=(MIDDLE_LENGTH - 0.0225, -y_roof, z_top + 0.012), material=paint)

    part.inertial = Inertial.from_geometry(Box((1.30, 0.16, 0.22)), mass=78.0, origin=Origin(xyz=(0.64, 0.0, 0.0)))
    return part


def _make_inner_beam(model: ArticulatedObject):
    part = model.part("inner_beam")
    steel = "inner_steel"
    roof_strip = (INNER_WIDTH - INNER_SLOT_WIDTH) / 2.0
    y_roof = INNER_SLOT_WIDTH / 2.0 + roof_strip / 2.0
    y_web = INNER_WIDTH / 2.0 - INNER_WALL / 2.0
    z_bottom = -INNER_HEIGHT / 2.0 + INNER_WALL / 2.0
    z_top = INNER_HEIGHT / 2.0 - INNER_WALL / 2.0

    _add_box(part, name="bottom_plate", size=(INNER_BODY_LENGTH, INNER_WIDTH, INNER_WALL), center=(INNER_BODY_LENGTH / 2.0, 0.0, z_bottom), material=steel)
    _add_box(part, name="left_web", size=(INNER_BODY_LENGTH, INNER_WALL, INNER_HEIGHT - 2.0 * INNER_WALL), center=(INNER_BODY_LENGTH / 2.0, y_web, 0.0), material=steel)
    _add_box(part, name="right_web", size=(INNER_BODY_LENGTH, INNER_WALL, INNER_HEIGHT - 2.0 * INNER_WALL), center=(INNER_BODY_LENGTH / 2.0, -y_web, 0.0), material=steel)
    _add_box(part, name="top_left_strip", size=(INNER_BODY_LENGTH, roof_strip, INNER_WALL), center=(INNER_BODY_LENGTH / 2.0, y_roof, z_top), material=steel)
    _add_box(part, name="top_right_strip", size=(INNER_BODY_LENGTH, roof_strip, INNER_WALL), center=(INNER_BODY_LENGTH / 2.0, -y_roof, z_top), material=steel)
    _add_box(part, name="rear_cap", size=(0.012, INNER_WIDTH, INNER_HEIGHT), center=(-0.006, 0.0, 0.0), material=steel)
    _add_box(part, name="pad_rear_left", size=(0.14, 0.024, 0.018), center=(0.22, 0.032, -0.080), material="pad_black")
    _add_box(part, name="pad_rear_right", size=(0.14, 0.024, 0.018), center=(0.22, -0.032, -0.080), material="pad_black")
    _add_box(part, name="pad_front_left", size=(0.14, 0.024, 0.018), center=(0.74, 0.032, -0.080), material="pad_black")
    _add_box(part, name="pad_front_right", size=(0.14, 0.024, 0.018), center=(0.74, -0.032, -0.080), material="pad_black")
    _add_box(part, name="fork_neck", size=(0.05, INNER_WIDTH, 0.11), center=(0.965, 0.0, 0.0), material=steel)
    _add_box(part, name="fork_left", size=(0.18, 0.026, 0.132), center=(1.07, 0.037, 0.0), material=steel)
    _add_box(part, name="fork_right", size=(0.18, 0.026, 0.132), center=(1.07, -0.037, 0.0), material=steel)
    _add_box(part, name="end_cap_left", size=(0.032, 0.026, 0.030), center=(1.144, 0.037, 0.0), material="pad_black")
    _add_box(part, name="end_cap_right", size=(0.032, 0.026, 0.030), center=(1.144, -0.037, 0.0), material="pad_black")

    part.inertial = Inertial.from_geometry(Box((1.17, 0.12, 0.17)), mass=49.0, origin=Origin(xyz=(0.58, 0.0, 0.0)))
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_telescoping_boom")
    model.material("root_charcoal", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("outer_paint", rgba=(0.84, 0.66, 0.18, 1.0))
    model.material("middle_paint", rgba=(0.76, 0.63, 0.20, 1.0))
    model.material("inner_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("pad_black", rgba=(0.12, 0.12, 0.12, 1.0))

    root_bracket = _make_root_bracket(model)
    outer_beam = _make_outer_beam(model)
    middle_beam = _make_middle_beam(model)
    inner_beam = _make_inner_beam(model)

    model.articulation("root_to_outer", ArticulationType.FIXED, parent=root_bracket, child=outer_beam, origin=Origin())
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_beam,
        child=middle_beam,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.45, lower=0.0, upper=OUTER_TO_MIDDLE_MAX),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_beam,
        child=inner_beam,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.55, lower=0.0, upper=MIDDLE_TO_INNER_MAX),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    outer_beam = object_model.get_part("outer_beam")
    middle_beam = object_model.get_part("middle_beam")
    inner_beam = object_model.get_part("inner_beam")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
        "prismatic boom axes are shared x-axis",
        tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0) and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        details=f"outer axis={outer_to_middle.axis}, middle axis={middle_to_inner.axis}",
    )
    ctx.expect_contact(root_bracket, outer_beam, name="root bracket physically cradles outer beam")
    ctx.expect_within(middle_beam, outer_beam, axes="yz", margin=0.0, name="middle beam fits inside outer beam section")
    ctx.expect_within(inner_beam, middle_beam, axes="yz", margin=0.0, name="inner beam fits inside middle beam section")

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_overlap(
            outer_beam,
            middle_beam,
            axes="x",
            min_overlap=1.20,
            name="middle beam retains deep overlap when retracted",
        )
        ctx.expect_overlap(
            middle_beam,
            inner_beam,
            axes="x",
            min_overlap=1.00,
            name="inner beam retains deep overlap when retracted",
        )

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_MAX, middle_to_inner: MIDDLE_TO_INNER_MAX}):
        ctx.fail_if_parts_overlap_in_current_pose(name="fully extended boom stays clear")
        ctx.expect_overlap(
            outer_beam,
            middle_beam,
            axes="x",
            min_overlap=0.55,
            name="outer to middle overlap stays believable at full reach",
        )
        ctx.expect_overlap(
            middle_beam,
            inner_beam,
            axes="x",
            min_overlap=0.72,
            name="middle to inner overlap stays believable at full reach",
        )
        ctx.expect_within(
            middle_beam,
            outer_beam,
            axes="yz",
            margin=0.0,
            name="middle beam stays guided in outer section at full reach",
        )
        ctx.expect_within(
            inner_beam,
            middle_beam,
            axes="yz",
            margin=0.0,
            name="inner beam stays guided in middle section at full reach",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
