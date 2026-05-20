from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _visual_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _visual_cylinder_z(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material,
) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=center), material=material, name=name)


def _add_box_section(
    part,
    prefix: str,
    x_min: float,
    x_max: float,
    width: float,
    height: float,
    wall: float,
    material,
) -> None:
    """Add four connected primitive walls forming an open box-section tube."""
    length = x_max - x_min
    center_x = 0.5 * (x_min + x_max)
    inner_width = width - 2.0 * wall
    inner_height = height - 2.0 * wall
    _visual_box(
        part,
        name=f"{prefix}_top",
        size=(length, width, wall),
        center=(center_x, 0.0, 0.5 * inner_height + 0.5 * wall),
        material=material,
    )
    _visual_box(
        part,
        name=f"{prefix}_bottom",
        size=(length, width, wall),
        center=(center_x, 0.0, -0.5 * inner_height - 0.5 * wall),
        material=material,
    )
    _visual_box(
        part,
        name=f"{prefix}_side_pos",
        size=(length, wall, height),
        center=(center_x, 0.5 * inner_width + 0.5 * wall, 0.0),
        material=material,
    )
    _visual_box(
        part,
        name=f"{prefix}_side_neg",
        size=(length, wall, height),
        center=(center_x, -0.5 * inner_width - 0.5 * wall, 0.0),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_section_three_stage_slide")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    outer_section = model.part("outer_section")
    _add_box_section(outer_section, "outer", -0.62, 0.02, 0.110, 0.070, 0.006, dark_steel)
    # A thicker front mouth ring makes the box-section sleeve read as a grounded
    # manufactured slide rail while keeping the same clear opening.
    _add_box_section(outer_section, "outer_mouth", -0.008, 0.024, 0.126, 0.086, 0.014, dark_steel)
    for plate_index, x_center in enumerate((-0.50, -0.14)):
        _visual_box(
            outer_section,
            name=f"mount_plate_{plate_index}",
            size=(0.130, 0.170, 0.012),
            center=(x_center, 0.0, -0.041),
            material=dark_steel,
        )
        for y_suffix, y_center in (("neg", -0.066), ("pos", 0.066)):
            _visual_cylinder_z(
                outer_section,
                name=f"mount_bolt_{plate_index}_{y_suffix}",
                radius=0.010,
                length=0.005,
                center=(x_center, y_center, -0.0325),
                material=dark_steel,
            )

    middle_runner = model.part("middle_runner")
    _add_box_section(middle_runner, "middle", -0.46, 0.12, 0.080, 0.042, 0.005, satin_steel)
    # Side glide shoes fill the working clearance and visibly support the
    # middle runner inside the grounded outer sleeve without volumetric overlap.
    _visual_box(
        middle_runner,
        name="middle_glide_neg",
        size=(0.300, 0.009, 0.014),
        center=(-0.23, -0.0445, 0.0),
        material=satin_steel,
    )
    _visual_box(
        middle_runner,
        name="middle_glide_pos",
        size=(0.300, 0.009, 0.014),
        center=(-0.23, 0.0445, 0.0),
        material=satin_steel,
    )
    _add_box_section(middle_runner, "middle_mouth", 0.104, 0.136, 0.092, 0.054, 0.011, satin_steel)

    inner_runner = model.part("inner_runner")
    _add_box_section(inner_runner, "inner", -0.32, 0.10, 0.058, 0.022, 0.0035, bright_steel)
    # Smaller internal shoes touch the middle runner's bore, making the second
    # prismatic support path explicit through the retained-insertion range.
    _visual_box(
        inner_runner,
        name="inner_glide_neg",
        size=(0.240, 0.006, 0.010),
        center=(-0.19, -0.032, 0.0),
        material=bright_steel,
    )
    _visual_box(
        inner_runner,
        name="inner_glide_pos",
        size=(0.240, 0.006, 0.010),
        center=(-0.19, 0.032, 0.0),
        material=bright_steel,
    )
    _add_box_section(inner_runner, "inner_mouth", 0.084, 0.108, 0.066, 0.030, 0.0075, bright_steel)

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_section,
        child=middle_runner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.32, effort=120.0, velocity=0.35),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_runner,
        child=inner_runner,
        # The inner stage is seated at the front mouth of the middle runner.
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.24, effort=80.0, velocity=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "serial prismatic stages",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC,
        details="The slide must be built from two serial prismatic joints.",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_within(
            middle,
            outer,
            axes="y",
            margin=0.0,
            name="middle runner fits inside outer width",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="z",
            margin=0.0,
            name="middle runner fits inside outer height",
        )
        ctx.expect_contact(
            middle,
            outer,
            contact_tol=0.0005,
            name="middle glide shoe bears on outer section",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.40,
            name="middle runner has collapsed insertion",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="y",
            margin=0.0,
            name="inner runner fits inside middle width",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="z",
            margin=0.0,
            name="inner runner fits inside middle height",
        )
        ctx.expect_contact(
            inner,
            middle,
            contact_tol=0.0005,
            name="inner glide shoe bears on middle section",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.25,
            name="inner runner has collapsed insertion",
        )

    middle_rest = ctx.part_world_position(middle)
    with ctx.pose({outer_to_middle: 0.32, middle_to_inner: 0.0}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.13,
            name="middle runner remains retained at full travel",
        )
        middle_extended = ctx.part_world_position(middle)

    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({outer_to_middle: 0.32, middle_to_inner: 0.24}):
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.07,
            name="inner runner remains retained at full travel",
        )
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "middle stage extends along slide axis",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.30,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "inner stage extends after middle stage",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.55,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    return ctx.report()


object_model = build_object_model()
