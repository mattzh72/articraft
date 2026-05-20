from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_prism_mesh(
    name: str,
    *,
    size: tuple[float, float, float],
    radius: float,
):
    profile = rounded_rect_profile(size[0], size[1], radius)
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(profile, size[2], cap=True, closed=True),
        name,
    )


def _handwheel_rim_mesh(name: str, *, radius: float, tube: float):
    rim = TorusGeometry(radius, tube, radial_segments=16, tubular_segments=36)
    rim.rotate_x(math.pi / 2.0)
    rim.translate(0.0, 0.047, 0.0)
    return mesh_from_geometry(rim, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_base_drill_stand")

    base_black = model.material("base_black", rgba=(0.11, 0.12, 0.13, 1.0))
    machine_red = model.material("machine_red", rgba=(0.72, 0.07, 0.08, 1.0))
    column_grey = model.material("column_grey", rgba=(0.44, 0.47, 0.49, 1.0))
    head_grey = model.material("head_grey", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.29, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        _rounded_prism_mesh(
            "magnetic_base_plate",
            size=(0.235, 0.135, 0.018),
            radius=0.012,
        ),
        material=base_black,
        name="magnet_plate",
    )
    base.visual(
        _rounded_prism_mesh(
            "magnetic_base_housing",
            size=(0.215, 0.120, 0.030),
            radius=0.010,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=machine_red,
        name="upper_housing",
    )
    base.visual(
        Box((0.120, 0.030, 0.014)),
        origin=Origin(xyz=(0.050, 0.0, 0.037)),
        material=base_black,
        name="control_panel",
    )
    base.visual(
        Box((0.110, 0.092, 0.012)),
        origin=Origin(xyz=(-0.065, 0.0, 0.054)),
        material=machine_red,
        name="column_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.235, 0.135, 0.060)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    column_outer = model.part("column_outer")
    column_outer.visual(
        Box((0.074, 0.074, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=column_grey,
        name="base_shoe",
    )
    column_outer.visual(
        Box((0.010, 0.074, 0.340)),
        origin=Origin(xyz=(-0.038, 0.0, 0.170)),
        material=column_grey,
        name="rear_wall",
    )
    column_outer.visual(
        Box((0.054, 0.010, 0.340)),
        origin=Origin(xyz=(-0.008, 0.032, 0.170)),
        material=column_grey,
        name="left_wall",
    )
    column_outer.visual(
        Box((0.054, 0.010, 0.340)),
        origin=Origin(xyz=(-0.008, -0.032, 0.170)),
        material=column_grey,
        name="right_wall",
    )
    column_outer.visual(
        Box((0.010, 0.074, 0.014)),
        origin=Origin(xyz=(-0.038, 0.0, 0.333)),
        material=column_grey,
        name="top_rear_cap",
    )
    column_outer.visual(
        Box((0.062, 0.010, 0.014)),
        origin=Origin(xyz=(-0.008, 0.032, 0.333)),
        material=column_grey,
        name="top_left_cap",
    )
    column_outer.visual(
        Box((0.062, 0.010, 0.014)),
        origin=Origin(xyz=(-0.008, -0.032, 0.333)),
        material=column_grey,
        name="top_right_cap",
    )
    column_outer.inertial = Inertial.from_geometry(
        Box((0.086, 0.074, 0.340)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    model.articulation(
        "base_to_outer_column",
        ArticulationType.FIXED,
        parent=base,
        child=column_outer,
        origin=Origin(xyz=(-0.065, 0.0, 0.060)),
    )

    column_inner = model.part("column_inner")
    column_inner.visual(
        Box((0.062, 0.046, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=dark_steel,
        name="post",
    )
    column_inner.visual(
        Box((0.012, 0.052, 0.300)),
        origin=Origin(xyz=(0.037, 0.0, 0.220)),
        material=column_grey,
        name="guide_rail",
    )
    column_inner.visual(
        Box((0.004, 0.018, 0.265)),
        origin=Origin(xyz=(0.041, 0.016, 0.205)),
        material=dark_steel,
        name="rack_strip",
    )
    column_inner.visual(
        Box((0.060, 0.050, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, 0.411)),
        material=column_grey,
        name="top_cap",
    )
    column_inner.inertial = Inertial.from_geometry(
        Box((0.074, 0.056, 0.420)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    model.articulation(
        "outer_to_inner_column",
        ArticulationType.PRISMATIC,
        parent=column_outer,
        child=column_inner,
        origin=Origin(xyz=(-0.002, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.12,
            lower=0.0,
            upper=0.150,
        ),
    )

    spindle_head = model.part("spindle_head")
    spindle_head.visual(
        Box((0.010, 0.074, 0.132)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=head_grey,
        name="guide_plate",
    )
    spindle_head.visual(
        Box((0.094, 0.110, 0.120)),
        origin=Origin(xyz=(0.057, 0.0, 0.010)),
        material=head_grey,
        name="carriage_body",
    )
    spindle_head.visual(
        Box((0.068, 0.096, 0.030)),
        origin=Origin(xyz=(0.052, 0.0, 0.076)),
        material=machine_red,
        name="top_cover",
    )
    spindle_head.visual(
        Box((0.036, 0.030, 0.056)),
        origin=Origin(xyz=(0.026, 0.070, 0.018)),
        material=machine_red,
        name="gearbox_housing",
    )
    spindle_head.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.076, 0.0, -0.112)),
        material=head_grey,
        name="spindle_collar",
    )
    spindle_head.visual(
        Cylinder(radius=0.022, length=0.090),
        origin=Origin(xyz=(0.076, 0.0, -0.072)),
        material=dark_steel,
        name="spindle_housing",
    )
    spindle_head.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.076, 0.0, -0.142)),
        material=polished_steel,
        name="quill_nose",
    )
    spindle_head.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(0.076, 0.0, -0.182)),
        material=polished_steel,
        name="chuck",
    )
    spindle_head.inertial = Inertial.from_geometry(
        Box((0.104, 0.110, 0.250)),
        mass=3.6,
        origin=Origin(xyz=(0.052, 0.0, -0.010)),
    )

    model.articulation(
        "column_to_head",
        ArticulationType.PRISMATIC,
        parent=column_inner,
        child=spindle_head,
        origin=Origin(xyz=(0.043, 0.0, 0.295)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.18,
            lower=0.0,
            upper=0.130,
        ),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="shaft",
    )
    handwheel.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    handwheel.visual(
        _handwheel_rim_mesh("handwheel_rim", radius=0.043, tube=0.0035),
        material=knob_black,
        name="rim",
    )
    handwheel.visual(
        Box((0.080, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.047, 0.0)),
        material=dark_steel,
        name="spoke_horizontal",
    )
    handwheel.visual(
        Box((0.006, 0.004, 0.080)),
        origin=Origin(xyz=(0.0, 0.047, 0.0)),
        material=dark_steel,
        name="spoke_vertical",
    )
    handwheel.inertial = Inertial.from_geometry(
        Box((0.094, 0.056, 0.094)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.047, 0.0)),
    )

    model.articulation(
        "head_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=spindle_head,
        child=handwheel,
        origin=Origin(xyz=(0.024, 0.085, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column_outer = object_model.get_part("column_outer")
    column_inner = object_model.get_part("column_inner")
    spindle_head = object_model.get_part("spindle_head")
    handwheel = object_model.get_part("handwheel")

    lift_joint = object_model.get_articulation("outer_to_inner_column")
    feed_joint = object_model.get_articulation("column_to_head")
    wheel_joint = object_model.get_articulation("head_to_handwheel")

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
        "telescoping column uses a vertical prismatic joint",
        lift_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift_joint.articulation_type}, axis={lift_joint.axis}",
    )
    ctx.check(
        "head feed uses a downward vertical prismatic joint",
        feed_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(feed_joint.axis) == (0.0, 0.0, -1.0),
        details=f"type={feed_joint.articulation_type}, axis={feed_joint.axis}",
    )
    ctx.check(
        "handwheel rotates on a lateral axis",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )

    ctx.expect_contact(
        base,
        column_outer,
        elem_a="column_boss",
        elem_b="base_shoe",
        name="outer column is seated on the electromagnetic base",
    )
    ctx.expect_contact(
        handwheel,
        spindle_head,
        elem_a="shaft",
        elem_b="gearbox_housing",
        name="handwheel shaft mounts to the spindle head gearbox",
    )

    lift_max = 0.150
    feed_max = 0.130

    with ctx.pose({lift_joint: 0.0}):
        ctx.expect_contact(
            column_inner,
            column_outer,
            elem_a="post",
            elem_b="rear_wall",
            name="inner column post bears against the outer mast rear guide",
        )
        ctx.expect_overlap(
            column_inner,
            column_outer,
            axes="xy",
            min_overlap=0.045,
            name="inner column nests in the outer mast at rest",
        )

    with ctx.pose({lift_joint: lift_max}):
        ctx.expect_overlap(
            column_inner,
            column_outer,
            axes="xy",
            min_overlap=0.045,
            name="inner column remains guided at maximum lift",
        )

    with ctx.pose({feed_joint: 0.0}):
        ctx.expect_contact(
            spindle_head,
            column_inner,
            elem_a="guide_plate",
            elem_b="rack_strip",
            name="spindle head engages the rack face at the upper position",
        )

    with ctx.pose({feed_joint: feed_max}):
        ctx.expect_overlap(
            spindle_head,
            column_inner,
            axes="yz",
            elem_a="guide_plate",
            elem_b="guide_rail",
            min_overlap=0.040,
            name="spindle head guide stays captured on the column rail",
        )

    with ctx.pose({lift_joint: 0.0, feed_joint: feed_max}):
        ctx.expect_gap(
            spindle_head,
            base,
            axis="z",
            positive_elem="chuck",
            min_gap=0.015,
            name="lowest chuck position still clears the base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
