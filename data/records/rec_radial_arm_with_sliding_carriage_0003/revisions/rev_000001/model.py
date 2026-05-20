from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_slider", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    anodized = model.material("anodized", rgba=(0.70, 0.71, 0.74, 1.0))
    safety_red = model.material("safety_red", rgba=(0.72, 0.18, 0.16, 1.0))
    polymer = model.material("polymer", rgba=(0.15, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.090, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="foot",
    )
    base.visual(
        Box((0.110, 0.110, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=steel,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=anodized,
        name="column",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.221)),
        material=steel,
        name="turntable",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.226),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.035, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=steel,
        name="hub",
    )
    arm.visual(
        Box((0.270, 0.020, 0.014)),
        origin=Origin(xyz=(0.170, 0.0, 0.011)),
        material=anodized,
        name="beam",
    )
    arm.visual(
        Box((0.240, 0.042, 0.005)),
        origin=Origin(xyz=(0.180, 0.0, 0.0205)),
        material=steel,
        name="rail",
    )
    arm.visual(
        Box((0.020, 0.028, 0.016)),
        origin=Origin(xyz=(0.305, 0.0, 0.011)),
        material=safety_red,
        name="end_stop",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.325, 0.050, 0.030)),
        mass=2.0,
        origin=Origin(xyz=(0.1625, 0.0, 0.015)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.060, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=polymer,
        name="cap",
    )
    carriage.visual(
        Box((0.055, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, 0.027, 0.017)),
        material=polymer,
        name="side_left",
    )
    carriage.visual(
        Box((0.055, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -0.027, 0.017)),
        material=polymer,
        name="side_right",
    )
    carriage.visual(
        Box((0.018, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=safety_red,
        name="clamp_body",
    )
    carriage.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=steel,
        name="knob",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.060)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.226)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-math.radians(120.0),
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=0.180,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    sweep = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

    turntable = base.get_visual("turntable")
    hub = arm.get_visual("hub")
    rail = arm.get_visual("rail")
    end_stop = arm.get_visual("end_stop")
    cap = carriage.get_visual("cap")
    side_left = carriage.get_visual("side_left")
    side_right = carriage.get_visual("side_right")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(arm, base, elem_a=hub, elem_b=turntable, contact_tol=1e-6)
    ctx.expect_overlap(arm, base, axes="xy", elem_a=hub, elem_b=turntable, min_overlap=0.070)
    ctx.expect_origin_distance(arm, base, axes="xy", max_dist=0.001)

    ctx.expect_gap(
        carriage,
        arm,
        axis="z",
        positive_elem=cap,
        negative_elem=rail,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(carriage, arm, axes="xy", elem_a=cap, elem_b=rail, min_overlap=0.040)
    ctx.expect_origin_gap(carriage, arm, axis="x", min_gap=0.069, max_gap=0.071)
    ctx.expect_origin_distance(carriage, arm, axes="y", max_dist=0.001)

    ctx.expect_gap(
        carriage,
        arm,
        axis="y",
        positive_elem=side_left,
        negative_elem=end_stop,
        min_gap=0.0,
    )
    ctx.expect_gap(
        arm,
        carriage,
        axis="y",
        positive_elem=end_stop,
        negative_elem=side_right,
        min_gap=0.0,
    )

    with ctx.pose({sweep: math.radians(120.0)}):
        ctx.expect_contact(arm, base, elem_a=hub, elem_b=turntable, contact_tol=1e-6)
        ctx.expect_origin_distance(arm, base, axes="xy", max_dist=0.001)
        ctx.expect_gap(
            carriage,
            arm,
            axis="z",
            positive_elem=cap,
            negative_elem=rail,
            max_gap=0.001,
            max_penetration=0.0,
        )

    with ctx.pose({sweep: -math.radians(120.0), slide: 0.180}):
        ctx.expect_contact(arm, base, elem_a=hub, elem_b=turntable, contact_tol=1e-6)
        ctx.expect_gap(
            carriage,
            arm,
            axis="z",
            positive_elem=cap,
            negative_elem=rail,
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_overlap(carriage, arm, axes="xy", elem_a=cap, elem_b=rail, min_overlap=0.040)
        ctx.expect_overlap(carriage, arm, axes="y", elem_a=cap, elem_b=rail, min_overlap=0.040)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
