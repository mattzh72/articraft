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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_searchlight_tower")

    powder_black = model.material("powder_black", rgba=(0.11, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.84, 0.92, 0.45))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.32, 0.16, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=charcoal,
        name="base_plate",
    )
    base.visual(
        Box((0.22, 0.11, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=graphite,
        name="ballast_cover",
    )
    base.visual(
        Box((0.060, 0.160, 0.050)),
        origin=Origin(xyz=(-0.075, 0.0, 0.049)),
        material=graphite,
        name="rear_hinge_pod",
    )
    base.visual(
        Box((0.020, 0.018, 0.042)),
        origin=Origin(xyz=(-0.075, 0.028, 0.095)),
        material=satin_metal,
        name="left_hinge_ear",
    )
    base.visual(
        Box((0.020, 0.018, 0.042)),
        origin=Origin(xyz=(-0.075, -0.028, 0.095)),
        material=satin_metal,
        name="right_hinge_ear",
    )
    base.visual(
        Box((0.030, 0.110, 0.018)),
        origin=Origin(xyz=(0.105, 0.0, 0.033)),
        material=rubber,
        name="stow_bumper",
    )
    for x in (-0.120, 0.120):
        for y in (-0.055, 0.055):
            base.visual(
                Box((0.026, 0.026, 0.006)),
                origin=Origin(xyz=(x, y, 0.003)),
                material=rubber,
                name=f"foot_{'p' if x > 0 else 'n'}x_{'p' if y > 0 else 'n'}y",
            )
    base.inertial = Inertial.from_geometry(
        Box((0.32, 0.16, 0.12)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.028, 0.038, 0.032)),
        material=satin_metal,
        name="fold_hinge_block",
    )
    mast.visual(
        Box((0.030, 0.032, 0.192)),
        origin=Origin(xyz=(0.018, 0.0, 0.112)),
        material=powder_black,
        name="main_column",
    )
    mast.visual(
        Box((0.028, 0.042, 0.056)),
        origin=Origin(xyz=(0.020, 0.0, 0.184)),
        material=graphite,
        name="upper_column",
    )
    mast.visual(
        Box((0.058, 0.080, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, 0.214)),
        material=graphite,
        name="headstock_cap",
    )
    mast.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.020, 0.0, 0.217)),
        material=satin_metal,
        name="pan_bearing_plate",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.070, 0.090, 0.230)),
        mass=1.0,
        origin=Origin(xyz=(0.008, 0.0, 0.115)),
    )

    pan_carriage = model.part("pan_carriage")
    pan_carriage.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_metal,
        name="turntable_disk",
    )
    pan_carriage.visual(
        Box((0.032, 0.070, 0.040)),
        origin=Origin(xyz=(-0.010, 0.0, 0.026)),
        material=graphite,
        name="pan_housing",
    )
    pan_carriage.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(-0.010, 0.0, 0.028)),
        material=powder_black,
        name="pan_spindle_cover",
    )
    pan_carriage.visual(
        Box((0.028, 0.008, 0.090)),
        origin=Origin(xyz=(0.006, 0.062, 0.065)),
        material=powder_black,
        name="left_yoke_arm",
    )
    pan_carriage.visual(
        Box((0.028, 0.008, 0.090)),
        origin=Origin(xyz=(0.006, -0.062, 0.065)),
        material=powder_black,
        name="right_yoke_arm",
    )
    pan_carriage.visual(
        Box((0.018, 0.016, 0.030)),
        origin=Origin(xyz=(0.020, 0.050, 0.068)),
        material=satin_metal,
        name="left_bearing_block",
    )
    pan_carriage.visual(
        Box((0.018, 0.016, 0.030)),
        origin=Origin(xyz=(0.020, -0.050, 0.068)),
        material=satin_metal,
        name="right_bearing_block",
    )
    pan_carriage.visual(
        Box((0.018, 0.116, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, 0.032)),
        material=powder_black,
        name="lower_yoke_bridge",
    )
    pan_carriage.visual(
        Box((0.014, 0.116, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, 0.090)),
        material=powder_black,
        name="rear_yoke_bridge",
    )
    pan_carriage.inertial = Inertial.from_geometry(
        Box((0.090, 0.130, 0.110)),
        mass=0.7,
        origin=Origin(xyz=(0.025, 0.0, 0.045)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.006, length=0.084),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="pivot_journal",
    )
    head.visual(
        Box((0.022, 0.028, 0.020)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=satin_metal,
        name="neck_block",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.102),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="lamp_body",
    )
    head.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.130, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.037, length=0.003),
        origin=Origin(xyz=(0.137, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_cap",
    )
    head.visual(
        Box((0.038, 0.008, 0.012)),
        origin=Origin(xyz=(0.076, 0.0, 0.036)),
        material=graphite,
        name="cooling_rib",
    )
    head.visual(
        Box((0.024, 0.030, 0.018)),
        origin=Origin(xyz=(0.028, 0.0, -0.022)),
        material=charcoal,
        name="driver_box",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.135, 0.100, 0.110)),
        mass=0.6,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "mast_fold",
        ArticulationType.REVOLUTE,
        parent=base,
        child=mast,
        origin=Origin(xyz=(-0.075, 0.0, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "pan",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=pan_carriage,
        origin=Origin(xyz=(0.020, 0.0, 0.222)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.5,
        ),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=pan_carriage,
        child=head,
        origin=Origin(xyz=(0.020, 0.0, 0.068)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=math.radians(-25.0),
            upper=math.radians(65.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    pan_carriage = object_model.get_part("pan_carriage")
    head = object_model.get_part("head")
    mast_fold = object_model.get_articulation("mast_fold")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "pan axis is vertical",
        tuple(round(value, 6) for value in pan.axis) == (0.0, 0.0, 1.0),
        details=f"pan axis was {pan.axis}",
    )
    ctx.check(
        "tilt axis is lateral",
        tuple(round(value, 6) for value in tilt.axis) == (0.0, -1.0, 0.0),
        details=f"tilt axis was {tilt.axis}",
    )
    ctx.check(
        "pan and tilt axes are separated",
        abs(tilt.origin.xyz[0]) > 0.015 and abs(tilt.origin.xyz[2]) > 0.05,
        details=f"tilt origin {tilt.origin.xyz} is too close to pan axis",
    )

    with ctx.pose({mast_fold: 0.0, pan: 0.0, tilt: 0.0}):
        ctx.expect_contact(
            base,
            mast,
            elem_a="left_hinge_ear",
            elem_b="fold_hinge_block",
            contact_tol=1e-6,
            name="left hinge ear carries mast hinge block",
        )
        ctx.expect_contact(
            base,
            mast,
            elem_a="right_hinge_ear",
            elem_b="fold_hinge_block",
            contact_tol=1e-6,
            name="right hinge ear carries mast hinge block",
        )
        ctx.expect_contact(
            mast,
            pan_carriage,
            elem_a="pan_bearing_plate",
            elem_b="turntable_disk",
            contact_tol=1e-6,
            name="mast bearing plate supports pan carriage",
        )
        ctx.expect_contact(
            pan_carriage,
            head,
            elem_a="left_bearing_block",
            elem_b="pivot_journal",
            contact_tol=1e-6,
            name="left bearing block supports pivot journal",
        )
        ctx.expect_contact(
            pan_carriage,
            head,
            elem_a="right_bearing_block",
            elem_b="pivot_journal",
            contact_tol=1e-6,
            name="right bearing block supports pivot journal",
        )

    with ctx.pose({mast_fold: math.radians(35.0), pan: 0.0, tilt: math.radians(60.0)}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.006,
            name="stowed head clears base",
        )
        ctx.expect_gap(
            pan_carriage,
            base,
            axis="z",
            min_gap=0.012,
            name="stowed yoke clears base",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="xy",
            min_overlap=0.040,
            name="stowed head stays over base footprint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
