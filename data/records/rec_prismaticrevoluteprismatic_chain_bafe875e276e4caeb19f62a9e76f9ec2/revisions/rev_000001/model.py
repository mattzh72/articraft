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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _guide_stage_shape() -> cq.Workplane:
    base_plate = _box((0.28, 0.18, 0.03), (0.0, 0.0, 0.015))
    mast = _box((0.05, 0.10, 0.44), (-0.025, 0.0, 0.25))
    rail = _box((0.026, 0.074, 0.32), (0.013, 0.0, 0.215))
    top_cap = _box((0.075, 0.125, 0.018), (-0.018, 0.0, 0.479))

    gusset_profile = [(-0.105, 0.03), (0.0, 0.03), (-0.01, 0.18), (-0.05, 0.18)]
    right_gusset = (
        cq.Workplane("XZ")
        .polyline(gusset_profile)
        .close()
        .extrude(0.012, both=True)
        .translate((0.0, 0.044, 0.0))
    )
    left_gusset = right_gusset.translate((0.0, -0.088, 0.0))

    return base_plate.union(mast).union(rail).union(top_cap).union(right_gusset).union(left_gusset)


def _carriage_shape() -> cq.Workplane:
    body = _box((0.082, 0.106, 0.092), (0.0, 0.0, 0.0))
    rail_channel = _box((0.028, 0.074, 0.096), (-0.027, 0.0, 0.0))
    body = body.cut(rail_channel)

    clevis = _box((0.024, 0.106, 0.06), (0.053, 0.0, 0.01))
    clevis_slot = _box((0.03, 0.02, 0.064), (0.053, 0.0, 0.01))
    return body.union(clevis).cut(clevis_slot)


def _elbow_plate_shape() -> cq.Workplane:
    pivot_hub = cq.Workplane("YZ").circle(0.024).extrude(0.034)

    upper_spine = _box((0.12, 0.008, 0.008), (0.077, 0.0, 0.015))
    lower_spine = _box((0.13, 0.008, 0.008), (0.082, 0.0, -0.015))
    root_web = _box((0.046, 0.006, 0.03), (0.023, 0.0, 0.0))

    diagonal_brace = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.026, -0.010),
                (0.09, 0.008),
                (0.145, 0.004),
                (0.145, -0.006),
                (0.05, -0.014),
                (0.026, -0.014),
            ]
        )
        .close()
        .extrude(0.003, both=True)
    )

    guide_floor = _box((0.09, 0.018, 0.004), (0.19, 0.0, -0.012))
    left_fence = _box((0.09, 0.003, 0.014), (0.19, -0.0095, -0.005))
    right_fence = _box((0.09, 0.003, 0.014), (0.19, 0.0095, -0.005))
    rear_stop = _box((0.012, 0.018, 0.012), (0.145, 0.0, -0.006))

    return (
        pivot_hub.union(upper_spine)
        .union(lower_spine)
        .union(root_web)
        .union(diagonal_brace)
        .union(guide_floor)
        .union(left_fence)
        .union(right_fence)
        .union(rear_stop)
    )


def _probe_shape() -> cq.Workplane:
    shoe = _box((0.08, 0.014, 0.006), (-0.04, 0.0, -0.007))
    riser = _box((0.018, 0.01, 0.012), (0.001, 0.0, -0.001))
    shaft = _box((0.105, 0.008, 0.008), (0.0625, 0.0, 0.002))
    tip_head = _box((0.026, 0.016, 0.016), (0.132, 0.0, 0.002))
    nose = cq.Workplane("YZ").circle(0.0075).extrude(0.012).translate((0.145, 0.0, 0.002))
    return shoe.union(riser).union(shaft).union(tip_head).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lift_slide_arm")

    charcoal = model.material("charcoal", color=(0.19, 0.21, 0.24, 1.0))
    graphite = model.material("graphite", color=(0.31, 0.33, 0.36, 1.0))
    alloy = model.material("alloy", color=(0.78, 0.8, 0.83, 1.0))
    matte_black = model.material("matte_black", color=(0.08, 0.08, 0.09, 1.0))

    guide_stage = model.part("guide_stage")
    guide_stage.visual(
        mesh_from_cadquery(_guide_stage_shape(), "guide_stage"),
        material=charcoal,
        name="guide_stage_body",
    )
    guide_stage.inertial = Inertial.from_geometry(
        Box((0.28, 0.18, 0.50)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        material=graphite,
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.09, 0.11, 0.10)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    elbow_plate = model.part("elbow_plate")
    elbow_plate.visual(
        Box((0.028, 0.048, 0.048)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=alloy,
        name="pivot_block",
    )
    elbow_plate.visual(
        Box((0.13, 0.012, 0.008)),
        origin=Origin(xyz=(0.093, 0.0, 0.015)),
        material=alloy,
        name="upper_spine",
    )
    elbow_plate.visual(
        Box((0.14, 0.012, 0.008)),
        origin=Origin(xyz=(0.098, 0.0, -0.015)),
        material=alloy,
        name="lower_spine",
    )
    elbow_plate.visual(
        Box((0.10, 0.018, 0.004)),
        origin=Origin(xyz=(0.21, 0.0, -0.012)),
        material=alloy,
        name="guide_floor",
    )
    elbow_plate.visual(
        Box((0.10, 0.003, 0.014)),
        origin=Origin(xyz=(0.21, -0.009, -0.003)),
        material=alloy,
        name="left_fence",
    )
    elbow_plate.visual(
        Box((0.10, 0.003, 0.014)),
        origin=Origin(xyz=(0.21, 0.009, -0.003)),
        material=alloy,
        name="right_fence",
    )
    elbow_plate.inertial = Inertial.from_geometry(
        Box((0.24, 0.04, 0.06)),
        mass=0.75,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
    )

    probe = model.part("probe")
    probe.visual(
        Box((0.08, 0.012, 0.006)),
        origin=Origin(xyz=(-0.04, 0.0, -0.007)),
        material=matte_black,
        name="guide_shoe",
    )
    probe.visual(
        Box((0.02, 0.01, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, -0.001)),
        material=matte_black,
        name="shoe_riser",
    )
    probe.visual(
        Box((0.11, 0.008, 0.008)),
        origin=Origin(xyz=(0.07, 0.0, 0.002)),
        material=matte_black,
        name="probe_shaft",
    )
    probe.visual(
        Box((0.024, 0.016, 0.016)),
        origin=Origin(xyz=(0.137, 0.0, 0.002)),
        material=matte_black,
        name="probe_tip",
    )
    probe.inertial = Inertial.from_geometry(
        Box((0.22, 0.02, 0.02)),
        mass=0.22,
        origin=Origin(xyz=(0.04, 0.0, -0.002)),
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_stage,
        child=carriage,
        origin=Origin(xyz=(0.067, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.25, lower=0.0, upper=0.2),
    )

    model.articulation(
        "carriage_to_elbow",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=elbow_plate,
        origin=Origin(xyz=(0.065, 0.0, 0.01)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=0.0, upper=1.15),
    )

    model.articulation(
        "elbow_to_probe",
        ArticulationType.PRISMATIC,
        parent=elbow_plate,
        child=probe,
        origin=Origin(xyz=(0.26, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.16, lower=0.0, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_stage = object_model.get_part("guide_stage")
    carriage = object_model.get_part("carriage")
    elbow_plate = object_model.get_part("elbow_plate")
    probe = object_model.get_part("probe")
    guide_slide = object_model.get_articulation("guide_to_carriage")
    elbow_joint = object_model.get_articulation("carriage_to_elbow")
    probe_slide = object_model.get_articulation("elbow_to_probe")

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
        carriage,
        guide_stage,
        contact_tol=0.001,
        name="carriage remains captured on the grounded guide",
    )
    ctx.expect_contact(
        elbow_plate,
        carriage,
        contact_tol=0.001,
        name="elbow plate is supported by the carriage clevis",
    )
    ctx.expect_contact(
        probe,
        elbow_plate,
        contact_tol=0.001,
        name="probe stays captured in the distal sleeve",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({guide_slide: guide_slide.motion_limits.upper}):
        carriage_raised = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            guide_stage,
            contact_tol=0.001,
            name="guide contact survives the lifted stage pose",
        )
    ctx.check(
        "positive root slide raises the carriage",
        carriage_rest is not None
        and carriage_raised is not None
        and carriage_raised[2] > carriage_rest[2] + 0.15,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )

    probe_rest = ctx.part_world_position(probe)
    with ctx.pose({elbow_joint: elbow_joint.motion_limits.upper}):
        probe_lifted = ctx.part_world_position(probe)
    ctx.check(
        "positive elbow rotation lifts the probe tip upward",
        probe_rest is not None
        and probe_lifted is not None
        and probe_lifted[2] > probe_rest[2] + 0.12,
        details=f"rest={probe_rest}, lifted={probe_lifted}",
    )

    with ctx.pose({probe_slide: probe_slide.motion_limits.upper}):
        probe_extended = ctx.part_world_position(probe)
        ctx.expect_contact(
            probe,
            elbow_plate,
            contact_tol=0.001,
            name="probe remains supported while extended",
        )
    ctx.check(
        "positive distal slide extends the probe forward",
        probe_rest is not None
        and probe_extended is not None
        and probe_extended[0] > probe_rest[0] + 0.04,
        details=f"rest={probe_rest}, extended={probe_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
