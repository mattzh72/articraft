import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plunger_chain")

    mat_housing = model.material("mat_housing", rgba=(0.3, 0.3, 0.3, 1.0))
    mat_plunger = model.material("mat_plunger", rgba=(0.8, 0.8, 0.8, 1.0))
    mat_lever = model.material("mat_lever", rgba=(0.8, 0.1, 0.1, 1.0))

    # Housing
    # Base block with bore
    housing_cq = (
        cq.Workplane("YZ")
        .rect(0.04, 0.04)
        .extrude(0.08)
        .faces("<X")
        .workplane()
        .hole(0.01)  # diameter 0.01 -> radius 0.005
    )
    # Brackets for the lever pivot
    left_bracket = (
        cq.Workplane("YZ")
        .workplane(offset=0.08)
        .center(0.015, 0.015)
        .rect(0.01, 0.03)
        .extrude(0.04)
    )
    right_bracket = (
        cq.Workplane("YZ")
        .workplane(offset=0.08)
        .center(-0.015, 0.015)
        .rect(0.01, 0.03)
        .extrude(0.04)
    )
    housing_cq = housing_cq.union(left_bracket).union(right_bracket)

    # Transverse pin hole through brackets
    pin_hole = (
        cq.Workplane("XZ")
        .workplane(offset=-0.025)
        .center(0.10, 0.02)
        .circle(0.002)
        .extrude(0.05)
    )
    housing_cq = housing_cq.cut(pin_hole)

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(housing_cq, "housing_mesh"),
        material=mat_housing,
        name="housing_vis",
    )

    # Plunger
    # Main cylinder sliding in the bore
    plunger_cq = (
        cq.Workplane("YZ")
        .workplane(offset=-0.08)
        .circle(0.0045)
        .extrude(0.12)
    )
    # Head block that holds the lever
    plunger_head = (
        cq.Workplane("YZ")
        .rect(0.012, 0.012)
        .extrude(0.04)
    )
    plunger_cq = plunger_cq.union(plunger_head)
    # Slot to capture the lever tip
    slot_cut = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .center(0.02, 0)
        .rect(0.006, 0.015)
        .extrude(0.02)
    )
    plunger_cq = plunger_cq.cut(slot_cut)

    plunger = model.part("plunger")
    plunger.visual(
        mesh_from_cadquery(plunger_cq, "plunger_mesh"),
        material=mat_plunger,
        name="plunger_vis",
    )

    model.articulation(
        "housing_to_plunger",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=plunger,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.015),
    )

    # Lever
    # Vertical body
    lever_cq = (
        cq.Workplane("XY")
        .workplane(offset=-0.02)
        .center(0, 0)
        .rect(0.004, 0.008)
        .extrude(0.03)
    )
    # Transverse pivot pin
    pivot_pin = (
        cq.Workplane("XZ")
        .workplane(offset=-0.02)
        .center(0, 0)
        .circle(0.0018)
        .extrude(0.04)
    )
    lever_cq = lever_cq.union(pivot_pin)

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(lever_cq, "lever_mesh"),
        material=mat_lever,
        name="lever_vis",
    )

    model.articulation(
        "housing_to_lever",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lever,
        origin=Origin(xyz=(0.10, 0.0, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0, lower=-1.0, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    plunger = object_model.get_part("plunger")
    lever = object_model.get_part("lever")
    plunger_joint = object_model.get_articulation("housing_to_plunger")
    lever_joint = object_model.get_articulation("housing_to_lever")

    ctx.allow_overlap(plunger, housing, reason="Plunger slides inside housing bore.")
    ctx.allow_overlap(lever, housing, reason="Lever pivot pin is captured in housing bracket holes.")
    ctx.allow_overlap(lever, plunger, reason="Lever tip is captured in plunger slot.")
    ctx.allow_isolated_part(lever, reason="Lever pivot pin has radial clearance inside housing hole.")

    ctx.expect_within(plunger, housing, axes="yz", margin=0.001, name="Plunger centered in bore")

    with ctx.pose({plunger_joint: 0.015, lever_joint: -0.75}):
        ctx.expect_overlap(plunger, housing, axes="x", min_overlap=0.02, name="Plunger retained in housing")

    return ctx.report()


object_model = build_object_model()
