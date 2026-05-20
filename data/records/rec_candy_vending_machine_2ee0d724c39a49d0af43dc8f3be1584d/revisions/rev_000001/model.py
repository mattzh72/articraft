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
    model = ArticulatedObject(name="wall_candy_vending_machine")

    body_red = model.material("body_red", rgba=(0.73, 0.10, 0.08, 1.0))
    body_dark = model.material("body_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.79, 0.82, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.78, 0.90, 0.98, 0.35))
    clear_acrylic = model.material("clear_acrylic", rgba=(0.92, 0.97, 1.00, 0.28))
    cup_black = model.material("cup_black", rgba=(0.11, 0.11, 0.12, 1.0))
    trim_black = model.material("trim_black", rgba=(0.07, 0.07, 0.08, 1.0))
    label_white = model.material("label_white", rgba=(0.92, 0.92, 0.90, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.006, 0.240, 0.420)),
        origin=Origin(xyz=(0.003, 0.000, 0.210)),
        material=body_dark,
        name="backplate",
    )
    cabinet.visual(
        Box((0.104, 0.232, 0.012)),
        origin=Origin(xyz=(0.058, 0.000, 0.414)),
        material=body_red,
        name="top_shell",
    )
    cabinet.visual(
        Box((0.104, 0.232, 0.012)),
        origin=Origin(xyz=(0.058, 0.000, 0.006)),
        material=body_red,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((0.104, 0.012, 0.396)),
        origin=Origin(xyz=(0.058, -0.110, 0.210)),
        material=body_red,
        name="left_shell",
    )
    cabinet.visual(
        Box((0.104, 0.012, 0.396)),
        origin=Origin(xyz=(0.058, 0.110, 0.210)),
        material=body_red,
        name="right_shell",
    )
    cabinet.visual(
        Box((0.012, 0.188, 0.060)),
        origin=Origin(xyz=(0.104, 0.000, 0.384)),
        material=body_red,
        name="hopper_header",
    )
    cabinet.visual(
        Box((0.012, 0.024, 0.250)),
        origin=Origin(xyz=(0.104, -0.082, 0.238)),
        material=body_red,
        name="left_front_rail",
    )
    cabinet.visual(
        Box((0.012, 0.024, 0.250)),
        origin=Origin(xyz=(0.104, 0.082, 0.238)),
        material=body_red,
        name="right_front_rail",
    )
    cabinet.visual(
        Box((0.012, 0.188, 0.020)),
        origin=Origin(xyz=(0.104, 0.000, 0.166)),
        material=body_red,
        name="hopper_sill",
    )
    cabinet.visual(
        Box((0.012, 0.150, 0.092)),
        origin=Origin(xyz=(0.104, 0.000, 0.143)),
        material=body_red,
        name="control_fascia",
    )
    cabinet.visual(
        Box((0.012, 0.030, 0.092)),
        origin=Origin(xyz=(0.104, -0.060, 0.058)),
        material=body_red,
        name="cup_left_apron",
    )
    cabinet.visual(
        Box((0.012, 0.030, 0.092)),
        origin=Origin(xyz=(0.104, 0.060, 0.058)),
        material=body_red,
        name="cup_right_apron",
    )
    cabinet.visual(
        Box((0.036, 0.050, 0.052)),
        origin=Origin(xyz=(0.024, 0.000, 0.060)),
        material=body_red,
        name="cup_support_block",
    )
    cabinet.visual(
        Box((0.082, 0.156, 0.012)),
        origin=Origin(
            xyz=(0.064, 0.000, 0.145),
            rpy=(0.000, -0.68, 0.000),
        ),
        material=body_dark,
        name="inner_chute_ramp",
    )
    cabinet.visual(
        Box((0.030, 0.062, 0.018)),
        origin=Origin(xyz=(0.096, 0.000, 0.198)),
        material=trim_black,
        name="coin_slot_bezel",
    )
    cabinet.visual(
        Box((0.004, 0.042, 0.004)),
        origin=Origin(xyz=(0.108, 0.000, 0.198)),
        material=body_dark,
        name="coin_slot",
    )
    cabinet.visual(
        Box((0.006, 0.078, 0.078)),
        origin=Origin(xyz=(0.107, 0.000, 0.128)),
        material=chrome,
        name="knob_escutcheon",
    )
    cabinet.visual(
        Box((0.004, 0.090, 0.028)),
        origin=Origin(xyz=(0.108, 0.000, 0.040)),
        material=label_white,
        name="lower_label",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.110, 0.240, 0.420)),
        mass=5.5,
        origin=Origin(xyz=(0.055, 0.000, 0.210)),
    )

    hopper = model.part("hopper")
    hopper.visual(
        Box((0.004, 0.170, 0.190)),
        origin=Origin(xyz=(0.088, 0.000, 0.000)),
        material=clear_acrylic,
        name="front_panel",
    )
    hopper.visual(
        Box((0.090, 0.004, 0.190)),
        origin=Origin(xyz=(0.045, -0.083, 0.000)),
        material=clear_acrylic,
        name="left_side_panel",
    )
    hopper.visual(
        Box((0.090, 0.170, 0.004)),
        origin=Origin(xyz=(0.045, 0.000, 0.093)),
        material=clear_acrylic,
        name="top_panel",
    )
    hopper.visual(
        Box((0.090, 0.004, 0.030)),
        origin=Origin(xyz=(0.045, 0.083, 0.078)),
        material=clear_acrylic,
        name="right_top_frame",
    )
    hopper.visual(
        Box((0.060, 0.004, 0.028)),
        origin=Origin(xyz=(0.030, 0.083, -0.078)),
        material=clear_acrylic,
        name="right_bottom_frame",
    )
    hopper.visual(
        Box((0.076, 0.170, 0.004)),
        origin=Origin(
            xyz=(0.038, 0.000, -0.054),
            rpy=(0.000, -0.86, 0.000),
        ),
        material=smoked_clear,
        name="funnel_panel",
    )
    hopper.visual(
        Box((0.024, 0.170, 0.018)),
        origin=Origin(xyz=(0.078, 0.000, -0.078)),
        material=clear_acrylic,
        name="front_lip",
    )
    hopper.visual(
        Cylinder(radius=0.004, length=0.126),
        origin=Origin(
            xyz=(0.005, 0.083, -0.002),
            rpy=(0.000, 0.000, 0.000),
        ),
        material=chrome,
        name="door_hinge_post",
    )
    hopper.inertial = Inertial.from_geometry(
        Box((0.090, 0.170, 0.190)),
        mass=0.8,
        origin=Origin(xyz=(0.045, 0.000, 0.000)),
    )

    refill_door = model.part("refill_door")
    refill_door.visual(
        Box((0.082, 0.004, 0.126)),
        origin=Origin(xyz=(0.041, 0.000, 0.000)),
        material=clear_acrylic,
        name="door_panel",
    )
    refill_door.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(
            xyz=(0.068, 0.007, 0.000),
            rpy=(math.pi / 2.0, 0.000, 0.000),
        ),
        material=chrome,
        name="door_pull",
    )
    refill_door.inertial = Inertial.from_geometry(
        Box((0.082, 0.010, 0.126)),
        mass=0.15,
        origin=Origin(xyz=(0.041, 0.000, 0.000)),
    )

    cup_housing = model.part("retrieval_cup")
    cup_housing.visual(
        Box((0.062, 0.078, 0.006)),
        origin=Origin(xyz=(-0.031, 0.000, -0.003)),
        material=cup_black,
        name="cup_floor",
    )
    cup_housing.visual(
        Box((0.058, 0.006, 0.054)),
        origin=Origin(xyz=(-0.029, -0.036, 0.027)),
        material=cup_black,
        name="left_wall",
    )
    cup_housing.visual(
        Box((0.058, 0.006, 0.054)),
        origin=Origin(xyz=(-0.029, 0.036, 0.027)),
        material=cup_black,
        name="right_wall",
    )
    cup_housing.visual(
        Box((0.006, 0.078, 0.054)),
        origin=Origin(xyz=(-0.059, 0.000, 0.027)),
        material=cup_black,
        name="rear_wall",
    )
    cup_housing.visual(
        Box((0.042, 0.078, 0.006)),
        origin=Origin(xyz=(-0.043, 0.000, 0.056)),
        material=cup_black,
        name="top_hood",
    )
    cup_housing.visual(
        Box((0.014, 0.042, 0.026)),
        origin=Origin(xyz=(-0.059, 0.000, 0.070)),
        material=body_dark,
        name="chute_collar",
    )
    cup_housing.inertial = Inertial.from_geometry(
        Box((0.062, 0.078, 0.090)),
        mass=0.4,
        origin=Origin(xyz=(-0.031, 0.000, 0.036)),
    )

    retrieval_flap = model.part("retrieval_flap")
    retrieval_flap.visual(
        Box((0.004, 0.074, 0.058)),
        origin=Origin(xyz=(0.002, 0.000, 0.029)),
        material=chrome,
        name="flap_panel",
    )
    retrieval_flap.visual(
        Box((0.014, 0.046, 0.010)),
        origin=Origin(xyz=(0.011, 0.000, 0.054)),
        material=trim_black,
        name="flap_lip",
    )
    retrieval_flap.inertial = Inertial.from_geometry(
        Box((0.014, 0.074, 0.058)),
        mass=0.10,
        origin=Origin(xyz=(0.007, 0.000, 0.029)),
    )

    coin_knob = model.part("coin_knob")
    coin_knob.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.007, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=chrome,
        name="shaft_stub",
    )
    coin_knob.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.021, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=chrome,
        name="knob_body",
    )
    coin_knob.visual(
        Box((0.014, 0.014, 0.032)),
        origin=Origin(xyz=(0.030, 0.000, 0.016)),
        material=trim_black,
        name="pointer_fin",
    )
    coin_knob.visual(
        Box((0.012, 0.034, 0.010)),
        origin=Origin(xyz=(0.028, 0.000, -0.016)),
        material=trim_black,
        name="grip_bar",
    )
    coin_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.040),
        mass=0.22,
        origin=Origin(xyz=(0.020, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
    )

    model.articulation(
        "cabinet_to_hopper",
        ArticulationType.FIXED,
        parent=cabinet,
        child=hopper,
        origin=Origin(xyz=(0.110, 0.000, 0.276)),
    )
    model.articulation(
        "hopper_to_refill_door",
        ArticulationType.REVOLUTE,
        parent=hopper,
        child=refill_door,
        origin=Origin(xyz=(0.004, 0.087, -0.002)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=0.000,
            upper=1.30,
        ),
    )
    model.articulation(
        "cabinet_to_retrieval_cup",
        ArticulationType.FIXED,
        parent=cabinet,
        child=cup_housing,
        origin=Origin(xyz=(0.104, 0.000, 0.034)),
    )
    model.articulation(
        "cup_to_flap",
        ArticulationType.REVOLUTE,
        parent=cup_housing,
        child=retrieval_flap,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        axis=(0.000, 1.000, 0.000),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.000,
            upper=1.18,
        ),
    )
    model.articulation(
        "cabinet_to_coin_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=coin_knob,
        origin=Origin(xyz=(0.110, 0.000, 0.128)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    hopper = object_model.get_part("hopper")
    coin_knob = object_model.get_part("coin_knob")
    cup_housing = object_model.get_part("retrieval_cup")
    retrieval_flap = object_model.get_part("retrieval_flap")
    refill_door = object_model.get_part("refill_door")

    knob_joint = object_model.get_articulation("cabinet_to_coin_knob")
    flap_joint = object_model.get_articulation("cup_to_flap")
    door_joint = object_model.get_articulation("hopper_to_refill_door")

    ctx.check(
        "all prompt parts present",
        all(
            part is not None
            for part in (cabinet, hopper, coin_knob, cup_housing, retrieval_flap, refill_door)
        ),
        details="One or more required parts could not be resolved.",
    )
    ctx.check(
        "coin knob uses continuous horizontal shaft",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )
    ctx.check(
        "retrieval flap uses bottom horizontal hinge",
        flap_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(flap_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={flap_joint.articulation_type}, axis={flap_joint.axis}",
    )
    ctx.check(
        "refill door uses vertical hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}",
    )

    ctx.expect_contact(
        hopper,
        cabinet,
        contact_tol=0.002,
        name="hopper mounts against cabinet face",
    )
    ctx.expect_contact(
        cup_housing,
        cabinet,
        contact_tol=0.002,
        name="retrieval cup housing seats in cabinet",
    )
    ctx.expect_contact(
        coin_knob,
        cabinet,
        contact_tol=0.002,
        name="coin knob shaft seats at cabinet face",
    )
    ctx.expect_contact(
        retrieval_flap,
        cup_housing,
        contact_tol=0.002,
        name="flap closes against retrieval cup",
    )
    ctx.expect_contact(
        refill_door,
        hopper,
        contact_tol=0.003,
        name="refill door closes against hopper side",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    flap_closed = _aabb_center(ctx.part_element_world_aabb(retrieval_flap, elem="flap_panel"))
    with ctx.pose({flap_joint: math.radians(55.0)}):
        flap_open = _aabb_center(ctx.part_element_world_aabb(retrieval_flap, elem="flap_panel"))
    ctx.check(
        "retrieval flap swings outward and down",
        flap_closed is not None
        and flap_open is not None
        and flap_open[0] > flap_closed[0] + 0.015
        and flap_open[2] < flap_closed[2] - 0.008,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    door_closed = _aabb_center(ctx.part_element_world_aabb(refill_door, elem="door_panel"))
    with ctx.pose({door_joint: math.radians(70.0)}):
        door_open = _aabb_center(ctx.part_element_world_aabb(refill_door, elem="door_panel"))
    ctx.check(
        "refill door swings outward from right side",
        door_closed is not None
        and door_open is not None
        and door_open[1] > door_closed[1] + 0.020,
        details=f"closed={door_closed}, open={door_open}",
    )

    knob_closed = _aabb_center(ctx.part_element_world_aabb(coin_knob, elem="pointer_fin"))
    with ctx.pose({knob_joint: math.pi / 2.0}):
        knob_quarter = _aabb_center(ctx.part_element_world_aabb(coin_knob, elem="pointer_fin"))
    ctx.check(
        "coin knob rotates about front facing shaft",
        knob_closed is not None
        and knob_quarter is not None
        and abs(knob_quarter[0] - knob_closed[0]) < 0.003
        and knob_quarter[1] < knob_closed[1] - 0.012,
        details=f"closed={knob_closed}, quarter_turn={knob_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
