from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_candy_vendor")

    red = model.material("red_enamel", rgba=(0.73, 0.05, 0.04, 1.0))
    dark_red = model.material("dark_red_edges", rgba=(0.36, 0.02, 0.02, 1.0))
    clear = model.material("clear_acrylic", rgba=(0.72, 0.95, 1.0, 0.34))
    smoked_clear = model.material("smoked_clear", rgba=(0.44, 0.58, 0.66, 0.46))
    chrome = model.material("brushed_chrome", rgba=(0.78, 0.76, 0.70, 1.0))
    black = model.material("black_shadow", rgba=(0.015, 0.013, 0.012, 1.0))
    brass = model.material("aged_brass", rgba=(0.93, 0.63, 0.21, 1.0))
    wall = model.material("painted_wall_plate", rgba=(0.78, 0.76, 0.71, 1.0))
    candy_red = model.material("candy_red", rgba=(0.95, 0.04, 0.07, 1.0))
    candy_yellow = model.material("candy_yellow", rgba=(1.0, 0.82, 0.05, 1.0))
    candy_blue = model.material("candy_blue", rgba=(0.08, 0.28, 0.95, 1.0))
    candy_green = model.material("candy_green", rgba=(0.0, 0.68, 0.18, 1.0))

    body = model.part("body")
    # Shallow wall-hung cabinet and mounting hardware.
    body.visual(
        Box((0.42, 0.140, 0.320)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=red,
        name="lower_body",
    )
    body.visual(
        Box((0.46, 0.030, 0.70)),
        origin=Origin(xyz=(0.0, 0.085, 0.350)),
        material=wall,
        name="wall_backer",
    )
    body.visual(
        Box((0.16, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.098, 0.635)),
        material=chrome,
        name="upper_wall_bracket",
    )
    body.visual(
        Box((0.16, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.098, 0.065)),
        material=chrome,
        name="lower_wall_bracket",
    )
    for sx in (-0.070, 0.070):
        for z in (0.065, 0.635):
            body.visual(
                Cylinder(radius=0.009, length=0.004),
                origin=Origin(xyz=(sx, 0.088, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=black,
                name=f"mount_screw_{sx}_{z}",
            )

    # Upper hopper frame: a clear window, boxed metal rails, and a flat back.
    body.visual(
        Box((0.36, 0.018, 0.310)),
        origin=Origin(xyz=(0.0, 0.062, 0.470)),
        material=dark_red,
        name="hopper_back",
    )
    body.visual(
        Box((0.310, 0.006, 0.270)),
        origin=Origin(xyz=(0.0, -0.081, 0.475)),
        material=clear,
        name="front_hopper_window",
    )
    body.visual(
        Box((0.380, 0.030, 0.034)),
        origin=Origin(xyz=(0.0, -0.068, 0.626)),
        material=red,
        name="hopper_top_cap",
    )
    body.visual(
        Box((0.380, 0.030, 0.034)),
        origin=Origin(xyz=(0.0, -0.068, 0.315)),
        material=red,
        name="hopper_floor",
    )
    for sx, nm in ((-0.182, "hopper_side_0"), (0.182, "hopper_side_1")):
        body.visual(
            Box((0.030, 0.036, 0.320)),
            origin=Origin(xyz=(sx, -0.066, 0.470)),
            material=red,
            name=nm,
        )
    body.visual(
        Box((0.330, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.087, 0.345)),
        material=chrome,
        name="hopper_lower_trim",
    )
    body.visual(
        Box((0.330, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.087, 0.604)),
        material=chrome,
        name="hopper_upper_trim",
    )
    body.visual(
        Box((0.015, 0.022, 0.292)),
        origin=Origin(xyz=(0.2015, 0.064, 0.462)),
        material=dark_red,
        name="refill_hinge_post",
    )

    # Coin mechanism face and bearing.
    body.visual(
        Box((0.245, 0.012, 0.155)),
        origin=Origin(xyz=(0.0, -0.077, 0.212)),
        material=chrome,
        name="coin_faceplate",
    )
    body.visual(
        Cylinder(radius=0.082, length=0.014),
        origin=Origin(xyz=(0.0, -0.079, 0.224), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="coin_bearing",
    )
    body.visual(
        Box((0.090, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.088, 0.284)),
        material=black,
        name="coin_slot_label",
    )

    # Small retrieval cup as an open tray below the coin works.
    body.visual(
        Box((0.205, 0.070, 0.012)),
        origin=Origin(xyz=(0.0, -0.105, 0.055)),
        material=chrome,
        name="cup_floor",
    )
    body.visual(
        Box((0.205, 0.012, 0.075)),
        origin=Origin(xyz=(0.0, -0.066, 0.090)),
        material=black,
        name="cup_shadow_back",
    )
    for sx, nm in ((-0.108, "cup_side_0"), (0.108, "cup_side_1")):
        body.visual(
            Box((0.012, 0.070, 0.068)),
            origin=Origin(xyz=(sx, -0.105, 0.087)),
            material=chrome,
            name=nm,
        )
    for sx, nm in ((-0.105, "cup_hinge_ear_0"), (0.105, "cup_hinge_ear_1")):
        body.visual(
            Box((0.014, 0.018, 0.020)),
            origin=Origin(xyz=(sx, -0.148, 0.058)),
            material=brass,
            name=nm,
        )

    # A single fixed candy mound, built from overlapping sweets seated on a tray.
    candy = model.part("candy_mound")
    candy.visual(
        Box((0.265, 0.076, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=candy_red,
        name="candy_base",
    )
    candy_specs = [
        (-0.105, -0.026, 0.026, candy_yellow),
        (-0.070, 0.020, 0.025, candy_blue),
        (-0.038, -0.018, 0.028, candy_green),
        (-0.006, 0.018, 0.027, candy_red),
        (0.032, -0.022, 0.026, candy_yellow),
        (0.066, 0.017, 0.025, candy_green),
        (0.104, -0.019, 0.027, candy_blue),
        (-0.085, -0.003, 0.044, candy_red),
        (-0.018, -0.004, 0.047, candy_blue),
        (0.047, 0.000, 0.046, candy_yellow),
        (0.088, 0.004, 0.044, candy_green),
    ]
    for idx, (x, y, z, mat) in enumerate(candy_specs):
        candy.visual(
            Sphere(radius=0.021),
            origin=Origin(xyz=(x, y, z)),
            material=mat,
            name=f"candy_{idx}",
        )
    model.articulation(
        "body_to_candy_mound",
        ArticulationType.FIXED,
        parent=body,
        child=candy,
        origin=Origin(xyz=(0.0, -0.020, 0.332)),
    )

    # Rotary coin knob on a horizontal front-to-back shaft.
    coin_knob = model.part("coin_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.078,
            0.034,
            body_style="lobed",
            base_diameter=0.058,
            top_diameter=0.070,
            edge_radius=0.002,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=8, depth=0.002, width=0.004),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0012),
            center=False,
        ),
        "coin_knob_mesh",
    )
    coin_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_body",
    )
    coin_knob.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="shaft_stub",
    )
    coin_knob.visual(
        Box((0.010, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, -0.036, 0.010), rpy=(0.0, 0.0, 0.0)),
        material=black,
        name="coin_slot_mark",
    )
    model.articulation(
        "body_to_coin_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=coin_knob,
        origin=Origin(xyz=(0.0, -0.086, 0.224)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    # Bottom-hinged cup flap; at positive rotation the top swings forward/down.
    cup_flap = model.part("cup_flap")
    cup_flap.visual(
        Box((0.182, 0.008, 0.095)),
        origin=Origin(xyz=(0.0, -0.004, 0.052)),
        material=smoked_clear,
        name="flap_panel",
    )
    cup_flap.visual(
        Cylinder(radius=0.006, length=0.196),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="flap_hinge_barrel",
    )
    cup_flap.visual(
        Box((0.052, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, 0.082)),
        material=chrome,
        name="flap_pull_lip",
    )
    model.articulation(
        "body_to_cup_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cup_flap,
        origin=Origin(xyz=(0.0, -0.148, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    # Side refill door on a vertical hinge along one side of the clear hopper.
    refill_door = model.part("side_refill_door")
    refill_door.visual(
        Box((0.008, 0.116, 0.266)),
        origin=Origin(xyz=(0.0, -0.058, 0.0)),
        material=clear,
        name="door_panel",
    )
    refill_door.visual(
        Cylinder(radius=0.005, length=0.282),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="door_hinge_barrel",
    )
    refill_door.visual(
        Box((0.012, 0.020, 0.050)),
        origin=Origin(xyz=(0.008, -0.110, 0.000)),
        material=chrome,
        name="door_pull_tab",
    )
    model.articulation(
        "body_to_side_refill_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=refill_door,
        origin=Origin(xyz=(0.214, 0.064, 0.470)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    candy = object_model.get_part("candy_mound")
    knob = object_model.get_part("coin_knob")
    flap = object_model.get_part("cup_flap")
    door = object_model.get_part("side_refill_door")
    knob_joint = object_model.get_articulation("body_to_coin_knob")
    flap_joint = object_model.get_articulation("body_to_cup_flap")
    door_joint = object_model.get_articulation("body_to_side_refill_door")

    ctx.check(
        "coin knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.check(
        "coin knob shaft is horizontal",
        abs(abs(knob_joint.axis[1]) - 1.0) < 1e-6,
        details=f"axis={knob_joint.axis}",
    )
    ctx.check(
        "cup flap has bounded bottom hinge",
        flap_joint.motion_limits is not None
        and flap_joint.motion_limits.lower == 0.0
        and flap_joint.motion_limits.upper is not None
        and 1.0 <= flap_joint.motion_limits.upper <= 1.4,
        details=f"limits={flap_joint.motion_limits}",
    )
    ctx.check(
        "refill door has vertical hinge",
        abs(door_joint.axis[2] - 1.0) < 1e-6,
        details=f"axis={door_joint.axis}",
    )

    ctx.expect_contact(
        candy,
        body,
        elem_a="candy_base",
        elem_b="hopper_floor",
        contact_tol=0.001,
        name="candy pile rests on hopper floor",
    )
    ctx.expect_gap(
        body,
        knob,
        axis="y",
        positive_elem="coin_bearing",
        negative_elem="shaft_stub",
        max_penetration=0.001,
        max_gap=0.006,
        name="coin shaft sits in front of bearing",
    )

    closed_flap_aabb = ctx.part_world_aabb(flap)
    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({flap_joint: 1.05}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({door_joint: 1.20}):
        open_door_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "cup flap opens outward and downward",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][1] < closed_flap_aabb[0][1] - 0.035
        and open_flap_aabb[1][2] < closed_flap_aabb[1][2] - 0.018,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )
    ctx.check(
        "side refill door swings outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.060,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
