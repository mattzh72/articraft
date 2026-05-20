from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


METAL = Material("warm_galvanized_metal", rgba=(0.62, 0.65, 0.64, 1.0))
DARK_METAL = Material("dark_motor_metal", rgba=(0.08, 0.09, 0.09, 1.0))
BLACK = Material("black_blades", rgba=(0.015, 0.017, 0.018, 1.0))
AGED_ALUMINUM = Material("aged_aluminum_louvers", rgba=(0.74, 0.77, 0.76, 1.0))
WOOD = Material("painted_gable_frame", rgba=(0.82, 0.84, 0.80, 1.0))
CONTROL = Material("cream_control_plate", rgba=(0.78, 0.75, 0.66, 1.0))
KNOB_MAT = Material("black_speed_knob", rgba=(0.025, 0.023, 0.021, 1.0))


def _drum_housing_mesh():
    """Hollow round fan drum with heavier rolled lips on both ends."""
    outer_radius = 0.315
    inner_radius = 0.270
    depth = 0.320
    lip_depth = 0.030
    flange_radius = 0.345

    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(depth / 2.0, both=True)
    front_lip = (
        cq.Workplane("XY")
        .circle(flange_radius)
        .circle(inner_radius)
        .extrude(lip_depth / 2.0, both=True)
        .translate((0.0, 0.0, depth / 2.0 - lip_depth / 2.0))
    )
    rear_lip = (
        cq.Workplane("XY")
        .circle(flange_radius)
        .circle(inner_radius)
        .extrude(lip_depth / 2.0, both=True)
        .translate((0.0, 0.0, -depth / 2.0 + lip_depth / 2.0))
    )
    return tube.union(front_lip).union(rear_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="attic_gable_box_fan")

    frame = model.part("frame")
    # Rectangular gable vent frame: thick painted boards around the shutter opening.
    frame.visual(Box((1.10, 0.10, 0.075)), origin=Origin(xyz=(0.0, 0.0, 0.3625)), material=WOOD, name="top_rail")
    frame.visual(Box((1.10, 0.10, 0.075)), origin=Origin(xyz=(0.0, 0.0, -0.3625)), material=WOOD, name="bottom_rail")
    frame.visual(Box((0.075, 0.10, 0.80)), origin=Origin(xyz=(-0.5125, 0.0, 0.0)), material=WOOD, name="side_rail_0")
    frame.visual(Box((0.075, 0.10, 0.80)), origin=Origin(xyz=(0.5125, 0.0, 0.0)), material=WOOD, name="side_rail_1")

    # Front side louver jambs and hinge headers, tied into the rectangular frame.
    frame.visual(Box((0.035, 0.080, 0.690)), origin=Origin(xyz=(-0.435, 0.075, 0.0)), material=METAL, name="louver_jamb_0")
    frame.visual(Box((0.035, 0.080, 0.690)), origin=Origin(xyz=(0.435, 0.075, 0.0)), material=METAL, name="louver_jamb_1")
    frame.visual(Box((0.820, 0.032, 0.025)), origin=Origin(xyz=(0.0, 0.120, 0.255)), material=METAL, name="upper_hinge_header")
    frame.visual(Box((0.820, 0.032, 0.025)), origin=Origin(xyz=(0.0, 0.120, -0.020)), material=METAL, name="lower_hinge_header")

    # Hinge support blocks at the jambs; the rotating flap barrels stop just inside them.
    for z, label in ((0.255, "upper"), (-0.020, "lower")):
        frame.visual(Box((0.040, 0.046, 0.046)), origin=Origin(xyz=(-0.415, 0.145, z)), material=METAL, name=f"{label}_hinge_block_0")
        frame.visual(Box((0.040, 0.046, 0.046)), origin=Origin(xyz=(0.415, 0.145, z)), material=METAL, name=f"{label}_hinge_block_1")

    # Hollow circular drum through the rectangular frame.
    frame.visual(
        mesh_from_cadquery(_drum_housing_mesh(), "drum_shell", tolerance=0.0015),
        origin=Origin(xyz=(0.0, -0.050, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=METAL,
        name="drum_shell",
    )

    # Four box-section welded straps visibly carry the round drum from the rectangular frame.
    frame.visual(Box((0.200, 0.065, 0.045)), origin=Origin(xyz=(-0.385, -0.050, 0.0)), material=METAL, name="drum_strap_0")
    frame.visual(Box((0.200, 0.065, 0.045)), origin=Origin(xyz=(0.385, -0.050, 0.0)), material=METAL, name="drum_strap_1")
    frame.visual(Box((0.065, 0.065, 0.120)), origin=Origin(xyz=(0.0, -0.050, 0.330)), material=METAL, name="drum_strap_2")
    frame.visual(Box((0.065, 0.065, 0.120)), origin=Origin(xyz=(0.0, -0.050, -0.330)), material=METAL, name="drum_strap_3")

    # Rear motor pod and four slim spokes supporting the fan shaft.
    frame.visual(Cylinder(radius=0.080, length=0.100), origin=Origin(xyz=(0.0, -0.185, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=DARK_METAL, name="motor_pod")
    frame.visual(Cylinder(radius=0.017, length=0.210), origin=Origin(xyz=(0.0, -0.075, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=DARK_METAL, name="fan_shaft")
    frame.visual(Box((0.026, 0.030, 0.630)), origin=Origin(xyz=(0.0, -0.160, 0.0), rpy=(0.0, 0.0, math.radians(45.0))), material=DARK_METAL, name="motor_spoke_0")
    frame.visual(Box((0.026, 0.030, 0.630)), origin=Origin(xyz=(0.0, -0.160, 0.0), rpy=(0.0, 0.0, -math.radians(45.0))), material=DARK_METAL, name="motor_spoke_1")

    # Wall-mounted speed control box on the lower right of the frame.
    frame.visual(Box((0.190, 0.050, 0.115)), origin=Origin(xyz=(0.640, 0.065, -0.275)), material=CONTROL, name="control_plate")
    frame.visual(Cylinder(radius=0.006, length=0.006), origin=Origin(xyz=(0.570, 0.093, -0.315), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=DARK_METAL, name="control_screw_0")
    frame.visual(Cylinder(radius=0.006, length=0.006), origin=Origin(xyz=(0.710, 0.093, -0.235), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=DARK_METAL, name="control_screw_1")

    # Continuously rotating fan blade, held on the central shaft inside the drum.
    blade = model.part("blade")
    rotor = FanRotorGeometry(
        0.245,
        0.060,
        5,
        thickness=0.035,
        blade_pitch_deg=34.0,
        blade_sweep_deg=28.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.12, tip_clearance=0.006),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.014, rear_collar_radius=0.043, bore_diameter=0.032),
    )
    blade.visual(mesh_from_geometry(rotor, "fan_blade"), material=BLACK, name="fan_blade")
    model.articulation(
        "frame_to_blade",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=blade,
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )

    # Two shutter/louver flaps, each hinged on its top edge. Closed panels sit just
    # proud of the frame; positive rotation swings their free lower edges outward.
    for name, hinge_z, height in (("upper_flap", 0.255, 0.225), ("lower_flap", -0.020, 0.225)):
        flap = model.part(name)
        flap.visual(Box((0.760, 0.014, height)), origin=Origin(xyz=(0.0, 0.000, -height / 2.0)), material=AGED_ALUMINUM, name="panel_sheet")
        flap.visual(Box((0.760, 0.020, 0.018)), origin=Origin(xyz=(0.0, 0.006, -height + 0.010)), material=AGED_ALUMINUM, name="bottom_return_lip")
        flap.visual(Cylinder(radius=0.012, length=0.790), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=METAL, name="rolled_hinge")
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=flap,
            origin=Origin(xyz=(0.0, 0.165, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.05),
        )

    # Wall speed knob, modelled as a separate continuously rotating control cap.
    knob = model.part("speed_knob")
    knob_mesh = KnobGeometry(
        0.072,
        0.042,
        body_style="skirted",
        top_diameter=0.056,
        base_diameter=0.078,
        edge_radius=0.0015,
        grip=KnobGrip(style="fluted", count=18, depth=0.0014),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(mesh_from_geometry(knob_mesh, "speed_knob"), material=KNOB_MAT, name="knob_cap")
    model.articulation(
        "frame_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=knob,
        origin=Origin(xyz=(0.640, 0.090, -0.275), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    blade = object_model.get_part("blade")
    speed_knob = object_model.get_part("speed_knob")
    upper_flap = object_model.get_part("upper_flap")
    lower_flap = object_model.get_part("lower_flap")
    blade_spin = object_model.get_articulation("frame_to_blade")
    knob_spin = object_model.get_articulation("frame_to_speed_knob")
    upper_hinge = object_model.get_articulation("frame_to_upper_flap")
    lower_hinge = object_model.get_articulation("frame_to_lower_flap")

    ctx.check("fan blade uses a continuous spin joint", blade_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("speed knob uses a continuous rotary joint", knob_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check(
        "louver hinges have realistic opening limits",
        upper_hinge.motion_limits is not None
        and lower_hinge.motion_limits is not None
        and 0.9 <= upper_hinge.motion_limits.upper <= 1.2
        and 0.9 <= lower_hinge.motion_limits.upper <= 1.2,
    )

    ctx.expect_within(
        blade,
        frame,
        axes="xz",
        inner_elem="fan_blade",
        outer_elem="drum_shell",
        margin=0.020,
        name="rotor disk fits inside round drum opening",
    )
    ctx.allow_overlap(
        frame,
        blade,
        elem_a="fan_shaft",
        elem_b="fan_blade",
        reason="The stationary shaft is intentionally captured in the rotor hub bore so the continuously spinning blade is physically supported.",
    )
    ctx.expect_contact(
        frame,
        blade,
        elem_a="fan_shaft",
        elem_b="fan_blade",
        contact_tol=0.003,
        name="rotor hub is carried by central shaft",
    )
    ctx.expect_gap(
        speed_knob,
        frame,
        axis="y",
        positive_elem="knob_cap",
        negative_elem="control_plate",
        min_gap=-0.001,
        max_gap=0.003,
        name="speed knob is seated on control plate",
    )

    closed_upper = ctx.part_element_world_aabb(upper_flap, elem="panel_sheet")
    closed_lower = ctx.part_element_world_aabb(lower_flap, elem="panel_sheet")
    with ctx.pose({upper_hinge: 1.05, lower_hinge: 1.05}):
        open_upper = ctx.part_element_world_aabb(upper_flap, elem="panel_sheet")
        open_lower = ctx.part_element_world_aabb(lower_flap, elem="panel_sheet")

    def _max_y(aabb):
        return aabb[1][1] if aabb is not None else None

    ctx.check(
        "upper louver flap opens outward",
        closed_upper is not None and open_upper is not None and _max_y(open_upper) > _max_y(closed_upper) + 0.10,
        details=f"closed={closed_upper}, open={open_upper}",
    )
    ctx.check(
        "lower louver flap opens outward",
        closed_lower is not None and open_lower is not None and _max_y(open_lower) > _max_y(closed_lower) + 0.10,
        details=f"closed={closed_lower}, open={open_lower}",
    )

    return ctx.report()


object_model = build_object_model()
