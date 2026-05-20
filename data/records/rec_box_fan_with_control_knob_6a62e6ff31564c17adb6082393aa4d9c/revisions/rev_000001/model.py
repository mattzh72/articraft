from __future__ import annotations

import math

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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clip_on_desk_fan")

    plastic = model.material("warm_white_plastic", rgba=(0.88, 0.86, 0.78, 1.0))
    dark = model.material("dark_gray_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    wire = model.material("white_wire", rgba=(0.94, 0.94, 0.90, 1.0))
    blade_blue = model.material("translucent_blue_blades", rgba=(0.34, 0.58, 0.82, 0.82))
    label = model.material("black_label_marks", rgba=(0.02, 0.02, 0.018, 1.0))

    # Root clip/C-clamp bracket.  The object frame is the swivel center.
    clamp = model.part("clamp")
    clamp.visual(
        Cylinder(radius=0.014, length=0.155),
        origin=Origin(xyz=(-0.028, 0.0, -0.078)),
        material=dark,
        name="vertical_post",
    )
    clamp.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=dark,
        name="lower_swivel_disk",
    )
    clamp.visual(
        Box((0.028, 0.058, 0.175)),
        origin=Origin(xyz=(-0.098, 0.0, -0.250)),
        material=dark,
        name="clamp_spine",
    )
    clamp.visual(
        Box((0.145, 0.058, 0.030)),
        origin=Origin(xyz=(-0.034, 0.0, -0.170)),
        material=dark,
        name="upper_jaw",
    )
    clamp.visual(
        Box((0.145, 0.058, 0.030)),
        origin=Origin(xyz=(-0.034, 0.0, -0.330)),
        material=dark,
        name="lower_jaw",
    )
    clamp.visual(
        Box((0.058, 0.046, 0.007)),
        origin=Origin(xyz=(0.022, 0.0, -0.188)),
        material=rubber,
        name="upper_rubber_pad",
    )
    clamp.visual(
        Cylinder(radius=0.008, length=0.156),
        origin=Origin(xyz=(0.020, 0.0, -0.306)),
        material=plastic,
        name="clamp_screw",
    )
    clamp.visual(
        Cylinder(radius=0.023, length=0.009),
        origin=Origin(xyz=(0.020, 0.0, -0.245)),
        material=rubber,
        name="lower_rubber_pad",
    )
    clamp.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.020, 0.0, -0.374)),
        material=plastic,
        name="thumbwheel",
    )

    # Swiveling fan body.  Local +X is the airflow/front direction.
    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=plastic,
        name="upper_swivel_disk",
    )
    housing.visual(
        Box((0.074, 0.026, 0.022)),
        origin=Origin(xyz=(0.040, 0.0, 0.009)),
        material=plastic,
        name="swivel_neck",
    )
    housing.visual(
        Box((0.074, 0.255, 0.026)),
        origin=Origin(xyz=(0.110, 0.0, 0.112)),
        material=plastic,
        name="top_frame_bar",
    )
    housing.visual(
        Box((0.074, 0.255, 0.026)),
        origin=Origin(xyz=(0.110, 0.0, -0.112)),
        material=plastic,
        name="bottom_frame_bar",
    )
    housing.visual(
        Box((0.074, 0.026, 0.198)),
        origin=Origin(xyz=(0.110, 0.112, 0.0)),
        material=plastic,
        name="side_frame_bar_0",
    )
    housing.visual(
        Box((0.074, 0.026, 0.198)),
        origin=Origin(xyz=(0.110, -0.112, 0.0)),
        material=plastic,
        name="side_frame_bar_1",
    )
    housing.visual(
        Box((0.026, 0.174, 0.174)),
        origin=Origin(xyz=(0.064, 0.0, 0.0)),
        material=plastic,
        name="rear_panel",
    )
    housing.visual(
        Box((0.026, 0.174, 0.014)),
        origin=Origin(xyz=(0.064, 0.0, 0.093)),
        material=plastic,
        name="rear_top_lip",
    )
    housing.visual(
        Box((0.026, 0.174, 0.014)),
        origin=Origin(xyz=(0.064, 0.0, -0.093)),
        material=plastic,
        name="rear_bottom_lip",
    )
    housing.visual(
        Box((0.026, 0.014, 0.174)),
        origin=Origin(xyz=(0.064, 0.093, 0.0)),
        material=plastic,
        name="rear_side_lip_0",
    )
    housing.visual(
        Box((0.026, 0.014, 0.174)),
        origin=Origin(xyz=(0.064, -0.093, 0.0)),
        material=plastic,
        name="rear_side_lip_1",
    )
    housing.visual(
        Cylinder(radius=0.031, length=0.040),
        origin=Origin(xyz=(0.094, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="motor_boss",
    )
    housing.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="axle_bearing",
    )

    # Rear face speed legends around the knob.
    housing.visual(
        Box((0.0025, 0.009, 0.026)),
        origin=Origin(xyz=(0.050, 0.068, 0.083)),
        material=label,
        name="speed_mark_off",
    )
    housing.visual(
        Box((0.0025, 0.022, 0.006)),
        origin=Origin(xyz=(0.050, 0.091, 0.058)),
        material=label,
        name="speed_mark_high",
    )

    # Wire grille: concentric rings plus diameter spokes in front of the blades.
    for index, radius in enumerate((0.102, 0.075, 0.048)):
        ring_mesh = mesh_from_geometry(
            TorusGeometry(radius=radius, tube=0.0022, radial_segments=14, tubular_segments=84),
            f"grille_ring_{index}",
        )
        housing.visual(
            ring_mesh,
            origin=Origin(xyz=(0.154, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wire,
            name=f"grille_ring_{index}",
        )
    for index, angle in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0)):
        housing.visual(
            Cylinder(radius=0.0018, length=0.202),
            origin=Origin(xyz=(0.154, 0.0, 0.0), rpy=(angle - math.pi / 2.0, 0.0, 0.0)),
            material=wire,
            name=f"grille_spoke_{index}",
        )
    for index, (y_pos, z_pos) in enumerate(((0.102, 0.0), (-0.102, 0.0), (0.0, 0.102), (0.0, -0.102))):
        housing.visual(
            Cylinder(radius=0.0030, length=0.014),
            origin=Origin(xyz=(0.150, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wire,
            name=f"grille_clip_{index}",
        )
    housing.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.156, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wire,
        name="grille_center_cap",
    )

    # Three-blade rotor, mounted on the central axle behind the grille.
    propeller = model.part("propeller")
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.073,
            0.020,
            3,
            thickness=0.016,
            blade_pitch_deg=30.0,
            blade_sweep_deg=24.0,
            blade=FanRotorBlade(shape="broad", tip_pitch_deg=13.0, camber=0.12, tip_clearance=0.002),
            hub=FanRotorHub(style="domed", bore_diameter=0.006),
        ),
        "three_blade_propeller",
    )
    propeller.visual(rotor_mesh, material=blade_blue, name="three_blade_propeller")
    propeller.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(),
        material=dark,
        name="axle_stub",
    )

    # Rear rotary speed selector knob.
    speed_knob = model.part("speed_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.041,
            0.019,
            body_style="faceted",
            base_diameter=0.043,
            top_diameter=0.034,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0010, width=0.0018),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "speed_selector_knob",
    )
    speed_knob.visual(knob_mesh, material=dark, name="speed_selector_knob")

    model.articulation(
        "swivel",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=housing,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "axle",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=propeller,
        origin=Origin(xyz=(0.127, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=40.0),
    )
    model.articulation(
        "speed_selector",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(0.051, 0.069, 0.056), rpy=(0.0, -math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=4.0, lower=-0.25, upper=2.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp = object_model.get_part("clamp")
    housing = object_model.get_part("housing")
    propeller = object_model.get_part("propeller")
    speed_knob = object_model.get_part("speed_knob")
    swivel = object_model.get_articulation("swivel")
    axle = object_model.get_articulation("axle")
    selector = object_model.get_articulation("speed_selector")

    ctx.expect_gap(
        housing,
        propeller,
        axis="x",
        min_gap=0.010,
        positive_elem="grille_ring_0",
        name="propeller sits behind the front wire grille",
    )
    ctx.expect_within(
        propeller,
        housing,
        axes="yz",
        outer_elem="grille_ring_0",
        margin=0.004,
        name="propeller fits inside the square housing grille opening",
    )
    ctx.expect_gap(
        housing,
        speed_knob,
        axis="x",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="rear_panel",
        name="speed selector is mounted on rear face",
    )
    ctx.expect_gap(
        housing,
        clamp,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="upper_swivel_disk",
        negative_elem="lower_swivel_disk",
        name="swivel disks are seated together",
    )

    ctx.check(
        "three primary rotary mechanisms are present",
        swivel is not None and axle is not None and selector is not None,
        details="Expected swivel, propeller axle, and rear speed selector joints.",
    )

    rest_aabb = ctx.part_element_world_aabb(housing, elem="rear_panel")
    with ctx.pose({swivel: 0.55, selector: 1.6, axle: 1.2}):
        turned_aabb = ctx.part_element_world_aabb(housing, elem="rear_panel")
    ctx.check(
        "swivel yaws the fan housing",
        rest_aabb is not None
        and turned_aabb is not None
        and abs((turned_aabb[0][1] + turned_aabb[1][1]) - (rest_aabb[0][1] + rest_aabb[1][1])) > 0.035,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
