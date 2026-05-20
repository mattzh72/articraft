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
    tube_from_spline_points,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_box_fan")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.76, 1.0))
    dark_gray = model.material("dark_gray_plastic", rgba=(0.055, 0.058, 0.062, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    translucent_blue = model.material("translucent_blue_blades", rgba=(0.22, 0.50, 0.74, 0.82))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.64, 0.62, 1.0))

    housing = model.part("housing")

    # A compact square plastic box fan body, about 40 cm across and 12 cm deep.
    housing.visual(Box((0.405, 0.120, 0.042)), origin=Origin(xyz=(0.0, 0.0, 0.181)), material=warm_white, name="top_rail")
    housing.visual(Box((0.405, 0.120, 0.042)), origin=Origin(xyz=(0.0, 0.0, -0.181)), material=warm_white, name="bottom_rail")
    housing.visual(Box((0.042, 0.120, 0.405)), origin=Origin(xyz=(-0.181, 0.0, 0.0)), material=warm_white, name="side_rail_0")
    housing.visual(Box((0.042, 0.120, 0.405)), origin=Origin(xyz=(0.181, 0.0, 0.0)), material=warm_white, name="side_rail_1")

    # Raised rear lip and rear safety guard inside the box.
    for radius, tube in ((0.160, 0.0038), (0.105, 0.0028), (0.062, 0.0024)):
        ring = TorusGeometry(radius=radius, tube=tube, radial_segments=16, tubular_segments=96).rotate_x(math.pi / 2)
        housing.visual(mesh_from_geometry(ring, f"rear_guard_ring_{radius:.3f}"), origin=Origin(xyz=(0.0, 0.058, 0.0)), material=warm_white, name=f"rear_guard_ring_{radius:.3f}")

    for angle in (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4):
        housing.visual(
            Cylinder(radius=0.0024, length=0.335),
            origin=Origin(xyz=(0.0, 0.058, 0.0), rpy=(0.0, math.pi / 2 - angle, 0.0)),
            material=warm_white,
            name=f"rear_guard_bar_{int(math.degrees(angle))}",
        )

    # Static rear motor pod and four support struts.
    housing.visual(
        Cylinder(radius=0.060, length=0.055),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_gray,
        name="motor_pod",
    )
    housing.visual(
        Cylinder(radius=0.020, length=0.035),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=satin_metal,
        name="motor_shaft_boss",
    )
    for angle in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4):
        r_mid = 0.140
        housing.visual(
            Cylinder(radius=0.0048, length=0.180),
            origin=Origin(
                xyz=(r_mid * math.cos(angle), 0.026, r_mid * math.sin(angle)),
                rpy=(0.0, math.pi / 2 - angle, 0.0),
            ),
            material=dark_gray,
            name=f"motor_strut_{int(math.degrees(angle))}",
        )

    # Small feet molded into the bottom rail.
    housing.visual(Box((0.095, 0.070, 0.026)), origin=Origin(xyz=(-0.095, 0.010, -0.214)), material=dark_gray, name="foot_0")
    housing.visual(Box((0.095, 0.070, 0.026)), origin=Origin(xyz=(0.095, 0.010, -0.214)), material=dark_gray, name="foot_1")

    # Socket blocks for the folding handle and for the front grille side hinge.
    for x in (-0.160, 0.160):
        housing.visual(Box((0.040, 0.028, 0.006)), origin=Origin(xyz=(x, -0.004, 0.205)), material=warm_white, name=f"handle_socket_{0 if x < 0 else 1}")
    for i, z in enumerate((-0.125, 0.0, 0.125)):
        housing.visual(Box((0.008, 0.028, 0.040)), origin=Origin(xyz=(-0.2055, -0.074, z)), material=warm_white, name=f"grille_hinge_socket_{i}")

    # Printed/tick details around the control knob, fixed to the right face.
    housing.visual(Box((0.0018, 0.044, 0.004)), origin=Origin(xyz=(0.2025, -0.012, 0.142)), material=black, name="speed_tick_high")
    housing.visual(Box((0.0018, 0.030, 0.004)), origin=Origin(xyz=(0.2025, -0.034, 0.105), rpy=(0.0, 0.0, -0.6)), material=black, name="speed_tick_low")
    housing.visual(Box((0.0018, 0.030, 0.004)), origin=Origin(xyz=(0.2025, 0.010, 0.105), rpy=(0.0, 0.0, 0.6)), material=black, name="speed_tick_off")

    blade = model.part("blade")
    rotor = FanRotorGeometry(
        outer_radius=0.134,
        hub_radius=0.034,
        blade_count=5,
        thickness=0.024,
        blade_pitch_deg=32.0,
        blade_sweep_deg=30.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.12, tip_clearance=0.002),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.026, bore_diameter=0.010),
    )
    blade.visual(
        mesh_from_geometry(rotor, "blue_fan_rotor"),
        origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
        material=translucent_blue,
        name="rotor_blades",
    )
    blade.visual(
        Cylinder(radius=0.016, length=0.0275),
        origin=Origin(xyz=(0.0, 0.01375, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=satin_metal,
        name="hub_collar",
    )

    front_grille = model.part("front_grille")
    # The grille part frame is its vertical hinge line; the circular grille extends along +X.
    for radius, tube in ((0.166, 0.0048), (0.122, 0.0034), (0.080, 0.0030), (0.040, 0.0026)):
        ring = TorusGeometry(radius=radius, tube=tube, radial_segments=18, tubular_segments=112).rotate_x(math.pi / 2)
        front_grille.visual(
            mesh_from_geometry(ring, f"front_grille_ring_{radius:.3f}"),
            origin=Origin(xyz=(0.215, 0.0, 0.0)),
            material=warm_white,
            name=f"front_ring_{radius:.3f}",
        )
    for angle in (0.0, math.pi / 6, math.pi / 3, math.pi / 2, 2 * math.pi / 3, 5 * math.pi / 6):
        front_grille.visual(
            Cylinder(radius=0.0026, length=0.340),
            origin=Origin(xyz=(0.215, 0.0, 0.0), rpy=(0.0, math.pi / 2 - angle, 0.0)),
            material=warm_white,
            name=f"front_spoke_{int(math.degrees(angle))}",
        )
    front_grille.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.215, -0.001, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=warm_white,
        name="front_center_badge",
    )
    front_grille.visual(
        Cylinder(radius=0.006, length=0.320),
        origin=Origin(rpy=(0.0, 0.0, 0.0)),
        material=satin_metal,
        name="side_hinge_barrel",
    )
    for i, z in enumerate((-0.075, 0.075)):
        front_grille.visual(Box((0.120, 0.008, 0.008)), origin=Origin(xyz=(0.060, -0.004, z)), material=warm_white, name=f"hinge_bridge_{i}")

    handle = model.part("handle")
    handle_curve = tube_from_spline_points(
        [
            (-0.160, 0.000, 0.000),
            (-0.120, 0.014, 0.010),
            (-0.045, 0.026, 0.018),
            (0.045, 0.026, 0.018),
            (0.120, 0.014, 0.010),
            (0.160, 0.000, 0.000),
        ],
        radius=0.0065,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    handle.visual(mesh_from_geometry(handle_curve, "folding_handle_tube"), material=warm_white, name="handle_tube")
    handle.visual(
        Cylinder(radius=0.0105, length=0.105),
        origin=Origin(xyz=(0.0, 0.026, 0.018), rpy=(0.0, math.pi / 2, 0.0)),
        material=black,
        name="rubber_grip",
    )

    control_knob = model.part("control_knob")
    knob = KnobGeometry(
        0.044,
        0.024,
        body_style="skirted",
        top_diameter=0.034,
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    control_knob.visual(
        mesh_from_geometry(knob, "side_speed_knob"),
        origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_gray,
        name="knob_cap",
    )

    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=80.0),
    )
    model.articulation(
        "housing_to_front_grille",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_grille,
        origin=Origin(xyz=(-0.215, -0.076, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "housing_to_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=handle,
        origin=Origin(xyz=(0.0, -0.004, 0.214)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "housing_to_control_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=control_knob,
        origin=Origin(xyz=(0.204, -0.012, 0.116)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    front_grille = object_model.get_part("front_grille")
    handle = object_model.get_part("handle")
    control_knob = object_model.get_part("control_knob")
    blade_spin = object_model.get_articulation("housing_to_blade")
    grille_hinge = object_model.get_articulation("housing_to_front_grille")
    handle_hinge = object_model.get_articulation("housing_to_handle")
    knob_spin = object_model.get_articulation("housing_to_control_knob")

    ctx.expect_within(
        blade,
        housing,
        axes="x",
        inner_elem="rotor_blades",
        outer_elem="top_rail",
        margin=0.020,
        name="blade fits within housing width",
    )
    ctx.expect_within(
        blade,
        housing,
        axes="z",
        inner_elem="rotor_blades",
        outer_elem="side_rail_0",
        margin=0.020,
        name="blade fits within housing height",
    )
    ctx.expect_gap(
        blade,
        front_grille,
        axis="y",
        min_gap=0.010,
        positive_elem="rotor_blades",
        negative_elem="front_center_badge",
        name="front grille stands ahead of spinning blade",
    )
    ctx.expect_gap(
        control_knob,
        housing,
        axis="x",
        min_gap=-0.002,
        max_gap=0.004,
        positive_elem="knob_cap",
        negative_elem="side_rail_1",
        name="control knob is seated on right face",
    )

    handle_rest_aabb = ctx.part_world_aabb(handle)
    grille_rest_aabb = ctx.part_element_world_aabb(front_grille, elem="front_center_badge")
    with ctx.pose({handle_hinge: 1.20, grille_hinge: 1.20, blade_spin: math.pi, knob_spin: math.pi / 2}):
        handle_lifted_aabb = ctx.part_world_aabb(handle)
        grille_open_aabb = ctx.part_element_world_aabb(front_grille, elem="front_center_badge")
        ctx.expect_gap(
            housing,
            front_grille,
            axis="y",
            min_gap=0.020,
            positive_elem="side_rail_0",
            negative_elem="front_center_badge",
            name="open grille swings out from the front face",
        )

    ctx.check(
        "carry handle lifts above folded pose",
        handle_rest_aabb is not None
        and handle_lifted_aabb is not None
        and handle_lifted_aabb[1][2] > handle_rest_aabb[1][2] + 0.006,
        details=f"rest_aabb={handle_rest_aabb}, lifted_aabb={handle_lifted_aabb}",
    )
    ctx.check(
        "front grille hinge moves the door outward",
        grille_rest_aabb is not None
        and grille_open_aabb is not None
        and ((grille_open_aabb[0][1] + grille_open_aabb[1][1]) * 0.5)
        < ((grille_rest_aabb[0][1] + grille_rest_aabb[1][1]) * 0.5) - 0.020,
        details=f"rest_aabb={grille_rest_aabb}, open_aabb={grille_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
