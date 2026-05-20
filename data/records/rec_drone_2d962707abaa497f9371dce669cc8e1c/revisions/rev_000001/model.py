from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_duct(outer_radius: float, inner_radius: float, depth: float) -> cq.Workplane:
    """Stationary hollow Kort-nozzle style thruster duct, local axis along +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(depth)
        .translate((0.0, 0.0, -0.5 * depth))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underwater_rov_drone")

    frame_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.08, 1.0))
    duct_black = model.material("matte_black", rgba=(0.01, 0.012, 0.015, 1.0))
    prop_gray = model.material("propeller_graphite", rgba=(0.11, 0.12, 0.13, 1.0))
    hull_blue = model.material("deep_blue_hull", rgba=(0.05, 0.18, 0.34, 1.0))
    foam_white = model.material("buoyancy_foam", rgba=(0.92, 0.94, 0.88, 1.0))
    glass = model.material("camera_glass", rgba=(0.02, 0.08, 0.11, 0.75))
    lens_blue = model.material("lens_blue", rgba=(0.0, 0.20, 0.42, 0.85))

    duct_mesh = mesh_from_cadquery(
        _annular_duct(outer_radius=0.085, inner_radius=0.071, depth=0.060),
        "thruster_duct",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            outer_radius=0.061,
            hub_radius=0.018,
            blade_count=5,
            thickness=0.014,
            blade_pitch_deg=32.0,
            blade_sweep_deg=24.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
            hub=FanRotorHub(style="spinner", bore_diameter=0.004),
        ),
        "thruster_propeller",
    )

    frame = model.part("frame")

    # Rectangular cage body: overlapping box rails make one rigid, connected frame.
    rail_t = 0.035
    for z in (-0.12, 0.12):
        for y in (-0.235, 0.235):
            frame.visual(
                Box((0.78, rail_t, rail_t)),
                origin=Origin(xyz=(0.0, y, z)),
                material=frame_yellow,
                name=f"long_rail_{z}_{y}",
            )
        for x in (-0.335, 0.335):
            frame.visual(
                Box((rail_t, 0.52, rail_t)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=frame_yellow,
                name=f"cross_rail_{z}_{x}",
            )

    for y in (-0.235, 0.235):
        frame.visual(
            Box((0.78, rail_t, rail_t)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=frame_yellow,
            name=f"mid_side_rail_{y}",
        )
    for x in (-0.335, 0.335):
        frame.visual(
            Box((rail_t, 0.52, rail_t)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=frame_yellow,
            name=f"mid_cross_rail_{x}",
        )

    for x in (-0.335, 0.335):
        for y in (-0.235, 0.235):
            frame.visual(
                Box((rail_t, rail_t, 0.275)),
                origin=Origin(xyz=(x, y, 0.0)),
                material=frame_yellow,
                name=f"corner_post_{x}_{y}",
            )

    # Internal electronics pressure canister and top buoyancy blocks.
    frame.visual(
        Cylinder(radius=0.082, length=0.46),
        origin=Origin(xyz=(-0.02, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hull_blue,
        name="pressure_hull",
    )
    for x in (-0.235, 0.175):
        frame.visual(
            Box((0.040, 0.50, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=frame_yellow,
            name=f"hull_saddle_{x}",
        )
    for y in (-0.115, 0.115):
        frame.visual(
            Box((0.56, 0.075, 0.070)),
            origin=Origin(xyz=(-0.035, y, 0.165)),
            material=foam_white,
            name=f"buoyancy_block_{y}",
        )
        frame.visual(
            Box((0.050, 0.035, 0.065)),
            origin=Origin(xyz=(-0.260, y, 0.122)),
            material=frame_yellow,
            name=f"float_clamp_front_{y}",
        )
        frame.visual(
            Box((0.050, 0.035, 0.065)),
            origin=Origin(xyz=(0.185, y, 0.122)),
            material=frame_yellow,
            name=f"float_clamp_rear_{y}",
        )
    for x in (-0.260, 0.185):
        frame.visual(
            Box((0.052, 0.50, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.122)),
            material=frame_yellow,
            name=f"float_cross_strap_{x}",
        )

    # Four ducted thruster nozzles in a cross around the rectangular frame.
    thrusters = {
        "front": {
            "center": (0.405, 0.0, -0.120),
            "axis": "x",
            "outward": 1.0,
            "duct_visual": "front_duct",
            "bearing_visual": "front_bearing",
        },
        "rear": {
            "center": (-0.405, 0.0, -0.120),
            "axis": "x",
            "outward": -1.0,
            "duct_visual": "rear_duct",
            "bearing_visual": "rear_bearing",
        },
        "port": {
            "center": (0.0, 0.340, 0.000),
            "axis": "y",
            "outward": 1.0,
            "duct_visual": "port_duct",
            "bearing_visual": "port_bearing",
        },
        "starboard": {
            "center": (0.0, -0.340, 0.000),
            "axis": "y",
            "outward": -1.0,
            "duct_visual": "starboard_duct",
            "bearing_visual": "starboard_bearing",
        },
    }

    for label, spec in thrusters.items():
        cx, cy, cz = spec["center"]
        if spec["axis"] == "x":
            duct_rpy = (0.0, pi / 2.0, 0.0)
            grille_x = cx + spec["outward"] * 0.031
            frame.visual(
                duct_mesh,
                origin=Origin(xyz=(cx, cy, cz), rpy=duct_rpy),
                material=duct_black,
                name=spec["duct_visual"],
            )
            for y in (-0.155, 0.155):
                frame.visual(
                    Box((0.060, 0.185, 0.026)),
                    origin=Origin(xyz=(cx - spec["outward"] * 0.055, y, cz)),
                    material=frame_yellow,
                    name=f"{label}_mount_arm_{y}",
                )
            frame.visual(
                Box((0.006, 0.148, 0.007)),
                origin=Origin(xyz=(grille_x, cy, cz)),
                material=duct_black,
                name=f"{label}_grille_bar_y",
            )
            frame.visual(
                Box((0.006, 0.007, 0.148)),
                origin=Origin(xyz=(grille_x, cy, cz)),
                material=duct_black,
                name=f"{label}_grille_bar_z",
            )
            bearing_x = cx - spec["outward"] * 0.020
            frame.visual(
                Cylinder(radius=0.010, length=0.016),
                origin=Origin(xyz=(bearing_x, cy, cz), rpy=duct_rpy),
                material=prop_gray,
                name=spec["bearing_visual"],
            )
            for y_offset in (-0.047, 0.047):
                frame.visual(
                    Box((0.008, 0.076, 0.008)),
                    origin=Origin(xyz=(bearing_x, y_offset, cz)),
                    material=duct_black,
                    name=f"{label}_spider_y_{y_offset}",
                )
            for z_offset in (-0.047, 0.047):
                frame.visual(
                    Box((0.008, 0.008, 0.076)),
                    origin=Origin(xyz=(bearing_x, cy, cz + z_offset)),
                    material=duct_black,
                    name=f"{label}_spider_z_{z_offset}",
                )
        else:
            duct_rpy = (-pi / 2.0, 0.0, 0.0)
            grille_y = cy + spec["outward"] * 0.031
            frame.visual(
                duct_mesh,
                origin=Origin(xyz=(cx, cy, cz), rpy=duct_rpy),
                material=duct_black,
                name=spec["duct_visual"],
            )
            for x in (-0.075, 0.075):
                frame.visual(
                    Box((0.026, 0.115, 0.024)),
                    origin=Origin(xyz=(x, cy - spec["outward"] * 0.055, cz)),
                    material=frame_yellow,
                    name=f"{label}_mount_arm_{x}",
                )
            frame.visual(
                Box((0.148, 0.006, 0.007)),
                origin=Origin(xyz=(cx, grille_y, cz)),
                material=duct_black,
                name=f"{label}_grille_bar_x",
            )
            frame.visual(
                Box((0.007, 0.006, 0.148)),
                origin=Origin(xyz=(cx, grille_y, cz)),
                material=duct_black,
                name=f"{label}_grille_bar_z",
            )
            bearing_y = cy - spec["outward"] * 0.020
            frame.visual(
                Cylinder(radius=0.010, length=0.016),
                origin=Origin(xyz=(cx, bearing_y, cz), rpy=duct_rpy),
                material=prop_gray,
                name=spec["bearing_visual"],
            )
            for x_offset in (-0.047, 0.047):
                frame.visual(
                    Box((0.076, 0.008, 0.008)),
                    origin=Origin(xyz=(x_offset, bearing_y, cz)),
                    material=duct_black,
                    name=f"{label}_spider_x_{x_offset}",
                )
            for z_offset in (-0.047, 0.047):
                frame.visual(
                    Box((0.008, 0.008, 0.076)),
                    origin=Origin(xyz=(cx, bearing_y, cz + z_offset)),
                    material=duct_black,
                    name=f"{label}_spider_z_{z_offset}",
                )

    # Nose yoke for the tilting camera, integrated into the frame.
    frame.visual(
        Box((0.165, 0.155, 0.025)),
        origin=Origin(xyz=(0.405, 0.0, 0.015)),
        material=frame_yellow,
        name="camera_base",
    )
    frame.visual(
        Box((0.145, 0.040, 0.040)),
        origin=Origin(xyz=(0.375, 0.0, 0.046)),
        material=frame_yellow,
        name="nose_boom",
    )
    for y in (-0.075, 0.075):
        frame.visual(
            Box((0.048, 0.018, 0.095)),
            origin=Origin(xyz=(0.462, y, 0.075)),
            material=frame_yellow,
            name=f"camera_cheek_{y}",
        )

    # Rotating thruster propellers.  The mesh spins about its local +Z axis, so
    # each visual is oriented to match the duct's own thrust axis.
    propeller_specs = {
        "front_propeller": {
            "origin": (0.405, 0.0, -0.120),
            "axis": (1.0, 0.0, 0.0),
            "visual_rpy": (0.0, pi / 2.0, 0.0),
        },
        "rear_propeller": {
            "origin": (-0.405, 0.0, -0.120),
            "axis": (-1.0, 0.0, 0.0),
            "visual_rpy": (0.0, pi / 2.0, 0.0),
        },
        "port_propeller": {
            "origin": (0.0, 0.340, 0.000),
            "axis": (0.0, 1.0, 0.0),
            "visual_rpy": (-pi / 2.0, 0.0, 0.0),
        },
        "starboard_propeller": {
            "origin": (0.0, -0.340, 0.000),
            "axis": (0.0, -1.0, 0.0),
            "visual_rpy": (-pi / 2.0, 0.0, 0.0),
        },
    }

    for part_name, spec in propeller_specs.items():
        prop = model.part(part_name)
        prop.visual(
            rotor_mesh,
            origin=Origin(rpy=spec["visual_rpy"]),
            material=prop_gray,
            name="rotor",
        )
        prop.visual(
            Cylinder(radius=0.006, length=0.052),
            origin=Origin(rpy=spec["visual_rpy"]),
            material=prop_gray,
            name="shaft",
        )
        model.articulation(
            f"frame_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=prop,
            origin=Origin(xyz=spec["origin"]),
            axis=spec["axis"],
            motion_limits=MotionLimits(effort=1.2, velocity=80.0),
        )

    camera = model.part("camera_pod")
    camera.visual(
        Cylinder(radius=0.040, length=0.078),
        origin=Origin(xyz=(0.043, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=duct_black,
        name="pod_shell",
    )
    camera.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_yellow,
        name="rear_gimbal_lug",
    )
    camera.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    camera.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.091, 0.0, 0.0)),
        material=lens_blue,
        name="lens_glint",
    )
    camera.visual(
        Cylinder(radius=0.012, length=0.132),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=prop_gray,
        name="tilt_trunnion",
    )
    model.articulation(
        "frame_to_camera_pod",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=camera,
        origin=Origin(xyz=(0.462, 0.0, 0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.45, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    propeller_names = (
        "front_propeller",
        "rear_propeller",
        "port_propeller",
        "starboard_propeller",
    )
    for name in propeller_names:
        joint = object_model.get_articulation(f"frame_to_{name}")
        ctx.check(
            f"{name} spins continuously",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{name} joint type is {joint.articulation_type}",
        )

    frame = object_model.get_part("frame")
    camera = object_model.get_part("camera_pod")
    tilt = object_model.get_articulation("frame_to_camera_pod")

    for propeller, bearing, radial_axes, axial_axis in (
        ("front_propeller", "front_bearing", "yz", "x"),
        ("rear_propeller", "rear_bearing", "yz", "x"),
        ("port_propeller", "port_bearing", "xz", "y"),
        ("starboard_propeller", "starboard_bearing", "xz", "y"),
    ):
        ctx.allow_overlap(
            frame,
            propeller,
            elem_a=bearing,
            elem_b="shaft",
            reason="The rotating propeller shaft is intentionally captured inside the stationary bearing boss.",
        )
        ctx.expect_within(
            propeller,
            frame,
            axes=radial_axes,
            inner_elem="shaft",
            outer_elem=bearing,
            margin=0.001,
            name=f"{propeller} shaft is centered in bearing",
        )
        ctx.expect_overlap(
            frame,
            propeller,
            axes=axial_axis,
            elem_a=bearing,
            elem_b="shaft",
            min_overlap=0.010,
            name=f"{propeller} shaft remains inserted in bearing",
        )

    ctx.expect_within(
        "front_propeller",
        frame,
        axes="yz",
        inner_elem="rotor",
        outer_elem="front_duct",
        margin=0.002,
        name="front rotor fits inside duct diameter",
    )
    ctx.expect_within(
        "rear_propeller",
        frame,
        axes="yz",
        inner_elem="rotor",
        outer_elem="rear_duct",
        margin=0.002,
        name="rear rotor fits inside duct diameter",
    )
    ctx.expect_within(
        "port_propeller",
        frame,
        axes="xz",
        inner_elem="rotor",
        outer_elem="port_duct",
        margin=0.002,
        name="port rotor fits inside duct diameter",
    )
    ctx.expect_within(
        "starboard_propeller",
        frame,
        axes="xz",
        inner_elem="rotor",
        outer_elem="starboard_duct",
        margin=0.002,
        name="starboard rotor fits inside duct diameter",
    )

    lens_closed = ctx.part_element_world_aabb(camera, elem="front_lens")
    with ctx.pose({tilt: 0.55}):
        lens_tilted = ctx.part_element_world_aabb(camera, elem="front_lens")
    closed_z = None if lens_closed is None else 0.5 * (lens_closed[0][2] + lens_closed[1][2])
    tilted_z = None if lens_tilted is None else 0.5 * (lens_tilted[0][2] + lens_tilted[1][2])
    ctx.check(
        "camera pod tilts upward",
        closed_z is not None and tilted_z is not None and tilted_z > closed_z + 0.020,
        details=f"closed_lens_z={closed_z}, tilted_lens_z={tilted_z}",
    )

    return ctx.report()


object_model = build_object_model()
