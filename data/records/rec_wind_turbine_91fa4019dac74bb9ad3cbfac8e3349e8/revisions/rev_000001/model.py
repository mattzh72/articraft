from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(math.tau * i / segments), radius * math.sin(math.tau * i / segments))
        for i in range(segments)
    ]


def _annular_disc_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 64,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments),
        [list(reversed(_circle_profile(inner_radius, segments)))],
        length,
        center=True,
    )


def _airfoil_section(
    *,
    span_z: float,
    chord: float,
    thickness: float,
    center_x: float,
    twist: float,
) -> list[tuple[float, float, float]]:
    """Eight-point calibration blade section in the local XY plane."""
    base = [
        (-0.50 * chord, -0.10 * thickness),
        (-0.24 * chord, -0.54 * thickness),
        (0.12 * chord, -0.48 * thickness),
        (0.50 * chord, -0.14 * thickness),
        (0.54 * chord, 0.08 * thickness),
        (0.18 * chord, 0.54 * thickness),
        (-0.18 * chord, 0.50 * thickness),
        (-0.48 * chord, 0.18 * thickness),
    ]
    cos_t = math.cos(twist)
    sin_t = math.sin(twist)
    return [
        (center_x + x * cos_t - y * sin_t, x * sin_t + y * cos_t, span_z)
        for x, y in base
    ]


def _blade_shell_mesh():
    sections = [
        _airfoil_section(span_z=0.035, chord=0.120, thickness=0.026, center_x=0.010, twist=math.radians(2.0)),
        _airfoil_section(span_z=0.250, chord=0.095, thickness=0.019, center_x=-0.002, twist=math.radians(-6.0)),
        _airfoil_section(span_z=0.500, chord=0.064, thickness=0.013, center_x=-0.018, twist=math.radians(-13.0)),
        _airfoil_section(span_z=0.725, chord=0.036, thickness=0.008, center_x=-0.030, twist=math.radians(-20.0)),
    ]
    return repair_loft(section_loft(sections), repair="mesh")


def _radial_pose(theta: float) -> Origin:
    return Origin(rpy=(-theta, 0.0, 0.0))


def _radial_xyz(radius: float, theta: float, *, x: float = 0.0) -> tuple[float, float, float]:
    return (x, radius * math.sin(theta), radius * math.cos(theta))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_wind_turbine")

    matte_white = model.material("matte_white", rgba=(0.88, 0.90, 0.91, 1.0))
    tower_paint = model.material("tower_paint", rgba=(0.72, 0.76, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.12, 0.13, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.48, 0.52, 0.55, 1.0))
    datum_blue = model.material("datum_blue", rgba=(0.05, 0.22, 0.55, 1.0))
    mark_black = model.material("mark_black", rgba=(0.01, 0.012, 0.014, 1.0))
    brass = model.material("brass_adjusters", rgba=(0.86, 0.65, 0.28, 1.0))

    tower = model.part("tower")
    tower.visual(Cylinder(radius=0.170, length=0.050), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=dark_steel, name="base_plate")
    tower.visual(Cylinder(radius=0.045, length=1.520), origin=Origin(xyz=(0.0, 0.0, 0.800)), material=tower_paint, name="tower_tube")
    tower.visual(Cylinder(radius=0.110, length=0.050), origin=Origin(xyz=(0.0, 0.0, 1.585)), material=satin_steel, name="yaw_lower_ring")
    tower.visual(Box((0.018, 0.008, 1.220)), origin=Origin(xyz=(0.0, 0.048, 0.820)), material=datum_blue, name="vertical_datum_strip")
    tower.visual(Box((0.070, 0.010, 0.006)), origin=Origin(xyz=(0.0, 0.110, 1.612)), material=mark_black, name="zero_yaw_index")
    for index, (x, y) in enumerate(((0.115, 0.115), (-0.115, 0.115), (-0.115, -0.115), (0.115, -0.115))):
        tower.visual(
            Cylinder(radius=0.014, length=0.022),
            origin=Origin(xyz=(x, y, 0.053)),
            material=brass,
            name=f"leveling_pad_{index}",
        )

    nacelle = model.part("nacelle")
    nacelle.visual(Cylinder(radius=0.100, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=satin_steel, name="yaw_upper_ring")
    nacelle.visual(Cylinder(radius=0.055, length=0.075), origin=Origin(xyz=(0.0, 0.0, 0.060)), material=tower_paint, name="yaw_pedestal")
    nacelle.visual(Box((0.550, 0.180, 0.160)), origin=Origin(xyz=(0.230, 0.0, 0.150)), material=matte_white, name="nacelle_body")
    nacelle.visual(Box((0.225, 0.126, 0.010)), origin=Origin(xyz=(0.180, 0.0, 0.235)), material=datum_blue, name="roof_datum_pad")
    nacelle.visual(
        mesh_from_geometry(_annular_disc_mesh(outer_radius=0.075, inner_radius=0.033, length=0.080), "front_bearing_ring"),
        origin=Origin(xyz=(0.545, 0.0, 0.150), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="front_bearing",
    )
    nacelle.visual(Box((0.006, 0.008, 0.034)), origin=Origin(xyz=(0.588, 0.0, 0.221)), material=mark_black, name="bearing_zero_tick")
    nacelle.visual(Box((0.006, 0.034, 0.008)), origin=Origin(xyz=(0.588, 0.071, 0.150)), material=mark_black, name="bearing_side_tick")
    nacelle.visual(Box((0.038, 0.012, 0.012)), origin=Origin(xyz=(0.200, 0.096, 0.150)), material=brass, name="side_adjuster_0")
    nacelle.visual(Box((0.038, 0.012, 0.012)), origin=Origin(xyz=(0.310, -0.096, 0.150)), material=brass, name="side_adjuster_1")

    rotor_hub = model.part("rotor_hub")
    rotor_hub.visual(Cylinder(radius=0.026, length=0.090), origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_steel, name="main_shaft")
    rotor_hub.visual(Cylinder(radius=0.050, length=0.025), origin=Origin(xyz=(0.0125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=satin_steel, name="front_collar")
    rotor_hub.visual(Cylinder(radius=0.090, length=0.120), origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_steel, name="hub_drum")
    rotor_hub.visual(Cylinder(radius=0.032, length=0.040), origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=datum_blue, name="nose_datum")

    blade_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, theta in enumerate(blade_angles):
        rotor_hub.visual(
            Cylinder(radius=0.040, length=0.130),
            origin=Origin(xyz=_radial_xyz(0.095, theta, x=0.080), rpy=(-theta, 0.0, 0.0)),
            material=satin_steel,
            name=f"pitch_socket_{index}",
        )
        rotor_hub.visual(
            Box((0.032, 0.006, 0.010)),
            origin=Origin(xyz=_radial_xyz(0.133, theta, x=0.120), rpy=(-theta, 0.0, 0.0)),
            material=mark_black,
            name=f"root_index_{index}",
        )

    blade_mesh = mesh_from_geometry(_blade_shell_mesh(), "calibration_blade_shell")
    for index, theta in enumerate(blade_angles):
        root = model.part(f"blade_root_{index}")
        root.visual(Cylinder(radius=0.024, length=0.060), origin=Origin(xyz=(0.0, 0.0, -0.025)), material=dark_steel, name="pitch_journal")
        root.visual(Cylinder(radius=0.052, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=satin_steel, name="root_flange")
        root.visual(Box((0.086, 0.018, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.055)), material=datum_blue, name="root_datum_flat")
        root.visual(blade_mesh, material=matte_white, name="blade_shell")
        root.visual(Box((0.010, 0.070, 0.006)), origin=Origin(xyz=(0.0, 0.0, 0.042)), material=mark_black, name="pitch_zero_line")
        for bolt_index, bolt_angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
            root.visual(
                Cylinder(radius=0.005, length=0.012),
                origin=Origin(xyz=(0.033 * math.cos(bolt_angle), 0.033 * math.sin(bolt_angle), 0.045)),
                material=dark_steel,
                name=f"flange_bolt_{bolt_index}",
            )
        model.articulation(
            f"hub_to_blade_root_{index}",
            ArticulationType.REVOLUTE,
            parent=rotor_hub,
            child=root,
            origin=Origin(xyz=_radial_xyz(0.160, theta, x=0.080), rpy=(-theta, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.45, lower=math.radians(-10.0), upper=math.radians(22.0)),
            motion_properties=MotionProperties(damping=0.08, friction=0.03),
        )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 1.610)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=math.radians(-180.0), upper=math.radians(180.0)),
        motion_properties=MotionProperties(damping=0.20, friction=0.10),
    )
    model.articulation(
        "nacelle_to_rotor_hub",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor_hub,
        origin=Origin(xyz=(0.585, 0.0, 0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    hub = object_model.get_part("rotor_hub")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor_hub")

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        positive_elem="yaw_upper_ring",
        negative_elem="yaw_lower_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="controlled yaw bearing stack gap",
    )
    ctx.expect_gap(
        hub,
        nacelle,
        axis="x",
        positive_elem="front_collar",
        negative_elem="front_bearing",
        max_gap=0.001,
        max_penetration=0.0005,
        name="rotor collar seated at bearing face",
    )
    ctx.allow_overlap(
        nacelle,
        hub,
        elem_a="front_bearing",
        elem_b="main_shaft",
        reason="The shaft is intentionally shown captured through the simplified bearing ring for the rotor load path.",
    )
    ctx.expect_within(
        hub,
        nacelle,
        axes="yz",
        inner_elem="main_shaft",
        outer_elem="front_bearing",
        margin=0.002,
        name="shaft centered through bearing datum",
    )
    ctx.expect_overlap(
        nacelle,
        hub,
        axes="x",
        elem_a="front_bearing",
        elem_b="main_shaft",
        min_overlap=0.070,
        name="shaft retained through bearing depth",
    )

    for index in range(3):
        root = object_model.get_part(f"blade_root_{index}")
        pitch = object_model.get_articulation(f"hub_to_blade_root_{index}")
        ctx.allow_overlap(
            hub,
            root,
            elem_a=f"pitch_socket_{index}",
            elem_b="pitch_journal",
            reason="The pitch journal is intentionally captured inside the hub socket as the load path for blade-root calibration.",
        )
        ctx.expect_overlap(
            hub,
            root,
            axes="xyz",
            elem_a=f"pitch_socket_{index}",
            elem_b="pitch_journal",
            min_overlap=0.010,
            name=f"blade root {index} retained in hub socket",
        )
        limits = pitch.motion_limits
        ctx.check(
            f"blade root {index} pitch has calibration travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0
            and limits.upper > math.radians(15.0),
            details=f"limits={limits}",
        )

    rest_root = ctx.part_world_position(object_model.get_part("blade_root_0"))
    with ctx.pose({spin: math.pi / 2.0}):
        spun_root = ctx.part_world_position(object_model.get_part("blade_root_0"))
    ctx.check(
        "rotor spin carries blade roots about shaft",
        rest_root is not None
        and spun_root is not None
        and abs(spun_root[1] - rest_root[1]) > 0.05
        and abs(spun_root[2] - rest_root[2]) > 0.05,
        details=f"rest={rest_root}, spun={spun_root}",
    )

    rest_hub = ctx.part_world_position(hub)
    with ctx.pose({yaw: math.radians(30.0)}):
        yawed_hub = ctx.part_world_position(hub)
    ctx.check(
        "yaw joint slews nacelle and rotor together",
        rest_hub is not None
        and yawed_hub is not None
        and abs(yawed_hub[1] - rest_hub[1]) > 0.05,
        details=f"rest={rest_hub}, yawed={yawed_hub}",
    )

    return ctx.report()


object_model = build_object_model()
