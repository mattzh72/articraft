from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _envelope_radius(x: float, length: float = 60.0, radius: float = 7.0) -> float:
    """Smooth cigar-envelope radius at longitudinal station x."""
    t = (x + length / 2.0) / length
    t = max(0.0, min(1.0, t))
    return radius * (math.sin(math.pi * t) ** 0.55)


def _make_envelope() -> MeshGeometry:
    profile = []
    for i in range(65):
        t = i / 64.0
        x = -30.0 + 60.0 * t
        r = _envelope_radius(x)
        profile.append((r, x))
    return LatheGeometry(profile, segments=112, closed=True).rotate_y(math.pi / 2.0)


def _prism_from_polygon(
    points: list[tuple[float, float]], thickness: float, *, plane: str
) -> MeshGeometry:
    """Extrude a convex polygon into a thin fin/control-surface prism."""
    geom = MeshGeometry()
    half = thickness / 2.0

    if plane == "xz":
        for y in (-half, half):
            for x, z in points:
                geom.add_vertex(x, y, z)
    elif plane == "xy":
        for z in (-half, half):
            for x, y in points:
                geom.add_vertex(x, y, z)
    else:
        raise ValueError(f"unsupported plane: {plane}")

    n = len(points)
    for i in range(1, n - 1):
        geom.add_face(0, i, i + 1)
        geom.add_face(n, n + i + 1, n + i)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(i, j, n + j)
        geom.add_face(i, n + j, n + i)
    return geom


def _tube(points: list[tuple[float, float, float]], radius: float) -> MeshGeometry:
    return wire_from_points(
        points,
        radius=radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="miter",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_blimp")

    envelope_mat = model.material("warm_white_envelope", rgba=(0.86, 0.84, 0.76, 1.0))
    seam_mat = model.material("muted_seam_gray", rgba=(0.47, 0.48, 0.47, 1.0))
    fin_mat = model.material("tail_fabric", rgba=(0.78, 0.80, 0.76, 1.0))
    gondola_mat = model.material("gondola_silver", rgba=(0.56, 0.60, 0.62, 1.0))
    dark_mat = model.material("dark_mechanical", rgba=(0.05, 0.055, 0.06, 1.0))
    window_mat = model.material("blue_tinted_glass", rgba=(0.10, 0.28, 0.40, 1.0))
    nacelle_mat = model.material("engine_nacelle", rgba=(0.34, 0.36, 0.36, 1.0))
    strut_mat = model.material("painted_struts", rgba=(0.18, 0.20, 0.21, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        mesh_from_geometry(_make_envelope(), "cigar_envelope"),
        material=envelope_mat,
        name="cigar_envelope",
    )

    # Subtle circumferential tapes make the scale and fabric construction legible.
    for x in (-20.0, -10.0, 0.0, 10.0, 20.0):
        band = TorusGeometry(_envelope_radius(x) - 0.030, 0.080, radial_segments=18, tubular_segments=112)
        band.rotate_y(math.pi / 2.0)
        airframe.visual(
            mesh_from_geometry(band, f"envelope_band_{int(x + 30):02d}"),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=seam_mat,
            name=f"envelope_band_{int(x + 30):02d}",
        )

    # Structural gondola hung below the envelope by real suspension struts.
    gondola = CapsuleGeometry(1.22, 5.75, radial_segments=36, height_segments=10)
    gondola.rotate_y(math.pi / 2.0).scale(1.0, 1.0, 0.72)
    airframe.visual(
        mesh_from_geometry(gondola, "gondola_shell"),
        origin=Origin(xyz=(1.0, 0.0, -9.05)),
        material=gondola_mat,
        name="gondola_shell",
    )
    airframe.visual(
        Box((6.2, 0.22, 0.18)),
        origin=Origin(xyz=(1.0, 0.0, -9.98)),
        material=dark_mat,
        name="gondola_keel",
    )
    for side in (-1.0, 1.0):
        side_name = "left" if side > 0.0 else "right"
        for i, x in enumerate((-1.9, -0.7, 0.5, 1.7, 2.9)):
            airframe.visual(
                Box((0.66, 0.090, 0.42)),
                origin=Origin(xyz=(x, side * 1.185, -8.73)),
                material=window_mat,
                name=f"{side_name}_window_{i}",
            )
    airframe.visual(
        Box((0.06, 1.25, 0.46)),
        origin=Origin(xyz=(4.95, 0.0, -8.72)),
        material=window_mat,
        name="front_windscreen",
    )

    for x in (-2.7, 0.8, 3.7):
        for y in (-0.82, 0.82):
            airframe.visual(
                mesh_from_geometry(_tube([(x, y, -6.55), (x + 0.15, y * 0.82, -8.52)], 0.055), f"gondola_strut_{x}_{y}"),
                material=strut_mat,
                name=f"gondola_strut_{x}_{y}",
            )
    for side in (-1.0, 1.0):
        airframe.visual(
            mesh_from_geometry(
                _tube([(-3.1, side * 0.72, -6.45), (1.0, side * 1.02, -8.50), (4.0, side * 0.72, -6.45)], 0.038),
                f"gondola_diagonal_{side}",
            ),
            material=strut_mat,
            name=f"gondola_diagonal_{side}",
        )

    # Twin side engine nacelles and triangulated support pylons.
    nacelle_base = CapsuleGeometry(0.72, 2.90, radial_segments=36, height_segments=8)
    nacelle_base.rotate_y(math.pi / 2.0).scale(1.0, 1.0, 0.78)
    for side in (-1.0, 1.0):
        side_name = "left" if side > 0.0 else "right"
        y = side * 8.25
        z = -5.45
        airframe.visual(
            mesh_from_geometry(nacelle_base.clone(), f"{side_name}_nacelle"),
            origin=Origin(xyz=(0.8, y, z)),
            material=nacelle_mat,
            name=f"{side_name}_nacelle",
        )
        airframe.visual(
            Cylinder(radius=0.30, length=0.15),
            origin=Origin(xyz=(3.02, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_mat,
            name=f"{side_name}_nose_collar",
        )
        for x in (-0.55, 1.85):
            airframe.visual(
                mesh_from_geometry(
                    _tube([(x, side * 5.55, -4.20), (x + 0.25, y, z + 0.38)], 0.070),
                    f"{side_name}_upper_pylon_{x}",
                ),
                material=strut_mat,
                name=f"{side_name}_upper_pylon_{x}",
            )
        airframe.visual(
            mesh_from_geometry(
                _tube([(-0.35, side * 6.05, -5.05), (0.95, y, z + 0.38), (2.2, side * 6.05, -5.05)], 0.048),
                f"{side_name}_nacelle_truss",
            ),
            material=strut_mat,
            name=f"{side_name}_nacelle_truss",
        )

    # Cruciform tail: fixed roots are part of the airframe; control panels are children.
    vertical_fin = _prism_from_polygon(
        [(-27.55, 2.35), (-22.35, 3.75), (-23.70, 9.10), (-27.55, 7.35)],
        0.28,
        plane="xz",
    )
    airframe.visual(
        mesh_from_geometry(vertical_fin, "vertical_stabilizer"),
        material=fin_mat,
        name="vertical_stabilizer",
    )
    lower_fin = _prism_from_polygon(
        [(-27.10, -2.35), (-22.65, -3.65), (-23.95, -7.35), (-27.15, -6.20)],
        0.26,
        plane="xz",
    )
    airframe.visual(
        mesh_from_geometry(lower_fin, "ventral_stabilizer"),
        material=fin_mat,
        name="ventral_stabilizer",
    )
    for side in (-1.0, 1.0):
        side_name = "left" if side > 0.0 else "right"
        fixed_panel = _prism_from_polygon(
            [
                (-22.65, side * 1.15),
                (-27.55, side * 1.20),
                (-27.55, side * 7.95),
                (-23.50, side * 7.00),
            ],
            0.24,
            plane="xy",
        )
        airframe.visual(
            mesh_from_geometry(fixed_panel, f"{side_name}_stabilizer_root"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=fin_mat,
            name=f"{side_name}_stabilizer_root",
        )

    # Articulated propellers.
    rotor_geom = FanRotorGeometry(
        1.18,
        0.23,
        5,
        thickness=0.14,
        blade_pitch_deg=32.0,
        blade_sweep_deg=27.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.12),
        hub=FanRotorHub(style="spinner", bore_diameter=0.08),
    )
    for side in (-1.0, 1.0):
        side_name = "left" if side > 0.0 else "right"
        prop = model.part(f"{side_name}_propeller")
        prop.visual(
            mesh_from_geometry(rotor_geom, f"{side_name}_propeller_rotor"),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_mat,
            name="rotor",
        )
        prop.visual(
            Cylinder(radius=0.115, length=0.58),
            origin=Origin(xyz=(-0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_mat,
            name="shaft",
        )
        model.articulation(
            f"airframe_to_{side_name}_propeller",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=prop,
            origin=Origin(xyz=(3.55, side * 8.25, -5.45)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=95.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_geometry(
            _prism_from_polygon([(-0.07, -2.15), (-2.35, -1.95), (-2.55, 2.55), (-0.07, 2.75)], 0.18, plane="xz"),
            "rudder_panel",
        ),
        material=fin_mat,
        name="rudder_panel",
    )
    rudder.visual(
        Cylinder(radius=0.095, length=5.85),
        origin=Origin(),
        material=dark_mat,
        name="rudder_hinge_sleeve",
    )
    model.articulation(
        "airframe_to_rudder",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=rudder,
        origin=Origin(xyz=(-27.55, 0.0, 5.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=1.6, lower=-0.52, upper=0.52),
    )

    for side in (-1.0, 1.0):
        side_name = "left" if side > 0.0 else "right"
        elevator = model.part(f"{side_name}_elevator")
        elevator.visual(
            mesh_from_geometry(
                _prism_from_polygon([(-0.07, -2.80), (-2.50, -2.55), (-2.75, 2.55), (-0.07, 2.80)], 0.20, plane="xy"),
                f"{side_name}_elevator_panel",
            ),
            material=fin_mat,
            name="elevator_panel",
        )
        elevator.visual(
            Cylinder(radius=0.085, length=5.55),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_mat,
            name="elevator_hinge_sleeve",
        )
        model.articulation(
            f"airframe_to_{side_name}_elevator",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=elevator,
            origin=Origin(xyz=(-27.55, side * 5.20, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=650.0, velocity=1.8, lower=-0.48, upper=0.48),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")

    # Bearings and hinges are deliberately represented as captured sleeves/pins
    # so the movable surface is physically supported instead of floating.
    for side_name, propeller in (("left", left_propeller), ("right", right_propeller)):
        ctx.allow_overlap(
            airframe,
            propeller,
            elem_a=f"{side_name}_nose_collar",
            elem_b="shaft",
            reason="The propeller shaft is intentionally seated a few millimeters inside the nacelle nose bearing.",
        )
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            positive_elem="shaft",
            negative_elem=f"{side_name}_nose_collar",
            max_penetration=0.020,
            max_gap=0.002,
            name=f"{side_name} propeller shaft is captured in the nacelle bearing",
        )

    ctx.allow_overlap(
        airframe,
        rudder,
        elem_a="vertical_stabilizer",
        elem_b="rudder_hinge_sleeve",
        reason="The rudder hinge sleeve is embedded in the vertical stabilizer trailing-edge hinge line.",
    )
    ctx.expect_gap(
        airframe,
        rudder,
        axis="x",
        positive_elem="vertical_stabilizer",
        negative_elem="rudder_hinge_sleeve",
        max_penetration=0.110,
        max_gap=0.002,
        name="rudder hinge sleeve is seated in the tail hinge line",
    )
    ctx.expect_overlap(
        rudder,
        airframe,
        axes="z",
        elem_a="rudder_hinge_sleeve",
        elem_b="vertical_stabilizer",
        min_overlap=4.0,
        name="rudder hinge has a long vertical bearing span",
    )

    for side_name, elevator in (("left", left_elevator), ("right", right_elevator)):
        ctx.allow_overlap(
            airframe,
            elevator,
            elem_a=f"{side_name}_stabilizer_root",
            elem_b="elevator_hinge_sleeve",
            reason="The elevator hinge sleeve is captured along the rear stabilizer hinge line.",
        )
        ctx.expect_gap(
            airframe,
            elevator,
            axis="x",
            positive_elem=f"{side_name}_stabilizer_root",
            negative_elem="elevator_hinge_sleeve",
            max_penetration=0.100,
            max_gap=0.002,
            name=f"{side_name} elevator hinge sleeve is seated in the stabilizer",
        )
        ctx.expect_overlap(
            elevator,
            airframe,
            axes="y",
            elem_a="elevator_hinge_sleeve",
            elem_b=f"{side_name}_stabilizer_root",
            min_overlap=4.5,
            name=f"{side_name} elevator hinge has broad horizontal bearing span",
        )

    left_prop_joint = object_model.get_articulation("airframe_to_left_propeller")
    right_prop_joint = object_model.get_articulation("airframe_to_right_propeller")
    ctx.check(
        "propellers use continuous axle joints",
        left_prop_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_prop_joint.articulation_type == ArticulationType.CONTINUOUS
        and left_prop_joint.axis == (1.0, 0.0, 0.0)
        and right_prop_joint.axis == (1.0, 0.0, 0.0),
        details=f"left={left_prop_joint.articulation_type}/{left_prop_joint.axis}, right={right_prop_joint.articulation_type}/{right_prop_joint.axis}",
    )

    left_elevator_joint = object_model.get_articulation("airframe_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("airframe_to_right_elevator")
    rudder_joint = object_model.get_articulation("airframe_to_rudder")
    rest_left = ctx.part_world_aabb(left_elevator)
    rest_right = ctx.part_world_aabb(right_elevator)
    with ctx.pose({left_elevator_joint: 0.35, right_elevator_joint: 0.35, rudder_joint: 0.35}):
        raised_left = ctx.part_world_aabb(left_elevator)
        raised_right = ctx.part_world_aabb(right_elevator)
        deflected_rudder = ctx.part_world_aabb(rudder)

    rest_rudder = ctx.part_world_aabb(rudder)
    ctx.check(
        "elevator upper deflection raises trailing panels",
        rest_left is not None
        and rest_right is not None
        and raised_left is not None
        and raised_right is not None
        and raised_left[1][2] > rest_left[1][2] + 0.35
        and raised_right[1][2] > rest_right[1][2] + 0.35,
        details=f"left rest={rest_left}, left raised={raised_left}, right rest={rest_right}, right raised={raised_right}",
    )
    ctx.check(
        "rudder deflection swings laterally",
        rest_rudder is not None
        and deflected_rudder is not None
        and (deflected_rudder[1][1] - deflected_rudder[0][1]) > (rest_rudder[1][1] - rest_rudder[0][1]) + 0.6,
        details=f"rest={rest_rudder}, deflected={deflected_rudder}",
    )

    return ctx.report()


object_model = build_object_model()
