from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _airfoil_section(
    y: float,
    *,
    chord: float,
    thickness_ratio: float,
    z_lift: float,
    samples: int = 10,
) -> list[tuple[float, float, float]]:
    """Return a closed NACA-like airfoil loop in X/Z at a span station."""

    def point(t: float, sign: float) -> tuple[float, float, float]:
        # NACA 00xx thickness distribution.  X is positive forward, so the
        # leading edge is at +0.38*chord and the trailing edge at -0.62*chord.
        yt = (
            5.0
            * thickness_ratio
            * chord
            * (
                0.2969 * math.sqrt(max(t, 0.0))
                - 0.1260 * t
                - 0.3516 * t * t
                + 0.2843 * t * t * t
                - 0.1036 * t * t * t * t
            )
        )
        camber = 0.010 * chord * (1.0 - (2.0 * t - 1.0) ** 2)
        x = 0.38 * chord - t * chord
        # LoftGeometry profiles are checked in XY at constant Z.  Author the
        # airfoil in local X/Y and put the span station in local Z; the full
        # wing mesh is rotated afterward so local Z becomes world Y.
        local_y = z_lift + camber + sign * yt
        local_z = -y
        return (x, local_y, local_z)

    ts = [i / (samples - 1) for i in range(samples)]
    upper = [point(t, 1.0) for t in reversed(ts)]
    lower = [point(t, -1.0) for t in ts[1:-1]]
    return upper + lower


def _build_main_wing_mesh() -> MeshGeometry:
    stations = [
        (-1.25, 0.34, 0.095, 0.045),
        (-0.82, 0.44, 0.105, 0.025),
        (0.00, 0.58, 0.115, 0.000),
        (0.82, 0.44, 0.105, 0.025),
        (1.25, 0.34, 0.095, 0.045),
    ]
    sections = [
        _airfoil_section(y, chord=chord, thickness_ratio=thick, z_lift=z)
        for y, chord, thick, z in stations
    ]
    return LoftGeometry(sections, cap=True, closed=True).rotate_x(math.pi / 2.0)


def _build_tail_fin_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    half_width = 0.010
    vertices = [
        (-1.17, -half_width, 0.045),
        (-0.97, -half_width, 0.045),
        (-1.10, -half_width, 0.265),
        (-1.17, half_width, 0.045),
        (-0.97, half_width, 0.045),
        (-1.10, half_width, 0.265),
    ]
    for vertex in vertices:
        geom.add_vertex(*vertex)
    for face in (
        (0, 1, 2),
        (3, 5, 4),
        (0, 3, 4),
        (0, 4, 1),
        (1, 4, 5),
        (1, 5, 2),
        (2, 5, 3),
        (2, 3, 0),
    ):
        geom.add_face(*face)
    return geom


def _build_nacelle_pod_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, 0.070),
            (0.058, 0.078),
            (0.098, 0.125),
            (0.112, 0.210),
            (0.102, 0.292),
            (0.062, 0.338),
            (0.000, 0.352),
        ],
        segments=48,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixed_wing_vtol_hybrid_drone")

    wing_skin = model.material("matte_white_composite", rgba=(0.86, 0.88, 0.84, 1.0))
    dark_composite = model.material("dark_composite", rgba=(0.055, 0.060, 0.070, 1.0))
    hinge_metal = model.material("anodized_hinge_metal", rgba=(0.35, 0.38, 0.41, 1.0))
    nacelle_paint = model.material("nacelle_white", rgba=(0.78, 0.80, 0.78, 1.0))
    blade_black = model.material("carbon_blade_black", rgba=(0.015, 0.017, 0.020, 1.0))

    wing_body = model.part("wing_body")
    wing_body.visual(
        mesh_from_geometry(_build_main_wing_mesh(), "tapered_airfoil_wing"),
        material=wing_skin,
        name="main_wing",
    )
    fuselage = CapsuleGeometry(radius=0.095, length=0.58, radial_segments=32).rotate_y(
        math.pi / 2.0
    ).translate(-0.020, 0.0, 0.030)
    wing_body.visual(
        mesh_from_geometry(fuselage, "capsule_fuselage"),
        material=wing_skin,
        name="fuselage",
    )
    tail_boom = CylinderGeometry(radius=0.033, height=0.78, radial_segments=28).rotate_y(
        math.pi / 2.0
    ).translate(-0.775, 0.0, 0.038)
    wing_body.visual(
        mesh_from_geometry(tail_boom, "tail_boom"),
        material=dark_composite,
        name="tail_boom",
    )
    wing_body.visual(
        Box((0.17, 0.56, 0.018)),
        origin=Origin(xyz=(-1.095, 0.0, 0.054)),
        material=wing_skin,
        name="horizontal_tail",
    )
    wing_body.visual(
        mesh_from_geometry(_build_tail_fin_mesh(), "vertical_tail_fin"),
        material=wing_skin,
        name="vertical_tail",
    )

    hinge_positions = {"left": 1.31, "right": -1.31}
    for side, y_joint in hinge_positions.items():
        sign = 1.0 if y_joint > 0.0 else -1.0
        wing_body.visual(
            Box((0.080, 0.220, 0.036)),
            origin=Origin(xyz=(-0.070, y_joint, 0.075)),
            material=hinge_metal,
            name=f"{side}_hinge_backstrap",
        )
        for suffix, y_offset in (("inner", -0.065), ("outer", 0.065)):
            lug_y = y_joint + sign * y_offset
            wing_body.visual(
                Cylinder(radius=0.032, length=0.040),
                origin=Origin(xyz=(0.000, lug_y, 0.075), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=hinge_metal,
                name=f"{side}_{suffix}_clevis_barrel",
            )
            wing_body.visual(
                Box((0.078, 0.040, 0.026)),
                origin=Origin(xyz=(-0.043, lug_y, 0.075)),
                material=hinge_metal,
                name=f"{side}_{suffix}_clevis_web",
            )

    rotor_mesh = FanRotorGeometry(
        0.220,
        0.045,
        3,
        thickness=0.018,
        blade_pitch_deg=31.0,
        blade_sweep_deg=26.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.10),
        hub=FanRotorHub(style="spinner", bore_diameter=0.010),
    )

    for side, y_joint in hinge_positions.items():
        nacelle = model.part(f"{side}_nacelle")
        nacelle.visual(
            Cylinder(radius=0.024, length=0.090),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name="pivot_boss",
        )
        nacelle.visual(
            Box((0.060, 0.052, 0.120)),
            origin=Origin(xyz=(0.020, 0.0, 0.060)),
            material=hinge_metal,
            name="tilt_yoke",
        )
        nacelle.visual(
            mesh_from_geometry(_build_nacelle_pod_mesh(), f"{side}_streamlined_nacelle"),
            origin=Origin(xyz=(0.020, 0.0, 0.0)),
            material=nacelle_paint,
            name="motor_pod",
        )
        nacelle.visual(
            Cylinder(radius=0.064, length=0.018),
            origin=Origin(xyz=(0.020, 0.0, 0.344)),
            material=dark_composite,
            name="propeller_mount_face",
        )
        propeller = model.part(f"{side}_propeller")
        propeller.visual(
            mesh_from_geometry(rotor_mesh, f"{side}_three_blade_propeller"),
            material=blade_black,
            name="propeller_disk",
        )

        model.articulation(
            f"wing_to_{side}_nacelle",
            ArticulationType.REVOLUTE,
            parent=wing_body,
            child=nacelle,
            origin=Origin(xyz=(0.020, y_joint, 0.075)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.4,
                lower=0.0,
                upper=math.pi / 2.0,
            ),
        )
        model.articulation(
            f"{side}_nacelle_to_propeller",
            ArticulationType.CONTINUOUS,
            parent=nacelle,
            child=propeller,
            origin=Origin(xyz=(0.020, 0.0, 0.3705)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=220.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for side in ("left", "right"):
        tilt = object_model.get_articulation(f"wing_to_{side}_nacelle")
        spin = object_model.get_articulation(f"{side}_nacelle_to_propeller")
        limits = tilt.motion_limits
        ctx.check(
            f"{side} nacelle uses a spanwise tilt hinge",
            tilt.articulation_type == ArticulationType.REVOLUTE
            and tuple(tilt.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and abs(limits.lower - 0.0) < 1e-6
            and abs(limits.upper - math.pi / 2.0) < 1e-6,
            details=f"type={tilt.articulation_type}, axis={tilt.axis}, limits={limits}",
        )
        ctx.check(
            f"{side} propeller spins about nacelle rotor axis",
            spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(spin.axis) == (0.0, 0.0, 1.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )

        ctx.expect_contact(
            f"{side}_nacelle",
            "wing_body",
            elem_a="pivot_boss",
            elem_b=f"{side}_inner_clevis_barrel",
            contact_tol=0.003,
            name=f"{side} pivot boss seated in inner clevis",
        )
        ctx.expect_contact(
            f"{side}_nacelle",
            "wing_body",
            elem_a="pivot_boss",
            elem_b=f"{side}_outer_clevis_barrel",
            contact_tol=0.003,
            name=f"{side} pivot boss seated in outer clevis",
        )
        ctx.expect_contact(
            f"{side}_propeller",
            f"{side}_nacelle",
            elem_a="propeller_disk",
            elem_b="propeller_mount_face",
            contact_tol=0.003,
            name=f"{side} propeller hub sits on nacelle mount",
        )

        propeller = object_model.get_part(f"{side}_propeller")
        rest_pos = ctx.part_world_position(propeller)
        with ctx.pose({tilt: math.pi / 2.0}):
            forward_pos = ctx.part_world_position(propeller)
        ctx.check(
            f"{side} tilt sends rotor from lift to forward thrust",
            rest_pos is not None
            and forward_pos is not None
            and forward_pos[0] > rest_pos[0] + 0.25
            and forward_pos[2] < rest_pos[2] - 0.25,
            details=f"rest={rest_pos}, forward={forward_pos}",
        )

    return ctx.report()


object_model = build_object_model()
