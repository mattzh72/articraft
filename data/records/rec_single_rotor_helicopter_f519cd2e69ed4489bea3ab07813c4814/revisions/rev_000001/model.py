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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="agricultural_helicopter")

    body_yellow = model.material("body_yellow", rgba=(0.90, 0.76, 0.18, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    skid_steel = model.material("skid_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.10, 0.11, 0.12, 1.0))
    cabin_glass = model.material("cabin_glass", rgba=(0.48, 0.66, 0.72, 0.45))

    airframe = model.part("airframe")

    fuselage_sections = [
        _yz_section(1.20, width=0.16, height=0.18, radius=0.04, z_center=0.52),
        _yz_section(0.86, width=0.64, height=0.76, radius=0.18, z_center=0.74),
        _yz_section(0.34, width=1.06, height=1.00, radius=0.24, z_center=0.89),
        _yz_section(-0.22, width=1.28, height=1.00, radius=0.24, z_center=0.90),
        _yz_section(-0.86, width=1.22, height=0.98, radius=0.20, z_center=0.89),
        _yz_section(-1.38, width=0.96, height=0.78, radius=0.18, z_center=0.84),
        _yz_section(-1.76, width=0.42, height=0.40, radius=0.10, z_center=0.88),
    ]
    airframe.visual(
        _save_mesh("agri_fuselage_shell", section_loft(fuselage_sections)),
        material=body_yellow,
        name="fuselage_shell",
    )
    airframe.visual(
        Box((0.82, 0.96, 0.58)),
        origin=Origin(xyz=(0.40, 0.0, 0.95), rpy=(0.0, math.radians(8.0), 0.0)),
        material=cabin_glass,
        name="canopy",
    )
    airframe.visual(
        Box((0.74, 0.68, 0.18)),
        origin=Origin(xyz=(0.44, 0.0, 0.34)),
        material=body_yellow,
        name="nose_keel",
    )
    airframe.visual(
        Box((0.54, 0.50, 0.22)),
        origin=Origin(xyz=(-0.10, 0.0, 1.49)),
        material=machinery_gray,
        name="mast_fairing",
    )
    airframe.visual(
        Cylinder(radius=0.16, length=0.46),
        origin=Origin(xyz=(-0.04, 0.0, 1.66)),
        material=machinery_gray,
        name="mast_pylon",
    )
    airframe.visual(
        Box((0.90, 0.18, 0.12)),
        origin=Origin(xyz=(-0.82, 0.0, 1.56)),
        material=machinery_gray,
        name="engine_spine",
    )
    airframe.visual(
        Box((0.84, 0.26, 0.04)),
        origin=Origin(xyz=(-0.82, 0.22, 1.495)),
        material=machinery_gray,
        name="left_panel_seat",
    )
    airframe.visual(
        Box((0.84, 0.26, 0.04)),
        origin=Origin(xyz=(-0.82, -0.22, 1.495)),
        material=machinery_gray,
        name="right_panel_seat",
    )
    airframe.visual(
        _save_mesh(
            "tail_boom",
            tube_from_spline_points(
                [
                    (-1.74, 0.0, 0.88),
                    (-2.70, 0.0, 0.96),
                    (-3.70, 0.0, 1.02),
                    (-4.70, 0.0, 1.06),
                    (-5.24, 0.0, 1.05),
                ],
                radius=0.10,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=body_yellow,
        name="tail_boom",
    )
    airframe.visual(
        Cylinder(radius=0.09, length=0.28),
        origin=Origin(xyz=(-5.28, 0.0, 1.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="tail_gearbox",
    )
    airframe.visual(
        Box((0.56, 0.05, 0.94)),
        origin=Origin(xyz=(-5.08, 0.0, 1.40)),
        material=body_yellow,
        name="vertical_fin",
    )
    airframe.visual(
        Box((0.34, 0.04, 0.30)),
        origin=Origin(xyz=(-5.06, 0.0, 0.78)),
        material=body_yellow,
        name="ventral_fin",
    )

    left_skid = tube_from_spline_points(
        [
            (1.02, 0.92, 0.22),
            (0.32, 0.92, 0.15),
            (-0.72, 0.92, 0.15),
            (-1.74, 0.92, 0.16),
            (-2.34, 0.92, 0.25),
        ],
        radius=0.045,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    right_skid = tube_from_spline_points(
        [
            (1.02, -0.92, 0.22),
            (0.32, -0.92, 0.15),
            (-0.72, -0.92, 0.15),
            (-1.74, -0.92, 0.16),
            (-2.34, -0.92, 0.25),
        ],
        radius=0.045,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    airframe.visual(_save_mesh("left_skid", left_skid), material=skid_steel, name="left_skid")
    airframe.visual(_save_mesh("right_skid", right_skid), material=skid_steel, name="right_skid")

    _add_member(airframe, (0.44, 0.48, 0.70), (0.58, 0.92, 0.20), 0.035, skid_steel, name="front_left_strut")
    _add_member(airframe, (0.44, -0.48, 0.70), (0.58, -0.92, 0.20), 0.035, skid_steel, name="front_right_strut")
    _add_member(airframe, (-0.72, 0.50, 0.70), (-0.66, 0.92, 0.15), 0.035, skid_steel, name="rear_left_strut")
    _add_member(airframe, (-0.72, -0.50, 0.70), (-0.66, -0.92, 0.15), 0.035, skid_steel, name="rear_right_strut")
    _add_member(airframe, (0.58, 0.82, 0.28), (0.58, -0.82, 0.28), 0.032, skid_steel, name="front_crosstube")
    _add_member(airframe, (-0.68, 0.82, 0.26), (-0.68, -0.82, 0.26), 0.032, skid_steel, name="rear_crosstube")

    airframe.inertial = Inertial.from_geometry(
        Box((6.70, 2.20, 2.15)),
        mass=1450.0,
        origin=Origin(xyz=(-2.05, 0.0, 1.05)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=machinery_gray,
        name="hub",
    )
    main_rotor.visual(
        Cylinder(radius=0.05, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=machinery_gray,
        name="mast_stub",
    )
    for index, angle_deg in enumerate((0.0, 120.0, 240.0)):
        angle = math.radians(angle_deg)
        main_rotor.visual(
            Box((0.44, 0.22, 0.06)),
            origin=Origin(
                xyz=(0.35 * math.cos(angle), 0.35 * math.sin(angle), 0.06),
                rpy=(0.0, 0.0, angle),
            ),
            material=machinery_gray,
            name=f"grip_{index}",
        )
        main_rotor.visual(
            Box((4.10, 0.18, 0.035)),
            origin=Origin(
                xyz=(2.16 * math.cos(angle), 2.16 * math.sin(angle), 0.06),
                rpy=(0.0, 0.0, angle),
            ),
            material=rotor_black,
            name=f"blade_{index}",
        )
    main_rotor.inertial = Inertial.from_geometry(
        Box((8.40, 8.40, 0.24)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.06, length=0.12),
        origin=Origin(xyz=(0.0, 0.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((0.66, 0.020, 0.09)),
        origin=Origin(xyz=(0.0, 0.10, 0.0)),
        material=rotor_black,
        name="tail_blade_x",
    )
    tail_rotor.visual(
        Box((0.09, 0.020, 0.72)),
        origin=Origin(xyz=(0.0, 0.10, 0.0)),
        material=rotor_black,
        name="tail_blade_z",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Box((0.72, 0.16, 0.78)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.08, 0.0)),
    )

    left_cowling = model.part("left_cowling")
    left_cowling.visual(
        Cylinder(radius=0.010, length=0.84),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="hinge_barrel",
    )
    left_cowling.visual(
        Box((0.84, 0.24, 0.03)),
        origin=Origin(xyz=(0.0, 0.12, -0.015)),
        material=body_yellow,
        name="panel_shell",
    )
    left_cowling.inertial = Inertial.from_geometry(
        Box((0.84, 0.24, 0.05)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.12, -0.02)),
    )

    right_cowling = model.part("right_cowling")
    right_cowling.visual(
        Cylinder(radius=0.010, length=0.84),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="hinge_barrel",
    )
    right_cowling.visual(
        Box((0.84, 0.24, 0.03)),
        origin=Origin(xyz=(0.0, -0.12, -0.015)),
        material=body_yellow,
        name="panel_shell",
    )
    right_cowling.inertial = Inertial.from_geometry(
        Box((0.84, 0.24, 0.05)),
        mass=11.0,
        origin=Origin(xyz=(0.0, -0.12, -0.02)),
    )

    model.articulation(
        "mast_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(-0.04, 0.0, 1.89)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=38.0),
    )
    model.articulation(
        "tail_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-5.28, 0.14, 1.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=180.0),
    )
    model.articulation(
        "left_cowling_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_cowling,
        origin=Origin(xyz=(-0.82, 0.11, 1.545)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )
    model.articulation(
        "right_cowling_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_cowling,
        origin=Origin(xyz=(-0.82, -0.11, 1.545)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    airframe = object_model.get_part("airframe")
    left_cowling = object_model.get_part("left_cowling")
    right_cowling = object_model.get_part("right_cowling")

    mast_spin = object_model.get_articulation("mast_spin")
    tail_spin = object_model.get_articulation("tail_spin")
    left_hinge = object_model.get_articulation("left_cowling_hinge")
    right_hinge = object_model.get_articulation("right_cowling_hinge")

    ctx.check(
        "main rotor spins about vertical mast axis",
        mast_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(mast_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={mast_spin.articulation_type}, axis={mast_spin.axis}",
    )
    ctx.check(
        "tail rotor spins about transverse boom axis",
        tail_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(tail_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={tail_spin.articulation_type}, axis={tail_spin.axis}",
    )

    ctx.expect_gap(
        left_cowling,
        airframe,
        axis="z",
        positive_elem="panel_shell",
        negative_elem="left_panel_seat",
        max_gap=0.001,
        max_penetration=0.0,
        name="left cowling closes onto its seat",
    )
    ctx.expect_overlap(
        left_cowling,
        airframe,
        axes="xy",
        elem_a="panel_shell",
        elem_b="left_panel_seat",
        min_overlap=0.20,
        name="left cowling covers its seat footprint",
    )
    ctx.expect_gap(
        right_cowling,
        airframe,
        axis="z",
        positive_elem="panel_shell",
        negative_elem="right_panel_seat",
        max_gap=0.001,
        max_penetration=0.0,
        name="right cowling closes onto its seat",
    )
    ctx.expect_overlap(
        right_cowling,
        airframe,
        axes="xy",
        elem_a="panel_shell",
        elem_b="right_panel_seat",
        min_overlap=0.20,
        name="right cowling covers its seat footprint",
    )

    open_angle = math.radians(60.0)
    with ctx.pose({left_hinge: open_angle, right_hinge: open_angle}):
        ctx.expect_gap(
            left_cowling,
            airframe,
            axis="z",
            positive_elem="panel_shell",
            negative_elem="left_panel_seat",
            min_gap=0.01,
            name="left cowling lifts when opened",
        )
        ctx.expect_gap(
            right_cowling,
            airframe,
            axis="z",
            positive_elem="panel_shell",
            negative_elem="right_panel_seat",
            min_gap=0.01,
            name="right cowling lifts when opened",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
