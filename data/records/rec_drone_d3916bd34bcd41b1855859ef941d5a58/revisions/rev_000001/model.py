from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    def yz_rounded_section(
        x_pos: float,
        width: float,
        height: float,
        radius: float,
        *,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y_pos, z_center + z_pos)
            for y_pos, z_pos in rounded_rect_profile(width, height, radius, corner_segments=6)
        ]

    def airfoil_loop_at_y(
        y_pos: float,
        chord: float,
        thickness: float,
        *,
        x_le: float = 0.0,
        z_base: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_le + 0.00 * chord, y_pos, z_base + 0.00 * thickness),
            (x_le + 0.05 * chord, y_pos, z_base + 0.46 * thickness),
            (x_le + 0.24 * chord, y_pos, z_base + 0.70 * thickness),
            (x_le + 0.58 * chord, y_pos, z_base + 0.40 * thickness),
            (x_le + 1.00 * chord, y_pos, z_base + 0.04 * thickness),
            (x_le + 0.84 * chord, y_pos, z_base - 0.06 * thickness),
            (x_le + 0.46 * chord, y_pos, z_base - 0.22 * thickness),
            (x_le + 0.10 * chord, y_pos, z_base - 0.14 * thickness),
        ]

    def fin_loop_at_y(
        y_pos: float,
        root_chord: float,
        height: float,
        *,
        sweep: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (0.00, y_pos, 0.00),
            (0.10 * root_chord, y_pos, 0.62 * height),
            (sweep, y_pos, height),
            (0.78 * root_chord, y_pos, 0.56 * height),
            (root_chord, y_pos, 0.00),
            (0.54 * root_chord, y_pos, 0.04 * height),
        ]

    def prop_blade_section(
        y_pos: float,
        chord: float,
        thickness: float,
        *,
        x_center: float,
        z_center: float,
        pitch_deg: float,
    ) -> list[tuple[float, float, float]]:
        pitch = math.radians(pitch_deg)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)
        base_loop = [
            (-0.50 * chord, 0.00 * thickness),
            (-0.16 * chord, 0.54 * thickness),
            (0.28 * chord, 0.36 * thickness),
            (0.48 * chord, 0.06 * thickness),
            (0.24 * chord, -0.16 * thickness),
            (-0.20 * chord, -0.24 * thickness),
        ]
        section: list[tuple[float, float, float]] = []
        for x_local, z_local in base_loop:
            x_rot = cos_pitch * x_local - sin_pitch * z_local
            z_rot = sin_pitch * x_local + cos_pitch * z_local
            section.append((x_center + x_rot, y_pos, z_center + z_rot))
        return section

    def merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
        merged = MeshGeometry()
        for geometry in geometries:
            merged.merge(geometry)
        return merged

    def build_wing_mesh(side: int) -> MeshGeometry:
        span_sign = 1.0 if side > 0 else -1.0
        section_specs = [
            (0.00, 0.58, 0.078, 0.00, 0.000),
            (0.78, 0.42, 0.052, 0.10, 0.060),
            (1.52, 0.24, 0.028, 0.20, 0.135),
        ]
        sections = [
            airfoil_loop_at_y(
                span_sign * y_pos,
                chord,
                thickness,
                x_le=x_le,
                z_base=z_base,
            )
            for y_pos, chord, thickness, x_le, z_base in section_specs
        ]
        if side < 0:
            sections = [list(reversed(section)) for section in sections]
        return repair_loft(section_loft(sections))

    def build_tailplane_mesh() -> MeshGeometry:
        sections = [
            airfoil_loop_at_y(-0.60, 0.14, 0.015, x_le=0.10, z_base=0.022),
            airfoil_loop_at_y(-0.26, 0.20, 0.020, x_le=0.05, z_base=0.010),
            airfoil_loop_at_y(0.00, 0.30, 0.026, x_le=0.00, z_base=0.000),
            airfoil_loop_at_y(0.26, 0.20, 0.020, x_le=0.05, z_base=0.010),
            airfoil_loop_at_y(0.60, 0.14, 0.015, x_le=0.10, z_base=0.022),
        ]
        return repair_loft(section_loft(sections))

    def build_fuselage_body_mesh() -> MeshGeometry:
        sections = [
            yz_rounded_section(0.00, 0.025, 0.030, 0.008, z_center=0.000),
            yz_rounded_section(0.12, 0.140, 0.165, 0.030, z_center=0.000),
            yz_rounded_section(0.40, 0.220, 0.240, 0.050, z_center=0.010),
            yz_rounded_section(0.90, 0.280, 0.290, 0.065, z_center=0.020),
            yz_rounded_section(1.28, 0.230, 0.245, 0.050, z_center=0.030),
            yz_rounded_section(1.66, 0.130, 0.170, 0.030, z_center=0.040),
            yz_rounded_section(1.96, 0.060, 0.090, 0.018, z_center=0.028),
            yz_rounded_section(2.08, 0.022, 0.034, 0.008, z_center=0.020),
        ]
        return repair_loft(section_loft(sections))

    def build_tail_fin_mesh() -> MeshGeometry:
        return repair_loft(
            section_loft(
                [
                    fin_loop_at_y(-0.020, 0.34, 0.48, sweep=0.20),
                    fin_loop_at_y(0.020, 0.34, 0.48, sweep=0.20),
                ]
            )
        )

    def build_propeller_mesh() -> MeshGeometry:
        blade = repair_loft(
            section_loft(
                [
                    prop_blade_section(
                        0.040,
                        0.115,
                        0.032,
                        x_center=0.022,
                        z_center=0.000,
                        pitch_deg=28.0,
                    ),
                    prop_blade_section(
                        0.175,
                        0.082,
                        0.020,
                        x_center=0.016,
                        z_center=0.000,
                        pitch_deg=18.0,
                    ),
                    prop_blade_section(
                        0.310,
                        0.050,
                        0.010,
                        x_center=0.010,
                        z_center=0.000,
                        pitch_deg=10.0,
                    ),
                ]
            )
        )
        opposite_blade = blade.copy().rotate_x(math.pi)
        hub = CylinderGeometry(radius=0.055, height=0.090, radial_segments=32).rotate_y(
            math.pi / 2.0
        ).translate(0.028, 0.0, 0.0)
        spinner = ConeGeometry(radius=0.040, height=0.085, radial_segments=32, closed=True).rotate_y(
            -math.pi / 2.0
        ).translate(0.088, 0.0, 0.0)
        shaft = CylinderGeometry(radius=0.015, height=0.045, radial_segments=24).rotate_y(
            math.pi / 2.0
        ).translate(-0.030, 0.0, 0.0)
        rear_collar = CylinderGeometry(radius=0.026, height=0.032, radial_segments=28).rotate_y(
            math.pi / 2.0
        ).translate(-0.020, 0.0, 0.0)
        return merge_geometries([hub, spinner, shaft, rear_collar, blade, opposite_blade])

    def build_camera_pod_mesh() -> MeshGeometry:
        return repair_loft(
            section_loft(
                [
                    yz_rounded_section(0.00, 0.120, 0.050, 0.014, z_center=-0.024),
                    yz_rounded_section(0.08, 0.190, 0.130, 0.030, z_center=-0.074),
                    yz_rounded_section(0.18, 0.180, 0.142, 0.032, z_center=-0.082),
                    yz_rounded_section(0.31, 0.110, 0.100, 0.022, z_center=-0.066),
                ]
            )
        )

    model = ArticulatedObject(name="mapping_survey_drone")

    fuselage_white = model.material("fuselage_white", rgba=(0.90, 0.92, 0.94, 1.0))
    wing_white = model.material("wing_white", rgba=(0.93, 0.94, 0.95, 1.0))
    carbon_black = model.material("carbon_black", rgba=(0.11, 0.12, 0.13, 1.0))
    fuselage = model.part("fuselage")
    fuselage.visual(
        mesh_from_geometry(build_fuselage_body_mesh(), "survey_drone_fuselage_body"),
        material=fuselage_white,
        name="fuselage_body",
    )
    fuselage.visual(
        mesh_from_geometry(build_tailplane_mesh(), "survey_drone_tailplane"),
        origin=Origin(xyz=(1.54, 0.0, 0.040)),
        material=wing_white,
        name="tailplane",
    )
    fuselage.visual(
        Box((0.18, 0.10, 0.026)),
        origin=Origin(xyz=(0.90, 0.0, -0.121)),
        material=carbon_black,
        name="camera_mount",
    )
    fuselage.visual(
        Box((0.22, 0.045, 0.055)),
        origin=Origin(xyz=(1.69, 0.0, 0.1385)),
        material=carbon_black,
        name="tail_fin_mount",
    )
    fuselage.visual(
        Box((0.055, 0.065, 0.145)),
        origin=Origin(xyz=(2.04, 0.0, 0.103)),
        material=carbon_black,
        name="motor_pylon",
    )
    fuselage.inertial = Inertial.from_geometry(
        Box((2.10, 0.30, 0.34)),
        mass=18.0,
        origin=Origin(xyz=(1.05, 0.0, 0.025)),
    )

    left_wing = model.part("left_wing")
    left_wing.visual(
        mesh_from_geometry(build_wing_mesh(side=1), "survey_drone_left_wing"),
        material=wing_white,
        name="wing_skin",
    )
    left_wing.inertial = Inertial.from_geometry(
        Box((0.60, 1.52, 0.09)),
        mass=2.7,
        origin=Origin(xyz=(0.30, 0.76, 0.065)),
    )

    right_wing = model.part("right_wing")
    right_wing.visual(
        mesh_from_geometry(build_wing_mesh(side=-1), "survey_drone_right_wing"),
        material=wing_white,
        name="wing_skin",
    )
    right_wing.inertial = Inertial.from_geometry(
        Box((0.60, 1.52, 0.09)),
        mass=2.7,
        origin=Origin(xyz=(0.30, -0.76, 0.065)),
    )

    tail_fin = model.part("tail_fin")
    tail_fin.visual(
        mesh_from_geometry(build_tail_fin_mesh(), "survey_drone_tail_fin"),
        material=fuselage_white,
        name="fin_skin",
    )
    tail_fin.inertial = Inertial.from_geometry(
        Box((0.34, 0.05, 0.48)),
        mass=0.7,
        origin=Origin(xyz=(0.17, 0.0, 0.24)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        mesh_from_geometry(build_propeller_mesh(), "survey_drone_propeller"),
        material=carbon_black,
        name="propeller_assembly",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.33, length=0.10),
        mass=1.1,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    camera_pod = model.part("camera_pod")
    camera_pod.visual(
        mesh_from_geometry(build_camera_pod_mesh(), "survey_drone_camera_pod"),
        material=carbon_black,
        name="pod_shell",
    )
    camera_pod.visual(
        Cylinder(radius=0.011, length=0.170),
        origin=Origin(xyz=(0.001, 0.0, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=carbon_black,
        name="hinge_barrel",
    )
    camera_pod.inertial = Inertial.from_geometry(
        Box((0.32, 0.20, 0.16)),
        mass=1.4,
        origin=Origin(xyz=(0.16, 0.0, -0.080)),
    )

    model.articulation(
        "fuselage_to_left_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=left_wing,
        origin=Origin(xyz=(0.74, 0.138, 0.085)),
    )
    model.articulation(
        "fuselage_to_right_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=right_wing,
        origin=Origin(xyz=(0.74, -0.138, 0.085)),
    )
    model.articulation(
        "fuselage_to_tail_fin",
        ArticulationType.FIXED,
        parent=fuselage,
        child=tail_fin,
        origin=Origin(xyz=(1.58, 0.0, 0.166)),
    )
    model.articulation(
        "fuselage_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(2.12, 0.0, 0.103)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=90.0),
    )
    model.articulation(
        "fuselage_to_camera_pod",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=camera_pod,
        origin=Origin(xyz=(0.82, 0.0, -0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
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

    fuselage = object_model.get_part("fuselage")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    tail_fin = object_model.get_part("tail_fin")
    propeller = object_model.get_part("propeller")
    camera_pod = object_model.get_part("camera_pod")

    prop_spin = object_model.get_articulation("fuselage_to_propeller")
    pod_tilt = object_model.get_articulation("fuselage_to_camera_pod")

    left_origin = ctx.part_world_position(left_wing)
    right_origin = ctx.part_world_position(right_wing)
    ctx.check(
        "main wings are mounted symmetrically",
        left_origin is not None
        and right_origin is not None
        and abs(left_origin[0] - right_origin[0]) < 0.001
        and abs(left_origin[2] - right_origin[2]) < 0.001
        and abs(left_origin[1] + right_origin[1]) < 0.001,
        details=f"left_origin={left_origin}, right_origin={right_origin}",
    )

    ctx.expect_gap(
        left_wing,
        fuselage,
        axis="y",
        positive_elem="wing_skin",
        negative_elem="fuselage_body",
        max_gap=0.02,
        max_penetration=0.003,
        name="left wing root seats against fuselage side",
    )
    ctx.expect_gap(
        fuselage,
        right_wing,
        axis="y",
        positive_elem="fuselage_body",
        negative_elem="wing_skin",
        max_gap=0.02,
        max_penetration=0.003,
        name="right wing root seats against fuselage side",
    )
    ctx.expect_gap(
        tail_fin,
        fuselage,
        axis="z",
        positive_elem="fin_skin",
        negative_elem="tail_fin_mount",
        min_gap=0.0,
        max_gap=0.02,
        name="vertical fin sits on the top of the tail cone",
    )
    ctx.expect_contact(
        propeller,
        fuselage,
        elem_a="propeller_assembly",
        elem_b="motor_pylon",
        contact_tol=0.02,
        name="pusher propeller hub stays mounted to the rear pylon",
    )
    ctx.expect_contact(
        camera_pod,
        fuselage,
        elem_a="hinge_barrel",
        elem_b="camera_mount",
        contact_tol=0.004,
        name="camera pod is carried by the underside hinge mount",
    )

    ctx.check(
        "propeller articulation is continuous about the fuselage axis",
        prop_spin.articulation_type == ArticulationType.CONTINUOUS
        and prop_spin.axis == (1.0, 0.0, 0.0)
        and prop_spin.motion_limits is not None
        and prop_spin.motion_limits.lower is None
        and prop_spin.motion_limits.upper is None,
        details=(
            f"type={prop_spin.articulation_type}, axis={prop_spin.axis}, "
            f"limits={prop_spin.motion_limits}"
        ),
    )
    ctx.check(
        "camera pod hinge uses a forward edge pitch axis and realistic range",
        pod_tilt.articulation_type == ArticulationType.REVOLUTE
        and pod_tilt.axis == (0.0, 1.0, 0.0)
        and pod_tilt.motion_limits is not None
        and pod_tilt.motion_limits.lower == 0.0
        and pod_tilt.motion_limits.upper is not None
        and pod_tilt.motion_limits.upper >= math.radians(60.0),
        details=f"type={pod_tilt.articulation_type}, axis={pod_tilt.axis}, limits={pod_tilt.motion_limits}",
    )

    pod_rest = ctx.part_element_world_aabb(camera_pod, elem="pod_shell")
    with ctx.pose({pod_tilt: math.radians(60.0)}):
        pod_open = ctx.part_element_world_aabb(camera_pod, elem="pod_shell")
        ctx.check(
            "camera pod tilts downward when opened",
            pod_rest is not None
            and pod_open is not None
            and (0.5 * (pod_open[0][2] + pod_open[1][2])) < (0.5 * (pod_rest[0][2] + pod_rest[1][2])) - 0.04,
            details=f"rest={pod_rest}, open={pod_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
