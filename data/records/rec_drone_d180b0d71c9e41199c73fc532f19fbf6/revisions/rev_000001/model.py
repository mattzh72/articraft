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
)


def build_object_model() -> ArticulatedObject:
    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def yz_section(
        width: float,
        height: float,
        x_pos: float,
        *,
        z_center: float = 0.0,
        radius: float | None = None,
    ) -> list[tuple[float, float, float]]:
        corner = radius if radius is not None else min(width, height) * 0.22
        return [
            (x_pos, y, z + z_center)
            for z, y in rounded_rect_profile(height, width, corner, corner_segments=8)
        ]

    def wing_section(
        y_pos: float,
        *,
        x_le: float,
        chord: float,
        thickness: float,
        z_offset: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_le, y_pos, z_offset + 0.00 * thickness),
            (x_le - 0.05 * chord, y_pos, z_offset + 0.34 * thickness),
            (x_le - 0.22 * chord, y_pos, z_offset + 0.50 * thickness),
            (x_le - 0.52 * chord, y_pos, z_offset + 0.42 * thickness),
            (x_le - 0.84 * chord, y_pos, z_offset + 0.18 * thickness),
            (x_le - 1.00 * chord, y_pos, z_offset + 0.02 * thickness),
            (x_le - 0.88 * chord, y_pos, z_offset - 0.05 * thickness),
            (x_le - 0.52 * chord, y_pos, z_offset - 0.12 * thickness),
            (x_le - 0.18 * chord, y_pos, z_offset - 0.10 * thickness),
            (x_le - 0.04 * chord, y_pos, z_offset - 0.04 * thickness),
        ]

    def blade_section(
        radial_y: float,
        *,
        chord: float,
        thickness: float,
        pitch: float,
    ) -> list[tuple[float, float, float]]:
        base_loop = [
            (-0.52 * chord, -0.42 * thickness),
            (0.30 * chord, -0.20 * thickness),
            (0.48 * chord, 0.22 * thickness),
            (-0.26 * chord, 0.40 * thickness),
        ]
        c = math.cos(pitch)
        s = math.sin(pitch)
        section: list[tuple[float, float, float]] = []
        for x_local, z_local in base_loop:
            x_rot = c * x_local - s * z_local
            z_rot = s * x_local + c * z_local
            section.append((x_rot, radial_y, z_rot))
        return section

    def build_fuselage_mesh():
        return section_loft(
            [
                yz_section(0.020, 0.025, 0.42, z_center=-0.005, radius=0.004),
                yz_section(0.090, 0.110, 0.24, z_center=-0.010, radius=0.020),
                yz_section(0.155, 0.180, 0.02, z_center=-0.015, radius=0.028),
                yz_section(0.135, 0.155, -0.18, z_center=0.005, radius=0.022),
                yz_section(0.090, 0.105, -0.46, z_center=0.045, radius=0.018),
                yz_section(0.035, 0.055, -0.70, z_center=0.080, radius=0.008),
            ]
        )

    def build_wing_mesh():
        return section_loft(
            [
                wing_section(-1.02, x_le=0.020, chord=0.220, thickness=0.045, z_offset=0.020),
                wing_section(-0.72, x_le=0.045, chord=0.260, thickness=0.052, z_offset=0.012),
                wing_section(-0.34, x_le=0.082, chord=0.305, thickness=0.060, z_offset=0.005),
                wing_section(0.00, x_le=0.115, chord=0.340, thickness=0.065, z_offset=0.000),
                wing_section(0.34, x_le=0.082, chord=0.305, thickness=0.060, z_offset=0.005),
                wing_section(0.72, x_le=0.045, chord=0.260, thickness=0.052, z_offset=0.012),
                wing_section(1.02, x_le=0.020, chord=0.220, thickness=0.045, z_offset=0.020),
            ]
        )

    def build_tailplane_mesh():
        return section_loft(
            [
                wing_section(-0.28, x_le=-0.470, chord=0.115, thickness=0.018, z_offset=0.095),
                wing_section(0.00, x_le=-0.500, chord=0.160, thickness=0.020, z_offset=0.095),
                wing_section(0.28, x_le=-0.470, chord=0.115, thickness=0.018, z_offset=0.095),
            ]
        )

    def build_tail_fin_mesh():
        def fin_loop(y_pos: float) -> list[tuple[float, float, float]]:
            return [
                (-0.070, y_pos, 0.000),
                (0.026, y_pos, 0.000),
                (0.054, y_pos, 0.046),
                (0.014, y_pos, 0.162),
                (-0.030, y_pos, 0.190),
                (-0.060, y_pos, 0.104),
            ]

        return section_loft(
            [
                fin_loop(-0.009),
                fin_loop(0.0),
                fin_loop(0.009),
            ]
        )

    def build_propeller_mesh():
        blade = section_loft(
            [
                blade_section(0.028, chord=0.080, thickness=0.014, pitch=0.42),
                blade_section(0.110, chord=0.065, thickness=0.010, pitch=0.24),
                blade_section(0.205, chord=0.038, thickness=0.006, pitch=0.12),
            ]
        )
        blade.merge(blade.copy().rotate_x(math.pi))
        return blade

    model = ArticulatedObject(name="fixed_wing_vtol_hybrid_drone")

    composite_white = model.material("composite_white", rgba=(0.86, 0.88, 0.90, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.27, 0.30, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.08, 0.09, 0.10, 1.0))
    amber = model.material("amber", rgba=(0.82, 0.48, 0.12, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        save_mesh("airframe_fuselage", build_fuselage_mesh()),
        material=composite_white,
        name="fuselage_shell",
    )
    airframe.visual(
        save_mesh("airframe_wing", build_wing_mesh()),
        material=composite_white,
        name="wing_shell",
    )
    airframe.visual(
        save_mesh("airframe_tailplane", build_tailplane_mesh()),
        material=composite_white,
        name="tailplane_shell",
    )
    airframe.visual(
        save_mesh("right_tail_fin", build_tail_fin_mesh()),
        origin=Origin(xyz=(-0.56, 0.18, 0.09)),
        material=composite_white,
        name="right_tail_fin",
    )
    airframe.visual(
        save_mesh("left_tail_fin", build_tail_fin_mesh()),
        origin=Origin(xyz=(-0.56, -0.18, 0.09)),
        material=composite_white,
        name="left_tail_fin",
    )
    airframe.visual(
        Box((0.10, 0.12, 0.09)),
        origin=Origin(xyz=(0.040, 0.960, -0.005)),
        material=dark_gray,
        name="right_tip_pylon",
    )
    airframe.visual(
        Box((0.10, 0.12, 0.09)),
        origin=Origin(xyz=(0.040, -0.960, -0.005)),
        material=dark_gray,
        name="left_tip_pylon",
    )
    airframe.visual(
        Box((0.060, 0.050, 0.080)),
        origin=Origin(xyz=(0.040, 1.045, 0.030)),
        material=amber,
        name="right_hinge_block",
    )
    airframe.visual(
        Box((0.060, 0.050, 0.080)),
        origin=Origin(xyz=(0.040, -1.045, 0.030)),
        material=amber,
        name="left_hinge_block",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((1.20, 2.30, 0.40)),
        mass=8.0,
        origin=Origin(xyz=(-0.05, 0.0, 0.05)),
    )

    right_nacelle = model.part("right_nacelle")
    right_nacelle.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="right_trunnion",
    )
    right_nacelle.visual(
        Box((0.120, 0.080, 0.060)),
        origin=Origin(xyz=(0.030, 0.0, -0.020)),
        material=dark_gray,
        name="right_hinge_arm",
    )
    right_nacelle.visual(
        Cylinder(radius=0.055, length=0.270),
        origin=Origin(xyz=(0.075, 0.0, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="right_motor_housing",
    )
    right_nacelle.visual(
        Cylinder(radius=0.060, length=0.038),
        origin=Origin(xyz=(0.214, 0.0, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_gray,
        name="right_front_ring",
    )
    right_nacelle.visual(
        Box((0.050, 0.060, 0.018)),
        origin=Origin(xyz=(0.125, 0.0, -0.106)),
        material=motor_gray,
        name="right_lower_keel",
    )
    right_nacelle.inertial = Inertial.from_geometry(
        Box((0.30, 0.12, 0.14)),
        mass=1.0,
        origin=Origin(xyz=(0.09, 0.0, -0.05)),
    )

    left_nacelle = model.part("left_nacelle")
    left_nacelle.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="left_trunnion",
    )
    left_nacelle.visual(
        Box((0.120, 0.080, 0.060)),
        origin=Origin(xyz=(0.030, 0.0, -0.020)),
        material=dark_gray,
        name="left_hinge_arm",
    )
    left_nacelle.visual(
        Cylinder(radius=0.055, length=0.270),
        origin=Origin(xyz=(0.075, 0.0, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="left_motor_housing",
    )
    left_nacelle.visual(
        Cylinder(radius=0.060, length=0.038),
        origin=Origin(xyz=(0.214, 0.0, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_gray,
        name="left_front_ring",
    )
    left_nacelle.visual(
        Box((0.050, 0.060, 0.018)),
        origin=Origin(xyz=(0.125, 0.0, -0.106)),
        material=motor_gray,
        name="left_lower_keel",
    )
    left_nacelle.inertial = Inertial.from_geometry(
        Box((0.30, 0.12, 0.14)),
        mass=1.0,
        origin=Origin(xyz=(0.09, 0.0, -0.05)),
    )

    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        Cylinder(radius=0.022, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_gray,
        name="right_prop_hub",
    )
    right_propeller.visual(
        Cylinder(radius=0.007, length=0.058),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_gray,
        name="right_drive_shaft",
    )
    right_propeller.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="right_spinner",
    )
    right_propeller.visual(
        save_mesh("right_propeller_blades", build_propeller_mesh()),
        material=rotor_black,
        name="right_prop_blades",
    )
    right_propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.040),
        mass=0.20,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        Cylinder(radius=0.022, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_gray,
        name="left_prop_hub",
    )
    left_propeller.visual(
        Cylinder(radius=0.007, length=0.058),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_gray,
        name="left_drive_shaft",
    )
    left_propeller.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="left_spinner",
    )
    left_propeller.visual(
        save_mesh("left_propeller_blades", build_propeller_mesh()),
        material=rotor_black,
        name="left_prop_blades",
    )
    left_propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.040),
        mass=0.20,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "airframe_to_right_nacelle",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_nacelle,
        origin=Origin(xyz=(0.040, 1.120, 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "airframe_to_left_nacelle",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_nacelle,
        origin=Origin(xyz=(0.040, -1.120, 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "right_nacelle_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=right_nacelle,
        child=right_propeller,
        origin=Origin(xyz=(0.280, 0.0, -0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=60.0),
    )
    model.articulation(
        "left_nacelle_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=left_nacelle,
        child=left_propeller,
        origin=Origin(xyz=(0.280, 0.0, -0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=60.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    right_nacelle = object_model.get_part("right_nacelle")
    left_nacelle = object_model.get_part("left_nacelle")
    right_propeller = object_model.get_part("right_propeller")
    left_propeller = object_model.get_part("left_propeller")

    right_tilt = object_model.get_articulation("airframe_to_right_nacelle")
    left_tilt = object_model.get_articulation("airframe_to_left_nacelle")
    right_spin = object_model.get_articulation("right_nacelle_to_propeller")
    left_spin = object_model.get_articulation("left_nacelle_to_propeller")

    ctx.check(
        "tilt hinges pitch nacelles upward",
        right_tilt.axis == (0.0, -1.0, 0.0)
        and left_tilt.axis == (0.0, -1.0, 0.0)
        and right_tilt.motion_limits is not None
        and left_tilt.motion_limits is not None
        and right_tilt.motion_limits.lower == 0.0
        and left_tilt.motion_limits.lower == 0.0
        and right_tilt.motion_limits.upper is not None
        and left_tilt.motion_limits.upper is not None
        and right_tilt.motion_limits.upper >= math.pi / 2.0 - 1e-6
        and left_tilt.motion_limits.upper >= math.pi / 2.0 - 1e-6,
        details=f"right_axis={right_tilt.axis}, left_axis={left_tilt.axis}",
    )
    ctx.check(
        "propellers spin about nacelle fore-aft axes",
        right_spin.axis == (1.0, 0.0, 0.0)
        and left_spin.axis == (1.0, 0.0, 0.0)
        and right_spin.motion_limits is not None
        and left_spin.motion_limits is not None
        and right_spin.motion_limits.lower is None
        and right_spin.motion_limits.upper is None
        and left_spin.motion_limits.lower is None
        and left_spin.motion_limits.upper is None,
        details=f"right_spin_axis={right_spin.axis}, left_spin_axis={left_spin.axis}",
    )

    ctx.expect_contact(
        airframe,
        right_nacelle,
        elem_a="right_hinge_block",
        elem_b="right_trunnion",
        contact_tol=0.001,
        name="right nacelle trunnion seats on wing hinge block",
    )
    ctx.expect_contact(
        airframe,
        left_nacelle,
        elem_a="left_hinge_block",
        elem_b="left_trunnion",
        contact_tol=0.001,
        name="left nacelle trunnion seats on wing hinge block",
    )
    ctx.expect_gap(
        right_propeller,
        right_nacelle,
        axis="x",
        positive_elem="right_prop_hub",
        negative_elem="right_front_ring",
        min_gap=0.015,
        max_gap=0.050,
        name="right propeller hub sits just ahead of nacelle nose",
    )
    ctx.expect_gap(
        left_propeller,
        left_nacelle,
        axis="x",
        positive_elem="left_prop_hub",
        negative_elem="left_front_ring",
        min_gap=0.015,
        max_gap=0.050,
        name="left propeller hub sits just ahead of nacelle nose",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

    rest_center = aabb_center(ctx.part_world_aabb(right_propeller))
    with ctx.pose({right_tilt: math.pi / 2.0}):
        raised_center = aabb_center(ctx.part_world_aabb(right_propeller))
        ctx.expect_gap(
            right_propeller,
            airframe,
            axis="z",
            positive_elem="right_prop_hub",
            negative_elem="wing_shell",
            min_gap=0.14,
            name="tilted right propeller lifts clear above the wing",
        )

    ctx.check(
        "right nacelle tilt raises its propeller for VTOL mode",
        rest_center is not None
        and raised_center is not None
        and raised_center[2] > rest_center[2] + 0.20,
        details=f"rest_center={rest_center}, raised_center={raised_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
