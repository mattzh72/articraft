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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    fillet = min(radius, width * 0.45, height * 0.45)
    return [
        (x, y, z)
        for y, z in rounded_rect_profile(
            width,
            height,
            fillet,
            corner_segments=8,
        )
    ]


def _build_center_pod_mesh():
    return section_loft(
        [
            _yz_section(0.32, width=0.10, height=0.06, radius=0.020),
            _yz_section(0.18, width=0.15, height=0.09, radius=0.028),
            _yz_section(0.02, width=0.22, height=0.13, radius=0.040),
            _yz_section(-0.22, width=0.18, height=0.11, radius=0.032),
            _yz_section(-0.48, width=0.11, height=0.08, radius=0.024),
        ]
    )


def _build_canopy_mesh():
    return section_loft(
        [
            _yz_section(0.13, width=0.10, height=0.035, radius=0.016),
            _yz_section(0.02, width=0.15, height=0.075, radius=0.030),
            _yz_section(-0.08, width=0.10, height=0.050, radius=0.020),
        ]
    )


def _build_nacelle_shell_mesh():
    return section_loft(
        [
            _yz_section(0.05, width=0.075, height=0.065, radius=0.020),
            _yz_section(0.14, width=0.100, height=0.090, radius=0.030),
            _yz_section(0.26, width=0.095, height=0.085, radius=0.028),
            _yz_section(0.34, width=0.060, height=0.060, radius=0.020),
        ]
    )


def _add_front_propeller(part, *, prop_material, spinner_material) -> None:
    part.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spinner_material,
        name="hub",
    )
    part.visual(
        Box((0.010, 0.580, 0.050)),
        origin=Origin(rpy=(math.radians(10.0), 0.0, 0.0)),
        material=prop_material,
        name="blade_pair",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spinner_material,
        name="spinner",
    )


def _add_rear_propeller(part, *, prop_material, spinner_material) -> None:
    part.visual(
        Cylinder(radius=0.028, length=0.024),
        material=spinner_material,
        name="hub",
    )
    part.visual(
        Box((0.480, 0.055, 0.008)),
        origin=Origin(rpy=(0.0, math.radians(10.0), 0.0)),
        material=prop_material,
        name="blade_pair",
    )
    part.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=spinner_material,
        name="cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_rotor_quadplane")

    wing_skin = model.material("wing_skin", rgba=(0.78, 0.80, 0.83, 1.0))
    fuselage_skin = model.material("fuselage_skin", rgba=(0.66, 0.68, 0.72, 1.0))
    canopy_glass = model.material("canopy_glass", rgba=(0.18, 0.30, 0.38, 0.55))
    carbon = model.material("carbon", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.21, 0.23, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.88, 0.41, 0.12, 1.0))

    center_pod_mesh = _save_mesh("quadplane_center_pod", _build_center_pod_mesh())
    canopy_mesh = _save_mesh("quadplane_canopy", _build_canopy_mesh())
    nacelle_mesh = _save_mesh("quadplane_front_nacelle", _build_nacelle_shell_mesh())

    airframe = model.part("airframe")
    airframe.visual(
        Box((0.36, 1.92, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=wing_skin,
        name="wing_body",
    )
    airframe.visual(
        center_pod_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=fuselage_skin,
        name="center_pod",
    )
    airframe.visual(
        canopy_mesh,
        origin=Origin(xyz=(0.03, 0.0, 0.120)),
        material=canopy_glass,
        name="canopy",
    )
    airframe.visual(
        Box((0.22, 0.18, 0.018)),
        origin=Origin(xyz=(0.00, 0.0, 0.104)),
        material=carbon,
        name="battery_hatch",
    )
    airframe.visual(
        Box((0.54, 0.15, 0.08)),
        origin=Origin(xyz=(-0.33, 0.58, 0.05)),
        material=fuselage_skin,
        name="left_rear_boom",
    )
    airframe.visual(
        Box((0.54, 0.15, 0.08)),
        origin=Origin(xyz=(-0.33, -0.58, 0.05)),
        material=fuselage_skin,
        name="right_rear_boom",
    )
    airframe.visual(
        Box((0.09, 0.09, 0.18)),
        origin=Origin(xyz=(-0.60, 0.58, 0.14)),
        material=dark_metal,
        name="left_rear_mast",
    )
    airframe.visual(
        Box((0.09, 0.09, 0.18)),
        origin=Origin(xyz=(-0.60, -0.58, 0.14)),
        material=dark_metal,
        name="right_rear_mast",
    )
    airframe.visual(
        Cylinder(radius=0.044, length=0.08),
        origin=Origin(xyz=(-0.60, 0.58, 0.27)),
        material=dark_metal,
        name="left_rear_motor",
    )
    airframe.visual(
        Cylinder(radius=0.044, length=0.08),
        origin=Origin(xyz=(-0.60, -0.58, 0.27)),
        material=dark_metal,
        name="right_rear_motor",
    )
    airframe.visual(
        Box((0.07, 0.10, 0.06)),
        origin=Origin(xyz=(0.210, 0.60, 0.045)),
        material=dark_metal,
        name="left_front_mount",
    )
    airframe.visual(
        Box((0.07, 0.10, 0.06)),
        origin=Origin(xyz=(0.210, -0.60, 0.045)),
        material=dark_metal,
        name="right_front_mount",
    )
    airframe.visual(
        Box((0.10, 0.16, 0.018)),
        origin=Origin(xyz=(0.12, 0.84, 0.051)),
        material=safety_orange,
        name="left_tip_marking",
    )
    airframe.visual(
        Box((0.10, 0.16, 0.018)),
        origin=Origin(xyz=(0.12, -0.84, 0.051)),
        material=safety_orange,
        name="right_tip_marking",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((1.20, 1.92, 0.36)),
        mass=9.5,
        origin=Origin(xyz=(-0.14, 0.0, 0.14)),
    )

    def _add_front_nacelle(side: str, y_pos: float) -> None:
        nacelle = model.part(f"{side}_front_nacelle")
        nacelle.visual(
            Cylinder(radius=0.025, length=0.09),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_collar",
        )
        nacelle.visual(
            Box((0.10, 0.09, 0.08)),
            origin=Origin(xyz=(0.025, 0.0, 0.0)),
            material=fuselage_skin,
            name="root_fairing",
        )
        nacelle.visual(
            nacelle_mesh,
            origin=Origin(xyz=(-0.03, 0.0, 0.0)),
            material=fuselage_skin,
            name="nacelle_shell",
        )
        nacelle.visual(
            Box((0.12, 0.018, 0.032)),
            origin=Origin(xyz=(0.18, 0.0, -0.028)),
            material=carbon,
            name="vent_fin",
        )
        nacelle.visual(
            Box((0.09, 0.040, 0.020)),
            origin=Origin(xyz=(0.11, 0.0, 0.030)),
            material=carbon,
            name="top_fairing",
        )
        nacelle.visual(
            Cylinder(radius=0.032, length=0.064),
            origin=Origin(xyz=(0.316, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="motor_can",
        )
        nacelle.inertial = Inertial.from_geometry(
            Box((0.38, 0.12, 0.11)),
            mass=0.8,
            origin=Origin(xyz=(0.19, 0.0, 0.0)),
        )

        model.articulation(
            f"{side}_front_tilt",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=nacelle,
            origin=Origin(xyz=(0.270, y_pos, 0.045)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.2,
                lower=0.0,
                upper=math.pi / 2.0,
            ),
        )

        propeller = model.part(f"{side}_front_propeller")
        _add_front_propeller(
            propeller,
            prop_material=carbon,
            spinner_material=dark_metal,
        )
        propeller.inertial = Inertial.from_geometry(
            Box((0.05, 0.58, 0.06)),
            mass=0.16,
        )

        model.articulation(
            f"{side}_front_prop_spin",
            ArticulationType.CONTINUOUS,
            parent=nacelle,
            child=propeller,
            origin=Origin(xyz=(0.360, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=45.0,
            ),
        )

    def _add_rear_prop(side: str, y_pos: float) -> None:
        propeller = model.part(f"{side}_rear_propeller")
        _add_rear_propeller(
            propeller,
            prop_material=carbon,
            spinner_material=dark_metal,
        )
        propeller.inertial = Inertial.from_geometry(
            Box((0.48, 0.06, 0.05)),
            mass=0.14,
        )

        model.articulation(
            f"{side}_rear_prop_spin",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=propeller,
            origin=Origin(xyz=(-0.60, y_pos, 0.322)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=55.0,
            ),
        )

    _add_front_nacelle("left", 0.60)
    _add_front_nacelle("right", -0.60)
    _add_rear_prop("left", 0.58)
    _add_rear_prop("right", -0.58)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    airframe = object_model.get_part("airframe")
    left_nacelle = object_model.get_part("left_front_nacelle")
    right_nacelle = object_model.get_part("right_front_nacelle")
    left_front_prop = object_model.get_part("left_front_propeller")
    right_front_prop = object_model.get_part("right_front_propeller")
    left_rear_prop = object_model.get_part("left_rear_propeller")
    right_rear_prop = object_model.get_part("right_rear_propeller")

    left_tilt = object_model.get_articulation("left_front_tilt")
    right_tilt = object_model.get_articulation("right_front_tilt")
    left_front_spin = object_model.get_articulation("left_front_prop_spin")
    right_front_spin = object_model.get_articulation("right_front_prop_spin")
    left_rear_spin = object_model.get_articulation("left_rear_prop_spin")
    right_rear_spin = object_model.get_articulation("right_rear_prop_spin")

    ctx.check(
        "front nacelles use horizontal tilt joints",
        left_tilt.axis == (0.0, -1.0, 0.0) and right_tilt.axis == (0.0, -1.0, 0.0),
        details=(
            f"left axis={left_tilt.axis}, right axis={right_tilt.axis}; "
            "expected spanwise horizontal pitch axes."
        ),
    )
    ctx.check(
        "forward propellers spin along nacelle axes",
        left_front_spin.axis == (1.0, 0.0, 0.0) and right_front_spin.axis == (1.0, 0.0, 0.0),
        details=(
            f"left axis={left_front_spin.axis}, right axis={right_front_spin.axis}; "
            "expected fore-aft propeller shafts."
        ),
    )
    ctx.check(
        "rear lift propellers spin on vertical axles",
        left_rear_spin.axis == (0.0, 0.0, 1.0) and right_rear_spin.axis == (0.0, 0.0, 1.0),
        details=(
            f"left axis={left_rear_spin.axis}, right axis={right_rear_spin.axis}; "
            "expected vertical lift rotors."
        ),
    )

    ctx.expect_contact(
        airframe,
        left_nacelle,
        elem_a="left_front_mount",
        elem_b="hinge_collar",
        contact_tol=0.001,
        name="left nacelle seated on leading-edge mount",
    )
    ctx.expect_contact(
        airframe,
        right_nacelle,
        elem_a="right_front_mount",
        elem_b="hinge_collar",
        contact_tol=0.001,
        name="right nacelle seated on leading-edge mount",
    )
    ctx.expect_contact(
        left_nacelle,
        left_front_prop,
        elem_a="motor_can",
        elem_b="hub",
        contact_tol=0.001,
        name="left front propeller mounted to nacelle motor",
    )
    ctx.expect_contact(
        right_nacelle,
        right_front_prop,
        elem_a="motor_can",
        elem_b="hub",
        contact_tol=0.001,
        name="right front propeller mounted to nacelle motor",
    )
    ctx.expect_contact(
        airframe,
        left_rear_prop,
        elem_a="left_rear_motor",
        elem_b="hub",
        contact_tol=0.001,
        name="left rear lift propeller mounted to rear motor",
    )
    ctx.expect_contact(
        airframe,
        right_rear_prop,
        elem_a="right_rear_motor",
        elem_b="hub",
        contact_tol=0.001,
        name="right rear lift propeller mounted to rear motor",
    )

    ctx.expect_gap(
        left_front_prop,
        airframe,
        axis="x",
        min_gap=0.12,
        negative_elem="wing_body",
        name="left tractor propeller stays ahead of wing body",
    )
    ctx.expect_gap(
        right_front_prop,
        airframe,
        axis="x",
        min_gap=0.12,
        negative_elem="wing_body",
        name="right tractor propeller stays ahead of wing body",
    )

    left_front_rest = ctx.part_world_position(left_front_prop)
    right_front_rest = ctx.part_world_position(right_front_prop)
    assert left_front_rest is not None
    assert right_front_rest is not None

    with ctx.pose({left_tilt: math.pi / 2.0, right_tilt: math.pi / 2.0}):
        left_front_hover = ctx.part_world_position(left_front_prop)
        right_front_hover = ctx.part_world_position(right_front_prop)
        assert left_front_hover is not None
        assert right_front_hover is not None

        ctx.check(
            "left nacelle lifts propeller into hover position",
            left_front_hover[2] > left_front_rest[2] + 0.28
            and left_front_hover[0] < left_front_rest[0] - 0.20,
            details=(
                f"rest={left_front_rest}, hover={left_front_hover}; "
                "expected strong upward motion and reduced forward reach."
            ),
        )
        ctx.check(
            "right nacelle lifts propeller into hover position",
            right_front_hover[2] > right_front_rest[2] + 0.28
            and right_front_hover[0] < right_front_rest[0] - 0.20,
            details=(
                f"rest={right_front_rest}, hover={right_front_hover}; "
                "expected strong upward motion and reduced forward reach."
            ),
        )
        ctx.expect_contact(
            airframe,
            left_nacelle,
            elem_a="left_front_mount",
            elem_b="hinge_collar",
            contact_tol=0.001,
            name="left nacelle hinge remains seated in hover",
        )
        ctx.expect_contact(
            airframe,
            right_nacelle,
            elem_a="right_front_mount",
            elem_b="hinge_collar",
            contact_tol=0.001,
            name="right nacelle hinge remains seated in hover",
        )
        ctx.expect_gap(
            left_front_prop,
            airframe,
            axis="z",
            min_gap=0.26,
            negative_elem="wing_body",
            name="left forward propeller clears airframe in hover",
        )
        ctx.expect_gap(
            right_front_prop,
            airframe,
            axis="z",
            min_gap=0.26,
            negative_elem="wing_body",
            name="right forward propeller clears airframe in hover",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
