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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    radial_segments: int = 72,
):
    return boolean_difference(
        CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments),
        CylinderGeometry(
            radius=inner_radius,
            height=height + 0.002,
            radial_segments=radial_segments,
        ),
    )


def _rounded_plate_mesh(width: float, depth: float, thickness: float, radius: float):
    return ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius=radius, corner_segments=8),
        thickness,
        center=True,
        cap=True,
    )


def _build_nozzle_shell():
    outer = LatheGeometry(
        [
            (0.0, 0.000),
            (0.00155, 0.000),
            (0.00170, 0.007),
            (0.00210, 0.015),
            (0.00330, 0.023),
            (0.00550, 0.031),
            (0.00860, 0.036),
            (0.00920, 0.040),
            (0.0, 0.040),
        ],
        segments=72,
    )
    bore = LatheGeometry(
        [
            (0.0, -0.002),
            (0.00045, -0.002),
            (0.00055, 0.012),
            (0.00090, 0.022),
            (0.00210, 0.031),
            (0.00620, 0.041),
            (0.00800, 0.043),
            (0.0, 0.043),
        ],
        segments=72,
    )
    return boolean_difference(outer, bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_syringe")

    clear_polymer = model.material("clear_polymer", rgba=(0.82, 0.90, 0.97, 0.30))
    satin_metal = model.material("satin_metal", rgba=(0.69, 0.72, 0.76, 1.0))
    painted_metal = model.material("painted_metal", rgba=(0.78, 0.80, 0.83, 1.0))
    warm_polymer = model.material("warm_polymer", rgba=(0.92, 0.93, 0.94, 1.0))
    graphite_ink = model.material("graphite_ink", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_elastomer = model.material("dark_elastomer", rgba=(0.12, 0.13, 0.14, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        _save_mesh("syringe_nozzle_shell", _build_nozzle_shell()),
        material=clear_polymer,
        name="nozzle_shell",
    )
    barrel.visual(
        _save_mesh(
            "syringe_barrel_tube",
            _ring_mesh(outer_radius=0.0092, inner_radius=0.0080, height=0.072),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=clear_polymer,
        name="barrel_tube",
    )
    barrel.visual(
        _save_mesh(
            "syringe_nozzle_accent",
            _ring_mesh(outer_radius=0.0097, inner_radius=0.0082, height=0.0032),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=satin_metal,
        name="nozzle_accent",
    )
    barrel.visual(
        _save_mesh(
            "syringe_rear_guide",
            _ring_mesh(outer_radius=0.0112, inner_radius=0.00745, height=0.008),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=clear_polymer,
        name="rear_guide",
    )

    flange_mesh = _save_mesh(
        "syringe_finger_flange",
        _rounded_plate_mesh(0.024, 0.011, 0.003, radius=0.004),
    )
    barrel.visual(
        flange_mesh,
        origin=Origin(xyz=(0.0165, 0.0, 0.1045)),
        material=clear_polymer,
        name="finger_flange_right",
    )
    barrel.visual(
        flange_mesh,
        origin=Origin(xyz=(-0.0165, 0.0, 0.1045)),
        material=clear_polymer,
        name="finger_flange_left",
    )

    graduation_z0 = 0.048
    graduation_pitch = 0.005
    for index in range(11):
        z = graduation_z0 + graduation_pitch * index
        major_len = 0.010 if index < 10 else 0.012
        barrel.visual(
            Box((major_len, 0.00045, 0.00075)),
            origin=Origin(xyz=(0.0, 0.00912, z)),
            material=graphite_ink,
            name=f"major_tick_{index:02d}",
        )
        if index < 10:
            for minor in range(1, 5):
                barrel.visual(
                    Box((0.0055 if minor == 2 else 0.0040, 0.00040, 0.00050)),
                    origin=Origin(
                        xyz=(
                            0.0,
                            0.00908,
                            z + graduation_pitch * minor / 5.0,
                        )
                    ),
                    material=graphite_ink,
                    name=f"minor_tick_{index:02d}_{minor}",
                )

    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0112, length=0.112),
        mass=0.042,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.00255, length=0.136),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=painted_metal,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.00770, length=0.0080),
        origin=Origin(xyz=(0.0, 0.0, -0.0620)),
        material=dark_elastomer,
        name="stopper_body",
    )
    plunger.visual(
        Cylinder(radius=0.00805, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, -0.0660)),
        material=dark_elastomer,
        name="stopper_front_rib",
    )
    plunger.visual(
        Cylinder(radius=0.00805, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, -0.0580)),
        material=dark_elastomer,
        name="stopper_rear_rib",
    )
    plunger.visual(
        Cylinder(radius=0.0054, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=warm_polymer,
        name="thumb_hub",
    )
    plunger.visual(
        _save_mesh(
            "syringe_thumb_plate",
            _rounded_plate_mesh(0.034, 0.018, 0.004, radius=0.005),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=warm_polymer,
        name="thumb_plate",
    )
    plunger.visual(
        _save_mesh(
            "syringe_thumb_pad",
            _rounded_plate_mesh(0.028, 0.014, 0.0026, radius=0.004),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0849)),
        material=dark_elastomer,
        name="thumb_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.034, 0.018, 0.154)),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.20,
            lower=0.0,
            upper=0.049,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slider = object_model.get_articulation("barrel_to_plunger")
    nozzle_shell = barrel.get_visual("nozzle_shell")
    barrel_tube = barrel.get_visual("barrel_tube")
    rear_guide = barrel.get_visual("rear_guide")
    stopper_body = plunger.get_visual("stopper_body")
    stopper_front_rib = plunger.get_visual("stopper_front_rib")
    stopper_rear_rib = plunger.get_visual("stopper_rear_rib")
    plunger_rod = plunger.get_visual("plunger_rod")
    thumb_pad = plunger.get_visual("thumb_pad")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        plunger,
        barrel,
        elem_a=stopper_front_rib,
        elem_b=barrel_tube,
        reason="Elastomer front seal rib intentionally compresses slightly against the barrel bore.",
    )
    ctx.allow_overlap(
        plunger,
        barrel,
        elem_a=stopper_rear_rib,
        elem_b=barrel_tube,
        reason="Elastomer rear seal rib intentionally compresses slightly against the barrel bore.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "plunger_joint_is_prismatic",
        slider.articulation_type == ArticulationType.PRISMATIC,
        details=f"expected PRISMATIC articulation, got {slider.articulation_type}",
    )
    ctx.check(
        "plunger_joint_axis_is_coaxial",
        tuple(round(value, 6) for value in slider.axis) == (0.0, 0.0, 1.0),
        details=f"expected axis (0, 0, 1), got {slider.axis}",
    )

    closed_origin_z = None
    open_origin_z = None
    with ctx.pose({slider: 0.0}):
        closed_origin = ctx.part_world_position(plunger)
        if closed_origin is not None:
            closed_origin_z = closed_origin[2]
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="xy",
            elem_a=stopper_body,
            elem_b=barrel_tube,
            min_overlap=0.014,
            name="stopper_stays_coaxial_in_barrel",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="xy",
            elem_a=plunger_rod,
            elem_b=rear_guide,
            min_overlap=0.0045,
            name="rod_runs_through_rear_guide",
        )
        ctx.expect_gap(
            plunger,
            barrel,
            axis="z",
            positive_elem=stopper_front_rib,
            negative_elem=nozzle_shell,
            min_gap=0.0,
            max_gap=0.003,
            name="closed_stopper_seats_near_nozzle",
        )

    upper = slider.motion_limits.upper if slider.motion_limits is not None else None
    if upper is None:
        ctx.fail("plunger_has_motion_limits", "plunger articulation is missing an upper limit")
    else:
        with ctx.pose({slider: upper}):
            open_origin = ctx.part_world_position(plunger)
            if open_origin is not None:
                open_origin_z = open_origin[2]
            ctx.expect_gap(
                barrel,
                plunger,
                axis="z",
                positive_elem=rear_guide,
                negative_elem=stopper_rear_rib,
                min_gap=0.0,
                max_gap=0.003,
                name="rear_guide_retains_plunger_stopper",
            )
            ctx.expect_gap(
                plunger,
                barrel,
                axis="z",
                positive_elem=thumb_pad,
                negative_elem=rear_guide,
                min_gap=0.008,
                name="thumb_pad_remains_accessible_when_retracted",
            )

    ctx.check(
        "plunger_moves_outward_on_positive_travel",
        closed_origin_z is not None
        and open_origin_z is not None
        and open_origin_z > closed_origin_z + 0.040,
        details=(
            f"expected positive-z extension greater than 0.040 m, "
            f"got closed={closed_origin_z}, open={open_origin_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
