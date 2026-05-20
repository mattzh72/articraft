from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_FOOT_OUTER_RADIUS = 0.115
BASE_FOOT_INNER_RADIUS = 0.086
BASE_FOOT_HEIGHT = 0.008

BASE_BODY_RADIUS = 0.103
BASE_BODY_HEIGHT = 0.104
BASE_BODY_Z0 = BASE_FOOT_HEIGHT

BASE_SHOULDER_RADIUS = 0.085
BASE_SHOULDER_HEIGHT = 0.010
BASE_SHOULDER_Z0 = BASE_BODY_Z0 + BASE_BODY_HEIGHT

BEARING_COLLAR_OUTER_RADIUS = 0.054
BEARING_COLLAR_INNER_RADIUS = 0.034
BEARING_COLLAR_HEIGHT = 0.014
BEARING_COLLAR_Z0 = BASE_SHOULDER_Z0 + BASE_SHOULDER_HEIGHT

ROTOR_HUB_RADIUS = 0.048
ROTOR_HUB_HEIGHT = 0.010
ROTOR_HUB_Z0 = BEARING_COLLAR_Z0 + BEARING_COLLAR_HEIGHT

SPINDLE_RADIUS = 0.030
SPINDLE_HEIGHT = 0.050
SPINDLE_Z0 = ROTOR_HUB_Z0 + ROTOR_HUB_HEIGHT

TOP_BOSS_RADIUS = 0.055
TOP_BOSS_HEIGHT = 0.010
TOP_BOSS_Z0 = SPINDLE_Z0 + SPINDLE_HEIGHT

PLATE_REINFORCEMENT_RADIUS = 0.086
PLATE_REINFORCEMENT_HEIGHT = 0.006
PLATE_REINFORCEMENT_Z0 = TOP_BOSS_Z0 + TOP_BOSS_HEIGHT

TOP_PLATE_RADIUS = 0.210
TOP_PLATE_HEIGHT = 0.017
TOP_PLATE_Z0 = PLATE_REINFORCEMENT_Z0 + PLATE_REINFORCEMENT_HEIGHT

WORK_PAD_RADIUS = 0.185
WORK_PAD_HEIGHT = 0.003
WORK_PAD_Z0 = TOP_PLATE_Z0 + TOP_PLATE_HEIGHT


def cylinder_at(radius: float, height: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def ring_at(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="labeling_platen")

    anodized_dark = model.material("anodized_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    machined_metal = model.material("machined_metal", rgba=(0.77, 0.79, 0.81, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))

    base_housing_shape = (
        cylinder_at(BASE_BODY_RADIUS, BASE_BODY_HEIGHT, BASE_BODY_Z0).union(
            cylinder_at(BASE_SHOULDER_RADIUS, BASE_SHOULDER_HEIGHT, BASE_SHOULDER_Z0)
        )
    )
    foot_ring_shape = ring_at(
        BASE_FOOT_OUTER_RADIUS,
        BASE_FOOT_INNER_RADIUS,
        BASE_FOOT_HEIGHT,
        0.0,
    )
    bearing_collar_shape = ring_at(
        BEARING_COLLAR_OUTER_RADIUS,
        BEARING_COLLAR_INNER_RADIUS,
        BEARING_COLLAR_HEIGHT,
        BEARING_COLLAR_Z0,
    )

    platen_z_shift = ROTOR_HUB_Z0
    rotor_body_shape = (
        cylinder_at(ROTOR_HUB_RADIUS, ROTOR_HUB_HEIGHT, ROTOR_HUB_Z0 - platen_z_shift)
        .union(cylinder_at(SPINDLE_RADIUS, SPINDLE_HEIGHT, SPINDLE_Z0 - platen_z_shift))
        .union(cylinder_at(TOP_BOSS_RADIUS, TOP_BOSS_HEIGHT, TOP_BOSS_Z0 - platen_z_shift))
    )
    top_plate_shape = (
        cylinder_at(
            PLATE_REINFORCEMENT_RADIUS,
            PLATE_REINFORCEMENT_HEIGHT,
            PLATE_REINFORCEMENT_Z0 - platen_z_shift,
        )
        .union(
            cylinder_at(
                TOP_PLATE_RADIUS,
                TOP_PLATE_HEIGHT,
                TOP_PLATE_Z0 - platen_z_shift,
            )
        )
    )
    work_pad_shape = cylinder_at(
        WORK_PAD_RADIUS,
        WORK_PAD_HEIGHT,
        WORK_PAD_Z0 - platen_z_shift,
    )

    base = model.part("base_housing")
    base.visual(
        mesh_from_cadquery(base_housing_shape, "base_housing_shell"),
        material=anodized_dark,
        name="housing_shell",
    )
    base.visual(
        mesh_from_cadquery(foot_ring_shape, "base_foot_ring"),
        material=rubber_black,
        name="foot_ring",
    )
    base.visual(
        mesh_from_cadquery(bearing_collar_shape, "base_bearing_collar"),
        material=machined_metal,
        name="bearing_collar",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_FOOT_OUTER_RADIUS, length=BEARING_COLLAR_Z0 + BEARING_COLLAR_HEIGHT),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, (BEARING_COLLAR_Z0 + BEARING_COLLAR_HEIGHT) / 2.0)),
    )

    platen = model.part("rotating_platen")
    platen.visual(
        mesh_from_cadquery(rotor_body_shape, "rotating_platen_rotor_body"),
        material=machined_metal,
        name="rotor_body",
    )
    platen.visual(
        mesh_from_cadquery(top_plate_shape, "rotating_platen_top_plate"),
        material=machined_metal,
        name="top_plate",
    )
    platen.visual(
        mesh_from_cadquery(work_pad_shape, "rotating_platen_work_pad"),
        material=rubber_black,
        name="work_pad",
    )
    platen.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_PLATE_RADIUS, length=WORK_PAD_Z0 + WORK_PAD_HEIGHT - ROTOR_HUB_Z0),
        mass=3.2,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (WORK_PAD_Z0 + WORK_PAD_HEIGHT - ROTOR_HUB_Z0) / 2.0,
            )
        ),
    )

    model.articulation(
        "base_to_platen_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=platen,
        origin=Origin(xyz=(0.0, 0.0, ROTOR_HUB_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    platen = object_model.get_part("rotating_platen")
    spin = object_model.get_articulation("base_to_platen_spin")

    collar = base.get_visual("bearing_collar")
    rotor_body = platen.get_visual("rotor_body")
    top_plate = platen.get_visual("top_plate")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "parts_and_joint_exist",
        base is not None and platen is not None and spin is not None,
        details="Expected base_housing, rotating_platen, and base_to_platen_spin.",
    )
    ctx.check(
        "continuous_vertical_spin_axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(spin.axis[0]) < 1e-9
        and abs(spin.axis[1]) < 1e-9
        and abs(spin.axis[2] - 1.0) < 1e-9
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details="Platen should spin continuously about the vertical centerline.",
    )

    ctx.expect_contact(
        platen,
        base,
        elem_a=rotor_body,
        elem_b=collar,
        name="bearing_interface_contacts_base",
    )
    ctx.expect_gap(
        platen,
        base,
        axis="z",
        min_gap=0.068,
        max_gap=0.076,
        positive_elem=top_plate,
        name="top_plate_stands_well_above_compact_base",
    )
    ctx.expect_overlap(
        platen,
        base,
        axes="xy",
        min_overlap=0.20,
        elem_a=top_plate,
        name="top_plate_is_centered_over_base",
    )

    base_aabb = ctx.part_world_aabb(base)
    plate_aabb = ctx.part_element_world_aabb(platen, elem=top_plate.name)
    if base_aabb is not None and plate_aabb is not None:
        base_span_x = base_aabb[1][0] - base_aabb[0][0]
        plate_span_x = plate_aabb[1][0] - plate_aabb[0][0]
        ctx.check(
            "top_plate_dominates_silhouette",
            plate_span_x >= base_span_x + 0.16,
            details=(
                f"Expected plate diameter to exceed base diameter by at least 0.16 m; "
                f"got {plate_span_x:.3f} m vs {base_span_x:.3f} m."
            ),
        )

    with ctx.pose(base_to_platen_spin=1.7):
        ctx.expect_contact(
            platen,
            base,
            elem_a=rotor_body,
            elem_b=collar,
            name="bearing_contact_persists_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
