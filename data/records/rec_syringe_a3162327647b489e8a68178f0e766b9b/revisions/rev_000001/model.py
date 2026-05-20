from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_OUTER_RADIUS = 0.0081
BODY_INNER_RADIUS = 0.00695
BODY_FRONT_Z = -0.065
TIP_Z = -0.082
REAR_FACE_Z = 0.0
PLUNGER_STROKE = 0.045


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
):
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, z0),
            (outer_radius, z1),
        ],
        [
            (inner_radius, z0),
            (inner_radius, z1),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _barrel_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.00155, TIP_Z),
            (0.00175, -0.079),
            (0.00270, -0.073),
            (BODY_OUTER_RADIUS, BODY_FRONT_Z),
            (BODY_OUTER_RADIUS, -0.006),
            (0.00860, -0.002),
            (0.00860, REAR_FACE_Z),
        ],
        [
            (0.00070, TIP_Z),
            (0.00085, -0.079),
            (0.00125, -0.073),
            (BODY_INNER_RADIUS, BODY_FRONT_Z),
            (BODY_INNER_RADIUS, -0.002),
            (0.00720, REAR_FACE_Z),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_syringe")

    clear_poly = model.material("clear_poly", rgba=(0.82, 0.87, 0.94, 0.34))
    milky_poly = model.material("milky_poly", rgba=(0.92, 0.93, 0.93, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.16, 0.18, 0.20, 1.0))
    print_ink = model.material("print_ink", rgba=(0.18, 0.24, 0.28, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        _save_mesh("barrel_shell", _barrel_shell_mesh()),
        material=clear_poly,
        name="barrel_shell",
    )
    barrel.visual(
        _save_mesh(
            "rear_collar",
            _ring_mesh(
                outer_radius=0.00920,
                inner_radius=0.00720,
                z0=-0.0018,
                z1=REAR_FACE_Z,
            ),
        ),
        material=clear_poly,
        name="rear_collar",
    )
    barrel.visual(
        _save_mesh(
            "front_stop_ring",
            _ring_mesh(
                outer_radius=BODY_INNER_RADIUS,
                inner_radius=0.00135,
                z0=BODY_FRONT_Z,
                z1=BODY_FRONT_Z + 0.0012,
            ),
        ),
        material=clear_poly,
        name="front_stop_ring",
    )
    barrel.visual(
        Box((0.0130, 0.0070, 0.0018)),
        origin=Origin(xyz=(-0.0115, 0.0, -0.0009)),
        material=clear_poly,
        name="finger_flange_left",
    )
    barrel.visual(
        Box((0.0130, 0.0070, 0.0018)),
        origin=Origin(xyz=(0.0115, 0.0, -0.0009)),
        material=clear_poly,
        name="finger_flange_right",
    )
    barrel.visual(
        Box((0.00035, 0.0010, 0.0400)),
        origin=Origin(xyz=(BODY_OUTER_RADIUS - 0.00015, 0.0, -0.036)),
        material=print_ink,
        name="graduation_spine",
    )
    for index, z_pos in enumerate((-0.014, -0.020, -0.026, -0.032, -0.038, -0.044, -0.050, -0.056)):
        barrel.visual(
            Box((0.00030, 0.0052 if index % 2 == 0 else 0.0032, 0.00075)),
            origin=Origin(xyz=(BODY_OUTER_RADIUS - 0.00015, 0.0, z_pos)),
            material=print_ink,
            name=f"grad_mark_{index:02d}",
        )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0105, length=0.084),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
    )

    retainer = model.part("retainer")
    retainer.visual(
        _save_mesh(
            "retainer_plate",
            _ring_mesh(
                outer_radius=0.01030,
                inner_radius=0.00360,
                z0=REAR_FACE_Z,
                z1=0.0024,
            ),
        ),
        material=milky_poly,
        name="retainer_plate",
    )
    retainer.visual(
        _save_mesh(
            "retainer_skirt",
            _ring_mesh(
                outer_radius=0.01030,
                inner_radius=0.00935,
                z0=REAR_FACE_Z,
                z1=0.0024,
            ),
        ),
        material=milky_poly,
        name="retainer_skirt",
    )
    retainer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0103, length=0.0024),
        mass=0.001,
        origin=Origin(xyz=(0.0, 0.0, 0.0012)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0132, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0515)),
        material=milky_poly,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0021, length=0.0660),
        origin=Origin(xyz=(0.0, 0.0, 0.0200)),
        material=milky_poly,
        name="guide_stem",
    )
    plunger.visual(
        Box((0.0054, 0.0010, 0.0340)),
        origin=Origin(xyz=(0.0, 0.0, 0.0250)),
        material=milky_poly,
        name="web_x",
    )
    plunger.visual(
        Box((0.0010, 0.0054, 0.0340)),
        origin=Origin(xyz=(0.0, 0.0, 0.0250)),
        material=milky_poly,
        name="web_y",
    )
    plunger.visual(
        Cylinder(radius=0.0052, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, -0.0010)),
        material=milky_poly,
        name="stop_flange",
    )
    plunger.visual(
        Cylinder(radius=0.0030, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, -0.0100)),
        material=milky_poly,
        name="seal_post",
    )
    plunger.visual(
        Cylinder(radius=0.00682, length=0.0076),
        origin=Origin(xyz=(0.0, 0.0, -0.0162)),
        material=seal_rubber,
        name="seal_body",
    )
    plunger.visual(
        Cylinder(radius=0.00690, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, -0.0193)),
        material=seal_rubber,
        name="seal_front_rib",
    )
    plunger.visual(
        Cylinder(radius=0.00690, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, -0.0127)),
        material=seal_rubber,
        name="seal_rear_rib",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.0264, 0.0264, 0.074)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    model.articulation(
        "barrel_to_retainer",
        ArticulationType.FIXED,
        parent=barrel,
        child=retainer,
        origin=Origin(),
    )
    plunger_slide = model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, REAR_FACE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=PLUNGER_STROKE,
        ),
    )
    plunger_slide.meta["design_intent"] = "Positive stroke pushes the plunger toward the nozzle."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    retainer = object_model.get_part("retainer")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

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

    ctx.expect_contact(
        retainer,
        barrel,
        elem_a="retainer_plate",
        elem_b="rear_collar",
        name="retainer_is_seated_on_barrel_collar",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_origin_distance(
            barrel,
            plunger,
            axes="xy",
            max_dist=1e-6,
            name="plunger_axis_is_coaxial_when_withdrawn",
        )
        ctx.expect_contact(
            plunger,
            retainer,
            elem_a="stop_flange",
            elem_b="retainer_plate",
            name="withdrawn_stop_hits_retainer",
        )

    with ctx.pose({slide: PLUNGER_STROKE}):
        ctx.expect_origin_distance(
            barrel,
            plunger,
            axes="xy",
            max_dist=1e-6,
            name="plunger_axis_stays_coaxial_when_inserted",
        )
        ctx.expect_contact(
            plunger,
            barrel,
            elem_a="seal_front_rib",
            elem_b="front_stop_ring",
            name="inserted_stop_hits_front_shoulder",
        )
        ctx.expect_gap(
            plunger,
            retainer,
            axis="z",
            positive_elem="thumb_pad",
            negative_elem="retainer_plate",
            min_gap=0.0015,
            max_gap=0.0060,
            name="thumb_pad_remains_outside_retainer_at_full_insert",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
