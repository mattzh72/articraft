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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_plate_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    radius: float,
    thickness: float,
):
    return _mesh(
        name,
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, depth, radius, corner_segments=10),
            thickness,
            cap=True,
            closed=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_swivel_bar_stool")

    powder_black = model.material("powder_black", rgba=(0.14, 0.15, 0.16, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.08, 0.08, 0.09, 1.0))
    index_white = model.material("index_white", rgba=(0.92, 0.93, 0.94, 1.0))
    signal_amber = model.material("signal_amber", rgba=(0.90, 0.60, 0.14, 1.0))

    footrest_ring_mesh = _mesh(
        "footrest_ring",
        TorusGeometry(
            radius=0.170,
            tube=0.012,
            radial_segments=18,
            tubular_segments=64,
        ),
    )
    seat_plate_mesh = _rounded_plate_mesh(
        "seat_plate",
        width=0.360,
        depth=0.310,
        radius=0.040,
        thickness=0.010,
    )
    seat_pad_mesh = _rounded_plate_mesh(
        "seat_pad",
        width=0.300,
        depth=0.260,
        radius=0.055,
        thickness=0.036,
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.230, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=powder_black,
        name="base_disc",
    )
    pedestal.visual(
        Cylinder(radius=0.150, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=powder_black,
        name="base_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.046, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=machined_steel,
        name="pedestal_column",
    )
    pedestal.visual(
        footrest_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=machined_steel,
        name="footrest_ring",
    )
    for index in range(3):
        pedestal.visual(
            Box((0.135, 0.030, 0.016)),
            origin=Origin(
                xyz=(0.103, 0.0, 0.315),
                rpy=(0.0, 0.0, index * math.tau / 3.0),
            ),
            material=machined_steel,
            name=f"footrest_bridge_{index:02d}",
        )
    pedestal.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=satin_aluminum,
        name="preload_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(
            xyz=(0.059, 0.0, 0.620),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined_steel,
        name="drag_adjust_shaft",
    )
    pedestal.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(
            xyz=(0.079, 0.0, 0.620),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_black,
        name="drag_adjust_knob",
    )
    pedestal.visual(
        Cylinder(radius=0.072, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.655)),
        material=satin_aluminum,
        name="bearing_body",
    )
    pedestal.visual(
        Cylinder(radius=0.104, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.679)),
        material=satin_aluminum,
        name="datum_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.060, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.6865)),
        material=machined_steel,
        name="thrust_washer",
    )
    for index in range(24):
        major = index % 6 == 0
        mark_size = (0.020, 0.004, 0.0020) if major else (0.011, 0.0025, 0.0012)
        mark_center_z = 0.683 - (mark_size[2] * 0.5) + 0.0002
        pedestal.visual(
            Box(mark_size),
            origin=Origin(
                xyz=(0.084, 0.0, mark_center_z),
                rpy=(0.0, 0.0, index * math.tau / 24.0),
            ),
            material=index_white,
            name=f"index_mark_{index:02d}",
        )
    pedestal.visual(
        Box((0.026, 0.010, 0.003)),
        origin=Origin(xyz=(0.094, 0.0, 0.6835)),
        material=index_white,
        name="zero_datum_block",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.230, length=0.690),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
    )

    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        Cylinder(radius=0.068, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=machined_steel,
        name="rotor_plate",
    )
    seat_stage.visual(
        Cylinder(radius=0.112, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=powder_black,
        name="rotating_shroud",
    )
    seat_stage.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=satin_aluminum,
        name="seat_spindle",
    )
    for index in range(3):
        seat_stage.visual(
            Box((0.138, 0.024, 0.014)),
            origin=Origin(
                xyz=(0.093, 0.0, 0.054),
                rpy=(0.0, 0.0, index * math.tau / 3.0),
            ),
            material=satin_aluminum,
            name=f"stage_rib_{index:02d}",
        )
    seat_stage.visual(
        seat_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=satin_aluminum,
        name="seat_plate",
    )
    seat_stage.visual(
        seat_pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=cushion_black,
        name="seat_pad",
    )
    seat_stage.visual(
        Box((0.012, 0.180, 0.0015)),
        origin=Origin(xyz=(0.164, 0.0, 0.0658)),
        material=index_white,
        name="front_centerline_strip",
    )
    seat_stage.visual(
        Box((0.040, 0.012, 0.0015)),
        origin=Origin(xyz=(0.160, 0.0, 0.0658)),
        material=index_white,
        name="front_datum_strip",
    )
    seat_stage.visual(
        Box((0.022, 0.006, 0.006)),
        origin=Origin(xyz=(0.108, 0.0, 0.010)),
        material=signal_amber,
        name="index_pointer",
    )
    seat_stage.visual(
        Box((0.010, 0.006, 0.010)),
        origin=Origin(xyz=(0.120, 0.0, 0.007)),
        material=signal_amber,
        name="pointer_nose",
    )
    seat_stage.inertial = Inertial.from_geometry(
        Box((0.360, 0.310, 0.102)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat_stage = object_model.get_part("seat_stage")
    swivel = object_model.get_articulation("seat_swivel")

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

    ctx.expect_origin_gap(
        seat_stage,
        pedestal,
        axis="z",
        min_gap=0.689,
        max_gap=0.691,
        name="seat_height_origin_is_bar_stool_scale",
    )
    ctx.expect_origin_distance(
        seat_stage,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="seat_swivel_is_centered_on_pedestal",
    )
    ctx.expect_contact(
        seat_stage,
        pedestal,
        elem_a="rotor_plate",
        elem_b="thrust_washer",
        contact_tol=0.0005,
        name="rotor_plate_bears_on_supported_washer",
    )
    ctx.expect_overlap(
        seat_stage,
        pedestal,
        axes="xy",
        elem_a="rotor_plate",
        elem_b="bearing_body",
        min_overlap=0.120,
        name="swivel_stack_has_concentric_support_overlap",
    )
    ctx.expect_gap(
        seat_stage,
        pedestal,
        axis="z",
        positive_elem="rotating_shroud",
        negative_elem="datum_ring",
        min_gap=0.014,
        max_gap=0.017,
        name="controlled_gap_between_shroud_and_index_ring",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_contact(
            seat_stage,
            pedestal,
            elem_a="rotor_plate",
            elem_b="thrust_washer",
            contact_tol=0.0005,
            name="swivel_support_contact_persists_when_rotated",
        )
        ctx.expect_origin_distance(
            seat_stage,
            pedestal,
            axes="xy",
            max_dist=0.001,
            name="rotated_seat_remains_coaxial",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
