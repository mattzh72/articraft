from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_HEIGHT = 0.06
COLUMN_HEIGHT = 0.78
HOUSING_HEIGHT = 0.12
STATOR_HEIGHT = 0.035
JOINT_Z = BASE_HEIGHT + COLUMN_HEIGHT + HOUSING_HEIGHT + STATOR_HEIGHT

RAIL_LENGTH = 0.84
RAIL_CENTER_X = 0.69
RAIL_Y = 0.075
RAIL_TOP_Z = 0.17
SLIDE_ORIGIN_X = 0.38
SLIDE_TRAVEL = 0.62


def fuse_all(base: cq.Workplane, *others: cq.Workplane) -> cq.Workplane:
    result = base
    for other in others:
        result = result.union(other)
    return result


def polar_points(radius: float, count: int, start_angle: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(start_angle + (2.0 * pi * index / count)),
            radius * sin(start_angle + (2.0 * pi * index / count)),
        )
        for index in range(count)
    ]


def annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(height)
    return outer.cut(inner)


def make_mast_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").circle(0.31).extrude(BASE_HEIGHT)
    lower_pedestal = cq.Workplane("XY").circle(0.20).extrude(0.12).translate((0.0, 0.0, BASE_HEIGHT))
    column = cq.Workplane("XY").circle(0.11).extrude(COLUMN_HEIGHT).translate((0.0, 0.0, BASE_HEIGHT))
    upper_housing = cq.Workplane("XY").circle(0.18).extrude(HOUSING_HEIGHT).translate(
        (0.0, 0.0, BASE_HEIGHT + COLUMN_HEIGHT)
    )
    stator_body = annulus(0.24, 0.135, STATOR_HEIGHT).translate(
        (0.0, 0.0, BASE_HEIGHT + COLUMN_HEIGHT + HOUSING_HEIGHT)
    )
    stator_register = annulus(0.205, 0.155, 0.012).translate(
        (0.0, 0.0, BASE_HEIGHT + COLUMN_HEIGHT + HOUSING_HEIGHT)
    )
    top_cap = cq.Workplane("XY").circle(0.14).extrude(0.018).translate(
        (0.0, 0.0, BASE_HEIGHT + COLUMN_HEIGHT + HOUSING_HEIGHT - 0.018)
    )

    rib = cq.Workplane("XY").box(0.06, 0.022, 0.64).translate((0.145, 0.0, 0.46))
    ribs = []
    for angle in (0.0, 90.0, 180.0, 270.0):
        ribs.append(rib.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle))

    mast = fuse_all(base_plate, lower_pedestal, column, upper_housing, stator_body, stator_register, top_cap, *ribs)

    access_flat = cq.Workplane("XY").box(0.13, 0.09, 0.20).translate((0.0, -0.16, 0.32))
    mast = mast.cut(access_flat)

    bolt_pad = cq.Workplane("XY").circle(0.018).extrude(0.008).translate(
        (0.19, 0.0, BASE_HEIGHT + COLUMN_HEIGHT + HOUSING_HEIGHT + STATOR_HEIGHT - 0.008)
    )
    bolt_pads = [bolt_pad.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle) for angle in range(0, 360, 45)]
    mast = fuse_all(mast, *bolt_pads)

    return mast


def make_beam_body() -> cq.Workplane:
    rotor_ring = annulus(0.225, 0.145, 0.028)
    rotor_register = annulus(0.19, 0.155, 0.012).translate((0.0, 0.0, 0.028))

    root_pedestal = cq.Workplane("XY").box(0.32, 0.24, 0.11).translate((0.11, 0.0, 0.083))
    main_girder = cq.Workplane("XY").box(0.98, 0.28, 0.10).translate((0.73, 0.0, 0.10))
    tip_cap = cq.Workplane("XY").box(0.08, 0.20, 0.07).translate((1.18, 0.0, 0.115))

    shoulder_left = cq.Workplane("XY").box(0.92, 0.028, 0.012).translate((0.72, 0.118, 0.156))
    shoulder_right = cq.Workplane("XY").box(0.92, 0.028, 0.012).translate((0.72, -0.118, 0.156))
    stop_inner = cq.Workplane("XY").box(0.03, 0.08, 0.010).translate((0.26, 0.0, 0.155))
    stop_outer = cq.Workplane("XY").box(0.03, 0.08, 0.010).translate((1.12, 0.0, 0.155))

    gusset_left = cq.Workplane("XY").box(0.42, 0.018, 0.075).translate((0.24, 0.09, 0.068))
    gusset_right = cq.Workplane("XY").box(0.42, 0.018, 0.075).translate((0.24, -0.09, 0.068))

    beam = fuse_all(
        rotor_ring,
        rotor_register,
        root_pedestal,
        main_girder,
        tip_cap,
        shoulder_left,
        shoulder_right,
        stop_inner,
        stop_outer,
        gusset_left,
        gusset_right,
    )

    underside_pocket = cq.Workplane("XY").box(0.74, 0.16, 0.05).translate((0.79, 0.0, 0.08))
    top_trough = cq.Workplane("XY").box(0.88, 0.06, 0.010).translate((0.70, 0.0, 0.145))
    mast_clearance = cq.Workplane("XY").box(0.24, 0.16, 0.12).translate((0.02, 0.0, 0.10))

    beam = beam.cut(underside_pocket).cut(top_trough).cut(mast_clearance)

    return beam


def make_truck_body() -> cq.Workplane:
    base_plate = cq.Workplane("XY").box(0.30, 0.22, 0.030).translate((0.0, 0.0, 0.033))
    upper_housing = cq.Workplane("XY").box(0.24, 0.18, 0.062).translate((0.0, 0.0, 0.079))
    end_cap_inner = cq.Workplane("XY").box(0.035, 0.20, 0.055).translate((-0.1325, 0.0, 0.0755))
    end_cap_outer = cq.Workplane("XY").box(0.035, 0.20, 0.055).translate((0.1325, 0.0, 0.0755))
    cover_left = cq.Workplane("XY").box(0.26, 0.018, 0.022).translate((0.0, 0.091, 0.029))
    cover_right = cq.Workplane("XY").box(0.26, 0.018, 0.022).translate((0.0, -0.091, 0.029))
    service_cover = cq.Workplane("XY").box(0.16, 0.11, 0.018).translate((0.0, 0.0, 0.119))
    top_mount = cq.Workplane("XY").box(0.12, 0.06, 0.012).translate((0.0, 0.0, 0.134))

    truck = fuse_all(
        base_plate,
        upper_housing,
        end_cap_inner,
        end_cap_outer,
        cover_left,
        cover_right,
        service_cover,
        top_mount,
    )

    cover_relief = cq.Workplane("XY").box(0.10, 0.05, 0.010).translate((0.0, 0.0, 0.126))
    truck = truck.cut(cover_relief)

    return truck


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_transfer_axis")

    mast_material = model.material("mast_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    beam_material = model.material("beam_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    guide_material = model.material("guide_steel", rgba=(0.46, 0.49, 0.53, 1.0))
    truck_material = model.material("truck_orange", rgba=(0.91, 0.52, 0.16, 1.0))

    mast = model.part("mast")
    mast.visual(mesh_from_cadquery(make_mast_shape(), "mast_body"), material=mast_material, name="mast_body")
    mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.31, length=JOINT_Z),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z / 2.0)),
    )

    beam = model.part("beam")
    beam.visual(mesh_from_cadquery(make_beam_body(), "beam_body"), material=beam_material, name="beam_body")
    beam.visual(
        Box((RAIL_LENGTH, 0.04, 0.02)),
        origin=Origin(xyz=(RAIL_CENTER_X, RAIL_Y, 0.16)),
        material=guide_material,
        name="rail_left",
    )
    beam.visual(
        Box((RAIL_LENGTH, 0.04, 0.02)),
        origin=Origin(xyz=(RAIL_CENTER_X, -RAIL_Y, 0.16)),
        material=guide_material,
        name="rail_right",
    )
    beam.inertial = Inertial.from_geometry(
        Box((1.26, 0.28, 0.20)),
        mass=95.0,
        origin=Origin(xyz=(0.63, 0.0, 0.10)),
    )

    truck = model.part("truck")
    truck.visual(mesh_from_cadquery(make_truck_body(), "truck_body"), material=truck_material, name="truck_body")
    truck.visual(
        Box((0.18, 0.04, 0.018)),
        origin=Origin(xyz=(0.0, RAIL_Y, 0.009)),
        material=guide_material,
        name="runner_left",
    )
    truck.visual(
        Box((0.18, 0.04, 0.018)),
        origin=Origin(xyz=(0.0, -RAIL_Y, 0.009)),
        material=guide_material,
        name="runner_right",
    )
    truck.inertial = Inertial.from_geometry(
        Box((0.30, 0.22, 0.14)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
    )

    model.articulation(
        "mast_to_beam",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=1.2,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "beam_to_truck",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=truck,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=1.0,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    beam = object_model.get_part("beam")
    truck = object_model.get_part("truck")
    swing = object_model.get_articulation("mast_to_beam")
    slide = object_model.get_articulation("beam_to_truck")

    rail_left = beam.get_visual("rail_left")
    rail_right = beam.get_visual("rail_right")
    runner_left = truck.get_visual("runner_left")
    runner_right = truck.get_visual("runner_right")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=28)

    ctx.expect_contact(beam, mast, name="beam slewing stage stays seated on mast")
    ctx.expect_contact(truck, beam, elem_a=runner_left, elem_b=rail_left, name="left runner seats on left rail")
    ctx.expect_contact(truck, beam, elem_a=runner_right, elem_b=rail_right, name="right runner seats on right rail")

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            truck,
            beam,
            axes="xy",
            inner_elem=runner_left,
            outer_elem=rail_left,
            margin=0.0,
            name="left runner stays supported at inner stroke",
        )
        ctx.expect_within(
            truck,
            beam,
            axes="xy",
            inner_elem=runner_right,
            outer_elem=rail_right,
            margin=0.0,
            name="right runner stays supported at inner stroke",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            truck,
            beam,
            axes="xy",
            inner_elem=runner_left,
            outer_elem=rail_left,
            margin=0.0,
            name="left runner stays supported at outer stroke",
        )
        ctx.expect_within(
            truck,
            beam,
            axes="xy",
            inner_elem=runner_right,
            outer_elem=rail_right,
            margin=0.0,
            name="right runner stays supported at outer stroke",
        )

    with ctx.pose({swing: swing.motion_limits.lower, slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            truck,
            beam,
            elem_a=runner_left,
            elem_b=rail_left,
            name="left runner keeps rail contact at negative swing limit",
        )

    with ctx.pose({swing: swing.motion_limits.upper, slide: 0.0}):
        ctx.expect_contact(
            truck,
            beam,
            elem_a=runner_right,
            elem_b=rail_right,
            name="right runner keeps rail contact at positive swing limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
