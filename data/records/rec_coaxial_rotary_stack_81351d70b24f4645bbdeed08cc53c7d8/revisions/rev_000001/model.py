from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def annulus(outer_radius: float, inner_radius: float, thickness: float):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(thickness)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(thickness + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return outer.cut(inner)


def make_rotor_frame(
    *,
    outer_radius: float,
    rim_width: float,
    hub_outer_radius: float,
    bore_radius: float,
    thickness: float,
    spoke_width: float,
    gap_width: float,
    pointer_length: float,
    pointer_width: float,
    spoke_angles_deg: tuple[float, ...],
):
    ring = annulus(outer_radius, outer_radius - rim_width, thickness)
    hub = annulus(hub_outer_radius, bore_radius, thickness)
    frame = ring.union(hub)

    spoke_span = (outer_radius - rim_width) - hub_outer_radius
    for angle_deg in spoke_angles_deg:
        spoke = (
            cq.Workplane("XY")
            .box(spoke_span, spoke_width, thickness)
            .translate((hub_outer_radius + 0.5 * spoke_span, 0.0, 0.5 * thickness))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        frame = frame.union(spoke)

    pointer = (
        cq.Workplane("XY")
        .box(pointer_length, pointer_width, thickness)
        .translate(
            (
                outer_radius + 0.5 * pointer_length - 0.25 * rim_width,
                0.0,
                0.5 * thickness,
            )
        )
    )
    frame = frame.union(pointer)

    gap_cutter = (
        cq.Workplane("XY")
        .box(gap_width, 1.8 * outer_radius, 2.0 * thickness)
        .translate(
            (
                -outer_radius + 0.35 * gap_width,
                0.0,
                0.5 * thickness,
            )
        )
    )
    frame = frame.cut(gap_cutter)

    return frame.translate((0.0, 0.0, -0.5 * thickness))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_coaxial_rotary_assembly")

    base_mat = model.material("base_black", rgba=(0.16, 0.17, 0.19, 1.0))
    steel_mat = model.material("shaft_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    inner_mat = model.material("inner_blue", rgba=(0.22, 0.39, 0.64, 1.0))
    middle_mat = model.material("middle_bronze", rgba=(0.63, 0.49, 0.29, 1.0))
    outer_mat = model.material("outer_silver", rgba=(0.80, 0.82, 0.84, 1.0))

    base_plate_size = 0.22
    base_plate_thickness = 0.018
    pedestal_radius = 0.032
    pedestal_height = 0.014

    shaft_radius = 0.013
    shaft_height = 0.255
    support_thickness = 0.008

    rotor_thickness = 0.014
    inner_z = 0.065
    middle_z = 0.129
    outer_z = 0.193

    inner_support_z = inner_z - 0.5 * rotor_thickness - 0.5 * support_thickness
    middle_support_z = middle_z - 0.5 * rotor_thickness - 0.5 * support_thickness
    outer_support_z = outer_z - 0.5 * rotor_thickness - 0.5 * support_thickness

    inner_shape = make_rotor_frame(
        outer_radius=0.056,
        rim_width=0.010,
        hub_outer_radius=0.025,
        bore_radius=0.0155,
        thickness=rotor_thickness,
        spoke_width=0.008,
        gap_width=0.034,
        pointer_length=0.020,
        pointer_width=0.013,
        spoke_angles_deg=(48.0, -48.0),
    )
    middle_shape = make_rotor_frame(
        outer_radius=0.078,
        rim_width=0.012,
        hub_outer_radius=0.027,
        bore_radius=0.0155,
        thickness=rotor_thickness,
        spoke_width=0.009,
        gap_width=0.046,
        pointer_length=0.026,
        pointer_width=0.015,
        spoke_angles_deg=(55.0, -40.0),
    )
    outer_shape = make_rotor_frame(
        outer_radius=0.100,
        rim_width=0.014,
        hub_outer_radius=0.029,
        bore_radius=0.0155,
        thickness=rotor_thickness,
        spoke_width=0.010,
        gap_width=0.058,
        pointer_length=0.032,
        pointer_width=0.017,
        spoke_angles_deg=(62.0, -34.0),
    )

    base = model.part("base")
    base.visual(
        Box((base_plate_size, base_plate_size, base_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * base_plate_thickness)),
        material=base_mat,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=pedestal_radius, length=pedestal_height),
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness + 0.5 * pedestal_height)),
        material=base_mat,
        name="pedestal",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=shaft_radius, length=shaft_height),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * shaft_height)),
        material=steel_mat,
        name="main_shaft",
    )
    shaft.visual(
        Cylinder(radius=0.022, length=support_thickness),
        origin=Origin(xyz=(0.0, 0.0, inner_support_z)),
        material=steel_mat,
        name="lower_collar",
    )
    shaft.visual(
        Cylinder(radius=0.023, length=support_thickness),
        origin=Origin(xyz=(0.0, 0.0, middle_support_z)),
        material=steel_mat,
        name="middle_collar",
    )
    shaft.visual(
        Cylinder(radius=0.024, length=support_thickness),
        origin=Origin(xyz=(0.0, 0.0, outer_support_z)),
        material=steel_mat,
        name="upper_collar",
    )
    shaft.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        material=steel_mat,
        name="top_cap",
    )

    inner_rotor = model.part("inner_rotor")
    inner_rotor.visual(
        mesh_from_cadquery(inner_shape, "inner_rotor_frame"),
        material=inner_mat,
        name="frame",
    )

    middle_rotor = model.part("middle_rotor")
    middle_rotor.visual(
        mesh_from_cadquery(middle_shape, "middle_rotor_frame"),
        material=middle_mat,
        name="frame",
    )

    outer_rotor = model.part("outer_rotor")
    outer_rotor.visual(
        mesh_from_cadquery(outer_shape, "outer_rotor_frame"),
        material=outer_mat,
        name="frame",
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.FIXED,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness + pedestal_height)),
    )
    model.articulation(
        "shaft_to_inner",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=inner_rotor,
        origin=Origin(xyz=(0.0, 0.0, inner_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "shaft_to_middle",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=middle_rotor,
        origin=Origin(xyz=(0.0, 0.0, middle_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "shaft_to_outer",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=outer_rotor,
        origin=Origin(xyz=(0.0, 0.0, outer_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base")
    shaft = object_model.get_part("shaft")
    inner_rotor = object_model.get_part("inner_rotor")
    middle_rotor = object_model.get_part("middle_rotor")
    outer_rotor = object_model.get_part("outer_rotor")

    pedestal = base.get_visual("pedestal")
    main_shaft = shaft.get_visual("main_shaft")
    lower_collar = shaft.get_visual("lower_collar")
    middle_collar = shaft.get_visual("middle_collar")
    upper_collar = shaft.get_visual("upper_collar")
    inner_frame = inner_rotor.get_visual("frame")
    middle_frame = middle_rotor.get_visual("frame")
    outer_frame = outer_rotor.get_visual("frame")

    inner_joint = object_model.get_articulation("shaft_to_inner")
    middle_joint = object_model.get_articulation("shaft_to_middle")
    outer_joint = object_model.get_articulation("shaft_to_outer")

    for joint_name, joint in (
        ("inner", inner_joint),
        ("middle", middle_joint),
        ("outer", outer_joint),
    ):
        ctx.check(
            f"{joint_name}_joint_is_revolute_about_shared_centerline",
            joint.articulation_type == ArticulationType.REVOLUTE and joint.axis == (0.0, 0.0, 1.0),
            details=f"{joint.name} should be a revolute joint about +Z, got type={joint.articulation_type} axis={joint.axis}",
        )

    ctx.expect_contact(
        shaft,
        base,
        elem_a=main_shaft,
        elem_b=pedestal,
        contact_tol=0.001,
        name="shaft_seated_on_base_pedestal",
    )

    ctx.expect_contact(
        inner_rotor,
        shaft,
        elem_a=inner_frame,
        elem_b=lower_collar,
        contact_tol=0.001,
        name="inner_rotor_supported_by_lower_collar",
    )
    ctx.expect_contact(
        middle_rotor,
        shaft,
        elem_a=middle_frame,
        elem_b=middle_collar,
        contact_tol=0.001,
        name="middle_rotor_supported_by_middle_collar",
    )
    ctx.expect_contact(
        outer_rotor,
        shaft,
        elem_a=outer_frame,
        elem_b=upper_collar,
        contact_tol=0.001,
        name="outer_rotor_supported_by_upper_collar",
    )

    ctx.expect_overlap(
        inner_rotor,
        shaft,
        axes="xy",
        min_overlap=0.035,
        name="inner_rotor_remains_coaxial_with_shaft",
    )
    ctx.expect_overlap(
        middle_rotor,
        shaft,
        axes="xy",
        min_overlap=0.040,
        name="middle_rotor_remains_coaxial_with_shaft",
    )
    ctx.expect_overlap(
        outer_rotor,
        shaft,
        axes="xy",
        min_overlap=0.042,
        name="outer_rotor_remains_coaxial_with_shaft",
    )

    ctx.expect_gap(
        middle_rotor,
        inner_rotor,
        axis="z",
        min_gap=0.048,
        max_gap=0.054,
        name="inner_and_middle_rotors_are_visibly_spaced",
    )
    ctx.expect_gap(
        outer_rotor,
        middle_rotor,
        axis="z",
        min_gap=0.048,
        max_gap=0.054,
        name="middle_and_outer_rotors_are_visibly_spaced",
    )

    with ctx.pose(
        {
            inner_joint: 0.9,
            middle_joint: -1.2,
            outer_joint: 0.6,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no_interference_in_representative_articulated_pose"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
