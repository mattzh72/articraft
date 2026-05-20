from __future__ import annotations

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
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_chest_freezer")

    # Overall proportions are in the range of a household / light-commercial
    # chest freezer: broad, deep, and heavy, with a thick insulated lid.
    depth = 0.72
    width = 1.32
    body_height = 0.82
    wall = 0.060
    floor_thickness = 0.120
    hinge_x = -depth / 2.0
    hinge_z = body_height

    off_white = model.material("warm_powder_coat", rgba=(0.88, 0.91, 0.91, 1.0))
    liner = model.material("molded_light_liner", rgba=(0.72, 0.78, 0.80, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.015, 0.017, 0.018, 1.0))
    metal = model.material("brushed_hinge_metal", rgba=(0.56, 0.58, 0.57, 1.0))
    dark_panel = model.material("black_control_plastic", rgba=(0.035, 0.040, 0.045, 1.0))
    label_blue = model.material("blue_energy_label", rgba=(0.04, 0.24, 0.85, 1.0))
    label_yellow = model.material("yellow_rating_strip", rgba=(0.95, 0.78, 0.08, 1.0))

    body = model.part("body")

    body_outer = (
        cq.Workplane("XY")
        .box(depth, width, body_height, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.032)
    )
    body_cavity = (
        cq.Workplane("XY")
        .box(
            depth - 2.0 * wall,
            width - 2.0 * wall,
            body_height - floor_thickness + 0.05,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, floor_thickness))
    )
    cabinet_shell = body_outer.cut(body_cavity)
    body.visual(
        mesh_from_cadquery(cabinet_shell, "cabinet_shell", tolerance=0.0015),
        material=off_white,
        name="cabinet_shell",
    )

    # Molded liner surfaces make the freezer read as a usable hollow appliance
    # when the lid is opened.
    liner_height = body_height - floor_thickness
    liner_z = floor_thickness + liner_height / 2.0
    inner_depth = depth - 2.0 * wall
    inner_width = width - 2.0 * wall
    body.visual(
        Box((inner_depth - 0.020, inner_width - 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness + 0.007)),
        material=liner,
        name="liner_floor",
    )
    body.visual(
        Box((0.010, inner_width - 0.020, liner_height)),
        origin=Origin(xyz=(inner_depth / 2.0 + 0.003, 0.0, liner_z)),
        material=liner,
        name="front_liner",
    )
    body.visual(
        Box((0.010, inner_width - 0.020, liner_height)),
        origin=Origin(xyz=(-inner_depth / 2.0 - 0.003, 0.0, liner_z)),
        material=liner,
        name="rear_liner",
    )
    body.visual(
        Box((inner_depth - 0.020, 0.010, liner_height)),
        origin=Origin(xyz=(0.0, inner_width / 2.0 + 0.003, liner_z)),
        material=liner,
        name="side_liner_0",
    )
    body.visual(
        Box((inner_depth - 0.020, 0.010, liner_height)),
        origin=Origin(xyz=(0.0, -inner_width / 2.0 - 0.003, liner_z)),
        material=liner,
        name="side_liner_1",
    )
    body.visual(
        Box((0.30, 0.35, 0.22)),
        origin=Origin(xyz=(-0.12, -0.405, floor_thickness + 0.110)),
        material=liner,
        name="compressor_step",
    )

    # Appliance details on the outside: a ventilation grille, control panel,
    # labels, plinth, leveling feet, catch plate, and exposed rear hinge leaves.
    grille = VentGrilleGeometry(
        (0.17, 0.36),
        frame=0.014,
        face_thickness=0.004,
        duct_depth=0.018,
        duct_wall=0.003,
        slat_pitch=0.022,
        slat_width=0.009,
        slat_angle_deg=30.0,
        corner_radius=0.006,
        slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=2),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.0015),
        mounts=VentGrilleMounts(style="holes", inset=0.014, hole_diameter=0.004),
        sleeve=VentGrilleSleeve(style="short", depth=0.018, wall=0.003),
        center=False,
    )
    body.visual(
        mesh_from_geometry(grille, "front_vent_grille"),
        origin=Origin(xyz=(depth / 2.0 - 0.001, -0.330, 0.295), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_panel,
        name="front_vent_grille",
    )
    body.visual(
        Box((0.018, 0.315, 0.115)),
        origin=Origin(xyz=(depth / 2.0 + 0.0085, 0.355, 0.365)),
        material=dark_panel,
        name="control_panel",
    )
    body.visual(
        Box((0.004, 0.105, 0.155)),
        origin=Origin(xyz=(depth / 2.0 + 0.002, 0.075, 0.430)),
        material=label_blue,
        name="energy_label",
    )
    body.visual(
        Box((0.005, 0.103, 0.025)),
        origin=Origin(xyz=(depth / 2.0 + 0.004, 0.075, 0.493)),
        material=label_yellow,
        name="rating_strip",
    )
    body.visual(
        Box((0.020, 1.10, 0.085)),
        origin=Origin(xyz=(depth / 2.0 + 0.009, 0.0, 0.055)),
        material=rubber,
        name="front_plinth",
    )
    for i, (x, y) in enumerate(((-0.260, -0.515), (-0.260, 0.515), (0.260, -0.515), (0.260, 0.515))):
        body.visual(
            Cylinder(radius=0.035, length=0.050),
            origin=Origin(xyz=(x, y, -0.024)),
            material=rubber,
            name=f"leveling_foot_{i}",
        )
    body.visual(
        Box((0.014, 0.140, 0.044)),
        origin=Origin(xyz=(depth / 2.0 + 0.006, 0.0, body_height - 0.050)),
        material=metal,
        name="front_catch",
    )
    for i, y in enumerate((-0.420, 0.420)):
        body.visual(
            Box((0.010, 0.220, 0.120)),
            origin=Origin(xyz=(-depth / 2.0 - 0.004, y, body_height - 0.060)),
            material=metal,
            name=f"hinge_leaf_{i}",
        )

    lid = model.part("lid")
    lid_depth = depth + 0.030
    lid_width = width + 0.060
    lid_thickness = 0.090
    lid_rear_overhang = 0.0
    lid_center_x = lid_depth / 2.0 - lid_rear_overhang
    lid_slab = (
        cq.Workplane("XY")
        .box(lid_depth, lid_width, lid_thickness, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.026)
        .translate((lid_center_x, 0.0, 0.012))
    )
    lid.visual(
        mesh_from_cadquery(lid_slab, "insulated_lid", tolerance=0.0012),
        material=off_white,
        name="insulated_lid",
    )
    # A dark compressible gasket is carried by the lid underside and sits on the
    # cabinet rim at the closed pose.
    lid.visual(
        Box((0.060, inner_width, 0.012)),
        origin=Origin(xyz=(wall / 2.0, 0.0, 0.006)),
        material=rubber,
        name="rear_gasket",
    )
    lid.visual(
        Box((0.060, inner_width, 0.012)),
        origin=Origin(xyz=(depth - wall / 2.0, 0.0, 0.006)),
        material=rubber,
        name="front_gasket",
    )
    lid.visual(
        Box((inner_depth, 0.060, 0.012)),
        origin=Origin(xyz=(depth / 2.0, inner_width / 2.0 + wall / 2.0, 0.006)),
        material=rubber,
        name="side_gasket_0",
    )
    lid.visual(
        Box((inner_depth, 0.060, 0.012)),
        origin=Origin(xyz=(depth / 2.0, -inner_width / 2.0 - wall / 2.0, 0.006)),
        material=rubber,
        name="side_gasket_1",
    )
    lid.visual(
        Box((0.070, 0.055, 0.045)),
        origin=Origin(xyz=(depth + 0.035, -0.280, 0.047)),
        material=dark_panel,
        name="handle_post_0",
    )
    lid.visual(
        Box((0.070, 0.055, 0.045)),
        origin=Origin(xyz=(depth + 0.035, 0.280, 0.047)),
        material=dark_panel,
        name="handle_post_1",
    )
    lid.visual(
        Box((0.048, 0.650, 0.048)),
        origin=Origin(xyz=(depth + 0.075, 0.0, 0.047)),
        material=dark_panel,
        name="front_handle",
    )
    lid.visual(
        Box((0.014, 0.165, 0.052)),
        origin=Origin(xyz=(depth + 0.037, 0.0, 0.047)),
        material=metal,
        name="latch_plate",
    )
    lid.visual(
        Box((0.150, 0.105, 0.003)),
        origin=Origin(xyz=(0.415, 0.350, lid_thickness + 0.013)),
        material=label_blue,
        name="lid_label",
    )
    lid.visual(
        Box((0.080, 0.215, 0.014)),
        origin=Origin(xyz=(0.035, -0.420, 0.018)),
        material=metal,
        name="hinge_strap_0",
    )
    lid.visual(
        Box((0.080, 0.215, 0.014)),
        origin=Origin(xyz=(0.035, 0.420, 0.018)),
        material=metal,
        name="hinge_strap_1",
    )
    for i, y in enumerate((-0.420, 0.420)):
        lid.visual(
            Cylinder(radius=0.016, length=0.230),
            origin=Origin(xyz=(0.000, y, 0.032), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"hinge_barrel_{i}",
        )

    thermostat = model.part("thermostat_knob")
    thermostat.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="knob_cap",
    )
    thermostat.visual(
        Box((0.004, 0.007, 0.024)),
        origin=Origin(xyz=(0.029, 0.0, 0.013)),
        material=dark_panel,
        name="pointer_mark",
    )

    drain_cap = model.part("drain_cap")
    drain_cap.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="drain_cap",
    )
    drain_cap.visual(
        Box((0.004, 0.041, 0.006)),
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material=dark_panel,
        name="slot_bar",
    )
    drain_cap.visual(
        Box((0.004, 0.006, 0.041)),
        origin=Origin(xyz=(0.0175, 0.0, 0.0)),
        material=dark_panel,
        name="slot_cross",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_thermostat",
        ArticulationType.REVOLUTE,
        parent=body,
        child=thermostat,
        origin=Origin(xyz=(depth / 2.0 + 0.0175, 0.355, 0.365)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "body_to_drain_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drain_cap,
        origin=Origin(xyz=(depth / 2.0, -0.455, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    thermostat = object_model.get_part("thermostat_knob")
    drain_cap = object_model.get_part("drain_cap")
    lid_joint = object_model.get_articulation("body_to_lid")
    thermostat_joint = object_model.get_articulation("body_to_thermostat")
    drain_joint = object_model.get_articulation("body_to_drain_cap")

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem="front_gasket",
            negative_elem="cabinet_shell",
            name="lid gasket sits on cabinet rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.60,
            elem_a="insulated_lid",
            elem_b="cabinet_shell",
            name="closed lid covers the freezer opening",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.45}):
        opened_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward from rear hinge",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][2] > closed_aabb[1][2] + 0.35,
            details=f"closed={closed_aabb}, opened={opened_aabb}",
        )

    ctx.expect_contact(
        thermostat,
        body,
        elem_a="knob_cap",
        elem_b="control_panel",
        contact_tol=0.001,
        name="thermostat knob is seated in control panel",
    )
    ctx.expect_contact(
        drain_cap,
        body,
        elem_a="drain_cap",
        elem_b="cabinet_shell",
        contact_tol=0.0015,
        name="drain cap is mounted on front wall",
    )
    ctx.check(
        "visible controls have rotary travel",
        thermostat_joint.motion_limits is not None
        and drain_joint.motion_limits is not None
        and thermostat_joint.motion_limits.upper - thermostat_joint.motion_limits.lower > 4.0
        and drain_joint.motion_limits.upper - drain_joint.motion_limits.lower > 6.0,
    )

    return ctx.report()


object_model = build_object_model()
